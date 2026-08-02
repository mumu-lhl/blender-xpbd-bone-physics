use glam::Vec3;

use super::capsule::{ContactGeometry, usable_normal};
use super::contact::MeshContactKey;
use super::shape::CapsuleCollider;
use crate::error::validate_nonnegative;
use crate::pose::validate_vec3;
use crate::{MeshId, RegionId, SolverError, TriangleId};

const GEOMETRY_EPSILON_SQUARED: f32 = 1.0e-12;
const FEATURE_EPSILON: f32 = 1.0e-4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MovingTriangle {
    pub mesh: MeshId,
    pub triangle: TriangleId,
    pub region: RegionId,
    pub vertices: [Vec3; 3],
    pub velocities: [Vec3; 3],
    pub thickness: f32,
    pub two_sided: bool,
    pub feature_owners: Option<TriangleFeatureOwners>,
}

impl MovingTriangle {
    pub(crate) fn validate(self) -> Result<Vec3, SolverError> {
        for vertex in self.vertices {
            validate_vec3("mesh_triangle_vertex", vertex)?;
        }
        for velocity in self.velocities {
            validate_vec3("mesh_triangle_velocity", velocity)?;
        }
        validate_nonnegative("mesh_triangle_thickness", self.thickness)?;
        let normal =
            (self.vertices[1] - self.vertices[0]).cross(self.vertices[2] - self.vertices[0]);
        usable_normal(normal).ok_or_else(|| {
            SolverError::invalid_value("mesh_triangle_area", normal.length_squared())
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleFeature {
    Face,
    Edge(u8),
    Vertex(u8),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TriangleFeatureOwner {
    pub triangle: TriangleId,
    pub feature: TriangleFeature,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TriangleFeatureOwners {
    pub edges: [TriangleFeatureOwner; 3],
    pub vertices: [TriangleFeatureOwner; 3],
}

impl TriangleFeatureOwners {
    fn owner(self, feature: TriangleFeature, triangle: TriangleId) -> TriangleFeatureOwner {
        match feature {
            TriangleFeature::Face => TriangleFeatureOwner { triangle, feature },
            TriangleFeature::Edge(index) => self.edges[index as usize],
            TriangleFeature::Vertex(index) => self.vertices[index as usize],
        }
    }
}

impl TriangleFeature {
    const fn encode(self) -> u32 {
        match self {
            Self::Face => 0,
            Self::Edge(index) => 1 + index as u32,
            Self::Vertex(index) => 4 + index as u32,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MeshContactFeature {
    pub capsule: u8,
    pub triangle: TriangleFeature,
}

impl MeshContactFeature {
    #[must_use]
    pub const fn encode(self) -> u32 {
        ((self.capsule as u32) << 8) | self.triangle.encode()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshContactHistory {
    pub normal: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshContactGeometry {
    pub key: MeshContactKey,
    pub source_triangle: TriangleId,
    pub geometry: ContactGeometry,
    pub barycentric: Vec3,
    pub segment_t: f32,
    pub surface_velocity: Vec3,
    pub feature: MeshContactFeature,
}

impl MeshContactGeometry {
    pub(crate) fn validate(self) -> Result<(), SolverError> {
        validate_vec3("mesh_contact_point_a", self.geometry.point_a)?;
        validate_vec3("mesh_contact_point_b", self.geometry.point_b)?;
        validate_vec3("mesh_contact_normal", self.geometry.normal)?;
        validate_nonnegative("mesh_contact_penetration", self.geometry.penetration)?;
        validate_vec3("mesh_contact_barycentric", self.barycentric)?;
        validate_vec3("mesh_contact_surface_velocity", self.surface_velocity)?;
        if self.barycentric.min_element() < -FEATURE_EPSILON
            || (self.barycentric.element_sum() - 1.0).abs() > FEATURE_EPSILON
        {
            return Err(SolverError::invalid_value(
                "mesh_contact_barycentric",
                self.barycentric.element_sum(),
            ));
        }
        if !self.segment_t.is_finite() || !(0.0..=1.0).contains(&self.segment_t) {
            return Err(SolverError::invalid_value(
                "mesh_contact_segment_t",
                self.segment_t,
            ));
        }
        Ok(())
    }
}

pub fn capsule_triangle_contact(
    capsule: &CapsuleCollider,
    triangle: &MovingTriangle,
    history: Option<MeshContactHistory>,
) -> Result<Option<MeshContactGeometry>, SolverError> {
    capsule.validate()?;
    let triangle_normal = triangle.validate()?;
    if let Some(history) = history {
        validate_vec3("mesh_contact_history_normal", history.normal)?;
    }
    let (segment_a, segment_b) = capsule.world_segment()?;
    let closest = closest_segment_triangle(segment_a, segment_b, triangle.vertices)?;
    let radius = capsule.shape.effective_radius()? + triangle.thickness;
    validate_nonnegative("mesh_contact_radius", radius)?;
    let distance_squared = closest
        .segment_point
        .distance_squared(closest.triangle_point);
    validate_nonnegative("mesh_contact_distance_squared", distance_squared)?;
    if distance_squared > radius * radius {
        return Ok(None);
    }

    let signed_side = (closest.segment_point - triangle.vertices[0]).dot(triangle_normal);
    if !triangle.two_sided && signed_side < 0.0 {
        return Ok(None);
    }
    let surface_velocity = triangle.velocities[0] * closest.barycentric.x
        + triangle.velocities[1] * closest.barycentric.y
        + triangle.velocities[2] * closest.barycentric.z;
    validate_vec3("mesh_contact_surface_velocity", surface_velocity)?;
    let delta = closest.triangle_point - closest.segment_point;
    let normal = usable_normal(delta)
        .or_else(|| history.and_then(|value| usable_normal(value.normal)))
        .or_else(|| usable_normal(surface_velocity - capsule.linear_velocity))
        .unwrap_or_else(|| {
            if signed_side != 0.0 {
                -triangle_normal * signed_side.signum()
            } else if (capsule.id.get() ^ triangle.mesh.get() ^ triangle.triangle.get()) & 1 == 0 {
                triangle_normal
            } else {
                -triangle_normal
            }
        });
    let distance = distance_squared.sqrt();
    let penetration = (radius - distance).max(0.0);
    let point_a = closest.segment_point + normal * capsule.shape.effective_radius()?;
    let point_b = closest.triangle_point - normal * triangle.thickness;
    let feature = MeshContactFeature {
        capsule: capsule_feature(closest.segment_t),
        triangle: triangle_feature(closest.barycentric),
    };
    let owner = triangle.feature_owners.map_or(
        TriangleFeatureOwner {
            triangle: triangle.triangle,
            feature: feature.triangle,
        },
        |owners| owners.owner(feature.triangle, triangle.triangle),
    );
    let key_feature = MeshContactFeature {
        capsule: feature.capsule,
        triangle: owner.feature,
    };
    let contact = MeshContactGeometry {
        key: MeshContactKey {
            collider: capsule.id,
            mesh: triangle.mesh,
            triangle: owner.triangle,
            feature: key_feature.encode(),
        },
        source_triangle: triangle.triangle,
        geometry: ContactGeometry {
            point_a,
            point_b,
            normal,
            penetration,
        },
        barycentric: closest.barycentric,
        segment_t: closest.segment_t,
        surface_velocity,
        feature,
    };
    contact.validate()?;
    Ok(Some(contact))
}

#[derive(Clone, Copy)]
struct ClosestTrianglePoints {
    segment_point: Vec3,
    triangle_point: Vec3,
    barycentric: Vec3,
    segment_t: f32,
    rank: u8,
}

impl ClosestTrianglePoints {
    fn distance_squared(self) -> f32 {
        self.segment_point.distance_squared(self.triangle_point)
    }
}

fn closest_segment_triangle(
    segment_a: Vec3,
    segment_b: Vec3,
    triangle: [Vec3; 3],
) -> Result<ClosestTrianglePoints, SolverError> {
    let mut candidates = Vec::with_capacity(7);
    if let Some((segment_t, barycentric)) =
        segment_triangle_intersection(segment_a, segment_b, triangle)
    {
        let point = segment_a.lerp(segment_b, segment_t);
        candidates.push(ClosestTrianglePoints {
            segment_point: point,
            triangle_point: point,
            barycentric,
            segment_t,
            rank: 0,
        });
    }
    for (rank, segment_t) in [(1, 0.5), (2, 0.0), (3, 1.0)] {
        let point = segment_a.lerp(segment_b, segment_t);
        let (triangle_point, barycentric) = closest_point_triangle(point, triangle);
        candidates.push(ClosestTrianglePoints {
            segment_point: point,
            triangle_point,
            barycentric,
            segment_t,
            rank,
        });
    }
    for (edge, [start, end]) in [[0, 1], [1, 2], [2, 0]].into_iter().enumerate() {
        let (segment_t, edge_t, segment_point, triangle_point) =
            closest_segment_points(segment_a, segment_b, triangle[start], triangle[end]);
        let mut barycentric = Vec3::ZERO;
        barycentric[start] = 1.0 - edge_t;
        barycentric[end] = edge_t;
        candidates.push(ClosestTrianglePoints {
            segment_point,
            triangle_point,
            barycentric,
            segment_t,
            rank: 4 + edge as u8,
        });
    }
    candidates
        .into_iter()
        .min_by(|left, right| {
            left.distance_squared()
                .total_cmp(&right.distance_squared())
                .then_with(|| left.rank.cmp(&right.rank))
        })
        .ok_or_else(|| SolverError::invalid_value("mesh_contact_candidates", 0.0))
}

fn segment_triangle_intersection(
    start: Vec3,
    end: Vec3,
    triangle: [Vec3; 3],
) -> Option<(f32, Vec3)> {
    let direction = end - start;
    let edge_a = triangle[1] - triangle[0];
    let edge_b = triangle[2] - triangle[0];
    let p = direction.cross(edge_b);
    let determinant = edge_a.dot(p);
    if determinant.abs() <= GEOMETRY_EPSILON_SQUARED {
        return None;
    }
    let inverse = determinant.recip();
    let translated = start - triangle[0];
    let u = translated.dot(p) * inverse;
    if !(-FEATURE_EPSILON..=1.0 + FEATURE_EPSILON).contains(&u) {
        return None;
    }
    let q = translated.cross(edge_a);
    let v = direction.dot(q) * inverse;
    if v < -FEATURE_EPSILON || u + v > 1.0 + FEATURE_EPSILON {
        return None;
    }
    let segment_t = edge_b.dot(q) * inverse;
    if !(-FEATURE_EPSILON..=1.0 + FEATURE_EPSILON).contains(&segment_t) {
        return None;
    }
    Some((segment_t.clamp(0.0, 1.0), Vec3::new(1.0 - u - v, u, v)))
}

fn closest_point_triangle(point: Vec3, triangle: [Vec3; 3]) -> (Vec3, Vec3) {
    let [a, b, c] = triangle;
    let ab = b - a;
    let ac = c - a;
    let ap = point - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, Vec3::X);
    }
    let bp = point - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, Vec3::Y);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, Vec3::new(1.0 - v, v, 0.0));
    }
    let cp = point - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, Vec3::Z);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, Vec3::new(1.0 - w, 0.0, w));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, Vec3::new(0.0, 1.0 - w, w));
    }
    let denominator = (va + vb + vc).recip();
    let v = vb * denominator;
    let w = vc * denominator;
    (a + ab * v + ac * w, Vec3::new(1.0 - v - w, v, w))
}

fn closest_segment_points(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (f32, f32, Vec3, Vec3) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let (mut s, t);
    if a <= GEOMETRY_EPSILON_SQUARED && e <= GEOMETRY_EPSILON_SQUARED {
        return (0.0, 0.0, p1, p2);
    }
    if a <= GEOMETRY_EPSILON_SQUARED {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= GEOMETRY_EPSILON_SQUARED {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denominator = a * e - b * b;
            s = if denominator.abs() > GEOMETRY_EPSILON_SQUARED {
                ((b * f - c * e) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let projected = b * s + f;
            if projected < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if projected > e {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            } else {
                t = projected / e;
            }
        }
    }
    (s, t, p1 + d1 * s, p2 + d2 * t)
}

fn capsule_feature(segment_t: f32) -> u8 {
    if segment_t <= FEATURE_EPSILON {
        0
    } else if segment_t >= 1.0 - FEATURE_EPSILON {
        1
    } else {
        2
    }
}

fn triangle_feature(barycentric: Vec3) -> TriangleFeature {
    for index in 0..3 {
        if barycentric[index] >= 1.0 - FEATURE_EPSILON {
            return TriangleFeature::Vertex(index as u8);
        }
    }
    for index in 0..3 {
        if barycentric[index] <= FEATURE_EPSILON {
            // Barycentric coordinate 0 is opposite vertex 0, so it maps to
            // stored edge [1, 2]; likewise 1 -> [2, 0] and 2 -> [0, 1].
            return TriangleFeature::Edge(((index + 1) % 3) as u8);
        }
    }
    TriangleFeature::Face
}
