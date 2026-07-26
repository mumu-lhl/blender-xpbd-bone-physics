use glam::Vec3;

use super::shape::CapsuleCollider;
use crate::pose::validate_vec3;
use crate::{ColliderId, SolverError};

const GEOMETRY_EPSILON_SQUARED: f32 = 1.0e-12;
const PARALLEL_DOT_THRESHOLD: f32 = 0.999_9;
const TWO_POINT_OVERLAP_MINIMUM: f32 = 1.0e-4;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct PairKey {
    first: ColliderId,
    second: ColliderId,
}

impl PairKey {
    pub fn ordered(a: ColliderId, b: ColliderId) -> Result<Self, SolverError> {
        if a == b {
            return Err(SolverError::invalid_value("collider_pair", a.get()));
        }
        let (first, second) = if a < b { (a, b) } else { (b, a) };
        Ok(Self { first, second })
    }

    #[must_use]
    pub const fn first(self) -> ColliderId {
        self.first
    }

    #[must_use]
    pub const fn second(self) -> ColliderId {
        self.second
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactGeometry {
    pub point_a: Vec3,
    pub point_b: Vec3,
    pub normal: Vec3,
    pub penetration: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CapsuleManifold {
    pub pair: PairKey,
    pub points: Vec<ContactGeometry>,
}

pub fn capsule_manifold(
    collider_a: &CapsuleCollider,
    collider_b: &CapsuleCollider,
    cached_normal: Option<Vec3>,
) -> Result<Option<CapsuleManifold>, SolverError> {
    let pair = PairKey::ordered(collider_a.id, collider_b.id)?;
    let (a, b) = if collider_a.id < collider_b.id {
        (collider_a, collider_b)
    } else {
        (collider_b, collider_a)
    };
    a.validate()?;
    b.validate()?;
    if let Some(normal) = cached_normal {
        validate_vec3("cached_contact_normal", normal)?;
    }

    let (a0, a1) = a.world_segment()?;
    let (b0, b1) = b.world_segment()?;
    let radius_a = a.shape.effective_radius()?;
    let radius_b = b.shape.effective_radius()?;
    let radius_sum = radius_a + radius_b;
    if !radius_sum.is_finite() {
        return Err(SolverError::invalid_value(
            "capsule_radius_sum",
            radius_sum,
        ));
    }

    let (closest_a, closest_b) = closest_segment_points(a0, a1, b0, b1);
    let closest_distance = closest_a.distance(closest_b);
    if !closest_distance.is_finite() {
        return Err(SolverError::invalid_value(
            "capsule_separation",
            closest_distance,
        ));
    }
    if closest_distance > radius_sum {
        return Ok(None);
    }

    let axis_a = a1 - a0;
    let axis_b = b1 - b0;
    let fallback = fallback_normal(pair, a, b, axis_a, axis_b, cached_normal)?;
    let mut points = parallel_contacts(
        a0,
        a1,
        b0,
        b1,
        radius_a,
        radius_b,
        radius_sum,
        fallback,
    )?;
    if points.is_empty() {
        points.push(make_contact(
            closest_a,
            closest_b,
            radius_a,
            radius_b,
            radius_sum,
            fallback,
        )?);
    }

    Ok(Some(CapsuleManifold { pair, points }))
}

#[allow(clippy::too_many_arguments)]
fn parallel_contacts(
    a0: Vec3,
    a1: Vec3,
    b0: Vec3,
    b1: Vec3,
    radius_a: f32,
    radius_b: f32,
    radius_sum: f32,
    fallback: Vec3,
) -> Result<Vec<ContactGeometry>, SolverError> {
    let axis_a = a1 - a0;
    let axis_b = b1 - b0;
    let length_a = axis_a.length();
    let length_b = axis_b.length();
    if length_a <= GEOMETRY_EPSILON_SQUARED.sqrt()
        || length_b <= GEOMETRY_EPSILON_SQUARED.sqrt()
    {
        return Ok(Vec::new());
    }

    let direction_a = axis_a / length_a;
    let direction_b = axis_b / length_b;
    if direction_a.dot(direction_b).abs() < PARALLEL_DOT_THRESHOLD {
        return Ok(Vec::new());
    }

    let projection_b0 = (b0 - a0).dot(direction_a);
    let projection_b1 = (b1 - a0).dot(direction_a);
    let overlap_start = 0.0_f32.max(projection_b0.min(projection_b1));
    let overlap_end = length_a.min(projection_b0.max(projection_b1));
    if overlap_end - overlap_start <= TWO_POINT_OVERLAP_MINIMUM {
        return Ok(Vec::new());
    }

    let mut contacts = Vec::with_capacity(2);
    for projection in [overlap_start, overlap_end] {
        let point_a = a0 + direction_a * projection;
        let point_b = closest_point_on_segment(point_a, b0, b1);
        if point_a.distance(point_b) <= radius_sum {
            contacts.push(make_contact(
                point_a,
                point_b,
                radius_a,
                radius_b,
                radius_sum,
                fallback,
            )?);
        }
    }
    if contacts.len() == 2
        && contacts[0].point_a.distance_squared(contacts[1].point_a)
            <= TWO_POINT_OVERLAP_MINIMUM * TWO_POINT_OVERLAP_MINIMUM
    {
        contacts.pop();
    }
    Ok(contacts)
}

fn make_contact(
    center_a: Vec3,
    center_b: Vec3,
    radius_a: f32,
    radius_b: f32,
    radius_sum: f32,
    fallback: Vec3,
) -> Result<ContactGeometry, SolverError> {
    let delta = center_b - center_a;
    let distance = delta.length();
    let normal = if distance * distance > GEOMETRY_EPSILON_SQUARED {
        delta / distance
    } else {
        fallback
    };
    let penetration = (radius_sum - distance).max(0.0);
    let point_a = center_a + normal * radius_a;
    let point_b = center_b - normal * radius_b;
    validate_vec3("contact_point_a", point_a)?;
    validate_vec3("contact_point_b", point_b)?;
    validate_vec3("contact_normal", normal)?;
    if !penetration.is_finite() {
        return Err(SolverError::invalid_value(
            "contact_penetration",
            penetration,
        ));
    }
    Ok(ContactGeometry {
        point_a,
        point_b,
        normal,
        penetration,
    })
}

fn fallback_normal(
    pair: PairKey,
    a: &CapsuleCollider,
    b: &CapsuleCollider,
    axis_a: Vec3,
    axis_b: Vec3,
    cached_normal: Option<Vec3>,
) -> Result<Vec3, SolverError> {
    if let Some(normal) = cached_normal.and_then(usable_normal) {
        return Ok(normal);
    }
    if let Some(normal) = usable_normal(b.linear_velocity - a.linear_velocity) {
        return Ok(normal);
    }
    if let Some(normal) = usable_normal(axis_a.cross(axis_b)) {
        return Ok(normal);
    }

    let axis = usable_normal(axis_a)
        .or_else(|| usable_normal(axis_b))
        .unwrap_or(Vec3::ZERO);
    let selector = pair.first().get().wrapping_mul(31) ^ pair.second().get();
    let basis = match selector % 3 {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    };
    if axis != Vec3::ZERO {
        for candidate in [basis, Vec3::X, Vec3::Y, Vec3::Z] {
            if let Some(normal) = usable_normal(axis.cross(candidate)) {
                return Ok(normal);
            }
        }
    }
    usable_normal(basis).ok_or(SolverError::invalid_value(
        "contact_normal",
        f64::NAN,
    ))
}

fn usable_normal(candidate: Vec3) -> Option<Vec3> {
    if candidate.is_finite() && candidate.length_squared() > GEOMETRY_EPSILON_SQUARED {
        Some(candidate.normalize())
    } else {
        None
    }
}

fn closest_point_on_segment(point: Vec3, start: Vec3, end: Vec3) -> Vec3 {
    let axis = end - start;
    let length_squared = axis.length_squared();
    if length_squared <= GEOMETRY_EPSILON_SQUARED {
        start
    } else {
        start + axis * ((point - start).dot(axis) / length_squared).clamp(0.0, 1.0)
    }
}

fn closest_segment_points(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> (Vec3, Vec3) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (mut s, t);
    if a <= GEOMETRY_EPSILON_SQUARED && e <= GEOMETRY_EPSILON_SQUARED {
        return (p1, p2);
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
    (p1 + d1 * s, p2 + d2 * t)
}
