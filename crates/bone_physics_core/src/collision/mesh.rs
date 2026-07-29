use std::collections::{BTreeMap, BTreeSet};

use glam::Vec3;
use parry3d::bounding_volume::Aabb as ParryAabb;
use parry3d::math::Vector as ParryVector;
use parry3d::partitioning::{Bvh, BvhBuildStrategy, BvhWorkspace};

use crate::error::validate_nonnegative;
use crate::pose::validate_vec3;
use crate::{MeshId, RegionId, SolverError, TriangleId};

const TRIANGLE_AREA_EPSILON_SQUARED: f32 = 1.0e-16;
const FNV_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshTopology {
    pub vertex_count: u32,
    pub triangles: Vec<[u32; 3]>,
    pub triangle_regions: Vec<RegionId>,
    pub closed_hint: bool,
}

impl MeshTopology {
    #[must_use]
    pub fn validate(&self, positions: &[Vec3]) -> MeshValidationReport {
        validate_mesh(self, positions).0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshValidationIssue {
    VertexCountMismatch {
        declared: u32,
        actual: usize,
    },
    TriangleRegionCountMismatch {
        triangles: usize,
        regions: usize,
    },
    NonFiniteVertex {
        vertex: u32,
    },
    VertexIndexOutOfRange {
        triangle: TriangleId,
        vertex: u32,
    },
    DegenerateTriangle {
        triangle: TriangleId,
    },
    DuplicateTriangle {
        triangle: TriangleId,
        first: TriangleId,
    },
    BoundaryEdge {
        vertices: [u32; 2],
    },
    NonManifoldEdge {
        vertices: [u32; 2],
        incident_triangles: usize,
    },
    InconsistentEdgeOrientation {
        vertices: [u32; 2],
    },
    NoValidTriangles,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MeshValidationReport {
    pub errors: Vec<MeshValidationIssue>,
    pub warnings: Vec<MeshValidationIssue>,
    pub topology_hash: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeshUpdate {
    Refit,
    Rebuilt,
}

pub struct DeformingMesh {
    id: MeshId,
    topology: MeshTopology,
    previous_positions: Vec<Vec3>,
    current_positions: Vec<Vec3>,
    thickness: f32,
    report: MeshValidationReport,
    active_triangles: Vec<TriangleId>,
    bvh: Bvh,
    bvh_workspace: BvhWorkspace,
}

impl DeformingMesh {
    pub fn build(
        id: MeshId,
        topology: MeshTopology,
        positions: Vec<Vec3>,
        thickness: f32,
    ) -> Result<Self, SolverError> {
        Self::rebuild(id, topology, positions.clone(), positions, thickness)
    }

    pub fn rebuild(
        id: MeshId,
        topology: MeshTopology,
        previous_positions: Vec<Vec3>,
        current_positions: Vec<Vec3>,
        thickness: f32,
    ) -> Result<Self, SolverError> {
        validate_nonnegative("mesh_thickness", thickness)?;
        validate_position_batch(&topology, &previous_positions, "mesh_previous_positions")?;
        let (report, active_triangles) = validate_mesh(&topology, &current_positions);
        if let Some(issue) = report.errors.first() {
            return Err(validation_error(*issue));
        }
        let aabbs = triangle_aabbs(
            &topology,
            &active_triangles,
            &previous_positions,
            &current_positions,
            thickness,
        )?;
        let bvh = Bvh::from_leaves(BvhBuildStrategy::default(), &aabbs);
        Ok(Self {
            id,
            topology,
            previous_positions,
            current_positions,
            thickness,
            report,
            active_triangles,
            bvh,
            bvh_workspace: BvhWorkspace::default(),
        })
    }

    pub fn update_vertices(
        &mut self,
        previous: Vec<Vec3>,
        current: Vec<Vec3>,
    ) -> Result<MeshUpdate, SolverError> {
        validate_position_batch(&self.topology, &previous, "mesh_previous_positions")?;
        validate_position_batch(&self.topology, &current, "mesh_current_positions")?;
        let aabbs = triangle_aabbs(
            &self.topology,
            &self.active_triangles,
            &previous,
            &current,
            self.thickness,
        )?;
        for (leaf, aabb) in aabbs.into_iter().enumerate() {
            let leaf = u32::try_from(leaf)
                .map_err(|_| SolverError::invalid_value("mesh_triangle_count", leaf as f64))?;
            self.bvh.insert_or_update_partially(aabb, leaf, 0.0);
        }
        self.bvh.refit(&mut self.bvh_workspace);
        self.previous_positions = previous;
        self.current_positions = current;
        Ok(MeshUpdate::Refit)
    }

    pub fn query_aabb(&self, minimum: Vec3, maximum: Vec3) -> Result<Vec<TriangleId>, SolverError> {
        validate_vec3("mesh_query_minimum", minimum)?;
        validate_vec3("mesh_query_maximum", maximum)?;
        if minimum.x > maximum.x || minimum.y > maximum.y || minimum.z > maximum.z {
            return Err(SolverError::invalid_value("mesh_query_bounds", 0.0));
        }
        let query = ParryAabb::new(parry_vector(minimum), parry_vector(maximum));
        let mut result = self
            .bvh
            .intersect_aabb(&query)
            .map(|leaf| {
                let index = usize::try_from(leaf)
                    .map_err(|_| SolverError::invalid_value("mesh_bvh_leaf", f64::from(leaf)))?;
                self.active_triangles
                    .get(index)
                    .copied()
                    .ok_or_else(|| SolverError::invalid_value("mesh_bvh_leaf", f64::from(leaf)))
            })
            .collect::<Result<Vec<_>, _>>()?;
        result.sort_unstable();
        result.dedup();
        Ok(result)
    }

    #[must_use]
    pub fn id(&self) -> MeshId {
        self.id
    }

    #[must_use]
    pub fn topology(&self) -> &MeshTopology {
        &self.topology
    }

    #[must_use]
    pub fn previous_positions(&self) -> &[Vec3] {
        &self.previous_positions
    }

    #[must_use]
    pub fn current_positions(&self) -> &[Vec3] {
        &self.current_positions
    }

    #[must_use]
    pub fn thickness(&self) -> f32 {
        self.thickness
    }

    #[must_use]
    pub fn validation_report(&self) -> &MeshValidationReport {
        &self.report
    }

    #[must_use]
    pub fn active_triangle_ids(&self) -> &[TriangleId] {
        &self.active_triangles
    }
}

fn validate_mesh(
    topology: &MeshTopology,
    positions: &[Vec3],
) -> (MeshValidationReport, Vec<TriangleId>) {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();
    if usize::try_from(topology.vertex_count).ok() != Some(positions.len()) {
        errors.push(MeshValidationIssue::VertexCountMismatch {
            declared: topology.vertex_count,
            actual: positions.len(),
        });
    }
    if topology.triangles.len() != topology.triangle_regions.len() {
        errors.push(MeshValidationIssue::TriangleRegionCountMismatch {
            triangles: topology.triangles.len(),
            regions: topology.triangle_regions.len(),
        });
    }
    for (index, position) in positions.iter().enumerate() {
        if !position.is_finite() {
            errors.push(MeshValidationIssue::NonFiniteVertex {
                vertex: u32::try_from(index).unwrap_or(u32::MAX),
            });
        }
    }

    let mut active = Vec::new();
    let mut seen = BTreeMap::<[u32; 3], TriangleId>::new();
    for (index, triangle) in topology.triangles.iter().copied().enumerate() {
        let id = TriangleId::new(u32::try_from(index).unwrap_or(u32::MAX));
        let mut in_range = true;
        for vertex in triangle {
            if vertex >= topology.vertex_count
                || usize::try_from(vertex).map_or(true, |i| i >= positions.len())
            {
                errors.push(MeshValidationIssue::VertexIndexOutOfRange {
                    triangle: id,
                    vertex,
                });
                in_range = false;
            }
        }
        if !in_range {
            continue;
        }
        let [a, b, c] = triangle;
        let area_squared = (positions[b as usize] - positions[a as usize])
            .cross(positions[c as usize] - positions[a as usize])
            .length_squared();
        if a == b
            || b == c
            || c == a
            || !area_squared.is_finite()
            || area_squared <= TRIANGLE_AREA_EPSILON_SQUARED
        {
            warnings.push(MeshValidationIssue::DegenerateTriangle { triangle: id });
            continue;
        }
        let mut canonical = triangle;
        canonical.sort_unstable();
        if let Some(first) = seen.get(&canonical).copied() {
            warnings.push(MeshValidationIssue::DuplicateTriangle {
                triangle: id,
                first,
            });
            continue;
        }
        seen.insert(canonical, id);
        active.push(id);
    }
    if active.is_empty() {
        errors.push(MeshValidationIssue::NoValidTriangles);
    }

    let active_set = active.iter().copied().collect::<BTreeSet<_>>();
    let mut edges = BTreeMap::<[u32; 2], Vec<bool>>::new();
    for (index, triangle) in topology.triangles.iter().copied().enumerate() {
        if !active_set.contains(&TriangleId::new(u32::try_from(index).unwrap_or(u32::MAX))) {
            continue;
        }
        for [a, b] in [
            [triangle[0], triangle[1]],
            [triangle[1], triangle[2]],
            [triangle[2], triangle[0]],
        ] {
            let key = if a < b { [a, b] } else { [b, a] };
            edges.entry(key).or_default().push(a < b);
        }
    }
    for (vertices, orientations) in edges {
        if orientations.len() > 2 {
            warnings.push(MeshValidationIssue::NonManifoldEdge {
                vertices,
                incident_triangles: orientations.len(),
            });
        } else if orientations.len() == 2 && orientations[0] == orientations[1] {
            warnings.push(MeshValidationIssue::InconsistentEdgeOrientation { vertices });
        } else if topology.closed_hint && orientations.len() == 1 {
            warnings.push(MeshValidationIssue::BoundaryEdge { vertices });
        }
    }
    (
        MeshValidationReport {
            errors,
            warnings,
            topology_hash: topology_hash(topology),
        },
        active,
    )
}

fn validate_position_batch(
    topology: &MeshTopology,
    positions: &[Vec3],
    field: &'static str,
) -> Result<(), SolverError> {
    if usize::try_from(topology.vertex_count).ok() != Some(positions.len()) {
        return Err(SolverError::invalid_value(field, positions.len() as f64));
    }
    for position in positions {
        validate_vec3(field, *position)?;
    }
    Ok(())
}

fn validation_error(issue: MeshValidationIssue) -> SolverError {
    match issue {
        MeshValidationIssue::VertexCountMismatch { actual, .. } => {
            SolverError::invalid_value("mesh_vertex_count", actual as f64)
        }
        MeshValidationIssue::TriangleRegionCountMismatch { regions, .. } => {
            SolverError::invalid_value("mesh_triangle_region_count", regions as f64)
        }
        MeshValidationIssue::NonFiniteVertex { vertex } => {
            SolverError::invalid_value("mesh_vertex_position", vertex)
        }
        MeshValidationIssue::VertexIndexOutOfRange { vertex, .. } => {
            SolverError::invalid_value("mesh_triangle_vertex", vertex)
        }
        MeshValidationIssue::NoValidTriangles => {
            SolverError::invalid_value("mesh_valid_triangle_count", 0.0)
        }
        _ => SolverError::invalid_value("mesh_validation", 0.0),
    }
}

fn triangle_aabbs(
    topology: &MeshTopology,
    active: &[TriangleId],
    previous: &[Vec3],
    current: &[Vec3],
    thickness: f32,
) -> Result<Vec<ParryAabb>, SolverError> {
    active
        .iter()
        .map(|id| {
            let index = usize::try_from(id.get())
                .map_err(|_| SolverError::invalid_value("triangle_id", id.get()))?;
            let triangle = topology
                .triangles
                .get(index)
                .ok_or_else(|| SolverError::invalid_value("triangle_id", id.get()))?;
            let mut minimum = Vec3::splat(f32::INFINITY);
            let mut maximum = Vec3::splat(f32::NEG_INFINITY);
            for positions in [previous, current] {
                for vertex in triangle {
                    let point = positions[*vertex as usize];
                    minimum = minimum.min(point);
                    maximum = maximum.max(point);
                }
            }
            minimum -= Vec3::splat(thickness);
            maximum += Vec3::splat(thickness);
            validate_vec3("mesh_triangle_aabb_minimum", minimum)?;
            validate_vec3("mesh_triangle_aabb_maximum", maximum)?;
            Ok(ParryAabb::new(parry_vector(minimum), parry_vector(maximum)))
        })
        .collect()
}

fn parry_vector(value: Vec3) -> ParryVector {
    ParryVector::new(value.x, value.y, value.z)
}

fn topology_hash(topology: &MeshTopology) -> u64 {
    let mut hash = FNV_OFFSET_BASIS;
    hash_u32(&mut hash, topology.vertex_count);
    hash_u32(
        &mut hash,
        u32::try_from(topology.triangles.len()).unwrap_or(u32::MAX),
    );
    for triangle in &topology.triangles {
        for vertex in triangle {
            hash_u32(&mut hash, *vertex);
        }
    }
    hash_u32(
        &mut hash,
        u32::try_from(topology.triangle_regions.len()).unwrap_or(u32::MAX),
    );
    for region in &topology.triangle_regions {
        hash_u32(&mut hash, region.get());
    }
    hash_byte(&mut hash, u8::from(topology.closed_hint));
    hash
}

fn hash_u32(hash: &mut u64, value: u32) {
    for byte in value.to_le_bytes() {
        hash_byte(hash, byte);
    }
}

fn hash_byte(hash: &mut u64, value: u8) {
    *hash ^= u64::from(value);
    *hash = hash.wrapping_mul(FNV_PRIME);
}
