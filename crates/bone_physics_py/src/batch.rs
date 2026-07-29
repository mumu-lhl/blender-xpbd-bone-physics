use std::collections::BTreeSet;

use bone_physics_core::{
    AnalyticProxy, AnalyticShape, AngularLimit, BodyId, BodyInit, BoneBodyDesc, Capsule,
    CapsuleColliderDesc, ColliderId, CollisionFilter, ConstraintId, ContactMaterial, DeformingMesh,
    EulerOrder, MeshId, MeshTopology, PersistentConstraint, Pose, RegionId, SolverError,
    SwingTwistLimit, Velocity, XyzLimit,
};
use glam::{Quat, Vec3};
use pyo3::buffer::{Element, PyBuffer};
use pyo3::prelude::*;
use pyo3::types::{PyAnyMethods, PyMemoryView};

use crate::error::input_error;

pub const BODY_COLUMNS: usize = 46;
pub const CONSTRAINT_COLUMNS: usize = 24;
pub const TARGET_COLUMNS: usize = 16;
pub const ANALYTIC_COLLIDER_COLUMNS: usize = 17;

#[derive(Clone, Debug, Eq, PartialEq, thiserror::Error)]
pub enum BatchValidationError {
    #[error("{name} must declare exactly {expected} columns, received {actual}")]
    Columns {
        name: String,
        expected: usize,
        actual: usize,
    },
    #[error("{name} value count {values} is not divisible by {columns}")]
    Ragged {
        name: String,
        values: usize,
        columns: usize,
    },
    #[error("{name} must be monotonic, start at zero, and end at {item_count}")]
    Offsets { name: String, item_count: usize },
    #[error("duplicate {kind} ID {id}")]
    DuplicateId { kind: String, id: u32 },
}

pub struct DecodedWorld {
    pub bodies: Vec<BodyInit>,
    pub body_ids: Vec<BodyId>,
    pub constraints: Vec<PersistentConstraint>,
    pub colliders: Vec<CapsuleColliderDesc>,
}

fn item<'py>(batch: &Bound<'py, PyAny>, key: &str) -> PyResult<Bound<'py, PyAny>> {
    batch.get_item(key).map_err(input_error)
}

fn usize_scalar(batch: &Bound<'_, PyAny>, key: &str) -> PyResult<usize> {
    item(batch, key)?.extract::<usize>().map_err(input_error)
}

fn owned_buffer<T>(
    py: Python<'_>,
    batch: &Bound<'_, PyAny>,
    key: &str,
    dtype: &str,
) -> PyResult<Vec<T>>
where
    T: Element,
{
    let object = item(batch, key)?;
    owned_value_buffer(py, &object, key, dtype)
}

fn owned_value_buffer<T>(
    py: Python<'_>,
    object: &Bound<'_, PyAny>,
    key: &str,
    dtype: &str,
) -> PyResult<Vec<T>>
where
    T: Element,
{
    let view = Bound::<PyMemoryView>::try_from(object)
        .map_err(|_| input_error(format!("{key} must expose a {dtype} buffer")))?;
    let format = view
        .getattr("format")
        .and_then(|value| value.extract::<String>())
        .map_err(input_error)?;
    let expected_format = match dtype {
        "f32" => "f",
        "u32" => "I",
        "u64" => "Q",
        _ => unreachable!("all bridge buffer dtypes are explicit"),
    };
    if format.trim_start_matches(['@', '=', '<', '>', '!']) != expected_format {
        return Err(input_error(format!("{key} must expose a {dtype} buffer")));
    }
    let contiguous = view
        .getattr("c_contiguous")
        .and_then(|value| value.extract::<bool>())
        .map_err(input_error)?;
    if !contiguous {
        return Err(input_error(format!("{key} must be C-contiguous")));
    }
    let byte_count = view
        .getattr("nbytes")
        .and_then(|value| value.extract::<usize>())
        .map_err(input_error)?;
    if byte_count == 0 {
        return Ok(Vec::new());
    }
    let buffer = PyBuffer::<T>::get(object)
        .map_err(|_| input_error(format!("{key} must expose a {dtype} buffer")))?;
    buffer
        .to_vec(py)
        .map_err(|error| input_error(format!("could not copy {key}: {error}")))
}

pub fn decode_mesh_topology(
    py: Python<'_>,
    mesh_id: u32,
    batch: &Bound<'_, PyAny>,
) -> PyResult<DeformingMesh> {
    if mesh_id == 0 {
        return Err(input_error("mesh ID must be greater than zero"));
    }
    let vertex_count = usize_scalar(batch, "vertex_count")?;
    let declared_vertex_count =
        u32::try_from(vertex_count).map_err(|_| input_error("mesh vertex count exceeds u32"))?;
    let triangle_values = owned_buffer::<u32>(py, batch, "triangles", "u32")?;
    if !triangle_values.len().is_multiple_of(3) {
        return Err(input_error(
            "mesh triangle buffer must contain exactly three indices per triangle",
        ));
    }
    let triangles = triangle_values
        .chunks_exact(3)
        .map(|row| [row[0], row[1], row[2]])
        .collect::<Vec<_>>();
    let region_values = owned_buffer::<u32>(py, batch, "regions", "u32")?;
    if region_values.len() != triangles.len() {
        return Err(input_error(format!(
            "mesh region buffer must contain {} items, received {}",
            triangles.len(),
            region_values.len()
        )));
    }
    if region_values.contains(&0) {
        return Err(input_error("mesh region IDs must be greater than zero"));
    }
    let vertex_values = owned_buffer::<f32>(py, batch, "vertices", "f32")?;
    let expected_values = vertex_count
        .checked_mul(3)
        .ok_or_else(|| input_error("mesh vertex value count overflow"))?;
    if vertex_values.len() != expected_values {
        return Err(input_error(format!(
            "mesh vertices must contain {expected_values} f32 items, received {}",
            vertex_values.len()
        )));
    }
    if vertex_values.iter().any(|value| !value.is_finite()) {
        return Err(input_error("mesh vertex values must all be finite"));
    }
    let positions = vertex_values
        .chunks_exact(3)
        .map(|row| Vec3::new(row[0], row[1], row[2]))
        .collect();
    let thickness = item(batch, "thickness")?
        .extract::<f32>()
        .map_err(input_error)?;
    let closed_hint = item(batch, "closed_hint")?
        .extract::<bool>()
        .map_err(input_error)?;

    let ignore_body_ids = owned_buffer::<u32>(py, batch, "ignore_body_ids", "u32")?;
    reject_duplicates("mesh ignored body", &ignore_body_ids).map_err(input_error)?;
    let ignore_offsets = owned_buffer::<u32>(py, batch, "ignore_region_offsets", "u32")?;
    let ignore_region_ids = owned_buffer::<u32>(py, batch, "ignore_region_ids", "u32")?;
    require_lengths(
        "mesh ownership",
        ignore_body_ids.len() + 1,
        &[("ignore_region_offsets", ignore_offsets.len())],
    )?;
    validate_offsets(
        "ignore_region_offsets",
        &ignore_offsets,
        ignore_region_ids.len(),
    )
    .map_err(input_error)?;
    if ignore_body_ids.contains(&0) || ignore_region_ids.contains(&0) {
        return Err(input_error(
            "mesh ownership body and region IDs must be greater than zero",
        ));
    }
    let ignores = ignore_body_ids
        .iter()
        .enumerate()
        .map(|(index, body)| {
            (
                BodyId::new(*body),
                ignore_region_ids
                    [ignore_offsets[index] as usize..ignore_offsets[index + 1] as usize]
                    .iter()
                    .copied()
                    .map(RegionId::new)
                    .collect(),
            )
        })
        .collect();

    let mut mesh = DeformingMesh::build(
        MeshId::new(mesh_id),
        MeshTopology {
            vertex_count: declared_vertex_count,
            triangles,
            triangle_regions: region_values.into_iter().map(RegionId::new).collect(),
            closed_hint,
        },
        positions,
        thickness,
    )
    .map_err(input_error)?;
    mesh.set_body_region_ignores(ignores).map_err(input_error)?;
    Ok(mesh)
}

pub fn decode_mesh_vertices(
    py: Python<'_>,
    previous: &Bound<'_, PyAny>,
    current: &Bound<'_, PyAny>,
    vertex_count: usize,
) -> PyResult<(Vec<Vec3>, Vec<Vec3>)> {
    let previous = owned_value_buffer::<f32>(py, previous, "previous", "f32")?;
    let current = owned_value_buffer::<f32>(py, current, "current", "f32")?;
    let expected = vertex_count
        .checked_mul(3)
        .ok_or_else(|| input_error("mesh vertex value count overflow"))?;
    for (name, values) in [("previous", &previous), ("current", &current)] {
        if values.len() != expected {
            return Err(input_error(format!(
                "mesh {name} vertices must contain {expected} f32 items, received {}",
                values.len()
            )));
        }
        if values.iter().any(|value| !value.is_finite()) {
            return Err(input_error(format!(
                "mesh {name} vertex values must all be finite"
            )));
        }
    }
    let convert = |values: Vec<f32>| {
        values
            .chunks_exact(3)
            .map(|row| Vec3::new(row[0], row[1], row[2]))
            .collect()
    };
    Ok((convert(previous), convert(current)))
}

fn rows(values: &[f32], columns: usize, expected: usize, name: &str) -> PyResult<usize> {
    validate_rows(values.len(), columns, expected, name).map_err(input_error)
}

pub fn validate_rows(
    value_count: usize,
    columns: usize,
    expected: usize,
    name: &str,
) -> Result<usize, BatchValidationError> {
    if columns != expected {
        return Err(BatchValidationError::Columns {
            name: name.to_owned(),
            expected,
            actual: columns,
        });
    }
    if !value_count.is_multiple_of(columns) {
        return Err(BatchValidationError::Ragged {
            name: name.to_owned(),
            values: value_count,
            columns,
        });
    }
    Ok(value_count / columns)
}

fn vec3(row: &[f32], start: usize) -> Vec3 {
    Vec3::new(row[start], row[start + 1], row[start + 2])
}

fn quat(row: &[f32], start: usize) -> Quat {
    Quat::from_xyzw(row[start], row[start + 1], row[start + 2], row[start + 3])
}

fn pose(row: &[f32], start: usize) -> Result<Pose, SolverError> {
    Pose::try_new(vec3(row, start), quat(row, start + 3))
}

fn require_lengths(name: &str, expected: usize, fields: &[(&str, usize)]) -> PyResult<()> {
    if let Some((field, actual)) = fields.iter().find(|(_, actual)| *actual != expected) {
        return Err(input_error(format!(
            "{name}.{field} must contain {expected} items, received {actual}"
        )));
    }
    Ok(())
}

fn decode_filters(
    py: Python<'_>,
    batch: &Bound<'_, PyAny>,
    count: usize,
) -> PyResult<Vec<CollisionFilter>> {
    let layers = owned_buffer::<u32>(py, batch, "layers", "u32")?;
    let masks = owned_buffer::<u32>(py, batch, "masks", "u32")?;
    let body_offsets = owned_buffer::<u32>(py, batch, "excluded_body_offsets", "u32")?;
    let body_ids = owned_buffer::<u32>(py, batch, "excluded_body_ids", "u32")?;
    let constraint_offsets = owned_buffer::<u32>(py, batch, "excluded_constraint_offsets", "u32")?;
    let constraint_ids = owned_buffer::<u32>(py, batch, "excluded_constraint_ids", "u32")?;
    require_lengths(
        "filters",
        count,
        &[("layers", layers.len()), ("masks", masks.len())],
    )?;
    require_lengths(
        "filters",
        count + 1,
        &[
            ("excluded_body_offsets", body_offsets.len()),
            ("excluded_constraint_offsets", constraint_offsets.len()),
        ],
    )?;
    validate_offsets("excluded_body_offsets", &body_offsets, body_ids.len())
        .map_err(input_error)?;
    validate_offsets(
        "excluded_constraint_offsets",
        &constraint_offsets,
        constraint_ids.len(),
    )
    .map_err(input_error)?;

    Ok((0..count)
        .map(|index| CollisionFilter {
            layer: layers[index],
            mask: masks[index],
            excluded_bodies: body_ids
                [body_offsets[index] as usize..body_offsets[index + 1] as usize]
                .iter()
                .copied()
                .map(BodyId::new)
                .collect(),
            excluded_constraints: constraint_ids
                [constraint_offsets[index] as usize..constraint_offsets[index + 1] as usize]
                .iter()
                .copied()
                .map(ConstraintId::new)
                .collect(),
        })
        .collect())
}

pub fn validate_offsets(
    name: &str,
    offsets: &[u32],
    item_count: usize,
) -> Result<(), BatchValidationError> {
    if offsets.first().copied() != Some(0)
        || offsets.last().copied() != u32::try_from(item_count).ok()
        || offsets.windows(2).any(|pair| pair[0] > pair[1])
    {
        return Err(BatchValidationError::Offsets {
            name: name.to_owned(),
            item_count,
        });
    }
    Ok(())
}

pub fn decode_world(
    py: Python<'_>,
    bodies_batch: &Bound<'_, PyAny>,
    constraints_batch: &Bound<'_, PyAny>,
    filters_batch: &Bound<'_, PyAny>,
) -> PyResult<DecodedWorld> {
    let body_ids = owned_buffer::<u32>(py, bodies_batch, "body_ids", "u32")?;
    let collider_ids = owned_buffer::<u32>(py, bodies_batch, "collider_ids", "u32")?;
    let values = owned_buffer::<f32>(py, bodies_batch, "values", "f32")?;
    let count = rows(
        &values,
        usize_scalar(bodies_batch, "columns")?,
        BODY_COLUMNS,
        "bodies",
    )?;
    require_lengths(
        "bodies",
        count,
        &[
            ("body_ids", body_ids.len()),
            ("collider_ids", collider_ids.len()),
        ],
    )?;
    reject_duplicates("body", &body_ids).map_err(input_error)?;
    reject_duplicates("collider", &collider_ids).map_err(input_error)?;
    if values.iter().any(|value| !value.is_finite()) {
        return Err(input_error("body values must all be finite"));
    }

    let filters = decode_filters(py, filters_batch, count)?;
    let mut bodies = Vec::with_capacity(count);
    let mut colliders = Vec::with_capacity(count);
    for (index, row) in values.chunks_exact(BODY_COLUMNS).enumerate() {
        let body_id = BodyId::new(body_ids[index]);
        bodies.push(BodyInit {
            desc: BoneBodyDesc {
                id: body_id,
                rest_pose: pose(row, 0).map_err(input_error)?,
                local_center_of_mass: vec3(row, 7),
                local_head: vec3(row, 10),
                local_tail: vec3(row, 13),
                inverse_mass: row[16],
                inverse_inertia: vec3(row, 17),
                gravity_scale: row[20],
                linear_damping: row[21],
                angular_damping: row[22],
            },
            pose: pose(row, 23).map_err(input_error)?,
            velocity: Velocity {
                linear: vec3(row, 30),
                angular: vec3(row, 33),
            },
        });
        colliders.push(CapsuleColliderDesc {
            id: ColliderId::new(collider_ids[index]),
            body: body_id,
            shape: Capsule::try_new(vec3(row, 36), vec3(row, 39), row[42], row[43])
                .map_err(input_error)?,
            material: ContactMaterial::try_new(row[44], row[45]).map_err(input_error)?,
            filter: filters[index].clone(),
        });
    }

    let constraints = decode_constraints(py, constraints_batch)?;
    Ok(DecodedWorld {
        body_ids: body_ids.into_iter().map(BodyId::new).collect(),
        bodies,
        constraints,
        colliders,
    })
}

fn decode_constraints(
    py: Python<'_>,
    batch: &Bound<'_, PyAny>,
) -> PyResult<Vec<PersistentConstraint>> {
    let ids = owned_buffer::<u32>(py, batch, "ids", "u32")?;
    let body_ids = owned_buffer::<u32>(py, batch, "body_ids", "u32")?;
    let kinds = owned_buffer::<u32>(py, batch, "kinds", "u32")?;
    let enabled = owned_buffer::<u32>(py, batch, "enabled", "u32")?;
    let values = owned_buffer::<f32>(py, batch, "values", "f32")?;
    let count = rows(
        &values,
        usize_scalar(batch, "columns")?,
        CONSTRAINT_COLUMNS,
        "constraints",
    )?;
    require_lengths(
        "constraints",
        count,
        &[
            ("ids", ids.len()),
            ("kinds", kinds.len()),
            ("enabled", enabled.len()),
        ],
    )?;
    require_lengths("constraints", count * 2, &[("body_ids", body_ids.len())])?;
    reject_duplicates("constraint", &ids).map_err(input_error)?;
    if values.iter().any(|value| !value.is_finite()) {
        return Err(input_error("constraint values must all be finite"));
    }

    values
        .chunks_exact(CONSTRAINT_COLUMNS)
        .enumerate()
        .map(|(index, row)| {
            let id = ConstraintId::new(ids[index]);
            let body_a = BodyId::new(body_ids[index * 2]);
            let body_b = BodyId::new(body_ids[index * 2 + 1]);
            let compliance = row[8];
            let damping = row[9];
            let local_a = vec3(row, 0);
            let local_b = vec3(row, 3);
            let mut constraint = match kinds[index] {
                0 => PersistentConstraint::anchor(
                    id, body_a, body_b, local_a, local_b, compliance, damping,
                ),
                1 => PersistentConstraint::distance(
                    id, body_a, body_b, local_a, local_b, row[6], compliance, damping,
                ),
                2 => PersistentConstraint::distance_range(
                    id, body_a, body_b, local_a, local_b, row[6], row[7], compliance, damping,
                ),
                3 => PersistentConstraint::attachment(
                    id, body_a, local_a, local_b, compliance, damping,
                ),
                4 => PersistentConstraint::orientation_recovery(id, body_a, compliance, damping),
                5 => PersistentConstraint::angular_limit(
                    id,
                    body_a,
                    body_b,
                    quat(row, 10),
                    AngularLimit::XyzBox(XyzLimit {
                        lower: vec3(row, 14),
                        upper: vec3(row, 17),
                        order: decode_euler_order(row[20])?,
                    }),
                    compliance,
                    damping,
                ),
                6 => PersistentConstraint::angular_limit(
                    id,
                    body_a,
                    body_b,
                    quat(row, 10),
                    AngularLimit::SwingTwist(SwingTwistLimit {
                        axis: vec3(row, 14),
                        max_swing: row[17],
                        min_twist: row[18],
                        max_twist: row[19],
                    }),
                    compliance,
                    damping,
                ),
                kind => {
                    return Err(input_error(format!(
                        "constraint kind {kind} is not supported"
                    )));
                }
            };
            constraint.enabled = match enabled[index] {
                0 => false,
                1 => true,
                value => {
                    return Err(input_error(format!(
                        "constraint enabled flag must be 0 or 1, received {value}"
                    )));
                }
            };
            Ok(constraint)
        })
        .collect()
}

fn decode_euler_order(raw: f32) -> PyResult<EulerOrder> {
    match raw {
        0.0 => Ok(EulerOrder::Xyz),
        1.0 => Ok(EulerOrder::Xzy),
        2.0 => Ok(EulerOrder::Yxz),
        3.0 => Ok(EulerOrder::Yzx),
        4.0 => Ok(EulerOrder::Zxy),
        5.0 => Ok(EulerOrder::Zyx),
        _ => Err(input_error(format!(
            "Euler order must be an integer from 0 through 5, received {raw}"
        ))),
    }
}

pub fn reject_duplicates(kind: &str, ids: &[u32]) -> Result<(), BatchValidationError> {
    let mut seen = BTreeSet::new();
    if let Some(id) = ids.iter().copied().find(|id| !seen.insert(*id)) {
        return Err(BatchValidationError::DuplicateId {
            kind: kind.to_owned(),
            id,
        });
    }
    Ok(())
}

pub fn decode_targets(
    py: Python<'_>,
    batch: &Bound<'_, PyAny>,
) -> PyResult<Vec<(BodyId, bone_physics_core::AnimationTarget)>> {
    let ids = owned_buffer::<u32>(py, batch, "body_ids", "u32")?;
    let values = owned_buffer::<f32>(py, batch, "values", "f32")?;
    let count = rows(
        &values,
        usize_scalar(batch, "columns")?,
        TARGET_COLUMNS,
        "animation targets",
    )?;
    require_lengths("animation targets", count, &[("body_ids", ids.len())])?;
    reject_duplicates("animation target", &ids).map_err(input_error)?;
    if values.iter().any(|value| !value.is_finite()) {
        return Err(input_error("animation target values must all be finite"));
    }
    values
        .chunks_exact(TARGET_COLUMNS)
        .enumerate()
        .map(|(index, row)| {
            Ok((
                BodyId::new(ids[index]),
                bone_physics_core::AnimationTarget {
                    previous: pose(row, 0).map_err(input_error)?,
                    current: pose(row, 7).map_err(input_error)?,
                    translation_inertia: row[14],
                    rotation_inertia: row[15],
                },
            ))
        })
        .collect()
}

pub fn decode_analytic_colliders(
    py: Python<'_>,
    batch: &Bound<'_, PyAny>,
) -> PyResult<Vec<AnalyticProxy>> {
    let ids = owned_buffer::<u32>(py, batch, "ids", "u32")?;
    let kinds = owned_buffer::<u32>(py, batch, "kinds", "u32")?;
    let enabled = owned_buffer::<u32>(py, batch, "enabled", "u32")?;
    let values = owned_buffer::<f32>(py, batch, "values", "f32")?;
    let count = rows(
        &values,
        usize_scalar(batch, "columns")?,
        ANALYTIC_COLLIDER_COLUMNS,
        "analytic colliders",
    )?;
    require_lengths(
        "analytic colliders",
        count,
        &[
            ("ids", ids.len()),
            ("kinds", kinds.len()),
            ("enabled", enabled.len()),
        ],
    )?;
    reject_duplicates("analytic collider", &ids).map_err(input_error)?;
    if values.iter().any(|value| !value.is_finite()) {
        return Err(input_error("analytic collider values must all be finite"));
    }

    values
        .chunks_exact(ANALYTIC_COLLIDER_COLUMNS)
        .enumerate()
        .map(|(index, row)| {
            let shape = match kinds[index] {
                0 => AnalyticShape::Sphere {
                    radius: nonnegative(row[10], "analytic sphere radius")?,
                },
                1 => AnalyticShape::Capsule {
                    local_a: vec3(row, 10),
                    local_b: vec3(row, 13),
                    radius: nonnegative(row[16], "analytic capsule radius")?,
                },
                2 => AnalyticShape::RoundedBox {
                    half_extents: Vec3::new(
                        nonnegative(row[10], "rounded box half extent")?,
                        nonnegative(row[11], "rounded box half extent")?,
                        nonnegative(row[12], "rounded box half extent")?,
                    ),
                    radius: nonnegative(row[13], "rounded box radius")?,
                },
                kind => {
                    return Err(input_error(format!(
                        "analytic collider kind {kind} is not supported"
                    )));
                }
            };
            let collision_enabled = match enabled[index] {
                0 => false,
                1 => true,
                value => {
                    return Err(input_error(format!(
                        "analytic collider enabled flag must be 0 or 1, received {value}"
                    )));
                }
            };
            Ok(AnalyticProxy {
                id: ColliderId::new(ids[index]),
                pose: pose(row, 0).map_err(input_error)?,
                linear_velocity: vec3(row, 7),
                shape,
                collision_enabled,
            })
        })
        .collect()
}

fn nonnegative(value: f32, field: &str) -> PyResult<f32> {
    if value >= 0.0 {
        Ok(value)
    } else {
        Err(input_error(format!("{field} must be nonnegative")))
    }
}

pub fn write_transforms(py: Python<'_>, output: &Bound<'_, PyAny>, values: &[f32]) -> PyResult<()> {
    let view = Bound::<PyMemoryView>::try_from(output)
        .map_err(|_| input_error("transform output must expose a writable f32 buffer"))?;
    let format = view
        .getattr("format")
        .and_then(|value| value.extract::<String>())
        .map_err(input_error)?;
    if format.trim_start_matches(['@', '=', '<', '>', '!']) != "f" {
        return Err(input_error(
            "transform output must expose a writable f32 buffer",
        ));
    }
    let readonly = view
        .getattr("readonly")
        .and_then(|value| value.extract::<bool>())
        .map_err(input_error)?;
    if readonly {
        return Err(input_error("transform output must be writable"));
    }
    let contiguous = view
        .getattr("c_contiguous")
        .and_then(|value| value.extract::<bool>())
        .map_err(input_error)?;
    if !contiguous {
        return Err(input_error("transform output must be C-contiguous"));
    }
    let item_count = view
        .getattr("nbytes")
        .and_then(|value| value.extract::<usize>())
        .map_err(input_error)?
        / core::mem::size_of::<f32>();
    if item_count != values.len() {
        return Err(input_error(format!(
            "transform output must contain exactly {} f32 items",
            values.len()
        )));
    }
    if values.is_empty() {
        return Ok(());
    }
    let buffer = PyBuffer::<f32>::get(output)
        .map_err(|_| input_error("transform output must expose a writable f32 buffer"))?;
    buffer.copy_from_slice(py, values).map_err(input_error)
}
