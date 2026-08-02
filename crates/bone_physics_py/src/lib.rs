mod batch;
mod error;
mod registry;
mod world;

use std::sync::{Mutex, OnceLock};

use batch::{
    decode_analytic_colliders, decode_mesh_topology, decode_mesh_vertices, decode_targets,
    decode_world, write_transforms,
};
use bone_physics_core::{
    MeshId, MeshUpdate, QualityPolicy, SolverConfig, SolverError, SolverWorld,
};
use error::{
    BonePhysicsInputError, BonePhysicsInternalError, BonePhysicsStateError,
    BonePhysicsTopologyInvalidatedError, catch_panic, input_error, state_error,
    topology_invalidated_error,
};
use glam::Vec3;
use pyo3::prelude::*;
use pyo3::types::{PyAnyMethods, PyBytes, PyDict, PyDictMethods};
use world::{AdvanceSummary, NativeWorld};

pub use batch::{
    ANALYTIC_COLLIDER_COLUMNS, BODY_COLUMNS, BatchValidationError, CONSTRAINT_COLUMNS,
    TARGET_COLUMNS, reject_duplicates, validate_offsets, validate_rows,
};
pub use registry::{RegistryError, WorldHandle, WorldRegistry};

pub const NATIVE_API_VERSION: u32 = 1;

static WORLDS: OnceLock<Mutex<WorldRegistry<NativeWorld>>> = OnceLock::new();

fn worlds() -> &'static Mutex<WorldRegistry<NativeWorld>> {
    WORLDS.get_or_init(|| Mutex::new(WorldRegistry::new()))
}

fn lock_worlds() -> PyResult<std::sync::MutexGuard<'static, WorldRegistry<NativeWorld>>> {
    worlds()
        .lock()
        .map_err(|_| BonePhysicsInternalError::new_err("native world registry lock was poisoned"))
}

#[pyfunction]
fn native_api_version() -> u32 {
    NATIVE_API_VERSION
}

fn extract_config(config: &Bound<'_, PyAny>) -> PyResult<SolverConfig> {
    let gravity = config
        .get_item("gravity")
        .and_then(|value| value.extract::<(f32, f32, f32)>())
        .map_err(input_error)?;
    let fixed_dt = config
        .get_item("fixed_dt")
        .and_then(|value| value.extract::<f32>())
        .map_err(input_error)?;
    Ok(SolverConfig {
        gravity: Vec3::new(gravity.0, gravity.1, gravity.2),
        fixed_dt,
    })
}

#[pyfunction]
fn create_world(config: &Bound<'_, PyAny>, build_batch: &Bound<'_, PyAny>) -> PyResult<u64> {
    catch_panic(|| {
        if build_batch.len().map_err(input_error)? != 0 {
            return Err(input_error(
                "build_batch must be empty until the batch API is supplied",
            ));
        }
        let config = extract_config(config)?;
        let solver = SolverWorld::new(config, Vec::new()).map_err(input_error)?;
        let world = NativeWorld::new(solver, Vec::new(), Vec::new(), config.fixed_dt)
            .map_err(input_error)?;
        let handle = lock_worlds()?.insert(world).map_err(state_error)?;
        Ok(handle.get())
    })
}

#[pyfunction]
fn build_world(
    py: Python<'_>,
    config: &Bound<'_, PyAny>,
    bodies: &Bound<'_, PyAny>,
    constraints: &Bound<'_, PyAny>,
    filters: &Bound<'_, PyAny>,
) -> PyResult<u64> {
    catch_panic(|| {
        let api_version = config
            .get_item("api_version")
            .and_then(|value| value.extract::<u32>())
            .map_err(input_error)?;
        if api_version != NATIVE_API_VERSION {
            return Err(input_error(format!(
                "native API version mismatch: expected {NATIVE_API_VERSION}, received {api_version}"
            )));
        }
        let config = extract_config(config)?;
        let decoded = decode_world(py, bodies, constraints, filters)?;
        let collider_ids = decoded
            .colliders
            .iter()
            .map(|collider| collider.id)
            .collect();
        let mut solver = SolverWorld::new(config, decoded.bodies).map_err(input_error)?;
        solver
            .set_constraints(decoded.constraints)
            .map_err(input_error)?;
        solver
            .set_colliders(decoded.colliders)
            .map_err(input_error)?;
        let world = NativeWorld::new(solver, decoded.body_ids, collider_ids, config.fixed_dt)
            .map_err(input_error)?;
        let handle = lock_worlds()?.insert(world).map_err(state_error)?;
        Ok(handle.get())
    })
}

#[pyfunction]
fn destroy_world(handle: u64) -> PyResult<()> {
    catch_panic(|| {
        lock_worlds()?
            .remove(WorldHandle::from_raw(handle))
            .map_err(state_error)?;
        Ok(())
    })
}

#[pyfunction]
fn resume_world(handle: u64) -> PyResult<()> {
    catch_panic(|| {
        lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), NativeWorld::resume)
            .map_err(state_error)?
            .map_err(operation_error)
    })
}

#[pyfunction]
fn set_animation_targets(
    py: Python<'_>,
    handle: u64,
    pose_batch: &Bound<'_, PyAny>,
) -> PyResult<()> {
    catch_panic(|| {
        let targets = decode_targets(py, pose_batch)?;
        lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.set_animation_targets(&targets)
            })
            .map_err(state_error)?
            .map_err(operation_error)
    })
}

#[pyfunction]
fn set_analytic_colliders(
    py: Python<'_>,
    handle: u64,
    collider_batch: &Bound<'_, PyAny>,
) -> PyResult<()> {
    catch_panic(|| {
        let colliders = decode_analytic_colliders(py, collider_batch)?;
        lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.set_analytic_colliders(colliders)
            })
            .map_err(state_error)?
            .map_err(operation_error)
    })
}

#[pyfunction]
fn set_mesh_topology(
    py: Python<'_>,
    handle: u64,
    mesh_id: u32,
    topology_batch: &Bound<'_, PyAny>,
) -> PyResult<u64> {
    catch_panic(|| {
        let mesh = decode_mesh_topology(py, mesh_id, topology_batch)?;
        lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| world.set_mesh(mesh))
            .map_err(state_error)?
            .map_err(operation_error)
    })
}

#[pyfunction]
fn update_mesh_vertices(
    py: Python<'_>,
    handle: u64,
    mesh_id: u32,
    previous: &Bound<'_, PyAny>,
    current: &Bound<'_, PyAny>,
    expected_hash: u64,
) -> PyResult<&'static str> {
    catch_panic(|| {
        let id = MeshId::new(mesh_id);
        let (vertex_count, actual_hash) = lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                (world.mesh_vertex_count(id), world.mesh_topology_hash(id))
            })
            .map_err(state_error)?;
        let vertex_count =
            vertex_count.ok_or_else(|| input_error(format!("mesh ID {mesh_id} does not exist")))?;
        let actual_hash =
            actual_hash.ok_or_else(|| input_error(format!("mesh ID {mesh_id} does not exist")))?;
        if expected_hash != actual_hash {
            return Err(topology_invalidated_error(format!(
                "mesh topology hash mismatch: expected {actual_hash}, received {expected_hash}"
            )));
        }
        let (previous, current) = decode_mesh_vertices(py, previous, current, vertex_count)?;
        let update = lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.update_mesh_vertices(id, previous, current, expected_hash)
            })
            .map_err(state_error)?
            .map_err(operation_error)?;
        Ok(match update {
            MeshUpdate::Refit => "refit",
            MeshUpdate::Rebuilt => "rebuilt",
        })
    })
}

#[pyfunction]
fn remove_mesh(handle: u64, mesh_id: u32) -> PyResult<()> {
    catch_panic(|| {
        lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.remove_mesh(MeshId::new(mesh_id))
            })
            .map_err(state_error)?
            .map_err(operation_error)
    })
}

#[pyfunction]
fn advance(
    py: Python<'_>,
    handle: u64,
    frame_seconds: f32,
    quality: &Bound<'_, PyAny>,
) -> PyResult<Py<PyDict>> {
    catch_panic(|| {
        let quality = extract_quality(quality)?;
        let summary = lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.advance(frame_seconds, quality)
            })
            .map_err(state_error)?
            .map_err(operation_error)?;
        summary_dict(py, summary)
    })
}

#[pyfunction]
fn read_transforms(
    py: Python<'_>,
    handle: u64,
    writable_pose_batch: &Bound<'_, PyAny>,
) -> PyResult<()> {
    catch_panic(|| {
        let values = lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.transform_values()
            })
            .map_err(state_error)?;
        write_transforms(py, writable_pose_batch, &values)
    })
}

#[pyfunction]
fn read_diagnostics(py: Python<'_>, handle: u64) -> PyResult<Py<PyDict>> {
    catch_panic(|| {
        let (body_count, analytic_collider_count, mesh_count, summary) = lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                (
                    world.body_count(),
                    world.analytic_collider_count(),
                    world.mesh_count(),
                    world.last_summary(),
                )
            })
            .map_err(state_error)?;
        let result = PyDict::new(py);
        result.set_item("body_count", body_count)?;
        result.set_item("analytic_collider_count", analytic_collider_count)?;
        result.set_item("mesh_count", mesh_count)?;
        add_summary(&result, summary)?;
        Ok(result.unbind())
    })
}

#[pyfunction]
fn world_fingerprint(py: Python<'_>, handle: u64) -> PyResult<Py<PyBytes>> {
    catch_panic(|| {
        let fingerprint = lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| world.world_fingerprint())
            .map_err(state_error)?;
        Ok(PyBytes::new(py, &fingerprint).unbind())
    })
}

#[pyfunction]
fn encode_checkpoint(
    py: Python<'_>,
    handle: u64,
    quality: &Bound<'_, PyAny>,
) -> PyResult<Py<PyBytes>> {
    catch_panic(|| {
        let quality = extract_quality(quality)?;
        let checkpoint = lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.encode_checkpoint(quality)
            })
            .map_err(state_error)?
            .map_err(operation_error)?;
        Ok(PyBytes::new(py, &checkpoint).unbind())
    })
}

#[pyfunction]
fn restore_checkpoint(
    handle: u64,
    checkpoint: &Bound<'_, PyAny>,
    fingerprint: &Bound<'_, PyAny>,
) -> PyResult<()> {
    catch_panic(|| {
        let checkpoint = checkpoint.extract::<Vec<u8>>().map_err(input_error)?;
        let fingerprint = fingerprint.extract::<Vec<u8>>().map_err(input_error)?;
        let fingerprint: [u8; 32] = fingerprint
            .try_into()
            .map_err(|_| input_error("checkpoint fingerprint must contain 32 bytes"))?;
        lock_worlds()?
            .with_world_mut(WorldHandle::from_raw(handle), |world| {
                world.restore_checkpoint(&checkpoint, fingerprint)
            })
            .map_err(state_error)?
            .map_err(operation_error)
    })
}

fn extract_quality(quality: &Bound<'_, PyAny>) -> PyResult<QualityPolicy> {
    let name = quality.extract::<String>().map_err(input_error)?;
    match name.as_str() {
        "deterministic_preview" => Ok(QualityPolicy::DeterministicPreview),
        "approximate_preview" => Ok(QualityPolicy::ApproximatePreview { max_steps: 8 }),
        "authoritative_bake" => Ok(QualityPolicy::AuthoritativeBake),
        _ => Err(input_error(format!("unsupported quality policy {name:?}"))),
    }
}

fn operation_error(error: SolverError) -> PyErr {
    match error {
        SolverError::WorldPaused | SolverError::NonFiniteState(_) => state_error(error),
        _ => input_error(error),
    }
}

fn summary_dict(py: Python<'_>, summary: AdvanceSummary) -> PyResult<Py<PyDict>> {
    let result = PyDict::new(py);
    add_summary(&result, summary)?;
    Ok(result.unbind())
}

fn add_summary(result: &Bound<'_, PyDict>, summary: AdvanceSummary) -> PyResult<()> {
    result.set_item("steps", summary.steps)?;
    result.set_item("dropped", summary.dropped)?;
    result.set_item("alpha", summary.alpha)?;
    result.set_item("generated_contacts", summary.generated_contacts)?;
    result.set_item("active_contacts", summary.active_contacts)?;
    result.set_item("deepest_penetration", summary.deepest_penetration)?;
    result.set_item("max_constraint_error", summary.max_constraint_error)?;
    result.set_item("broad_phase_candidates", summary.broad_phase_candidates)?;
    result.set_item("ccd_fallbacks", summary.ccd_fallbacks)?;
    result.set_item("ccd_failures", summary.ccd_failures)?;
    result.set_item("substep_profiled", summary.substep_profiled)?;
    let timings = PyDict::new(result.py());
    timings.set_item("prediction_seconds", summary.phase_timings.prediction_seconds)?;
    timings.set_item(
        "animation_inertia_seconds",
        summary.phase_timings.animation_inertia_seconds,
    )?;
    timings.set_item(
        "contact_generation_seconds",
        summary.phase_timings.contact_generation_seconds,
    )?;
    timings.set_item(
        "position_solve_seconds",
        summary.phase_timings.position_solve_seconds,
    )?;
    timings.set_item("diagnostics_seconds", summary.phase_timings.diagnostics_seconds)?;
    timings.set_item("velocity_seconds", summary.phase_timings.velocity_seconds)?;
    result.set_item("phase_timings", timings)?;
    result.set_item("rolled_back", summary.rolled_back)
}

#[cfg(feature = "test-panic")]
#[pyfunction]
fn _trigger_test_panic() -> PyResult<()> {
    catch_panic(|| -> PyResult<()> {
        panic!("injected Python bridge panic");
    })
}

#[pymodule]
fn bone_physics_native(py: Python<'_>, module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add(
        "BonePhysicsInputError",
        py.get_type::<BonePhysicsInputError>(),
    )?;
    module.add(
        "BonePhysicsStateError",
        py.get_type::<BonePhysicsStateError>(),
    )?;
    module.add(
        "BonePhysicsTopologyInvalidatedError",
        py.get_type::<BonePhysicsTopologyInvalidatedError>(),
    )?;
    module.add(
        "BonePhysicsInternalError",
        py.get_type::<BonePhysicsInternalError>(),
    )?;
    module.add_function(wrap_pyfunction!(native_api_version, module)?)?;
    module.add_function(wrap_pyfunction!(create_world, module)?)?;
    module.add_function(wrap_pyfunction!(build_world, module)?)?;
    module.add_function(wrap_pyfunction!(destroy_world, module)?)?;
    module.add_function(wrap_pyfunction!(resume_world, module)?)?;
    module.add_function(wrap_pyfunction!(set_animation_targets, module)?)?;
    module.add_function(wrap_pyfunction!(set_analytic_colliders, module)?)?;
    module.add_function(wrap_pyfunction!(set_mesh_topology, module)?)?;
    module.add_function(wrap_pyfunction!(update_mesh_vertices, module)?)?;
    module.add_function(wrap_pyfunction!(remove_mesh, module)?)?;
    module.add_function(wrap_pyfunction!(advance, module)?)?;
    module.add_function(wrap_pyfunction!(read_transforms, module)?)?;
    module.add_function(wrap_pyfunction!(read_diagnostics, module)?)?;
    module.add_function(wrap_pyfunction!(world_fingerprint, module)?)?;
    module.add_function(wrap_pyfunction!(encode_checkpoint, module)?)?;
    module.add_function(wrap_pyfunction!(restore_checkpoint, module)?)?;
    #[cfg(feature = "test-panic")]
    module.add_function(wrap_pyfunction!(_trigger_test_panic, module)?)?;
    Ok(())
}
