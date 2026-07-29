mod error;
mod registry;

use std::sync::{Mutex, OnceLock};

use bone_physics_core::{SolverConfig, SolverWorld};
use error::{
    BonePhysicsInputError, BonePhysicsInternalError, BonePhysicsStateError,
    catch_panic, input_error, state_error,
};
use glam::Vec3;
use pyo3::prelude::*;
use pyo3::types::PyAnyMethods;

pub use registry::{RegistryError, WorldHandle, WorldRegistry};

pub const NATIVE_API_VERSION: u32 = 1;

static WORLDS: OnceLock<Mutex<WorldRegistry>> = OnceLock::new();

fn worlds() -> &'static Mutex<WorldRegistry> {
    WORLDS.get_or_init(|| Mutex::new(WorldRegistry::new()))
}

fn lock_worlds() -> PyResult<std::sync::MutexGuard<'static, WorldRegistry>> {
    worlds().lock().map_err(|_| {
        BonePhysicsInternalError::new_err(
            "native world registry lock was poisoned",
        )
    })
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
fn create_world(
    config: &Bound<'_, PyAny>,
    build_batch: &Bound<'_, PyAny>,
) -> PyResult<u64> {
    catch_panic(|| {
        if build_batch.len().map_err(input_error)? != 0 {
            return Err(input_error(
                "build_batch must be empty until the batch API is supplied",
            ));
        }
        let config = extract_config(config)?;
        let world =
            SolverWorld::new(config, Vec::new()).map_err(input_error)?;
        let handle = lock_worlds()?
            .insert(world)
            .map_err(state_error)?;
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

#[cfg(feature = "test-panic")]
#[pyfunction]
fn _trigger_test_panic() -> PyResult<()> {
    catch_panic(|| -> PyResult<()> {
        panic!("injected Python bridge panic");
    })
}

#[pymodule]
fn bone_physics_native(
    py: Python<'_>,
    module: &Bound<'_, PyModule>,
) -> PyResult<()> {
    module.add(
        "BonePhysicsInputError",
        py.get_type::<BonePhysicsInputError>(),
    )?;
    module.add(
        "BonePhysicsStateError",
        py.get_type::<BonePhysicsStateError>(),
    )?;
    module.add(
        "BonePhysicsInternalError",
        py.get_type::<BonePhysicsInternalError>(),
    )?;
    module.add_function(wrap_pyfunction!(native_api_version, module)?)?;
    module.add_function(wrap_pyfunction!(create_world, module)?)?;
    module.add_function(wrap_pyfunction!(destroy_world, module)?)?;
    #[cfg(feature = "test-panic")]
    module.add_function(wrap_pyfunction!(_trigger_test_panic, module)?)?;
    Ok(())
}
