use std::any::Any;
use std::panic::{AssertUnwindSafe, catch_unwind};

use pyo3::create_exception;
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;

create_exception!(
    bone_physics_native,
    BonePhysicsInputError,
    PyValueError
);
create_exception!(
    bone_physics_native,
    BonePhysicsStateError,
    PyRuntimeError
);
create_exception!(
    bone_physics_native,
    BonePhysicsInternalError,
    PyRuntimeError
);

pub(crate) fn input_error(error: impl std::fmt::Display) -> PyErr {
    BonePhysicsInputError::new_err(error.to_string())
}

pub(crate) fn state_error(error: impl std::fmt::Display) -> PyErr {
    BonePhysicsStateError::new_err(error.to_string())
}

fn panic_message(payload: &(dyn Any + Send)) -> &str {
    payload
        .downcast_ref::<&str>()
        .copied()
        .or_else(|| payload.downcast_ref::<String>().map(String::as_str))
        .unwrap_or("native solver panicked without a string payload")
}

pub(crate) fn catch_panic<T>(
    operation: impl FnOnce() -> PyResult<T>,
) -> PyResult<T> {
    match catch_unwind(AssertUnwindSafe(operation)) {
        Ok(result) => result,
        Err(payload) => Err(BonePhysicsInternalError::new_err(format!(
            "native solver panic: {}",
            panic_message(payload.as_ref())
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn panic_is_translated_to_internal_error() {
        let error = catch_panic(|| -> PyResult<()> {
            panic!("injected bridge panic");
        })
        .unwrap_err();

        let message = error.to_string();
        assert!(message.contains("native solver panic"));
        assert!(message.contains("injected bridge panic"));
    }
}
