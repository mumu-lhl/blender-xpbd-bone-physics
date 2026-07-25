use crate::BodyId;

#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SolverError {
    #[error("duplicate {kind} ID {id}")]
    DuplicateId { kind: &'static str, id: u32 },

    #[error("invalid value for {field}: {value}")]
    InvalidValue { field: &'static str, value: f64 },

    #[error("body {0} does not exist")]
    MissingBody(u32),

    #[error("body {0:?} contains non-finite state")]
    NonFiniteState(BodyId),
}

impl SolverError {
    #[must_use]
    pub fn invalid_value(field: &'static str, value: impl Into<f64>) -> Self {
        Self::InvalidValue {
            field,
            value: value.into(),
        }
    }
}

pub(crate) fn validate_finite(field: &'static str, value: f32) -> Result<(), SolverError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(SolverError::invalid_value(field, value))
    }
}

pub(crate) fn validate_nonnegative(field: &'static str, value: f32) -> Result<(), SolverError> {
    validate_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(SolverError::invalid_value(field, value))
    }
}
