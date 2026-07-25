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

