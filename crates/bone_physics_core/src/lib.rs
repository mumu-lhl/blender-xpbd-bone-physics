#![forbid(unsafe_code)]

mod error;
mod id;

pub use error::SolverError;
pub use id::{BodyId, ColliderId, ConstraintId};
