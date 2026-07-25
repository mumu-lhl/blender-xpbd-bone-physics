#![forbid(unsafe_code)]

mod body;
mod constraint;
mod error;
mod id;
mod math;
mod pose;
mod time;
mod world;

pub use body::{BoneBodyDesc, BoneBodyState};
pub use constraint::{
    solve_scalar_row, AnimationTarget, ConstraintKind, PersistentConstraint,
    RowResult, XpbdGradient, XpbdRow,
};
pub use error::SolverError;
pub use id::{BodyId, ColliderId, ConstraintId};
pub use math::{quat_exp, quat_log, world_inverse_inertia};
pub use pose::{Pose, Velocity};
pub use time::{FixedClock, QualityPolicy, StepBatch};
pub use world::{
    BodyInit, CorrectionKind, SolverConfig, SolverWorld, StepDiagnostics,
    StepOptions,
};
