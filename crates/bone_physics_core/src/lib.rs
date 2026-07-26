#![forbid(unsafe_code)]

mod body;
mod chain;
mod constraint;
mod error;
mod id;
mod math;
mod parameter;
mod pose;
mod time;
mod world;

pub use body::{BoneBodyDesc, BoneBodyState};
pub use chain::{build_cross_links, sample_chain_s, BodyLookup, ChainDesc, CrossLinkSpec};
pub use constraint::{
    project_angular_limit, solve_scalar_row, AngularLimit, AnimationTarget, ConstraintKind,
    ContinuousAngles, EulerOrder, PersistentConstraint, RowResult, SwingTwistLimit, XpbdGradient,
    XpbdRow, XyzLimit,
};
pub use error::SolverError;
pub use id::{BodyId, ColliderId, ConstraintId};
pub use math::{quat_exp, quat_log, world_inverse_inertia};
pub use parameter::{CurveInterpolation, CurvePoint, ParameterCurve};
pub use pose::{Pose, Velocity};
pub use time::{FixedClock, QualityPolicy, StepBatch};
pub use world::{
    BodyInit, CorrectionKind, SolverConfig, SolverWorld, StepDiagnostics, StepOptions,
};
