#![forbid(unsafe_code)]

mod body;
mod cache;
mod chain;
mod collision;
mod constraint;
mod error;
mod graph;
mod hash;
mod id;
mod math;
mod parameter;
mod pose;
mod time;
mod world;

pub use body::{BoneBodyDesc, BoneBodyState};
pub use chain::{
    BodyLookup, ChainDesc, CrossLinkLatticeSpec, CrossLinkSpec, build_cross_link_lattice,
    build_cross_links, sample_chain_s,
};
pub use collision::{
    AnalyticContact, AnalyticProxy, AnalyticShape, BroadPhaseBackend, CachedContact, Capsule,
    CapsuleCollider, CapsuleColliderDesc, CapsuleManifold, CcdGeometry, CcdResult, CollisionFilter,
    CollisionPipeline, ContactGeometry, ContactKey, ContactLambdas, ContactMaterial, DeformingMesh,
    MeshContactFeature, MeshContactGeometry, MeshContactHistory, MeshContactKey,
    MeshManifoldLimits, MeshTopology, MeshUpdate, MeshValidationIssue, MeshValidationReport,
    MovingTriangle, PairKey, TriangleFeature, TriangleFeatureOwner, TriangleFeatureOwners,
    capsule_analytic_contact, capsule_capsule_toi, capsule_manifold, capsule_triangle_contact,
    cluster_mesh_contacts,
};
pub use constraint::{
    AngularLimit, AnimationTarget, ConstraintKind, ContinuousAngles, EulerOrder,
    PersistentConstraint, RowResult, SwingTwistLimit, XpbdGradient, XpbdRow, XyzLimit,
    project_angular_limit, solve_scalar_row,
};
pub use error::SolverError;
pub use graph::{ConstraintClass, ConstraintColor, ConstraintSchedule};
pub use hash::WorldFingerprint;
pub use id::{BodyId, ColliderId, ConstraintId, MeshId, RegionId, TriangleId};
pub use math::{quat_exp, quat_log, world_inverse_inertia};
pub use parameter::{CurveInterpolation, CurvePoint, ParameterCurve};
pub use pose::{Pose, QUATERNION_NORMALIZATION_TOLERANCE, Velocity};
pub use time::{FixedClock, QualityPolicy, StepBatch};
pub use world::{
    BodyInit, CorrectionKind, MAX_MULTIPLIER_MAGNITUDE, SolverConfig, SolverWorld, StepDiagnostics,
    StepFailure, StepFailureKind, StepOptions, StepPhaseTimings,
};
