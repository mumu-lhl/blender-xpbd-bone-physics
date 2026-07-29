use glam::{Mat3, Quat, Vec3};
use std::time::Instant;

use crate::error::{validate_finite, validate_nonnegative};
use crate::math::world_inverse_inertia;
use crate::pose::validate_vec3;
use crate::{
    AnalyticProxy, AnimationTarget, CachedContact, CapsuleCollider, CapsuleColliderDesc,
    CollisionPipeline, ConstraintId, ConstraintSchedule, ContactKey, ContactLambdas, DeformingMesh,
    MeshContactHistory, MeshContactKey, MeshId, MeshManifoldLimits, PairKey, PersistentConstraint,
    QualityPolicy, capsule_triangle_contact, cluster_mesh_contacts,
};
use crate::{BodyId, BoneBodyDesc, BoneBodyState, Pose, SolverError, Velocity};

const MIN_CONTACT_DENOMINATOR: f32 = 1.0e-12;
const CONTACT_SEPARATION_TOLERANCE: f32 = 1.0e-6;
pub const MAX_MULTIPLIER_MAGNITUDE: f32 = 1.0e20;
// Warm starts may move by at most one effective pair radius, and normals must
// stay within about 18 degrees. Dropping a marginal match is safer than
// applying a cached impulse at an unrelated feature.
const CONTACT_MATCH_RADIUS_SCALE: f64 = 1.0;
const CONTACT_MATCH_WORLD_MIDPOINT_WEIGHT: f64 = 2.0;
const CONTACT_MATCH_NORMAL_MIN_DOT: f32 = 0.95;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SolverConfig {
    pub gravity: Vec3,
    pub fixed_dt: f32,
}

impl SolverConfig {
    fn validate(self) -> Result<(), SolverError> {
        validate_vec3("gravity", self.gravity)?;
        validate_finite("fixed_dt", self.fixed_dt)?;
        if self.fixed_dt <= 0.0 {
            return Err(SolverError::invalid_value("fixed_dt", self.fixed_dt));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodyInit {
    pub desc: BoneBodyDesc,
    pub pose: Pose,
    pub velocity: Velocity,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CorrectionKind {
    Constraint,
    NonVelocityContact,
}

#[derive(Clone, Debug)]
struct BodyEntry {
    desc: BoneBodyDesc,
    state: BoneBodyState,
}

#[derive(Clone, Debug)]
struct DynamicSnapshot {
    bodies: Vec<BodyEntry>,
    constraints: Vec<PersistentConstraint>,
    cached_contacts: Vec<(ContactKey, CachedContact)>,
    cached_analytic_contacts: Vec<AnalyticCachedContact>,
    cached_mesh_contacts: Vec<MeshCachedContact>,
    target_alpha: f32,
}

#[derive(Clone, Copy, Debug)]
struct AnalyticCachedContact {
    key: ContactKey,
    body_collider: crate::ColliderId,
    proxy: crate::ColliderId,
    contact: CachedContact,
    was_separated: bool,
}

#[derive(Clone, Copy, Debug)]
struct AnalyticContactGeometry {
    body: BodyId,
    body_to_proxy: Vec3,
    surface_offset: Vec3,
    separation: f32,
    proxy_velocity: Vec3,
}

#[derive(Clone, Copy, Debug)]
struct MeshCachedContact {
    key: MeshContactKey,
    body_collider: crate::ColliderId,
    contact: CachedContact,
    was_separated: bool,
}

#[derive(Clone, Copy, Debug)]
struct MeshContactSolveGeometry {
    body: BodyId,
    body_to_mesh: Vec3,
    surface_offset: Vec3,
    separation: f32,
    surface_velocity: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct SolveProperties {
    pub inverse_mass: f32,
    pub world_inverse_inertia: Mat3,
}

#[derive(Clone, Debug)]
pub struct SolverWorld {
    config: SolverConfig,
    bodies: Vec<BodyEntry>,
    constraints: Vec<PersistentConstraint>,
    constraint_schedule: ConstraintSchedule,
    colliders: Vec<CapsuleColliderDesc>,
    analytic_proxies: Vec<AnalyticProxy>,
    meshes: Vec<DeformingMesh>,
    cached_contacts: Vec<(ContactKey, CachedContact)>,
    cached_analytic_contacts: Vec<AnalyticCachedContact>,
    cached_mesh_contacts: Vec<MeshCachedContact>,
    animation_targets: Vec<(BodyId, AnimationTarget)>,
    target_alpha: f32,
    paused: bool,
    last_failure: Option<StepFailure>,
    failure_hint: Option<StepFailure>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StepOptions {
    pub position_iterations: u32,
    pub contact_iterations: u32,
    pub quality: QualityPolicy,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepDiagnostics {
    pub generated_contacts: u32,
    pub active_contacts: u32,
    pub deepest_penetration: f32,
    pub max_constraint_error: f32,
    pub position_iterations: u32,
    pub rolled_back: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepPhaseTimings {
    pub prediction_seconds: f64,
    pub animation_inertia_seconds: f64,
    pub contact_generation_seconds: f64,
    pub position_solve_seconds: f64,
    pub diagnostics_seconds: f64,
    pub velocity_seconds: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StepFailureKind {
    NonFiniteBodyState,
    UnnormalizedQuaternion,
    NonFiniteConstraintResidual,
    ExcessiveConstraintMultiplier,
    ExcessiveContactMultiplier,
    InvalidSolverState,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StepFailure {
    pub kind: StepFailureKind,
    pub body: Option<BodyId>,
    pub constraint: Option<ConstraintId>,
    pub contact: Option<ContactKey>,
    pub mesh_contact: Option<MeshContactKey>,
    pub source: Option<SolverError>,
}

impl StepFailure {
    const fn new(kind: StepFailureKind) -> Self {
        Self {
            kind,
            body: None,
            constraint: None,
            contact: None,
            mesh_contact: None,
            source: None,
        }
    }

    const fn for_body(kind: StepFailureKind, body: BodyId) -> Self {
        Self {
            kind,
            body: Some(body),
            constraint: None,
            contact: None,
            mesh_contact: None,
            source: None,
        }
    }

    const fn for_constraint(kind: StepFailureKind, constraint: ConstraintId) -> Self {
        Self {
            kind,
            body: None,
            constraint: Some(constraint),
            contact: None,
            mesh_contact: None,
            source: None,
        }
    }

    const fn for_contact(kind: StepFailureKind, contact: ContactKey) -> Self {
        Self {
            kind,
            body: None,
            constraint: None,
            contact: Some(contact),
            mesh_contact: None,
            source: None,
        }
    }

    const fn for_mesh_contact(kind: StepFailureKind, mesh_contact: MeshContactKey) -> Self {
        Self {
            kind,
            body: None,
            constraint: None,
            contact: None,
            mesh_contact: Some(mesh_contact),
            source: None,
        }
    }

    fn with_source(mut self, source: SolverError) -> Self {
        self.source = Some(source);
        self
    }
}

impl SolverWorld {
    pub fn new(config: SolverConfig, mut bodies: Vec<BodyInit>) -> Result<Self, SolverError> {
        config.validate()?;
        bodies.sort_by_key(|init| init.desc.id);

        for pair in bodies.windows(2) {
            if pair[0].desc.id == pair[1].desc.id {
                return Err(SolverError::DuplicateId {
                    kind: "body",
                    id: pair[0].desc.id.get(),
                });
            }
        }

        let mut entries = Vec::with_capacity(bodies.len());
        for init in bodies {
            validate_vec3("linear_velocity", init.velocity.linear)?;
            validate_vec3("angular_velocity", init.velocity.angular)?;
            let mut state = BoneBodyState::from_desc(&init.desc, init.pose)?;
            state.velocity = init.velocity;
            entries.push(BodyEntry {
                desc: init.desc,
                state,
            });
        }

        Ok(Self {
            config,
            bodies: entries,
            constraints: Vec::new(),
            constraint_schedule: ConstraintSchedule::default(),
            colliders: Vec::new(),
            analytic_proxies: Vec::new(),
            meshes: Vec::new(),
            cached_contacts: Vec::new(),
            cached_analytic_contacts: Vec::new(),
            cached_mesh_contacts: Vec::new(),
            animation_targets: Vec::new(),
            target_alpha: 0.0,
            paused: false,
            last_failure: None,
            failure_hint: None,
        })
    }

    pub fn set_constraints(
        &mut self,
        mut constraints: Vec<PersistentConstraint>,
    ) -> Result<(), SolverError> {
        constraints.sort_by_key(|constraint| constraint.id);
        for pair in constraints.windows(2) {
            if pair[0].id == pair[1].id {
                return Err(SolverError::DuplicateId {
                    kind: "constraint",
                    id: pair[0].id.get(),
                });
            }
        }
        for constraint in &constraints {
            constraint.validate()?;
            self.solve_properties(constraint.body_a)?;
            if let Some(body_b) = constraint.body_b {
                self.solve_properties(body_b)?;
            }
        }
        let schedule = ConstraintSchedule::build(&constraints)?;
        self.constraints = constraints;
        self.constraint_schedule = schedule;
        Ok(())
    }

    pub fn set_colliders(
        &mut self,
        mut colliders: Vec<CapsuleColliderDesc>,
    ) -> Result<(), SolverError> {
        colliders.sort_by_key(|collider| collider.id);
        for pair in colliders.windows(2) {
            if pair[0].id == pair[1].id {
                return Err(SolverError::DuplicateId {
                    kind: "collider",
                    id: pair[0].id.get(),
                });
            }
        }
        if let Some(collider) = colliders.iter().find(|collider| {
            self.analytic_proxies
                .binary_search_by_key(&collider.id, |proxy| proxy.id)
                .is_ok()
        }) {
            return Err(SolverError::DuplicateId {
                kind: "collider",
                id: collider.id.get(),
            });
        }
        for collider in &colliders {
            collider.validate()?;
            self.solve_properties(collider.body)?;
            for body in &collider.filter.excluded_bodies {
                self.solve_properties(*body)?;
            }
            for constraint in &collider.filter.excluded_constraints {
                if self
                    .constraints
                    .binary_search_by_key(constraint, |entry| entry.id)
                    .is_err()
                {
                    return Err(SolverError::invalid_value(
                        "collision_filter_constraint",
                        constraint.get(),
                    ));
                }
            }
        }

        self.cached_contacts.retain(|(key, _)| {
            collider_metadata_unchanged(&self.colliders, &colliders, key.pair.first())
                && collider_metadata_unchanged(&self.colliders, &colliders, key.pair.second())
        });
        self.cached_analytic_contacts.retain(|cached| {
            collider_metadata_unchanged(&self.colliders, &colliders, cached.body_collider)
                && self
                    .analytic_proxies
                    .binary_search_by_key(&cached.proxy, |proxy| proxy.id)
                    .is_ok()
        });
        self.cached_mesh_contacts.retain(|cached| {
            collider_metadata_unchanged(&self.colliders, &colliders, cached.body_collider)
                && self
                    .meshes
                    .binary_search_by_key(&cached.key.mesh, DeformingMesh::id)
                    .is_ok()
        });
        self.colliders = colliders;
        Ok(())
    }

    pub fn set_analytic_proxies(
        &mut self,
        mut proxies: Vec<AnalyticProxy>,
    ) -> Result<(), SolverError> {
        proxies.sort_by_key(|proxy| proxy.id);
        for pair in proxies.windows(2) {
            if pair[0].id == pair[1].id {
                return Err(SolverError::DuplicateId {
                    kind: "collider",
                    id: pair[0].id.get(),
                });
            }
        }
        for proxy in &proxies {
            proxy.validate()?;
            if self
                .colliders
                .binary_search_by_key(&proxy.id, |collider| collider.id)
                .is_ok()
            {
                return Err(SolverError::DuplicateId {
                    kind: "collider",
                    id: proxy.id.get(),
                });
            }
        }
        self.cached_analytic_contacts.retain(|cached| {
            analytic_proxy_metadata_unchanged(&self.analytic_proxies, &proxies, cached.proxy)
                && self
                    .colliders
                    .binary_search_by_key(&cached.body_collider, |collider| collider.id)
                    .is_ok()
        });
        self.analytic_proxies = proxies;
        Ok(())
    }

    #[must_use]
    pub fn analytic_proxy_count(&self) -> usize {
        self.analytic_proxies.len()
    }

    pub fn set_meshes(&mut self, mut meshes: Vec<DeformingMesh>) -> Result<(), SolverError> {
        meshes.sort_by_key(DeformingMesh::id);
        for pair in meshes.windows(2) {
            if pair[0].id() == pair[1].id() {
                return Err(SolverError::DuplicateId {
                    kind: "mesh",
                    id: pair[0].id().get(),
                });
            }
        }
        self.cached_mesh_contacts.retain(|cached| {
            mesh_metadata_unchanged(&self.meshes, &meshes, cached.key.mesh)
                && self
                    .colliders
                    .binary_search_by_key(&cached.body_collider, |collider| collider.id)
                    .is_ok()
        });
        self.meshes = meshes;
        Ok(())
    }

    #[must_use]
    pub fn mesh_count(&self) -> usize {
        self.meshes.len()
    }

    #[must_use]
    pub fn cached_contacts(&self) -> Vec<(ContactKey, CachedContact)> {
        self.cached_contacts.clone()
    }

    pub fn set_animation_targets(
        &mut self,
        targets: &[(BodyId, AnimationTarget)],
    ) -> Result<(), SolverError> {
        let mut sorted = targets.to_vec();
        sorted.sort_by_key(|(id, _)| *id);
        for pair in sorted.windows(2) {
            if pair[0].0 == pair[1].0 {
                return Err(SolverError::DuplicateId {
                    kind: "animation_target",
                    id: pair[0].0.get(),
                });
            }
        }
        for (id, target) in &sorted {
            self.solve_properties(*id)?;
            target.validate()?;
        }
        self.animation_targets = sorted;
        self.target_alpha = 0.0;
        Ok(())
    }

    pub fn substep(
        &mut self,
        dt: f32,
        target_alpha: f32,
        options: StepOptions,
    ) -> Result<StepDiagnostics, SolverError> {
        self.substep_internal(dt, target_alpha, options, None)
    }

    pub fn substep_profiled(
        &mut self,
        dt: f32,
        target_alpha: f32,
        options: StepOptions,
    ) -> Result<(StepDiagnostics, StepPhaseTimings), SolverError> {
        let mut timings = StepPhaseTimings::default();
        let diagnostics = self.substep_internal(dt, target_alpha, options, Some(&mut timings))?;
        Ok((diagnostics, timings))
    }

    fn substep_internal(
        &mut self,
        dt: f32,
        target_alpha: f32,
        options: StepOptions,
        mut timings: Option<&mut StepPhaseTimings>,
    ) -> Result<StepDiagnostics, SolverError> {
        if self.paused {
            return Err(SolverError::WorldPaused);
        }
        validate_dt(dt)?;
        validate_finite("target_alpha", target_alpha)?;
        if !(0.0..=1.0).contains(&target_alpha) {
            return Err(SolverError::invalid_value("target_alpha", target_alpha));
        }

        let snapshot = self.dynamic_snapshot();
        self.failure_hint = None;
        let result = self.run_substep(dt, target_alpha, options, &mut timings);
        let source = result.as_ref().err().cloned();
        let failure = self
            .detect_step_failure()
            .or_else(|| self.failure_hint.clone())
            .or_else(|| {
                source
                    .as_ref()
                    .map(|_| StepFailure::new(StepFailureKind::InvalidSolverState))
            })
            .map(|failure| {
                if failure.source.is_none() {
                    source
                        .clone()
                        .map_or(failure.clone(), |source| failure.with_source(source))
                } else {
                    failure
                }
            });
        self.failure_hint = None;
        if let Some(failure) = failure {
            self.restore_dynamic_snapshot(snapshot);
            self.paused = true;
            self.last_failure = Some(failure);
            return Ok(self.rollback_diagnostics(options.position_iterations));
        }

        let diagnostics = result?;
        self.last_failure = None;
        Ok(diagnostics)
    }

    fn run_substep(
        &mut self,
        dt: f32,
        target_alpha: f32,
        options: StepOptions,
        timings: &mut Option<&mut StepPhaseTimings>,
    ) -> Result<StepDiagnostics, SolverError> {
        let started = timings.is_some().then(Instant::now);
        let prediction_result = self.predict(dt);
        record_phase(timings, started, |value, elapsed| {
            value.prediction_seconds += elapsed;
        });
        prediction_result?;

        let started = timings.is_some().then(Instant::now);
        let animation_result = self.apply_animation_inertia(target_alpha);
        record_phase(timings, started, |value, elapsed| {
            value.animation_inertia_seconds += elapsed;
        });
        animation_result?;

        let started = timings.is_some().then(Instant::now);
        let contact_result = self.generate_contacts(dt);
        record_phase(timings, started, |value, elapsed| {
            value.contact_generation_seconds += elapsed;
        });
        let generated_contacts = contact_result?;

        let started = timings.is_some().then(Instant::now);
        let schedule = self.constraint_schedule.clone();
        let colliders = self.colliders.clone();
        let analytic_proxies = self.analytic_proxies.clone();
        let meshes = core::mem::take(&mut self.meshes);
        let mut constraints = core::mem::take(&mut self.constraints);
        let mut contacts = core::mem::take(&mut self.cached_contacts);
        let mut analytic_contacts = core::mem::take(&mut self.cached_analytic_contacts);
        let mut mesh_contacts = core::mem::take(&mut self.cached_mesh_contacts);
        let solve_result = (|| {
            self.warm_start_contacts(&colliders, &contacts)?;
            self.warm_start_analytic_contacts(&colliders, &analytic_proxies, &analytic_contacts)?;
            self.warm_start_mesh_contacts(&colliders, &meshes, &mesh_contacts, dt)?;
            for iteration in 0..options.position_iterations {
                for (_, id) in schedule.iteration_order(iteration) {
                    self.failure_hint = Some(StepFailure::for_constraint(
                        StepFailureKind::InvalidSolverState,
                        id,
                    ));
                    let index = constraints
                        .binary_search_by_key(&id, |constraint| constraint.id)
                        .map_err(|_| {
                            SolverError::invalid_value("constraint_schedule_id", id.get())
                        })?;
                    self.solve_constraint(&mut constraints[index], target_alpha, dt)?;
                    self.failure_hint = None;
                }
                self.solve_contact_iteration(&colliders, &mut contacts, iteration)?;
                self.solve_analytic_contact_iteration(
                    &colliders,
                    &analytic_proxies,
                    &mut analytic_contacts,
                    iteration,
                )?;
                self.solve_mesh_contact_iteration(
                    &colliders,
                    &meshes,
                    &mut mesh_contacts,
                    iteration,
                    dt,
                )?;
            }
            Ok::<(), SolverError>(())
        })();
        self.constraints = constraints;
        self.cached_contacts = contacts;
        self.cached_analytic_contacts = analytic_contacts;
        self.cached_mesh_contacts = mesh_contacts;
        self.meshes = meshes;
        record_phase(timings, started, |value, elapsed| {
            value.position_solve_seconds += elapsed;
        });
        solve_result?;

        let started = timings.is_some().then(Instant::now);
        let diagnostics_result = (|| {
            let max_constraint_error = self.max_constraint_error()?;
            let (active_contacts, deepest_penetration) = self.contact_residuals()?;
            Ok::<_, SolverError>((max_constraint_error, active_contacts, deepest_penetration))
        })();
        record_phase(timings, started, |value, elapsed| {
            value.diagnostics_seconds += elapsed;
        });
        let (max_constraint_error, active_contacts, deepest_penetration) = diagnostics_result?;

        let started = timings.is_some().then(Instant::now);
        let rebuild_result = self.rebuild_velocities(dt);
        if let Err(error) = rebuild_result {
            record_phase(timings, started, |value, elapsed| {
                value.velocity_seconds += elapsed;
            });
            return Err(error);
        }
        let previous_target_alpha = self.target_alpha;
        let constraints = core::mem::take(&mut self.constraints);
        let damping_result = (|| {
            for constraint in &constraints {
                self.failure_hint = Some(StepFailure::for_constraint(
                    StepFailureKind::InvalidSolverState,
                    constraint.id,
                ));
                constraint.apply_damping(self, dt)?;
                let target = self.animation_target(constraint.body_a);
                constraint.apply_recovery_damping(
                    self,
                    target,
                    previous_target_alpha,
                    target_alpha,
                    dt,
                )?;
                self.failure_hint = None;
            }
            Ok::<(), SolverError>(())
        })();
        self.constraints = constraints;
        damping_result?;

        let colliders = self.colliders.clone();
        let analytic_proxies = self.analytic_proxies.clone();
        let meshes = core::mem::take(&mut self.meshes);
        let mut contacts = core::mem::take(&mut self.cached_contacts);
        let mut analytic_contacts = core::mem::take(&mut self.cached_analytic_contacts);
        let mut mesh_contacts = core::mem::take(&mut self.cached_mesh_contacts);
        let velocity_result = self.apply_contact_velocity_response(&colliders, &mut contacts);
        let analytic_velocity_result = self.apply_analytic_contact_velocity_response(
            &colliders,
            &analytic_proxies,
            &mut analytic_contacts,
        );
        let mesh_velocity_result =
            self.apply_mesh_contact_velocity_response(&colliders, &meshes, &mut mesh_contacts, dt);
        self.cached_contacts = contacts;
        self.cached_analytic_contacts = analytic_contacts;
        self.cached_mesh_contacts = mesh_contacts;
        self.meshes = meshes;
        record_phase(timings, started, |value, elapsed| {
            value.velocity_seconds += elapsed;
        });
        velocity_result?;
        analytic_velocity_result?;
        mesh_velocity_result?;

        self.target_alpha = target_alpha;
        let _ = options.contact_iterations;
        let _ = options.quality;
        Ok(StepDiagnostics {
            generated_contacts,
            active_contacts,
            deepest_penetration,
            max_constraint_error,
            position_iterations: options.position_iterations,
            rolled_back: false,
        })
    }

    #[must_use]
    pub const fn is_paused(&self) -> bool {
        self.paused
    }

    #[must_use]
    pub fn last_failure(&self) -> Option<&StepFailure> {
        self.last_failure.as_ref()
    }

    pub fn resume(&mut self) -> Result<(), SolverError> {
        if let Some(failure) = self.detect_step_failure() {
            return Err(step_failure_error(&failure));
        }
        self.paused = false;
        self.last_failure = None;
        Ok(())
    }

    fn dynamic_snapshot(&self) -> DynamicSnapshot {
        DynamicSnapshot {
            bodies: self.bodies.clone(),
            constraints: self.constraints.clone(),
            cached_contacts: self.cached_contacts.clone(),
            cached_analytic_contacts: self.cached_analytic_contacts.clone(),
            cached_mesh_contacts: self.cached_mesh_contacts.clone(),
            target_alpha: self.target_alpha,
        }
    }

    fn restore_dynamic_snapshot(&mut self, snapshot: DynamicSnapshot) {
        self.bodies = snapshot.bodies;
        self.constraints = snapshot.constraints;
        self.cached_contacts = snapshot.cached_contacts;
        self.cached_analytic_contacts = snapshot.cached_analytic_contacts;
        self.cached_mesh_contacts = snapshot.cached_mesh_contacts;
        self.target_alpha = snapshot.target_alpha;
        self.failure_hint = None;
    }

    fn rollback_diagnostics(&self, position_iterations: u32) -> StepDiagnostics {
        StepDiagnostics {
            generated_contacts: 0,
            active_contacts: u32::try_from(
                self.cached_contacts
                    .len()
                    .saturating_add(self.cached_analytic_contacts.len())
                    .saturating_add(self.cached_mesh_contacts.len()),
            )
            .unwrap_or(u32::MAX),
            deepest_penetration: 0.0,
            max_constraint_error: 0.0,
            position_iterations,
            rolled_back: true,
        }
    }

    fn detect_step_failure(&self) -> Option<StepFailure> {
        for entry in &self.bodies {
            let id = entry.desc.id;
            let state = &entry.state;
            let poses = [
                state.pose,
                state.previous_pose,
                state.predicted_pose,
                state.integrated_pose,
            ];
            if poses.iter().any(|pose| {
                !pose.translation.is_finite()
                    || !pose.rotation.is_finite()
                    || !state.velocity.linear.is_finite()
                    || !state.velocity.angular.is_finite()
                    || !state.contact_translation_correction.is_finite()
                    || !state.contact_rotation_correction.is_finite()
            }) {
                return Some(StepFailure::for_body(
                    StepFailureKind::NonFiniteBodyState,
                    id,
                ));
            }
            if poses.iter().any(|pose| {
                (pose.rotation.length_squared() - 1.0).abs()
                    > crate::QUATERNION_NORMALIZATION_TOLERANCE
            }) {
                return Some(StepFailure::for_body(
                    StepFailureKind::UnnormalizedQuaternion,
                    id,
                ));
            }
        }

        for constraint in &self.constraints {
            if !constraint.lambda.is_finite() {
                return Some(StepFailure::for_constraint(
                    StepFailureKind::InvalidSolverState,
                    constraint.id,
                ));
            }
            if max_abs_vec3(constraint.lambda) > MAX_MULTIPLIER_MAGNITUDE {
                return Some(StepFailure::for_constraint(
                    StepFailureKind::ExcessiveConstraintMultiplier,
                    constraint.id,
                ));
            }
        }

        for (key, contact) in &self.cached_contacts {
            if !contact.local_a.is_finite()
                || !contact.local_b.is_finite()
                || !contact.normal.is_finite()
                || contact.tangents.iter().any(|tangent| !tangent.is_finite())
                || !contact.lambdas.normal.is_finite()
                || contact
                    .lambdas
                    .tangents
                    .iter()
                    .any(|lambda| !lambda.is_finite())
            {
                return Some(StepFailure::for_contact(
                    StepFailureKind::InvalidSolverState,
                    *key,
                ));
            }
            let maximum_lambda = contact
                .lambdas
                .tangents
                .iter()
                .fold(contact.lambdas.normal.abs(), |maximum, lambda| {
                    maximum.max(lambda.abs())
                });
            if maximum_lambda > MAX_MULTIPLIER_MAGNITUDE {
                return Some(StepFailure::for_contact(
                    StepFailureKind::ExcessiveContactMultiplier,
                    *key,
                ));
            }
        }
        for cached in &self.cached_analytic_contacts {
            let key = cached.key;
            let contact = cached.contact;
            if !contact.local_a.is_finite()
                || !contact.local_b.is_finite()
                || !contact.normal.is_finite()
                || contact.tangents.iter().any(|tangent| !tangent.is_finite())
                || !contact.lambdas.normal.is_finite()
                || contact
                    .lambdas
                    .tangents
                    .iter()
                    .any(|lambda| !lambda.is_finite())
            {
                return Some(StepFailure::for_contact(
                    StepFailureKind::InvalidSolverState,
                    key,
                ));
            }
            let maximum_lambda = contact
                .lambdas
                .tangents
                .iter()
                .fold(contact.lambdas.normal.abs(), |maximum, lambda| {
                    maximum.max(lambda.abs())
                });
            if maximum_lambda > MAX_MULTIPLIER_MAGNITUDE {
                return Some(StepFailure::for_contact(
                    StepFailureKind::ExcessiveContactMultiplier,
                    key,
                ));
            }
        }
        for cached in &self.cached_mesh_contacts {
            let key = cached.key;
            let contact = cached.contact;
            if !contact.local_a.is_finite()
                || !contact.local_b.is_finite()
                || !contact.normal.is_finite()
                || contact.tangents.iter().any(|tangent| !tangent.is_finite())
                || !contact.lambdas.normal.is_finite()
                || contact
                    .lambdas
                    .tangents
                    .iter()
                    .any(|lambda| !lambda.is_finite())
            {
                return Some(StepFailure::for_mesh_contact(
                    StepFailureKind::InvalidSolverState,
                    key,
                ));
            }
            let maximum_lambda = contact
                .lambdas
                .tangents
                .iter()
                .fold(contact.lambdas.normal.abs(), |maximum, lambda| {
                    maximum.max(lambda.abs())
                });
            if maximum_lambda > MAX_MULTIPLIER_MAGNITUDE {
                return Some(StepFailure::for_mesh_contact(
                    StepFailureKind::ExcessiveContactMultiplier,
                    key,
                ));
            }
        }
        None
    }

    pub fn predict(&mut self, dt: f32) -> Result<(), SolverError> {
        validate_dt(dt)?;

        for entry in &mut self.bodies {
            let state = &mut entry.state;
            state.previous_pose = state.pose;
            state.contact_translation_correction = Vec3::ZERO;
            state.contact_rotation_correction = Vec3::ZERO;

            state.velocity.linear += self.config.gravity * entry.desc.gravity_scale * dt;
            state.velocity.linear *= (-entry.desc.linear_damping * dt).exp();
            state.velocity.angular *= (-entry.desc.angular_damping * dt).exp();

            let translation = state.pose.translation + state.velocity.linear * dt;
            let delta_rotation = Quat::from_scaled_axis(state.velocity.angular * dt);
            let rotation = (delta_rotation * state.pose.rotation).normalize();
            state.predicted_pose = Pose {
                translation,
                rotation,
            };
            state.integrated_pose = state.predicted_pose;
        }

        Ok(())
    }

    pub fn apply_pose_correction(
        &mut self,
        id: BodyId,
        translation: Vec3,
        rotation_scaled_axis: Vec3,
        kind: CorrectionKind,
    ) -> Result<(), SolverError> {
        validate_vec3("translation_correction", translation)?;
        validate_vec3("rotation_correction", rotation_scaled_axis)?;
        let state = self.body_mut(id)?;
        let previous_translation = state.predicted_pose.translation;
        let previous_rotation = state.predicted_pose.rotation;

        state.predicted_pose.translation += translation;
        let delta_rotation = Quat::from_scaled_axis(rotation_scaled_axis);
        state.predicted_pose.rotation =
            (delta_rotation * state.predicted_pose.rotation).normalize();

        if kind == CorrectionKind::NonVelocityContact {
            state.contact_translation_correction +=
                state.predicted_pose.translation - previous_translation;
            let mut applied_rotation =
                state.predicted_pose.rotation * previous_rotation.conjugate();
            if applied_rotation.w < 0.0 {
                applied_rotation = -applied_rotation;
            }
            state.contact_rotation_correction += applied_rotation.to_scaled_axis();
        }

        Ok(())
    }

    pub fn rebuild_velocities(&mut self, dt: f32) -> Result<(), SolverError> {
        validate_dt(dt)?;

        for entry in &mut self.bodies {
            let state = &mut entry.state;
            let translation_correction = state.predicted_pose.translation
                - state.integrated_pose.translation
                - state.contact_translation_correction;
            state.velocity.linear += translation_correction / dt;

            let mut rotation_delta =
                state.predicted_pose.rotation * state.integrated_pose.rotation.conjugate();
            if rotation_delta.w < 0.0 {
                rotation_delta = -rotation_delta;
            }
            state.velocity.angular +=
                (rotation_delta.to_scaled_axis() - state.contact_rotation_correction) / dt;

            state.previous_pose = state.pose;
            state.pose = state.predicted_pose;
            state.contact_translation_correction = Vec3::ZERO;
            state.contact_rotation_correction = Vec3::ZERO;
        }

        Ok(())
    }

    #[must_use]
    pub fn body(&self, id: BodyId) -> Option<&BoneBodyState> {
        self.bodies
            .binary_search_by_key(&id, |entry| entry.desc.id)
            .ok()
            .map(|index| &self.bodies[index].state)
    }

    pub fn anchor_error(&self, id: ConstraintId) -> Result<f32, SolverError> {
        let constraint = self.constraint(id)?;
        constraint.error(self)
    }

    pub fn constraint_lambda(&self, id: ConstraintId) -> Option<f32> {
        let constraint = self
            .constraints
            .binary_search_by_key(&id, |constraint| constraint.id)
            .ok()
            .map(|index| &self.constraints[index])?;
        Some(match &constraint.kind {
            crate::ConstraintKind::DistanceRange { .. } => {
                constraint.lambda.x.max(constraint.lambda.y)
            }
            _ => constraint.lambda.length(),
        })
    }

    fn body_mut(&mut self, id: BodyId) -> Result<&mut BoneBodyState, SolverError> {
        let index = self
            .bodies
            .binary_search_by_key(&id, |entry| entry.desc.id)
            .map_err(|_| SolverError::MissingBody(id.get()))?;
        Ok(&mut self.bodies[index].state)
    }

    pub(crate) fn solve_properties(&self, id: BodyId) -> Result<SolveProperties, SolverError> {
        let index = self
            .bodies
            .binary_search_by_key(&id, |entry| entry.desc.id)
            .map_err(|_| SolverError::MissingBody(id.get()))?;
        let entry = &self.bodies[index];
        Ok(SolveProperties {
            inverse_mass: entry.desc.inverse_mass,
            world_inverse_inertia: world_inverse_inertia(
                entry.state.predicted_pose.rotation,
                entry.desc.inverse_inertia,
            ),
        })
    }

    pub(crate) fn anchor_geometry(
        &self,
        id: BodyId,
        local_anchor: Vec3,
    ) -> Result<(Vec3, Vec3), SolverError> {
        let state = self.body(id).ok_or(SolverError::MissingBody(id.get()))?;
        let offset = state.predicted_pose.rotation * local_anchor;
        Ok((state.predicted_pose.translation + offset, offset))
    }

    pub(crate) fn apply_velocity_impulse(
        &mut self,
        id: BodyId,
        linear_gradient: Vec3,
        angular_gradient: Vec3,
        impulse: f32,
    ) -> Result<(), SolverError> {
        let properties = self.solve_properties(id)?;
        let state = self.body_mut(id)?;
        state.velocity.linear += linear_gradient * properties.inverse_mass * impulse;
        state.velocity.angular += properties.world_inverse_inertia * angular_gradient * impulse;
        Ok(())
    }

    fn solve_constraint(
        &mut self,
        constraint: &mut PersistentConstraint,
        target_alpha: f32,
        dt: f32,
    ) -> Result<(), SolverError> {
        constraint.solve_joint(self, dt)?;
        let target = self.animation_target(constraint.body_a);
        constraint.solve_recovery(self, target, target_alpha, dt)
    }

    fn constraint(&self, id: ConstraintId) -> Result<&PersistentConstraint, SolverError> {
        let index = self
            .constraints
            .binary_search_by_key(&id, |constraint| constraint.id)
            .map_err(|_| SolverError::InvalidValue {
                field: "constraint_id",
                value: f64::from(id.get()),
            })?;
        Ok(&self.constraints[index])
    }

    fn max_constraint_error(&mut self) -> Result<f32, SolverError> {
        let mut maximum: f32 = 0.0;
        for index in 0..self.constraints.len() {
            let constraint = self.constraints[index].clone();
            self.failure_hint = Some(StepFailure::for_constraint(
                StepFailureKind::NonFiniteConstraintResidual,
                constraint.id,
            ));
            let error = constraint.error(self)?;
            validate_finite("constraint_residual", error)?;
            maximum = maximum.max(error);
            self.failure_hint = None;
        }
        Ok(maximum)
    }

    fn generate_contacts(&mut self, dt: f32) -> Result<u32, SolverError> {
        let (previous, predicted) = self.collider_snapshots()?;
        let pipeline = CollisionPipeline::try_new(&self.constraints)?;
        let candidates = pipeline.candidate_pairs(&previous, &predicted)?;
        let mut generated = Vec::new();

        for pair in candidates {
            self.failure_hint = Some(StepFailure::for_contact(
                StepFailureKind::InvalidSolverState,
                ContactKey::new(pair, 0, 0),
            ));
            let previous_a = collider_by_id(&previous, pair.first())?;
            let previous_b = collider_by_id(&previous, pair.second())?;
            let collider_a = collider_by_id(&predicted, pair.first())?;
            let collider_b = collider_by_id(&predicted, pair.second())?;
            let cached_normal = cached_normal_for_pair(&self.cached_contacts, pair);
            let Some(manifold) = crate::capsule_manifold(collider_a, collider_b, cached_normal)?
            else {
                self.failure_hint = None;
                continue;
            };
            let radius_a = collider_a.shape.effective_radius()?;
            let radius_b = collider_b.shape.effective_radius()?;
            let match_context = ContactMatchContext {
                previous_a: previous_a.pose,
                previous_b: previous_b.pose,
                current_a: collider_a.pose,
                current_b: collider_b.pose,
                radius_sum: radius_a + radius_b,
            };
            let mut current_contacts = Vec::with_capacity(manifold.points.len());

            for geometry in manifold.points {
                let center_a = geometry.point_a - geometry.normal * radius_a;
                let center_b = geometry.point_b + geometry.normal * radius_b;
                let contact = CachedContact {
                    local_a: local_point(collider_a.pose, center_a),
                    local_b: local_point(collider_b.pose, center_b),
                    normal: geometry.normal,
                    tangents: tangent_basis(geometry.normal),
                    lambdas: ContactLambdas::default(),
                    age: 1,
                };
                contact.validate()?;
                current_contacts.push(contact);
            }
            let previous_contacts: Vec<_> = self
                .cached_contacts
                .iter()
                .filter(|(key, _)| key.pair == pair)
                .copied()
                .collect();
            generated.extend(assign_contact_slots(
                pair,
                current_contacts,
                &previous_contacts,
                match_context,
            )?);
            self.failure_hint = None;
        }

        generated.sort_by_key(|(key, _)| *key);
        self.cached_contacts = generated;
        let analytic_count = self.generate_analytic_contacts(&previous, &predicted, dt)?;
        let meshes = core::mem::take(&mut self.meshes);
        let mesh_result = self.generate_mesh_contacts(&meshes, &previous, &predicted, dt);
        self.meshes = meshes;
        let mesh_count = mesh_result?;
        let total = self
            .cached_contacts
            .len()
            .checked_add(analytic_count)
            .and_then(|count| count.checked_add(mesh_count))
            .ok_or_else(|| SolverError::invalid_value("contact_count", usize::MAX as f64))?;
        contact_count(total)
    }

    fn generate_analytic_contacts(
        &mut self,
        previous: &[CapsuleCollider],
        predicted: &[CapsuleCollider],
        dt: f32,
    ) -> Result<usize, SolverError> {
        let proxies = self.analytic_proxies.clone();
        let mut generated = Vec::new();
        for collider in predicted {
            let previous_collider = collider_by_id(previous, collider.id)?;
            let radius = collider.shape.effective_radius()?;
            for proxy in &proxies {
                if !proxy.collision_enabled {
                    continue;
                }
                let pair = PairKey::ordered(collider.id, proxy.id)?;
                self.failure_hint = Some(StepFailure::for_contact(
                    StepFailureKind::InvalidSolverState,
                    ContactKey::new(pair, 0, 0),
                ));
                let Some(geometry) = crate::capsule_analytic_contact(collider, proxy, dt)? else {
                    self.failure_hint = None;
                    continue;
                };
                let body_first = pair.first() == collider.id;
                let body_to_proxy = geometry.normal;
                let pair_normal = if body_first {
                    body_to_proxy
                } else {
                    -body_to_proxy
                };
                let body_center = geometry.point_capsule - body_to_proxy * radius;
                let body_local = local_point(collider.pose, body_center);
                let proxy_local = local_point(proxy.pose, geometry.point_proxy);
                let mut contact = CachedContact {
                    local_a: if body_first { body_local } else { proxy_local },
                    local_b: if body_first { proxy_local } else { body_local },
                    normal: pair_normal,
                    tangents: tangent_basis(pair_normal),
                    lambdas: ContactLambdas::default(),
                    age: 1,
                };
                let key = ContactKey::new(pair, 0, 0);
                if let Some(previous_cached) = self
                    .cached_analytic_contacts
                    .iter()
                    .find(|cached| cached.key == key)
                    && analytic_contact_matches(&contact, &previous_cached.contact, radius)?
                {
                    contact.lambdas = previous_cached.contact.lambdas;
                    contact.age = previous_cached.contact.age.checked_add(1).ok_or_else(|| {
                        SolverError::invalid_value("contact_age", previous_cached.contact.age)
                    })?;
                }
                contact.validate()?;

                let mut previous_proxy = *proxy;
                previous_proxy.pose.translation -= proxy.linear_velocity * dt;
                previous_proxy.linear_velocity = Vec3::ZERO;
                let mut stationary_previous = previous_collider.clone();
                stationary_previous.linear_velocity = Vec3::ZERO;
                let was_separated =
                    crate::capsule_analytic_contact(&stationary_previous, &previous_proxy, dt)?
                        .is_none();
                generated.push(AnalyticCachedContact {
                    key,
                    body_collider: collider.id,
                    proxy: proxy.id,
                    contact,
                    was_separated,
                });
                self.failure_hint = None;
            }
        }
        generated.sort_by_key(|cached| cached.key);
        self.cached_analytic_contacts = generated;
        Ok(self.cached_analytic_contacts.len())
    }

    fn generate_mesh_contacts(
        &mut self,
        meshes: &[DeformingMesh],
        previous: &[CapsuleCollider],
        predicted: &[CapsuleCollider],
        dt: f32,
    ) -> Result<usize, SolverError> {
        let mut generated = Vec::new();
        for collider in predicted {
            let previous_collider = collider_by_id(previous, collider.id)?;
            let radius = collider.shape.effective_radius()?;
            for mesh in meshes {
                let candidates = mesh.query_capsule_sweep(previous_collider, collider)?;
                let mut raw = Vec::new();
                for triangle_id in candidates {
                    self.failure_hint = Some(StepFailure::for_mesh_contact(
                        StepFailureKind::InvalidSolverState,
                        MeshContactKey {
                            collider: collider.id,
                            mesh: mesh.id(),
                            triangle: triangle_id,
                            feature: 0,
                        },
                    ));
                    let history = self
                        .cached_mesh_contacts
                        .iter()
                        .find(|cached| {
                            cached.body_collider == collider.id
                                && cached.key.mesh == mesh.id()
                                && cached.key.triangle == triangle_id
                        })
                        .map(|cached| MeshContactHistory {
                            normal: cached.contact.normal,
                        });
                    let triangle = mesh.moving_triangle(triangle_id, dt)?;
                    if let Some(contact) = capsule_triangle_contact(collider, &triangle, history)? {
                        raw.push(contact);
                    }
                    self.failure_hint = None;
                }
                let limits = MeshManifoldLimits::try_new(4, radius * 0.25, 0.95)?;
                for geometry in cluster_mesh_contacts(raw, limits)? {
                    self.failure_hint = Some(StepFailure::for_mesh_contact(
                        StepFailureKind::InvalidSolverState,
                        geometry.key,
                    ));
                    let body_center = geometry.geometry.point_a - geometry.geometry.normal * radius;
                    let mut contact = CachedContact {
                        local_a: local_point(collider.pose, body_center),
                        local_b: geometry.barycentric,
                        normal: geometry.geometry.normal,
                        tangents: tangent_basis(geometry.geometry.normal),
                        lambdas: ContactLambdas::default(),
                        age: 1,
                    };
                    if let Some(previous_cached) = self
                        .cached_mesh_contacts
                        .iter()
                        .find(|cached| cached.key == geometry.key)
                        && mesh_contact_matches(&contact, &previous_cached.contact, radius)?
                    {
                        contact.lambdas = previous_cached.contact.lambdas;
                        contact.age =
                            previous_cached.contact.age.checked_add(1).ok_or_else(|| {
                                SolverError::invalid_value(
                                    "contact_age",
                                    previous_cached.contact.age,
                                )
                            })?;
                    }
                    contact.validate()?;

                    let mut previous_triangle = mesh.moving_triangle(geometry.key.triangle, dt)?;
                    for index in 0..3 {
                        previous_triangle.vertices[index] -=
                            previous_triangle.velocities[index] * dt;
                        previous_triangle.velocities[index] = Vec3::ZERO;
                    }
                    let mut stationary_previous = previous_collider.clone();
                    stationary_previous.linear_velocity = Vec3::ZERO;
                    let was_separated =
                        capsule_triangle_contact(&stationary_previous, &previous_triangle, None)?
                            .is_none();
                    generated.push(MeshCachedContact {
                        key: geometry.key,
                        body_collider: collider.id,
                        contact,
                        was_separated,
                    });
                    self.failure_hint = None;
                }
            }
        }
        generated.sort_by_key(|cached| cached.key);
        self.cached_mesh_contacts = generated;
        Ok(self.cached_mesh_contacts.len())
    }

    fn warm_start_contacts(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        contacts: &[(ContactKey, CachedContact)],
    ) -> Result<(), SolverError> {
        for (key, contact) in contacts {
            self.failure_hint = Some(StepFailure::for_contact(
                StepFailureKind::InvalidSolverState,
                *key,
            ));
            if contact.lambdas.normal > 0.0 {
                self.apply_contact_lambda(colliders, *key, contact, contact.lambdas.normal)?;
            }
            self.failure_hint = None;
        }
        Ok(())
    }

    fn warm_start_analytic_contacts(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        proxies: &[AnalyticProxy],
        contacts: &[AnalyticCachedContact],
    ) -> Result<(), SolverError> {
        for cached in contacts {
            self.failure_hint = Some(StepFailure::for_contact(
                StepFailureKind::InvalidSolverState,
                cached.key,
            ));
            if cached.contact.lambdas.normal > 0.0 {
                self.apply_analytic_contact_lambda(
                    colliders,
                    proxies,
                    cached,
                    cached.contact.lambdas.normal,
                )?;
            }
            self.failure_hint = None;
        }
        Ok(())
    }

    fn solve_contact_iteration(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        contacts: &mut [(ContactKey, CachedContact)],
        iteration: u32,
    ) -> Result<(), SolverError> {
        if iteration.is_multiple_of(2) {
            for (key, contact) in contacts {
                self.failure_hint = Some(StepFailure::for_contact(
                    StepFailureKind::InvalidSolverState,
                    *key,
                ));
                self.solve_contact(colliders, *key, contact)?;
                self.failure_hint = None;
            }
        } else {
            for (key, contact) in contacts.iter_mut().rev() {
                self.failure_hint = Some(StepFailure::for_contact(
                    StepFailureKind::InvalidSolverState,
                    *key,
                ));
                self.solve_contact(colliders, *key, contact)?;
                self.failure_hint = None;
            }
        }
        Ok(())
    }

    fn solve_analytic_contact_iteration(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        proxies: &[AnalyticProxy],
        contacts: &mut [AnalyticCachedContact],
        iteration: u32,
    ) -> Result<(), SolverError> {
        if iteration.is_multiple_of(2) {
            for cached in contacts {
                self.solve_analytic_contact(colliders, proxies, cached)?;
            }
        } else {
            for cached in contacts.iter_mut().rev() {
                self.solve_analytic_contact(colliders, proxies, cached)?;
            }
        }
        Ok(())
    }

    fn solve_analytic_contact(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        proxies: &[AnalyticProxy],
        cached: &mut AnalyticCachedContact,
    ) -> Result<(), SolverError> {
        self.failure_hint = Some(StepFailure::for_contact(
            StepFailureKind::InvalidSolverState,
            cached.key,
        ));
        cached.contact.validate()?;
        let geometry = self.analytic_contact_geometry(colliders, proxies, cached)?;
        let properties = self.solve_properties(geometry.body)?;
        let linear = -geometry.body_to_proxy;
        let angular = -geometry.surface_offset.cross(geometry.body_to_proxy);
        let inverse_mass = properties.inverse_mass * linear.length_squared()
            + angular.dot(properties.world_inverse_inertia * angular);
        validate_nonnegative("analytic_contact_inverse_mass", inverse_mass)?;
        if inverse_mass <= MIN_CONTACT_DENOMINATOR {
            self.failure_hint = None;
            return Ok(());
        }
        let unconstrained_delta = -geometry.separation / inverse_mass;
        validate_finite("analytic_contact_delta_lambda", unconstrained_delta)?;
        let previous_lambda = cached.contact.lambdas.normal;
        let next_lambda = (previous_lambda + unconstrained_delta).max(0.0);
        let delta_lambda = next_lambda - previous_lambda;
        if delta_lambda != 0.0 {
            self.apply_analytic_contact_lambda(colliders, proxies, cached, delta_lambda)?;
        }
        cached.contact.lambdas.normal = next_lambda;
        self.failure_hint = None;
        Ok(())
    }

    fn apply_analytic_contact_lambda(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        proxies: &[AnalyticProxy],
        cached: &AnalyticCachedContact,
        delta_lambda: f32,
    ) -> Result<(), SolverError> {
        validate_finite("analytic_contact_delta_lambda", delta_lambda)?;
        let geometry = self.analytic_contact_geometry(colliders, proxies, cached)?;
        let properties = self.solve_properties(geometry.body)?;
        let linear = -geometry.body_to_proxy;
        let angular = -geometry.surface_offset.cross(geometry.body_to_proxy);
        self.apply_pose_correction(
            geometry.body,
            linear * properties.inverse_mass * delta_lambda,
            properties.world_inverse_inertia * angular * delta_lambda,
            CorrectionKind::NonVelocityContact,
        )
    }

    fn analytic_contact_geometry(
        &self,
        colliders: &[CapsuleColliderDesc],
        proxies: &[AnalyticProxy],
        cached: &AnalyticCachedContact,
    ) -> Result<AnalyticContactGeometry, SolverError> {
        let collider = collider_desc_by_id(colliders, cached.body_collider)?;
        let proxy = analytic_proxy_by_id(proxies, cached.proxy)?;
        let body_first = cached.key.pair.first() == collider.id;
        if cached.key.pair != PairKey::ordered(collider.id, proxy.id)? {
            return Err(SolverError::invalid_value(
                "analytic_contact_pair",
                cached.key.pair.first().get(),
            ));
        }
        let body_local = if body_first {
            cached.contact.local_a
        } else {
            cached.contact.local_b
        };
        let proxy_local = if body_first {
            cached.contact.local_b
        } else {
            cached.contact.local_a
        };
        let body_to_proxy = if body_first {
            cached.contact.normal
        } else {
            -cached.contact.normal
        };
        let (body_center, center_offset) = self.anchor_geometry(collider.body, body_local)?;
        let radius = collider.shape.effective_radius()?;
        let surface_offset = center_offset + body_to_proxy * radius;
        let body_surface = body_center + body_to_proxy * radius;
        let proxy_surface = proxy.pose.translation + proxy.pose.rotation * proxy_local;
        validate_vec3("analytic_contact_body_surface", body_surface)?;
        validate_vec3("analytic_contact_proxy_surface", proxy_surface)?;
        validate_vec3("analytic_contact_surface_offset", surface_offset)?;
        let separation = (proxy_surface - body_surface).dot(body_to_proxy);
        validate_finite("analytic_contact_separation", separation)?;
        Ok(AnalyticContactGeometry {
            body: collider.body,
            body_to_proxy,
            surface_offset,
            separation,
            proxy_velocity: proxy.linear_velocity,
        })
    }

    fn warm_start_mesh_contacts(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        meshes: &[DeformingMesh],
        contacts: &[MeshCachedContact],
        dt: f32,
    ) -> Result<(), SolverError> {
        for cached in contacts {
            self.failure_hint = Some(StepFailure::for_mesh_contact(
                StepFailureKind::InvalidSolverState,
                cached.key,
            ));
            if cached.contact.lambdas.normal > 0.0 {
                self.apply_mesh_contact_lambda(
                    colliders,
                    meshes,
                    cached,
                    cached.contact.lambdas.normal,
                    dt,
                )?;
            }
            self.failure_hint = None;
        }
        Ok(())
    }

    fn solve_mesh_contact_iteration(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        meshes: &[DeformingMesh],
        contacts: &mut [MeshCachedContact],
        iteration: u32,
        dt: f32,
    ) -> Result<(), SolverError> {
        if iteration.is_multiple_of(2) {
            for cached in contacts {
                self.solve_mesh_contact(colliders, meshes, cached, dt)?;
            }
        } else {
            for cached in contacts.iter_mut().rev() {
                self.solve_mesh_contact(colliders, meshes, cached, dt)?;
            }
        }
        Ok(())
    }

    fn solve_mesh_contact(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        meshes: &[DeformingMesh],
        cached: &mut MeshCachedContact,
        dt: f32,
    ) -> Result<(), SolverError> {
        self.failure_hint = Some(StepFailure::for_mesh_contact(
            StepFailureKind::InvalidSolverState,
            cached.key,
        ));
        cached.contact.validate()?;
        let geometry = self.mesh_contact_geometry(colliders, meshes, cached, dt)?;
        let properties = self.solve_properties(geometry.body)?;
        let linear = -geometry.body_to_mesh;
        let angular = -geometry.surface_offset.cross(geometry.body_to_mesh);
        let inverse_mass = properties.inverse_mass * linear.length_squared()
            + angular.dot(properties.world_inverse_inertia * angular);
        validate_nonnegative("mesh_contact_inverse_mass", inverse_mass)?;
        if inverse_mass <= MIN_CONTACT_DENOMINATOR {
            self.failure_hint = None;
            return Ok(());
        }
        let unconstrained_delta = -geometry.separation / inverse_mass;
        validate_finite("mesh_contact_delta_lambda", unconstrained_delta)?;
        let previous_lambda = cached.contact.lambdas.normal;
        let next_lambda = (previous_lambda + unconstrained_delta).max(0.0);
        let delta_lambda = next_lambda - previous_lambda;
        if delta_lambda != 0.0 {
            self.apply_mesh_contact_lambda(colliders, meshes, cached, delta_lambda, dt)?;
        }
        cached.contact.lambdas.normal = next_lambda;
        self.failure_hint = None;
        Ok(())
    }

    fn apply_mesh_contact_lambda(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        meshes: &[DeformingMesh],
        cached: &MeshCachedContact,
        delta_lambda: f32,
        dt: f32,
    ) -> Result<(), SolverError> {
        validate_finite("mesh_contact_delta_lambda", delta_lambda)?;
        let geometry = self.mesh_contact_geometry(colliders, meshes, cached, dt)?;
        let properties = self.solve_properties(geometry.body)?;
        let linear = -geometry.body_to_mesh;
        let angular = -geometry.surface_offset.cross(geometry.body_to_mesh);
        self.apply_pose_correction(
            geometry.body,
            linear * properties.inverse_mass * delta_lambda,
            properties.world_inverse_inertia * angular * delta_lambda,
            CorrectionKind::NonVelocityContact,
        )
    }

    fn mesh_contact_geometry(
        &self,
        colliders: &[CapsuleColliderDesc],
        meshes: &[DeformingMesh],
        cached: &MeshCachedContact,
        dt: f32,
    ) -> Result<MeshContactSolveGeometry, SolverError> {
        let collider = collider_desc_by_id(colliders, cached.body_collider)?;
        if cached.key.collider != collider.id {
            return Err(SolverError::invalid_value(
                "mesh_contact_collider",
                cached.key.collider.get(),
            ));
        }
        let mesh = mesh_by_id(meshes, cached.key.mesh)?;
        let triangle = mesh.moving_triangle(cached.key.triangle, dt)?;
        let barycentric = cached.contact.local_b;
        validate_vec3("mesh_contact_barycentric", barycentric)?;
        if barycentric.min_element() < -1.0e-4 || (barycentric.element_sum() - 1.0).abs() > 1.0e-4 {
            return Err(SolverError::invalid_value(
                "mesh_contact_barycentric",
                barycentric.element_sum(),
            ));
        }
        let body_to_mesh = cached.contact.normal;
        let (body_center, center_offset) =
            self.anchor_geometry(collider.body, cached.contact.local_a)?;
        let radius = collider.shape.effective_radius()?;
        let surface_offset = center_offset + body_to_mesh * radius;
        let body_surface = body_center + body_to_mesh * radius;
        let triangle_center = triangle.vertices[0] * barycentric.x
            + triangle.vertices[1] * barycentric.y
            + triangle.vertices[2] * barycentric.z;
        let mesh_surface = triangle_center - body_to_mesh * triangle.thickness;
        let surface_velocity = triangle.velocities[0] * barycentric.x
            + triangle.velocities[1] * barycentric.y
            + triangle.velocities[2] * barycentric.z;
        validate_vec3("mesh_contact_body_surface", body_surface)?;
        validate_vec3("mesh_contact_surface", mesh_surface)?;
        validate_vec3("mesh_contact_surface_offset", surface_offset)?;
        validate_vec3("mesh_contact_surface_velocity", surface_velocity)?;
        let separation = (mesh_surface - body_surface).dot(body_to_mesh);
        validate_finite("mesh_contact_separation", separation)?;
        Ok(MeshContactSolveGeometry {
            body: collider.body,
            body_to_mesh,
            surface_offset,
            separation,
            surface_velocity,
        })
    }

    fn solve_contact(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        key: ContactKey,
        contact: &mut CachedContact,
    ) -> Result<(), SolverError> {
        contact.validate()?;
        let collider_a = collider_desc_by_id(colliders, key.pair.first())?;
        let collider_b = collider_desc_by_id(colliders, key.pair.second())?;
        let (point_a, offset_a) = self.anchor_geometry(collider_a.body, contact.local_a)?;
        let (point_b, offset_b) = self.anchor_geometry(collider_b.body, contact.local_b)?;
        let linear_a = -contact.normal;
        let linear_b = contact.normal;
        let angular_a = -offset_a.cross(contact.normal);
        let angular_b = offset_b.cross(contact.normal);
        let inverse_mass = self.contact_inverse_mass(
            collider_a.body,
            collider_b.body,
            linear_a,
            linear_b,
            angular_a,
            angular_b,
        )?;
        if inverse_mass <= MIN_CONTACT_DENOMINATOR {
            return Ok(());
        }

        let radius_sum =
            collider_a.shape.effective_radius()? + collider_b.shape.effective_radius()?;
        validate_nonnegative("contact_radius_sum", radius_sum)?;
        let constraint_value = (point_b - point_a).dot(contact.normal) - radius_sum;
        validate_finite("contact_constraint_value", constraint_value)?;
        let unconstrained_delta = -constraint_value / inverse_mass;
        validate_finite("contact_delta_lambda", unconstrained_delta)?;
        let previous_lambda = contact.lambdas.normal;
        let next_lambda = (previous_lambda + unconstrained_delta).max(0.0);
        let delta_lambda = next_lambda - previous_lambda;
        if delta_lambda != 0.0 {
            self.apply_contact_lambda(colliders, key, contact, delta_lambda)?;
        }
        contact.lambdas.normal = next_lambda;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn contact_inverse_mass(
        &self,
        body_a: BodyId,
        body_b: BodyId,
        linear_a: Vec3,
        linear_b: Vec3,
        angular_a: Vec3,
        angular_b: Vec3,
    ) -> Result<f32, SolverError> {
        let properties_a = self.solve_properties(body_a)?;
        let properties_b = self.solve_properties(body_b)?;
        let inverse_mass = properties_a.inverse_mass * linear_a.length_squared()
            + angular_a.dot(properties_a.world_inverse_inertia * angular_a)
            + properties_b.inverse_mass * linear_b.length_squared()
            + angular_b.dot(properties_b.world_inverse_inertia * angular_b);
        validate_nonnegative("contact_inverse_mass", inverse_mass)?;
        Ok(inverse_mass)
    }

    fn apply_contact_lambda(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        key: ContactKey,
        contact: &CachedContact,
        delta_lambda: f32,
    ) -> Result<(), SolverError> {
        validate_finite("contact_delta_lambda", delta_lambda)?;
        let collider_a = collider_desc_by_id(colliders, key.pair.first())?;
        let collider_b = collider_desc_by_id(colliders, key.pair.second())?;
        let (_, offset_a) = self.anchor_geometry(collider_a.body, contact.local_a)?;
        let (_, offset_b) = self.anchor_geometry(collider_b.body, contact.local_b)?;
        let properties_a = self.solve_properties(collider_a.body)?;
        let properties_b = self.solve_properties(collider_b.body)?;
        let linear_a = -contact.normal;
        let linear_b = contact.normal;
        let angular_a = -offset_a.cross(contact.normal);
        let angular_b = offset_b.cross(contact.normal);

        self.apply_pose_correction(
            collider_a.body,
            linear_a * properties_a.inverse_mass * delta_lambda,
            properties_a.world_inverse_inertia * angular_a * delta_lambda,
            CorrectionKind::NonVelocityContact,
        )?;
        self.apply_pose_correction(
            collider_b.body,
            linear_b * properties_b.inverse_mass * delta_lambda,
            properties_b.world_inverse_inertia * angular_b * delta_lambda,
            CorrectionKind::NonVelocityContact,
        )
    }

    fn apply_contact_velocity_response(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        contacts: &mut [(ContactKey, CachedContact)],
    ) -> Result<(), SolverError> {
        for (key, contact) in contacts {
            self.failure_hint = Some(StepFailure::for_contact(
                StepFailureKind::InvalidSolverState,
                *key,
            ));
            self.solve_contact_velocity(colliders, *key, contact)?;
            self.failure_hint = None;
        }
        Ok(())
    }

    fn apply_analytic_contact_velocity_response(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        proxies: &[AnalyticProxy],
        contacts: &mut [AnalyticCachedContact],
    ) -> Result<(), SolverError> {
        for cached in contacts {
            self.failure_hint = Some(StepFailure::for_contact(
                StepFailureKind::InvalidSolverState,
                cached.key,
            ));
            self.solve_analytic_contact_velocity(colliders, proxies, cached)?;
            self.failure_hint = None;
        }
        Ok(())
    }

    fn solve_analytic_contact_velocity(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        proxies: &[AnalyticProxy],
        cached: &mut AnalyticCachedContact,
    ) -> Result<(), SolverError> {
        cached.contact.validate()?;
        cached.contact.lambdas.tangents = [0.0; 2];
        let collider = collider_desc_by_id(colliders, cached.body_collider)?;
        let geometry = self.analytic_contact_geometry(colliders, proxies, cached)?;
        if geometry.separation > CONTACT_SEPARATION_TOLERANCE {
            return Ok(());
        }
        let body_state = self
            .body(geometry.body)
            .ok_or(SolverError::MissingBody(geometry.body.get()))?;
        let body_velocity =
            body_state.velocity.linear + body_state.velocity.angular.cross(geometry.surface_offset);
        let relative_velocity = geometry.proxy_velocity - body_velocity;
        validate_vec3(
            "analytic_contact_relative_surface_velocity",
            relative_velocity,
        )?;
        let normal_velocity = relative_velocity.dot(geometry.body_to_proxy);
        validate_finite("analytic_contact_normal_velocity", normal_velocity)?;
        if normal_velocity >= 0.0 {
            return Ok(());
        }

        let restitution = if cached.contact.age == 1 && cached.was_separated {
            collider.material.restitution
        } else {
            0.0
        };
        let target_normal_velocity = -restitution * normal_velocity;
        validate_nonnegative(
            "analytic_contact_target_normal_velocity",
            target_normal_velocity,
        )?;
        let properties = self.solve_properties(geometry.body)?;
        let normal_linear = -geometry.body_to_proxy;
        let normal_angular = -geometry.surface_offset.cross(geometry.body_to_proxy);
        let normal_inverse_mass = properties.inverse_mass * normal_linear.length_squared()
            + normal_angular.dot(properties.world_inverse_inertia * normal_angular);
        validate_nonnegative("analytic_contact_normal_inverse_mass", normal_inverse_mass)?;
        if normal_inverse_mass <= MIN_CONTACT_DENOMINATOR {
            return Ok(());
        }
        let normal_impulse =
            ((target_normal_velocity - normal_velocity) / normal_inverse_mass).max(0.0);
        validate_nonnegative("analytic_contact_normal_impulse", normal_impulse)?;
        if normal_impulse == 0.0 {
            return Ok(());
        }
        self.apply_velocity_impulse(geometry.body, normal_linear, normal_angular, normal_impulse)?;

        let body_state = self
            .body(geometry.body)
            .ok_or(SolverError::MissingBody(geometry.body.get()))?;
        let body_velocity =
            body_state.velocity.linear + body_state.velocity.angular.cross(geometry.surface_offset);
        let relative_velocity = geometry.proxy_velocity - body_velocity;
        let tangential_velocity = relative_velocity
            - geometry.body_to_proxy * relative_velocity.dot(geometry.body_to_proxy);
        validate_vec3("analytic_contact_tangential_velocity", tangential_velocity)?;
        let tangential_speed = tangential_velocity.length();
        validate_nonnegative("analytic_contact_tangential_speed", tangential_speed)?;
        if tangential_speed <= MIN_CONTACT_DENOMINATOR.sqrt() {
            return Ok(());
        }
        let tangent = tangential_velocity / tangential_speed;
        let tangent_angular = geometry.surface_offset.cross(tangent);
        let tangent_inverse_mass = properties.inverse_mass
            + tangent_angular.dot(properties.world_inverse_inertia * tangent_angular);
        validate_nonnegative(
            "analytic_contact_tangent_inverse_mass",
            tangent_inverse_mass,
        )?;
        if tangent_inverse_mass <= MIN_CONTACT_DENOMINATOR {
            return Ok(());
        }
        let maximum_friction_impulse = collider.material.friction * normal_impulse;
        validate_nonnegative(
            "analytic_contact_maximum_friction_impulse",
            maximum_friction_impulse,
        )?;
        let tangent_impulse =
            (tangential_speed / tangent_inverse_mass).min(maximum_friction_impulse);
        validate_nonnegative("analytic_contact_tangent_impulse", tangent_impulse)?;
        if tangent_impulse == 0.0 {
            return Ok(());
        }
        self.apply_velocity_impulse(geometry.body, tangent, tangent_angular, tangent_impulse)?;
        let pair_impulse = if cached.key.pair.first() == cached.body_collider {
            tangent * tangent_impulse
        } else {
            -tangent * tangent_impulse
        };
        cached.contact.lambdas.tangents = [
            pair_impulse.dot(cached.contact.tangents[0]),
            pair_impulse.dot(cached.contact.tangents[1]),
        ];
        cached.contact.lambdas.validate()
    }

    fn apply_mesh_contact_velocity_response(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        meshes: &[DeformingMesh],
        contacts: &mut [MeshCachedContact],
        dt: f32,
    ) -> Result<(), SolverError> {
        for cached in contacts {
            self.failure_hint = Some(StepFailure::for_mesh_contact(
                StepFailureKind::InvalidSolverState,
                cached.key,
            ));
            self.solve_mesh_contact_velocity(colliders, meshes, cached, dt)?;
            self.failure_hint = None;
        }
        Ok(())
    }

    fn solve_mesh_contact_velocity(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        meshes: &[DeformingMesh],
        cached: &mut MeshCachedContact,
        dt: f32,
    ) -> Result<(), SolverError> {
        cached.contact.validate()?;
        cached.contact.lambdas.tangents = [0.0; 2];
        let collider = collider_desc_by_id(colliders, cached.body_collider)?;
        let geometry = self.mesh_contact_geometry(colliders, meshes, cached, dt)?;
        if geometry.separation > CONTACT_SEPARATION_TOLERANCE {
            return Ok(());
        }
        let body_state = self
            .body(geometry.body)
            .ok_or(SolverError::MissingBody(geometry.body.get()))?;
        let body_velocity =
            body_state.velocity.linear + body_state.velocity.angular.cross(geometry.surface_offset);
        let relative_velocity = geometry.surface_velocity - body_velocity;
        validate_vec3("mesh_contact_relative_surface_velocity", relative_velocity)?;
        let normal_velocity = relative_velocity.dot(geometry.body_to_mesh);
        validate_finite("mesh_contact_normal_velocity", normal_velocity)?;
        if normal_velocity >= 0.0 {
            return Ok(());
        }

        let restitution = if cached.contact.age == 1 && cached.was_separated {
            collider.material.restitution
        } else {
            0.0
        };
        let target_normal_velocity = -restitution * normal_velocity;
        validate_nonnegative(
            "mesh_contact_target_normal_velocity",
            target_normal_velocity,
        )?;
        let properties = self.solve_properties(geometry.body)?;
        let normal_linear = -geometry.body_to_mesh;
        let normal_angular = -geometry.surface_offset.cross(geometry.body_to_mesh);
        let normal_inverse_mass = properties.inverse_mass * normal_linear.length_squared()
            + normal_angular.dot(properties.world_inverse_inertia * normal_angular);
        validate_nonnegative("mesh_contact_normal_inverse_mass", normal_inverse_mass)?;
        if normal_inverse_mass <= MIN_CONTACT_DENOMINATOR {
            return Ok(());
        }
        let normal_impulse =
            ((target_normal_velocity - normal_velocity) / normal_inverse_mass).max(0.0);
        validate_nonnegative("mesh_contact_normal_impulse", normal_impulse)?;
        if normal_impulse == 0.0 {
            return Ok(());
        }
        self.apply_velocity_impulse(geometry.body, normal_linear, normal_angular, normal_impulse)?;

        let body_state = self
            .body(geometry.body)
            .ok_or(SolverError::MissingBody(geometry.body.get()))?;
        let body_velocity =
            body_state.velocity.linear + body_state.velocity.angular.cross(geometry.surface_offset);
        let relative_velocity = geometry.surface_velocity - body_velocity;
        let tangential_velocity = relative_velocity
            - geometry.body_to_mesh * relative_velocity.dot(geometry.body_to_mesh);
        validate_vec3("mesh_contact_tangential_velocity", tangential_velocity)?;
        let tangential_speed = tangential_velocity.length();
        validate_nonnegative("mesh_contact_tangential_speed", tangential_speed)?;
        if tangential_speed <= MIN_CONTACT_DENOMINATOR.sqrt() {
            return Ok(());
        }
        let tangent = tangential_velocity / tangential_speed;
        let tangent_angular = geometry.surface_offset.cross(tangent);
        let tangent_inverse_mass = properties.inverse_mass
            + tangent_angular.dot(properties.world_inverse_inertia * tangent_angular);
        validate_nonnegative("mesh_contact_tangent_inverse_mass", tangent_inverse_mass)?;
        if tangent_inverse_mass <= MIN_CONTACT_DENOMINATOR {
            return Ok(());
        }
        let maximum_friction_impulse = collider.material.friction * normal_impulse;
        validate_nonnegative(
            "mesh_contact_maximum_friction_impulse",
            maximum_friction_impulse,
        )?;
        let tangent_impulse =
            (tangential_speed / tangent_inverse_mass).min(maximum_friction_impulse);
        validate_nonnegative("mesh_contact_tangent_impulse", tangent_impulse)?;
        if tangent_impulse == 0.0 {
            return Ok(());
        }
        self.apply_velocity_impulse(geometry.body, tangent, tangent_angular, tangent_impulse)?;
        let impulse = tangent * tangent_impulse;
        cached.contact.lambdas.tangents = [
            impulse.dot(cached.contact.tangents[0]),
            impulse.dot(cached.contact.tangents[1]),
        ];
        cached.contact.lambdas.validate()
    }

    fn solve_contact_velocity(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        key: ContactKey,
        contact: &mut CachedContact,
    ) -> Result<(), SolverError> {
        contact.validate()?;
        contact.lambdas.tangents = [0.0; 2];

        let collider_a = collider_desc_by_id(colliders, key.pair.first())?;
        let collider_b = collider_desc_by_id(colliders, key.pair.second())?;
        let radius_a = collider_a.shape.effective_radius()?;
        let radius_b = collider_b.shape.effective_radius()?;
        let radius_sum = radius_a + radius_b;
        validate_nonnegative("contact_radius_sum", radius_sum)?;

        let (center_a, center_offset_a) = self.anchor_geometry(collider_a.body, contact.local_a)?;
        let (center_b, center_offset_b) = self.anchor_geometry(collider_b.body, contact.local_b)?;
        let centerline_separation = center_a.distance(center_b);
        validate_nonnegative("contact_centerline_separation", centerline_separation)?;
        if centerline_separation > radius_sum + CONTACT_SEPARATION_TOLERANCE {
            return Ok(());
        }

        let surface_offset_a = center_offset_a + contact.normal * radius_a;
        let surface_offset_b = center_offset_b - contact.normal * radius_b;
        validate_vec3("contact_surface_offset_a", surface_offset_a)?;
        validate_vec3("contact_surface_offset_b", surface_offset_b)?;

        let friction_product = collider_a.material.friction * collider_b.material.friction;
        let combined_friction = friction_product.sqrt();
        let combined_restitution = collider_a
            .material
            .restitution
            .max(collider_b.material.restitution);
        validate_nonnegative("combined_contact_friction", combined_friction)?;
        validate_nonnegative("combined_contact_restitution", combined_restitution)?;

        let relative_velocity = self.contact_relative_surface_velocity(
            collider_a.body,
            collider_b.body,
            surface_offset_a,
            surface_offset_b,
        )?;
        let normal_velocity = relative_velocity.dot(contact.normal);
        validate_finite("contact_normal_velocity", normal_velocity)?;
        if normal_velocity >= 0.0 {
            return Ok(());
        }

        let restitution = if contact.age == 1
            && self.contact_pair_was_separated(collider_a, collider_b, contact.normal)?
        {
            combined_restitution
        } else {
            0.0
        };
        let target_normal_velocity = -restitution * normal_velocity;
        validate_nonnegative("contact_target_normal_velocity", target_normal_velocity)?;

        let normal_inverse_mass = self.contact_inverse_mass(
            collider_a.body,
            collider_b.body,
            -contact.normal,
            contact.normal,
            -surface_offset_a.cross(contact.normal),
            surface_offset_b.cross(contact.normal),
        )?;
        if normal_inverse_mass <= MIN_CONTACT_DENOMINATOR {
            return Ok(());
        }
        let normal_impulse =
            ((target_normal_velocity - normal_velocity) / normal_inverse_mass).max(0.0);
        validate_nonnegative("contact_normal_velocity_impulse", normal_impulse)?;
        if normal_impulse == 0.0 {
            return Ok(());
        }
        let maximum_friction_impulse = combined_friction * normal_impulse;
        validate_nonnegative("contact_maximum_friction_impulse", maximum_friction_impulse)?;
        self.apply_contact_velocity_impulse(
            collider_a.body,
            collider_b.body,
            surface_offset_a,
            surface_offset_b,
            contact.normal,
            normal_impulse,
        )?;

        let relative_velocity = self.contact_relative_surface_velocity(
            collider_a.body,
            collider_b.body,
            surface_offset_a,
            surface_offset_b,
        )?;
        let tangential_velocity =
            relative_velocity - contact.normal * relative_velocity.dot(contact.normal);
        validate_vec3("contact_tangential_velocity", tangential_velocity)?;
        let tangential_speed = tangential_velocity.length();
        validate_nonnegative("contact_tangential_speed", tangential_speed)?;
        if tangential_speed <= MIN_CONTACT_DENOMINATOR.sqrt() {
            return Ok(());
        }

        let tangent = tangential_velocity / tangential_speed;
        validate_vec3("contact_tangent_direction", tangent)?;
        let tangent_inverse_mass = self.contact_inverse_mass(
            collider_a.body,
            collider_b.body,
            -tangent,
            tangent,
            -surface_offset_a.cross(tangent),
            surface_offset_b.cross(tangent),
        )?;
        if tangent_inverse_mass <= MIN_CONTACT_DENOMINATOR {
            return Ok(());
        }

        let tangent_impulse = (-tangential_speed / tangent_inverse_mass)
            .clamp(-maximum_friction_impulse, maximum_friction_impulse);
        validate_finite("contact_tangent_velocity_impulse", tangent_impulse)?;
        if tangent_impulse == 0.0 {
            return Ok(());
        }
        self.apply_contact_velocity_impulse(
            collider_a.body,
            collider_b.body,
            surface_offset_a,
            surface_offset_b,
            tangent,
            tangent_impulse,
        )?;

        let tangent_impulse_vector = tangent * tangent_impulse;
        contact.lambdas.tangents = [
            tangent_impulse_vector.dot(contact.tangents[0]),
            tangent_impulse_vector.dot(contact.tangents[1]),
        ];
        contact.lambdas.validate()
    }

    fn contact_relative_surface_velocity(
        &self,
        body_a: BodyId,
        body_b: BodyId,
        surface_offset_a: Vec3,
        surface_offset_b: Vec3,
    ) -> Result<Vec3, SolverError> {
        let state_a = self
            .body(body_a)
            .ok_or(SolverError::MissingBody(body_a.get()))?;
        let state_b = self
            .body(body_b)
            .ok_or(SolverError::MissingBody(body_b.get()))?;
        let velocity_a = state_a.velocity.linear + state_a.velocity.angular.cross(surface_offset_a);
        let velocity_b = state_b.velocity.linear + state_b.velocity.angular.cross(surface_offset_b);
        let relative_velocity = velocity_b - velocity_a;
        validate_vec3("contact_relative_surface_velocity", relative_velocity)?;
        Ok(relative_velocity)
    }

    fn contact_pair_was_separated(
        &self,
        collider_a: &CapsuleColliderDesc,
        collider_b: &CapsuleColliderDesc,
        cached_normal: Vec3,
    ) -> Result<bool, SolverError> {
        let state_a = self
            .body(collider_a.body)
            .ok_or(SolverError::MissingBody(collider_a.body.get()))?;
        let state_b = self
            .body(collider_b.body)
            .ok_or(SolverError::MissingBody(collider_b.body.get()))?;
        let previous_a =
            runtime_collider(collider_a, state_a.previous_pose, state_a.velocity.linear);
        let previous_b =
            runtime_collider(collider_b, state_b.previous_pose, state_b.velocity.linear);
        Ok(crate::capsule_manifold(&previous_a, &previous_b, Some(cached_normal))?.is_none())
    }

    #[allow(clippy::too_many_arguments)]
    fn apply_contact_velocity_impulse(
        &mut self,
        body_a: BodyId,
        body_b: BodyId,
        surface_offset_a: Vec3,
        surface_offset_b: Vec3,
        direction: Vec3,
        impulse: f32,
    ) -> Result<(), SolverError> {
        validate_vec3("contact_impulse_direction", direction)?;
        validate_finite("contact_velocity_impulse", impulse)?;
        self.apply_velocity_impulse(
            body_a,
            -direction,
            -surface_offset_a.cross(direction),
            impulse,
        )?;
        self.apply_velocity_impulse(
            body_b,
            direction,
            surface_offset_b.cross(direction),
            impulse,
        )
    }

    fn contact_residuals(&mut self) -> Result<(u32, f32), SolverError> {
        let colliders = self.predicted_colliders()?;
        let pipeline = CollisionPipeline::try_new(&self.constraints)?;
        let candidates = pipeline.candidate_pairs(&colliders, &colliders)?;
        let mut active = 0_usize;
        let mut deepest: f32 = 0.0;
        for pair in candidates {
            let contact = self
                .cached_contacts
                .iter()
                .find(|(key, _)| key.pair == pair)
                .map_or_else(|| ContactKey::new(pair, 0, 0), |(key, _)| *key);
            self.failure_hint = Some(StepFailure::for_contact(
                StepFailureKind::InvalidSolverState,
                contact,
            ));
            let collider_a = collider_by_id(&colliders, pair.first())?;
            let collider_b = collider_by_id(&colliders, pair.second())?;
            let cached_normal = cached_normal_for_pair(&self.cached_contacts, pair);
            if let Some(manifold) = crate::capsule_manifold(collider_a, collider_b, cached_normal)?
            {
                for point in manifold.points {
                    active = active.checked_add(1).ok_or_else(|| {
                        SolverError::invalid_value("contact_count", active as f64)
                    })?;
                    validate_nonnegative("contact_penetration", point.penetration)?;
                    deepest = deepest.max(point.penetration);
                }
            }
            self.failure_hint = None;
        }
        let proxies = self.analytic_proxies.clone();
        for collider in &colliders {
            for proxy in &proxies {
                if !proxy.collision_enabled {
                    continue;
                }
                let pair = PairKey::ordered(collider.id, proxy.id)?;
                let contact = ContactKey::new(pair, 0, 0);
                self.failure_hint = Some(StepFailure::for_contact(
                    StepFailureKind::InvalidSolverState,
                    contact,
                ));
                let mut stationary_collider = collider.clone();
                stationary_collider.linear_velocity = Vec3::ZERO;
                let mut stationary_proxy = *proxy;
                stationary_proxy.linear_velocity = Vec3::ZERO;
                if let Some(geometry) = crate::capsule_analytic_contact(
                    &stationary_collider,
                    &stationary_proxy,
                    self.config.fixed_dt,
                )? {
                    active = active.checked_add(1).ok_or_else(|| {
                        SolverError::invalid_value("contact_count", active as f64)
                    })?;
                    let penetration = (-geometry.separation).max(0.0);
                    validate_nonnegative("contact_penetration", penetration)?;
                    deepest = deepest.max(penetration);
                }
                self.failure_hint = None;
            }
        }
        let meshes = core::mem::take(&mut self.meshes);
        let mesh_result = (|| {
            for collider in &colliders {
                let mut stationary_collider = collider.clone();
                stationary_collider.linear_velocity = Vec3::ZERO;
                let radius = collider.shape.effective_radius()?;
                for mesh in &meshes {
                    let mut raw = Vec::new();
                    for triangle_id in mesh.query_capsule_sweep(collider, collider)? {
                        let key_hint = MeshContactKey {
                            collider: collider.id,
                            mesh: mesh.id(),
                            triangle: triangle_id,
                            feature: 0,
                        };
                        self.failure_hint = Some(StepFailure::for_mesh_contact(
                            StepFailureKind::InvalidSolverState,
                            key_hint,
                        ));
                        let mut triangle =
                            mesh.moving_triangle(triangle_id, self.config.fixed_dt)?;
                        triangle.velocities = [Vec3::ZERO; 3];
                        let history = self
                            .cached_mesh_contacts
                            .iter()
                            .find(|cached| {
                                cached.body_collider == collider.id
                                    && cached.key.mesh == mesh.id()
                                    && cached.key.triangle == triangle_id
                            })
                            .map(|cached| MeshContactHistory {
                                normal: cached.contact.normal,
                            });
                        if let Some(contact) =
                            capsule_triangle_contact(&stationary_collider, &triangle, history)?
                        {
                            raw.push(contact);
                        }
                    }
                    for contact in cluster_mesh_contacts(
                        raw,
                        MeshManifoldLimits::try_new(4, radius * 0.25, 0.95)?,
                    )? {
                        self.failure_hint = Some(StepFailure::for_mesh_contact(
                            StepFailureKind::InvalidSolverState,
                            contact.key,
                        ));
                        active = active.checked_add(1).ok_or_else(|| {
                            SolverError::invalid_value("contact_count", active as f64)
                        })?;
                        validate_nonnegative(
                            "mesh_contact_penetration",
                            contact.geometry.penetration,
                        )?;
                        deepest = deepest.max(contact.geometry.penetration);
                    }
                    self.failure_hint = None;
                }
            }
            Ok::<(), SolverError>(())
        })();
        self.meshes = meshes;
        mesh_result?;
        let active = contact_count(active)?;
        validate_finite("deepest_penetration", deepest)?;
        Ok((active, deepest))
    }

    fn collider_snapshots(
        &self,
    ) -> Result<(Vec<CapsuleCollider>, Vec<CapsuleCollider>), SolverError> {
        let mut previous = Vec::with_capacity(self.colliders.len());
        let mut predicted = Vec::with_capacity(self.colliders.len());
        for collider in &self.colliders {
            let state = self
                .body(collider.body)
                .ok_or(SolverError::MissingBody(collider.body.get()))?;
            previous.push(runtime_collider(
                collider,
                state.previous_pose,
                state.velocity.linear,
            ));
            predicted.push(runtime_collider(
                collider,
                state.predicted_pose,
                state.velocity.linear,
            ));
        }
        Ok((previous, predicted))
    }

    fn predicted_colliders(&self) -> Result<Vec<CapsuleCollider>, SolverError> {
        let mut colliders = Vec::with_capacity(self.colliders.len());
        for collider in &self.colliders {
            let state = self
                .body(collider.body)
                .ok_or(SolverError::MissingBody(collider.body.get()))?;
            colliders.push(runtime_collider(
                collider,
                state.predicted_pose,
                state.velocity.linear,
            ));
        }
        Ok(colliders)
    }

    fn animation_target(&self, id: BodyId) -> Option<AnimationTarget> {
        self.animation_targets
            .binary_search_by_key(&id, |(body_id, _)| *body_id)
            .ok()
            .map(|index| self.animation_targets[index].1)
    }

    fn apply_animation_inertia(&mut self, target_alpha: f32) -> Result<(), SolverError> {
        let targets = self.animation_targets.clone();
        for (id, target) in targets {
            self.failure_hint = Some(StepFailure::for_body(
                StepFailureKind::InvalidSolverState,
                id,
            ));
            let previous = target.pose_at(self.target_alpha);
            let current = target.pose_at(target_alpha);
            let (translation, rotation) = crate::constraint::reference_frame_delta(
                previous,
                current,
                target.translation_inertia,
                target.rotation_inertia,
            );
            self.shift_reference_frame(id, previous, translation, rotation)?;
            self.failure_hint = None;
        }
        Ok(())
    }

    fn shift_reference_frame(
        &mut self,
        id: BodyId,
        previous_target: Pose,
        translation: Vec3,
        rotation: Quat,
    ) -> Result<(), SolverError> {
        let state = self.body_mut(id)?;
        state.predicted_pose.translation = previous_target.translation
            + translation
            + rotation * (state.predicted_pose.translation - previous_target.translation);
        state.predicted_pose.rotation = (rotation * state.predicted_pose.rotation).normalize();
        state.integrated_pose.translation = previous_target.translation
            + translation
            + rotation * (state.integrated_pose.translation - previous_target.translation);
        state.integrated_pose.rotation = (rotation * state.integrated_pose.rotation).normalize();
        Ok(())
    }
}

fn collider_metadata_unchanged(
    previous: &[CapsuleColliderDesc],
    current: &[CapsuleColliderDesc],
    id: crate::ColliderId,
) -> bool {
    let previous = previous
        .binary_search_by_key(&id, |collider| collider.id)
        .ok()
        .map(|index| &previous[index]);
    let current = current
        .binary_search_by_key(&id, |collider| collider.id)
        .ok()
        .map(|index| &current[index]);
    previous.is_some() && previous == current
}

fn analytic_proxy_metadata_unchanged(
    previous: &[AnalyticProxy],
    current: &[AnalyticProxy],
    id: crate::ColliderId,
) -> bool {
    let previous = previous
        .binary_search_by_key(&id, |proxy| proxy.id)
        .ok()
        .map(|index| previous[index]);
    let current = current
        .binary_search_by_key(&id, |proxy| proxy.id)
        .ok()
        .map(|index| current[index]);
    matches!(
        (previous, current),
        (Some(previous), Some(current))
            if previous.shape == current.shape
                && previous.collision_enabled == current.collision_enabled
    )
}

fn mesh_metadata_unchanged(
    previous: &[DeformingMesh],
    current: &[DeformingMesh],
    id: MeshId,
) -> bool {
    let previous = previous
        .binary_search_by_key(&id, DeformingMesh::id)
        .ok()
        .map(|index| &previous[index]);
    let current = current
        .binary_search_by_key(&id, DeformingMesh::id)
        .ok()
        .map(|index| &current[index]);
    matches!(
        (previous, current),
        (Some(previous), Some(current))
            if previous.validation_report().topology_hash
                == current.validation_report().topology_hash
                && previous.thickness() == current.thickness()
    )
}

fn runtime_collider(
    collider: &CapsuleColliderDesc,
    pose: Pose,
    linear_velocity: Vec3,
) -> CapsuleCollider {
    CapsuleCollider {
        id: collider.id,
        body: collider.body,
        shape: collider.shape,
        pose,
        linear_velocity,
        filter: collider.filter.clone(),
    }
}

fn collider_by_id(
    colliders: &[CapsuleCollider],
    id: crate::ColliderId,
) -> Result<&CapsuleCollider, SolverError> {
    let index = colliders
        .binary_search_by_key(&id, |collider| collider.id)
        .map_err(|_| SolverError::invalid_value("contact_collider_id", id.get()))?;
    Ok(&colliders[index])
}

fn collider_desc_by_id(
    colliders: &[CapsuleColliderDesc],
    id: crate::ColliderId,
) -> Result<&CapsuleColliderDesc, SolverError> {
    let index = colliders
        .binary_search_by_key(&id, |collider| collider.id)
        .map_err(|_| SolverError::invalid_value("contact_collider_id", id.get()))?;
    Ok(&colliders[index])
}

fn analytic_proxy_by_id(
    proxies: &[AnalyticProxy],
    id: crate::ColliderId,
) -> Result<&AnalyticProxy, SolverError> {
    let index = proxies
        .binary_search_by_key(&id, |proxy| proxy.id)
        .map_err(|_| SolverError::invalid_value("analytic_proxy_id", id.get()))?;
    Ok(&proxies[index])
}

fn mesh_by_id(meshes: &[DeformingMesh], id: MeshId) -> Result<&DeformingMesh, SolverError> {
    let index = meshes
        .binary_search_by_key(&id, DeformingMesh::id)
        .map_err(|_| SolverError::invalid_value("mesh_id", id.get()))?;
    Ok(&meshes[index])
}

fn analytic_contact_matches(
    current: &CachedContact,
    previous: &CachedContact,
    radius: f32,
) -> Result<bool, SolverError> {
    current.validate()?;
    previous.validate()?;
    validate_nonnegative("analytic_contact_match_radius", radius)?;
    let threshold_squared = f64::from(radius).powi(2);
    let anchor_distance_squared = f64::from((current.local_a - previous.local_a).length_squared())
        + f64::from((current.local_b - previous.local_b).length_squared());
    Ok(
        current.normal.dot(previous.normal) >= CONTACT_MATCH_NORMAL_MIN_DOT
            && anchor_distance_squared <= threshold_squared,
    )
}

fn mesh_contact_matches(
    current: &CachedContact,
    previous: &CachedContact,
    radius: f32,
) -> Result<bool, SolverError> {
    current.validate()?;
    previous.validate()?;
    validate_nonnegative("mesh_contact_match_radius", radius)?;
    let body_distance_squared = f64::from((current.local_a - previous.local_a).length_squared());
    let barycentric_distance_squared =
        f64::from((current.local_b - previous.local_b).length_squared());
    Ok(
        current.normal.dot(previous.normal) >= CONTACT_MATCH_NORMAL_MIN_DOT
            && body_distance_squared <= f64::from(radius).powi(2)
            && barycentric_distance_squared <= 0.25_f64.powi(2),
    )
}

fn cached_normal_for_pair(contacts: &[(ContactKey, CachedContact)], pair: PairKey) -> Option<Vec3> {
    contacts
        .iter()
        .find(|(key, _)| key.pair == pair)
        .map(|(_, contact)| contact.normal)
}

fn local_point(pose: Pose, point: Vec3) -> Vec3 {
    pose.rotation.conjugate() * (point - pose.translation)
}

#[derive(Clone, Copy)]
struct ContactMatchContext {
    previous_a: Pose,
    previous_b: Pose,
    current_a: Pose,
    current_b: Pose,
    radius_sum: f32,
}

fn assign_contact_slots(
    pair: PairKey,
    mut current: Vec<CachedContact>,
    previous: &[(ContactKey, CachedContact)],
    context: ContactMatchContext,
) -> Result<Vec<(ContactKey, CachedContact)>, SolverError> {
    if current.len() > 2 {
        return Err(SolverError::invalid_value(
            "contact_manifold_points",
            current.len() as f64,
        ));
    }
    if previous.len() > 2 {
        return Err(SolverError::invalid_value(
            "cached_contact_count",
            previous.len() as f64,
        ));
    }

    let mut reserved_keys = Vec::with_capacity(previous.len() + current.len());
    for (key, contact) in previous {
        if key.pair != pair || reserved_keys.contains(key) {
            return Err(SolverError::invalid_value(
                "cached_contact_feature",
                key.feature_a,
            ));
        }
        contact.validate()?;
        reserved_keys.push(*key);
    }

    current.sort_by(contact_anchor_order);
    let matches = match_contact_anchors(&current, previous, context)?;
    let mut keys: Vec<Option<ContactKey>> = vec![None; current.len()];
    let mut used_keys = Vec::with_capacity(current.len());

    for (current_index, previous_index) in matches.into_iter().enumerate() {
        let Some(previous_index) = previous_index else {
            continue;
        };
        let (key, cached) = previous[previous_index];
        if used_keys.contains(&key) {
            return Err(SolverError::invalid_value(
                "cached_contact_feature",
                key.feature_a,
            ));
        }
        current[current_index].lambdas = cached.lambdas;
        current[current_index].age = cached
            .age
            .checked_add(1)
            .ok_or_else(|| SolverError::invalid_value("contact_age", cached.age))?;
        current[current_index].validate()?;
        keys[current_index] = Some(key);
        used_keys.push(key);
    }

    for key in &mut keys {
        if key.is_some() {
            continue;
        }
        let mut slot = 0_u32;
        let assigned = loop {
            let candidate = ContactKey::new(pair, slot, slot);
            if !reserved_keys.contains(&candidate) && !used_keys.contains(&candidate) {
                break candidate;
            }
            slot = slot
                .checked_add(1)
                .ok_or_else(|| SolverError::invalid_value("contact_feature_slot", slot))?;
        };
        *key = Some(assigned);
        used_keys.push(assigned);
    }

    let mut assigned = Vec::with_capacity(current.len());
    for (contact, key) in current.into_iter().zip(keys) {
        let key = key.ok_or_else(|| SolverError::invalid_value("contact_feature_slot", -1.0))?;
        assigned.push((key, contact));
    }
    Ok(assigned)
}

fn match_contact_anchors(
    current: &[CachedContact],
    previous: &[(ContactKey, CachedContact)],
    context: ContactMatchContext,
) -> Result<Vec<Option<usize>>, SolverError> {
    let mut matches = vec![None; current.len()];
    match (current.len(), previous.len()) {
        (0, _) | (_, 0) => {}
        (1, 1) => {
            if contact_match_score(&current[0], &previous[0].1, context)?.is_some() {
                matches[0] = Some(0);
            }
        }
        (1, 2) => {
            let first = contact_match_score(&current[0], &previous[0].1, context)?;
            let second = contact_match_score(&current[0], &previous[1].1, context)?;
            matches[0] = preferred_score(first, second);
        }
        (2, 1) => {
            let first = contact_match_score(&current[0], &previous[0].1, context)?;
            let second = contact_match_score(&current[1], &previous[0].1, context)?;
            if let Some(current_index) = preferred_score(first, second) {
                matches[current_index] = Some(0);
            }
        }
        (2, 2) => {
            let scores = [
                [
                    contact_match_score(&current[0], &previous[0].1, context)?,
                    contact_match_score(&current[0], &previous[1].1, context)?,
                ],
                [
                    contact_match_score(&current[1], &previous[0].1, context)?,
                    contact_match_score(&current[1], &previous[1].1, context)?,
                ],
            ];
            match_contact_pairs(&mut matches, scores);
        }
        _ => {
            return Err(SolverError::invalid_value(
                "contact_match_count",
                current.len().max(previous.len()) as f64,
            ));
        }
    }
    Ok(matches)
}

fn preferred_score(first: Option<f64>, second: Option<f64>) -> Option<usize> {
    match (first, second) {
        (Some(first), Some(second)) => Some(if second < first { 1 } else { 0 }),
        (Some(_), None) => Some(0),
        (None, Some(_)) => Some(1),
        (None, None) => None,
    }
}

fn match_contact_pairs(matches: &mut [Option<usize>], scores: [[Option<f64>; 2]; 2]) {
    let direct = scores[0][0].zip(scores[1][1]).map(|(a, b)| a + b);
    let crossed = scores[0][1].zip(scores[1][0]).map(|(a, b)| a + b);
    match (direct, crossed) {
        (Some(direct), Some(crossed)) if crossed < direct => {
            matches[0] = Some(1);
            matches[1] = Some(0);
        }
        (Some(_), _) => {
            matches[0] = Some(0);
            matches[1] = Some(1);
        }
        (None, Some(_)) => {
            matches[0] = Some(1);
            matches[1] = Some(0);
        }
        (None, None) => {
            let mut best = None;
            for (current_index, row) in scores.into_iter().enumerate() {
                for (previous_index, score) in row.into_iter().enumerate() {
                    let Some(score) = score else {
                        continue;
                    };
                    if best.is_none_or(|(best_score, _, _)| score < best_score) {
                        best = Some((score, current_index, previous_index));
                    }
                }
            }
            if let Some((_, current_index, previous_index)) = best {
                matches[current_index] = Some(previous_index);
            }
        }
    }
}

fn contact_match_score(
    current: &CachedContact,
    previous: &CachedContact,
    context: ContactMatchContext,
) -> Result<Option<f64>, SolverError> {
    let normal_dot = current.normal.dot(previous.normal);
    if !normal_dot.is_finite() {
        return Err(SolverError::invalid_value("contact_normal_dot", normal_dot));
    }
    if normal_dot < CONTACT_MATCH_NORMAL_MIN_DOT {
        return Ok(None);
    }

    let local_score = vec3_distance_squared(current.local_a, previous.local_a)
        + vec3_distance_squared(current.local_b, previous.local_b);
    let previous_midpoint =
        contact_world_midpoint(previous, context.previous_a, context.previous_b)?;
    let current_midpoint = contact_world_midpoint(current, context.current_a, context.current_b)?;
    let world_score = CONTACT_MATCH_WORLD_MIDPOINT_WEIGHT
        * vec3_distance_squared(current_midpoint, previous_midpoint);
    let score = local_score.min(world_score);

    let radius_sum = f64::from(context.radius_sum);
    if !radius_sum.is_finite() || radius_sum <= 0.0 {
        return Err(SolverError::invalid_value(
            "contact_match_radius_sum",
            radius_sum,
        ));
    }
    // The local metric sums two squared anchor displacements. Weighting the
    // world midpoint by two gives it the same scale when both anchors
    // translate together, so the same factor also scales the radius threshold.
    let maximum_displacement = CONTACT_MATCH_RADIUS_SCALE * radius_sum;
    let maximum_score =
        CONTACT_MATCH_WORLD_MIDPOINT_WEIGHT * maximum_displacement * maximum_displacement;
    if !score.is_finite() || !maximum_score.is_finite() {
        return Err(SolverError::invalid_value("contact_match_score", score));
    }
    Ok((score <= maximum_score).then_some(score))
}

fn contact_world_midpoint(
    contact: &CachedContact,
    pose_a: Pose,
    pose_b: Pose,
) -> Result<Vec3, SolverError> {
    let world_a = pose_a.translation + pose_a.rotation * contact.local_a;
    let world_b = pose_b.translation + pose_b.rotation * contact.local_b;
    validate_vec3("contact_world_anchor_a", world_a)?;
    validate_vec3("contact_world_anchor_b", world_b)?;
    let midpoint = (world_a + world_b) * 0.5;
    validate_vec3("contact_world_midpoint", midpoint)?;
    Ok(midpoint)
}

fn vec3_distance_squared(a: Vec3, b: Vec3) -> f64 {
    let x = f64::from(a.x) - f64::from(b.x);
    let y = f64::from(a.y) - f64::from(b.y);
    let z = f64::from(a.z) - f64::from(b.z);
    x * x + y * y + z * z
}

fn contact_anchor_order(a: &CachedContact, b: &CachedContact) -> core::cmp::Ordering {
    vec3_order(a.local_a, b.local_a).then_with(|| vec3_order(a.local_b, b.local_b))
}

fn vec3_order(a: Vec3, b: Vec3) -> core::cmp::Ordering {
    a.x.total_cmp(&b.x)
        .then_with(|| a.y.total_cmp(&b.y))
        .then_with(|| a.z.total_cmp(&b.z))
}

fn tangent_basis(normal: Vec3) -> [Vec3; 2] {
    let reference = if normal.x.abs() <= normal.y.abs() && normal.x.abs() <= normal.z.abs() {
        Vec3::X
    } else if normal.y.abs() <= normal.z.abs() {
        Vec3::Y
    } else {
        Vec3::Z
    };
    let first = normal.cross(reference).normalize();
    [first, normal.cross(first).normalize()]
}

fn contact_count(count: usize) -> Result<u32, SolverError> {
    u32::try_from(count).map_err(|_| SolverError::invalid_value("contact_count", count as f64))
}

fn validate_dt(dt: f32) -> Result<(), SolverError> {
    validate_nonnegative("dt", dt)?;
    if dt == 0.0 {
        Err(SolverError::invalid_value("dt", dt))
    } else {
        Ok(())
    }
}

fn max_abs_vec3(value: Vec3) -> f32 {
    value.x.abs().max(value.y.abs()).max(value.z.abs())
}

fn record_phase(
    timings: &mut Option<&mut StepPhaseTimings>,
    started: Option<Instant>,
    update: impl FnOnce(&mut StepPhaseTimings, f64),
) {
    if let (Some(timings), Some(started)) = (timings.as_deref_mut(), started) {
        update(timings, started.elapsed().as_secs_f64());
    }
}

fn step_failure_error(failure: &StepFailure) -> SolverError {
    if let Some(source) = &failure.source {
        source.clone()
    } else if let Some(body) = failure.body {
        SolverError::NonFiniteState(body)
    } else {
        SolverError::invalid_value("solver_state", failure.kind as u8)
    }
}
