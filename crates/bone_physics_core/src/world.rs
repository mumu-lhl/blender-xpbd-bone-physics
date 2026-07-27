use glam::{Mat3, Quat, Vec3};

use crate::error::{validate_finite, validate_nonnegative};
use crate::math::world_inverse_inertia;
use crate::pose::validate_vec3;
use crate::{
    AnimationTarget, CachedContact, CapsuleCollider, CapsuleColliderDesc, CollisionPipeline,
    ConstraintId, ConstraintSchedule, ContactKey, ContactLambdas, PairKey, PersistentConstraint,
    QualityPolicy,
};
use crate::{BodyId, BoneBodyDesc, BoneBodyState, Pose, SolverError, Velocity};

const MIN_CONTACT_DENOMINATOR: f32 = 1.0e-12;

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
    cached_contacts: Vec<(ContactKey, CachedContact)>,
    animation_targets: Vec<(BodyId, AnimationTarget)>,
    target_alpha: f32,
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
            cached_contacts: Vec::new(),
            animation_targets: Vec::new(),
            target_alpha: 0.0,
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
        self.colliders = colliders;
        Ok(())
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
        validate_finite("target_alpha", target_alpha)?;
        if !(0.0..=1.0).contains(&target_alpha) {
            return Err(SolverError::invalid_value("target_alpha", target_alpha));
        }
        self.predict(dt)?;
        self.apply_animation_inertia(target_alpha)?;
        let generated_contacts = self.generate_contacts()?;

        let schedule = self.constraint_schedule.clone();
        let colliders = self.colliders.clone();
        let mut constraints = core::mem::take(&mut self.constraints);
        let mut contacts = core::mem::take(&mut self.cached_contacts);
        let solve_result = (|| {
            self.warm_start_contacts(&colliders, &contacts)?;
            for iteration in 0..options.position_iterations {
                for (_, id) in schedule.iteration_order(iteration) {
                    let index = constraints
                        .binary_search_by_key(&id, |constraint| constraint.id)
                        .map_err(|_| {
                            SolverError::invalid_value("constraint_schedule_id", id.get())
                        })?;
                    self.solve_constraint(&mut constraints[index], target_alpha, dt)?;
                }
                self.solve_contact_iteration(&colliders, &mut contacts, iteration)?;
            }
            Ok::<(), SolverError>(())
        })();
        self.constraints = constraints;
        self.cached_contacts = contacts;
        solve_result?;

        let max_constraint_error = self.max_constraint_error()?;
        let (active_contacts, deepest_penetration) = self.contact_residuals()?;
        self.rebuild_velocities(dt)?;

        let previous_target_alpha = self.target_alpha;
        let constraints = core::mem::take(&mut self.constraints);
        let damping_result = (|| {
            for constraint in &constraints {
                constraint.apply_damping(self, dt)?;
                let target = self.animation_target(constraint.body_a);
                constraint.apply_recovery_damping(
                    self,
                    target,
                    previous_target_alpha,
                    target_alpha,
                    dt,
                )?;
            }
            Ok::<(), SolverError>(())
        })();
        self.constraints = constraints;
        damping_result?;

        self.target_alpha = target_alpha;
        let _ = options.contact_iterations;
        let _ = options.quality;
        Ok(StepDiagnostics {
            generated_contacts,
            active_contacts,
            deepest_penetration,
            max_constraint_error,
            position_iterations: options.position_iterations,
        })
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

        state.predicted_pose.translation += translation;
        let delta_rotation = Quat::from_scaled_axis(rotation_scaled_axis);
        state.predicted_pose.rotation =
            (delta_rotation * state.predicted_pose.rotation).normalize();

        if kind == CorrectionKind::NonVelocityContact {
            state.contact_translation_correction += translation;
            state.contact_rotation_correction += rotation_scaled_axis;
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

    fn max_constraint_error(&self) -> Result<f32, SolverError> {
        let mut maximum: f32 = 0.0;
        for constraint in &self.constraints {
            maximum = maximum.max(constraint.error(self)?);
        }
        Ok(maximum)
    }

    fn generate_contacts(&mut self) -> Result<u32, SolverError> {
        let (previous, predicted) = self.collider_snapshots()?;
        let pipeline = CollisionPipeline::try_new(&self.constraints)?;
        let candidates = pipeline.candidate_pairs(&previous, &predicted)?;
        let mut generated = Vec::new();

        for pair in candidates {
            let collider_a = collider_by_id(&predicted, pair.first())?;
            let collider_b = collider_by_id(&predicted, pair.second())?;
            let cached_normal = cached_normal_for_pair(&self.cached_contacts, pair);
            let Some(manifold) = crate::capsule_manifold(collider_a, collider_b, cached_normal)?
            else {
                continue;
            };

            for geometry in manifold.points {
                let feature = capsule_contact_feature(collider_a, geometry.point_a);
                let key = ContactKey::new(pair, feature, feature);
                let previous = self
                    .cached_contacts
                    .binary_search_by_key(&key, |(cached_key, _)| *cached_key)
                    .ok()
                    .map(|index| self.cached_contacts[index].1);
                let (lambdas, age) = if let Some(previous) = previous {
                    (
                        previous.lambdas,
                        previous.age.checked_add(1).ok_or_else(|| {
                            SolverError::invalid_value("contact_age", previous.age)
                        })?,
                    )
                } else {
                    (ContactLambdas::default(), 1)
                };
                let contact = CachedContact {
                    local_a: local_point(collider_a.pose, geometry.point_a),
                    local_b: local_point(collider_b.pose, geometry.point_b),
                    normal: geometry.normal,
                    tangents: tangent_basis(geometry.normal),
                    lambdas,
                    age,
                };
                contact.validate()?;
                generated.push((key, contact));
            }
        }

        generated.sort_by_key(|(key, _)| *key);
        self.cached_contacts = generated;
        contact_count(self.cached_contacts.len())
    }

    fn warm_start_contacts(
        &mut self,
        colliders: &[CapsuleColliderDesc],
        contacts: &[(ContactKey, CachedContact)],
    ) -> Result<(), SolverError> {
        for (key, contact) in contacts {
            if contact.lambdas.normal > 0.0 {
                self.apply_contact_lambda(colliders, *key, contact, contact.lambdas.normal)?;
            }
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
                self.solve_contact(colliders, *key, contact)?;
            }
        } else {
            for (key, contact) in contacts.iter_mut().rev() {
                self.solve_contact(colliders, *key, contact)?;
            }
        }
        Ok(())
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

        let constraint_value = (point_b - point_a).dot(contact.normal);
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

    fn contact_residuals(&self) -> Result<(u32, f32), SolverError> {
        let colliders = self.predicted_colliders()?;
        let pipeline = CollisionPipeline::try_new(&self.constraints)?;
        let candidates = pipeline.candidate_pairs(&colliders, &colliders)?;
        let mut active = 0_usize;
        let mut deepest: f32 = 0.0;
        for pair in candidates {
            let collider_a = collider_by_id(&colliders, pair.first())?;
            let collider_b = collider_by_id(&colliders, pair.second())?;
            let cached_normal = cached_normal_for_pair(&self.cached_contacts, pair);
            if let Some(manifold) = crate::capsule_manifold(collider_a, collider_b, cached_normal)?
            {
                for point in manifold.points {
                    if point.penetration > 0.0 {
                        active = active.checked_add(1).ok_or_else(|| {
                            SolverError::invalid_value("contact_count", active as f64)
                        })?;
                    }
                    deepest = deepest.max(point.penetration);
                }
            }
        }
        Ok((contact_count(active)?, deepest))
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
            let previous = target.pose_at(self.target_alpha);
            let current = target.pose_at(target_alpha);
            let (translation, rotation) = crate::constraint::reference_frame_delta(
                previous,
                current,
                target.translation_inertia,
                target.rotation_inertia,
            );
            self.shift_reference_frame(id, previous, translation, rotation)?;
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

fn cached_normal_for_pair(contacts: &[(ContactKey, CachedContact)], pair: PairKey) -> Option<Vec3> {
    contacts
        .iter()
        .find(|(key, _)| key.pair == pair)
        .map(|(_, contact)| contact.normal)
}

fn local_point(pose: Pose, point: Vec3) -> Vec3 {
    pose.rotation.conjugate() * (point - pose.translation)
}

fn capsule_contact_feature(collider: &CapsuleCollider, surface_point: Vec3) -> u32 {
    let local_surface = local_point(collider.pose, surface_point);
    let axis = collider.shape.local_b - collider.shape.local_a;
    let axis_length_squared = axis.length_squared();
    if axis_length_squared <= MIN_CONTACT_DENOMINATOR {
        return 0;
    }
    let projection = (local_surface - collider.shape.local_a).dot(axis) / axis_length_squared;
    u32::from(projection > 0.5)
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
