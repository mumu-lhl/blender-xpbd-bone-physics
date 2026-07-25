use glam::{Mat3, Quat, Vec3};

use crate::error::{validate_finite, validate_nonnegative};
use crate::math::world_inverse_inertia;
use crate::pose::validate_vec3;
use crate::{AnimationTarget, ConstraintId, PersistentConstraint, QualityPolicy};
use crate::{BodyId, BoneBodyDesc, BoneBodyState, Pose, SolverError, Velocity};

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
        self.constraints = constraints;
        Ok(())
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

        let mut constraints = core::mem::take(&mut self.constraints);
        let solve_result = (|| {
            for iteration in 0..options.position_iterations {
                if iteration % 2 == 0 {
                    for constraint in &mut constraints {
                        self.solve_constraint(constraint, target_alpha, dt)?;
                    }
                } else {
                    for constraint in constraints.iter_mut().rev() {
                        self.solve_constraint(constraint, target_alpha, dt)?;
                    }
                }
            }
            Ok::<(), SolverError>(())
        })();
        self.constraints = constraints;
        solve_result?;

        let max_constraint_error = self.max_constraint_error()?;
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

fn validate_dt(dt: f32) -> Result<(), SolverError> {
    validate_nonnegative("dt", dt)?;
    if dt == 0.0 {
        Err(SolverError::invalid_value("dt", dt))
    } else {
        Ok(())
    }
}
