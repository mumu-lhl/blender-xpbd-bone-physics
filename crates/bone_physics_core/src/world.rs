use glam::{Mat3, Quat, Vec3};

use crate::error::{validate_finite, validate_nonnegative};
use crate::math::world_inverse_inertia;
use crate::pose::validate_vec3;
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

    fn body_mut(&mut self, id: BodyId) -> Result<&mut BoneBodyState, SolverError> {
        let index = self
            .bodies
            .binary_search_by_key(&id, |entry| entry.desc.id)
            .map_err(|_| SolverError::MissingBody(id.get()))?;
        Ok(&mut self.bodies[index].state)
    }

    pub(crate) fn solve_properties(
        &self,
        id: BodyId,
    ) -> Result<SolveProperties, SolverError> {
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
}

fn validate_dt(dt: f32) -> Result<(), SolverError> {
    validate_nonnegative("dt", dt)?;
    if dt == 0.0 {
        Err(SolverError::invalid_value("dt", dt))
    } else {
        Ok(())
    }
}
