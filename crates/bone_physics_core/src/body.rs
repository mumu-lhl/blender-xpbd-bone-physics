use glam::Vec3;

use crate::error::{validate_finite, validate_nonnegative};
use crate::pose::validate_vec3;
use crate::{BodyId, Pose, SolverError, Velocity};

const MIN_BONE_LENGTH: f32 = 1.0e-6;

#[derive(Clone, Debug, PartialEq)]
pub struct BoneBodyDesc {
    pub id: BodyId,
    pub rest_pose: Pose,
    pub local_center_of_mass: Vec3,
    pub local_head: Vec3,
    pub local_tail: Vec3,
    pub inverse_mass: f32,
    pub inverse_inertia: Vec3,
    pub gravity_scale: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

impl BoneBodyDesc {
    pub fn validate(&self) -> Result<(), SolverError> {
        self.rest_pose.validate()?;
        validate_vec3("local_center_of_mass", self.local_center_of_mass)?;
        validate_vec3("local_head", self.local_head)?;
        validate_vec3("local_tail", self.local_tail)?;
        validate_nonnegative("inverse_mass", self.inverse_mass)?;
        validate_vec3("inverse_inertia", self.inverse_inertia)?;
        validate_nonnegative("inverse_inertia_x", self.inverse_inertia.x)?;
        validate_nonnegative("inverse_inertia_y", self.inverse_inertia.y)?;
        validate_nonnegative("inverse_inertia_z", self.inverse_inertia.z)?;
        validate_finite("gravity_scale", self.gravity_scale)?;
        validate_nonnegative("linear_damping", self.linear_damping)?;
        validate_nonnegative("angular_damping", self.angular_damping)?;

        let length = self.local_head.distance(self.local_tail);
        if length < MIN_BONE_LENGTH {
            return Err(SolverError::invalid_value("bone_length", length));
        }

        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct BoneBodyState {
    pub pose: Pose,
    pub previous_pose: Pose,
    pub predicted_pose: Pose,
    pub(crate) integrated_pose: Pose,
    pub velocity: Velocity,
    pub contact_translation_correction: Vec3,
    pub contact_rotation_correction: Vec3,
}

impl BoneBodyState {
    pub fn from_desc(desc: &BoneBodyDesc, initial_pose: Pose) -> Result<Self, SolverError> {
        desc.validate()?;
        initial_pose.validate()?;

        Ok(Self {
            pose: initial_pose,
            previous_pose: initial_pose,
            predicted_pose: initial_pose,
            integrated_pose: initial_pose,
            velocity: Velocity::ZERO,
            contact_translation_correction: Vec3::ZERO,
            contact_rotation_correction: Vec3::ZERO,
        })
    }
}
