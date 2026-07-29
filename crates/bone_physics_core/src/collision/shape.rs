use glam::Vec3;

use super::filter::CollisionFilter;
use crate::error::validate_nonnegative;
use crate::pose::validate_vec3;
use crate::{BodyId, ColliderId, Pose, SolverError};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Capsule {
    pub local_a: Vec3,
    pub local_b: Vec3,
    pub radius: f32,
    pub margin: f32,
}

impl Capsule {
    pub fn try_new(
        local_a: Vec3,
        local_b: Vec3,
        radius: f32,
        margin: f32,
    ) -> Result<Self, SolverError> {
        let capsule = Self {
            local_a,
            local_b,
            radius,
            margin,
        };
        capsule.validate()?;
        Ok(capsule)
    }

    pub(crate) fn validate(self) -> Result<(), SolverError> {
        validate_vec3("capsule_local_a", self.local_a)?;
        validate_vec3("capsule_local_b", self.local_b)?;
        validate_nonnegative("capsule_radius", self.radius)?;
        validate_nonnegative("capsule_margin", self.margin)?;
        if self.radius == 0.0 {
            return Err(SolverError::invalid_value("capsule_radius", self.radius));
        }
        Ok(())
    }

    pub(crate) fn effective_radius(self) -> Result<f32, SolverError> {
        let radius = self.radius + self.margin;
        validate_nonnegative("capsule_effective_radius", radius)?;
        Ok(radius)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CapsuleCollider {
    pub id: ColliderId,
    pub body: BodyId,
    pub shape: Capsule,
    pub pose: Pose,
    pub linear_velocity: Vec3,
    pub filter: CollisionFilter,
}

impl CapsuleCollider {
    pub(crate) fn validate(&self) -> Result<(), SolverError> {
        self.shape.validate()?;
        self.pose.validate()?;
        validate_vec3("collider_linear_velocity", self.linear_velocity)
    }

    pub(crate) fn world_segment(&self) -> Result<(Vec3, Vec3), SolverError> {
        let a = self.pose.translation + self.pose.rotation * self.shape.local_a;
        let b = self.pose.translation + self.pose.rotation * self.shape.local_b;
        validate_vec3("capsule_world_a", a)?;
        validate_vec3("capsule_world_b", b)?;
        Ok((a, b))
    }
}
