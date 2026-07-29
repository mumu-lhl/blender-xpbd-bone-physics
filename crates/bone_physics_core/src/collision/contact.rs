use glam::Vec3;

use super::{Capsule, CollisionFilter, PairKey};
use crate::error::{validate_finite, validate_nonnegative};
use crate::pose::validate_vec3;
use crate::{BodyId, ColliderId, SolverError};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactMaterial {
    pub friction: f32,
    pub restitution: f32,
}

impl ContactMaterial {
    pub fn try_new(friction: f32, restitution: f32) -> Result<Self, SolverError> {
        let material = Self {
            friction,
            restitution,
        };
        material.validate()?;
        Ok(material)
    }

    pub(crate) fn validate(self) -> Result<(), SolverError> {
        validate_nonnegative("contact_friction", self.friction)?;
        validate_nonnegative("contact_restitution", self.restitution)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct CapsuleColliderDesc {
    pub id: ColliderId,
    pub body: BodyId,
    pub shape: Capsule,
    pub material: ContactMaterial,
    pub filter: CollisionFilter,
}

impl CapsuleColliderDesc {
    pub(crate) fn validate(&self) -> Result<(), SolverError> {
        self.shape.validate()?;
        self.material.validate()
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct ContactKey {
    pub pair: PairKey,
    pub feature_a: u32,
    pub feature_b: u32,
}

impl ContactKey {
    #[must_use]
    pub const fn new(pair: PairKey, feature_a: u32, feature_b: u32) -> Self {
        Self {
            pair,
            feature_a,
            feature_b,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactLambdas {
    pub normal: f32,
    pub tangents: [f32; 2],
}

impl ContactLambdas {
    pub(crate) fn validate(self) -> Result<(), SolverError> {
        validate_nonnegative("contact_normal_lambda", self.normal)?;
        validate_finite("contact_tangent_lambda_0", self.tangents[0])?;
        validate_finite("contact_tangent_lambda_1", self.tangents[1])
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CachedContact {
    pub local_a: Vec3,
    pub local_b: Vec3,
    pub normal: Vec3,
    pub tangents: [Vec3; 2],
    pub lambdas: ContactLambdas,
    pub age: u32,
}

impl CachedContact {
    pub(crate) fn validate(self) -> Result<(), SolverError> {
        validate_vec3("contact_local_a", self.local_a)?;
        validate_vec3("contact_local_b", self.local_b)?;
        validate_vec3("contact_normal", self.normal)?;
        validate_vec3("contact_tangent_0", self.tangents[0])?;
        validate_vec3("contact_tangent_1", self.tangents[1])?;
        self.lambdas.validate()
    }
}
