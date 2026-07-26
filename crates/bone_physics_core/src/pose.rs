use glam::{Quat, Vec3};

use crate::error::validate_finite;
use crate::SolverError;

const MIN_QUATERNION_LENGTH_SQUARED: f32 = 1.0e-12;
const NORMALIZED_TOLERANCE: f32 = 1.0e-5;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation: Vec3,
    pub rotation: Quat,
}

impl Pose {
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    };

    pub fn try_new(translation: Vec3, rotation: Quat) -> Result<Self, SolverError> {
        validate_vec3("translation", translation)?;
        validate_quaternion(rotation)?;

        Ok(Self {
            translation,
            rotation: rotation.normalize(),
        })
    }

    pub(crate) fn validate(self) -> Result<(), SolverError> {
        validate_vec3("translation", self.translation)?;
        validate_quaternion(self.rotation)?;
        let normalization_error = (self.rotation.length_squared() - 1.0).abs();
        if normalization_error <= NORMALIZED_TOLERANCE {
            Ok(())
        } else {
            Err(SolverError::invalid_value(
                "rotation_normalization",
                normalization_error,
            ))
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Velocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Velocity {
    pub const ZERO: Self = Self {
        linear: Vec3::ZERO,
        angular: Vec3::ZERO,
    };
}

pub(crate) fn validate_vec3(field: &'static str, value: Vec3) -> Result<(), SolverError> {
    validate_finite(field, value.x)?;
    validate_finite(field, value.y)?;
    validate_finite(field, value.z)
}

fn validate_quaternion(rotation: Quat) -> Result<(), SolverError> {
    if !rotation.is_finite() {
        return Err(SolverError::invalid_value("rotation", f32::NAN));
    }

    let length_squared = rotation.length_squared();
    if length_squared <= MIN_QUATERNION_LENGTH_SQUARED {
        return Err(SolverError::invalid_value(
            "rotation_length_squared",
            length_squared,
        ));
    }

    Ok(())
}
