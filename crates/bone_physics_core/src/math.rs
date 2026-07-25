use glam::{Mat3, Quat, Vec3};

const MIN_QUATERNION_LENGTH_SQUARED: f32 = 1.0e-12;

#[must_use]
pub fn quat_log(rotation: Quat) -> Vec3 {
    let mut normalized = normalized_or_identity(rotation);
    if normalized.w < 0.0 {
        normalized = -normalized;
    }
    normalized.to_scaled_axis()
}

#[must_use]
pub fn quat_exp(scaled_axis: Vec3) -> Quat {
    if !scaled_axis.is_finite() {
        return Quat::IDENTITY;
    }
    normalized_or_identity(Quat::from_scaled_axis(scaled_axis))
}

#[must_use]
pub fn world_inverse_inertia(rotation: Quat, local_diagonal: Vec3) -> Mat3 {
    let basis = Mat3::from_quat(normalized_or_identity(rotation));
    basis * Mat3::from_diagonal(local_diagonal) * basis.transpose()
}

fn normalized_or_identity(rotation: Quat) -> Quat {
    if !rotation.is_finite()
        || rotation.length_squared() <= MIN_QUATERNION_LENGTH_SQUARED
    {
        Quat::IDENTITY
    } else {
        rotation.normalize()
    }
}
