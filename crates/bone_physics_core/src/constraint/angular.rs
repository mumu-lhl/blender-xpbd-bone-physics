use glam::{EulerRot, Quat, Vec3};

use crate::error::{validate_finite, validate_nonnegative};
use crate::pose::validate_vec3;
use crate::{
    quat_exp, quat_log, solve_scalar_row, BodyId, PersistentConstraint, Pose, SolverError,
    SolverWorld, XpbdGradient, XpbdRow,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EulerOrder {
    Xyz,
    Xzy,
    Yxz,
    Yzx,
    Zxy,
    Zyx,
}

impl EulerOrder {
    fn glam(self) -> EulerRot {
        match self {
            Self::Xyz => EulerRot::XYZ,
            Self::Xzy => EulerRot::XZY,
            Self::Yxz => EulerRot::YXZ,
            Self::Yzx => EulerRot::YZX,
            Self::Zxy => EulerRot::ZXY,
            Self::Zyx => EulerRot::ZYX,
        }
    }

    fn to_xyz(self, ordered: (f32, f32, f32)) -> Vec3 {
        let (a, b, c) = ordered;
        match self {
            Self::Xyz => Vec3::new(a, b, c),
            Self::Xzy => Vec3::new(a, c, b),
            Self::Yxz => Vec3::new(b, a, c),
            Self::Yzx => Vec3::new(c, a, b),
            Self::Zxy => Vec3::new(b, c, a),
            Self::Zyx => Vec3::new(c, b, a),
        }
    }

    fn to_quat(self, xyz: Vec3) -> Quat {
        let (a, b, c) = match self {
            Self::Xyz => (xyz.x, xyz.y, xyz.z),
            Self::Xzy => (xyz.x, xyz.z, xyz.y),
            Self::Yxz => (xyz.y, xyz.x, xyz.z),
            Self::Yzx => (xyz.y, xyz.z, xyz.x),
            Self::Zxy => (xyz.z, xyz.x, xyz.y),
            Self::Zyx => (xyz.z, xyz.y, xyz.x),
        };
        Quat::from_euler(self.glam(), a, b, c)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XyzLimit {
    pub lower: Vec3,
    pub upper: Vec3,
    pub order: EulerOrder,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwingTwistLimit {
    pub axis: Vec3,
    pub max_swing: f32,
    pub min_twist: f32,
    pub max_twist: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AngularLimit {
    XyzBox(XyzLimit),
    SwingTwist(SwingTwistLimit),
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContinuousAngles {
    value: Vec3,
    initialized: bool,
}

impl ContinuousAngles {
    pub fn unwrap(&mut self, wrapped: Vec3) -> Vec3 {
        if !self.initialized {
            self.value = wrapped;
            self.initialized = true;
            return wrapped;
        }

        self.value = Vec3::new(
            unwrap_angle(wrapped.x, self.value.x),
            unwrap_angle(wrapped.y, self.value.y),
            unwrap_angle(wrapped.z, self.value.z),
        );
        self.value
    }

    #[must_use]
    pub fn value(self) -> Option<Vec3> {
        self.initialized.then_some(self.value)
    }
}

pub fn project_angular_limit(
    candidate: Quat,
    limit: &AngularLimit,
    continuous: &mut ContinuousAngles,
) -> Result<Quat, SolverError> {
    validate_limit(limit)?;
    let candidate = Pose::try_new(Vec3::ZERO, candidate)?.rotation;
    let projected = match limit {
        AngularLimit::XyzBox(xyz) => {
            let wrapped = xyz.order.to_xyz(candidate.to_euler(xyz.order.glam()));
            let unwrapped = continuous.unwrap(wrapped);
            xyz.order.to_quat(unwrapped.clamp(xyz.lower, xyz.upper))
        }
        AngularLimit::SwingTwist(swing_twist) => project_swing_twist(candidate, *swing_twist),
    };
    Ok(projected.normalize())
}

pub(crate) fn validate_limit(limit: &AngularLimit) -> Result<(), SolverError> {
    match limit {
        AngularLimit::XyzBox(xyz) => {
            validate_vec3("angular_lower", xyz.lower)?;
            validate_vec3("angular_upper", xyz.upper)?;
            if xyz.lower.x > xyz.upper.x || xyz.lower.y > xyz.upper.y || xyz.lower.z > xyz.upper.z {
                return Err(SolverError::invalid_value(
                    "angular_range",
                    (xyz.upper - xyz.lower).min_element(),
                ));
            }
        }
        AngularLimit::SwingTwist(swing_twist) => {
            validate_vec3("swing_twist_axis", swing_twist.axis)?;
            validate_nonnegative("max_swing", swing_twist.max_swing)?;
            validate_finite("min_twist", swing_twist.min_twist)?;
            validate_finite("max_twist", swing_twist.max_twist)?;
            if swing_twist.axis.length_squared() <= 1.0e-12 {
                return Err(SolverError::invalid_value("swing_twist_axis", 0.0));
            }
            if swing_twist.max_swing > core::f32::consts::PI {
                return Err(SolverError::invalid_value(
                    "max_swing",
                    swing_twist.max_swing,
                ));
            }
            if swing_twist.min_twist > swing_twist.max_twist {
                return Err(SolverError::invalid_value(
                    "twist_range",
                    swing_twist.max_twist - swing_twist.min_twist,
                ));
            }
        }
    }
    Ok(())
}

pub(crate) fn solve_angular_constraint(
    constraint: &mut PersistentConstraint,
    world: &mut SolverWorld,
    rest_relative: Quat,
    limit: &AngularLimit,
    continuous: &mut ContinuousAngles,
    dt: f32,
) -> Result<(), SolverError> {
    let body_b = constraint
        .body_b
        .ok_or(SolverError::MissingBody(u32::MAX))?;
    for (index, local_axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        let rotation_a = body_rotation(world, constraint.body_a)?;
        let rotation_b = body_rotation(world, body_b)?;
        let relative = rotation_a.conjugate() * rotation_b;
        let candidate = rest_relative.conjugate() * relative;
        let projected = project_angular_limit(candidate, limit, continuous)?;
        let violation = quat_log(candidate * projected.conjugate());
        let world_axis = rotation_a * rest_relative * local_axis;
        let mut row = XpbdRow {
            c: violation.dot(local_axis),
            compliance: constraint.compliance,
            lambda: constraint.lambda[index],
            gradients: vec![
                XpbdGradient {
                    body: constraint.body_a,
                    linear: Vec3::ZERO,
                    angular: -world_axis,
                },
                XpbdGradient {
                    body: body_b,
                    linear: Vec3::ZERO,
                    angular: world_axis,
                },
            ],
        };
        solve_scalar_row(world, &mut row, dt)?;
        constraint.lambda[index] = row.lambda;
    }
    Ok(())
}

pub(crate) fn angular_error(
    constraint: &PersistentConstraint,
    world: &SolverWorld,
    rest_relative: Quat,
    limit: &AngularLimit,
    continuous: ContinuousAngles,
) -> Result<f32, SolverError> {
    let body_b = constraint
        .body_b
        .ok_or(SolverError::MissingBody(u32::MAX))?;
    let rotation_a = body_rotation(world, constraint.body_a)?;
    let rotation_b = body_rotation(world, body_b)?;
    let candidate = rest_relative.conjugate() * rotation_a.conjugate() * rotation_b;
    let mut continuous = continuous;
    let projected = project_angular_limit(candidate, limit, &mut continuous)?;
    Ok(candidate.angle_between(projected))
}

fn body_rotation(world: &SolverWorld, id: BodyId) -> Result<Quat, SolverError> {
    world
        .body(id)
        .map(|state| state.predicted_pose.rotation)
        .ok_or(SolverError::MissingBody(id.get()))
}

fn project_swing_twist(candidate: Quat, limit: SwingTwistLimit) -> Quat {
    let axis = limit.axis.normalize();
    let vector = Vec3::new(candidate.x, candidate.y, candidate.z);
    let projected_vector = axis * vector.dot(axis);
    let mut twist = Quat::from_xyzw(
        projected_vector.x,
        projected_vector.y,
        projected_vector.z,
        candidate.w,
    );
    if twist.length_squared() <= 1.0e-12 {
        twist = Quat::IDENTITY;
    } else {
        twist = twist.normalize();
    }
    if twist.w < 0.0 {
        twist = -twist;
    }

    let swing = (candidate * twist.conjugate()).normalize();
    let swing_axis_angle = quat_log(swing);
    let projected_swing = if swing_axis_angle.length() > limit.max_swing {
        quat_exp(swing_axis_angle.normalize() * limit.max_swing)
    } else {
        swing
    };

    let signed_twist = 2.0
        * Vec3::new(twist.x, twist.y, twist.z)
            .dot(axis)
            .atan2(twist.w);
    let projected_twist =
        Quat::from_axis_angle(axis, signed_twist.clamp(limit.min_twist, limit.max_twist));
    (projected_swing * projected_twist).normalize()
}

fn unwrap_angle(wrapped: f32, reference: f32) -> f32 {
    let turns = ((reference - wrapped) / core::f32::consts::TAU).round();
    wrapped + turns * core::f32::consts::TAU
}
