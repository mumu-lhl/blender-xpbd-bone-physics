use glam::Vec3;

use crate::error::{validate_finite, validate_nonnegative};
use crate::pose::validate_vec3;
use crate::{
    BodyId, CorrectionKind, SolverError, SolverWorld,
};

const MIN_EFFECTIVE_DENOMINATOR: f32 = 1.0e-12;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XpbdGradient {
    pub body: BodyId,
    pub linear: Vec3,
    pub angular: Vec3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct XpbdRow {
    pub c: f32,
    pub compliance: f32,
    pub lambda: f32,
    pub gradients: Vec<XpbdGradient>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RowResult {
    pub delta_lambda: f32,
    pub applied: bool,
}

pub fn solve_scalar_row(
    world: &mut SolverWorld,
    row: &mut XpbdRow,
    dt: f32,
) -> Result<RowResult, SolverError> {
    validate_finite("constraint_value", row.c)?;
    validate_nonnegative("compliance", row.compliance)?;
    validate_finite("lambda", row.lambda)?;
    validate_finite("dt", dt)?;
    if dt <= 0.0 {
        return Err(SolverError::invalid_value("dt", dt));
    }

    let mut effective_inverse_mass = 0.0;
    for gradient in &row.gradients {
        validate_vec3("linear_gradient", gradient.linear)?;
        validate_vec3("angular_gradient", gradient.angular)?;
        let properties = world.solve_properties(gradient.body)?;
        effective_inverse_mass +=
            properties.inverse_mass * gradient.linear.length_squared();
        effective_inverse_mass += gradient
            .angular
            .dot(properties.world_inverse_inertia * gradient.angular);
    }

    let alpha_tilde = row.compliance / (dt * dt);
    let denominator = effective_inverse_mass + alpha_tilde;
    if denominator <= MIN_EFFECTIVE_DENOMINATOR {
        return Ok(RowResult {
            delta_lambda: 0.0,
            applied: false,
        });
    }

    let delta_lambda =
        (-row.c - alpha_tilde * row.lambda) / denominator;
    validate_finite("delta_lambda", delta_lambda)?;

    for gradient in &row.gradients {
        let properties = world.solve_properties(gradient.body)?;
        let translation =
            gradient.linear * properties.inverse_mass * delta_lambda;
        let rotation = properties.world_inverse_inertia
            * gradient.angular
            * delta_lambda;
        world.apply_pose_correction(
            gradient.body,
            translation,
            rotation,
            CorrectionKind::Constraint,
        )?;
    }
    row.lambda += delta_lambda;

    Ok(RowResult {
        delta_lambda,
        applied: true,
    })
}
