use glam::Vec3;

use crate::error::validate_nonnegative;
use crate::pose::validate_vec3;
use crate::{
    solve_scalar_row, BodyId, ConstraintId, SolverError, SolverWorld, XpbdGradient, XpbdRow,
};

#[derive(Clone, Debug, PartialEq)]
pub enum ConstraintKind {
    Anchor {
        local_a: Vec3,
        local_b: Vec3,
    },
    Distance {
        local_a: Vec3,
        local_b: Vec3,
        rest_length: f32,
    },
    DistanceRange {
        local_a: Vec3,
        local_b: Vec3,
        min: f32,
        max: f32,
    },
    Attachment {
        local_anchor: Vec3,
        target_local_anchor: Vec3,
    },
    OrientationRecovery,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PersistentConstraint {
    pub id: ConstraintId,
    pub enabled: bool,
    pub body_a: BodyId,
    pub body_b: Option<BodyId>,
    pub compliance: f32,
    pub damping: f32,
    pub lambda: Vec3,
    pub kind: ConstraintKind,
}

impl PersistentConstraint {
    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn anchor(
        id: ConstraintId,
        body_a: BodyId,
        body_b: BodyId,
        local_a: Vec3,
        local_b: Vec3,
        compliance: f32,
        damping: f32,
    ) -> Self {
        Self::two_body(
            id,
            body_a,
            body_b,
            compliance,
            damping,
            ConstraintKind::Anchor { local_a, local_b },
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn distance(
        id: ConstraintId,
        body_a: BodyId,
        body_b: BodyId,
        local_a: Vec3,
        local_b: Vec3,
        rest_length: f32,
        compliance: f32,
        damping: f32,
    ) -> Self {
        Self::two_body(
            id,
            body_a,
            body_b,
            compliance,
            damping,
            ConstraintKind::Distance {
                local_a,
                local_b,
                rest_length,
            },
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn distance_range(
        id: ConstraintId,
        body_a: BodyId,
        body_b: BodyId,
        local_a: Vec3,
        local_b: Vec3,
        min: f32,
        max: f32,
        compliance: f32,
        damping: f32,
    ) -> Self {
        Self::two_body(
            id,
            body_a,
            body_b,
            compliance,
            damping,
            ConstraintKind::DistanceRange {
                local_a,
                local_b,
                min,
                max,
            },
        )
    }

    #[must_use]
    pub fn attachment(
        id: ConstraintId,
        body: BodyId,
        local_anchor: Vec3,
        target_local_anchor: Vec3,
        compliance: f32,
        damping: f32,
    ) -> Self {
        Self {
            id,
            enabled: true,
            body_a: body,
            body_b: None,
            compliance,
            damping,
            lambda: Vec3::ZERO,
            kind: ConstraintKind::Attachment {
                local_anchor,
                target_local_anchor,
            },
        }
    }

    #[must_use]
    pub fn orientation_recovery(
        id: ConstraintId,
        body: BodyId,
        compliance: f32,
        damping: f32,
    ) -> Self {
        Self {
            id,
            enabled: true,
            body_a: body,
            body_b: None,
            compliance,
            damping,
            lambda: Vec3::ZERO,
            kind: ConstraintKind::OrientationRecovery,
        }
    }

    fn two_body(
        id: ConstraintId,
        body_a: BodyId,
        body_b: BodyId,
        compliance: f32,
        damping: f32,
        kind: ConstraintKind,
    ) -> Self {
        Self {
            id,
            enabled: true,
            body_a,
            body_b: Some(body_b),
            compliance,
            damping,
            lambda: Vec3::ZERO,
            kind,
        }
    }

    pub(crate) fn validate(&self) -> Result<(), SolverError> {
        validate_nonnegative("constraint_compliance", self.compliance)?;
        validate_nonnegative("constraint_damping", self.damping)?;
        match &self.kind {
            ConstraintKind::Anchor { local_a, local_b }
            | ConstraintKind::Distance {
                local_a, local_b, ..
            }
            | ConstraintKind::DistanceRange {
                local_a, local_b, ..
            } => {
                validate_vec3("constraint_local_a", *local_a)?;
                validate_vec3("constraint_local_b", *local_b)?;
            }
            ConstraintKind::Attachment {
                local_anchor,
                target_local_anchor,
            } => {
                validate_vec3("constraint_local_anchor", *local_anchor)?;
                validate_vec3("constraint_target_local_anchor", *target_local_anchor)?;
            }
            ConstraintKind::OrientationRecovery => {}
        }
        if let ConstraintKind::Distance { rest_length, .. } = &self.kind {
            validate_nonnegative("rest_length", *rest_length)?;
        }
        if let ConstraintKind::DistanceRange { min, max, .. } = &self.kind {
            validate_nonnegative("distance_min", *min)?;
            validate_nonnegative("distance_max", *max)?;
            if max < min {
                return Err(SolverError::invalid_value("distance_range", *max - *min));
            }
        }
        Ok(())
    }

    pub(crate) fn solve_joint(
        &mut self,
        world: &mut SolverWorld,
        dt: f32,
    ) -> Result<(), SolverError> {
        if !self.enabled {
            return Ok(());
        }
        match self.kind.clone() {
            ConstraintKind::Anchor { local_a, local_b } => {
                solve_anchor(self, world, local_a, local_b, dt)
            }
            ConstraintKind::Distance {
                local_a,
                local_b,
                rest_length,
            } => solve_distance(self, world, local_a, local_b, rest_length, dt),
            ConstraintKind::DistanceRange {
                local_a,
                local_b,
                min,
                max,
            } => solve_distance_range(self, world, local_a, local_b, min, max, dt),
            ConstraintKind::Attachment { .. } | ConstraintKind::OrientationRecovery => Ok(()),
        }
    }

    pub(crate) fn error(&self, world: &SolverWorld) -> Result<f32, SolverError> {
        match &self.kind {
            ConstraintKind::Anchor { local_a, local_b } => {
                let (point_a, _) = world.anchor_geometry(self.body_a, *local_a)?;
                let body_b = required_body_b(self)?;
                let (point_b, _) = world.anchor_geometry(body_b, *local_b)?;
                Ok(point_a.distance(point_b))
            }
            ConstraintKind::Distance {
                local_a,
                local_b,
                rest_length,
            } => {
                let (point_a, _) = world.anchor_geometry(self.body_a, *local_a)?;
                let body_b = required_body_b(self)?;
                let (point_b, _) = world.anchor_geometry(body_b, *local_b)?;
                Ok((point_a.distance(point_b) - *rest_length).abs())
            }
            ConstraintKind::DistanceRange {
                local_a,
                local_b,
                min,
                max,
            } => {
                let (point_a, _) = world.anchor_geometry(self.body_a, *local_a)?;
                let body_b = required_body_b(self)?;
                let (point_b, _) = world.anchor_geometry(body_b, *local_b)?;
                let distance = point_a.distance(point_b);
                Ok((*min - distance).max(0.0).max((distance - *max).max(0.0)))
            }
            ConstraintKind::Attachment { .. } | ConstraintKind::OrientationRecovery => Ok(0.0),
        }
    }

    pub(crate) fn apply_damping(
        &self,
        world: &mut SolverWorld,
        dt: f32,
    ) -> Result<(), SolverError> {
        if !self.enabled || self.damping == 0.0 {
            return Ok(());
        }
        match self.kind {
            ConstraintKind::Anchor { local_a, local_b } => {
                damp_anchor(self, world, local_a, local_b, dt)
            }
            ConstraintKind::Distance {
                local_a, local_b, ..
            }
            | ConstraintKind::DistanceRange {
                local_a, local_b, ..
            } => damp_distance(self, world, local_a, local_b, dt),
            ConstraintKind::Attachment { .. } | ConstraintKind::OrientationRecovery => Ok(()),
        }
    }
}

fn damp_anchor(
    constraint: &PersistentConstraint,
    world: &mut SolverWorld,
    local_a: Vec3,
    local_b: Vec3,
    dt: f32,
) -> Result<(), SolverError> {
    let body_b = required_body_b(constraint)?;
    for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let (_, offset_a) = world.anchor_geometry(constraint.body_a, local_a)?;
        let (_, offset_b) = world.anchor_geometry(body_b, local_b)?;
        damp_velocity_row(
            constraint,
            world,
            dt,
            &[
                XpbdGradient {
                    body: constraint.body_a,
                    linear: axis,
                    angular: offset_a.cross(axis),
                },
                XpbdGradient {
                    body: body_b,
                    linear: -axis,
                    angular: -offset_b.cross(axis),
                },
            ],
        )?;
    }
    Ok(())
}

fn damp_distance(
    constraint: &PersistentConstraint,
    world: &mut SolverWorld,
    local_a: Vec3,
    local_b: Vec3,
    dt: f32,
) -> Result<(), SolverError> {
    let body_b = required_body_b(constraint)?;
    let (point_a, _) = world.anchor_geometry(constraint.body_a, local_a)?;
    let (point_b, _) = world.anchor_geometry(body_b, local_b)?;
    let normal = normal_or_x(point_a - point_b);
    let (_, offset_a) = world.anchor_geometry(constraint.body_a, local_a)?;
    let (_, offset_b) = world.anchor_geometry(body_b, local_b)?;
    damp_velocity_row(
        constraint,
        world,
        dt,
        &[
            XpbdGradient {
                body: constraint.body_a,
                linear: normal,
                angular: offset_a.cross(normal),
            },
            XpbdGradient {
                body: body_b,
                linear: -normal,
                angular: -offset_b.cross(normal),
            },
        ],
    )
}

fn damp_velocity_row(
    constraint: &PersistentConstraint,
    world: &mut SolverWorld,
    dt: f32,
    gradients: &[XpbdGradient],
) -> Result<(), SolverError> {
    let mut relative_velocity = 0.0;
    let mut effective_inverse_mass = 0.0;
    for gradient in gradients {
        let state = world
            .body(gradient.body)
            .ok_or(SolverError::MissingBody(gradient.body.get()))?;
        relative_velocity += gradient.linear.dot(state.velocity.linear);
        relative_velocity += gradient.angular.dot(state.velocity.angular);
        let properties = world.solve_properties(gradient.body)?;
        effective_inverse_mass += properties.inverse_mass * gradient.linear.length_squared();
        effective_inverse_mass += gradient
            .angular
            .dot(properties.world_inverse_inertia * gradient.angular);
    }
    if effective_inverse_mass <= 1.0e-12 {
        return Ok(());
    }

    let retained_velocity = (-constraint.damping * dt).exp();
    let impulse = (retained_velocity - 1.0) * relative_velocity / effective_inverse_mass;
    for gradient in gradients {
        world.apply_velocity_impulse(
            gradient.body,
            gradient.linear,
            gradient.angular,
            impulse,
        )?;
    }
    Ok(())
}

fn solve_anchor(
    constraint: &mut PersistentConstraint,
    world: &mut SolverWorld,
    local_a: Vec3,
    local_b: Vec3,
    dt: f32,
) -> Result<(), SolverError> {
    let body_b = required_body_b(constraint)?;
    for (index, axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        let (point_a, offset_a) = world.anchor_geometry(constraint.body_a, local_a)?;
        let (point_b, offset_b) = world.anchor_geometry(body_b, local_b)?;
        let mut row = XpbdRow {
            c: (point_a - point_b).dot(axis),
            compliance: constraint.compliance,
            lambda: constraint.lambda[index],
            gradients: vec![
                XpbdGradient {
                    body: constraint.body_a,
                    linear: axis,
                    angular: offset_a.cross(axis),
                },
                XpbdGradient {
                    body: body_b,
                    linear: -axis,
                    angular: -offset_b.cross(axis),
                },
            ],
        };
        solve_scalar_row(world, &mut row, dt)?;
        constraint.lambda[index] = row.lambda;
    }
    Ok(())
}

fn solve_distance(
    constraint: &mut PersistentConstraint,
    world: &mut SolverWorld,
    local_a: Vec3,
    local_b: Vec3,
    rest_length: f32,
    dt: f32,
) -> Result<(), SolverError> {
    let body_b = required_body_b(constraint)?;
    let (point_a, offset_a) = world.anchor_geometry(constraint.body_a, local_a)?;
    let (point_b, offset_b) = world.anchor_geometry(body_b, local_b)?;
    let delta = point_a - point_b;
    let distance = delta.length();
    let normal = normal_or_x(delta);
    let mut row = distance_row(
        constraint,
        body_b,
        offset_a,
        offset_b,
        normal,
        distance - rest_length,
        constraint.lambda.x,
    );
    solve_scalar_row(world, &mut row, dt)?;
    constraint.lambda.x = row.lambda;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn solve_distance_range(
    constraint: &mut PersistentConstraint,
    world: &mut SolverWorld,
    local_a: Vec3,
    local_b: Vec3,
    min: f32,
    max: f32,
    dt: f32,
) -> Result<(), SolverError> {
    let body_b = required_body_b(constraint)?;
    let (point_a, offset_a) = world.anchor_geometry(constraint.body_a, local_a)?;
    let (point_b, offset_b) = world.anchor_geometry(body_b, local_b)?;
    let delta = point_a - point_b;
    let distance = delta.length();
    let normal = normal_or_x(delta);

    if distance < min {
        let mut row = distance_row(
            constraint,
            body_b,
            offset_a,
            offset_b,
            normal,
            distance - min,
            constraint.lambda.x,
        );
        solve_scalar_row(world, &mut row, dt)?;
        constraint.lambda.x = row.lambda.max(0.0);
        constraint.lambda.y = 0.0;
    } else if distance > max {
        let mut row = distance_row(
            constraint,
            body_b,
            offset_a,
            offset_b,
            -normal,
            max - distance,
            constraint.lambda.y,
        );
        solve_scalar_row(world, &mut row, dt)?;
        constraint.lambda.x = 0.0;
        constraint.lambda.y = row.lambda.max(0.0);
    } else {
        constraint.lambda.x = 0.0;
        constraint.lambda.y = 0.0;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn distance_row(
    constraint: &PersistentConstraint,
    body_b: BodyId,
    offset_a: Vec3,
    offset_b: Vec3,
    normal: Vec3,
    c: f32,
    lambda: f32,
) -> XpbdRow {
    XpbdRow {
        c,
        compliance: constraint.compliance,
        lambda,
        gradients: vec![
            XpbdGradient {
                body: constraint.body_a,
                linear: normal,
                angular: offset_a.cross(normal),
            },
            XpbdGradient {
                body: body_b,
                linear: -normal,
                angular: -offset_b.cross(normal),
            },
        ],
    }
}

fn required_body_b(constraint: &PersistentConstraint) -> Result<BodyId, SolverError> {
    constraint.body_b.ok_or(SolverError::MissingBody(u32::MAX))
}

fn normal_or_x(delta: Vec3) -> Vec3 {
    if delta.length_squared() > 1.0e-12 {
        delta.normalize()
    } else {
        Vec3::X
    }
}
