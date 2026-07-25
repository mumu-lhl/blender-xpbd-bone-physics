use glam::{Quat, Vec3};

use crate::{
    quat_exp, quat_log, solve_scalar_row, ConstraintKind, PersistentConstraint, Pose, SolverError,
    SolverWorld, XpbdGradient, XpbdRow,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnimationTarget {
    pub previous: Pose,
    pub current: Pose,
    pub translation_inertia: f32,
    pub rotation_inertia: f32,
}

impl AnimationTarget {
    pub(crate) fn validate(self) -> Result<(), SolverError> {
        self.previous.validate()?;
        self.current.validate()?;
        if !self.translation_inertia.is_finite() {
            return Err(SolverError::invalid_value(
                "translation_inertia",
                self.translation_inertia,
            ));
        }
        if !self.rotation_inertia.is_finite() {
            return Err(SolverError::invalid_value(
                "rotation_inertia",
                self.rotation_inertia,
            ));
        }
        Ok(())
    }

    pub(crate) fn pose_at(self, alpha: f32) -> Pose {
        Pose {
            translation: self
                .previous
                .translation
                .lerp(self.current.translation, alpha),
            rotation: self.previous.rotation.slerp(self.current.rotation, alpha),
        }
    }
}

impl PersistentConstraint {
    pub(crate) fn solve_recovery(
        &mut self,
        world: &mut SolverWorld,
        target: Option<AnimationTarget>,
        target_alpha: f32,
        dt: f32,
    ) -> Result<(), SolverError> {
        if !self.enabled {
            return Ok(());
        }
        match self.kind.clone() {
            ConstraintKind::Attachment {
                local_anchor,
                target_local_anchor,
            } => solve_attachment(
                self,
                world,
                target.ok_or(SolverError::MissingBody(self.body_a.get()))?,
                target_alpha,
                local_anchor,
                target_local_anchor,
                dt,
            ),
            ConstraintKind::OrientationRecovery => solve_orientation(
                self,
                world,
                target.ok_or(SolverError::MissingBody(self.body_a.get()))?,
                target_alpha,
                dt,
            ),
            _ => Ok(()),
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn solve_attachment(
    constraint: &mut PersistentConstraint,
    world: &mut SolverWorld,
    target: AnimationTarget,
    target_alpha: f32,
    local_anchor: Vec3,
    target_local_anchor: Vec3,
    dt: f32,
) -> Result<(), SolverError> {
    let target_pose = target.pose_at(target_alpha);
    let target_point = target_pose.translation + target_pose.rotation * target_local_anchor;
    for (index, axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        let (point, offset) = world.anchor_geometry(constraint.body_a, local_anchor)?;
        let mut row = XpbdRow {
            c: (point - target_point).dot(axis),
            compliance: constraint.compliance,
            lambda: constraint.lambda[index],
            gradients: vec![XpbdGradient {
                body: constraint.body_a,
                linear: axis,
                angular: offset.cross(axis),
            }],
        };
        solve_scalar_row(world, &mut row, dt)?;
        constraint.lambda[index] = row.lambda;
    }
    Ok(())
}

fn solve_orientation(
    constraint: &mut PersistentConstraint,
    world: &mut SolverWorld,
    target: AnimationTarget,
    target_alpha: f32,
    dt: f32,
) -> Result<(), SolverError> {
    let target_pose = target.pose_at(target_alpha);
    for (index, axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        let current = world
            .body(constraint.body_a)
            .ok_or(SolverError::MissingBody(constraint.body_a.get()))?
            .predicted_pose
            .rotation;
        let error = quat_log(current * target_pose.rotation.conjugate());
        let mut row = XpbdRow {
            c: error.dot(axis),
            compliance: constraint.compliance,
            lambda: constraint.lambda[index],
            gradients: vec![XpbdGradient {
                body: constraint.body_a,
                linear: Vec3::ZERO,
                angular: axis,
            }],
        };
        solve_scalar_row(world, &mut row, dt)?;
        constraint.lambda[index] = row.lambda;
    }
    Ok(())
}

pub(crate) fn reference_frame_delta(
    previous: Pose,
    current: Pose,
    translation_inertia: f32,
    rotation_inertia: f32,
) -> (Vec3, Quat) {
    let translation = (current.translation - previous.translation) * (1.0 - translation_inertia);
    let full_rotation = current.rotation * previous.rotation.conjugate();
    let rotation = quat_exp(quat_log(full_rotation) * (1.0 - rotation_inertia));
    (translation, rotation)
}
