use super::shape::CapsuleCollider;
use crate::{BodyId, ConstraintId, ConstraintKind, PersistentConstraint, SolverError};

#[derive(Clone, Copy, Debug)]
struct ConstraintEdge {
    id: ConstraintId,
    body_a: BodyId,
    body_b: BodyId,
    hard_joint: bool,
}

impl ConstraintEdge {
    fn connects(self, a: BodyId, b: BodyId) -> bool {
        (self.body_a == a && self.body_b == b) || (self.body_a == b && self.body_b == a)
    }
}

#[derive(Clone, Debug, Default)]
pub struct CollisionPipeline {
    constraint_edges: Vec<ConstraintEdge>,
}

impl CollisionPipeline {
    pub fn try_new(constraints: &[PersistentConstraint]) -> Result<Self, SolverError> {
        let mut sorted = constraints.iter().collect::<Vec<_>>();
        sorted.sort_by_key(|constraint| constraint.id);
        for pair in sorted.windows(2) {
            if pair[0].id == pair[1].id {
                return Err(SolverError::DuplicateId {
                    kind: "constraint",
                    id: pair[0].id.get(),
                });
            }
        }

        let mut constraint_edges = Vec::new();
        for constraint in sorted {
            constraint.validate()?;
            if let Some(body_b) = constraint.body_b {
                constraint_edges.push(ConstraintEdge {
                    id: constraint.id,
                    body_a: constraint.body_a,
                    body_b,
                    hard_joint: constraint.enabled
                        && constraint.compliance == 0.0
                        && matches!(&constraint.kind, ConstraintKind::Anchor { .. }),
                });
            }
        }
        Ok(Self { constraint_edges })
    }

    pub fn allows_pair(
        &self,
        collider_a: &CapsuleCollider,
        collider_b: &CapsuleCollider,
    ) -> Result<bool, SolverError> {
        collider_a.validate()?;
        collider_b.validate()?;
        if collider_a.id == collider_b.id || collider_a.body == collider_b.body {
            return Ok(false);
        }
        if collider_a.filter.layer & collider_b.filter.mask == 0
            || collider_b.filter.layer & collider_a.filter.mask == 0
        {
            return Ok(false);
        }
        if collider_a
            .filter
            .excluded_bodies
            .contains(&collider_b.body)
            || collider_b
                .filter
                .excluded_bodies
                .contains(&collider_a.body)
        {
            return Ok(false);
        }

        for edge in &self.constraint_edges {
            if !edge.connects(collider_a.body, collider_b.body) {
                continue;
            }
            if edge.hard_joint
                || collider_a
                    .filter
                    .excluded_constraints
                    .contains(&edge.id)
                || collider_b
                    .filter
                    .excluded_constraints
                    .contains(&edge.id)
            {
                return Ok(false);
            }
        }
        Ok(true)
    }
}
