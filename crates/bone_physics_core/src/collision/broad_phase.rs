use std::collections::BTreeSet;

use glam::Vec3;
use parry3d::bounding_volume::Aabb as ParryAabb;
use parry3d::math::Vector as ParryVector;
use parry3d::partitioning::Qbvh;

use super::capsule::PairKey;
use super::shape::CapsuleCollider;
use crate::pose::validate_vec3;
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

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum BroadPhaseBackend {
    SweepAndPrune,
    #[default]
    ParryBvh,
}

#[derive(Clone, Copy, Debug)]
struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    fn from_collider(collider: &CapsuleCollider) -> Result<Self, SolverError> {
        let (a, b) = collider.world_segment()?;
        let radius = Vec3::splat(collider.shape.effective_radius()?);
        let min = a.min(b) - radius;
        let max = a.max(b) + radius;
        validate_vec3("collider_aabb_min", min)?;
        validate_vec3("collider_aabb_max", max)?;
        Ok(Self { min, max })
    }

    fn merged(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    fn intersects(self, other: Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    fn parry(self) -> ParryAabb {
        ParryAabb::new(
            ParryVector::new(self.min.x, self.min.y, self.min.z),
            ParryVector::new(self.max.x, self.max.y, self.max.z),
        )
    }
}

#[derive(Clone, Copy)]
struct SweptEntry<'a> {
    collider: &'a CapsuleCollider,
    bounds: Aabb,
}

#[derive(Clone, Debug)]
pub struct CollisionPipeline {
    constraint_edges: Vec<ConstraintEdge>,
    backend: BroadPhaseBackend,
}

impl CollisionPipeline {
    pub fn try_new(constraints: &[PersistentConstraint]) -> Result<Self, SolverError> {
        Self::try_new_with_backend(constraints, BroadPhaseBackend::default())
    }

    pub fn try_new_with_backend(
        constraints: &[PersistentConstraint],
        backend: BroadPhaseBackend,
    ) -> Result<Self, SolverError> {
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
        Ok(Self {
            constraint_edges,
            backend,
        })
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
        if collider_a.filter.excluded_bodies.contains(&collider_b.body)
            || collider_b.filter.excluded_bodies.contains(&collider_a.body)
        {
            return Ok(false);
        }

        for edge in &self.constraint_edges {
            if !edge.connects(collider_a.body, collider_b.body) {
                continue;
            }
            if edge.hard_joint
                || collider_a.filter.excluded_constraints.contains(&edge.id)
                || collider_b.filter.excluded_constraints.contains(&edge.id)
            {
                return Ok(false);
            }
        }
        Ok(true)
    }

    pub fn candidate_pairs(
        &self,
        previous: &[CapsuleCollider],
        predicted: &[CapsuleCollider],
    ) -> Result<Vec<PairKey>, SolverError> {
        let entries = swept_entries(previous, predicted)?;
        let candidates = match self.backend {
            BroadPhaseBackend::SweepAndPrune => sweep_candidates(&entries),
            BroadPhaseBackend::ParryBvh => parry_candidates(&entries),
        };

        let mut result = BTreeSet::new();
        for (index_a, index_b) in candidates {
            let a = entries[index_a].collider;
            let b = entries[index_b].collider;
            if self.allows_pair(a, b)? {
                result.insert(PairKey::ordered(a.id, b.id)?);
            }
        }
        Ok(result.into_iter().collect())
    }
}

fn swept_entries<'a>(
    previous: &'a [CapsuleCollider],
    predicted: &'a [CapsuleCollider],
) -> Result<Vec<SweptEntry<'a>>, SolverError> {
    let mut previous = previous.iter().collect::<Vec<_>>();
    let mut predicted = predicted.iter().collect::<Vec<_>>();
    previous.sort_by_key(|collider| collider.id);
    predicted.sort_by_key(|collider| collider.id);
    validate_snapshot_ids(&previous)?;
    validate_snapshot_ids(&predicted)?;
    if previous.len() != predicted.len() {
        return Err(SolverError::invalid_value(
            "collider_snapshot_count",
            predicted.len() as f64 - previous.len() as f64,
        ));
    }

    let mut entries = Vec::with_capacity(predicted.len());
    for (previous, predicted) in previous.into_iter().zip(predicted) {
        previous.validate()?;
        predicted.validate()?;
        if previous.id != predicted.id
            || previous.body != predicted.body
            || previous.shape != predicted.shape
            || previous.filter != predicted.filter
        {
            return Err(SolverError::invalid_value(
                "collider_snapshot_metadata",
                predicted.id.get(),
            ));
        }
        entries.push(SweptEntry {
            collider: predicted,
            bounds: Aabb::from_collider(previous)?.merged(Aabb::from_collider(predicted)?),
        });
    }
    Ok(entries)
}

fn validate_snapshot_ids(colliders: &[&CapsuleCollider]) -> Result<(), SolverError> {
    for pair in colliders.windows(2) {
        if pair[0].id == pair[1].id {
            return Err(SolverError::DuplicateId {
                kind: "collider",
                id: pair[0].id.get(),
            });
        }
    }
    Ok(())
}

fn sweep_candidates(entries: &[SweptEntry<'_>]) -> Vec<(usize, usize)> {
    let mut order = (0..entries.len()).collect::<Vec<_>>();
    order.sort_by(|a, b| {
        entries[*a]
            .bounds
            .min
            .x
            .total_cmp(&entries[*b].bounds.min.x)
            .then_with(|| entries[*a].collider.id.cmp(&entries[*b].collider.id))
    });

    let mut active: Vec<usize> = Vec::new();
    let mut result = Vec::new();
    for index in order {
        active.retain(|other| entries[*other].bounds.max.x >= entries[index].bounds.min.x);
        for other in &active {
            if entries[*other].bounds.intersects(entries[index].bounds) {
                result.push(ordered_indices(*other, index));
            }
        }
        active.push(index);
    }
    result
}

fn parry_candidates(entries: &[SweptEntry<'_>]) -> Vec<(usize, usize)> {
    let aabbs = entries
        .iter()
        .map(|entry| entry.bounds.parry())
        .collect::<Vec<_>>();
    let mut bvh = Qbvh::<usize>::new();
    bvh.clear_and_rebuild(aabbs.iter().copied().enumerate(), 0.0);
    let mut result = BTreeSet::new();
    for (index, aabb) in aabbs.iter().enumerate() {
        let mut intersections = Vec::new();
        bvh.intersect_aabb(aabb, &mut intersections);
        for other in intersections {
            if index != other {
                result.insert(ordered_indices(index, other));
            }
        }
    }
    result.into_iter().collect()
}

fn ordered_indices(a: usize, b: usize) -> (usize, usize) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}
