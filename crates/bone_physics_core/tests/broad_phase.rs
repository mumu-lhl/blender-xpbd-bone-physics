use bone_physics_core::{
    BodyId, BroadPhaseBackend, Capsule, CapsuleCollider, ColliderId, CollisionFilter,
    CollisionPipeline, PairKey, Pose, SolverError,
};
use glam::{Quat, Vec3};
use std::collections::BTreeSet;

fn sphere(id: u32, body: u32, x: f32, radius: f32) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(Vec3::ZERO, Vec3::ZERO, radius, 0.0).unwrap(),
        pose: Pose::try_new(Vec3::new(x, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        linear_velocity: Vec3::ZERO,
        filter: CollisionFilter::default(),
    }
}

fn pair(a: u32, b: u32) -> PairKey {
    PairKey::ordered(ColliderId::new(a), ColliderId::new(b)).unwrap()
}

fn capsule(
    id: u32,
    body: u32,
    local_a: Vec3,
    local_b: Vec3,
    radius: f32,
    translation: Vec3,
    rotation: Quat,
    filter: CollisionFilter,
) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(local_a, local_b, radius, 0.0).unwrap(),
        pose: Pose::try_new(translation, rotation).unwrap(),
        linear_velocity: Vec3::ZERO,
        filter,
    }
}

#[derive(Clone, Copy)]
struct TestAabb {
    min: Vec3,
    max: Vec3,
}

impl TestAabb {
    fn from_collider(collider: &CapsuleCollider) -> Self {
        let a = collider.pose.translation + collider.pose.rotation * collider.shape.local_a;
        let b = collider.pose.translation + collider.pose.rotation * collider.shape.local_b;
        let radius = Vec3::splat(collider.shape.radius + collider.shape.margin);
        Self {
            min: a.min(b) - radius,
            max: a.max(b) + radius,
        }
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
}

fn brute_force_swept_pairs(
    pipeline: &CollisionPipeline,
    previous: &[CapsuleCollider],
    predicted: &[CapsuleCollider],
) -> Vec<PairKey> {
    let mut previous = previous.iter().collect::<Vec<_>>();
    let mut predicted = predicted.iter().collect::<Vec<_>>();
    previous.sort_by_key(|collider| collider.id);
    predicted.sort_by_key(|collider| collider.id);
    assert_eq!(previous.len(), predicted.len());

    let swept = previous
        .iter()
        .zip(predicted.iter())
        .map(|(previous, predicted)| {
            assert_eq!(previous.id, predicted.id);
            (
                *predicted,
                TestAabb::from_collider(previous).merged(TestAabb::from_collider(predicted)),
            )
        })
        .collect::<Vec<_>>();

    let mut pairs = BTreeSet::new();
    for (index, (a, aabb_a)) in swept.iter().enumerate() {
        for (b, aabb_b) in swept.iter().skip(index + 1) {
            if aabb_a.intersects(*aabb_b) && pipeline.allows_pair(a, b).unwrap() {
                pairs.insert(PairKey::ordered(a.id, b.id).unwrap());
            }
        }
    }
    pairs.into_iter().collect()
}

#[test]
fn swept_bounds_emit_a_candidate_when_endpoint_frames_are_separated() {
    let pipeline = CollisionPipeline::try_new(&[]).unwrap();
    let previous = vec![sphere(1, 1, 0.0, 0.25), sphere(2, 2, 2.0, 0.25)];
    let predicted = vec![sphere(1, 1, 4.0, 0.25), sphere(2, 2, 2.0, 0.25)];

    let pairs = pipeline.candidate_pairs(&previous, &predicted).unwrap();

    assert_eq!(pairs, vec![pair(1, 2)]);
}

#[test]
fn non_overlapping_swept_bounds_do_not_emit_a_candidate() {
    let pipeline = CollisionPipeline::try_new(&[]).unwrap();
    let previous = vec![sphere(1, 1, 0.0, 0.25), sphere(2, 2, 3.0, 0.25)];
    let predicted = vec![sphere(1, 1, 1.0, 0.25), sphere(2, 2, 3.0, 0.25)];

    assert!(
        pipeline
            .candidate_pairs(&previous, &predicted)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn candidates_are_sorted_unique_and_independent_of_snapshot_order() {
    let a = sphere(4, 4, 0.0, 0.5);
    let b = sphere(1, 1, 0.2, 0.5);
    let c = sphere(3, 3, 0.4, 0.5);
    let d = sphere(2, 2, 5.0, 0.5);
    let previous = vec![a.clone(), b.clone(), c.clone(), d.clone()];
    let predicted = vec![d, c, a, b];
    let pipeline = CollisionPipeline::try_new(&[]).unwrap();

    let pairs = pipeline.candidate_pairs(&previous, &predicted).unwrap();

    assert_eq!(pairs, vec![pair(1, 3), pair(1, 4), pair(3, 4)]);
}

#[test]
fn broad_phase_applies_collision_filters_before_returning_pairs() {
    let mut a = sphere(1, 1, 0.0, 0.5);
    let b = sphere(2, 2, 0.25, 0.5);
    a.filter.excluded_bodies.insert(b.body);
    let pipeline = CollisionPipeline::try_new(&[]).unwrap();

    assert!(
        pipeline
            .candidate_pairs(&[a.clone(), b.clone()], &[a, b])
            .unwrap()
            .is_empty()
    );
}

#[test]
fn sweep_and_parry_backends_return_the_same_stable_pairs() {
    let previous = vec![
        sphere(8, 8, -2.0, 0.4),
        sphere(2, 2, 0.0, 0.4),
        sphere(5, 5, 2.0, 0.4),
    ];
    let predicted = vec![
        sphere(8, 8, 2.0, 0.4),
        sphere(2, 2, 0.0, 0.4),
        sphere(5, 5, -2.0, 0.4),
    ];
    let sweep =
        CollisionPipeline::try_new_with_backend(&[], BroadPhaseBackend::SweepAndPrune).unwrap();
    let parry = CollisionPipeline::try_new_with_backend(&[], BroadPhaseBackend::ParryBvh).unwrap();

    assert_eq!(
        sweep.candidate_pairs(&previous, &predicted).unwrap(),
        parry.candidate_pairs(&previous, &predicted).unwrap()
    );
}

#[test]
fn backends_match_an_independent_swept_aabb_oracle_for_edge_case_matrix() {
    let touching = (
        "touching swept bounds",
        vec![sphere(1, 1, 0.0, 0.5), sphere(2, 2, 1.0, 0.5)],
        vec![sphere(2, 2, 1.0, 0.5), sphere(1, 1, 0.0, 0.5)],
        vec![pair(1, 2)],
    );
    let rotated = (
        "rotated non-spherical capsules",
        vec![
            capsule(
                3,
                3,
                -Vec3::X,
                Vec3::X,
                0.1,
                Vec3::ZERO,
                Quat::from_rotation_z(0.5),
                CollisionFilter::default(),
            ),
            capsule(
                4,
                4,
                -Vec3::X,
                Vec3::X,
                0.1,
                Vec3::new(0.1, 0.0, 0.0),
                Quat::from_rotation_z(-0.5),
                CollisionFilter::default(),
            ),
        ],
        vec![
            capsule(
                4,
                4,
                -Vec3::X,
                Vec3::X,
                0.1,
                Vec3::new(0.3, 0.0, 0.0),
                Quat::from_rotation_z(-0.5),
                CollisionFilter::default(),
            ),
            capsule(
                3,
                3,
                -Vec3::X,
                Vec3::X,
                0.1,
                Vec3::ZERO,
                Quat::from_rotation_z(0.5),
                CollisionFilter::default(),
            ),
        ],
        vec![pair(3, 4)],
    );
    let zero_length = (
        "zero-length capsules",
        vec![sphere(5, 5, 0.0, 0.3), sphere(6, 6, 0.6, 0.3)],
        vec![sphere(6, 6, 0.6, 0.3), sphere(5, 5, 0.0, 0.3)],
        vec![pair(5, 6)],
    );

    let mut excluded_by_body = sphere(20, 20, -0.5, 0.7);
    excluded_by_body.filter.excluded_bodies.insert(BodyId::new(10));
    let mut excluded_by_mask = sphere(40, 40, 0.25, 0.7);
    excluded_by_mask.filter.layer = 0b0010;
    excluded_by_mask.filter.mask = 0b0010;
    let shuffled_filtered = (
        "shuffled snapshots, filters, and multiple overlaps",
        vec![
            excluded_by_mask.clone(),
            sphere(10, 10, 0.0, 0.7),
            sphere(50, 50, 5.0, 0.7),
            excluded_by_body.clone(),
            sphere(30, 30, 0.5, 0.7),
        ],
        vec![
            sphere(30, 30, 0.5, 0.7),
            excluded_by_body,
            sphere(10, 10, 0.0, 0.7),
            excluded_by_mask,
            sphere(50, 50, 5.0, 0.7),
        ],
        vec![pair(10, 30), pair(20, 30)],
    );

    for (name, previous, predicted, expected) in
        [touching, rotated, zero_length, shuffled_filtered]
    {
        let oracle_pipeline = CollisionPipeline::try_new(&[]).unwrap();
        let oracle = brute_force_swept_pairs(&oracle_pipeline, &previous, &predicted);
        assert_eq!(oracle, expected, "{name}");
        assert!(
            oracle.windows(2).all(|pair| pair[0] < pair[1]),
            "{name} must not contain duplicate pairs"
        );

        for backend in [BroadPhaseBackend::SweepAndPrune, BroadPhaseBackend::ParryBvh] {
            let pipeline = CollisionPipeline::try_new_with_backend(&[], backend).unwrap();
            assert_eq!(
                pipeline.candidate_pairs(&previous, &predicted).unwrap(),
                oracle,
                "{name}: {backend:?}"
            );
        }
    }
}

#[test]
fn snapshots_reject_duplicate_missing_or_changed_collider_metadata() {
    let pipeline = CollisionPipeline::try_new(&[]).unwrap();
    let a = sphere(1, 1, 0.0, 0.5);
    let b = sphere(2, 2, 0.0, 0.5);

    assert!(
        pipeline
            .candidate_pairs(&[a.clone(), a.clone()], &[a.clone(), b.clone()])
            .is_err()
    );
    assert!(
        pipeline
            .candidate_pairs(&[a.clone(), b.clone()], std::slice::from_ref(&a))
            .is_err()
    );

    let changed_body = sphere(1, 99, 0.0, 0.5);
    assert!(
        pipeline
            .candidate_pairs(&[a, b.clone()], &[changed_body, b])
            .is_err()
    );
}

#[test]
fn both_backends_reject_changed_shape_and_filter_snapshot_metadata() {
    let a = sphere(1, 1, 0.0, 0.5);
    let b = sphere(2, 2, 0.0, 0.5);
    let mut changed_shape = a.clone();
    changed_shape.shape = Capsule::try_new(Vec3::ZERO, Vec3::X, 0.5, 0.0).unwrap();
    let mut changed_filter = a.clone();
    changed_filter.filter.layer = 0b0010;

    for backend in [BroadPhaseBackend::SweepAndPrune, BroadPhaseBackend::ParryBvh] {
        let pipeline = CollisionPipeline::try_new_with_backend(&[], backend).unwrap();
        for (name, changed) in [
            ("shape", changed_shape.clone()),
            ("filter", changed_filter.clone()),
        ] {
            assert!(
                matches!(
                    pipeline.candidate_pairs(&[a.clone(), b.clone()], &[changed, b.clone()]),
                    Err(SolverError::InvalidValue {
                        field: "collider_snapshot_metadata",
                        ..
                    })
                ),
                "{backend:?} must reject changed {name} metadata"
            );
        }
    }
}
