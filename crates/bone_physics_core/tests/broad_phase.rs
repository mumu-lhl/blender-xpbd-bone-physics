use bone_physics_core::{
    BodyId, BroadPhaseBackend, Capsule, CapsuleCollider, ColliderId, CollisionFilter,
    CollisionPipeline, PairKey, Pose,
};
use glam::{Quat, Vec3};

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
