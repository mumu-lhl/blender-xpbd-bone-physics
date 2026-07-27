use bone_physics_core::{
    BodyId, BodyInit, BoneBodyDesc, Capsule, CapsuleColliderDesc, ColliderId, CollisionFilter,
    ConstraintId, ContactKey, ContactMaterial, PairKey, PersistentConstraint, Pose, QualityPolicy,
    SolverConfig, SolverError, SolverWorld, StepFailureKind, StepOptions, Velocity,
    MAX_MULTIPLIER_MAGNITUDE, QUATERNION_NORMALIZATION_TOLERANCE,
};
use glam::{Quat, Vec3};

const SAFE_DT: f32 = 1.0e-38;

fn body(id: u32, translation: Vec3, linear_velocity: Vec3, inverse_mass: f32) -> BodyInit {
    BodyInit {
        desc: BoneBodyDesc {
            id: BodyId::new(id),
            rest_pose: Pose::IDENTITY,
            local_center_of_mass: Vec3::ZERO,
            local_head: -0.5 * Vec3::Y,
            local_tail: 0.5 * Vec3::Y,
            inverse_mass,
            inverse_inertia: Vec3::splat(inverse_mass),
            gravity_scale: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
        },
        pose: Pose::try_new(translation, Quat::IDENTITY).unwrap(),
        velocity: Velocity {
            linear: linear_velocity,
            angular: Vec3::ZERO,
        },
    }
}

fn world(bodies: Vec<BodyInit>) -> SolverWorld {
    SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        bodies,
    )
    .unwrap()
}

fn options(position_iterations: u32) -> StepOptions {
    StepOptions {
        position_iterations,
        contact_iterations: position_iterations,
        quality: QualityPolicy::AuthoritativeBake,
    }
}

fn sphere(id: u32, body: u32, radius: f32) -> CapsuleColliderDesc {
    CapsuleColliderDesc {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(Vec3::ZERO, Vec3::ZERO, radius, 0.0).unwrap(),
        material: ContactMaterial::try_new(0.0, 0.0).unwrap(),
        filter: CollisionFilter::default(),
    }
}

fn scalar_bits(value: f32) -> u32 {
    value.to_bits()
}

fn vec_bits(value: Vec3) -> [u32; 3] {
    value.to_array().map(scalar_bits)
}

fn quat_bits(value: Quat) -> [u32; 4] {
    value.to_array().map(scalar_bits)
}

fn body_bits(world: &SolverWorld, id: BodyId) -> Vec<u32> {
    let state = world.body(id).unwrap();
    let mut bits = Vec::with_capacity(32);
    bits.extend(vec_bits(state.pose.translation));
    bits.extend(quat_bits(state.pose.rotation));
    bits.extend(vec_bits(state.previous_pose.translation));
    bits.extend(quat_bits(state.previous_pose.rotation));
    bits.extend(vec_bits(state.predicted_pose.translation));
    bits.extend(quat_bits(state.predicted_pose.rotation));
    bits.extend(vec_bits(state.velocity.linear));
    bits.extend(vec_bits(state.velocity.angular));
    bits.extend(vec_bits(state.contact_translation_correction));
    bits.extend(vec_bits(state.contact_rotation_correction));
    bits
}

fn assert_finite_diagnostics(diagnostics: bone_physics_core::StepDiagnostics) {
    assert!(diagnostics.deepest_penetration.is_finite());
    assert!(diagnostics.max_constraint_error.is_finite());
}

fn overflow_world(input_order: [u32; 2]) -> SolverWorld {
    let base = 0.5 * f32::MAX;
    let speed = 0.25 * f32::MAX;
    let bodies = vec![
        body(
            input_order[0],
            base * Vec3::X,
            speed * Vec3::X,
            1.0,
        ),
        body(
            input_order[1],
            base * Vec3::X,
            speed * Vec3::X,
            1.0,
        ),
        body(1, base * Vec3::X, Vec3::ZERO, 0.0),
    ];
    let mut world = world(bodies);
    world
        .set_colliders(vec![sphere(10, 1, 2.0), sphere(20, 2, 2.0)])
        .unwrap();
    world
}

#[test]
fn successful_substep_reports_finite_residuals_without_rollback() {
    assert!(QUATERNION_NORMALIZATION_TOLERANCE > 0.0);
    assert!(QUATERNION_NORMALIZATION_TOLERANCE < 1.0);
    assert!(MAX_MULTIPLIER_MAGNITUDE.is_finite());
    assert!(MAX_MULTIPLIER_MAGNITUDE > 0.0);

    let mut world = world(vec![body(1, Vec3::ZERO, Vec3::ZERO, 1.0)]);
    let diagnostics = world.substep(1.0 / 120.0, 1.0, options(4)).unwrap();

    assert!(!diagnostics.rolled_back);
    assert!(!world.is_paused());
    assert_eq!(world.last_failure(), None);
    assert_finite_diagnostics(diagnostics);
}

#[test]
fn prediction_overflow_rolls_back_every_observable_dynamic_value_and_names_body() {
    let mut world = overflow_world([2, 9]);
    world.substep(SAFE_DT, 1.0, options(1)).unwrap();
    assert!(!world.cached_contacts().is_empty());

    let body_before = body_bits(&world, BodyId::new(2));
    let cache_before = world.cached_contacts();
    let diagnostics = world.substep(3.0, 1.0, options(1)).unwrap();

    assert!(diagnostics.rolled_back);
    assert_finite_diagnostics(diagnostics);
    assert!(world.is_paused());
    assert_eq!(body_bits(&world, BodyId::new(2)), body_before);
    assert_eq!(world.cached_contacts(), cache_before);
    let failure = world.last_failure().unwrap();
    assert_eq!(failure.kind, StepFailureKind::NonFiniteBodyState);
    assert_eq!(failure.body, Some(BodyId::new(2)));
    assert_eq!(failure.constraint, None);
    assert_eq!(failure.contact, None);
}

#[test]
fn paused_world_rejects_steps_without_changing_non_vacuous_cache_or_lambda() {
    let mut world = overflow_world([2, 9]);
    let constraint_id = ConstraintId::new(30);
    world
        .set_constraints(vec![PersistentConstraint::distance_range(
            constraint_id,
            BodyId::new(1),
            BodyId::new(2),
            Vec3::ZERO,
            Vec3::ZERO,
            1.0,
            2.0,
            0.0,
            0.0,
        )])
        .unwrap();
    world.substep(SAFE_DT, 1.0, options(1)).unwrap();
    assert!(!world.cached_contacts().is_empty());
    let lambda_before = world.constraint_lambda(constraint_id).unwrap();
    assert!(lambda_before > 0.0);
    world.substep(3.0, 1.0, options(1)).unwrap();

    let body_before = body_bits(&world, BodyId::new(2));
    let cache_before = world.cached_contacts();
    let failure_before = world.last_failure().cloned();
    let error = world.substep(SAFE_DT, 1.0, options(1)).unwrap_err();

    assert_eq!(error, SolverError::WorldPaused);
    assert_eq!(body_bits(&world, BodyId::new(2)), body_before);
    assert_eq!(world.cached_contacts(), cache_before);
    assert_eq!(world.constraint_lambda(constraint_id), Some(lambda_before));
    assert_eq!(world.last_failure(), failure_before.as_ref());
}

#[test]
fn resume_clears_failure_and_allows_a_deterministic_safe_step() {
    let mut world = overflow_world([2, 9]);
    let mut fresh = overflow_world([2, 9]);
    world.substep(3.0, 1.0, options(1)).unwrap();
    let restored_bodies: Vec<_> = [BodyId::new(1), BodyId::new(2), BodyId::new(9)]
        .map(|id| body_bits(&world, id))
        .into();
    let restored_cache = world.cached_contacts();

    world.resume().unwrap();

    assert!(!world.is_paused());
    assert_eq!(world.last_failure(), None);
    for (index, id) in [BodyId::new(1), BodyId::new(2), BodyId::new(9)]
        .into_iter()
        .enumerate()
    {
        assert_eq!(body_bits(&world, id), restored_bodies[index]);
        assert_eq!(body_bits(&world, id), body_bits(&fresh, id));
    }
    assert_eq!(world.cached_contacts(), restored_cache);
    assert_eq!(world.cached_contacts(), fresh.cached_contacts());

    let resumed_diagnostics = world.substep(SAFE_DT, 1.0, options(1)).unwrap();
    let fresh_diagnostics = fresh.substep(SAFE_DT, 1.0, options(1)).unwrap();
    assert_eq!(resumed_diagnostics, fresh_diagnostics);
    assert!(!resumed_diagnostics.rolled_back);
    assert_finite_diagnostics(resumed_diagnostics);
    for id in [BodyId::new(1), BodyId::new(2), BodyId::new(9)] {
        assert_eq!(body_bits(&world, id), body_bits(&fresh, id));
    }
    assert_eq!(world.cached_contacts(), fresh.cached_contacts());
}

#[test]
fn excessive_finite_constraint_multiplier_rolls_back_and_names_constraint() {
    let inverse_mass = 1.0e-6;
    let error = 2.0 * MAX_MULTIPLIER_MAGNITUDE * inverse_mass;
    assert!(error.is_finite());
    let mut world = world(vec![
        body(1, Vec3::ZERO, Vec3::ZERO, 0.0),
        body(2, error * Vec3::X, Vec3::ZERO, inverse_mass),
    ]);
    let constraint_id = ConstraintId::new(7);
    world
        .set_constraints(vec![PersistentConstraint::anchor(
            constraint_id,
            BodyId::new(1),
            BodyId::new(2),
            Vec3::ZERO,
            Vec3::ZERO,
            0.0,
            0.0,
        )])
        .unwrap();
    let body_before = body_bits(&world, BodyId::new(2));

    let diagnostics = world.substep(1.0, 1.0, options(1)).unwrap();

    assert!(diagnostics.rolled_back);
    assert_eq!(body_bits(&world, BodyId::new(2)), body_before);
    assert_eq!(world.constraint_lambda(constraint_id), Some(0.0));
    let failure = world.last_failure().unwrap();
    assert_eq!(
        failure.kind,
        StepFailureKind::ExcessiveConstraintMultiplier
    );
    assert_eq!(failure.body, None);
    assert_eq!(failure.constraint, Some(constraint_id));
    assert_eq!(failure.contact, None);
}

#[test]
fn excessive_finite_contact_multiplier_rolls_back_and_names_contact() {
    let radius = 0.75 * MAX_MULTIPLIER_MAGNITUDE;
    assert!(radius.is_finite());
    assert!((2.0 * radius).is_finite());
    let mut world = world(vec![
        body(1, Vec3::ZERO, Vec3::ZERO, 0.0),
        body(2, Vec3::ZERO, Vec3::ZERO, 1.0),
    ]);
    world
        .set_colliders(vec![sphere(10, 1, radius), sphere(20, 2, radius)])
        .unwrap();
    let body_before = body_bits(&world, BodyId::new(2));

    let diagnostics = world.substep(1.0, 1.0, options(1)).unwrap();

    assert!(diagnostics.rolled_back);
    assert_eq!(body_bits(&world, BodyId::new(2)), body_before);
    assert!(world.cached_contacts().is_empty());
    let failure = world.last_failure().unwrap();
    assert_eq!(failure.kind, StepFailureKind::ExcessiveContactMultiplier);
    assert_eq!(failure.body, None);
    assert_eq!(failure.constraint, None);
    assert_eq!(
        failure.contact,
        Some(ContactKey::new(
            PairKey::ordered(ColliderId::new(10), ColliderId::new(20)).unwrap(),
            0,
            0,
        ))
    );
}

#[test]
fn multiple_non_finite_bodies_report_lowest_stable_id_independent_of_input_order() {
    for input_order in [[9, 2], [2, 9]] {
        let mut world = overflow_world(input_order);
        let diagnostics = world.substep(3.0, 1.0, options(1)).unwrap();

        assert!(diagnostics.rolled_back);
        assert_eq!(
            world.last_failure().unwrap().body,
            Some(BodyId::new(2))
        );
    }
}
