use bone_physics_core::{
    AnimationTarget, BodyId, BodyInit, BoneBodyDesc, ConstraintId,
    PersistentConstraint, Pose, QualityPolicy, SolverConfig, SolverWorld,
    StepOptions, Velocity,
};
use glam::{Quat, Vec3};

fn body(rotation: Quat) -> BodyInit {
    BodyInit {
        desc: BoneBodyDesc {
            id: BodyId::new(1),
            rest_pose: Pose::IDENTITY,
            local_center_of_mass: Vec3::ZERO,
            local_head: Vec3::new(0.0, -0.5, 0.0),
            local_tail: Vec3::new(0.0, 0.5, 0.0),
            inverse_mass: 1.0,
            inverse_inertia: Vec3::ONE,
            gravity_scale: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
        },
        pose: Pose::try_new(Vec3::ZERO, rotation).unwrap(),
        velocity: Velocity::ZERO,
    }
}

fn options() -> StepOptions {
    StepOptions {
        position_iterations: 12,
        contact_iterations: 0,
        quality: QualityPolicy::AuthoritativeBake,
    }
}

fn translated_target(translation_inertia: f32) -> AnimationTarget {
    AnimationTarget {
        previous: Pose::IDENTITY,
        current: Pose::try_new(Vec3::X, Quat::IDENTITY).unwrap(),
        translation_inertia,
        rotation_inertia: 1.0,
    }
}

fn run_translation_inertia(factor: f32) -> f32 {
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![body(Quat::IDENTITY)],
    )
    .unwrap();
    world
        .set_animation_targets(&[(BodyId::new(1), translated_target(factor))])
        .unwrap();

    world.substep(1.0 / 120.0, 1.0, options()).unwrap();

    world.body(BodyId::new(1)).unwrap().pose.translation.x
}

#[test]
fn translation_inertia_controls_reference_frame_following() {
    assert!((run_translation_inertia(0.0) - 1.0).abs() < 1.0e-6);
    assert!(run_translation_inertia(1.0).abs() < 1.0e-6);
    assert!((run_translation_inertia(1.5) + 0.5).abs() < 1.0e-6);
}

#[test]
fn hard_orientation_recovery_matches_the_animation_target() {
    let initial = Quat::from_rotation_z(1.0);
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![body(initial)],
    )
    .unwrap();
    world
        .set_animation_targets(&[(
            BodyId::new(1),
            AnimationTarget {
                previous: Pose::IDENTITY,
                current: Pose::IDENTITY,
                translation_inertia: 1.0,
                rotation_inertia: 1.0,
            },
        )])
        .unwrap();
    world
        .set_constraints(vec![PersistentConstraint::orientation_recovery(
            ConstraintId::new(30),
            BodyId::new(1),
            0.0,
            0.0,
        )])
        .unwrap();

    world.substep(1.0 / 120.0, 1.0, options()).unwrap();

    let actual = world.body(BodyId::new(1)).unwrap().pose.rotation;
    assert!(actual.angle_between(Quat::IDENTITY) < 1.0e-5);
}

#[test]
fn animation_target_rejects_non_finite_inertia() {
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![body(Quat::IDENTITY)],
    )
    .unwrap();

    let result = world.set_animation_targets(&[(
        BodyId::new(1),
        AnimationTarget {
            previous: Pose::IDENTITY,
            current: Pose::IDENTITY,
            translation_inertia: f32::NAN,
            rotation_inertia: 1.0,
        },
    )]);

    assert!(result.is_err());
}
