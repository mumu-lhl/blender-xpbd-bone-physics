use bone_physics_core::{
    BodyId, BodyInit, BoneBodyDesc, CorrectionKind, FixedClock, Pose,
    QualityPolicy, SolverConfig, SolverError, SolverWorld, Velocity,
};
use glam::{Quat, Vec3};

fn body_desc(id: u32, linear_damping: f32, angular_damping: f32) -> BoneBodyDesc {
    BoneBodyDesc {
        id: BodyId::new(id),
        rest_pose: Pose::IDENTITY,
        local_center_of_mass: Vec3::ZERO,
        local_head: Vec3::new(0.0, -0.5, 0.0),
        local_tail: Vec3::new(0.0, 0.5, 0.0),
        inverse_mass: 1.0,
        inverse_inertia: Vec3::ONE,
        gravity_scale: 1.0,
        linear_damping,
        angular_damping,
    }
}

fn world(
    gravity: Vec3,
    velocity: Velocity,
    linear_damping: f32,
    angular_damping: f32,
) -> SolverWorld {
    SolverWorld::new(
        SolverConfig {
            gravity,
            fixed_dt: 1.0 / 120.0,
        },
        vec![BodyInit {
            desc: body_desc(7, linear_damping, angular_damping),
            pose: Pose::IDENTITY,
            velocity,
        }],
    )
    .unwrap()
}

fn run_for_one_second(mut world: SolverWorld, fps: u32) -> SolverWorld {
    let mut clock = FixedClock::new(1.0 / 120.0).unwrap();
    for _ in 0..fps {
        let batch = clock
            .try_accumulate(1.0 / fps as f32, QualityPolicy::AuthoritativeBake)
            .unwrap();
        for _ in 0..batch.steps {
            world.predict(batch.dt).unwrap();
            world.rebuild_velocities(batch.dt).unwrap();
        }
    }
    world
}

#[test]
fn world_rejects_duplicate_body_ids() {
    let desc = body_desc(7, 0.0, 0.0);
    let init = BodyInit {
        desc,
        pose: Pose::IDENTITY,
        velocity: Velocity::ZERO,
    };

    let result = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![init.clone(), init],
    );

    assert!(matches!(
        result,
        Err(SolverError::DuplicateId {
            kind: "body",
            id: 7
        })
    ));
}

#[test]
fn gravity_is_independent_of_output_frame_rate() {
    let pose_24 = run_for_one_second(
        world(Vec3::new(0.0, 0.0, -9.81), Velocity::ZERO, 0.0, 0.0),
        24,
    )
    .body(BodyId::new(7))
    .unwrap()
    .pose;
    let pose_60 = run_for_one_second(
        world(Vec3::new(0.0, 0.0, -9.81), Velocity::ZERO, 0.0, 0.0),
        60,
    )
    .body(BodyId::new(7))
    .unwrap()
    .pose;

    assert!((pose_24.translation - pose_60.translation).length() < 5.0e-5);
}

#[test]
fn exponential_linear_damping_is_per_second() {
    let initial = Velocity {
        linear: Vec3::new(10.0, 0.0, 0.0),
        angular: Vec3::ZERO,
    };
    let world = run_for_one_second(world(Vec3::ZERO, initial, 2.0, 0.0), 30);
    let actual = world.body(BodyId::new(7)).unwrap().velocity.linear.x;
    let expected = 10.0 * (-2.0_f32).exp();

    assert!((actual - expected).abs() < 1.0e-5);
}

#[test]
fn angular_velocity_uses_quaternion_exponential_integration() {
    let initial = Velocity {
        linear: Vec3::ZERO,
        angular: Vec3::Z * core::f32::consts::PI,
    };
    let mut world = world(Vec3::ZERO, initial, 0.0, 0.0);
    for _ in 0..60 {
        world.predict(1.0 / 120.0).unwrap();
        world.rebuild_velocities(1.0 / 120.0).unwrap();
    }

    let actual = world.body(BodyId::new(7)).unwrap().pose.rotation;
    let expected = Quat::from_rotation_z(core::f32::consts::FRAC_PI_2);

    assert!(actual.angle_between(expected) < 1.0e-5);
}

#[test]
fn non_velocity_contact_correction_does_not_create_linear_velocity() {
    let mut world = world(Vec3::ZERO, Velocity::ZERO, 0.0, 0.0);
    world.predict(1.0 / 120.0).unwrap();
    world
        .apply_pose_correction(
            BodyId::new(7),
            Vec3::X,
            Vec3::ZERO,
            CorrectionKind::NonVelocityContact,
        )
        .unwrap();
    world.rebuild_velocities(1.0 / 120.0).unwrap();

    let state = world.body(BodyId::new(7)).unwrap();
    assert_eq!(state.pose.translation, Vec3::X);
    assert_eq!(state.velocity.linear, Vec3::ZERO);
}
