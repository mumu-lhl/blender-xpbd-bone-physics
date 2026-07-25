use bone_physics_core::{
    AngularLimit, BodyId, BodyInit, BoneBodyDesc, ConstraintId, ContinuousAngles, EulerOrder,
    PersistentConstraint, Pose, QualityPolicy, SolverConfig, SolverWorld, StepOptions,
    SwingTwistLimit, Velocity, XyzLimit,
};
use glam::{EulerRot, Quat, Vec3};

fn body(id: u32, rotation: Quat, dynamic: bool) -> BodyInit {
    let inverse = if dynamic { 1.0 } else { 0.0 };
    BodyInit {
        desc: BoneBodyDesc {
            id: BodyId::new(id),
            rest_pose: Pose::IDENTITY,
            local_center_of_mass: Vec3::ZERO,
            local_head: Vec3::new(0.0, -0.5, 0.0),
            local_tail: Vec3::new(0.0, 0.5, 0.0),
            inverse_mass: inverse,
            inverse_inertia: Vec3::splat(inverse),
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
        position_iterations: 20,
        contact_iterations: 0,
        quality: QualityPolicy::AuthoritativeBake,
    }
}

fn solve_xyz_lock(initial: Vec3, lower: Vec3, upper: Vec3) -> Vec3 {
    let rotation = Quat::from_euler(EulerRot::XYZ, initial.x, initial.y, initial.z);
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![
            body(1, Quat::IDENTITY, false),
            body(2, rotation, true),
        ],
    )
    .unwrap();
    world
        .set_constraints(vec![PersistentConstraint::angular_limit(
            ConstraintId::new(40),
            BodyId::new(1),
            BodyId::new(2),
            Quat::IDENTITY,
            AngularLimit::XyzBox(XyzLimit {
                lower,
                upper,
                order: EulerOrder::Xyz,
            }),
            0.0,
            0.0,
        )])
        .unwrap();

    world.substep(1.0 / 120.0, 1.0, options()).unwrap();
    let solved = world.body(BodyId::new(2)).unwrap().pose.rotation;
    let (x, y, z) = solved.to_euler(EulerRot::XYZ);
    Vec3::new(x, y, z)
}

#[test]
fn equal_xyz_bounds_lock_each_individual_axis() {
    let free = core::f32::consts::PI;
    let initial = Vec3::new(0.45, -0.35, 0.25);

    let x_locked = solve_xyz_lock(
        initial,
        Vec3::new(0.0, -free, -free),
        Vec3::new(0.0, free, free),
    );
    let y_locked = solve_xyz_lock(
        initial,
        Vec3::new(-free, 0.0, -free),
        Vec3::new(free, 0.0, free),
    );
    let z_locked = solve_xyz_lock(
        initial,
        Vec3::new(-free, -free, 0.0),
        Vec3::new(free, free, 0.0),
    );

    assert!(x_locked.x.abs() < 1.0e-4);
    assert!(y_locked.y.abs() < 1.0e-4);
    assert!(z_locked.z.abs() < 1.0e-4);
}

#[test]
fn continuous_angles_do_not_jump_at_the_pi_branch() {
    let mut continuous = ContinuousAngles::default();
    let before = continuous.unwrap(Vec3::new(0.0, 0.0, 3.10));
    let after = continuous.unwrap(Vec3::new(0.0, 0.0, -3.10));

    assert!((before.z - 3.10).abs() < 1.0e-6);
    assert!(after.z > before.z);
    assert!(after.z < 3.20);
}

#[test]
fn twist_inside_range_is_unchanged_by_an_unconstrained_swing_cone() {
    let initial = Quat::from_axis_angle(Vec3::Y, 0.6);
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![
            body(1, Quat::IDENTITY, false),
            body(2, initial, true),
        ],
    )
    .unwrap();
    world
        .set_constraints(vec![PersistentConstraint::angular_limit(
            ConstraintId::new(41),
            BodyId::new(1),
            BodyId::new(2),
            Quat::IDENTITY,
            AngularLimit::SwingTwist(SwingTwistLimit {
                axis: Vec3::Y,
                max_swing: core::f32::consts::PI,
                min_twist: -1.0,
                max_twist: 1.0,
            }),
            0.0,
            0.0,
        )])
        .unwrap();

    world.substep(1.0 / 120.0, 1.0, options()).unwrap();

    let solved = world.body(BodyId::new(2)).unwrap().pose.rotation;
    assert!(solved.angle_between(initial) < 1.0e-5);
}

#[test]
fn invalid_angular_ranges_are_rejected() {
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![
            body(1, Quat::IDENTITY, false),
            body(2, Quat::IDENTITY, true),
        ],
    )
    .unwrap();
    let result = world.set_constraints(vec![PersistentConstraint::angular_limit(
        ConstraintId::new(42),
        BodyId::new(1),
        BodyId::new(2),
        Quat::IDENTITY,
        AngularLimit::XyzBox(XyzLimit {
            lower: Vec3::X,
            upper: Vec3::ZERO,
            order: EulerOrder::Xyz,
        }),
        0.0,
        0.0,
    )]);

    assert!(result.is_err());
}
