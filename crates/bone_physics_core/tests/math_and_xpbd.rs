use bone_physics_core::{
    quat_exp, quat_log, solve_scalar_row, world_inverse_inertia, BodyId, BodyInit, BoneBodyDesc,
    Pose, RowResult, SolverConfig, SolverWorld, Velocity, XpbdGradient, XpbdRow,
};
use glam::{Quat, Vec3};

fn world_with_inverse_properties(inverse_mass: f32, inverse_inertia: Vec3) -> SolverWorld {
    let desc = BoneBodyDesc {
        id: BodyId::new(1),
        rest_pose: Pose::IDENTITY,
        local_center_of_mass: Vec3::ZERO,
        local_head: Vec3::new(0.0, -0.5, 0.0),
        local_tail: Vec3::new(0.0, 0.5, 0.0),
        inverse_mass,
        inverse_inertia,
        gravity_scale: 0.0,
        linear_damping: 0.0,
        angular_damping: 0.0,
    };
    SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![BodyInit {
            desc,
            pose: Pose::IDENTITY,
            velocity: Velocity::ZERO,
        }],
    )
    .unwrap()
}

#[test]
fn quaternion_log_exp_round_trips_near_pi() {
    let rotation = Quat::from_axis_angle(Vec3::Y, core::f32::consts::PI - 1.0e-4);

    let round_trip = quat_exp(quat_log(rotation));

    assert!(rotation.angle_between(round_trip) < 1.0e-5);
}

#[test]
fn quaternion_log_uses_the_same_short_branch_for_opposite_representations() {
    let rotation = Quat::from_axis_angle(Vec3::X, 0.75);

    let positive = quat_log(rotation);
    let negative = quat_log(-rotation);

    assert!((positive - negative).length() < 1.0e-6);
}

#[test]
fn inverse_inertia_rotates_from_local_to_world_space() {
    let rotation = Quat::from_rotation_z(core::f32::consts::FRAC_PI_2);

    let inertia = world_inverse_inertia(rotation, Vec3::new(1.0, 2.0, 3.0));

    assert!((inertia * Vec3::X - Vec3::X * 2.0).length() < 1.0e-5);
    assert!((inertia * Vec3::Y - Vec3::Y).length() < 1.0e-5);
    assert!((inertia * Vec3::Z - Vec3::Z * 3.0).length() < 1.0e-5);
}

#[test]
fn xpbd_row_scales_compliance_by_dt_squared() {
    let mut world = world_with_inverse_properties(1.0, Vec3::ONE);
    world.predict(0.5).unwrap();
    let mut row = XpbdRow {
        c: 1.0,
        compliance: 0.25,
        lambda: 0.0,
        gradients: vec![XpbdGradient {
            body: BodyId::new(1),
            linear: Vec3::X,
            angular: Vec3::ZERO,
        }],
    };

    let result = solve_scalar_row(&mut world, &mut row, 0.5).unwrap();

    assert_eq!(
        result,
        RowResult {
            delta_lambda: -0.5,
            applied: true
        }
    );
    assert_eq!(row.lambda, -0.5);
    assert_eq!(
        world
            .body(BodyId::new(1))
            .unwrap()
            .predicted_pose
            .translation,
        Vec3::new(-0.5, 0.0, 0.0)
    );
}

#[test]
fn zero_effective_mass_returns_a_finite_non_applied_row() {
    let mut world = world_with_inverse_properties(0.0, Vec3::ZERO);
    world.predict(1.0 / 120.0).unwrap();
    let mut row = XpbdRow {
        c: 1.0,
        compliance: 0.0,
        lambda: 0.0,
        gradients: vec![XpbdGradient {
            body: BodyId::new(1),
            linear: Vec3::X,
            angular: Vec3::Y,
        }],
    };

    let result = solve_scalar_row(&mut world, &mut row, 1.0 / 120.0).unwrap();

    assert_eq!(
        result,
        RowResult {
            delta_lambda: 0.0,
            applied: false
        }
    );
    assert!(row.lambda.is_finite());
}
