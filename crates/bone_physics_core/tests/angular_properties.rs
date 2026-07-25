use bone_physics_core::{
    project_angular_limit, AngularLimit, ContinuousAngles, EulerOrder, SwingTwistLimit, XyzLimit,
};
use glam::{Quat, Vec3};

fn next_f32(state: &mut u64) -> f32 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    let unit = ((*state >> 40) as u32) as f32 / ((1_u32 << 24) - 1) as f32;
    unit * 2.0 - 1.0
}

#[test]
fn ten_thousand_finite_rotations_project_without_nan() {
    let xyz = AngularLimit::XyzBox(XyzLimit {
        lower: Vec3::splat(-0.75),
        upper: Vec3::splat(0.75),
        order: EulerOrder::Xyz,
    });
    let swing_twist = AngularLimit::SwingTwist(SwingTwistLimit {
        axis: Vec3::Y,
        max_swing: 0.8,
        min_twist: -0.6,
        max_twist: 0.6,
    });
    let mut continuous = ContinuousAngles::default();
    let mut state = 0x9E37_79B9_7F4A_7C15;

    for index in 0..10_000 {
        let mut axis = Vec3::new(
            next_f32(&mut state),
            next_f32(&mut state),
            next_f32(&mut state),
        );
        if axis.length_squared() < 1.0e-8 {
            axis = Vec3::X;
        }
        let angle = next_f32(&mut state) * core::f32::consts::PI;
        let candidate = Quat::from_axis_angle(axis.normalize(), angle);
        let limit = if index % 2 == 0 { &xyz } else { &swing_twist };
        let projected = project_angular_limit(candidate, limit, &mut continuous).unwrap();

        assert!(projected.is_finite());
        assert!((projected.length_squared() - 1.0).abs() < 1.0e-5);
    }
}
