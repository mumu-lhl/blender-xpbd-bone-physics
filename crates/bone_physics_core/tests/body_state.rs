use bone_physics_core::{
    BodyId, BoneBodyDesc, BoneBodyState, Pose, SolverError, Velocity,
};
use glam::{Quat, Vec3};

fn valid_desc() -> BoneBodyDesc {
    BoneBodyDesc {
        id: BodyId::new(11),
        rest_pose: Pose::IDENTITY,
        local_center_of_mass: Vec3::ZERO,
        local_head: Vec3::new(0.0, -0.5, 0.0),
        local_tail: Vec3::new(0.0, 0.5, 0.0),
        inverse_mass: 0.5,
        inverse_inertia: Vec3::new(1.0, 2.0, 3.0),
        gravity_scale: 1.0,
        linear_damping: 0.25,
        angular_damping: 0.5,
    }
}

#[test]
fn descriptor_rejects_a_zero_length_bone() {
    let mut desc = valid_desc();
    desc.local_tail = desc.local_head;

    let error = desc.validate().unwrap_err();

    assert!(matches!(
        error,
        SolverError::InvalidValue {
            field: "bone_length",
            ..
        }
    ));
}

#[test]
fn descriptor_rejects_negative_physical_coefficients() {
    for (field, mutate) in [
        (
            "inverse_mass",
            (|desc: &mut BoneBodyDesc| desc.inverse_mass = -0.1)
                as fn(&mut BoneBodyDesc),
        ),
        (
            "linear_damping",
            (|desc: &mut BoneBodyDesc| desc.linear_damping = -0.1)
                as fn(&mut BoneBodyDesc),
        ),
        (
            "angular_damping",
            (|desc: &mut BoneBodyDesc| desc.angular_damping = -0.1)
                as fn(&mut BoneBodyDesc),
        ),
    ] {
        let mut desc = valid_desc();
        mutate(&mut desc);

        let error = desc.validate().unwrap_err();

        assert!(matches!(
            error,
            SolverError::InvalidValue {
                field: actual,
                ..
            } if actual == field
        ));
    }
}

#[test]
fn pose_normalizes_a_finite_nonzero_quaternion() {
    let pose = Pose::try_new(
        Vec3::new(1.0, 2.0, 3.0),
        Quat::from_xyzw(0.0, 0.0, 0.0, 2.0),
    )
    .unwrap();

    assert_eq!(pose.translation, Vec3::new(1.0, 2.0, 3.0));
    assert_eq!(pose.rotation, Quat::IDENTITY);
}

#[test]
fn pose_rejects_non_finite_or_near_zero_input() {
    assert!(Pose::try_new(Vec3::new(f32::NAN, 0.0, 0.0), Quat::IDENTITY).is_err());
    assert!(Pose::try_new(Vec3::ZERO, Quat::from_xyzw(0.0, 0.0, 0.0, 0.0)).is_err());
}

#[test]
fn body_state_initializes_all_pose_history_without_velocity() {
    let desc = valid_desc();
    let initial = Pose::try_new(
        Vec3::new(4.0, 5.0, 6.0),
        Quat::from_rotation_z(0.25),
    )
    .unwrap();

    let state = BoneBodyState::from_desc(&desc, initial).unwrap();

    assert_eq!(state.pose, initial);
    assert_eq!(state.previous_pose, initial);
    assert_eq!(state.predicted_pose, initial);
    assert_eq!(state.velocity, Velocity::ZERO);
    assert_eq!(state.contact_translation_correction, Vec3::ZERO);
    assert_eq!(state.contact_rotation_correction, Vec3::ZERO);
}
