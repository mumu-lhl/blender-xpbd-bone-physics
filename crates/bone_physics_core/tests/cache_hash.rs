use bone_physics_core::{BodyId, BodyInit, BoneBodyDesc, Pose, SolverConfig, SolverWorld, WorldFingerprint};
use glam::{Quat, Vec3};

fn world(rotation: Quat, order: &[u32]) -> SolverWorld {
    SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: 1.0 / 120.0,
        },
        order
            .iter()
            .map(|id| BodyInit {
                desc: BoneBodyDesc {
                    id: BodyId::new(*id),
                    rest_pose: Pose::try_new(Vec3::ZERO, rotation).unwrap(),
                    local_center_of_mass: Vec3::ZERO,
                    local_head: -Vec3::Y,
                    local_tail: Vec3::Y,
                    inverse_mass: 1.0,
                    inverse_inertia: Vec3::ONE,
                    gravity_scale: 1.0,
                    linear_damping: 0.1,
                    angular_damping: 0.1,
                },
                pose: Pose::try_new(Vec3::new(*id as f32, 0.0, 0.0), rotation).unwrap(),
                velocity: bone_physics_core::Velocity::ZERO,
            })
            .collect(),
    )
    .unwrap()
}

#[test]
fn fingerprint_is_independent_of_input_order() {
    assert_eq!(
        world(Quat::IDENTITY, &[1, 2, 3]).world_fingerprint(),
        world(Quat::IDENTITY, &[3, 1, 2]).world_fingerprint()
    );
}

#[test]
fn equivalent_quaternion_representations_have_the_same_fingerprint() {
    let q = Quat::from_rotation_y(0.37);
    assert_eq!(
        world(q, &[1]).world_fingerprint(),
        world(-q, &[1]).world_fingerprint()
    );
}

#[test]
fn fingerprint_bytes_are_fixed_width_and_copyable() {
    let fingerprint = WorldFingerprint::from_bytes([3; 32]);
    assert_eq!(fingerprint.as_bytes(), &[3; 32]);
}
