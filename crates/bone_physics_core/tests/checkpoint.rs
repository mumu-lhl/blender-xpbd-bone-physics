use bone_physics_core::{
    BodyId, BodyInit, BoneBodyDesc, Pose, QualityPolicy, SolverConfig, SolverWorld, StepOptions,
    Velocity, WorldFingerprint,
};
use glam::{Quat, Vec3};

fn world(rotation: Quat, order: &[u32]) -> SolverWorld {
    let bodies = order
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
                gravity_scale: 0.0,
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
            pose: Pose::try_new(Vec3::new(*id as f32, 0.0, 0.0), rotation).unwrap(),
            velocity: Velocity {
                linear: Vec3::new(0.1 * *id as f32, 0.0, 0.0),
                angular: Vec3::ZERO,
            },
        })
        .collect();
    SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        bodies,
    )
    .unwrap()
}

fn options() -> StepOptions {
    StepOptions {
        position_iterations: 4,
        contact_iterations: 2,
        quality: QualityPolicy::AuthoritativeBake,
    }
}

#[test]
fn checkpoint_round_trip_restores_state_and_replays_identically() {
    let mut original = world(Quat::IDENTITY, &[2, 1]);
    original.substep(1.0 / 120.0, 1.0, options()).unwrap();
    let fingerprint = original.world_fingerprint();
    let checkpoint = original.encode_checkpoint().unwrap();
    let expected = original.body(BodyId::new(1)).unwrap().clone();

    original.substep(1.0 / 120.0, 1.0, options()).unwrap();
    original
        .restore_checkpoint(&checkpoint, fingerprint)
        .unwrap();

    assert_eq!(original.body(BodyId::new(1)).unwrap(), &expected);
    assert_eq!(original.world_fingerprint(), fingerprint);
}

#[test]
fn corrupt_or_mismatched_checkpoint_is_rejected_atomically() {
    let mut world = world(Quat::IDENTITY, &[1]);
    let fingerprint = world.world_fingerprint();
    let checkpoint = world.encode_checkpoint().unwrap();
    let before = world.body(BodyId::new(1)).unwrap().clone();

    let mut corrupt = checkpoint.clone();
    corrupt[corrupt.len() - 1] ^= 0x80;
    assert!(world.restore_checkpoint(&corrupt, fingerprint).is_err());
    assert_eq!(world.body(BodyId::new(1)).unwrap(), &before);
    assert!(world
        .restore_checkpoint(&checkpoint, WorldFingerprint::from_bytes([7; 32]))
        .is_err());
    assert_eq!(world.body(BodyId::new(1)).unwrap(), &before);
}

#[test]
fn checkpoint_rejects_wrong_format_version_and_truncation() {
    let world = world(Quat::IDENTITY, &[1]);
    let fingerprint = world.world_fingerprint();
    let checkpoint = world.encode_checkpoint().unwrap();
    let mut wrong_version = checkpoint.clone();
    wrong_version[8] ^= 1;
    assert!(world.clone().restore_checkpoint(&wrong_version, fingerprint).is_err());
    assert!(world
        .clone()
        .restore_checkpoint(&checkpoint[..checkpoint.len() - 1], fingerprint)
        .is_err());
}
