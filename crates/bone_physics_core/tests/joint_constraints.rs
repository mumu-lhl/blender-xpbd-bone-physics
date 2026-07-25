use bone_physics_core::{
    BodyId, BodyInit, BoneBodyDesc, ConstraintId, PersistentConstraint, Pose, QualityPolicy,
    SolverConfig, SolverWorld, StepOptions, Velocity,
};
use glam::{Quat, Vec3};

fn body(id: u32, translation: Vec3, dynamic: bool) -> BodyInit {
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
            gravity_scale: if dynamic { 1.0 } else { 0.0 },
            linear_damping: 0.0,
            angular_damping: 0.0,
        },
        pose: Pose::try_new(translation, Quat::IDENTITY).unwrap(),
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

#[test]
fn hard_anchor_stays_coincident_under_gravity() {
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::new(0.0, 0.0, -9.81),
            fixed_dt: 1.0 / 120.0,
        },
        vec![body(1, Vec3::ZERO, false), body(2, Vec3::ZERO, true)],
    )
    .unwrap();
    let joint_id = ConstraintId::new(10);
    world
        .set_constraints(vec![PersistentConstraint::anchor(
            joint_id,
            BodyId::new(1),
            BodyId::new(2),
            Vec3::ZERO,
            Vec3::ZERO,
            0.0,
            0.0,
        )])
        .unwrap();

    for _ in 0..120 {
        world.substep(1.0 / 120.0, 1.0, options()).unwrap();
    }

    assert!(world.anchor_error(joint_id).unwrap() < 1.0e-5);
}

#[test]
fn distance_range_separates_bodies_without_an_attractive_multiplier() {
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![
            body(1, Vec3::ZERO, false),
            body(2, Vec3::new(0.5, 0.0, 0.0), true),
        ],
    )
    .unwrap();
    let range_id = ConstraintId::new(20);
    world
        .set_constraints(vec![PersistentConstraint::distance_range(
            range_id,
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

    world.substep(1.0 / 120.0, 1.0, options()).unwrap();

    let separation = world
        .body(BodyId::new(1))
        .unwrap()
        .pose
        .translation
        .distance(world.body(BodyId::new(2)).unwrap().pose.translation);
    assert!(separation >= 1.0 - 1.0e-5);
    assert!(world.constraint_lambda(range_id).unwrap() >= 0.0);
}

#[test]
fn constraints_are_sorted_by_stable_id_before_solving() {
    let mut world_a = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![
            body(1, Vec3::ZERO, false),
            body(2, Vec3::new(2.0, 0.0, 0.0), true),
        ],
    )
    .unwrap();
    let mut world_b = world_a.clone();
    let first = PersistentConstraint::distance(
        ConstraintId::new(1),
        BodyId::new(1),
        BodyId::new(2),
        Vec3::ZERO,
        Vec3::ZERO,
        1.0,
        1.0e-4,
        0.0,
    );
    let second = PersistentConstraint::anchor(
        ConstraintId::new(2),
        BodyId::new(1),
        BodyId::new(2),
        Vec3::ZERO,
        Vec3::new(-1.0, 0.0, 0.0),
        1.0e-4,
        0.0,
    );
    world_a
        .set_constraints(vec![first.clone(), second.clone()])
        .unwrap();
    world_b.set_constraints(vec![second, first]).unwrap();

    world_a.substep(1.0 / 120.0, 1.0, options()).unwrap();
    world_b.substep(1.0 / 120.0, 1.0, options()).unwrap();

    assert_eq!(
        world_a.body(BodyId::new(2)).unwrap().pose,
        world_b.body(BodyId::new(2)).unwrap().pose
    );
}
