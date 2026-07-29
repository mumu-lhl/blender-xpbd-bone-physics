use std::collections::BTreeSet;

use bone_physics_core::{
    AngularLimit, BodyId, BodyInit, BoneBodyDesc, ConstraintClass, ConstraintId,
    ConstraintSchedule, EulerOrder, PersistentConstraint, Pose, QualityPolicy, SolverConfig,
    SolverWorld, StepOptions, Velocity, XyzLimit,
};
use glam::{Quat, Vec3};

fn distance(id: u32, body_a: u32, body_b: u32) -> PersistentConstraint {
    PersistentConstraint::distance(
        ConstraintId::new(id),
        BodyId::new(body_a),
        BodyId::new(body_b),
        Vec3::ZERO,
        Vec3::ZERO,
        1.0,
        0.0,
        0.0,
    )
}

fn representative_constraints() -> Vec<PersistentConstraint> {
    vec![
        PersistentConstraint::anchor(
            ConstraintId::new(40),
            BodyId::new(1),
            BodyId::new(2),
            Vec3::ZERO,
            Vec3::ZERO,
            0.0,
            0.0,
        ),
        PersistentConstraint::orientation_recovery(ConstraintId::new(20), BodyId::new(3), 0.0, 0.0),
        distance(30, 2, 3),
        PersistentConstraint::angular_limit(
            ConstraintId::new(10),
            BodyId::new(1),
            BodyId::new(3),
            Quat::IDENTITY,
            AngularLimit::XyzBox(XyzLimit {
                lower: -Vec3::ONE,
                upper: Vec3::ONE,
                order: EulerOrder::Xyz,
            }),
            0.0,
            0.0,
        ),
    ]
}

#[test]
fn schedule_is_stable_under_input_permutation_and_rejects_duplicate_ids() {
    let constraints = representative_constraints();
    let shuffled = vec![
        constraints[2].clone(),
        constraints[0].clone(),
        constraints[3].clone(),
        constraints[1].clone(),
    ];

    let expected = ConstraintSchedule::build(&constraints).unwrap();
    let actual = ConstraintSchedule::build(&shuffled).unwrap();

    assert_eq!(actual, expected);
    let mut duplicate = constraints;
    duplicate.push(distance(40, 4, 5));
    assert!(ConstraintSchedule::build(&duplicate).is_err());
}

#[test]
fn greedy_colors_are_stable_and_never_write_one_body_twice() {
    let constraints = vec![
        distance(1, 1, 2),
        distance(2, 3, 4),
        distance(3, 2, 3),
        distance(4, 4, 5),
    ];
    let schedule = ConstraintSchedule::build(&constraints).unwrap();
    let colors = schedule.colors(ConstraintClass::Structural);

    assert_eq!(colors.len(), 2);
    assert_eq!(
        colors[0].constraint_ids(),
        &[ConstraintId::new(1), ConstraintId::new(2)]
    );
    assert_eq!(
        colors[1].constraint_ids(),
        &[ConstraintId::new(3), ConstraintId::new(4)]
    );

    for color in colors {
        let mut written = BTreeSet::new();
        for id in color.constraint_ids() {
            let constraint = constraints
                .iter()
                .find(|constraint| constraint.id == *id)
                .unwrap();
            assert!(written.insert(constraint.body_a));
            if let Some(body_b) = constraint.body_b {
                assert!(written.insert(body_b));
            }
        }
    }
}

#[test]
fn class_order_is_fixed_while_each_class_alternates_direction() {
    let mut constraints = representative_constraints();
    constraints.extend([distance(31, 4, 5), distance(32, 3, 4), distance(33, 5, 6)]);
    let schedule = ConstraintSchedule::build(&constraints).unwrap();

    assert_eq!(
        schedule
            .iteration_order(0)
            .iter()
            .map(|(class, id)| (*class, id.get()))
            .collect::<Vec<_>>(),
        vec![
            (ConstraintClass::Anchor, 40),
            (ConstraintClass::Recovery, 20),
            (ConstraintClass::Structural, 30),
            (ConstraintClass::Structural, 31),
            (ConstraintClass::Structural, 32),
            (ConstraintClass::Structural, 33),
            (ConstraintClass::AngularLimit, 10),
        ]
    );
    assert_eq!(
        schedule
            .iteration_order(1)
            .iter()
            .map(|(class, id)| (*class, id.get()))
            .collect::<Vec<_>>(),
        vec![
            (ConstraintClass::Anchor, 40),
            (ConstraintClass::Recovery, 20),
            (ConstraintClass::Structural, 33),
            (ConstraintClass::Structural, 32),
            (ConstraintClass::Structural, 31),
            (ConstraintClass::Structural, 30),
            (ConstraintClass::AngularLimit, 10),
        ]
    );
    assert!(schedule.colors(ConstraintClass::Contact).is_empty());
}

fn body(id: u32, x: f32, dynamic: bool) -> BodyInit {
    let inverse = if dynamic { 1.0 } else { 0.0 };
    BodyInit {
        desc: BoneBodyDesc {
            id: BodyId::new(id),
            rest_pose: Pose::IDENTITY,
            local_center_of_mass: Vec3::ZERO,
            local_head: Vec3::ZERO,
            local_tail: Vec3::Y,
            inverse_mass: inverse,
            inverse_inertia: Vec3::splat(inverse),
            gravity_scale: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
        },
        pose: Pose::try_new(Vec3::new(x, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        velocity: Velocity::ZERO,
    }
}

fn class_order_world(constraints: Vec<PersistentConstraint>) -> SolverWorld {
    let mut world = SolverWorld::new(
        SolverConfig {
            gravity: Vec3::ZERO,
            fixed_dt: 1.0 / 120.0,
        },
        vec![body(1, 0.0, false), body(2, 2.0, true), body(3, 4.0, true)],
    )
    .unwrap();
    world.set_constraints(constraints).unwrap();
    world
}

#[test]
fn world_uses_class_schedule_instead_of_global_constraint_id_order() {
    let anchor = PersistentConstraint::anchor(
        ConstraintId::new(100),
        BodyId::new(1),
        BodyId::new(2),
        Vec3::ZERO,
        Vec3::ZERO,
        0.0,
        0.0,
    );
    let structural = PersistentConstraint::distance(
        ConstraintId::new(1),
        BodyId::new(2),
        BodyId::new(3),
        Vec3::ZERO,
        Vec3::ZERO,
        0.0,
        0.0,
        0.0,
    );
    let mut world_a = class_order_world(vec![structural.clone(), anchor.clone()]);
    let mut world_b = class_order_world(vec![anchor, structural]);
    let options = StepOptions {
        position_iterations: 1,
        contact_iterations: 0,
        quality: QualityPolicy::AuthoritativeBake,
    };

    world_a.substep(1.0 / 120.0, 1.0, options).unwrap();
    world_b.substep(1.0 / 120.0, 1.0, options).unwrap();

    let body_c_a = world_a.body(BodyId::new(3)).unwrap().pose.translation.x;
    let body_c_b = world_b.body(BodyId::new(3)).unwrap().pose.translation.x;
    assert!((body_c_a - 2.0).abs() < 1.0e-6, "body_c={body_c_a}");
    assert_eq!(body_c_a, body_c_b);
}

#[test]
fn public_constraint_classification_matches_solver_phases() {
    for (constraint, expected) in representative_constraints().into_iter().zip([
        ConstraintClass::Anchor,
        ConstraintClass::Recovery,
        ConstraintClass::Structural,
        ConstraintClass::AngularLimit,
    ]) {
        let id = constraint.id;
        let schedule = ConstraintSchedule::build(&[constraint]).unwrap();
        assert_eq!(
            schedule.colors(expected)[0].constraint_ids(),
            &[id],
            "class={expected:?}"
        );
    }
}
