use bone_physics_core::{
    AngularLimit, BodyId, BodyInit, BoneBodyDesc, Capsule, CapsuleColliderDesc, ColliderId,
    CollisionFilter, ConstraintId, ContactMaterial, EulerOrder, PersistentConstraint, Pose,
    QualityPolicy, SolverConfig, SolverError, SolverWorld, StepOptions, Velocity, XyzLimit,
};
use glam::{Quat, Vec3};

const DT: f32 = 1.0 / 120.0;
const COLLIDER_RADIUS: f32 = 0.25;
const RESIDUAL_PENETRATION: f32 = 0.01 * COLLIDER_RADIUS;
const LOCKED_AXIS_ERROR: f32 = 1.0e-3;
const HARD_ANCHOR_ERROR: f32 = 1.0e-4;

fn body(id: u32, translation: Vec3, velocity: Vec3, dynamic: bool) -> BodyInit {
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
        velocity: Velocity {
            linear: velocity,
            angular: Vec3::ZERO,
        },
    }
}

fn world(gravity: Vec3, bodies: Vec<BodyInit>) -> SolverWorld {
    SolverWorld::new(
        SolverConfig {
            gravity,
            fixed_dt: DT,
        },
        bodies,
    )
    .unwrap()
}

fn options() -> StepOptions {
    StepOptions {
        position_iterations: 24,
        contact_iterations: 24,
        quality: QualityPolicy::AuthoritativeBake,
    }
}

fn capsule(
    id: u32,
    body: u32,
    local_a: Vec3,
    local_b: Vec3,
    material: ContactMaterial,
) -> CapsuleColliderDesc {
    CapsuleColliderDesc {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(local_a, local_b, COLLIDER_RADIUS, 0.0).unwrap(),
        material,
        filter: CollisionFilter::default(),
    }
}

fn ground(id: u32, body: u32, material: ContactMaterial) -> CapsuleColliderDesc {
    capsule(id, body, -Vec3::X * 4.0, Vec3::X * 4.0, material)
}

fn sphere(
    id: u32,
    body: u32,
    local_center: Vec3,
    material: ContactMaterial,
) -> CapsuleColliderDesc {
    capsule(id, body, local_center, local_center, material)
}

fn capsule_with_radius(
    id: u32,
    body: u32,
    local_a: Vec3,
    local_b: Vec3,
    radius: f32,
    material: ContactMaterial,
) -> CapsuleColliderDesc {
    CapsuleColliderDesc {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(local_a, local_b, radius, 0.0).unwrap(),
        material,
        filter: CollisionFilter::default(),
    }
}

#[test]
fn zero_restitution_resting_capsule_has_no_artificial_bounce_for_ten_seconds() {
    let material = ContactMaterial::try_new(0.8, 0.0).unwrap();
    let mut world = world(
        Vec3::new(0.0, -9.81, 0.0),
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::new(0.0, 0.49, 0.0), Vec3::ZERO, true),
        ],
    );
    world
        .set_colliders(vec![
            ground(10, 1, material),
            sphere(20, 2, Vec3::ZERO, material),
        ])
        .unwrap();

    let mut diagnostics = None;
    for _ in 0..(10 * 120) {
        diagnostics = Some(world.substep(DT, 1.0, options()).unwrap());
    }

    let diagnostics = diagnostics.unwrap();
    assert!(diagnostics.deepest_penetration <= RESIDUAL_PENETRATION);
    assert!(world.body(BodyId::new(2)).unwrap().velocity.linear.y.abs() <= 1.0e-3);
}

#[test]
fn friction_uses_the_moving_surface_velocity_not_world_zero() {
    let material = ContactMaterial::try_new(1.0, 0.0).unwrap();
    let mut world = world(
        Vec3::new(0.0, -9.81, 0.0),
        vec![
            body(1, Vec3::ZERO, Vec3::X, false),
            body(2, Vec3::new(0.0, 0.49, 0.0), Vec3::ZERO, true),
        ],
    );
    world
        .set_colliders(vec![
            ground(10, 1, material),
            sphere(20, 2, Vec3::ZERO, material),
        ])
        .unwrap();

    for _ in 0..60 {
        world.substep(DT, 1.0, options()).unwrap();
    }

    let surface_velocity = world.body(BodyId::new(1)).unwrap().velocity.linear.x;
    let body_velocity = world.body(BodyId::new(2)).unwrap().velocity.linear.x;
    assert!(body_velocity > 0.1);
    assert!(body_velocity < surface_velocity + 1.0e-3);
}

#[test]
fn one_new_closing_impact_applies_restitution_once() {
    let material = ContactMaterial::try_new(0.0, 0.5).unwrap();
    let mut world = world(
        Vec3::ZERO,
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::new(0.0, 0.51, 0.0), -Vec3::Y * 4.0, true),
        ],
    );
    world
        .set_colliders(vec![
            ground(10, 1, material),
            sphere(20, 2, Vec3::ZERO, material),
        ])
        .unwrap();

    world.substep(DT, 1.0, options()).unwrap();

    // For a static surface and a closing speed of 4 m/s, e = 0.5 gives +2 m/s.
    assert!((world.body(BodyId::new(2)).unwrap().velocity.linear.y - 2.0).abs() <= 1.0e-3);
}

#[test]
fn persistent_resting_contact_does_not_reapply_restitution() {
    let material = ContactMaterial::try_new(0.8, 0.9).unwrap();
    let mut world = world(
        Vec3::new(0.0, -9.81, 0.0),
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::new(0.0, 0.49, 0.0), Vec3::ZERO, true),
        ],
    );
    world
        .set_colliders(vec![
            ground(10, 1, material),
            sphere(20, 2, Vec3::ZERO, material),
        ])
        .unwrap();

    world.substep(DT, 1.0, options()).unwrap();
    world.substep(DT, 1.0, options()).unwrap();

    let contacts = world.cached_contacts();
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].1.age >= 2);
    assert!(world.body(BodyId::new(2)).unwrap().velocity.linear.y <= 1.0e-3);
}

#[test]
fn normal_contact_multipliers_are_never_attractive() {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let mut world = world(
        Vec3::ZERO,
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::new(0.0, 0.4, 0.0), Vec3::ZERO, true),
        ],
    );
    world
        .set_colliders(vec![
            ground(10, 1, material),
            sphere(20, 2, Vec3::ZERO, material),
        ])
        .unwrap();

    let diagnostics = world.substep(DT, 1.0, options()).unwrap();

    assert!(diagnostics.active_contacts > 0);
    assert_eq!(world.cached_contacts().len(), 1);
    assert!(
        world
            .cached_contacts()
            .iter()
            .all(|(_, contact)| contact.lambdas.normal >= 0.0)
    );
}

#[test]
fn contact_cache_is_permutation_stable_ages_persistent_contacts_and_expires_stale_ones() {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let bodies = vec![
        body(1, Vec3::ZERO, Vec3::ZERO, false),
        body(2, Vec3::new(0.0, 0.49, 0.0), Vec3::ZERO, true),
    ];
    let colliders = vec![ground(10, 1, material), sphere(20, 2, Vec3::ZERO, material)];
    let mut forward = world(Vec3::ZERO, bodies.clone());
    let mut reversed = world(Vec3::ZERO, bodies);
    forward.set_colliders(colliders.clone()).unwrap();
    reversed
        .set_colliders(colliders.iter().cloned().rev().collect())
        .unwrap();

    forward.substep(DT, 1.0, options()).unwrap();
    reversed.substep(DT, 1.0, options()).unwrap();
    assert_eq!(forward.cached_contacts(), reversed.cached_contacts());
    assert_eq!(forward.cached_contacts().len(), 1);
    assert_eq!(forward.cached_contacts()[0].1.age, 1);

    forward.substep(DT, 1.0, options()).unwrap();
    let contacts = forward.cached_contacts();
    assert_eq!(contacts.len(), 1);
    assert_eq!(contacts[0].1.age, 2);

    forward
        .set_colliders(vec![ground(10, 1, material)])
        .unwrap();
    assert!(forward.cached_contacts().is_empty());
    let diagnostics = forward.substep(DT, 1.0, options()).unwrap();
    assert_eq!(diagnostics.active_contacts, 0);
    assert!(forward.cached_contacts().is_empty());
}

#[test]
fn contact_and_angular_limit_share_every_full_solver_iteration() {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let mut world = world(
        Vec3::ZERO,
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::ZERO, Vec3::ZERO, true),
            body(3, Vec3::new(0.5, 0.25, 0.0), Vec3::ZERO, false),
        ],
    );
    let anchor_id = ConstraintId::new(20);
    let limit_id = ConstraintId::new(30);
    world
        .set_constraints(vec![
            PersistentConstraint::anchor(
                anchor_id,
                BodyId::new(1),
                BodyId::new(2),
                Vec3::ZERO,
                Vec3::ZERO,
                0.0,
                0.0,
            ),
            PersistentConstraint::angular_limit(
                limit_id,
                BodyId::new(1),
                BodyId::new(2),
                Quat::IDENTITY,
                AngularLimit::XyzBox(XyzLimit {
                    lower: Vec3::new(0.0, 0.0, -0.6),
                    upper: Vec3::ZERO,
                    order: EulerOrder::Xyz,
                }),
                0.0,
                0.0,
            ),
        ])
        .unwrap();
    world
        .set_colliders(vec![
            sphere(20, 2, Vec3::X * 0.5, material),
            sphere(30, 3, Vec3::ZERO, material),
        ])
        .unwrap();

    let mut options = options();
    options.position_iterations = 48;
    options.contact_iterations = 48;
    let diagnostics = world.substep(DT, 1.0, options).unwrap();

    assert!(diagnostics.generated_contacts > 0);
    let anchor_error = world.anchor_error(anchor_id).unwrap();
    let limit_error = world.anchor_error(limit_id).unwrap();
    let rotation_z = world
        .body(BodyId::new(2))
        .unwrap()
        .pose
        .rotation
        .to_scaled_axis()
        .z;
    assert!(
        diagnostics.deepest_penetration <= RESIDUAL_PENETRATION,
        "penetration={}, anchor_error={anchor_error}, limit_error={limit_error}, rotation_z={rotation_z}",
        diagnostics.deepest_penetration
    );
    assert!(anchor_error <= HARD_ANCHOR_ERROR);
    assert!(limit_error <= LOCKED_AXIS_ERROR);
    assert!(rotation_z < -0.1);
}

#[test]
fn parallel_manifold_feature_slots_survive_one_two_one_transition() {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    // B advances exactly one meter per substep. Its segment is therefore
    // [-1, 0] at the first left-endpoint touch, [0, 1] at full overlap, and
    // [1, 2] at the final right-endpoint touch. The 0.2 m centerline spacing
    // equals the sum of the two 0.1 m radii, so no normal impulse can disguise
    // a feature-slot mismatch.
    let bodies = vec![
        body(1, Vec3::ZERO, Vec3::ZERO, false),
        body(2, Vec3::new(-2.0, 0.2, 0.0), Vec3::X * 120.0, false),
    ];
    let colliders = vec![
        capsule_with_radius(10, 1, Vec3::ZERO, Vec3::X, 0.1, material),
        capsule_with_radius(20, 2, Vec3::ZERO, Vec3::X, 0.1, material),
    ];
    let mut forward = world(Vec3::ZERO, bodies.clone());
    let mut reversed = world(Vec3::ZERO, bodies);
    forward.set_colliders(colliders.clone()).unwrap();
    reversed
        .set_colliders(colliders.iter().cloned().rev().collect())
        .unwrap();

    let first_forward = forward.substep(DT, 1.0, options()).unwrap();
    let first_reversed = reversed.substep(DT, 1.0, options()).unwrap();
    assert!(first_forward.deepest_penetration <= 1.0e-6);
    assert!(first_reversed.deepest_penetration <= 1.0e-6);
    let first = forward.cached_contacts();
    assert_eq!(first, reversed.cached_contacts());
    assert_eq!(first.len(), 1);
    assert_eq!(first[0].1.age, 1);
    assert_eq!(first[0].1.lambdas.normal, 0.0);

    let middle_forward = forward.substep(DT, 1.0, options()).unwrap();
    let middle_reversed = reversed.substep(DT, 1.0, options()).unwrap();
    assert!(middle_forward.deepest_penetration <= 1.0e-6);
    assert!(middle_reversed.deepest_penetration <= 1.0e-6);
    let middle = forward.cached_contacts();
    assert_eq!(middle, reversed.cached_contacts());
    assert_eq!(middle.len(), 2);
    assert!(middle[0].0 < middle[1].0);
    assert!(middle[0].1.local_a.x < middle[1].1.local_a.x);
    assert_eq!(middle[0].0, first[0].0);
    assert_eq!(middle[0].1.age, 2);
    assert_eq!(middle[1].1.age, 1);
    assert_eq!(middle[1].1.lambdas.normal, 0.0);

    let final_forward = forward.substep(DT, 1.0, options()).unwrap();
    let final_reversed = reversed.substep(DT, 1.0, options()).unwrap();
    assert!(final_forward.deepest_penetration <= 1.0e-6);
    assert!(final_reversed.deepest_penetration <= 1.0e-6);
    let final_contacts = forward.cached_contacts();
    assert_eq!(final_contacts, reversed.cached_contacts());
    assert_eq!(final_contacts.len(), 1);
    assert_eq!(final_contacts[0].0, middle[1].0);
    assert_ne!(final_contacts[0].0, first[0].0);
    assert_eq!(final_contacts[0].1.age, middle[1].1.age + 1);
    assert_eq!(final_contacts[0].1.lambdas, middle[1].1.lambdas);
}

#[test]
fn unequal_parallel_capsules_keep_two_same_half_feature_slots_distinct() {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    // Canonical collider A spans [0, 10]. Canonical collider B advances from
    // [-1, 0] to [0, 1], so the manifold transitions from one point to two
    // points while both two-point anchors remain in A's first half. The
    // centerline spacing equals the 0.2 m radius sum, isolating cache identity
    // from normal response.
    let bodies = vec![
        body(1, Vec3::ZERO, Vec3::ZERO, false),
        body(2, Vec3::new(-2.0, 0.2, 0.0), Vec3::X * 120.0, false),
    ];
    let colliders = vec![
        capsule_with_radius(10, 1, Vec3::ZERO, Vec3::X * 10.0, 0.1, material),
        capsule_with_radius(20, 2, Vec3::ZERO, Vec3::X, 0.1, material),
    ];
    let mut forward = world(Vec3::ZERO, bodies.clone());
    let mut reversed = world(Vec3::ZERO, bodies);
    forward.set_colliders(colliders.clone()).unwrap();
    reversed
        .set_colliders(colliders.iter().cloned().rev().collect())
        .unwrap();

    let first_forward = forward.substep(DT, 1.0, options()).unwrap();
    let first_reversed = reversed.substep(DT, 1.0, options()).unwrap();
    assert!(first_forward.deepest_penetration <= 1.0e-6);
    assert!(first_reversed.deepest_penetration <= 1.0e-6);
    let first = forward.cached_contacts();
    assert_eq!(first, reversed.cached_contacts());
    assert_eq!(first.len(), 1);
    assert_eq!(first[0].1.age, 1);
    assert_eq!(first[0].1.lambdas, Default::default());

    let second_forward = forward.substep(DT, 1.0, options()).unwrap();
    let second_reversed = reversed.substep(DT, 1.0, options()).unwrap();
    assert!(second_forward.deepest_penetration <= 1.0e-6);
    assert!(second_reversed.deepest_penetration <= 1.0e-6);
    let second = forward.cached_contacts();
    assert_eq!(second, reversed.cached_contacts());
    assert_eq!(second.len(), 2);
    assert!(second[0].0 < second[1].0);
    assert!(second[0].1.local_a.x < 5.0);
    assert!(second[1].1.local_a.x < 5.0);
    assert_eq!(second[0].0, first[0].0);
    assert_eq!(second[0].1.age, first[0].1.age + 1);
    assert_eq!(second[0].1.lambdas, first[0].1.lambdas);
    assert_eq!(second[1].1.age, 1);
    assert_eq!(second[1].1.lambdas, Default::default());
}

fn near_parallel_boundary_world(
    reverse_endpoints: bool,
    reverse_registration: bool,
) -> SolverWorld {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let angle = (0.02_f32 / 1.5).asin();
    let mut rotating = body(2, Vec3::new(7.5, 0.199, 0.0), Vec3::ZERO, false);
    rotating.pose = Pose::try_new(
        rotating.pose.translation,
        Quat::from_rotation_z(2.0 * angle),
    )
    .unwrap();
    rotating.velocity.angular = -Vec3::Z * (angle / DT);

    let (a0, a1) = if reverse_endpoints {
        (Vec3::X * 10.0, Vec3::ZERO)
    } else {
        (Vec3::ZERO, Vec3::X * 10.0)
    };
    let (b0, b1) = if reverse_endpoints {
        (Vec3::X * 1.5, -Vec3::X * 1.5)
    } else {
        (-Vec3::X * 1.5, Vec3::X * 1.5)
    };
    let mut colliders = vec![
        capsule_with_radius(10, 1, a0, a1, 0.1, material),
        capsule_with_radius(20, 2, b0, b1, 0.1, material),
    ];
    if reverse_registration {
        colliders.reverse();
    }

    let mut result = world(
        Vec3::ZERO,
        vec![body(1, Vec3::ZERO, Vec3::ZERO, false), rotating],
    );
    result.set_colliders(colliders).unwrap();
    result
}

#[test]
fn near_parallel_conditional_boundary_keeps_each_physical_slot() {
    // With theta = asin(0.02 / 1.5), B's predicted endpoint heights progress
    // from [0.179, 0.219] through [0.199, 0.199] to [0.219, 0.179].
    // Against the 0.2 m radius sum this admits the lower x=6 boundary, then
    // both x=6/x=9 boundaries, then only the upper x=9 boundary.
    let mut forward = near_parallel_boundary_world(false, false);
    let mut reversed_registration = near_parallel_boundary_world(false, true);

    forward.substep(DT, 1.0, options()).unwrap();
    reversed_registration.substep(DT, 1.0, options()).unwrap();
    let first = forward.cached_contacts();
    assert_eq!(first, reversed_registration.cached_contacts());
    assert_eq!(first.len(), 1);
    assert!((first[0].1.local_a.x - 6.0).abs() <= 1.0e-3);
    assert_eq!(first[0].1.age, 1);
    assert_eq!(first[0].1.lambdas, Default::default());

    forward.substep(DT, 1.0, options()).unwrap();
    reversed_registration.substep(DT, 1.0, options()).unwrap();
    let second = forward.cached_contacts();
    assert_eq!(second, reversed_registration.cached_contacts());
    assert_eq!(second.len(), 2);
    assert!(second[0].0 < second[1].0);
    let (lower, upper) = if second[0].1.local_a.x < second[1].1.local_a.x {
        (&second[0], &second[1])
    } else {
        (&second[1], &second[0])
    };
    assert!((lower.1.local_a.x - 6.0).abs() <= 1.0e-3);
    assert!((upper.1.local_a.x - 9.0).abs() <= 1.0e-3);
    assert_eq!(lower.0, first[0].0);
    assert_eq!(lower.1.age, first[0].1.age + 1);
    assert_eq!(lower.1.lambdas, first[0].1.lambdas);
    assert_eq!(upper.1.age, 1);
    assert_eq!(upper.1.lambdas, Default::default());

    forward.substep(DT, 1.0, options()).unwrap();
    reversed_registration.substep(DT, 1.0, options()).unwrap();
    let third = forward.cached_contacts();
    assert_eq!(third, reversed_registration.cached_contacts());
    assert_eq!(third.len(), 1);
    assert!((third[0].1.local_a.x - 9.0).abs() <= 1.0e-3);
    assert_eq!(third[0].0, upper.0);
    assert_eq!(third[0].1.age, upper.1.age + 1);
    assert_eq!(third[0].1.lambdas, upper.1.lambdas);
}

#[test]
fn near_parallel_contact_slots_ignore_local_endpoint_order() {
    let mut canonical = near_parallel_boundary_world(false, false);
    let mut registration_reversed = near_parallel_boundary_world(false, true);
    let mut endpoints_reversed = near_parallel_boundary_world(true, false);
    let mut both_reversed = near_parallel_boundary_world(true, true);

    for expected_count in [1, 2, 1] {
        for world in [
            &mut canonical,
            &mut registration_reversed,
            &mut endpoints_reversed,
            &mut both_reversed,
        ] {
            world.substep(DT, 1.0, options()).unwrap();
        }

        let expected = canonical.cached_contacts();
        assert_eq!(expected.len(), expected_count);
        assert_eq!(registration_reversed.cached_contacts(), expected);
        assert_eq!(endpoints_reversed.cached_contacts(), expected);
        assert_eq!(both_reversed.cached_contacts(), expected);
    }
}

#[derive(Clone, Copy)]
enum FarContactTransition {
    OneToOne,
    OneToTwo,
    TwoToOne,
    TwoToTwo,
}

impl FarContactTransition {
    fn geometry(self) -> (usize, usize, f32, f32, f32) {
        match self {
            Self::OneToOne => (1, 1, 10.0, -12.0, 11.0),
            Self::OneToTwo => (1, 2, 100.0, -51.0, 50.0),
            Self::TwoToOne => (2, 1, 100.0, -100.0, 100.0),
            Self::TwoToTwo => (2, 2, 100.0, -50.0, 50.0),
        }
    }
}

fn far_contact_transition_world(
    transition: FarContactTransition,
    reverse_endpoints: bool,
    reverse_registration: bool,
) -> SolverWorld {
    let (_, _, length_a, initial_b, step_b) = transition.geometry();
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let (a0, a1) = if reverse_endpoints {
        (Vec3::X * length_a, Vec3::ZERO)
    } else {
        (Vec3::ZERO, Vec3::X * length_a)
    };
    let (b0, b1) = if reverse_endpoints {
        (Vec3::X, Vec3::ZERO)
    } else {
        (Vec3::ZERO, Vec3::X)
    };
    let mut colliders = vec![
        capsule_with_radius(10, 1, a0, a1, 0.1, material),
        capsule_with_radius(20, 2, b0, b1, 0.1, material),
    ];
    if reverse_registration {
        colliders.reverse();
    }

    let mut result = world(
        Vec3::ZERO,
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(
                2,
                Vec3::new(initial_b, 0.2, 0.0),
                Vec3::X * (step_b / DT),
                false,
            ),
        ],
    );
    result.set_colliders(colliders).unwrap();
    result
}

fn assert_far_contact_transition_resets_cache(transition: FarContactTransition) {
    let (first_count, second_count, _, _, _) = transition.geometry();
    let mut worlds = [
        far_contact_transition_world(transition, false, false),
        far_contact_transition_world(transition, false, true),
        far_contact_transition_world(transition, true, false),
        far_contact_transition_world(transition, true, true),
    ];

    for world in &mut worlds {
        let diagnostics = world.substep(DT, 1.0, options()).unwrap();
        assert!(diagnostics.deepest_penetration <= 1.0e-6);
    }
    let first = worlds[0].cached_contacts();
    assert_eq!(first.len(), first_count);
    assert!(first.windows(2).all(|pair| pair[0].0 < pair[1].0));
    assert!(
        first
            .iter()
            .all(|(_, contact)| contact.age == 1 && contact.lambdas == Default::default())
    );
    for world in &worlds[1..] {
        assert_eq!(world.cached_contacts(), first);
    }

    for world in &mut worlds {
        let diagnostics = world.substep(DT, 1.0, options()).unwrap();
        assert!(diagnostics.deepest_penetration <= 1.0e-6);
    }
    let second = worlds[0].cached_contacts();
    assert_eq!(second.len(), second_count);
    assert!(second.windows(2).all(|pair| pair[0].0 < pair[1].0));
    assert!(
        second
            .iter()
            .all(|(_, contact)| contact.age == 1 && contact.lambdas == Default::default())
    );
    assert!(second.iter().all(|(_, current)| {
        first
            .iter()
            .all(|(_, previous)| current.local_a.distance(previous.local_a) > 5.0)
    }));
    assert!(
        second
            .iter()
            .all(|(key, _)| first.iter().all(|(previous_key, _)| key != previous_key))
    );
    for world in &worlds[1..] {
        assert_eq!(world.cached_contacts(), second);
    }
}

#[test]
fn direct_far_endpoint_jump_starts_a_new_contact_feature() {
    assert_far_contact_transition_resets_cache(FarContactTransition::OneToOne);
}

#[test]
fn far_disjoint_cardinality_transitions_do_not_warm_start_unrelated_anchors() {
    for transition in [
        FarContactTransition::OneToTwo,
        FarContactTransition::TwoToOne,
        FarContactTransition::TwoToTwo,
    ] {
        assert_far_contact_transition_resets_cache(transition);
    }
}

#[test]
fn final_diagnostics_drop_a_generated_contact_separated_by_a_constraint() {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let mut world = world(
        Vec3::ZERO,
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::Y * 0.4, Vec3::ZERO, true),
        ],
    );
    world
        .set_constraints(vec![PersistentConstraint::distance(
            ConstraintId::new(30),
            BodyId::new(1),
            BodyId::new(2),
            Vec3::ZERO,
            Vec3::ZERO,
            1.0,
            0.0,
            0.0,
        )])
        .unwrap();
    world
        .set_colliders(vec![
            sphere(10, 1, Vec3::ZERO, material),
            sphere(20, 2, Vec3::ZERO, material),
        ])
        .unwrap();
    let mut options = options();
    options.position_iterations = 1;
    options.contact_iterations = 1;

    let diagnostics = world.substep(DT, 1.0, options).unwrap();

    assert!(diagnostics.generated_contacts > 0);
    assert_eq!(diagnostics.active_contacts, 0);
    assert_eq!(diagnostics.deepest_penetration, 0.0);
}

#[test]
fn final_diagnostics_include_contact_created_by_a_constraint() {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let mut world = world(
        Vec3::ZERO,
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::Y, Vec3::ZERO, true),
        ],
    );
    world
        .set_constraints(vec![PersistentConstraint::distance(
            ConstraintId::new(30),
            BodyId::new(1),
            BodyId::new(2),
            Vec3::ZERO,
            Vec3::ZERO,
            0.25,
            0.0,
            0.0,
        )])
        .unwrap();
    world
        .set_colliders(vec![
            sphere(10, 1, Vec3::ZERO, material),
            sphere(20, 2, Vec3::ZERO, material),
        ])
        .unwrap();
    let mut options = options();
    options.position_iterations = 1;
    options.contact_iterations = 1;

    let diagnostics = world.substep(DT, 1.0, options).unwrap();

    assert_eq!(diagnostics.generated_contacts, 0);
    assert!(diagnostics.active_contacts > 0);
    assert!(diagnostics.deepest_penetration > 0.0);
}

fn seeded_static_contact_world() -> (SolverWorld, Vec<CapsuleColliderDesc>) {
    let material = ContactMaterial::try_new(0.0, 0.0).unwrap();
    let mut world = world(
        Vec3::ZERO,
        vec![
            body(1, Vec3::ZERO, Vec3::ZERO, false),
            body(2, Vec3::Y * 0.4, Vec3::ZERO, false),
            body(3, Vec3::Y * 4.0, Vec3::ZERO, false),
        ],
    );
    let colliders = vec![
        sphere(10, 1, Vec3::ZERO, material),
        sphere(20, 2, Vec3::ZERO, material),
    ];
    world.set_colliders(colliders.clone()).unwrap();
    let diagnostics = world.substep(DT, 1.0, options()).unwrap();
    assert!(diagnostics.active_contacts > 0);
    assert_eq!(world.cached_contacts().len(), 1);
    (world, colliders)
}

fn assert_rejected_collider_set_preserves_world(
    world: &mut SolverWorld,
    colliders: Vec<CapsuleColliderDesc>,
    original: &[CapsuleColliderDesc],
    expected: SolverError,
) {
    let cached_before = world.cached_contacts();
    assert_eq!(world.set_colliders(colliders).unwrap_err(), expected);
    assert_eq!(world.cached_contacts(), cached_before);
    world.set_colliders(original.to_vec()).unwrap();
    assert_eq!(world.cached_contacts(), cached_before);

    let diagnostics = world.substep(DT, 1.0, options()).unwrap();
    assert!(diagnostics.generated_contacts > 0);
    assert!(diagnostics.active_contacts > 0);
    assert_eq!(world.cached_contacts().len(), 1);
}

#[test]
fn invalid_collider_replacements_are_atomic_and_preserve_prior_behavior() {
    let (mut world, colliders) = seeded_static_contact_world();

    let mut duplicate = colliders.clone();
    duplicate[1].id = duplicate[0].id;
    assert_rejected_collider_set_preserves_world(
        &mut world,
        duplicate,
        &colliders,
        SolverError::DuplicateId {
            kind: "collider",
            id: 10,
        },
    );

    let mut missing_body = colliders.clone();
    missing_body[1].body = BodyId::new(999);
    assert_rejected_collider_set_preserves_world(
        &mut world,
        missing_body,
        &colliders,
        SolverError::MissingBody(999),
    );

    let mut missing_excluded_body = colliders.clone();
    missing_excluded_body[1]
        .filter
        .excluded_bodies
        .insert(BodyId::new(998));
    assert_rejected_collider_set_preserves_world(
        &mut world,
        missing_excluded_body,
        &colliders,
        SolverError::MissingBody(998),
    );

    let mut missing_excluded_constraint = colliders.clone();
    missing_excluded_constraint[1]
        .filter
        .excluded_constraints
        .insert(ConstraintId::new(997));
    assert_rejected_collider_set_preserves_world(
        &mut world,
        missing_excluded_constraint,
        &colliders,
        SolverError::InvalidValue {
            field: "collision_filter_constraint",
            value: 997.0,
        },
    );

    let mut invalid_material = colliders.clone();
    invalid_material[1].material = ContactMaterial {
        friction: -1.0,
        restitution: 0.0,
    };
    assert_rejected_collider_set_preserves_world(
        &mut world,
        invalid_material,
        &colliders,
        SolverError::InvalidValue {
            field: "contact_friction",
            value: -1.0,
        },
    );

    let mut non_finite_material = colliders.clone();
    non_finite_material[1].material = ContactMaterial {
        friction: f32::INFINITY,
        restitution: 0.0,
    };
    assert_rejected_collider_set_preserves_world(
        &mut world,
        non_finite_material,
        &colliders,
        SolverError::InvalidValue {
            field: "contact_friction",
            value: f64::INFINITY,
        },
    );

    let mut invalid_shape = colliders.clone();
    invalid_shape[1].shape = Capsule {
        local_a: Vec3::ZERO,
        local_b: Vec3::ZERO,
        radius: 0.0,
        margin: 0.0,
    };
    assert_rejected_collider_set_preserves_world(
        &mut world,
        invalid_shape,
        &colliders,
        SolverError::InvalidValue {
            field: "capsule_radius",
            value: 0.0,
        },
    );

    let mut non_finite_geometry = colliders.clone();
    non_finite_geometry[1].shape = Capsule {
        local_a: Vec3::new(f32::INFINITY, 0.0, 0.0),
        local_b: Vec3::ZERO,
        radius: COLLIDER_RADIUS,
        margin: 0.0,
    };
    assert_rejected_collider_set_preserves_world(
        &mut world,
        non_finite_geometry,
        &colliders,
        SolverError::InvalidValue {
            field: "capsule_local_a",
            value: f64::INFINITY,
        },
    );
}

fn reseed_contact_cache(world: &mut SolverWorld, colliders: &[CapsuleColliderDesc]) {
    world.set_colliders(colliders.to_vec()).unwrap();
    let diagnostics = world.substep(DT, 1.0, options()).unwrap();
    assert!(diagnostics.active_contacts > 0);
    assert_eq!(world.cached_contacts().len(), 1);
}

#[test]
fn changed_collider_metadata_invalidates_affected_cached_contacts() {
    let (mut world, colliders) = seeded_static_contact_world();

    let mut changed_shape = colliders.clone();
    changed_shape[1].shape =
        Capsule::try_new(Vec3::ZERO, Vec3::ZERO, COLLIDER_RADIUS * 0.75, 0.0).unwrap();
    world.set_colliders(changed_shape).unwrap();
    assert!(world.cached_contacts().is_empty());

    reseed_contact_cache(&mut world, &colliders);
    let mut changed_body = colliders.clone();
    changed_body[1].body = BodyId::new(3);
    world.set_colliders(changed_body).unwrap();
    assert!(world.cached_contacts().is_empty());

    reseed_contact_cache(&mut world, &colliders);
    let mut changed_material = colliders.clone();
    changed_material[1].material = ContactMaterial::try_new(1.0, 0.0).unwrap();
    world.set_colliders(changed_material).unwrap();
    assert!(world.cached_contacts().is_empty());

    reseed_contact_cache(&mut world, &colliders);
    let mut changed_filter = colliders;
    changed_filter[1].filter.mask = 0;
    world.set_colliders(changed_filter).unwrap();
    assert!(world.cached_contacts().is_empty());
}
