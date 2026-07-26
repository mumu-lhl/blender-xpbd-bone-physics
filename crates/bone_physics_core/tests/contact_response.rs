use bone_physics_core::{
    AngularLimit, BodyId, BodyInit, BoneBodyDesc, Capsule, CapsuleColliderDesc, ColliderId,
    CollisionFilter, ContactMaterial, ConstraintId, EulerOrder, PersistentConstraint, Pose,
    QualityPolicy, SolverConfig, SolverWorld, StepOptions, Velocity, XyzLimit,
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
        .set_colliders(vec![ground(10, 1, material), sphere(20, 2, Vec3::ZERO, material)])
        .unwrap();

    let mut diagnostics = None;
    for _ in 0..(10 * 120) {
        diagnostics = Some(world.substep(DT, 1.0, options()).unwrap());
    }

    let diagnostics = diagnostics.unwrap();
    assert!(diagnostics.deepest_penetration <= RESIDUAL_PENETRATION);
    assert!(
        world.body(BodyId::new(2)).unwrap().velocity.linear.y.abs() <= 1.0e-3
    );
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
        .set_colliders(vec![ground(10, 1, material), sphere(20, 2, Vec3::ZERO, material)])
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
        .set_colliders(vec![ground(10, 1, material), sphere(20, 2, Vec3::ZERO, material)])
        .unwrap();

    world.substep(DT, 1.0, options()).unwrap();

    // For a static surface and a closing speed of 4 m/s, e = 0.5 gives +2 m/s.
    assert!(
        (world.body(BodyId::new(2)).unwrap().velocity.linear.y - 2.0).abs() <= 1.0e-3
    );
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
        .set_colliders(vec![ground(10, 1, material), sphere(20, 2, Vec3::ZERO, material)])
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
        .set_colliders(vec![ground(10, 1, material), sphere(20, 2, Vec3::ZERO, material)])
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

    forward.set_colliders(vec![ground(10, 1, material)]).unwrap();
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

    assert!(diagnostics.active_contacts > 0);
    assert!(diagnostics.deepest_penetration <= RESIDUAL_PENETRATION);
    assert!(world.anchor_error(anchor_id).unwrap() <= HARD_ANCHOR_ERROR);
    assert!(world.anchor_error(limit_id).unwrap() <= LOCKED_AXIS_ERROR);
    assert!(world.body(BodyId::new(2)).unwrap().pose.rotation.to_scaled_axis().z < -0.1);
}
