use bone_physics_core::{
    BodyId, Capsule, CapsuleCollider, ColliderId, CollisionFilter, PairKey, Pose, capsule_manifold,
};
use glam::{Quat, Vec3};

fn collider(
    id: u32,
    body: u32,
    local_a: Vec3,
    local_b: Vec3,
    radius: f32,
    translation: Vec3,
    linear_velocity: Vec3,
) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(id),
        body: BodyId::new(body),
        shape: Capsule::try_new(local_a, local_b, radius, 0.0).unwrap(),
        pose: Pose::try_new(translation, Quat::IDENTITY).unwrap(),
        linear_velocity,
        filter: CollisionFilter::default(),
    }
}

fn sphere(id: u32, x: f32, radius: f32) -> CapsuleCollider {
    collider(
        id,
        id,
        Vec3::ZERO,
        Vec3::ZERO,
        radius,
        Vec3::new(x, 0.0, 0.0),
        Vec3::ZERO,
    )
}

fn assert_finite_manifold(manifold: &bone_physics_core::CapsuleManifold) {
    assert!(!manifold.points.is_empty());
    assert!(manifold.points.len() <= 2);
    for point in &manifold.points {
        assert!(point.point_a.is_finite());
        assert!(point.point_b.is_finite());
        assert!(point.normal.is_finite());
        assert!((point.normal.length() - 1.0).abs() < 1.0e-5);
        assert!(point.penetration.is_finite());
        assert!(point.penetration >= 0.0);
    }
}

#[test]
fn separated_touching_and_penetrating_capsules_have_explicit_semantics() {
    let a = sphere(1, 0.0, 0.5);
    let separated = sphere(2, 1.1, 0.5);
    let touching = sphere(2, 1.0, 0.5);
    let penetrating = sphere(2, 0.75, 0.5);

    assert!(capsule_manifold(&a, &separated, None).unwrap().is_none());

    let touching = capsule_manifold(&a, &touching, None).unwrap().unwrap();
    assert_finite_manifold(&touching);
    assert!(touching.points[0].penetration.abs() < 1.0e-6);

    let penetrating = capsule_manifold(&a, &penetrating, None).unwrap().unwrap();
    assert_finite_manifold(&penetrating);
    assert!((penetrating.points[0].penetration - 0.25).abs() < 1.0e-6);
}

#[test]
fn crossing_capsules_use_axis_cross_product_for_a_finite_normal() {
    let a = collider(10, 10, -Vec3::X, Vec3::X, 0.2, Vec3::ZERO, Vec3::ZERO);
    let b = collider(11, 11, -Vec3::Y, Vec3::Y, 0.2, Vec3::ZERO, Vec3::ZERO);

    let manifold = capsule_manifold(&a, &b, None).unwrap().unwrap();

    assert_finite_manifold(&manifold);
    assert_eq!(manifold.points.len(), 1);
    assert!(manifold.points[0].normal.dot(Vec3::Z).abs() > 0.999);
    assert!((manifold.points[0].penetration - 0.4).abs() < 1.0e-6);
}

#[test]
fn coincidence_uses_cached_normal_before_relative_velocity_or_ids() {
    let mut a = collider(20, 20, -Vec3::X, Vec3::X, 0.25, Vec3::ZERO, Vec3::Z);
    let b = collider(21, 21, -Vec3::X, Vec3::X, 0.25, Vec3::ZERO, -Vec3::Z);
    a.linear_velocity = Vec3::Z;

    let manifold = capsule_manifold(&a, &b, Some(Vec3::Y)).unwrap().unwrap();

    assert_finite_manifold(&manifold);
    assert_eq!(manifold.points.len(), 2);
    for point in manifold.points {
        assert!(point.normal.dot(Vec3::Y) > 0.999);
        assert!((point.penetration - 0.5).abs() < 1.0e-6);
    }
}

#[test]
fn coincident_cached_normal_is_canonical_under_argument_reversal() {
    let a = collider(72, 72, -Vec3::X, Vec3::X, 0.25, Vec3::ZERO, Vec3::Z);
    let b = collider(19, 19, -Vec3::X, Vec3::X, 0.25, Vec3::ZERO, -Vec3::Z);

    let forward = capsule_manifold(&a, &b, Some(Vec3::Y)).unwrap().unwrap();
    let reverse = capsule_manifold(&b, &a, Some(Vec3::Y)).unwrap().unwrap();

    assert_eq!(
        forward.pair,
        PairKey::ordered(ColliderId::new(19), ColliderId::new(72)).unwrap()
    );
    assert_eq!(forward, reverse);
    for point in forward.points {
        assert!(point.normal.dot(Vec3::Y) > 0.999);
    }
}

#[test]
fn zero_length_coincidence_falls_back_to_velocity_then_stable_ids() {
    let stationary = sphere(30, 0.0, 0.25);
    let mut moving = sphere(31, 0.0, 0.25);
    moving.linear_velocity = Vec3::Z;

    let velocity_normal = capsule_manifold(&stationary, &moving, None)
        .unwrap()
        .unwrap();
    assert!(velocity_normal.points[0].normal.dot(Vec3::Z) > 0.999);

    moving.linear_velocity = Vec3::ZERO;
    let id_normal = capsule_manifold(&stationary, &moving, None)
        .unwrap()
        .unwrap();
    assert_finite_manifold(&id_normal);
}

#[test]
fn long_parallel_overlap_emits_two_stable_contacts() {
    let a = collider(
        40,
        40,
        Vec3::new(-2.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        0.4,
        Vec3::ZERO,
        Vec3::ZERO,
    );
    let b = collider(
        41,
        41,
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.4,
        Vec3::new(0.0, 0.5, 0.0),
        Vec3::ZERO,
    );

    let manifold = capsule_manifold(&a, &b, None).unwrap().unwrap();

    assert_finite_manifold(&manifold);
    assert_eq!(manifold.points.len(), 2);
    assert!((manifold.points[0].penetration - 0.3).abs() < 1.0e-6);
    assert!((manifold.points[1].penetration - 0.3).abs() < 1.0e-6);
    assert!(
        manifold.points[0]
            .point_a
            .distance(manifold.points[1].point_a)
            > 1.9
    );
}

#[test]
fn pair_and_manifold_are_canonical_under_argument_reversal() {
    let a = sphere(90, 0.0, 0.75);
    let b = sphere(70, 1.0, 0.75);

    let forward = capsule_manifold(&a, &b, None).unwrap().unwrap();
    let reverse = capsule_manifold(&b, &a, None).unwrap().unwrap();

    assert_eq!(
        forward.pair,
        PairKey::ordered(ColliderId::new(70), ColliderId::new(90)).unwrap()
    );
    assert_eq!(forward, reverse);
}

#[test]
fn capsules_reject_invalid_dimensions_but_allow_zero_length_segments() {
    assert!(Capsule::try_new(Vec3::ZERO, Vec3::ZERO, 0.25, 0.0).is_ok());
    assert!(Capsule::try_new(Vec3::ZERO, Vec3::X, -0.25, 0.0).is_err());
    assert!(Capsule::try_new(Vec3::ZERO, Vec3::X, 0.25, f32::NAN).is_err());
    assert!(Capsule::try_new(Vec3::ZERO, Vec3::splat(f32::INFINITY), 0.25, 0.0).is_err());
    assert!(PairKey::ordered(ColliderId::new(1), ColliderId::new(1)).is_err());
}
