use bone_physics_core::{
    capsule_capsule_toi, capsule_triangle_toi, Capsule, CapsuleCollider, ColliderId,
    CollisionFilter, CcdResult, MovingTriangle, BodyId, MeshId, Pose, RegionId, TriangleId,
};
use glam::Vec3;

fn collider(id: u32, start: Vec3, end: Vec3, radius: f32) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(id),
        body: BodyId::new(id),
        shape: Capsule::try_new(Vec3::ZERO, Vec3::ZERO, radius, 0.0).unwrap(),
        pose: Pose::try_new(start, glam::Quat::IDENTITY).unwrap(),
        linear_velocity: (end - start),
        filter: CollisionFilter::default(),
    }
}

fn at(collider: &CapsuleCollider, translation: Vec3) -> CapsuleCollider {
    let mut value = collider.clone();
    value.pose = Pose::try_new(translation, glam::Quat::IDENTITY).unwrap();
    value
}

fn triangle() -> MovingTriangle {
    MovingTriangle {
        mesh: MeshId::new(4),
        triangle: TriangleId::new(5),
        region: RegionId::new(6),
        vertices: [
            Vec3::new(-2.0, -2.0, 0.0),
            Vec3::new(2.0, -2.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ],
        velocities: [Vec3::ZERO; 3],
        thickness: 0.0,
        two_sided: true,
        feature_owners: None,
    }
}

#[test]
fn capsule_capsule_query_returns_earliest_impact() {
    let a0 = collider(1, Vec3::new(-2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), 0.2);
    let a1 = at(&a0, Vec3::new(2.0, 0.0, 0.0));
    let b0 = collider(2, Vec3::ZERO, Vec3::ZERO, 0.2);
    let b1 = at(&b0, Vec3::ZERO);

    let result = capsule_capsule_toi(&a0, &a1, &b0, &b1, 1.0).unwrap();
    let CcdResult::Impact { toi, .. } = result else {
        panic!("expected capsule impact");
    };
    assert!((toi - 0.4).abs() < 2.0e-3, "toi={toi}");
}

#[test]
fn capsule_triangle_query_handles_moving_capsule_and_zero_time_contact() {
    let start = collider(1, Vec3::new(0.0, 0.0, -1.0), Vec3::new(0.0, 0.0, 1.0), 0.1);
    let end = at(&start, Vec3::new(0.0, 0.0, 1.0));
    let result = capsule_triangle_toi(&start, &end, &triangle(), 1.0).unwrap();
    let CcdResult::Impact { toi, .. } = result else {
        panic!("expected triangle impact");
    };
    assert!((toi - 0.45).abs() < 2.0e-3, "toi={toi}");

    let touching = at(&start, Vec3::new(0.0, 0.0, -0.1));
    let result = capsule_triangle_toi(&touching, &touching, &triangle(), 1.0).unwrap();
    let CcdResult::Impact { toi, .. } = result else {
        panic!("expected zero-time impact");
    };
    assert_eq!(toi, 0.0);
}

#[test]
fn separated_grazing_motion_reports_no_impact_and_never_nonfinite_toi() {
    let a0 = collider(1, Vec3::new(-2.0, 1.0, 0.0), Vec3::new(2.0, 1.0, 0.0), 0.1);
    let a1 = at(&a0, Vec3::new(2.0, 1.0, 0.0));
    let b0 = collider(2, Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 2.0, 0.0), 0.1);
    let b1 = at(&b0, Vec3::new(0.0, 2.0, 0.0));
    let result = capsule_capsule_toi(&a0, &a1, &b0, &b1, 1.0).unwrap();
    assert!(matches!(result, CcdResult::NoImpact));
}
