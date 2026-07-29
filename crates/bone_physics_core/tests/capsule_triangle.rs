use bone_physics_core::{
    BodyId, Capsule, CapsuleCollider, ColliderId, CollisionFilter, MeshContactHistory, MeshId,
    MovingTriangle, Pose, RegionId, TriangleFeature, TriangleId, capsule_triangle_contact,
};
use glam::Vec3;

fn capsule(a: Vec3, b: Vec3, radius: f32, velocity: Vec3) -> CapsuleCollider {
    CapsuleCollider {
        id: ColliderId::new(7),
        body: BodyId::new(3),
        shape: Capsule::try_new(a, b, radius, 0.0).unwrap(),
        pose: Pose::IDENTITY,
        linear_velocity: velocity,
        filter: CollisionFilter::default(),
    }
}

fn triangle(vertices: [Vec3; 3], velocities: [Vec3; 3]) -> MovingTriangle {
    MovingTriangle {
        mesh: MeshId::new(11),
        triangle: TriangleId::new(13),
        region: RegionId::new(17),
        vertices,
        velocities,
        thickness: 0.0,
        two_sided: true,
        feature_owners: None,
    }
}

fn plane() -> MovingTriangle {
    triangle(
        [
            Vec3::new(-2.0, -2.0, 0.0),
            Vec3::new(2.0, -2.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ],
        [Vec3::X, Vec3::Y, Vec3::Z],
    )
}

fn assert_valid(contact: &bone_physics_core::MeshContactGeometry) {
    assert!(contact.geometry.point_a.is_finite());
    assert!(contact.geometry.point_b.is_finite());
    assert!(contact.geometry.normal.is_finite());
    assert!(contact.geometry.penetration.is_finite());
    assert!(contact.barycentric.is_finite());
    assert!((contact.barycentric.element_sum() - 1.0).abs() <= 1.0e-5);
    assert!(contact.barycentric.min_element() >= -1.0e-5);
    assert!(contact.surface_velocity.is_finite());
    assert!(contact.segment_t.is_finite());
    assert!((0.0..=1.0).contains(&contact.segment_t));
}

#[test]
fn whole_capsule_detects_side_and_both_caps_against_a_face() {
    for (a, b, expected_t) in [
        (Vec3::new(-0.5, 0.0, 0.2), Vec3::new(0.5, 0.0, 0.2), 0.5),
        (Vec3::new(0.0, 0.0, 0.2), Vec3::Z, 0.0),
        (-Vec3::Z, Vec3::new(0.0, 0.0, -0.2), 1.0),
    ] {
        let contact = capsule_triangle_contact(&capsule(a, b, 0.25, Vec3::ZERO), &plane(), None)
            .unwrap()
            .unwrap();
        assert_valid(&contact);
        assert!((contact.segment_t - expected_t).abs() <= 1.0e-4);
        assert_eq!(contact.feature.triangle, TriangleFeature::Face);
        assert!((contact.geometry.penetration - 0.05).abs() <= 1.0e-4);
    }
}

#[test]
fn triangle_edge_and_vertex_features_are_stable() {
    let edge = capsule_triangle_contact(
        &capsule(
            Vec3::new(-0.5, -2.1, 0.1),
            Vec3::new(0.5, -2.1, 0.1),
            0.2,
            Vec3::ZERO,
        ),
        &plane(),
        None,
    )
    .unwrap()
    .unwrap();
    assert_valid(&edge);
    assert!(matches!(edge.feature.triangle, TriangleFeature::Edge(_)));

    let vertex = capsule_triangle_contact(
        &capsule(
            Vec3::new(-2.1, -2.1, 0.0),
            Vec3::new(-2.1, -2.1, 0.3),
            0.2,
            Vec3::ZERO,
        ),
        &plane(),
        None,
    )
    .unwrap()
    .unwrap();
    assert_valid(&vertex);
    assert_eq!(vertex.feature.triangle, TriangleFeature::Vertex(0));
}

#[test]
fn exact_coincidence_and_coplanar_motion_use_finite_history_or_velocity_normals() {
    let history = MeshContactHistory { normal: -Vec3::Z };
    let coincident = capsule_triangle_contact(
        &capsule(-Vec3::X * 0.5, Vec3::X * 0.5, 0.1, Vec3::ZERO),
        &plane(),
        Some(history),
    )
    .unwrap()
    .unwrap();
    assert_valid(&coincident);
    assert_eq!(coincident.geometry.normal, -Vec3::Z);

    let stationary_plane = triangle(plane().vertices, [Vec3::ZERO; 3]);
    let moving = capsule_triangle_contact(
        &capsule(-Vec3::X * 0.5, Vec3::X * 0.5, 0.1, Vec3::Z),
        &stationary_plane,
        None,
    )
    .unwrap()
    .unwrap();
    assert_valid(&moving);
    assert!(moving.geometry.normal.dot(Vec3::Z).abs() > 0.99);
}

#[test]
fn barycentric_surface_velocity_is_interpolated() {
    let contact = capsule_triangle_contact(
        &capsule(
            Vec3::new(0.0, -2.0 / 3.0, 0.1),
            Vec3::new(0.0, -2.0 / 3.0, 0.2),
            0.2,
            Vec3::ZERO,
        ),
        &plane(),
        None,
    )
    .unwrap()
    .unwrap();
    assert_valid(&contact);
    assert!((contact.barycentric - Vec3::splat(1.0 / 3.0)).length() <= 1.0e-4);
    assert!((contact.surface_velocity - Vec3::splat(1.0 / 3.0)).length() <= 1.0e-4);
}

#[test]
fn reversed_winding_is_symmetric_when_two_sided_history_selects_the_side() {
    let capsule = capsule(
        Vec3::new(-0.5, 0.0, 0.2),
        Vec3::new(0.5, 0.0, 0.2),
        0.25,
        Vec3::ZERO,
    );
    let forward = plane();
    let mut reversed = forward;
    reversed.vertices.swap(1, 2);
    reversed.velocities.swap(1, 2);
    let history = Some(MeshContactHistory { normal: -Vec3::Z });

    let a = capsule_triangle_contact(&capsule, &forward, history)
        .unwrap()
        .unwrap();
    let b = capsule_triangle_contact(&capsule, &reversed, history)
        .unwrap()
        .unwrap();

    assert!((a.geometry.point_a - b.geometry.point_a).length() <= 1.0e-5);
    assert!((a.geometry.point_b - b.geometry.point_b).length() <= 1.0e-5);
    assert!((a.geometry.normal - b.geometry.normal).length() <= 1.0e-5);
    assert!((a.surface_velocity - b.surface_velocity).length() <= 1.0e-5);
}

#[test]
fn degenerate_triangle_is_rejected_and_separated_geometry_has_no_contact() {
    let degenerate = triangle([Vec3::ZERO, Vec3::X, Vec3::X * 2.0], [Vec3::ZERO; 3]);
    assert!(
        capsule_triangle_contact(
            &capsule(Vec3::ZERO, Vec3::Y, 0.2, Vec3::ZERO),
            &degenerate,
            None,
        )
        .is_err()
    );

    let separated = capsule_triangle_contact(
        &capsule(
            Vec3::new(0.0, 0.0, 2.0),
            Vec3::new(0.0, 1.0, 2.0),
            0.2,
            Vec3::ZERO,
        ),
        &plane(),
        None,
    )
    .unwrap();
    assert!(separated.is_none());
}
