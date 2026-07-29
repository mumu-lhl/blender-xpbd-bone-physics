use bone_physics_core::{
    BodyId, Capsule, CapsuleCollider, ColliderId, CollisionFilter, ContactGeometry, DeformingMesh,
    MeshContactFeature, MeshContactGeometry, MeshContactKey, MeshId, MeshManifoldLimits,
    MeshTopology, Pose, RegionId, TriangleFeature, TriangleId, capsule_triangle_contact,
    cluster_mesh_contacts,
};
use glam::Vec3;

fn raw(triangle: u32, position: Vec3, normal: Vec3, penetration: f32) -> MeshContactGeometry {
    let feature = MeshContactFeature {
        capsule: 1,
        triangle: TriangleFeature::Face,
    };
    MeshContactGeometry {
        key: MeshContactKey {
            collider: ColliderId::new(3),
            mesh: MeshId::new(5),
            triangle: TriangleId::new(triangle),
            feature: feature.encode(),
        },
        source_triangle: TriangleId::new(triangle),
        geometry: ContactGeometry {
            point_a: position,
            point_b: position - normal * penetration,
            normal,
            penetration,
        },
        barycentric: Vec3::splat(1.0 / 3.0),
        segment_t: 0.5,
        surface_velocity: Vec3::ZERO,
        feature,
    }
}

#[test]
fn dense_plane_clusters_to_four_stable_contacts_under_small_deformation() {
    let limits = MeshManifoldLimits::try_new(4, 0.08, 0.95).unwrap();
    let anchors = [
        Vec3::new(-0.4, -0.2, 0.0),
        Vec3::new(0.4, -0.2, 0.0),
        Vec3::new(-0.4, 0.2, 0.0),
        Vec3::new(0.4, 0.2, 0.0),
    ];
    let mut contacts = Vec::new();
    for (cluster, anchor) in anchors.into_iter().enumerate() {
        for sample in 0..5 {
            contacts.push(raw(
                (cluster * 10 + sample) as u32,
                anchor + Vec3::X * sample as f32 * 0.005,
                Vec3::Z,
                0.02 + sample as f32 * 0.001,
            ));
        }
    }

    let first = cluster_mesh_contacts(contacts.clone(), limits).unwrap();
    let deformed = contacts
        .into_iter()
        .map(|mut contact| {
            contact.geometry.point_a += Vec3::new(1.0e-4, -2.0e-4, 0.0);
            contact.geometry.point_b += Vec3::new(1.0e-4, -2.0e-4, 0.0);
            contact
        })
        .collect();
    let second = cluster_mesh_contacts(deformed, limits).unwrap();

    assert_eq!(first.len(), 4);
    assert_eq!(
        first.iter().map(|contact| contact.key).collect::<Vec<_>>(),
        second.iter().map(|contact| contact.key).collect::<Vec<_>>()
    );
    assert!(first.windows(2).all(|pair| pair[0].key < pair[1].key));
}

#[test]
fn convex_crease_preserves_materially_different_normals() {
    let limits = MeshManifoldLimits::try_new(4, 0.1, 0.95).unwrap();
    let contacts = vec![
        raw(1, Vec3::ZERO, Vec3::Z, 0.1),
        raw(2, Vec3::new(0.01, 0.0, 0.0), Vec3::Z, 0.09),
        raw(3, Vec3::new(0.0, 0.01, 0.0), Vec3::X, 0.08),
        raw(4, Vec3::new(0.0, 0.02, 0.0), Vec3::X, 0.07),
    ];

    let manifold = cluster_mesh_contacts(contacts, limits).unwrap();

    assert_eq!(manifold.len(), 2);
    assert!(
        manifold
            .iter()
            .any(|contact| contact.geometry.normal == Vec3::Z)
    );
    assert!(
        manifold
            .iter()
            .any(|contact| contact.geometry.normal == Vec3::X)
    );
}

#[test]
fn shared_feature_owner_never_replaces_the_source_triangle_geometry() {
    let positions = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    ];
    let mesh = DeformingMesh::build(
        MeshId::new(5),
        MeshTopology {
            vertex_count: 4,
            triangles: vec![[0, 1, 2], [0, 2, 3]],
            triangle_regions: vec![RegionId::new(1); 2],
            closed_hint: false,
        },
        positions,
        0.0,
    )
    .unwrap();
    let capsule = CapsuleCollider {
        id: ColliderId::new(3),
        body: BodyId::new(1),
        shape: Capsule::try_new(Vec3::new(0.5, 0.5, 0.1), Vec3::new(0.5, 0.5, 0.1), 0.2, 0.0)
            .unwrap(),
        pose: Pose::IDENTITY,
        linear_velocity: Vec3::ZERO,
        filter: CollisionFilter::default(),
    };
    let triangle = mesh
        .moving_triangle(TriangleId::new(1), 1.0 / 60.0)
        .unwrap();

    let contact = capsule_triangle_contact(&capsule, &triangle, None)
        .unwrap()
        .unwrap();

    assert_eq!(contact.key.triangle, TriangleId::new(0));
    assert_eq!(contact.source_triangle, TriangleId::new(1));
}

#[test]
fn whole_capsule_over_dense_tessellation_has_a_stable_four_point_manifold() {
    let capsule = CapsuleCollider {
        id: ColliderId::new(3),
        body: BodyId::new(1),
        shape: Capsule::try_new(
            Vec3::new(-0.8, 0.0, 0.1),
            Vec3::new(0.8, 0.0, 0.1),
            0.2,
            0.0,
        )
        .unwrap(),
        pose: Pose::IDENTITY,
        linear_velocity: Vec3::ZERO,
        filter: CollisionFilter::default(),
    };
    let raw_contacts = |deformation: f32| {
        let mut positions = Vec::new();
        for y in 0..=4 {
            for x in 0..=4 {
                let x = -1.0 + x as f32 * 0.5;
                let y = -1.0 + y as f32 * 0.5;
                positions.push(Vec3::new(x, y, deformation * (x - y)));
            }
        }
        let mut triangles = Vec::new();
        for y in 0..4 {
            for x in 0..4 {
                let lower_left = y * 5 + x;
                let lower_right = lower_left + 1;
                let upper_left = lower_left + 5;
                let upper_right = upper_left + 1;
                triangles.extend([
                    [lower_left, lower_right, upper_right],
                    [lower_left, upper_right, upper_left],
                ]);
            }
        }
        let topology = MeshTopology {
            vertex_count: positions.len() as u32,
            triangle_regions: vec![RegionId::new(1); triangles.len()],
            triangles,
            closed_hint: false,
        };
        let mesh =
            DeformingMesh::build(MeshId::new(5), topology, positions, 0.0).expect("valid grid");
        let mut raw = Vec::new();
        for triangle_id in mesh.active_triangle_ids() {
            let triangle = mesh.moving_triangle(*triangle_id, 1.0 / 60.0).unwrap();
            if let Some(contact) = capsule_triangle_contact(&capsule, &triangle, None).unwrap() {
                raw.push(contact);
            }
        }
        raw
    };
    let limits = MeshManifoldLimits::try_new(4, 0.3, 0.95).unwrap();

    let first = cluster_mesh_contacts(raw_contacts(1.0e-3), limits).unwrap();
    let second = cluster_mesh_contacts(raw_contacts(1.01e-3), limits).unwrap();

    assert!(!first.is_empty());
    assert!(first.len() <= 4);
    assert_eq!(
        first.iter().map(|contact| contact.key).collect::<Vec<_>>(),
        second.iter().map(|contact| contact.key).collect::<Vec<_>>()
    );
}

#[test]
fn limits_and_raw_non_finite_values_are_rejected() {
    assert!(MeshManifoldLimits::try_new(0, 0.1, 0.9).is_err());
    assert!(MeshManifoldLimits::try_new(4, -0.1, 0.9).is_err());
    assert!(MeshManifoldLimits::try_new(4, 0.1, 1.1).is_err());

    let mut invalid = raw(1, Vec3::ZERO, Vec3::Z, 0.1);
    invalid.geometry.normal.x = f32::NAN;
    assert!(
        cluster_mesh_contacts(
            vec![invalid],
            MeshManifoldLimits::try_new(4, 0.1, 0.9).unwrap(),
        )
        .is_err()
    );
}
