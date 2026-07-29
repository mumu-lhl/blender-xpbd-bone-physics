use bone_physics_core::{
    DeformingMesh, MeshId, MeshTopology, MeshUpdate, MeshValidationIssue, RegionId, TriangleId,
};
use glam::Vec3;

fn tetrahedron() -> (MeshTopology, Vec<Vec3>) {
    let positions = vec![
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(-1.0, -1.0, 1.0),
        Vec3::new(-1.0, 1.0, -1.0),
        Vec3::new(1.0, -1.0, -1.0),
    ];
    let triangles = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    let topology = MeshTopology {
        vertex_count: 4,
        triangle_regions: vec![RegionId::new(7); triangles.len()],
        triangles,
        closed_hint: true,
    };
    (topology, positions)
}

#[test]
fn stable_ids_and_vertex_only_refit_preserve_owned_topology() {
    let (topology, positions) = tetrahedron();
    let mut mesh =
        DeformingMesh::build(MeshId::new(9), topology.clone(), positions.clone(), 0.02).unwrap();

    assert!(mesh.validation_report().errors.is_empty());
    assert!(mesh.validation_report().warnings.is_empty());
    assert_eq!(mesh.topology(), &topology);
    assert_eq!(
        mesh.active_triangle_ids(),
        &[
            TriangleId::new(0),
            TriangleId::new(1),
            TriangleId::new(2),
            TriangleId::new(3)
        ]
    );

    let current = positions
        .iter()
        .map(|position| *position + Vec3::new(0.25, -0.5, 0.75))
        .collect::<Vec<_>>();
    assert_eq!(
        mesh.update_vertices(positions.clone(), current.clone())
            .unwrap(),
        MeshUpdate::Refit
    );
    assert_eq!(mesh.previous_positions(), positions);
    assert_eq!(mesh.current_positions(), current);
    assert_eq!(mesh.topology(), &topology);
}

#[test]
fn fatal_shape_and_vertex_errors_are_reported_before_build() {
    let topology = MeshTopology {
        vertex_count: 3,
        triangles: vec![[0, 1, 3]],
        triangle_regions: vec![RegionId::new(1)],
        closed_hint: false,
    };
    let positions = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
    let report = topology.validate(&positions);
    assert!(report.errors.iter().any(|issue| matches!(
        issue,
        MeshValidationIssue::VertexIndexOutOfRange {
            triangle,
            vertex: 3
        } if *triangle == TriangleId::new(0)
    )));

    let (_, mut non_finite) = tetrahedron();
    non_finite[2].z = f32::NAN;
    let (valid_topology, _) = tetrahedron();
    let report = valid_topology.validate(&non_finite);
    assert!(
        report
            .errors
            .iter()
            .any(|issue| matches!(issue, MeshValidationIssue::NonFiniteVertex { vertex: 2 }))
    );

    let mut wrong_regions = valid_topology;
    wrong_regions.triangle_regions.pop();
    let report = wrong_regions.validate(&non_finite);
    assert!(report.errors.iter().any(|issue| matches!(
        issue,
        MeshValidationIssue::TriangleRegionCountMismatch { .. }
    )));
}

#[test]
fn recoverable_faces_and_adjacency_defects_are_warnings() {
    let (mut topology, mut positions) = tetrahedron();
    positions.push(Vec3::new(0.0, 3.0, 0.0));
    topology.vertex_count = positions.len() as u32;
    topology.closed_hint = true;
    topology.triangles.extend([[0, 0, 1], [1, 2, 0], [0, 2, 4]]);
    topology
        .triangle_regions
        .extend([RegionId::new(8), RegionId::new(9), RegionId::new(10)]);

    let report = topology.validate(&positions);

    assert!(report.errors.is_empty());
    assert!(report.warnings.iter().any(|issue| matches!(
        issue,
        MeshValidationIssue::DegenerateTriangle { triangle }
            if *triangle == TriangleId::new(4)
    )));
    assert!(report.warnings.iter().any(|issue| matches!(
        issue,
        MeshValidationIssue::DuplicateTriangle { triangle, .. }
            if *triangle == TriangleId::new(5)
    )));
    assert!(report.warnings.iter().any(|issue| matches!(
        issue,
        MeshValidationIssue::NonManifoldEdge { .. }
            | MeshValidationIssue::InconsistentEdgeOrientation { .. }
    )));

    let mesh = DeformingMesh::build(MeshId::new(1), topology, positions, 0.0).unwrap();
    assert_eq!(mesh.active_triangle_ids().len(), 5);
}

#[test]
fn edge_warnings_distinguish_open_orientation_and_nonmanifold_cases() {
    let positions = vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z, -Vec3::Y];
    let topology = |triangles: Vec<[u32; 3]>, closed_hint| MeshTopology {
        vertex_count: positions.len() as u32,
        triangle_regions: vec![RegionId::new(1); triangles.len()],
        triangles,
        closed_hint,
    };

    let open = topology(vec![[0, 1, 2]], true).validate(&positions);
    assert!(
        open.warnings
            .iter()
            .any(|issue| matches!(issue, MeshValidationIssue::BoundaryEdge { .. }))
    );

    let inconsistent = topology(vec![[0, 1, 2], [0, 1, 3]], false).validate(&positions);
    assert!(inconsistent.warnings.iter().any(|issue| matches!(
        issue,
        MeshValidationIssue::InconsistentEdgeOrientation { vertices: [0, 1] }
    )));

    let nonmanifold = topology(vec![[0, 1, 2], [1, 0, 3], [0, 1, 4]], false).validate(&positions);
    assert!(nonmanifold.warnings.iter().any(|issue| matches!(
        issue,
        MeshValidationIssue::NonManifoldEdge {
            vertices: [0, 1],
            incident_triangles: 3
        }
    )));
}

#[test]
fn topology_hash_is_deterministic_and_covers_regions_and_closed_hint() {
    let (topology, positions) = tetrahedron();
    let baseline = topology.validate(&positions).topology_hash;
    assert_eq!(
        baseline,
        topology.clone().validate(&positions).topology_hash
    );

    let mut changed_region = topology.clone();
    changed_region.triangle_regions[0] = RegionId::new(99);
    assert_ne!(baseline, changed_region.validate(&positions).topology_hash);

    let mut changed_hint = topology;
    changed_hint.closed_hint = false;
    assert_ne!(baseline, changed_hint.validate(&positions).topology_hash);
}

#[test]
fn invalid_vertex_update_is_atomic() {
    let (topology, positions) = tetrahedron();
    let mut mesh = DeformingMesh::build(MeshId::new(4), topology, positions.clone(), 0.01).unwrap();
    let before_candidates = mesh
        .query_aabb(Vec3::splat(-2.0), Vec3::splat(2.0))
        .unwrap();
    let mut invalid = positions.clone();
    invalid[0].x = f32::INFINITY;

    assert!(
        mesh.update_vertices(positions.clone(), invalid).is_err(),
        "non-finite updates must fail before mutating the BVH or position history"
    );

    assert_eq!(mesh.previous_positions(), positions);
    assert_eq!(mesh.current_positions(), positions);
    assert_eq!(
        mesh.query_aabb(Vec3::splat(-2.0), Vec3::splat(2.0))
            .unwrap(),
        before_candidates
    );
}

#[test]
fn a_mesh_without_any_valid_triangle_is_rejected() {
    let topology = MeshTopology {
        vertex_count: 3,
        triangles: vec![[0, 0, 1]],
        triangle_regions: vec![RegionId::new(1)],
        closed_hint: false,
    };
    let positions = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
    let report = topology.validate(&positions);
    assert!(
        report
            .errors
            .iter()
            .any(|issue| matches!(issue, MeshValidationIssue::NoValidTriangles))
    );
    assert!(DeformingMesh::build(MeshId::new(3), topology, positions, 0.0).is_err());
}

#[test]
fn query_results_use_stable_triangle_ids() {
    let positions = vec![
        Vec3::ZERO,
        Vec3::X,
        Vec3::Y,
        Vec3::new(10.0, 0.0, 0.0),
        Vec3::new(11.0, 0.0, 0.0),
        Vec3::new(10.0, 1.0, 0.0),
    ];
    let topology = MeshTopology {
        vertex_count: positions.len() as u32,
        triangles: vec![[0, 0, 1], [0, 1, 2], [3, 4, 5]],
        triangle_regions: vec![RegionId::new(1); 3],
        closed_hint: false,
    };
    let mesh = DeformingMesh::build(MeshId::new(5), topology, positions, 0.01).unwrap();

    let ids = mesh
        .query_aabb(Vec3::splat(-0.1), Vec3::splat(1.1))
        .unwrap();

    assert_eq!(ids, vec![TriangleId::new(1)]);
    let owners = mesh
        .moving_triangle(TriangleId::new(1), 1.0 / 60.0)
        .unwrap()
        .feature_owners
        .unwrap();
    assert!(
        owners
            .vertices
            .iter()
            .all(|owner| owner.triangle == TriangleId::new(1)),
        "invalid triangles must never own persistent contact features"
    );
}
