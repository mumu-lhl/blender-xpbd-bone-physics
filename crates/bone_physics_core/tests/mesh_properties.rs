use bone_physics_core::{DeformingMesh, MeshId, MeshTopology, MeshUpdate, RegionId};
use glam::Vec3;

fn octahedron() -> (MeshTopology, Vec<Vec3>) {
    let positions = vec![Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y, Vec3::Z, -Vec3::Z];
    let triangles = vec![
        [0, 2, 4],
        [2, 1, 4],
        [1, 3, 4],
        [3, 0, 4],
        [2, 0, 5],
        [1, 2, 5],
        [3, 1, 5],
        [0, 3, 5],
    ];
    (
        MeshTopology {
            vertex_count: positions.len() as u32,
            triangle_regions: vec![RegionId::new(11); triangles.len()],
            triangles,
            closed_hint: true,
        },
        positions,
    )
}

#[test]
fn one_hundred_refits_match_full_rebuild_queries() {
    let (topology, initial) = octahedron();
    let mut previous = initial.clone();
    let mut refitted =
        DeformingMesh::build(MeshId::new(1), topology.clone(), initial, 0.025).unwrap();

    for frame in 1..=100 {
        let time = frame as f32 * 0.037;
        let current = previous
            .iter()
            .enumerate()
            .map(|(index, position)| {
                let phase = time + index as f32 * 0.173;
                Vec3::new(
                    position.x * (1.0 + 0.1 * phase.sin()) + 0.2 * time.sin(),
                    position.y * (1.0 + 0.08 * phase.cos()),
                    position.z + 0.15 * (phase * 0.7).sin(),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(
            refitted
                .update_vertices(previous.clone(), current.clone())
                .unwrap(),
            MeshUpdate::Refit
        );
        let rebuilt = DeformingMesh::rebuild(
            MeshId::new(1),
            topology.clone(),
            previous.clone(),
            current.clone(),
            0.025,
        )
        .unwrap();

        for query in 0..12 {
            let center = Vec3::new(
                -1.2 + query as f32 * 0.22,
                0.35 * (time + query as f32).sin(),
                0.25 * (time * 0.5 + query as f32).cos(),
            );
            let extent = Vec3::splat(0.18 + query as f32 * 0.01);
            assert_eq!(
                refitted
                    .query_aabb(center - extent, center + extent)
                    .unwrap(),
                rebuilt
                    .query_aabb(center - extent, center + extent)
                    .unwrap(),
                "frame {frame}, query {query}"
            );
        }
        previous = current;
    }
}
