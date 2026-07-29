import unittest

from blender_extension.bone_physics.mesh_builder import (
    MeshSource,
    build_collision_surface,
)
from blender_extension.bone_physics.model import CollisionSurfaceSpec


class MeshBuilderTests(unittest.TestCase):
    def test_triangulates_filters_and_bakes_nonuniform_transform_deterministically(self):
        source = MeshSource(
            vertices=(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (1.0, 1.0, 0.0),
                (0.0, 1.0, 0.0),
            ),
            polygons=((0, 1, 2, 3), (0, 0, 1)),
            polygon_regions=(7, 8),
            polygon_materials=("skin", "skin"),
            object_to_simulation=(
                2.0, 0.0, 0.0, 3.0,
                0.0, 3.0, 0.0, -2.0,
                0.0, 0.0, 4.0, 1.0,
                0.0, 0.0, 0.0, 1.0,
            ),
        )
        policy = CollisionSurfaceSpec(id=5, object_name="Body", default_region_id=1)

        first = build_collision_surface(source, policy)
        second = build_collision_surface(source, policy)

        self.assertEqual(first, second)
        self.assertEqual(first.topology.triangles, ((0, 1, 2), (0, 2, 3)))
        self.assertEqual(first.regions, (7, 7))
        self.assertEqual(first.current_vertices[0], (3.0, -2.0, 1.0))
        self.assertEqual(first.current_vertices[2], (5.0, 1.0, 1.0))
        self.assertTrue(first.validation.discarded_degenerate_faces >= 1)
        self.assertEqual(first.previous_vertices, first.current_vertices)

    def test_excluded_material_duplicate_and_non_finite_geometry_are_handled(self):
        policy = CollisionSurfaceSpec(
            id=8,
            object_name="Body",
            excluded_materials=("eyes",),
        )
        source = MeshSource(
            vertices=((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
            polygons=((0, 1, 2), (2, 1, 0), (0, 1, 3)),
            polygon_regions=(1, 1, 2),
            polygon_materials=("skin", "skin", "eyes"),
        )
        batch = build_collision_surface(source, policy)
        self.assertEqual(batch.topology.triangles, ((0, 1, 2),))
        self.assertEqual(batch.validation.discarded_duplicate_faces, 1)
        self.assertEqual(batch.validation.discarded_excluded_faces, 1)

        with self.assertRaisesRegex(ValueError, "finite"):
            build_collision_surface(
                MeshSource(
                    vertices=((0, 0, 0), (float("nan"), 0, 0), (0, 1, 0)),
                    polygons=((0, 1, 2),),
                ),
                policy,
            )

    def test_previous_vertices_and_topology_fingerprints_are_explicit(self):
        policy = CollisionSurfaceSpec(id=9, object_name="Body", thickness=0.02)
        previous = MeshSource(
            vertices=((0, 0, 0), (1, 0, 0), (0, 1, 0)),
            polygons=((0, 1, 2),),
        )
        current = MeshSource(
            vertices=((0, 0, 0.1), (1, 0, 0.1), (0, 1, 0.1)),
            polygons=((0, 1, 2),),
        )
        batch = build_collision_surface(current, policy, previous=previous)
        self.assertNotEqual(batch.previous_vertices, batch.current_vertices)
        self.assertEqual(batch.topology_fingerprint, build_collision_surface(previous, policy).topology_fingerprint)
        self.assertNotEqual(batch.deformation_fingerprint, build_collision_surface(previous, policy).deformation_fingerprint)

    def test_simplification_template_keeps_topology_when_vertices_cross_grid_cells(self):
        policy = CollisionSurfaceSpec(
            id=10,
            object_name="Body",
            simplify_max_error=0.2,
        )
        reference = MeshSource(
            vertices=((0.01, 0, 0), (0.02, 0, 0), (1, 0, 0), (0, 1, 0)),
            polygons=((0, 2, 3), (1, 2, 3)),
        )
        template = build_collision_surface(reference, policy)
        deformed = MeshSource(
            vertices=((0.01, 0, 0.1), (0.4, 0, 0.1), (1, 0, 0.1), (0, 1, 0.1)),
            polygons=reference.polygons,
        )

        updated = build_collision_surface(
            deformed,
            policy,
            previous=reference,
            template=template,
        )

        self.assertEqual(updated.topology, template.topology)
        self.assertEqual(updated.topology_fingerprint, template.topology_fingerprint)
        self.assertEqual(updated.source_vertex_indices, template.source_vertex_indices)


if __name__ == "__main__":
    unittest.main()
