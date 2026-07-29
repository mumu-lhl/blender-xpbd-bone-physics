from array import array
import unittest

import bone_physics_native as native

from tests.python.test_native_batches import (
    bodies,
    config,
    empty_constraints,
    filters,
)


def topology(**overrides):
    result = {
        "vertex_count": 4,
        "triangles": array("I", [0, 2, 1, 0, 3, 2]),
        "regions": array("I", [7, 7]),
        "vertices": array(
            "f",
            [-2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2],
        ),
        "thickness": 0.0,
        "closed_hint": False,
        "ignore_body_ids": array("I", [1]),
        "ignore_region_offsets": array("I", [0, 1]),
        "ignore_region_ids": array("I", [7]),
    }
    result.update(overrides)
    return result


class NativeMeshTests(unittest.TestCase):
    def setUp(self):
        self.handle = native.build_world(
            config(),
            bodies(),
            empty_constraints(),
            filters(),
        )
        self.addCleanup(native.destroy_world, self.handle)

    def test_topology_once_refit_and_remove_have_explicit_hash_contract(self):
        batch = topology()
        topology_hash = native.set_mesh_topology(self.handle, 501, batch)
        batch["vertices"][0] = 1000.0
        self.assertEqual(native.read_diagnostics(self.handle)["mesh_count"], 1)

        previous = array("f", [-2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2])
        current = array("f", [-2, 0.1, -2, 2, 0.1, -2, 2, 0.1, 2, -2, 0.1, 2])
        self.assertEqual(
            native.update_mesh_vertices(
                self.handle,
                501,
                previous,
                current,
                topology_hash,
            ),
            "refit",
        )
        native.remove_mesh(self.handle, 501)
        self.assertEqual(native.read_diagnostics(self.handle)["mesh_count"], 0)

    def test_bad_topology_buffers_and_non_finite_values_are_atomic(self):
        with self.assertRaisesRegex(native.BonePhysicsInputError, "triangle"):
            native.set_mesh_topology(
                self.handle,
                501,
                topology(triangles=array("I", [0, 1])),
            )
        with self.assertRaisesRegex(native.BonePhysicsInputError, "region"):
            native.set_mesh_topology(
                self.handle,
                501,
                topology(regions=array("I", [7])),
            )
        values = topology()
        values["vertices"][0] = float("nan")
        with self.assertRaisesRegex(native.BonePhysicsInputError, "finite"):
            native.set_mesh_topology(self.handle, 501, values)
        self.assertEqual(native.read_diagnostics(self.handle)["mesh_count"], 0)

    def test_refit_rejects_stale_hash_counts_nan_and_current_only_updates(self):
        topology_hash = native.set_mesh_topology(self.handle, 501, topology())
        vertices = array("f", [-2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2])
        with self.assertRaisesRegex(
            native.BonePhysicsTopologyInvalidatedError,
            "topology hash",
        ):
            native.update_mesh_vertices(
                self.handle,
                501,
                vertices,
                vertices,
                topology_hash + 1,
            )
        with self.assertRaisesRegex(native.BonePhysicsInputError, "12"):
            native.update_mesh_vertices(
                self.handle,
                501,
                array("f", vertices[:-3]),
                vertices,
                topology_hash,
            )
        non_finite = array("f", vertices)
        non_finite[0] = float("inf")
        with self.assertRaisesRegex(native.BonePhysicsInputError, "finite"):
            native.update_mesh_vertices(
                self.handle,
                501,
                vertices,
                non_finite,
                topology_hash,
            )
        with self.assertRaises(TypeError):
            native.update_mesh_vertices(
                self.handle,
                501,
                vertices,
                topology_hash,
            )


if __name__ == "__main__":
    unittest.main()
