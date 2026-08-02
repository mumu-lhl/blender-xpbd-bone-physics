import unittest

from blender_extension.bone_physics.mesh_builder import MeshSource
from blender_extension.bone_physics.model import CollisionSurfaceSpec, WorldSpec
from blender_extension.bone_physics.prephysics_eval import PrePhysicsEvaluator


IDENTITY = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)


class Scene:
    seconds = 0.0


class Adapter:
    def set_scene_time(self, scene, seconds):
        scene.seconds = seconds

    def restore_animation_references(self, scene, references):
        del scene, references

    def evaluate_prephysics(self, scene, depsgraph):
        del scene, depsgraph

    def scene_time(self, scene):
        return scene.seconds

    def object_to_simulation(self, scene, depsgraph, world_spec):
        del scene, depsgraph, world_spec
        return IDENTITY

    def target_pose(self, scene, depsgraph, bone):
        raise AssertionError((scene, depsgraph, bone))

    def collider_pose(self, scene, depsgraph, collider):
        raise AssertionError((scene, depsgraph, collider))

    def collision_mesh_source(self, scene, depsgraph, surface, object_to_simulation):
        del depsgraph, surface
        self.last_capture_matrix = object_to_simulation
        z = scene.seconds
        return MeshSource(
            vertices=((0, 0, z), (1, 0, z), (0, 1, z)),
            polygons=((0, 1, 2),),
        )


class PrePhysicsMeshTests(unittest.TestCase):
    def test_mesh_motion_is_captured_at_each_prephysics_sample_and_rolls_back(self):
        scene = Scene()
        adapter = Adapter()
        evaluator = PrePhysicsEvaluator(adapter)
        spec = WorldSpec(
            bones=(),
            collision_surfaces=(
                CollisionSurfaceSpec(id=5, object_name="Body"),
            ),
        )

        first = evaluator.capture_at(scene, None, spec, 1.0)
        saved = evaluator.save_state()
        second = evaluator.capture_at(scene, None, spec, 2.0)
        self.assertEqual(first.mesh_batches[0].previous_vertices, first.mesh_batches[0].current_vertices)
        self.assertEqual(second.mesh_batches[0].previous_vertices[0], (0.0, 0.0, 1.0))
        self.assertEqual(second.mesh_batches[0].current_vertices[0], (0.0, 0.0, 2.0))
        self.assertEqual(adapter.last_capture_matrix, IDENTITY)

        evaluator.restore_state(saved)
        replay = evaluator.capture_at(scene, None, spec, 2.0)
        self.assertEqual(second.mesh_batches, replay.mesh_batches)


if __name__ == "__main__":
    unittest.main()
