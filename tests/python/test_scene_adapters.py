import unittest

from blender_extension.bone_physics.model import (
    AnalyticColliderSpec,
    AnalyticShape,
    BoneSpec,
    WorldSpec,
)
from blender_extension.bone_physics.pose_writer import PoseWriter
from blender_extension.bone_physics.prephysics_eval import PrePhysicsEvaluator
from blender_extension.bone_physics.scene_adapter import Pose, validate_positive_scale


def pose(x):
    return Pose((x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class FakeScene:
    def __init__(self):
        self.time = 1.0
        self.animation = {1: pose(1.0), 2: pose(2.0)}
        self.visible = dict(self.animation)
        self.colliders = {30: pose(3.0)}


class FakeCaptureAdapter:
    def __init__(self):
        self.events = []

    def restore_animation_references(self, scene, references):
        self.events.append("restore")
        scene.visible.update(references)

    def set_scene_time(self, scene, seconds):
        scene.time = seconds

    def evaluate_prephysics(self, scene, depsgraph):
        self.events.append("evaluate")
        scene.visible.update(scene.animation)

    def scene_time(self, scene):
        return scene.time

    def object_to_simulation(self, scene, depsgraph, world_spec):
        return (
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        )

    def target_pose(self, scene, depsgraph, bone):
        self.events.append(f"target:{bone.id}")
        return scene.visible[bone.id]

    def collider_pose(self, scene, depsgraph, collider):
        self.events.append(f"collider:{collider.id}")
        return scene.colliders[collider.id]


class FakeWriteAdapter:
    def __init__(self, channels):
        self.channels = channels
        self.fail_body = None

    def stage_pose(self, armature, body_id, simulation_pose):
        if body_id == self.fail_body:
            raise ValueError(f"cannot convert body {body_id}")
        return simulation_pose

    def read_pose_channel(self, armature, body_id):
        return self.channels[body_id]

    def write_pose_channel(self, armature, body_id, value):
        self.channels[body_id] = value


class SceneAdapterTests(unittest.TestCase):
    def world(self):
        return WorldSpec(
            bones=(
                BoneSpec(
                    id=1,
                    name="hair_a",
                    local_head=(0.0, 0.0, 0.0),
                    local_tail=(0.0, 1.0, 0.0),
                ),
                BoneSpec(
                    id=2,
                    name="hair_b",
                    local_head=(0.0, 0.0, 0.0),
                    local_tail=(0.0, 1.0, 0.0),
                ),
            ),
        )

    def test_capture_restores_reference_before_evaluation_and_excludes_prior_output(self):
        scene = FakeScene()
        adapter = FakeCaptureAdapter()
        evaluator = PrePhysicsEvaluator(adapter)
        first = evaluator.capture(scene, object(), self.world())
        scene.visible[1] = pose(1000.0)
        scene.animation[1] = pose(4.0)
        scene.time = 2.0

        second = evaluator.capture(scene, object(), self.world())

        self.assertEqual(first.target_poses[0].previous, pose(1.0))
        self.assertEqual(second.target_poses[0].previous, pose(1.0))
        self.assertEqual(second.target_poses[0].current, pose(4.0))
        self.assertNotEqual(second.target_poses[0].current, pose(1000.0))
        self.assertEqual(
            adapter.events,
            [
                "evaluate",
                "target:1",
                "target:2",
                "restore",
                "evaluate",
                "target:1",
                "target:2",
            ],
        )

    def test_pose_writer_changes_only_declared_channels(self):
        channels = {1: pose(1.0), 2: pose(2.0), 99: pose(99.0)}
        writer = PoseWriter(FakeWriteAdapter(channels))

        writer.apply(object(), (1, 2), (pose(10.0), pose(20.0)))

        self.assertEqual(channels[1], pose(10.0))
        self.assertEqual(channels[2], pose(20.0))
        self.assertEqual(channels[99], pose(99.0))

    def test_collider_snapshot_keeps_previous_and_current_prephysics_poses(self):
        scene = FakeScene()
        adapter = FakeCaptureAdapter()
        evaluator = PrePhysicsEvaluator(adapter)
        world = WorldSpec(
            bones=self.world().bones,
            analytic_colliders=(
                AnalyticColliderSpec(
                    id=30,
                    shape=AnalyticShape.SPHERE,
                    radius=0.5,
                ),
            ),
        )
        first = evaluator.capture(scene, object(), world)
        scene.colliders[30] = pose(4.0)
        second = evaluator.capture(scene, object(), world)

        self.assertEqual(first.collider_poses[0].previous, pose(3.0))
        self.assertEqual(second.collider_poses[0].previous, pose(3.0))
        self.assertEqual(second.collider_poses[0].current, pose(4.0))

    def test_pose_writer_stages_all_conversions_before_mutating_channels(self):
        channels = {1: pose(1.0), 2: pose(2.0)}
        adapter = FakeWriteAdapter(channels)
        adapter.fail_body = 2
        writer = PoseWriter(adapter)

        with self.assertRaisesRegex(ValueError, "body 2"):
            writer.apply(object(), (1, 2), (pose(10.0), pose(20.0)))

        self.assertEqual(channels, {1: pose(1.0), 2: pose(2.0)})

    def test_pose_writer_rejects_duplicate_or_mismatched_body_batches(self):
        writer = PoseWriter(FakeWriteAdapter({1: pose(1.0), 2: pose(2.0)}))

        with self.assertRaisesRegex(ValueError, "same length"):
            writer.apply(object(), (1, 2), (pose(10.0),))
        with self.assertRaisesRegex(ValueError, "duplicate body ID 1"):
            writer.apply(object(), (1, 1), (pose(10.0), pose(20.0)))

    def test_negative_zero_and_non_finite_scales_are_rejected(self):
        for scale in ((-1.0, 1.0, 1.0), (0.0, 1.0, 1.0), (float("nan"), 1.0, 1.0)):
            with self.subTest(scale=scale):
                with self.assertRaises(ValueError):
                    validate_positive_scale(scale)


if __name__ == "__main__":
    unittest.main()
