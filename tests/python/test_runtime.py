import unittest

try:
    import bone_physics_native as native
except ModuleNotFoundError:
    native = None

from blender_extension.bone_physics.model import BoneSpec, WorldSpec
from blender_extension.bone_physics.parameter_builder import compile_world_spec
from blender_extension.bone_physics.prephysics_eval import PrePhysicsSnapshot, TargetPose
from blender_extension.bone_physics.runtime import NativeRuntimeBridge, RuntimeWorld
from blender_extension.bone_physics.scene_adapter import Pose
from blender_extension.bone_physics.timeline import (
    CacheIndex,
    TimelineAction,
    TimelineActionKind,
    TimelineState,
    TimelineTime,
)


IDENTITY = (
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


def pose(x):
    return Pose((x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class Scene:
    def __init__(self):
        self.seconds = 0.0


class Evaluator:
    def __init__(self, events):
        self.events = events
        self.runtime = None
        self.reenter = False

    def capture(self, scene, depsgraph, world_spec):
        self.events.append("capture")
        if self.reenter:
            self.runtime.on_frame_change(scene, depsgraph)
        return PrePhysicsSnapshot(scene.seconds, (), (), IDENTITY)


class Bridge:
    def __init__(self, events):
        self.events = events
        self.fail = False
        self.paused = []

    def prepare(self, handle, action):
        self.events.append(f"prepare:{action.kind.value}")
        return handle

    def set_snapshot(self, handle, snapshot):
        self.events.append("snapshot")

    def solve(self, handle, action):
        self.events.append("solve")
        if self.fail:
            raise RuntimeError("native solve failed")

    def read_poses(self, handle, body_ids):
        self.events.append("read")
        return tuple(Pose((float(body_id), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)) for body_id in body_ids)

    def pause(self, handle, error):
        self.events.append("pause")
        self.paused.append((handle, error))


class Writer:
    def __init__(self, events):
        self.events = events
        self.writes = []
        self.restores = []

    def snapshot(self, armature, body_ids):
        return tuple(f"previous-{body_id}" for body_id in body_ids)

    def restore(self, armature, body_ids, values):
        self.restores.append((armature, tuple(body_ids), tuple(values)))

    def apply(self, armature, body_ids, poses):
        self.events.append("write")
        self.writes.append((armature, tuple(body_ids), tuple(poses)))


class RuntimeTests(unittest.TestCase):
    def runtime(self, events):
        evaluator = Evaluator(events)
        bridge = Bridge(events)
        writer = Writer(events)
        runtime = RuntimeWorld(
            handle=77,
            world_spec=object(),
            armature="armature",
            body_ids=(1, 2),
            timeline=TimelineState(
                start_time=0.0,
                fixed_dt=1.0 / 120.0,
                configuration_hash="world-a",
            ),
            cache_index=CacheIndex(),
            evaluator=evaluator,
            bridge=bridge,
            pose_writer=writer,
            time_source=lambda scene: TimelineTime(scene.seconds, 24.0),
        )
        evaluator.runtime = runtime
        return runtime, evaluator, bridge, writer

    def test_handler_captures_then_solves_then_writes_exactly_once_under_reentry(self):
        events = []
        runtime, evaluator, _bridge, writer = self.runtime(events)
        evaluator.reenter = True
        scene = Scene()

        result = runtime.on_frame_change(scene, object())

        self.assertTrue(result.executed)
        self.assertFalse(result.paused)
        self.assertEqual(
            events,
            [
                "prepare:reset",
                "capture",
                "snapshot",
                "solve",
                "read",
                "write",
            ],
        )
        self.assertEqual(len(writer.writes), 1)

    def test_unchanged_frame_does_not_capture_or_solve_again(self):
        events = []
        runtime, _evaluator, _bridge, _writer = self.runtime(events)
        scene = Scene()
        runtime.on_frame_change(scene, object())
        events.clear()

        result = runtime.on_frame_change(scene, object())

        self.assertFalse(result.executed)
        self.assertEqual(events, [])

    def test_bridge_failure_retains_pose_and_pauses_only_this_world(self):
        events = []
        runtime, _evaluator, bridge, writer = self.runtime(events)
        bridge.fail = True
        scene = Scene()

        result = runtime.on_frame_change(scene, object())

        self.assertTrue(result.paused)
        self.assertEqual(writer.writes, [])
        self.assertEqual(
            writer.restores,
            [("armature", (1, 2), ("previous-1", "previous-2"))],
        )
        self.assertEqual(events[-1], "pause")
        self.assertEqual(bridge.paused[0][0], 77)

        other_events = []
        other, _evaluator, _bridge, other_writer = self.runtime(other_events)
        other.on_frame_change(scene, object())
        self.assertEqual(len(other_writer.writes), 1)

    def test_reentry_and_paused_worlds_are_explicit_noops(self):
        events = []
        runtime, _evaluator, bridge, _writer = self.runtime(events)
        bridge.fail = True
        scene = Scene()
        runtime.on_frame_change(scene, object())
        events.clear()

        result = runtime.on_frame_change(scene, object())

        self.assertFalse(result.executed)
        self.assertTrue(result.paused)
        self.assertEqual(events, [])

    @unittest.skipIf(native is None, "native bridge is tested in native-contract")
    def test_native_runtime_bridge_batches_snapshot_solve_read_and_resume(self):
        bone = BoneSpec(
            id=1,
            name="hair",
            local_head=(0.0, 0.0, 0.0),
            local_tail=(0.0, 1.0, 0.0),
        )
        world = WorldSpec(bones=(bone,))
        compiled = compile_world_spec(world)
        bridge = NativeRuntimeBridge(
            native,
            compiled,
            world.bones,
            fixed_dt=world.fixed_dt,
        )
        handle = bridge.create_world()
        self.addCleanup(native.destroy_world, handle)
        snapshot = PrePhysicsSnapshot(
            time=world.fixed_dt,
            target_poses=(TargetPose(1, pose(0.0), pose(0.0)),),
            collider_poses=(),
            object_to_simulation=IDENTITY,
        )
        action = TimelineAction(
            kind=TimelineActionKind.ADVANCE,
            fixed_times=(world.fixed_dt,),
            checkpoint=None,
            authoritative=True,
            reason="test",
        )

        bridge.set_snapshot(handle, snapshot)
        bridge.solve(handle, action)
        poses = bridge.read_poses(handle, (1,))
        bridge.resume(handle)

        self.assertEqual(len(poses), 1)
        self.assertAlmostEqual(
            sum(component * component for component in poses[0].rotation),
            1.0,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
