import unittest

try:
    import bone_physics_native as native
except ModuleNotFoundError:
    native = None

from blender_extension.bone_physics.mesh_builder import MeshSource, build_collision_surface
from blender_extension.bone_physics.model import BoneSpec, CollisionSurfaceSpec, WorldSpec
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
        self.capture_times = []
        self.saved_state = 0
        self.state = 0

    def capture_at(self, scene, depsgraph, world_spec, sample_time):
        scene.seconds = sample_time
        self.events.append("capture")
        self.capture_times.append(sample_time)
        self.state += 1
        if self.reenter:
            self.runtime.on_frame_change(scene, depsgraph)
        return PrePhysicsSnapshot(scene.seconds, (), (), IDENTITY)

    def restore_time(self, scene, depsgraph, world_spec, time):
        del depsgraph, world_spec
        scene.seconds = time

    def reset(self):
        self.state = 0

    def save_state(self):
        self.saved_state = self.state
        return self.state

    def restore_state(self, state):
        self.state = state


class Bridge:
    def __init__(self, events):
        self.events = events
        self.fail = False
        self.fail_after = None
        self.paused = []
        self.solve_actions = []

    def prepare(self, handle, action):
        self.events.append(f"prepare:{action.kind.value}")
        return handle

    def set_snapshot(self, handle, snapshot):
        self.events.append("snapshot")

    def solve(self, handle, action):
        self.events.append("solve")
        self.solve_actions.append(action)
        if self.fail or (
            self.fail_after is not None
            and len(self.solve_actions) >= self.fail_after
        ):
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
        self.current = ("previous-1", "previous-2")

    def snapshot(self, armature, body_ids):
        del armature
        return tuple(self.current[index] for index, _ in enumerate(body_ids))

    def restore(self, armature, body_ids, values):
        self.restores.append((armature, tuple(body_ids), tuple(values)))

    def apply(self, armature, body_ids, poses):
        self.events.append("write")
        self.writes.append((armature, tuple(body_ids), tuple(poses)))
        self.current = tuple(poses)


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

    def test_skipped_frame_samples_and_solves_each_fixed_time(self):
        events = []
        runtime, evaluator, bridge, _writer = self.runtime(events)
        scene = Scene()
        runtime.on_frame_change(scene, object())
        evaluator.capture_times.clear()
        bridge.solve_actions.clear()

        scene.seconds = 0.03
        result = runtime.on_frame_change(scene, object())

        self.assertTrue(result.executed)
        self.assertEqual(
            evaluator.capture_times,
            [1.0 / 120.0, 2.0 / 120.0, 3.0 / 120.0],
        )
        self.assertEqual(
            [action.fixed_times for action in bridge.solve_actions],
            [
                (1.0 / 120.0,),
                (2.0 / 120.0,),
                (3.0 / 120.0,),
            ],
        )
        self.assertAlmostEqual(scene.seconds, 0.03)

    def test_fractional_time_without_a_fixed_tick_does_not_consume_input_history(self):
        events = []
        runtime, evaluator, bridge, _writer = self.runtime(events)
        scene = Scene()
        runtime.on_frame_change(scene, object())
        state_before = evaluator.state
        evaluator.capture_times.clear()
        bridge.solve_actions.clear()

        scene.seconds = 0.004
        result = runtime.on_frame_change(scene, object())

        self.assertTrue(result.executed)
        self.assertEqual(evaluator.capture_times, [])
        self.assertEqual(bridge.solve_actions, [])
        self.assertEqual(evaluator.state, state_before)
        self.assertAlmostEqual(scene.seconds, 0.004)

    def test_failed_replay_restores_sample_history_and_requested_time(self):
        events = []
        runtime, evaluator, bridge, _writer = self.runtime(events)
        scene = Scene()
        runtime.on_frame_change(scene, object())
        state_before = evaluator.state
        bridge.solve_actions.clear()
        bridge.fail_after = 2

        scene.seconds = 0.03
        result = runtime.on_frame_change(scene, object())

        self.assertTrue(result.paused)
        self.assertEqual(evaluator.state, state_before)
        self.assertAlmostEqual(scene.seconds, 0.03)

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

    def test_failure_after_success_restores_last_written_pose(self):
        events = []
        runtime, _evaluator, bridge, writer = self.runtime(events)
        scene = Scene()

        first = runtime.on_frame_change(scene, object())
        self.assertTrue(first.executed)
        last_written = writer.writes[-1][2]

        bridge.fail = True
        writer.current = ("animated-1", "animated-2")
        scene.seconds = 0.03
        failed = runtime.on_frame_change(scene, object())

        self.assertTrue(failed.paused)
        self.assertEqual(writer.restores[-1][2], last_written)

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
        surface = CollisionSurfaceSpec(id=501, object_name="Body")
        world = WorldSpec(bones=(bone,), collision_surfaces=(surface,))
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
            mesh_batches=(
                build_collision_surface(
                    MeshSource(
                        vertices=((-2, -0.2, -2), (2, -0.2, -2), (0, -0.2, 2)),
                        polygons=((0, 1, 2),),
                    ),
                    surface,
                ),
            ),
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
        self.assertEqual(native.read_diagnostics(handle)["mesh_count"], 1)
        self.assertAlmostEqual(
            sum(component * component for component in poses[0].rotation),
            1.0,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
