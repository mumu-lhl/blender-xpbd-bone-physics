from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension.bone_physics.model import BoneSpec, WorldSpec  # noqa: E402
from blender_extension.bone_physics.pose_writer import PoseWriter  # noqa: E402
from blender_extension.bone_physics.prephysics_eval import PrePhysicsEvaluator  # noqa: E402
from blender_extension.bone_physics.runtime import RuntimeWorld  # noqa: E402
from blender_extension.bone_physics.scene_adapter import BlenderSceneAdapter  # noqa: E402
from blender_extension.bone_physics.timeline import (  # noqa: E402
    CacheIndex,
    TimelineState,
    TimelineTime,
)
from tests.blender.generate_runtime_scene import generate_runtime_scene  # noqa: E402


class ReenteringEvaluator:
    def __init__(self, evaluator):
        self.evaluator = evaluator
        self.runtime = None
        self.capture_count = 0

    def capture(self, scene, depsgraph, world_spec):
        self.capture_count += 1
        self.runtime.on_frame_change(scene, depsgraph)
        return self.evaluator.capture(scene, depsgraph, world_spec)


class FakeBridge:
    def __init__(self):
        self.snapshot = None
        self.solve_count = 0
        self.fail = False
        self.paused = False

    def prepare(self, handle, action):
        return handle

    def set_snapshot(self, handle, snapshot):
        self.snapshot = snapshot

    def solve(self, handle, action):
        self.solve_count += 1
        if self.fail:
            raise RuntimeError("injected bridge failure")

    def read_poses(self, handle, body_ids):
        targets = {target.body_id: target.current for target in self.snapshot.target_poses}
        return tuple(targets[body_id] for body_id in body_ids)

    def pause(self, handle, error):
        self.paused = True

    def resume(self, handle):
        self.paused = False


scene, armature, _proxy, handler = generate_runtime_scene()
try:
    spec = WorldSpec(
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
        )
    )
    adapter = BlenderSceneAdapter("RuntimeArmature", {1: "hair_a", 2: "hair_b"})
    evaluator = ReenteringEvaluator(PrePhysicsEvaluator(adapter))
    bridge = FakeBridge()
    runtime = RuntimeWorld(
        handle=1,
        world_spec=spec,
        armature=armature,
        body_ids=(1, 2),
        timeline=TimelineState(
            start_time=0.0,
            fixed_dt=spec.fixed_dt,
            configuration_hash="runtime-handler-test",
        ),
        cache_index=CacheIndex(),
        evaluator=evaluator,
        bridge=bridge,
        pose_writer=PoseWriter(adapter),
        time_source=lambda current_scene: TimelineTime(
            (
                current_scene.frame_current
                + current_scene.frame_subframe
            )
            / (current_scene.render.fps / current_scene.render.fps_base),
            current_scene.render.fps / current_scene.render.fps_base,
        ),
    )
    evaluator.runtime = runtime
    result = runtime.on_frame_change(scene, bpy.context.evaluated_depsgraph_get())
    assert result.executed
    assert evaluator.capture_count == 1
    assert bridge.solve_count == 1

    valid = tuple(
        armature.pose.bones[name].matrix_basis.copy()
        for name in ("hair_a", "hair_b")
    )
    bridge.fail = True
    scene.frame_set(2)
    failed = runtime.on_frame_change(scene, bpy.context.evaluated_depsgraph_get())
    assert failed.paused and bridge.paused
    assert all(
        armature.pose.bones[name].matrix_basis == expected
        for name, expected in zip(("hair_a", "hair_b"), valid)
    )
    print(
        json.dumps(
            {
                "test": "runtime_handler",
                "status": "ok",
                "solve_count": bridge.solve_count,
            },
            sort_keys=True,
        )
    )
finally:
    bpy.app.handlers.depsgraph_update_post.remove(handler)
