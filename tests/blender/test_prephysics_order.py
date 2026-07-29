from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension.bone_physics.model import (  # noqa: E402
    AnalyticColliderSpec,
    AnalyticShape,
    BoneSpec,
    WorldSpec,
)
from blender_extension.bone_physics.pose_writer import PoseWriter  # noqa: E402
from blender_extension.bone_physics.prephysics_eval import PrePhysicsEvaluator  # noqa: E402
from blender_extension.bone_physics.scene_adapter import (  # noqa: E402
    BlenderSceneAdapter,
    Pose,
)
from tests.blender.generate_runtime_scene import generate_runtime_scene  # noqa: E402


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
        ),
        analytic_colliders=(
            AnalyticColliderSpec(
                id=30,
                shape=AnalyticShape.SPHERE,
                radius=0.5,
                source_object="ChestProxy",
            ),
        ),
    )
    adapter = BlenderSceneAdapter(
        "RuntimeArmature",
        {1: "hair_a", 2: "hair_b"},
        {30: "ChestProxy"},
    )
    evaluator = PrePhysicsEvaluator(adapter)
    first = evaluator.capture(scene, bpy.context.evaluated_depsgraph_get(), spec)
    writer = PoseWriter(adapter)
    writer.apply(
        armature,
        (1, 2),
        (
            Pose((1000.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            first.target_poses[1].current,
        ),
    )
    scene.frame_set(2)
    second = evaluator.capture(scene, bpy.context.evaluated_depsgraph_get(), spec)
    assert second.target_poses[0].previous == first.target_poses[0].current
    assert abs(second.target_poses[0].current.translation[0]) < 10.0
    assert second.collider_poses[0].collider_id == 30
    print(
        json.dumps(
            {
                "test": "prephysics_order",
                "status": "ok",
                "depsgraph_updates": scene["depsgraph_updates"],
            },
            sort_keys=True,
        )
    )
finally:
    bpy.app.handlers.depsgraph_update_post.remove(handler)
