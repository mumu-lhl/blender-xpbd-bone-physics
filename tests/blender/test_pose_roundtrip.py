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
from blender_extension.bone_physics.scene_adapter import (  # noqa: E402
    BlenderSceneAdapter,
    pose_from_matrix,
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
        )
    )
    adapter = BlenderSceneAdapter("RuntimeArmature", {1: "hair_a", 2: "hair_b"})
    evaluator = PrePhysicsEvaluator(adapter)
    snapshot = evaluator.capture(scene, bpy.context.evaluated_depsgraph_get(), spec)
    torso_before = armature.pose.bones["torso"].matrix_basis.copy()
    PoseWriter(adapter).apply(
        armature,
        (1, 2),
        tuple(target.current for target in snapshot.target_poses),
    )
    assert armature.pose.bones["torso"].matrix_basis == torso_before
    for target, body_id in zip(snapshot.target_poses, (1, 2)):
        actual = adapter.target_pose(
            scene,
            bpy.context.evaluated_depsgraph_get(),
            next(bone for bone in spec.bones if bone.id == body_id),
        )
        error = sum(
            abs(left - right)
            for left, right in zip(actual.translation, target.current.translation)
        )
        assert error < 1.0e-5

    armature.scale = (-1.0, 1.0, 1.0)
    bpy.context.view_layer.update()
    try:
        pose_from_matrix(armature.matrix_world)
    except ValueError:
        negative_rejected = True
    else:
        negative_rejected = False
    assert negative_rejected
    print(json.dumps({"test": "pose_roundtrip", "status": "ok"}, sort_keys=True))
finally:
    bpy.app.handlers.depsgraph_update_post.remove(handler)
