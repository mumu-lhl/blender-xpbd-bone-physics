from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension import bone_physics
from blender_extension.bone_physics.authoring import ArmatureAuthoring
from blender_extension.bone_physics.authoring_adapter import (
    SkeletonBone,
    compile_authoring,
    physics_system_from_rna,
)
from blender_extension.bone_physics.parameter_builder import compile_world_spec


def create_armature() -> bpy.types.Object:
    data = bpy.data.armatures.new("UIStructuralArmature")
    armature = bpy.data.objects.new("UIStructuralArmature", data)
    bpy.context.scene.collection.objects.link(armature)
    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    for index in range(4):
        root = data.edit_bones.new(f"skirt_{index}")
        root.head = (float(index), 0.0, 0.0)
        root.tail = (float(index), 1.0, 0.0)
    bpy.ops.object.mode_set(mode="POSE")
    return armature


bpy.ops.wm.read_factory_settings(use_empty=True)
bone_physics.unregister()
bone_physics.register()
try:
    armature = create_armature()
    assert bpy.ops.bone_physics.initialize() == {"FINISHED"}
    assert bpy.ops.bone_physics.system_add(name="Skirt") == {"FINISHED"}
    settings = armature.bone_physics
    system = settings.physics_systems[0]
    for index in range(4):
        chain = system.chains.add()
        chain.id = f"skirt_{index}"
        chain.stable_id = index + 2
        chain.head_uuid = chain.tail_uuid = chain.id
        chain.selected = True
    # Runtime body/joint/recovery IDs occupy the low range; keep generated
    # cross-link IDs in the persisted, non-overlapping authoring range.
    settings.next_stable_id = 100
    assert bpy.ops.bone_physics.structural_add(
        samples="0.0,0.5,1.0", diagonals=True, closed=True, compliance=1.0e-5
    ) == {"FINISHED"}
    authoring = physics_system_from_rna(
        system, {f"skirt_{index}": (f"skirt_{index}",) for index in range(4)}
    )
    skeleton = tuple(
        SkeletonBone(f"skirt_{index}", f"skirt_{index}", None, (float(index), 0, 0), (float(index), 1, 0))
        for index in range(4)
    )
    compiled = compile_world_spec(compile_authoring(ArmatureAuthoring(id="armature", physics_systems=(authoring,)), skeleton))
    # 4 ring pairs × (3 horizontal + 4 diagonal), plus four recovery rows.
    assert len(compiled.constraints["ids"]) == 32
    assert sum(kind == 1 for kind in compiled.constraints["kinds"]) == 28
    assert bpy.ops.bone_physics.structural_remove() == {"FINISHED"}
    assert not system.structural_groups
finally:
    bone_physics.unregister()

print(json.dumps({"status": "ok", "test": "ui_constraints"}, sort_keys=True))
