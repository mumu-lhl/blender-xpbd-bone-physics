from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension import bone_physics


def create_armature() -> bpy.types.Object:
    armature_data = bpy.data.armatures.new("UIParametersArmature")
    armature = bpy.data.objects.new("UIParametersArmature", armature_data)
    bpy.context.scene.collection.objects.link(armature)
    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    root = armature_data.edit_bones.new("hair_root.L")
    root.head = (0.0, 0.0, 0.0)
    root.tail = (0.0, 1.0, 0.0)
    tip = armature_data.edit_bones.new("hair_tip.L")
    tip.head = root.tail
    tip.tail = (0.0, 2.0, 0.0)
    tip.parent = root
    bpy.ops.object.mode_set(mode="POSE")
    return armature


def select_pose_bones(armature: bpy.types.Object, names: tuple[str, ...]) -> None:
    for bone in armature.pose.bones:
        bone.select = bone.name in names
    armature.data.bones.active = armature.data.bones[names[0]]


bpy.ops.wm.read_factory_settings(use_empty=True)
bone_physics.unregister()
bone_physics.register()
try:
    armature = create_armature()
    assert bpy.ops.bone_physics.initialize() == {"FINISHED"}
    assert bpy.ops.bone_physics.system_add(name="Hair A") == {"FINISHED"}
    assert bpy.ops.bone_physics.system_add(name="Hair B") == {"FINISHED"}
    settings = armature.bone_physics
    settings.physics_systems[0].selected = True
    settings.physics_systems[1].selected = True

    assert bpy.ops.bone_physics.apply_endpoint_value(
        endpoint="TAIL", field="GRAVITY_SCALE", value=0.35
    ) == {"FINISHED"}
    assert all(abs(system.tail.gravity_scale - 0.35) < 1.0e-6 for system in settings.physics_systems)

    assert bpy.ops.bone_physics.apply_endpoint_value(
        endpoint="HEAD", field="ANGULAR_COMPLIANCE", value=0.125
    ) == {"FINISHED"}
    assert all(abs(system.head.angular_compliance - 0.125) < 1.0e-6 for system in settings.physics_systems)

    assert bpy.ops.bone_physics.curve_point_add(
        field="GRAVITY_SCALE", position=0.25, value=0.8
    ) == {"FINISHED"}
    points = settings.physics_systems[1].curve_points
    assert len(points) == 1
    assert points[0].field == "GRAVITY_SCALE"
    assert points[0].s == 0.25

    select_pose_bones(armature, ("hair_tip.L",))
    settings.active_system_index = 0
    assert bpy.ops.bone_physics.override_add_selected() == {"FINISHED"}
    assert settings.physics_systems[0].overrides[0].bone_name == "hair_tip.L"
    assert hasattr(bpy.types.BONE_PHYSICS_PT_parameters, "bl_idname")
finally:
    bone_physics.unregister()

print(json.dumps({"status": "ok", "test": "ui_parameters"}, sort_keys=True))
