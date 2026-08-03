from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import bone_physics
except ModuleNotFoundError:
    from blender_extension import bone_physics


def create_armature() -> bpy.types.Object:
    armature_data = bpy.data.armatures.new("UISystemsArmature")
    armature = bpy.data.objects.new("UISystemsArmature", armature_data)
    bpy.context.scene.collection.objects.link(armature)
    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    for name, x in (("hair_root.L", -0.5), ("hair_root.R", 0.5)):
        root = armature_data.edit_bones.new(name)
        root.head = (x, 0.0, 0.0)
        root.tail = (x, 1.0, 0.0)
        tip = armature_data.edit_bones.new(f"{name}.tip")
        tip.head = root.tail
        tip.tail = (x, 2.0, 0.0)
        tip.parent = root
    bpy.ops.object.mode_set(mode="POSE")
    return armature


def select_pose_bones(armature: bpy.types.Object, names: tuple[str, ...]) -> None:
    for bone in armature.data.bones:
        bone.select_set(bone.name in names)
    armature.data.bones.active = armature.data.bones[names[0]]


bpy.ops.wm.read_factory_settings(use_empty=True)
bone_physics.unregister()
bone_physics.register()
try:
    armature = create_armature()
    select_pose_bones(armature, ("hair_root.L", "hair_root.R"))

    assert bpy.ops.bone_physics.initialize() == {"FINISHED"}
    assert bpy.ops.bone_physics.system_add(name="Hair") == {"FINISHED"}
    assert bpy.ops.bone_physics.chain_add_selected() == {"FINISHED"}

    settings = armature.bone_physics
    assert settings.physics_systems[0].name == "Hair"
    assert len(settings.physics_systems[0].chains) == 2
    assert settings.workspace == "SETUP"
    assert {item.identifier for item in bpy.types.BONE_PHYSICS_OT_system_add.bl_rna.properties["template"].enum_items} == {
        "LOOSE_HAIR", "EARS", "PONYTAIL", "SKIRT", "ACCESSORY", "BLANK"
    }
    assert bpy.types.BONE_PHYSICS_PT_root.bl_category == "Bone Physics"

    assert bpy.ops.bone_physics.system_duplicate() == {"FINISHED"}
    assert len(settings.physics_systems) == 2
    assert bpy.ops.bone_physics.system_remove() == {"FINISHED"}
    assert len(settings.physics_systems) == 1
finally:
    bone_physics.unregister()

print(json.dumps({"status": "ok", "test": "ui_systems"}, sort_keys=True))
