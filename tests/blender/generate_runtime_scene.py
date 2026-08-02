"""Build the deterministic armature/proxy fixture used by background tests."""

from __future__ import annotations

import bpy


def generate_runtime_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.fps = 24
    scene.render.fps_base = 1.0
    scene["depsgraph_updates"] = 0

    armature_data = bpy.data.armatures.new("RuntimeArmature")
    armature = bpy.data.objects.new("RuntimeArmature", armature_data)
    scene.collection.objects.link(armature)
    armature.scale = (1.5, 0.75, 2.0)
    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    torso = armature_data.edit_bones.new("torso")
    torso.head = (0.0, 0.0, 0.0)
    torso.tail = (0.0, 1.0, 0.0)
    hair_a = armature_data.edit_bones.new("hair_a")
    hair_a.head = torso.tail
    hair_a.tail = (0.0, 2.0, 0.0)
    hair_a.parent = torso
    hair_b = armature_data.edit_bones.new("hair_b")
    hair_b.head = hair_a.tail
    hair_b.tail = (0.0, 3.0, 0.0)
    hair_b.parent = hair_a
    bpy.ops.object.mode_set(mode="POSE")
    for name, frame_two_x in (("hair_a", 0.25), ("hair_b", 0.5)):
        bone = armature.pose.bones[name]
        bone.rotation_mode = "QUATERNION"
        bone.location = (0.0, 0.0, 0.0)
        bone.keyframe_insert("location", frame=1)
        bone.location = (frame_two_x, 0.0, 0.0)
        bone.keyframe_insert("location", frame=2)
    bpy.ops.object.mode_set(mode="OBJECT")

    proxy = bpy.data.objects.new("ChestProxy", None)
    proxy.empty_display_type = "SPHERE"
    proxy.location = (0.0, 0.5, 0.0)
    scene.collection.objects.link(proxy)

    def count_updates(_scene, _depsgraph):
        scene["depsgraph_updates"] = scene.get("depsgraph_updates", 0) + 1

    bpy.app.handlers.depsgraph_update_post.append(count_updates)
    scene.frame_set(1)
    return scene, armature, proxy, count_updates
