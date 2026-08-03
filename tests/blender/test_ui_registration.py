from __future__ import annotations

import bpy

import bone_physics


bone_physics.unregister()
bone_physics.register()

armature_data = bpy.data.armatures.new("UIArmature")
armature = bpy.data.objects.new("UIArmature", armature_data)
bpy.context.scene.collection.objects.link(armature)

assert hasattr(armature, "bone_physics")
assert armature.bone_physics.schema_version == 1
armature.bone_physics.schema_version = 1
assert armature.bone_physics.schema_version == 1

bone_physics.unregister()
assert not hasattr(bpy.types.Object, "bone_physics")
bone_physics.register()

print('{"status": "ok", "test": "ui_registration"}')
