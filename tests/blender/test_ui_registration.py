from __future__ import annotations

from pathlib import Path
import tempfile

import bpy

import bone_physics


with tempfile.TemporaryDirectory(prefix="bone-physics-ui-registration-") as directory:
    blend_path = Path(directory) / "ui-registration.blend"
    try:
        bone_physics.unregister()
        bone_physics.unregister()
        assert not hasattr(bpy.types.Object, "bone_physics")

        bone_physics.register()
        bone_physics.register()

        armature_data = bpy.data.armatures.new("UIArmature")
        armature = bpy.data.objects.new("UIArmature", armature_data)
        bpy.context.scene.collection.objects.link(armature)

        assert hasattr(armature, "bone_physics")
        armature.bone_physics.schema_version = 1
        assert armature.bone_physics.schema_version == 1

        bpy.ops.wm.save_as_mainfile(filepath=str(blend_path), check_existing=False)
        bpy.ops.wm.open_mainfile(filepath=str(blend_path), load_ui=False, use_scripts=False)

        reopened_armature = bpy.data.objects["UIArmature"]
        assert reopened_armature.bone_physics.schema_version == 1

        bone_physics.unregister()
        assert not hasattr(bpy.types.Object, "bone_physics")
        bone_physics.unregister()
        assert not hasattr(bpy.types.Object, "bone_physics")
    finally:
        bone_physics.unregister()
        bone_physics.unregister()

print('{"status": "ok", "test": "ui_registration"}')
