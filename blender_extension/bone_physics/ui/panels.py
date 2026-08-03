"""Initial 3D View sidebar shell for Bone Physics authoring."""

from __future__ import annotations

import bpy


class BONE_PHYSICS_PT_root(bpy.types.Panel):
    bl_idname = "BONE_PHYSICS_PT_root"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Bone Physics"
    bl_label = "Bone Physics"

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return context.object is not None and context.object.type == "ARMATURE"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        settings = context.object.bone_physics
        layout.prop(settings, "global_enabled")
        layout.prop(settings, "workspace", expand=True)
        layout.operator("bone_physics.initialize", icon="SETTINGS")
        if settings.workspace != "SETUP":
            layout.label(text="Controls will appear in this workspace.")
            return
        row = layout.row()
        row.template_list(
            "BONE_PHYSICS_UL_systems",
            "",
            settings,
            "physics_systems",
            settings,
            "active_system_index",
        )
        column = row.column(align=True)
        column.operator("bone_physics.system_add", text="", icon="ADD")
        column.operator("bone_physics.system_duplicate", text="", icon="DUPLICATE")
        column.operator("bone_physics.system_remove", text="", icon="REMOVE")
        if not settings.physics_systems:
            layout.label(text="No physics systems. Add one to begin.")
        elif context.object.mode == "POSE":
            layout.operator("bone_physics.chain_add_selected", icon="BONE_DATA")
        else:
            layout.label(text="Enter Pose Mode to add chains from selected bones.")


CLASSES = (BONE_PHYSICS_PT_root,)
