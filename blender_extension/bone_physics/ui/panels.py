"""Initial 3D View sidebar shell for Bone Physics authoring."""

from __future__ import annotations

import bpy


_GROUP_FIELDS = {
    "show_motion": ("inverse_mass", "inertia_scale", "gravity_scale", "translation_inertia", "rotation_inertia", "linear_damping", "angular_damping"),
    "show_recovery": ("recovery_compliance", "recovery_damping"),
    "show_angular_limits": ("angular_limit_mode", "xyz_lower", "xyz_upper", "swing_axis", "max_swing", "min_twist", "max_twist"),
    "show_collision": ("radius", "margin", "capsule_head_inset", "capsule_tail_inset", "friction", "restitution"),
    "show_advanced": ("joint_compliance", "joint_damping", "angular_compliance", "angular_damping_constraint"),
}


def _draw_endpoint_group(layout: bpy.types.UILayout, system: bpy.types.PropertyGroup, group: str) -> None:
    if not getattr(system, group):
        return
    columns = layout.row(align=True).split(factor=0.5)
    for column, endpoint, label in ((columns.column(), system.head, "Head"), (columns.column(), system.tail, "Tail")):
        column.label(text=label)
        for field in _GROUP_FIELDS[group]:
            column.prop(endpoint, field)


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


class BONE_PHYSICS_PT_parameters(bpy.types.Panel):
    bl_idname = "BONE_PHYSICS_PT_parameters"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Bone Physics"
    bl_label = "Parameters"
    bl_parent_id = "BONE_PHYSICS_PT_root"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return BONE_PHYSICS_PT_root.poll(context) and bool(context.object.bone_physics.physics_systems)

    def draw(self, context: bpy.types.Context) -> None:
        settings = context.object.bone_physics
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        layout = self.layout
        layout.prop(system, "selected")
        layout.prop(system, "interpolation")
        layout.operator("bone_physics.apply_endpoint_value", text="Batch Edit Selected Systems")
        curve_box = layout.box()
        curve_box.label(text="Per-Field Curves")
        curve_box.operator("bone_physics.curve_point_add", icon="ADD")
        if system.curve_points:
            curve_box.template_list(
                "BONE_PHYSICS_UL_curve_points", "", system, "curve_points",
                system, "active_curve_point_index",
            )
        for group, label in (("show_motion", "Motion"), ("show_recovery", "Recovery"), ("show_angular_limits", "Angular Limits"), ("show_collision", "Collision"), ("show_advanced", "Advanced")):
            layout.prop(system, group, text=label, toggle=True)
            _draw_endpoint_group(layout, system, group)
        box = layout.box()
        box.label(text="Per-Bone Overrides")
        box.operator("bone_physics.override_add_selected")
        if system.overrides:
            box.template_list("BONE_PHYSICS_UL_overrides", "", system, "overrides", system, "active_override_index")
            override = system.overrides[min(system.active_override_index, len(system.overrides) - 1)]
            box.label(text=f"Override Values: {override.bone_name}")
            for fields in _GROUP_FIELDS.values():
                for field in fields:
                    box.prop(override.parameters, field)


CLASSES = (BONE_PHYSICS_PT_root, BONE_PHYSICS_PT_parameters)
