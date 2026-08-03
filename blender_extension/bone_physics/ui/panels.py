"""Initial 3D View sidebar shell for Bone Physics authoring."""

from __future__ import annotations

import bpy

from ..preset_io import EXACT_ARMATURE_PRESET_KIND, PRESET_KIND, list_presets


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
        preset_box = layout.box()
        preset_box.label(text="Bone Parameter Presets")
        preset_box.operator("bone_physics.preset_save", icon="FILE_TICK")
        for preset in list_presets(PRESET_KIND):
            row = preset_box.row(align=True)
            apply = row.operator("bone_physics.preset_apply", text=preset["name"], icon="IMPORT")
            apply.filepath = str(preset["path"])
            apply.mask = "ALL"
            apply.target = "ALL"
            for mask, label in (("MOTION", "Motion"), ("LOCAL_CONSTRAINT", "Local Constraint"), ("COLLISION", "Collision")):
                masked = row.operator("bone_physics.preset_apply", text=label)
                masked.filepath = str(preset["path"])
                masked.mask = mask
                masked.target = "ALL"
            if preset["read_only"]:
                row.label(text="Packaged", icon="LOCKED")
                copy = row.operator("bone_physics.preset_copy", text="Copy", icon="DUPLICATE")
                copy.filepath = str(preset["path"])
            else:
                rename = row.operator("bone_physics.preset_rename", text="", icon="GREASEPENCIL")
                rename.filepath = str(preset["path"])
                delete = row.operator("bone_physics.preset_delete", text="", icon="REMOVE")
                delete.filepath = str(preset["path"])
        exact_box = layout.box()
        exact_box.label(text="Exact Armature Presets")
        exact_box.label(text="Full systems, chains, constraints, and collision bundle", icon="LOCKED")
        exact_box.operator("bone_physics.exact_preset_save", icon="FILE_TICK")
        for preset in list_presets(EXACT_ARMATURE_PRESET_KIND):
            row = exact_box.row(align=True)
            apply = row.operator("bone_physics.exact_preset_apply", text=preset["name"], icon="IMPORT")
            apply.filepath = str(preset["path"])
            if preset["read_only"]:
                row.label(text="Packaged", icon="LOCKED")
            else:
                delete = row.operator("bone_physics.exact_preset_delete", text="", icon="REMOVE")
                delete.filepath = str(preset["path"])
        layout.prop(system, "interpolation")
        layout.operator("bone_physics.apply_endpoint_value", text="Batch Edit Selected Systems")
        curve_box = layout.box()
        curve_box.label(text="Per-Field Curves")
        curve_actions = curve_box.row(align=True)
        curve_actions.operator("bone_physics.curve_point_add", text="", icon="ADD")
        curve_actions.operator("bone_physics.curve_point_remove", text="", icon="REMOVE")
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


class BONE_PHYSICS_PT_runtime(bpy.types.Panel):
    bl_idname = "BONE_PHYSICS_PT_runtime"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Bone Physics"
    bl_label = "Runtime"
    bl_parent_id = "BONE_PHYSICS_PT_root"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return BONE_PHYSICS_PT_root.poll(context)

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        settings = context.object.bone_physics
        layout.prop(settings, "global_enabled", text="Enable Physics")
        row = layout.row(align=True)
        enable = row.operator("bone_physics.runtime_enable", text="Enable")
        enable.enabled = True
        disable = row.operator("bone_physics.runtime_enable", text="Disable")
        disable.enabled = False
        row = layout.row(align=True)
        pause = row.operator("bone_physics.runtime_pause", text="Pause")
        pause.paused = True
        resume = row.operator("bone_physics.runtime_pause", text="Resume")
        resume.paused = False
        layout.operator("bone_physics.runtime_isolate")
        layout.operator("bone_physics.runtime_validate_rebuild")
        layout.operator("bone_physics.runtime_reset")


class BONE_PHYSICS_PT_constraints(bpy.types.Panel):
    bl_idname = "BONE_PHYSICS_PT_constraints"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Bone Physics"
    bl_label = "Structural Constraints"
    bl_parent_id = "BONE_PHYSICS_PT_root"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return BONE_PHYSICS_PT_parameters.poll(context)

    def draw(self, context: bpy.types.Context) -> None:
        settings = context.object.bone_physics
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        layout = self.layout
        selection = layout.box()
        selection.label(text="Selected Chains")
        for chain in system.chains:
            selection.prop(chain, "selected", text=chain.id or "Unnamed chain")
        actions = layout.row(align=True)
        actions.operator("bone_physics.structural_add", icon="ADD")
        actions.operator("bone_physics.structural_edit", text="", icon="GREASEPENCIL")
        actions.operator("bone_physics.structural_remove", text="", icon="REMOVE")
        if not system.structural_groups:
            layout.label(text="Select chains, then add horizontal, diagonal, or seam links.")
            return
        layout.template_list(
            "BONE_PHYSICS_UL_structural_groups", "", system, "structural_groups",
            system, "active_structural_group_index",
        )
        group = system.structural_groups[min(system.active_structural_group_index, len(system.structural_groups) - 1)]
        sample_count = len(tuple(part for part in group.samples.split(",") if part.strip()))
        chain_count = len(tuple(part for part in group.chain_ids.split(",") if part.strip()))
        pair_count = max(0, chain_count - 1 + int(group.closed))
        generated_count = pair_count * (sample_count + (2 * max(0, sample_count - 1) if group.diagonals else 0))
        inspector = layout.box()
        inspector.label(text="Generated Inspector (read-only)")
        inspector.label(text=f"Constraints: {generated_count}")
        inspector.label(text=f"IDs: {group.first_constraint_id}…{group.first_constraint_id + max(0, generated_count - 1)}")
        inspector.label(text="Current error: rebuild required")


class BONE_PHYSICS_PT_collision_resources(bpy.types.Panel):
    bl_idname = "BONE_PHYSICS_PT_collision_resources"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Bone Physics"
    bl_label = "Collision Resources"
    bl_parent_id = "BONE_PHYSICS_PT_root"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return BONE_PHYSICS_PT_root.poll(context)

    def draw(self, context: bpy.types.Context) -> None:
        settings = context.object.bone_physics
        layout = self.layout
        row = layout.row()
        row.template_list("BONE_PHYSICS_UL_collision_resources", "", settings,
                          "collision_resources", settings, "active_collision_resource_index")
        actions = row.column(align=True)
        actions.operator("bone_physics.collision_resource_add", text="", icon="ADD")
        actions.operator("bone_physics.collision_resource_duplicate", text="", icon="DUPLICATE")
        actions.operator("bone_physics.collision_resource_mirror", text="", icon="MOD_MIRROR")
        actions.operator("bone_physics.collision_resource_remove", text="", icon="REMOVE")
        if not settings.collision_resources:
            return
        resource = settings.collision_resources[min(settings.active_collision_resource_index, len(settings.collision_resources) - 1)]
        layout.prop(resource, "kind")
        layout.prop(resource, "enabled")
        row = layout.row(align=True)
        row.operator("bone_physics.collision_resource_bind", icon="CONSTRAINT_BONE")
        row.operator("bone_physics.collision_resource_toggle", icon="HIDE_OFF")
        if resource.kind == "ANALYTIC":
            layout.prop(resource, "shape")
            layout.prop(resource, "binding")
            layout.prop(resource, "object_name")
            if resource.binding == "BONE":
                layout.prop(resource, "bone_name")
            layout.prop(resource, "radius")
            if resource.shape == "CAPSULE":
                layout.prop(resource, "local_a")
                layout.prop(resource, "local_b")
            elif resource.shape == "ROUNDED_BOX":
                layout.prop(resource, "half_extents")
            layout.prop(resource, "translation")
            layout.prop(resource, "rotation")
        elif resource.kind == "MESH":
            for name in ("object_name", "thickness", "simplification", "excluded_materials", "excluded_regions", "ignored_body_regions", "weight_merges"):
                layout.prop(resource, name)
        else:
            layout.prop(resource, "member_ids")
        if settings.physics_systems:
            layout.operator("bone_physics.collision_resource_assign")
            system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
            collision = layout.box()
            collision.label(text="System Collision Policy")
            collision.prop(system, "self_collision")
            collision.prop(system, "cross_chain_collision")
            collision.prop(system, "collide_with_system_ids")
            collision.prop(system, "collision_resource_ids")


class BONE_PHYSICS_PT_diagnostics(bpy.types.Panel):
    bl_idname = "BONE_PHYSICS_PT_diagnostics"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Bone Physics"
    bl_label = "Diagnostics"
    bl_parent_id = "BONE_PHYSICS_PT_root"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return BONE_PHYSICS_PT_root.poll(context)

    def draw(self, context: bpy.types.Context) -> None:
        settings = context.object.bone_physics
        layout = self.layout
        layout.prop(settings, "show_viewport_overlays")
        # Runtime diagnostics are intentionally optional: validation failures
        # must remain inspectable even when no native world can be created.
        items = tuple(getattr(settings, "diagnostic_items", ()))
        counts = {"error": 0, "warning": 0}
        for item in items:
            counts[item.severity] = counts.get(item.severity, 0) + 1
        layout.label(text=f"Errors: {counts['error']}", icon="ERROR")
        layout.label(text=f"Warnings: {counts['warning']}", icon="ERROR")
        if items:
            layout.template_list("BONE_PHYSICS_UL_diagnostics", "", settings, "diagnostic_items", settings, "active_diagnostic_index")
        else:
            layout.label(text="No current diagnostics.", icon="CHECKMARK")


CLASSES = (BONE_PHYSICS_PT_root, BONE_PHYSICS_PT_parameters, BONE_PHYSICS_PT_runtime, BONE_PHYSICS_PT_constraints, BONE_PHYSICS_PT_collision_resources, BONE_PHYSICS_PT_diagnostics)
