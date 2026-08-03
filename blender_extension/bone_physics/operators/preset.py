"""Blender UI actions for portable bone-parameter presets."""

from __future__ import annotations

from pathlib import Path

import bpy

from ..preset_io import (
    PRESET_KIND, PresetError, copy_preset, delete_preset, load_parameter_preset, preset_directories,
    rename_preset, sanitize_preset_filename, save_parameter_preset,
)
from .system import _active_armature


_FIELDS = {
    "MOTION": (
        "inverse_mass", "inertia_scale", "gravity_scale", "translation_inertia",
        "rotation_inertia", "linear_damping", "angular_damping", "recovery_compliance",
        "recovery_damping", "interpolation",
    ),
    "LOCAL_CONSTRAINT": (
        "joint_compliance", "joint_damping", "angular_compliance",
        "angular_damping_constraint", "angular_limit_mode", "xyz_lower", "xyz_upper",
        "swing_axis", "max_swing", "min_twist", "max_twist",
    ),
    "COLLISION": (
        "radius", "margin", "capsule_head_inset", "capsule_tail_inset", "friction",
        "restitution",
    ),
}


def _copy_parameters(source: dict, target: bpy.types.PropertyGroup, mask: str) -> None:
    fields = tuple(field for group, values in _FIELDS.items() if mask in {"ALL", group} for field in values)
    for field in fields:
        setattr(target, field, source[field])


def _active_system(context: bpy.types.Context):
    armature = _active_armature(context)
    if armature is None or not armature.bone_physics.physics_systems:
        return None
    settings = armature.bone_physics
    return settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]


def _chain_bone_names(armature: bpy.types.Object, chain: bpy.types.PropertyGroup) -> set[str]:
    """Return every member of the stored head-to-tail armature chain."""
    bones = armature.pose.bones
    tail = bones.get(chain.tail_uuid)
    names: set[str] = set()
    while tail is not None:
        names.add(tail.name)
        if tail.name == chain.head_uuid:
            return names
        tail = tail.parent
    return set()


def _selected_override_names(context: bpy.types.Context, system: bpy.types.PropertyGroup, target: str) -> set[str]:
    if target == "SELECTED_BONES":
        return {bone.name for bone in context.selected_pose_bones or ()}
    chains = system.chains if target == "CHAINS" else (chain for chain in system.chains if chain.selected)
    return {name for chain in chains for name in _chain_bone_names(context.object, chain)}


def _system_preset(system: bpy.types.PropertyGroup, name: str) -> dict:
    def endpoint(item):
        return {field: list(getattr(item, field)) if field in {"inertia_scale", "xyz_lower", "xyz_upper", "swing_axis"} else getattr(item, field)
                for fields in _FIELDS.values() for field in fields}
    return {
        "kind": PRESET_KIND, "schema_version": 1, "name": name,
        "head": endpoint(system.head), "tail": endpoint(system.tail),
        "overrides": [
            {"bone_name": override.bone_name, "parameters": endpoint(override.parameters)}
            for override in system.overrides
        ],
    }


class BONE_PHYSICS_OT_preset_apply(bpy.types.Operator):
    bl_idname = "bone_physics.preset_apply"
    bl_label = "Apply Bone Parameter Preset"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    mask: bpy.props.EnumProperty(
        name="Apply", items=(("ALL", "All", "Apply all parameter groups."),
                              ("MOTION", "Motion", "Apply motion and recovery parameters."),
                              ("LOCAL_CONSTRAINT", "Local Constraint", "Apply joint and angular limits."),
                              ("COLLISION", "Collision", "Apply capsule and contact parameters.")),
        default="ALL",
    )
    target: bpy.props.EnumProperty(
        name="Target", items=(
            ("ALL", "All", "Apply to both endpoints and all matching overrides."),
            ("HEAD", "Head", "Apply to the system head endpoint."),
            ("TAIL", "Tail", "Apply to the system tail endpoint."),
            ("SELECTED_BONES", "Selected Bones", "Apply only matching selected-bone overrides."),
            ("CHAINS", "Chains", "Apply only overrides on this system's chain endpoints."),
            ("SELECTED_CHAINS", "Selected Chains", "Apply only overrides on selected chain endpoints."),
        ), default="ALL",
    )

    @classmethod
    def poll(cls, context):
        return _active_system(context) is not None

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(self, width=420, title="Apply Bone Parameter Preset")

    def draw(self, _context):
        self.layout.label(text=Path(self.filepath).name)
        self.layout.label(text=self.filepath)
        self.layout.prop(self, "mask")
        self.layout.prop(self, "target")

    def execute(self, context):
        system = _active_system(context)
        try:
            preset = load_parameter_preset(self.filepath)
            if self.target in {"ALL", "HEAD"}:
                _copy_parameters(preset["head"], system.head, self.mask)
            if self.target in {"ALL", "TAIL"}:
                _copy_parameters(preset["tail"], system.tail, self.mask)
            by_name = {override.bone_name: override for override in system.overrides}
            names = None if self.target == "ALL" else _selected_override_names(context, system, self.target)
            for source in preset["overrides"]:
                if names is not None and source["bone_name"] not in names:
                    continue
                target = by_name.get(source["bone_name"])
                if target is None:
                    target = system.overrides.add()
                    target.bone_name = source["bone_name"]
                _copy_parameters(source["parameters"], target.parameters, self.mask)
        except (OSError, PresetError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_preset_save(bpy.types.Operator):
    bl_idname = "bone_physics.preset_save"
    bl_label = "Save Bone Parameter Preset"
    bl_options = {"REGISTER", "UNDO"}

    name: bpy.props.StringProperty(name="Preset Name", default="Bone Parameters")
    filepath: bpy.props.StringProperty(subtype="FILE_PATH", options={"HIDDEN"})
    confirmed: bpy.props.BoolProperty(default=False, options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return _active_system(context) is not None

    def invoke(self, context, _event):
        directory = preset_directories(PRESET_KIND, create=True)[0]
        self.filepath = str(directory / sanitize_preset_filename(self.name))
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, _context):
        self.layout.prop(self, "name")
        self.layout.label(text=f"User preset: {self.filepath}")

    def execute(self, context):
        directory = preset_directories(PRESET_KIND, create=True)[0]
        try:
            path = directory / sanitize_preset_filename(self.name)
            if path.exists() and not self.confirmed:
                self.filepath = str(path)
                self.confirmed = True
                return context.window_manager.invoke_props_dialog(
                    self, width=420, title="Overwrite Preset", confirm_text="Overwrite", cancel_default=True
                )
            save_parameter_preset(path, _system_preset(_active_system(context), self.name.strip()))
        except (OSError, PresetError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_preset_delete(bpy.types.Operator):
    bl_idname = "bone_physics.preset_delete"
    bl_label = "Delete Bone Parameter Preset"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(
            self, width=420, title="Delete Preset", confirm_text="Delete", cancel_default=True
        )

    def draw(self, _context):
        self.layout.label(text=Path(self.filepath).name)
        self.layout.label(text=self.filepath)

    def execute(self, _context):
        try:
            delete_preset(self.filepath, preset_directories(PRESET_KIND, create=True)[0])
        except (OSError, PresetError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_preset_copy(bpy.types.Operator):
    bl_idname = "bone_physics.preset_copy"
    bl_label = "Copy Bone Parameter Preset"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    name: bpy.props.StringProperty(name="Copy Name")

    def invoke(self, context, _event):
        try:
            self.name = f"{load_parameter_preset(self.filepath)['name']} Copy"
        except (OSError, PresetError):
            self.name = f"{Path(self.filepath).stem} Copy"
        return context.window_manager.invoke_props_dialog(self, width=420, title="Copy Preset")

    def draw(self, _context):
        self.layout.label(text=Path(self.filepath).name)
        self.layout.label(text=self.filepath)
        self.layout.prop(self, "name")

    def execute(self, _context):
        try:
            copy_preset(self.filepath, preset_directories(PRESET_KIND, create=True)[0], self.name)
        except (OSError, PresetError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_preset_rename(bpy.types.Operator):
    bl_idname = "bone_physics.preset_rename"
    bl_label = "Rename Bone Parameter Preset"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    name: bpy.props.StringProperty(name="New Name")

    def invoke(self, context, _event):
        self.name = Path(self.filepath).stem
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, _context):
        try:
            rename_preset(self.filepath, self.name, preset_directories(PRESET_KIND, create=True)[0])
        except (OSError, PresetError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


CLASSES = (BONE_PHYSICS_OT_preset_apply, BONE_PHYSICS_OT_preset_save, BONE_PHYSICS_OT_preset_delete, BONE_PHYSICS_OT_preset_copy, BONE_PHYSICS_OT_preset_rename)
