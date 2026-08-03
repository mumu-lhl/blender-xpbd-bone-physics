"""Blender UI actions for portable bone-parameter presets."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import bpy

from ..preset_io import (
    EXACT_ARMATURE_PRESET_KIND, PRESET_KIND, PresetError, apply_exact_armature, copy_preset,
    delete_preset, load_exact_armature_preset, load_parameter_preset, preset_directories,
    rename_preset, sanitize_preset_filename, save_exact_armature_preset, save_parameter_preset,
    serialize_exact_armature,
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


_ENDPOINT_FIELDS = tuple(field for fields in _FIELDS.values() for field in fields)
_SYSTEM_FIELDS = ("id", "stable_id", "name", "enabled", "interpolation", "collision_resource_ids",
                  "collide_with_system_ids", "self_collision", "cross_chain_collision")
_CHAIN_FIELDS = ("id", "stable_id", "head_uuid", "tail_uuid", "target_id", "closed")
_STRUCTURAL_FIELDS = ("first_constraint_id", "chain_ids", "samples", "diagonals", "closed", "compliance", "damping")
_COLLIDER_FIELDS = ("id", "stable_id", "name", "enabled", "kind", "shape", "binding", "object_name",
                    "bone_name", "translation", "rotation", "radius", "local_a", "local_b", "half_extents",
                    "thickness", "simplification", "excluded_materials", "excluded_regions", "ignored_body_regions",
                    "weight_merges", "member_ids")


def _plain(item, fields):
    return {field: list(getattr(item, field)) if field in {"inertia_scale", "xyz_lower", "xyz_upper", "swing_axis", "translation", "rotation", "local_a", "local_b", "half_extents"} else getattr(item, field) for field in fields}


def _exact_snapshot(armature):
    bones = []
    for bone in armature.data.bones:
        matrix = tuple(float(value) for row in bone.matrix_local for value in row)
        bones.append(SimpleNamespace(uuid=bone.name, name=bone.name,
            parent_uuid=bone.parent.name if bone.parent else None, length=float(bone.length), rest_matrix=matrix))
    return SimpleNamespace(bones=tuple(bones))


def _exact_chain_path(armature, chain):
    current = armature.pose.bones.get(chain.tail_uuid)
    path = []
    while current is not None:
        path.append(current.name)
        if current.name == chain.head_uuid:
            return list(reversed(path))
        current = current.parent
    raise PresetError(f"chain {chain.id!r} has an invalid bone path")


def _exact_bundle(settings, armature=None):
    systems = []
    for system in settings.physics_systems:
        systems.append({**_plain(system, _SYSTEM_FIELDS),
            "head": _plain(system.head, _ENDPOINT_FIELDS), "tail": _plain(system.tail, _ENDPOINT_FIELDS),
            "curve_points": [_plain(item, ("field", "s", "value", "tangent")) for item in system.curve_points],
            "overrides": [{"bone_name": item.bone_name, "parameters": _plain(item.parameters, _ENDPOINT_FIELDS)} for item in system.overrides],
            "chains": [{**_plain(item, _CHAIN_FIELDS),
                        "bone_uuids": _exact_chain_path(armature, item) if armature is not None else [],
                        "curve_points": [_plain(point, ("field", "s", "value", "tangent")) for point in item.curve_points]} for item in system.chains],
            "structural_groups": [_plain(item, _STRUCTURAL_FIELDS) for item in system.structural_groups],
        })
    return {"id": settings.id, "next_stable_id": settings.next_stable_id,
            "gravity": list(settings.gravity), "fixed_dt": settings.fixed_dt,
            "physics_systems": systems,
            "collision_resources": [_plain(item, _COLLIDER_FIELDS) for item in settings.collision_resources]}


def _replace_collection(collection, values, writer):
    while len(collection):
        collection.remove(len(collection) - 1)
    for value in values:
        writer(collection.add(), value)


def _write_values(item, value):
    for field, current in value.items():
        if not isinstance(current, (dict, list)):
            setattr(item, field, current)
        elif field in {"inertia_scale", "xyz_lower", "xyz_upper", "swing_axis", "translation", "rotation", "local_a", "local_b", "half_extents", "gravity"}:
            setattr(item, field, current)


def _write_system(system, value):
    _write_values(system, value)
    _write_values(system.head, value["head"])
    _write_values(system.tail, value["tail"])
    _replace_collection(system.curve_points, value["curve_points"], _write_values)
    _replace_collection(system.overrides, value["overrides"], lambda item, data: (_write_values(item, {"bone_name": data["bone_name"]}), _write_values(item.parameters, data["parameters"])))
    _replace_collection(system.chains, value["chains"], lambda item, data: (_write_values(item, data), _replace_collection(item.curve_points, data["curve_points"], _write_values)))
    _replace_collection(system.structural_groups, value["structural_groups"], _write_values)


def _replace_exact_bundle(settings, replacement):
    _write_values(settings, replacement)
    _replace_collection(settings.physics_systems, replacement["physics_systems"], _write_system)
    _replace_collection(settings.collision_resources, replacement["collision_resources"], _write_values)
    settings.active_system_index = min(settings.active_system_index, max(0, len(settings.physics_systems) - 1))
    settings.active_collision_resource_index = min(settings.active_collision_resource_index, max(0, len(settings.collision_resources) - 1))


class _ExactOwner:
    def __init__(self, settings):
        self.settings = settings

    def replace_exact_authoring(self, replacement):
        _replace_exact_bundle(self.settings, replacement)


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


class BONE_PHYSICS_OT_exact_preset_save(bpy.types.Operator):
    bl_idname = "bone_physics.exact_preset_save"
    bl_label = "Save Exact Armature Preset"
    bl_options = {"REGISTER", "UNDO"}

    name: bpy.props.StringProperty(name="Preset Name", default="Exact Armature")
    confirmed: bpy.props.BoolProperty(default=False, options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return _active_armature(context) is not None

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(self, width=460)

    def draw(self, _context):
        self.layout.prop(self, "name")
        self.layout.label(text="Saves all systems, chains, constraints, and collision resources.")

    def execute(self, context):
        armature = _active_armature(context)
        try:
            directory = preset_directories(EXACT_ARMATURE_PRESET_KIND, create=True)[0]
            path = directory / sanitize_preset_filename(self.name)
            if path.exists() and not self.confirmed:
                self.confirmed = True
                return context.window_manager.invoke_props_dialog(
                    self, width=460, title="Overwrite Exact Armature Preset",
                    confirm_text="Overwrite", cancel_default=True,
                )
            save_exact_armature_preset(path, serialize_exact_armature(
                _exact_bundle(armature.bone_physics, armature), _exact_snapshot(armature), self.name
            ))
        except (OSError, PresetError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_exact_preset_apply(bpy.types.Operator):
    bl_idname = "bone_physics.exact_preset_apply"
    bl_label = "Apply Exact Armature Preset"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    confirmed: bpy.props.BoolProperty(default=False, options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return _active_armature(context) is not None

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(self, width=460)

    def draw(self, _context):
        self.layout.label(text=Path(self.filepath).name)
        self.layout.label(text="This replaces every configured physics and collision setting.")

    def execute(self, context):
        armature = _active_armature(context)
        try:
            preset = load_exact_armature_preset(self.filepath)
            # Validation happens inside apply before any RNA collection is cleared.
            apply_exact_armature(_ExactOwner(armature.bone_physics), preset, _exact_snapshot(armature))
            from ..runtime_registry import REGISTRY, mark_dirty
            REGISTRY.mark_validated(armature.as_pointer())
            mark_dirty(armature.as_pointer(), "world")
            REGISTRY.flush_dirty()
        except (OSError, PresetError, RuntimeError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_exact_preset_delete(bpy.types.Operator):
    bl_idname = "bone_physics.exact_preset_delete"
    bl_label = "Delete Exact Armature Preset"
    bl_options = {"REGISTER", "UNDO"}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(
            self, width=420, title="Delete Exact Armature Preset",
            confirm_text="Delete", cancel_default=True,
        )

    def draw(self, _context):
        self.layout.label(text=Path(self.filepath).name)

    def execute(self, _context):
        try:
            delete_preset(self.filepath, preset_directories(EXACT_ARMATURE_PRESET_KIND, create=True)[0])
        except (OSError, PresetError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


CLASSES = (
    BONE_PHYSICS_OT_preset_apply, BONE_PHYSICS_OT_preset_save, BONE_PHYSICS_OT_preset_delete,
    BONE_PHYSICS_OT_preset_copy, BONE_PHYSICS_OT_preset_rename, BONE_PHYSICS_OT_exact_preset_save,
    BONE_PHYSICS_OT_exact_preset_apply, BONE_PHYSICS_OT_exact_preset_delete,
)
