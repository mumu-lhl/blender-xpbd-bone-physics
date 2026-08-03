"""Blender RNA storage for armature physics authoring data.

This module is deliberately Blender-only.  The immutable authoring records and
their validation remain importable without :mod:`bpy`.
"""

from __future__ import annotations

import math

import bpy


_CURVE_INTERPOLATION_ITEMS = (
    ("CONSTANT", "Constant", "Hold the preceding value."),
    ("LINEAR", "Linear", "Interpolate linearly."),
    ("CUBIC_HERMITE", "Cubic Hermite", "Interpolate using curve tangents."),
)
_ANGULAR_LIMIT_MODE_ITEMS = (
    ("FREE", "Free", "Do not constrain angular motion."),
    ("XYZ", "Local XYZ", "Constrain local XYZ rotation."),
    ("SWING_TWIST", "Swing-Twist", "Constrain swing and twist."),
)


def _mark_dirty(properties: bpy.types.PropertyGroup, category: str) -> None:
    """Queue a rebuild request when the optional runtime registry is present."""
    try:
        from . import runtime_registry

        mark_dirty = runtime_registry.mark_dirty
    except (AttributeError, ImportError):
        return
    mark_dirty(properties.id_data.as_pointer(), category)


def _curve_update(properties: bpy.types.PropertyGroup, _context: bpy.types.Context) -> None:
    _mark_dirty(properties, "parameters")


def _endpoint_update(
    properties: bpy.types.PropertyGroup, _context: bpy.types.Context
) -> None:
    _mark_dirty(properties, "parameters")


def _chain_update(properties: bpy.types.PropertyGroup, _context: bpy.types.Context) -> None:
    _mark_dirty(properties, "chains")


def _system_update(properties: bpy.types.PropertyGroup, _context: bpy.types.Context) -> None:
    _mark_dirty(properties, "systems")


def _armature_update(
    properties: bpy.types.PropertyGroup, _context: bpy.types.Context
) -> None:
    _mark_dirty(properties, "world")


class CurvePointProperties(bpy.types.PropertyGroup):
    s: bpy.props.FloatProperty(name="Position", min=0.0, max=1.0, update=_curve_update)
    value: bpy.props.FloatProperty(name="Value", default=0.0, update=_curve_update)
    tangent: bpy.props.FloatProperty(name="Tangent", default=0.0, update=_curve_update)


class EndpointProperties(bpy.types.PropertyGroup):
    inverse_mass: bpy.props.FloatProperty(name="Inverse Mass", default=1.0, min=0.0, update=_endpoint_update)
    inertia_scale: bpy.props.FloatVectorProperty(name="Inertia Scale", size=3, default=(1.0, 1.0, 1.0), min=0.0, update=_endpoint_update)
    gravity_scale: bpy.props.FloatProperty(name="Gravity Scale", default=1.0, update=_endpoint_update)
    translation_inertia: bpy.props.FloatProperty(name="Translation Inertia", default=1.0, min=0.0, update=_endpoint_update)
    rotation_inertia: bpy.props.FloatProperty(name="Rotation Inertia", default=1.0, min=0.0, update=_endpoint_update)
    linear_damping: bpy.props.FloatProperty(name="Linear Damping", default=0.0, min=0.0, update=_endpoint_update)
    angular_damping: bpy.props.FloatProperty(name="Angular Damping", default=0.0, min=0.0, update=_endpoint_update)
    joint_compliance: bpy.props.FloatProperty(name="Joint Compliance", default=0.0, min=0.0, update=_endpoint_update)
    joint_damping: bpy.props.FloatProperty(name="Joint Damping", default=0.0, min=0.0, update=_endpoint_update)
    recovery_compliance: bpy.props.FloatProperty(name="Recovery Compliance", default=0.0, min=0.0, update=_endpoint_update)
    recovery_damping: bpy.props.FloatProperty(name="Recovery Damping", default=0.0, min=0.0, update=_endpoint_update)
    angular_limit_mode: bpy.props.EnumProperty(name="Angular Limit", items=_ANGULAR_LIMIT_MODE_ITEMS, default="FREE", update=_endpoint_update)
    xyz_lower: bpy.props.FloatVectorProperty(name="XYZ Lower", size=3, default=(-math.pi, -math.pi, -math.pi), subtype="EULER", update=_endpoint_update)
    xyz_upper: bpy.props.FloatVectorProperty(name="XYZ Upper", size=3, default=(math.pi, math.pi, math.pi), subtype="EULER", update=_endpoint_update)
    swing_axis: bpy.props.FloatVectorProperty(name="Swing Axis", size=3, default=(0.0, 1.0, 0.0), update=_endpoint_update)
    max_swing: bpy.props.FloatProperty(name="Maximum Swing", default=math.pi, min=0.0, max=math.pi, subtype="ANGLE", update=_endpoint_update)
    min_twist: bpy.props.FloatProperty(name="Minimum Twist", default=-math.pi, min=-math.pi, max=math.pi, subtype="ANGLE", update=_endpoint_update)
    max_twist: bpy.props.FloatProperty(name="Maximum Twist", default=math.pi, min=-math.pi, max=math.pi, subtype="ANGLE", update=_endpoint_update)
    radius: bpy.props.FloatProperty(name="Capsule Radius", default=0.1, min=0.0, update=_endpoint_update)
    margin: bpy.props.FloatProperty(name="Collision Margin", default=0.01, min=0.0, update=_endpoint_update)
    capsule_head_inset: bpy.props.FloatProperty(name="Capsule Head Inset", default=0.0, min=0.0, update=_endpoint_update)
    capsule_tail_inset: bpy.props.FloatProperty(name="Capsule Tail Inset", default=0.0, min=0.0, update=_endpoint_update)
    friction: bpy.props.FloatProperty(name="Friction", default=0.5, min=0.0, update=_endpoint_update)
    restitution: bpy.props.FloatProperty(name="Restitution", default=0.0, min=0.0, max=1.0, update=_endpoint_update)
    interpolation: bpy.props.EnumProperty(name="Interpolation", items=_CURVE_INTERPOLATION_ITEMS, default="LINEAR", update=_endpoint_update)


class ChainProperties(bpy.types.PropertyGroup):
    id: bpy.props.StringProperty(name="Chain ID", update=_chain_update)
    stable_id: bpy.props.IntProperty(name="Stable ID", min=0, update=_chain_update)
    head_uuid: bpy.props.StringProperty(name="Head Bone UUID", update=_chain_update)
    tail_uuid: bpy.props.StringProperty(name="Tail Bone UUID", update=_chain_update)
    target_id: bpy.props.StringProperty(name="Target ID", update=_chain_update)
    closed: bpy.props.BoolProperty(name="Closed", default=False, update=_chain_update)
    curve_points: bpy.props.CollectionProperty(type=CurvePointProperties)


class PhysicsSystemProperties(bpy.types.PropertyGroup):
    id: bpy.props.StringProperty(name="System ID", update=_system_update)
    stable_id: bpy.props.IntProperty(name="Stable ID", min=0, update=_system_update)
    name: bpy.props.StringProperty(name="Name", update=_system_update)
    enabled: bpy.props.BoolProperty(name="Enabled", default=True, update=_system_update)
    head: bpy.props.PointerProperty(type=EndpointProperties)
    tail: bpy.props.PointerProperty(type=EndpointProperties)
    chains: bpy.props.CollectionProperty(type=ChainProperties)


class ArmaturePhysicsProperties(bpy.types.PropertyGroup):
    schema_version: bpy.props.IntProperty(name="Schema Version", default=1, min=1, update=_armature_update)
    id: bpy.props.StringProperty(name="Armature ID", update=_armature_update)
    next_stable_id: bpy.props.IntProperty(name="Next Stable ID", default=1, min=1, update=_armature_update)
    gravity: bpy.props.FloatVectorProperty(name="Gravity", size=3, default=(0.0, -9.81, 0.0), update=_armature_update)
    fixed_dt: bpy.props.FloatProperty(name="Fixed Time Step", default=1.0 / 120.0, min=0.0, update=_armature_update)
    global_enabled: bpy.props.BoolProperty(name="Enable Physics", default=True, update=_armature_update)
    workspace: bpy.props.EnumProperty(
        name="Workspace",
        items=(
            ("SETUP", "Setup", "Configure physics systems and chains."),
            ("SIMULATE", "Simulate", "Control simulation and baking."),
            ("DIAGNOSTICS", "Diagnostics", "Inspect validation and runtime information."),
        ),
        default="SETUP",
        update=_armature_update,
    )
    active_system_index: bpy.props.IntProperty(
        name="Active Physics System", default=0, min=0, update=_armature_update
    )
    physics_systems: bpy.props.CollectionProperty(type=PhysicsSystemProperties)


CLASSES = (
    CurvePointProperties,
    EndpointProperties,
    ChainProperties,
    PhysicsSystemProperties,
    ArmaturePhysicsProperties,
)


def register_properties() -> None:
    for cls in CLASSES:
        if not getattr(cls, "is_registered", False):
            bpy.utils.register_class(cls)
    if not hasattr(bpy.types.Object, "bone_physics"):
        bpy.types.Object.bone_physics = bpy.props.PointerProperty(
            type=ArmaturePhysicsProperties
        )


def unregister_properties() -> None:
    if hasattr(bpy.types.Object, "bone_physics"):
        del bpy.types.Object.bone_physics
    for cls in reversed(CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
