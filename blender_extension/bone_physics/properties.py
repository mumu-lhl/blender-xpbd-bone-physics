"""Blender RNA storage for armature physics authoring data.

This module is deliberately Blender-only.  The immutable authoring records and
their validation remain importable without :mod:`bpy`.
"""

from __future__ import annotations

import math

import bpy

from .model import ChainField


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
ENDPOINT_FIELD_ITEMS = (
    ("INVERSE_MASS", "Inverse Mass", "Inverse mass."),
    ("INERTIA_SCALE", "Inertia Scale", "Local inertia scale vector."),
    ("GRAVITY_SCALE", "Gravity Scale", "Gravity multiplier."),
    ("TRANSLATION_INERTIA", "Translation Inertia", "Translation animation inertia."),
    ("ROTATION_INERTIA", "Rotation Inertia", "Rotation animation inertia."),
    ("LINEAR_DAMPING", "Linear Damping", "Linear damping."),
    ("ANGULAR_DAMPING", "Angular Damping", "Angular damping."),
    ("JOINT_COMPLIANCE", "Joint Compliance", "Parent joint compliance."),
    ("JOINT_DAMPING", "Joint Damping", "Parent joint damping."),
    ("ANGULAR_COMPLIANCE", "Angular Compliance", "Angular constraint compliance."),
    ("ANGULAR_DAMPING_CONSTRAINT", "Angular Constraint Damping", "Angular constraint damping."),
    ("RECOVERY_COMPLIANCE", "Recovery Compliance", "Animation recovery compliance."),
    ("RECOVERY_DAMPING", "Recovery Damping", "Animation recovery damping."),
    ("ANGULAR_LIMIT_MODE", "Angular Limit", "Angular limit model."),
    ("XYZ_LOWER", "XYZ Lower", "Local XYZ lower limits."),
    ("XYZ_UPPER", "XYZ Upper", "Local XYZ upper limits."),
    ("SWING_AXIS", "Swing Axis", "Swing cone axis."),
    ("MAX_SWING", "Maximum Swing", "Maximum swing angle."),
    ("MIN_TWIST", "Minimum Twist", "Minimum twist angle."),
    ("MAX_TWIST", "Maximum Twist", "Maximum twist angle."),
    ("RADIUS", "Capsule Radius", "Capsule radius."),
    ("MARGIN", "Collision Margin", "Collision margin."),
    ("CAPSULE_HEAD_INSET", "Capsule Head Inset", "Capsule head inset."),
    ("CAPSULE_TAIL_INSET", "Capsule Tail Inset", "Capsule tail inset."),
    ("FRICTION", "Friction", "Contact friction."),
    ("RESTITUTION", "Restitution", "Contact restitution."),
)
CURVE_FIELD_ITEMS = tuple(
    (field.name, field.name.replace("_", " ").title(), "Per-field curve")
    for field in ChainField
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


def _constraint_update(properties: bpy.types.PropertyGroup, _context: bpy.types.Context) -> None:
    _mark_dirty(properties, "constraints")


def _system_update(properties: bpy.types.PropertyGroup, _context: bpy.types.Context) -> None:
    _mark_dirty(properties, "systems")


def _collision_update(properties: bpy.types.PropertyGroup, _context: bpy.types.Context) -> None:
    _mark_dirty(properties, "collisions")


def _armature_update(
    properties: bpy.types.PropertyGroup, _context: bpy.types.Context
) -> None:
    _mark_dirty(properties, "world")


def _overlay_update(
    properties: bpy.types.PropertyGroup, _context: bpy.types.Context
) -> None:
    """Synchronize viewport draw handlers with the persisted overlay toggle."""
    try:
        from . import viewport
        import bpy as _bpy

        for screen in _bpy.data.screens:
            for area in screen.areas:
                if area.type != "VIEW_3D":
                    continue
                if properties.show_viewport_overlays:
                    viewport.enable_overlay(area)
                else:
                    viewport.disable_overlay(area)
    except (AttributeError, ImportError, RuntimeError):
        return


class DiagnosticItemProperties(bpy.types.PropertyGroup):
    code: bpy.props.StringProperty(name="Code")
    severity: bpy.props.EnumProperty(
        name="Severity",
        items=(("error", "Error", ""), ("warning", "Warning", ""), ("info", "Info", "")),
        default="error",
    )
    entity_kind: bpy.props.StringProperty(name="Entity Kind")
    entity_id: bpy.props.StringProperty(name="Entity ID")
    stable_id: bpy.props.IntProperty(name="Stable ID", min=0)
    object_name: bpy.props.StringProperty(name="Object")


def _global_enabled_update(
    properties: bpy.types.PropertyGroup, _context: bpy.types.Context
) -> None:
    _mark_dirty(properties, "enabled")


class CurvePointProperties(bpy.types.PropertyGroup):
    field: bpy.props.EnumProperty(name="Field", items=CURVE_FIELD_ITEMS, default="GRAVITY_SCALE", update=_curve_update)
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
    angular_compliance: bpy.props.FloatProperty(name="Angular Compliance", default=0.0, min=0.0, update=_endpoint_update)
    angular_damping_constraint: bpy.props.FloatProperty(name="Angular Constraint Damping", default=0.0, min=0.0, update=_endpoint_update)
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


class BoneOverrideProperties(bpy.types.PropertyGroup):
    bone_name: bpy.props.StringProperty(name="Bone", update=_system_update)
    parameters: bpy.props.PointerProperty(type=EndpointProperties)


class ChainProperties(bpy.types.PropertyGroup):
    id: bpy.props.StringProperty(name="Chain ID", update=_chain_update)
    stable_id: bpy.props.IntProperty(name="Stable ID", min=0, max=0x7FFFFFFF, update=_chain_update)
    head_uuid: bpy.props.StringProperty(name="Head Bone UUID", update=_chain_update)
    tail_uuid: bpy.props.StringProperty(name="Tail Bone UUID", update=_chain_update)
    target_id: bpy.props.StringProperty(name="Target ID", update=_chain_update)
    closed: bpy.props.BoolProperty(name="Closed", default=False, update=_chain_update)
    selected: bpy.props.BoolProperty(name="Selected", default=False, update=_constraint_update)
    curve_points: bpy.props.CollectionProperty(type=CurvePointProperties)


class StructuralGroupProperties(bpy.types.PropertyGroup):
    """Persisted authoring data for one generated cross-link lattice."""

    first_constraint_id: bpy.props.IntProperty(name="First Constraint ID", min=1, max=0x7FFFFFFF, update=_constraint_update)
    chain_ids: bpy.props.StringProperty(name="Chain IDs", update=_constraint_update)
    samples: bpy.props.StringProperty(name="Samples", default="0.0, 1.0", update=_constraint_update)
    diagonals: bpy.props.BoolProperty(name="Diagonals", default=False, update=_constraint_update)
    closed: bpy.props.BoolProperty(name="Closed", default=False, update=_constraint_update)
    compliance: bpy.props.FloatProperty(name="Compliance", default=1.0e-5, min=0.0, update=_constraint_update)
    damping: bpy.props.FloatProperty(name="Damping", default=0.0, min=0.0, update=_constraint_update)


class PhysicsSystemProperties(bpy.types.PropertyGroup):
    id: bpy.props.StringProperty(name="System ID", update=_system_update)
    stable_id: bpy.props.IntProperty(name="Stable ID", min=0, max=0x7FFFFFFF, update=_system_update)
    name: bpy.props.StringProperty(name="Name", update=_system_update)
    enabled: bpy.props.BoolProperty(name="Enabled", default=True, update=_system_update)
    selected: bpy.props.BoolProperty(name="Selected", default=False, update=_system_update)
    show_motion: bpy.props.BoolProperty(name="Motion", default=False)
    show_recovery: bpy.props.BoolProperty(name="Recovery", default=False)
    show_angular_limits: bpy.props.BoolProperty(name="Angular Limits", default=False)
    show_collision: bpy.props.BoolProperty(name="Collision", default=False)
    show_advanced: bpy.props.BoolProperty(name="Advanced", default=False)
    active_override_index: bpy.props.IntProperty(name="Active Override", default=0, min=0)
    active_curve_point_index: bpy.props.IntProperty(name="Active Curve Point", default=0, min=0)
    active_structural_group_index: bpy.props.IntProperty(name="Active Structural Group", default=0, min=0)
    head: bpy.props.PointerProperty(type=EndpointProperties)
    tail: bpy.props.PointerProperty(type=EndpointProperties)
    interpolation: bpy.props.EnumProperty(name="Interpolation", items=_CURVE_INTERPOLATION_ITEMS, default="LINEAR", update=_system_update)
    curve_points: bpy.props.CollectionProperty(type=CurvePointProperties)
    overrides: bpy.props.CollectionProperty(type=BoneOverrideProperties)
    chains: bpy.props.CollectionProperty(type=ChainProperties)
    structural_groups: bpy.props.CollectionProperty(type=StructuralGroupProperties)
    collision_resource_ids: bpy.props.StringProperty(name="Collision Resources", update=_collision_update)
    collide_with_system_ids: bpy.props.StringProperty(name="Collide With Systems", update=_collision_update)
    self_collision: bpy.props.BoolProperty(name="Self Collision", default=True, update=_collision_update)
    cross_chain_collision: bpy.props.BoolProperty(name="Cross-Chain Collision", default=True, update=_collision_update)


class CollisionResourceProperties(bpy.types.PropertyGroup):
    id: bpy.props.StringProperty(name="Resource ID", update=_collision_update)
    stable_id: bpy.props.IntProperty(name="Stable ID", min=1, max=0x7FFFFFFF, update=_collision_update)
    name: bpy.props.StringProperty(name="Name", update=_collision_update)
    enabled: bpy.props.BoolProperty(name="Enabled", default=True, update=_collision_update)
    kind: bpy.props.EnumProperty(name="Kind", items=(("ANALYTIC", "Analytic", "Sphere, capsule, or rounded box."), ("MESH", "Mesh", "Pre-physics deforming collision mesh."), ("COLLECTION", "Collection", "Reusable group of collision resources.")), default="ANALYTIC", update=_collision_update)
    shape: bpy.props.EnumProperty(name="Shape", items=(("SPHERE", "Sphere", ""), ("CAPSULE", "Capsule", ""), ("ROUNDED_BOX", "Rounded Box", "")), default="SPHERE", update=_collision_update)
    binding: bpy.props.EnumProperty(name="Bind To", items=(("WORLD", "World", ""), ("OBJECT", "Object", ""), ("BONE", "Bone", "")), default="WORLD", update=_collision_update)
    object_name: bpy.props.StringProperty(name="Object", update=_collision_update)
    bone_name: bpy.props.StringProperty(name="Bone", update=_collision_update)
    translation: bpy.props.FloatVectorProperty(name="Translation", size=3, default=(0.0, 0.0, 0.0), update=_collision_update)
    rotation: bpy.props.FloatVectorProperty(name="Rotation", size=4, default=(0.0, 0.0, 0.0, 1.0), subtype="QUATERNION", update=_collision_update)
    radius: bpy.props.FloatProperty(name="Radius", default=0.1, min=0.0, update=_collision_update)
    local_a: bpy.props.FloatVectorProperty(name="Capsule A", size=3, default=(0.0, 0.0, 0.0), update=_collision_update)
    local_b: bpy.props.FloatVectorProperty(name="Capsule B", size=3, default=(0.0, 1.0, 0.0), update=_collision_update)
    half_extents: bpy.props.FloatVectorProperty(name="Half Extents", size=3, default=(0.1, 0.1, 0.1), min=0.0, update=_collision_update)
    thickness: bpy.props.FloatProperty(name="Thickness", default=0.0, min=0.0, update=_collision_update)
    simplification: bpy.props.FloatProperty(name="Simplification", default=0.0, min=0.0, update=_collision_update)
    excluded_materials: bpy.props.StringProperty(name="Excluded Materials", update=_collision_update)
    excluded_regions: bpy.props.StringProperty(name="Excluded Regions", update=_collision_update)
    ignored_body_regions: bpy.props.StringProperty(name="Body Region Ignores", update=_collision_update)
    weight_merges: bpy.props.StringProperty(name="Weight Merges", update=_collision_update)
    member_ids: bpy.props.StringProperty(name="Members", update=_collision_update)


class ArmaturePhysicsProperties(bpy.types.PropertyGroup):
    schema_version: bpy.props.IntProperty(name="Schema Version", default=1, min=1, update=_armature_update)
    id: bpy.props.StringProperty(name="Armature ID", update=_armature_update)
    next_stable_id: bpy.props.IntProperty(name="Next Stable ID", default=1, min=1, max=0x7FFFFFFF, update=_armature_update)
    gravity: bpy.props.FloatVectorProperty(name="Gravity", size=3, default=(0.0, -9.81, 0.0), update=_armature_update)
    fixed_dt: bpy.props.FloatProperty(name="Fixed Time Step", default=1.0 / 120.0, min=0.0, update=_armature_update)
    global_enabled: bpy.props.BoolProperty(name="Enable Physics", default=True, update=_global_enabled_update)
    show_viewport_overlays: bpy.props.BoolProperty(
        name="Viewport Overlays", default=False, update=_overlay_update
    )
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
    collision_resources: bpy.props.CollectionProperty(type=CollisionResourceProperties)
    active_collision_resource_index: bpy.props.IntProperty(name="Active Collision Resource", min=0, update=_collision_update)
    diagnostic_items: bpy.props.CollectionProperty(type=DiagnosticItemProperties)
    active_diagnostic_index: bpy.props.IntProperty(name="Active Diagnostic", min=0)


CLASSES = (
    CurvePointProperties,
    EndpointProperties,
    BoneOverrideProperties,
    ChainProperties,
    StructuralGroupProperties,
    CollisionResourceProperties,
    DiagnosticItemProperties,
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
