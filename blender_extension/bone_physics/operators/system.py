"""Context-safe physics-system and selected-chain operators."""

from __future__ import annotations

import uuid

import bpy

from ..chain_discovery import BoneNode, BranchSelectionRequired, resolve_chain, validate_new_paths


SYSTEM_TEMPLATES = (
    ("LOOSE_HAIR", "Loose Hair", "Starting values for loose hair."),
    ("EARS", "Ears", "Starting values for ears."),
    ("PONYTAIL", "Ponytail", "Starting values for a ponytail or tail."),
    ("SKIRT", "Skirt", "Starting values for a skirt."),
    ("ACCESSORY", "Accessory", "Starting values for a small accessory."),
    ("BLANK", "Blank", "An empty advanced system."),
)


def _active_armature(context: bpy.types.Context) -> bpy.types.Object | None:
    obj = context.object
    return obj if obj is not None and obj.type == "ARMATURE" else None


def _new_id() -> str:
    return str(uuid.uuid4())


def _allocate_id(settings: bpy.types.PropertyGroup) -> int:
    stable_id = settings.next_stable_id
    settings.next_stable_id = 1 if stable_id == 0xFFFFFFFF else stable_id + 1
    return stable_id


def _system_name(template: str) -> str:
    return dict((identifier, label) for identifier, label, _description in SYSTEM_TEMPLATES)[template]


def populate_system_template(
    system: bpy.types.PropertyGroup, template: str, name: str
) -> None:
    """Fill editable defaults only; templates never create object or bone bindings."""
    system.id = _new_id()
    system.name = name.strip() or _system_name(template)
    system.enabled = True
    # These restrained defaults distinguish common templates without prescribing a rig.
    if template == "EARS":
        system.head.gravity_scale = system.tail.gravity_scale = 0.25
        system.head.recovery_compliance = system.tail.recovery_compliance = 0.002
    elif template in {"PONYTAIL", "SKIRT"}:
        system.head.recovery_compliance = system.tail.recovery_compliance = 0.001
    elif template == "ACCESSORY":
        system.head.recovery_compliance = system.tail.recovery_compliance = 0.0005
        system.head.restitution = system.tail.restitution = 0.0


def _copy_endpoint(source: bpy.types.PropertyGroup, destination: bpy.types.PropertyGroup) -> None:
    for property_name in (
        "inverse_mass", "inertia_scale", "gravity_scale", "translation_inertia",
        "rotation_inertia", "linear_damping", "angular_damping", "joint_compliance",
        "joint_damping", "recovery_compliance", "recovery_damping", "angular_limit_mode",
        "xyz_lower", "xyz_upper", "swing_axis", "max_swing", "min_twist", "max_twist",
        "radius", "margin", "capsule_head_inset", "capsule_tail_inset", "friction",
        "restitution", "interpolation",
    ):
        setattr(destination, property_name, getattr(source, property_name))


def _copy_system(settings: bpy.types.PropertyGroup, source: bpy.types.PropertyGroup) -> None:
    destination = settings.physics_systems.add()
    destination.id = _new_id()
    destination.stable_id = _allocate_id(settings)
    destination.name = source.name
    destination.enabled = source.enabled
    _copy_endpoint(source.head, destination.head)
    _copy_endpoint(source.tail, destination.tail)
    for source_chain in source.chains:
        chain = destination.chains.add()
        chain.id = _new_id()
        chain.stable_id = _allocate_id(settings)
        chain.head_uuid = source_chain.head_uuid
        chain.tail_uuid = source_chain.tail_uuid
        chain.target_id = source_chain.target_id
        chain.closed = source_chain.closed
    return destination


def _nodes(armature: bpy.types.Object) -> tuple[BoneNode, ...]:
    return tuple(
        BoneNode(
            uuid=bone.name,
            name=bone.name,
            parent_uuid=bone.parent.name if bone.parent else None,
            length=bone.length,
        )
        for bone in armature.pose.bones
    )


class BONE_PHYSICS_OT_initialize(bpy.types.Operator):
    bl_idname = "bone_physics.initialize"
    bl_label = "Initialize Physics Configuration"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return _active_armature(context) is not None

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        if not settings.id:
            settings.id = _new_id()
        settings.workspace = "SETUP"
        return {"FINISHED"}


class BONE_PHYSICS_OT_system_add(bpy.types.Operator):
    bl_idname = "bone_physics.system_add"
    bl_label = "Add Physics System"
    bl_options = {"REGISTER", "UNDO"}

    template: bpy.props.EnumProperty(name="Template", items=SYSTEM_TEMPLATES, default="LOOSE_HAIR")
    name: bpy.props.StringProperty(name="Name")

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return _active_armature(context) is not None

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        system = settings.physics_systems.add()
        system.stable_id = _allocate_id(settings)
        populate_system_template(system, self.template, self.name)
        settings.active_system_index = len(settings.physics_systems) - 1
        return {"FINISHED"}


class BONE_PHYSICS_OT_system_duplicate(bpy.types.Operator):
    bl_idname = "bone_physics.system_duplicate"
    bl_label = "Duplicate Physics System"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and bool(armature.bone_physics.physics_systems)

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        index = min(settings.active_system_index, len(settings.physics_systems) - 1)
        duplicate = _copy_system(settings, settings.physics_systems[index])
        settings.active_system_index = len(settings.physics_systems) - 1
        duplicate.name = f"{duplicate.name} Copy"
        return {"FINISHED"}


class BONE_PHYSICS_OT_system_remove(bpy.types.Operator):
    bl_idname = "bone_physics.system_remove"
    bl_label = "Remove Physics System"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and bool(armature.bone_physics.physics_systems)

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        settings.physics_systems.remove(min(settings.active_system_index, len(settings.physics_systems) - 1))
        settings.active_system_index = min(settings.active_system_index, max(0, len(settings.physics_systems) - 1))
        return {"FINISHED"}


class BONE_PHYSICS_OT_chain_add_selected(bpy.types.Operator):
    bl_idname = "bone_physics.chain_add_selected"
    bl_label = "Add Chains from Selected Bones"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and armature.mode == "POSE" and bool(context.selected_pose_bones)

    def execute(self, context: bpy.types.Context) -> set[str]:
        armature = _active_armature(context)
        settings = armature.bone_physics
        if not settings.physics_systems:
            self.report({"ERROR"}, "Add a physics system before adding chains")
            return {"CANCELLED"}
        nodes = _nodes(armature)
        selected = tuple(sorted((bone.name for bone in context.selected_pose_bones)))
        try:
            new_paths = tuple(resolve_chain(nodes, head) for head in selected)
            existing_paths = tuple(
                resolve_chain(nodes, chain.head_uuid, chain.tail_uuid)
                for system in settings.physics_systems
                for chain in system.chains
            )
            validate_new_paths(existing_paths, new_paths)
        except BranchSelectionRequired as error:
            self.report({"ERROR"}, f"Select a tail for branched chain {error.head_uuid}")
            return {"CANCELLED"}
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        for path in new_paths:
            chain = system.chains.add()
            chain.id = _new_id()
            chain.stable_id = _allocate_id(settings)
            chain.head_uuid = path[0]
            chain.tail_uuid = path[-1]
        return {"FINISHED"}


CLASSES = (
    BONE_PHYSICS_OT_initialize,
    BONE_PHYSICS_OT_system_add,
    BONE_PHYSICS_OT_system_duplicate,
    BONE_PHYSICS_OT_system_remove,
    BONE_PHYSICS_OT_chain_add_selected,
)
