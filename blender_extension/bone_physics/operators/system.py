"""Context-safe physics-system and selected-chain operators."""

import uuid

import bpy

from ..properties import CURVE_FIELD_ITEMS, ENDPOINT_FIELD_ITEMS

from ..chain_discovery import (
    BoneNode,
    BranchSelectionRequired,
    descendant_leaves,
    resolve_chain,
    validate_new_paths,
)


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


_MAX_RNA_STABLE_ID = 0x7FFFFFFF


def _allocate_id(settings: bpy.types.PropertyGroup) -> int:
    """Allocate a free positive RNA integer without reusing an occupied ID."""
    used = {
        item.stable_id
        for system in settings.physics_systems
        for item in (system, *system.chains)
        if item.stable_id
    }
    candidate = settings.next_stable_id
    while candidate in used:
        candidate = 1 if candidate == _MAX_RNA_STABLE_ID else candidate + 1
        if candidate == settings.next_stable_id:
            raise ValueError("stable ID space is exhausted")
    settings.next_stable_id = 1 if candidate == _MAX_RNA_STABLE_ID else candidate + 1
    return candidate


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
        "joint_damping", "angular_compliance", "angular_damping_constraint",
        "recovery_compliance", "recovery_damping", "angular_limit_mode",
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

    def _tail_items(self, context: bpy.types.Context):
        options = getattr(self, "_tail_options", ())
        if not options:
            armature = _active_armature(context)
            if armature is not None:
                selected = tuple(sorted(bone.name for bone in context.selected_pose_bones))
                nodes = _nodes(armature)
                branches = tuple(
                    (head, descendant_leaves(nodes, head))
                    for head in selected
                    if len(descendant_leaves(nodes, head)) > 1
                )
                if len(branches) == 1:
                    options = branches[0][1]
        return tuple((tail, tail, "Descendant leaf") for tail in options)

    tail_uuid: bpy.props.EnumProperty(name="Chain Tail", items=_tail_items)

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and armature.mode == "POSE" and bool(context.selected_pose_bones)

    def invoke(self, context: bpy.types.Context, _event: bpy.types.Event) -> set[str]:
        armature = _active_armature(context)
        self._armature_pointer = armature.as_pointer()
        self._selected_heads = tuple(sorted(bone.name for bone in context.selected_pose_bones))
        nodes = _nodes(armature)
        branches = tuple(
            (head, descendant_leaves(nodes, head))
            for head in self._selected_heads
            if len(descendant_leaves(nodes, head)) > 1
        )
        if not branches:
            return self.execute(context)
        if len(branches) != 1:
            self.report({"ERROR"}, "Select one branched chain head at a time")
            return {"CANCELLED"}
        self._branch_head, self._tail_options = branches[0]
        self.tail_uuid = self._tail_options[0]
        context.window_manager.invoke_search_popup(self)
        return {"RUNNING_MODAL"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        armature = _active_armature(context)
        if armature is None:
            return {"CANCELLED"}
        captured_pointer = getattr(self, "_armature_pointer", armature.as_pointer())
        if armature.as_pointer() != captured_pointer:
            self.report({"ERROR"}, "Active armature changed while choosing a chain tail")
            return {"CANCELLED"}
        settings = armature.bone_physics
        if not settings.physics_systems:
            self.report({"ERROR"}, "Add a physics system before adding chains")
            return {"CANCELLED"}
        nodes = _nodes(armature)
        selected = getattr(
            self, "_selected_heads", tuple(sorted(bone.name for bone in context.selected_pose_bones))
        )
        try:
            branch_head = getattr(self, "_branch_head", None)
            if branch_head is None:
                branches = tuple(
                    head for head in selected if len(descendant_leaves(nodes, head)) > 1
                )
                if len(branches) == 1 and self.tail_uuid:
                    branch_head = branches[0]
            new_paths = tuple(
                resolve_chain(nodes, head, self.tail_uuid if head == branch_head else None)
                for head in selected
            )
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


def _selected_systems(settings: bpy.types.PropertyGroup) -> tuple[bpy.types.PropertyGroup, ...]:
    return tuple(system for system in settings.physics_systems if system.selected)


class BONE_PHYSICS_OT_apply_endpoint_value(bpy.types.Operator):
    bl_idname = "bone_physics.apply_endpoint_value"
    bl_label = "Apply Endpoint Value"
    bl_options = {"REGISTER", "UNDO"}

    endpoint: bpy.props.EnumProperty(
        name="Endpoint", items=(("HEAD", "Head", "Chain head"), ("TAIL", "Tail", "Chain tail"))
    )
    field: bpy.props.EnumProperty(name="Field", items=ENDPOINT_FIELD_ITEMS)
    value: bpy.props.FloatProperty(name="Value")
    vector_value: bpy.props.FloatVectorProperty(name="Vector Value", size=3)
    enum_value: bpy.props.EnumProperty(
        name="Angular Limit",
        items=(("FREE", "Free", ""), ("XYZ", "Local XYZ", ""), ("SWING_TWIST", "Swing-Twist", "")),
    )

    def draw(self, _context: bpy.types.Context) -> None:
        property_name = self.field.lower()
        self.layout.prop(self, "endpoint")
        self.layout.prop(self, "field")
        self.layout.prop(
            self,
            "enum_value" if property_name == "angular_limit_mode" else
            "vector_value" if property_name in {"inertia_scale", "xyz_lower", "xyz_upper", "swing_axis"} else
            "value",
        )

    def invoke(self, context: bpy.types.Context, _event: bpy.types.Event) -> set[str]:
        return context.window_manager.invoke_props_dialog(self)

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return _active_armature(context) is not None

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        targets = _selected_systems(settings)
        if not targets:
            self.report({"ERROR"}, "Select at least one physics system")
            return {"CANCELLED"}
        property_name = self.field.lower()
        applied_value = (
            self.enum_value
            if property_name == "angular_limit_mode"
            else self.vector_value
            if property_name in {"inertia_scale", "xyz_lower", "xyz_upper", "swing_axis"}
            else self.value
        )
        for system in targets:
            setattr(system.head if self.endpoint == "HEAD" else system.tail, property_name, applied_value)
        return {"FINISHED"}


class BONE_PHYSICS_OT_curve_point_add(bpy.types.Operator):
    bl_idname = "bone_physics.curve_point_add"
    bl_label = "Add Curve Point"
    bl_options = {"REGISTER", "UNDO"}

    field: bpy.props.EnumProperty(name="Field", items=CURVE_FIELD_ITEMS)
    position: bpy.props.FloatProperty(name="Position", min=0.0, max=1.0)
    value: bpy.props.FloatProperty(name="Value")

    def invoke(self, context: bpy.types.Context, _event: bpy.types.Event) -> set[str]:
        return context.window_manager.invoke_props_dialog(self)

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and bool(armature.bone_physics.physics_systems)

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        targets = _selected_systems(settings) or (
            settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)],
        )
        for system in targets:
            positions = sorted(point.s for point in system.curve_points if point.field == self.field)
            if any(abs(position - self.position) <= 1.0e-6 for position in positions):
                self.report({"ERROR"}, "Curve point positions must be strictly increasing")
                return {"CANCELLED"}
        for system in targets:
            point = system.curve_points.add()
            point.field = self.field
            point.s = self.position
            point.value = self.value
            source = len(system.curve_points) - 1
            destination = next(
                (
                    index
                    for index, candidate in enumerate(system.curve_points)
                    if index != source
                    and candidate.field == self.field
                    and candidate.s > self.position
                ),
                source,
            )
            system.curve_points.move(source, destination)
        return {"FINISHED"}


class BONE_PHYSICS_OT_override_add_selected(bpy.types.Operator):
    bl_idname = "bone_physics.override_add_selected"
    bl_label = "Add Selected Bone Override"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and armature.mode == "POSE" and bool(context.selected_pose_bones)

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        if not settings.physics_systems:
            self.report({"ERROR"}, "Add a physics system before adding an override")
            return {"CANCELLED"}
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        existing = {override.bone_name for override in system.overrides}
        for bone in sorted(context.selected_pose_bones, key=lambda item: item.name):
            if bone.name not in existing:
                override = system.overrides.add()
                override.bone_name = bone.name
        return {"FINISHED"}


CLASSES = (
    BONE_PHYSICS_OT_initialize,
    BONE_PHYSICS_OT_system_add,
    BONE_PHYSICS_OT_system_duplicate,
    BONE_PHYSICS_OT_system_remove,
    BONE_PHYSICS_OT_chain_add_selected,
    BONE_PHYSICS_OT_apply_endpoint_value,
    BONE_PHYSICS_OT_curve_point_add,
    BONE_PHYSICS_OT_override_add_selected,
)
