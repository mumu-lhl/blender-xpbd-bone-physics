"""Guarded collision-resource authoring operators."""

from __future__ import annotations

import uuid

import bpy

from .system import _active_armature, _allocate_id


def _new_id() -> str:
    return str(uuid.uuid4())


def _active_resource(settings: bpy.types.PropertyGroup):
    if not settings.collision_resources:
        return None
    return settings.collision_resources[min(
        settings.active_collision_resource_index, len(settings.collision_resources) - 1
    )]


class BONE_PHYSICS_OT_collision_resource_add(bpy.types.Operator):
    bl_idname = "bone_physics.collision_resource_add"
    bl_label = "Add Collision Resource"
    bl_options = {"REGISTER", "UNDO"}

    kind: bpy.props.EnumProperty(
        name="Kind", items=(("ANALYTIC", "Analytic", ""), ("MESH", "Mesh", ""), ("COLLECTION", "Collection", "")), default="ANALYTIC"
    )

    @classmethod
    def poll(cls, context):
        return _active_armature(context) is not None

    def execute(self, context):
        settings = _active_armature(context).bone_physics
        resource = settings.collision_resources.add()
        resource.id = _new_id()
        resource.stable_id = _allocate_id(settings)
        resource.kind = self.kind
        resource.name = {"ANALYTIC": "Collider", "MESH": "Collision Mesh", "COLLECTION": "Collision Collection"}[self.kind]
        settings.active_collision_resource_index = len(settings.collision_resources) - 1
        return {"FINISHED"}


class BONE_PHYSICS_OT_collision_resource_duplicate(bpy.types.Operator):
    bl_idname = "bone_physics.collision_resource_duplicate"
    bl_label = "Duplicate Collision Resource"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        armature = _active_armature(context)
        return armature is not None and bool(armature.bone_physics.collision_resources)

    def execute(self, context):
        settings = _active_armature(context).bone_physics
        source = _active_resource(settings)
        copy = settings.collision_resources.add()
        for name in source.bl_rna.properties.keys():
            if name not in {"rna_type", "id", "stable_id"}:
                setattr(copy, name, getattr(source, name))
        copy.id, copy.stable_id = _new_id(), _allocate_id(settings)
        copy.name = f"{source.name} Copy"
        settings.active_collision_resource_index = len(settings.collision_resources) - 1
        return {"FINISHED"}


class BONE_PHYSICS_OT_collision_resource_remove(bpy.types.Operator):
    bl_idname = "bone_physics.collision_resource_remove"
    bl_label = "Remove Collision Resource"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return BONE_PHYSICS_OT_collision_resource_duplicate.poll(context)

    def execute(self, context):
        settings = _active_armature(context).bone_physics
        resource = _active_resource(settings)
        references = [system.name or system.id for system in settings.physics_systems if resource.id in system.collision_resource_ids.split(",")]
        references += [item.name or item.id for item in settings.collision_resources if item.kind == "COLLECTION" and resource.id in item.member_ids.split(",")]
        if references:
            self.report({"ERROR"}, "Resource is referenced by: " + ", ".join(sorted(references)))
            return {"CANCELLED"}
        settings.collision_resources.remove(settings.active_collision_resource_index)
        settings.active_collision_resource_index = min(settings.active_collision_resource_index, max(0, len(settings.collision_resources) - 1))
        return {"FINISHED"}


class BONE_PHYSICS_OT_collision_resource_assign(bpy.types.Operator):
    bl_idname = "bone_physics.collision_resource_assign"
    bl_label = "Assign Collision Resource"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        armature = _active_armature(context)
        return armature is not None and bool(armature.bone_physics.physics_systems) and bool(armature.bone_physics.collision_resources)

    def execute(self, context):
        settings = _active_armature(context).bone_physics
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        resource = _active_resource(settings)
        ids = {value.strip() for value in system.collision_resource_ids.split(",") if value.strip()}
        ids.add(resource.id)
        system.collision_resource_ids = ",".join(sorted(ids))
        return {"FINISHED"}


class BONE_PHYSICS_OT_collision_resource_bind(bpy.types.Operator):
    bl_idname = "bone_physics.collision_resource_bind"
    bl_label = "Bind Collision Resource"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        armature = _active_armature(context)
        if armature is None or _active_resource(armature.bone_physics) is None:
            return {"CANCELLED"}
        resource, target = _active_resource(armature.bone_physics), context.active_object
        resource.binding = "BONE" if target == armature and context.active_pose_bone else "OBJECT"
        resource.object_name = target.name if target else ""
        resource.bone_name = context.active_pose_bone.name if resource.binding == "BONE" else ""
        return {"FINISHED"}


class BONE_PHYSICS_OT_collision_resource_toggle(bpy.types.Operator):
    bl_idname = "bone_physics.collision_resource_toggle"
    bl_label = "Enable Collision Resource"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        armature = _active_armature(context)
        resource = _active_resource(armature.bone_physics) if armature else None
        if resource is None:
            return {"CANCELLED"}
        resource.enabled = not resource.enabled
        return {"FINISHED"}


class BONE_PHYSICS_OT_collision_resource_mirror(bpy.types.Operator):
    bl_idname = "bone_physics.collision_resource_mirror"
    bl_label = "Mirror Collision Resource"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        armature = _active_armature(context)
        if armature is None or _active_resource(armature.bone_physics) is None:
            return {"CANCELLED"}
        settings, source = armature.bone_physics, _active_resource(armature.bone_physics)
        copy = settings.collision_resources.add()
        for name in source.bl_rna.properties.keys():
            if name not in {"rna_type", "id", "stable_id"}:
                setattr(copy, name, getattr(source, name))
        copy.id, copy.stable_id = _new_id(), _allocate_id(settings)
        copy.name = f"{source.name} Mirrored"
        settings.active_collision_resource_index = len(settings.collision_resources) - 1
        return {"FINISHED"}


CLASSES = (BONE_PHYSICS_OT_collision_resource_add, BONE_PHYSICS_OT_collision_resource_duplicate, BONE_PHYSICS_OT_collision_resource_remove, BONE_PHYSICS_OT_collision_resource_assign, BONE_PHYSICS_OT_collision_resource_bind, BONE_PHYSICS_OT_collision_resource_toggle, BONE_PHYSICS_OT_collision_resource_mirror)
