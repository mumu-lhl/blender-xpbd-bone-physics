"""Explicit main-thread controls for armature runtime worlds."""

from __future__ import annotations

import bpy

from ..runtime_registry import REGISTRY, mark_dirty, reset_configured_bones
from ..translations import report_exception


def _armature(context):
    armature = context.object
    return armature if armature is not None and armature.type == "ARMATURE" else None


class BONE_PHYSICS_OT_runtime_enable(bpy.types.Operator):
    bl_idname = "bone_physics.runtime_enable"
    bl_label = "Enable Physics"

    enabled: bpy.props.BoolProperty(name="Enabled", default=True)

    def execute(self, context):
        armature = _armature(context)
        if armature is None:
            return {"CANCELLED"}
        armature.bone_physics.global_enabled = self.enabled
        if not self.enabled:
            try:
                REGISTRY.restore_configured_bones(context)
                REGISTRY.remove(context.scene, armature)
            except Exception as error:
                report_exception(self, {"WARNING"}, "Physics operation failed: {error}", error)
        else:
            try:
                REGISTRY.ensure(context.scene, armature)
            except Exception as error:
                report_exception(self, {"ERROR"}, "Physics operation failed: {error}", error)
                return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_runtime_pause(bpy.types.Operator):
    bl_idname = "bone_physics.runtime_pause"
    bl_label = "Pause or Resume Physics"

    paused: bpy.props.BoolProperty(name="Paused", default=True)

    def execute(self, context):
        armature = _armature(context)
        if armature is None:
            return {"CANCELLED"}
        try:
            world = REGISTRY.world(context.scene, armature)
        except RuntimeError:
            return {"CANCELLED"}
        if self.paused:
            world.paused = True
        else:
            world.resume()
        return {"FINISHED"}


class BONE_PHYSICS_OT_runtime_isolate(bpy.types.Operator):
    bl_idname = "bone_physics.runtime_isolate"
    bl_label = "Isolate Armature Physics"

    def execute(self, context):
        armature = _armature(context)
        if armature is None:
            return {"CANCELLED"}
        for entry in tuple(REGISTRY._worlds.values()):
            entry.world.paused = entry.armature != armature
        return {"FINISHED"}


class BONE_PHYSICS_OT_runtime_validate_rebuild(bpy.types.Operator):
    bl_idname = "bone_physics.runtime_validate_rebuild"
    bl_label = "Validate and Rebuild"

    def execute(self, context):
        armature = _armature(context)
        if armature is None:
            return {"CANCELLED"}
        try:
            REGISTRY.mark_validated(armature.as_pointer())
            REGISTRY.ensure(context.scene, armature)
            mark_dirty(armature.as_pointer(), "world")
            REGISTRY.flush_dirty()
        except Exception as error:
            report_exception(self, {"ERROR"}, "Physics operation failed: {error}", error)
            return {"CANCELLED"}
        return {"FINISHED"}


class BONE_PHYSICS_OT_runtime_reset(bpy.types.Operator):
    bl_idname = "bone_physics.runtime_reset"
    bl_label = "Reset Physics Bones"

    def execute(self, context):
        try:
            reset_configured_bones(context)
        except RuntimeError as error:
            report_exception(self, {"WARNING"}, "Physics operation failed: {error}", error)
            return {"CANCELLED"}
        return {"FINISHED"}


CLASSES = (
    BONE_PHYSICS_OT_runtime_enable,
    BONE_PHYSICS_OT_runtime_pause,
    BONE_PHYSICS_OT_runtime_isolate,
    BONE_PHYSICS_OT_runtime_validate_rebuild,
    BONE_PHYSICS_OT_runtime_reset,
)
