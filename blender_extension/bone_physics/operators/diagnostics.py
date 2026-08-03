"""Safe navigation from a diagnostic item to its Blender target."""

from __future__ import annotations

import bpy


class BONE_PHYSICS_OT_diagnostic_select(bpy.types.Operator):
    bl_idname = "bone_physics.diagnostic_select"
    bl_label = "Select Diagnostic Target"

    entity_kind: bpy.props.StringProperty(name="Entity Kind")
    entity_id: bpy.props.StringProperty(name="Entity ID")
    object_name: bpy.props.StringProperty(name="Object")

    def execute(self, context):
        target = bpy.data.objects.get(self.object_name) if self.object_name else context.object
        if target is None:
            self.report({"WARNING"}, "Diagnostic target no longer exists")
            return {"CANCELLED"}
        if target.hide_get() or target.hide_viewport or not target.visible_get():
            self.report({"WARNING"}, "Diagnostic target is hidden; it was not made visible")
            return {"CANCELLED"}
        if not target.select_get() and target.hide_select:
            self.report({"WARNING"}, "Diagnostic target is not selectable")
            return {"CANCELLED"}
        if self.entity_kind == "bone":
            if target.type != "ARMATURE":
                self.report({"WARNING"}, "Diagnostic bone requires an armature target")
                return {"CANCELLED"}
            if context.view_layer.objects.active != target:
                context.view_layer.objects.active = target
            target.select_set(True)
            if target.mode != "POSE" and bpy.ops.object.mode_set.poll():
                bpy.ops.object.mode_set(mode="POSE")
            bone = target.data.bones.get(self.entity_id)
            if bone is None:
                self.report({"WARNING"}, "Diagnostic bone no longer exists")
                return {"CANCELLED"}
            bone.select = True
            target.data.bones.active = bone
        else:
            target.select_set(True)
            context.view_layer.objects.active = target
        return {"FINISHED"}


CLASSES = (BONE_PHYSICS_OT_diagnostic_select,)
