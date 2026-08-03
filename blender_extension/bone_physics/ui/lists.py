"""Compact lists for high-level physics systems."""

from __future__ import annotations

import bpy


class BONE_PHYSICS_UL_systems(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.prop(item, "enabled", text="")
        layout.label(text=item.name or "Unnamed System")
        layout.label(text=f"{len(item.chains)} chains")


class BONE_PHYSICS_UL_overrides(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.label(text=item.bone_name)


class BONE_PHYSICS_UL_curve_points(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        row = layout.row(align=True)
        row.prop(item, "field", text="")
        row.prop(item, "s", text="Position")
        row.prop(item, "value", text="Value")
        row.prop(item, "tangent", text="Tangent")


CLASSES = (BONE_PHYSICS_UL_systems, BONE_PHYSICS_UL_overrides, BONE_PHYSICS_UL_curve_points)
