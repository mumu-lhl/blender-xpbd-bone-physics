"""Compact lists for high-level physics systems."""

from __future__ import annotations

import bpy

from ..translations import format_issue


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


class BONE_PHYSICS_UL_structural_groups(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        topology = "Seam" if item.closed else "Open"
        if item.diagonals:
            topology += " + diagonals"
        layout.label(text=f"{topology}: {item.chain_ids}")


class BONE_PHYSICS_UL_collision_resources(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.prop(item, "enabled", text="")
        layout.label(text=item.name or "Unnamed Collision Resource")
        layout.label(text=item.kind.title())


class BONE_PHYSICS_UL_diagnostics(bpy.types.UIList):
    """Diagnostic entries expose severity and a navigation affordance."""

    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.label(text=format_issue(item), icon={"error": "ERROR", "warning": "ERROR"}.get(item.severity, "INFO"))
        operator = layout.operator("bone_physics.diagnostic_select", text="", icon="RESTRICT_SELECT_OFF")
        operator.entity_kind = item.entity_kind
        operator.entity_id = item.entity_id
        operator.object_name = getattr(item, "object_name", "")


CLASSES = (
    BONE_PHYSICS_UL_systems,
    BONE_PHYSICS_UL_overrides,
    BONE_PHYSICS_UL_curve_points,
    BONE_PHYSICS_UL_structural_groups,
    BONE_PHYSICS_UL_collision_resources,
    BONE_PHYSICS_UL_diagnostics,
)
