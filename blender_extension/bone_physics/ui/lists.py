"""Compact lists for high-level physics systems."""

from __future__ import annotations

import bpy

from ..chain_discovery import chain_display_name
from ..translations import format_issue


class BONE_PHYSICS_UL_systems(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.prop(item, "enabled", text="")
        if item.name:
            layout.label(text=item.name, translate=False)
        else:
            layout.label(text="Unnamed System")
        layout.label(text=f"{len(item.chains)} chains")
        chain_names = ", ".join(
            chain_display_name(chain.head_uuid, chain.tail_uuid) for chain in item.chains
        )
        if chain_names:
            layout.label(text=chain_names, translate=False)


class BONE_PHYSICS_UL_overrides(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.label(text=item.bone_name, translate=False)


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
        row = layout.row(align=True)
        row.label(text=topology)
        row.label(text=f": {item.chain_ids}", translate=False)


class BONE_PHYSICS_UL_collision_resources(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.prop(item, "enabled", text="")
        if item.name:
            layout.label(text=item.name, translate=False)
        else:
            layout.label(text="Unnamed Collision Resource")
        layout.label(text=item.kind.title())


class BONE_PHYSICS_UL_diagnostics(bpy.types.UIList):
    """Diagnostic entries expose severity and a navigation affordance."""

    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_property, _index):
        layout.label(text=format_issue(item), translate=False, icon={"error": "ERROR", "warning": "ERROR"}.get(item.severity, "INFO"))
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
