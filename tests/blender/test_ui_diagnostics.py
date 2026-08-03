"""Static diagnostics UI contract; execute this script inside Blender."""

from blender_extension.bone_physics.operators.diagnostics import CLASSES as DIAGNOSTIC_OPERATORS
from blender_extension.bone_physics.ui.lists import BONE_PHYSICS_UL_diagnostics
from blender_extension.bone_physics.ui.panels import BONE_PHYSICS_PT_diagnostics
from blender_extension.bone_physics import viewport


assert {item.bl_idname for item in DIAGNOSTIC_OPERATORS} == {"bone_physics.diagnostic_select"}
assert BONE_PHYSICS_PT_diagnostics.bl_idname == "BONE_PHYSICS_PT_diagnostics"
assert BONE_PHYSICS_UL_diagnostics.bl_idname == "BONE_PHYSICS_UL_diagnostics"
assert set(viewport._COLORS) == {"dynamic", "kinematic", "mesh", "structural", "invalid"}
