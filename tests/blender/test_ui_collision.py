"""Static Blender UI registration smoke; executed through bounded Blender MCP."""

from blender_extension.bone_physics.operators.collision import CLASSES as COLLISION_OPERATORS
from blender_extension.bone_physics.ui.panels import BONE_PHYSICS_PT_collision_resources
from blender_extension.bone_physics.ui.lists import BONE_PHYSICS_UL_collision_resources

assert len(COLLISION_OPERATORS) >= 4
assert BONE_PHYSICS_PT_collision_resources.bl_idname == "BONE_PHYSICS_PT_collision_resources"
assert BONE_PHYSICS_UL_collision_resources.bl_idname == "BONE_PHYSICS_UL_collision_resources"
