"""Blender operators for Bone Physics authoring."""

from .constraint import CLASSES as CONSTRAINT_CLASSES
from .collision import CLASSES as COLLISION_CLASSES
from .system import CLASSES as SYSTEM_CLASSES
from .runtime import CLASSES as RUNTIME_CLASSES

CLASSES = SYSTEM_CLASSES + CONSTRAINT_CLASSES + COLLISION_CLASSES + RUNTIME_CLASSES


def register() -> None:
    for cls in CLASSES:
        if not getattr(cls, "is_registered", False):
            import bpy

            bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(CLASSES):
        if getattr(cls, "is_registered", False):
            import bpy

            bpy.utils.unregister_class(cls)
