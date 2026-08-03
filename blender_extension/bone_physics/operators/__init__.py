"""Blender operators for Bone Physics authoring."""

from .constraint import CLASSES as CONSTRAINT_CLASSES
from .system import CLASSES as SYSTEM_CLASSES

CLASSES = SYSTEM_CLASSES + CONSTRAINT_CLASSES


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
