"""Blender operators for Bone Physics authoring."""

from .system import CLASSES


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
