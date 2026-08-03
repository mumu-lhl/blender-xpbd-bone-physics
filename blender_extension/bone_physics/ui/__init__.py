"""N-panel UI registration for Bone Physics."""

from .lists import CLASSES as LIST_CLASSES
from .panels import CLASSES as PANEL_CLASSES

CLASSES = LIST_CLASSES + PANEL_CLASSES


def register() -> None:
    import bpy

    for cls in CLASSES:
        if not getattr(cls, "is_registered", False):
            bpy.utils.register_class(cls)


def unregister() -> None:
    import bpy

    for cls in reversed(CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
