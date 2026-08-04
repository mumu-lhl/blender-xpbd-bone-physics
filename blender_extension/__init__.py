"""Source-tree entry point for Blender's extension-development loader.

The packaged archive flattens ``bone_physics`` into its extension root.  When
Blender loads this repository directory directly, it instead imports this
wrapper and expects the standard add-on lifecycle functions here.
"""

from .bone_physics import register, unregister

__all__ = ["register", "unregister"]
