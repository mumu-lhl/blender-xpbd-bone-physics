"""Bounded 3D-view diagnostic overlays.

This module owns every draw handler it creates.  Geometry is supplied as plain
line segments so runtime and authoring code never need to retain Blender GPU
objects across redraws or file loads.
"""

from __future__ import annotations

from collections.abc import Iterable

import bpy


_DRAW_HANDLERS: dict[int, object] = {}
_OVERLAY_LINES: dict[int, dict[str, tuple[tuple[float, float, float], ...]]] = {}
_COLORS = {
    "dynamic": (0.15, 0.75, 1.0, 0.85),
    "kinematic": (0.2, 1.0, 0.45, 0.85),
    "mesh": (0.75, 0.75, 0.75, 0.65),
    "structural": (1.0, 0.65, 0.1, 0.85),
    "invalid": (1.0, 0.1, 0.15, 1.0),
}


def set_overlay_lines(area: object, category: str, points: Iterable[Iterable[float]]) -> None:
    """Replace one category of paired line vertices for an area."""
    if category not in _COLORS:
        raise ValueError(f"unknown overlay category {category!r}")
    values = tuple(tuple(float(component) for component in point) for point in points)
    if len(values) % 2 or any(len(point) != 3 for point in values):
        raise ValueError("overlay lines require paired three-dimensional points")
    _OVERLAY_LINES.setdefault(area.as_pointer(), {})[category] = values


def enable_overlay(area: object) -> None:
    """Register at most one post-view handler for a 3D View area."""
    key = area.as_pointer()
    if key not in _DRAW_HANDLERS:
        _DRAW_HANDLERS[key] = bpy.types.SpaceView3D.draw_handler_add(
            _draw_overlay, (key,), "WINDOW", "POST_VIEW"
        )


def disable_overlay(area: object) -> None:
    key = area.as_pointer()
    handler = _DRAW_HANDLERS.pop(key, None)
    if handler is not None:
        bpy.types.SpaceView3D.draw_handler_remove(handler, "WINDOW")
    _OVERLAY_LINES.pop(key, None)


def disable_all_overlays(*_unused: object) -> None:
    """Remove handlers on extension shutdown and after loading a file."""
    for handler in tuple(_DRAW_HANDLERS.values()):
        bpy.types.SpaceView3D.draw_handler_remove(handler, "WINDOW")
    _DRAW_HANDLERS.clear()
    _OVERLAY_LINES.clear()


def register() -> None:
    if disable_all_overlays not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(disable_all_overlays)


def unregister() -> None:
    disable_all_overlays()
    if disable_all_overlays in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(disable_all_overlays)


def _draw_overlay(key: int) -> None:
    """Draw the current immutable snapshot; failures must not leak handlers."""
    try:
        import gpu
        from gpu_extras.batch import batch_for_shader
        shader = gpu.shader.from_builtin("POLYLINE_UNIFORM_COLOR")
        for category, points in _OVERLAY_LINES.get(key, {}).items():
            if not points:
                continue
            batch = batch_for_shader(shader, "LINES", {"pos": points})
            shader.bind()
            shader.uniform_float("color", _COLORS[category])
            batch.draw(shader)
    except Exception:
        # Blender can destroy an area while a redraw is queued.
        return
