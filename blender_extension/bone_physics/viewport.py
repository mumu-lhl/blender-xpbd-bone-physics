"""Bounded 3D-view diagnostic overlays.

This module owns every draw handler it creates.  Geometry is supplied as plain
line segments so runtime and authoring code never need to retain Blender GPU
objects across redraws or file loads.
"""

from __future__ import annotations

from collections.abc import Iterable
import math
from typing import Mapping

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
    try:
        key = area.as_pointer()
        _OVERLAY_LINES.setdefault(key, {})[category] = values
    except (ReferenceError, AttributeError):
        return


def enable_overlay(area: object) -> None:
    """Register at most one post-view handler for a 3D View area."""
    try:
        key = area.as_pointer()
    except (ReferenceError, AttributeError):
        return
    if key not in _DRAW_HANDLERS:
        _DRAW_HANDLERS[key] = bpy.types.SpaceView3D.draw_handler_add(
            _draw_overlay, (key,), "WINDOW", "POST_VIEW"
        )


def disable_overlay(area: object) -> None:
    try:
        key = area.as_pointer()
    except (ReferenceError, AttributeError):
        return
    handler = _DRAW_HANDLERS.pop(key, None)
    if handler is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(handler, "WINDOW")
        except (ReferenceError, RuntimeError):
            pass
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


def overlay_lines_for_world(spec: object, poses: Mapping[int, object] | None = None, mesh_batches: Iterable[object] = ()) -> dict[str, tuple[tuple[float, float, float], ...]]:
    """Build bounded line snapshots from immutable world data and poses."""
    categories = {name: [] for name in _COLORS}
    bones = tuple(getattr(spec, "bones", ()))
    poses = poses or {}
    for bone in bones:
        try:
            start = _body_point(bone, getattr(bone, "local_head"), poses.get(bone.id))
            end = _body_point(bone, getattr(bone, "local_tail"), poses.get(bone.id))
            category = "dynamic" if float(getattr(bone, "inverse_mass", 0.0)) > 0.0 else "kinematic"
            if not _valid_point(start) or not _valid_point(end):
                categories["invalid"].extend(((0.0, 0.0, 0.0), (0.05, 0.0, 0.0)))
            else:
                categories[category].extend((start, end))
        except (AttributeError, TypeError, ValueError):
            categories["invalid"].extend(((0.0, 0.0, 0.0), (0.05, 0.0, 0.0)))
    # Structural links are represented by rest-space chain samples.  Keeping
    # these as line segments makes them safe to redraw even while RNA changes.
    try:
        from .parameter_builder import sample_chain_anchor
        by_id = {bone.id: bone for bone in bones}
        chains = {chain.id: chain for chain in getattr(spec, "chains", ())}
        for lattice in getattr(spec, "cross_links", ()):
            pairs = list(zip(lattice.chain_ids, lattice.chain_ids[1:]))
            if lattice.closed:
                pairs.append((lattice.chain_ids[-1], lattice.chain_ids[0]))
            for left, right in pairs:
                for sample in lattice.samples:
                    first = sample_chain_anchor(chains[left], sample, by_id).world_anchor
                    second = sample_chain_anchor(chains[right], sample, by_id).world_anchor
                    categories["structural"].extend((first, second))
    except (AttributeError, KeyError, TypeError, ValueError):
        categories["invalid"].extend(((0.0, 0.0, 0.0), (0.05, 0.05, 0.0)))
    for batch in mesh_batches:
        try:
            vertices = tuple(batch.current_vertices)
            for triangle in batch.topology.triangles:
                a, b, c = (vertices[index] for index in triangle)
                categories["mesh"].extend((a, b, b, c, c, a))
        except (AttributeError, IndexError, TypeError, ValueError):
            categories["invalid"].extend(((0.0, 0.0, 0.0), (0.05, 0.0, 0.05)))
    return {name: tuple(points) for name, points in categories.items()}


def refresh_world_overlay(armature: object, spec: object, poses: Mapping[int, object] | None = None, mesh_batches: Iterable[object] = ()) -> None:
    """Publish a new snapshot to every live 3D view for an armature."""
    try:
        import bpy as _bpy
        settings = getattr(armature, "bone_physics", None)
        if settings is not None and not getattr(settings, "show_viewport_overlays", False):
            return
        lines = overlay_lines_for_world(spec, poses, mesh_batches)
        matrix = getattr(armature, "matrix_world", None)
        for screen in tuple(_bpy.data.screens):
            for area in tuple(screen.areas):
                try:
                    if area.type != "VIEW_3D":
                        continue
                    enable_overlay(area)
                    for category, points in lines.items():
                        set_overlay_lines(area, category, (_transform_point(matrix, point) for point in points))
                except (ReferenceError, AttributeError, RuntimeError):
                    continue
    except (ImportError, ReferenceError, AttributeError, RuntimeError):
        return


def _transform_point(matrix: object, point: tuple[float, float, float]) -> tuple[float, float, float]:
    if matrix is None:
        return point
    try:
        result = matrix @ point
        values = tuple(float(value) for value in result)
        if len(values) >= 3:
            return values[:3]  # type: ignore[return-value]
    except (TypeError, AttributeError, ValueError):
        pass
    try:
        return tuple(
            sum(float(matrix[row][column]) * (point[column] if column < 3 else 1.0) for column in range(4))
            for row in range(3)
        )  # type: ignore[return-value]
    except (TypeError, AttributeError, IndexError, ValueError):
        return point


def _body_point(bone: object, local: Iterable[float], pose: object | None) -> tuple[float, float, float]:
    if pose is None:
        origin = tuple(float(value) for value in bone.rest_translation)
        rotation = tuple(float(value) for value in bone.rest_rotation)
    else:
        origin = tuple(float(value) for value in pose.translation)
        rotation = tuple(float(value) for value in pose.rotation)
    return _add(origin, _rotate(rotation, tuple(float(value) for value in local)))


def _add(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _rotate(quaternion: tuple[float, float, float, float], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # q * v * q^-1, expanded to avoid Blender mathutils dependencies.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _valid_point(point: tuple[float, float, float]) -> bool:
    return len(point) == 3 and all(math.isfinite(value) for value in point)


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
