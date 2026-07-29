"""Compile collision authoring data into contiguous native batches."""

from __future__ import annotations

from array import array
from collections.abc import Iterable

from .model import AnalyticColliderSpec, AnalyticShape, BoneSpec

ANALYTIC_COLLIDER_COLUMNS = 17


def compile_collision_filters(bones: Iterable[BoneSpec]) -> dict[str, object]:
    ordered = tuple(sorted(bones, key=lambda bone: bone.id))
    body_offsets = array("I", [0])
    body_ids = array("I")
    constraint_offsets = array("I", [0])
    constraint_ids = array("I")
    for bone in ordered:
        body_ids.extend(sorted(bone.excluded_body_ids))
        body_offsets.append(len(body_ids))
        constraint_ids.extend(sorted(bone.excluded_constraint_ids))
        constraint_offsets.append(len(constraint_ids))
    return {
        "layers": array("I", (bone.collision_layer for bone in ordered)),
        "masks": array("I", (bone.collision_mask for bone in ordered)),
        "excluded_body_offsets": body_offsets,
        "excluded_body_ids": body_ids,
        "excluded_constraint_offsets": constraint_offsets,
        "excluded_constraint_ids": constraint_ids,
    }


def compile_analytic_colliders(
    colliders: Iterable[AnalyticColliderSpec],
) -> dict[str, object]:
    ordered = tuple(sorted(colliders, key=lambda collider: collider.id))
    values = array("f")
    for collider in ordered:
        row = [0.0] * ANALYTIC_COLLIDER_COLUMNS
        row[0:3] = collider.translation
        row[3:7] = collider.rotation
        row[7:10] = collider.linear_velocity
        if collider.shape is AnalyticShape.SPHERE:
            row[10] = collider.radius
        elif collider.shape is AnalyticShape.CAPSULE:
            row[10:13] = collider.local_a
            row[13:16] = collider.local_b
            row[16] = collider.radius
        elif collider.shape is AnalyticShape.ROUNDED_BOX:
            row[10:13] = collider.half_extents
            row[13] = collider.radius
        else:
            raise AssertionError("AnalyticShape exhaustively covers native shape kinds")
        values.extend(row)
    return {
        "ids": array("I", (collider.id for collider in ordered)),
        "kinds": array("I", (int(collider.shape) for collider in ordered)),
        "enabled": array("I", (int(collider.enabled) for collider in ordered)),
        "values": values,
        "columns": ANALYTIC_COLLIDER_COLUMNS,
    }
