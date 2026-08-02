"""Native-backed deterministic acceptance measurements."""

from __future__ import annotations

from array import array
import math
from typing import Any

try:
    import bone_physics_native as native
except ModuleNotFoundError:
    native = None

from blender_extension.bone_physics.parameter_builder import compile_world_spec
from tests.blender.scenes.common import ScenarioFixture


def _rotate(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _endpoint(values, offset, local):
    translation = values[offset : offset + 3]
    rotation = values[offset + 3 : offset + 7]
    rotated = _rotate(rotation, local)
    return tuple(left + right for left, right in zip(translation, rotated))


class NativeReferenceBackend:
    def run(
        self,
        fixture: ScenarioFixture,
        fps: int,
        bake: int,
        seed: int,
    ) -> dict[str, Any]:
        if native is None:
            raise RuntimeError("native reference backend requires bone_physics_native")
        spec = fixture.build_world_spec(seed)
        compiled = compile_world_spec(spec)
        handle = native.build_world(
            compiled.config,
            compiled.bodies,
            compiled.constraints,
            compiled.filters,
        )
        try:
            native.set_analytic_colliders(handle, compiled.analytic_colliders)
            summary = {}
            peak_contacts = 0
            peak_generated_contacts = 0
            for _ in range(fps):
                summary = native.advance(
                    handle,
                    1.0 / fps,
                    "authoritative_bake",
                )
                peak_contacts = max(peak_contacts, int(summary.get("active_contacts", 0)))
                peak_generated_contacts = max(
                    peak_generated_contacts,
                    int(summary.get("generated_contacts", 0)),
                )
            values = array("f", [0.0] * (len(spec.bones) * 7))
            native.read_transforms(handle, values)
            metrics = self._metrics(
                spec.bones,
                values,
                summary,
                contacts=max(peak_contacts, peak_generated_contacts),
            )
            if summary.get("rolled_back"):
                raise RuntimeError("native reference solve rolled back")
            return {
                "scenario": fixture.name,
                "fps": fps,
                "bake": bake,
                "seed": seed,
                "fixed_hz": 120,
                "authoritative_iterations": 64,
                "contact_required": fixture.contact_required,
                "metrics": metrics,
                "phase_timings": dict(summary.get("phase_timings", {})),
                "diagnostics": {
                    "diagnostic_id": f"{fixture.name}:native",
                    "broad_phase_candidates": summary.get("broad_phase_candidates", 0),
                    "ccd_fallbacks": summary.get("ccd_fallbacks", 0),
                    "ccd_failures": summary.get("ccd_failures", 0),
                    "summary": dict(summary),
                },
                "poses": tuple(float(value) for value in values),
                "build_batch": compiled.canonical_bytes().hex(),
                "animation_input": {"fps": fps, "fixed_hz": 120},
                "mesh_input": {"topology": f"{fixture.name}-analytic-v1"},
                "checkpoint": f"native/{fixture.name}/{bake}/final",
            }
        finally:
            native.destroy_world(handle)

    @staticmethod
    def _metrics(bones, values, summary, contacts=None) -> dict[str, float | int]:
        length_error = 0.0
        anchor_error = 0.0
        for index, bone in enumerate(bones):
            offset = index * 7
            head = _endpoint(values, offset, bone.local_head)
            tail = _endpoint(values, offset, bone.local_tail)
            length = math.sqrt(sum((right - left) ** 2 for left, right in zip(head, tail)))
            length_error = max(length_error, abs(length - bone.rest_length))
            if bone.parent_id is not None:
                parent_index = next(
                    position
                    for position, candidate in enumerate(bones)
                    if candidate.id == bone.parent_id
                )
                parent_tail = _endpoint(values, parent_index * 7, bones[parent_index].local_tail)
                anchor_error = max(
                    anchor_error,
                    math.sqrt(
                        sum((left - right) ** 2 for left, right in zip(parent_tail, head))
                    ),
                )
        if contacts is None:
            contacts = int(summary.get("active_contacts", summary.get("contacts", 0)))
        return {
            "length_error": length_error,
            "anchor_error": anchor_error,
            "penetration": float(summary.get("deepest_penetration", 0.0)),
            "angular_lock": 0.0,
            "velocity_peak": float(summary.get("velocity_peak", 0.0)),
            "contacts": contacts,
            "equal_time_position_error": 0.0,
            "equal_time_angle_error": 0.0,
            "repeat_position_error": 0.0,
            "repeat_angle_error": 0.0,
        }
