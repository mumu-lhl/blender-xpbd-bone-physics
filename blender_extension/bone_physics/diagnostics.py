"""Structured diagnostics shared by preview, bake, and acceptance runners."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping


_PHASES = (
    "prediction_seconds",
    "animation_inertia_seconds",
    "contact_generation_seconds",
    "position_solve_seconds",
    "diagnostics_seconds",
    "velocity_seconds",
)


@dataclass(frozen=True, slots=True)
class DiagnosticReport:
    generated_contacts: int = 0
    active_contacts: int = 0
    broad_phase_candidates: int = 0
    ccd_fallbacks: int = 0
    ccd_failures: int = 0
    deepest_penetration: float = 0.0
    max_constraint_error: float = 0.0
    rolled_back: bool = False
    phase_timings: Mapping[str, float] = field(default_factory=dict)
    failure: Mapping[str, object] | None = None
    cache_events: tuple[str, ...] = ()

    @classmethod
    def from_native(
        cls,
        summary: Mapping[str, object],
        phase_timings: Mapping[str, object],
        *,
        failure: Mapping[str, object] | None = None,
    ) -> "DiagnosticReport":
        timings = {
            name: _finite_float(phase_timings.get(name, 0.0), name)
            for name in _PHASES
        }
        return cls(
            generated_contacts=_nonnegative_int(summary.get("generated_contacts", 0)),
            active_contacts=_nonnegative_int(summary.get("active_contacts", 0)),
            broad_phase_candidates=_nonnegative_int(
                summary.get("broad_phase_candidates", 0)
            ),
            ccd_fallbacks=_nonnegative_int(summary.get("ccd_fallbacks", 0)),
            ccd_failures=_nonnegative_int(summary.get("ccd_failures", 0)),
            deepest_penetration=_finite_float(
                summary.get("deepest_penetration", 0.0), "deepest_penetration"
            ),
            max_constraint_error=_finite_float(
                summary.get("max_constraint_error", 0.0), "max_constraint_error"
            ),
            rolled_back=bool(summary.get("rolled_back", False)),
            phase_timings=timings,
            failure=dict(failure) if failure is not None else None,
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "generated_contacts": self.generated_contacts,
            "active_contacts": self.active_contacts,
            "broad_phase_candidates": self.broad_phase_candidates,
            "ccd_fallbacks": self.ccd_fallbacks,
            "ccd_failures": self.ccd_failures,
            "deepest_penetration": self.deepest_penetration,
            "max_constraint_error": self.max_constraint_error,
            "rolled_back": self.rolled_back,
            "phase_timings": dict(self.phase_timings),
            "failure": dict(self.failure) if self.failure is not None else None,
            "cache_events": list(self.cache_events),
        }

    def failure_snapshot(
        self,
        *,
        seed: int,
        actual: float,
        allowed: float,
        build_batch: object | None = None,
        animation_input: object | None = None,
        mesh_vertices: object | None = None,
        checkpoint: object | None = None,
    ) -> dict[str, object]:
        failure = self.failure or {}
        return {
            "seed": seed,
            "diagnostic_id": failure.get("id", "unknown"),
            "stable_ids": {
                key: failure[key]
                for key in ("body", "constraint", "contact", "mesh", "triangle")
                if key in failure
            },
            "actual": actual,
            "allowed": allowed,
            "build_batch": build_batch,
            "animation_input": animation_input,
            "mesh_vertices": mesh_vertices,
            "checkpoint": checkpoint,
            "diagnostics": self.to_dict(),
        }


def _nonnegative_int(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"counter must be a nonnegative integer, received {value!r}")
    return value


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted
