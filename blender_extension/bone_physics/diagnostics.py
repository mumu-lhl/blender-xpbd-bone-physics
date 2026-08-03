"""Structured diagnostics shared by preview, bake, and acceptance runners."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable, Mapping


_SEVERITY_ORDER = {"error": 0, "warning": 1, "info": 2}


@dataclass(frozen=True, slots=True)
class DiagnosticItem:
    """A presentation-ready diagnostic with a stable authoring location."""

    code: str
    severity: str
    entity_kind: str
    entity_id: str
    arguments: tuple[tuple[str, object], ...] = ()
    stable_id: int = 0

    def __post_init__(self) -> None:
        if self.severity not in _SEVERITY_ORDER:
            raise ValueError(f"unsupported diagnostic severity {self.severity!r}")
        if not self.code or not self.entity_kind or not self.entity_id:
            raise ValueError("diagnostics require code, entity kind, and entity ID")
        if isinstance(self.stable_id, bool) or not isinstance(self.stable_id, int) or self.stable_id < 0:
            raise ValueError("diagnostic stable ID must be a nonnegative integer")
        object.__setattr__(self, "arguments", tuple(sorted(self.arguments)))


@dataclass(frozen=True, slots=True)
class DiagnosticOwner:
    """The authoring entity that owns one native stable ID."""

    entity_kind: str
    entity_id: str
    system_id: str | None = None


@dataclass(frozen=True, slots=True)
class DiagnosticBindings:
    """Stable-ID lookup tables retained beside a compiled runtime world."""

    bodies: Mapping[int, DiagnosticOwner] = field(default_factory=dict)
    constraints: Mapping[int, DiagnosticOwner] = field(default_factory=dict)
    colliders: Mapping[int, DiagnosticOwner] = field(default_factory=dict)
    meshes: Mapping[int, DiagnosticOwner] = field(default_factory=dict)
    regions: Mapping[tuple[int, int], DiagnosticOwner] = field(default_factory=dict)

    def body_owner(self, stable_id: int) -> DiagnosticOwner | None:
        return self.bodies.get(stable_id)


def diagnostic_bindings(config: object) -> DiagnosticBindings:
    """Build ID-to-authoring ownership without importing Blender or native code."""
    body_owner: dict[str, DiagnosticOwner] = {}
    chain_owner: dict[str, DiagnosticOwner] = {}
    for system in getattr(config, "physics_systems", ()):
        for chain in getattr(system, "chains", ()):
            chain_owner[chain.id] = DiagnosticOwner("chain", chain.id, system.id)
            for bone_uuid in chain.bone_uuids:
                body_owner[bone_uuid] = DiagnosticOwner("bone", bone_uuid, system.id)
    assignments = dict(getattr(config, "stable_ids", ()))
    bodies = {
        stable_id: body_owner[key.removeprefix("body:")]
        for key, stable_id in assignments.items()
        if key.startswith("body:") and key.removeprefix("body:") in body_owner
    }
    constraints = {
        stable_id: body_owner[key.split(":", 1)[1]]
        for key, stable_id in assignments.items()
        if key.startswith(("joint:", "angular:", "recovery:"))
        and key.split(":", 1)[1] in body_owner
    }
    colliders = {
        stable_id: body_owner[key.removeprefix("collider:")]
        for key, stable_id in assignments.items()
        if key.startswith("collider:") and key.removeprefix("collider:") in body_owner
    }
    meshes = {
        int(resource.stable_id): DiagnosticOwner("mesh", resource.id)
        for resource in getattr(config, "collision_resources", ())
        if resource.__class__.__name__ == "MeshCollisionResource"
    }
    return DiagnosticBindings(bodies, constraints, colliders, meshes)


def map_runtime_error(config: object, values: Mapping[str, object]) -> DiagnosticItem:
    """Map one native failure to the most specific known authoring entity.

    Unknown IDs deliberately remain visible as native diagnostics rather than
    being dropped: a stale world and a stale authoring snapshot are actionable.
    """
    lookup = (
        ("body_id", "bodies", None),
        ("constraint_id", "constraints", None),
        ("collider_id", "colliders", None),
        ("mesh_id", "meshes", None),
    )
    owner = None
    stable_id = 0
    identifier = "native"
    if "mesh_id" in values and "region_id" in values:
        mesh_id, region_id = int(values["mesh_id"]), int(values["region_id"])
        owner = getattr(config, "regions", {}).get((mesh_id, region_id))
        stable_id, identifier = region_id, "region_id"
    if owner is None:
        for key, collection, _unused in lookup:
            if key not in values:
                continue
            stable_id, identifier = int(values[key]), key
            if key == "body_id" and hasattr(config, "body_owner"):
                owner = config.body_owner(stable_id)
            else:
                owner = getattr(config, collection, {}).get(stable_id)
            if owner is not None:
                break
    severity = str(values.get("severity", "error")).lower()
    if severity not in _SEVERITY_ORDER:
        severity = "error"
    code = str(values.get("code", f"runtime.{identifier.replace('_id', '')}_failure"))
    arguments = [(str(key), value) for key, value in values.items()]
    if owner is None:
        return DiagnosticItem(code, severity, "runtime", str(stable_id), tuple(arguments), stable_id)
    if owner.system_id is not None:
        arguments.append(("system_id", owner.system_id))
    return DiagnosticItem(code, severity, owner.entity_kind, owner.entity_id, tuple(arguments), stable_id)


def authoring_diagnostics(issues: Iterable[object]) -> tuple[DiagnosticItem, ...]:
    """Adapt presentation-free authoring validation issues without Blender."""
    result = []
    for issue in issues:
        entity_ids = tuple(getattr(issue, "entity_ids", ()))
        entity_id = entity_ids[0] if entity_ids else "armature"
        code = str(getattr(issue, "code"))
        prefix = code.split(".", 1)[0]
        kind = {"chain": "chain", "resource": "resource", "target": "bone"}.get(prefix, "armature")
        severity = str(getattr(issue, "severity")).lower().split(".")[-1]
        result.append(DiagnosticItem(code, severity, kind, entity_id, tuple(getattr(issue, "arguments", ()))) )
    return merge_diagnostics(result)


def merge_diagnostics(*sources: Iterable[DiagnosticItem]) -> tuple[DiagnosticItem, ...]:
    """Return a deterministic merged view: errors, warnings, then stable IDs."""
    items = (item for source in sources for item in source)
    return tuple(sorted(items, key=lambda item: (_SEVERITY_ORDER[item.severity], item.stable_id, item.entity_kind, item.entity_id, item.code)))


def diagnostic_counts(items: Iterable[DiagnosticItem]) -> dict[str, int]:
    counts = {severity: 0 for severity in _SEVERITY_ORDER}
    for item in items:
        counts[item.severity] += 1
    return counts


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
