"""Deterministic reference acceptance runner.

The runner is Blender-independent for contract tests. Blender CI can replace
the fixture backend with the native scene backend while retaining the same
report schema and fail-closed checks.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

SCRIPT_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from tests.blender.scenes.body_collision import (
    breast_torso,
    deforming_body,
    fast_arm,
    initial_penetration,
)
from tests.blender.scenes.hair import multiple_hair, single_hair
from tests.blender.scenes.skirt import closed_skirt, moving_legs
from tests.blender.scenes.tail import graded_tail, single_pendulum
from tests.reference.native_backend import NativeReferenceBackend

FIXED_HZ = 120
AUTHORITATIVE_ITERATIONS = 64
FPS = (24, 30, 60)
SCENARIO_NAMES = (
    "single_pendulum",
    "single_hair",
    "multiple_hair",
    "graded_tail",
    "closed_skirt",
    "fast_arm",
    "moving_legs",
    "initial_penetration",
    "deforming_body",
    "breast_torso",
)
CONTACT_REQUIRED = frozenset(
    {"multiple_hair", "fast_arm", "moving_legs", "initial_penetration", "breast_torso"}
)
PHASE_NAMES = (
    "prediction_seconds",
    "animation_inertia_seconds",
    "contact_generation_seconds",
    "position_solve_seconds",
    "diagnostics_seconds",
    "velocity_seconds",
)


def load_acceptance(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported acceptance schema")
    if tuple(data.get("scenarios", ())) != SCENARIO_NAMES:
        raise ValueError("acceptance scenario registry is incomplete or reordered")
    if tuple(data.get("fps", ())) != FPS:
        raise ValueError("acceptance FPS matrix must be 24/30/60")
    if data.get("fixed_hz") != FIXED_HZ:
        raise ValueError("acceptance must use a 120 Hz physical step")
    if data.get("authoritative_iterations") != AUTHORITATIVE_ITERATIONS:
        raise ValueError("authoritative iteration budget changed")
    thresholds = data.get("thresholds")
    required = {
        "rigid_length",
        "hard_anchor",
        "residual_penetration",
        "angular_lock",
        "equal_time_position",
        "equal_time_angle",
        "repeated_position",
        "repeated_angle",
    }
    if not isinstance(thresholds, dict) or not required.issubset(thresholds):
        raise ValueError("acceptance thresholds are incomplete")
    return data


FIXTURE_BUILDERS = {
    "single_pendulum": single_pendulum,
    "single_hair": single_hair,
    "multiple_hair": multiple_hair,
    "graded_tail": graded_tail,
    "closed_skirt": closed_skirt,
    "fast_arm": fast_arm,
    "moving_legs": moving_legs,
    "initial_penetration": initial_penetration,
    "deforming_body": deforming_body,
    "breast_torso": breast_torso,
}


def run_matrix(
    acceptance: dict[str, Any],
    *,
    seed: int,
    backend: str = "fixture",
) -> dict[str, Any]:
    thresholds = acceptance["thresholds"]
    runs: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    native_backend = NativeReferenceBackend() if backend == "native" else None
    if backend not in {"fixture", "native"}:
        raise ValueError(f"unsupported reference backend: {backend}")
    for scenario in SCENARIO_NAMES:
        for fps in FPS:
            for bake in (1, 2):
                if native_backend is None:
                    record = _run_fixture(scenario, fps, bake, seed)
                else:
                    record = native_backend.run(
                        FIXTURE_BUILDERS[scenario](),
                        fps,
                        bake,
                        seed,
                    )
                runs.append(record)
    _add_comparison_metrics(runs)
    for record in runs:
        failures.extend(_check_record(record, thresholds, seed))

    return {
        "schema_version": 1,
        "backend": backend,
        "seed": seed,
        "fixed_hz": FIXED_HZ,
        "authoritative_iterations": AUTHORITATIVE_ITERATIONS,
        "scenario_count": len(SCENARIO_NAMES),
        "run_count": len(runs),
        "runs": runs,
        "failures": failures,
    }


def _add_comparison_metrics(runs: list[dict[str, Any]]) -> None:
    groups: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for record in runs:
        groups.setdefault((record["scenario"], record["bake"]), []).append(record)
    for records in groups.values():
        baseline = next(record for record in records if record["fps"] == 24)
        for record in records:
            if "poses" not in record:
                continue
            position_error, angle_error = _pose_difference(record["poses"], baseline["poses"])
            record["metrics"]["equal_time_position_error"] = position_error
            record["metrics"]["equal_time_angle_error"] = angle_error

    repeats: dict[tuple[str, int], dict[int, dict[str, Any]]] = {}
    for record in runs:
        repeats.setdefault((record["scenario"], record["fps"]), {})[record["bake"]] = record
    for pair in repeats.values():
        if 1 not in pair or 2 not in pair or "poses" not in pair[1]:
            continue
        position_error, angle_error = _pose_difference(pair[1]["poses"], pair[2]["poses"])
        pair[1]["metrics"]["repeat_position_error"] = position_error
        pair[1]["metrics"]["repeat_angle_error"] = angle_error
        pair[2]["metrics"]["repeat_position_error"] = position_error
        pair[2]["metrics"]["repeat_angle_error"] = angle_error


def _pose_difference(first, second) -> tuple[float, float]:
    position_error = 0.0
    angle_error = 0.0
    for offset in range(0, len(first), 7):
        position_error = max(
            position_error,
            max(abs(left - right) for left, right in zip(first[offset : offset + 3], second[offset : offset + 3])),
        )
        dot = abs(sum(left * right for left, right in zip(first[offset + 3 : offset + 7], second[offset + 3 : offset + 7])))
        angle_error = max(angle_error, 2.0 * math.acos(min(1.0, dot)))
    return position_error, angle_error


def _run_fixture(scenario: str, fps: int, bake: int, seed: int) -> dict[str, Any]:
    contacts = {
        "multiple_hair": 18,
        "fast_arm": 3,
        "moving_legs": 8,
        "initial_penetration": 2,
        "breast_torso": 1,
    }.get(scenario, 0)
    metrics = {
        "length_error": 5.0e-7,
        "anchor_error": 5.0e-6,
        "penetration": 4.0e-6,
        "angular_lock": 4.0e-4,
        "velocity_peak": 8.0,
        "contacts": contacts,
        "equal_time_position_error": 2.0e-5,
        "equal_time_angle_error": 4.0e-4,
        "repeat_position_error": 5.0e-7,
        "repeat_angle_error": 5.0e-6,
    }
    return {
        "scenario": scenario,
        "fps": fps,
        "bake": bake,
        "seed": seed,
        "fixed_hz": FIXED_HZ,
        "authoritative_iterations": AUTHORITATIVE_ITERATIONS,
        "contact_required": scenario in CONTACT_REQUIRED,
        "metrics": metrics,
        "phase_timings": {name: 0.001 for name in PHASE_NAMES},
        "diagnostics": {
            "diagnostic_id": f"{scenario}:ok",
            "broad_phase_candidates": max(contacts, 1),
            "ccd_fallbacks": 0,
            "ccd_failures": 0,
        },
    }


def _check_record(record: dict[str, Any], thresholds: dict[str, float], seed: int) -> list[dict[str, Any]]:
    metrics = record["metrics"]
    checks = {
        "rigid_length": ("length_error", metrics["length_error"]),
        "hard_anchor": ("anchor_error", metrics["anchor_error"]),
        "residual_penetration": ("penetration", metrics["penetration"]),
        "angular_lock": ("angular_lock", metrics["angular_lock"]),
        "equal_time_position": ("equal_time_position_error", metrics["equal_time_position_error"]),
        "equal_time_angle": ("equal_time_angle_error", metrics["equal_time_angle_error"]),
        "repeated_position": ("repeat_position_error", metrics["repeat_position_error"]),
        "repeated_angle": ("repeat_angle_error", metrics["repeat_angle_error"]),
    }
    failures = []
    for criterion, (metric, actual) in checks.items():
        allowed = thresholds[criterion]
        if actual > allowed:
            failures.append(_failure(record, seed, criterion, metric, actual, allowed))
    if record["contact_required"] and metrics["contacts"] <= 0:
        failures.append(_failure(record, seed, "contacts", "contacts", metrics["contacts"], "> 0"))
    return failures


def _failure(record: dict[str, Any], seed: int, criterion: str, metric: str, actual: Any, allowed: Any) -> dict[str, Any]:
    scenario = record["scenario"]
    return {
        "scenario": scenario,
        "fps": record["fps"],
        "bake": record["bake"],
        "seed": seed,
        "build_batch": record.get("build_batch", {"scenario": scenario, "fps": record["fps"], "bake": record["bake"]}),
        "animation_input": record.get("animation_input", {"frame_rate": record["fps"], "fixed_hz": FIXED_HZ}),
        "mesh_input": record.get("mesh_input", {"vertices": [], "topology": f"{scenario}-topology-v1"}),
        "checkpoint": record.get("checkpoint", f"checkpoints/{scenario}/{record['bake']}/frame-000120.bin"),
        "diagnostic_id": record.get("diagnostics", {}).get("diagnostic_id", f"{scenario}:{criterion}"),
        "metric": metric,
        "actual": actual,
        "allowed": allowed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--acceptance", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260801)
    parser.add_argument("--backend", choices=("native", "fixture"), default="native")
    args = parser.parse_args()
    acceptance = load_acceptance(args.acceptance)
    report = run_matrix(acceptance, seed=args.seed, backend=args.backend)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 1 if report["failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
