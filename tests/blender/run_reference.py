"""Deterministic reference acceptance runner.

The runner is Blender-independent for contract tests. Blender CI can replace
the fixture backend with the native scene backend while retaining the same
report schema and fail-closed checks.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

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


def run_matrix(acceptance: dict[str, Any], *, seed: int) -> dict[str, Any]:
    thresholds = acceptance["thresholds"]
    runs: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    for scenario in SCENARIO_NAMES:
        for fps in FPS:
            for bake in (1, 2):
                record = _run_fixture(scenario, fps, bake, seed)
                runs.append(record)
                failures.extend(_check_record(record, thresholds, seed))

    return {
        "schema_version": 1,
        "seed": seed,
        "fixed_hz": FIXED_HZ,
        "authoritative_iterations": AUTHORITATIVE_ITERATIONS,
        "scenario_count": len(SCENARIO_NAMES),
        "run_count": len(runs),
        "runs": runs,
        "failures": failures,
    }


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
        "build_batch": {"scenario": scenario, "fps": record["fps"], "bake": record["bake"]},
        "animation_input": {"frame_rate": record["fps"], "fixed_hz": FIXED_HZ},
        "mesh_input": {"vertices": [], "topology": f"{scenario}-topology-v1"},
        "checkpoint": f"checkpoints/{scenario}/{record['bake']}/frame-000120.bin",
        "diagnostic_id": f"{scenario}:{criterion}",
        "metric": metric,
        "actual": actual,
        "allowed": allowed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--acceptance", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260801)
    args = parser.parse_args()
    acceptance = load_acceptance(args.acceptance)
    report = run_matrix(acceptance, seed=args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 1 if report["failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
