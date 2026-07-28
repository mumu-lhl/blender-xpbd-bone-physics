#!/usr/bin/env python3
"""Validate the deterministic Rust reference-scene report."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


EXPECTED_SCENES = {
    "single_pendulum",
    "colliding_hair_chains",
    "root_stiff_tip_soft_tail",
    "closed_skirt_lattice",
}
FINITE_FIELDS = {
    "max_bone_length_error",
    "max_joint_separation",
    "max_hard_anchor_separation",
    "max_residual_penetration",
    "peak_velocity",
    "peak_angular_velocity",
}
COUNT_FIELDS = {"generated_contacts", "active_contacts"}
PHASE_TIMING_FIELDS = {
    "prediction_seconds",
    "animation_inertia_seconds",
    "contact_generation_seconds",
    "position_solve_seconds",
    "diagnostics_seconds",
    "velocity_seconds",
}


def fail(message: str) -> None:
    raise SystemExit(f"reference report invalid: {message}")


def initialize(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "status": "pending",
        "scenes": [],
        "failure": {
            "kind": "reference_test_not_completed",
            "message": "The reference test did not produce a passing report.",
        },
        "replay": {
            "fixture_version": 1,
            "fixed_dt": 1.0 / 120.0,
            "steps": 240,
        },
    }
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"initialized failure-safe reference report at {path}")


def main() -> None:
    if len(sys.argv) == 3 and sys.argv[1] == "--initialize":
        initialize(Path(sys.argv[2]))
        return
    if len(sys.argv) != 2:
        fail("usage: reference_report.py [--initialize] PATH")
    path = Path(sys.argv[1])
    if not path.is_file() or path.stat().st_size == 0:
        fail(f"{path} is missing or empty")

    report = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(report, dict) or report.get("status") != "passed":
        fail("status must be passed")
    rows = report.get("scenes")
    if not isinstance(rows, list):
        fail("scenes must be an array")
    scenes = {row.get("scene") for row in rows if isinstance(row, dict)}
    if scenes != EXPECTED_SCENES or len(rows) != len(EXPECTED_SCENES):
        fail(f"expected {sorted(EXPECTED_SCENES)}, got {sorted(map(str, scenes))}")

    for row in rows:
        scene = row["scene"]
        for field in FINITE_FIELDS:
            value = row.get(field)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                fail(f"{scene}.{field} must be finite")
        for field in COUNT_FIELDS:
            value = row.get(field)
            if not isinstance(value, int) or value < 0:
                fail(f"{scene}.{field} must be a non-negative integer")
        timings = row.get("phase_timings")
        if not isinstance(timings, dict) or set(timings) != PHASE_TIMING_FIELDS:
            fail(
                f"{scene}.phase_timings must contain "
                f"{sorted(PHASE_TIMING_FIELDS)}"
            )
        if any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
            for value in timings.values()
        ):
            fail(f"{scene}.phase_timings values must be finite and non-negative")

    if report.get("failure") is not None:
        fail("a passing report cannot contain failure metadata")
    replay = report.get("replay")
    if not isinstance(replay, dict) or replay.get("fixture_version") != 1:
        fail("replay metadata is missing or incompatible")

    print(f"validated {len(rows)} reference rows from {path}")


if __name__ == "__main__":
    main()
