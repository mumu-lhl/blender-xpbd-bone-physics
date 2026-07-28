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


def fail(message: str) -> None:
    raise SystemExit(f"reference report invalid: {message}")


def main() -> None:
    if len(sys.argv) != 2:
        fail("usage: reference_report.py PATH")
    path = Path(sys.argv[1])
    if not path.is_file() or path.stat().st_size == 0:
        fail(f"{path} is missing or empty")

    rows = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(rows, list):
        fail("top-level value must be an array")
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
        if not isinstance(timings, dict) or not timings:
            fail(f"{scene}.phase_timings must be nonempty")
        if any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
            for value in timings.values()
        ):
            fail(f"{scene}.phase_timings values must be finite and non-negative")

    print(f"validated {len(rows)} reference rows from {path}")


if __name__ == "__main__":
    main()
