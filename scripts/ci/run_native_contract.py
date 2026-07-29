#!/usr/bin/env python3
"""Build and validate the native bridge in an isolated Python environment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys

from inspect_native_wheel import inspect_wheel
from run_python_tests import command_report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--work-dir", type=Path, required=True)
    arguments = parser.parse_args()
    root = arguments.root.resolve()
    report_directory = arguments.report_dir.resolve()
    work_directory = arguments.work_dir.resolve()
    wheel_directory = work_directory / "dist"
    environment_directory = work_directory / "venv"
    report_directory.mkdir(parents=True, exist_ok=True)
    wheel_directory.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, object]] = []
    status = 0

    maturin = shutil.which("maturin")
    if maturin is None:
        snapshot = {"error": "maturin executable is unavailable", "status": "failed"}
        (report_directory / "failure-snapshot.json").write_text(
            json.dumps(snapshot, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        status = 1
    else:
        build = command_report(
            [
                maturin,
                "build",
                "--release",
                "--manifest-path",
                "crates/bone_physics_py/Cargo.toml",
                "--out",
                str(wheel_directory),
            ],
            report_directory,
            "wheel-build",
            cwd=root,
        )
        results.append(build)
        status = int(build["exit_code"] != 0)

    wheels = sorted(wheel_directory.glob("*.whl"))
    if status == 0 and len(wheels) != 1:
        snapshot = {
            "error": f"expected one built wheel, found {len(wheels)}",
            "status": "failed",
        }
        (report_directory / "failure-snapshot.json").write_text(
            json.dumps(snapshot, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        status = 1

    if status == 0:
        wheel = wheels[0]
        create_environment = command_report(
            [sys.executable, "-m", "venv", str(environment_directory)],
            report_directory,
            "create-venv",
            cwd=root,
        )
        results.append(create_environment)
        status = int(create_environment["exit_code"] != 0)

    if status == 0:
        environment_python = environment_directory / "bin" / "python"
        install = command_report(
            [
                str(environment_python),
                "-m",
                "pip",
                "install",
                "--no-deps",
                str(wheel),
            ],
            report_directory,
            "wheel-install",
            cwd=root,
        )
        results.append(install)
        status = int(install["exit_code"] != 0)

    if status == 0:
        tests = command_report(
            [
                str(environment_python),
                "scripts/ci/run_python_tests.py",
                "--python",
                str(environment_python),
                "--root",
                str(root),
                "--report-dir",
                str(report_directory / "python"),
            ],
            report_directory,
            "native-python-tests",
            cwd=root,
        )
        results.append(tests)
        status = int(tests["exit_code"] != 0)

    if status == 0:
        try:
            dependency_report = inspect_wheel(wheel)
        except Exception as error:
            dependency_report = {
                "error": f"{type(error).__name__}: {error}",
                "status": "failed",
            }
        (report_directory / "wheel-dependencies.json").write_text(
            json.dumps(dependency_report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        status = int(dependency_report["status"] != "ok")
        if status:
            (report_directory / "failure-snapshot.json").write_text(
                json.dumps(dependency_report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

    summary = {
        "commands": results,
        "status": "ok" if status == 0 else "failed",
        "wheel": wheels[0].name if len(wheels) == 1 else None,
    }
    (report_directory / "native-contract-report.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return status


if __name__ == "__main__":
    raise SystemExit(main())
