#!/usr/bin/env python3
"""Run a command while always producing structured CI evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def command_report(
    command: list[str],
    report_directory: Path,
    name: str,
    *,
    cwd: Path | None = None,
) -> dict[str, object]:
    report_directory.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    duration = time.monotonic() - started
    log_path = report_directory / f"{name}.log"
    log_path.write_text(completed.stdout, encoding="utf-8")
    result: dict[str, object] = {
        "command": command,
        "duration_seconds": duration,
        "exit_code": completed.returncode,
        "log": log_path.name,
        "name": name,
        "status": "ok" if completed.returncode == 0 else "failed",
    }
    (report_directory / f"{name}.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if completed.returncode:
        snapshot = {
            "exit_code": completed.returncode,
            "failed_command": name,
            "log_tail": completed.stdout.splitlines()[-100:],
        }
        (report_directory / "failure-snapshot.json").write_text(
            json.dumps(snapshot, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    if completed.stdout:
        print(completed.stdout, end="")
    return result


def unittest_arguments(root: Path, exclude_prefixes: list[str]) -> list[str]:
    if not exclude_prefixes:
        return ["-m", "unittest", "discover", "-s", "tests/python", "-v"]
    return [
        "-m",
        "unittest",
        "-v",
        *(
            f"tests.python.{path.stem}"
            for path in sorted((root / "tests" / "python").glob("test_*.py"))
            if not any(path.stem.startswith(prefix) for prefix in exclude_prefixes)
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--exclude-prefix", action="append", default=[])
    arguments = parser.parse_args()
    test_arguments = unittest_arguments(arguments.root, arguments.exclude_prefix)
    commands = (
        (
            "compileall",
            [
                arguments.python,
                "-m",
                "compileall",
                "-q",
                "blender_extension/bone_physics",
                "tests/python",
            ],
        ),
        (
            "unittest",
            [arguments.python, *test_arguments],
        ),
    )
    results = []
    status = 0
    for name, command in commands:
        result = command_report(
            command, arguments.report_dir, name, cwd=arguments.root.resolve()
        )
        results.append(result)
        if result["exit_code"] != 0:
            status = 1
    summary = {
        "python": arguments.python,
        "status": "ok" if status == 0 else "failed",
        "commands": results,
    }
    (arguments.report_dir / "python-test-report.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return status


if __name__ == "__main__":
    raise SystemExit(main())
