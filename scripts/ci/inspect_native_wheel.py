#!/usr/bin/env python3
"""Inspect the Linux extension module's declared shared-library dependencies."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile
from zipfile import ZipFile


_SYSTEM_DEPENDENCY_PATTERNS = (
    re.compile(r"libc\.so\.\d+"),
    re.compile(r"libm\.so\.\d+"),
    re.compile(r"libgcc_s\.so\.\d+"),
    re.compile(r"libpthread\.so\.\d+"),
    re.compile(r"libdl\.so\.\d+"),
    re.compile(r"ld-linux-[a-z0-9_-]+\.so\.\d+"),
)


def unexpected_dependencies(dependencies: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(
        sorted(
            dependency
            for dependency in dependencies
            if not any(
                pattern.fullmatch(dependency)
                for pattern in _SYSTEM_DEPENDENCY_PATTERNS
            )
        )
    )


def inspect_wheel(wheel: Path) -> dict[str, object]:
    with tempfile.TemporaryDirectory(prefix="bone-physics-wheel-") as directory:
        root = Path(directory)
        with ZipFile(wheel) as archive:
            candidates = sorted(
                name
                for name in archive.namelist()
                if Path(name).name.startswith("bone_physics_native")
                and name.endswith(".so")
            )
            if len(candidates) != 1:
                raise ValueError(
                    "wheel must contain exactly one Linux extension module; "
                    f"found {len(candidates)}"
                )
            archive.extract(candidates[0], root)
        native = root / candidates[0]
        completed = subprocess.run(
            ["readelf", "-d", str(native)],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        if completed.returncode:
            raise RuntimeError(f"readelf failed: {completed.stdout}")
        dependencies = tuple(
            match.group(1)
            for line in completed.stdout.splitlines()
            if (match := re.search(r"\(NEEDED\).*\[([^\]]+)\]", line))
        )
    unexpected = unexpected_dependencies(dependencies)
    return {
        "dependencies": dependencies,
        "native_binary": candidates[0],
        "status": "ok" if not unexpected else "failed",
        "unexpected_dependencies": unexpected,
        "wheel": wheel.name,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheel", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    try:
        result = inspect_wheel(arguments.wheel)
    except Exception as error:
        result = {
            "error": f"{type(error).__name__}: {error}",
            "status": "failed",
            "wheel": arguments.wheel.name,
        }
    arguments.report.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if result["status"] != "ok":
        (arguments.report.parent / "failure-snapshot.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
