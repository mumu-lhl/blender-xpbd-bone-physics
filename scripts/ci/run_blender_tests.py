#!/usr/bin/env python3
"""Build, install, and test the extension under background Blender."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from zipfile import ZIP_DEFLATED, ZipFile

SCRIPT_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from tests.blender.validate_manifest import validate_extension


BLENDER_TEST_SCRIPTS = (
    "test_installed_extension.py",
    "test_prephysics_order.py",
    "test_pose_roundtrip.py",
    "test_runtime_handler.py",
    "test_ui_registration.py",
)


def _wheel_native(wheel: Path) -> tuple[str, bytes]:
    with ZipFile(wheel) as archive:
        candidates = sorted(
            name
            for name in archive.namelist()
            if Path(name).name.startswith("bone_physics_native")
            and name.endswith(".so")
        )
        if len(candidates) != 1:
            raise ValueError(
                "native wheel must contain exactly one Linux extension module; "
                f"found {len(candidates)}"
            )
        return Path(candidates[0]).name, archive.read(candidates[0])


def build_extension_bundle(
    extension_source: Path, wheel: Path, output_directory: Path
) -> Path:
    output_directory.mkdir(parents=True, exist_ok=True)
    bundle = output_directory / "bone_physics-0.1.0.zip"
    package = extension_source / "bone_physics"
    manifest = extension_source / "blender_manifest.toml"
    if not manifest.is_file() or not (package / "__init__.py").is_file():
        raise FileNotFoundError("extension source is missing its manifest or package")
    native_name, native_bytes = _wheel_native(wheel)
    with ZipFile(bundle, "w", compression=ZIP_DEFLATED) as archive:
        archive.write(manifest, "blender_manifest.toml")
        for source in sorted(package.rglob("*")):
            if (
                source.is_file()
                and "__pycache__" not in source.parts
                and not source.name.endswith((".pyc", ".pyo"))
                and "native" not in source.relative_to(package).parts
            ):
                archive.write(source, source.relative_to(package).as_posix())
        archive.writestr(
            f"native/linux-x86_64/{native_name}",
            native_bytes,
        )
    return bundle


def _run(
    command: list[str],
    log_path: Path,
    *,
    cwd: Path,
    environment: dict[str, str],
) -> dict[str, object]:
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=cwd,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    log_path.write_text(completed.stdout, encoding="utf-8")
    if completed.stdout:
        print(completed.stdout, end="")
    return {
        "command": command,
        "duration_seconds": time.monotonic() - started,
        "exit_code": completed.returncode,
        "log": log_path.name,
        "status": "ok" if completed.returncode == 0 else "failed",
    }


def run_blender_tests(
    blender: Path,
    root: Path,
    wheel: Path,
    report_directory: Path,
) -> int:
    root = root.resolve()
    report_directory.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, object]] = []
    status = 0
    try:
        with tempfile.TemporaryDirectory(prefix="bone-physics-blender-") as directory:
            temporary = Path(directory)
            bundle = build_extension_bundle(
                root / "blender_extension", wheel.resolve(), temporary / "dist"
            )
            validation_root = temporary / "validation"
            with ZipFile(bundle) as archive:
                archive.extractall(validation_root)
            validation = validate_extension(
                validation_root / "blender_manifest.toml",
                validation_root,
                "linux-x86_64",
            )
            (report_directory / "extension-validation.json").write_text(
                json.dumps(validation, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            staged_bundle = report_directory / bundle.name
            shutil.copy2(bundle, staged_bundle)
            environment = os.environ.copy()
            environment["BLENDER_USER_RESOURCES"] = str(temporary / "user-resources")
            install = _run(
                [
                    str(blender.resolve()),
                    "--command",
                    "extension",
                    "install-file",
                    "--repo",
                    "user_default",
                    "--enable",
                    str(bundle),
                ],
                report_directory / "extension-install.log",
                cwd=root,
                environment=environment,
            )
            install["name"] = "extension-install"
            results.append(install)
            if install["exit_code"] != 0:
                status = 1
            for script_name in BLENDER_TEST_SCRIPTS:
                script = root / "tests" / "blender" / script_name
                result = _run(
                    [
                        str(blender.resolve()),
                        "--background",
                        "--factory-startup",
                        "--python",
                        str(script),
                    ],
                    report_directory / f"{script.stem}.log",
                    cwd=root,
                    environment=environment,
                )
                result["name"] = script.stem
                results.append(result)
                if result["exit_code"] != 0:
                    status = 1
    except Exception as error:
        status = 1
        results.append(
            {
                "error": f"{type(error).__name__}: {error}",
                "exit_code": 1,
                "name": "preflight",
                "status": "failed",
            }
        )
    summary = {
        "blender": str(blender.resolve()),
        "status": "ok" if status == 0 else "failed",
        "tests": results,
        "wheel": wheel.name,
    }
    (report_directory / "blender-runtime-report.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if status:
        failures = [result for result in results if result["exit_code"] != 0]
        (report_directory / "failure-snapshot.json").write_text(
            json.dumps(
                {"failed_commands": failures}, indent=2, sort_keys=True
            )
            + "\n",
            encoding="utf-8",
        )
    return status


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--blender", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--wheel", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    arguments = parser.parse_args()
    return run_blender_tests(
        arguments.blender, arguments.root, arguments.wheel, arguments.report_dir
    )


if __name__ == "__main__":
    raise SystemExit(main())
