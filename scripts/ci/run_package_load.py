#!/usr/bin/env python3
"""Build, package, inspect, and load one platform extension archive."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from zipfile import ZipFile

SCRIPT_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from scripts.check_native_deps import dependencies
from scripts.package_extension import package
from scripts.verify_package import verify


PLATFORM_SYSTEMS = {
    "linux-x86_64": "linux",
    "windows-x86_64": "windows",
    "macos-arm64": "darwin",
    "macos-x86_64": "darwin",
}


def platform_system(platform_tag: str) -> str:
    try:
        return PLATFORM_SYSTEMS[platform_tag]
    except KeyError as error:
        raise ValueError(f"unsupported package platform: {platform_tag}") from error


def _native_suffix(platform_tag: str) -> str:
    return ".pyd" if platform_tag.startswith("windows-") else ".so"


def find_native_member(wheel: Path, platform_tag: str) -> str:
    suffix = _native_suffix(platform_tag)
    with ZipFile(wheel) as archive:
        candidates = sorted(
            name
            for name in archive.namelist()
            if Path(name).name.startswith("bone_physics_native")
            and name.endswith(suffix)
        )
    if len(candidates) != 1:
        raise ValueError(
            f"wheel for {platform_tag} must contain exactly one native module; "
            f"found {len(candidates)}"
        )
    return candidates[0]


def _run(
    command: list[str],
    *,
    cwd: Path,
    output: Path,
    environment: dict[str, str] | None = None,
) -> dict[str, object]:
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        env=environment,
    )
    output.write_text(completed.stdout, encoding="utf-8")
    return {
        "command": command,
        "exit_code": completed.returncode,
        "status": "ok" if completed.returncode == 0 else "failed",
        "log": output.name,
    }


def _python_load_smoke(package_root: Path, *, cwd: Path, output: Path) -> dict[str, object]:
    code = (
        f"import sys; sys.path.insert(0, {str(package_root)!r}); "
        "import json; "
        "import bone_physics; "
        "native = bone_physics.load_native(1); "
        "print(json.dumps({'native_api_version': native.native_api_version(), 'status': 'ok'}))"
    )
    return _run(
        [sys.executable, "-c", code],
        cwd=cwd,
        output=output,
    ) | {"package_root": str(package_root)}


def run_package_load(
    root: Path,
    platform_tag: str,
    report_directory: Path,
    *,
    blender: Path | None = None,
    commit: str | None = None,
    reference_output: Path | None = None,
) -> int:
    root = root.resolve()
    report_directory.mkdir(parents=True, exist_ok=True)
    (report_directory / "failure-snapshot.json").unlink(missing_ok=True)
    status = 0
    reports: dict[str, object] = {"platform": platform_tag}
    if commit is not None:
        reports["commit"] = commit
    try:
        platform_system(platform_tag)
        maturin = shutil.which("maturin")
        maturin_command = [maturin] if maturin is not None else [sys.executable, "-m", "maturin"]
        with tempfile.TemporaryDirectory(prefix="bone-physics-package-") as directory:
            temporary = Path(directory)
            wheel_directory = temporary / "wheel"
            wheel_directory.mkdir()
            build = _run(
                [
                    *maturin_command,
                    "build",
                    "--release",
                    "--manifest-path",
                    str(root / "crates/bone_physics_py/Cargo.toml"),
                    "--out",
                    str(wheel_directory),
                ],
                cwd=root,
                output=report_directory / "wheel-build.log",
            )
            reports["wheel_build"] = build
            if build["exit_code"] != 0:
                raise RuntimeError("maturin wheel build failed")
            wheels = sorted(wheel_directory.glob("*.whl"))
            if len(wheels) != 1:
                raise ValueError(f"expected exactly one wheel, found {len(wheels)}")
            wheel = wheels[0]
            member = find_native_member(wheel, platform_tag)
            native = temporary / Path(member).name
            with ZipFile(wheel) as archive:
                native.write_bytes(archive.read(member))

            archive_path = report_directory / f"bone_physics-{platform_tag}.zip"
            package_report = package(
                root,
                archive_path,
                native=native,
                platform=platform_tag,
            )
            reports["package"] = package_report
            (report_directory / "package.json").write_text(
                json.dumps(package_report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            (report_directory / "package.sha256").write_text(
                (archive_path.with_suffix(archive_path.suffix + ".sha256")).read_text(
                    encoding="utf-8"
                ),
                encoding="utf-8",
            )
            reports["verification"] = verify(
                archive_path,
                expected_platform=platform_tag,
            )
            dependency_report = dependencies(native, platform_system(platform_tag))
            (report_directory / "dependencies.json").write_text(
                json.dumps(dependency_report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            reports["dependencies"] = dependency_report
            if dependency_report["status"] != "ok" or dependency_report["unexpected"]:
                raise RuntimeError("native dependency inspection did not pass")

            installed = temporary / "installed"
            installed_package = installed / "bone_physics"
            installed_package.mkdir(parents=True)
            with ZipFile(archive_path) as archive:
                archive.extractall(installed_package)
            smoke = _python_load_smoke(
                installed,
                cwd=root,
                output=report_directory / "load-test.log",
            )
            reports["load_test"] = smoke
            (report_directory / "load-test.json").write_text(
                json.dumps(smoke, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            if smoke["exit_code"] != 0:
                raise RuntimeError("packaged Python load test failed")
            if reference_output is not None:
                reference_environment = temporary / "reference-env"
                reference_environment.mkdir()
                with ZipFile(wheel) as archive:
                    archive.extractall(reference_environment)
                install = {
                    "status": "ok",
                    "exit_code": 0,
                    "method": "wheel-extract",
                    "wheel": wheel.name,
                }
                reports["reference_install"] = install
                environment = dict(os.environ)
                environment["PYTHONPATH"] = os.pathsep.join(
                    (str(reference_environment), str(root), environment.get("PYTHONPATH", ""))
                )
                reference = _run(
                    [
                        sys.executable,
                        str(root / "tests/blender/run_reference.py"),
                        "--acceptance",
                        str(root / "tests/reference/acceptance.json"),
                        "--output",
                        str(reference_output),
                        "--backend",
                        "native",
                        *( ["--commit", commit] if commit else [] ),
                    ],
                    cwd=root,
                    output=report_directory / "reference.log",
                    environment=environment,
                )
                reports["reference"] = reference
                if reference["exit_code"] != 0:
                    raise RuntimeError("native reference acceptance failed")
            if blender is not None:
                from scripts.ci.run_blender_tests import run_blender_tests

                blender_status = run_blender_tests(
                    blender,
                    root,
                    wheel,
                    report_directory / "blender",
                )
                reports["blender"] = {
                    "status": "ok" if blender_status == 0 else "failed",
                    "report_directory": str(report_directory / "blender"),
                }
                if blender_status:
                    raise RuntimeError("Blender package load test failed")
    except Exception as error:
        status = 1
        reports["error"] = f"{type(error).__name__}: {error}"
        (report_directory / "failure-snapshot.json").write_text(
            json.dumps(reports, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    reports["status"] = "ok" if status == 0 else "failed"
    (report_directory / "package-load-report.json").write_text(
        json.dumps(reports, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return status


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--platform", required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--blender", type=Path)
    parser.add_argument("--commit")
    parser.add_argument("--reference-output", type=Path)
    arguments = parser.parse_args()
    return run_package_load(
        arguments.root,
        arguments.platform,
        arguments.report_dir,
        blender=arguments.blender,
        commit=arguments.commit,
        reference_output=arguments.reference_output,
    )


if __name__ == "__main__":
    raise SystemExit(main())
