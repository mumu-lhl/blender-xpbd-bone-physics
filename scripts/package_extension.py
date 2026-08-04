"""Build a byte-reproducible Blender extension archive."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
import zipfile
from pathlib import Path

EPOCH = (1980, 1, 1, 0, 0, 0)


def _native_suffix(platform: str) -> str:
    return ".pyd" if platform.startswith("windows-") else ".so"


def find_native_member(wheel: Path, platform: str) -> str:
    """Return the single native extension member for a package platform."""
    suffix = _native_suffix(platform)
    with zipfile.ZipFile(wheel) as archive:
        candidates = sorted(
            name
            for name in archive.namelist()
            if Path(name).name.startswith("bone_physics_native") and name.endswith(suffix)
        )
    if len(candidates) != 1:
        raise ValueError(
            f"wheel for {platform} must contain exactly one native module; "
            f"found {len(candidates)}"
        )
    return candidates[0]


def extract_native(wheel: Path, destination: Path, platform: str) -> Path:
    """Extract the native module from a wheel into a temporary/build directory."""
    member = find_native_member(wheel, platform)
    destination.mkdir(parents=True, exist_ok=True)
    native = destination / Path(member).name
    with zipfile.ZipFile(wheel) as archive:
        native.write_bytes(archive.read(member))
    return native


def resolve_wheel(wheel: Path | None, wheel_dir: Path | None) -> Path:
    if wheel is not None and wheel_dir is not None:
        raise ValueError("pass either --wheel or --wheel-dir, not both")
    if wheel is not None:
        if not wheel.is_file():
            raise ValueError(f"native wheel does not exist: {wheel}")
        return wheel
    if wheel_dir is None:
        raise ValueError("one of --native, --wheel, or --wheel-dir is required")
    wheels = sorted(wheel_dir.glob("bone_physics_native-*.whl"))
    if len(wheels) != 1:
        raise ValueError(f"expected exactly one native wheel in {wheel_dir}; found {len(wheels)}")
    return wheels[0]


def package(source: Path, output: Path, *, native: Path | None = None, platform: str = "linux-x86_64") -> dict[str, object]:
    files: dict[str, bytes] = {}
    manifest = source / "blender_extension" / "blender_manifest.toml"
    files["blender_manifest.toml"] = manifest.read_bytes()
    package_root = source / "blender_extension" / "bone_physics"
    for path in sorted(package_root.rglob("*")):
        relative = path.relative_to(package_root)
        if (
            path.is_file()
            and "__pycache__" not in path.parts
            and path.name != "blender_manifest.toml"
            and "native" not in relative.parts
        ):
            files[str(path.relative_to(package_root)).replace(os.sep, "/")] = path.read_bytes()
    if native is not None:
        if not native.is_file():
            raise ValueError(f"native module does not exist: {native}")
        files[f"native/{platform}/{native.name}"] = native.read_bytes()
    if not files:
        raise ValueError("extension archive is empty")

    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name in sorted(files):
            info = zipfile.ZipInfo(name, date_time=EPOCH)
            info.create_system = 3
            info.external_attr = 0o100644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, files[name])
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(output.suffix + ".sha256").write_text(f"{digest}  {output.name}\n", encoding="utf-8")
    report = {
        "archive": output.name,
        "platform": platform,
        "sha256": digest,
        "files": sorted(files),
        "native_count": sum(name.startswith("native/") for name in files),
        "license": "GPL-3.0-or-later",
        "licenses": [
            {
                "component": "bone_physics",
                "spdx": "GPL-3.0-or-later",
                "source": "blender_manifest.toml",
            }
        ],
    }
    output.with_suffix(output.suffix + ".json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    native_group = parser.add_mutually_exclusive_group()
    native_group.add_argument("--native", type=Path)
    native_group.add_argument("--wheel", type=Path)
    native_group.add_argument("--wheel-dir", type=Path)
    parser.add_argument("--platform", default="linux-x86_64")
    args = parser.parse_args()
    native = args.native
    if args.wheel is not None or args.wheel_dir is not None:
        wheel = resolve_wheel(args.wheel, args.wheel_dir)
        with tempfile.TemporaryDirectory(prefix="bone-physics-package-") as directory:
            native = extract_native(wheel, Path(directory), args.platform)
            package(args.source, args.output, native=native, platform=args.platform)
    else:
        package(args.source, args.output, native=native, platform=args.platform)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
