"""Build a byte-reproducible Blender extension archive."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import zipfile
from pathlib import Path

EPOCH = (1980, 1, 1, 0, 0, 0)


def package(source: Path, output: Path, *, native: Path | None = None, platform: str = "linux-x86_64") -> dict[str, object]:
    files: dict[str, bytes] = {}
    manifest = source / "blender_extension" / "blender_manifest.toml"
    files["blender_manifest.toml"] = manifest.read_bytes()
    package_root = source / "blender_extension" / "bone_physics"
    for path in sorted(package_root.rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts and path.name != "blender_manifest.toml":
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
    }
    output.with_suffix(output.suffix + ".json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native", type=Path)
    parser.add_argument("--platform", default="linux-x86_64")
    args = parser.parse_args()
    package(args.source, args.output, native=args.native, platform=args.platform)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
