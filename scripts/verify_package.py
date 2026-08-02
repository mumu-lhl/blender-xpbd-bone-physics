"""Fail-closed structural verification for an extension archive."""

from __future__ import annotations

import argparse
import hashlib
import json
import zipfile
from pathlib import Path


def verify(archive: Path, *, expected_platform: str | None = None) -> dict[str, object]:
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    sidecar = archive.with_suffix(archive.suffix + ".sha256")
    if not sidecar.is_file() or not sidecar.read_text(encoding="utf-8").startswith(digest + "  "):
        raise ValueError("archive SHA-256 sidecar is missing or stale")
    with zipfile.ZipFile(archive) as zipped:
        names = zipped.namelist()
        if names != sorted(names) or len(names) != len(set(names)):
            raise ValueError("archive entries are not unique and sorted")
        if "blender_manifest.toml" not in names:
            raise ValueError("Blender manifest is missing")
        if any("__pycache__" in name or name.endswith(".pyc") for name in names):
            raise ValueError("archive contains Python cache files")
        native = [name for name in names if name.startswith("native/")]
        if expected_platform is not None and len(native) != 1 or expected_platform is not None and expected_platform not in native[0]:
            raise ValueError("archive does not contain exactly one matching native module")
        report = {
            "sha256": digest,
            "files": names,
            "native_count": len(native),
            "platform": expected_platform,
        }
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("archive", type=Path)
    parser.add_argument("--platform")
    args = parser.parse_args()
    print(json.dumps(verify(args.archive, expected_platform=args.platform), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
