"""Install one built native wheel into the source Blender extension tree."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import sys

SCRIPT_ROOT = Path(__file__).resolve().parents[1]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from scripts.package_extension import extract_native, resolve_wheel


def install_native(
    extension_root: Path,
    wheel: Path | None = None,
    wheel_dir: Path | None = None,
    *,
    platform: str = "linux-x86_64",
) -> Path:
    """Copy the wheel's native module to ``bone_physics/native/<platform>``."""
    selected_wheel = resolve_wheel(wheel, wheel_dir)
    destination = extension_root / "bone_physics" / "native" / platform
    destination.mkdir(parents=True, exist_ok=True)
    suffix = ".pyd" if platform.startswith("windows-") else ".so"
    for stale in destination.glob(f"bone_physics_native*{suffix}"):
        if stale.is_file() and not stale.is_symlink():
            stale.unlink()
    temporary = destination / ".native-staging"
    temporary.mkdir(exist_ok=True)
    try:
        extracted = extract_native(selected_wheel, temporary, platform)
        installed = destination / extracted.name
        shutil.copy2(extracted, installed)
    finally:
        shutil.rmtree(temporary, ignore_errors=True)
    return installed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--extension-root", type=Path, required=True)
    native_group = parser.add_mutually_exclusive_group(required=True)
    native_group.add_argument("--wheel", type=Path)
    native_group.add_argument("--wheel-dir", type=Path)
    parser.add_argument("--platform", default="linux-x86_64")
    args = parser.parse_args()
    installed = install_native(
        args.extension_root,
        args.wheel,
        args.wheel_dir,
        platform=args.platform,
    )
    print(installed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
