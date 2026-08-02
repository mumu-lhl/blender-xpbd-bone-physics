"""Validate staged Blender extension metadata and native binary selection."""

from __future__ import annotations

from pathlib import Path
import tomllib

from blender_extension.bone_physics.native import matching_native_candidates


def validate_extension(
    manifest_path: Path,
    package_directory: Path,
    platform_tag: str,
) -> dict[str, object]:
    with manifest_path.open("rb") as stream:
        manifest = tomllib.load(stream)
    minimum = manifest.get("blender_version_min")
    if not isinstance(minimum, str) or _version(minimum) < (5, 2, 0):
        raise ValueError("extension must require Blender 5.2.0 or newer")
    maximum = manifest.get("blender_version_max")
    if maximum is not None and (
        not isinstance(maximum, str) or _version(maximum) <= _version(minimum)
    ):
        raise ValueError("Blender maximum version must be greater than its minimum")
    licenses = manifest.get("license")
    if not isinstance(licenses, list) or not any(
        isinstance(value, str) and "GPL-3.0-or-later" in value for value in licenses
    ):
        raise ValueError("extension manifest must declare GPL-3.0-or-later")
    candidates = matching_native_candidates(package_directory, platform_tag)
    if len(candidates) != 1:
        raise ValueError(
            f"staged extension requires exactly one native binary for {platform_tag}; "
            f"found {len(candidates)}"
        )
    return {
        "blender_version_min": minimum,
        "blender_version_max": maximum,
        "license": tuple(licenses),
        "native_binary_count": len(candidates),
        "native_binary": str(candidates[0]),
    }


def _version(value: str) -> tuple[int, int, int]:
    parts = value.split(".")
    if len(parts) != 3:
        raise ValueError(f"Blender version must have three components: {value!r}")
    try:
        return tuple(int(part) for part in parts)  # type: ignore[return-value]
    except ValueError as error:
        raise ValueError(f"invalid Blender version {value!r}") from error
