"""Portable, validated JSON presets for Bone Physics parameters.

The schema and file operations in this module are intentionally Blender-free.
Only :func:`preset_directories` consults Blender's user-resource paths when
Blender is available.
"""

from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path
from typing import Any, Mapping


PRESET_KIND = "bone_parameters"
SCHEMA_VERSION = 1
_PACKAGE_ROOT = Path(__file__).with_name("presets")
_ENDPOINT_FIELDS = frozenset((
    "inverse_mass", "inertia_scale", "gravity_scale", "translation_inertia",
    "rotation_inertia", "linear_damping", "angular_damping", "joint_compliance",
    "joint_damping", "angular_compliance", "angular_damping_constraint",
    "recovery_compliance", "recovery_damping", "angular_limit_mode", "xyz_lower",
    "xyz_upper", "swing_axis", "max_swing", "min_twist", "max_twist", "radius",
    "margin", "capsule_head_inset", "capsule_tail_inset", "friction", "restitution",
    "interpolation",
))
_ROOT_FIELDS = frozenset(("kind", "schema_version", "name", "head", "tail", "overrides"))
_NON_NEGATIVE = frozenset((
    "inverse_mass", "linear_damping", "angular_damping", "joint_compliance",
    "joint_damping", "angular_compliance", "angular_damping_constraint",
    "recovery_compliance", "recovery_damping", "radius", "margin",
    "capsule_head_inset", "capsule_tail_inset", "friction", "max_swing",
))
_VECTOR_FIELDS = frozenset(("inertia_scale", "xyz_lower", "xyz_upper", "swing_axis"))
_LIMIT_MODES = frozenset(("FREE", "XYZ", "SWING_TWIST"))
_INTERPOLATIONS = frozenset(("CONSTANT", "LINEAR", "CUBIC_HERMITE"))


class PresetError(ValueError):
    """Raised when a preset is malformed or targets an unsafe path."""


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PresetError(f"{label} must be an object")
    return value


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise PresetError(f"{label} must be a finite number")
    return float(value)


def _endpoint(value: object, label: str) -> dict[str, Any]:
    source = _mapping(value, label)
    if frozenset(source) != _ENDPOINT_FIELDS:
        raise PresetError(f"{label} must contain exactly the supported endpoint fields")
    result: dict[str, Any] = {}
    for field in _ENDPOINT_FIELDS:
        item = source[field]
        if field in _VECTOR_FIELDS:
            if not isinstance(item, list) or len(item) != 3:
                raise PresetError(f"{label}.{field} must be a three-number array")
            numbers = [_number(number, f"{label}.{field}") for number in item]
            if field == "inertia_scale" and any(number < 0.0 for number in numbers):
                raise PresetError(f"{label}.{field} must be non-negative")
            result[field] = numbers
        elif field == "angular_limit_mode":
            if item not in _LIMIT_MODES:
                raise PresetError(f"{label}.{field} is not a supported enum identifier")
            result[field] = item
        elif field == "interpolation":
            if item not in _INTERPOLATIONS:
                raise PresetError(f"{label}.{field} is not a supported enum identifier")
            result[field] = item
        else:
            number = _number(item, f"{label}.{field}")
            if field in _NON_NEGATIVE and number < 0.0:
                raise PresetError(f"{label}.{field} must be non-negative")
            if field == "restitution" and number > 1.0:
                raise PresetError(f"{label}.restitution must not exceed 1")
            if field in {"max_swing", "min_twist", "max_twist"} and abs(number) > math.pi:
                raise PresetError(f"{label}.{field} must be within [-pi, pi]")
            result[field] = number
    if any(lower > upper for lower, upper in zip(result["xyz_lower"], result["xyz_upper"])):
        raise PresetError(f"{label}.xyz_lower must not exceed xyz_upper")
    if result["min_twist"] > result["max_twist"]:
        raise PresetError(f"{label}.min_twist must not exceed max_twist")
    return result


def validate_parameter_preset(value: object) -> dict[str, Any]:
    """Return a detached canonical preset, or raise without mutating input."""
    source = _mapping(value, "preset")
    if source.get("kind") != PRESET_KIND:
        raise PresetError("preset kind must be bone_parameters")
    schema = source.get("schema_version")
    if isinstance(schema, bool) or not isinstance(schema, int):
        raise PresetError("schema_version must be an integer")
    if schema > SCHEMA_VERSION:
        raise PresetError("preset uses a newer schema version")
    if schema != SCHEMA_VERSION:
        raise PresetError("preset schema version is unsupported")
    if frozenset(source) != _ROOT_FIELDS:
        raise PresetError("preset must contain exactly the schema fields")
    name = source["name"]
    if not isinstance(name, str) or not name.strip():
        raise PresetError("preset name must be a non-empty string")
    overrides = source["overrides"]
    if not isinstance(overrides, list):
        raise PresetError("overrides must be an array")
    clean_overrides: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, override in enumerate(overrides):
        item = _mapping(override, f"overrides[{index}]")
        if frozenset(item) != {"bone_name", "parameters"}:
            raise PresetError(f"overrides[{index}] must contain bone_name and parameters")
        bone_name = item["bone_name"]
        if not isinstance(bone_name, str) or not bone_name or bone_name in names:
            raise PresetError(f"overrides[{index}].bone_name must be unique and non-empty")
        names.add(bone_name)
        clean_overrides.append({"bone_name": bone_name, "parameters": _endpoint(item["parameters"], f"overrides[{index}].parameters")})
    return {
        "kind": PRESET_KIND, "schema_version": SCHEMA_VERSION, "name": name,
        "head": _endpoint(source["head"], "head"), "tail": _endpoint(source["tail"], "tail"),
        "overrides": clean_overrides,
    }


def atomic_write_json(path: Path, value: object) -> None:
    payload = json.dumps(value, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8", newline="\n") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def save_parameter_preset(path: str | Path, preset: object) -> Path:
    target = Path(path)
    if target.suffix.lower() != ".json":
        raise PresetError("preset filename must end in .json")
    canonical = validate_parameter_preset(preset)
    atomic_write_json(target, canonical)
    return target


def load_parameter_preset(path: str | Path) -> dict[str, Any]:
    try:
        with Path(path).open("r", encoding="utf-8") as handle:
            return validate_parameter_preset(json.load(handle))
    except json.JSONDecodeError as error:
        raise PresetError(f"invalid JSON preset: {error.msg}") from error


def sanitize_preset_filename(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip()).strip("._-")
    if not cleaned or cleaned in {".", ".."}:
        raise PresetError("preset filename must contain letters or numbers")
    if cleaned.lower().endswith(".json"):
        cleaned = cleaned[:-5].rstrip("._-")
    if not cleaned:
        raise PresetError("preset filename must contain letters or numbers")
    return cleaned + ".json"


def _under_directory(path: str | Path, directory: str | Path) -> Path:
    target, root = Path(path).resolve(), Path(directory).resolve()
    if target.parent != root:
        raise PresetError("preset path must be directly inside the user preset directory")
    return target


def rename_preset(path: str | Path, name: str, directory: str | Path) -> Path:
    source = _under_directory(path, directory)
    if source.suffix.lower() != ".json":
        raise PresetError("preset filename must end in .json")
    target = _under_directory(Path(directory) / sanitize_preset_filename(name), directory)
    if not source.is_file():
        raise PresetError("preset does not exist")
    if target.exists() and target != source:
        raise PresetError("a preset with that name already exists")
    os.replace(source, target)
    return target


def delete_preset(path: str | Path, directory: str | Path) -> bool:
    target = _under_directory(path, directory)
    if target.suffix.lower() != ".json":
        raise PresetError("preset filename must end in .json")
    if not target.is_file():
        raise PresetError("preset does not exist")
    target.unlink()
    return True


def preset_directories(kind: str, create: bool = False) -> tuple[Path, ...]:
    if kind != PRESET_KIND:
        raise PresetError(f"unsupported preset kind: {kind}")
    packaged = _PACKAGE_ROOT / kind
    user: Path | None = None
    try:
        import bpy
        root = bpy.utils.user_resource("SCRIPTS", path="presets/bone_physics", create=create)
        if root:
            user = Path(root) / kind
            if create:
                user.mkdir(parents=True, exist_ok=True)
    except (ImportError, AttributeError):
        pass
    return tuple(path for path in (user, packaged) if path is not None and (create or path.exists()))


def list_presets(kind: str) -> tuple[dict[str, Any], ...]:
    entries: list[dict[str, Any]] = []
    for directory in preset_directories(kind):
        read_only = directory == _PACKAGE_ROOT / kind
        for path in sorted(directory.glob("*.json")):
            try:
                preset = load_parameter_preset(path)
            except (OSError, PresetError):
                continue
            entries.append({"name": preset["name"], "path": path, "read_only": read_only, "preset": preset})
    return tuple(entries)
