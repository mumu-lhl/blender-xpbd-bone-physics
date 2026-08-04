"""Portable, validated JSON presets for Bone Physics parameters.

The schema and file operations in this module are intentionally Blender-free.
Only :func:`preset_directories` consults Blender's user-resource paths when
Blender is available.
"""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from enum import Enum
import hashlib
import json
import math
import os
import re
import struct
from pathlib import Path
from typing import Any, Mapping

from .model import PI_EPSILON


PRESET_KIND = "bone_parameters"
SCHEMA_VERSION = 1
EXACT_ARMATURE_PRESET_KIND = "exact_armature"
EXACT_ARMATURE_SCHEMA_VERSION = 1
_PACKAGE_ROOT = Path(__file__).with_name("presets")
_RUNTIME_ONLY_FIELDS = frozenset((
    "velocity", "angular_velocity", "checkpoint", "checkpoints", "cache", "runtime",
    "runtime_state", "last_error", "diagnostic_items",
))
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


def _exact_value(value: object) -> Any:
    """Make authoring data JSON-only, without leaking transient simulation state."""
    if is_dataclass(value):
        value = asdict(value)
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, Mapping):
        return {
            str(key): _exact_value(item)
            for key, item in value.items()
            if str(key) not in _RUNTIME_ONLY_FIELDS
        }
    if isinstance(value, (tuple, list)):
        return [_exact_value(item) for item in value]
    if isinstance(value, (str, bool)) or value is None:
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return _number(value, "authoring value")
    raise PresetError(f"authoring value {type(value).__name__} is not JSON serializable")


def _snapshot_bones(snapshot: object) -> tuple[dict[str, Any], ...]:
    bones = snapshot.get("bones") if isinstance(snapshot, Mapping) else getattr(snapshot, "bones", snapshot)
    try:
        items = tuple(bones)
    except TypeError as error:
        raise PresetError("armature snapshot must provide bones") from error
    result = []
    for index, bone in enumerate(items):
        source = bone if isinstance(bone, Mapping) else {
            name: getattr(bone, name)
            for name in ("uuid", "name", "parent_uuid", "length", "rest_matrix", "head", "tail", "rest_rotation")
            if hasattr(bone, name)
        }
        try:
            uuid = str(source["uuid"])
            name = str(source["name"])
            parent_uuid = source.get("parent_uuid")
            parent_uuid = None if parent_uuid is None else str(parent_uuid)
            raw_length = source["length"] if "length" in source else _bone_length(source)
            length = _number(raw_length, f"bones[{index}].length")
            matrix = source.get("rest_matrix")
            if matrix is None:
                matrix = _rest_matrix(source)
            matrix = tuple(_number(value, f"bones[{index}].rest_matrix") for value in matrix)
        except (KeyError, TypeError, ValueError) as error:
            raise PresetError(f"invalid armature bone at index {index}") from error
        if not uuid or not name or len(matrix) != 16 or length < 0.0:
            raise PresetError(f"invalid armature bone at index {index}")
        result.append({"uuid": uuid, "name": name, "parent_uuid": parent_uuid,
                       "length": length, "rest_matrix": list(matrix)})
    if len({bone["uuid"] for bone in result}) != len(result):
        raise PresetError("armature snapshot bone UUIDs must be unique")
    return tuple(sorted(result, key=lambda item: item["uuid"]))


def _bone_length(source: Mapping[str, Any]) -> float:
    head, tail = source["head"], source["tail"]
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(head, tail)))


def _rest_matrix(source: Mapping[str, Any]) -> tuple[float, ...]:
    """Stable fallback for Blender-independent SkeletonBone snapshots."""
    head = tuple(source.get("head", (0.0, 0.0, 0.0)))
    rotation = tuple(source.get("rest_rotation", (0.0, 0.0, 0.0, 1.0)))
    if len(head) != 3 or len(rotation) != 4:
        raise PresetError("bone rest transform is incomplete")
    x, y, z, w = (float(value) for value in rotation)
    return (
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), float(head[0]),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), float(head[1]),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), float(head[2]),
        0.0, 0.0, 0.0, 1.0,
    )


def armature_fingerprint(snapshot: object) -> str:
    """Hash exact rest topology with explicit portable IEEE-754 encoding."""
    digest = hashlib.sha256()
    for bone in _snapshot_bones(snapshot):
        for value in (bone["uuid"], bone["name"], bone["parent_uuid"] or ""):
            encoded = value.encode("utf-8")
            digest.update(struct.pack("<I", len(encoded)))
            digest.update(encoded)
        digest.update(struct.pack("<d", bone["length"]))
        digest.update(struct.pack("<16d", *bone["rest_matrix"]))
    return digest.hexdigest()


def serialize_exact_armature(config: object, snapshot: object, name: str = "Exact Armature") -> dict[str, Any]:
    """Serialize the complete authoring bundle, never runtime state or caches."""
    if not isinstance(name, str) or not name.strip():
        raise PresetError("preset name must be a non-empty string")
    bones = _snapshot_bones(snapshot)
    return {
        "kind": EXACT_ARMATURE_PRESET_KIND,
        "schema_version": EXACT_ARMATURE_SCHEMA_VERSION,
        "name": name.strip(),
        "armature_fingerprint": armature_fingerprint(snapshot),
        "skeleton": list(bones),
        "authoring": _exact_value(config),
    }


def _exact_mapping(value: object) -> dict[str, Any]:
    source = _mapping(value, "exact armature preset")
    expected = frozenset(("kind", "schema_version", "name", "armature_fingerprint", "skeleton", "authoring"))
    if frozenset(source) != expected:
        raise PresetError("exact armature preset must contain exactly the schema fields")
    if source.get("kind") != EXACT_ARMATURE_PRESET_KIND:
        raise PresetError("preset kind must be exact_armature")
    if source.get("schema_version") != EXACT_ARMATURE_SCHEMA_VERSION:
        raise PresetError("exact armature preset schema version is unsupported")
    if not isinstance(source["name"], str) or not source["name"].strip():
        raise PresetError("preset name must be a non-empty string")
    if not isinstance(source["armature_fingerprint"], str) or not re.fullmatch(r"[0-9a-f]{64}", source["armature_fingerprint"]):
        raise PresetError("exact armature preset fingerprint is invalid")
    if not isinstance(source["skeleton"], list) or not isinstance(source["authoring"], Mapping):
        raise PresetError("exact armature preset has invalid authoring data")
    # Normalize skeleton through the same strict parser used for live snapshots.
    bones = _snapshot_bones({"bones": source["skeleton"]})
    canonical = {
        "kind": EXACT_ARMATURE_PRESET_KIND, "schema_version": EXACT_ARMATURE_SCHEMA_VERSION,
        "name": source["name"].strip(), "armature_fingerprint": source["armature_fingerprint"],
        "skeleton": list(bones), "authoring": _exact_value(source["authoring"]),
    }
    if armature_fingerprint({"bones": bones}) != canonical["armature_fingerprint"]:
        raise PresetError("exact armature preset fingerprint does not match rest topology")
    return canonical


def _validate_authoring_references(authoring: Mapping[str, Any], bones: tuple[dict[str, Any], ...]) -> None:
    known = {item["uuid"] for item in bones}
    stable_ids: set[int] = set()
    named_ids: dict[str, set[str]] = {"system": set(), "chain": set(), "resource": set()}

    def object_list(value: Any, label: str) -> list[Mapping[str, Any]]:
        if not isinstance(value, list) or any(not isinstance(item, Mapping) for item in value):
            raise PresetError(f"exact armature preset {label} must be an array of objects")
        return value

    def unique_id(value: Mapping[str, Any], label: str) -> None:
        identifier = value.get("id")
        if not isinstance(identifier, str) or not identifier:
            raise PresetError(f"exact armature preset {label} ID is invalid")
        if identifier in named_ids[label]:
            raise PresetError(f"exact armature preset {label} IDs must be unique")
        named_ids[label].add(identifier)

    systems = object_list(authoring.get("physics_systems", []), "physics systems")
    resources = object_list(authoring.get("collision_resources", []), "collision resources")
    for system in systems:
        unique_id(system, "system")
        chains = object_list(system.get("chains", []), "chains")
        for chain in chains:
            unique_id(chain, "chain")
            path = chain.get("bone_uuids")
            if not isinstance(path, list) or not path:
                raise PresetError("exact armature preset bone path must be a non-empty array")
            if any(not isinstance(item, str) or item not in known for item in path):
                raise PresetError("exact armature preset references a missing bone path")
    for resource in resources:
        unique_id(resource, "resource")

    def walk(value: Any, key: str = "") -> None:
        if isinstance(value, Mapping):
            kind = value.get("kind")
            if "stable_id" in value:
                stable_id = value["stable_id"]
                if isinstance(stable_id, bool) or not isinstance(stable_id, int) or stable_id < 1 or stable_id in stable_ids:
                    raise PresetError("exact armature preset has duplicate or invalid stable ID")
                stable_ids.add(stable_id)
            if kind == "MESH" and not isinstance(value.get("object_name"), str):
                raise PresetError("exact armature preset mesh collider object is invalid")
            binding = value.get("binding")
            if kind == "ANALYTIC" and isinstance(binding, str) and binding in {"OBJECT", "BONE"} and not value.get("object_name"):
                raise PresetError("exact armature preset analytic collider object is invalid")
            for region_key in ("excluded_regions", "ignored_body_regions"):
                if region_key not in value:
                    continue
                regions = value[region_key]
                if not isinstance(regions, list) or any(isinstance(region, bool) or not isinstance(region, int) for region in regions):
                    raise PresetError("exact armature preset collision region rule is invalid")
            for child_key, child in value.items():
                walk(child, str(child_key))
        elif isinstance(value, list):
            for child in value:
                walk(child, key)
        elif key in {"bone_uuid", "head_uuid", "tail_uuid", "parent_uuid"} and value is not None:
            if not isinstance(value, str) or value not in known:
                raise PresetError(f"exact armature preset references missing bone {value!r}")
        elif key in {"bone_uuids", "override_bone_uuids"}:
            if not isinstance(value, list) or not all(isinstance(item, str) and item in known for item in value):
                raise PresetError("exact armature preset references a missing bone path")
        elif key == "target_id" and value is not None and value != "" and not isinstance(value, str):
            raise PresetError("exact armature preset target is invalid")
    walk(authoring)


def validate_exact_armature(data: object, snapshot: object) -> dict[str, Any]:
    """Preflight an exact preset without touching live RNA or authoring data."""
    canonical = _exact_mapping(data)
    live_bones = _snapshot_bones(snapshot)
    if canonical["armature_fingerprint"] != armature_fingerprint({"bones": live_bones}):
        raise PresetError("exact armature preset rest topology does not match this armature")
    if canonical["skeleton"] != list(live_bones):
        raise PresetError("exact armature preset rest topology does not match this armature")
    _validate_authoring_references(canonical["authoring"], live_bones)
    return canonical


def deserialize_exact_armature(data: object) -> dict[str, Any]:
    """Return detached authoring data after schema validation only."""
    return _exact_mapping(data)["authoring"]


def apply_exact_armature(owner: object, data: object, snapshot: object, registry: object | None = None) -> dict[str, Any]:
    """Validate first, then replace authoring in one mutation step.

    Mapping owners are supported directly for pure tests.  Blender owners may
    expose ``replace_exact_authoring`` so RNA collection replacement stays in
    the integration layer.  A registry can build a detached replacement before
    the authoring write and swap it only after that write succeeds.
    """
    canonical = validate_exact_armature(data, snapshot)
    replacement = canonical["authoring"]
    build = getattr(registry, "build_detached", None)
    world = build(owner, replacement) if build is not None else None
    replace = getattr(owner, "replace_exact_authoring", None)
    if replace is not None:
        replace(replacement)
    elif isinstance(owner, dict):
        owner.clear()
        owner.update(_exact_value(replacement))
    else:
        raise PresetError("exact armature owner cannot replace authoring")
    swap = getattr(registry, "swap", None)
    if swap is not None:
        swap(owner, world)
    return replacement


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
                if abs(number) <= math.pi + PI_EPSILON:
                    number = math.copysign(math.pi, number)
                else:
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


def save_exact_armature_preset(path: str | Path, preset: object) -> Path:
    target = Path(path)
    if target.suffix.lower() != ".json":
        raise PresetError("preset filename must end in .json")
    atomic_write_json(target, _exact_mapping(preset))
    return target


def load_exact_armature_preset(path: str | Path) -> dict[str, Any]:
    try:
        with Path(path).open("r", encoding="utf-8") as handle:
            return _exact_mapping(json.load(handle))
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


def copy_preset(path: str | Path, directory: str | Path, name: str | None = None) -> Path:
    """Copy a validated (including packaged read-only) preset into user storage."""
    preset = load_parameter_preset(path)
    destination = Path(directory).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    display_name = name.strip() if name is not None else f"{preset['name']} Copy"
    target = _under_directory(destination / sanitize_preset_filename(display_name), destination)
    if target.exists():
        raise PresetError("a preset with that name already exists")
    preset["name"] = display_name
    return save_parameter_preset(target, preset)


def preset_directories(kind: str, create: bool = False) -> tuple[Path, ...]:
    if kind not in {PRESET_KIND, EXACT_ARMATURE_PRESET_KIND}:
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
                preset = (load_parameter_preset(path) if kind == PRESET_KIND
                          else load_exact_armature_preset(path))
            except (OSError, PresetError):
                continue
            entries.append({"name": preset["name"], "path": path, "read_only": read_only, "preset": preset})
    return tuple(entries)
