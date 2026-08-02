"""Deterministic object, face, and body ownership for collision surfaces."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable


class ObjectRole(str, Enum):
    COLLISION = "collision"
    SIMULATED = "simulated"
    EXCLUDED = "excluded"


@dataclass(frozen=True, slots=True)
class FaceOwnership:
    region: int
    material: str | None = None
    excluded: bool = False
    driven_by_body_ids: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        _positive_id("region", self.region)
        if self.material == "":
            raise ValueError("face material must not be empty")
        body_ids = tuple(self.driven_by_body_ids)
        _unique_ids("driven body", body_ids)
        object.__setattr__(self, "driven_by_body_ids", body_ids)


@dataclass(frozen=True, slots=True)
class ObjectOwnership:
    name: str
    role: ObjectRole
    faces: tuple[FaceOwnership, ...] = ()
    excluded_materials: tuple[str, ...] = ()
    body_region_ignores: tuple[tuple[int, tuple[int, ...]], ...] = ()

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("ownership object name must not be empty")
        object.__setattr__(self, "role", ObjectRole(self.role))
        object.__setattr__(self, "faces", tuple(self.faces))
        excluded_materials = tuple(self.excluded_materials)
        if any(not material for material in excluded_materials):
            raise ValueError("excluded material names must not be empty")
        if len(set(excluded_materials)) != len(excluded_materials):
            raise ValueError("duplicate excluded material")
        object.__setattr__(self, "excluded_materials", excluded_materials)

        ignores = tuple(
            (body_id, tuple(regions))
            for body_id, regions in self.body_region_ignores
        )
        body_ids = tuple(body_id for body_id, _regions in ignores)
        if len(set(body_ids)) != len(body_ids):
            raise ValueError("duplicate body-region ignore")
        for body_id, regions in ignores:
            _positive_id("body", body_id)
            _unique_ids("ignored region", regions)
        object.__setattr__(self, "body_region_ignores", ignores)


@dataclass(frozen=True, slots=True)
class OwnershipMap:
    object_roles: tuple[tuple[str, ObjectRole], ...]
    face_regions: tuple[tuple[str, int, int | None], ...]
    body_region_ignores: tuple[tuple[int, tuple[int, ...]], ...]

    def collision_objects(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, role in self.object_roles
            if role is ObjectRole.COLLISION
        )

    def region_for_face(self, object_name: str, face_index: int) -> int | None:
        for name, index, region in self.face_regions:
            if name == object_name and index == face_index:
                return region
        return None

    def body_sees_region(self, body_id: int, region: int) -> bool:
        _positive_id("body", body_id)
        _positive_id("region", region)
        for ignored_body, ignored_regions in self.body_region_ignores:
            if ignored_body == body_id:
                return region not in ignored_regions
        return True


def build_ownership_map(objects: Iterable[ObjectOwnership]) -> OwnershipMap:
    ordered = sorted(tuple(objects), key=lambda item: item.name)
    for first, second in zip(ordered, ordered[1:]):
        if first.name == second.name:
            raise ValueError(f"duplicate ownership object {first.name!r}")

    object_roles = tuple((item.name, item.role) for item in ordered)
    face_regions = []
    body_ignores: dict[int, set[int]] = {}
    for item in ordered:
        excluded_materials = set(item.excluded_materials)
        for face_index, face in enumerate(item.faces):
            included = (
                item.role is ObjectRole.COLLISION
                and not face.excluded
                and face.material not in excluded_materials
            )
            face_regions.append(
                (item.name, face_index, face.region if included else None)
            )
            if included:
                for body_id in face.driven_by_body_ids:
                    body_ignores.setdefault(body_id, set()).add(face.region)
        for body_id, regions in item.body_region_ignores:
            body_ignores.setdefault(body_id, set()).update(regions)

    return OwnershipMap(
        object_roles=object_roles,
        face_regions=tuple(face_regions),
        body_region_ignores=tuple(
            (body_id, tuple(sorted(regions)))
            for body_id, regions in sorted(body_ignores.items())
        ),
    )


def _positive_id(kind: str, value: int) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 < value <= 0xFFFFFFFF
    ):
        raise ValueError(f"{kind} ID must be an unsigned 32-bit integer greater than zero")


def _unique_ids(kind: str, values: tuple[int, ...]) -> None:
    for value in values:
        _positive_id(kind, value)
    if len(set(values)) != len(values):
        raise ValueError(f"duplicate {kind} ID")
