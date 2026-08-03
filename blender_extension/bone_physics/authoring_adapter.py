"""Deterministic conversion from immutable authoring records to solver specs."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable

from .authoring import ArmatureAuthoring, BoneOverride, EndpointParameters, StableIdAllocator
from .model import (
    AnalyticColliderSpec,
    AngularLimitMode,
    BoneSpec,
    ChainSpec,
    CollisionSurfaceSpec,
    CurvePoint,
    ParameterCurve,
    TargetBinding,
    TargetKind,
    WorldSpec,
)


@dataclass(frozen=True, slots=True)
class SkeletonBone:
    uuid: str
    name: str
    parent_uuid: str | None
    head: tuple[float, float, float]
    tail: tuple[float, float, float]
    rest_rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        if not self.uuid or not self.name:
            raise ValueError("skeleton bones require UUID and name")
        for value in (*self.head, *self.tail, *self.rest_rotation):
            if not math.isfinite(value):
                raise ValueError("skeleton rest transforms must be finite")


def compile_authoring(
    config: ArmatureAuthoring,
    skeleton: Iterable[SkeletonBone],
    collision_snapshot: Iterable[AnalyticColliderSpec | CollisionSurfaceSpec] = (),
) -> WorldSpec:
    """Compile concrete, stable-sorted runtime values from authoring records."""
    skeleton = tuple(skeleton)
    nodes = {node.uuid: node for node in skeleton}
    if len(nodes) != len(skeleton):
        raise ValueError("skeleton bone UUIDs must be unique")
    allocator = StableIdAllocator(config.stable_ids, config.next_stable_id)
    bones: list[BoneSpec] = []
    chains: list[ChainSpec] = []
    claimed: set[str] = set()
    for system in sorted((item for item in config.physics_systems if item.enabled), key=lambda item: item.stable_id):
        for chain_authoring in sorted(system.chains, key=lambda item: item.stable_id):
            path = tuple(chain_authoring.bone_uuids)
            if not path or any(uuid not in nodes for uuid in path):
                raise ValueError(f"chain {chain_authoring.id!r} references a missing skeleton bone")
            if claimed.intersection(path):
                raise ValueError("a physical bone may belong to only one chain")
            claimed.update(path)
            ids = {uuid: allocator.allocate("body", uuid) for uuid in path}
            sampled = _sample_chain(path, nodes, system.head, system.tail, chain_authoring.overrides)
            for index, uuid in enumerate(path):
                node = nodes[uuid]
                parent_id = ids.get(node.parent_uuid)
                values = sampled[uuid]
                binding = _binding(chain_authoring.target_id, node, index)
                bones.append(BoneSpec(
                    id=ids[uuid], name=node.name, parent_id=parent_id,
                    joint_constraint_id=allocator.allocate("joint", uuid) if parent_id is not None else None,
                    angular_constraint_id=allocator.allocate("angular", uuid) if parent_id is not None else None,
                    recovery_constraint_id=allocator.allocate("recovery", uuid),
                    rest_translation=node.head, local_head=(0.0, 0.0, 0.0),
                    local_tail=(0.0, _length(node), 0.0), rest_rotation=node.rest_rotation,
                    target_binding=binding, **values,
                ))
            chains.append(ChainSpec(id=chain_authoring.id, body_ids=tuple(ids[uuid] for uuid in path), closed=chain_authoring.closed))
    analytic = tuple(item for item in collision_snapshot if isinstance(item, AnalyticColliderSpec))
    surfaces = tuple(item for item in collision_snapshot if isinstance(item, CollisionSurfaceSpec))
    return WorldSpec(
        bones=tuple(sorted(bones, key=lambda item: item.id)),
        chains=tuple(sorted(chains, key=lambda item: item.id)),
        analytic_colliders=tuple(sorted(analytic, key=lambda item: item.id)),
        collision_surfaces=tuple(sorted(surfaces, key=lambda item: item.id)),
        gravity=config.gravity, fixed_dt=config.fixed_dt,
    )


def _length(node: SkeletonBone) -> float:
    return math.sqrt(sum((tail - head) ** 2 for head, tail in zip(node.head, node.tail)))


def _sample_chain(path, nodes, head: EndpointParameters, tail: EndpointParameters, overrides: tuple[BoneOverride, ...]):
    lengths = tuple(_length(nodes[uuid]) for uuid in path)
    total = math.fsum(lengths)
    if total <= 1.0e-6:
        raise ValueError("physical chain must have positive rest length")
    override_by_bone = {item.bone_uuid: dict(item.values) for item in overrides}
    if len(override_by_bone) != len(overrides) or any(uuid not in path for uuid in override_by_bone):
        raise ValueError("overrides must name unique bones in their chain")
    result = {}
    traversed = 0.0
    for uuid, length in zip(path, lengths):
        midpoint = (traversed + length * 0.5) / total
        values = _interpolate(head, tail, midpoint)
        values.update(_override_values(override_by_bone.get(uuid, {})))
        result[uuid] = values
        traversed += length
    return result


def _interpolate(head: EndpointParameters, tail: EndpointParameters, s: float) -> dict[str, object]:
    scalar_names = (
        "inverse_mass", "gravity_scale", "translation_inertia", "rotation_inertia", "linear_damping", "angular_damping",
        "joint_compliance", "joint_damping", "angular_compliance", "angular_damping_constraint", "recovery_compliance", "recovery_damping",
        "radius", "margin", "capsule_head_inset", "capsule_tail_inset", "friction", "restitution", "max_swing", "min_twist", "max_twist",
    )
    result = {name: _curve_value(getattr(head, name), getattr(tail, name), s, head.interpolation) for name in scalar_names}
    for name in ("inverse_inertia", "xyz_lower", "xyz_upper", "swing_axis"):
        source = head.inertia_scale if name == "inverse_inertia" else getattr(head, name)
        destination = tail.inertia_scale if name == "inverse_inertia" else getattr(tail, name)
        result[name] = tuple(_curve_value(a, b, s, head.interpolation) for a, b in zip(source, destination))
    result["angular_limit_mode"] = head.angular_limit_mode if s < 1.0 else tail.angular_limit_mode
    return result


def _curve_value(head: float, tail: float, s: float, interpolation) -> float:
    return ParameterCurve((CurvePoint(0.0, head), CurvePoint(1.0, tail)), interpolation).sample(s)


def _override_values(values: dict[str, object]) -> dict[str, object]:
    allowed = set(BoneSpec.__dataclass_fields__) - {"id", "name", "parent_id", "joint_constraint_id", "angular_constraint_id", "recovery_constraint_id", "collider_id", "rest_translation", "rest_rotation", "local_head", "local_tail", "target_binding", "initial_translation", "initial_rotation", "capsule_local_a", "capsule_local_b", "local_center_of_mass"}
    unknown = set(values) - allowed
    if unknown:
        raise ValueError(f"unsupported bone override field {min(unknown)!r}")
    return values


def _binding(target_id: str | None, node: SkeletonBone, index: int) -> TargetBinding:
    if target_id is not None:
        return TargetBinding(TargetKind.BONE, target_id)
    if index == 0 and node.parent_uuid is not None:
        return TargetBinding(TargetKind.BONE, node.parent_uuid)
    return TargetBinding(TargetKind.OWN_ANIMATION, node.name)
