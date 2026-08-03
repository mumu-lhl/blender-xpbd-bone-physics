"""Deterministic conversion from immutable authoring records to solver specs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .authoring import ArmatureAuthoring, BoneOverride, EndpointParameters, StableIdAllocator
from .model import (
    AnalyticColliderSpec,
    BoneSpec,
    ChainBoneOverride,
    ChainField,
    ChainFieldCurve,
    ChainSpec,
    CollisionSurfaceSpec,
    CurvePoint,
    CurveInterpolation,
    ParameterCurve,
    TargetBinding,
    TargetKind,
    WorldSpec,
)


def field_curves_from_points(
    points: Iterable[object], interpolation: CurveInterpolation | str
) -> tuple[ChainFieldCurve, ...]:
    """Convert RNA-like curve point records into deterministic immutable curves."""
    mode = (
        CurveInterpolation[interpolation]
        if isinstance(interpolation, str)
        else CurveInterpolation(interpolation)
    )
    grouped: dict[ChainField, list[CurvePoint]] = {}
    for point in points:
        field = ChainField[getattr(point, "field")]
        grouped.setdefault(field, []).append(
            CurvePoint(float(point.s), float(point.value), float(point.tangent))
        )
    return tuple(
        ChainFieldCurve(
            field,
            ParameterCurve(tuple(sorted(grouped[field], key=lambda item: item.s)), mode),
        )
        for field in sorted(grouped)
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
            for index, uuid in enumerate(path):
                node = nodes[uuid]
                parent_id = ids.get(node.parent_uuid)
                binding = _binding(chain_authoring.target_binding, chain_authoring.target_id, node, index, nodes)
                bones.append(BoneSpec(
                    id=ids[uuid], name=node.name, parent_id=parent_id,
                    joint_constraint_id=allocator.allocate("joint", uuid) if parent_id is not None else None,
                    angular_constraint_id=allocator.allocate("angular", uuid) if parent_id is not None else None,
                    recovery_constraint_id=allocator.allocate("recovery", uuid),
                    rest_translation=node.head, local_head=(0.0, 0.0, 0.0),
                    local_tail=(0.0, _length(node), 0.0), rest_rotation=node.rest_rotation,
                    target_binding=binding, angular_limit_mode=system.head.angular_limit_mode,
                ))
            chains.append(ChainSpec(
                id=chain_authoring.id,
                body_ids=tuple(ids[uuid] for uuid in path),
                closed=chain_authoring.closed,
                field_curves=_field_curves(
                    system.head,
                    system.tail,
                    (*system.field_curves, *chain_authoring.field_curves),
                ),
                bone_overrides=tuple(
                    ChainBoneOverride(ids[override.bone_uuid], override.values)
                    for override in _validated_overrides(path, chain_authoring.overrides)
                ),
            ))
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


def _validated_overrides(path: tuple[str, ...], overrides: tuple[BoneOverride, ...]) -> tuple[BoneOverride, ...]:
    override_by_bone = {item.bone_uuid: dict(item.values) for item in overrides}
    if len(override_by_bone) != len(overrides) or any(uuid not in path for uuid in override_by_bone):
        raise ValueError("overrides must name unique bones in their chain")
    for values in override_by_bone.values():
        _override_values(values)
    return tuple(sorted(overrides, key=lambda item: item.bone_uuid))


def _field_curves(
    head: EndpointParameters,
    tail: EndpointParameters,
    custom: tuple[ChainFieldCurve, ...] = (),
) -> tuple[ChainFieldCurve, ...]:
    fields = (
        (ChainField.INVERSE_MASS, head.inverse_mass, tail.inverse_mass),
        (ChainField.INERTIA_SCALE_X, head.inertia_scale[0], tail.inertia_scale[0]),
        (ChainField.INERTIA_SCALE_Y, head.inertia_scale[1], tail.inertia_scale[1]),
        (ChainField.INERTIA_SCALE_Z, head.inertia_scale[2], tail.inertia_scale[2]),
        (ChainField.GRAVITY_SCALE, head.gravity_scale, tail.gravity_scale),
        (ChainField.TRANSLATION_INERTIA, head.translation_inertia, tail.translation_inertia),
        (ChainField.ROTATION_INERTIA, head.rotation_inertia, tail.rotation_inertia),
        (ChainField.LINEAR_DAMPING, head.linear_damping, tail.linear_damping),
        (ChainField.ANGULAR_DAMPING, head.angular_damping, tail.angular_damping),
        (ChainField.JOINT_COMPLIANCE, head.joint_compliance, tail.joint_compliance),
        (ChainField.JOINT_DAMPING, head.joint_damping, tail.joint_damping),
        (ChainField.ANGULAR_COMPLIANCE, head.angular_compliance, tail.angular_compliance),
        (ChainField.ANGULAR_CONSTRAINT_DAMPING, head.angular_damping_constraint, tail.angular_damping_constraint),
        (ChainField.RECOVERY_COMPLIANCE, head.recovery_compliance, tail.recovery_compliance),
        (ChainField.RECOVERY_DAMPING, head.recovery_damping, tail.recovery_damping),
        (ChainField.RADIUS, head.radius, tail.radius),
        (ChainField.MARGIN, head.margin, tail.margin),
        (ChainField.FRICTION, head.friction, tail.friction),
        (ChainField.RESTITUTION, head.restitution, tail.restitution),
        (ChainField.XYZ_LOWER_X, head.xyz_lower[0], tail.xyz_lower[0]),
        (ChainField.XYZ_LOWER_Y, head.xyz_lower[1], tail.xyz_lower[1]),
        (ChainField.XYZ_LOWER_Z, head.xyz_lower[2], tail.xyz_lower[2]),
        (ChainField.XYZ_UPPER_X, head.xyz_upper[0], tail.xyz_upper[0]),
        (ChainField.XYZ_UPPER_Y, head.xyz_upper[1], tail.xyz_upper[1]),
        (ChainField.XYZ_UPPER_Z, head.xyz_upper[2], tail.xyz_upper[2]),
        (ChainField.SWING_AXIS_X, head.swing_axis[0], tail.swing_axis[0]),
        (ChainField.SWING_AXIS_Y, head.swing_axis[1], tail.swing_axis[1]),
        (ChainField.SWING_AXIS_Z, head.swing_axis[2], tail.swing_axis[2]),
        (ChainField.MAX_SWING, head.max_swing, tail.max_swing),
        (ChainField.MIN_TWIST, head.min_twist, tail.min_twist),
        (ChainField.MAX_TWIST, head.max_twist, tail.max_twist),
        (ChainField.CAPSULE_HEAD_INSET, head.capsule_head_inset, tail.capsule_head_inset),
        (ChainField.CAPSULE_TAIL_INSET, head.capsule_tail_inset, tail.capsule_tail_inset),
    )
    defaults = {
        item.field: item
        for item in (
        ChainFieldCurve(
            field,
            ParameterCurve((CurvePoint(0.0, start), CurvePoint(1.0, end)), head.interpolation),
        )
        for field, start, end in fields
        )
    }
    for item in custom:
        if not isinstance(item, ChainFieldCurve):
            raise ValueError("field curves must use ChainFieldCurve records")
        defaults[item.field] = item
    return tuple(defaults[field] for field in ChainField)


def _override_values(values: dict[str, object]) -> dict[str, object]:
    allowed = set(BoneSpec.__dataclass_fields__) - {"id", "name", "parent_id", "joint_constraint_id", "angular_constraint_id", "recovery_constraint_id", "collider_id", "rest_translation", "rest_rotation", "local_head", "local_tail", "target_binding", "initial_translation", "initial_rotation", "capsule_local_a", "capsule_local_b", "local_center_of_mass"}
    unknown = set(values) - allowed
    if unknown:
        raise ValueError(f"unsupported bone override field {min(unknown)!r}")
    return values


def _binding(
    explicit: TargetBinding | None,
    target_id: str | None,
    node: SkeletonBone,
    index: int,
    nodes: dict[str, SkeletonBone],
) -> TargetBinding:
    if explicit is not None:
        return _resolve_bone_name(explicit, nodes)
    if target_id is not None:
        return _resolve_bone_name(TargetBinding(TargetKind.BONE, target_id), nodes)
    if index == 0 and node.parent_uuid is not None:
        return _resolve_bone_name(TargetBinding(TargetKind.BONE, node.parent_uuid), nodes)
    return TargetBinding(TargetKind.OWN_ANIMATION, node.name)


def _resolve_bone_name(binding: TargetBinding, nodes: dict[str, SkeletonBone]) -> TargetBinding:
    if binding.kind is not TargetKind.BONE:
        return binding
    source = nodes.get(binding.name)
    if source is None:
        return binding
    return TargetBinding(TargetKind.BONE, source.name, binding.local_anchor)
