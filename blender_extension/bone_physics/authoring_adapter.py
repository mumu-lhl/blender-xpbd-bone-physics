"""Deterministic conversion from immutable authoring records to solver specs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .authoring import (
    AnalyticCollisionResource,
    ArmatureAuthoring,
    BoneOverride,
    ChainAuthoring,
    CollisionCollectionAuthoring,
    CollisionPolicy,
    compile_collision_matrix,
    EndpointParameters,
    MeshCollisionResource,
    PhysicsSystemAuthoring,
    StableIdAllocator,
)
from .model import (
    AnalyticColliderSpec,
    AngularLimitMode,
    AnalyticShape,
    BoneSpec,
    ChainBoneOverride,
    ChainField,
    ChainFieldCurve,
    ChainSpec,
    CrossLinkSpec,
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


_ENDPOINT_NAMES = tuple(EndpointParameters.__dataclass_fields__)


def physics_system_from_rna(
    system: object,
    chain_bone_uuids: dict[str, tuple[str, ...]],
) -> PhysicsSystemAuthoring:
    """Snapshot one Blender RNA system into immutable compiler input."""
    head = _endpoint_from_rna(system.head)
    tail = _endpoint_from_rna(system.tail)
    system_curves = field_curves_from_points(system.curve_points, system.interpolation)
    overrides = tuple(system.overrides)
    chains = []
    for chain in system.chains:
        path = tuple(chain_bone_uuids[chain.id])
        chain_overrides = tuple(
            BoneOverride(item.bone_name, _override_values_from_rna(item.parameters))
            for item in overrides
            if item.bone_name in path
        )
        chains.append(
            ChainAuthoring(
                id=chain.id,
                stable_id=chain.stable_id,
                bone_uuids=path,
                head_uuid=chain.head_uuid,
                tail_uuid=chain.tail_uuid,
                target_id=chain.target_id or None,
                closed=chain.closed,
                overrides=chain_overrides,
                field_curves=field_curves_from_points(
                    chain.curve_points, system.interpolation
                ),
            )
        )
    return PhysicsSystemAuthoring(
        id=system.id,
        stable_id=system.stable_id,
        chains=tuple(chains),
        head=head,
        tail=tail,
        enabled=system.enabled,
        collision_policy=CollisionPolicy(
            resource_ids=_csv(getattr(system, "collision_resource_ids", "")),
            collide_with_system_ids=_csv(getattr(system, "collide_with_system_ids", "")),
            self_collision=bool(getattr(system, "self_collision", True)),
            cross_chain_collision=bool(getattr(system, "cross_chain_collision", True)),
        ),
        field_curves=system_curves,
        cross_links=tuple(
            CrossLinkSpec(
                first_constraint_id=int(group.first_constraint_id),
                chain_ids=_structural_chain_ids(group.chain_ids),
                samples=_structural_samples(group.samples),
                diagonals=bool(group.diagonals),
                closed=bool(group.closed),
                compliance=float(group.compliance),
                damping=float(group.damping),
            )
            for group in getattr(system, "structural_groups", ())
        ),
    )


def collision_resources_from_rna(resources: Iterable[object]):
    """Snapshot RNA collision resources, preserving their persisted stable IDs."""
    result = []
    for item in resources:
        kind = getattr(item, "kind")
        if kind == "ANALYTIC":
            shape = getattr(item, "shape")
            if isinstance(shape, str):
                shape = AnalyticShape[shape]
            result.append(AnalyticCollisionResource(
                id=item.id, stable_id=int(item.stable_id), shape=shape,
                binding=getattr(item, "binding"), object_name=getattr(item, "object_name") or None,
                bone_name=getattr(item, "bone_name") or None, translation=tuple(item.translation),
                rotation=tuple(item.rotation), radius=float(item.radius), local_a=tuple(item.local_a),
                local_b=tuple(item.local_b), half_extents=tuple(item.half_extents), enabled=bool(item.enabled),
            ))
        elif kind == "MESH":
            result.append(MeshCollisionResource(
                id=item.id, stable_id=int(item.stable_id), object_name=item.object_name,
                thickness=float(item.thickness), simplification=float(item.simplification),
                excluded_materials=_csv(item.excluded_materials),
                excluded_regions=tuple(int(value) for value in _csv(item.excluded_regions)),
                ignored_body_regions=tuple(int(value) for value in _csv(item.ignored_body_regions)),
                weight_merges=tuple(tuple(part.strip() for part in value.split(":", 1)) for value in _csv(item.weight_merges)),
                enabled=bool(item.enabled),
            ))
        else:
            result.append(CollisionCollectionAuthoring(item.id, int(item.stable_id), _csv(item.member_ids), item.enabled))
    return tuple(sorted(result, key=lambda item: item.stable_id))


def compile_armature_rna(scene: object, armature: object) -> WorldSpec:
    """Take a fresh RNA snapshot and compile it into concrete runtime values."""
    del scene
    paths = _chain_paths(armature)
    config = armature_authoring_from_rna(armature, paths)
    skeleton = tuple(
        SkeletonBone(
            uuid=bone.name,
            name=bone.name,
            parent_uuid=bone.parent.name if bone.parent is not None else None,
            head=tuple(bone.head_local),
            tail=tuple(bone.tail_local),
            rest_rotation=_quaternion_tuple(bone.matrix_local.to_quaternion()),
        )
        for bone in sorted(armature.data.bones, key=lambda item: item.name)
    )
    return compile_authoring(config, skeleton)


def armature_authoring_from_rna(
    armature: object, paths: dict[str, tuple[str, ...]] | None = None
) -> ArmatureAuthoring:
    """Snapshot RNA into the presentation-free authoring validator input."""
    settings = armature.bone_physics
    paths = _chain_paths(armature) if paths is None else paths
    systems = tuple(
        physics_system_from_rna(system, paths) for system in settings.physics_systems
    )
    return ArmatureAuthoring(
        id=settings.id,
        physics_systems=systems,
        collision_resources=collision_resources_from_rna(settings.collision_resources),
        next_stable_id=int(settings.next_stable_id),
        gravity=tuple(settings.gravity),
        fixed_dt=float(settings.fixed_dt),
    )


def validate_armature_rna(armature: object):
    """Return fresh authoring issues before attempting native world creation."""
    from .authoring import validate_authoring

    return validate_authoring(armature_authoring_from_rna(armature))


def _chain_paths(armature: object) -> dict[str, tuple[str, ...]]:
    pose_bones = armature.pose.bones
    paths: dict[str, tuple[str, ...]] = {}
    for system in armature.bone_physics.physics_systems:
        for chain in system.chains:
            current = pose_bones.get(chain.tail_uuid)
            names: list[str] = []
            while current is not None:
                names.append(current.name)
                if current.name == chain.head_uuid:
                    paths[chain.id] = tuple(reversed(names))
                    break
                current = current.parent
            else:
                raise ValueError(f"chain {chain.id!r} tail is not below its head")
    return paths


def _quaternion_tuple(value: object) -> tuple[float, float, float, float]:
    return (float(value.x), float(value.y), float(value.z), float(value.w))


def _csv(value: str) -> tuple[str, ...]:
    return tuple(part.strip() for part in value.split(",") if part.strip())


def _structural_chain_ids(value: str) -> tuple[str, ...]:
    return tuple(part.strip() for part in value.split(",") if part.strip())


def _structural_samples(value: str) -> tuple[float, ...]:
    return tuple(float(part.strip()) for part in value.split(",") if part.strip())


def _endpoint_from_rna(endpoint: object) -> EndpointParameters:
    values = {name: getattr(endpoint, name) for name in _ENDPOINT_NAMES}
    values["angular_limit_mode"] = AngularLimitMode[values["angular_limit_mode"]]
    values["interpolation"] = CurveInterpolation[values["interpolation"]]
    return EndpointParameters(**values)


def _override_values_from_rna(endpoint: object) -> tuple[tuple[str, object], ...]:
    values = _endpoint_from_rna(endpoint)
    result = {
        name: getattr(values, name)
        for name in _ENDPOINT_NAMES
        if name != "interpolation"
    }
    result["inverse_inertia"] = result.pop("inertia_scale")
    return tuple(result.items())


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
    for system in config.physics_systems:
        if not system.enabled:
            continue
        for cross_link in system.cross_links:
            link_count = (len(cross_link.chain_ids) - 1 + int(cross_link.closed)) * (
                len(cross_link.samples)
                + (2 * (len(cross_link.samples) - 1) if cross_link.diagonals else 0)
            )
            for offset in range(link_count):
                allocator.reserve(
                    "cross_link_constraint",
                    str(cross_link.first_constraint_id + offset),
                    cross_link.first_constraint_id + offset,
                )
    matrix = compile_collision_matrix(config.physics_systems)
    bones: list[BoneSpec] = []
    bone_system: dict[int, PhysicsSystemAuthoring] = {}
    bone_uuid_ids: dict[str, int] = {}
    bone_chain: dict[int, str] = {}
    chains: list[ChainSpec] = []
    cross_links: list[CrossLinkSpec] = []
    claimed: set[str] = set()
    physical_uuids = {
        uuid
        for system in config.physics_systems
        if system.enabled
        for chain in system.chains
        for uuid in chain.bone_uuids
    }
    for system in sorted((item for item in config.physics_systems if item.enabled), key=lambda item: item.stable_id):
        for chain_authoring in sorted(system.chains, key=lambda item: item.stable_id):
            path = tuple(chain_authoring.bone_uuids)
            if not path or any(uuid not in nodes for uuid in path):
                raise ValueError(f"chain {chain_authoring.id!r} references a missing skeleton bone")
            if claimed.intersection(path):
                raise ValueError("a physical bone may belong to only one chain")
            claimed.update(path)
            ids = {uuid: allocator.allocate("body", uuid) for uuid in path}
            root = nodes[path[0]]
            # A chain head below an unconfigured bone must follow that
            # animated parent.  Keep the parent kinematic; the native
            # attachment constraint targets the root's animation source.
            root_target = (
                chain_authoring.target_binding is not None
                or chain_authoring.target_id is not None
                or root.parent_uuid is not None
            )
            root_is_animated = (
                root_target
                and root.parent_uuid not in physical_uuids
            )
            for index, uuid in enumerate(path):
                node = nodes[uuid]
                parent_id = ids.get(node.parent_uuid)
                binding = _binding(chain_authoring.target_binding, chain_authoring.target_id, node, index, nodes)
                layer, mask = matrix[system.id]
                bone = BoneSpec(
                    id=ids[uuid], name=node.name, parent_id=parent_id,
                    joint_constraint_id=allocator.allocate("joint", uuid) if parent_id is not None else None,
                    angular_constraint_id=allocator.allocate("angular", uuid) if parent_id is not None else None,
                    recovery_constraint_id=allocator.allocate("recovery", uuid),
                    rest_translation=node.head, local_head=(0.0, 0.0, 0.0),
                    local_tail=(0.0, _length(node), 0.0), rest_rotation=node.rest_rotation,
                    target_binding=binding, angular_limit_mode=system.head.angular_limit_mode,
                    collision_layer=layer, collision_mask=mask,
                )
                bones.append(bone)
                bone_system[bone.id] = system
                bone_uuid_ids[uuid] = bone.id
                bone_chain[bone.id] = chain_authoring.id
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
                attachment_constraint_id=(
                    allocator.allocate("attachment", chain_authoring.id)
                    if root_is_animated
                    else None
                ),
            ))
        cross_links.extend(system.cross_links)
    bones = _compile_body_collision_filters(bones, bone_system, bone_uuid_ids, bone_chain)
    analytic, surfaces = _collision_resources(config, collision_snapshot, tuple(bone.id for bone in bones))
    return WorldSpec(
        bones=tuple(sorted(bones, key=lambda item: item.id)),
        chains=tuple(sorted(chains, key=lambda item: item.id)),
        cross_links=tuple(sorted(cross_links, key=lambda item: item.first_constraint_id)),
        analytic_colliders=tuple(sorted(analytic, key=lambda item: item.id)),
        collision_surfaces=tuple(sorted(surfaces, key=lambda item: item.id)),
        gravity=config.gravity, fixed_dt=config.fixed_dt,
    )


def _compile_body_collision_filters(
    bones: list[BoneSpec], systems: dict[int, PhysicsSystemAuthoring],
    uuid_ids: dict[str, int], chains: dict[int, str],
) -> list[BoneSpec]:
    """Apply direct parent and explicit authoring exclusions after system masks."""
    from dataclasses import replace

    result = []
    for bone in bones:
        policy = systems[bone.id].collision_policy
        body_ids = {uuid_ids[uuid] for uuid in policy.explicit_excluded_bone_uuids if uuid in uuid_ids}
        if not policy.cross_chain_collision:
            body_ids.update(
                other.id for other in bones
                if systems[other.id].id == systems[bone.id].id and chains[other.id] != chains[bone.id]
            )
        if bone.parent_id is not None:
            body_ids.add(bone.parent_id)
        constraint_ids = () if bone.joint_constraint_id is None else (bone.joint_constraint_id,)
        result.append(replace(
            bone,
            excluded_body_ids=tuple(sorted(body_ids)),
            excluded_constraint_ids=constraint_ids,
        ))
    return result


def _collision_resources(
    config: ArmatureAuthoring,
    snapshot: Iterable[AnalyticColliderSpec | CollisionSurfaceSpec],
    body_ids: tuple[int, ...],
) -> tuple[tuple[AnalyticColliderSpec, ...], tuple[CollisionSurfaceSpec, ...]]:
    """Resolve selected reusable resources without leaking geometry backend types."""
    raw = tuple(snapshot)
    resources = {item.id: item for item in config.collision_resources}
    if len(resources) != len(config.collision_resources):
        raise ValueError("collision resource IDs must be unique")
    wanted = {
        resource_id
        for system in config.physics_systems if system.enabled
        for resource_id in system.collision_policy.resource_ids
    }
    expanded: set[str] = set()

    def expand(resource_id: str, ancestry: tuple[str, ...] = ()) -> None:
        if resource_id in ancestry:
            raise ValueError("collision resource collections must not cycle")
        resource = resources.get(resource_id)
        if resource is None:
            raise ValueError(f"collision resource {resource_id!r} is missing")
        if not resource.enabled:
            return
        if isinstance(resource, CollisionCollectionAuthoring):
            for member_id in resource.resource_ids:
                expand(member_id, (*ancestry, resource_id))
            return
        expanded.add(resource_id)

    for resource_id in wanted:
        expand(resource_id)
    analytic = [item for item in raw if isinstance(item, AnalyticColliderSpec)]
    surfaces = [item for item in raw if isinstance(item, CollisionSurfaceSpec)]
    for resource_id in sorted(expanded):
        resource = resources.get(resource_id)
        if isinstance(resource, AnalyticCollisionResource) and resource.enabled:
            analytic.append(AnalyticColliderSpec(
                id=resource.stable_id, shape=resource.shape, translation=resource.translation,
                rotation=resource.rotation, radius=resource.radius, local_a=resource.local_a,
                local_b=resource.local_b, half_extents=resource.half_extents,
                enabled=True, source_object=resource.object_name,
                source_binding=resource.binding, source_bone=resource.bone_name,
            ))
        elif isinstance(resource, MeshCollisionResource) and resource.enabled:
            surfaces.append(CollisionSurfaceSpec(
                id=resource.stable_id, object_name=resource.object_name,
                excluded_materials=resource.excluded_materials,
                excluded_region_ids=resource.excluded_regions,
                body_region_ignores=tuple((body_id, resource.ignored_body_regions) for body_id in body_ids),
                upstream_weight_merges=resource.weight_merges, thickness=resource.thickness,
                simplify_max_error=resource.simplification,
            ))
        else:
            raise ValueError(f"collision resource {resource_id!r} has an invalid kind")
    return (
        tuple(sorted({item.id: item for item in analytic}.values(), key=lambda item: item.id)),
        tuple(sorted({item.id: item for item in surfaces}.values(), key=lambda item: item.id)),
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
