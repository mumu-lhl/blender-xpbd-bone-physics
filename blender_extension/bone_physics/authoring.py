"""Immutable, Blender-independent records for armature physics authoring.

The records intentionally retain raw authoring values.  ``validate_authoring``
turns invalid or unresolved Blender-facing data into stable, localizable issue
codes instead of raising presentation-oriented exceptions.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable

from .model import AngularLimitMode, CurveInterpolation, TargetBinding


class ValidationSeverity(str, Enum):
    ERROR = "error"
    WARNING = "warning"


@dataclass(frozen=True, slots=True)
class ValidationIssue:
    code: str
    severity: ValidationSeverity
    entity_ids: tuple[str, ...]
    arguments: tuple[tuple[str, object], ...] = ()


@dataclass(frozen=True, slots=True)
class EndpointParameters:
    """One end of a root-to-tip parameter field.

    Curves and per-bone overrides are layered by the adapter; this record only
    holds endpoint values shared by a physics system's chains.
    """

    inverse_mass: float = 1.0
    inertia_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    gravity_scale: float = 1.0
    translation_inertia: float = 1.0
    rotation_inertia: float = 1.0
    linear_damping: float = 0.0
    angular_damping: float = 0.0
    joint_compliance: float = 0.0
    joint_damping: float = 0.0
    angular_compliance: float = 0.0
    angular_damping_constraint: float = 0.0
    recovery_compliance: float = 0.0
    recovery_damping: float = 0.0
    angular_limit_mode: AngularLimitMode = AngularLimitMode.FREE
    xyz_lower: tuple[float, float, float] = (-math.pi, -math.pi, -math.pi)
    xyz_upper: tuple[float, float, float] = (math.pi, math.pi, math.pi)
    swing_axis: tuple[float, float, float] = (0.0, 1.0, 0.0)
    max_swing: float = math.pi
    min_twist: float = -math.pi
    max_twist: float = math.pi
    radius: float = 0.1
    margin: float = 0.01
    capsule_head_inset: float = 0.0
    capsule_tail_inset: float = 0.0
    friction: float = 0.5
    restitution: float = 0.0
    interpolation: CurveInterpolation = CurveInterpolation.LINEAR

    def __post_init__(self) -> None:
        object.__setattr__(self, "inertia_scale", tuple(self.inertia_scale))
        object.__setattr__(self, "xyz_lower", tuple(self.xyz_lower))
        object.__setattr__(self, "xyz_upper", tuple(self.xyz_upper))
        object.__setattr__(self, "swing_axis", tuple(self.swing_axis))
        object.__setattr__(self, "angular_limit_mode", AngularLimitMode(self.angular_limit_mode))
        object.__setattr__(self, "interpolation", CurveInterpolation(self.interpolation))


@dataclass(frozen=True, slots=True)
class ChainAuthoring:
    id: str
    stable_id: int
    bone_uuids: tuple[str, ...]
    head_uuid: str | None = None
    tail_uuid: str | None = None
    target_id: str | None = None
    target_binding: TargetBinding | None = None
    closed: bool = False
    override_bone_uuids: tuple[str, ...] = ()
    overrides: tuple["BoneOverride", ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "bone_uuids", tuple(self.bone_uuids))
        object.__setattr__(self, "override_bone_uuids", tuple(self.override_bone_uuids))
        object.__setattr__(self, "overrides", tuple(self.overrides))
        if self.target_binding is not None and not isinstance(self.target_binding, TargetBinding):
            raise ValueError("target binding must be a TargetBinding")


@dataclass(frozen=True, slots=True)
class BoneOverride:
    """Explicit per-field replacement applied after chain sampling."""

    bone_uuid: str
    values: tuple[tuple[str, object], ...]

    def __post_init__(self) -> None:
        if not self.bone_uuid:
            raise ValueError("override bone UUID must not be empty")
        values = tuple((str(name), value) for name, value in self.values)
        if not values or any(not name for name, _value in values):
            raise ValueError("bone override requires named values")
        if len({name for name, _value in values}) != len(values):
            raise ValueError("bone override fields must be unique")
        object.__setattr__(self, "values", values)


@dataclass(frozen=True, slots=True)
class CollisionPolicy:
    """Named collision choices before deterministic layer/mask compilation."""

    resource_ids: tuple[str, ...] = ()
    collide_with_system_ids: tuple[str, ...] = ()
    self_collision: bool = True
    cross_chain_collision: bool = True
    initial_penetration: bool = False
    estimated_self_collision_candidates: int = 0
    rigid_constraint_edges: tuple[tuple[str, str], ...] = ()
    compliant_constraint_edges: tuple[tuple[str, str], ...] = ()
    suspicious_weight_bone_uuids: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "resource_ids", tuple(self.resource_ids))
        object.__setattr__(self, "collide_with_system_ids", tuple(self.collide_with_system_ids))
        object.__setattr__(self, "rigid_constraint_edges", tuple(tuple(edge) for edge in self.rigid_constraint_edges))
        object.__setattr__(self, "compliant_constraint_edges", tuple(tuple(edge) for edge in self.compliant_constraint_edges))
        object.__setattr__(self, "suspicious_weight_bone_uuids", tuple(self.suspicious_weight_bone_uuids))


@dataclass(frozen=True, slots=True)
class PhysicsSystemAuthoring:
    id: str
    stable_id: int
    chains: tuple[ChainAuthoring, ...] = ()
    head: EndpointParameters = EndpointParameters()
    tail: EndpointParameters = EndpointParameters()
    collision_policy: CollisionPolicy = CollisionPolicy()
    enabled: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "chains", tuple(self.chains))


@dataclass(frozen=True, slots=True)
class ArmatureAuthoring:
    id: str
    physics_systems: tuple[PhysicsSystemAuthoring, ...] = ()
    resource_ids: tuple[str, ...] = ()
    target_ids: tuple[str, ...] = ()
    stable_ids: tuple[tuple[str, int], ...] = ()
    next_stable_id: int = 1
    gravity: tuple[float, float, float] = (0.0, -9.81, 0.0)
    fixed_dt: float = 1.0 / 120.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "physics_systems", tuple(self.physics_systems))
        object.__setattr__(self, "resource_ids", tuple(self.resource_ids))
        object.__setattr__(self, "target_ids", tuple(self.target_ids))
        object.__setattr__(self, "stable_ids", tuple((key, value) for key, value in self.stable_ids))
        object.__setattr__(self, "gravity", tuple(self.gravity))


class StableIdAllocator:
    """Allocates nonzero u32 IDs while preserving serialized assignments."""

    def __init__(self, assignments: Iterable[tuple[str, int]] = (), next_id: int = 1) -> None:
        self._assignments: dict[str, int] = {}
        self._used: set[int] = set()
        for persisted_key, stable_id in assignments:
            _validate_stable_id(stable_id)
            if not persisted_key or persisted_key in self._assignments or stable_id in self._used:
                raise ValueError("persisted stable IDs must have unique nonempty keys and values")
            self._assignments[persisted_key] = stable_id
            self._used.add(stable_id)
        _validate_stable_id(next_id)
        self._next_id = next_id

    @property
    def assignments(self) -> tuple[tuple[str, int], ...]:
        return tuple(sorted(self._assignments.items()))

    @property
    def next_id(self) -> int:
        return self._next_id

    def allocate(self, kind: str, key: str) -> int:
        if not kind or not key:
            raise ValueError("stable ID kind and key must not be empty")
        persisted_key = f"{kind}:{key}"
        existing = self._assignments.get(persisted_key)
        if existing is not None:
            return existing
        stable_id = self._next_available_id()
        self._assignments[persisted_key] = stable_id
        self._used.add(stable_id)
        self._next_id = self._following_id(stable_id)
        return stable_id

    def _next_available_id(self) -> int:
        candidate = self._next_id
        while candidate in self._used:
            candidate = self._following_id(candidate)
            if candidate == self._next_id:
                raise ValueError("stable ID space is exhausted")
        return candidate

    @staticmethod
    def _following_id(stable_id: int) -> int:
        return 1 if stable_id == 0xFFFFFFFF else stable_id + 1


def validate_authoring(config: ArmatureAuthoring) -> tuple[ValidationIssue, ...]:
    """Return deterministic, presentation-free validation diagnostics."""
    issues = list(_identity_issues(config))
    issues.extend(_chain_issues(config))
    issues.extend(_constraint_issues(config))
    issues.extend(_collision_issues(config))
    issues.extend(_scalar_issues(config))
    return tuple(sorted(issues, key=lambda issue: (issue.severity.value, issue.code, issue.entity_ids)))


def _identity_issues(config: ArmatureAuthoring) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    _duplicates(issues, "system", (system.id for system in config.physics_systems), "uuid.duplicate")
    _duplicates(
        issues,
        "resource",
        config.resource_ids,
        "uuid.duplicate",
    )
    stable_values = tuple(system.stable_id for system in config.physics_systems)
    stable_values += tuple(chain.stable_id for system in config.physics_systems for chain in system.chains)
    stable_values += tuple(stable_id for _key, stable_id in config.stable_ids)
    for stable_id in stable_values:
        if isinstance(stable_id, bool) or not isinstance(stable_id, int) or not 0 < stable_id <= 0xFFFFFFFF:
            issues.append(_issue("stable_id.invalid", ValidationSeverity.ERROR, (str(stable_id),)))
    _duplicates(issues, "stable_id", (str(stable_id) for stable_id in stable_values), "stable_id.duplicate")
    _duplicates(issues, "stable_key", (key for key, _stable_id in config.stable_ids), "uuid.duplicate")
    return issues


def _chain_issues(config: ArmatureAuthoring) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    chains = tuple(chain for system in config.physics_systems for chain in system.chains)
    _duplicates(issues, "chain", (chain.id for chain in chains), "uuid.duplicate")
    ownership: dict[str, str] = {}
    for chain in sorted(chains, key=lambda item: item.id):
        if not chain.id or not chain.bone_uuids or any(not value for value in chain.bone_uuids):
            issues.append(_issue("chain.path.invalid", ValidationSeverity.ERROR, (chain.id,)))
        _duplicates(issues, chain.id, chain.bone_uuids, "uuid.duplicate")
        for bone_uuid in chain.bone_uuids:
            previous = ownership.setdefault(bone_uuid, chain.id)
            if previous != chain.id:
                issues.append(_issue("chain.overlap", ValidationSeverity.ERROR, tuple(sorted((previous, chain.id))), (("bone_uuid", bone_uuid),)))
        if chain.target_id is not None and chain.target_id not in config.target_ids:
            issues.append(_issue("target.missing", ValidationSeverity.ERROR, (chain.id, chain.target_id)))
    duplicate_bones = tuple(
        bone_uuid
        for chain in chains
        for bone_uuid in chain.bone_uuids
    )
    _duplicates(issues, "bone", duplicate_bones, "uuid.duplicate")
    return issues


def _constraint_issues(config: ArmatureAuthoring) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    for system in config.physics_systems:
        rigid = system.collision_policy.rigid_constraint_edges
        compliant = system.collision_policy.compliant_constraint_edges
        if _has_cycle(rigid):
            issues.append(_issue("constraint.cycle.all_rigid", ValidationSeverity.ERROR, (system.id,)))
        redundant_edges = _redundant_edges(rigid, compliant)
        for left, right in redundant_edges:
            issues.append(_issue(
                "constraint.redundant",
                ValidationSeverity.WARNING,
                (system.id, left, right),
                (("edge", (left, right)),),
            ))
        if len(rigid) + len(compliant) > max(4, len(system.chains) * 4):
            issues.append(_issue("constraint.overconstrained", ValidationSeverity.WARNING, (system.id,)))
    return issues


def _collision_issues(config: ArmatureAuthoring) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    enabled = tuple(system for system in config.physics_systems if system.enabled)
    if len(enabled) > 32:
        issues.append(_issue("collision.layers.exhausted", ValidationSeverity.ERROR, tuple(sorted(system.id for system in enabled)), (("limit", 32),)))
    resources = set(config.resource_ids)
    systems = {system.id for system in config.physics_systems}
    for system in config.physics_systems:
        policy = system.collision_policy
        if not policy.resource_ids:
            issues.append(_issue("collision.external_resource.missing", ValidationSeverity.WARNING, (system.id,)))
        for resource_id in policy.resource_ids:
            if resource_id not in resources:
                issues.append(_issue("resource.missing", ValidationSeverity.ERROR, (system.id, resource_id)))
        for other_id in policy.collide_with_system_ids:
            if other_id not in systems:
                issues.append(_issue("target.missing", ValidationSeverity.ERROR, (system.id, other_id)))
        for chain in system.chains:
            if chain.bone_uuids and policy.self_collision and not policy.cross_chain_collision:
                issues.append(_issue("collision.root.unprotected", ValidationSeverity.WARNING, (chain.id,)))
        if policy.initial_penetration:
            issues.append(_issue("collision.initial_penetration", ValidationSeverity.WARNING, (system.id,)))
        if policy.estimated_self_collision_candidates > 10_000:
            issues.append(_issue("collision.candidates.excessive", ValidationSeverity.WARNING, (system.id,), (("count", policy.estimated_self_collision_candidates),)))
        for bone_uuid in sorted(set(policy.suspicious_weight_bone_uuids)):
            issues.append(_issue(
                "collision.weight.suspicious",
                ValidationSeverity.WARNING,
                (system.id, bone_uuid),
                (("bone_uuid", bone_uuid),),
            ))
    return issues


def _scalar_issues(config: ArmatureAuthoring) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    if not _finite_tuple(config.gravity, 3):
        issues.append(_issue("parameter.scalar.nonfinite", ValidationSeverity.ERROR, (config.id,), (("field", "gravity"),)))
    if not _finite(config.fixed_dt) or config.fixed_dt <= 0.0:
        issues.append(_issue("parameter.scalar.invalid", ValidationSeverity.ERROR, (config.id,), (("field", "fixed_dt"),)))
    for system in config.physics_systems:
        for endpoint_name, endpoint in (("head", system.head), ("tail", system.tail)):
            entity_ids = (system.id, endpoint_name)
            for field_name, value in _endpoint_scalars(endpoint):
                if not _finite(value):
                    issues.append(_issue("parameter.scalar.nonfinite", ValidationSeverity.ERROR, entity_ids, (("field", field_name),)))
                elif field_name != "gravity_scale" and value < 0.0:
                    issues.append(_issue("parameter.scalar.negative", ValidationSeverity.ERROR, entity_ids, (("field", field_name), ("value", value))))
            if not _finite_tuple(endpoint.inertia_scale, 3) or any(value < 0.0 for value in endpoint.inertia_scale):
                issues.append(_issue("parameter.scalar.invalid", ValidationSeverity.ERROR, entity_ids, (("field", "inertia_scale"),)))
            if not _finite_tuple(endpoint.xyz_lower, 3) or not _finite_tuple(endpoint.xyz_upper, 3):
                issues.append(_issue("parameter.scalar.nonfinite", ValidationSeverity.ERROR, entity_ids, (("field", "xyz_bounds"),)))
            elif any(lower > upper for lower, upper in zip(endpoint.xyz_lower, endpoint.xyz_upper)):
                issues.append(_issue("parameter.range.inverted", ValidationSeverity.ERROR, entity_ids, (("field", "xyz"),)))
            if not _finite(endpoint.min_twist) or not _finite(endpoint.max_twist) or endpoint.min_twist > endpoint.max_twist:
                issues.append(_issue("parameter.range.inverted", ValidationSeverity.ERROR, entity_ids, (("field", "twist"),)))
            if _finite(endpoint.radius) and endpoint.radius == 0.0:
                issues.append(_issue("parameter.scalar.zero", ValidationSeverity.ERROR, entity_ids, (("field", "radius"),)))
            if _finite(endpoint.max_swing) and endpoint.max_swing > math.pi:
                issues.append(_issue("parameter.range.exceeds_limit", ValidationSeverity.ERROR, entity_ids, (("field", "max_swing"), ("limit", math.pi))))
            if _finite(endpoint.radius) and endpoint.radius > 0.5:
                issues.append(_issue("collision.radius.suspicious", ValidationSeverity.WARNING, entity_ids, (("radius", endpoint.radius),)))
    return issues


def _endpoint_scalars(endpoint: EndpointParameters) -> tuple[tuple[str, float], ...]:
    return tuple((name, getattr(endpoint, name)) for name in (
        "inverse_mass", "gravity_scale", "translation_inertia", "rotation_inertia",
        "linear_damping", "angular_damping", "joint_compliance", "joint_damping",
        "angular_compliance", "angular_damping_constraint",
        "recovery_compliance", "recovery_damping", "max_swing", "radius", "margin",
        "capsule_head_inset", "capsule_tail_inset", "friction", "restitution",
    ))


def _issue(code: str, severity: ValidationSeverity, entity_ids: tuple[str, ...], arguments: tuple[tuple[str, object], ...] = ()) -> ValidationIssue:
    return ValidationIssue(code, severity, entity_ids, arguments)


def _duplicates(issues: list[ValidationIssue], kind: str, values: Iterable[str], code: str) -> None:
    owners: dict[str, int] = {}
    for value in values:
        if value:
            owners[value] = owners.get(value, 0) + 1
    for value, count in sorted(owners.items()):
        if count > 1:
            issues.append(_issue(code, ValidationSeverity.ERROR, (kind, value), (("count", count),)))


def _has_cycle(edges: Iterable[tuple[str, str]]) -> bool:
    graph: dict[str, set[str]] = {}
    for left, right in edges:
        if not left or not right:
            continue
        graph.setdefault(left, set()).add(right)
        graph.setdefault(right, set()).add(left)
    visited: set[str] = set()
    for root in sorted(graph):
        if root in visited:
            continue
        stack = [(root, None)]
        while stack:
            node, parent = stack.pop()
            if node in visited:
                return True
            visited.add(node)
            for neighbor in graph[node]:
                if neighbor != parent:
                    stack.append((neighbor, node))
    return False


def _redundant_edges(
    rigid: Iterable[tuple[str, str]],
    compliant: Iterable[tuple[str, str]],
) -> tuple[tuple[str, str], ...]:
    seen: set[tuple[str, str]] = set()
    redundant: set[tuple[str, str]] = set()
    for left, right in (*rigid, *compliant):
        if not left or not right:
            continue
        edge = tuple(sorted((left, right)))
        if edge in seen:
            redundant.add(edge)
        seen.add(edge)
    return tuple(sorted(redundant))


def _finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _finite_tuple(values: object, size: int) -> bool:
    return isinstance(values, tuple) and len(values) == size and all(_finite(value) for value in values)


def _validate_stable_id(value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 < value <= 0xFFFFFFFF:
        raise ValueError("stable ID must be a nonzero unsigned 32-bit integer")
