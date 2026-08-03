"""Immutable Blender-independent authoring model for native batch compilation."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import math
from typing import Iterable

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

ZERO3: Vec3 = (0.0, 0.0, 0.0)
ONE3: Vec3 = (1.0, 1.0, 1.0)
IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)


class CurveInterpolation(IntEnum):
    CONSTANT = 0
    LINEAR = 1
    CUBIC_HERMITE = 2


class AngularLimitMode(IntEnum):
    FREE = 0
    XYZ = 1
    SWING_TWIST = 2


class TargetKind(IntEnum):
    OWN_ANIMATION = 0
    BONE = 1
    OBJECT = 2
    WORLD = 3


@dataclass(frozen=True, slots=True)
class TargetBinding:
    """Animation source for a physical body's pre-physics target pose."""

    kind: TargetKind = TargetKind.OWN_ANIMATION
    name: str | None = None
    local_anchor: Vec3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "kind", TargetKind(self.kind))
        if self.kind is TargetKind.WORLD:
            if self.name is not None:
                raise ValueError("world target binding must not name a source")
        elif self.kind in (TargetKind.BONE, TargetKind.OBJECT) and not self.name:
            raise ValueError("bone and object target bindings require a source name")
        object.__setattr__(self, "local_anchor", _vec3("target local anchor", self.local_anchor))


class ChainField(IntEnum):
    INVERSE_MASS = 0
    INERTIA_SCALE_X = 1
    INERTIA_SCALE_Y = 2
    INERTIA_SCALE_Z = 3
    GRAVITY_SCALE = 4
    TRANSLATION_INERTIA = 5
    ROTATION_INERTIA = 6
    LINEAR_DAMPING = 7
    ANGULAR_DAMPING = 8
    JOINT_COMPLIANCE = 9
    JOINT_DAMPING = 10
    ANGULAR_COMPLIANCE = 11
    ANGULAR_CONSTRAINT_DAMPING = 12
    RECOVERY_COMPLIANCE = 13
    RECOVERY_DAMPING = 14
    RADIUS = 15
    MARGIN = 16
    FRICTION = 17
    RESTITUTION = 18
    XYZ_LOWER_X = 19
    XYZ_LOWER_Y = 20
    XYZ_LOWER_Z = 21
    XYZ_UPPER_X = 22
    XYZ_UPPER_Y = 23
    XYZ_UPPER_Z = 24
    SWING_AXIS_X = 25
    SWING_AXIS_Y = 26
    SWING_AXIS_Z = 27
    MAX_SWING = 28
    MIN_TWIST = 29
    MAX_TWIST = 30
    CAPSULE_HEAD_INSET = 31
    CAPSULE_TAIL_INSET = 32


class AnalyticShape(IntEnum):
    SPHERE = 0
    CAPSULE = 1
    ROUNDED_BOX = 2


@dataclass(frozen=True, slots=True)
class CurvePoint:
    s: float
    value: float
    tangent: float = 0.0

    def __post_init__(self) -> None:
        _finite_values("curve point", (self.s, self.value, self.tangent))
        if not 0.0 <= self.s <= 1.0:
            raise ValueError(f"curve point coordinate must be in [0, 1], received {self.s}")


@dataclass(frozen=True, slots=True)
class ParameterCurve:
    points: tuple[CurvePoint, ...]
    interpolation: CurveInterpolation = CurveInterpolation.LINEAR

    def __post_init__(self) -> None:
        object.__setattr__(self, "points", tuple(self.points))
        object.__setattr__(self, "interpolation", CurveInterpolation(self.interpolation))
        if not self.points:
            raise ValueError("parameter curve requires at least one point")
        if any(first.s >= second.s for first, second in zip(self.points, self.points[1:])):
            raise ValueError("parameter curve coordinates must be strictly increasing")

    def sample(self, s: float) -> float:
        if not math.isfinite(s) or not 0.0 <= s <= 1.0:
            raise ValueError(f"curve sample must be finite and in [0, 1], received {s}")
        if s <= self.points[0].s:
            return self.points[0].value
        if s >= self.points[-1].s:
            return self.points[-1].value
        for left, right in zip(self.points, self.points[1:]):
            if s <= right.s:
                span = right.s - left.s
                t = (s - left.s) / span
                if self.interpolation is CurveInterpolation.CONSTANT:
                    return left.value
                if self.interpolation is CurveInterpolation.LINEAR:
                    return left.value + (right.value - left.value) * t
                t2 = t * t
                t3 = t2 * t
                return (
                    (2.0 * t3 - 3.0 * t2 + 1.0) * left.value
                    + (t3 - 2.0 * t2 + t) * span * left.tangent
                    + (-2.0 * t3 + 3.0 * t2) * right.value
                    + (t3 - t2) * span * right.tangent
                )
        raise AssertionError("validated curve interval lookup must succeed")


@dataclass(frozen=True, slots=True)
class ChainFieldCurve:
    field: ChainField
    curve: ParameterCurve

    def __post_init__(self) -> None:
        object.__setattr__(self, "field", ChainField(self.field))
        if not isinstance(self.curve, ParameterCurve):
            raise ValueError("chain field curve must use a ParameterCurve")


@dataclass(frozen=True, slots=True)
class BoneSpec:
    id: int
    name: str
    local_head: Vec3
    local_tail: Vec3
    parent_id: int | None = None
    joint_constraint_id: int | None = None
    angular_constraint_id: int | None = None
    recovery_constraint_id: int | None = None
    collider_id: int | None = None
    rest_translation: Vec3 = ZERO3
    rest_rotation: Quat = IDENTITY_QUAT
    initial_translation: Vec3 | None = None
    initial_rotation: Quat | None = None
    local_center_of_mass: Vec3 = ZERO3
    inverse_mass: float = 1.0
    inverse_inertia: Vec3 = ONE3
    gravity_scale: float = 1.0
    translation_inertia: float = 1.0
    rotation_inertia: float = 1.0
    linear_damping: float = 0.0
    angular_damping: float = 0.0
    capsule_local_a: Vec3 | None = None
    capsule_local_b: Vec3 | None = None
    radius: float = 0.1
    margin: float = 0.01
    friction: float = 0.5
    restitution: float = 0.0
    collision_layer: int = 1
    collision_mask: int = 0xFFFFFFFF
    excluded_body_ids: tuple[int, ...] = ()
    excluded_constraint_ids: tuple[int, ...] = ()
    joint_compliance: float = 0.0
    joint_damping: float = 0.0
    angular_compliance: float = 0.0
    angular_damping_constraint: float = 0.0
    recovery_compliance: float = 0.0
    recovery_damping: float = 0.0
    xyz_lower: Vec3 = (-math.pi, -math.pi, -math.pi)
    xyz_upper: Vec3 = (math.pi, math.pi, math.pi)
    euler_order: int = 0
    angular_limit_mode: AngularLimitMode = AngularLimitMode.FREE
    swing_axis: Vec3 = (0.0, 1.0, 0.0)
    max_swing: float = math.pi
    min_twist: float = -math.pi
    max_twist: float = math.pi
    capsule_head_inset: float = 0.0
    capsule_tail_inset: float = 0.0
    target_binding: TargetBinding = field(default_factory=TargetBinding)

    def __post_init__(self) -> None:
        _id("body", self.id)
        if not self.name:
            raise ValueError("bone name must not be empty")
        _optional_id("parent body", self.parent_id)
        _optional_id("joint constraint", self.joint_constraint_id)
        _optional_id("angular constraint", self.angular_constraint_id)
        _optional_id("recovery constraint", self.recovery_constraint_id)
        if self.collider_id is None:
            object.__setattr__(self, "collider_id", self.id)
        _id("collider", self.collider_id)
        for field_name in (
            "local_head",
            "local_tail",
            "rest_translation",
            "local_center_of_mass",
            "inverse_inertia",
            "xyz_lower",
            "xyz_upper",
            "swing_axis",
        ):
            object.__setattr__(self, field_name, _vec3(field_name, getattr(self, field_name)))
        object.__setattr__(self, "rest_rotation", _quat("rest_rotation", self.rest_rotation))
        object.__setattr__(self, "angular_limit_mode", AngularLimitMode(self.angular_limit_mode))
        if not isinstance(self.target_binding, TargetBinding):
            raise ValueError("target binding must be a TargetBinding")
        initial_translation = (
            self.rest_translation
            if self.initial_translation is None
            else _vec3("initial_translation", self.initial_translation)
        )
        initial_rotation = (
            self.rest_rotation
            if self.initial_rotation is None
            else _quat("initial_rotation", self.initial_rotation)
        )
        object.__setattr__(self, "initial_translation", initial_translation)
        object.__setattr__(self, "initial_rotation", initial_rotation)
        capsule_a = self.local_head if self.capsule_local_a is None else self.capsule_local_a
        capsule_b = self.local_tail if self.capsule_local_b is None else self.capsule_local_b
        object.__setattr__(self, "capsule_local_a", _vec3("capsule_local_a", capsule_a))
        object.__setattr__(self, "capsule_local_b", _vec3("capsule_local_b", capsule_b))
        object.__setattr__(self, "excluded_body_ids", _id_tuple("excluded body", self.excluded_body_ids))
        object.__setattr__(
            self,
            "excluded_constraint_ids",
            _id_tuple("excluded constraint", self.excluded_constraint_ids),
        )

        if _distance(self.local_head, self.local_tail) <= 1.0e-6:
            raise ValueError("bone length must be greater than 1e-6")
        for field_name in (
            "inverse_mass",
            "linear_damping",
            "angular_damping",
            "radius",
            "margin",
            "friction",
            "restitution",
            "joint_compliance",
            "joint_damping",
            "angular_compliance",
            "angular_damping_constraint",
            "recovery_compliance",
            "recovery_damping",
            "max_swing",
            "capsule_head_inset",
            "capsule_tail_inset",
        ):
            _nonnegative(field_name, getattr(self, field_name))
        if self.radius == 0.0:
            raise ValueError("radius must be greater than zero")
        _finite_values("gravity_scale", (self.gravity_scale,))
        _nonnegative("translation_inertia", self.translation_inertia)
        _nonnegative("rotation_inertia", self.rotation_inertia)
        if any(value < 0.0 for value in self.inverse_inertia):
            raise ValueError("inverse inertia values must be nonnegative")
        if any(lower > upper for lower, upper in zip(self.xyz_lower, self.xyz_upper)):
            raise ValueError("XYZ lower bounds must not exceed XYZ upper bounds")
        _finite_values("twist bounds", (self.min_twist, self.max_twist))
        if self.max_swing > math.pi:
            raise ValueError("maximum swing must not exceed pi")
        if self.min_twist > self.max_twist:
            raise ValueError("minimum twist must not exceed maximum twist")
        if self.angular_limit_mode is AngularLimitMode.SWING_TWIST:
            if math.sqrt(sum(value * value for value in self.swing_axis)) <= 1.0e-6:
                raise ValueError("swing axis must be nonzero")
        if self.euler_order not in range(6):
            raise ValueError("Euler order must be an integer from 0 through 5")
        _u32("collision_layer", self.collision_layer)
        _u32("collision_mask", self.collision_mask)

    @property
    def rest_length(self) -> float:
        return _distance(self.local_head, self.local_tail)


@dataclass(frozen=True, slots=True)
class ChainSpec:
    id: str
    body_ids: tuple[int, ...]
    closed: bool = False
    gravity_scale_curve: ParameterCurve | None = None
    linear_damping_curve: ParameterCurve | None = None
    angular_damping_curve: ParameterCurve | None = None
    radius_curve: ParameterCurve | None = None
    field_curves: tuple[ChainFieldCurve, ...] = ()
    attachment_constraint_id: int | None = None
    attachment_local_anchor: Vec3 | None = None
    attachment_target_local_anchor: Vec3 | None = None

    def __post_init__(self) -> None:
        if not self.id:
            raise ValueError("chain ID must not be empty")
        object.__setattr__(self, "body_ids", _id_tuple("chain body", self.body_ids))
        if not self.body_ids:
            raise ValueError("chain requires at least one body")
        _reject_duplicates("chain body", self.body_ids)
        field_curves = tuple(self.field_curves)
        legacy_curves = (
            (ChainField.GRAVITY_SCALE, self.gravity_scale_curve),
            (ChainField.LINEAR_DAMPING, self.linear_damping_curve),
            (ChainField.ANGULAR_DAMPING, self.angular_damping_curve),
            (ChainField.RADIUS, self.radius_curve),
        )
        field_curves += tuple(
            ChainFieldCurve(chain_field, curve)
            for chain_field, curve in legacy_curves
            if curve is not None
        )
        seen: set[ChainField] = set()
        for item in field_curves:
            if not isinstance(item, ChainFieldCurve):
                raise ValueError("chain field curves must contain ChainFieldCurve values")
            if item.field in seen:
                raise ValueError(f"duplicate chain field {item.field.name}")
            seen.add(item.field)
        object.__setattr__(self, "field_curves", field_curves)
        _optional_id("attachment constraint", self.attachment_constraint_id)
        if self.attachment_local_anchor is not None:
            object.__setattr__(
                self,
                "attachment_local_anchor",
                _vec3("attachment local anchor", self.attachment_local_anchor),
            )
        if self.attachment_target_local_anchor is not None:
            object.__setattr__(
                self,
                "attachment_target_local_anchor",
                _vec3("attachment target local anchor", self.attachment_target_local_anchor),
            )


@dataclass(frozen=True, slots=True)
class CrossLinkSpec:
    first_constraint_id: int
    chain_ids: tuple[str, ...]
    samples: tuple[float, ...]
    diagonals: bool = False
    closed: bool = False
    compliance: float = 1.0e-5
    damping: float = 0.0
    compliance_curve: ParameterCurve | None = None

    def __post_init__(self) -> None:
        _id("constraint", self.first_constraint_id)
        object.__setattr__(self, "chain_ids", tuple(self.chain_ids))
        object.__setattr__(self, "samples", tuple(float(value) for value in self.samples))
        if len(self.chain_ids) < 2 or (self.closed and len(self.chain_ids) < 3):
            raise ValueError("cross-link lattice requires enough chains for its topology")
        if len(set(self.chain_ids)) != len(self.chain_ids):
            raise ValueError("cross-link chain IDs must be unique")
        if not self.samples:
            raise ValueError("cross-link samples must not be empty")
        _finite_values("cross-link samples", self.samples)
        if any(value < 0.0 or value > 1.0 for value in self.samples):
            raise ValueError("cross-link samples must be in [0, 1]")
        if any(first >= second for first, second in zip(self.samples, self.samples[1:])):
            raise ValueError("cross-link samples must be strictly increasing")
        _nonnegative("cross-link compliance", self.compliance)
        _nonnegative("cross-link damping", self.damping)
        if self.closed and self.compliance == 0.0 and self.compliance_curve is None:
            raise ValueError("closed cross-link cycles require finite nonzero compliance")


@dataclass(frozen=True, slots=True)
class AnalyticColliderSpec:
    id: int
    shape: AnalyticShape
    translation: Vec3 = ZERO3
    rotation: Quat = IDENTITY_QUAT
    linear_velocity: Vec3 = ZERO3
    radius: float = 0.0
    local_a: Vec3 = ZERO3
    local_b: Vec3 = ZERO3
    half_extents: Vec3 = ZERO3
    enabled: bool = True
    source_object: str | None = None

    def __post_init__(self) -> None:
        _id("analytic collider", self.id)
        object.__setattr__(self, "shape", AnalyticShape(self.shape))
        for field_name in ("translation", "linear_velocity", "local_a", "local_b", "half_extents"):
            object.__setattr__(self, field_name, _vec3(field_name, getattr(self, field_name)))
        object.__setattr__(self, "rotation", _quat("rotation", self.rotation))
        _nonnegative("radius", self.radius)
        if any(value < 0.0 for value in self.half_extents):
            raise ValueError("half extents must be nonnegative")
        if self.source_object == "":
            raise ValueError("analytic collider source object must not be empty")


@dataclass(frozen=True, slots=True)
class CollisionSurfaceSpec:
    id: int
    object_name: str
    default_region_id: int = 1
    region_attribute: str = "bone_physics_region"
    excluded_materials: tuple[str, ...] = ()
    excluded_region_ids: tuple[int, ...] = ()
    body_region_ignores: tuple[tuple[int, tuple[int, ...]], ...] = ()
    upstream_weight_merges: tuple[tuple[str, str], ...] = ()
    thickness: float = 0.0
    simplify_max_error: float = 0.0
    closed_hint: bool = False

    def __post_init__(self) -> None:
        _id("collision mesh", self.id)
        if not self.object_name:
            raise ValueError("collision surface object name must not be empty")
        _id("default region", self.default_region_id)
        if not self.region_attribute:
            raise ValueError("collision region attribute must not be empty")
        excluded_materials = tuple(self.excluded_materials)
        if any(not material for material in excluded_materials):
            raise ValueError("excluded material names must not be empty")
        if len(set(excluded_materials)) != len(excluded_materials):
            raise ValueError("duplicate excluded material")
        object.__setattr__(self, "excluded_materials", excluded_materials)
        object.__setattr__(
            self,
            "excluded_region_ids",
            _id_tuple("excluded region", self.excluded_region_ids),
        )

        ignores = tuple(
            (body_id, _id_tuple("ignored region", regions))
            for body_id, regions in self.body_region_ignores
        )
        body_ids = tuple(body_id for body_id, _regions in ignores)
        for body_id in body_ids:
            _id("ignored body", body_id)
        if len(set(body_ids)) != len(body_ids):
            raise ValueError("duplicate body-region ignore")
        object.__setattr__(self, "body_region_ignores", ignores)

        merges = tuple(tuple(pair) for pair in self.upstream_weight_merges)
        if any(
            len(pair) != 2 or not pair[0] or not pair[1] or pair[0] == pair[1]
            for pair in merges
        ):
            raise ValueError("upstream weight merges require distinct nonempty bone names")
        physical = tuple(pair[0] for pair in merges)
        if len(set(physical)) != len(physical):
            raise ValueError("duplicate physical weight merge")
        object.__setattr__(self, "upstream_weight_merges", merges)
        _nonnegative("collision thickness", self.thickness)
        _nonnegative("simplification maximum error", self.simplify_max_error)


@dataclass(frozen=True, slots=True)
class WorldSpec:
    bones: tuple[BoneSpec, ...]
    chains: tuple[ChainSpec, ...] = ()
    cross_links: tuple[CrossLinkSpec, ...] = ()
    analytic_colliders: tuple[AnalyticColliderSpec, ...] = ()
    collision_surfaces: tuple[CollisionSurfaceSpec, ...] = ()
    gravity: Vec3 = (0.0, -9.81, 0.0)
    fixed_dt: float = 1.0 / 120.0
    api_version: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(self, "bones", tuple(self.bones))
        object.__setattr__(self, "chains", tuple(self.chains))
        object.__setattr__(self, "cross_links", tuple(self.cross_links))
        object.__setattr__(self, "analytic_colliders", tuple(self.analytic_colliders))
        object.__setattr__(self, "collision_surfaces", tuple(self.collision_surfaces))
        object.__setattr__(self, "gravity", _vec3("gravity", self.gravity))
        _finite_values("fixed_dt", (self.fixed_dt,))
        if self.fixed_dt <= 0.0:
            raise ValueError("fixed_dt must be greater than zero")
        _id("API version", self.api_version)

        body_ids = tuple(bone.id for bone in self.bones)
        _reject_duplicates("body", body_ids)
        body_id_set = set(body_ids)
        collider_ids = tuple(bone.collider_id for bone in self.bones) + tuple(
            collider.id for collider in self.analytic_colliders
        )
        _reject_duplicates("collider", collider_ids)
        _reject_duplicates(
            "collision mesh",
            (surface.id for surface in self.collision_surfaces),
        )
        constraint_ids = tuple(
            constraint_id
            for bone in self.bones
            for constraint_id in (
                bone.joint_constraint_id,
                bone.angular_constraint_id,
                bone.recovery_constraint_id,
            )
            if constraint_id is not None
        )
        constraint_ids += tuple(
            chain.attachment_constraint_id
            for chain in self.chains
            if chain.attachment_constraint_id is not None
        )
        _reject_duplicates("constraint", constraint_ids)
        for bone in self.bones:
            if bone.parent_id is not None and bone.parent_id not in body_id_set:
                raise ValueError(f"parent body {bone.parent_id} does not exist")
            if (
                bone.parent_id is not None
                and bone.joint_constraint_id is None
                and bone.angular_constraint_id is None
            ):
                raise ValueError(
                    f"body {bone.id} with a parent requires a joint or angular constraint ID"
                )
            if bone.angular_constraint_id is not None and bone.parent_id is None:
                raise ValueError(f"body {bone.id} angular constraint requires a parent")
            for excluded in bone.excluded_body_ids:
                if excluded not in body_id_set:
                    raise ValueError(f"excluded body {excluded} does not exist")
        for surface in self.collision_surfaces:
            for body_id, _regions in surface.body_region_ignores:
                if body_id not in body_id_set:
                    raise ValueError(
                        f"collision surface {surface.object_name!r} references missing body {body_id}"
                    )

        chains = {chain.id: chain for chain in self.chains}
        if len(chains) != len(self.chains):
            raise ValueError("duplicate chain ID")
        for chain in self.chains:
            for body_id in chain.body_ids:
                if body_id not in body_id_set:
                    raise ValueError(f"chain {chain.id!r} references missing body {body_id}")
        for cross_link in self.cross_links:
            for chain_id in cross_link.chain_ids:
                if chain_id not in chains:
                    raise ValueError(f"chain {chain_id!r} does not exist")


def _vec3(name: str, values: Iterable[float]) -> Vec3:
    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    _finite_values(name, result)
    return result  # type: ignore[return-value]


def _quat(name: str, values: Iterable[float]) -> Quat:
    result = tuple(float(value) for value in values)
    if len(result) != 4:
        raise ValueError(f"{name} must contain exactly four values")
    _finite_values(name, result)
    length = math.sqrt(sum(value * value for value in result))
    if length <= 1.0e-6:
        raise ValueError(f"{name} must be a nonzero quaternion")
    return tuple(value / length for value in result)  # type: ignore[return-value]


def _distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def _finite_values(name: str, values: Iterable[float]) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} values must all be finite")


def _nonnegative(name: str, value: float) -> None:
    _finite_values(name, (value,))
    if value < 0.0:
        raise ValueError(f"{name} must be nonnegative")


def _u32(name: str, value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"{name} must be an unsigned 32-bit integer")


def _id(kind: str, value: int) -> None:
    _u32(f"{kind} ID", value)
    if value == 0:
        raise ValueError(f"{kind} ID must be greater than zero")


def _optional_id(kind: str, value: int | None) -> None:
    if value is not None:
        _id(kind, value)


def _id_tuple(kind: str, values: Iterable[int]) -> tuple[int, ...]:
    result = tuple(values)
    for value in result:
        _id(kind, value)
    _reject_duplicates(kind, result)
    return result


def _reject_duplicates(kind: str, values: Iterable[int]) -> None:
    seen: set[int] = set()
    for value in values:
        if value in seen:
            raise ValueError(f"duplicate {kind} ID {value}")
        seen.add(value)
