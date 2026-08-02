"""Compile immutable authoring specifications into Native Batch API v1 arrays."""

from __future__ import annotations

from array import array
from dataclasses import dataclass, replace
import json
import math
import struct
import sys
from typing import Mapping

from .collider_builder import (
    compile_analytic_colliders,
    compile_collision_filters,
)
from .model import BoneSpec, ChainSpec, CrossLinkSpec, ParameterCurve, Quat, Vec3, WorldSpec

BODY_COLUMNS = 46
CONSTRAINT_COLUMNS = 24


@dataclass(frozen=True, slots=True)
class ChainAnchor:
    body_id: int
    local_anchor: Vec3
    world_anchor: Vec3


@dataclass(frozen=True, slots=True)
class CompiledConstraint:
    id: int
    body_a: int
    body_b: int
    kind: int
    enabled: bool
    values: tuple[float, ...]
    chain_a: str | None = None
    chain_b: str | None = None
    compliance: float = 0.0


@dataclass(frozen=True, slots=True)
class NativeBuildBatch:
    config: dict[str, object]
    bodies: dict[str, object]
    constraints: dict[str, object]
    filters: dict[str, object]
    analytic_colliders: dict[str, object]

    def constraint_row(self, constraint_id: int) -> tuple[float, ...]:
        ids = self.constraints["ids"]
        values = self.constraints["values"]
        assert isinstance(ids, array)
        assert isinstance(values, array)
        try:
            index = ids.index(constraint_id)
        except ValueError as error:
            raise KeyError(constraint_id) from error
        start = index * CONSTRAINT_COLUMNS
        return tuple(values[start : start + CONSTRAINT_COLUMNS])

    def canonical_bytes(self) -> bytes:
        result = bytearray(b"BNPBATCH\x01")
        config = json.dumps(
            self.config,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        result.extend(struct.pack("<I", len(config)))
        result.extend(config)
        for name, batch in (
            ("bodies", self.bodies),
            ("constraints", self.constraints),
            ("filters", self.filters),
            ("analytic_colliders", self.analytic_colliders),
        ):
            encoded_name = name.encode("ascii")
            result.extend(struct.pack("<I", len(encoded_name)))
            result.extend(encoded_name)
            for key in sorted(batch):
                encoded_key = key.encode("ascii")
                result.extend(struct.pack("<I", len(encoded_key)))
                result.extend(encoded_key)
                value = batch[key]
                if isinstance(value, array):
                    result.extend(value.typecode.encode("ascii"))
                    result.extend(struct.pack("<Q", len(value)))
                    result.extend(_little_endian_bytes(value))
                elif isinstance(value, int):
                    result.extend(b"i")
                    result.extend(struct.pack("<Q", value))
                else:
                    raise TypeError(f"unsupported canonical batch value {name}.{key}")
        return bytes(result)


def chain_boundaries(chain: ChainSpec, bones: Mapping[int, BoneSpec]) -> tuple[float, ...]:
    lengths = tuple(_chain_bone(chain, body_id, bones).rest_length for body_id in chain.body_ids)
    total = math.fsum(lengths)
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError(f"chain {chain.id!r} has invalid total rest length")
    accumulated = 0.0
    result = [0.0]
    for index, length in enumerate(lengths):
        accumulated += length
        result.append(1.0 if index + 1 == len(lengths) else accumulated / total)
    return tuple(result)


def sample_chain_anchor(
    chain: ChainSpec,
    s: float,
    bones: Mapping[int, BoneSpec],
) -> ChainAnchor:
    if not math.isfinite(s) or not 0.0 <= s <= 1.0:
        raise ValueError(f"chain sample must be finite and in [0, 1], received {s}")
    boundaries = chain_boundaries(chain, bones)
    if s == 1.0:
        index = len(chain.body_ids) - 1
        fraction = 1.0
    else:
        index = next(
            index
            for index, upper in enumerate(boundaries[1:])
            if s < upper
        )
        lower = boundaries[index]
        upper = boundaries[index + 1]
        fraction = (s - lower) / (upper - lower)
    bone = _chain_bone(chain, chain.body_ids[index], bones)
    local = _lerp(bone.local_head, bone.local_tail, fraction)
    world = _add(bone.rest_translation, _rotate(bone.rest_rotation, local))
    return ChainAnchor(body_id=bone.id, local_anchor=local, world_anchor=world)


def build_cross_link_constraints(
    spec: CrossLinkSpec,
    chains: tuple[ChainSpec, ...],
    bones: Mapping[int, BoneSpec],
) -> tuple[CompiledConstraint, ...]:
    chain_lookup = {chain.id: chain for chain in chains}
    try:
        selected = tuple(chain_lookup[chain_id] for chain_id in spec.chain_ids)
    except KeyError as error:
        raise ValueError(f"chain {error.args[0]!r} does not exist") from error
    owners: dict[int, str] = {}
    for chain in selected:
        for body_id in chain.body_ids:
            if body_id in owners:
                raise ValueError(
                    f"body {body_id} belongs to multiple cross-link chains "
                    f"{owners[body_id]!r} and {chain.id!r}"
                )
            owners[body_id] = chain.id
    pair_count = len(selected) - 1 + int(spec.closed)
    links_per_pair = len(spec.samples) + (
        2 * (len(spec.samples) - 1) if spec.diagonals else 0
    )
    link_count = pair_count * links_per_pair
    if spec.first_constraint_id + link_count - 1 > 0xFFFFFFFF:
        raise ValueError("generated constraint ID range exceeds u32")
    sampled = {
        chain.id: tuple(sample_chain_anchor(chain, s, bones) for s in spec.samples)
        for chain in selected
    }
    pairs = list(zip(selected, selected[1:]))
    if spec.closed:
        pairs.append((selected[-1], selected[0]))

    candidates: list[tuple[ChainSpec, ChainSpec, ChainAnchor, ChainAnchor, float]] = []
    for chain_a, chain_b in pairs:
        anchors_a = sampled[chain_a.id]
        anchors_b = sampled[chain_b.id]
        for index, sample in enumerate(spec.samples):
            candidates.append((chain_a, chain_b, anchors_a[index], anchors_b[index], sample))
        if spec.diagonals:
            for index in range(len(spec.samples) - 1):
                midpoint = (spec.samples[index] + spec.samples[index + 1]) * 0.5
                candidates.append(
                    (chain_a, chain_b, anchors_a[index], anchors_b[index + 1], midpoint)
                )
                candidates.append(
                    (chain_a, chain_b, anchors_b[index], anchors_a[index + 1], midpoint)
                )

    result = []
    for offset, (chain_a, chain_b, anchor_a, anchor_b, sample) in enumerate(candidates):
        compliance = _sample_compliance(spec, sample)
        if compliance == 0.0 and (spec.closed or spec.diagonals):
            raise ValueError("cycle-forming cross links require nonzero compliance")
        row = [0.0] * CONSTRAINT_COLUMNS
        row[0:3] = anchor_a.local_anchor
        row[3:6] = anchor_b.local_anchor
        row[6] = _distance(anchor_a.world_anchor, anchor_b.world_anchor)
        row[8] = compliance
        row[9] = spec.damping
        result.append(
            CompiledConstraint(
                id=spec.first_constraint_id + offset,
                body_a=anchor_a.body_id,
                body_b=anchor_b.body_id,
                kind=1,
                enabled=True,
                values=tuple(row),
                chain_a=chain_a.id,
                chain_b=chain_b.id,
                compliance=compliance,
            )
        )
    return tuple(result)


def compile_world_spec(spec: WorldSpec) -> NativeBuildBatch:
    bones = _apply_chain_curves(spec)
    bone_lookup = {bone.id: bone for bone in bones}
    constraints: list[CompiledConstraint] = []
    for bone in bones:
        if bone.parent_id is not None:
            parent = bone_lookup[bone.parent_id]
            constraints.append(_anchor_constraint(parent, bone))
        if bone.angular_constraint_id is not None:
            parent = bone_lookup[bone.parent_id]
            constraints.append(_angular_constraint(parent, bone))
        if bone.recovery_constraint_id is not None:
            constraints.append(_recovery_constraint(bone))
    for cross_link in spec.cross_links:
        constraints.extend(
            build_cross_link_constraints(cross_link, spec.chains, bone_lookup)
        )
    constraints.sort(key=lambda constraint: constraint.id)
    _reject_duplicate_constraint_ids(constraints)
    known_constraint_ids = {constraint.id for constraint in constraints}
    for bone in bones:
        missing = set(bone.excluded_constraint_ids) - known_constraint_ids
        if missing:
            raise ValueError(
                f"body {bone.id} excludes missing constraint {min(missing)}"
            )

    return NativeBuildBatch(
        config={
            "api_version": spec.api_version,
            "gravity": spec.gravity,
            "fixed_dt": spec.fixed_dt,
        },
        bodies=_compile_bodies(bones),
        constraints=_compile_constraints(constraints),
        filters=compile_collision_filters(bones),
        analytic_colliders=compile_analytic_colliders(spec.analytic_colliders),
    )


def _apply_chain_curves(spec: WorldSpec) -> tuple[BoneSpec, ...]:
    replacements: dict[int, BoneSpec] = {bone.id: bone for bone in spec.bones}
    claimed: set[int] = set()
    for chain in spec.chains:
        if not any(
            (
                chain.gravity_scale_curve,
                chain.linear_damping_curve,
                chain.angular_damping_curve,
                chain.radius_curve,
            )
        ):
            continue
        overlap = claimed.intersection(chain.body_ids)
        if overlap:
            raise ValueError(f"body {min(overlap)} receives parameter curves from multiple chains")
        claimed.update(chain.body_ids)
        boundaries = chain_boundaries(chain, replacements)
        for index, body_id in enumerate(chain.body_ids):
            s = (boundaries[index] + boundaries[index + 1]) * 0.5
            bone = replacements[body_id]
            updates = {}
            for field_name, curve in (
                ("gravity_scale", chain.gravity_scale_curve),
                ("linear_damping", chain.linear_damping_curve),
                ("angular_damping", chain.angular_damping_curve),
                ("radius", chain.radius_curve),
            ):
                if curve is not None:
                    updates[field_name] = curve.sample(s)
            replacements[body_id] = replace(bone, **updates)
    return tuple(sorted(replacements.values(), key=lambda bone: bone.id))


def _compile_bodies(bones: tuple[BoneSpec, ...]) -> dict[str, object]:
    values = array("f")
    for bone in bones:
        row = [0.0] * BODY_COLUMNS
        row[0:3] = bone.rest_translation
        row[3:7] = bone.rest_rotation
        row[7:10] = bone.local_center_of_mass
        row[10:13] = bone.local_head
        row[13:16] = bone.local_tail
        row[16] = bone.inverse_mass
        row[17:20] = bone.inverse_inertia
        row[20] = bone.gravity_scale
        row[21] = bone.linear_damping
        row[22] = bone.angular_damping
        row[23:26] = bone.initial_translation
        row[26:30] = bone.initial_rotation
        row[30:36] = (0.0,) * 6
        row[36:39] = bone.capsule_local_a
        row[39:42] = bone.capsule_local_b
        row[42] = bone.radius
        row[43] = bone.margin
        row[44] = bone.friction
        row[45] = bone.restitution
        values.extend(row)
    return {
        "body_ids": array("I", (bone.id for bone in bones)),
        "collider_ids": array("I", (bone.collider_id for bone in bones)),
        "values": values,
        "columns": BODY_COLUMNS,
    }


def _compile_constraints(
    constraints: list[CompiledConstraint],
) -> dict[str, object]:
    values = array("f")
    body_ids = array("I")
    for constraint in constraints:
        body_ids.extend((constraint.body_a, constraint.body_b))
        values.extend(constraint.values)
    return {
        "ids": array("I", (constraint.id for constraint in constraints)),
        "body_ids": body_ids,
        "kinds": array("I", (constraint.kind for constraint in constraints)),
        "enabled": array("I", (int(constraint.enabled) for constraint in constraints)),
        "values": values,
        "columns": CONSTRAINT_COLUMNS,
    }


def _anchor_constraint(parent: BoneSpec, child: BoneSpec) -> CompiledConstraint:
    row = [0.0] * CONSTRAINT_COLUMNS
    row[0:3] = parent.local_tail
    row[3:6] = child.local_head
    row[8] = child.joint_compliance
    row[9] = child.joint_damping
    return CompiledConstraint(
        id=child.joint_constraint_id,
        body_a=parent.id,
        body_b=child.id,
        kind=0,
        enabled=True,
        values=tuple(row),
        compliance=child.joint_compliance,
    )


def _angular_constraint(parent: BoneSpec, child: BoneSpec) -> CompiledConstraint:
    row = [0.0] * CONSTRAINT_COLUMNS
    rest_relative = _quat_multiply(_quat_conjugate(parent.rest_rotation), child.rest_rotation)
    row[8] = child.angular_compliance
    row[9] = child.angular_damping_constraint
    row[10:14] = rest_relative
    row[14:17] = child.xyz_lower
    row[17:20] = child.xyz_upper
    row[20] = float(child.euler_order)
    return CompiledConstraint(
        id=child.angular_constraint_id,
        body_a=parent.id,
        body_b=child.id,
        kind=5,
        enabled=True,
        values=tuple(row),
        compliance=child.angular_compliance,
    )


def _recovery_constraint(bone: BoneSpec) -> CompiledConstraint:
    row = [0.0] * CONSTRAINT_COLUMNS
    row[8] = bone.recovery_compliance
    row[9] = bone.recovery_damping
    return CompiledConstraint(
        id=bone.recovery_constraint_id,
        body_a=bone.id,
        body_b=0,
        kind=4,
        enabled=True,
        values=tuple(row),
        compliance=bone.recovery_compliance,
    )


def _sample_compliance(spec: CrossLinkSpec, s: float) -> float:
    curve: ParameterCurve | None = spec.compliance_curve
    value = spec.compliance if curve is None else curve.sample(s)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("cross-link compliance curve must sample finite nonnegative values")
    return value


def _chain_bone(
    chain: ChainSpec,
    body_id: int,
    bones: Mapping[int, BoneSpec],
) -> BoneSpec:
    try:
        return bones[body_id]
    except KeyError as error:
        raise ValueError(f"chain {chain.id!r} references missing body {body_id}") from error


def _reject_duplicate_constraint_ids(constraints: list[CompiledConstraint]) -> None:
    for first, second in zip(constraints, constraints[1:]):
        if first.id == second.id:
            raise ValueError(f"duplicate constraint ID {first.id}")


def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return tuple(left + (right - left) * t for left, right in zip(a, b))  # type: ignore[return-value]


def _add(a: Vec3, b: Vec3) -> Vec3:
    return tuple(left + right for left, right in zip(a, b))  # type: ignore[return-value]


def _distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def _quat_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def _quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q: Quat, value: Vec3) -> Vec3:
    rotated = _quat_multiply(
        _quat_multiply(q, (value[0], value[1], value[2], 0.0)),
        _quat_conjugate(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def _little_endian_bytes(values: array) -> bytes:
    copy = array(values.typecode, values)
    if sys.byteorder != "little" and copy.itemsize > 1:
        copy.byteswap()
    return copy.tobytes()
