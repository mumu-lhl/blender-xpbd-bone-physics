"""Operators for persisted structural cross-link lattices."""

from __future__ import annotations

import math

import bpy

from ..authoring import stable_id_range_end
from .system import _active_armature


def parse_structural_samples(text: str) -> tuple[float, ...]:
    """Parse the compact RNA representation into validated normalized samples."""
    try:
        values = tuple(float(part.strip()) for part in text.split(","))
    except ValueError as error:
        raise ValueError("samples must be comma-separated numbers") from error
    if not values or any(not math.isfinite(value) or not 0.0 <= value <= 1.0 for value in values):
        raise ValueError("samples must be finite and in [0, 1]")
    if any(left >= right for left, right in zip(values, values[1:])):
        raise ValueError("samples must be strictly increasing")
    return values


def _selected_chain_ids(system: bpy.types.PropertyGroup) -> tuple[str, ...]:
    return tuple(chain.id for chain in system.chains if chain.selected and chain.id)


def _required_constraint_count(chain_count: int, sample_count: int, diagonals: bool, closed: bool) -> int:
    pair_count = chain_count - 1 + int(closed)
    return pair_count * (sample_count + (2 * (sample_count - 1) if diagonals else 0))


def _allocate_constraint_range(settings: bpy.types.PropertyGroup, count: int) -> int:
    if count <= 0:
        raise ValueError("structural group must generate at least one constraint")
    stable_id_range_end(settings.next_stable_id, count)
    used = {
        item.stable_id
        for system in settings.physics_systems
        for item in (system, *system.chains)
        if item.stable_id
    }
    for system in settings.physics_systems:
        for group in system.structural_groups:
            samples = parse_structural_samples(group.samples)
            chain_count = len(tuple(part for part in group.chain_ids.split(",") if part.strip()))
            used.update(range(group.first_constraint_id, group.first_constraint_id + _required_constraint_count(chain_count, len(samples), group.diagonals, group.closed)))
    candidate = settings.next_stable_id
    while any(value in used for value in range(candidate, candidate + count)):
        candidate += 1
        stable_id_range_end(candidate, count)
    settings.next_stable_id = candidate + count
    return candidate


def _range_is_available(
    settings: bpy.types.PropertyGroup,
    current_group: bpy.types.PropertyGroup,
    first_constraint_id: int,
    count: int,
) -> bool:
    end = first_constraint_id + count
    for system in settings.physics_systems:
        for group in system.structural_groups:
            if group == current_group:
                continue
            samples = parse_structural_samples(group.samples)
            chain_count = len(tuple(part for part in group.chain_ids.split(",") if part.strip()))
            other_end = group.first_constraint_id + _required_constraint_count(
                chain_count, len(samples), group.diagonals, group.closed
            )
            if first_constraint_id < other_end and group.first_constraint_id < end:
                return False
    try:
        stable_id_range_end(first_constraint_id, count)
    except ValueError:
        return False
    return True


def _validate_topology(chain_ids: tuple[str, ...], samples: tuple[float, ...], diagonals: bool, closed: bool, compliance: float) -> None:
    if len(chain_ids) < 2:
        raise ValueError("select at least two chains")
    if closed and len(chain_ids) < 3:
        raise ValueError("closed topology requires at least three chains")
    if len(set(chain_ids)) != len(chain_ids):
        raise ValueError("structural group chains must be unique")
    if compliance == 0.0 and (closed or diagonals):
        raise ValueError("cycle-forming structural links require nonzero compliance")
    if not math.isfinite(compliance) or compliance < 0.0:
        raise ValueError("compliance must be finite and nonnegative")
    del samples


class BONE_PHYSICS_OT_structural_add(bpy.types.Operator):
    bl_idname = "bone_physics.structural_add"
    bl_label = "Add Structural Constraints"
    bl_options = {"REGISTER", "UNDO"}

    samples: bpy.props.StringProperty(name="Samples", default="0.0, 1.0")
    diagonals: bpy.props.BoolProperty(name="Diagonals", default=False)
    closed: bpy.props.BoolProperty(name="Closed", default=False)
    compliance: bpy.props.FloatProperty(name="Compliance", default=1.0e-5, min=0.0)
    damping: bpy.props.FloatProperty(name="Damping", default=0.0, min=0.0)

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and bool(armature.bone_physics.physics_systems)

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        try:
            parsed_samples = parse_structural_samples(self.samples)
            chain_ids = _selected_chain_ids(system)
            _validate_topology(chain_ids, parsed_samples, self.diagonals, self.closed, self.compliance)
            group = system.structural_groups.add()
            group.first_constraint_id = _allocate_constraint_range(
                settings, _required_constraint_count(len(chain_ids), len(parsed_samples), self.diagonals, self.closed)
            )
            group.chain_ids = ",".join(chain_ids)
            group.samples = ",".join(format(value, ".17g") for value in parsed_samples)
            group.diagonals = self.diagonals
            group.closed = self.closed
            group.compliance = self.compliance
            group.damping = self.damping
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        system.active_structural_group_index = len(system.structural_groups) - 1
        return {"FINISHED"}


class BONE_PHYSICS_OT_structural_remove(bpy.types.Operator):
    bl_idname = "bone_physics.structural_remove"
    bl_label = "Remove Structural Constraints"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        armature = _active_armature(context)
        return armature is not None and bool(armature.bone_physics.physics_systems) and bool(armature.bone_physics.physics_systems[min(armature.bone_physics.active_system_index, len(armature.bone_physics.physics_systems) - 1)].structural_groups)

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        system.structural_groups.remove(min(system.active_structural_group_index, len(system.structural_groups) - 1))
        system.active_structural_group_index = min(system.active_structural_group_index, max(0, len(system.structural_groups) - 1))
        return {"FINISHED"}


class BONE_PHYSICS_OT_structural_edit(bpy.types.Operator):
    bl_idname = "bone_physics.structural_edit"
    bl_label = "Update Structural Constraints"
    bl_options = {"REGISTER", "UNDO"}

    samples: bpy.props.StringProperty(name="Samples", default="0.0, 1.0")
    diagonals: bpy.props.BoolProperty(name="Diagonals", default=False)
    closed: bpy.props.BoolProperty(name="Closed", default=False)
    compliance: bpy.props.FloatProperty(name="Compliance", default=1.0e-5, min=0.0)
    damping: bpy.props.FloatProperty(name="Damping", default=0.0, min=0.0)

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return BONE_PHYSICS_OT_structural_remove.poll(context)

    def invoke(self, context: bpy.types.Context, _event) -> set[str]:
        settings = _active_armature(context).bone_physics
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        group = system.structural_groups[min(system.active_structural_group_index, len(system.structural_groups) - 1)]
        self.samples = group.samples
        self.diagonals = group.diagonals
        self.closed = group.closed
        self.compliance = group.compliance
        self.damping = group.damping
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context: bpy.types.Context) -> set[str]:
        settings = _active_armature(context).bone_physics
        system = settings.physics_systems[min(settings.active_system_index, len(settings.physics_systems) - 1)]
        group = system.structural_groups[min(system.active_structural_group_index, len(system.structural_groups) - 1)]
        try:
            parsed_samples = parse_structural_samples(self.samples)
            chain_ids = _selected_chain_ids(system)
            _validate_topology(chain_ids, parsed_samples, self.diagonals, self.closed, self.compliance)
            count = _required_constraint_count(len(chain_ids), len(parsed_samples), self.diagonals, self.closed)
            if not _range_is_available(settings, group, group.first_constraint_id, count):
                raise ValueError("edited structural group would overlap another constraint ID range")
            group.chain_ids = ",".join(chain_ids)
            group.samples = ",".join(format(value, ".17g") for value in parsed_samples)
            group.diagonals = self.diagonals
            group.closed = self.closed
            group.compliance = self.compliance
            group.damping = self.damping
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


CLASSES = (
    BONE_PHYSICS_OT_structural_add,
    BONE_PHYSICS_OT_structural_edit,
    BONE_PHYSICS_OT_structural_remove,
)
