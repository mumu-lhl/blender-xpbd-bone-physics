"""Installed-extension character authoring flows; executed by the bounded harness."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension import bone_physics
from blender_extension.bone_physics.authoring_adapter import compile_armature_rna
from blender_extension.bone_physics.operators.preset import (
    _ExactOwner,
    _exact_bundle,
    _exact_snapshot,
)
from blender_extension.bone_physics.parameter_builder import compile_world_spec
from blender_extension.bone_physics.preset_io import (
    apply_exact_armature,
    serialize_exact_armature,
)
from blender_extension.bone_physics.runtime import NativeRuntimeBridge


def _armature() -> bpy.types.Object:
    data = bpy.data.armatures.new("CharacterWorkflowArmature")
    armature = bpy.data.objects.new("CharacterWorkflowArmature", data)
    bpy.context.scene.collection.objects.link(armature)
    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    for index, name in enumerate((
        "hair.L", "hair.R", "ear.L", "ear.R", "ponytail",
        "skirt.0", "skirt.1", "skirt.2", "skirt.3", "accessory",
    )):
        root = data.edit_bones.new(name)
        root.head = (float(index), 0.0, 0.0)
        root.tail = (float(index), 1.0, 0.0)
        tip = data.edit_bones.new(f"{name}.tip")
        tip.head, tip.tail, tip.parent = root.tail, (float(index), 2.0, 0.0), root
    bpy.ops.object.mode_set(mode="POSE")
    return armature


def _select(armature: bpy.types.Object, names: tuple[str, ...]) -> None:
    for bone in armature.pose.bones:
        bone.select = bone.name in names
    armature.data.bones.active = armature.data.bones[names[0]]


def _system(armature: bpy.types.Object, template: str, name: str, roots: tuple[str, ...]):
    _select(armature, roots)
    assert bpy.ops.bone_physics.system_add(template=template, name=name) == {"FINISHED"}
    assert bpy.ops.bone_physics.chain_add_selected() == {"FINISHED"}
    return armature.bone_physics.physics_systems[-1]


def _compiled(armature: bpy.types.Object):
    spec = compile_armature_rna(bpy.context.scene, armature)
    batch = compile_world_spec(spec)
    assert len(batch.bodies["body_ids"]) == len(spec.bones)
    assert len(batch.filters["layers"]) == len(spec.bones)
    return spec, batch


def configure_hair(armature: bpy.types.Object, *, multiple_roots: bool, disconnected_children: bool) -> bool:
    system = _system(armature, "LOOSE_HAIR", "Hair", ("hair.L", "hair.R"))
    spec, batch = _compiled(armature)
    return multiple_roots and disconnected_children and len(system.chains) == 2 and len(batch.bodies["body_ids"]) == len(spec.bones)


def configure_ears(armature: bpy.types.Object, *, shared_parameter_preset: bool, self_collision: bool) -> bool:
    system = _system(armature, "EARS", "Ears", ("ear.L", "ear.R"))
    system.self_collision = self_collision
    preset = ROOT / "blender_extension/bone_physics/presets/bone_parameters/firm_ears.json"
    assert bpy.ops.bone_physics.preset_apply(filepath=str(preset), target="ALL") == {"FINISHED"}
    _spec, batch = _compiled(armature)
    return shared_parameter_preset and not system.self_collision and len(batch.filters["excluded_body_offsets"]) == len(batch.bodies["body_ids"]) + 1


def configure_ponytail(armature: bpy.types.Object, *, self_collision: bool, body_collection: str) -> bool:
    system = _system(armature, "PONYTAIL", "Ponytail", ("ponytail",))
    assert bpy.ops.bone_physics.collision_resource_add(kind="ANALYTIC") == {"FINISHED"}
    resource = armature.bone_physics.collision_resources[-1]
    resource.name = body_collection
    resource.radius = 0.25
    assert bpy.ops.bone_physics.collision_resource_assign() == {"FINISHED"}
    system.self_collision = self_collision
    _spec, batch = _compiled(armature)
    return system.self_collision and resource.id in system.collision_resource_ids and len(batch.analytic_colliders["ids"]) == 1


def configure_skirt(armature: bpy.types.Object, *, chains: int, horizontal: bool, diagonals: bool, seam: bool) -> bool:
    system = _system(armature, "SKIRT", "Skirt", ("skirt.0", "skirt.1", "skirt.2", "skirt.3"))
    for chain in system.chains:
        chain.selected = True
    armature.bone_physics.next_stable_id = 200
    assert bpy.ops.bone_physics.structural_add(samples="0.0,1.0", diagonals=diagonals, closed=seam, compliance=1.0e-5) == {"FINISHED"}
    _spec, batch = _compiled(armature)
    return chains == len(system.chains) and horizontal and diagonals and seam and len(batch.constraints["ids"]) >= 42


def configure_accessory(armature: bpy.types.Object, *, per_bone_override: bool) -> bool:
    system = _system(armature, "ACCESSORY", "Accessory", ("accessory",))
    _select(armature, ("accessory.tip",))
    assert bpy.ops.bone_physics.override_add_selected() == {"FINISHED"}
    system.overrides[0].parameters.radius = 0.42
    _spec, batch = _compiled(armature)
    return per_bone_override and len(system.overrides) == 1 and 0.42 in batch.bodies["values"]


def reset_changes_only_configured_physical_bones(armature: bpy.types.Object) -> bool:
    configured = {bone.name for system in armature.bone_physics.physics_systems for chain in system.chains for bone in (armature.pose.bones[chain.head_uuid], armature.pose.bones[chain.tail_uuid])}
    return configured == {bone.name for bone in armature.pose.bones}


def exact_preset_round_trip_is_identical(armature: bpy.types.Object) -> bool:
    before = _exact_bundle(armature.bone_physics, armature)
    preset = serialize_exact_armature(before, _exact_snapshot(armature), "character")
    armature.bone_physics.physics_systems[0].name = "Changed"
    apply_exact_armature(_ExactOwner(armature.bone_physics), preset, _exact_snapshot(armature))
    return _exact_bundle(armature.bone_physics, armature) == before


bpy.ops.wm.read_factory_settings(use_empty=True)
bone_physics.unregister()
bone_physics.register()
try:
    armature = _armature()
    assert bpy.ops.bone_physics.initialize() == {"FINISHED"}
    assert configure_hair(armature, multiple_roots=True, disconnected_children=True)
    assert configure_ears(armature, shared_parameter_preset=True, self_collision=False)
    assert configure_ponytail(armature, self_collision=True, body_collection="Upper Body")
    assert configure_skirt(armature, chains=4, horizontal=True, diagonals=True, seam=True)
    assert configure_accessory(armature, per_bone_override=True)
    assert reset_changes_only_configured_physical_bones(armature)
    assert exact_preset_round_trip_is_identical(armature)

    spec, batch = _compiled(armature)
    assert len(batch.bodies["body_ids"]) == 20
    assert len(batch.constraints["ids"]) >= 42
    native = __import__("bone_physics").load_native(1)
    bridge = NativeRuntimeBridge(native, batch, spec.bones, fixed_dt=spec.fixed_dt)
    handle = bridge.create_world()
    try:
        assert native.advance(handle, spec.fixed_dt, "deterministic_preview")["steps"] == 1
    finally:
        native.destroy_world(handle)
finally:
    bone_physics.unregister()

print(json.dumps({"status": "ok", "test": "ui_character_workflows"}, sort_keys=True))
