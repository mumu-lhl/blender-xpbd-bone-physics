from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension.bone_physics.mesh_builder import (  # noqa: E402
    build_collision_surface,
    prepare_collision_source,
)
from blender_extension.bone_physics.model import CollisionSurfaceSpec  # noqa: E402
from blender_extension.bone_physics.ownership import (  # noqa: E402
    FaceOwnership,
    ObjectOwnership,
    ObjectRole,
    build_ownership_map,
)


bpy.ops.wm.read_factory_settings(use_empty=True)
mesh = bpy.data.meshes.new("TorsoMesh")
mesh.from_pydata(
    [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)],
    [],
    [(0, 1, 2), (0, 3, 1)],
)
mesh.update()
torso = bpy.data.objects.new("Torso", mesh)
bpy.context.collection.objects.link(torso)
physical = torso.vertex_groups.new(name="breast.L")
physical.add((0, 1, 2), 0.75, "REPLACE")
upstream = torso.vertex_groups.new(name="spine.003")
upstream.add((0, 1, 2, 3), 0.25, "REPLACE")
policy = CollisionSurfaceSpec(
    id=6,
    object_name="Torso",
    upstream_weight_merges=(("breast.L", "spine.003"),),
    body_region_ignores=((7, (20,)),),
)
collision = prepare_collision_source(torso, policy)

assert torso.vertex_groups["breast.L"].weight(0) == 0.75
assert collision.vertex_groups["spine.003"].weight(0) == 1.0
try:
    collision.vertex_groups["breast.L"].weight(0)
except RuntimeError:
    pass
else:
    raise AssertionError("physical weight must be removed from collision copy")

ownership = build_ownership_map(
    (
        ObjectOwnership(
            "Torso",
            ObjectRole.COLLISION,
            (
                FaceOwnership(10),
                FaceOwnership(20, driven_by_body_ids=(7,)),
            ),
            body_region_ignores=policy.body_region_ignores,
        ),
        ObjectOwnership("Hair", ObjectRole.SIMULATED),
    )
)
assert ownership.collision_objects() == ("Torso",)
assert ownership.body_sees_region(7, 10)
assert not ownership.body_sees_region(7, 20)

bpy.context.view_layer.update()
first = build_collision_surface(
    collision.evaluated_get(bpy.context.evaluated_depsgraph_get()),
    policy,
)
collision.data.vertices[3].co.z += 0.1
bpy.context.view_layer.update()
second = build_collision_surface(
    collision.evaluated_get(bpy.context.evaluated_depsgraph_get()),
    policy,
)
assert first.topology_fingerprint == second.topology_fingerprint
assert first.deformation_fingerprint != second.deformation_fingerprint
print(
    json.dumps(
        {
            "test": "deformation_ownership",
            "status": "ok",
            "collision_object": collision.name,
        },
        sort_keys=True,
    )
)
