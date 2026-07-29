from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy
import bone_physics_native as native

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension.bone_physics.mesh_builder import (  # noqa: E402
    build_collision_surface,
    mesh_source_from_evaluated,
)
from blender_extension.bone_physics.model import (  # noqa: E402
    BoneSpec,
    CollisionSurfaceSpec,
    WorldSpec,
)
from blender_extension.bone_physics.parameter_builder import compile_world_spec  # noqa: E402
from blender_extension.bone_physics.prephysics_eval import PrePhysicsSnapshot  # noqa: E402
from blender_extension.bone_physics.runtime import NativeRuntimeBridge  # noqa: E402


IDENTITY = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)

bpy.ops.wm.read_factory_settings(use_empty=True)
mesh = bpy.data.meshes.new("CollisionMesh")
mesh.from_pydata(
    [(-2, 0, -2), (2, 0, -2), (2, 0, 2), (-2, 0, 2)],
    [],
    [(0, 1, 2, 3)],
)
mesh.update()
body_object = bpy.data.objects.new("Body", mesh)
bpy.context.collection.objects.link(body_object)
bpy.context.view_layer.update()

surface = CollisionSurfaceSpec(id=501, object_name="Body")
world = WorldSpec(
    bones=(
        BoneSpec(
            id=1,
            name="hair",
            local_head=(0, 0, 0),
            local_tail=(0, 1, 0),
        ),
    ),
    collision_surfaces=(surface,),
)
bridge = NativeRuntimeBridge(
    native,
    compile_world_spec(world),
    world.bones,
    fixed_dt=world.fixed_dt,
)
handle = bridge.create_world()
try:
    depsgraph = bpy.context.evaluated_depsgraph_get()
    first_source = mesh_source_from_evaluated(
        body_object.evaluated_get(depsgraph),
        surface,
    )
    first = build_collision_surface(first_source, surface)
    bridge.set_snapshot(
        handle,
        PrePhysicsSnapshot(0.0, (), (), IDENTITY, (first,)),
    )
    assert native.read_diagnostics(handle)["mesh_count"] == 1

    body_object.data.vertices[0].co.y = 0.1
    bpy.context.view_layer.update()
    second_source = mesh_source_from_evaluated(
        body_object.evaluated_get(depsgraph),
        surface,
    )
    second = build_collision_surface(
        second_source,
        surface,
        previous=first_source,
        template=first,
    )
    bridge.set_snapshot(
        handle,
        PrePhysicsSnapshot(world.fixed_dt, (), (), IDENTITY, (second,)),
    )
    assert bridge.mesh_topologies[(handle, 501)][0] == first.topology_fingerprint
    print(
        json.dumps(
            {
                "test": "mesh_upload",
                "status": "ok",
                "mesh_count": native.read_diagnostics(handle)["mesh_count"],
            },
            sort_keys=True,
        )
    )
finally:
    native.destroy_world(handle)
