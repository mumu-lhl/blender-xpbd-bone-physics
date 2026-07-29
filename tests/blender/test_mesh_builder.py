from __future__ import annotations

import json
from pathlib import Path
import sys

import bpy

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension.bone_physics.mesh_builder import build_collision_surface  # noqa: E402
from blender_extension.bone_physics.model import CollisionSurfaceSpec  # noqa: E402


bpy.ops.wm.read_factory_settings(use_empty=True)
mesh = bpy.data.meshes.new("BodyMesh")
mesh.from_pydata(
    [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)],
    [],
    [(0, 1, 2, 3)],
)
mesh.update()
region = mesh.attributes.new("bone_physics_region", "INT", "FACE")
region.data[0].value = 7
material = bpy.data.materials.new("skin")
mesh.materials.append(material)
body = bpy.data.objects.new("Body", mesh)
bpy.context.collection.objects.link(body)
body.scale = (2.0, 3.0, 4.0)
body.location = (3.0, -2.0, 1.0)
bpy.context.view_layer.update()

evaluated = body.evaluated_get(bpy.context.evaluated_depsgraph_get())
batch = build_collision_surface(
    evaluated,
    CollisionSurfaceSpec(id=5, object_name="Body"),
)
assert batch.topology.triangles == ((0, 1, 2), (0, 2, 3))
assert batch.regions == (7, 7)
assert batch.current_vertices[0] == (3.0, -2.0, 1.0)
assert batch.current_vertices[2] == (5.0, 1.0, 1.0)
print(
    json.dumps(
        {
            "test": "mesh_builder",
            "status": "ok",
            "topology": batch.topology_fingerprint,
        },
        sort_keys=True,
    )
)
