from __future__ import annotations

from array import array
import json
import math


import bone_physics


native = bone_physics.load_native(1)
assert native.native_api_version() == 1
assert callable(bone_physics.register)
assert callable(bone_physics.unregister)


def body_row(x):
    return (
        x, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, -0.5, 0.0,
        0.0, 0.5, 0.0,
        1.0, 1.0, 1.0, 1.0,
        1.0, 0.0, 0.0,
        x, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, -0.4, 0.0,
        0.0, 0.4, 0.0,
        0.1, 0.01, 0.5, 0.0,
    )


def empty_constraints():
    return {
        "ids": array("I"),
        "body_ids": array("I"),
        "kinds": array("I"),
        "enabled": array("I"),
        "values": array("f"),
        "columns": 24,
    }


def filters():
    return {
        "layers": array("I", (1, 1)),
        "masks": array("I", (0xFFFFFFFF, 0xFFFFFFFF)),
        "excluded_body_offsets": array("I", (0, 0, 0)),
        "excluded_body_ids": array("I"),
        "excluded_constraint_offsets": array("I", (0, 0, 0)),
        "excluded_constraint_ids": array("I"),
    }


handle = native.build_world(
    {
        "api_version": native.native_api_version(),
        "gravity": (0.0, -9.81, 0.0),
        "fixed_dt": 1.0 / 120.0,
    },
    {
        "body_ids": array("I", (1, 2)),
        "collider_ids": array("I", (101, 102)),
        "values": array("f", body_row(0.0) + body_row(2.0)),
        "columns": 46,
    },
    empty_constraints(),
    filters(),
)
try:
    summary = native.advance(handle, 1.0 / 120.0, "authoritative_bake")
    output = array("f", [0.0] * 14)
    native.read_transforms(handle, output)
    quaternion_lengths = [
        math.sqrt(sum(component * component for component in output[offset + 3 : offset + 7]))
        for offset in (0, 7)
    ]
    assert summary["steps"] == 1
    assert all(math.isfinite(value) for value in output)
    assert all(abs(length - 1.0) < 1.0e-5 for length in quaternion_lengths)
finally:
    native.destroy_world(handle)

print(
    json.dumps(
        {
            "test": "installed_extension",
            "status": "ok",
            "native_api_version": native.native_api_version(),
            "steps": summary["steps"],
            "quaternion_lengths": quaternion_lengths,
        },
        sort_keys=True,
    )
)
