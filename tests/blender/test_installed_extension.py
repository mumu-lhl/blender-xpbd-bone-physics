from __future__ import annotations

import json


import bone_physics


native = bone_physics.load_native(1)
assert native.native_api_version() == 1
assert callable(bone_physics.register)
assert callable(bone_physics.unregister)

print(
    json.dumps(
        {
            "test": "installed_extension",
            "status": "ok",
            "native_api_version": native.native_api_version(),
        },
        sort_keys=True,
    )
)
