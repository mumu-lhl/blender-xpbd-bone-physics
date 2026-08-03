"""Static preset-contract checks; run as part of the Blender test suite."""

from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from blender_extension.bone_physics.preset_io import load_parameter_preset


directory = ROOT / "blender_extension" / "bone_physics" / "presets" / "bone_parameters"
names = ("soft_hair.json", "firm_ears.json", "flexible_skirt.json")
for name in names:
    path = directory / name
    assert path.is_file(), path
    assert load_parameter_preset(path)["kind"] == "bone_parameters"
    assert json.loads(path.read_text("utf-8"))["schema_version"] == 1

print(json.dumps({"status": "ok", "test": "ui_presets"}, sort_keys=True))
