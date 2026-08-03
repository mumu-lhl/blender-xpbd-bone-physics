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

operator_source = (ROOT / "blender_extension" / "bone_physics" / "operators" / "preset.py").read_text("utf-8")
panel_source = (ROOT / "blender_extension" / "bone_physics" / "ui" / "panels.py").read_text("utf-8")
for identifier in (
    "bone_physics.exact_preset_save",
    "bone_physics.exact_preset_apply",
    "bone_physics.exact_preset_delete",
):
    assert identifier in operator_source, identifier
assert "Overwrite Exact Armature Preset" in operator_source
assert "Delete Exact Armature Preset" in operator_source
assert "Exact Armature Presets" in panel_source
assert "EXACT_ARMATURE_PRESET_KIND" in panel_source

print(json.dumps({"status": "ok", "test": "ui_presets"}, sort_keys=True))
