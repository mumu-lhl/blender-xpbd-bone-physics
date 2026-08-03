from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from blender_extension.bone_physics.preset_io import (
    PresetError,
    delete_preset,
    load_parameter_preset,
    rename_preset,
    sanitize_preset_filename,
    save_parameter_preset,
)


def endpoint_parameters() -> dict[str, object]:
    return {
        "inverse_mass": 1.0, "inertia_scale": [1.0, 1.0, 1.0], "gravity_scale": 1.0,
        "translation_inertia": 1.0, "rotation_inertia": 1.0, "linear_damping": 0.0,
        "angular_damping": 0.0, "joint_compliance": 0.0, "joint_damping": 0.0,
        "angular_compliance": 0.0, "angular_damping_constraint": 0.0,
        "recovery_compliance": 0.0, "recovery_damping": 0.0,
        "angular_limit_mode": "FREE", "xyz_lower": [-3.141592653589793] * 3,
        "xyz_upper": [3.141592653589793] * 3, "swing_axis": [0.0, 1.0, 0.0],
        "max_swing": 3.141592653589793, "min_twist": -3.141592653589793,
        "max_twist": 3.141592653589793, "radius": 0.1, "margin": 0.01,
        "capsule_head_inset": 0.0, "capsule_tail_inset": 0.0, "friction": 0.5,
        "restitution": 0.0, "interpolation": "LINEAR",
    }


def parameter_preset(name: str) -> dict[str, object]:
    endpoint = endpoint_parameters()
    return {
        "kind": "bone_parameters", "schema_version": 1, "name": name,
        "head": endpoint, "tail": endpoint, "overrides": [
            {"bone_name": "tip", "parameters": endpoint},
        ],
    }


class PresetIoTests(unittest.TestCase):
    def test_parameter_preset_round_trip_is_canonical_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "soft_hair.json"
            preset = parameter_preset("Soft Hair")
            save_parameter_preset(path, preset)
            self.assertEqual(load_parameter_preset(path), preset)
            self.assertTrue(path.read_text("utf-8").endswith("\n"))
            self.assertEqual(json.loads(path.read_text("utf-8"))["name"], "Soft Hair")

    def test_unknown_newer_schema_is_rejected_without_changing_caller_data(self):
        before = endpoint_parameters()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "future.json"
            path.write_text('{"kind":"bone_parameters","schema_version":99}', "utf-8")
            with self.assertRaisesRegex(PresetError, "newer schema"):
                load_parameter_preset(path)
        self.assertEqual(before, endpoint_parameters())

    def test_invalid_preset_never_replaces_existing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "existing.json"
            path.write_text("original", "utf-8")
            invalid = parameter_preset("Bad")
            invalid["head"]["radius"] = -1.0  # type: ignore[index]
            with self.assertRaisesRegex(PresetError, "radius"):
                save_parameter_preset(path, invalid)
            self.assertEqual(path.read_text("utf-8"), "original")

    def test_filename_sanitization_blocks_paths_and_rename_delete_stay_in_directory(self):
        self.assertEqual(sanitize_preset_filename(" ../Soft Hair "), "Soft_Hair.json")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "soft.json"
            save_parameter_preset(source, parameter_preset("Soft Hair"))
            renamed = rename_preset(source, "Firm Ears", root)
            self.assertEqual(renamed.name, "Firm_Ears.json")
            self.assertTrue(delete_preset(renamed, root))
            with self.assertRaises(PresetError):
                delete_preset(root.parent / "outside.json", root)

