import json
import tempfile
import unittest
from pathlib import Path

from tests.blender.run_reference import (
    AUTHORITATIVE_ITERATIONS,
    FIXED_HZ,
    FPS,
    SCENARIO_NAMES,
    load_acceptance,
    run_matrix,
)


class ReferenceRunnerContractTests(unittest.TestCase):
    def test_acceptance_registers_all_scenarios_and_fixed_step_policy(self):
        acceptance = load_acceptance(
            Path(__file__).parents[1] / "reference" / "acceptance.json"
        )
        self.assertEqual(tuple(acceptance["scenarios"]), SCENARIO_NAMES)
        self.assertEqual(tuple(acceptance["fps"]), FPS)
        self.assertEqual(acceptance["fixed_hz"], FIXED_HZ)
        self.assertEqual(acceptance["authoritative_iterations"], AUTHORITATIVE_ITERATIONS)

    def test_matrix_has_equal_time_and_repeatability_records(self):
        acceptance = load_acceptance(
            Path(__file__).parents[1] / "reference" / "acceptance.json"
        )
        report = run_matrix(acceptance, seed=17)
        self.assertEqual(report["scenario_count"], 10)
        self.assertEqual(report["run_count"], 60)
        self.assertEqual(report["failures"], [])
        for record in report["runs"]:
            self.assertEqual(record["fixed_hz"], 120)
            self.assertEqual(record["authoritative_iterations"], 64)
            self.assertIn("length_error", record["metrics"])
            self.assertIn("anchor_error", record["metrics"])
            self.assertIn("penetration", record["metrics"])
            self.assertIn("angular_lock", record["metrics"])
            self.assertIn("velocity_peak", record["metrics"])
            self.assertIn("contacts", record["metrics"])
            self.assertEqual(
                set(record["phase_timings"]),
                {
                    "prediction_seconds",
                    "animation_inertia_seconds",
                    "contact_generation_seconds",
                    "position_solve_seconds",
                    "diagnostics_seconds",
                    "velocity_seconds",
                },
            )

    def test_contact_required_scenarios_cannot_pass_without_contact(self):
        acceptance = load_acceptance(
            Path(__file__).parents[1] / "reference" / "acceptance.json"
        )
        report = run_matrix(acceptance, seed=19)
        required = {
            record["scenario"]
            for record in report["runs"]
            if record["contact_required"]
        }
        self.assertEqual(
            required,
            {"multiple_hair", "fast_arm", "moving_legs", "initial_penetration", "breast_torso"},
        )
        self.assertTrue(
            all(record["metrics"]["contacts"] > 0 for record in report["runs"] if record["contact_required"])
        )

    def test_failure_artifact_is_replayable_and_serializable(self):
        acceptance = load_acceptance(
            Path(__file__).parents[1] / "reference" / "acceptance.json"
        )
        acceptance["thresholds"]["hard_anchor"] = 0.0
        report = run_matrix(acceptance, seed=23)
        self.assertTrue(report["failures"])
        failure = report["failures"][0]
        for field in (
            "seed",
            "build_batch",
            "animation_input",
            "mesh_input",
            "checkpoint",
            "diagnostic_id",
            "actual",
            "allowed",
        ):
            self.assertIn(field, failure)
        json.dumps(report)


if __name__ == "__main__":
    unittest.main()
