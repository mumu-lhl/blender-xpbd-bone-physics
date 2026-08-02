import unittest

from blender_extension.bone_physics.diagnostics import DiagnosticReport


class DiagnosticReportTests(unittest.TestCase):
    def test_report_preserves_native_counters_and_phase_timings(self):
        report = DiagnosticReport.from_native(
            {
                "generated_contacts": 4,
                "active_contacts": 2,
                "broad_phase_candidates": 7,
                "ccd_fallbacks": 1,
                "ccd_failures": 0,
                "deepest_penetration": 0.001,
                "max_constraint_error": 0.00001,
                "rolled_back": False,
            },
            {
                "prediction_seconds": 0.1,
                "animation_inertia_seconds": 0.2,
                "contact_generation_seconds": 0.3,
                "position_solve_seconds": 0.4,
                "diagnostics_seconds": 0.5,
                "velocity_seconds": 0.6,
            },
        )
        data = report.to_dict()
        self.assertEqual(data["broad_phase_candidates"], 7)
        self.assertEqual(data["ccd_fallbacks"], 1)
        self.assertEqual(data["phase_timings"]["velocity_seconds"], 0.6)

    def test_failure_snapshot_contains_stable_ids_and_actual_allowed(self):
        report = DiagnosticReport.from_native(
            {"rolled_back": True, "ccd_failures": 1},
            {},
            failure={"id": "ccd-17", "body": 4, "triangle": 9},
        )
        snapshot = report.failure_snapshot(seed=12, actual=0.2, allowed=0.1)
        self.assertEqual(snapshot["diagnostic_id"], "ccd-17")
        self.assertEqual(snapshot["stable_ids"]["triangle"], 9)
        self.assertEqual(snapshot["actual"], 0.2)
        self.assertEqual(snapshot["allowed"], 0.1)
