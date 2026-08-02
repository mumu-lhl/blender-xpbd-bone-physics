from __future__ import annotations

import math
import unittest

from tests.blender.scenes.body_collision import fast_arm
from tests.reference.native_backend import NativeReferenceBackend


class NativeReferenceBackendTests(unittest.TestCase):
    def test_backend_advances_native_world_and_measures_pose(self):
        record = NativeReferenceBackend().run(fast_arm(), 24, 1, 17)

        self.assertEqual(record["fixed_hz"], 120)
        self.assertEqual(record["authoritative_iterations"], 64)
        self.assertEqual(len(record["poses"]), 12 * 7)
        self.assertTrue(math.isfinite(record["metrics"]["length_error"]))
        self.assertIn("phase_timings", record)
        self.assertIn("summary", record["diagnostics"])

    def test_contact_required_fixture_reports_contacts(self):
        record = NativeReferenceBackend().run(fast_arm(), 24, 1, 17)

        self.assertTrue(record["contact_required"])
        self.assertGreater(record["metrics"]["contacts"], 0)


if __name__ == "__main__":
    unittest.main()
