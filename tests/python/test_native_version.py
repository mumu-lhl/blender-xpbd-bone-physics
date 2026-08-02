import unittest

import bone_physics_native as native


VALID_CONFIG = {
    "gravity": (0.0, -9.81, 0.0),
    "fixed_dt": 1.0 / 120.0,
}


class NativeVersionTests(unittest.TestCase):
    def test_api_version_matches_the_first_public_contract(self):
        self.assertEqual(native.native_api_version(), 1)

    def test_destroyed_handle_remains_stale_after_slot_reuse(self):
        stale = native.create_world(VALID_CONFIG, {})
        native.destroy_world(stale)
        replacement = native.create_world(VALID_CONFIG, {})
        self.addCleanup(native.destroy_world, replacement)

        self.assertNotEqual(stale, replacement)
        with self.assertRaises(native.BonePhysicsStateError):
            native.destroy_world(stale)

    def test_double_destroy_is_a_state_error(self):
        handle = native.create_world(VALID_CONFIG, {})
        native.destroy_world(handle)

        with self.assertRaises(native.BonePhysicsStateError):
            native.destroy_world(handle)

    def test_invalid_configuration_is_an_input_error(self):
        for config in (
            {"gravity": (0.0, -9.81, 0.0), "fixed_dt": 0.0},
            {"gravity": (0.0, float("nan"), 0.0), "fixed_dt": 1.0 / 120.0},
        ):
            with self.subTest(config=config):
                with self.assertRaises(native.BonePhysicsInputError):
                    native.create_world(config, {})

    def test_nonempty_build_batch_is_rejected_until_batch_contract_exists(self):
        with self.assertRaises(native.BonePhysicsInputError):
            native.create_world(VALID_CONFIG, {"bodies": [1]})


if __name__ == "__main__":
    unittest.main()
