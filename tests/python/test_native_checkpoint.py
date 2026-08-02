import unittest

try:
    import bone_physics_native as native
except ModuleNotFoundError:
    native = None

if native is not None:
    from tests.python.test_native_batches import bodies, config, empty_constraints, filters
else:
    bodies = config = empty_constraints = filters = None


@unittest.skipIf(native is None, "native bridge is tested in native-contract")
class NativeCheckpointTests(unittest.TestCase):
    def setUp(self):
        self.handle = native.build_world(
            config(), bodies(), empty_constraints(), filters()
        )
        self.addCleanup(native.destroy_world, self.handle)

    def test_fingerprint_and_authoritative_checkpoint_restore(self):
        fingerprint = native.world_fingerprint(self.handle)
        self.assertIsInstance(fingerprint, bytes)
        self.assertEqual(len(fingerprint), 32)

        checkpoint = native.encode_checkpoint(self.handle, "authoritative_bake")
        native.advance(self.handle, 1.0 / 120.0, "authoritative_bake")
        native.restore_checkpoint(self.handle, checkpoint, fingerprint)

        self.assertEqual(native.world_fingerprint(self.handle), fingerprint)

    def test_approximate_preview_cannot_write_authoritative_checkpoint(self):
        with self.assertRaisesRegex(native.BonePhysicsInputError, "approximate"):
            native.encode_checkpoint(self.handle, "approximate_preview")
