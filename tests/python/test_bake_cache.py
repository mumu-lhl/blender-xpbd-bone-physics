import json
import tempfile
import unittest
from pathlib import Path

from blender_extension.bone_physics.bake_cache import (
    BakeCache,
    CacheInvalidation,
)


class BakeCacheTests(unittest.TestCase):
    def test_incomplete_generation_is_not_visible_as_current(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = BakeCache(Path(directory), "a" * 64, quality="authoritative_bake")
            cache.begin()
            cache.write_frame(0, ((0.0, 0.0, 0.0),))

            self.assertIsNone(BakeCache.open_current(Path(directory)))
            cache.abort()

    def test_commit_publishes_one_atomic_complete_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = BakeCache(Path(directory), "b" * 64, quality="authoritative_bake")
            cache.begin()
            cache.write_frame(0, ((0.0, 0.0, 0.0),))
            cache.write_checkpoint(0, b"checkpoint")
            manifest = cache.commit()

            self.assertTrue(manifest.complete)
            loaded = BakeCache.open_current(Path(directory))
            self.assertIsNotNone(loaded)
            self.assertEqual(loaded.manifest.generation, manifest.generation)
            self.assertEqual(loaded.nearest_checkpoint(0).payload, b"checkpoint")

    def test_approximate_preview_cannot_write_authoritative_checkpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = BakeCache(Path(directory), "c" * 64, quality="approximate_preview")
            cache.begin()
            with self.assertRaisesRegex(ValueError, "approximate"):
                cache.write_checkpoint(0, b"checkpoint")
            cache.abort()

    def test_changed_fingerprint_has_named_invalidation_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = BakeCache(Path(directory), "d" * 64, quality="authoritative_bake")
            cache.begin()
            cache.write_frame(0, ((0.0, 0.0, 0.0),))
            cache.commit()

            with self.assertRaises(CacheInvalidation) as context:
                BakeCache(Path(directory), "e" * 64, quality="authoritative_bake").open()
            self.assertEqual(context.exception.reason, "fingerprint_changed")

    def test_corrupt_generation_pointer_is_rejected_before_path_join(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "current.json").write_text(
                json.dumps({"generation": "../escape"}), encoding="utf-8"
            )
            with self.assertRaises(CacheInvalidation) as context:
                BakeCache.open_current(root)
            self.assertEqual(context.exception.reason, "invalid_generation")

    def test_empty_bake_cannot_commit_complete_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = BakeCache(Path(directory), "f" * 64, quality="authoritative_bake")
            cache.begin()
            with self.assertRaisesRegex(ValueError, "at least one frame"):
                cache.commit()
            cache.abort()
