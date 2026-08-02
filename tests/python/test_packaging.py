import tempfile
import tomllib
import unittest
from pathlib import Path

from scripts.package_extension import package
from scripts.verify_package import verify


class PackagingTests(unittest.TestCase):
    def test_native_metadata_matches_extension_license(self):
        metadata = tomllib.loads(
            Path("crates/bone_physics_py/pyproject.toml").read_text(encoding="utf-8")
        )
        self.assertEqual(metadata["project"]["license"], "GPL-3.0-or-later")

    def test_archive_is_byte_reproducible_and_structurally_verified(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            extension = root / "blender_extension"
            extension.mkdir()
            (extension / "blender_manifest.toml").write_text('version = "0.1.0"\n', encoding="utf-8")
            (extension / "__init__.py").write_text("\n", encoding="utf-8")
            package_root = extension / "bone_physics"
            package_root.mkdir()
            (package_root / "__init__.py").write_text("VALUE = 1\n", encoding="utf-8")
            native = root / "bone_physics_native.so"
            native.write_bytes(b"native")
            first = root / "one.zip"
            second = root / "two.zip"
            package(root, first, native=native, platform="linux-x86_64")
            package(root, second, native=native, platform="linux-x86_64")
            self.assertEqual(first.read_bytes(), second.read_bytes())
            report = verify(first, expected_platform="linux-x86_64")
            self.assertEqual(report["native_count"], 1)
            self.assertIn("__init__.py", report["files"])
            self.assertNotIn("blender_extension/__init__.py", report["files"])
            self.assertNotIn("bone_physics/__init__.py", report["files"])

    def test_verifier_rejects_platform_without_matching_native_module(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            extension = root / "blender_extension"
            extension.mkdir()
            (extension / "blender_manifest.toml").write_text('version = "0.1.0"\n', encoding="utf-8")
            archive = root / "package.zip"
            package(root, archive)
            with self.assertRaisesRegex(ValueError, "native"):
                verify(archive, expected_platform="windows-x86_64")
