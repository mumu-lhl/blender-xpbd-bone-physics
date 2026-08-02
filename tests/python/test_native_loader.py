from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from blender_extension.bone_physics import native


class NativeLoaderTests(unittest.TestCase):
    def test_extension_entrypoint_exposes_blender_lifecycle(self) -> None:
        import blender_extension.bone_physics as extension

        self.assertTrue(callable(extension.register))
        self.assertTrue(callable(extension.unregister))

    def setUp(self):
        native._reset_for_tests()
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.package = Path(self.temporary.name)
        self.platform_tag = "linux-x86_64"
        self.native_dir = self.package / "native" / self.platform_tag
        self.native_dir.mkdir(parents=True)

    def binary(self, name="bone_physics_native.abi3.so"):
        path = self.native_dir / name
        path.write_bytes(b"test placeholder")
        return path

    def patches(self, loader):
        loader_patch = (
            mock.patch.object(native, "_load_extension", side_effect=loader)
            if isinstance(loader, BaseException)
            else mock.patch.object(native, "_load_extension", return_value=loader)
        )
        return (
            mock.patch.object(native, "_package_directory", return_value=self.package),
            mock.patch.object(native, "_platform_tag", return_value=self.platform_tag),
            loader_patch,
        )

    def test_missing_binary_reports_platform_without_fallback_install(self):
        with (
            mock.patch.object(native, "_package_directory", return_value=self.package),
            mock.patch.object(native, "_platform_tag", return_value=self.platform_tag),
        ):
            with self.assertRaisesRegex(
                native.NativeUnavailableError,
                "linux-x86_64.*exactly one",
            ):
                native.load_native(1)

    def test_multiple_matching_binaries_are_rejected(self):
        self.binary()
        self.binary("bone_physics_native.debug.so")
        with (
            mock.patch.object(native, "_package_directory", return_value=self.package),
            mock.patch.object(native, "_platform_tag", return_value=self.platform_tag),
        ):
            with self.assertRaisesRegex(native.NativeUnavailableError, "found 2"):
                native.load_native(1)

    def test_import_failure_preserves_original_exception_and_platform(self):
        self.binary()
        failure = OSError("ELF architecture mismatch")
        package_patch, platform_patch, loader_patch = self.patches(failure)
        with package_patch, platform_patch, loader_patch:
            with self.assertRaises(native.NativeUnavailableError) as raised:
                native.load_native(1)

        self.assertIn("ELF architecture mismatch", str(raised.exception))
        self.assertIn(self.platform_tag, str(raised.exception))
        self.assertIs(raised.exception.__cause__, failure)

    def test_wrong_api_never_exposes_module_or_creates_world(self):
        self.binary()
        fake = SimpleNamespace(
            native_api_version=mock.Mock(return_value=2),
            create_world=mock.Mock(),
        )
        package_patch, platform_patch, loader_patch = self.patches(fake)
        with package_patch, platform_patch, loader_patch:
            with self.assertRaisesRegex(native.NativeVersionError, "expected 1.*received 2"):
                native.load_native(1)

        fake.create_world.assert_not_called()
        with self.assertRaisesRegex(native.NativeUnavailableError, "load_native"):
            native.get_native()

    def test_matching_api_returns_and_caches_the_validated_module(self):
        self.binary()
        fake = SimpleNamespace(native_api_version=mock.Mock(return_value=1))
        package_patch, platform_patch, loader_patch = self.patches(fake)
        with package_patch, platform_patch, loader_patch as loader_mock:
            first = native.load_native(1)
            second = native.load_native(1)

        self.assertIs(first, fake)
        self.assertIs(second, fake)
        self.assertIs(native.get_native(), fake)
        fake.native_api_version.assert_called_once_with()
        loader_mock.assert_called_once()

    def test_manifest_and_staged_binary_set_validate_together(self):
        from tests.blender.validate_manifest import validate_extension

        self.binary()
        manifest = Path(__file__).parents[2] / "blender_extension" / "blender_manifest.toml"

        result = validate_extension(manifest, self.package, self.platform_tag)

        self.assertEqual(result["blender_version_min"], "5.2.0")
        self.assertEqual(result["native_binary_count"], 1)
        self.assertIn("SPDX:GPL-3.0-or-later", result["license"])

    def test_supported_platforms_have_stable_package_tags(self):
        cases = (
            ("Linux", "x86_64", "linux-x86_64"),
            ("Linux", "aarch64", "linux-arm64"),
            ("Windows", "AMD64", "windows-x86_64"),
            ("Darwin", "arm64", "macos-arm64"),
            ("Darwin", "x86_64", "macos-x86_64"),
        )
        for system, machine, expected in cases:
            with self.subTest(system=system, machine=machine):
                with (
                    mock.patch.object(native.platform, "system", return_value=system),
                    mock.patch.object(native.platform, "machine", return_value=machine),
                ):
                    self.assertEqual(native._platform_tag(), expected)


if __name__ == "__main__":
    unittest.main()
