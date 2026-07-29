from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from zipfile import ZipFile

from scripts.ci.install_blender import (
    BLENDER_ARCHIVE_SHA256,
    safe_extract_tar,
    verify_sha256,
)
from scripts.ci.run_blender_tests import (
    BLENDER_TEST_SCRIPTS,
    build_extension_bundle,
    run_blender_tests,
)
from scripts.ci.inspect_native_wheel import unexpected_dependencies
from scripts.ci.run_python_tests import command_report, unittest_arguments


class BlenderInstallerTests(unittest.TestCase):
    def test_pinned_checksum_has_sha256_shape(self) -> None:
        self.assertEqual(len(BLENDER_ARCHIVE_SHA256), 64)
        int(BLENDER_ARCHIVE_SHA256, 16)

    def test_verify_sha256_rejects_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / "blender.tar.xz"
            archive.write_bytes(b"not blender")
            with self.assertRaisesRegex(ValueError, "SHA-256"):
                verify_sha256(archive, "0" * 64)
            verify_sha256(archive, hashlib.sha256(b"not blender").hexdigest())

    def test_safe_extract_rejects_parent_traversal(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "malicious.tar"
            payload = root / "payload"
            payload.write_text("escape", encoding="utf-8")
            with tarfile.open(archive, "w") as output:
                output.add(payload, arcname="../escape")
            with tarfile.open(archive) as input_archive:
                with self.assertRaisesRegex(ValueError, "escapes"):
                    safe_extract_tar(input_archive, root / "output")


class BlenderRunnerTests(unittest.TestCase):
    def test_blender_script_set_is_explicit_and_complete(self) -> None:
        self.assertEqual(
            BLENDER_TEST_SCRIPTS,
            (
                "test_prephysics_order.py",
                "test_pose_roundtrip.py",
                "test_runtime_handler.py",
            ),
        )

    def test_extension_bundle_contains_manifest_package_and_native(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "blender_extension"
            package = source / "bone_physics"
            package.mkdir(parents=True)
            (source / "blender_manifest.toml").write_text(
                'schema_version = "1.0.0"\nid = "bone_physics"\n',
                encoding="utf-8",
            )
            (package / "__init__.py").write_text("", encoding="utf-8")
            wheel = root / "native.whl"
            with ZipFile(wheel, "w") as output:
                output.writestr("bone_physics_native.abi3.so", b"\x7fELF")

            bundle = build_extension_bundle(source, wheel, root / "dist")

            with ZipFile(bundle) as built:
                names = set(built.namelist())
            self.assertIn("blender_manifest.toml", names)
            self.assertIn("__init__.py", names)
            self.assertIn(
                "native/linux-x86_64/bone_physics_native.abi3.so",
                names,
            )

    def test_preflight_failure_still_writes_blender_report_and_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            wheel = root / "invalid.whl"
            with ZipFile(wheel, "w"):
                pass
            report = root / "report"

            result = run_blender_tests(
                root / "missing-blender",
                root,
                wheel,
                report,
            )

            self.assertEqual(result, 1)
            self.assertTrue((report / "blender-runtime-report.json").is_file())
            self.assertTrue((report / "failure-snapshot.json").is_file())


class CommandReportTests(unittest.TestCase):
    def test_native_job_installs_toolchain_file_components_explicitly(self) -> None:
        workflow = Path(".github/workflows/ci.yml").read_text(encoding="utf-8")
        native_job = workflow.split("  native-contract:", 1)[1].split(
            "  blender-runtime:", 1
        )[0]
        self.assertIn("components: rustfmt, clippy", native_job)

    def test_discovery_arguments_are_compatible_with_python_311(self) -> None:
        self.assertEqual(
            unittest_arguments(Path("."), []),
            ["-m", "unittest", "discover", "-s", "tests/python", "-v"],
        )

    def test_failure_always_writes_report_and_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            report_dir = Path(directory)
            result = command_report(
                ["python", "-c", "raise SystemExit(7)"],
                report_dir,
                "intentional-failure",
            )

            self.assertEqual(result["exit_code"], 7)
            report = json.loads(
                (report_dir / "intentional-failure.json").read_text(encoding="utf-8")
            )
            snapshot = json.loads(
                (report_dir / "failure-snapshot.json").read_text(encoding="utf-8")
            )
            self.assertEqual(report["exit_code"], 7)
            self.assertEqual(snapshot["failed_command"], "intentional-failure")

    def test_native_dependency_allowlist_rejects_python_and_unknown_libraries(
        self,
    ) -> None:
        self.assertEqual(
            unexpected_dependencies(
                (
                    "libc.so.6",
                    "libm.so.6",
                    "libgcc_s.so.1",
                    "ld-linux-x86-64.so.2",
                )
            ),
            (),
        )
        self.assertEqual(
            unexpected_dependencies(("libpython3.11.so.1.0", "libmystery.so")),
            ("libmystery.so", "libpython3.11.so.1.0"),
        )


if __name__ == "__main__":
    unittest.main()
