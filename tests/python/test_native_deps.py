from pathlib import Path
import subprocess
from unittest.mock import patch
import unittest

from scripts.check_native_deps import dependencies


class NativeDependencyTests(unittest.TestCase):
    def test_linux_readelf_names_are_parsed_and_private_library_is_unexpected(self):
        completed = subprocess.CompletedProcess(
            ["readelf"],
            0,
            stdout=" 0x0000000000000001 (NEEDED) Shared library: [libc.so.6]\n"
            " 0x0000000000000001 (NEEDED) Shared library: [libm.so.6]\n"
            " 0x0000000000000001 (NEEDED) Shared library: [libprivate.so.1]\n",
            stderr="",
        )
        with patch("scripts.check_native_deps.subprocess.run", return_value=completed) as run:
            report = dependencies(Path("solver.so"), "linux")

        run.assert_called_once_with(
            ["readelf", "-d", "solver.so"],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual(report["status"], "ok")
        self.assertEqual(
            report["dependencies"],
            ["libc.so.6", "libm.so.6", "libprivate.so.1"],
        )
        self.assertEqual(report["unexpected"], ["libprivate.so.1"])

    def test_macos_otool_names_are_parsed(self):
        completed = subprocess.CompletedProcess(
            ["otool"],
            0,
            stdout="solver.so:\n\t/usr/lib/libSystem.B.dylib (compatibility version 1.0)\n\t/usr/lib/libiconv.2.dylib\n",
            stderr="",
        )
        with patch("scripts.check_native_deps.subprocess.run", return_value=completed):
            report = dependencies(Path("solver.so"), "darwin")

        self.assertEqual(report["status"], "ok")
        self.assertEqual(report["dependencies"], ["libSystem.B.dylib", "libiconv.2.dylib"])
        self.assertEqual(report["unexpected"], [])

    def test_windows_dumpbin_names_are_parsed(self):
        completed = subprocess.CompletedProcess(
            ["dumpbin"],
            0,
            stdout="\tKERNEL32.dll\n\tntdll.dll\n\tpython3.dll\n\tUSER32.dll\n\tBONE_PRIVATE.dll\n",
            stderr="",
        )
        with patch("scripts.check_native_deps.subprocess.run", return_value=completed):
            report = dependencies(Path("solver.pyd"), "windows")

        self.assertEqual(report["status"], "ok")
        self.assertEqual(
            report["dependencies"],
            ["BONE_PRIVATE.dll", "KERNEL32.dll", "USER32.dll", "ntdll.dll", "python3.dll"],
        )
        self.assertEqual(report["unexpected"], ["BONE_PRIVATE.dll"])

    def test_missing_inspection_tool_is_not_a_pass(self):
        with patch(
            "scripts.check_native_deps.subprocess.run",
            side_effect=FileNotFoundError("readelf"),
        ):
            report = dependencies(Path("solver.so"), "linux")

        self.assertEqual(report["status"], "unavailable")
        self.assertEqual(report["dependencies"], [])
        self.assertNotEqual(report["status"], "ok")


if __name__ == "__main__":
    unittest.main()
