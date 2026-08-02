"""Report native dynamic dependencies without installing anything."""

from __future__ import annotations

import argparse
import json
import platform
import re
import subprocess
from pathlib import Path

WHITELIST = (
    "libc.so",
    "libm.so",
    "libpthread.so",
    "libgcc_s.so",
    "libdl.so",
    "ld-linux-",
    "libSystem",
    "KERNEL32.dll",
    "USER32.dll",
    "ADVAPI32.dll",
    "VCRUNTIME",
    "api-ms-win-",
)


def dependencies(path: Path, system: str | None = None) -> dict[str, object]:
    system = system or platform.system().lower()
    if system.startswith("linux"):
        command = ["readelf", "-d", str(path)]
    elif system.startswith("darwin") or system == "macos":
        command = ["otool", "-L", str(path)]
    elif system.startswith("windows"):
        command = ["dumpbin", "/DEPENDENTS", str(path)]
    else:
        raise ValueError(f"unsupported native dependency platform: {system}")
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        output = result.stdout + result.stderr
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        return {"platform": system, "path": str(path), "status": "unavailable", "error": str(error), "dependencies": []}
    if system.startswith("linux"):
        pattern = r"lib[A-Za-z0-9_.+-]+\.so(?:\.[0-9]+)*"
    elif system.startswith("darwin") or system == "macos":
        pattern = r"lib[A-Za-z0-9_.+-]+\.dylib"
    else:
        pattern = r"[A-Za-z0-9_.+-]+\.dll"
    names = sorted(set(re.findall(pattern, output, flags=re.IGNORECASE)))
    unexpected = [name for name in names if not any(allowed in name for allowed in WHITELIST)]
    return {"platform": system, "path": str(path), "status": "ok", "dependencies": names, "unexpected": unexpected}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("native", type=Path)
    parser.add_argument("--system")
    args = parser.parse_args()
    report = dependencies(args.native, args.system)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 1 if report.get("status") != "ok" or report.get("unexpected") else 0


if __name__ == "__main__":
    raise SystemExit(main())
