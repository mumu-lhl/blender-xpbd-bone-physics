#!/usr/bin/env python3
"""Download and verify the pinned Blender used by runtime CI."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import urllib.request


BLENDER_VERSION = "5.2.0"
BLENDER_ARCHIVE_NAME = f"blender-{BLENDER_VERSION}-linux-x64.tar.xz"
BLENDER_ARCHIVE_URL = (
    f"https://download.blender.org/release/Blender5.2/{BLENDER_ARCHIVE_NAME}"
)
BLENDER_ARCHIVE_SHA256 = (
    "96f6c181a30f4950607839dc84d42a354b250d8a0231b098b59b7bc69c351c48"
)
BLENDER_DOWNLOAD_USER_AGENT = "bone-physics-ci/1.0"


def blender_download_request() -> urllib.request.Request:
    return urllib.request.Request(
        BLENDER_ARCHIVE_URL,
        headers={"User-Agent": BLENDER_DOWNLOAD_USER_AGENT},
    )


def verify_sha256(path: Path, expected: str) -> None:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    actual = digest.hexdigest()
    if actual != expected:
        raise ValueError(
            f"Blender archive SHA-256 mismatch: expected {expected}, received {actual}"
        )


def safe_extract_tar(archive: tarfile.TarFile, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    root = destination.resolve()
    for member in archive.getmembers():
        target = (destination / member.name).resolve()
        if not target.is_relative_to(root):
            raise ValueError(f"archive member escapes extraction root: {member.name}")
        if member.isdev() or member.isfifo():
            raise ValueError(f"archive member has unsupported type: {member.name}")
        if member.issym():
            link_target = (target.parent / member.linkname).resolve()
            if not link_target.is_relative_to(root):
                raise ValueError(f"archive link escapes extraction root: {member.name}")
        elif member.islnk():
            link_target = (destination / member.linkname).resolve()
            if not link_target.is_relative_to(root):
                raise ValueError(f"archive link escapes extraction root: {member.name}")
    archive.extractall(destination)


def install_blender(destination: Path) -> Path:
    destination = destination.resolve()
    archive_path = destination / BLENDER_ARCHIVE_NAME
    install_root = destination / f"blender-{BLENDER_VERSION}-linux-x64"
    executable = install_root / "blender"
    destination.mkdir(parents=True, exist_ok=True)
    if executable.is_file():
        return executable
    temporary = archive_path.with_suffix(f"{archive_path.suffix}.part")
    try:
        with urllib.request.urlopen(blender_download_request()) as response:
            with temporary.open("wb") as output:
                shutil.copyfileobj(response, output, length=1024 * 1024)
        verify_sha256(temporary, BLENDER_ARCHIVE_SHA256)
        temporary.replace(archive_path)
        with tarfile.open(archive_path, "r:xz") as archive:
            safe_extract_tar(archive, destination)
    finally:
        temporary.unlink(missing_ok=True)
    if not executable.is_file():
        raise FileNotFoundError(f"Blender executable was not extracted to {executable}")
    return executable


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.report.parent.mkdir(parents=True, exist_ok=True)
    result: dict[str, object] = {
        "version": BLENDER_VERSION,
        "url": BLENDER_ARCHIVE_URL,
        "sha256": BLENDER_ARCHIVE_SHA256,
        "status": "failed",
    }
    try:
        executable = install_blender(arguments.destination)
        result.update(status="ok", executable=str(executable))
        return_code = 0
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        return_code = 1
    arguments.report.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if return_code:
        (arguments.report.parent / "failure-snapshot.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    else:
        print(result["executable"])
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
