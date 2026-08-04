"""Validated loader for the native solver bundled inside the extension."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import platform
from types import ModuleType
from typing import Protocol, runtime_checkable
import threading


class NativeUnavailableError(RuntimeError):
    """The current extension package has no usable native solver."""


class NativeVersionError(RuntimeError):
    """The packaged Python and native APIs are incompatible."""


NATIVE_API_VERSION = 1


@runtime_checkable
class NativeModule(Protocol):
    def native_api_version(self) -> int: ...


_load_lock = threading.Lock()
_loaded_module: NativeModule | None = None
_loaded_api: int | None = None


def load_native(expected_api: int) -> NativeModule:
    """Load and expose one package-local native module after version validation."""
    if isinstance(expected_api, bool) or not isinstance(expected_api, int) or expected_api <= 0:
        raise ValueError("expected native API version must be a positive integer")
    with _load_lock:
        global _loaded_module, _loaded_api
        if _loaded_module is not None:
            if _loaded_api != expected_api:
                raise NativeVersionError(
                    f"native API version mismatch: expected {expected_api}, "
                    f"received {_loaded_api}"
                )
            return _loaded_module

        package = _package_directory()
        tag = _platform_tag()
        candidates = matching_native_candidates(package, tag)
        if len(candidates) != 1:
            raise NativeUnavailableError(
                f"platform {tag} requires exactly one packaged native solver; "
                f"found {len(candidates)} under {package / 'native' / tag}"
            )
        path = candidates[0]
        try:
            module = _load_extension(path)
            version_function = getattr(module, "native_api_version")
            actual_api = version_function()
        except Exception as error:
            raise NativeUnavailableError(
                f"could not load packaged native solver for {tag} from {path}: {error}"
            ) from error
        if isinstance(actual_api, bool) or not isinstance(actual_api, int):
            raise NativeVersionError(
                f"native API version for {tag} must be an integer, "
                f"received {actual_api!r}"
            )
        if actual_api != expected_api:
            raise NativeVersionError(
                f"native API version mismatch for {tag}: expected {expected_api}, "
                f"received {actual_api}"
            )
        _loaded_module = module
        _loaded_api = actual_api
        return module


def get_native() -> NativeModule:
    """Return the already validated native module without attempting discovery."""
    if _loaded_module is None:
        raise NativeUnavailableError(
            "native solver is unavailable until load_native(expected_api) succeeds"
        )
    return _loaded_module


def matching_native_candidates(package: Path, platform_tag: str) -> tuple[Path, ...]:
    """Return sorted package-local extension binaries for one normalized platform."""
    package = package.resolve()
    directory = package / "native" / platform_tag
    suffix = ".pyd" if platform_tag.startswith("windows-") else ".so"
    if not directory.is_dir():
        return ()
    candidates = []
    for candidate in directory.glob(f"bone_physics_native*{suffix}"):
        if candidate.is_symlink() or not candidate.is_file():
            continue
        resolved = candidate.resolve()
        if not resolved.is_relative_to(package):
            continue
        candidates.append(resolved)
    return tuple(sorted(candidates))


def _package_directory() -> Path:
    return Path(__file__).resolve().parent


def _platform_tag() -> str:
    system = platform.system().lower()
    machine = platform.machine().lower()
    architecture = {
        "amd64": "x86_64",
        "x86_64": "x86_64",
        "aarch64": "arm64",
        "arm64": "arm64",
    }.get(machine)
    operating_system = {
        "linux": "linux",
        "windows": "windows",
        "darwin": "macos",
    }.get(system)
    if operating_system is None or architecture is None:
        raise NativeUnavailableError(
            f"unsupported native solver platform {system}-{machine}"
        )
    return f"{operating_system}-{architecture}"


def _load_extension(path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location("bone_physics_native", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"could not create an extension-module spec for {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _reset_for_tests() -> None:
    global _loaded_module, _loaded_api
    with _load_lock:
        _loaded_module = None
        _loaded_api = None
