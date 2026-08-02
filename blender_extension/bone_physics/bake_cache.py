"""Transactional, Blender-independent bake generations and replay checkpoints."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import shutil
import uuid


_GENERATION_RE = re.compile(r"^[0-9a-f]{32}$")


class CacheInvalidation(RuntimeError):
    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


@dataclass(frozen=True, slots=True)
class CheckpointRecord:
    frame: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class BakeManifest:
    generation: str
    fingerprint: str
    quality: str
    frame_count: int
    checkpoint_frames: tuple[int, ...]
    complete: bool


class BakeCache:
    def __init__(
        self,
        root: Path,
        fingerprint: str,
        *,
        quality: str,
        generation: str | None = None,
    ) -> None:
        self.root = Path(root)
        self.fingerprint = _fingerprint(fingerprint)
        self.quality = quality
        self.generation = generation
        self._staging: Path | None = None
        self.manifest: BakeManifest | None = None

    def begin(self) -> None:
        if self._staging is not None:
            raise RuntimeError("bake transaction is already open")
        self.root.mkdir(parents=True, exist_ok=True)
        staging = self.root / "generations" / f".staging-{uuid.uuid4().hex}"
        (staging / "frames").mkdir(parents=True, exist_ok=False)
        (staging / "checkpoints").mkdir(parents=True, exist_ok=False)
        self._staging = staging

    def write_frame(self, frame: int, poses) -> None:
        directory = self._require_staging() / "frames"
        _frame_number(frame)
        _atomic_write_json(directory / f"{frame:08d}.json", {"frame": frame, "poses": poses})

    def write_checkpoint(self, frame: int, payload: bytes) -> None:
        if self.quality == "approximate_preview":
            raise ValueError("approximate preview cannot write authoritative checkpoint")
        if not isinstance(payload, bytes):
            raise TypeError("checkpoint payload must be bytes")
        directory = self._require_staging() / "checkpoints"
        _frame_number(frame)
        _atomic_write_bytes(directory / f"{frame:08d}.bin", payload)

    def commit(self) -> BakeManifest:
        staging = self._require_staging()
        frames = sorted(staging.joinpath("frames").glob("*.json"))
        if not frames:
            raise ValueError("bake must contain at least one frame")
        checkpoints = tuple(
            int(path.stem) for path in staging.joinpath("checkpoints").glob("*.bin")
        )
        generation = uuid.uuid4().hex
        manifest = BakeManifest(
            generation=generation,
            fingerprint=self.fingerprint,
            quality=self.quality,
            frame_count=len(frames),
            checkpoint_frames=tuple(sorted(checkpoints)),
            complete=True,
        )
        _atomic_write_json(staging / "manifest.json", _manifest_dict(manifest))
        final = self.root / "generations" / generation
        os.replace(staging, final)
        _atomic_write_json(self.root / "current.json", {"generation": generation})
        self.generation = generation
        self.manifest = manifest
        self._staging = None
        return manifest

    def abort(self) -> None:
        if self._staging is not None:
            shutil.rmtree(self._staging, ignore_errors=True)
            self._staging = None

    def open(self) -> "BakeCache":
        loaded = self.open_current(self.root)
        if loaded is None:
            raise CacheInvalidation("missing_current")
        if loaded.manifest is None or loaded.manifest.fingerprint != self.fingerprint:
            raise CacheInvalidation("fingerprint_changed")
        return loaded

    def nearest_checkpoint(self, frame: int) -> CheckpointRecord | None:
        directory = self._generation_directory()
        candidates = []
        for path in directory.joinpath("checkpoints").glob("*.bin"):
            try:
                checkpoint_frame = int(path.stem)
            except ValueError:
                continue
            if checkpoint_frame <= frame:
                candidates.append((checkpoint_frame, path))
        if not candidates:
            return None
        checkpoint_frame, path = max(candidates, key=lambda item: item[0])
        return CheckpointRecord(checkpoint_frame, path.read_bytes())

    @classmethod
    def open_current(cls, root: Path) -> "BakeCache | None":
        root = Path(root)
        pointer = root / "current.json"
        if not pointer.is_file():
            return None
        try:
            generation = json.loads(pointer.read_text(encoding="utf-8"))["generation"]
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise CacheInvalidation("invalid_current_pointer") from error
        if not isinstance(generation, str) or _GENERATION_RE.fullmatch(generation) is None:
            raise CacheInvalidation("invalid_generation")
        directory = root / "generations" / generation
        manifest_path = directory / "manifest.json"
        if not directory.is_dir() or not manifest_path.is_file():
            raise CacheInvalidation("missing_generation")
        try:
            data = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest = _manifest_from_dict(data)
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise CacheInvalidation("invalid_manifest") from error
        if not manifest.complete:
            raise CacheInvalidation("incomplete_generation")
        cache = cls(
            root,
            manifest.fingerprint,
            quality=manifest.quality,
            generation=manifest.generation,
        )
        cache.manifest = manifest
        return cache

    def _require_staging(self) -> Path:
        if self._staging is None:
            raise RuntimeError("bake transaction is not open")
        return self._staging

    def _generation_directory(self) -> Path:
        if self.generation is None:
            raise RuntimeError("cache generation is not committed")
        if _GENERATION_RE.fullmatch(self.generation) is None:
            raise CacheInvalidation("invalid_generation")
        return self.root / "generations" / self.generation


def _fingerprint(value: str) -> str:
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError("fingerprint must be 64 lowercase hexadecimal characters")
    return value


def _frame_number(frame: int) -> None:
    if isinstance(frame, bool) or not isinstance(frame, int) or frame < 0:
        raise ValueError("frame must be a nonnegative integer")


def _atomic_write_bytes(path: Path, payload: bytes) -> None:
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_bytes(payload)
    os.replace(temporary, path)


def _atomic_write_json(path: Path, value: object) -> None:
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    temporary.write_text(
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _manifest_dict(manifest: BakeManifest) -> dict[str, object]:
    return {
        "generation": manifest.generation,
        "fingerprint": manifest.fingerprint,
        "quality": manifest.quality,
        "frame_count": manifest.frame_count,
        "checkpoint_frames": list(manifest.checkpoint_frames),
        "complete": manifest.complete,
    }


def _manifest_from_dict(data: dict[str, object]) -> BakeManifest:
    generation = data["generation"]
    if not isinstance(generation, str) or _GENERATION_RE.fullmatch(generation) is None:
        raise ValueError("invalid generation")
    fingerprint = _fingerprint(data["fingerprint"])
    quality = data["quality"]
    frame_count = data["frame_count"]
    checkpoint_frames = data["checkpoint_frames"]
    complete = data["complete"]
    if not isinstance(quality, str) or not isinstance(frame_count, int) or frame_count < 1:
        raise ValueError("invalid manifest counts")
    if not isinstance(checkpoint_frames, list) or not all(
        isinstance(frame, int) and frame >= 0 for frame in checkpoint_frames
    ):
        raise ValueError("invalid checkpoint frames")
    if not isinstance(complete, bool):
        raise ValueError("invalid completion flag")
    return BakeManifest(
        generation,
        fingerprint,
        quality,
        frame_count,
        tuple(checkpoint_frames),
        complete,
    )
