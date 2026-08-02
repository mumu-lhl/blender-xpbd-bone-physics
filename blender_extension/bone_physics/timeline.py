"""Pure fixed-time timeline transitions for preview, seek, and replay."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class TimelineActionKind(Enum):
    ADVANCE = "advance"
    RESTORE_AND_REPLAY = "restore_and_replay"
    RESET = "reset"
    NO_OP = "no_op"


@dataclass(frozen=True, slots=True)
class TimelineTime:
    seconds: float
    fps: float
    approximate_preview: bool = False

    def __post_init__(self) -> None:
        if not math.isfinite(self.seconds):
            raise ValueError("timeline time must be finite")
        if not math.isfinite(self.fps) or self.fps <= 0.0:
            raise ValueError("timeline FPS must be finite and positive")


@dataclass(frozen=True, slots=True)
class Checkpoint:
    identifier: str
    time: float
    configuration_hash: str
    fps: float
    fixed_dt: float
    authoritative: bool = True

    def __post_init__(self) -> None:
        if not self.identifier:
            raise ValueError("checkpoint identifier must not be empty")
        if not self.configuration_hash:
            raise ValueError("checkpoint configuration hash must not be empty")
        for name, value in (
            ("checkpoint time", self.time),
            ("checkpoint FPS", self.fps),
            ("checkpoint fixed_dt", self.fixed_dt),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.fps <= 0.0 or self.fixed_dt <= 0.0:
            raise ValueError("checkpoint FPS and fixed_dt must be positive")


@dataclass(frozen=True, slots=True)
class CacheIndex:
    checkpoints: tuple[Checkpoint, ...] = ()

    def __post_init__(self) -> None:
        ordered = tuple(sorted(self.checkpoints, key=lambda item: (item.time, item.identifier)))
        object.__setattr__(self, "checkpoints", ordered)

    def nearest_compatible(
        self,
        time: float,
        *,
        configuration_hash: str,
        fps: float,
        fixed_dt: float,
    ) -> Checkpoint | None:
        compatible = (
            checkpoint
            for checkpoint in self.checkpoints
            if checkpoint.time <= time
            and checkpoint.configuration_hash == configuration_hash
            and _close(checkpoint.fps, fps)
            and _close(checkpoint.fixed_dt, fixed_dt)
            and checkpoint.authoritative
        )
        return max(compatible, key=lambda item: item.time, default=None)


@dataclass(frozen=True, slots=True)
class TimelineAction:
    kind: TimelineActionKind
    fixed_times: tuple[float, ...]
    checkpoint: Checkpoint | None
    authoritative: bool
    reason: str


class TimelineState:
    def __init__(
        self,
        *,
        start_time: float,
        fixed_dt: float,
        configuration_hash: str,
    ) -> None:
        if not math.isfinite(start_time):
            raise ValueError("timeline start time must be finite")
        if not math.isfinite(fixed_dt) or fixed_dt <= 0.0:
            raise ValueError("fixed_dt must be finite and positive")
        if not configuration_hash:
            raise ValueError("configuration hash must not be empty")
        self.start_time = start_time
        self.fixed_dt = fixed_dt
        self.configuration_hash = configuration_hash
        self._last: TimelineTime | None = None
        self._last_tick = 0

    def observe(
        self,
        time: TimelineTime,
        cache_index: CacheIndex,
    ) -> TimelineAction:
        target_tick = self._tick_at_or_before(time.seconds)
        authoritative = not time.approximate_preview
        previous = self._last
        if previous is None:
            action = TimelineAction(
                kind=TimelineActionKind.RESET,
                fixed_times=self._fixed_times(0, target_tick),
                checkpoint=None,
                authoritative=authoritative,
                reason="initial observation",
            )
        elif _close(time.seconds, previous.seconds):
            action = TimelineAction(
                kind=TimelineActionKind.NO_OP,
                fixed_times=(),
                checkpoint=None,
                authoritative=authoritative,
                reason="unchanged time",
            )
        elif not _close(time.fps, previous.fps):
            action = TimelineAction(
                kind=TimelineActionKind.RESET,
                fixed_times=self._fixed_times(0, target_tick),
                checkpoint=None,
                authoritative=authoritative,
                reason="output FPS changed",
            )
        elif self._is_sequential_forward(previous, time):
            action = TimelineAction(
                kind=TimelineActionKind.ADVANCE,
                fixed_times=self._fixed_times(self._last_tick, target_tick),
                checkpoint=None,
                authoritative=authoritative,
                reason="sequential forward playback",
            )
        else:
            checkpoint = cache_index.nearest_compatible(
                time.seconds,
                configuration_hash=self.configuration_hash,
                fps=time.fps,
                fixed_dt=self.fixed_dt,
            )
            if checkpoint is not None:
                checkpoint_tick = self._tick_at_or_before(checkpoint.time)
                aligned_time = self.start_time + checkpoint_tick * self.fixed_dt
                if not _close(checkpoint.time, aligned_time):
                    checkpoint = None
            if checkpoint is None:
                action = TimelineAction(
                    kind=TimelineActionKind.RESET,
                    fixed_times=self._fixed_times(0, target_tick),
                    checkpoint=None,
                    authoritative=authoritative,
                    reason="seek without a compatible checkpoint",
                )
            else:
                checkpoint_tick = self._tick_at_or_before(checkpoint.time)
                action = TimelineAction(
                    kind=TimelineActionKind.RESTORE_AND_REPLAY,
                    fixed_times=self._fixed_times(checkpoint_tick, target_tick),
                    checkpoint=checkpoint,
                    authoritative=authoritative,
                    reason="seek from nearest compatible checkpoint",
                )
        self._last = time
        self._last_tick = target_tick
        return action

    def reset_observation(self) -> None:
        self._last = None
        self._last_tick = 0

    def save_observation(self) -> tuple[TimelineTime | None, int]:
        return self._last, self._last_tick

    def restore_observation(
        self,
        saved: tuple[TimelineTime | None, int],
    ) -> None:
        self._last, self._last_tick = saved

    def _tick_at_or_before(self, time: float) -> int:
        elapsed = time - self.start_time
        if elapsed <= 0.0:
            return 0
        return math.floor(elapsed / self.fixed_dt + 1.0e-7)

    def _fixed_times(self, after_tick: int, through_tick: int) -> tuple[float, ...]:
        if through_tick <= after_tick:
            return ()
        return tuple(
            self.start_time + tick * self.fixed_dt
            for tick in range(after_tick + 1, through_tick + 1)
        )

    @staticmethod
    def _is_sequential_forward(previous: TimelineTime, current: TimelineTime) -> bool:
        delta = current.seconds - previous.seconds
        frame_seconds = 1.0 / current.fps
        return delta > 0.0 and delta <= frame_seconds * (1.0 + 1.0e-5)


def _close(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=1.0e-9, abs_tol=1.0e-12)
