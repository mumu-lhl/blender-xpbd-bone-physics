"""Guarded frame-change orchestration with per-world failure isolation."""

from __future__ import annotations

from contextlib import contextmanager
from array import array
from dataclasses import dataclass
from typing import Callable, Protocol, Sequence

from .parameter_builder import NativeBuildBatch
from .pose_writer import PoseWriter
from .prephysics_eval import PrePhysicsEvaluator, PrePhysicsSnapshot
from .scene_adapter import Pose
from .timeline import CacheIndex, TimelineAction, TimelineActionKind, TimelineState, TimelineTime


class RuntimeBridge(Protocol):
    def prepare(self, handle: int, action: TimelineAction) -> int: ...

    def set_snapshot(self, handle: int, snapshot: PrePhysicsSnapshot) -> None: ...

    def solve(self, handle: int, action: TimelineAction) -> None: ...

    def read_poses(self, handle: int, body_ids: Sequence[int]) -> Sequence[Pose]: ...

    def pause(self, handle: int, error: Exception) -> None: ...

    def resume(self, handle: int) -> None: ...


@dataclass(frozen=True, slots=True)
class RuntimeResult:
    action: TimelineAction | None
    executed: bool
    paused: bool
    reentrant: bool = False
    error: Exception | None = None


class ReentryGuard:
    def __init__(self) -> None:
        self._active_scenes: set[int] = set()

    @contextmanager
    def enter(self, scene):
        key = id(scene)
        if key in self._active_scenes:
            yield False
            return
        self._active_scenes.add(key)
        try:
            yield True
        finally:
            self._active_scenes.remove(key)


class RuntimeWorld:
    def __init__(
        self,
        *,
        handle: int,
        world_spec,
        armature,
        body_ids: Sequence[int],
        timeline: TimelineState,
        cache_index: CacheIndex,
        evaluator: PrePhysicsEvaluator,
        bridge: RuntimeBridge,
        pose_writer: PoseWriter,
        time_source: Callable[[object], TimelineTime],
    ) -> None:
        self.handle = handle
        self.world_spec = world_spec
        self.armature = armature
        self.body_ids = tuple(body_ids)
        if tuple(sorted(set(self.body_ids))) != self.body_ids:
            raise ValueError("runtime body IDs must be unique and in ascending order")
        self.timeline = timeline
        self.cache_index = cache_index
        self.evaluator = evaluator
        self.bridge = bridge
        self.pose_writer = pose_writer
        self.time_source = time_source
        self.guard = ReentryGuard()
        self.paused = False
        self.last_error: Exception | None = None

    def on_frame_change(self, scene, depsgraph) -> RuntimeResult:
        if self.paused:
            return RuntimeResult(None, executed=False, paused=True, error=self.last_error)
        with self.guard.enter(scene) as entered:
            if not entered:
                return RuntimeResult(
                    None,
                    executed=False,
                    paused=False,
                    reentrant=True,
                )
            saved_timeline = self.timeline.save_observation()
            action = self.timeline.observe(
                self.time_source(scene),
                self.cache_index,
            )
            if action.kind is TimelineActionKind.NO_OP:
                return RuntimeResult(action, executed=False, paused=False)
            previous_channels = None
            try:
                previous_channels = self.pose_writer.snapshot(
                    self.armature,
                    self.body_ids,
                )
                self.handle = self.bridge.prepare(self.handle, action)
                snapshot = self.evaluator.capture(scene, depsgraph, self.world_spec)
                self.bridge.set_snapshot(self.handle, snapshot)
                self.bridge.solve(self.handle, action)
                poses = tuple(self.bridge.read_poses(self.handle, self.body_ids))
                if len(poses) != len(self.body_ids):
                    raise RuntimeError(
                        "native bridge returned a pose count that does not match body IDs"
                    )
                self.pose_writer.apply(
                    self.armature,
                    self.body_ids,
                    poses,
                )
            except Exception as error:
                self.timeline.restore_observation(saved_timeline)
                if previous_channels is not None:
                    try:
                        self.pose_writer.restore(
                            self.armature,
                            self.body_ids,
                            previous_channels,
                        )
                    except Exception:
                        pass
                self.paused = True
                self.last_error = error
                try:
                    self.bridge.pause(self.handle, error)
                except Exception:
                    pass
                return RuntimeResult(
                    action,
                    executed=False,
                    paused=True,
                    error=error,
                )
            self.last_error = None
            return RuntimeResult(action, executed=True, paused=False)

    def resume(self) -> None:
        resume = getattr(self.bridge, "resume", None)
        if resume is not None:
            resume(self.handle)
        self.paused = False
        self.last_error = None


class NativeRuntimeBridge:
    """Coarse adapter from runtime snapshots to the validated PyO3 batch API."""

    def __init__(
        self,
        native,
        compiled: NativeBuildBatch,
        bones,
        *,
        fixed_dt: float,
        quality: str = "deterministic_preview",
        restore_checkpoint: Callable[[object, int], int] | None = None,
    ) -> None:
        self.native = native
        self.compiled = compiled
        self.bones = {bone.id: bone for bone in bones}
        self.fixed_dt = fixed_dt
        self.quality = quality
        self.restore_checkpoint = restore_checkpoint
        self.last_snapshot_time: dict[int, float] = {}
        self.last_diagnostics: dict[int, object] = {}

    def create_world(self) -> int:
        handle = self.native.build_world(
            self.compiled.config,
            self.compiled.bodies,
            self.compiled.constraints,
            self.compiled.filters,
        )
        self.native.set_analytic_colliders(
            handle,
            self.compiled.analytic_colliders,
        )
        return handle

    def prepare(self, handle: int, action: TimelineAction) -> int:
        if action.kind is TimelineActionKind.RESET:
            replacement = self.create_world()
            try:
                self.native.destroy_world(handle)
            except Exception:
                self.native.destroy_world(replacement)
                raise
            self.last_snapshot_time.pop(handle, None)
            return replacement
        if action.kind is TimelineActionKind.RESTORE_AND_REPLAY:
            if self.restore_checkpoint is None:
                raise RuntimeError(
                    "timeline requested checkpoint restore before a checkpoint backend was configured"
                )
            replacement = self.restore_checkpoint(action.checkpoint, handle)
            self.last_snapshot_time.pop(handle, None)
            return replacement
        return handle

    def set_snapshot(self, handle: int, snapshot: PrePhysicsSnapshot) -> None:
        target_values = array("f")
        target_ids = array("I")
        targets = {target.body_id: target for target in snapshot.target_poses}
        for body_id in sorted(targets):
            target = targets[body_id]
            bone = self.bones[body_id]
            target_ids.append(body_id)
            target_values.extend(target.previous.translation)
            target_values.extend(target.previous.rotation)
            target_values.extend(target.current.translation)
            target_values.extend(target.current.rotation)
            target_values.extend(
                (bone.translation_inertia, bone.rotation_inertia)
            )
        self.native.set_animation_targets(
            handle,
            {
                "body_ids": target_ids,
                "values": target_values,
                "columns": 16,
            },
        )

        analytic = _copy_batch(self.compiled.analytic_colliders)
        ids = analytic["ids"]
        values = analytic["values"]
        assert isinstance(ids, array)
        assert isinstance(values, array)
        previous_time = self.last_snapshot_time.get(handle)
        elapsed = (
            snapshot.time - previous_time
            if previous_time is not None
            else 0.0
        )
        for collider in snapshot.collider_poses:
            try:
                index = ids.index(collider.collider_id)
            except ValueError as error:
                raise ValueError(
                    f"snapshot references unknown analytic collider {collider.collider_id}"
                ) from error
            start = index * 17
            values[start : start + 3] = array("f", collider.current.translation)
            values[start + 3 : start + 7] = array("f", collider.current.rotation)
            if elapsed > 0.0:
                velocity = tuple(
                    (current - previous) / elapsed
                    for previous, current in zip(
                        collider.previous.translation,
                        collider.current.translation,
                    )
                )
                values[start + 7 : start + 10] = array("f", velocity)
        self.native.set_analytic_colliders(handle, analytic)
        self.last_snapshot_time[handle] = snapshot.time

    def solve(self, handle: int, action: TimelineAction) -> None:
        if not action.fixed_times:
            return
        quality = "approximate_preview" if not action.authoritative else self.quality
        summary = self.native.advance(
            handle,
            len(action.fixed_times) * self.fixed_dt,
            quality,
        )
        self.last_diagnostics[handle] = summary
        if summary["rolled_back"]:
            raise RuntimeError("native solver rolled back and paused the world")

    def read_poses(self, handle: int, body_ids: Sequence[int]) -> tuple[Pose, ...]:
        values = array("f", [0.0] * (len(body_ids) * 7))
        self.native.read_transforms(handle, values)
        return tuple(
            Pose(
                tuple(values[index * 7 : index * 7 + 3]),
                tuple(values[index * 7 + 3 : index * 7 + 7]),
            )
            for index in range(len(body_ids))
        )

    def pause(self, handle: int, error: Exception) -> None:
        self.last_diagnostics[handle] = {
            "runtime_error": f"{type(error).__name__}: {error}",
            "native": self.native.read_diagnostics(handle),
        }

    def resume(self, handle: int) -> None:
        self.native.resume_world(handle)


def _copy_batch(batch: dict[str, object]) -> dict[str, object]:
    return {
        key: array(value.typecode, value) if isinstance(value, array) else value
        for key, value in batch.items()
    }
