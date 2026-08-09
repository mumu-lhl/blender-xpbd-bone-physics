"""Guarded frame-change orchestration with per-world failure isolation."""

from __future__ import annotations

from contextlib import contextmanager
from array import array
from dataclasses import dataclass
import math
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

    def save_state(self, handle: int) -> object: ...

    def restore_state(self, handle: int, state: object) -> int: ...

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
        self._last_valid_channels: tuple[object, ...] | None = None
        self._last_mesh_batches: tuple[object, ...] = ()
        self._destroyed = False

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
            observed_time = self.time_source(scene)
            saved_timeline = self.timeline.save_observation()
            saved_evaluator = self.evaluator.save_state()
            action = self.timeline.observe(observed_time, self.cache_index)
            if action.kind is TimelineActionKind.NO_OP:
                return RuntimeResult(action, executed=False, paused=False)
            previous_channels = self._last_valid_channels
            native_state = None
            try:
                if previous_channels is None:
                    previous_channels = self.pose_writer.snapshot(
                        self.armature,
                        self.body_ids,
                    )
                self.handle = self.bridge.prepare(self.handle, action)
                save_state = getattr(self.bridge, "save_state", None)
                if save_state is not None:
                    native_state = save_state(self.handle)
                if action.kind is TimelineActionKind.RESET:
                    self.evaluator.reset()
                elif action.kind is TimelineActionKind.RESTORE_AND_REPLAY:
                    if action.checkpoint is None:
                        raise RuntimeError(
                            "restore-and-replay action is missing its checkpoint"
                        )
                    self.evaluator.reset()
                    self.evaluator.capture_at(
                        scene,
                        depsgraph,
                        self.world_spec,
                        action.checkpoint.time,
                    )
                sample_times = action.fixed_times
                if (
                    not sample_times
                    and action.kind is not TimelineActionKind.ADVANCE
                ):
                    sample_times = (observed_time.seconds,)
                for sample_time in sample_times:
                    snapshot = self.evaluator.capture_at(
                        scene,
                        depsgraph,
                        self.world_spec,
                        sample_time,
                    )
                    self._last_mesh_batches = tuple(snapshot.mesh_batches)
                    self.bridge.set_snapshot(self.handle, snapshot)
                    self.bridge.solve(
                        self.handle,
                        _single_step_action(action, sample_time),
                    )
                if not sample_times or not math.isclose(
                    sample_times[-1],
                    observed_time.seconds,
                    rel_tol=1.0e-9,
                    abs_tol=1.0e-12,
                ):
                    self.evaluator.restore_time(
                        scene,
                        depsgraph,
                        self.world_spec,
                        observed_time.seconds,
                    )
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
                self._last_valid_channels = self.pose_writer.snapshot(
                    self.armature,
                    self.body_ids,
                )
            except Exception as error:
                self.timeline.restore_observation(saved_timeline)
                self.evaluator.restore_state(saved_evaluator)
                try:
                    self.evaluator.restore_time(
                        scene,
                        depsgraph,
                        self.world_spec,
                        observed_time.seconds,
                    )
                except Exception:
                    pass
                if previous_channels is not None:
                    try:
                        self.pose_writer.restore(
                            self.armature,
                            self.body_ids,
                            previous_channels,
                        )
                    except Exception:
                        pass
                if native_state is not None:
                    restore_state = getattr(self.bridge, "restore_state", None)
                    if restore_state is not None:
                        try:
                            restored_handle = restore_state(self.handle, native_state)
                            if restored_handle is not None:
                                self.handle = restored_handle
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

    def reset_to_initial(self, scene, depsgraph) -> None:
        """Write the freshly built native rest pose to the physical bones."""
        object_to_simulation = getattr(
            self.evaluator.adapter, "object_to_simulation", None
        )
        if object_to_simulation is None:
            raise RuntimeError("scene adapter cannot initialize pose writing")
        object_to_simulation(scene, depsgraph, self.world_spec)
        self.evaluator.reset()
        poses = tuple(self.bridge.read_poses(self.handle, self.body_ids))
        if len(poses) != len(self.body_ids):
            raise RuntimeError(
                "native bridge returned a pose count that does not match body IDs"
            )
        self.pose_writer.apply(self.armature, self.body_ids, poses)
        self._last_valid_channels = self.pose_writer.snapshot(
            self.armature, self.body_ids
        )

    def destroy(self) -> None:
        """Release the native handle exactly once after pose restoration."""
        if self._destroyed:
            return
        destroy = getattr(self.bridge, "destroy_world", None)
        if destroy is not None:
            destroy(self.handle)
        self._destroyed = True


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
        effective_bones = compiled.effective_bones or tuple(bones)
        self.bones = {bone.id: bone for bone in effective_bones}
        self.fixed_dt = fixed_dt
        self.quality = quality
        self.restore_checkpoint = restore_checkpoint
        self.last_snapshot_time: dict[int, float] = {}
        self.last_diagnostics: dict[int, object] = {}
        self.mesh_topologies: dict[tuple[int, int], tuple[str, int]] = {}

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
            self._forget_meshes(handle)
            return replacement
        if action.kind is TimelineActionKind.RESTORE_AND_REPLAY:
            if self.restore_checkpoint is None:
                raise RuntimeError(
                    "timeline requested checkpoint restore before a checkpoint backend was configured"
                )
            replacement = self.restore_checkpoint(action.checkpoint, handle)
            self.last_snapshot_time.pop(handle, None)
            self._forget_meshes(handle)
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
        for mesh in snapshot.mesh_batches:
            key = (handle, mesh.mesh_id)
            installed = self.mesh_topologies.get(key)
            if installed is None or installed[0] != mesh.topology_fingerprint:
                topology_hash = self.native.set_mesh_topology(
                    handle,
                    mesh.mesh_id,
                    _mesh_topology_batch(mesh),
                )
                self.mesh_topologies[key] = (
                    mesh.topology_fingerprint,
                    topology_hash,
                )
            else:
                topology_hash = installed[1]
            update = self.native.update_mesh_vertices(
                handle,
                mesh.mesh_id,
                _vertex_buffer(mesh.previous_vertices),
                _vertex_buffer(mesh.current_vertices),
                topology_hash,
            )
            if update != "refit":
                raise RuntimeError(
                    f"mesh {mesh.mesh_id} update unexpectedly returned {update!r}"
                )
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

    def save_state(self, handle: int) -> tuple[object, ...]:
        return (
            self.native.encode_checkpoint(handle, "authoritative_bake"),
            self.native.world_fingerprint(handle),
            self.last_snapshot_time.get(handle),
            tuple(
                (mesh_id, value)
                for (saved_handle, mesh_id), value in self.mesh_topologies.items()
                if saved_handle == handle
            ),
            self.last_diagnostics.get(handle),
        )

    def restore_state(self, handle: int, state: object) -> int:
        (
            checkpoint,
            fingerprint,
            snapshot_time,
            mesh_entries,
            diagnostics,
        ) = state
        self.native.restore_checkpoint(handle, checkpoint, fingerprint)
        if snapshot_time is None:
            self.last_snapshot_time.pop(handle, None)
        else:
            self.last_snapshot_time[handle] = snapshot_time
        self._forget_meshes(handle)
        for mesh_id, value in mesh_entries:
            self.mesh_topologies[(handle, mesh_id)] = value
        if diagnostics is None:
            self.last_diagnostics.pop(handle, None)
        else:
            self.last_diagnostics[handle] = diagnostics
        return handle

    def pause(self, handle: int, error: Exception) -> None:
        self.last_diagnostics[handle] = {
            "runtime_error": f"{type(error).__name__}: {error}",
            "native": self.native.read_diagnostics(handle),
        }

    def resume(self, handle: int) -> None:
        self.native.resume_world(handle)

    def _forget_meshes(self, handle: int) -> None:
        stale = [key for key in self.mesh_topologies if key[0] == handle]
        for key in stale:
            del self.mesh_topologies[key]


def _copy_batch(batch: dict[str, object]) -> dict[str, object]:
    return {
        key: array(value.typecode, value) if isinstance(value, array) else value
        for key, value in batch.items()
    }


def _mesh_topology_batch(mesh) -> dict[str, object]:
    ignore_body_ids = array("I")
    ignore_region_offsets = array("I", [0])
    ignore_region_ids = array("I")
    for body_id, regions in mesh.body_region_ignores:
        ignore_body_ids.append(body_id)
        ignore_region_ids.extend(regions)
        ignore_region_offsets.append(len(ignore_region_ids))
    return {
        "vertex_count": mesh.topology.vertex_count,
        "triangles": array(
            "I",
            (index for triangle in mesh.topology.triangles for index in triangle),
        ),
        "regions": array("I", mesh.regions),
        "vertices": _vertex_buffer(mesh.previous_vertices),
        "thickness": mesh.thickness,
        "closed_hint": mesh.topology.closed_hint,
        "ignore_body_ids": ignore_body_ids,
        "ignore_region_offsets": ignore_region_offsets,
        "ignore_region_ids": ignore_region_ids,
    }


def _vertex_buffer(vertices) -> array:
    return array("f", (value for vertex in vertices for value in vertex))


def _single_step_action(
    action: TimelineAction,
    sample_time: float,
) -> TimelineAction:
    if not action.fixed_times:
        return action
    return TimelineAction(
        kind=action.kind,
        fixed_times=(sample_time,),
        checkpoint=action.checkpoint,
        authoritative=action.authoritative,
        reason=action.reason,
    )
