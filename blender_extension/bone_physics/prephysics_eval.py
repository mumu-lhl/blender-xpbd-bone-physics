"""Capture ordinary animation and collision inputs before physics output."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from .mesh_builder import CollisionMeshBatch, MeshSource, build_collision_surface
from .model import AnalyticColliderSpec, BoneSpec, CollisionSurfaceSpec, WorldSpec
from .scene_adapter import Pose


@dataclass(frozen=True, slots=True)
class TargetPose:
    body_id: int
    previous: Pose
    current: Pose


@dataclass(frozen=True, slots=True)
class ColliderPose:
    collider_id: int
    previous: Pose
    current: Pose


@dataclass(frozen=True, slots=True)
class PrePhysicsSnapshot:
    time: float
    target_poses: tuple[TargetPose, ...]
    collider_poses: tuple[ColliderPose, ...]
    object_to_simulation: tuple[float, ...]
    mesh_batches: tuple[CollisionMeshBatch, ...] = ()

    def __post_init__(self) -> None:
        if not math.isfinite(self.time):
            raise ValueError("pre-physics time must be finite")
        if len(self.object_to_simulation) != 16 or not all(
            math.isfinite(value) for value in self.object_to_simulation
        ):
            raise ValueError("object_to_simulation must be one finite 4x4 matrix")


class CaptureAdapter(Protocol):
    def set_scene_time(self, scene, seconds: float) -> None: ...

    def restore_animation_references(self, scene, references: dict[int, Pose]) -> None: ...

    # Blender pose channels are separate from the evaluated target poses that
    # physics writes back.  Adapters that can preserve those channels should
    # provide these optional methods.  The evaluator keeps the old reference
    # path as a compatibility fallback for lightweight/test adapters.
    def capture_animation_state(self, scene, world_spec: WorldSpec) -> dict[int, object]: ...

    def restore_animation_state(self, scene, state: dict[int, object]) -> None: ...

    def evaluate_prephysics(self, scene, depsgraph) -> None: ...

    def scene_time(self, scene) -> float: ...

    def object_to_simulation(
        self,
        scene,
        depsgraph,
        world_spec: WorldSpec,
    ) -> tuple[float, ...]: ...

    def target_pose(self, scene, depsgraph, bone: BoneSpec) -> Pose: ...

    def collider_pose(
        self,
        scene,
        depsgraph,
        collider: AnalyticColliderSpec,
    ) -> Pose: ...

    def collision_mesh_source(
        self,
        scene,
        depsgraph,
        surface: CollisionSurfaceSpec,
        object_to_simulation: tuple[float, ...],
    ) -> MeshSource: ...


class PrePhysicsEvaluator:
    def __init__(self, adapter: CaptureAdapter) -> None:
        self.adapter = adapter
        self._last_targets: dict[int, Pose] = {}
        self._last_colliders: dict[int, Pose] = {}
        self._last_mesh_sources: dict[int, MeshSource] = {}
        self._mesh_templates: dict[int, CollisionMeshBatch] = {}
        self._last_animation_state: dict[int, object] = {}

    def capture(self, scene, depsgraph, world_spec: WorldSpec) -> PrePhysicsSnapshot:
        return self._capture(
            scene,
            depsgraph,
            world_spec,
            restore_references=True,
        )

    def _capture(
        self,
        scene,
        depsgraph,
        world_spec: WorldSpec,
        *,
        restore_references: bool,
    ) -> PrePhysicsSnapshot:
        body_ids = {bone.id for bone in world_spec.bones}
        if restore_references:
            self._restore_clean_animation(scene, body_ids)
        self.adapter.evaluate_prephysics(scene, depsgraph)
        object_to_simulation = tuple(
            self.adapter.object_to_simulation(scene, depsgraph, world_spec)
        )

        targets = []
        next_targets = {}
        for bone in sorted(world_spec.bones, key=lambda item: item.id):
            current = self.adapter.target_pose(scene, depsgraph, bone)
            previous = self._last_targets.get(bone.id, current)
            targets.append(TargetPose(bone.id, previous, current))
            next_targets[bone.id] = current

        colliders = []
        next_colliders = {}
        for collider in sorted(
            world_spec.analytic_colliders,
            key=lambda item: item.id,
        ):
            current = self.adapter.collider_pose(scene, depsgraph, collider)
            previous = self._last_colliders.get(collider.id, current)
            colliders.append(ColliderPose(collider.id, previous, current))
            next_colliders[collider.id] = current

        mesh_batches = []
        next_mesh_sources = {}
        for surface in sorted(world_spec.collision_surfaces, key=lambda item: item.id):
            current = self.adapter.collision_mesh_source(
                scene,
                depsgraph,
                surface,
                object_to_simulation,
            )
            previous = self._last_mesh_sources.get(surface.id, current)
            batch = build_collision_surface(
                current,
                surface,
                previous=previous,
                template=self._mesh_templates.get(surface.id),
            )
            mesh_batches.append(batch)
            self._mesh_templates.setdefault(surface.id, batch)
            next_mesh_sources[surface.id] = current

        snapshot = PrePhysicsSnapshot(
            time=float(self.adapter.scene_time(scene)),
            target_poses=tuple(targets),
            collider_poses=tuple(colliders),
            object_to_simulation=object_to_simulation,
            mesh_batches=tuple(mesh_batches),
        )
        self._last_targets = next_targets
        self._last_colliders = next_colliders
        self._last_mesh_sources = next_mesh_sources
        self._capture_animation_state(scene, world_spec)
        return snapshot

    def capture_at(
        self,
        scene,
        depsgraph,
        world_spec: WorldSpec,
        sample_time: float,
    ) -> PrePhysicsSnapshot:
        if not math.isfinite(sample_time):
            raise ValueError("pre-physics sample time must be finite")
        body_ids = {bone.id for bone in world_spec.bones}
        # Restore local animation channels before changing Blender time.  A
        # child without its own keyframes otherwise inherits the previous
        # physics-deformed parent pose when frame_set() evaluates the new time.
        self._restore_clean_animation(scene, body_ids)
        self.adapter.set_scene_time(scene, sample_time)
        return self._capture(
            scene,
            depsgraph,
            world_spec,
            restore_references=False,
        )

    def restore_time(
        self,
        scene,
        depsgraph,
        world_spec: WorldSpec,
        time: float,
    ) -> None:
        if not math.isfinite(time):
            raise ValueError("pre-physics restore time must be finite")
        self._restore_clean_animation(scene, {bone.id for bone in world_spec.bones})
        self.adapter.set_scene_time(scene, time)
        self.adapter.evaluate_prephysics(scene, depsgraph)
        self.adapter.object_to_simulation(scene, depsgraph, world_spec)

    def save_state(
        self,
    ) -> tuple[
        dict[int, Pose],
        dict[int, Pose],
        dict[int, MeshSource],
        dict[int, CollisionMeshBatch],
        dict[int, object],
    ]:
        return (
            dict(self._last_targets),
            dict(self._last_colliders),
            dict(self._last_mesh_sources),
            dict(self._mesh_templates),
            dict(self._last_animation_state),
        )

    def restore_state(
        self,
        state: tuple[
            dict[int, Pose],
            dict[int, Pose],
            dict[int, MeshSource],
            dict[int, CollisionMeshBatch],
            dict[int, object],
        ],
    ) -> None:
        targets, colliders, meshes, templates, animation_state = state
        self._last_targets = dict(targets)
        self._last_colliders = dict(colliders)
        self._last_mesh_sources = dict(meshes)
        self._mesh_templates = dict(templates)
        self._last_animation_state = dict(animation_state)

    def reset(self) -> None:
        self._last_targets.clear()
        self._last_colliders.clear()
        self._last_mesh_sources.clear()
        self._mesh_templates.clear()
        # Keep authored local channels: timeline seeks must still restore a
        # clean pose before frame_set() evaluates the replay target.

    def _capture_animation_state(self, scene, world_spec: WorldSpec) -> None:
        capture = getattr(self.adapter, "capture_animation_state", None)
        if capture is None:
            return
        state = capture(scene, world_spec)
        self._last_animation_state = dict(state)

    def _restore_clean_animation(self, scene, body_ids: set[int]) -> None:
        state = {
            body_id: value
            for body_id, value in self._last_animation_state.items()
            if body_id in body_ids
        }
        restore = getattr(self.adapter, "restore_animation_state", None)
        if restore is not None and state:
            restore(scene, state)
            return
        references = {
            body_id: pose
            for body_id, pose in self._last_targets.items()
            if body_id in body_ids
        }
        if references:
            self.adapter.restore_animation_references(scene, references)
