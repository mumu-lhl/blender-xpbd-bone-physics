"""Capture ordinary animation and collision inputs before physics output."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from .model import AnalyticColliderSpec, BoneSpec, WorldSpec
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

    def __post_init__(self) -> None:
        if not math.isfinite(self.time):
            raise ValueError("pre-physics time must be finite")
        if len(self.object_to_simulation) != 16 or not all(
            math.isfinite(value) for value in self.object_to_simulation
        ):
            raise ValueError("object_to_simulation must be one finite 4x4 matrix")


class CaptureAdapter(Protocol):
    def restore_animation_references(self, scene, references: dict[int, Pose]) -> None: ...

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


class PrePhysicsEvaluator:
    def __init__(self, adapter: CaptureAdapter) -> None:
        self.adapter = adapter
        self._last_targets: dict[int, Pose] = {}
        self._last_colliders: dict[int, Pose] = {}

    def capture(self, scene, depsgraph, world_spec: WorldSpec) -> PrePhysicsSnapshot:
        body_ids = {bone.id for bone in world_spec.bones}
        references = {
            body_id: pose
            for body_id, pose in self._last_targets.items()
            if body_id in body_ids
        }
        if references:
            self.adapter.restore_animation_references(scene, references)
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

        snapshot = PrePhysicsSnapshot(
            time=float(self.adapter.scene_time(scene)),
            target_poses=tuple(targets),
            collider_poses=tuple(colliders),
            object_to_simulation=object_to_simulation,
        )
        self._last_targets = next_targets
        self._last_colliders = next_colliders
        return snapshot

    def reset(self) -> None:
        self._last_targets.clear()
        self._last_colliders.clear()
