"""Transactional final-pose write boundary."""

from __future__ import annotations

from typing import Protocol, Sequence

from .scene_adapter import Pose


class WriteAdapter(Protocol):
    def stage_pose(self, armature, body_id: int, simulation_pose: Pose): ...

    def read_pose_channel(self, armature, body_id: int): ...

    def write_pose_channel(self, armature, body_id: int, value) -> None: ...


class PoseWriter:
    def __init__(self, adapter: WriteAdapter) -> None:
        self.adapter = adapter

    def apply(
        self,
        armature,
        body_ids: Sequence[int],
        simulation_poses: Sequence[Pose],
    ) -> None:
        ids = tuple(body_ids)
        poses = tuple(simulation_poses)
        if len(ids) != len(poses):
            raise ValueError("body IDs and simulation poses must have the same length")
        seen = set()
        for body_id in ids:
            if body_id in seen:
                raise ValueError(f"duplicate body ID {body_id}")
            seen.add(body_id)

        staged = tuple(
            self.adapter.stage_pose(armature, body_id, pose)
            for body_id, pose in zip(ids, poses)
        )
        previous = tuple(
            self.adapter.read_pose_channel(armature, body_id) for body_id in ids
        )
        written = 0
        try:
            for body_id, value in zip(ids, staged):
                self.adapter.write_pose_channel(armature, body_id, value)
                written += 1
        except Exception:
            for body_id, value in zip(ids[:written], previous[:written]):
                self.adapter.write_pose_channel(armature, body_id, value)
            raise

    def snapshot(self, armature, body_ids: Sequence[int]) -> tuple[object, ...]:
        return tuple(
            self.adapter.read_pose_channel(armature, body_id)
            for body_id in body_ids
        )

    def restore(
        self,
        armature,
        body_ids: Sequence[int],
        values: Sequence[object],
    ) -> None:
        ids = tuple(body_ids)
        stored = tuple(values)
        if len(ids) != len(stored):
            raise ValueError("body IDs and stored pose channels must have the same length")
        first_error = None
        for body_id, value in zip(ids, stored):
            try:
                self.adapter.write_pose_channel(armature, body_id, value)
            except Exception as error:
                if first_error is None:
                    first_error = error
        if first_error is not None:
            raise first_error
