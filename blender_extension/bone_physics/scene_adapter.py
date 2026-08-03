"""Thin duck-typed scene access and Blender transform conversion."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping

from .mesh_builder import MeshSource, mesh_source_from_evaluated
from .model import AnalyticColliderSpec, BoneSpec, CollisionSurfaceSpec, TargetKind


@dataclass(frozen=True, slots=True)
class Pose:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        translation = tuple(float(value) for value in self.translation)
        rotation = tuple(float(value) for value in self.rotation)
        if len(translation) != 3 or len(rotation) != 4:
            raise ValueError("pose requires three translation and four rotation values")
        if not all(math.isfinite(value) for value in translation + rotation):
            raise ValueError("pose values must all be finite")
        length = math.sqrt(sum(value * value for value in rotation))
        if length <= 1.0e-6:
            raise ValueError("pose quaternion must be nonzero")
        object.__setattr__(self, "translation", translation)
        object.__setattr__(
            self,
            "rotation",
            tuple(value / length for value in rotation),
        )


def validate_positive_scale(scale: Iterable[float]) -> tuple[float, float, float]:
    result = tuple(float(value) for value in scale)
    if len(result) != 3:
        raise ValueError("scale must contain exactly three values")
    if not all(math.isfinite(value) and value > 1.0e-8 for value in result):
        raise ValueError("scale must contain finite positive values")
    return result  # type: ignore[return-value]


def pose_from_matrix(matrix) -> Pose:
    determinant = float(matrix.to_3x3().determinant())
    if not math.isfinite(determinant) or determinant <= 0.0:
        raise ValueError("transform must have a finite positive determinant")
    validate_positive_scale(matrix.to_scale())
    translation = matrix.to_translation()
    rotation = matrix.to_quaternion()
    return Pose(tuple(translation), (rotation.x, rotation.y, rotation.z, rotation.w))


def flatten_matrix(matrix) -> tuple[float, ...]:
    values = tuple(float(matrix[row][column]) for row in range(4) for column in range(4))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("matrix values must all be finite")
    return values


class BlenderSceneAdapter:
    """Blender-specific operations imported lazily so pure modules stay importable."""

    def __init__(
        self,
        armature_name: str,
        bone_names: Mapping[int, str],
        collider_objects: Mapping[int, str] | None = None,
    ) -> None:
        if not armature_name:
            raise ValueError("armature object name must not be empty")
        self.armature_name = armature_name
        self.bone_names = dict(bone_names)
        self.collider_objects = dict(collider_objects or {})
        self._object_to_simulation = None
        self._simulation_to_world = None

    def restore_animation_references(self, scene, references: Mapping[int, Pose]) -> None:
        if not references:
            return
        armature = scene.objects[self.armature_name]
        for body_id, reference in references.items():
            staged = self.stage_pose(armature, body_id, reference)
            self.write_pose_channel(armature, body_id, staged)

    @staticmethod
    def set_scene_time(scene, seconds: float) -> None:
        if not math.isfinite(seconds):
            raise ValueError("scene time must be finite")
        fps = float(scene.render.fps) / float(scene.render.fps_base)
        if not math.isfinite(fps) or fps <= 0.0:
            raise ValueError("scene FPS must be finite and positive")
        frame_time = seconds * fps
        frame = math.floor(frame_time)
        scene.frame_set(frame, subframe=frame_time - frame)

    @staticmethod
    def evaluate_prephysics(scene, depsgraph) -> None:
        del scene
        depsgraph.update()

    @staticmethod
    def scene_time(scene) -> float:
        fps = float(scene.render.fps) / float(scene.render.fps_base)
        if not math.isfinite(fps) or fps <= 0.0:
            raise ValueError("scene FPS must be finite and positive")
        return (float(scene.frame_current) + float(scene.frame_subframe)) / fps

    def object_to_simulation(self, scene, depsgraph, world_spec) -> tuple[float, ...]:
        del world_spec
        armature = self._evaluated_object(scene, depsgraph, self.armature_name)
        self._object_to_simulation = armature.matrix_world.inverted_safe()
        self._simulation_to_world = armature.matrix_world.copy()
        return flatten_matrix(self._object_to_simulation)

    def target_pose(self, scene, depsgraph, bone: BoneSpec) -> Pose:
        armature = self._evaluated_object(scene, depsgraph, self.armature_name)
        simulation = self._require_object_to_simulation()
        binding = bone.target_binding
        if binding.kind is TargetKind.WORLD:
            return Pose(binding.local_anchor, (0.0, 0.0, 0.0, 1.0))
        if binding.kind is TargetKind.OBJECT:
            target = self._evaluated_object(scene, depsgraph, binding.name)
            return _with_local_anchor(
                pose_from_matrix(simulation @ target.matrix_world), binding.local_anchor
            )
        target_name = (
            self.bone_names.get(bone.id, bone.name)
            if binding.kind is TargetKind.OWN_ANIMATION
            else binding.name
        )
        pose_bone = armature.pose.bones[target_name]
        return _with_local_anchor(
            pose_from_matrix(simulation @ armature.matrix_world @ pose_bone.matrix),
            binding.local_anchor,
        )

    def collider_pose(
        self,
        scene,
        depsgraph,
        collider: AnalyticColliderSpec,
    ) -> Pose:
        object_name = self.collider_objects.get(collider.id, collider.source_object)
        if object_name is None:
            return Pose(collider.translation, collider.rotation)
        evaluated = self._evaluated_object(scene, depsgraph, object_name)
        return pose_from_matrix(self._require_object_to_simulation() @ evaluated.matrix_world)

    def collision_mesh_source(
        self,
        scene,
        depsgraph,
        surface: CollisionSurfaceSpec,
        object_to_simulation: tuple[float, ...],
    ) -> MeshSource:
        evaluated = self._evaluated_object(
            scene,
            depsgraph,
            surface.object_name,
        )
        return mesh_source_from_evaluated(
            evaluated,
            surface,
            object_to_simulation,
        )

    def stage_pose(self, armature, body_id: int, simulation_pose: Pose):
        mathutils = _mathutils()
        translation = mathutils.Matrix.Translation(simulation_pose.translation)
        quaternion = mathutils.Quaternion(
            (
                simulation_pose.rotation[3],
                simulation_pose.rotation[0],
                simulation_pose.rotation[1],
                simulation_pose.rotation[2],
            )
        )
        simulation_matrix = translation @ quaternion.to_matrix().to_4x4()
        simulation_to_world = self._simulation_to_world
        if simulation_to_world is None:
            raise RuntimeError("object_to_simulation must be captured before pose writing")
        armature_matrix = (
            armature.matrix_world.inverted_safe()
            @ simulation_to_world
            @ simulation_matrix
        )
        return ("pose", armature_matrix)

    def read_pose_channel(self, armature, body_id: int):
        pose_bone = armature.pose.bones[self.bone_names[body_id]]
        return ("basis", pose_bone.matrix_basis.copy())

    def write_pose_channel(self, armature, body_id: int, value) -> None:
        pose_bone = armature.pose.bones[self.bone_names[body_id]]
        kind, matrix = value
        if kind == "pose":
            pose_bone.matrix = matrix
        elif kind == "basis":
            pose_bone.matrix_basis = matrix
        else:
            raise ValueError(f"unsupported staged pose channel kind {kind!r}")

    def _require_object_to_simulation(self):
        if self._object_to_simulation is None:
            raise RuntimeError("object_to_simulation must be captured before pose reads")
        return self._object_to_simulation

    @staticmethod
    def _evaluated_object(scene, depsgraph, name: str):
        original = scene.objects.get(name)
        if original is None:
            raise ValueError(f"scene object {name!r} does not exist")
        return original.evaluated_get(depsgraph)


def _mathutils():
    try:
        import mathutils
    except ImportError as error:
        raise RuntimeError("Blender mathutils is required for scene transform access") from error
    return mathutils


def _with_local_anchor(pose: Pose, anchor: tuple[float, float, float]) -> Pose:
    x, y, z, w = pose.rotation
    ax, ay, az = anchor
    rotated = (
        (1.0 - 2.0 * (y * y + z * z)) * ax + 2.0 * (x * y - z * w) * ay + 2.0 * (x * z + y * w) * az,
        2.0 * (x * y + z * w) * ax + (1.0 - 2.0 * (x * x + z * z)) * ay + 2.0 * (y * z - x * w) * az,
        2.0 * (x * z - y * w) * ax + 2.0 * (y * z + x * w) * ay + (1.0 - 2.0 * (x * x + y * y)) * az,
    )
    return Pose(tuple(value + offset for value, offset in zip(pose.translation, rotated)), pose.rotation)
