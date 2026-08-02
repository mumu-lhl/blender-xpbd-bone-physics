"""Blender-independent authoring and compilation surface."""

from .model import (
    AnalyticColliderSpec,
    AnalyticShape,
    BoneSpec,
    ChainSpec,
    CrossLinkSpec,
    WorldSpec,
)
from .native import (
    NativeUnavailableError,
    NativeVersionError,
    get_native,
    load_native,
)
from .parameter_builder import NativeBuildBatch, compile_world_spec
from .pose_writer import PoseWriter
from .prephysics_eval import PrePhysicsEvaluator, PrePhysicsSnapshot
from .scene_adapter import BlenderSceneAdapter, Pose
from .runtime import NativeRuntimeBridge, ReentryGuard, RuntimeWorld
from .timeline import TimelineAction, TimelineActionKind, TimelineState, TimelineTime


def register() -> None:
    """Blender extension lifecycle entry point.

    Runtime worlds are attached explicitly by the authoring layer, so importing
    or enabling the extension does not mutate the current scene.
    """


def unregister() -> None:
    """Blender extension lifecycle exit point."""


__all__ = [
    "AnalyticColliderSpec",
    "AnalyticShape",
    "BoneSpec",
    "ChainSpec",
    "CrossLinkSpec",
    "NativeBuildBatch",
    "NativeUnavailableError",
    "NativeVersionError",
    "Pose",
    "PoseWriter",
    "PrePhysicsEvaluator",
    "PrePhysicsSnapshot",
    "BlenderSceneAdapter",
    "NativeRuntimeBridge",
    "ReentryGuard",
    "RuntimeWorld",
    "TimelineAction",
    "TimelineActionKind",
    "TimelineState",
    "TimelineTime",
    "WorldSpec",
    "compile_world_spec",
    "get_native",
    "load_native",
    "register",
    "unregister",
]
