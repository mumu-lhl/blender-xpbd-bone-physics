"""Blender-independent authoring and compilation surface."""

from .model import (
    AnalyticColliderSpec,
    AnalyticShape,
    BoneSpec,
    ChainSpec,
    CrossLinkSpec,
    WorldSpec,
    TargetBinding,
    TargetKind,
)
from .authoring_adapter import SkeletonBone, compile_authoring
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
    the extension does not mutate the current scene.
    """
    from .operators import register as register_operators
    from .properties import register_properties
    from .ui import register as register_ui
    from .runtime_registry import register_timer

    register_properties()
    register_operators()
    register_ui()
    register_timer()


def unregister() -> None:
    """Blender extension lifecycle exit point."""
    from .operators import unregister as unregister_operators
    from .properties import unregister_properties
    from .ui import unregister as unregister_ui
    from .runtime_registry import unregister_timer

    unregister_timer()
    unregister_ui()
    unregister_operators()
    unregister_properties()


__all__ = [
    "AnalyticColliderSpec",
    "AnalyticShape",
    "BoneSpec",
    "TargetBinding",
    "TargetKind",
    "SkeletonBone",
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
    "compile_authoring",
    "get_native",
    "load_native",
    "register",
    "unregister",
]
