"""Static runtime-control registration smoke; run with Blender by the caller."""

from blender_extension.bone_physics.operators.runtime import CLASSES as RUNTIME_OPERATORS
from blender_extension.bone_physics.prephysics_eval import PrePhysicsSnapshot, TargetPose
from blender_extension.bone_physics.runtime_registry import RuntimeRegistry
from blender_extension.bone_physics.scene_adapter import Pose
from blender_extension.bone_physics.ui.panels import BONE_PHYSICS_PT_runtime


assert {operator.bl_idname for operator in RUNTIME_OPERATORS} == {
    "bone_physics.runtime_enable",
    "bone_physics.runtime_pause",
    "bone_physics.runtime_isolate",
    "bone_physics.runtime_validate_rebuild",
    "bone_physics.runtime_reset",
}
assert BONE_PHYSICS_PT_runtime.bl_idname == "BONE_PHYSICS_PT_runtime"


class _Armature:
    def as_pointer(self):
        return 2


class _Scene:
    def as_pointer(self):
        return 1


class _Adapter:
    def __init__(self):
        self.channels = {10: "physical-before", 20: "parent-before", 30: "unrelated-before"}

    def restore_animation_references(self, _scene, references):
        for body_id, pose in references.items():
            self.channels[body_id] = pose


class _Evaluator:
    def __init__(self, adapter):
        self.adapter = adapter

    def capture(self, _scene, _depsgraph, _spec):
        return PrePhysicsSnapshot(
            0.0,
            (TargetPose(10, _pose(), _pose()),),
            (),
            (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        )


class _World:
    def __init__(self, adapter):
        self.evaluator = _Evaluator(adapter)

    def destroy(self):
        pass


class _Factory:
    def __init__(self, adapter):
        self.adapter = adapter

    def create(self, _scene, _armature, _spec):
        return _World(self.adapter)


class _Context:
    def __init__(self, scene, armature):
        self.scene = scene
        self.object = armature

    def evaluated_depsgraph_get(self):
        return object()


def _pose():
    return Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


adapter = _Adapter()
registry = RuntimeRegistry(factory=_Factory(adapter))
scene, armature = _Scene(), _Armature()
registry.rebuild(scene, armature, object())
registry.reset_configured_bones(_Context(scene, armature))
assert adapter.channels[10] == _pose()
assert adapter.channels[20] == "parent-before"
assert adapter.channels[30] == "unrelated-before"
