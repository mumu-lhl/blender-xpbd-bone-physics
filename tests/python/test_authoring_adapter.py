import unittest

from blender_extension.bone_physics.authoring import (
    ArmatureAuthoring,
    BoneOverride,
    ChainAuthoring,
    EndpointParameters,
    PhysicsSystemAuthoring,
)
from blender_extension.bone_physics.authoring_adapter import (
    SkeletonBone,
    compile_authoring,
)
from blender_extension.bone_physics.model import AngularLimitMode
from blender_extension.bone_physics.model import (
    ChainField,
    CurveInterpolation,
    TargetBinding,
    TargetKind,
)
from blender_extension.bone_physics.parameter_builder import compile_world_spec


def skeleton():
    return (
        SkeletonBone("root", "hair_root", None, (0, 0, 0), (0, 1, 0)),
        SkeletonBone("mid", "hair_mid", "root", (0, 1, 0), (0, 3, 0)),
        SkeletonBone("tip", "hair_tip", "mid", (0, 3, 0), (0, 4, 0)),
    )


class AuthoringAdapterTests(unittest.TestCase):
    def test_compile_preserves_full_hermite_curves_and_applies_overrides_after_sampling(self):
        config = ArmatureAuthoring(
            id="armature",
            physics_systems=(
                PhysicsSystemAuthoring(
                    id="hair",
                    stable_id=10,
                    head=EndpointParameters(
                        gravity_scale=1.0, radius=0.2, inertia_scale=(1.0, 2.0, 3.0),
                        joint_compliance=0.01, angular_compliance=0.02,
                        interpolation=CurveInterpolation.CUBIC_HERMITE,
                    ),
                    tail=EndpointParameters(
                        gravity_scale=0.0, radius=0.4, inertia_scale=(5.0, 6.0, 7.0),
                        joint_compliance=0.05, angular_compliance=0.06,
                    ),
                    chains=(
                        ChainAuthoring(
                            "hair-chain",
                            20,
                            ("root", "mid", "tip"),
                            overrides=(
                                BoneOverride(
                                    "tip",
                                    (("gravity_scale", 0.25), ("angular_limit_mode", AngularLimitMode.XYZ)),
                                ),
                            ),
                        ),
                    ),
                ),
            ),
        )

        world = compile_authoring(config, skeleton(), ())
        chain = world.chains[0]
        fields = {item.field: item.curve for item in chain.field_curves}
        self.assertEqual(len(fields), len(ChainField))
        self.assertEqual(fields[ChainField.GRAVITY_SCALE].interpolation, CurveInterpolation.CUBIC_HERMITE)
        self.assertEqual(fields[ChainField.JOINT_COMPLIANCE].interpolation, CurveInterpolation.CUBIC_HERMITE)

        compiled = compile_world_spec(world)
        bones = {bone.name: bone for bone in world.bones}
        values = compiled.bodies["values"]
        mid_index = compiled.bodies["body_ids"].index(bones["hair_mid"].id)
        tip_index = compiled.bodies["body_ids"].index(bones["hair_tip"].id)
        self.assertAlmostEqual(values[mid_index * 46 + 20], 0.5)
        self.assertAlmostEqual(values[tip_index * 46 + 20], 0.25)
        self.assertAlmostEqual(compiled.constraint_row(bones["hair_mid"].joint_constraint_id)[8], 0.01625)
        self.assertAlmostEqual(compiled.constraint_row(bones["hair_mid"].angular_constraint_id)[8], 0.02625)

    def test_compile_resolves_bone_uuids_and_preserves_explicit_target_bindings(self):
        source = (*skeleton(), SkeletonBone("control", "control_bone", None, (0, 0, 0), (0, 1, 0)))
        config = ArmatureAuthoring(
            id="armature",
            physics_systems=(
                PhysicsSystemAuthoring(
                    id="targets",
                    stable_id=10,
                    chains=(
                        ChainAuthoring("root", 20, ("root",), target_binding=TargetBinding(TargetKind.BONE, "control", (1.0, 2.0, 3.0))),
                        ChainAuthoring("mid", 21, ("mid",), target_binding=TargetBinding(TargetKind.OBJECT, "ChestProxy", (4.0, 5.0, 6.0))),
                        ChainAuthoring("tip", 22, ("tip",), target_binding=TargetBinding(TargetKind.WORLD, local_anchor=(7.0, 8.0, 9.0))),
                    ),
                ),
            ),
        )

        world = compile_authoring(config, source)
        bindings = {bone.name: bone.target_binding for bone in world.bones}

        self.assertEqual(bindings["hair_root"], TargetBinding(TargetKind.BONE, "control_bone", (1.0, 2.0, 3.0)))
        self.assertEqual(bindings["hair_mid"], TargetBinding(TargetKind.OBJECT, "ChestProxy", (4.0, 5.0, 6.0)))
        self.assertEqual(bindings["hair_tip"], TargetBinding(TargetKind.WORLD, local_anchor=(7.0, 8.0, 9.0)))

    def test_compile_is_deterministic_and_preserves_persisted_body_ids(self):
        config = ArmatureAuthoring(
            id="armature",
            stable_ids=(("body:tip", 9), ("body:root", 7), ("body:mid", 8)),
            next_stable_id=10,
            physics_systems=(
                PhysicsSystemAuthoring(
                    id="hair",
                    stable_id=10,
                    chains=(ChainAuthoring("hair-chain", 20, ("root", "mid", "tip")),),
                ),
            ),
        )

        first = compile_authoring(config, tuple(reversed(skeleton())), ())
        second = compile_authoring(config, skeleton(), ())

        self.assertEqual(tuple(bone.id for bone in first.bones), (7, 8, 9))
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
