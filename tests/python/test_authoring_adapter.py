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


def skeleton():
    return (
        SkeletonBone("root", "hair_root", None, (0, 0, 0), (0, 1, 0)),
        SkeletonBone("mid", "hair_mid", "root", (0, 1, 0), (0, 3, 0)),
        SkeletonBone("tip", "hair_tip", "mid", (0, 3, 0), (0, 4, 0)),
    )


class AuthoringAdapterTests(unittest.TestCase):
    def test_compile_samples_full_gradient_before_applying_field_overrides(self):
        config = ArmatureAuthoring(
            id="armature",
            physics_systems=(
                PhysicsSystemAuthoring(
                    id="hair",
                    stable_id=10,
                    head=EndpointParameters(
                        gravity_scale=1.0, radius=0.2, inertia_scale=(1.0, 2.0, 3.0),
                        joint_compliance=0.01, angular_compliance=0.02,
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
        bones = {bone.name: bone for bone in world.bones}

        self.assertAlmostEqual(bones["hair_mid"].gravity_scale, 0.5)
        self.assertAlmostEqual(bones["hair_mid"].radius, 0.3)
        self.assertEqual(bones["hair_mid"].inverse_inertia, (3.0, 4.0, 5.0))
        self.assertAlmostEqual(bones["hair_mid"].joint_compliance, 0.03)
        self.assertAlmostEqual(bones["hair_mid"].angular_compliance, 0.04)
        self.assertAlmostEqual(bones["hair_tip"].gravity_scale, 0.25)
        self.assertAlmostEqual(bones["hair_tip"].radius, 0.375)
        self.assertEqual(bones["hair_tip"].angular_limit_mode, AngularLimitMode.XYZ)

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
