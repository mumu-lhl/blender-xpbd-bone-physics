from dataclasses import FrozenInstanceError, replace
import math
import unittest

from blender_extension.bone_physics.model import (
    AnalyticColliderSpec,
    AnalyticShape,
    AngularLimitMode,
    BoneSpec,
    ChainField,
    ChainFieldCurve,
    ChainSpec,
    CollisionSurfaceSpec,
    CrossLinkSpec,
    CurvePoint,
    ParameterCurve,
    WorldSpec,
)


def bone(body_id, *, parent_id=None, joint_id=None, angular_id=None, length=1.0):
    return BoneSpec(
        id=body_id,
        name=f"bone_{body_id}",
        parent_id=parent_id,
        joint_constraint_id=joint_id,
        angular_constraint_id=angular_id,
        local_head=(0.0, 0.0, 0.0),
        local_tail=(0.0, length, 0.0),
    )


class ModelValidationTests(unittest.TestCase):
    def test_chain_field_curves_reject_duplicates(self):
        curve = ParameterCurve((CurvePoint(0.0, 0.0), CurvePoint(1.0, 1.0)))

        with self.assertRaisesRegex(ValueError, "duplicate chain field"):
            ChainSpec(
                id="hair",
                body_ids=(1,),
                field_curves=(
                    ChainFieldCurve(ChainField.GRAVITY_SCALE, curve),
                    ChainFieldCurve(ChainField.GRAVITY_SCALE, curve),
                ),
            )

        with self.assertRaisesRegex(ValueError, "duplicate chain field"):
            ChainSpec(
                id="legacy_and_general",
                body_ids=(1,),
                gravity_scale_curve=curve,
                field_curves=(ChainFieldCurve(ChainField.GRAVITY_SCALE, curve),),
            )

    def test_bone_uses_one_primary_angular_limit_model(self):
        spec = bone(1)
        self.assertEqual(spec.angular_limit_mode, AngularLimitMode.FREE)

        with self.assertRaisesRegex(ValueError, "swing axis"):
            replace(
                spec,
                angular_limit_mode=AngularLimitMode.SWING_TWIST,
                swing_axis=(0.0, 0.0, 0.0),
            )

    def test_authoring_dataclasses_are_immutable(self):
        spec = bone(1)

        with self.assertRaises(FrozenInstanceError):
            spec.id = 2

    def test_duplicate_explicit_ids_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate body ID 1"):
            WorldSpec(bones=(bone(1), bone(1)))

        with self.assertRaisesRegex(ValueError, "duplicate constraint ID 50"):
            WorldSpec(
                bones=(
                    bone(1),
                    bone(2, parent_id=1, joint_id=50),
                    bone(3, parent_id=1, joint_id=50),
                )
            )

    def test_missing_references_and_illegal_ranges_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "parent body 99"):
            WorldSpec(bones=(bone(1, parent_id=99, joint_id=10),))

        with self.assertRaisesRegex(ValueError, "XYZ lower"):
            BoneSpec(
                id=1,
                name="bad_limit",
                local_head=(0.0, 0.0, 0.0),
                local_tail=(0.0, 1.0, 0.0),
                xyz_lower=(0.5, -math.pi, -math.pi),
                xyz_upper=(0.25, math.pi, math.pi),
            )

    def test_equal_xyz_bounds_are_valid_axis_locks(self):
        locked = BoneSpec(
            id=1,
            name="locked",
            local_head=(0.0, 0.0, 0.0),
            local_tail=(0.0, 1.0, 0.0),
            xyz_lower=(0.0, -0.5, 0.25),
            xyz_upper=(0.0, 0.5, 0.25),
        )

        self.assertEqual(locked.xyz_lower[0], locked.xyz_upper[0])
        self.assertEqual(locked.xyz_lower[2], locked.xyz_upper[2])

    def test_cross_links_require_explicit_existing_chains(self):
        chain = ChainSpec(id="left", body_ids=(1,))
        link = CrossLinkSpec(
            first_constraint_id=100,
            chain_ids=("left", "missing"),
            samples=(0.0, 1.0),
        )

        with self.assertRaisesRegex(ValueError, "chain 'missing'"):
            WorldSpec(bones=(bone(1),), chains=(chain,), cross_links=(link,))

    def test_analytic_collider_dimensions_are_validated(self):
        with self.assertRaisesRegex(ValueError, "radius"):
            AnalyticColliderSpec(
                id=500,
                shape=AnalyticShape.SPHERE,
                radius=-0.1,
            )

    def test_collision_surface_configuration_is_validated_and_owned(self):
        surface = CollisionSurfaceSpec(
            id=600,
            object_name="Torso",
            default_region_id=10,
            excluded_materials=("eyes",),
            body_region_ignores=((7, (20, 30)),),
            upstream_weight_merges=(("breast.L", "spine.003"),),
            thickness=0.01,
            simplify_max_error=0.002,
        )
        world = WorldSpec(bones=(bone(7),), collision_surfaces=(surface,))
        self.assertEqual(world.collision_surfaces, (surface,))

        with self.assertRaisesRegex(ValueError, "duplicate collision mesh ID"):
            WorldSpec(
                bones=(bone(7),),
                collision_surfaces=(surface, surface),
            )

        with self.assertRaisesRegex(ValueError, "missing body 99"):
            WorldSpec(
                bones=(bone(7),),
                collision_surfaces=(
                    CollisionSurfaceSpec(
                        id=601,
                        object_name="Body",
                        body_region_ignores=((99, (1,)),),
                    ),
                ),
            )
