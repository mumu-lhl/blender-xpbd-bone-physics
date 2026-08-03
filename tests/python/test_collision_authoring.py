import unittest

from blender_extension.bone_physics.authoring import (
    AnalyticCollisionResource, ArmatureAuthoring, ChainAuthoring,
    CollisionCollectionAuthoring, CollisionPolicy, MeshCollisionResource,
    PhysicsSystemAuthoring, compile_collision_matrix,
    validate_authoring,
)
from blender_extension.bone_physics.authoring_adapter import SkeletonBone, compile_authoring
from blender_extension.bone_physics.model import AnalyticShape


def system(name, stable_id, *, collide=(), resources=()):
    return PhysicsSystemAuthoring(
        name, stable_id, (ChainAuthoring(name + "-chain", stable_id + 100, (name + "-root",)),),
        collision_policy=CollisionPolicy(collide_with_system_ids=collide, resource_ids=resources),
    )


class CollisionAuthoringTests(unittest.TestCase):
    def test_named_collision_matrix_is_stable_and_symmetric_when_requested(self):
        hair = system("hair", 2, collide=("skirt",))
        skirt = system("skirt", 4)
        matrix = compile_collision_matrix((skirt, hair))
        self.assertEqual(matrix["hair"][0], 1)
        self.assertEqual(matrix["skirt"][0], 2)
        self.assertTrue(matrix["hair"][1] & 2)
        self.assertTrue(matrix["skirt"][1] & 1)

    def test_more_than_32_collision_layers_is_an_error(self):
        with self.assertRaisesRegex(ValueError, "32 collision layers"):
            compile_collision_matrix(tuple(system(str(index), index + 1) for index in range(33)))
        with self.assertRaisesRegex(ValueError, "stable IDs must be unique"):
            compile_collision_matrix((system("hair", 1), system("skirt", 1)))

    def test_resources_expand_collection_and_apply_direct_parent_exclusion(self):
        hair = PhysicsSystemAuthoring(
            "hair", 1, (ChainAuthoring("hair", 2, ("head", "tip")),),
            collision_policy=CollisionPolicy(resource_ids=("head-and-torso",)),
        )
        resources = (
            AnalyticCollisionResource("head", 30, AnalyticShape.SPHERE, binding="BONE", bone_name="head"),
            MeshCollisionResource("torso", 31, "Torso", thickness=0.02, ignored_body_regions=(7,)),
            CollisionCollectionAuthoring("head-and-torso", 32, ("head", "torso")),
        )
        world = compile_authoring(ArmatureAuthoring("armature", (hair,), collision_resources=resources), (
            SkeletonBone("head", "head", None, (0, 0, 0), (0, 1, 0)),
            SkeletonBone("tip", "tip", "head", (0, 1, 0), (0, 2, 0)),
        ))
        self.assertEqual(len(world.analytic_colliders), 1)
        self.assertEqual(len(world.collision_surfaces), 1)
        self.assertEqual(world.bones[1].excluded_body_ids, (world.bones[0].id,))
        self.assertEqual(world.bones[0].collision_layer, 1)
        self.assertEqual(
            world.collision_surfaces[0].body_region_ignores,
            tuple((bone.id, (7,)) for bone in world.bones),
        )

    def test_nested_collections_and_uuid_exclusions_do_not_silently_skip(self):
        system_a = PhysicsSystemAuthoring(
            "hair", 1,
            (ChainAuthoring("left", 2, ("uuid-left",)), ChainAuthoring("right", 3, ("uuid-right",))),
            collision_policy=CollisionPolicy(resource_ids=("outer",), cross_chain_collision=False, explicit_excluded_bone_uuids=("uuid-right",)),
        )
        resources = (
            AnalyticCollisionResource("head", 10, AnalyticShape.CAPSULE, binding="BONE", bone_name="head", local_a=(1, 2, 3), local_b=(4, 5, 6)),
            CollisionCollectionAuthoring("inner", 11, ("head",)),
            CollisionCollectionAuthoring("outer", 12, ("inner",)),
        )
        world = compile_authoring(ArmatureAuthoring("armature", (system_a,), collision_resources=resources), (
            SkeletonBone("uuid-left", "different-left-name", None, (0, 0, 0), (0, 1, 0)),
            SkeletonBone("uuid-right", "different-right-name", None, (1, 0, 0), (1, 1, 0)),
        ))
        self.assertEqual(world.analytic_colliders[0].local_a, (1.0, 2.0, 3.0))
        self.assertEqual(world.analytic_colliders[0].source_binding, "BONE")
        self.assertEqual(world.analytic_colliders[0].source_bone, "head")
        self.assertEqual(world.bones[0].excluded_body_ids, (world.bones[1].id,))
        cyclic = ArmatureAuthoring("armature", (system_a,), collision_resources=(
            CollisionCollectionAuthoring("outer", 10, ("inner",)), CollisionCollectionAuthoring("inner", 11, ("outer",)),
        ))
        with self.assertRaisesRegex(ValueError, "must not cycle"):
            compile_authoring(cyclic, (SkeletonBone("uuid-left", "left", None, (0, 0, 0), (0, 1, 0)), SkeletonBone("uuid-right", "right", None, (1, 0, 0), (1, 1, 0))))

    def test_resource_identity_uses_the_same_namespace_as_systems_and_chains(self):
        config = ArmatureAuthoring("armature", (system("hair", 1),), resource_ids=("head",), collision_resources=(
            AnalyticCollisionResource("head", 1),
        ))
        issues = validate_authoring(config)
        self.assertIn("uuid.duplicate", {issue.code for issue in issues})
        self.assertIn("stable_id.duplicate", {issue.code for issue in issues})


if __name__ == "__main__":
    unittest.main()
