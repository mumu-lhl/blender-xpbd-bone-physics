import builtins
import unittest

from blender_extension.bone_physics.collider_builder import (
    compile_analytic_colliders,
    compile_collision_filters,
)
from blender_extension.bone_physics.model import (
    AnalyticColliderSpec,
    AnalyticShape,
    BoneSpec,
    WorldSpec,
)


def bone(
    body_id,
    *,
    parent_id=None,
    joint_constraint_id=None,
    excluded_bodies=(),
    excluded_constraints=(),
):
    return BoneSpec(
        id=body_id,
        name=f"bone_{body_id}",
        parent_id=parent_id,
        joint_constraint_id=joint_constraint_id,
        collider_id=100 + body_id,
        local_head=(0.0, 0.0, 0.0),
        local_tail=(0.0, 1.0, 0.0),
        excluded_body_ids=excluded_bodies,
        excluded_constraint_ids=excluded_constraints,
    )


class ColliderBuilderTests(unittest.TestCase):
    def test_collision_ownership_exclusions_compile_to_stable_offsets(self):
        world = WorldSpec(
            bones=(
                bone(1, excluded_bodies=(2,), excluded_constraints=(50,)),
                bone(2, parent_id=1, joint_constraint_id=50),
            )
        )

        filters = compile_collision_filters(world.bones)

        self.assertEqual(filters["excluded_body_offsets"].tolist(), [0, 1, 1])
        self.assertEqual(filters["excluded_body_ids"].tolist(), [2])
        self.assertEqual(filters["excluded_constraint_offsets"].tolist(), [0, 1, 1])
        self.assertEqual(filters["excluded_constraint_ids"].tolist(), [50])

    def test_analytic_shapes_compile_without_per_collider_objects(self):
        colliders = (
            AnalyticColliderSpec(id=300, shape=AnalyticShape.SPHERE, radius=0.5),
            AnalyticColliderSpec(
                id=301,
                shape=AnalyticShape.CAPSULE,
                local_a=(0.0, -1.0, 0.0),
                local_b=(0.0, 1.0, 0.0),
                radius=0.25,
            ),
            AnalyticColliderSpec(
                id=302,
                shape=AnalyticShape.ROUNDED_BOX,
                half_extents=(1.0, 2.0, 3.0),
                radius=0.1,
            ),
        )

        batch = compile_analytic_colliders(colliders)

        self.assertEqual(batch["ids"].tolist(), [300, 301, 302])
        self.assertEqual(batch["kinds"].tolist(), [0, 1, 2])
        self.assertEqual(batch["columns"], 17)
        self.assertEqual(len(batch["values"]), 3 * 17)

    def test_all_pure_modules_import_and_compile_without_bpy(self):
        original_import = builtins.__import__

        def reject_bpy(name, *args, **kwargs):
            if name == "bpy" or name.startswith("bpy."):
                raise AssertionError("pure model compilation imported bpy")
            return original_import(name, *args, **kwargs)

        builtins.__import__ = reject_bpy
        try:
            from blender_extension.bone_physics import model, parameter_builder

            compiled = parameter_builder.compile_world_spec(
                model.WorldSpec(bones=(bone(1),))
            )
        finally:
            builtins.__import__ = original_import

        self.assertEqual(compiled.bodies["body_ids"].tolist(), [1])


if __name__ == "__main__":
    unittest.main()
