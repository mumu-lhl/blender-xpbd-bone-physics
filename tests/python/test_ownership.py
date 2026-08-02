import unittest

from blender_extension.bone_physics.ownership import (
    FaceOwnership,
    ObjectOwnership,
    ObjectRole,
    build_ownership_map,
)


class OwnershipTests(unittest.TestCase):
    def test_simulated_and_explicitly_excluded_objects_never_become_obstacles(self):
        ownership = build_ownership_map(
            (
                ObjectOwnership("Torso", ObjectRole.COLLISION, (FaceOwnership(1),)),
                ObjectOwnership("Hair", ObjectRole.SIMULATED, (FaceOwnership(2),)),
                ObjectOwnership("Cape", ObjectRole.EXCLUDED, (FaceOwnership(3),)),
            )
        )

        self.assertEqual(ownership.collision_objects(), ("Torso",))
        self.assertEqual(ownership.region_for_face("Torso", 0), 1)
        self.assertIsNone(ownership.region_for_face("Hair", 0))
        self.assertIsNone(ownership.region_for_face("Cape", 0))

    def test_material_and_face_attribute_exclusions_are_removed(self):
        ownership = build_ownership_map(
            (
                ObjectOwnership(
                    "Body",
                    ObjectRole.COLLISION,
                    (
                        FaceOwnership(10, material="skin"),
                        FaceOwnership(11, material="eyes"),
                        FaceOwnership(12, excluded=True),
                    ),
                    excluded_materials=("eyes",),
                ),
            )
        )

        self.assertEqual(ownership.region_for_face("Body", 0), 10)
        self.assertIsNone(ownership.region_for_face("Body", 1))
        self.assertIsNone(ownership.region_for_face("Body", 2))

    def test_each_body_ignores_regions_it_directly_drives_and_explicit_masks(self):
        ownership = build_ownership_map(
            (
                ObjectOwnership(
                    "Torso",
                    ObjectRole.COLLISION,
                    (
                        FaceOwnership(20),
                        FaceOwnership(21, driven_by_body_ids=(7,)),
                        FaceOwnership(22, driven_by_body_ids=(8, 9)),
                    ),
                    body_region_ignores=((7, (30,)), (9, (31, 32))),
                ),
            )
        )

        self.assertFalse(ownership.body_sees_region(7, 21))
        self.assertFalse(ownership.body_sees_region(7, 30))
        self.assertTrue(ownership.body_sees_region(7, 20))
        self.assertFalse(ownership.body_sees_region(8, 22))
        self.assertFalse(ownership.body_sees_region(9, 22))
        self.assertFalse(ownership.body_sees_region(9, 31))
        self.assertTrue(ownership.body_sees_region(9, 20))

    def test_duplicate_objects_and_conflicting_region_drivers_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate ownership object"):
            build_ownership_map(
                (
                    ObjectOwnership("Body", ObjectRole.COLLISION),
                    ObjectOwnership("Body", ObjectRole.COLLISION),
                )
            )

        with self.assertRaisesRegex(ValueError, "duplicate body-region ignore"):
            ObjectOwnership(
                "Body",
                ObjectRole.COLLISION,
                body_region_ignores=((7, (1,)), (7, (2,))),
            )


if __name__ == "__main__":
    unittest.main()
