from dataclasses import FrozenInstanceError, replace
import unittest

from blender_extension.bone_physics.authoring import (
    ArmatureAuthoring,
    ChainAuthoring,
    CollisionPolicy,
    EndpointParameters,
    PhysicsSystemAuthoring,
    StableIdAllocator,
    ValidationSeverity,
    validate_authoring,
)


def armature_with_chains(*chains):
    return ArmatureAuthoring(
        id="armature",
        physics_systems=(
            PhysicsSystemAuthoring(
                id="system",
                stable_id=1,
                chains=tuple(
                    ChainAuthoring(
                        id=chain_id,
                        stable_id=index + 2,
                        bone_uuids=bones,
                    )
                    for index, (chain_id, bones) in enumerate(chains)
                ),
            ),
        ),
    )


class AuthoringTests(unittest.TestCase):
    def test_validation_reports_overlapping_bone_at_both_chains(self):
        config = armature_with_chains(("a", ("root", "mid")), ("b", ("mid", "tip")))

        issues = validate_authoring(config)

        self.assertEqual(issues[0].code, "chain.overlap")
        self.assertEqual(issues[0].entity_ids, ("a", "b"))

    def test_allocator_reuses_persisted_ids_and_never_returns_zero(self):
        allocator = StableIdAllocator((("body:uuid-a", 7),), next_id=8)

        self.assertEqual(allocator.allocate("body", "uuid-a"), 7)
        self.assertEqual(allocator.allocate("body", "uuid-b"), 8)
        self.assertEqual(allocator.assignments, (("body:uuid-a", 7), ("body:uuid-b", 8)))

    def test_records_are_frozen_and_validation_keeps_display_text_out_of_model(self):
        endpoint = EndpointParameters(radius=-1.0, min_twist=1.0, max_twist=0.0)
        config = armature_with_chains(("hair", ("root",)))
        config = replace(config, physics_systems=(replace(
            config.physics_systems[0],
            head=endpoint,
            collision_policy=CollisionPolicy(cross_chain_collision=False),
        ),))

        with self.assertRaises(FrozenInstanceError):
            config.id = "other"
        issues = validate_authoring(config)
        self.assertEqual(
            [(issue.code, issue.severity) for issue in issues],
            [
                ("parameter.range.inverted", ValidationSeverity.ERROR),
                ("parameter.scalar.negative", ValidationSeverity.ERROR),
                ("collision.external_resource.missing", ValidationSeverity.WARNING),
                ("collision.root.unprotected", ValidationSeverity.WARNING),
            ],
        )
        self.assertTrue(all(" " not in issue.code for issue in issues))

    def test_validation_covers_missing_references_cycles_and_layer_exhaustion(self):
        systems = tuple(
            PhysicsSystemAuthoring(
                id=f"system-{index}",
                stable_id=index + 1,
                chains=(
                    ChainAuthoring(
                        id=f"chain-{index}",
                        stable_id=index + 100,
                        bone_uuids=(f"bone-{index}",),
                        target_id="gone",
                    ),
                ),
                collision_policy=CollisionPolicy(
                    resource_ids=("missing",),
                    rigid_constraint_edges=(
                        ("chain-a", "chain-b"),
                        ("chain-b", "chain-c"),
                        ("chain-c", "chain-a"),
                    ),
                ),
            )
            for index in range(33)
        )

        issues = validate_authoring(ArmatureAuthoring(id="armature", physics_systems=systems))

        self.assertIn("target.missing", {issue.code for issue in issues})
        self.assertIn("resource.missing", {issue.code for issue in issues})
        self.assertIn("constraint.cycle.all_rigid", {issue.code for issue in issues})
        self.assertIn("collision.layers.exhausted", {issue.code for issue in issues})

    def test_duplicate_uuids_and_stable_ids_are_reported(self):
        config = ArmatureAuthoring(
            id="armature",
            physics_systems=(
                PhysicsSystemAuthoring(
                    id="one",
                    stable_id=1,
                    chains=(ChainAuthoring("a", 2, ("shared",)),),
                ),
                PhysicsSystemAuthoring(
                    id="two",
                    stable_id=1,
                    chains=(ChainAuthoring("b", 2, ("shared",)),),
                ),
            ),
            stable_ids=(("body:shared", 2), ("body:other", 2)),
        )

        issues = validate_authoring(config)

        self.assertEqual(
            {issue.code for issue in issues if issue.severity is ValidationSeverity.ERROR},
            {"chain.overlap", "stable_id.duplicate", "uuid.duplicate"},
        )


if __name__ == "__main__":
    unittest.main()
