from dataclasses import FrozenInstanceError, replace
import math
import unittest

from blender_extension.bone_physics.authoring import (
    ArmatureAuthoring,
    ChainAuthoring,
    CollisionPolicy,
    EndpointParameters,
    PhysicsSystemAuthoring,
    StableIdAllocator,
    ValidationSeverity,
    stable_id_range_end,
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

    def test_allocator_can_allocate_u32_maximum_and_wraps_to_an_unused_id(self):
        allocator = StableIdAllocator(next_id=0xFFFFFFFF)

        self.assertEqual(allocator.allocate("body", "last"), 0xFFFFFFFF)
        self.assertEqual(allocator.assignments, (("body:last", 0xFFFFFFFF),))
        self.assertEqual(allocator.allocate("body", "wrapped"), 1)
        self.assertEqual(
            allocator.assignments,
            (("body:last", 0xFFFFFFFF), ("body:wrapped", 1)),
        )

    def test_structural_constraint_range_rejects_rna_integer_overflow(self):
        self.assertEqual(stable_id_range_end(0x7FFFFFFE, 2), 0x7FFFFFFF)
        with self.assertRaisesRegex(ValueError, "RNA integer limit"):
            stable_id_range_end(0x7FFFFFFF, 2)

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

    def test_validation_rejects_zero_radius_and_excessive_swing_without_crashing(self):
        config = armature_with_chains(("hair", ("root",)))
        invalid = EndpointParameters(radius="not-a-number", max_swing=math.pi + 0.1)
        config = replace(config, physics_systems=(replace(config.physics_systems[0], head=invalid),))

        issues = validate_authoring(config)

        self.assertIn("parameter.scalar.nonfinite", {issue.code for issue in issues})
        self.assertIn("parameter.range.exceeds_limit", {issue.code for issue in issues})
        zero_radius = replace(invalid, radius=0.0)
        zero_config = replace(config, physics_systems=(replace(config.physics_systems[0], head=zero_radius),))
        self.assertIn("parameter.scalar.zero", {issue.code for issue in validate_authoring(zero_config)})

    def test_validation_warns_for_redundant_constraints_and_suspicious_weights(self):
        config = ArmatureAuthoring(
            id="armature",
            physics_systems=(
                PhysicsSystemAuthoring(
                    id="hair",
                    stable_id=1,
                    chains=(ChainAuthoring("hair-chain", 2, ("root",)),),
                    collision_policy=CollisionPolicy(
                        rigid_constraint_edges=(("root", "tip"), ("tip", "root")),
                        suspicious_weight_bone_uuids=("root",),
                    ),
                ),
            ),
        )

        issues = validate_authoring(config)

        self.assertIn("constraint.redundant", {issue.code for issue in issues})
        weight_issue = next(issue for issue in issues if issue.code == "collision.weight.suspicious")
        self.assertEqual(weight_issue.entity_ids, ("hair", "root"))
        self.assertEqual(weight_issue.arguments, (("bone_uuid", "root"),))

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
