import unittest

try:
    import bone_physics_native as native
except ModuleNotFoundError:
    native = None

from blender_extension.bone_physics.model import (
    BoneSpec,
    ChainSpec,
    ChainField,
    ChainFieldCurve,
    CurvePoint,
    CrossLinkSpec,
    AngularLimitMode,
    ParameterCurve,
    WorldSpec,
)
from blender_extension.bone_physics.parameter_builder import (
    build_cross_link_constraints,
    chain_boundaries,
    compile_world_spec,
    sample_chain_anchor,
)


def bone(body_id, x, length):
    return BoneSpec(
        id=body_id,
        name=f"bone_{body_id}",
        rest_translation=(x, 0.0, 0.0),
        local_head=(0.0, 0.0, 0.0),
        local_tail=(0.0, length, 0.0),
    )


def body_value(compiled, body_id, column):
    ids = compiled.bodies["body_ids"]
    values = compiled.bodies["values"]
    return values[ids.index(body_id) * 46 + column]


def full_field_world():
    curve = ParameterCurve((CurvePoint(0.0, 0.0), CurvePoint(1.0, 1.0)))
    root = bone(1, 0.0, 1.0)
    child = BoneSpec(
        id=2,
        name="child",
        parent_id=1,
        joint_constraint_id=20,
        rest_translation=(0.0, 1.0, 0.0),
        local_head=(0.0, 0.0, 0.0),
        local_tail=(0.0, 1.0, 0.0),
    )
    return WorldSpec(
        bones=(root, child),
        chains=(
            ChainSpec(
                id="full",
                body_ids=(1, 2),
                field_curves=(
                    ChainFieldCurve(ChainField.GRAVITY_SCALE, curve),
                    ChainFieldCurve(ChainField.JOINT_COMPLIANCE, curve),
                ),
            ),
        ),
    )


def attachment_and_swing_world():
    root = bone(1, 0.0, 1.0)
    child = BoneSpec(
        id=2,
        name="child",
        parent_id=1,
        angular_constraint_id=22,
        rest_translation=(0.0, 1.0, 0.0),
        local_head=(0.0, 0.0, 0.0),
        local_tail=(0.0, 1.0, 0.0),
        angular_limit_mode=AngularLimitMode.SWING_TWIST,
        swing_axis=(0.0, 1.0, 0.0),
        max_swing=0.6,
        min_twist=-0.4,
        max_twist=0.4,
    )
    return WorldSpec(
        bones=(root, child),
        chains=(ChainSpec(id="attached", body_ids=(1, 2), attachment_constraint_id=21),),
    )


class ParameterBuilderTests(unittest.TestCase):
    def test_general_fields_sample_bodies_and_joint_boundaries(self):
        compiled = compile_world_spec(full_field_world())

        self.assertAlmostEqual(body_value(compiled, 1, 20), 0.25)
        self.assertAlmostEqual(body_value(compiled, 2, 20), 0.75)
        self.assertAlmostEqual(compiled.constraint_row(20)[8], 0.5)

    def test_root_attachment_and_swing_twist_use_existing_batch_kinds(self):
        compiled = compile_world_spec(attachment_and_swing_world())

        self.assertEqual(tuple(compiled.constraints["kinds"]), (3, 6))
        self.assertEqual(compiled.constraint_row(22)[14:17], (0.0, 1.0, 0.0))
        self.assertAlmostEqual(compiled.constraint_row(22)[17], 0.6)
        self.assertAlmostEqual(compiled.constraint_row(22)[18], -0.4)
        self.assertAlmostEqual(compiled.constraint_row(22)[19], 0.4)
    def test_chain_coordinates_use_accumulated_rest_length(self):
        bones = {
            item.id: item
            for item in (bone(1, 0.0, 1.0), bone(2, 0.0, 3.0), bone(3, 0.0, 2.0))
        }
        chain = ChainSpec(id="uneven", body_ids=(1, 2, 3))

        self.assertEqual(chain_boundaries(chain, bones), (0.0, 1 / 6, 4 / 6, 1.0))

    def test_unequal_chains_match_normalized_length_not_body_index(self):
        bones = {
            item.id: item
            for item in (
                bone(1, 0.0, 1.0),
                bone(2, 0.0, 3.0),
                bone(10, 2.0, 2.0),
                bone(11, 2.0, 1.0),
                bone(12, 2.0, 1.0),
            )
        }
        short = ChainSpec(id="short", body_ids=(1, 2))
        long = ChainSpec(id="long", body_ids=(10, 11, 12))

        left = sample_chain_anchor(short, 0.5, bones)
        right = sample_chain_anchor(long, 0.5, bones)

        self.assertEqual(left.body_id, 2)
        self.assertAlmostEqual(left.local_anchor[1], 1.0)
        self.assertEqual(right.body_id, 11)
        self.assertAlmostEqual(right.local_anchor[1], 0.0)

    def test_closed_skirt_has_one_seam_and_deterministic_diagonals(self):
        bones = {}
        chains = []
        next_id = 1
        for chain_index in range(4):
            ids = []
            for row, length in enumerate((1.0, 1.5, 0.5)):
                item = bone(next_id, float(chain_index), length)
                bones[next_id] = item
                ids.append(next_id)
                next_id += 1
            chains.append(ChainSpec(id=f"c{chain_index}", body_ids=tuple(ids)))
        spec = CrossLinkSpec(
            first_constraint_id=1000,
            chain_ids=tuple(chain.id for chain in chains),
            samples=(0.0, 0.5, 1.0),
            diagonals=True,
            closed=True,
            compliance=1.0e-5,
        )

        links = build_cross_link_constraints(spec, tuple(chains), bones)

        self.assertEqual(len(links), 28)
        self.assertEqual(len({link.id for link in links}), 28)
        seam = [link for link in links if {link.chain_a, link.chain_b} == {"c0", "c3"}]
        self.assertEqual(len(seam), 7)
        self.assertTrue(all(link.compliance > 0.0 for link in links))

        world = WorldSpec(
            bones=tuple(reversed(tuple(bones.values()))),
            chains=tuple(reversed(chains)),
            cross_links=(spec,),
        )
        first = compile_world_spec(world)
        second = compile_world_spec(world)
        self.assertEqual(first.canonical_bytes(), second.canonical_bytes())
        if native is not None:
            handle = native.build_world(
                first.config,
                first.bodies,
                first.constraints,
                first.filters,
            )
            self.addCleanup(native.destroy_world, handle)

    def test_compiler_is_byte_deterministic_and_emits_xyz_locks(self):
        root = bone(1, 0.0, 1.0)
        child = BoneSpec(
            id=2,
            name="child",
            parent_id=1,
            joint_constraint_id=20,
            angular_constraint_id=21,
            rest_translation=(0.0, 1.0, 0.0),
            local_head=(0.0, 0.0, 0.0),
            local_tail=(0.0, 1.0, 0.0),
            xyz_lower=(0.0, -0.5, 0.25),
            xyz_upper=(0.0, 0.5, 0.25),
        )
        world = WorldSpec(bones=(root, child))

        first = compile_world_spec(world)
        second = compile_world_spec(world)

        self.assertEqual(first.canonical_bytes(), second.canonical_bytes())
        angular_row = first.constraint_row(21)
        self.assertEqual(angular_row[14:17], (0.0, -0.5, 0.25))
        self.assertEqual(angular_row[17:20], (0.0, 0.5, 0.25))

    def test_root_to_tip_curves_sample_each_bone_at_its_span_midpoint(self):
        source = (
            bone(1, 0.0, 1.0),
            bone(2, 0.0, 3.0),
            bone(3, 0.0, 2.0),
        )
        chain = ChainSpec(
            id="graded",
            body_ids=(1, 2, 3),
            gravity_scale_curve=ParameterCurve(
                points=(CurvePoint(0.0, 0.0), CurvePoint(1.0, 1.0))
            ),
        )

        compiled = compile_world_spec(WorldSpec(bones=source, chains=(chain,)))
        values = compiled.bodies["values"]
        gravity = [values[index * 46 + 20] for index in range(3)]

        self.assertAlmostEqual(gravity[0], 1 / 12)
        self.assertAlmostEqual(gravity[1], 5 / 12)
        self.assertAlmostEqual(gravity[2], 5 / 6)

    @unittest.skipIf(native is None, "native bridge is tested in native-contract")
    def test_compiled_batch_is_accepted_by_the_native_bridge(self):
        root = bone(1, 0.0, 1.0)
        child = BoneSpec(
            id=2,
            name="child",
            parent_id=1,
            joint_constraint_id=20,
            rest_translation=(0.0, 1.0, 0.0),
            local_head=(0.0, 0.0, 0.0),
            local_tail=(0.0, 1.0, 0.0),
        )
        compiled = compile_world_spec(WorldSpec(bones=(child, root)))

        handle = native.build_world(
            compiled.config,
            compiled.bodies,
            compiled.constraints,
            compiled.filters,
        )
        self.addCleanup(native.destroy_world, handle)
        native.set_analytic_colliders(handle, compiled.analytic_colliders)

        summary = native.advance(handle, 1.0 / 120.0, "deterministic_preview")
        self.assertEqual(summary["steps"], 1)

    def test_cross_link_generation_rejects_overlapping_bodies_and_id_overflow(self):
        bones = {
            item.id: item
            for item in (
                bone(1, 0.0, 1.0),
                bone(2, 1.0, 1.0),
                bone(3, 2.0, 1.0),
            )
        }
        overlapping = (
            ChainSpec(id="a", body_ids=(1, 2)),
            ChainSpec(id="b", body_ids=(2, 3)),
        )
        with self.assertRaisesRegex(ValueError, "body 2.*multiple cross-link chains"):
            build_cross_link_constraints(
                CrossLinkSpec(
                    first_constraint_id=100,
                    chain_ids=("a", "b"),
                    samples=(0.0, 1.0),
                ),
                overlapping,
                bones,
            )

        distinct = (
            ChainSpec(id="a", body_ids=(1,)),
            ChainSpec(id="b", body_ids=(3,)),
        )
        with self.assertRaisesRegex(ValueError, "constraint ID range"):
            build_cross_link_constraints(
                CrossLinkSpec(
                    first_constraint_id=0xFFFFFFFF,
                    chain_ids=("a", "b"),
                    samples=(0.0, 1.0),
                ),
                distinct,
                bones,
            )


if __name__ == "__main__":
    unittest.main()
