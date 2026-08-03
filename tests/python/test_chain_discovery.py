import unittest

from blender_extension.bone_physics.chain_discovery import (
    BoneNode,
    BranchSelectionRequired,
    descendant_leaves,
    resolve_chain,
    validate_new_paths,
)


def skeleton(*entries):
    return tuple(
        BoneNode(uuid=uuid, name=name or uuid, parent_uuid=parent_uuid, length=1.0)
        for uuid, parent_uuid, *name in entries
    )


class ChainDiscoveryTests(unittest.TestCase):
    def test_resolve_chain_follows_parent_links(self):
        nodes = skeleton(("root", None), ("mid", "root"), ("tip", "mid"))

        self.assertEqual(resolve_chain(nodes, "root"), ("root", "mid", "tip"))

    def test_descendant_leaves_use_name_then_uuid_order(self):
        nodes = skeleton(
            ("root", None),
            ("zeta", "root", "same"),
            ("alpha", "root", "same"),
            ("omega", "root", "later"),
        )

        self.assertEqual(descendant_leaves(nodes, "root"), ("omega", "alpha", "zeta"))

    def test_branch_requires_an_explicit_leaf_tail(self):
        nodes = skeleton(("root", None), ("left", "root"), ("right", "root"))

        with self.assertRaisesRegex(BranchSelectionRequired, "left.*right"):
            resolve_chain(nodes, "root")
        self.assertEqual(resolve_chain(nodes, "root", "right"), ("root", "right"))

    def test_tail_must_be_an_existing_descendant_leaf(self):
        nodes = skeleton(("root", None), ("mid", "root"), ("tip", "mid"), ("other", None))

        with self.assertRaisesRegex(ValueError, "tail must be a descendant leaf"):
            resolve_chain(nodes, "root", "mid")
        with self.assertRaisesRegex(ValueError, "tail must be a descendant leaf"):
            resolve_chain(nodes, "root", "other")
        with self.assertRaisesRegex(ValueError, "tail .*missing"):
            resolve_chain(nodes, "root", "missing")

    def test_malformed_hierarchies_fail_deterministically(self):
        missing_parent = skeleton(("root", "gone"))
        cycle = skeleton(("z", "a"), ("a", "z"))

        with self.assertRaisesRegex(ValueError, "missing parent.*gone"):
            descendant_leaves(missing_parent, "root")
        with self.assertRaisesRegex(ValueError, "parent cycle: a -> z -> a"):
            descendant_leaves(cycle, "a")
        with self.assertRaisesRegex(ValueError, "duplicate bone UUID: root"):
            descendant_leaves(skeleton(("root", None), ("root", None)), "root")

    def test_path_validation_rejects_duplicate_new_paths_and_bone_overlap(self):
        with self.assertRaisesRegex(ValueError, r"duplicate new path: \('root', 'tip'\)"):
            validate_new_paths((), (("root", "tip"), ("root", "tip")))
        with self.assertRaisesRegex(ValueError, "physical bone overlap: mid"):
            validate_new_paths((("root", "mid"),), (("mid", "tip"),))
        with self.assertRaisesRegex(ValueError, "physical bone overlap: mid"):
            validate_new_paths((), (("root", "mid"), ("mid", "tip")))


if __name__ == "__main__":
    unittest.main()
