import unittest

from blender_extension.bone_physics.diagnostics import (
    DiagnosticBindings,
    DiagnosticItem,
    DiagnosticOwner,
    diagnostic_counts,
    map_runtime_error,
    merge_diagnostics,
)


class AuthoringDiagnosticsTests(unittest.TestCase):
    def test_native_body_error_maps_back_to_chain_and_bone(self):
        config = DiagnosticBindings(
            bodies={17: DiagnosticOwner("bone", "hair_tip_uuid", "hair")}
        )

        item = map_runtime_error(config, {"body_id": 17, "penetration": 0.02})

        self.assertEqual(item.entity_kind, "bone")
        self.assertEqual(item.entity_id, "hair_tip_uuid")
        self.assertEqual(dict(item.arguments)["system_id"], "hair")

    def test_merging_orders_errors_then_warnings_by_stable_id(self):
        items = merge_diagnostics(
            (DiagnosticItem("authoring.warn", "warning", "chain", "late", stable_id=9),),
            (
                DiagnosticItem("runtime.error", "error", "bone", "later", stable_id=7),
                DiagnosticItem("runtime.error", "error", "bone", "first", stable_id=2),
            ),
        )

        self.assertEqual([item.entity_id for item in items], ["first", "later", "late"])

    def test_runtime_ids_map_each_supported_owner_kind(self):
        bindings = DiagnosticBindings(
            constraints={3: DiagnosticOwner("chain", "hair", "hair")},
            colliders={4: DiagnosticOwner("collider", "head_proxy")},
            meshes={5: DiagnosticOwner("mesh", "body_mesh")},
            regions={(5, 6): DiagnosticOwner("region", "scalp")},
        )
        for values, expected in (
            ({"constraint_id": 3}, "hair"),
            ({"collider_id": 4}, "head_proxy"),
            ({"mesh_id": 5}, "body_mesh"),
            ({"mesh_id": 5, "region_id": 6}, "scalp"),
        ):
            self.assertEqual(map_runtime_error(bindings, values).entity_id, expected)

    def test_counts_are_grouped_by_severity(self):
        self.assertEqual(
            diagnostic_counts((
                DiagnosticItem("a", "error", "bone", "a"),
                DiagnosticItem("b", "warning", "bone", "b"),
                DiagnosticItem("c", "warning", "bone", "c"),
            )),
            {"error": 1, "warning": 2, "info": 0},
        )
