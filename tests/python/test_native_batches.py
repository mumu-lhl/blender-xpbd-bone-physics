from array import array
import math
import unittest

import bone_physics_native as native


BODY_COLUMNS = 46


def config(**overrides):
    result = {
        "api_version": native.native_api_version(),
        "gravity": (0.0, -9.81, 0.0),
        "fixed_dt": 1.0 / 120.0,
    }
    result.update(overrides)
    return result


def body_row(x):
    return [
        # Rest pose.
        x,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        # Center of mass, head, and tail.
        0.0,
        0.0,
        0.0,
        0.0,
        -0.5,
        0.0,
        0.0,
        0.5,
        0.0,
        # Inverse mass and inverse inertia.
        1.0,
        1.0,
        1.0,
        1.0,
        # Gravity scale and damping.
        1.0,
        0.0,
        0.0,
        # Initial pose.
        x,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        # Linear and angular velocity.
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        # Capsule and material.
        0.0,
        -0.4,
        0.0,
        0.0,
        0.4,
        0.0,
        0.1,
        0.01,
        0.5,
        0.0,
    ]


def bodies(rows=None):
    rows = [body_row(0.0), body_row(2.0)] if rows is None else rows
    values = array("f", (value for row in rows for value in row))
    return {
        "body_ids": array("I", range(1, len(rows) + 1)),
        "collider_ids": array("I", range(101, 101 + len(rows))),
        "values": values,
        "columns": BODY_COLUMNS,
    }


def empty_constraints():
    return {
        "ids": array("I"),
        "body_ids": array("I"),
        "kinds": array("I"),
        "enabled": array("I"),
        "values": array("f"),
        "columns": 24,
    }


def anchor_constraint():
    values = [0.0] * 24
    values[8] = 0.0
    values[9] = 0.0
    return {
        "ids": array("I", [201]),
        "body_ids": array("I", [1, 2]),
        "kinds": array("I", [0]),
        "enabled": array("I", [1]),
        "values": array("f", values),
        "columns": 24,
    }


def filters(count=2):
    return {
        "layers": array("I", [1] * count),
        "masks": array("I", [0xFFFFFFFF] * count),
        "excluded_body_offsets": array("I", [0] * (count + 1)),
        "excluded_body_ids": array("I"),
        "excluded_constraint_offsets": array("I", [0] * (count + 1)),
        "excluded_constraint_ids": array("I"),
    }


class NativeBatchTests(unittest.TestCase):
    def build(self, body_batch=None, constraint_batch=None, filter_batch=None):
        handle = native.build_world(
            config(),
            bodies() if body_batch is None else body_batch,
            empty_constraints() if constraint_batch is None else constraint_batch,
            filters() if filter_batch is None else filter_batch,
        )
        self.addCleanup(native.destroy_world, handle)
        return handle

    def test_valid_two_body_batch_advances_and_writes_normalized_quaternions(self):
        handle = self.build()

        summary = native.advance(handle, 1.0 / 60.0, "deterministic_preview")
        output = array("f", [0.0] * 14)
        native.read_transforms(handle, output)

        self.assertEqual(summary["steps"], 2)
        self.assertFalse(summary["dropped"])
        for offset in (0, 7):
            quaternion = output[offset + 3 : offset + 7]
            length = math.sqrt(sum(component * component for component in quaternion))
            self.assertAlmostEqual(length, 1.0, places=6)
        self.assertEqual(native.read_diagnostics(handle)["body_count"], 2)

    def test_wrong_dtype_is_rejected(self):
        batch = bodies()
        batch["values"] = array("d", batch["values"])

        with self.assertRaisesRegex(native.BonePhysicsInputError, "f32"):
            self.build(body_batch=batch)

    def test_non_contiguous_buffer_is_rejected(self):
        batch = bodies()
        backing = array("f", [value for value in batch["values"] for _ in range(2)])
        batch["values"] = memoryview(backing)[::2]

        with self.assertRaisesRegex(native.BonePhysicsInputError, "contiguous"):
            self.build(body_batch=batch)

    def test_missing_columns_are_rejected(self):
        batch = bodies()
        batch["columns"] = BODY_COLUMNS - 1

        with self.assertRaisesRegex(native.BonePhysicsInputError, "46 columns"):
            self.build(body_batch=batch)

    def test_duplicate_ids_and_nan_are_rejected(self):
        duplicate = bodies()
        duplicate["body_ids"] = array("I", [7, 7])
        with self.assertRaisesRegex(native.BonePhysicsInputError, "duplicate body ID 7"):
            self.build(body_batch=duplicate)

        non_finite = bodies()
        non_finite["values"][0] = float("nan")
        with self.assertRaisesRegex(native.BonePhysicsInputError, "finite"):
            self.build(body_batch=non_finite)

    def test_stale_handle_is_rejected_by_every_world_operation(self):
        handle = self.build()
        native.destroy_world(handle)
        self._cleanups.pop()

        with self.assertRaises(native.BonePhysicsStateError):
            native.advance(handle, 1.0 / 60.0, "deterministic_preview")

    def test_writable_output_must_have_enough_f32_items(self):
        handle = self.build()

        with self.assertRaisesRegex(native.BonePhysicsInputError, "14"):
            native.read_transforms(handle, array("f", [0.0] * 13))
        with self.assertRaisesRegex(native.BonePhysicsInputError, "writable"):
            native.read_transforms(handle, bytes(14 * 4))

    def test_api_version_mismatch_is_rejected_before_world_creation(self):
        with self.assertRaisesRegex(native.BonePhysicsInputError, "API version"):
            native.build_world(
                config(api_version=native.native_api_version() + 1),
                bodies(),
                empty_constraints(),
                filters(),
            )

    def test_valid_constraint_and_filter_batches_are_installed_atomically(self):
        filtered = filters()
        filtered["excluded_body_offsets"] = array("I", [0, 1, 1])
        filtered["excluded_body_ids"] = array("I", [2])
        filtered["excluded_constraint_offsets"] = array("I", [0, 1, 1])
        filtered["excluded_constraint_ids"] = array("I", [201])

        handle = self.build(
            constraint_batch=anchor_constraint(),
            filter_batch=filtered,
        )

        summary = native.advance(handle, 1.0 / 120.0, "authoritative_bake")
        self.assertEqual(summary["steps"], 1)

    def test_invalid_filter_offsets_are_rejected(self):
        malformed = filters()
        malformed["excluded_body_offsets"] = array("I", [0, 2, 1])

        with self.assertRaisesRegex(native.BonePhysicsInputError, "monotonic"):
            self.build(filter_batch=malformed)

    def test_animation_targets_are_owned_copies_and_validate_finite_values(self):
        handle = self.build()
        values = array(
            "f",
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                1.0,
                1.0,
            ],
        )
        batch = {"body_ids": array("I", [1]), "values": values, "columns": 16}
        native.set_animation_targets(handle, batch)
        values[7] = 1000.0

        native.advance(handle, 1.0 / 120.0, "deterministic_preview")
        output = array("f", [0.0] * 14)
        native.read_transforms(handle, output)
        self.assertLess(output[0], 10.0)

        batch["values"][0] = float("nan")
        with self.assertRaisesRegex(native.BonePhysicsInputError, "finite"):
            native.set_animation_targets(handle, batch)

    def test_analytic_collider_batch_is_validated_and_counted(self):
        handle = self.build()
        values = [0.0] * 17
        values[6] = 1.0
        values[10] = 0.5
        batch = {
            "ids": array("I", [301]),
            "kinds": array("I", [0]),
            "enabled": array("I", [1]),
            "values": array("f", values),
            "columns": 17,
        }

        native.set_analytic_colliders(handle, batch)

        self.assertEqual(native.read_diagnostics(handle)["analytic_collider_count"], 1)
        batch["values"][10] = -1.0
        with self.assertRaisesRegex(native.BonePhysicsInputError, "nonnegative"):
            native.set_analytic_colliders(handle, batch)
        batch["values"][10] = 0.5
        batch["ids"][0] = 101
        with self.assertRaisesRegex(native.BonePhysicsInputError, "duplicate collider ID 101"):
            native.set_analytic_colliders(handle, batch)

    def test_transform_output_rejects_wrong_dtype_and_noncontiguous_storage(self):
        handle = self.build()

        with self.assertRaisesRegex(native.BonePhysicsInputError, "f32"):
            native.read_transforms(handle, array("d", [0.0] * 14))
        backing = array("f", [0.0] * 28)
        with self.assertRaisesRegex(native.BonePhysicsInputError, "contiguous"):
            native.read_transforms(handle, memoryview(backing)[::2])

    def test_empty_world_accepts_an_empty_writable_transform_buffer(self):
        handle = native.create_world(config(), {})
        self.addCleanup(native.destroy_world, handle)

        native.read_transforms(handle, array("f"))


if __name__ == "__main__":
    unittest.main()
