"""Pure-Python coverage for transactional runtime-world ownership."""

from __future__ import annotations

import unittest

from blender_extension.bone_physics.runtime_registry import RuntimeRegistry


class Factory:
    def __init__(self, handles):
        self.handles = iter(handles)

    def create(self, scene, armature, spec):
        del scene, armature, spec
        value = next(self.handles)
        if isinstance(value, Exception):
            raise value
        return World(value)


class World:
    def __init__(self, handle):
        self.handle = handle
        self.destroyed = False

    def destroy(self):
        self.destroyed = True


def valid_spec():
    return object()


def invalid_spec():
    return object()


class RuntimeRegistryTests(unittest.TestCase):
    def test_dirty_rebuild_compiles_a_fresh_spec(self):
        built = []

        def build_spec(scene, armature):
            del scene, armature
            built.append(len(built))
            return built[-1]

        registry = RuntimeRegistry(factory=Factory(handles=[10, 20]), spec_builder=build_spec)
        scene, armature = Pointer(1), Pointer(2)
        registry.rebuild(scene, armature)
        registry.mark_dirty(2, "parameters")

        registry.flush_dirty()

        self.assertEqual(built, [0, 1])
        self.assertEqual(registry.world(scene, armature).handle, 20)

    def test_ensure_compiles_and_creates_an_initial_world(self):
        registry = RuntimeRegistry(
            factory=Factory(handles=[10]), spec_builder=lambda _scene, _armature: "fresh"
        )
        scene, armature = Pointer(1), Pointer(2)

        world = registry.ensure(scene, armature)

        self.assertEqual(world.handle, 10)
        self.assertEqual(registry.world(scene, armature), world)
    def test_failed_rebuild_keeps_previous_valid_world(self):
        registry = RuntimeRegistry(factory=Factory(handles=[10, ValueError("bad config")]))
        registry.rebuild(scene_id=1, armature_id=2, spec=valid_spec())
        with self.assertRaisesRegex(ValueError, "bad config"):
            registry.rebuild(scene_id=1, armature_id=2, spec=invalid_spec())
        self.assertEqual(registry.world_by_key((1, 2)).handle, 10)

    def test_replacement_is_created_before_old_world_is_destroyed(self):
        registry = RuntimeRegistry(factory=Factory(handles=[10, 20]))
        old = registry.rebuild(scene_id=1, armature_id=2, spec=valid_spec())

        replacement = registry.rebuild(scene_id=1, armature_id=2, spec=valid_spec())

        self.assertEqual(replacement.handle, 20)
        self.assertTrue(old.destroyed)

    def test_structural_dirty_record_stays_pending_until_validation(self):
        registry = RuntimeRegistry(factory=Factory(handles=[]))
        registry.mark_dirty(7, "chains")

        self.assertEqual(registry.flush_dirty(), ())
        self.assertEqual(registry.pending_structural(7), ("chains",))

        registry.mark_validated(7)
        self.assertEqual(registry.flush_dirty(), ((7, "chains"),))

    def test_failed_initial_compile_publishes_authoring_diagnostic(self):
        settings = DiagnosticSettings()
        armature = DiagnosticArmature(2, settings)

        def build_spec(_scene, _armature):
            raise ValueError("invalid chain")

        registry = RuntimeRegistry(
            factory=Factory(handles=[]), spec_builder=build_spec
        )
        with self.assertRaisesRegex(ValueError, "invalid chain"):
            registry.ensure(Pointer(1), armature)

        self.assertEqual(len(settings.diagnostic_items), 1)
        self.assertEqual(settings.diagnostic_items[0].entity_kind, "runtime")
        self.assertTrue(settings.diagnostic_items[0].code.startswith("authoring."))


if __name__ == "__main__":
    unittest.main()


class Pointer:
    def __init__(self, value):
        self.value = value

    def as_pointer(self):
        return self.value


class DiagnosticItem:
    pass


class DiagnosticCollection(list):
    def add(self):
        item = DiagnosticItem()
        self.append(item)
        return item

    def remove(self, index):
        del self[index]


class DiagnosticSettings:
    def __init__(self):
        self.diagnostic_items = DiagnosticCollection()


class DiagnosticArmature(Pointer):
    def __init__(self, value, settings):
        super().__init__(value)
        self.bone_physics = settings
