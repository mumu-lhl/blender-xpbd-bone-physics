"""Pure-Python coverage for transactional runtime-world ownership."""

from __future__ import annotations

import ast
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from blender_extension.bone_physics.authoring import ValidationIssue, ValidationSeverity
from blender_extension.bone_physics import runtime_registry
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
    def test_register_timer_tolerates_restricted_blender_data_context(self):
        class Timers:
            def __init__(self):
                self.registered = set()

            def is_registered(self, function):
                return function in self.registered

            def register(self, function, **_kwargs):
                self.registered.add(function)

            def unregister(self, function):
                self.registered.discard(function)

        fake_bpy = SimpleNamespace(
            app=SimpleNamespace(
                timers=Timers(),
                handlers=SimpleNamespace(load_post=[], frame_change_post=[]),
            ),
            data=SimpleNamespace(),
        )
        with mock.patch.dict("sys.modules", {"bpy": fake_bpy}):
            runtime_registry.register_timer()
            runtime_registry.unregister_timer()

    def test_enabled_configured_settings_are_bootstrapped(self):
        registry = RecordingRegistry()
        scene = SimpleScene((FakeArmature(2, enabled=True, chains=1), FakeArmature(3, enabled=False, chains=1), FakeArmature(4, enabled=True, chains=0)))

        runtime_registry._bootstrap_enabled_worlds((scene,), registry=registry)

        self.assertEqual(registry.calls, [(1, 2)])

    def test_runtime_handlers_are_persistent_across_blend_load(self):
        source = (
            Path(__file__).parents[2]
            / "blender_extension"
            / "bone_physics"
            / "runtime_registry.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(source)
        decorated = {
            node.name
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
            and node.name in {"_frame_change_handler", "_load_post_handler"}
            and any(
                isinstance(decorator, ast.Name) and decorator.id == "persistent"
                for decorator in node.decorator_list
            )
        }

        self.assertEqual(decorated, {"_frame_change_handler", "_load_post_handler"})

    def test_runtime_native_is_validated_before_world_creation(self):
        fake_native = object()
        with mock.patch.object(runtime_registry, "load_native", return_value=fake_native) as loader:
            self.assertIs(runtime_registry._runtime_native(), fake_native)
        loader.assert_called_once_with(runtime_registry.NATIVE_API_VERSION)

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

    def test_flush_dirty_discards_removed_blender_armatures(self):
        registry = RuntimeRegistry(factory=Factory(handles=[10]))
        scene, armature = Pointer(1), Pointer(2)
        registry.rebuild(scene, armature, valid_spec())
        entry = registry._worlds[(1, 2)]
        entry.armature = RemovedPointer()
        registry.mark_dirty(2, "parameters")

        registry.flush_dirty()

        self.assertEqual(registry._worlds, {})
        self.assertTrue(entry.world.destroyed)
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

        self.assertEqual(len(settings.diagnostic_items), 2)
        self.assertEqual(settings.diagnostic_items[0].code, "authoring.validation")
        self.assertEqual(settings.diagnostic_items[0].entity_kind, "armature")
        self.assertEqual(settings.diagnostic_items[1].entity_kind, "runtime")
        self.assertTrue(settings.diagnostic_items[1].code.startswith("authoring."))

    def test_injected_validator_issue_is_persisted_on_compile_failure(self):
        settings = DiagnosticSettings()
        armature = DiagnosticArmature(2, settings)
        issue = ValidationIssue("chain.invalid", ValidationSeverity.ERROR, ("hair",))
        registry = RuntimeRegistry(
            factory=Factory(handles=[]),
            spec_builder=lambda _scene, _armature: (_ for _ in ()).throw(ValueError("bad")),
            validator=lambda _scene, _armature: (issue,),
        )
        with self.assertRaises(ValueError):
            registry.ensure(Pointer(1), armature)
        self.assertEqual(settings.diagnostic_items[0].code, "chain.invalid")


if __name__ == "__main__":
    unittest.main()


class Pointer:
    def __init__(self, value):
        self.value = value

    def as_pointer(self):
        return self.value


class RemovedPointer:
    def as_pointer(self):
        raise ReferenceError("StructRNA of type Object has been removed")


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


class RecordingRegistry:
    def __init__(self):
        self.calls = []

    def ensure(self, scene, armature):
        self.calls.append((scene.as_pointer(), armature.as_pointer()))


class SimpleScene(Pointer):
    def __init__(self, objects):
        super().__init__(1)
        self.objects = objects


class FakeArmature(Pointer):
    def __init__(self, value, *, enabled, chains):
        super().__init__(value)
        self.type = "ARMATURE"
        self.bone_physics = SimpleNamespace(
            global_enabled=enabled,
            physics_systems=[
                SimpleNamespace(enabled=True, chains=[object()] * chains)
            ],
        )
