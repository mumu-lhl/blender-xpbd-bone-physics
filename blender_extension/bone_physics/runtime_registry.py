"""Main-thread ownership and transactional replacement of runtime worlds.

RNA update callbacks may run while Blender is evaluating data.  They only put
small scalar records in this module's lock-protected queue; a timer consumes
them later on Blender's main thread.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Protocol


_STRUCTURAL_CATEGORIES = frozenset({"chains", "constraints", "systems", "collisions", "world"})
_DIRTY_LOCK = threading.Lock()
_DIRTY: set[tuple[int, str, float]] = set()


class RuntimeWorldFactory(Protocol):
    def create(self, scene: object, armature: object, spec: object) -> object: ...


@dataclass(slots=True)
class RuntimeEntry:
    scene: object
    armature: object
    spec: object
    world: object


class RuntimeRegistry:
    """Own one isolated solver world for each scene/armature session pair."""

    def __init__(self, *, factory: RuntimeWorldFactory | None = None) -> None:
        self.factory = factory
        self._worlds: dict[tuple[int, int], RuntimeEntry] = {}
        self._pending_structural: dict[int, set[str]] = {}
        self._validated: set[int] = set()

    @staticmethod
    def key(scene: object | None = None, armature: object | None = None, *, scene_id: int | None = None, armature_id: int | None = None) -> tuple[int, int]:
        if scene_id is not None or armature_id is not None:
            if scene_id is None or armature_id is None:
                raise ValueError("scene_id and armature_id must be supplied together")
            return (int(scene_id), int(armature_id))
        if scene is None or armature is None:
            raise ValueError("scene and armature must be supplied")
        return (int(scene.as_pointer()), int(armature.as_pointer()))

    def rebuild(self, scene: object | None = None, armature: object | None = None, spec: object = None, *, scene_id: int | None = None, armature_id: int | None = None):
        if self.factory is None:
            raise RuntimeError("runtime registry has no world factory")
        key = self.key(scene, armature, scene_id=scene_id, armature_id=armature_id)
        # Create first: a validation/native failure leaves the live world intact.
        replacement = self.factory.create(scene, armature, spec)
        previous = self._worlds.get(key)
        self._worlds[key] = RuntimeEntry(scene, armature, spec, replacement)
        if previous is not None:
            _destroy(previous.world)
        return replacement

    def world(self, scene: object, armature: object):
        return self.require(scene, armature).world

    def world_by_key(self, key: tuple[int, int]):
        return self._worlds[key].world

    def require(self, scene: object, armature: object) -> RuntimeEntry:
        key = self.key(scene, armature)
        try:
            return self._worlds[key]
        except KeyError as error:
            raise RuntimeError("no runtime world is configured for this armature") from error

    def remove(self, scene: object, armature: object) -> None:
        entry = self._worlds.pop(self.key(scene, armature), None)
        if entry is not None:
            _destroy(entry.world)

    def mark_dirty(self, owner_pointer: int, category: str) -> None:
        mark_dirty(owner_pointer, category)

    def pending_structural(self, owner_pointer: int) -> tuple[str, ...]:
        return tuple(sorted(self._pending_structural.get(owner_pointer, ())))

    def mark_validated(self, owner_pointer: int) -> None:
        self._validated.add(int(owner_pointer))

    def flush_dirty(self) -> tuple[tuple[int, str], ...]:
        records = _take_dirty()
        ready: list[tuple[int, str]] = []
        for owner_pointer, category, _timestamp in records:
            if category in _STRUCTURAL_CATEGORIES and owner_pointer not in self._validated:
                self._pending_structural.setdefault(owner_pointer, set()).add(category)
                continue
            ready.append((owner_pointer, category))
        for owner_pointer in tuple(self._validated):
            for category in sorted(self._pending_structural.pop(owner_pointer, ())):
                ready.append((owner_pointer, category))
        self._validated.clear()
        result = tuple(sorted(set(ready)))
        for owner_pointer, _category in result:
            for entry in tuple(self._worlds.values()):
                pointer = getattr(entry.armature, "as_pointer", lambda: None)()
                if pointer == owner_pointer:
                    self.rebuild(entry.scene, entry.armature, entry.spec)
        return result

    def restore_configured_bones(self, context) -> RuntimeEntry:
        armature = context.object
        entry = self.require(context.scene, armature)
        depsgraph = context.evaluated_depsgraph_get()
        snapshot = entry.world.evaluator.capture(context.scene, depsgraph, entry.spec)
        references = {target.body_id: target.current for target in snapshot.target_poses}
        entry.world.evaluator.adapter.restore_animation_references(context.scene, references)
        return entry

    def reset_configured_bones(self, context) -> None:
        armature = context.object
        entry = self.restore_configured_bones(context)
        self.rebuild(context.scene, armature, entry.spec)
        cache = getattr(entry.world, "cache", None)
        invalidate = getattr(cache, "invalidate", None)
        if invalidate is not None:
            invalidate("physics reset")


def _destroy(world: object) -> None:
    destroy = getattr(world, "destroy", None)
    if destroy is not None:
        destroy()


def mark_dirty(owner_pointer: int, category: str) -> None:
    if not isinstance(owner_pointer, int):
        raise TypeError("dirty owner pointer must be an integer")
    if not category:
        raise ValueError("dirty category must not be empty")
    with _DIRTY_LOCK:
        _DIRTY.add((owner_pointer, category, time.monotonic()))


def _take_dirty() -> tuple[tuple[int, str, float], ...]:
    with _DIRTY_LOCK:
        records = tuple(_DIRTY)
        _DIRTY.clear()
    return records


REGISTRY = RuntimeRegistry()


def flush_dirty() -> tuple[tuple[int, str], ...]:
    return REGISTRY.flush_dirty()


def reset_configured_bones(context) -> None:
    REGISTRY.reset_configured_bones(context)


def _timer_tick():
    flush_dirty()
    return 0.2


def _frame_change_handler(scene, depsgraph) -> None:
    for entry in tuple(REGISTRY._worlds.values()):
        if entry.scene != scene or not getattr(entry.armature.bone_physics, "global_enabled", True):
            continue
        entry.world.on_frame_change(scene, depsgraph)


def register_timer() -> None:
    """Register a coalescing timer only when running inside Blender."""
    try:
        import bpy
    except ImportError:
        return

    if not bpy.app.timers.is_registered(_timer_tick):
        bpy.app.timers.register(_timer_tick, first_interval=0.2, persistent=True)
    if _frame_change_handler not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(_frame_change_handler)


def unregister_timer() -> None:
    try:
        import bpy
    except ImportError:
        return
    if bpy.app.timers.is_registered(_timer_tick):
        bpy.app.timers.unregister(_timer_tick)
    if _frame_change_handler in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(_frame_change_handler)
