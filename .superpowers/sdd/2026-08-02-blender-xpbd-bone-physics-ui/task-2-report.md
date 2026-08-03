# Task 2 report: immutable authoring records, validation, and stable IDs

## Result

- Added Blender-independent frozen records for armatures, physics systems,
  chains, parameter endpoints, collision policies, and validation issues.
- Added deterministic validation with stable error/warning codes, entity IDs,
  and structured arguments only; no display text is stored in the model.
- Added a persisted `StableIdAllocator` with deterministic nonzero u32 IDs,
  including atomic allocation of `0xFFFFFFFF`.
- Added checks for duplicate IDs, overlap, unresolved targets/resources,
  scalar/range errors, all-rigid cycles, collision layer exhaustion, redundant
  constraints, and suspicious weight UUIDs.

## TDD evidence

Initial RED:

```text
ModuleNotFoundError: No module named 'blender_extension.bone_physics.authoring'
```

Initial GREEN:

```text
python -m unittest tests.python.test_authoring -v
Ran 5 tests in 0.002s
OK
```

Fix round 1 RED:

```text
python -m unittest tests.python.test_authoring -v
Ran 8 tests in 0.006s
FAILED (errors=3)
```

Fix round 1 GREEN:

```text
python -m unittest tests.python.test_authoring -v
Ran 8 tests in 0.004s
OK

python -m compileall -q blender_extension/bone_physics
git diff --check
```

Validation remains non-throwing for malformed authoring values. Mesh-topology
warnings requiring evaluated surfaces remain in the collision task.
