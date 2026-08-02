# Task 2 report: immutable authoring records, validation, and stable IDs

## Result

- Added Blender-independent frozen records for armatures, physics systems,
  chains, parameter endpoints, collision policies, and validation issues.
- Added deterministic validation with stable error/warning codes, entity IDs,
  and structured arguments only; no display text is stored in the model.
- Covered duplicate UUID/stable-ID detection, overlapping chains, unresolved
  targets/resources, scalar and range checks, all-rigid cycles, and collision
  layer exhaustion. Recoverable collision/constraint diagnostics are warnings.
- Added a persisted `StableIdAllocator`: existing assignments are reused,
  new nonzero u32 identifiers are allocated monotonically while skipping saved
  assignments, and invalid/exhausted assignment state is rejected.

## TDD evidence

RED was recorded before implementation:

```text
ModuleNotFoundError: No module named 'blender_extension.bone_physics.authoring'
```

GREEN verification:

```text
python -m unittest tests.python.test_authoring -v
Ran 5 tests in 0.002s
OK

python -m compileall -q blender_extension/bone_physics
git diff --check
```

## Contract notes

- Validation is intentionally non-throwing for invalid authoring values so RNA
  conversion and preset loading can report all stable, localizable issues in
  one pass.
- Constraint cycles are assessed from the rigid-edge graph: a compliant edge
  is not treated as making an already all-rigid cycle valid.
- Mesh-topology warnings remain outside this pure record layer because they
  require an evaluated collision-surface snapshot.
