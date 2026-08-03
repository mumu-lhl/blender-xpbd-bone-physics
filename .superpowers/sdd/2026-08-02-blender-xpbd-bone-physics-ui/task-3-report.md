# Task 3 report: chain discovery and branch selection

## Result

- Added Blender-independent `BoneNode` snapshots and deterministic parent-link
  traversal. Blender's connected state is neither consumed nor inferred.
- Child and descendant-leaf ordering is `(name, uuid)`; unbranched heads choose
  their sole deepest leaf automatically.
- Branches require an explicit descendant leaf and report the stable leaf list.
- Invalid topologies reject duplicate UUIDs, missing parents, and parent cycles
  with stable errors. New exact paths and all physical-bone reuse are rejected.

## TDD evidence

Initial RED:

```text
python -m unittest tests.python.test_chain_discovery -v
ModuleNotFoundError: No module named
'blender_extension.bone_physics.chain_discovery'
```

GREEN:

```text
python -m unittest tests.python.test_chain_discovery -v
Ran 6 tests in 0.001s
OK

python -m compileall -q blender_extension/bone_physics
git diff --check
```
