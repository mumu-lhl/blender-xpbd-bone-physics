# Task 1 report: general parameter fields and constraint compilation

## Result

- Added `ChainField`, `ChainFieldCurve`, and `AngularLimitMode` to the
  Blender-independent authoring model.
- Added validated swing-twist values and capsule insets to `BoneSpec`.
- Normalized the four legacy named curves into `ChainSpec.field_curves` and
  reject duplicate destinations, including legacy/general collisions.
- Added chain root-attachment metadata. Compilation emits native kind `3`
  using `body_b = 0`; compliance and damping sample from the root joint fields
  at `s = 0`.
- Replaced the four-field compiler special case with exhaustive field dispatch.
  Body properties sample span midpoints and constraint properties sample child
  head boundaries. Invalid sampled values remain rejected by `BoneSpec`.
- Compiled `SWING_TWIST` limits as native kind `6` in the existing 24-column
  layout; existing angular constraint IDs retain their prior XYZ compilation
  behavior when the new mode remains `FREE` for the migration release.

## TDD evidence

RED was recorded with:

```text
ImportError: cannot import name 'AngularLimitMode'
ImportError: cannot import name 'ChainField'
```

from the requested model/compiler test command, before implementation.

GREEN verification:

```text
python -m unittest tests.python.test_model_validation \
  tests.python.test_parameter_builder \
  tests.python.test_collider_builder -v
Ran 21 tests in 0.014s
OK (skipped=1)

python -m compileall -q blender_extension/bone_physics
```

`git diff --check` also passed.

## Self-review

- Native batch dimensions and API version are unchanged: 46 body columns and
  24 constraint columns.
- Discrete angular mode and Euler order are not represented by float curves.
- Constraint IDs include generated attachment IDs during duplicate validation.
- The focused native bridge test remains skipped because the local native
  module is not installed; the compiled row shapes match the existing Rust
  decoder for kinds `3` and `6`.
