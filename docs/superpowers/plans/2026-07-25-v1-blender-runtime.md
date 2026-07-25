# V1 PyO3 and Blender Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose the Rust solver through a versioned coarse-grained PyO3 API and run it safely from a minimal Blender extension without per-body Python work in the solve loop.

**Architecture:** `bone_physics_py` owns one opaque native world registry and converts validated contiguous batch payloads to core descriptors. Blender-independent Python modules define serialized models, parameter compilation, cache keys, and timeline transitions. Thin `bpy` adapters evaluate the pre-physics pose, advance a runtime world, and write pose deltas under a re-entry guard.

**Tech Stack:** Rust stable, PyO3, maturin, Python 3.11+, stdlib `unittest`, Blender 5.2 LTS background mode.

## Global Constraints

- Depends on the completed core and primitive-collision plans.
- The bridge exposes no Python proxy per Rust body, constraint, or contact.
- `native_api_version` is checked before world creation.
- Steady-state stepping performs no Python callback.
- Rust panics are caught and translated to a Python exception.
- Blender-independent modules do not import `bpy`.
- Dynamic output is never read back while constructing the same time-point's pre-physics input.
- The extension bundle requires no pip installation, NumPy, Rust toolchain, or system physics library.

## File Map

- `crates/bone_physics_py/Cargo.toml`: PyO3 bridge dependencies and features.
- `crates/bone_physics_py/pyproject.toml`: maturin build metadata.
- `crates/bone_physics_py/src/lib.rs`: module registration and API version.
- `crates/bone_physics_py/src/error.rs`: panic/error translation.
- `crates/bone_physics_py/src/batch.rs`: validated batch decoders.
- `crates/bone_physics_py/src/registry.rs`: generation-checked opaque handles.
- `crates/bone_physics_py/src/world.rs`: coarse world methods.
- `blender_extension/blender_manifest.toml`: extension manifest.
- `blender_extension/bone_physics/__init__.py`: registration boundary.
- `blender_extension/bone_physics/native.py`: packaged native loader.
- `blender_extension/bone_physics/model.py`: pure Python serialized models.
- `blender_extension/bone_physics/parameter_builder.py`: chain compilation.
- `blender_extension/bone_physics/collider_builder.py`: analytic collider batches.
- `blender_extension/bone_physics/timeline.py`: pure timeline state machine.
- `blender_extension/bone_physics/scene_adapter.py`: thin `bpy` read adapter.
- `blender_extension/bone_physics/prephysics_eval.py`: pre-physics snapshots.
- `blender_extension/bone_physics/pose_writer.py`: final pose deltas.
- `blender_extension/bone_physics/runtime.py`: handler and world lifetime.
- `tests/python/`: ordinary CPython tests.
- `tests/blender/`: deterministic background Blender tests.

---

### Task 1: PyO3 crate, API version, and safe handle registry

**Files:**
- Modify: `Cargo.toml`
- Create: `crates/bone_physics_py/Cargo.toml`
- Create: `crates/bone_physics_py/pyproject.toml`
- Create: `crates/bone_physics_py/src/lib.rs`
- Create: `crates/bone_physics_py/src/error.rs`
- Create: `crates/bone_physics_py/src/registry.rs`
- Test: `crates/bone_physics_py/tests/registry.rs`
- Test: `tests/python/test_native_version.py`

**Interfaces:**
- Produces: Python function `native_api_version() -> int`.
- Produces: Python functions `create_world(config, build_batch) -> int` and `destroy_world(handle)`.
- Produces: generation-checked `WorldHandle(u64)`.
- Produces: Python exceptions `BonePhysicsInputError`, `BonePhysicsStateError`, and `BonePhysicsInternalError`.

- [ ] **Step 1: Write failing registry and Python contract tests**

Test stale handles after destroy/recreate, double destroy, API version equality,
invalid configuration, and a deliberately injected core panic behind a
test-only feature.

- [ ] **Step 2: Verify failures**

Run:

```bash
cargo test -p bone_physics_py
maturin develop --manifest-path crates/bone_physics_py/Cargo.toml
python -m unittest tests.python.test_native_version
```

Expected: FAIL because the bridge does not exist.

- [ ] **Step 3: Implement the minimal extension module**

Use `pyo3` with `extension-module` and `abi3-py311`. A process-wide registry
stores `SolverWorld` by slot and generation. `catch_unwind` surrounds each
entry point and translates a panic without exposing a Rust unwind to Blender.
Do not add unsafe buffer code in this task.

- [ ] **Step 4: Run checks**

Run:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
maturin build --manifest-path crates/bone_physics_py/Cargo.toml
```

Expected: PASS and one importable wheel for the current platform.

- [ ] **Step 5: Commit**

```bash
git add Cargo.toml crates/bone_physics_py tests/python
git commit -m "feat: add versioned PyO3 world bridge"
```

### Task 2: Validated batch world contract

**Files:**
- Create: `crates/bone_physics_py/src/batch.rs`
- Create: `crates/bone_physics_py/src/world.rs`
- Modify: `crates/bone_physics_py/src/lib.rs`
- Test: `tests/python/test_native_batches.py`
- Test: `crates/bone_physics_py/tests/batch_validation.rs`

**Interfaces:**
- Consumes Python buffer-protocol `f32`, `u32`, and `u64` arrays plus shape metadata.
- Produces: `build_world(config, bodies, constraints, filters) -> handle`.
- Produces: `set_animation_targets(handle, pose_batch)`.
- Produces: `set_analytic_colliders(handle, collider_batch)`.
- Produces: `advance(handle, frame_seconds, quality) -> StepSummary`.
- Produces: `read_transforms(handle, writable_pose_batch)`.
- Produces: `read_diagnostics(handle) -> dict[str, int | float | str]`.

- [ ] **Step 1: Write failing batch-shape tests**

Reject wrong dtype, non-contiguous buffers, missing columns, duplicate IDs,
NaN, stale handles, undersized writable outputs, and API-version mismatch.
Verify a valid two-body batch advances and returns normalized quaternions.

- [ ] **Step 2: Verify failure**

Run: `python -m unittest tests.python.test_native_batches`  
Expected: FAIL because batch methods do not exist.

- [ ] **Step 3: Implement owned-copy decoding first**

Validate every buffer before creating or mutating a world. Copy data into typed
Rust vectors, release Python borrows, then enter the solver. The only Python
object returned per advance is a small summary; transforms are written into a
caller-owned contiguous output buffer.

- [ ] **Step 4: Document any later zero-copy optimization boundary**

If profiling later justifies unsafe zero-copy access, isolate it in
`src/unsafe_buffer.rs` with pointer, length, alignment, lifetime, ownership,
mutation, and validation invariants beside each unsafe block. V1 remains on
the safe owned-copy path unless the performance report proves it necessary.

- [ ] **Step 5: Run and commit**

Run:

```bash
cargo test --workspace --all-targets
maturin develop --manifest-path crates/bone_physics_py/Cargo.toml
python -m unittest discover -s tests/python -v
```

Expected: PASS.

```bash
git add crates/bone_physics_py tests/python
git commit -m "feat: expose batched solver operations"
```

### Task 3: Pure Python data model and parameter compilation

**Files:**
- Create: `blender_extension/blender_manifest.toml`
- Create: `blender_extension/bone_physics/__init__.py`
- Create: `blender_extension/bone_physics/model.py`
- Create: `blender_extension/bone_physics/parameter_builder.py`
- Create: `blender_extension/bone_physics/collider_builder.py`
- Test: `tests/python/test_model_validation.py`
- Test: `tests/python/test_parameter_builder.py`
- Test: `tests/python/test_collider_builder.py`

**Interfaces:**
- Produces immutable dataclasses `BoneSpec`, `ChainSpec`, `CrossLinkSpec`, `AnalyticColliderSpec`, and `WorldSpec`.
- Produces: `compile_world_spec(spec: WorldSpec) -> NativeBuildBatch`.
- Produces stable IDs by explicit serialized identifiers, never object addresses or Python hash randomization.

- [ ] **Step 1: Write failing pure-Python tests**

Test accumulated-rest-length curves, unequal-chain matching, skirt seam and
diagonals, XYZ axis locks, duplicate IDs, illegal ranges, collision ownership
exclusions, and deterministic byte-identical batches from repeated builds.

- [ ] **Step 2: Verify failure without Blender**

Run: `python -m unittest discover -s tests/python -p 'test_*builder.py' -v`  
Expected: FAIL because the modules are absent.

- [ ] **Step 3: Implement strict dataclasses and batch compiler**

Use only Python standard-library types (`array`, `dataclasses`, `enum`,
`hashlib`, `struct`). The compiler flattens all bodies, constraints, local
anchors, curve samples, layers, masks, and exclusions into typed arrays.

- [ ] **Step 4: Prove no Blender import**

Add a test that installs an import hook rejecting `bpy`, imports every pure
module, and compiles a reference skirt.

- [ ] **Step 5: Run and commit**

Run:

```bash
python -m compileall -q blender_extension/bone_physics tests/python
python -m unittest discover -s tests/python -v
```

Expected: PASS under ordinary CPython.

```bash
git add blender_extension tests/python
git commit -m "feat: compile Python physics specifications"
```

### Task 4: Native loader and extension compatibility boundary

**Files:**
- Create: `blender_extension/bone_physics/native.py`
- Modify: `blender_extension/bone_physics/__init__.py`
- Test: `tests/python/test_native_loader.py`

**Interfaces:**
- Produces: `load_native(expected_api: int) -> NativeModule`.
- Produces: `NativeUnavailableError` and `NativeVersionError`.
- Consumes packaged module locations for Linux, Windows, and macOS.

- [ ] **Step 1: Write failing loader tests**

Inject fake modules for missing binary, import failure, wrong API version, and
correct API version. Assert no world can be created before compatibility is
confirmed.

- [ ] **Step 2: Implement platform-neutral loading**

Load only from the installed extension package. Include the original exception
and platform tag in errors without trying pip or downloading a binary.

- [ ] **Step 3: Validate manifest**

Add a headless script that loads the extension manifest, checks the Blender
version range and GPL license metadata, and verifies exactly one matching
native module is selected for the running platform.

- [ ] **Step 4: Run and commit**

Run: `python -m unittest tests.python.test_native_loader -v`  
Expected: PASS.

```bash
git add blender_extension tests/python
git commit -m "feat: validate bundled native solver"
```

### Task 5: Pre-physics scene evaluation and pose writing

**Files:**
- Create: `blender_extension/bone_physics/scene_adapter.py`
- Create: `blender_extension/bone_physics/prephysics_eval.py`
- Create: `blender_extension/bone_physics/pose_writer.py`
- Test: `tests/blender/generate_runtime_scene.py`
- Test: `tests/blender/test_prephysics_order.py`
- Test: `tests/blender/test_pose_roundtrip.py`

**Interfaces:**
- Produces: `PrePhysicsSnapshot { time, target_poses, collider_poses, object_to_simulation }`.
- Produces: `PrePhysicsEvaluator.capture(scene, depsgraph, world_spec)`.
- Produces: `PoseWriter.apply(armature, body_ids, simulation_poses)`.
- Produces simulation-local transforms with nonuniform scale baked before native upload.

- [ ] **Step 1: Generate a deterministic Blender fixture**

Create an animated torso armature, two physical hair bones, one analytic proxy,
and a driver counter that detects dependency-graph re-entry.

- [ ] **Step 2: Write failing background tests**

Assert:

- the captured targets equal ordinary animation poses;
- physical output from the preceding step is not present in the same time-point snapshot;
- object, armature, and bone transforms round-trip through simulation-local space;
- applying output changes only declared physical pose channels;
- negative/unhandled transforms fail validation.

- [ ] **Step 3: Implement thin Blender adapters**

Read evaluated data once into Python arrays. Keep physical bones at their
animation-reference transforms during collision input capture. Write deltas
only after native stepping. Do not include a solve loop in these modules.

- [ ] **Step 4: Run under Blender**

Run:

```bash
blender --background --factory-startup \
  --python tests/blender/test_prephysics_order.py
blender --background --factory-startup \
  --python tests/blender/test_pose_roundtrip.py
```

Expected: both exit zero and print one JSON result line.

- [ ] **Step 5: Commit**

```bash
git add blender_extension tests/blender
git commit -m "feat: evaluate and write Blender bone poses"
```

### Task 6: Timeline state machine and guarded runtime handler

**Files:**
- Create: `blender_extension/bone_physics/timeline.py`
- Create: `blender_extension/bone_physics/runtime.py`
- Test: `tests/python/test_timeline.py`
- Test: `tests/blender/test_runtime_handler.py`

**Interfaces:**
- Produces: `TimelineAction::{Advance, RestoreAndReplay, Reset, NoOp}`.
- Produces: `TimelineState.observe(time, cache_index) -> TimelineAction`.
- Produces: `RuntimeWorld.on_frame_change(scene, depsgraph)`.
- Produces: `ReentryGuard` scoped per Blender scene/runtime world.

- [ ] **Step 1: Write failing state-machine tests**

Cover sequential forward playback, skipped future frames, backward seek,
unchanged frame, invalidated checkpoint, changed FPS, and approximate-preview
state. No action may convert a jump into one oversized `dt`.

- [ ] **Step 2: Implement pure state transitions**

`Advance` contains a sequence of fixed physical time points.
`RestoreAndReplay` names the nearest compatible checkpoint supplied by a cache
index and the exact forward times. Checkpoint serialization and persistence are
implemented in the final plan. Approximate preview state is tagged
non-authoritative.

- [ ] **Step 3: Write and implement guarded Blender handler tests**

Run a frame change that triggers dependency-graph evaluation and assert the
handler executes exactly once, captures pre-physics data before solve, and
writes output after solve. On a bridge exception, retain the last valid
Blender pose and pause only the affected runtime world.

- [ ] **Step 4: Run and commit**

Run:

```bash
python -m unittest tests.python.test_timeline -v
blender --background --factory-startup \
  --python tests/blender/test_runtime_handler.py
```

Expected: PASS.

```bash
git add blender_extension tests
git commit -m "feat: add deterministic Blender runtime"
```

### Task 7: Python, bridge, and Blender CI jobs

**Files:**
- Modify: `.github/workflows/ci.yml`
- Create: `scripts/ci/install_blender.py`
- Create: `scripts/ci/run_blender_tests.py`
- Modify: `scripts/ci/capture.sh`

**Interfaces:**
- Produces required jobs `python-unit`, `native-contract`, and `blender-runtime`.
- Produces artifacts `python-test-report`, `native-contract-report`, and `blender-runtime-report`.

- [ ] **Step 1: Add ordinary Python job**

Use the minimum supported CPython and latest stable CPython. Run `compileall`
and stdlib `unittest`; do not install Blender or NumPy.

- [ ] **Step 2: Add native contract job**

Build the abi3 wheel with maturin, install it into an isolated virtual
environment, run all bridge tests, and inspect the wheel for unexpected shared
library dependencies.

- [ ] **Step 3: Add Blender 5.2 LTS Linux job**

Download Blender from its official release URL, verify a pinned SHA-256, unpack
inside the runner workspace, install the extension bundle, and run all scripts
under `tests/blender`.

- [ ] **Step 4: Upload evidence even on failure**

Each job writes command output, JUnit-compatible XML or structured JSON, and a
failure snapshot to its own artifact directory. Upload steps use `if: always()`.

- [ ] **Step 5: Verify the pull-request run**

Expected: all three required jobs succeed and all artifacts are nonempty.

- [ ] **Step 6: Commit**

```bash
git add .github/workflows/ci.yml scripts/ci
git commit -m "ci: test native and Blender runtime"
```
