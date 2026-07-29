# V1 Core Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a deterministic, Blender-independent Rust foundation with oriented bone bodies, fixed-step time, prediction, and velocity reconstruction.

**Architecture:** A two-crate Cargo workspace keeps all physics in `bone_physics_core` and reserves `bone_physics_py` for the later PyO3 bridge. Core state uses stable integer IDs, `glam` math, explicit descriptors versus mutable state, and no unsafe code.

**Tech Stack:** Rust stable, Cargo, glam, thiserror, proptest, GitHub Actions.

## Handoff Status — Complete

All tasks in this plan were implemented and merged through PR #1. The
constraints-and-collision work continues in
`docs/plans/v1-constraints-collision.md`.

## Global Constraints

- Target Blender 5.2 LTS.
- The solver core is Rust and does not depend on Python or Blender.
- `bone_physics_core` uses `#![forbid(unsafe_code)]`.
- Simulation state uses `f32` in a simulation-local frame.
- IDs, iteration order, and test seeds are deterministic.
- Users install no Rust, pip package, NumPy, or system physics library.
- No wind, GPU solving, tearing, or PMX/Bullet numerical compatibility.

## File Map

- `Cargo.toml`: workspace members and release profile.
- `rust-toolchain.toml`: pinned stable toolchain components.
- `crates/bone_physics_core/Cargo.toml`: core dependencies and test dependencies.
- `crates/bone_physics_core/src/lib.rs`: public core API.
- `crates/bone_physics_core/src/id.rs`: typed stable IDs.
- `crates/bone_physics_core/src/error.rs`: validation and runtime errors.
- `crates/bone_physics_core/src/pose.rs`: normalized pose and velocity types.
- `crates/bone_physics_core/src/body.rs`: immutable body description and mutable state.
- `crates/bone_physics_core/src/time.rs`: fixed-step accumulator.
- `crates/bone_physics_core/src/world.rs`: body storage, prediction, and velocity rebuild.
- `.github/workflows/ci.yml`: authoritative Rust checks and uploaded reports.

---

### Task 1: Workspace, stable IDs, and validation errors

**Files:**
- Create: `Cargo.toml`
- Create: `rust-toolchain.toml`
- Create: `crates/bone_physics_core/Cargo.toml`
- Create: `crates/bone_physics_core/src/lib.rs`
- Create: `crates/bone_physics_core/src/id.rs`
- Create: `crates/bone_physics_core/src/error.rs`
- Test: `crates/bone_physics_core/tests/id_and_error.rs`

**Interfaces:**
- Produces: `BodyId(u32)`, `ConstraintId(u32)`, `ColliderId(u32)`.
- Produces: `SolverError::{DuplicateId, InvalidValue, MissingBody, NonFiniteState}`.

- [x] **Step 1: Write the failing ID and validation tests**

```rust
use bone_physics_core::{BodyId, SolverError};

#[test]
fn typed_ids_are_ordered_and_stable() {
    let mut ids = [BodyId::new(7), BodyId::new(2)];
    ids.sort();
    assert_eq!(ids, [BodyId::new(2), BodyId::new(7)]);
}

#[test]
fn invalid_value_error_keeps_field_name() {
    let err = SolverError::invalid_value("radius", -1.0);
    assert!(err.to_string().contains("radius"));
}
```

- [x] **Step 2: Commit the test-only change and verify CI fails**

Run in CI: `cargo test --workspace --all-targets`
Expected: FAIL because the crate and types do not exist.

- [x] **Step 3: Add the minimal workspace and types**

```rust
#![forbid(unsafe_code)]

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BodyId(u32);

impl BodyId {
    pub const fn new(raw: u32) -> Self { Self(raw) }
    pub const fn get(self) -> u32 { self.0 }
}
```

Use equivalent typed newtypes for constraints and colliders. Implement
`SolverError` with `thiserror` and reject non-finite scalar input through a
shared `validate_finite(field, value)` helper.

- [x] **Step 4: Run formatting, lint, and tests**

Run:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

Expected: all commands exit zero.

- [x] **Step 5: Commit**

```bash
git add Cargo.toml rust-toolchain.toml crates/bone_physics_core
git commit -m "build: initialize Rust solver workspace"
```

### Task 2: Pose, body description, and body state

**Files:**
- Create: `crates/bone_physics_core/src/pose.rs`
- Create: `crates/bone_physics_core/src/body.rs`
- Modify: `crates/bone_physics_core/src/lib.rs`
- Test: `crates/bone_physics_core/tests/body_state.rs`

**Interfaces:**
- Consumes: `BodyId`, `SolverError`.
- Produces: `Pose { translation: Vec3, rotation: Quat }`.
- Produces: `Velocity { linear: Vec3, angular: Vec3 }`.
- Produces: `BoneBodyDesc::validate() -> Result<(), SolverError>`.
- Produces: `BoneBodyState::from_desc(&BoneBodyDesc, Pose)`.

- [x] **Step 1: Write failing descriptor tests**

```rust
#[test]
fn rejects_zero_length_body() {
    let desc = fixture_desc(Vec3::ZERO, Vec3::ZERO);
    assert!(desc.validate().is_err());
}

#[test]
fn computes_capsule_center_of_mass_pose() {
    let desc = fixture_desc(Vec3::Y * -0.5, Vec3::Y * 0.5);
    let state = BoneBodyState::from_desc(&desc, Pose::IDENTITY).unwrap();
    assert_eq!(state.pose, Pose::IDENTITY);
    assert!(state.pose.rotation.is_normalized());
}
```

- [x] **Step 2: Verify the tests fail in CI**

Run: `cargo test -p bone_physics_core --test body_state`
Expected: FAIL because body types do not exist.

- [x] **Step 3: Implement validated immutable and mutable body data**

```rust
pub struct BoneBodyDesc {
    pub id: BodyId,
    pub local_head: Vec3,
    pub local_tail: Vec3,
    pub inverse_mass: f32,
    pub inverse_inertia: Vec3,
    pub gravity_scale: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

pub struct BoneBodyState {
    pub pose: Pose,
    pub previous_pose: Pose,
    pub predicted_pose: Pose,
    pub velocity: Velocity,
    pub contact_translation_correction: Vec3,
    pub contact_rotation_correction: Vec3,
}
```

Validate positive bone length, nonnegative inverse properties and damping, and
finite normalized poses. Quaternion constructors normalize once and reject
near-zero quaternions.

- [x] **Step 4: Run the focused and full test suites**

Run:

```bash
cargo test -p bone_physics_core --test body_state
cargo test --workspace --all-targets
```

Expected: PASS.

- [x] **Step 5: Commit**

```bash
git add crates/bone_physics_core
git commit -m "feat: add oriented bone body state"
```

### Task 3: Fixed-step clock

**Files:**
- Create: `crates/bone_physics_core/src/time.rs`
- Modify: `crates/bone_physics_core/src/lib.rs`
- Test: `crates/bone_physics_core/tests/fixed_clock.rs`

**Interfaces:**
- Produces: `FixedClock::new(step_seconds: f32)`.
- Produces: `FixedClock::try_accumulate(frame_seconds: f32, quality: QualityPolicy) -> Result<StepBatch, SolverError>`.
- Produces: `StepBatch { steps: u32, dt: f32, alpha: f32, dropped: bool }`.
- Produces: `QualityPolicy::{DeterministicPreview, ApproximatePreview { max_steps }, AuthoritativeBake}`.
- Produces: `FixedClock::reset()`.

- [x] **Step 1: Write failing frame-rate equivalence tests**

```rust
#[test]
fn one_second_has_same_step_count_at_common_frame_rates() {
    for fps in [24_u32, 30, 60] {
        let mut clock = FixedClock::new(1.0 / 120.0).unwrap();
        let steps: u32 = (0..fps)
            .map(|_| {
                clock
                    .try_accumulate(
                        1.0 / fps as f32,
                        QualityPolicy::AuthoritativeBake,
                    )
                    .unwrap()
                    .steps
            })
            .sum();
        assert_eq!(steps, 120);
    }
}

#[test]
fn negative_or_non_finite_frame_delta_is_rejected() {
    let mut clock = FixedClock::new(1.0 / 120.0).unwrap();
    assert!(clock
        .try_accumulate(f32::NAN, QualityPolicy::DeterministicPreview)
        .is_err());
}
```

- [x] **Step 2: Verify the focused test fails**

Run: `cargo test -p bone_physics_core --test fixed_clock`
Expected: FAIL because `FixedClock` is absent.

- [x] **Step 3: Implement an integer-tick-backed accumulator**

Store accumulated time as `f64` internally to reduce drift while returning
`f32` fixed steps to the solver. Only `ApproximatePreview { max_steps }` may
mark a batch as dropped; deterministic preview and authoritative bake never
discard time.

- [x] **Step 4: Run tests**

Run:

```bash
cargo test -p bone_physics_core --test fixed_clock
cargo test --workspace --all-targets
```

Expected: PASS with exactly 120 steps for each tested frame rate.

- [x] **Step 5: Commit**

```bash
git add crates/bone_physics_core
git commit -m "feat: add deterministic fixed-step clock"
```

### Task 4: Solver world prediction and velocity reconstruction

**Files:**
- Create: `crates/bone_physics_core/src/world.rs`
- Modify: `crates/bone_physics_core/src/lib.rs`
- Test: `crates/bone_physics_core/tests/world_step.rs`

**Interfaces:**
- Consumes: `BoneBodyDesc`, `BoneBodyState`, `FixedClock`.
- Produces: `SolverConfig { gravity: Vec3, fixed_dt: f32 }`.
- Produces: `SolverWorld::new(config, bodies)`.
- Produces: internal `SolverWorld::predict(dt)`.
- Produces: `SolverWorld::rebuild_velocities(dt)`.
- Produces: `SolverWorld::body(id) -> Option<&BoneBodyState>`.

- [x] **Step 1: Write failing prediction tests**

```rust
#[test]
fn gravity_is_frame_rate_independent() {
    let a = run_free_body(24, 1.0);
    let b = run_free_body(60, 1.0);
    assert!((a.translation - b.translation).length() < 5.0e-5);
}

#[test]
fn exponential_damping_is_per_second() {
    let v24 = run_damped_body(24, 1.0);
    let v60 = run_damped_body(60, 1.0);
    assert!((v24 - v60).length() < 1.0e-5);
}
```

- [x] **Step 2: Verify the tests fail**

Run: `cargo test -p bone_physics_core --test world_step`
Expected: FAIL because `SolverWorld` is absent.

- [x] **Step 3: Implement prediction and quaternion integration**

Use semi-implicit velocity prediction:

```rust
velocity.linear += gravity * desc.gravity_scale * dt;
velocity.linear *= (-desc.linear_damping * dt).exp();
predicted.translation = pose.translation + velocity.linear * dt;
```

Integrate angular velocity through an exponential-map quaternion increment.
After future constraints modify `predicted_pose`, reconstruct velocities from
the old and corrected poses. Subtract explicitly recorded depenetration-only
corrections before the velocity phase.

- [x] **Step 4: Run all core checks**

Run:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

Expected: all commands exit zero and frame-rate tests satisfy the declared
tolerance.

- [x] **Step 5: Commit**

```bash
git add crates/bone_physics_core
git commit -m "feat: add oriented world prediction"
```

### Task 5: Authoritative Rust CI

**Files:**
- Create: `.github/workflows/ci.yml`
- Create: `scripts/ci/capture.sh`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: Cargo workspace.
- Produces: `rust-ci-report` artifact on success or failure.
- Produces: required job `rust-core`.

- [x] **Step 1: Add a workflow that intentionally runs the current checks**

The `rust-core` job runs:

```yaml
- run: cargo fmt --all --check 2>&1 | tee artifacts/cargo-fmt.log
- run: cargo clippy --workspace --all-targets -- -D warnings 2>&1 | tee artifacts/cargo-clippy.log
- run: cargo test --workspace --all-targets 2>&1 | tee artifacts/cargo-test.log
- if: always()
  uses: actions/upload-artifact@v4
  with:
    name: rust-ci-report
    path: artifacts/
```

- [x] **Step 2: Push the workflow branch and verify a pull-request run starts**

Expected: GitHub reports a `rust-core` job for the head commit.

- [x] **Step 3: Inspect the uploaded report and fix any workflow-only issue**

The artifact must contain all three nonempty log files. Do not accept only a
green status without the report.

- [x] **Step 4: Re-run CI after fixes**

Expected: `rust-core` succeeds and the report remains downloadable.

- [x] **Step 5: Commit**

```bash
git add .github/workflows/ci.yml scripts/ci/capture.sh .gitignore
git commit -m "ci: validate Rust core"
```
