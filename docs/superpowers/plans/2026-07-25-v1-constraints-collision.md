# V1 Constraints and Primitive Collision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the oriented-body foundation into a deterministic XPBD constraint graph that supports bone joints, animation recovery, angular limits, arbitrary cross-chain links, capsule self-collision, and analytic body collision.

**Architecture:** Persistent constraints and transient contacts share one iterative solve schedule. Constraint rows use stable IDs and cached multipliers; rotational corrections use world-space inverse inertia. Chain metadata is compiled into graph data before stepping. Collision detection produces sorted, filtered contact manifolds and never mutates bodies outside the solver.

**Tech Stack:** Rust stable, glam, parry3d through an internal adapter, proptest, criterion.

## Global Constraints

- Depends on the public types from `2026-07-25-v1-core-foundation.md`.
- `bone_physics_core` remains `#![forbid(unsafe_code)]`.
- Persistent constraints and contacts are solved inside every XPBD iteration.
- A final collision-only or limit-only pass is forbidden.
- Constraint and pair ordering derives only from stable IDs.
- Rotations remain quaternions; Euler values are an authoring/limit boundary.
- Exact coincident or parallel geometry must return a finite deterministic result.
- Hard parent joints may use zero compliance; cyclic cross-links must validate as compliant.

## File Map

- `crates/bone_physics_core/src/math.rs`: quaternion logarithm/exponential, world inverse inertia, robust bases.
- `crates/bone_physics_core/src/constraint/mod.rs`: persistent constraint enum and common solve context.
- `crates/bone_physics_core/src/constraint/xpbd.rs`: scalar and vector XPBD row helpers.
- `crates/bone_physics_core/src/constraint/joint.rs`: anchor, distance, and range constraints.
- `crates/bone_physics_core/src/constraint/recovery.rs`: animated attachment and orientation recovery.
- `crates/bone_physics_core/src/constraint/angular.rs`: bend, twist, XYZ, and swing-twist limits.
- `crates/bone_physics_core/src/graph.rs`: stable schedule, coloring, and iteration loop.
- `crates/bone_physics_core/src/parameter.rs`: rest-length parameter fields.
- `crates/bone_physics_core/src/chain.rs`: chain and cross-chain graph construction.
- `crates/bone_physics_core/src/collision/shape.rs`: capsules and analytic shapes.
- `crates/bone_physics_core/src/collision/filter.rs`: layers and structural exclusions.
- `crates/bone_physics_core/src/collision/broad_phase.rs`: deterministic swept-AABB candidates.
- `crates/bone_physics_core/src/collision/capsule.rs`: robust capsule manifolds.
- `crates/bone_physics_core/src/collision/contact.rs`: contact cache and XPBD/velocity response.
- `crates/bone_physics_core/src/collision/analytic.rs`: sphere, capsule, rounded-box queries.
- `crates/bone_physics_core/src/diagnostics.rs`: convergence and penetration counters.
- `crates/bone_physics_core/benches/solver.rs`: representative graph benchmarks.

---

### Task 1: Rotational math and XPBD row helpers

**Files:**
- Create: `crates/bone_physics_core/src/math.rs`
- Create: `crates/bone_physics_core/src/constraint/mod.rs`
- Create: `crates/bone_physics_core/src/constraint/xpbd.rs`
- Modify: `crates/bone_physics_core/src/lib.rs`
- Test: `crates/bone_physics_core/tests/math_and_xpbd.rs`

**Interfaces:**
- Produces: `quat_log(Quat) -> Vec3` and `quat_exp(Vec3) -> Quat`.
- Produces: `world_inverse_inertia(rotation, local_diagonal) -> Mat3`.
- Produces: `XpbdRow { c, compliance, lambda, gradients }`.
- Produces: `solve_scalar_row(&mut SolveBodies, &mut XpbdRow, dt) -> RowResult`.

- [ ] **Step 1: Write failing round-trip, branch, and compliance tests**

```rust
#[test]
fn quaternion_log_exp_round_trip_near_pi() {
    let q = Quat::from_axis_angle(Vec3::Y, core::f32::consts::PI - 1.0e-4);
    assert!(q.angle_between(quat_exp(quat_log(q))) < 1.0e-5);
}

#[test]
fn xpbd_compliance_scales_with_dt_squared() {
    let coarse = solve_spring(1.0 / 60.0, 60);
    let fine = solve_spring(1.0 / 120.0, 120);
    assert!((coarse - fine).abs() < 2.0e-4);
}
```

- [ ] **Step 2: Verify failure**

Run: `cargo test -p bone_physics_core --test math_and_xpbd`  
Expected: FAIL because the math and row APIs do not exist.

- [ ] **Step 3: Implement minimal robust helpers**

Use:

```text
alpha_tilde = compliance / (dt * dt)
delta_lambda = (-C - alpha_tilde * lambda)
             / (sum_weighted_gradient_squared + alpha_tilde)
```

Normalize quaternion results, use the shortest quaternion representative, and
return a non-applied row when the effective denominator is below `1e-12`.
Never manufacture an infinite correction.

- [ ] **Step 4: Run checks**

Run:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test -p bone_physics_core --test math_and_xpbd
```

Expected: PASS, including the near-π case.

- [ ] **Step 5: Commit**

```bash
git add crates/bone_physics_core
git commit -m "feat: add rotational XPBD primitives"
```

### Task 2: Joint, distance, and animated recovery constraints

**Files:**
- Create: `crates/bone_physics_core/src/constraint/joint.rs`
- Create: `crates/bone_physics_core/src/constraint/recovery.rs`
- Modify: `crates/bone_physics_core/src/constraint/mod.rs`
- Modify: `crates/bone_physics_core/src/world.rs`
- Test: `crates/bone_physics_core/tests/joint_constraints.rs`
- Test: `crates/bone_physics_core/tests/recovery_constraints.rs`

**Interfaces:**
- Produces: `ConstraintKind::{Anchor, Distance, DistanceRange, Attachment, OrientationRecovery}`.
- Produces: `PersistentConstraint { id, enabled, bodies, compliance, damping, lambda, kind }`.
- Produces: `AnimationTarget { previous: Pose, current: Pose, translation_inertia: f32, rotation_inertia: f32 }`.
- Produces: `SolverWorld::set_animation_targets(&[(BodyId, AnimationTarget)])`.

- [ ] **Step 1: Write failing physical-behavior tests**

Cover a hard parent anchor under gravity, a compliant distance link at two
fixed-step sizes, a distance inequality whose multiplier never attracts, and
a moving animation target at inertia factors `0`, `1`, and `1.5`.

```rust
assert!(world.anchor_error(joint_id) <= acceptance.hard_anchor);
assert!(world.constraint_lambda(range_id) >= 0.0);
```

- [ ] **Step 2: Verify failure**

Run:

```bash
cargo test -p bone_physics_core --test joint_constraints
cargo test -p bone_physics_core --test recovery_constraints
```

Expected: FAIL because the constraints and target batch are absent.

- [ ] **Step 3: Implement constraints and target interpolation**

Anchor constraints use local points and distribute translation and rotation
with inverse mass/inertia. Recovery constraints compare against the
interpolated pre-physics target inside each fixed substep. Damping acts on
constraint-relative velocity and is distinct from compliance.

- [ ] **Step 4: Integrate constraints into `SolverWorld::substep`**

Add:

```rust
pub struct StepOptions {
    pub position_iterations: u32,
    pub contact_iterations: u32,
    pub quality: QualityPolicy,
}

pub fn substep(
    &mut self,
    dt: f32,
    target_alpha: f32,
    options: StepOptions,
) -> Result<StepDiagnostics, SolverError>;
```

The step sequence is predict, generate contacts, iterate all scheduled classes,
validate, rebuild velocities, then apply velocity contact response.

- [ ] **Step 5: Run and commit**

Run: `cargo test --workspace --all-targets`  
Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: solve joint and animation recovery constraints"
```

### Task 3: Bend, twist, XYZ, and swing-twist angular limits

**Files:**
- Create: `crates/bone_physics_core/src/constraint/angular.rs`
- Modify: `crates/bone_physics_core/src/constraint/mod.rs`
- Test: `crates/bone_physics_core/tests/angular_constraints.rs`
- Property test: `crates/bone_physics_core/tests/angular_properties.rs`

**Interfaces:**
- Produces: `AngularLimit::{XyzBox(XyzLimit), SwingTwist(SwingTwistLimit)}`.
- Produces: `XyzLimit { lower: Vec3, upper: Vec3, order: EulerOrder }`.
- Produces: `SwingTwistLimit { axis: Vec3, max_swing: f32, min_twist: f32, max_twist: f32 }`.
- Produces: `ContinuousAngles` stored per XYZ constraint.

- [ ] **Step 1: Write failing limit tests**

Test:

- equal lower/upper values lock each individual axis;
- candidate rotations crossing `+π/-π` keep a continuous branch;
- a twist-only rotation is unchanged by an unconstrained swing cone;
- invalid lower/upper ranges are rejected;
- 10,000 deterministic random finite rotations never produce NaN.

- [ ] **Step 2: Verify failure**

Run:

```bash
cargo test -p bone_physics_core --test angular_constraints
cargo test -p bone_physics_core --test angular_properties
```

Expected: FAIL because angular constraint types are absent.

- [ ] **Step 3: Implement rest-frame projection**

Compute the relative rotation in the joint rest frame. For XYZ mode, unwrap
each selected Euler branch against `ContinuousAngles`, clamp it, and turn the
difference back into a quaternion correction. For swing-twist mode, decompose
about the declared rest-frame axis and project cone/range inequalities. A
constraint selects exactly one primary model.

- [ ] **Step 4: Prove collision cannot be permanently invalidated by limits**

Add a regression world in which a contact pushes a limited joint. Assert both
the angular and penetration thresholds after the complete iteration loop.

- [ ] **Step 5: Run and commit**

Run:

```bash
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: add quaternion angular constraints"
```

### Task 4: Parameter fields and arbitrary cross-chain graph construction

**Files:**
- Create: `crates/bone_physics_core/src/parameter.rs`
- Create: `crates/bone_physics_core/src/chain.rs`
- Test: `crates/bone_physics_core/tests/parameter_fields.rs`
- Test: `crates/bone_physics_core/tests/cross_chain.rs`

**Interfaces:**
- Produces: `ParameterCurve { points: Vec<CurvePoint>, interpolation: CurveInterpolation }`.
- Produces: `ChainDesc { bodies: Vec<BodyId>, rest_lengths: Vec<f32>, closed: bool }`.
- Produces: `CrossLinkSpec { chain_a, chain_b, samples, diagonals, compliance_curve }`.
- Produces: `sample_chain_s(&ChainDesc) -> Vec<f32>`.
- Produces: `build_cross_links(&CrossLinkSpec, &BodyLookup) -> Result<Vec<PersistentConstraint>, SolverError>`.

- [ ] **Step 1: Write failing geometry-based sampling tests**

Use uneven lengths `[1, 3, 2]` and assert the normalized coordinates are based
on accumulated rest length. Connect chains with different body counts and
assert anchors interpolate at matched `s`, not matched indices.

- [ ] **Step 2: Write failing skirt-lattice tests**

Build four unequal chains with a closed seam. Assert deterministic horizontal
and diagonal link IDs, no duplicate seam, and finite compliance on every
cycle-forming link.

- [ ] **Step 3: Implement curves and builders**

Support constant, linear, and piecewise cubic Hermite interpolation with
monotonic point coordinates. Sample concrete mass, inertia, gravity, damping,
recovery, radius, angular-limit, and cross-link values during graph build.
The runtime solver never evaluates authoring curves.

- [ ] **Step 4: Validate graph references**

Reject missing body IDs, duplicate generated constraint IDs, zero-length
chains, non-finite samples, and all-rigid cyclic cross-link components.

- [ ] **Step 5: Run and commit**

Run: `cargo test --workspace --all-targets`  
Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: build parameterized cross-chain graphs"
```

### Task 5: Stable scheduling and graph coloring

**Files:**
- Create: `crates/bone_physics_core/src/graph.rs`
- Modify: `crates/bone_physics_core/src/world.rs`
- Test: `crates/bone_physics_core/tests/constraint_schedule.rs`

**Interfaces:**
- Produces: `ConstraintClass::{Anchor, Recovery, Structural, AngularLimit, Contact}`.
- Produces: `ConstraintSchedule::build(&[PersistentConstraint])`.
- Produces: `ConstraintSchedule::colors(class) -> &[ConstraintColor]`.
- Produces: alternating stable direction by solver iteration.

- [ ] **Step 1: Write failing determinism and conflict tests**

Shuffle input constraints with a seeded permutation. Assert identical sorted
schedule and identical final poses. Within a color, assert no two constraints
write the same body.

- [ ] **Step 2: Verify failure**

Run: `cargo test -p bone_physics_core --test constraint_schedule`  
Expected: FAIL because the schedule does not exist.

- [ ] **Step 3: Implement stable greedy coloring**

Sort by `(class, ConstraintId)` before coloring. Store colors as ordered ID
vectors. Alternate traversal direction per iteration without changing color
membership. Keep execution serial initially; expose colors for later optional
Rayon execution.

- [ ] **Step 4: Integrate the exact class loop**

Every iteration visits anchor, recovery/bend/twist, structural, angular, and
contact classes. The next iteration always returns to anchor.

- [ ] **Step 5: Run and commit**

Run: `cargo test --workspace --all-targets`  
Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: add deterministic constraint scheduling"
```

### Task 6: Capsule broad phase, filtering, and robust manifolds

**Files:**
- Create: `crates/bone_physics_core/src/collision/mod.rs`
- Create: `crates/bone_physics_core/src/collision/shape.rs`
- Create: `crates/bone_physics_core/src/collision/filter.rs`
- Create: `crates/bone_physics_core/src/collision/broad_phase.rs`
- Create: `crates/bone_physics_core/src/collision/capsule.rs`
- Modify: `crates/bone_physics_core/src/body.rs`
- Test: `crates/bone_physics_core/tests/capsule_collision.rs`
- Property test: `crates/bone_physics_core/tests/collision_properties.rs`

**Interfaces:**
- Produces: `Capsule { local_a, local_b, radius, margin }`.
- Produces: `CollisionFilter { layer, mask, excluded_bodies, excluded_constraints }`.
- Produces: `PairKey::ordered(ColliderId, ColliderId)`.
- Produces: `CapsuleManifold { pair, points: Vec<ContactGeometry> }` with at most two points.
- Produces: `CollisionPipeline::candidate_pairs(previous, predicted)`.

- [ ] **Step 1: Write failing capsule regression tests**

Cover separated, touching, crossing, coincident-centerline, zero-length segment,
and long parallel-overlap pairs. Assert symmetry under pair reversal, finite
normals, positive penetration only when expected, and two contacts for stable
long overlap.

- [ ] **Step 2: Write filtering tests**

Assert self pairs, configured parent-child pairs, shared hard joints, layers,
masks, and structural-neighbor exclusions are rejected while non-neighbor
members of one chain remain eligible.

- [ ] **Step 3: Implement swept bounds and deterministic candidates**

The initial implementation may use sorted sweep-and-prune behind
`BroadPhaseBackend`; the public contract must accept previous and predicted
AABBs and return sorted unique `PairKey`s. Add a parry-backed BVH without
leaking parry types across the module boundary.

- [ ] **Step 4: Implement normal fallback order**

Use cached normal, relative velocity, axis cross product, then a deterministic
orthogonal vector derived from `PairKey`. Never skip exact coincidence.

- [ ] **Step 5: Run and commit**

Run:

```bash
cargo test -p bone_physics_core --test capsule_collision
cargo test -p bone_physics_core --test collision_properties
cargo test --workspace --all-targets
```

Expected: PASS and no generated NaN.

```bash
git add crates/bone_physics_core
git commit -m "feat: detect robust capsule contacts"
```

### Task 7: Contact constraints, friction, restitution, and analytic proxies

**Files:**
- Create: `crates/bone_physics_core/src/collision/contact.rs`
- Create: `crates/bone_physics_core/src/collision/analytic.rs`
- Modify: `crates/bone_physics_core/src/collision/mod.rs`
- Modify: `crates/bone_physics_core/src/world.rs`
- Create: `crates/bone_physics_core/src/diagnostics.rs`
- Test: `crates/bone_physics_core/tests/contact_response.rs`
- Test: `crates/bone_physics_core/tests/analytic_collision.rs`

**Interfaces:**
- Produces: `AnalyticShape::{Sphere, Capsule, RoundedBox}` with explicit pose and dimensions.
- Produces: `ContactKey { pair, feature_a, feature_b }`.
- Produces: `CachedContact { local_a, local_b, normal, tangents, lambdas, age }`.
- Produces: `ContactMaterial { friction, restitution }`.
- Produces: `StepDiagnostics { generated_contacts, active_contacts, deepest_penetration, max_constraint_error, rolled_back }`.

- [ ] **Step 1: Write failing contact-response tests**

Test zero-restitution resting contact for 10 seconds, moving-surface friction,
one new impact with restitution, no restitution on persistent resting contact,
and non-attractive normal multipliers.

- [ ] **Step 2: Write failing analytic-shape tests**

Test capsule versus sphere/capsule/rounded box, transforms with baked
nonuniform scale, a fast moving proxy using speculative separation, and an
initially embedded root with collision disabled.

- [ ] **Step 3: Implement contact lifecycle**

Generate and warm-start contacts before the main iterations. Solve unilateral
normal position rows in the contact class. Record depenetration translation and
rotation separately, subtract it during velocity reconstruction, then solve
surface-relative friction and new-impact restitution in the velocity phase.

- [ ] **Step 4: Add rollback and diagnostics**

Snapshot the last valid dynamic state at substep entry. On non-finite pose,
unnormalized quaternion, excessive multiplier, or solver error, restore the
snapshot, pause the world, and report the offending stable IDs.

- [ ] **Step 5: Run reference simulations**

Add deterministic single pendulum, colliding hair chains, root-stiff/tip-soft
tail, and closed skirt lattice fixtures. Write numerical summaries under the
test temporary directory and assert the specification tolerances.

- [ ] **Step 6: Run and commit**

Run:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

Expected: all checks pass; residual contact and angular errors satisfy the
declared authoritative thresholds.

```bash
git add crates/bone_physics_core
git commit -m "feat: solve primitive collision contacts"
```

### Task 8: Benchmarks and CI artifact expansion

**Files:**
- Modify: `crates/bone_physics_core/Cargo.toml`
- Create: `crates/bone_physics_core/benches/solver.rs`
- Modify: `.github/workflows/ci.yml`
- Create: `scripts/ci/reference_report.py`

**Interfaces:**
- Produces: benchmark groups `hair_64`, `skirt_12x8`, and `contacts_256`.
- Produces: `rust-reference-report` containing JSON numerical diagnostics.

- [ ] **Step 1: Add deterministic benchmark fixtures**

Criterion measurements must build the same stable graph on every invocation
and use a fixed input animation.

- [ ] **Step 2: Extend CI**

Run reference tests on pull requests and `cargo bench --no-run`. Run measured
benchmarks only on the scheduled workflow. Upload command output and numerical
JSON with `if: always()`.

- [ ] **Step 3: Verify CI artifacts**

Expected artifacts contain nonempty test output plus one JSON row per reference
scene with length error, joint error, penetration, peak velocities, contact
counts, and phase timings.

- [ ] **Step 4: Commit**

```bash
git add crates/bone_physics_core .github/workflows/ci.yml scripts/ci
git commit -m "test: add solver references and benchmarks"
```
