# Codex Cloud Handoff

Status date: 2026-07-26
Repository: `mumu-lhl/blender-xpbd-bone-physics`
Current development PR: [#2](https://github.com/mumu-lhl/blender-xpbd-bone-physics/pull/2)

## 1. Purpose of This Handoff

Development is moving from ChatGPT Work to Codex Cloud. This file records the
auditable state that must survive that move: branch and CI evidence, completed
and unfinished work, design decisions, known deferred tests, and the required
toolchain migration.

The old chat is not a source of truth. The approved specification, tracked
plan, tests, code, and this handoff are the source of truth.

Read in this order:

1. `AGENTS.md`
2. `docs/design/blender-xpbd-bone-physics-design.md`
3. `docs/plans/v1-core-foundation.md`
4. `docs/plans/v1-constraints-collision.md`
5. this file
6. PR #2 and its latest CI

The two later approved phase plans are preserved for use after PR #2:

- `docs/plans/v1-blender-runtime.md`
- `docs/plans/v1-mesh-bake-release.md`

## 2. Repository and Pull Request State

| Item | Verified state before this documentation commit |
|---|---|
| PR #1 | Merged into `main` |
| PR #2 | Open Draft |
| PR #2 branch | `feat/v1-constraints-collision` |
| PR #2 base | `main` |
| Last verified code head | `e8c037a195c684adf4ed9341047bad6a2bd7fd47` |
| Last verified CI | Run `30210746067`, conclusion `success` |
| CI checks | `cargo fmt`, Clippy with `-D warnings`, complete workspace tests |
| Test evidence | 72 passed, 0 failed |
| CI artifact | `rust-ci-report`, artifact `8634377633`, 4306 bytes |

The documentation commit that contains this file necessarily advances the PR
head. Treat `e8c037a...` as the last verified **code** baseline, then verify the
new documentation-only head independently.

The former local worktree has a content-equivalent implementation commit named
`ac7a9b7`, but its commit graph differs from GitHub because earlier files were
synchronized with the GitHub Contents API. Codex Cloud must start from the
remote branch and must not reset it to that local SHA.

## 3. What Is Implemented

### Foundation

- validated stable `BodyId`, `ConstraintId`, and `ColliderId`;
- oriented rigid bone state with quaternion rotation;
- fixed-step clock and deterministic world prediction;
- finite-input validation and typed solver errors;
- velocity reconstruction that can exclude recorded non-velocity contact
  corrections.

### XPBD Constraints

- quaternion logarithm/exponential and world inverse inertia;
- scalar XPBD rows with compliance scaled by `dt²`;
- hard/compliant anchor, distance, and distance-range constraints;
- animation target interpolation and animation-relative inertia;
- constraint-relative damping and recovery damping;
- quaternion bend, twist, XYZ-box, and swing-twist limits;
- continuous XYZ branch tracking across `+π/-π`.

### Parameter Fields and Graph Construction

- constant, linear, and cubic Hermite parameter curves;
- rest-arc-length chain sampling;
- arbitrary cross-chain horizontal and diagonal links;
- closed lattices with one deterministic seam;
- normalized anchor de-duplication and finite-geometry validation;
- global all-rigid-cycle detection that still permits mixed rigid/compliant
  cycles;
- deterministic constraint IDs and overflow checks.

### Stable Scheduling

- stable constraint classification;
- greedy graph coloring after sorting by class and stable ID;
- no same-body write conflict inside one color;
- alternating traversal direction without changing color membership;
- world execution through the schedule without copying persistent multiplier
  state.

### Collision Detection

- continuous capsule collider representation;
- stable ordered `PairKey`;
- robust segment closest points and capsule manifolds;
- two-point manifolds for long parallel overlaps;
- finite deterministic fallbacks for coincidence and zero-length segments;
- direct-neighbor, explicit body, explicit constraint, and bidirectional
  layer/mask filtering without transitive chain exclusion;
- swept AABB candidate generation;
- deterministic sorted/unique pair output;
- equivalent sweep-and-prune and Parry BVH backends;
- no Parry type in the public API.

## 4. What Is Not Implemented

Task 6 detection is implemented and CI-tested, but its final focused code review
was not completed before the handoff.

Task 7 has not started. In particular:

- `SolverWorld::substep` does not generate or solve contacts;
- `StepOptions::contact_iterations` and `quality` are currently placeholders;
- there is no persistent contact cache or warm start;
- there is no unilateral XPBD contact row in the iteration schedule;
- there is no moving-surface-relative friction or new-impact restitution;
- analytic sphere/capsule/rounded-box body proxies are not implemented;
- rollback, pause state, and expanded diagnostics are not implemented.

Task 3 has one deliberately deferred integration regression: a real contact
must push a limited joint while both penetration and angular-limit thresholds
remain satisfied after the complete iterative loop. Add this only when Task 7
provides real contacts; do not fake a contact to make Task 3 appear closed.

Task 8 benchmarks, reference reports, and expanded CI artifacts have not
started.

The wider Blender extension remains future work: PyO3 batch bridge, Blender
runtime, pre-physics deforming mesh pipeline, mesh collision, bake/cache, and
packaging are specified in the two later plans but are outside the current PR.

## 5. Review History That Must Not Be Lost

- Task 4's first review found four Important issues: public curve mutation could
  bypass validation, internal arc-length boundaries could select the wrong
  bone, reverse/degenerate links were not de-duplicated, and all-rigid cycles
  were not detected across the whole lattice. Regression tests and fixes were
  added; the focused re-review closed all four with no new Critical/Important
  findings.
- Task 5's focused review found no Critical/Important issue. The schedule keeps
  one persistent multiplier state per constraint.
- Task 6a/6b/6c each completed RED/GREEN CI waves. The final Task 6 focused
  review remains outstanding.

## 6. Required Toolchain and Dependency Migration

The user wants the newest stable Rust and current direct dependencies. Perform
this migration before resuming Task 6 review or starting Task 7, and keep it in
a standalone commit with no new physics behavior.

Verified on 2026-07-26:

| Component | Current manifest/toolchain | Migration target |
|---|---:|---:|
| Rust | 1.85.1 | 1.97.1 |
| Rust edition | 2021 | 2024 |
| `rust-version` | 1.85 | 1.97 |
| `glam` | 0.30 | 0.33.2 |
| `parry3d` | 0.21.1 | 0.29.0 |
| `thiserror` | 2.0 range | 2.0.19 |

Authoritative version sources:

- <https://blog.rust-lang.org/releases/latest/>
- <https://docs.rs/glam/latest/glam/>
- <https://docs.rs/parry3d/latest/parry3d/>
- <https://crates.io/crates/thiserror>

Migration sequence:

1. Confirm the PR head is a descendant of the documentation commit and the
   working tree is clean.
2. Change `rust-toolchain.toml` to exact channel `1.97.1`.
3. Change workspace edition to `2024` and `rust-version` to `1.97`.
4. Upgrade direct dependencies to the target versions.
5. Adapt only the internal broad-phase backend from Parry 0.21.1 `Qbvh` to the
   Parry 0.29 API. Keep `BroadPhaseBackend` and `PairKey` stable and public.
6. Keep all 72 existing tests unchanged unless a compiler/API migration has a
   documented semantic reason for a test edit.
7. Run:

   ```bash
   cargo fmt --all --check
   cargo clippy --workspace --all-targets -- -D warnings
   cargo test --workspace --all-targets
   cargo tree --duplicates
   ```

8. Push the migration as its own commit and wait for fresh CI. Inspect the
   first real compiler/test failure rather than cycling dependency versions
   blindly.

Do not raise or remove the Rust pin merely to bypass an incompatible
dependency. The exact stable pin makes Cloud and CI reproducible.

## 7. Required Continuation Order

After the migration is green:

1. Perform a focused read-only review of Task 6 against the design and tests.
2. Fix any Critical/Important finding test-first and re-review.
3. Implement Task 7 in small RED/GREEN waves:
   - persistent contact key/cache and warm start;
   - normal XPBD contact inside every solver iteration;
   - non-velocity depenetration accounting;
   - moving-surface-relative friction;
   - restitution only for new closing impacts;
   - sphere, capsule, and rounded-box analytic proxies;
   - rollback, pause state, and expanded diagnostics;
   - the deferred limited-joint-plus-contact integration regression.
4. Implement Task 8 benchmarks, reference scenes, CI JSON, and artifact
   expansion.
5. Run a final independent review. Only then may PR #2 become Ready.

Do not merge PR #2 during these steps.

## 8. Codex Cloud Start Prompt

Use this prompt after selecting the repository and
`feat/v1-constraints-collision` branch:

> Continue `mumu-lhl/blender-xpbd-bone-physics` on Draft PR #2. Do not work on
> `main`, merge the PR, or add new physics behavior yet. Read `AGENTS.md`,
> `docs/design/blender-xpbd-bone-physics-design.md`,
> `docs/plans/v1-core-foundation.md`,
> `docs/plans/v1-constraints-collision.md`, and `docs/HANDOFF.md` completely.
> Verify the current remote head and fresh CI. First perform only the standalone
> Rust 1.97.1 / Edition 2024 / latest direct-dependency migration described in
> the handoff. Preserve existing tests and public physics semantics. Diagnose
> failures from compiler or test evidence, and complete fmt, Clippy, full tests,
> duplicate-dependency inspection, and CI before returning to Task 6 review.

## 9. Completion Evidence Standard

A stage is not complete because code was written or a previous run was green.
For the exact final head:

- run the complete local gate where available;
- require the GitHub Actions job to succeed;
- confirm the uploaded reports are present and nonempty;
- count tests and failures from the report;
- perform the task's focused review;
- record deferred work explicitly rather than simulating it.
