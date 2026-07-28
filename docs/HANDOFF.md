# Codex Cloud Handoff

Status date: 2026-07-28
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
| Last verified code head | `afb0519810b993f99e7b9d806a1758ee98fad1a3` |
| Last verified CI | Run `30381963693`, conclusion `success` |
| CI checks | `cargo fmt`, Clippy with `-D warnings`, complete workspace tests, benchmark build, reference validation |
| Test evidence | 129 passed, 0 failed across 21 summaries |
| CI artifacts | `rust-ci-report` `8697430252` (9749 bytes); `rust-reference-report` `8697430874` (1128 bytes) |

The documentation commit that contains this update necessarily advances the PR
head. Treat `afb0519...` as the final reviewed **code** baseline, then verify
the documentation-only head independently.

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

### Contact Response, Diagnostics, and Acceptance

- persistent contact cache, stable feature matching, and warm start;
- unilateral XPBD contacts interleaved with persistent constraints;
- non-velocity depenetration accounting;
- moving-surface friction and new-impact-only restitution;
- sphere, capsule, and rounded-box analytic proxies;
- transactional rollback, pause/resume, stable-ID failures, and source errors;
- deterministic pendulum, hair, tail, and closed-skirt references;
- real per-phase timing and deterministic replay metadata;
- Criterion groups `hair_64`, `skirt_12x8`, and `contacts_256`;
- pull-request benchmark compilation, scheduled measurements, and always-on
  diagnostic artifacts.

## 4. What Is Not Implemented

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
  review passed.
- Task 7e review found two Important diagnostic-context issues; both were fixed
  test-first and the re-review passed.
- Final Task 7/8 review found analytic overflow normalization, reference metric,
  skirt-fixture, failure-artifact, phase-timing, and fixed-animation benchmark
  issues. All were fixed, and the final re-review reported no remaining
  Critical/Important findings.

## 6. Completed Toolchain and Dependency Migration

The requested stable Rust and direct-dependency migration is complete.

Verified on 2026-07-26:

| Component | Pre-migration | Completed target |
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

Completed migration sequence:

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

PR #2 is complete after the documentation-only head receives fresh CI and may
then become Ready. Do not silently merge it.

The next implementation phase is `docs/plans/v1-blender-runtime.md`, followed
by `docs/plans/v1-mesh-bake-release.md`. Start those changes from an integrated
PR #2 baseline; do not mix Blender runtime code into PR #2.

## 8. Next-Phase Start Prompt

After PR #2 is reviewed and integrated, start a new branch and PR for
`docs/plans/v1-blender-runtime.md`. Re-read the design, all completed phase
plans, and this handoff; preserve the stable-ID and deterministic batch
contracts established by the core. Keep Blender/PyO3 work outside PR #2.

## 9. Completion Evidence Standard

A stage is not complete because code was written or a previous run was green.
For the exact final head:

- run the complete local gate where available;
- require the GitHub Actions job to succeed;
- confirm the uploaded reports are present and nonempty;
- count tests and failures from the report;
- perform the task's focused review;
- record deferred work explicitly rather than simulating it.
