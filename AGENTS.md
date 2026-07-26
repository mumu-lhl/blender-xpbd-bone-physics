# Repository Agent Instructions

These instructions apply to the whole repository.

## Read First

Before changing physics behavior, public APIs, dependencies, or CI, read:

1. `docs/HANDOFF.md` for the current branch state and next action.
2. `docs/design/blender-xpbd-bone-physics-design.md` for approved semantics.
3. `docs/plans/v1-constraints-collision.md` for task boundaries and progress.

The complete approved roadmap is:

1. `docs/plans/v1-core-foundation.md` — completed and merged through PR #1.
2. `docs/plans/v1-constraints-collision.md` — current Draft PR #2.
3. `docs/plans/v1-blender-runtime.md` — not started.
4. `docs/plans/v1-mesh-bake-release.md` — not started.

If the documents appear inconsistent, preserve the design invariants, treat
`HANDOFF.md` as the newest status record, and stop to resolve any semantic
conflict before changing code.

## Architecture Boundaries

- `bone_physics_core` owns physical behavior and remains independent of Python
  and Blender.
- Keep `#![forbid(unsafe_code)]` in the core. Any future unsafe buffer exchange
  belongs in a small documented PyO3 bridge module.
- Parry is an internal geometry/BVH backend. Do not expose Parry types through
  the public API.
- Runtime constraints store concrete sampled values. They do not retain or
  evaluate authoring parameter curves.
- Users must not need Rust, pip packages, NumPy, or system physics libraries to
  install the final Blender extension.

## Non-negotiable Physics Invariants

- Stable IDs alone determine body, constraint, contact, and pair ordering.
- Persistent constraints and contacts share every XPBD iteration.
- Do not add a final collision-only, limit-only, or post-blend correction pass.
- Rotational state remains quaternion based. Every resulting quaternion must be
  finite and normalized.
- Exact coincidence, parallel capsules, and zero-length collider segments must
  produce finite deterministic results.
- A normal contact multiplier is unilateral and must not become attractive.
- Record pure depenetration separately so velocity reconstruction does not turn
  it into artificial bounce.
- Compute friction from velocity relative to the moving collision surface.
- Apply restitution only to a new closing impact, never to a persistent resting
  contact.
- Direct structural neighbors may be filtered without taking a transitive
  closure over the chain.
- Hard parent joints may have zero compliance. Reject actual all-rigid cycles;
  do not reject a cycle merely because it contains one hard edge.
- Evaluate body collision geometry from the pre-physics animation state. Never
  feed the final physics-deformed mesh back into the same simulation time.
- Do not loosen an acceptance threshold merely to make a test pass.

## Development Workflow

- Make the toolchain/dependency migration described in `docs/HANDOFF.md` as a
  standalone change before adding more physics behavior.
- For every behavior change, first add a test whose failure demonstrates the
  missing behavior, record the RED result, implement the smallest GREEN change,
  then refactor without changing the contract.
- Keep one concern per commit. Do not combine dependency migration with Task 7
  contact-response work.
- Diagnose the first compiler, Clippy, formatting, or behavior failure before
  editing code. Do not guess from a failed overall CI status.
- Preserve existing tests during migrations. Any necessary expectation change
  needs a documented semantic reason.
- Keep PR #2 as Draft until Tasks 7 and 8 are complete, fresh CI is green, and
  an independent review reports no Critical or Important findings.

## Required Verification

Run the complete gate before claiming a stage is complete:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

For dependency migrations also run:

```bash
cargo tree --duplicates
```

Task-specific tests establish RED/GREEN evidence, but they do not replace the
complete gate. GitHub Actions is authoritative when the local environment
lacks the pinned toolchain or dependencies. Confirm that uploaded CI reports
are present and nonempty.
