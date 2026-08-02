# Codex CLI Handoff

Status: 2026-08-01 (America/Chicago)

This file is the operational handoff from ChatGPT Work to a local Codex CLI
checkout. It records the current remote facts, the implementation state, the
verification limits, and the next safe action. The old conversation is not a
source of truth; verify claims against this repository, PR #3, CI artifacts,
and the design documents.

## 1. Repository and remote state

| Item | Current fact |
|---|---|
| Repository | `mumu-lhl/blender-xpbd-bone-physics` (private) |
| PR | [#3](https://github.com/mumu-lhl/blender-xpbd-bone-physics/pull/3) |
| PR state | Open, Draft, not merged |
| Base | `main` at `cedfb330e2cbdb98888df3a338566cace7c08840` |
| Head branch | `feat/v1-blender-runtime` |
| Remote head | `55f0c9eed21921463e1e100f8f24eabc3de1038b` (`55f0c9e`) |
| Remote update | Fast-forward from `41638e6`; no history rewrite |
| PR size at handoff | 21 commits, 131 changed files |

The current remote tree is the handoff baseline. Do not reset it to the old
recovery-workspace commits `4383442`, `534a951`, `fed8559`, `d595ab5`, or
`5e726a2`: those names are useful audit breadcrumbs, but their commit graph is
not the remote PR graph. A fresh local CLI checkout should clone the repository,
fetch `feat/v1-blender-runtime`, and work from `origin/feat/v1-blender-runtime`.

```bash
git clone https://github.com/mumu-lhl/blender-xpbd-bone-physics.git
cd blender-xpbd-bone-physics
git fetch origin feat/v1-blender-runtime
git switch --track origin/feat/v1-blender-runtime
```

Continue on this branch and this PR. Do not create another branch or PR, merge
PR #3, create a Release, or mark the PR ready without explicit user approval.
Pushing a confirmed commit to this existing Draft PR is allowed when requested.

## 2. Required reading order

Before changing physics behavior, public APIs, dependencies, or CI, read:

1. `AGENTS.md`
2. `docs/design/blender-xpbd-bone-physics-design.md`
3. `docs/plans/v1-constraints-collision.md`
4. `docs/plans/v1-blender-runtime.md`
5. `docs/plans/v1-mesh-bake-release.md`
6. this file

The approved design is authoritative for physical semantics. `AGENTS.md` is
authoritative for repository workflow and non-negotiable invariants. If this
file and a plan disagree, preserve the design invariants and verify the remote
tree before editing.

## 3. Architecture and hard boundaries

- `bone_physics_core` owns physical behavior and has no Python or Blender
  dependency. Keep `#![forbid(unsafe_code)]` in the core.
- `bone_physics_py` is the narrow PyO3 batch bridge. Keep panic translation,
  API-version checks, generation-checked handles, and validation at the bridge.
- Blender Python modules adapt scene data, evaluate pre-physics inputs, manage
  timeline/cache state, and write output poses. Pure modules must import without
  `bpy`.
- Parry is an internal geometry/BVH implementation detail. Never expose Parry
  types through the public API.
- Runtime constraints contain concrete sampled values; they do not retain or
  evaluate authoring curves.
- The shipped extension must bundle the matching native module. Users must not
  install Rust, pip packages, NumPy, or a system physics library.

The intended output path remains: an XPBD proxy/physics result drives declared
real bone pose channels after pre-physics collision input has been captured.
Do not feed the final physics-deformed mesh back into the same simulation time.

## 4. Work completed in the current PR tree

The branch contains the recovered runtime implementation plus the later V1
checkpoint, bake, acceptance, packaging, and release-gate work:

- PyO3 API v1, typed errors, opaque generation-checked world handles, owned-copy
  batch validation, animation-target upload, collider upload, stepping, output
  transforms, and diagnostics.
- Blender-independent model/parameter/collider builders, native loader, scene
  adapter, pre-physics snapshots, pose writer, timeline state machine, guarded
  runtime handler, and ownership-safe deforming-mesh input.
- Versioned deterministic checkpoints with canonical BLAKE3 fingerprints,
  fixed-clock remainder, corruption/version/fingerprint rejection, and
  validate-then-atomic restore.
- Transactional bake generations: incomplete generations are invisible,
  `os.replace` publishes only complete generations, approximate preview cannot
  write authoritative checkpoints, and compatible nearest checkpoints support
  replay.
- Bounded continuous-collision fallback and native diagnostics for broad-phase,
  CCD, contact counts, failure IDs, and phase timings.
- Ten reference scenarios in `tests/reference/acceptance.json`: single pendulum,
  single hair, multiple hair, graded tail, closed skirt, fast arm, moving legs,
  initial penetration, deforming body, and breast-torso contact.
- Acceptance policy: fixed 120 Hz physical time, authoritative 64 iterations,
  24/30/60 FPS equal-time comparison, two runs for repeatability, explicit
  nonzero-contact requirements for contact scenarios, and failure snapshots.
- Reproducible extension archive tooling, SHA-256 sidecars, package structure
  checks, native dynamic-dependency checks, license/dependency reporting, and a
  fail-closed RC aggregator.
- Ordinary PR CI is limited to pull requests, `main` pushes, and the scheduled
  benchmark path. Package/RC workflow is manual and does not create a Release
  during ordinary CI.

Historical recovery breadcrumbs (not a substitute for remote verification):

- `db1d1c5`: deterministic solver checkpoints and fingerprints.
- `5e726a2`: native checkpoint bridge.
- `d595ab5`: CCD and native diagnostics.
- `fed8559`: ten-scenario acceptance matrix.
- `534a951`: reproducible extension packaging.
- `4383442`: RC gate, documentation, and manual package workflow.

## 5. Fresh verification evidence and limitations

The following was run against the current recovery workspace before this file
was prepared:

- `python -m compileall -q blender_extension tests/python tests/blender scripts`:
  passed.
- Packaging, RC-gate, and reference-runner subset:
  10/10 passed.
- Full Python discovery: 90 tests collected; 86 passed, 4 skipped by contract,
  and 3 native-module imports failed because `bone_physics_native` was not
  built/installed in this workspace. This is not evidence of a native failure.
- `git diff --check`: passed for the recovery workspace.
- Rust and Blender full gates were not run successfully in this workspace:
  the recovered environment lacks a usable Rust 1.97.1 toolchain and a built
  native wheel; no local Blender 5.2 executable is available.

Earlier Work-session evidence reported a complete Python suite, core fmt/
Clippy/core tests, a Linux package smoke test, and a 10-scene × 3-FPS × 2-run
matrix. Treat those as historical evidence only until fresh CI artifacts for
exact head `55f0c9e` are inspected. Do not report the V1 release gate as green
without platform-matched artifacts and `HEAD == GITHUB_SHA` evidence.

Known tree inconsistency to resolve before claiming packaging is complete:

- `blender_extension/blender_manifest.toml` and workspace Cargo metadata use
  `GPL-3.0-or-later`, but
  `crates/bone_physics_py/pyproject.toml` still says `GPL-3.0-only`.
  Confirm the intended license, add a regression test, and update the metadata
  consistently if the approved policy remains `or-later`.
- The current remote tree has no `rust-toolchain.toml`; CI explicitly requests
  Rust `1.97.1`. Decide whether the repository should add an exact toolchain
  file, based on the approved build policy, before relying on local reproducible
  builds.

## 6. Immediate next actions for local Codex CLI

1. Confirm the fresh checkout is exactly `55f0c9e`, the worktree is clean, and
   PR #3 still targets `main` as a Draft.
2. Inspect the actual remote diff and current CI checks. Do not trust the old
   PR description, which still says “Task 1” although the tree contains later
   work.
3. Install or activate the pinned Rust 1.97.1 toolchain in the CLI environment,
   then run the complete gate and diagnose the first real failure:

   ```bash
   cargo fmt --all --check
   cargo clippy --workspace --all-targets -- -D warnings
   cargo test --workspace --all-targets
   cargo tree --duplicates
   ```

4. Build the abi3 native wheel with `maturin==1.14.1`, install it into an
   isolated environment, and run the native contract tests. Then run the
   Blender-independent Python suite again with native tests included.
5. Resolve the license metadata mismatch and audit the package workflow against
   the approved four-platform/fresh-host contract. Do not weaken thresholds or
   replace real Blender/native evidence with a pure-Python fixture.
6. Run the Linux package/native smoke checks locally. Leave Windows, macOS
   arm64, and the macOS Intel limitation as CI facts; there is no official
   Blender 5.2 Intel build, so Intel can only receive native/CPython loading
   evidence.
7. Inspect all CI artifacts, count tests/failures, and perform an independent
   review for Critical/Important findings. Only then make a separate focused
   commit and push it to the existing Draft PR when requested.

## 7. Physics invariants that must not regress

- Stable IDs alone determine body, constraint, contact, and pair ordering.
- Persistent constraints and contacts share every XPBD iteration; no final
  collision-only, limit-only, or post-blend correction pass.
- Every resulting quaternion is finite and normalized.
- Exact coincidence, parallel capsules, and zero-length segments are finite and
  deterministic.
- Normal contact multipliers are unilateral. Pure depenetration is recorded
  separately from velocity reconstruction.
- Friction uses velocity relative to the moving collision surface; restitution
  applies only to a new closing impact.
- Direct structural-neighbor filtering is not transitive chain closure.
- Hard parent joints may have zero compliance; reject actual all-rigid cycles,
  not every cycle containing a hard edge.
- Body collision geometry comes from the pre-physics animation state.
- Never loosen an acceptance threshold merely to make a test pass.

## 8. Commit and release policy

Use one concern per commit and follow RED → GREEN → refactor for behavior
changes. Before committing, inspect status and the exact staged diff; never stage
unrelated files. Before claiming a stage complete, run fresh verification and
record the actual result.

The current user authorization is to prepare and push this handoff file to PR
#3. It does not authorize merge, Release creation, PR conversion from Draft,
new branches, or new pull requests. Keep PR #3 Draft until the runtime,
cross-platform evidence, and independent review requirements are genuinely
complete.
