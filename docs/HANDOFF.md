# Codex CLI Handoff

Status: 2026-08-02 (Asia/Shanghai)

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
| Parent head before this handoff update | `55f0c9eed21921463e1e100f8f24eabc3de1038b` (`55f0c9e`) |
| Parent update | Fast-forward from `41638e6`; no history rewrite |
| PR size before this handoff update | 21 commits, 131 changed files |

The local workspace is currently two focused commits ahead of the recorded
remote head: `745ea20` restores workspace build integrity and `e8b95ca` fixes
extension archive paths. These commits are local only until explicitly pushed.

This file is itself pushed as the next commit on the PR branch, so its commit
SHA cannot be embedded into its own contents ahead of time. On startup, always
read the live PR head and use that exact commit as the baseline. The parent
remote tree for this handoff was `55f0c9e`; the handoff commit is the current
newest commit unless another collaborator has advanced the branch.

Do not reset the remote tree to the old
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

The following was run against the local workspace after the runtime fixes:

- `python -m compileall -q blender_extension tests/python tests/blender scripts`:
  passed.
- Full Python discovery with the built abi3 wheel installed:
  112 tests passed.
- Rust formatting, Clippy with `-D warnings`, and workspace tests:
  passed; 77 Rust tests passed.
- `cargo tree --duplicates` completed; duplicate dependency groups are
  reported for `syn`, `thiserror`, and `zerocopy`.
- Native abi3 wheel build with `maturin==1.14.1`, dependency inspection, and
  isolated installation: passed.
- Blender MCP background tests for pre-physics ordering, pose round-trip, and
  runtime handler: passed.
- A real native extension archive was built and structurally verified with one
  Linux native module; archive paths are rooted at the extension package.
- `git diff --check` and Python compileall: passed.

Earlier Work-session evidence reported a complete Python suite, core fmt/
Clippy/core tests, a Linux package smoke test, and a 10-scene × 3-FPS × 2-run
matrix. Treat those as historical evidence only until fresh CI artifacts for
the live PR head are inspected. Do not report the V1 release gate as green
without platform-matched artifacts and `HEAD == GITHUB_SHA` evidence.

Resolved tree consistency items:

- License metadata now consistently uses `GPL-3.0-or-later`, with a regression
  test.
- No `rust-toolchain.toml` was added. The local stable toolchain is 1.97.1 and
  CI explicitly installs Rust 1.97.1, so the repository keeps the existing CI
  pin without adding a toolchain file.

## 6. Immediate next actions for local Codex CLI

1. Read PR #3's live `head_sha`, confirm the fresh checkout matches it, ensure
   the worktree is clean, and verify that the PR still targets `main` as Draft.
2. Inspect the actual remote diff and current CI checks. Do not trust the old
   PR description, which still says “Task 1” although the tree contains later
   work.
3. Keep the local and remote heads distinct until the local commits are
   explicitly pushed. After that, inspect fresh CI artifacts for the pushed
   head and diagnose the first real failure, if any:

   ```bash
   cargo fmt --all --check
   cargo clippy --workspace --all-targets -- -D warnings
   cargo test --workspace --all-targets
   cargo tree --duplicates
   ```

4. Audit the package workflow against the approved four-platform/fresh-host
   contract. Do not weaken thresholds or replace real Blender/native evidence
   with a pure-Python fixture.
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
