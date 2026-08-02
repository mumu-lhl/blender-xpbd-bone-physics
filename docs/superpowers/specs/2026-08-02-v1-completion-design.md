# V1 Completion Design

Status: approved by the user on 2026-08-02; implementation on `feat/v1-complete`

## Authority and scope

`docs/design/blender-xpbd-bone-physics-design.md` is the authority for solver
semantics and first-version acceptance. This document only defines the delta
needed to finish and verify that approved design after the merged runtime work.

V1 completion means that the existing solver, native bridge, Blender runtime,
deforming-mesh path, bake/cache path, reference matrix, package tooling, and
release gate are all connected and have reproducible evidence. It does not add
wind, GPU solving, plasticity, fluids, cross-armature dynamics, or production
UI. It does not publish a GitHub Release.

## Baseline and branch

Work starts from the tree that produced merged PR #3 and continues on
`feat/v1-complete`. The first task is an evidence audit, not a rewrite: files
already implementing approved behavior remain unchanged unless a failing
contract identifies a real gap.

## Completion architecture

The completion work has four boundaries:

1. The Rust/PyO3 and Blender runtime remain the physical implementation. New
   code must preserve stable-ID ordering, quaternion state, pre-physics mesh
   ownership, fixed physical time, and transactional rollback.
2. The reference runner is the numerical acceptance boundary. It produces one
   JSON report containing all ten scenarios, 24/30/60 FPS equal-time records,
   repeated authoritative runs, diagnostics, and replayable failure snapshots.
3. The package workflow is the distribution boundary. Each supported platform
   builds exactly one matching abi3 native module, packages it into a normalized
   extension archive, verifies dynamic dependencies and license metadata, then
   loads it in a clean Blender 5.2 profile with network disabled.
4. The release aggregator is the final evidence boundary. It accepts only
   reports for the exact commit, requires every approved criterion and all four
   platform reports, and permits draft-RC preparation only when the explicit
   flag and `GITHUB_SHA` match are both true. No release mutation is performed.

## Required changes

- Correct and test native dependency discovery on Linux, Windows, and macOS.
- Add a platform package/load workflow with Linux x86_64, Windows x86_64,
  macOS arm64, and macOS x86_64 native-contract jobs. Blender installation is
  run where a supported Blender 5.2 binary exists; macOS Intel receives native
  loading evidence because no official Blender 5.2 Intel build exists.
- Add a clean-profile installed-extension smoke test that imports the packaged
  archive, checks the native API version, creates and advances a two-body world,
  and writes a JSON report.
- Connect real CI/reference/package reports to `v1-acceptance.json`; missing,
  stale, incomplete, or failed evidence must fail closed.
- Update README, build/testing/limitations/changelog, handoff, and plan status
  to distinguish implemented V1 from non-goal features and from the unpublished
  release step.

## Test strategy

Every behavior change follows RED → GREEN. Pure Python contracts run with the
uv-managed environment. Rust uses the pinned 1.97.1-compatible workspace gate.
Blender debugging and headless runtime validation use Blender MCP; no direct
long-lived Blender process is started from the shell. CI is authoritative for
platform-specific artifacts. The final local gate includes Python, Rust,
package structure, native dependency inspection, and the available Blender MCP
tests; the final CI gate must provide nonempty reports for every required job.

## Out of scope for this branch

This branch will not create a GitHub Release, sign artifacts with unavailable
credentials, add unsupported Blender binaries, or weaken acceptance thresholds
to compensate for missing platform evidence. Those actions require a separate
explicit authorization or external platform state.
