# Project Status / Codex Handoff

Status: 2026-08-09 (Asia/Shanghai)

## Current state

- V1.0.0 is complete and released from the `v1.0.0` tag. The four platform
  packages and acceptance evidence are produced by the package workflow.
- Post-release development is local maintenance through Codex. The latest
  maintenance fix is `1b610288` (`fix(ci): handle Blender RNA and
  registration`).
- UI Tasks 1-15, installed-extension character workflows, package-content
  assertions, and user/testing documentation are complete. There is no
  remaining V1 feature work.
- `docs/design/blender-xpbd-bone-physics-design.md` remains the semantic
  authority. The completion delta is recorded in
  `docs/superpowers/specs/2026-08-02-v1-completion-design.md`.
- Cross-platform package evidence must come from the package workflow; local
  evidence is sufficient for day-to-day Linux development and regression
  checks.

## Completed implementation

- Rust XPBD core, constraints, contacts, deforming meshes, CCD, checkpoints,
  diagnostics, and safe PyO3 batch bridge.
- Blender runtime: pre-physics evaluation, ownership filtering, pose output,
  timeline state machine, bake cache, replay, and guarded handlers.
- Native-backed acceptance: 10 scenarios × 24/30/60 FPS × 2 authoritative runs,
  with equal-time and repeatability checks.
- Reproducible extension packaging, SHA-256 and license inventory reports,
  fail-closed native dependency inspection, and the four-platform package/load
  workflow.
- Exact-commit V1 acceptance artifact builder with failure snapshots.

## Fresh local evidence

- `cargo fmt --all --check`: passed.
- `cargo clippy --workspace --all-targets -- -D warnings`: passed.
- `cargo test --workspace --all-targets`: passed.
- `cargo tree --duplicates`: completed; duplicate groups are reported, not
  hidden.
- Python discovery: 216 tests passed with the host native module available.
- Linux package/load harness: passed twice; byte-identical archives and SHA-256
  sidecars matched.
- Native reference matrix: 60 runs, zero failures.
- Blender MCP: installed-extension two-body smoke plus native reference matrix
  passed; Blender 5.2.0 was used through the bounded MCP CLI runner.

## Release and tag evidence

- The package workflow inspects Linux, Windows x86_64, macOS arm64, and macOS
  x86_64 reports from one exact commit and uploads acceptance evidence as
  Actions artifacts.
- Pushing a `v*` tag runs the package gate and creates a GitHub Release after
  the contract passes.
- Automatic Release assets are the four platform ZIPs only. SHA-256 sidecars,
  `platform-evidence.json`, and `v1-acceptance.json` remain Actions artifacts.
- Manual dispatch runs verification only; it never creates a Release.

## Non-negotiable boundaries

- Keep `bone_physics_core` independent of Python/Blender and retain
  `#![forbid(unsafe_code)]`.
- Stable IDs determine all ordering; do not loosen acceptance thresholds.
- Collision input is captured from the pre-physics animation state.
- Persistent constraints and contacts share every XPBD iteration; no final
  correction pass.
- Approximate preview cannot write authoritative checkpoints.
- Users install no Rust, pip package, NumPy, or system physics library.

## Required verification before completion claims

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo tree --duplicates
python -m unittest discover tests/python -v
```

For Blender Python debugging and UI/runtime checks, use Blender MCP. Do not
launch an unbounded Blender process from the shell. For native builds and
source-tree installation, use the `justfile` targets documented in
`docs/building.md`.
