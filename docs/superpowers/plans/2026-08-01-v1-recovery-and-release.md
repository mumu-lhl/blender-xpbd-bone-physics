# V1 Recovery and Release Completion Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Recover the verified `41638e6` runtime baseline after workspace pruning, reimplement the lost Task 6–10 changes with tests, and publish the resulting commits to the existing `feat/v1-blender-runtime` branch behind Draft PR #3.

**Architecture:** Keep the existing Rust solver and PyO3 batch boundary. Add checkpoint/hash behavior in Rust, transactional bake/replay and diagnostics in Blender-independent Python, deterministic reference/package/release tooling around those APIs, and use GitHub only for the final fast-forward update of the existing branch.

**Tech Stack:** Rust 1.97, `postcard` plus BLAKE3 for bounded versioned checkpoints, PyO3 abi3, Python 3.11+ standard library, Blender 5.2 scripts, GitHub Actions.

## Global Constraints

- Preserve the existing PR #3 head lineage; do not create a second branch or PR.
- Approximate preview cannot create or overwrite authoritative checkpoints.
- Checkpoint restore is validate-then-atomic; corruption or fingerprint mismatch leaves the live world unchanged.
- Fingerprints include relevant solver/topology/ownership/animation inputs and physical contact materials, but not render lighting/material inputs.
- Authoritative bake uses fixed substeps and the existing CCD/quality policy; no threshold is relaxed to make a fixture pass.
- Platform archives contain one matching abi3 native module, no pip/runtime installation requirement, and a complete license/dependency report.
- Normal CI must not duplicate feature-branch push and pull-request runs; draft RC creation is manual and fail-closed.

---

### Task 0: Recover and baseline the remote runtime

**Files:**
- Recover: all files currently changed by Draft PR #3 at `feat/v1-blender-runtime`.
- Create: this plan.

- [ ] **Step 1: Confirm remote facts**

Verify repository `mumu-lhl/blender-xpbd-bone-physics`, PR #3, base `main`, head `feat/v1-blender-runtime`, and remote head `41638e68f95419854726714e94dd9574b5688bed`.

- [ ] **Step 2: Verify the recovered tree**

Run `python -m compileall -q blender_extension tests/python`, `cargo fmt --all --check`, and `cargo test -p bone_physics_core --all-targets`. Record failures caused only by missing local Python linkage separately from code failures.

- [ ] **Step 3: Commit the recovery checkpoint**

Create a local recovery commit with message `chore: recover remote runtime baseline`; do not push this synthetic commit directly because it does not share the remote parent object.

### Task 6: Versioned checkpoints and canonical fingerprints

**Files:**
- Create: `crates/bone_physics_core/src/hash.rs`, `crates/bone_physics_core/src/cache.rs`.
- Modify: `crates/bone_physics_core/src/lib.rs`, `crates/bone_physics_core/src/world.rs`, `crates/bone_physics_core/Cargo.toml`.
- Test: `crates/bone_physics_core/tests/checkpoint.rs`, `crates/bone_physics_core/tests/cache_hash.rs`.

- [ ] **Step 1: Add failing tests**

Assert round-trip replay, truncated/corrupt bytes, cache/solver version mismatch, changed topology/ownership/constraint/contact-material rejection, `q` versus `-q` hash equivalence, map-order independence, and fixed-clock remainder restoration.

- [ ] **Step 2: Verify RED**

Run `cargo test -p bone_physics_core --test checkpoint --test cache_hash`; expect missing fingerprint/checkpoint APIs.

- [ ] **Step 3: Implement the bounded format**

Encode an 80-byte little-endian header containing magic, cache/solver versions, a 32-byte fingerprint, payload length, and BLAKE3 payload digest, followed by a postcard payload. Decode into a cloned world, validate every length/version/hash, and replace state only after complete success.

- [ ] **Step 4: Verify GREEN and commit**

Run the two tests, core all-target tests, fmt, and strict Clippy. Commit `feat: serialize deterministic solver checkpoints`.

### Task 7: Native bridge, bake cache, replay, and diagnostics

**Files:**
- Modify: `crates/bone_physics_py/src/world.rs`, `crates/bone_physics_py/src/lib.rs`.
- Create: `blender_extension/bone_physics/bake_cache.py`, `blender_extension/bone_physics/diagnostics.py`.
- Modify: `blender_extension/bone_physics/runtime.py`.
- Test: `tests/python/test_bake_cache.py`, `tests/python/test_native_checkpoint.py`, `tests/blender/test_bake_seek.py`, `tests/blender/test_bake_subframes.py`.

- [ ] **Step 1: Add failing bridge/cache tests**

Require a 32-byte native fingerprint, checkpoint encode/restore, rejection of approximate-preview writes, atomic corruption rejection, no incomplete generation visibility, strict generation-name validation, and nonempty-bake enforcement.

- [ ] **Step 2: Verify RED**

Run `python -m unittest tests.python.test_bake_cache tests.python.test_native_checkpoint -v`; expect the new bridge/cache contract to be absent.

- [ ] **Step 3: Implement the transaction boundary**

Write frames/checkpoints into a temporary generation, publish `current.json` only after commit, retain the last compatible checkpoint on failure, and make runtime diagnostics carry native phase timings, candidate/contact counts, CCD fallback counts, and stable failure IDs.

- [ ] **Step 4: Verify and commit**

Run all Python tests, native wheel contract tests, compileall, Rust fmt/Clippy/core tests. Commit `feat: add transactional bake cache and diagnostics`.

### Task 8: Machine-checked reference acceptance matrix

**Files:**
- Create: `tests/blender/scenes/common.py`, `hair.py`, `tail.py`, `skirt.py`, `body_collision.py`, `tests/blender/run_reference.py`, `tests/reference/acceptance.json`, `benchmarks/large_scene.json`.
- Test: `tests/python/test_reference_runner.py`.

- [ ] **Step 1: Add failing contract tests**

Require all ten named scenarios, 24/30/60 FPS equal-time comparison, two authoritative runs, rigid-length/anchor/penetration/angular-lock/diagnostic fields, and failure snapshots containing seed, build batch, inputs, checkpoint, stable ID, actual, and allowed values.

- [ ] **Step 2: Verify RED**

Run `python -m unittest tests.python.test_reference_runner -v`; expect missing scenario registry/runner APIs.

- [ ] **Step 3: Implement deterministic fixtures and runner**

Use one fixed 120 Hz physical step, authoritative iteration budget 64, explicit contact-required scenarios, and separate measurement of final-state error versus historical peak error. Do not loosen acceptance thresholds.

- [ ] **Step 4: Run the full local matrix and commit**

Run the 10-scene × 3-FPS × 2-bake matrix and require all contact-required scenarios to produce nonzero contacts. Commit `test: add first-version acceptance scenes`.

### Task 9: Reproducible platform packaging

**Files:**
- Create: `scripts/package_extension.py`, `scripts/verify_package.py`, `scripts/check_native_deps.py`, `.github/workflows/package.yml`, `docs/building.md`, `docs/testing.md`.
- Modify: `.github/workflows/ci.yml`, `Cargo.toml`, `blender_extension/blender_manifest.toml`, Python/Cargo license metadata.
- Test: `tests/python/test_packaging.py`, `tests/python/test_native_deps.py`.

- [ ] **Step 1: Add failing packaging tests**

Check deterministic archive bytes, normalized order/timestamp/permissions, one platform-matching native module, API/manifest agreement, SHA-256 sidecar, license inventory, and dynamic-library dependency whitelist.

- [ ] **Step 2: Verify RED and implement**

Build the abi3 wheel, extract only runtime files, normalize the archive, generate checksums/licenses/dependency reports, and make fresh-profile failure paths upload evidence before failing.

- [ ] **Step 3: Verify Linux locally and commit**

Run the complete packaging/native smoke suite twice and compare bytes; leave Windows/macOS Blender installation as explicit CI facts, including the absence of an official Blender 5.2 Intel build. Commit `build: package self-contained Blender extension`.

### Task 10: Fail-closed V1 release-candidate gate

**Files:**
- Modify: `README.md`, `.github/workflows/package.yml`.
- Create: `CHANGELOG.md`, `docs/limitations.md`, `scripts/ci/aggregate_acceptance.py`, `tests/python/test_release_gate.py`.

- [ ] **Step 1: Add failing gate tests**

Reject missing reports, mismatched artifact commit, missing criteria, failed numerical values, and `create_draft=true` without complete four-platform evidence.

- [ ] **Step 2: Implement the aggregate artifact**

Emit one record per approved criterion with source, observed value, allowed value, commit, platform, and pass/fail. Permit draft creation only when the manual boolean, acceptance success, and `HEAD == GITHUB_SHA` all agree.

- [ ] **Step 3: Verify documentation and commit**

Run Python/Rust/docs/native checks and parse a real reference report. Commit `docs: prepare first-version release candidate`; do not create or publish a Release.

### Final publish checkpoint

- [ ] Inspect each commit and generated diff for unrelated files.
- [ ] Rebase or construct GitHub API commits with parent `41638e6` only after exact content verification.
- [ ] Fast-forward the existing `feat/v1-blender-runtime` branch and confirm PR #3 updates; do not merge it or create another PR.
- [ ] Trigger only the intended final CI/manual gate and report its artifacts.
