# V1 Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish and verify the approved V1 design with real native acceptance evidence, four-platform packaging checks, a fail-closed artifact gate, and complete build/test documentation.

**Architecture:** Preserve the existing Rust XPBD core, PyO3 batch boundary, and Blender runtime. Replace release-only placeholders with a native-backed acceptance runner, a portable package/load harness, and a CI aggregator that consumes exact-commit reports. Keep unsupported features outside V1.

**Tech Stack:** Rust 1.97.1, PyO3 abi3, maturin 1.14.1, Python standard library under uv, Blender 5.2 LTS, GitHub Actions, ZIP/SHA-256, BLAKE3 checkpoints.

## Global Constraints

- `docs/design/blender-xpbd-bone-physics-design.md` is authoritative for physics semantics.
- `bone_physics_core` remains `#![forbid(unsafe_code)]` and independent of Python and Blender.
- Stable IDs determine deterministic ordering; no threshold is loosened to hide a regression.
- Pre-physics collision ownership and transactional rollback remain mandatory.
- Approximate preview cannot create or overwrite authoritative checkpoints.
- Each supported package contains exactly one matching native module and no runtime pip dependency.
- Local Blender debugging uses Blender MCP; CI may use bounded background invocations with explicit termination.
- No GitHub Release is created by this branch.

---

### Task 1: Make native dependency and package evidence portable

**Files:**
- Modify: `scripts/check_native_deps.py`
- Modify: `scripts/verify_package.py`
- Create: `tests/python/test_native_deps.py`
- Modify: `tests/python/test_packaging.py`
- Modify: `docs/building.md`

**Interfaces:**
- Produces `dependencies(path, system) -> {platform, status, dependencies, unexpected}` with correct Linux/macOS/Windows parsing.
- Produces `verify(archive, expected_platform) -> {sha256, files, native_count, platform}` and rejects zero or mismatched native modules.
- Produces a deterministic dependency/license report suitable for CI artifacts.

- [ ] **Step 1: Write failing dependency parser tests**

Add fixture outputs for `readelf`, `otool`, and `dumpbin`. Assert system
libraries are accepted, project/private libraries are reported as unexpected,
tool absence is reported as `unavailable`, and the report never silently turns
an uninspected binary into a passing result.

- [ ] **Step 2: Run the focused tests and record RED**

Run:

```bash
UV_CACHE_DIR=/tmp/bone-physics-uv-cache uv run --python /tmp/bone-physics-uv/bin/python \
  python -m unittest tests.python.test_native_deps tests.python.test_packaging -v
```

Expected: failure from the current escaped-dot parser and missing dependency
contract coverage.

- [ ] **Step 3: Implement the smallest portable parser fix**

Use platform-specific regular expressions and an injectable command runner so
tests do not require `dumpbin` or `otool`. Return `status="unavailable"` only
for an explicitly unsupported inspection tool; package CI treats that status as
failure. Add a license inventory entry to the package report and make archive
verification reject a package with no native module when a platform is given.

- [ ] **Step 4: Run the focused tests and the Python suite**

Run the focused command again, then run the full Python discovery. Expected:
all focused tests and 116-or-more Python tests pass.

- [ ] **Step 5: Commit the concern**

```bash
git add scripts/check_native_deps.py scripts/verify_package.py tests/python/test_native_deps.py tests/python/test_packaging.py docs/building.md
git commit -m "fix: make package evidence fail closed"
```

### Task 2: Build and load packages on every supported platform

**Files:**
- Create: `scripts/ci/run_package_load.py`
- Create: `tests/blender/test_installed_extension.py` changes for a two-body native smoke step
- Modify: `.github/workflows/package.yml`
- Modify: `.github/workflows/ci.yml` only if shared artifact/report helpers are required
- Modify: `docs/testing.md`

**Interfaces:**
- `run_package_load.py --platform TAG --report-dir DIR` builds the current runner's abi3 wheel, packages it, verifies it, inspects dependencies, and emits nonempty JSON reports.
- The installed-extension smoke imports the package, checks `native_api_version() == 1`, builds a valid two-body batch, advances one fixed step, reads normalized transforms, and records the result.
- The package workflow exposes matrix tags `linux-x86_64`, `windows-x86_64`, `macos-arm64`, and `macos-x86_64`.

- [ ] **Step 1: Write the failing clean-profile smoke contract**

Extend the installed-extension test so a package without a native module, a
wrong API version, or a world that cannot advance fails with a structured JSON
error. Add a Python contract test for the harness report requiring archive,
checksum, dependency, and load-test records.

- [ ] **Step 2: Run the focused test and record RED**

Run the installed-extension and CI-script tests with the uv environment. The
new report assertions must fail because the harness and native world step do
not yet exist.

- [ ] **Step 3: Implement the portable harness**

Use `sys.executable`/`maturin` from the runner, locate `.so`/`.pyd` inside the
wheel, call `package()` and `verify()`, run the native dependency inspector,
and write `package.json`, `package.sha256`, `dependencies.json`,
`load-test.json`, and `failure-snapshot.json` on failure. Do not use network
access during extension loading.

- [ ] **Step 4: Implement the matrix workflow**

Use GitHub Actions runners for Linux x86_64, Windows x86_64, macOS arm64, and
macOS x86_64. Install the pinned Rust toolchain and maturin, run the harness,
upload every report with `if: always()`, and make missing artifacts fail. Run
Blender 5.2 headless where an official binary is available; on macOS Intel
retain native/CPython loading evidence and document the official Blender binary
limitation instead of pretending to have Blender evidence.

- [ ] **Step 5: Run Linux package/load evidence locally**

Build the package twice, compare archive bytes and SHA-256, run the native
smoke test through Blender MCP using the installed bundle, and inspect the
generated reports. Do not launch an unbounded Blender process from the shell.

- [ ] **Step 6: Commit the concern**

```bash
git add scripts/ci/run_package_load.py tests/blender/test_installed_extension.py .github/workflows/package.yml .github/workflows/ci.yml docs/testing.md tests/python
git commit -m "ci: verify self-contained platform packages"
```

### Task 3: Replace the reference placeholder with native-backed acceptance

**Files:**
- Modify: `tests/blender/run_reference.py`
- Modify: `tests/blender/scenes/common.py`
- Modify: `tests/blender/scenes/hair.py`
- Modify: `tests/blender/scenes/tail.py`
- Modify: `tests/blender/scenes/skirt.py`
- Modify: `tests/blender/scenes/body_collision.py`
- Create: `tests/reference/native_backend.py`
- Modify: `tests/python/test_reference_runner.py`
- Create: `tests/blender/test_reference_backend.py`
- Modify: `tests/reference/acceptance.json`

**Interfaces:**
- `NativeReferenceBackend.run(fixture, fps, bake, seed) -> ReferenceRecord` builds a deterministic `WorldSpec`, uses the real native API, and returns measured transforms, contacts, phase timings, and diagnostics.
- `run_matrix(..., backend="native")` performs 10 scenarios × 3 FPS × 2 authoritative runs and compares equal physical times and repeated runs.
- Failure records include seed, compiled batch, animation input, mesh input, checkpoint identifier, stable diagnostic IDs, actual values, and allowed values.

- [ ] **Step 1: Write the failing native-backend contract**

Require the backend to import the native module, build and destroy a world,
advance fixed steps, compute segment-length and anchor metrics from returned
poses, and report nonzero contacts for the five contact-required scenarios.

- [ ] **Step 2: Run the contract and record RED**

Run the native backend contract with the installed wheel. Expected: failure
because the current runner emits hard-coded fixture metrics and does not call
the native solver.

- [ ] **Step 3: Implement deterministic scenario builders**

Build the ten named worlds from stable IDs. Use explicit analytic proxies and
mesh batches for contact scenarios, finite compliance for closed lattices, and
fixed animation target samples. Keep scenario geometry deterministic and keep
all metric computation outside the solver loop.

- [ ] **Step 4: Implement equal-time/repeatability checks**

Compare transforms at equal physical times across 24/30/60 FPS and compare two
authoritative runs on the same platform. Use the thresholds from the approved
acceptance JSON exactly; emit replayable failure snapshots without changing
thresholds.

- [ ] **Step 5: Verify with Blender MCP**

Run the native reference backend from a bounded Blender MCP CLI invocation with
the installed extension and inspect the JSON result. Verify no NaN, no solver
abort, and contact-required scenarios report contacts.

- [ ] **Step 6: Commit the concern**

```bash
git add tests/blender tests/reference tests/python/test_reference_runner.py
git commit -m "test: run native first-version acceptance matrix"
```

### Task 4: Produce and enforce the final V1 acceptance artifact

**Files:**
- Modify: `scripts/ci/aggregate_acceptance.py`
- Modify: `tests/python/test_release_gate.py`
- Create: `scripts/ci/build_v1_acceptance.py`
- Modify: `.github/workflows/package.yml`
- Modify: `README.md`
- Create: `CHANGELOG.md`
- Create: `docs/limitations.md`
- Modify: `docs/building.md`
- Modify: `docs/testing.md`

**Interfaces:**
- `build_v1_acceptance.py` consumes exact-commit platform reports and the native reference report, emits `v1-acceptance.json`, and fails on missing criteria or nonempty failures.
- `aggregate_acceptance()` records observed/allowed values, source artifact, commit, platform, and pass state for every approved criterion.
- Draft-RC eligibility remains a pure report field; no GitHub Release is created.

- [ ] **Step 1: Write failing artifact completeness tests**

Reject a missing reference report, a report whose commit differs from
`GITHUB_SHA`, absent criteria, missing package/dependency/load artifacts, and a
platform matrix that omits any supported tag. Require all approved design
criteria to appear once per platform where applicable.

- [ ] **Step 2: Implement the artifact builder and workflow wiring**

Collect the native reference report and package matrix reports, normalize them
into one schema, preserve all observed/allowed values, and write failure
snapshots before returning nonzero. Run the builder only after matrix jobs
complete and upload the final artifact with `if: always()`.

- [ ] **Step 3: Document factual V1 limitations and instructions**

State the non-goal features, invalid-mesh inside/outside limitation, platform
floating-point tolerance, macOS Intel Blender limitation, exact build commands,
test commands, package installation, and the fact that release creation is
manual and not performed by ordinary CI.

- [ ] **Step 4: Run all focused gate tests and commit**

```bash
UV_CACHE_DIR=/tmp/bone-physics-uv-cache uv run --python /tmp/bone-physics-uv/bin/python \
  python -m unittest tests.python.test_release_gate tests.python.test_ci_scripts -v
git add scripts/ci/aggregate_acceptance.py scripts/ci/build_v1_acceptance.py tests/python/test_release_gate.py .github/workflows/package.yml README.md CHANGELOG.md docs
git commit -m "docs: finalize V1 acceptance gate"
```

### Task 5: Final verification and handoff

**Files:**
- Modify: `docs/HANDOFF.md`
- Modify: `docs/plans/v1-constraints-collision.md`
- Modify: `docs/plans/v1-blender-runtime.md`
- Modify: `docs/plans/v1-mesh-bake-release.md`

- [ ] **Step 1: Run the complete local gate**

Run fmt, strict Clippy, all Rust tests, duplicate dependency audit, full Python
discovery, package reproducibility, native dependency inspection, and the
Blender MCP runtime/reference smoke suite.

- [ ] **Step 2: Inspect artifacts and branch state**

Require nonempty Rust, Python, native, Blender, reference, package, checksum,
dependency, and acceptance reports. Confirm the worktree is clean, the branch
contains only V1 completion commits, and no Release mutation occurred.

- [ ] **Step 3: Update status documents**

Mark only evidence-backed tasks complete, record the exact commit and CI run,
and distinguish V1 completion from unsupported features and unpublished release
creation.

- [ ] **Step 4: Commit the handoff and stop at release authorization**

```bash
git add docs/HANDOFF.md docs/plans
git commit -m "docs: record V1 completion evidence"
```

The branch is complete only after the exact-head CI gate is green. Do not
create a GitHub Release without a separate explicit user request.
