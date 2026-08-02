# Automatic Tag Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish four platform ZIP packages automatically when a `v*` tag is pushed.

**Architecture:** Extend `.github/workflows/package.yml` with a tag trigger and a tag-only release job after the existing exact-commit contract. The release job downloads package artifacts, validates the exact four ZIP assets, and invokes `gh release create`; evidence files remain Actions artifacts.

**Tech Stack:** GitHub Actions, `actions/download-artifact@v4`, GitHub CLI, Python `unittest`.

## Global Constraints

- Keep `bone_physics_core` independent of Python/Blender.
- Users install no Rust, pip package, NumPy, or system physics library.
- Release job receives `contents: write`; all package jobs remain read-only.
- Only `bone_physics-*.zip` files become Release assets.
- Do not upload `.sha256`, `platform-evidence.json`, or `v1-acceptance.json` to Releases.

### Task 1: Workflow contract test

**Files:**
- Modify: `tests/python/test_ci_scripts.py`

- [x] **Step 1: Write the failing test**

Add `test_tag_workflow_publishes_only_platform_zips` to read
`.github/workflows/package.yml` and assert that it contains a `push` tag
trigger, a `publish-release` job gated by `startsWith(github.ref, 'refs/tags/v')`,
`contents: write`, `gh release create`, and `bone_physics-*.zip`; assert that
the publish job does not mention `.sha256`, `platform-evidence.json`, or
`v1-acceptance.json`.

- [x] **Step 2: Run the focused test and verify RED**

Run:

```bash
python -m unittest tests.python.test_ci_scripts.CommandReportTests.test_tag_workflow_publishes_only_platform_zips -v
```

Expected: FAIL because the current workflow has no tag trigger or publish job.

### Task 2: Add tag-triggered release job

**Files:**
- Modify: `.github/workflows/package.yml`

- [x] **Step 1: Add `push.tags: ["v*"]` while preserving `workflow_dispatch`.**

- [x] **Step 2: Add `publish-release` after `contract` with `needs: contract`,
  tag-only condition, job-level `contents: write`, artifact download, exact
  four-ZIP validation, and `gh release create` with `--generate-notes`.

- [x] **Step 3: Run the focused test and verify GREEN.**

Run the focused command from Task 1; expected: PASS.

### Task 3: Update repository instructions and run gates

**Files:**
- Modify: `AGENTS.md`

- [x] **Step 1: Record that `v*` tag pushes now build and publish Releases and
  that Release assets exclude sidecars and evidence JSON.**

- [x] **Step 2: Run verification.**

```bash
python -m unittest discover tests/python -v
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

- [ ] **Step 3: Commit implementation.**

```bash
git add .github/workflows/package.yml tests/python/test_ci_scripts.py AGENTS.md
git commit -m "ci: publish releases from version tags"
```
