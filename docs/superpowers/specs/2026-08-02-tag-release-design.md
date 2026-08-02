# Automatic Tag Release Design

## Goal

Pushing a version tag such as `v1.0.1` must build the supported platform
packages and publish a GitHub Release automatically.

## Behavior

- Keep `workflow_dispatch` for manual package verification.
- Add a `push` trigger for tags matching `v*`.
- Reuse the existing four-platform package matrix and contract gate.
- Run a tag-only publish job after the contract job succeeds.
- Download package artifacts and require exactly four files named
  `bone_physics-*.zip`, one per supported platform.
- Create a non-draft GitHub Release for the pushed tag with generated notes.
- Upload only the four platform ZIP files to the Release. SHA-256 sidecars,
  `platform-evidence.json`, and `v1-acceptance.json` remain Actions artifacts,
  not Release assets.
- Fail the publish job if the tag is not a version tag, the package count is
  wrong, or GitHub cannot create the Release.

## Permissions and safety

The existing package jobs retain read-only repository access. Only the
tag-gated publish job receives `contents: write`. A manual dispatch never
creates or modifies a Release.

## Verification

Add a Python workflow contract test that checks the tag trigger, tag-only
publish job, write permission, exact ZIP selection, and exclusion of the three
non-release asset names. Run the focused test, the full Python suite, and the
Rust verification gate before pushing the feature branch.
