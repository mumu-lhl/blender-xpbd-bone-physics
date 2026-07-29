# V1 Deforming Mesh, Bake, Cache, and Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete V1 with ownership-safe deforming body collision, authoritative continuous-collision bake, deterministic checkpoints, diagnostics, reference scenes, and self-contained platform packages.

**Architecture:** Blender builds a stable pre-physics collision surface and uploads topology once, then vertex positions per evaluated sample. Rust owns topology, refittable acceleration data, capsule-triangle manifolds, history, and continuous queries. Preview and bake share the same world semantics but use separate quality policies and cache namespaces.

**Tech Stack:** Rust stable, parry3d behind the collision adapter, PyO3/maturin, Blender 5.2 LTS Python API, serde/postcard for versioned checkpoints, GitHub Actions.

## Handoff Status — Not Started

Do not begin this plan until the core, constraints/collision, and Blender
runtime plans are complete.

## Global Constraints

- Depends on `docs/plans/v1-core-foundation.md`,
  `docs/plans/v1-constraints-collision.md`, and
  `docs/plans/v1-blender-runtime.md`.
- A physical bone never collides with a mesh region it directly drives.
- Collision input is evaluated before applying the current physical output.
- Stable topology updates refit; topology changes rebuild and invalidate.
- Mesh contact defines the visible boundary; analytic proxies sit inside as fallback.
- Open or invalid meshes use two-sided contact plus history, never an invented global inside.
- Approximate preview cannot create or overwrite authoritative checkpoints.
- High-quality bake progresses strictly forward at fixed steps and may request real Blender subframes.
- Users receive bundled native modules and install no external dependency.

## File Map

- `crates/bone_physics_core/src/collision/mesh.rs`: topology, vertices, validation, refit.
- `crates/bone_physics_core/src/collision/triangle.rs`: capsule-triangle contact geometry.
- `crates/bone_physics_core/src/collision/manifold.rs`: stable contact clustering.
- `crates/bone_physics_core/src/collision/ccd.rs`: moving capsule/moving triangle queries.
- `crates/bone_physics_core/src/cache.rs`: checkpoint encoding and validation.
- `crates/bone_physics_core/src/hash.rs`: canonical configuration hashes.
- `crates/bone_physics_core/src/diagnostics.rs`: mesh and CCD counters.
- `crates/bone_physics_py/src/world.rs`: mesh batch and checkpoint bridge.
- `blender_extension/bone_physics/mesh_builder.py`: pre-physics collision mesh generation.
- `blender_extension/bone_physics/ownership.py`: object/face/body filtering.
- `blender_extension/bone_physics/bake_cache.py`: cache index, replay, bake.
- `blender_extension/bone_physics/diagnostics.py`: structured reports.
- `tests/reference/`: JSON reference expectations.
- `tests/blender/scenes/`: deterministic fixture generators.
- `benchmarks/`: large-scene benchmark definitions.
- `scripts/package_extension.py`: reproducible extension packaging.

---

### Task 1: Stable deforming-mesh storage, validation, and refit

**Files:**
- Create: `crates/bone_physics_core/src/collision/mesh.rs`
- Modify: `crates/bone_physics_core/src/collision/mod.rs`
- Modify: `crates/bone_physics_core/Cargo.toml`
- Test: `crates/bone_physics_core/tests/deforming_mesh.rs`
- Property test: `crates/bone_physics_core/tests/mesh_properties.rs`

**Interfaces:**
- Produces: `MeshId(u32)`, `TriangleId(u32)`, and stable `RegionId(u32)`.
- Produces: `MeshTopology { vertex_count, triangles, triangle_regions, closed_hint }`.
- Produces: `DeformingMesh::build(topology, positions, thickness)`.
- Produces: `DeformingMesh::update_vertices(previous, current) -> MeshUpdate`.
- Produces: `MeshValidationReport { errors, warnings, topology_hash }`.
- Produces: `MeshUpdate::{Refit, Rebuilt}`.

- [ ] **Step 1: Write failing topology and validation tests**

Test out-of-range indices, zero-area/duplicate faces, empty valid topology,
non-manifold edges, inconsistent orientation, non-finite vertices, stable
triangle IDs, and a vertex-only update with unchanged topology.

- [ ] **Step 2: Write failing refit equivalence test**

Deform a closed mesh for 100 deterministic frames. Compare refitted broad-phase
query results against a full rebuild on every frame and require identical
sorted candidate triangle IDs.

- [ ] **Step 3: Implement owned topology and backend adapter**

Keep public mesh types independent of parry. Store previous and current
positions, triangle metadata, topology hash, and refittable acceleration data.
Reject a mesh with no valid triangles; retain warnings for recoverable defects.

- [ ] **Step 4: Run and commit**

Run:

```bash
cargo test -p bone_physics_core --test deforming_mesh
cargo test -p bone_physics_core --test mesh_properties
cargo test --workspace --all-targets
```

Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: store and refit deforming collision meshes"
```

### Task 2: Whole-capsule triangle contact and stable manifolds

**Files:**
- Create: `crates/bone_physics_core/src/collision/triangle.rs`
- Create: `crates/bone_physics_core/src/collision/manifold.rs`
- Modify: `crates/bone_physics_core/src/collision/contact.rs`
- Test: `crates/bone_physics_core/tests/capsule_triangle.rs`
- Test: `crates/bone_physics_core/tests/mesh_manifold.rs`

**Interfaces:**
- Produces: `capsule_triangle_contact(capsule, triangle, history) -> Option<ContactGeometry>`.
- Produces: `MeshContactKey { collider, mesh, triangle, feature }`.
- Produces: `cluster_mesh_contacts(raw, limits) -> Vec<ContactGeometry>` with at most four points.
- Produces: interpolated `surface_velocity` at triangle barycentric coordinates.

- [ ] **Step 1: Write failing narrow-phase tests**

Cover capsule side, cap, endpoint, triangle edge, triangle vertex, exact
coincidence, coplanar motion, degenerate rejected triangle, and contact
symmetry. Require finite barycentric coordinates, normals, and separation.

- [ ] **Step 2: Write failing manifold tests**

Place one capsule over a dense planar tessellation and assert at most four
clustered contacts with stable keys under small deformation. Place it at a
convex crease and assert both materially different normals remain.

- [ ] **Step 3: Implement geometry and clustering**

Query the continuous capsule segment against the complete triangle. Cluster by
world position and normal angle, retain deepest/stablest features, and sort by
stable contact key. Closed valid meshes may use pseudo-normal and local
inside/outside evidence; invalid meshes remain two-sided and use contact
history plus analytic fallback.

- [ ] **Step 4: Integrate contacts in the existing contact class**

Mesh and primitive contacts use the same unilateral XPBD row, friction,
restitution, warm starting, depenetration bookkeeping, and iteration schedule.

- [ ] **Step 5: Run and commit**

Run:

```bash
cargo test -p bone_physics_core --test capsule_triangle
cargo test -p bone_physics_core --test mesh_manifold
cargo test --workspace --all-targets
```

Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: solve deforming mesh contacts"
```

### Task 3: Collision ownership and Blender mesh generation

**Files:**
- Create: `blender_extension/bone_physics/ownership.py`
- Create: `blender_extension/bone_physics/mesh_builder.py`
- Modify: `blender_extension/bone_physics/prephysics_eval.py`
- Modify: `blender_extension/bone_physics/model.py`
- Test: `tests/python/test_ownership.py`
- Test: `tests/blender/test_mesh_builder.py`
- Test: `tests/blender/test_deformation_ownership.py`

**Interfaces:**
- Produces: `OwnershipMap { object_roles, face_regions, body_region_ignores }`.
- Produces: `CollisionMeshBatch { topology, previous_vertices, current_vertices, regions, thickness }`.
- Produces: `build_collision_surface(evaluated_object, policy)`.
- Produces topology fingerprint and deformation-input fingerprint.

- [ ] **Step 1: Write failing pure ownership tests**

Assert simulated hair/garment objects are excluded, excluded materials and face
attributes map to ignored regions, and every physical body omits regions
directly driven by that body's weight group.

- [ ] **Step 2: Generate and test a Blender ownership fixture**

Build a torso with upstream spine weights and a breast surface driven by a
physical breast bone. Assert the breast bone sees the upstream torso/internal
chest proxy and never sees its own driven surface. Build hair roots embedded in
the scalp and assert configured root insets/region masks prevent false contact.

- [ ] **Step 3: Implement collision-surface construction**

From evaluated pre-physics geometry: triangulate, discard degenerate/duplicate
faces, preserve stable region attributes, bake nonuniform scale, and emit
validation metadata. Provide deterministic simplification with a bounded
surface-error setting; when disabled, preserve the full evaluated topology.

- [ ] **Step 4: Implement upstream-only deformation**

Transfer or merge physical-bone weights into declared upstream parent bones for
the collision copy. Evaluate it before physical output. Topology must remain
stable across normal animation frames.

- [ ] **Step 5: Run and commit**

Run:

```bash
python -m unittest tests.python.test_ownership -v
blender --background --factory-startup \
  --python tests/blender/test_mesh_builder.py
blender --background --factory-startup \
  --python tests/blender/test_deformation_ownership.py
```

Expected: PASS, including proof that a physical body does not collide with the
surface it directly deforms.

```bash
git add blender_extension tests
git commit -m "feat: build ownership-safe collision surfaces"
```

### Task 4: Mesh upload through the batch bridge

**Files:**
- Modify: `crates/bone_physics_py/src/batch.rs`
- Modify: `crates/bone_physics_py/src/world.rs`
- Modify: `blender_extension/bone_physics/runtime.py`
- Test: `tests/python/test_native_mesh.py`
- Test: `tests/blender/test_mesh_upload.py`

**Interfaces:**
- Produces: `set_mesh_topology(handle, mesh_id, topology_batch) -> topology_hash`.
- Produces: `update_mesh_vertices(handle, mesh_id, previous, current, expected_hash) -> "refit"`.
- Produces: `remove_mesh(handle, mesh_id)`.

- [ ] **Step 1: Write failing bridge validation tests**

Reject mismatched vertex counts, stale topology hash, bad triangle/region
buffers, NaN, and a current-only update lacking previous motion data.

- [ ] **Step 2: Implement topology-once and vertex-update paths**

Copy and validate buffers before world mutation. A topology hash mismatch
returns a typed invalidation error rather than silently rebuilding during a
step.

- [ ] **Step 3: Integrate runtime update ordering**

At each evaluated time: capture pre-physics target/collider/mesh data, update
the native world, step, read transforms, then write pose deltas.

- [ ] **Step 4: Run and commit**

Run:

```bash
maturin develop --manifest-path crates/bone_physics_py/Cargo.toml
python -m unittest tests.python.test_native_mesh -v
blender --background --factory-startup \
  --python tests/blender/test_mesh_upload.py
```

Expected: PASS.

```bash
git add crates/bone_physics_py blender_extension tests
git commit -m "feat: upload deforming meshes in batches"
```

### Task 5: Continuous capsule collision for authoritative quality

**Files:**
- Create: `crates/bone_physics_core/src/collision/ccd.rs`
- Modify: `crates/bone_physics_core/src/collision/capsule.rs`
- Modify: `crates/bone_physics_core/src/collision/triangle.rs`
- Modify: `crates/bone_physics_core/src/world.rs`
- Test: `crates/bone_physics_core/tests/continuous_collision.rs`
- Property test: `crates/bone_physics_core/tests/ccd_properties.rs`

**Interfaces:**
- Produces: `CcdResult::{Impact { toi, geometry }, NoImpact, Failed { fallback }}`.
- Consumes: `QualityPolicy::{DeterministicPreview, ApproximatePreview { max_steps }, AuthoritativeBake}` from the fixed-step module.
- Produces earliest-time-of-impact queries for moving capsule/capsule and moving capsule/triangle.

- [ ] **Step 1: Write failing tunneling tests**

Test a fast arm proxy sweeping through hair, a capsule crossing a moving
triangle in one fixed step, two capsules crossing, contact at `toi=0`, grazing
motion, and degenerate parallel motion. Require no authoritative residual
penetration beyond tolerance.

- [ ] **Step 2: Verify failure**

Run: `cargo test -p bone_physics_core --test continuous_collision`
Expected: FAIL because CCD is absent.

- [ ] **Step 3: Implement conservative advancement**

Use swept broad-phase candidates, bounded conservative advancement/root
refinement, stable maximum iterations, and earliest sorted impact. On
non-convergence, record a diagnostic and fall back to speculative contact plus
substep refinement; never return a non-finite TOI.

- [ ] **Step 4: Separate quality policies**

Deterministic preview retains all fixed time and uses speculative contacts.
Approximate preview may cap display-frame work and cannot checkpoint.
Authoritative bake enables CCD, higher iterations, and no dropped physical
time. Constraint semantics and material parameters remain identical.

- [ ] **Step 5: Run and commit**

Run:

```bash
cargo test -p bone_physics_core --test continuous_collision
cargo test -p bone_physics_core --test ccd_properties
cargo test --workspace --all-targets
```

Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: add authoritative continuous collision"
```

### Task 6: Versioned checkpoints and canonical invalidation

**Files:**
- Create: `crates/bone_physics_core/src/hash.rs`
- Create: `crates/bone_physics_core/src/cache.rs`
- Modify: `crates/bone_physics_core/Cargo.toml`
- Modify: `crates/bone_physics_core/src/world.rs`
- Test: `crates/bone_physics_core/tests/checkpoint.rs`
- Test: `crates/bone_physics_core/tests/cache_hash.rs`

**Interfaces:**
- Produces: `CacheFormatVersion`.
- Produces: `WorldFingerprint([u8; 32])`.
- Produces: `SolverWorld::encode_checkpoint() -> Result<Vec<u8>, SolverError>`.
- Produces: `SolverWorld::restore_checkpoint(bytes, expected_fingerprint)`.
- Serializes bodies, velocities, multipliers, contact cache, previous mesh vertices, fixed-clock remainder, versions, and hashes.

- [ ] **Step 1: Write failing round-trip and corruption tests**

Run a colliding graph, checkpoint, continue, restore, and replay. Assert the
same-platform tolerance. Reject corrupt bytes, wrong format/API version,
changed topology, changed relevant animation/configuration inputs, and truncated
contact state.

- [ ] **Step 2: Write canonical hash tests**

Equivalent deterministic inputs produce the same hash independent of map
insertion order. Bone topology, rest transforms, constraints, physical
parameters, ownership, collision topology, relevant animation/deformation,
FPS, fixed step, and versions change the hash. Materials and lighting do not.

- [ ] **Step 3: Implement versioned encoding**

Use fixed field order and explicit little-endian/version headers. Set bounded
decode limits before allocation. Restore into a temporary world, validate all
state, then atomically replace the live state.

- [ ] **Step 4: Run and commit**

Run:

```bash
cargo test -p bone_physics_core --test checkpoint
cargo test -p bone_physics_core --test cache_hash
cargo test --workspace --all-targets
```

Expected: PASS.

```bash
git add crates/bone_physics_core
git commit -m "feat: serialize deterministic solver checkpoints"
```

### Task 7: Blender bake cache, replay, and diagnostics

**Files:**
- Create: `blender_extension/bone_physics/bake_cache.py`
- Create: `blender_extension/bone_physics/diagnostics.py`
- Modify: `blender_extension/bone_physics/runtime.py`
- Test: `tests/python/test_bake_cache.py`
- Test: `tests/blender/test_bake_seek.py`
- Test: `tests/blender/test_bake_subframes.py`

**Interfaces:**
- Produces: `BakeManifest { fingerprint, quality, frame_poses, checkpoints, complete }`.
- Produces: `BakeCache.begin`, `write_frame`, `write_checkpoint`, `commit`, `abort`, `restore_nearest`.
- Produces: `DiagnosticReport` with phase timing, contacts, candidates, errors, penetration, CCD fallbacks, and cache events.

- [ ] **Step 1: Write failing transactional cache tests**

Assert incomplete bake data is never treated as complete, writes are atomic,
approximate preview cannot write authoritative paths, changed fingerprints
invalidate with a named reason, and failed steps retain replayable snapshots.

- [ ] **Step 2: Write failing Blender seek tests**

Bake a moving hair scene, save and reopen, seek backward and forward, and
compare against uninterrupted playback. Confirm changed relevant animation
invalidates while changed lighting does not.

- [ ] **Step 3: Implement strict forward bake**

At each physical time, optionally request actual Blender dependency-graph
evaluation at the subframe, upload pre-physics targets and mesh, advance once,
validate diagnostics, store frame poses, and periodically store complete native
checkpoints. Abort transactionally on failure.

- [ ] **Step 4: Implement replay and readable diagnostics**

Restore the nearest compatible checkpoint and advance with exact fixed steps.
JSON diagnostics include stable offending body, constraint, contact, mesh, and
triangle IDs where applicable.

- [ ] **Step 5: Run and commit**

Run:

```bash
python -m unittest tests.python.test_bake_cache -v
blender --background --factory-startup \
  --python tests/blender/test_bake_seek.py
blender --background --factory-startup \
  --python tests/blender/test_bake_subframes.py
```

Expected: PASS.

```bash
git add blender_extension tests
git commit -m "feat: bake and restore authoritative physics"
```

### Task 8: Complete reference-scene acceptance suite

**Files:**
- Create: `tests/blender/scenes/common.py`
- Create: `tests/blender/scenes/hair.py`
- Create: `tests/blender/scenes/tail.py`
- Create: `tests/blender/scenes/skirt.py`
- Create: `tests/blender/scenes/body_collision.py`
- Create: `tests/blender/run_reference.py`
- Create: `tests/reference/acceptance.json`
- Create: `benchmarks/large_scene.json`

**Interfaces:**
- Produces deterministic generated scenes for all specification references.
- Produces one numerical JSON report per scene and FPS.

- [ ] **Step 1: Generate reference fixtures in code**

Include single pendulum, one/multiple hair chains, graded tail, closed skirt
lattice, fast arm, skirt between moving legs, embedded/accidental penetration,
and breast physics against upstream torso.

- [ ] **Step 2: Run equal-time and repeatability matrices**

For 24, 30, and 60 FPS, compare at equal physical times. Run authoritative
bakes twice on one platform. Assert rigid-length, anchor, penetration, angular
lock, equal-time, and repeat-bake tolerances from the approved specification.

- [ ] **Step 3: Add randomized failure snapshots**

When a case fails, emit the seed, build batch, animation inputs, previous/current
mesh vertices, checkpoint, diagnostic IDs, and actual versus allowed values.

- [ ] **Step 4: Run the suite**

Run:

```bash
blender --background --factory-startup \
  --python tests/blender/run_reference.py -- \
  --acceptance tests/reference/acceptance.json \
  --output artifacts/reference
```

Expected: every scene reports `"accepted": true`; no NaN or solver abort.

- [ ] **Step 5: Commit**

```bash
git add tests benchmarks
git commit -m "test: add first-version acceptance scenes"
```

### Task 9: Cross-platform packages and installation verification

**Files:**
- Create: `scripts/package_extension.py`
- Create: `scripts/verify_package.py`
- Create: `.github/workflows/package.yml`
- Modify: `.github/workflows/ci.yml`
- Create: `docs/building.md`
- Create: `docs/testing.md`

**Interfaces:**
- Produces platform extension archives for Windows x86_64, Linux x86_64, macOS x86_64, and macOS arm64.
- Produces SHA-256 manifest and software-license inventory.
- Produces required matrix job `package-and-load`.

- [ ] **Step 1: Write failing package-structure tests**

Verify manifest metadata, Python sources, exactly one matching native module
per archive, API version, license files, and absence of source/toolchain/pip
runtime requirements.

- [ ] **Step 2: Implement reproducible packaging**

Build abi3 wheels with maturin, extract only required native artifacts into the
extension layout, normalize archive timestamps/order/permissions, and generate
checksums plus dependency inventory.

- [ ] **Step 3: Verify fresh-host loading**

On each supported runner, install the archive into a clean Blender 5.2 LTS
profile with network disabled for the load test. Import the extension, verify
the native API, build a two-bone world, step it, and uninstall cleanly.

- [ ] **Step 4: Upload all platform evidence**

Upload archives, checksums, dependency inspection, Blender console output, and
load-test JSON with `if: always()`. Do not publish a GitHub release until the
matrix and full acceptance suite pass.

- [ ] **Step 5: Commit**

```bash
git add scripts .github/workflows docs
git commit -m "build: package self-contained Blender extension"
```

### Task 10: V1 release-candidate gate

**Files:**
- Modify: `README.md`
- Create: `CHANGELOG.md`
- Create: `docs/limitations.md`
- Modify: `.github/workflows/package.yml`

**Interfaces:**
- Produces a signed-off V1 acceptance artifact and draft release candidate.

- [ ] **Step 1: Run the complete required check set**

Run in CI:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo doc --workspace --no-deps
python -m compileall -q blender_extension/bone_physics tests/python
python -m unittest discover -s tests/python -v
```

Then run all native-contract, Blender runtime, reference, seek/rebake, and
platform package jobs.

- [ ] **Step 2: Inspect every always-uploaded artifact**

Require nonempty Cargo, Python, Blender, numerical reference, failure snapshot,
package, checksum, and dependency reports. A green status with missing evidence
does not pass the gate.

- [ ] **Step 3: Compare every acceptance criterion**

Generate `v1-acceptance.json` containing each approved criterion, test/job
source, observed value, allowed tolerance, commit SHA, platform, and pass/fail.
Any missing criterion is a failure.

- [ ] **Step 4: Document factual limitations**

Document unsupported wind, GPU, tearing, cross-armature constraints,
non-guaranteed invalid-mesh inside/outside behavior, and platform floating-point
tolerances. Include build, test, install, and generated-scene instructions.

- [ ] **Step 5: Commit and create a draft release candidate**

```bash
git add README.md CHANGELOG.md docs .github/workflows/package.yml
git commit -m "docs: prepare first-version release candidate"
```

Create a draft release only from the exact commit whose full acceptance
artifact passes. Attach archives, checksums, license inventory, and
`v1-acceptance.json`.
