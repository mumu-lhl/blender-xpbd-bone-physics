# Blender XPBD Bone Physics — Design Specification

Status: approved design, pending written-spec review  
Date: 2026-07-25  
Target: Blender 5.2 LTS, with future compatibility expansion  
License: GPL-3.0

## 1. Purpose

This project implements a new general-purpose Blender secondary-motion solver for
physics bones. It is not a compatibility layer for UMA, PMX, Bullet, or any
existing Blender add-on. Those systems are references only.

The solver must support hair, skirts, cloth rigs, tails, breasts, accessories,
and other bone-driven secondary motion. It prioritizes physical quality,
stability, and artistic control over minimum computational cost.

The core solver is written in Rust. Blender integration is written in Python.
Users receive a self-contained Blender extension and do not install Rust, pip
packages, NumPy, or system physics libraries.

## 2. Goals

The first version shall provide:

- oriented rigid bone segments solved with XPBD;
- fixed-step simulation independent of Blender output frame rate;
- controllable gravity and animation-relative inertia;
- animated rest-pose recovery for realistic and stylized motion;
- root-to-tip parameter fields;
- local XYZ angular lower and upper limits, including axis locking;
- constraints within and between arbitrary bone chains;
- capsule self-collision and cross-chain collision;
- collision against analytic body proxies;
- collision against pre-physics deforming triangle meshes;
- a high-quality bake mode with subframe evaluation and continuous collision;
- deterministic cache invalidation and recoverable simulation checkpoints;
- a zero-user-install-dependency distribution.

## 3. Non-goals for the First Version

The first version does not include:

- wind or aerodynamic surface models;
- GPU solving;
- plasticity, tearing, or breakable constraints;
- fluids;
- bidirectional physics between separate animated characters;
- cross-armature dynamic constraints;
- numerical reproduction of PMX/Bullet, UMA, or other solvers;
- production interaction design beyond the minimum needed to exercise and test
  the solver.

PMX import may later translate rigid bodies and joints into this project's
constraint graph, but PMX semantics do not define the solver's limits.

## 4. Lessons from the Reference Add-ons

### 4.1 cyspring

Useful ideas:

- strict separation between Blender adaptation and a native solver;
- reference-space bone rotation handling;
- collision correction must not create artificial velocity;
- collision must be rechecked after angular projection;
- typed batch data, deterministic caches, and strong tests;
- root-to-tip parameter gradients.

Ideas not carried forward:

- UMA-specific constants, layout, and compatibility behavior;
- point-only bone collision;
- a single forward sequential Verlet chain pass;
- Euler-only rotational state;
- skipped-frame simulation using one oversized logical update.

### 4.2 blender-jiggle-physics

Useful ideas:

- stable Verlet history;
- separation of local and global motion;
- animation-reference blending;
- handling branching directions.

Limitations:

- endpoints represent bones incompletely;
- no true twist state;
- no self-collision;
- mesh collision relies on closest points for individual particles;
- one-pass sequential constraints create order dependence.

### 4.3 blender-wiggle-2

Useful ideas:

- capsule/segment self-collision;
- analytic collider variety;
- explicit treatment of bounce and velocity history;
- iterative constraints.

Limitations:

- self-collision occurs after the main solve instead of inside it;
- later corrections can break length, body collision, or angle limits;
- exact zero-distance pairs may be skipped;
- primitive transforms and nonuniform scale are not handled rigorously;
- behavior is patched across a large monolithic implementation.

### 4.4 wiggle-bones

Useful ideas:

- modularization attempts;
- forward and reverse traversal illustrate Gauss-Seidel order bias;
- pinning naturally maps to constraints.

Rejected approach:

- sampling points along a bone is not a replacement for continuous capsule
  geometry and can produce invalid distant collision responses.

## 5. Selected Solver Architecture

The core is an oriented XPBD constraint graph, also describable as a
position-based rod system.

Each physical bone is an oriented rigid segment with:

- position and quaternion orientation;
- linear and angular velocity;
- mass and local inertia;
- fixed local head and tail anchors;
- an optional capsule collider;
- an animation reference pose;
- persistent constraint state.

The solver does not iterate "chain A, then chain B." A chain is authoring and
construction metadata. At runtime, all bodies and constraints form one graph.

This architecture was selected over:

1. endpoint Verlet/PBD, which cannot represent twist or robust rotational
   constraints cleanly; and
2. a complete impulse-based rigid-body engine, which is less direct for
   animation rest targets, bone sheets, and controllable stylization.

## 6. Dependency and Distribution Boundary

The user-facing extension has no separately installed runtime dependencies.
Platform-specific wheels are bundled with the Blender extension.

The Rust workspace may use a small audited dependency set:

- PyO3 for the compiled Python bridge;
- maturin as a build and packaging tool;
- glam for vector, quaternion, and transform math;
- parry3d behind an internal collision-backend interface;
- Rayon as an optional, profiled parallel execution feature.

Rapier and Bullet are not solver dependencies. Parry may provide geometric
queries and BVH infrastructure, but XPBD constraints, scheduling, state, and
contact response remain project-owned.

The core crate has no dependency on Python or Blender. Only the bridge crate
depends on PyO3.

## 7. Module Boundaries

### 7.1 Rust

- `solver-core`: world state, integration, fixed-step loop, velocity rebuild;
- `constraints`: joints, pose recovery, bend, twist, limits, cross-links;
- `constraint-graph`: stable IDs, ordering, coloring, dependency groups;
- `collision-primitives`: capsules and analytic colliders;
- `collision-mesh`: triangle meshes, BVH, manifolds, continuous queries;
- `cache`: versioned checkpoints and state hashes;
- `solver-python`: batched PyO3 API and panic boundary.

### 7.2 Blender Python

- `scene_adapter`: coordinate, unit, armature, and object conversion;
- `prephysics_eval`: animation-reference and collision-surface evaluation;
- `parameter_builder`: parameter-field sampling and constraint construction;
- `collider_builder`: analytic proxies and simplified collision surfaces;
- `runtime`: solver-world lifetime, frame progression, re-entry prevention;
- `bake_cache`: bake execution, checkpoints, playback, invalidation;
- `pose_writer`: final pose-delta application;
- `diagnostics`: validation, convergence, contact, and performance reports.

Python never performs a per-constraint or per-contact solve loop. Rust worker
threads never call the Blender API.

## 8. Core Data Model

### 8.1 BoneBody

Static data:

- stable body ID;
- rest transform;
- local center of mass;
- local head and tail anchors;
- inverse mass and inverse local inertia;
- capsule local endpoints, radius, and collision margin;
- collision groups and exclusions.

Dynamic data:

- current and previous position;
- current and previous quaternion;
- predicted position and quaternion;
- linear and angular velocity;
- accumulated non-velocity contact correction;
- validity state.

Scale is not simulated. Nonuniform object scale is baked into geometry and
shape parameters before data crosses the Rust boundary.

### 8.2 AnimationTarget

Each target contains the current pre-physics animation transform and the
previous target transform. Targets are interpolated at fixed substeps.

Targets act as:

- kinematic roots;
- soft positional attachments;
- animated orientation rest states;
- sources of controlled translation and rotation inertia.

### 8.3 PersistentConstraint

Constraints reference stable body IDs and local anchors. They store:

- type and enabled state;
- local frames or anchor points;
- rest value or allowed range;
- compliance;
- damping;
- cached Lagrange multipliers;
- collision-exclusion behavior.

### 8.4 Contact

Contacts are generated and updated per substep. A cached contact contains:

- stable pair and feature IDs;
- local anchors;
- normal and tangent basis;
- separation or penetration;
- surface velocity;
- normal and friction multipliers;
- previous normal;
- contact age.

## 9. Motion Model and Parameters

The model separates five concepts that must not be hidden behind one generic
stiffness control:

1. gravity;
2. animation-relative inertia;
3. rest-pose recovery;
4. velocity damping;
5. geometric and angular limits.

Per-body physical values include:

- mass or density;
- inertia scale;
- gravity scale;
- linear damping per second;
- angular damping per second;
- collision radius and margin;
- friction;
- restitution.

Mass does not scale gravitational acceleration. It affects inertia and how
constraint corrections are distributed.

Per-joint recovery values include:

- swing compliance;
- twist compliance;
- pose damping;
- positional attachment compliance.

The animated reference orientation is sampled from the pre-physics pose.
Stylized hair that must retain a designed silhouette uses lower gravity and
stronger pose recovery. Realistic motion uses normal gravity and weaker
recovery.

Translation-inertia and rotation-inertia factors control how much armature
root motion produces secondary lag. A value of one is world-inertial behavior;
zero moves with the character reference frame. Values above one are permitted
for deliberate exaggeration.

Final animation/physics blending is not performed after collision because it
could blend a valid pose back into the body. Physical influence is expressed
through animation-target constraints inside the solve.

## 10. Root-to-tip Parameter Fields

Chains provide normalized arc-length coordinate `s` in `[0, 1]`. Authoring
curves are sampled into concrete per-body and per-constraint values when the
runtime graph is built.

The following values can vary along a chain:

- mass or density;
- inertia scale;
- gravity scale;
- linear and angular damping;
- swing and twist compliance;
- animation inertia;
- collision radius;
- XYZ angular limits;
- cross-chain compliance.

Sampling uses accumulated rest length rather than only bone index, so
unevenly sized bones receive a geometrically meaningful distribution.

## 11. Constraint System

The first version supports:

- hard or compliant anchor coincidence;
- target distance;
- distance lower and upper range;
- animated attachment;
- relative-orientation recovery;
- bend;
- twist;
- local XYZ angular limits;
- swing-cone and twist-range limits.

XPBD uses per-constraint compliance `alpha`, with:

```text
alpha_tilde = alpha / dt^2
delta_lambda =
    (-C - alpha_tilde * lambda)
    / (sum(weighted_gradient_squared) + alpha_tilde)
```

Rotational constraints use inverse inertia and rotational Jacobians, not only
endpoint position corrections.

### 11.1 Local XYZ Limits

The public model specifies lower and upper X, Y, and Z rotation relative to
the joint rest frame. Equal lower and upper values lock an axis.

Internal rotation remains quaternion-based. Limit projection:

- transforms candidate relative rotation into the rest joint frame;
- maintains a continuous angular branch across `+/- pi`;
- projects the selected parameterization into the allowed set;
- converts the correction back into a quaternion update;
- remains inside the iterative loop so collision is solved again afterward.

When an exact XYZ box is not required, a swing cone plus twist interval is the
preferred stable representation. A joint selects one primary limit model to
avoid contradictory constraints.

### 11.2 Cross-chain Constraints

Any local point on one bone may connect to any local point on another bone.
Constraint construction is not limited to matching heads or tails.

Chains with different bone counts are matched using normalized accumulated
length, not raw index.

A skirt lattice uses:

- vertical rigid bone segments and joints;
- horizontal distance links between neighboring chains;
- diagonal distance links to resist shear;
- a closed seam between the first and last chain where applicable;
- stronger waist attachments and softer lower links.

The same graph can represent grouped hair, braids, capes, branching tails,
paired soft-tissue chains, and hanging accessories.

Direct structural neighbors may be excluded from collision. More distant
members of the same chain remain eligible for self-collision.

### 11.3 Scheduling

Each solver iteration visits:

1. joint anchors;
2. recovery, bend, and twist;
3. cross-chain structural constraints;
4. angular limits;
5. contact constraints.

The next iteration returns to anchors, so no final pass is allowed to
permanently invalidate an earlier class.

Iteration direction alternates. Stable graph coloring identifies constraint
sets that do not write the same bodies. Only independent colors may execute in
parallel.

Hard parent joints are allowed. Cyclic cross-links and diagonals retain finite
compliance to avoid an unsatisfiable all-rigid graph.

## 12. Bone Self-collision and Cross-chain Collision

Each bone collider is a continuous capsule, not a particle and not a set of
sample points:

```text
Capsule = Segment(local_a, local_b) expanded by radius
```

The capsule may be shorter than the visible bone. Head and tail inset allow
roots embedded in the scalp or waist to remain non-colliding.

### 12.1 Broad Phase

Swept AABBs include previous and predicted capsule poses. A dynamic BVH
produces candidate pairs. Filtering applies:

- collision layers;
- explicit masks;
- self pairs;
- parent-child neighbors;
- shared hard-joint neighbors;
- configured structural-neighbor exclusions.

### 12.2 Narrow Phase

Capsule pairs use closest points between center segments. Long parallel
overlaps may generate a two-point manifold to provide stable torque.

Exact centerline coincidence is never skipped. Degenerate normals use:

1. a cached previous normal;
2. relative velocity;
3. bone-axis cross products;
4. a deterministic orthogonal direction derived from stable IDs.

### 12.3 Contact Response

Normal contact is a unilateral XPBD inequality. Its multiplier cannot become
attractive.

Position and orientation corrections are distributed using inverse mass and
inverse inertia. Contact correction is recorded separately so pure
depenetration does not become accidental bounce during velocity rebuild.

The velocity phase handles moving-surface-relative friction and explicit
restitution. Restitution is applied only to new impacts with closing normal
velocity.

### 12.4 Continuous Collision

Real-time mode uses:

- swept broad-phase bounds;
- fixed substeps;
- speculative contacts.

High-quality mode adds continuous capsule motion queries and earliest-time-of-
impact handling. Preview and bake share the same solver and constraints; bake
changes quality settings rather than physics semantics.

## 13. Body Collision

Body collision uses a hybrid representation:

1. analytic proxies for stability and deep-penetration fallback;
2. a simplified deforming triangle surface for normal high-quality use;
3. an optional full evaluated triangle surface for maximum-quality bake.

Analytic proxies are placed slightly inside the true body surface. They act as
a safety cage, not as a second external shell. When mesh contact is valid, the
mesh defines the visible collision boundary.

### 13.1 Pre-physics Ownership Rule

The collision surface is evaluated after ordinary animation but before
secondary-physics results are applied.

The required order is:

1. evaluate ordinary animation and drivers;
2. keep physical bones at their animation-reference transforms;
3. evaluate the collision surface;
4. solve secondary physics;
5. write physical bone deltas;
6. evaluate the final render mesh;
7. do not feed that final mesh back into the same physical time point.

A dynamic bone must not collide with the mesh region it directly drives.
Otherwise, bone motion deforms the obstacle, and the obstacle then pushes the
same bone again.

Collision ownership is filtered at three levels:

- object level: simulated hair or garments are not body obstacles;
- face level: excluded materials, attributes, or driven regions;
- body level: explicit bone-to-region ignore relations.

For soft body parts such as breasts, the collision surface is the upstream
torso or an internal chest proxy, not the final breast surface driven by the
same physical bone.

### 13.2 Deforming Mesh Pipeline

On initial build, Blender uploads:

- stable vertex and triangle IDs;
- triangle topology;
- current vertex positions;
- validation metadata.

When topology is unchanged, only vertex positions are uploaded and the Rust
BVH is refitted. Topology changes require rebuild and cache invalidation.

Real-time substeps interpolate previous and current pre-physics vertices.
High-quality bake may request actual Blender dependency-graph evaluation at
each physical subframe.

Capsule-to-mesh queries use the whole capsule. Triangle contacts are clustered
by position and normal into a small stable manifold so dense meshes do not
produce artificially stiff hundreds-of-contact responses.

The contact includes surface velocity. Animated arms and torso therefore push
and drag secondary bones with correct relative friction.

### 13.3 Collision Surface Generation

A simplified deforming collision mesh is derived from an identified body
surface:

- remove simulated hair, garments, and accessories;
- triangulate;
- remove degenerate and duplicate faces;
- simplify under a bounded surface-error criterion;
- preserve high-curvature body regions;
- retain or transfer weights only for upstream animation bones;
- merge physical-bone weights into appropriate upstream parent bones;
- support uniform or attribute-driven collision thickness.

The result maintains stable topology while following the primary animation.

### 13.4 Analytic Proxy Generation

Proxy fitting:

1. collects vertices influenced by an upstream body bone;
2. transforms them into bone-local coordinates;
3. bins them along the bone axis;
4. estimates radial extent using robust percentiles;
5. fits capsules or tapered capsules to limbs;
6. fits spheres or convex shapes to heads;
7. uses multiple rounded boxes, capsules, or convex shapes for torso/pelvis;
8. fills gaps around shoulders, elbows, hips, and knees.

Generated proxies are starting points and remain editable at the data level.
Nonuniform scale is baked into explicit shape dimensions.

### 13.5 Inside/outside and Invalid Meshes

Closed oriented meshes use pseudo-normals, local inside/outside tests, and
contact-history continuity.

Open, non-manifold, self-intersecting, or inconsistently oriented meshes
cannot guarantee a globally correct depenetration direction. They use
two-sided surface collision plus history, while the inner proxy provides a
fallback direction.

The build validates and reports:

- non-manifold edges;
- zero-area faces;
- inconsistent orientation;
- self-intersection indicators;
- open thin regions.

### 13.6 Initial Penetration

Intentional embedded roots use:

- shortened local capsules;
- head and tail insets;
- disabled body collision for selected root bodies;
- region masks.

Accidental initial penetration is removed through staged, zero-restitution
projection. Historical state is corrected alongside pose state so the first
frame does not explode.

## 14. Frame Evaluation and Fixed Time

The simulation uses a configurable fixed physical step, with `1/120 s` as the
initial default.

The Blender frame duration is accumulated into fixed steps. Kinematic targets
and collider motion are evaluated or interpolated at each physical time.

Timeline behavior:

- ordinary forward playback advances continuously;
- a future jump restores the nearest checkpoint and replays;
- a backward jump restores a previous checkpoint;
- a jump outside valid cache starts from the simulation start or an earlier
  complete checkpoint;
- no jump is converted into one oversized `dt`.

Character motion is computed in a simulation-local frame near the character
to retain `f32` precision far from the Blender world origin.

## 15. Preview and Bake

Deterministic preview does not drop physical time. If the machine cannot keep
up, viewport playback slows.

An explicitly approximate preview mode may:

- cap steps per display frame;
- reduce contact iterations;
- disable expensive CCD.

Approximate preview never writes authoritative bake checkpoints.

High-quality bake:

- progresses strictly forward;
- uses fixed steps;
- may evaluate real Blender subframes;
- enables continuous body and bone collision;
- records convergence and penetration diagnostics;
- stores output poses and periodic complete solver checkpoints.

Physics frames cannot be baked independently because state is sequential.
Within a frame, collision queries and colored constraints may run in parallel.

## 16. Cache

Complete checkpoints include:

- body positions, quaternions, and velocities;
- XPBD multipliers;
- contact caches;
- previous pre-physics collision surface state;
- fixed-step accumulator remainder;
- solver and cache format versions;
- configuration hashes.

Fast playback stores final per-frame bone transforms in addition to periodic
complete checkpoints.

Cache validity includes:

- bone topology and rest transforms;
- constraint graph;
- physical parameters;
- collision topology and ownership filters;
- relevant animation and deformation inputs;
- output FPS and physical fixed step;
- solver and cache-format version.

Unrelated material or lighting changes do not invalidate physics.

## 17. Failure Handling and Diagnostics

Build errors prevent world creation for:

- near-zero bone length;
- duplicate or missing stable IDs;
- invalid constraint references;
- NaN or infinite inputs;
- negative mass, radius, or illegal ranges;
- collision meshes with no valid triangles;
- unsupported negative transforms that were not normalized.

Warnings identify non-manifold surfaces, suspicious weights, redundant
constraints, and highly overconstrained cycles.

Each substep validates:

- finite positions, rotations, and velocities;
- quaternion normalization;
- maximum anchor and angular error;
- maximum remaining penetration;
- continuous-query convergence;
- multiplier magnitude.

On divergence or non-finite state:

1. do not write the invalid pose into Blender;
2. roll back to the most recent valid state;
3. pause the affected simulation world;
4. report frame, body, constraint, and contact IDs;
5. allow deterministic replay from a checkpoint.

Rust panics are caught at the PyO3 boundary and never unwind into Blender.

Diagnostic counters include:

- time per solver phase;
- active and generated contacts;
- broad-phase candidate count;
- constraint error by type;
- deepest penetration;
- CCD failures and fallbacks;
- cache hit, replay, and invalidation reason.

## 18. Determinism

Stable IDs determine body, constraint, and contact ordering.

The implementation does not depend on randomized hash-map iteration. Candidate
pairs are sorted before constraint generation. Graph coloring and color order
are stable.

Authoritative bake uses a fixed scheduling policy. Repeated runs on the same
platform, solver version, and inputs must reproduce results within the defined
floating-point tolerance.

Bit-identical output across CPU architectures is not required, but
cross-platform differences must remain below visible animation thresholds.

## 19. Test Strategy

### 19.1 Rust Unit and Property Tests

- vector, quaternion, and reference-frame operations;
- position and orientation XPBD constraints;
- compliance behavior across fixed-step choices;
- XYZ locks and limits around `+/- pi`;
- swing-twist decomposition;
- parallel, coincident, and zero-distance capsules;
- capsule-triangle contact;
- speculative and continuous collision;
- friction and zero-restitution resting contact;
- cache serialization and restoration;
- randomized degenerate inputs that must not produce NaN.

### 19.2 Reference Simulations

- single pendulum;
- single hair chain;
- multiple colliding hair chains;
- root-stiff, tip-soft tail;
- closed skirt lattice with horizontal and diagonal cross-links;
- a fast arm sweeping through hair;
- skirt trapped between moving legs;
- intentional embedded roots;
- accidental initial penetration;
- breast physics against an upstream torso collision surface;
- timeline seek, reverse seek, reset, and rebake;
- equivalent time comparison at 24, 30, and 60 FPS.

Each reference records:

- maximum bone-length error;
- maximum joint separation;
- maximum residual penetration;
- peak velocity and angular velocity;
- repeat-run pose difference;
- phase timings and contact counts.

### 19.3 Blender Integration

Headless Blender tests verify:

- no pre/post-physics dependency-graph recursion;
- batch input and output transforms;
- cache persistence across save and reopen;
- correct cache invalidation;
- evaluated collision-surface ownership;
- no requirement for NumPy, Rust, or system libraries;
- loading on supported Windows, Linux, and macOS packages.

### 19.4 Numerical Acceptance Tolerances

All distances are evaluated after conversion to meters. Let `L_ref` be the
median positive rest-bone length in the simulation world, and let `r_contact`
be the smaller positive collider radius in a tested contact.

Default authoritative acceptance thresholds are:

- rigid segment length error:
  `max(1e-6 m, 1e-5 * segment_length)`;
- hard anchor separation:
  `max(1e-5 m, 1e-4 * L_ref)`;
- residual collision penetration:
  `max(1e-5 m, 0.01 * r_contact)`;
- locked-axis angular error: `1e-3 rad`;
- equal-time 24/30/60 FPS pose difference:
  `max(5e-5 m, 5e-4 * L_ref)` and `1e-3 rad`;
- repeated authoritative bake difference on the same platform:
  `max(1e-6 m, 1e-5 * L_ref)` and `1e-5 rad`;
- supported-platform reference-scene difference:
  `max(1e-4 m, 1e-3 * L_ref)` and `2e-3 rad`.

Tests may declare a stricter tolerance. A looser tolerance requires a
documented numerical reason and cannot be introduced only to hide a regression.

## 20. First-version Acceptance Criteria

The first version is complete when:

- all Rust unit, property, and cache tests pass;
- Blender headless integration tests pass on supported platforms;
- the reference hair, tail, and skirt scenes run without NaN or solver abort;
- enabled non-neighbor bone capsules do not remain interpenetrating beyond the
  numerical acceptance tolerance after authoritative solve;
- body contacts remain outside the pre-physics surface penetration tolerance;
- locked angular axes remain within the locked-axis tolerance;
- 24, 30, and 60 FPS scenes agree at equal physical times within the declared
  transform tolerance;
- repeated authoritative bakes agree within the deterministic tolerance;
- timeline seeking restores the same result as forward playback;
- the extension installs without external package installation;
- the repository contains build, test, and packaging instructions.

Performance is measured continuously, but quality acceptance is not waived to
meet a fixed real-time number. Real-time tiers may lower quality explicitly;
the authoritative bake tier remains the correctness reference.

## 21. Implementation Order

After this specification is approved, implementation planning shall decompose
the work in this order:

1. Rust workspace, math conventions, IDs, and deterministic state;
2. oriented body integration and fixed-step clock;
3. joint, recovery, bend, twist, and angular constraints;
4. chain construction, parameter fields, and cross-chain constraints;
5. capsule collision and contact response;
6. analytic body colliders;
7. PyO3 batch bridge and minimal Blender runtime;
8. pre-physics deforming mesh pipeline;
9. mesh contact, BVH refit, and high-quality continuous collision;
10. cache, bake, diagnostics, integration tests, and packaging.

Every stage requires tests before the next stage depends on it.
