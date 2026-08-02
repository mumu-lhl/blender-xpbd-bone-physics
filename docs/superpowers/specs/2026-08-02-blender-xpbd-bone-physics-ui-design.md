# Blender XPBD Bone Physics UI Design

Status: approved by the user on 2026-08-02; implementation not started

## Authority and scope

This specification defines the production authoring UI for the existing Blender
XPBD bone-physics extension. It extends, but does not replace, the solver
semantics in `docs/design/blender-xpbd-bone-physics-design.md`.

The UI targets Blender 5.2 LTS and lives in the 3D View N-panel. It supports
advanced character setup for hair, ears, ponytails, tails, skirts, small
accessories, and similar secondary-motion systems. It exposes physical
parameters, root-to-tip fields, constraints, collision, runtime controls,
diagnostics, JSON presets, and English and Simplified Chinese UI.

The current repository has no production Panel, PropertyGroup, registration
entry point, or preset system. The V1 solver and runtime remain the physical
implementation. This work adds an authoring layer and fills only the backend
authoring gaps needed to expose already-approved physics behavior.

## Design principles

- A physics system, such as Hair or Skirt, is the primary user-facing unit.
- Users configure many chains together; individual-bone editing is an advanced
  override.
- Physical behavior, local constraints, and collision are configured in one
  workflow without hiding their distinct meanings.
- Character collision resources are built once and reused by physics systems.
- Generated runtime constraints and physical capsules remain inspectable but do
  not flood primary lists.
- Every configuration operation is transactional and undoable.
- Stable IDs alone determine runtime ordering.
- The UI never offers options that violate approved XPBD scheduling,
  pre-physics ownership, or collision semantics.

## Authoring hierarchy

Configuration is stored per Armature Object:

```text
Armature Object
|-- Global settings
|-- Physics systems
|   |-- Hair
|   |   |-- Chains
|   |   |-- Endpoint parameter fields
|   |   |-- Local constraints
|   |   |-- Structural constraints
|   |   `-- Collision policy
|   |-- Ears
|   |-- Ponytail
|   |-- Skirt
|   `-- Accessories
|-- Collision resources
|   |-- Analytic colliders
|   |-- Pre-physics mesh surfaces
|   `-- Collision collections
`-- Stable-ID and schema state
```

An Armature Object owns configuration because multiple objects may share one
Armature Data datablock while using different animation, transforms, collision
objects, and caches.

A physical bone belongs to exactly one physics system. Collision resources may
be shared by any number of systems.

## N-panel information architecture

The 3D View sidebar contains a `Bone Physics` category. Full controls appear
only when the active object is an Armature. Other contexts show a short
instruction instead of stale controls.

### Fixed header

The top section always shows:

- active armature name and configuration status;
- global physics enable switch;
- configured physical-bone, chain, collider, and error counts;
- `Validate and Rebuild`;
- exact-armature preset menu with load, save, overwrite, rename, and delete.

### Workspaces

A compact enum row switches among `Setup`, `Simulate`, and `Diagnostics`.

#### Setup

The top-level browser contains `Physics Systems`, `Collision Resources`, and
`All Problems`. It supports search, enabled/error filters, creation,
duplication, deletion, sorting, and viewport-selection synchronization.

Selecting a physics system opens its chain, parameter, constraint, collision,
and override editors. Selecting a collision resource opens its shape, binding,
ownership, and visualization editor. Advanced fields are collapsed by default.

#### Simulate

This workspace contains:

- global enable and pause;
- per-system enable and isolation;
- reset of configured physical bones;
- deterministic and approximate preview quality;
- fixed step, iterations, and contact quality;
- cache state, invalidation reason, and clear-cache action;
- authoritative bake range, start, cancel, progress, and result.

#### Diagnostics

This workspace contains validation errors, warnings, contact statistics,
constraint error, performance counters, and cache state. Clicking an entry
selects or highlights its physics system, chain, bone, collider, mesh object,
or region. A copy action exports a concise report.

## End-to-end character workflow

### 1. Initialize the armature

`Initialize Physics Configuration` creates the Armature Object settings and
validates units and positive object scale. The user sets global gravity, fixed
step, preview quality, default collision layer, cache behavior, and bake range.

### 2. Build reusable character collision

The user creates pre-physics mesh surfaces and analytic proxies once. Named
collections such as `Head`, `Upper Body`, `Legs`, and `Whole Body` group these
resources for reuse.

### 3. Create a physics system

The add menu offers starting templates for loose hair, ears, ponytail/tail,
skirt, small accessory, and blank advanced setup. Templates supply editable
defaults only. They contain no object or bone bindings.

### 4. Add chains

In Pose Mode, the user selects one or more chain-head bones and chooses
`Add Chains from Selected Bones`. The extension follows direct child hierarchy;
it does not require Blender's `connected` flag.

An unbranched hierarchy ends at the deepest leaf. At a branch, the UI lists
descendant leaves and requires one tail choice for that selected head. The
chain is the unique parent path between head and tail.

Creation reports chain head, tail, and bone count. It rejects overlapping
physical chains, duplicate physical bones, missing paths, and zero-length
bones. A physical root under a non-physical parent attaches to that animated
parent without converting the parent into a dynamic body.

### 5. Configure motion

The user starts from a bone-parameter preset, then edits head values, tail
values, and shared root-to-tip interpolation. Multiple selected systems or
chains support mixed-value batch editing.

Typical intent:

- hair: strong root recovery, softer tips, moderate damping;
- ears: low gravity, strong recovery, constrained rotation, normally no
  self-collision;
- ponytails and tails: visible gravity and inertia, moderate recovery, enabled
  self-collision;
- skirts: strong waist attachment, soft hem, consistent multi-chain fields;
- small accessories: stronger recovery and angular limits, low restitution.

### 6. Configure structural constraints when needed

Ordinary chains need no manual constraint construction. Skirts, braids, and
bundled structures can add horizontal, diagonal, and closed-seam cross-chain
links.

### 7. Configure collision

Each system selects reusable collision collections, enables chain and
cross-chain self-collision as needed, and configures its collision matrix.
Physical capsules are generated from chain fields; they are not manually added
one bone at a time.

### 8. Validate and preview

`Validate and Rebuild` reports blocking errors before warnings. The user can
isolate one system, play the timeline, inspect collision and constraint
visualization, and rebuild only dirty systems.

### 9. Save and bake

The user can save reusable bone-parameter presets or one exact-armature preset.
Authoritative bake remains the final output path.

## Physics systems and chain editing

### Chain paths

Each chain stores a persistent chain ID, head and tail bone UUIDs, the resolved
bone path, and stable body IDs. Editing a head or tail re-resolves and validates
the entire path. Bone UUIDs allow ordinary renames to repair display names;
deletion and duplication conflicts remain explicit errors.

### Batch editing

The entity browser permits multi-selection of chains. A mixed-value field is
shown when selected chains disagree. Users may apply a value to all selected
chain heads, all selected chain tails, all shared interpolation settings, or
selected parameter categories. A per-field lock prevents accidental batch
replacement.

### Parameter groups

Head and tail endpoints expose matching groups:

- Motion: density/mass mode, inertia scale, gravity scale, translation and
  rotation animation inertia, linear damping, angular damping.
- Recovery: orientation-recovery compliance and damping, root positional
  attachment compliance and damping.
- Angular limits: free, local XYZ, or swing-cone plus twist range.
- Collision: capsule radius, margin, head and tail inset, friction,
  restitution, and advanced per-bone filtering overrides.
- Advanced: joint compliance, constraint damping, and local inertia scaling.

Mass does not scale gravitational acceleration. Recovery, damping, inertia, and
limits remain separate controls and retain the approved physical meanings.

### Root-to-tip fields

Every supported parameter may vary from head to tail. A shared curve supplies
the default interpolation. Built-in curve modes are constant, linear, smooth
Hermite, ease-in, and ease-out. Advanced users may define a multi-point Hermite
curve for an individual field.

Sampling uses accumulated rest length:

- body parameters sample at each bone midpoint;
- parent-child joint parameters sample at the child head boundary;
- root attachment samples at `s = 0`;
- per-bone overrides replace sampled values afterward.

Runtime constraints store only sampled concrete values. They never retain or
evaluate authoring curves.

### Per-bone overrides

The override section is collapsed by default and initially empty. `Add Selected
Bone Override` exposes a complete parameter editor for selected physical bones.
The list displays only bones with overrides. Overrides can be cleared by field,
category, bone, chain, or system.

## Constraint authoring

Constraints must act on at least one physical bone. Non-physical bones may be
kinematic targets but are never added as dynamic bodies solely to host a
constraint.

### Automatic local constraints

Chain creation produces:

- animated root attachment to a non-physical parent, selected bone, object,
  world, or a fully free root;
- head-tail anchor constraints between adjacent physical bones;
- animated orientation recovery for physical bones;
- one selected angular-limit model per joint.

Joint parameters are edited on the child bone because they control the child's
motion relative to its parent. Local XYZ and swing-twist are mutually exclusive
primary limit models.

### Explicit structural constraints

A structural-constraint group selects participating chains and normalized
length samples. It can create horizontal links, diagonal anti-shear links, and
a closed seam. Compliance and damping may vary from waist/root to hem/tip.

Changes to chain count or path rebuild generated links and report invalid
references. Cycle-forming links require finite nonzero compliance. The UI does
not allow an all-rigid cycle.

### Constraint inspection

Primary lists show counts and aggregate error, not every generated row. An
advanced read-only inspector exposes stable IDs, connected bones, kinds,
sampled values, and error for diagnosis.

All persistent constraints and contacts share every XPBD iteration. No final
collision-only, limit-only, blending, or post-correction option is exposed.

## Collision authoring

### Collision resource library

Collision resources live at armature-object scope and are reusable.

Analytic colliders support sphere, capsule, and rounded box shapes. A collider
may bind to a non-physical bone, an object, or the world. Bone- and
object-bound colliders follow the pre-physics animation and act as kinematic
obstacles. Viewport gizmos support selection and direct shape adjustment.

Pre-physics mesh surfaces specify:

- source Mesh Object;
- simplified deforming or full evaluated mode;
- thickness, simplification error, and closed-surface hint;
- excluded materials and region IDs;
- physical-body-to-region ignores;
- physical-weight merges into upstream non-physical bones;
- validation status for topology, orientation, and degeneracy.

The final physics-deformed mesh never feeds the same simulation time. Simulated
hair, garments, and accessories are not body obstacles. Physical-bone-driven
regions require explicit exclusion or upstream weight merging.

Collision collections reference resources without copying them. They may not
form nested cycles. Deleting a referenced collection or resource is blocked
until references are removed or explicitly reassigned.

### Physical-bone capsules

Every physical bone receives one continuous dynamic capsule generated from its
chain field. Capsule radius, margin, and head/tail inset may be overridden per
bone. Generated capsules live under their chains and are not duplicated in the
top-level analytic resource list.

V1 exposes one dynamic capsule per physical bone. It does not promise extra
dynamic spheres, boxes, or compound shapes.

### Self-collision and filtering

Each physics system controls:

- collision within one chain;
- collision between chains in the same system;
- collision with other named physics systems;
- referenced character collision collections;
- direct parent-child and shared-hard-joint exclusion;
- separate root-bone counts for body-collision and self-collision suppression;
- explicit excluded bone pairs;
- advanced layer/mask inspection and override.

Direct structural neighbors may be excluded. No transitive closure is applied.

Named policy compiles to stable 32-bit collision layers and masks. Exhausting
available bits is a validation error; policies are never silently merged.

### Collision visualization

Distinct colors identify dynamic capsules, kinematic proxies, mesh surfaces,
disabled resources, and invalid resources. Filters include current-system-only
and problems-only. Clicking a resource selects its Blender object or bound bone.

## Runtime controls

The global switch applies only to the active Armature Object. Disabling physics
restores configured physical bones to the current frame's pre-physics animated
pose and stops solving. Re-enabling creates fresh state from the current frame.

`Reset Physical Bones` clears solver state, velocities, contact history, and
invalidated cache for all configured physical bones on the active armature. It
restores only those bones to the current frame's pre-physics animated pose. It
does not alter non-physical bones and does not force Blender's Edit Mode rest
position.

Each physics system may be enabled, paused, or isolated for tuning. A runtime
failure pauses the affected armature world, preserves the last valid Blender
pose, and reports stable body, constraint, and contact IDs.

## Presets

### Location and discovery

Presets use Blender's user scripts preset hierarchy:

```text
<SCRIPTS>/presets/bone_physics/bone_parameters/*.json
<SCRIPTS>/presets/bone_physics/armatures/*.json
```

The implementation obtains writable directories with
`bpy.utils.user_resource("SCRIPTS", path=..., create=True)` and discovers all
registered preset roots with `bpy.utils.preset_paths(subdir)`. JSON lookup may
use `bpy.utils.preset_find(..., ext=".json")`.

Packaged presets are read-only. Users may copy them into the user preset root.

### Bone-parameter presets

A bone-parameter preset is one named value bundle. It has no head/tail identity
and contains no bone, object, or stable-ID binding. It may contain Motion,
Local Constraint, and Collision categories.

Users may apply selected categories to:

- heads or tails of selected chains;
- selected physical bones;
- every bone in one chain;
- every bone in selected chains.

Applying a preset copies values. It does not create a dynamic link to the JSON
file. Gradient curves are not part of this preset kind.

### Exact-armature presets

An exact-armature preset contains:

- schema metadata and global settings;
- all physics systems and enable states;
- exact bone paths, UUIDs, stable IDs, endpoint fields, curves, and overrides;
- root targets, local constraints, and structural constraints;
- physical-capsule policies and system collision matrix;
- collision resources, collections, mesh ownership, and bindings;
- armature topology and rest-transform fingerprints.

It excludes runtime position, orientation, velocity, multipliers, contacts,
cache data, and baked output.

Loading validates the complete file against the active armature before
mutation. Missing bones, changed paths, mismatched rest topology, missing
external objects, illegal values, duplicate IDs, or unsupported schema reject
the whole operation. Partial exact-preset application is forbidden.

Successful application atomically replaces configuration, invalidates affected
cache, rebuilds the world, and produces one Blender Undo step.

### Preset management

Both preset kinds support create, rename, overwrite, copy, and delete.

- Name conflicts block create.
- Overwrite opens a confirmation dialog showing display name and path.
- Delete opens a small confirmation dialog showing display name and full path.
- Built-in presets cannot be overwritten, renamed, or deleted.
- Writes use a temporary file and atomic replacement.
- Unknown newer schemas are rejected; supported older schemas use explicit
  migration functions.
- Invalid JSON never modifies the current `.blend` configuration.

The `.blend` file always stores the current complete authoring configuration.
External JSON files exist only for reuse and exchange.

## Persistence and module boundaries

### RNA data

The implementation attaches a root PointerProperty to Armature Objects:

```text
Object.bone_physics
|-- global_settings
|-- physics_systems[]
|-- collision_resources[]
|-- collision_collections[]
|-- stable_id_state
`-- schema_version
```

Nested PropertyGroups and CollectionProperties store authoring data in the
`.blend`. Classes register before their dependent pointer and collection
properties and unregister in reverse order.

### Blender modules

- `properties.py`: RNA PropertyGroups and stable enum identifiers.
- `operators/`: system, chain, constraint, collision, preset, reset, and bake
  operations.
- `ui/`: N-panel panels, menus, and UILists.
- `authoring_adapter.py`: RNA-to-immutable-model conversion.
- `preset_io.py`: JSON schemas, validation, migration, and atomic I/O.
- `validation.py`: stable authoring error codes and locations.
- `viewport.py`: capsules, constraint links, colliders, and problem overlays.
- `translations.py`: English source strings and Simplified Chinese messages.
- `runtime_registry.py`: scene and Armature Object runtime-world ownership.

Blender-independent conversion and validation remain importable under ordinary
CPython. `bone_physics_core` remains independent of Python and Blender.

### Dirty state and rebuilds

RNA update callbacks may run in threaded contexts. They only enqueue a cheap
dirty record and never call Blender scene APIs or native solver methods.
Collection mutations run through operators because CollectionProperty has no
update callback.

The main-thread queue coalesces scalar changes for roughly 200 ms. When idle it
rebuilds the affected system; during playback it applies changes at a guarded
frame boundary. Bone paths, topology, cross-links, and mesh bindings require
explicit `Validate and Rebuild`.

Rebuild flow:

1. snapshot current valid authoring and runtime state;
2. validate dirty authoring data;
3. convert RNA into immutable WorldSpec data;
4. sample parameter curves into concrete values;
5. compile deterministic native batches;
6. create and validate a replacement world;
7. swap worlds only after success;
8. retain the prior world and report errors on failure.

## Internationalization

English is the stable source language. Simplified Chinese translations register
through `bpy.app.translations.register()` and follow Blender's Interface
Language setting. The extension has no separate language preference.

All labels, tooltips, menus, confirmation dialogs, operator reports, validation
messages, runtime errors, and preset actions are translatable. Internal enum
identifiers, JSON keys, schema names, and error codes are never localized.
User-provided names and descriptions are preserved as UTF-8. Missing messages
fall back to English.

Translation contexts distinguish ambiguous physics terms. Automated checks
require complete English and Simplified Chinese coverage for all critical
actions and errors.

## Validation and failure handling

### Blocking errors

- overlapping chains or duplicate physical bones;
- missing bones, paths, targets, objects, or stable IDs;
- zero-length bones and non-finite or illegal parameter values;
- negative mass, radius, margin, compliance, or inverted ranges;
- all-rigid constraint cycles;
- exhausted collision layers;
- mesh surfaces with no valid triangles;
- exact-preset topology or binding mismatch.

### Warnings

- initial penetration;
- open, non-manifold, inconsistent, or degenerate mesh surfaces;
- suspicious capsule radius relative to bone length;
- roots without adequate inset or exclusion;
- highly overconstrained or extremely stiff systems;
- systems with no external collision resource;
- excessive self-collision candidate counts.

Every issue has a stable code, severity, entity location, English source
message, and localized presentation. Clicking it navigates to the owner.

All authoring changes, preset application, and rebuilds are transactional.
Native divergence rolls back to the latest valid state and pauses the affected
world. Invalid poses are never written into Blender.

## Backend authoring extensions

The current code requires these scoped extensions:

- expand ChainSpec fields beyond gravity, linear damping, angular damping, and
  radius so every approved motion, recovery, limit, and collision field can use
  root-to-tip sampling;
- model physical-root attachment to an animated non-physical parent or other
  kinematic target;
- compile swing-twist authoring data;
- compile physics-system collision policy into stable layers and masks;
- add complete RNA registration and authoring conversion;
- add per-system pause, isolation, rebuild, and diagnostic mapping.

These changes do not authorize new solver semantics. They expose approved
capabilities through validated public authoring data. Any native batch change
requires an explicit API-version migration and compatibility tests.

## Testing and acceptance

Every behavior change follows RED, GREEN, then refactor.

### Pure Python tests

- child traversal independent of Blender's `connected` flag;
- branch leaf selection and unique head-tail path resolution;
- multi-chain creation and overlap rejection;
- accumulated-rest-length sampling for every parameter category;
- joint-boundary sampling and per-bone override precedence;
- physical-root kinematic targets;
- swing-twist and structural-constraint compilation;
- collision collection and matrix compilation;
- stable-ID persistence and duplicate repair errors;
- JSON round-trip, migration, atomic overwrite, rename, and deletion;
- strict exact-preset matching and zero mutation on failure;
- English and Simplified Chinese critical-message coverage.

### Blender MCP and headless tests

- register, unregister, and reload the extension;
- N-panel polling and Setup/Simulate/Diagnostics workspaces;
- PropertyGroup persistence through save, close, and reopen;
- multi-selected Pose Bone chain creation;
- Undo and Redo for authoring operations;
- user preset directory and JSON discovery;
- overwrite and delete confirmation dialogs;
- English and Simplified Chinese presentation;
- viewport collider and constraint visualization and selection sync;
- reset affects only configured physical bones;
- enable, disable, pause, isolation, rebuild, preview, and bake behavior.

### Complete gate

Before completion claims:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
python -m unittest discover tests/python -v
```

Batch API migrations also run compatibility and invalid-version tests.
Cross-platform Blender CI remains authoritative for release evidence.

### Delivery stages

1. RNA data, extension registration, and N-panel shell.
2. Physics systems, chain discovery, full parameter fields, and overrides.
3. Local and structural constraints, collision resources, and filtering.
4. Runtime controls, reset, isolation, diagnostics, and viewport overlays.
5. JSON presets, English and Simplified Chinese translations.
6. Blender integration, persistence, complete verification, and packaging.

Each stage must leave the repository testable. No stage weakens physics
acceptance thresholds or distribution boundaries.
