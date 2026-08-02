# Blender XPBD Bone Physics UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a production Blender 5.2 N-panel authoring UI for multi-chain XPBD physics systems, constraints, collision, runtime control, diagnostics, JSON presets, and English/Simplified Chinese presentation.

**Architecture:** Store authoring state as PropertyGroups on each Armature Object, convert it through Blender-independent validation and compilation into the existing immutable WorldSpec/native batches, and swap runtime worlds transactionally. Keep generated bodies, constraints, and colliders inspectable but subordinate to physics systems; keep presets and translations independent of solver semantics.

**Tech Stack:** Blender 5.2 Python API (`bpy`, PropertyGroup, Panel, UIList, Operator, `bpy.app.translations`), Python standard library (`dataclasses`, `enum`, `json`, `pathlib`, `tempfile`, `unittest`), existing PyO3 batch bridge and Rust XPBD core.

## Global Constraints

- Read `docs/HANDOFF.md`, `docs/design/blender-xpbd-bone-physics-design.md`, and `docs/superpowers/specs/2026-08-02-blender-xpbd-bone-physics-ui-design.md` before implementation.
- Target Blender 5.2 LTS; users install no Rust, pip packages, NumPy, or system physics libraries.
- `bone_physics_core` remains Blender/Python independent and retains `#![forbid(unsafe_code)]`.
- Stable IDs alone determine body, constraint, contact, and pair ordering.
- Runtime constraints store sampled concrete values, never authoring curves.
- Persistent constraints and contacts share every XPBD iteration; no final correction or blend pass.
- Collision input comes from pre-physics animation and never the final physics-deformed mesh at the same time.
- Direct structural neighbors may be filtered; never compute transitive collision exclusions.
- Reset affects only configured physical bones on the active Armature Object and restores the current pre-physics animated pose.
- Mass/density mode, primary angular-limit model, Euler order, and interpolation mode are discrete chain settings; their numeric values may vary along the chain. A per-bone override may select a different angular model for that joint.
- All behavior changes follow RED, GREEN, refactor; keep one concern per commit.
- Do not weaken numerical acceptance thresholds.

---

## File map

New focused modules:

- `blender_extension/bone_physics/authoring.py`: Blender-independent authoring records, enums, paths, fingerprints, and validation issues.
- `blender_extension/bone_physics/chain_discovery.py`: child traversal, branch resolution, overlap checks, and stable-ID allocation.
- `blender_extension/bone_physics/authoring_adapter.py`: authoring records and Blender snapshots to immutable WorldSpec.
- `blender_extension/bone_physics/properties.py`: registered RNA PropertyGroups and enum identifiers.
- `blender_extension/bone_physics/operators/system.py`: initialization, physics-system, and chain operators.
- `blender_extension/bone_physics/operators/collision.py`: collision resource and collection operators.
- `blender_extension/bone_physics/operators/runtime.py`: enable, reset, isolate, validate/rebuild, cache, and bake operators.
- `blender_extension/bone_physics/operators/preset.py`: preset menu actions and confirmation dialogs.
- `blender_extension/bone_physics/ui/panels.py`: N-panel workspaces and subpanels.
- `blender_extension/bone_physics/ui/lists.py`: physics-system, chain, collision, override, and diagnostic UILists.
- `blender_extension/bone_physics/runtime_registry.py`: per-scene/per-armature worlds and guarded dirty rebuild queue.
- `blender_extension/bone_physics/preset_io.py`: JSON schemas, discovery, migration, atomic writes, and exact matching.
- `blender_extension/bone_physics/viewport.py`: draw-handler state and selection-safe overlays.
- `blender_extension/bone_physics/translations.py`: English message IDs and Simplified Chinese dictionary.

Existing modules retain their current responsibilities. Extend `model.py`, `parameter_builder.py`, `collider_builder.py`, `prephysics_eval.py`, `scene_adapter.py`, `runtime.py`, and package registration only where the authoring boundary requires it.

### Task 1: General parameter fields and missing constraint compilation

**Files:**
- Modify: `blender_extension/bone_physics/model.py`
- Modify: `blender_extension/bone_physics/parameter_builder.py`
- Modify: `docs/batch-api-v1.md`
- Test: `tests/python/test_model_validation.py`
- Test: `tests/python/test_parameter_builder.py`

**Interfaces:**
- Produces: `ChainField`, `ChainFieldCurve`, `AngularLimitMode`, and concrete `ChainSpec.field_curves`.
- Produces: attachment constraints of native kind `3` and swing-twist constraints of native kind `6` without changing the existing 24-column batch layout.
- Consumes: existing `ParameterCurve`, `BoneSpec`, `WorldSpec`, and native constraint decoder.

- [ ] **Step 1: Write failing model tests for general scalar fields and exclusive angular modes**

```python
def test_chain_field_curves_reject_duplicates():
    curve = ParameterCurve((CurvePoint(0.0, 0.0), CurvePoint(1.0, 1.0)))
    with self.assertRaisesRegex(ValueError, "duplicate chain field"):
        ChainSpec(
            id="hair",
            body_ids=(1,),
            field_curves=(
                ChainFieldCurve(ChainField.GRAVITY_SCALE, curve),
                ChainFieldCurve(ChainField.GRAVITY_SCALE, curve),
            ),
        )

def test_bone_uses_one_primary_angular_limit_model():
    spec = bone(1)
    self.assertEqual(spec.angular_limit_mode, AngularLimitMode.FREE)
    with self.assertRaisesRegex(ValueError, "swing axis"):
        replace(spec, angular_limit_mode=AngularLimitMode.SWING_TWIST,
                swing_axis=(0.0, 0.0, 0.0))
```

- [ ] **Step 2: Run the model tests and record RED**

Run: `python -m unittest tests.python.test_model_validation -v`

Expected: FAIL because `ChainField`, `ChainFieldCurve`, and `AngularLimitMode` do not exist.

- [ ] **Step 3: Add validated authoring enums and general field curves**

```python
class AngularLimitMode(IntEnum):
    FREE = 0
    XYZ = 1
    SWING_TWIST = 2

class ChainField(IntEnum):
    INVERSE_MASS = 0
    INERTIA_SCALE_X = 1
    INERTIA_SCALE_Y = 2
    INERTIA_SCALE_Z = 3
    GRAVITY_SCALE = 4
    TRANSLATION_INERTIA = 5
    ROTATION_INERTIA = 6
    LINEAR_DAMPING = 7
    ANGULAR_DAMPING = 8
    JOINT_COMPLIANCE = 9
    JOINT_DAMPING = 10
    ANGULAR_COMPLIANCE = 11
    ANGULAR_CONSTRAINT_DAMPING = 12
    RECOVERY_COMPLIANCE = 13
    RECOVERY_DAMPING = 14
    RADIUS = 15
    MARGIN = 16
    FRICTION = 17
    RESTITUTION = 18
    XYZ_LOWER_X = 19
    XYZ_LOWER_Y = 20
    XYZ_LOWER_Z = 21
    XYZ_UPPER_X = 22
    XYZ_UPPER_Y = 23
    XYZ_UPPER_Z = 24
    SWING_AXIS_X = 25
    SWING_AXIS_Y = 26
    SWING_AXIS_Z = 27
    MAX_SWING = 28
    MIN_TWIST = 29
    MAX_TWIST = 30
    CAPSULE_HEAD_INSET = 31
    CAPSULE_TAIL_INSET = 32

@dataclass(frozen=True, slots=True)
class ChainFieldCurve:
    field: ChainField
    curve: ParameterCurve
```

Add `field_curves`, attachment metadata, and validated swing-twist values to `ChainSpec`/`BoneSpec`. Keep old four named curve arguments for one migration release by normalizing them into `field_curves` and rejecting duplicate destinations.

- [ ] **Step 4: Write failing compiler tests for body, joint-boundary, attachment, and swing-twist rows**

```python
def test_general_fields_sample_bodies_and_joint_boundaries(self):
    compiled = compile_world_spec(full_field_world())
    self.assertAlmostEqual(body_value(compiled, 1, 20), 0.25)
    self.assertAlmostEqual(body_value(compiled, 2, 20), 0.75)
    self.assertAlmostEqual(compiled.constraint_row(20)[8], 0.5)

def test_root_attachment_and_swing_twist_use_existing_batch_kinds(self):
    compiled = compile_world_spec(attachment_and_swing_world())
    self.assertEqual(tuple(compiled.constraints["kinds"]), (3, 6))
    self.assertEqual(compiled.constraint_row(22)[14:20],
                     (0.0, 1.0, 0.0, 0.6, -0.4, 0.4))
```

- [ ] **Step 5: Run focused compiler tests and record RED**

Run: `python -m unittest tests.python.test_parameter_builder.ParameterBuilderTests.test_general_fields_sample_bodies_and_joint_boundaries tests.python.test_parameter_builder.ParameterBuilderTests.test_root_attachment_and_swing_twist_use_existing_batch_kinds -v`

Expected: FAIL because general sampling and row generation are absent.

- [ ] **Step 6: Implement field dispatch and constraint rows**

Implement one exhaustive `ChainField` dispatch table. Sample body fields at span midpoint and constraint fields at the child head boundary. Emit attachment kind `3` with `body_b=0`; emit swing-twist kind `6` using columns `14..20`. Reject an unsupported field instead of ignoring it.

```python
def _sample_chain_fields(chain, boundaries, bones):
    replacements = {}
    for index, body_id in enumerate(chain.body_ids):
        midpoint = (boundaries[index] + boundaries[index + 1]) * 0.5
        joint_s = boundaries[index]
        body = bones[body_id]
        for item in chain.field_curves:
            sample_s = joint_s if item.field in JOINT_FIELDS else midpoint
            body = _apply_chain_field(body, item.field, item.curve.sample(sample_s))
        replacements[body_id] = body
    return replacements
```

- [ ] **Step 7: Run the complete Python model/compiler slice**

Run: `python -m unittest tests.python.test_model_validation tests.python.test_parameter_builder tests.python.test_collider_builder -v`

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add blender_extension/bone_physics/model.py blender_extension/bone_physics/parameter_builder.py tests/python/test_model_validation.py tests/python/test_parameter_builder.py docs/batch-api-v1.md
git commit -m "feat(authoring): compile full chain fields"
```

### Task 2: Pure authoring records, validation, and stable IDs

**Files:**
- Create: `blender_extension/bone_physics/authoring.py`
- Create: `tests/python/test_authoring.py`

**Interfaces:**
- Produces: `ArmatureAuthoring`, `PhysicsSystemAuthoring`, `ChainAuthoring`, `EndpointParameters`, `CollisionPolicy`, `ValidationIssue`, `ValidationSeverity`, `validate_authoring()`.
- Produces: `StableIdAllocator.allocate(kind: str, key: str) -> int` with deterministic persisted assignments.
- Consumes: only Python standard library and immutable model enums.

- [ ] **Step 1: Write failing tests for immutable configuration and validation locations**

```python
def test_validation_reports_overlapping_bone_at_both_chains():
    config = armature_with_chains(("a", ("root", "mid")), ("b", ("mid", "tip")))
    issues = validate_authoring(config)
    self.assertEqual(issues[0].code, "chain.overlap")
    self.assertEqual(issues[0].entity_ids, ("a", "b"))

def test_allocator_reuses_persisted_ids_and_never_returns_zero():
    allocator = StableIdAllocator((("body:uuid-a", 7),), next_id=8)
    self.assertEqual(allocator.allocate("body", "uuid-a"), 7)
    self.assertEqual(allocator.allocate("body", "uuid-b"), 8)
```

- [ ] **Step 2: Run the test and record RED**

Run: `python -m unittest tests.python.test_authoring -v`

Expected: FAIL because `authoring.py` does not exist.

- [ ] **Step 3: Implement focused frozen records and validator**

```python
class ValidationSeverity(str, Enum):
    ERROR = "error"
    WARNING = "warning"

@dataclass(frozen=True, slots=True)
class ValidationIssue:
    code: str
    severity: ValidationSeverity
    entity_ids: tuple[str, ...]
    arguments: tuple[tuple[str, object], ...] = ()

def validate_authoring(config: ArmatureAuthoring) -> tuple[ValidationIssue, ...]:
    issues = list(_chain_issues(config))
    issues.extend(_constraint_issues(config))
    issues.extend(_collision_issues(config))
    return tuple(sorted(issues, key=lambda issue: (issue.severity.value, issue.code, issue.entity_ids)))
```

Cover duplicate UUID/stable ID, overlap, missing target/resource, illegal scalar/range, all-rigid cycles, layer exhaustion, and warnings named in the spec. Keep text out of issues; store stable codes and arguments.

- [ ] **Step 4: Run authoring tests**

Run: `python -m unittest tests.python.test_authoring -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add blender_extension/bone_physics/authoring.py tests/python/test_authoring.py
git commit -m "feat(authoring): add validated config model"
```

### Task 3: Chain discovery and branch selection

**Files:**
- Create: `blender_extension/bone_physics/chain_discovery.py`
- Create: `tests/python/test_chain_discovery.py`

**Interfaces:**
- Consumes: `BoneNode(uuid: str, name: str, parent_uuid: str | None, length: float)` snapshots.
- Produces: `descendant_leaves(nodes, head_uuid) -> tuple[str, ...]` and `resolve_chain(nodes, head_uuid, tail_uuid=None) -> tuple[str, ...]`.
- Produces: `validate_new_paths(existing_paths, new_paths) -> None`.

- [ ] **Step 1: Write failing tests for disconnected children, branches, and overlap**

```python
def test_resolve_chain_ignores_connected_flag_by_using_parent_links():
    nodes = skeleton(("root", None), ("mid", "root"), ("tip", "mid"))
    self.assertEqual(resolve_chain(nodes, "root"), ("root", "mid", "tip"))

def test_branch_requires_explicit_tail():
    nodes = skeleton(("root", None), ("left", "root"), ("right", "root"))
    with self.assertRaisesRegex(BranchSelectionRequired, "left.*right"):
        resolve_chain(nodes, "root")
    self.assertEqual(resolve_chain(nodes, "root", "right"), ("root", "right"))
```

- [ ] **Step 2: Run and record RED**

Run: `python -m unittest tests.python.test_chain_discovery -v`

Expected: FAIL because chain discovery is absent.

- [ ] **Step 3: Implement parent-link traversal with stable sorting**

Build children from `parent_uuid`, sort children and leaves by `(name, uuid)`, detect cycles, reject missing heads/tails, and require the explicit tail to be a descendant leaf. Ignore Blender's connected flag entirely.

```python
def resolve_chain(nodes, head_uuid, tail_uuid=None):
    by_id = {node.uuid: node for node in nodes}
    children = _children_by_parent(nodes)
    leaves = descendant_leaves(nodes, head_uuid)
    if tail_uuid is None and len(leaves) != 1:
        raise BranchSelectionRequired(head_uuid, leaves)
    selected = leaves[0] if tail_uuid is None else tail_uuid
    if selected not in leaves:
        raise ValueError("tail must be a descendant leaf")
    return _parent_path(by_id, head_uuid, selected)
```

- [ ] **Step 4: Run tests**

Run: `python -m unittest tests.python.test_chain_discovery -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add blender_extension/bone_physics/chain_discovery.py tests/python/test_chain_discovery.py
git commit -m "feat(authoring): discover hierarchical bone chains"
```

### Task 4: RNA properties and extension lifecycle

**Files:**
- Create: `blender_extension/bone_physics/properties.py`
- Modify: `blender_extension/bone_physics/__init__.py`
- Create: `tests/blender/test_ui_registration.py`
- Modify: `scripts/ci/run_blender_tests.py`

**Interfaces:**
- Produces: `bpy.types.Object.bone_physics` PointerProperty for Armature Objects.
- Produces: PropertyGroups mirroring authoring records, with stable enum identifiers and `schema_version=1`.
- Consumes: `runtime_registry.mark_dirty(owner_pointer, category)`; scalar callbacks never invoke Blender/native operations.

- [ ] **Step 1: Write Blender registration/persistence test**

```python
import bpy
import bone_physics

bone_physics.unregister()
bone_physics.register()
armature_data = bpy.data.armatures.new("UIArmature")
armature = bpy.data.objects.new("UIArmature", armature_data)
bpy.context.scene.collection.objects.link(armature)
assert hasattr(armature, "bone_physics")
armature.bone_physics.schema_version = 1
assert armature.bone_physics.schema_version == 1
bone_physics.unregister()
assert not hasattr(bpy.types.Object, "bone_physics")
bone_physics.register()
```

- [ ] **Step 2: Run through Blender MCP and record RED**

Run the new script with Blender MCP background execution.

Expected: FAIL because lifecycle functions do not register RNA properties.

- [ ] **Step 3: Implement PropertyGroups and dependency-ordered registration**

Register leaf groups first, then groups containing CollectionProperty/PointerProperty, then attach `Object.bone_physics`. Delete the Object property first during unregister and unregister classes in reverse order. Add/remove/reorder collections only through operators.

```python
CLASSES = (CurvePointProperties, EndpointProperties, ChainProperties,
           PhysicsSystemProperties, ArmaturePhysicsProperties)

def register_properties():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Object.bone_physics = bpy.props.PointerProperty(
        type=ArmaturePhysicsProperties)

def unregister_properties():
    del bpy.types.Object.bone_physics
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
```

- [ ] **Step 4: Add the script to the bounded Blender CI tuple and rerun**

Run via Blender MCP; then run: `python -m unittest tests.python.test_packaging -v`

Expected: both PASS.

- [ ] **Step 5: Commit**

```bash
git add blender_extension/bone_physics/properties.py blender_extension/bone_physics/__init__.py tests/blender/test_ui_registration.py scripts/ci/run_blender_tests.py
git commit -m "feat(blender): register armature authoring data"
```

### Task 5: Physics-system operators and N-panel shell

**Files:**
- Create: `blender_extension/bone_physics/operators/__init__.py`
- Create: `blender_extension/bone_physics/operators/system.py`
- Create: `blender_extension/bone_physics/ui/__init__.py`
- Create: `blender_extension/bone_physics/ui/lists.py`
- Create: `blender_extension/bone_physics/ui/panels.py`
- Modify: `blender_extension/bone_physics/__init__.py`
- Create: `tests/blender/test_ui_systems.py`

**Interfaces:**
- Produces operators `bone_physics.initialize`, `bone_physics.system_add`, `bone_physics.system_duplicate`, `bone_physics.system_remove`, and `bone_physics.chain_add_selected`; `system_add.template` supports `LOOSE_HAIR`, `EARS`, `PONYTAIL`, `SKIRT`, `ACCESSORY`, and `BLANK`.
- Produces N-panel category `Bone Physics` and workspace enum `SETUP`, `SIMULATE`, `DIAGNOSTICS`.
- Consumes chain discovery and registered Object properties.

- [ ] **Step 1: Write failing Blender operator test**

```python
select_pose_bones(armature, ("hair_root.L", "hair_root.R"))
assert bpy.ops.bone_physics.initialize() == {'FINISHED'}
assert bpy.ops.bone_physics.system_add(name="Hair") == {'FINISHED'}
assert bpy.ops.bone_physics.chain_add_selected() == {'FINISHED'}
settings = armature.bone_physics
assert settings.physics_systems[0].name == "Hair"
assert len(settings.physics_systems[0].chains) == 2
```

- [ ] **Step 2: Run with Blender MCP and record RED**

Expected: FAIL because operators are not registered.

- [ ] **Step 3: Implement context-safe operators**

Each operator `poll()` requires an active Armature Object; chain creation additionally requires Pose Mode and selected pose bones. Capture selection before any popup. If branch selection is needed, populate a stable EnumProperty and open `invoke_search_popup`; re-check object identity in `execute()`.

```python
class BONE_PHYSICS_OT_system_add(bpy.types.Operator):
    bl_idname = "bone_physics.system_add"
    bl_options = {'REGISTER', 'UNDO'}
    template: bpy.props.EnumProperty(items=SYSTEM_TEMPLATES)
    name: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == 'ARMATURE'

    def execute(self, context):
        item = context.object.bone_physics.physics_systems.add()
        populate_system_template(item, self.template, self.name)
        return {'FINISHED'}
```

- [ ] **Step 4: Implement panel and UILists**

Use `bl_space_type='VIEW_3D'`, `bl_region_type='UI'`, and `bl_category='Bone Physics'`. Draw fixed header, workspace selector, system list, counts, and explicit empty states. Do not draw every generated bone or constraint in primary lists.

```python
class BONE_PHYSICS_PT_root(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Bone Physics'
    bl_label = "Bone Physics"

    @classmethod
    def poll(cls, context):
        return context.object is not None and context.object.type == 'ARMATURE'

    def draw(self, context):
        settings = context.object.bone_physics
        self.layout.prop(settings, "global_enabled")
        self.layout.prop(settings, "workspace", expand=True)
```

- [ ] **Step 5: Run Blender test and registration test**

Run both scripts with Blender MCP background execution.

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add blender_extension/bone_physics/operators blender_extension/bone_physics/ui blender_extension/bone_physics/__init__.py tests/blender/test_ui_systems.py
git commit -m "feat(ui): add physics system workspace"
```

### Task 6: Authoring adapter, full gradients, and per-bone overrides

**Files:**
- Create: `blender_extension/bone_physics/authoring_adapter.py`
- Modify: `blender_extension/bone_physics/prephysics_eval.py`
- Modify: `blender_extension/bone_physics/scene_adapter.py`
- Create: `tests/python/test_authoring_adapter.py`
- Modify: `tests/blender/test_prephysics_order.py`

**Interfaces:**
- Produces: `compile_authoring(config, skeleton, collision_snapshot) -> WorldSpec`.
- Produces: `TargetBinding(kind, name, local_anchor)` for own animation, non-physical bone, object, or world.
- Consumes: authoring records, chain field curves, stable IDs, and existing pre-physics evaluator.

- [ ] **Step 1: Write failing pure tests for endpoint interpolation and override precedence**

```python
def test_compile_authoring_samples_all_fields_then_applies_override():
    world = compile_authoring(hair_config(), uneven_skeleton(), empty_collision())
    bones = {bone.name: bone for bone in world.bones}
    self.assertAlmostEqual(bones["hair_mid"].gravity_scale, 0.5)
    self.assertAlmostEqual(bones["hair_tip"].gravity_scale, 0.25)  # override
    self.assertEqual(bones["hair_tip"].angular_limit_mode, AngularLimitMode.XYZ)
```

- [ ] **Step 2: Run and record RED**

Run: `python -m unittest tests.python.test_authoring_adapter -v`

Expected: FAIL because the adapter does not exist.

- [ ] **Step 3: Implement deterministic authoring conversion**

Convert mass or density to finite positive inverse mass and local inverse inertia using rest capsule dimensions. Convert endpoints plus shared/per-field Hermite controls into ChainFieldCurve entries. Apply per-bone overrides after sampling. Sort systems, chains, bones, constraints, and collision resources by persisted stable ID.

```python
def compile_authoring(config, skeleton, collision_snapshot):
    allocator = StableIdAllocator(config.stable_ids, config.next_stable_id)
    bones = _compile_bones(config, skeleton, allocator)
    chains = _compile_chains(config, skeleton, allocator)
    bones = _apply_overrides(bones, config.physics_systems)
    return WorldSpec(
        bones=tuple(sorted(bones, key=lambda item: item.id)),
        chains=tuple(sorted(chains, key=lambda item: item.id)),
        cross_links=_compile_cross_links(config, allocator),
        analytic_colliders=_compile_analytic(config, collision_snapshot, allocator),
        collision_surfaces=_compile_surfaces(config, allocator),
        gravity=config.gravity,
        fixed_dt=config.fixed_dt,
    )
```

- [ ] **Step 4: Write failing Blender test for a non-physical root target**

Add one animated control bone parenting a physical root. Assert the captured animation target for the physical root follows the control's pre-physics pose and does not create a body for the control.

- [ ] **Step 5: Extend pre-physics target capture**

Resolve `TargetBinding` in `BlenderSceneAdapter.target_pose()`. Compute target poses in simulation space from the selected bone, object, or fixed world transform. Preserve the ownership order: restore animation references, evaluate, capture targets/colliders, solve, write physical deltas.

- [ ] **Step 6: Run focused Python and Blender tests**

Run: `python -m unittest tests.python.test_authoring_adapter tests.python.test_parameter_builder -v`

Run the updated pre-physics test through Blender MCP.

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add blender_extension/bone_physics/authoring_adapter.py blender_extension/bone_physics/prephysics_eval.py blender_extension/bone_physics/scene_adapter.py tests/python/test_authoring_adapter.py tests/blender/test_prephysics_order.py
git commit -m "feat(authoring): compile armature physics systems"
```

### Task 7: Parameter, constraint, and override UI

**Files:**
- Modify: `blender_extension/bone_physics/properties.py`
- Modify: `blender_extension/bone_physics/operators/system.py`
- Modify: `blender_extension/bone_physics/ui/panels.py`
- Modify: `blender_extension/bone_physics/ui/lists.py`
- Create: `tests/blender/test_ui_parameters.py`

**Interfaces:**
- Produces: matching head/tail parameter groups, shared interpolation, per-field curve points, category masks, mixed-value batch apply, and override operators.
- Consumes: authoring adapter field names exactly; enum identifiers remain English and stable.

- [ ] **Step 1: Write Blender test for multi-chain endpoint application and override creation**

```python
settings.physics_systems[0].selected = True
settings.physics_systems[1].selected = True
assert bpy.ops.bone_physics.apply_endpoint_value(
    endpoint='TAIL', field='GRAVITY_SCALE', value=0.35) == {'FINISHED'}
assert all(system.tail.gravity_scale == 0.35 for system in selected_systems(settings))
select_pose_bones(armature, ("hair_tip.L",))
assert bpy.ops.bone_physics.override_add_selected() == {'FINISHED'}
assert settings.physics_systems[0].overrides[0].bone_name == "hair_tip.L"
```

- [ ] **Step 2: Run with Blender MCP and record RED**

Expected: FAIL because endpoint and override operators are absent.

- [ ] **Step 3: Implement RNA groups and batch operators**

Add explicit fields for Motion, Recovery, Angular Limits, Collision, and Advanced groups. Add category-mask BoolProperties and operators that mutate all selected targets in one Undo step. Show mixed values without writing until the user edits a field. Per-field curve editing stores strictly increasing normalized points.

```python
def apply_endpoint_value(settings, endpoint, field, value):
    targets = [system for system in settings.physics_systems if system.selected]
    if not targets:
        raise ValueError("select at least one physics system")
    for system in targets:
        values = system.head if endpoint == 'HEAD' else system.tail
        setattr(values, field.lower(), value)
```

- [ ] **Step 4: Draw compact parameter panels**

Draw head and tail in parallel subpanels, shared gradient controls below, then collapsed Motion/Recovery/Limits/Collision/Advanced groups. Keep `Per-Bone Overrides` collapsed and list only overridden bones.

- [ ] **Step 5: Run Blender UI tests and pure adapter tests**

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add blender_extension/bone_physics/properties.py blender_extension/bone_physics/operators/system.py blender_extension/bone_physics/ui tests/blender/test_ui_parameters.py
git commit -m "feat(ui): edit graded chain parameters"
```

### Task 8: Structural constraints and read-only generated inspector

**Files:**
- Modify: `blender_extension/bone_physics/properties.py`
- Create: `blender_extension/bone_physics/operators/constraint.py`
- Modify: `blender_extension/bone_physics/operators/__init__.py`
- Modify: `blender_extension/bone_physics/ui/panels.py`
- Modify: `blender_extension/bone_physics/ui/lists.py`
- Create: `tests/blender/test_ui_constraints.py`

**Interfaces:**
- Produces operators to add/remove horizontal, diagonal, and seam structural groups.
- Produces read-only generated rows from `NativeBuildBatch.constraints` and diagnostics.
- Consumes existing `CrossLinkSpec` and deterministic normalized-length matching.

- [ ] **Step 1: Write failing Blender skirt-lattice test**

```python
select_system_chains(settings, ("skirt_0", "skirt_1", "skirt_2", "skirt_3"))
result = bpy.ops.bone_physics.structural_add(
    samples="0.0,0.5,1.0", diagonals=True, closed=True,
    compliance=1.0e-5)
assert result == {'FINISHED'}
compiled = compile_active_armature(bpy.context)
assert len(generated_cross_links(compiled)) == 28
```

- [ ] **Step 2: Run with Blender MCP and record RED**

Expected: FAIL because structural operators are absent.

- [ ] **Step 3: Implement structural operators and validation**

Parse samples into finite strictly increasing values in `[0,1]`; require at least two chains and three for closed topology; reject zero compliance when links form a cycle. Re-resolve chain IDs after edits and retain no dangling links.

```python
def parse_structural_samples(text):
    values = tuple(float(part.strip()) for part in text.split(","))
    if not values or any(not math.isfinite(value) or not 0.0 <= value <= 1.0
                         for value in values):
        raise ValueError("samples must be finite and in [0, 1]")
    if any(left >= right for left, right in zip(values, values[1:])):
        raise ValueError("samples must be strictly increasing")
    return values
```

- [ ] **Step 4: Draw structural groups and generated inspector**

Show group topology and aggregate counts in normal UI. Advanced inspector rows are read-only and include stable constraint ID, kind, connected bodies, sampled compliance, and current error.

- [ ] **Step 5: Run Blender test and existing cross-link tests**

Run: `python -m unittest tests.python.test_parameter_builder -v`

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add blender_extension/bone_physics/properties.py blender_extension/bone_physics/operators/constraint.py blender_extension/bone_physics/operators/__init__.py blender_extension/bone_physics/ui tests/blender/test_ui_constraints.py
git commit -m "feat(ui): author structural constraints"
```

### Task 9: Collision resources, collections, and system matrix

**Files:**
- Modify: `blender_extension/bone_physics/authoring.py`
- Modify: `blender_extension/bone_physics/authoring_adapter.py`
- Modify: `blender_extension/bone_physics/collider_builder.py`
- Create: `blender_extension/bone_physics/operators/collision.py`
- Modify: `blender_extension/bone_physics/operators/__init__.py`
- Modify: `blender_extension/bone_physics/properties.py`
- Modify: `blender_extension/bone_physics/ui/panels.py`
- Modify: `blender_extension/bone_physics/ui/lists.py`
- Create: `tests/python/test_collision_authoring.py`
- Create: `tests/blender/test_ui_collision.py`

**Interfaces:**
- Produces analytic resources bound to non-physical bone, object, or world; mesh resources; reusable collision collections; and system collision policies.
- Produces `compile_collision_matrix(systems) -> dict[str, tuple[int, int]]` containing stable layer and mask.
- Consumes existing `AnalyticColliderSpec`, `CollisionSurfaceSpec`, ownership filters, and pre-physics mesh pipeline.

- [ ] **Step 1: Write failing pure matrix and ownership tests**

```python
def test_named_collision_matrix_is_stable_and_symmetric_when_requested():
    matrix = compile_collision_matrix((hair_system(), skirt_system()))
    self.assertEqual(matrix["hair"][0], 1)
    self.assertEqual(matrix["skirt"][0], 2)
    self.assertTrue(matrix["hair"][1] & 2)
    self.assertTrue(matrix["skirt"][1] & 1)

def test_more_than_32_collision_layers_is_an_error():
    with self.assertRaisesRegex(ValueError, "32 collision layers"):
        compile_collision_matrix(tuple(system(str(i)) for i in range(33)))
```

- [ ] **Step 2: Run and record RED**

Run: `python -m unittest tests.python.test_collision_authoring -v`

Expected: FAIL because system-level collision compilation is absent.

- [ ] **Step 3: Implement deterministic collision compilation**

Assign bits by stable system ID, compile chain/self/cross-system policies into body layers/masks, add only direct structural exclusions, and apply explicit exclusions last. Compile kinematic resources and mesh ownership without exposing Parry types.

```python
def compile_collision_matrix(systems):
    ordered = tuple(sorted(systems, key=lambda item: item.stable_id))
    if len(ordered) > 32:
        raise ValueError("physics systems exceed 32 collision layers")
    layers = {system.id: 1 << index for index, system in enumerate(ordered)}
    return {
        system.id: (layers[system.id], _mask_for_system(system, layers))
        for system in ordered
    }
```

- [ ] **Step 4: Write failing Blender resource workflow test**

Create a sphere bound to a non-physical head bone, one torso mesh surface, and a `HeadAndTorso` collection. Assign it to Hair and assert the adapter produces one analytic collider, one collision surface, and correct body filters.

- [ ] **Step 5: Implement resource/collection operators and UI**

Provide create, duplicate, bind, mirror, enable, show, assign, and guarded delete. Reject deletion while referenced and report referencing systems. Generated physical capsules appear only under chains. Mesh UI exposes thickness, simplification, material/region exclusions, body-region ignores, and weight merges.

- [ ] **Step 6: Run collision slices**

Run: `python -m unittest tests.python.test_collision_authoring tests.python.test_collider_builder tests.python.test_mesh_builder tests.python.test_ownership -v`

Run the Blender collision UI test with Blender MCP.

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add blender_extension/bone_physics/authoring.py blender_extension/bone_physics/authoring_adapter.py blender_extension/bone_physics/collider_builder.py blender_extension/bone_physics/operators/collision.py blender_extension/bone_physics/operators/__init__.py blender_extension/bone_physics/properties.py blender_extension/bone_physics/ui tests/python/test_collision_authoring.py tests/blender/test_ui_collision.py
git commit -m "feat(collision): author reusable character colliders"
```

### Task 10: Runtime registry, dirty rebuilds, enable, isolation, and reset

**Files:**
- Create: `blender_extension/bone_physics/runtime_registry.py`
- Create: `blender_extension/bone_physics/operators/runtime.py`
- Modify: `blender_extension/bone_physics/operators/__init__.py`
- Modify: `blender_extension/bone_physics/runtime.py`
- Modify: `blender_extension/bone_physics/ui/panels.py`
- Create: `tests/python/test_runtime_registry.py`
- Create: `tests/blender/test_ui_runtime.py`

**Interfaces:**
- Produces: `RuntimeRegistry.world(scene, armature)`, `mark_dirty(owner_pointer, category)`, `flush_dirty()`, `reset_configured_bones()`, and transactional `rebuild()`.
- Consumes authoring adapter, existing RuntimeWorld, NativeRuntimeBridge, PoseWriter, TimelineState, and cache.

- [ ] **Step 1: Write failing registry transaction test**

```python
def test_failed_rebuild_keeps_previous_valid_world():
    registry = RuntimeRegistry(factory=Factory(handles=[10, ValueError("bad config")]))
    registry.rebuild(scene_id=1, armature_id=2, spec=valid_spec())
    with self.assertRaisesRegex(ValueError, "bad config"):
        registry.rebuild(scene_id=1, armature_id=2, spec=invalid_spec())
    self.assertEqual(registry.world_by_key((1, 2)).handle, 10)
```

- [ ] **Step 2: Run and record RED**

Run: `python -m unittest tests.python.test_runtime_registry -v`

Expected: FAIL because the registry does not exist.

- [ ] **Step 3: Implement guarded registry and main-thread dirty queue**

Key worlds by scene and Armature Object session pointers. Scalar RNA callbacks only add a `(pointer, category)` record protected by a lock. A Blender timer coalesces records for 0.2 seconds and performs Blender/native work on the main thread. Structural categories remain pending until explicit validation. Create replacement worlds before destroying old handles.

```python
def rebuild(self, scene, armature, spec):
    key = (scene.as_pointer(), armature.as_pointer())
    replacement = self._factory.create(scene, armature, spec)
    previous = self._worlds.get(key)
    self._worlds[key] = replacement
    if previous is not None:
        previous.destroy()
    return replacement

def mark_dirty(owner_pointer, category):
    with _DIRTY_LOCK:
        _DIRTY.add((owner_pointer, category, time.monotonic()))
```

- [ ] **Step 4: Write failing Blender reset-scope test**

Animate one configured physical bone, one configured non-physical parent, and one unrelated pose bone. Advance physics, call reset, and assert only the physical bone returns to its current pre-physics animation channel while other channels remain byte-equivalent.

- [ ] **Step 5: Implement runtime operators**

Add global enable, system enable, pause/resume, isolate, validate/rebuild, and reset. Disable and reset first restore animation references under the existing re-entry guard, then destroy or replace native state. Re-enable starts from the observed current frame.

```python
def reset_configured_bones(context):
    armature = context.object
    entry = REGISTRY.require(context.scene, armature)
    references = entry.evaluator.capture_animation_references(
        context.scene, context.evaluated_depsgraph_get(), entry.world_spec)
    entry.adapter.restore_animation_references(context.scene, references)
    entry.replace_with_fresh_world()
    entry.cache.invalidate("physics reset")
```

- [ ] **Step 6: Run runtime tests**

Run: `python -m unittest tests.python.test_runtime tests.python.test_runtime_registry tests.python.test_timeline -v`

Run the Blender runtime UI script with Blender MCP.

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add blender_extension/bone_physics/runtime_registry.py blender_extension/bone_physics/operators/runtime.py blender_extension/bone_physics/operators/__init__.py blender_extension/bone_physics/runtime.py blender_extension/bone_physics/ui/panels.py tests/python/test_runtime_registry.py tests/blender/test_ui_runtime.py
git commit -m "feat(runtime): manage armature physics worlds"
```

### Task 11: Diagnostics navigation and viewport overlays

**Files:**
- Modify: `blender_extension/bone_physics/diagnostics.py`
- Create: `blender_extension/bone_physics/viewport.py`
- Create: `blender_extension/bone_physics/operators/diagnostics.py`
- Modify: `blender_extension/bone_physics/operators/__init__.py`
- Modify: `blender_extension/bone_physics/ui/panels.py`
- Modify: `blender_extension/bone_physics/ui/lists.py`
- Create: `tests/python/test_authoring_diagnostics.py`
- Create: `tests/blender/test_ui_diagnostics.py`

**Interfaces:**
- Produces merged authoring/runtime `DiagnosticItem(code, severity, entity_kind, entity_id, arguments)` records.
- Produces safe draw-handler registration and `bone_physics.diagnostic_select`.
- Consumes stable IDs and Blender object/bone bindings.

- [ ] **Step 1: Write failing diagnostic mapping test**

```python
def test_native_body_error_maps_back_to_chain_and_bone():
    item = map_runtime_error(authoring_config(), {"body_id": 17, "penetration": 0.02})
    self.assertEqual(item.entity_kind, "bone")
    self.assertEqual(item.entity_id, "hair_tip_uuid")
    self.assertEqual(dict(item.arguments)["system_id"], "hair")
```

- [ ] **Step 2: Run and record RED**

Run: `python -m unittest tests.python.test_authoring_diagnostics -v`

Expected: FAIL because authoring/runtime mapping is absent.

- [ ] **Step 3: Implement stable diagnostic merge and navigation**

Sort errors before warnings and then by stable IDs. Map body, constraint, collider, mesh, and region IDs back to authoring entities. Selection operators switch to the required object/mode only when safe and report hidden/unselectable targets instead of mutating visibility.

```python
def map_runtime_error(config, values):
    body_id = int(values["body_id"])
    owner = config.body_owner(body_id)
    return DiagnosticItem(
        code="runtime.body_failure",
        severity="error",
        entity_kind="bone",
        entity_id=owner.bone_uuid,
        arguments=(("system_id", owner.system_id), ("body_id", body_id)),
    )
```

- [ ] **Step 4: Implement bounded viewport overlays**

Register one draw handler per 3D View area through explicit enable/disable state. Draw dynamic capsules, kinematic proxies, mesh wire overlays, structural links, and invalid entities with distinct colors. Remove every handler during unregister and file load.

```python
def enable_overlay(area):
    key = area.as_pointer()
    if key not in _DRAW_HANDLERS:
        _DRAW_HANDLERS[key] = bpy.types.SpaceView3D.draw_handler_add(
            _draw_overlay, (key,), 'WINDOW', 'POST_VIEW')

def disable_all_overlays():
    for handler in _DRAW_HANDLERS.values():
        bpy.types.SpaceView3D.draw_handler_remove(handler, 'WINDOW')
    _DRAW_HANDLERS.clear()
```

- [ ] **Step 5: Run pure and Blender diagnostics tests**

Expected: PASS, including no leaked draw handlers after unregister/reload.

- [ ] **Step 6: Commit**

```bash
git add blender_extension/bone_physics/diagnostics.py blender_extension/bone_physics/viewport.py blender_extension/bone_physics/operators/diagnostics.py blender_extension/bone_physics/operators/__init__.py blender_extension/bone_physics/ui tests/python/test_authoring_diagnostics.py tests/blender/test_ui_diagnostics.py
git commit -m "feat(ui): navigate physics diagnostics"
```

### Task 12: Bone-parameter JSON presets

**Files:**
- Create: `blender_extension/bone_physics/preset_io.py`
- Create: `blender_extension/bone_physics/operators/preset.py`
- Create: `blender_extension/bone_physics/presets/bone_parameters/soft_hair.json`
- Create: `blender_extension/bone_physics/presets/bone_parameters/firm_ears.json`
- Create: `blender_extension/bone_physics/presets/bone_parameters/flexible_skirt.json`
- Modify: `blender_extension/bone_physics/operators/__init__.py`
- Modify: `blender_extension/bone_physics/ui/panels.py`
- Create: `tests/python/test_preset_io.py`
- Create: `tests/blender/test_ui_presets.py`

**Interfaces:**
- Produces: `preset_directories(kind, create=False)`, `list_presets(kind)`, `load_parameter_preset(path)`, `save_parameter_preset(path, preset)`, `rename_preset()`, and `delete_preset()`.
- Consumes `bpy.utils.user_resource`, `preset_paths`, and `preset_find(..., ext='.json')` only in a thin Blender path provider; JSON validation remains pure Python.

- [ ] **Step 1: Write failing atomic JSON and schema tests**

```python
def test_parameter_preset_round_trip_is_canonical_json(self):
    save_parameter_preset(path, parameter_preset("Soft Hair"))
    self.assertEqual(load_parameter_preset(path), parameter_preset("Soft Hair"))
    self.assertTrue(path.read_text("utf-8").endswith("\n"))

def test_unknown_newer_schema_does_not_mutate_target(self):
    before = endpoint_parameters()
    path.write_text('{"kind":"bone_parameters","schema_version":99}', "utf-8")
    with self.assertRaisesRegex(PresetError, "newer schema"):
        load_parameter_preset(path)
    self.assertEqual(before, endpoint_parameters())
```

- [ ] **Step 2: Run and record RED**

Run: `python -m unittest tests.python.test_preset_io -v`

Expected: FAIL because preset I/O is absent.

- [ ] **Step 3: Implement canonical schema and atomic writes**

Write UTF-8 JSON with sorted keys, `allow_nan=False`, two-space indentation, and final newline. Write a sibling temporary file, flush and close it, then use `os.replace`. Sanitize filenames separately from display names. Validate kind, schema, finite numbers, ranges, enum identifiers, and exact allowed keys.

```python
def atomic_write_json(path, value):
    payload = json.dumps(value, sort_keys=True, indent=2,
                         ensure_ascii=False, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(payload, encoding="utf-8")
    os.replace(temporary, path)
```

- [ ] **Step 4: Implement preset operators and confirmation dialogs**

Use `invoke_props_dialog(..., title=..., confirm_text=..., cancel_default=True)` for overwrite and delete. Display preset name and full path. Packaged presets are read-only; `Copy` targets the user directory. Apply Motion, Local Constraint, and Collision masks to heads, tails, selected bones, chains, or selected chains in one Undo step.

```python
class BONE_PHYSICS_OT_preset_delete(bpy.types.Operator):
    bl_idname = "bone_physics.preset_delete"
    bl_options = {'REGISTER', 'UNDO'}
    filepath: bpy.props.StringProperty(subtype='FILE_PATH')

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(
            self, width=420, title="Delete Preset",
            confirm_text="Delete", cancel_default=True)

    def execute(self, context):
        delete_user_preset(Path(self.filepath))
        return {'FINISHED'}
```

Serialize the three packaged files with the same schema used for user presets. Discovery marks them read-only. JSON keys and enum identifiers remain English; optional localized descriptions are values, not localized keys.

- [ ] **Step 5: Run pure tests and Blender preset test**

Use a temporary `BLENDER_USER_RESOURCES` root in Blender MCP/background tests and assert files appear below `presets/bone_physics/bone_parameters`.

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add blender_extension/bone_physics/preset_io.py blender_extension/bone_physics/operators/preset.py blender_extension/bone_physics/operators/__init__.py blender_extension/bone_physics/ui/panels.py blender_extension/bone_physics/presets tests/python/test_preset_io.py tests/blender/test_ui_presets.py
git commit -m "feat(presets): save bone parameter JSON"
```

### Task 13: Exact-armature presets and transactional restore

**Files:**
- Modify: `blender_extension/bone_physics/preset_io.py`
- Modify: `blender_extension/bone_physics/operators/preset.py`
- Modify: `blender_extension/bone_physics/ui/panels.py`
- Modify: `tests/python/test_preset_io.py`
- Modify: `tests/blender/test_ui_presets.py`

**Interfaces:**
- Produces: `armature_fingerprint(snapshot) -> str`, `serialize_exact_armature(config, snapshot) -> dict`, `validate_exact_armature(data, snapshot)`, and transactional `apply_exact_armature()`.
- Consumes full authoring config, stable IDs, bone UUID/name/path/rest transforms, and external object bindings.

- [ ] **Step 1: Write failing strict-match and zero-mutation tests**

```python
def test_exact_preset_rejects_changed_rest_topology_without_partial_apply(self):
    target = deepcopy(source_config())
    before = canonical_authoring_bytes(target)
    with self.assertRaisesRegex(PresetError, "rest topology"):
        apply_exact_armature(target, exact_preset(), changed_rest_skeleton())
    self.assertEqual(canonical_authoring_bytes(target), before)

def test_exact_preset_excludes_runtime_state_and_cache():
    data = serialize_exact_armature(source_config(), source_skeleton())
    self.assertNotIn("velocity", json.dumps(data))
    self.assertNotIn("checkpoint", json.dumps(data))
```

- [ ] **Step 2: Run and record RED**

Run the two named tests.

Expected: FAIL because exact presets are absent.

- [ ] **Step 3: Implement canonical fingerprints and strict preflight**

Hash sorted bone UUID, name, parent UUID, length, and rest transform with explicit float encoding. Validate every bone path, stable ID, target, collider object, mesh object, region rule, and schema before constructing a replacement config. Never mutate the live PropertyGroups during validation.

```python
def armature_fingerprint(snapshot):
    digest = hashlib.sha256()
    for bone in sorted(snapshot.bones, key=lambda item: item.uuid):
        digest.update(bone.uuid.encode("utf-8"))
        digest.update(bone.name.encode("utf-8"))
        digest.update((bone.parent_uuid or "").encode("utf-8"))
        digest.update(struct.pack("<d", bone.length))
        digest.update(struct.pack("<16d", *bone.rest_matrix))
    return digest.hexdigest()
```

- [ ] **Step 4: Implement one-step replacement and runtime invalidation**

Build replacement RNA collections in an isolated Python representation, clear and repopulate live PropertyGroups only after validation succeeds, then mark all systems structurally dirty and rebuild transactionally. The operator has `UNDO` and reports counts before confirmation.

```python
def apply_exact_armature(owner, data, snapshot, registry):
    replacement = deserialize_exact_armature(data)
    validate_exact_armature(replacement, snapshot)
    compiled = compile_authoring(replacement, snapshot, snapshot.collision)
    replacement_world = registry.build_detached(owner, compiled)
    write_authoring_properties(owner.bone_physics, replacement)
    registry.swap(owner, replacement_world)
```

- [ ] **Step 5: Run pure and Blender exact-preset tests**

Expected: PASS for round-trip on the same armature and unchanged configuration on every mismatch case.

- [ ] **Step 6: Commit**

```bash
git add blender_extension/bone_physics/preset_io.py blender_extension/bone_physics/operators/preset.py blender_extension/bone_physics/ui/panels.py tests/python/test_preset_io.py tests/blender/test_ui_presets.py
git commit -m "feat(presets): restore exact armature config"
```

### Task 14: English and Simplified Chinese internationalization

**Files:**
- Create: `blender_extension/bone_physics/translations.py`
- Modify: `blender_extension/bone_physics/__init__.py`
- Modify: all files under `blender_extension/bone_physics/ui/`
- Modify: all files under `blender_extension/bone_physics/operators/`
- Create: `tests/python/test_translations.py`
- Create: `tests/blender/test_ui_translations.py`

**Interfaces:**
- Produces English stable message IDs, translation contexts, `zh_HANS` catalog, `register_translations()`, `unregister_translations()`, and `format_issue(issue)`.
- Consumes validation error codes and operator report arguments; internal enum/JSON identifiers remain untranslated.

- [ ] **Step 1: Write failing critical-message coverage test**

```python
def test_every_critical_message_has_simplified_chinese_translation():
    missing = critical_message_ids() - translated_message_ids("zh_HANS")
    self.assertEqual(missing, set())

def test_json_and_enum_identifiers_are_not_translation_keys():
    self.assertNotIn("gravity_scale", translated_message_ids("zh_HANS"))
    self.assertNotIn("SWING_TWIST", translated_message_ids("zh_HANS"))
```

- [ ] **Step 2: Run and record RED**

Run: `python -m unittest tests.python.test_translations -v`

Expected: FAIL because translation catalog is absent.

- [ ] **Step 3: Implement translation registration and contextual messages**

Register `{locale: {(context, msgid): translation}}` through `bpy.app.translations.register(__name__, catalog)` after classes register; unregister translations before class teardown. Use English source strings in UI definitions and stable issue codes plus arguments for runtime formatting.

```python
CATALOG = {
    "zh_HANS": {
        ("*", "Bone Physics"): "骨骼物理",
        ("*", "Validate and Rebuild"): "验证并重建",
        ("*", "Delete Preset"): "删除预设",
    }
}

def register_translations():
    bpy.app.translations.register(__name__, CATALOG)

def unregister_translations():
    bpy.app.translations.unregister(__name__)
```

- [ ] **Step 4: Write Blender language-switch smoke test**

Set Blender interface translation to Simplified Chinese, call representative translated labels/tooltips/confirm text through `pgettext_iface`/`pgettext_tip`, and assert Chinese output. Restore preferences in `finally`.

- [ ] **Step 5: Run translation tests through Python and Blender MCP**

Expected: PASS with English fallback and complete Simplified Chinese critical coverage.

- [ ] **Step 6: Commit**

```bash
git add blender_extension/bone_physics/translations.py blender_extension/bone_physics/__init__.py blender_extension/bone_physics/ui blender_extension/bone_physics/operators tests/python/test_translations.py tests/blender/test_ui_translations.py
git commit -m "feat(i18n): add Simplified Chinese UI"
```

### Task 15: Integrated character workflows, packaging, and final documentation

**Files:**
- Create: `tests/blender/test_ui_character_workflows.py`
- Modify: `scripts/ci/run_blender_tests.py`
- Modify: `tests/python/test_packaging.py`
- Modify: `README.md`
- Modify: `docs/testing.md`
- Modify: `docs/limitations.md`
- Modify: `docs/HANDOFF.md`

**Interfaces:**
- Consumes every UI, authoring, collision, runtime, preset, and translation interface from Tasks 1-14.
- Produces installed-extension evidence for hair, ears, ponytail, skirt, and accessory authoring flows.

- [ ] **Step 1: Write installed Blender workflow scenarios**

The Blender script must build deterministic fixtures and assert:

```python
assert configure_hair(multiple_roots=True, disconnected_children=True)
assert configure_ears(shared_parameter_preset=True, self_collision=False)
assert configure_ponytail(self_collision=True, body_collection="Upper Body")
assert configure_skirt(chains=4, horizontal=True, diagonals=True, seam=True)
assert configure_accessory(per_bone_override=True)
assert reset_changes_only_configured_physical_bones()
assert exact_preset_round_trip_is_identical()
```

Use actual operators and PropertyGroups; helper return values must be derived from compiled WorldSpec/batch assertions, not unconditional booleans.

- [ ] **Step 2: Run through Blender MCP and diagnose the first failure**

Expected: PASS only when every workflow produces valid compiled bodies,
constraints, filters, and resources and can advance one deterministic step.

- [ ] **Step 3: Add the workflow to bounded CI and package assertions**

Append `test_ui_character_workflows.py` to `BLENDER_TEST_SCRIPTS`. Extend packaging tests to assert every new Python module and bundled read-only preset is present, while `__pycache__`, `.pyc`, and user JSON remain absent.

- [ ] **Step 4: Update user and developer documentation**

Document the Physics System workflow, N-panel locations, collision ownership,
reset semantics, preset locations, English/Simplified Chinese support, Blender
MCP test command, and known V1 dynamic-shape limitation. Update HANDOFF with
exact completed tasks and next action; do not claim cross-platform evidence
before CI artifacts exist.

- [ ] **Step 5: Run Python discovery**

Run: `python -m unittest discover tests/python -v`

Expected: all tests PASS with zero failures.

- [ ] **Step 6: Run Blender MCP installed-extension suite**

Run every script listed in `BLENDER_TEST_SCRIPTS` through the bounded Blender MCP/background harness.

Expected: all scripts exit zero and their report files are nonempty.

- [ ] **Step 7: Run Rust and formatting gate**

Run:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
```

Expected: every command exits zero. If dependency metadata changed, also run `cargo tree --duplicates` and record the result.

- [ ] **Step 8: Verify package structure and staged diff**

Run:

```bash
python -m unittest tests.python.test_packaging -v
git diff --check
git status --short
```

Expected: packaging tests PASS, `git diff --check` prints nothing, and status lists only intended implementation/documentation files.

- [ ] **Step 9: Commit final integration concern**

```bash
git add tests/blender/test_ui_character_workflows.py scripts/ci/run_blender_tests.py tests/python/test_packaging.py README.md docs/testing.md docs/limitations.md docs/HANDOFF.md
git commit -m "test(ui): cover character authoring flows"
```

## Execution checkpoints

- After Task 5: installable extension registers persistent RNA and displays a functional N-panel system browser.
- After Task 8: users can author multi-chain motion and local/structural constraints without collision.
- After Task 11: collision, runtime controls, diagnostics, and viewport inspection work together.
- After Task 14: both preset kinds and English/Simplified Chinese UI are complete.
- After Task 15: installed-extension workflow and full repository gates provide completion evidence.

## Spec coverage audit

- N-panel hierarchy, system-first workflow, and configured-entity lists: Tasks 4, 5, and 7.
- Multi-selected chain heads, disconnected parenting, branch tails, and overlap rejection: Tasks 3 and 5.
- Full head/tail parameter fields, accumulated-length gradients, and per-bone overrides: Tasks 1, 6, and 7.
- Automatic local constraints, non-physical root targets, swing-twist, skirt lattices, and generated inspection: Tasks 1, 6, and 8.
- Dynamic capsules, kinematic analytic shapes, pre-physics mesh collision, ownership, self-collision, and system matrices: Task 9.
- Global/system enable, isolation, reset scope, guarded dirty rebuilds, cache/bake controls, and rollback: Task 10.
- Configured entity diagnostics, issue navigation, generated counts, and viewport overlays: Task 11.
- Classic user preset paths, universal bone values, exact-armature files, CRUD confirmations, migrations, and atomic JSON: Tasks 12 and 13.
- Armature Object persistence, stable IDs, package registration, English, and Simplified Chinese: Tasks 2, 4, and 14.
- Hair, ears, ponytail, skirt, accessory workflows, packaging, documentation, and complete verification: Task 15.
