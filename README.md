# blender-xpbd-bone-physics

A general-purpose Blender secondary-motion solver based on oriented XPBD
rigid bone segments and a unified constraint graph.

The project targets hair, skirts, cloth rigs, tails, breasts, accessories, and
other bone-driven physics. It is a new solver rather than a compatibility layer
for UMA, PMX/Bullet, or existing Blender jiggle-bone add-ons.

## Status

Design specification approved; V1.0.0 is released from the `v1.0.0` tag.
The V1 feature set is complete, and ongoing work is limited to local
maintenance and bug fixes.

Read the approved architecture:

- [Design specification](docs/design/blender-xpbd-bone-physics-design.md)
- [Core foundation plan](docs/plans/v1-core-foundation.md)
- [Constraints and collision plan](docs/plans/v1-constraints-collision.md)
- [PyO3 and Blender runtime plan](docs/plans/v1-blender-runtime.md)
- [Mesh, bake, and release plan](docs/plans/v1-mesh-bake-release.md)
- [Current project status](docs/HANDOFF.md)

## Core decisions

- Rust oriented-XPBD solver with a batched PyO3 bridge
- fixed physical time step and deterministic bake checkpoints
- local XYZ angular limits and swing-twist limits
- root-to-tip parameter fields
- constraints within and between arbitrary bone chains
- continuous capsule self-collision and cross-chain collision
- analytic body proxies plus pre-physics deforming-mesh collision
- self-contained Blender extension with no separately installed runtime
  dependencies

## First-version boundaries

The first version focuses on foundational bone motion, constraints, collision,
cache, and bake. Wind, GPU solving, tearing, and other post-V1 extensions remain
outside the initial scope.

## Blender workflow

Select an armature, open the **Bone Physics** tab in the 3D Viewport N-panel,
then initialize its configuration. Add a Physics System first, choose a
template, and add selected pose-bone heads as chains. Setup contains systems,
chains, parameters, local and structural constraints, and collision resources;
Simulate contains enable, pause, isolate, validation/rebuild, reset, and bake
controls; Diagnostics contains configured-entity issues and viewport overlays.

Collision resources belong to the armature configuration and are explicitly
assigned to systems. An analytic resource may be bound to the world, an object,
or an armature bone; mesh resources are evaluated from the pre-physics mesh.
Reset restores animation references only for configured physical bones, leaving
unconfigured pose channels alone.

The packaged read-only bone parameter presets are in the extension's
`presets/bone_parameters` directory. User-created parameter and exact-armature
presets are stored in Blender's user preset location and are never bundled.
The interface is available in English and Simplified Chinese.

## Local development

The project is developed with the local Codex CLI workflow. From the repository
root, use the `justfile` targets to build the native module, install it into the
source extension used by the Blender VS Code launcher, or create the final
archive:

```bash
just build-native
just install-native
just package-extension
```

Use Blender MCP for Blender runtime and UI checks; do not launch an unbounded
Blender process from the shell.

## Verification

Run the complete local gate with the pinned toolchain and `uv` environment:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
python -m unittest discover tests/python -v
python tests/blender/run_reference.py \
  --acceptance tests/reference/acceptance.json \
  --output artifacts/reference.json --backend native
```

For installed-extension UI coverage, use the bounded Blender MCP/background
harness (with the wheel produced for the host platform):

```bash
python scripts/ci/run_blender_tests.py \
  --blender /path/to/blender \
  --wheel dist/bone_physics_native-*.whl \
  --report-dir artifacts/blender-tests
```

Package/load evidence is produced by `scripts/ci/run_package_load.py` and the
four-platform workflow. Pushing a version tag runs the package gate and
automatically creates the GitHub Release; manual dispatch performs verification
only.

## License

GPL-3.0.
