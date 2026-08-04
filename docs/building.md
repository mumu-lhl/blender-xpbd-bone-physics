# Building

The native extension is built with Rust 1.97.1 and maturin. The supported
runtime is Blender 5.2.x, which embeds Python 3.13. The PyO3 bridge uses
`abi3-py311`, so its `cp311-abi3` wheel is compatible with Blender's Python
3.13 runtime.

```bash
python -m pip install maturin==1.14.1
maturin build --release --manifest-path crates/bone_physics_py/Cargo.toml
```

From the repository root, the pinned equivalent is:

```bash
just build-native
```

This writes the abi3 wheel to `target/wheels/`. Set `MATURIN` to override
the default `uv tool run` command when using a locally installed maturin.

For the VS Code Blender development workflow, copy the built native module
into the source extension directory:

```bash
just install-native
```

This installs it under `blender_extension/bone_physics/native/linux-x86_64/`.
Set `BLENDER_EXTENSION_DIR` when the VS Code launcher uses another source
directory, and set `NATIVE_PLATFORM` for another supported platform.
When loading the source tree directly, select `blender_extension/` as the
extension directory; its root entry point forwards Blender's `register()` and
`unregister()` lifecycle calls to the add-on package.

To create the final self-contained extension archive, including its checksum
and package report, run:

```bash
just package-extension
```

The archive is written to `dist/bone_physics-linux-x86_64.zip`. Both commands
use the wheel in `target/wheels/` produced by `just build-native`.

Use `scripts/package_extension.py` to create a normalized archive and
`scripts/verify_package.py` to verify its checksum, manifest, ordering, and
platform-native module. `scripts/check_native_deps.py` records dynamic-library
dependencies without installing a system runtime. The package report includes
the GPL-3.0-or-later license inventory.

For a complete platform evidence record, run:

```bash
python scripts/ci/run_package_load.py \
  --platform linux-x86_64 --report-dir artifacts/linux-x86_64 \
  --commit "$(git rev-parse HEAD)"
```
