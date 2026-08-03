# Building

The native extension is built with Rust 1.97.1 and maturin. The supported
runtime is Blender 5.2.x with Python 3.11-compatible abi3 bindings.

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
