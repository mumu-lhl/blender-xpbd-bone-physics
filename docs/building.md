# Building

The native extension is built with Rust 1.97.1 and maturin. The supported
runtime is Blender 5.2.x with Python 3.11-compatible abi3 bindings.

```bash
python -m pip install maturin==1.14.1
maturin build --release --manifest-path crates/bone_physics_py/Cargo.toml
```

Use `scripts/package_extension.py` to create a normalized archive and
`scripts/verify_package.py` to verify its checksum, manifest, ordering, and
platform-native module. `scripts/check_native_deps.py` records dynamic-library
dependencies without installing a system runtime.
