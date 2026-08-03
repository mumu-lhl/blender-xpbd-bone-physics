set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

native_manifest := "crates/bone_physics_py/Cargo.toml"
native_wheel_dir := "target/wheels"
maturin := env_var_or_default("MATURIN", "uv tool run --from maturin==1.14.1 maturin")

# Build the abi3 bone_physics_native wheel used by Blender and native tests.
build-native:
    {{maturin}} build --release --manifest-path {{native_manifest}} --out {{native_wheel_dir}}
