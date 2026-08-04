set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

native_manifest := "crates/bone_physics_py/Cargo.toml"
native_wheel_dir := "target/wheels"
extension_source := env_var_or_default("BLENDER_EXTENSION_DIR", "blender_extension")
native_platform := env_var_or_default("NATIVE_PLATFORM", "linux-x86_64")
maturin := env_var_or_default("MATURIN", "uv tool run --from maturin==1.14.1 maturin")

# Build the abi3 bone_physics_native wheel used by Blender and native tests.
build-native:
    {{maturin}} build --release --manifest-path {{native_manifest}} --out {{native_wheel_dir}}

# Build and copy the native module into the source tree used by Blender's
# extension-development workflow (for example, a VS Code Blender launcher).
install-native: build-native
    python scripts/install_native.py --extension-root {{extension_source}} --wheel-dir {{native_wheel_dir}} --platform {{native_platform}}

# Build a self-contained, reproducible Blender extension archive and verify it.
package-extension: build-native
    python scripts/package_extension.py --source . --output dist/bone_physics-{{native_platform}}.zip --wheel-dir {{native_wheel_dir}} --platform {{native_platform}}
    python scripts/verify_package.py dist/bone_physics-{{native_platform}}.zip --platform {{native_platform}}
