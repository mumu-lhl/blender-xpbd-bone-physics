# V1 limitations

V1 deliberately does not include wind, GPU solving, tearing, or cross-armature
constraints. Invalid or open meshes use two-sided collision history; they do
not receive a guessed global inside/outside classification.

Floating-point results may differ within the approved acceptance tolerances
between supported platforms. Blender 5.2 headless package evidence is required
on Linux; macOS Intel records native/CPython loading evidence because an
official Blender runner binary is not guaranteed there.

The package contains the native module and Python runtime code needed by the
extension. Users do not install Rust, pip packages, NumPy, or a system physics
library.
