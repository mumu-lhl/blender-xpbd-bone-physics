# V1 limitations

V1 deliberately does not include wind, GPU solving, tearing, or cross-armature
constraints. Invalid or open meshes use two-sided collision history; they do
not receive a guessed global inside/outside classification.

Floating-point results may differ within the approved acceptance tolerances
between supported platforms. Blender 5.2 headless package evidence is required
on Linux; macOS Intel records native/CPython loading evidence because an
official Blender runner binary is not guaranteed there.

V1 dynamic analytic collision shapes are kinematic collision inputs: their
motion affects the relative contact velocity, but a dynamic physics bone does
not push or carry an animated collider. Use a separate authored animation or a
future bidirectional-coupling feature for that behavior.

The package contains the native module and Python runtime code needed by the
extension. Users do not install Rust, pip packages, NumPy, or a system physics
library.
