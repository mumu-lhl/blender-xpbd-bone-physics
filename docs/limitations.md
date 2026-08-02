# V1 limitations

- Authoritative baking is an offline correctness path; preview quality tiers
  intentionally use smaller iteration budgets.
- Blender 5.2 official distribution coverage is platform-specific. macOS Intel
  native loading can be checked with CPython, but there is no official Blender
  5.2 Intel release to claim as a fresh-host Blender installation.
- Native Rust and Blender validation must be completed by the corresponding CI
  jobs when those toolchains are unavailable on the development host.
- A release is not created by ordinary pull-request CI. A draft RC requires
  explicit approval and complete, commit-matched platform evidence.
