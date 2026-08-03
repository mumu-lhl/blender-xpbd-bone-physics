# Testing

The Blender-independent contracts run with Python's standard library:

```bash
python -m unittest discover tests/python -v
```

The reference acceptance runner uses a fixed 120 Hz physical step, runs the
24/30/60 FPS equal-time matrix twice, and writes a machine-readable report:

```bash
python tests/blender/run_reference.py \
  --acceptance tests/reference/acceptance.json \
  --output artifacts/reference.json --backend native
```

The native matrix requires the locally built abi3 module. Blender-side
debugging is run through Blender MCP; do not start an unbounded Blender process
from a shell. The installed-extension suite is bounded by the CI harness: it
creates an isolated Blender user-resource directory, installs the generated
extension ZIP, runs each `BLENDER_TEST_SCRIPTS` entry in background Blender,
and writes one log plus a JSON report for every command.

```bash
python scripts/ci/run_blender_tests.py \
  --blender /path/to/blender \
  --wheel dist/bone_physics_native-*.whl \
  --report-dir artifacts/blender-tests
```

`test_ui_character_workflows.py` covers hair, ears, ponytail, skirt, and
accessory authoring through registered operators and PropertyGroups. Its
assertions inspect the compiled `WorldSpec` and native build batch; it does not
depend on an interactive Blender window.

Rust, native-wheel, and Blender-headless checks run in the CI jobs when those
toolchains are unavailable locally. Ordinary PR checks do not create Releases;
the manual package workflow only records the explicit request and evidence.
