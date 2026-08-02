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
  --output artifacts/reference.json
```

Rust, native-wheel, and Blender-headless checks run in the CI jobs when those
toolchains are unavailable locally. Ordinary PR checks do not create Releases;
the manual package workflow only records the explicit request and evidence.
