# Native Batch API v1

The Python bridge accepts standard-library buffer providers such as
`array.array` and `memoryview`. Every input buffer must have the exact dtype
listed below and be C-contiguous. The bridge validates the complete batch,
copies it into owned Rust vectors, and only then creates or mutates a solver
world.

Stable solver IDs are unsigned 32-bit integers. World handles are unsigned
64-bit integers returned by the bridge. Quaternions use `(x, y, z, w)` order.

## World build

`build_world(config, bodies, constraints, filters) -> handle`

`config` contains:

- `api_version: int`
- `gravity: tuple[float, float, float]`
- `fixed_dt: float`

`bodies` contains `body_ids: u32[N]`, `collider_ids: u32[N]`,
`values: f32[N, 46]`, and `columns: 46`.

| Columns | Meaning |
|---|---|
| 0..3 | rest translation |
| 3..7 | rest quaternion |
| 7..10 | local center of mass |
| 10..13 | local head |
| 13..16 | local tail |
| 16 | inverse mass |
| 17..20 | inverse local inertia |
| 20 | gravity scale |
| 21..23 | linear and angular damping |
| 23..26 | initial translation |
| 26..30 | initial quaternion |
| 30..33 | initial linear velocity |
| 33..36 | initial angular velocity |
| 36..39 | capsule local endpoint A |
| 39..42 | capsule local endpoint B |
| 42..44 | capsule radius and margin |
| 44..46 | friction and restitution |

`constraints` contains `ids: u32[M]`, `body_ids: u32[M, 2]`,
`kinds: u32[M]`, `enabled: u32[M]`, `values: f32[M, 24]`, and
`columns: 24`.

Constraint kinds are anchor `0`, distance `1`, distance range `2`, attachment
`3`, orientation recovery `4`, XYZ angular limit `5`, and swing-twist limit
`6`. Common columns are local anchors at `0..6`, range/rest values at `6..8`,
compliance and damping at `8..10`, rest quaternion at `10..14`, and angular
limit data at `14..21`. XYZ Euler order is encoded as an integer-valued float
from `0` (`XYZ`) through `5` (`ZYX`).

Authoring compiles root attachments as kind `3` with `body_b = 0`; the two
anchor vectors are the physical root and its animation-target local anchors.
Swing-twist rows use kind `6`: axis at `14..17`, maximum swing at `17`, and
minimum/maximum twist at `18..20`. These encodings retain the v1 24-column
constraint layout and API version.

`filters` contains `layers: u32[N]`, `masks: u32[N]`, and two compressed
offset/value pairs:

- `excluded_body_offsets: u32[N + 1]` with `excluded_body_ids: u32[*]`
- `excluded_constraint_offsets: u32[N + 1]` with
  `excluded_constraint_ids: u32[*]`

Offsets start at zero, are monotonic, and end at the corresponding value
count.

## Steady-state batches

`set_animation_targets(handle, batch)` consumes `body_ids: u32[N]`,
`values: f32[N, 16]`, and `columns: 16`. Each row is previous pose `0..7`,
current pose `7..14`, translation inertia `14`, and rotation inertia `15`.

`set_analytic_colliders(handle, batch)` consumes `ids`, `kinds`, and `enabled`
as `u32[N]`, plus `values: f32[N, 17]` and `columns: 17`. Every row starts
with pose `0..7` and linear velocity `7..10`. Shape `0` is a sphere with
radius at `10`; shape `1` is a capsule with endpoints at `10..16` and radius
at `16`; shape `2` is a rounded box with half extents at `10..13` and radius
at `13`.

`advance(handle, frame_seconds, quality)` accepts quality
`deterministic_preview`, `approximate_preview`, or `authoritative_bake` and
returns one small diagnostics dictionary.

`resume_world(handle)` revalidates a rolled-back world and clears its native
pause state. A stale handle or still-invalid restored state is rejected.

`read_transforms(handle, output)` writes exactly `7 * body_count` contiguous
`f32` values in ascending stable body-ID order. Each row is translation
followed by quaternion. `read_diagnostics(handle)` returns scalar world and
last-step diagnostics.
