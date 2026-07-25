# blender-xpbd-bone-physics

A general-purpose Blender secondary-motion solver based on oriented XPBD
rigid bone segments and a unified constraint graph.

The project targets hair, skirts, cloth rigs, tails, breasts, accessories, and
other bone-driven physics. It is a new solver rather than a compatibility layer
for UMA, PMX/Bullet, or existing Blender jiggle-bone add-ons.

## Status

Design specification complete; implementation planning pending written-spec
review.

Read the approved architecture:

- [Design specification](docs/superpowers/specs/2026-07-25-blender-xpbd-bone-physics-design.md)

## Core decisions

- Rust oriented-XPBD solver with a batched PyO3 bridge
- fixed physical time step and deterministic bake checkpoints
- local XYZ angular limits and swing-twist limits
- root-to-tip parameter fields
- constraints within and between arbitrary bone chains
- continuous capsule self-collision and cross-chain collision
- analytic body proxies plus pre-physics deforming-mesh collision
- self-contained Blender extension with no separately installed runtime
  dependencies

## First-version boundaries

The first version focuses on foundational bone motion, constraints, collision,
cache, and bake. Wind, GPU solving, tearing, and full interaction design are
outside the initial scope.

## License

GPL-3.0.
