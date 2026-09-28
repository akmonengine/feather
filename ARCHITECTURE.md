# Feather - Architecture

## Packages
```
feather/
├── world.go            # World.Step: collision detection, then solver
├── solver.go           # TGS Soft solver
├── collision.go        # BroadPhase, NarrowPhase, Collide
├── collision_capsule.go# spheres & capsules: closest points of segments
├── spatialgrid.go      # broad phase: uniform grid
├── event.go            # collision, trigger & sleep events
├── actor/              # RigidBody, Material, Transform, shapes (Sphere, Box, Plane, Capsule)
├── constraint/         # Manifold, ContactPoint, friction & restitution mixing
├── gjk/                # GJK (overlap test, with margin)
├── epa/                # EPA (penetration depth) & contact points (manifold)
└── bench/              # comparison with v0.2.0 (separate module)
```

## World.Step
```
Step(dt)
├── wake the sleeping bodies touched by a moving body
├── Phase 1: collision detection (once per step)
│   ├── AABBs enlarged by the distance each body can travel during dt
│   ├── broad phase: pairs of overlapping AABBs (spatial grid)
│   ├── narrow phase: manifold of each pair (parallel, Workers goroutines)
│   ├── events: pairs touching or overlapping (triggers are not solved)
│   └── warm start: each point takes the impulses of the same point in the previous step
├── Phase 2: solver (substeps), then restitution
└── Phase 3: sleep & events
```

## Collision detection
| Pair | Method |
|------|--------|
| any shape - plane | `CollideWithPlane` of the shape |
| sphere / capsule - sphere / capsule | closest points of the segments (a sphere is a segment of length 0) |
| other pairs | GJK + EPA, then clipping of the contact points |

Contacts are kept up to a margin: `SpeculativeDistance` (2 cm) + the relative speed of the bodies * dt.
Each manifold has a normal (from A to B) and up to 4 points. Each point has its own separation (< 0 when the bodies overlap).

## Solver
See [ALGORITHMS.md](ALGORITHMS.md#solver). The solver works on copies of the dynamic bodies (`bodyState`):
the static and sleeping bodies share a state with no mass.

## Threading & determinism
- The broad phase and the narrow phase are split between `Workers` goroutines. Each pair writes its result at its own index,
  so the result never depends on the order of execution.
- The pairs are sorted (index of the first body, then of the second body), the solver and the events follow this order.
- The solver is sequential (Gauss-Seidel): it needs the result of the previous contact.

## Current limitations
- No joints yet (distance, hinge...).
- The broad phase is a uniform grid: very large and very small bodies in the same scene are slow.
- Sleep is per body (no islands): a stack falls asleep body by body.
- No continuous collision for very fast rotating bodies (the speculative margin covers the translation).
