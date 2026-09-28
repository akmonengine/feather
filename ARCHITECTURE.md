# Feather - Architecture

## Packages
```
feather/
├── world.go            # World.Step: collision detection, then solver
├── solver.go           # TGS Soft solver
├── graph.go            # graph coloring of the contacts, for the parallel solver
├── pool.go             # workers of the step
├── island.go           # sleep islands
├── joint.go            # joints: distance, ball, hinge, fixed
├── joint_configurable.go # configurable joint: each axis locked, limited or free
├── articulation.go     # the anchors of the trees of joints, solved together (Baraff 1996)
├── collision.go        # BroadPhase, NarrowPhase, Collide
├── collision_capsule.go# spheres & capsules: closest points of segments
├── collision_heightfield.go # heightfields: triangles, inner edges, patches
├── ccd.go              # continuous collision: time of impact of the fast bodies
├── spatialgrid.go      # broad phase: uniform grid
├── event.go            # collision, trigger & sleep events
├── actor/              # RigidBody, Material, Transform, shapes (Sphere, Box, Plane, Capsule, Heightfield)
├── constraint/         # Manifold, ContactPoint, friction & restitution mixing
├── gjk/                # GJK (overlap test with margin, distance)
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
│   ├── a sleeping body touched by an awake body wakes up: the detection runs again
│   ├── events: pairs touching or overlapping (triggers are not solved)
│   └── warm start: each point takes the impulses of the same point in the previous step
├── Phase 2: solver (substeps: joints, then contacts), then restitution
├── continuous collision: the fast bodies are moved back to their first impact
└── Phase 3: sleep islands & events
```

## Collision detection
| Pair | Method |
|------|--------|
| any shape - plane | `CollideWithPlane` of the shape |
| any shape - heightfield | each triangle under the body: GJK + EPA, inner edges, patches (up to 8 manifolds) |
| sphere / capsule - sphere / capsule | closest points of the segments (a sphere is a segment of length 0) |
| other pairs | GJK + EPA, then clipping of the contact points |

Pair cache (like Jolt): if a body moved less than 1 mm and 2° relative to the other since their contact points were computed,
the previous contact points are moved with the bodies, the collision detection doesn't run again.

Contacts are kept up to a margin: `SpeculativeDistance` (2 cm), + the relative speed of the bodies * dt against a
static body.
Each manifold has a normal (from A to B) and up to 4 points. A pair has 1 manifold, up to 8 against a heightfield
(the manifolds of a pair follow each other). Each point has its own separation (< 0 when the bodies overlap).

## Solver
See [ALGORITHMS.md](ALGORITHMS.md#solver). The solver works on copies of the dynamic bodies (`bodyState`):
the static and sleeping bodies share a state with no mass.

## Threading & determinism
- From 256 bodies, a step runs on `Workers` goroutines. The workers are created once and sleep between the steps.
  `World.Close()` stops them (they are also stopped when the World is garbage collected).
- The broad phase, the narrow phase, the preparation of the contacts and the integration of the bodies:
  each body, pair or contact writes its result at its own index, the order of execution doesn't matter.
- The pairs are sorted (index of the first body, then of the second body).
- The solver is a Gauss-Seidel: a contact uses the result of the previous one. The contacts are colored
  (like Box2D v3): the contacts of a color don't share any dynamic body, so a color is solved in parallel.
  The colors are always solved in the same order: the result is the same bit for bit, whatever the number of workers.
- A step doesn't allocate memory after the first steps: the buffers are reused.

## Tests & benchmarks
**The reference is Box3D** (Erin Catto, 2026): Feather must do at least as well on the same scenes at 60 Hz, Feather
with 8 substeps (its setting for the games), Box3D with its default 4. The scenes come from Solver2D, extruded by 1 m in
3D (the same supports, the same mass ratios). Box3D (`bench/box3d`) and Jolt (`bench/jolt`) are compiled outside the
repository to measure the references; the values of Box3D are written in the tests with their date. The known gaps are
logged, and followed by #821.

Four levels, from the most precise to the widest:
1. **Minimal scenes** (`scenes_test.go`): one or two bodies isolating a mechanism. The bound of each scene is derived
   from a quantity of the engine (`LinearSlop` for a depth), never fixed after a measure.
2. **Invariants** (`invariants_test.go`): `checkInvariants` runs at every step of 60 random scenes (piles on a plane,
   on a terrain, bodies & joints in free flight), in parallel, in about 1 s. Each tolerance comes from the method:
   the rounding for the momentum, the first order gyroscopic torque and the couple of the joints (gap × impulse) for
   the angular momentum, `LinearSlop` for the depth & the energy pushed out of the ground. Reintroducing the bugs fixed on 27/09 (rotation capped per step,
   contact points frozen during the step, a box touching a plane with its 8 corners) makes it fail.
3. **Scenes of Solver2D** (`bench/scenes`, `cd bench && go test ./...`): the samples of Erin Catto's Solver2D in 3D
   (stacks, high mass ratios, overlap recovery, house of cards, chains, far from the origin...), small in the tests,
   full in the bench (`go run . -scenes`, `go run . -compare` side by side with v0.2.0). Each scene holds (a derived
   criterion) and, where Box2D has the scene, does at least as well as Box2D v3.1.
4. **Regressions** (`bench/regression.go`): 6 chaotic scenes and the scenes of Solver2D, compared to
   `bench/baseline.json`:
   - the fingerprint of the final state (identical on the same GOARCH);
   - quality metrics, 0.5 mm of tolerance on a depth, 0.1 % on an energy gain;
   - the time of a step (+20 %) and of its phases (+30 %, over 5 % of the step), best of 3 runs, only on the machine
     of the reference, and for the steps over 0.1 ms (under it, the noise of the timer dominates).

`World.Profile()` gives the time of each phase of the last step (broad phase, narrow phase, prepare, substeps,
restitution, continuous collision, islands), without allocation.
`World.parallelFrom` (tests only) runs the parallel paths under 256 bodies, for the determinism.

## Current limitations
- The broad phase is a uniform grid: very large and very small bodies in the same scene are slow
  (the planes & the heightfields are not in the grid, they are tested with every body).
- A heightfield is a surface: a body entirely under it is not pushed up.
- The contacts are computed once per step: on a rough terrain, a corner of a tumbling body can slide over another
  triangle during the step, and sink by a few mm before the next step.
- The friction around the normal comes from the lever arms of the points: a ball spinning on itself on its single point
  of contact never stops (no sleep).
- A capsule resting across a bump of a terrain can stay a few mm in the terrain: the contact of a triangle comes from
  the feature of the body above the triangle, the middle of the capsule is missed.
- The restitution is applied once per step, with the velocity before the step: a body not round (box, capsule),
  bouncy (`e` over 0.5) and spinning fast (20-50 rad/s) can bounce higher than it fell. Measured: up to +60 % of
  energy at `e = 1`, never up to `e = 0.5`. Jolt documents the same limit.
- No kinematic bodies (moving platforms): a body is static or dynamic.
- The continuous collision stops the fast bodies against the static bodies (and the bullets against all the bodies),
  not the other pairs: 2 fast dynamic bodies rely on their speculative contacts (2 cm) and on the spring of the contact.
