# Feather - Architecture

## Packages
```
feather/
├── world.go            # World.Step: collision detection, then solver
├── solver.go           # TGS Soft solver
├── graph.go            # graph coloring of the contacts and the joints, for the parallel solver
├── pool.go             # workers of the step
├── island.go           # sleep islands
├── joint.go            # joints: distance, ball, hinge, fixed
├── joint_configurable.go # configurable joint: each axis locked, limited or free
├── articulation.go     # the anchors of the trees of joints, solved together (Baraff 1996)
├── collision.go        # BroadPhase, NarrowPhase, Collide
├── collision_capsule.go# spheres & capsules: closest points of segments
├── collision_heightfield.go # heightfields: triangles, inner edges, patches
├── ccd.go              # continuous collision: time of impact of the fast bodies
├── tree.go             # broad phase: dynamic AABB trees, the pairs kept from a step to the next
├── filter.go           # collision filtering: layers & masks, ignored pairs, linked bodies, the filter of the queries
├── query.go            # queries: Raycast, RaycastAll, SyncQueries, the guard, the traversal of the trees by a segment
├── query_sweep.go      # Sweep: conservative advancement on the cores, against a plane, a convex body, a triangle
├── query_overlap.go    # Overlap
├── query_heightfield.go# a sweep & an overlap against the triangles of a heightfield
├── event.go            # collision, trigger & sleep events
├── actor/              # RigidBody, Material, Transform, shapes (Sphere, Box, Plane, Capsule, Heightfield), their CastRay
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
│   ├── broad phase: pairs of overlapping AABBs (AABB trees) which pass the collision filters
│   ├── narrow phase: manifold of each pair (parallel, Workers goroutines)
│   ├── a sleeping body touched by an awake body wakes up: the detection runs again
│   ├── events: pairs touching or overlapping (triggers are not solved)
│   └── warm start: each point takes the impulses of the closest point of the pair in the previous step
├── Phase 2: solver (substeps: articulations, then contacts and joints by color), then restitution
├── continuous collision: the fast bodies are moved back to their first impact (with the bodies they collide with)
└── Phase 3: sleep islands, the trees updated for the queries, then the events
```

## Collision detection
| Pair | Method |
|------|--------|
| any shape - plane | `CollideWithPlane` of the shape |
| any shape - heightfield | each triangle under the body: GJK + EPA, inner edges, patches (up to 8 manifolds) |
| sphere / capsule - sphere / capsule | closest points of the segments (a sphere is a segment of length 0) |
| sphere / capsule - other shape | GJK distance between the core (a point, a segment) and the shape, plus the radius; EPA only if the core is inside |
| other pairs | GJK + EPA, then clipping of the contact points |

Pair cache (the body pair cache of Jolt): if a body moved less than 1 mm and 2° relative to the other since their contact
points were computed, the previous contact points are moved with the bodies, the collision detection doesn't run again.
The contacts of a pair are kept by its pair in the broad phase: no lookup.

Contacts are kept up to a margin: `SpeculativeDistance` (2 cm), + the relative speed of the bodies * dt against a
static body.
Each manifold has a normal (from A to B) and up to 4 points. A pair has 1 manifold, up to 8 against a heightfield
(the manifolds of a pair follow each other). Each point has its own separation (< 0 when the bodies overlap).

## Collision filtering
`filter.go`. A pair is emitted by the broad phase (`Tree.scan`) only if it passes 2 filters, in this order:
1. **the layers**: `actor.RigidBody.Layer` (one bit among 32) and `Mask`. `ShouldCollide(a, b)`: each body has its
   layer in the mask of the other. `NewRigidBody` gives `LayerDefault` and `AllLayers`: everything collides.
2. **the pairs which never collide**: one table (`pairFilter`, a map keyed by the pair of bodies, O(1) per pair), with
   the 2 reasons a pair can have: ignored by the game (`World.IgnoreCollision`), or linked by joints without
   `CollideConnected` (their count). A pair leaves the table with its last reason; a removed body leaves it with its
   pairs. The table is read by the workers of the broad phase, and only written between 2 steps.

The filters are read at each step: a change of `Layer`, `Mask` or of the table applies at the next step, the pair stays
in the records of the broad phase (its stored AABBs still overlap) and is emitted again when the filter lets it.
`World.SetFilter` and `World.IgnoreCollision` also wake the bodies up.

| Where | What the filter does |
|-------|----------------------|
| broad phase | a filtered pair is not emitted: no narrow phase, no manifold, no contact |
| triggers | a trigger is a body like the others in the broad phase: a filtered pair sends no trigger event |
| continuous collision | `stopAtImpact` skips the bodies the fast body doesn't collide with (`World.ShouldCollide`) |
| sleep | a filtered pair has no contact: it wakes nobody up and links no island. `RemoveBody` and `UpdateHeightfield` only wake the sleeping bodies which collide with the body |
| queries | `QueryFilter{Mask, Excluded}.Accepts(body)`: the layer of the body is in the mask of the query, and the body is not excluded. The mask of the body is not read |

A body of empty mask (`actor.NoLayers`) collides with nothing but keeps its leaf in the tree, with its layer: the
"queries only" bodies.

## Queries
`World.Raycast`, `RaycastAll`, `Sweep` and `Overlap` read the trees of the broad phase; see
[ALGORITHMS.md](ALGORITHMS.md#queries).

| Query | Planes & heightfields (in no tree) | Trees (static, then dynamic) | A body |
|---|---|---|---|
| `Raycast`, `RaycastAll` | each one, first | the segment against the AABB of the nodes, the closest child first | `CastRay` of its shape, in its local space |
| `Sweep` | each one, first | the same, the AABBs enlarged by the half sizes of the shape | conservative advancement on the cores (GJK distance) |
| `Overlap` | each one | `Tree.queryCandidates` with the AABB of the shape | GJK distance of the cores against the radii |

**The trees are up to date between 2 steps.** A step updates them at its start, for its pairs, and once more at its
end, after the continuous collision and the sleep: the AABB stored for each body then contains its AABB. The end of a
step only reads the bodies the step moved (the bodies of its solver: awake when it started, or woken by it): a static
body or a sleeping body costs nothing there, a world asleep pays nothing. What the game
writes between 2 steps (a body added, a transform or a shape written, followed by `UpdateAABB`) is taken by
`World.SyncQueries`, or by the next step. `RemoveBody` updates the trees at once. All of them give the trees the same
AABBs (enlarged by the distance the body can travel in a step): a body which didn't change is never put back in its
tree, `SyncQueries` on such a world only costs a test per body (about 30 ns).

**Concurrency: several readers or one writer.** A query writes no field of the world, of a tree, of a body nor of a
shape: its buffers are on its stack, or come from a `sync.Pool`. Any number of goroutines can run queries at the same
time, as long as nothing writes the world: `Step`, `SyncQueries`, `AddBody`, `RemoveBody`, `AddJoint`, `RemoveJoint`,
`SetFilter`, `IgnoreCollision`, `UpdateHeightfield`, a write to a body or to the heights of a terrain. Feather takes
no lock: the caller orders the queries and the writes (Jolt offers both, its queries without lock are "use with great
care"). A query started while `Step` runs panics with `feather: query during Step` (one atomic read per query; Box3D
refuses a locked world the same way). The listeners of the events run after the trees are updated and the guard is
lifted, on the goroutine of `Step`: they can run queries, and see the end of the step.

## Solver
See [ALGORITHMS.md](ALGORITHMS.md#solver). The solver works on copies of the dynamic bodies (`bodyState`):
the static and sleeping bodies share a state with no mass.

The axis locks (`actor.Axes`, `RigidBody.LinearLock` & `AngularLock`) live in the state too: its inverse mass by axis
is null along the locked axes, its inverse inertia in world space is the one of the body held around them (`lock.go`, see
[ALGORITHMS.md](ALGORITHMS.md#axis-locks)). The contacts and the joints read them as the mass of the body: nothing
else knows about the locks, apart from the integration (gravity, gyroscopic torque) and the continuous collision.

## Threading & determinism
- From 256 bodies, a step runs on `Workers` goroutines. The workers are created once and sleep between the steps.
  `World.Close()` stops them (they are also stopped when the World is garbage collected).
- The broad phase, the narrow phase, the preparation of the contacts and the integration of the bodies:
  each body, pair or contact writes its result at its own index, the order of execution doesn't matter.
- The order of the bodies is the one of `World.Bodies`: `AddBody` puts a body last, `RemoveBody` moves the following
  bodies up by one (the trees, the pairs and the contacts follow, `Tree.removed`), a body added again is last whatever
  its former index. The same bodies added and removed in the same order give the same result.
- The pairs are sorted by the indices of their bodies (`Tree.sortPairs`: the first body by a counting sort, then the few
  pairs of a body, its planes first, by an insertion sort). The pair search already gives them in the same order whatever
  the workers (its chunks are read in their order): the sort makes it the order of a search from scratch, whatever the
  history of the trees, and keeps the contacts of a body next to each other for the solver. It costs 4.4 ns per pair on
  one goroutine: 3.8 µs for the 856 pairs of 500 bodies landing, 15 µs for the 3471 pairs of 2000 bodies, 0.1 to 0.2 %
  of their step with 1 worker and 0.4 to 1 % with 8 (`go test -run xxx -bench SortPairs`, on a Ryzen 7 5800X).
  Box2D states the rule (`contact.c` in v3.1, "Contacts and determinism": the contacts must exist in the same order
  whatever the thread count, the Gauss-Seidel solver is order dependent) and now sorts the keys of its new pairs
  (`b2UpdateBroadPhasePairs`, `broad_phase.c`, read on 02/10/2026: "Pairs arrive in deterministic order but scrambled
  relative to body and shape order, sorting them here improves solver performance"); Jolt sorts the constraints and the
  contacts of each island (`PhysicsSettings::mDeterministicSimulation`, on by default in 5.5.0: off, it runs "faster
  but it will no longer be deterministic").
- The solver is a Gauss-Seidel: a constraint uses the result of the previous one. The contacts and the joints are
  colored (like Box2D v3): the constraints of a color don't share any dynamic body, so a color is solved in parallel.
  The colors are always solved in the same order: the result is the same bit for bit, whatever the number of workers.
- It is tested (`determinism_test.go`): the bits of the bodies and of the contacts after each of the 1000 steps of 200
  bodies (a pile bouncing on chains and on linked boxes), and of a pile of 600 bodies, are the same between two runs and
  with 1, 4 and 8 workers, also when bodies are removed and added again during the run. Each scene must reach the
  parallel path of every stage, or its test fails. The result is the same on a given GOARCH only: Go may fuse
  `x*y + z` into a single rounding on some architectures.
- A step doesn't allocate memory after the first steps: the buffers are reused.

## Tests & benchmarks
**The reference is Box3D** (Erin Catto, 2026): Feather must do at least as well on the same scenes at 60 Hz, Feather
with 8 substeps (its setting for the games), Box3D with its default 4. The scenes come from Solver2D, extruded by 1 m in
3D (the same supports, the same mass ratios). The drivers of Box3D and Jolt on the same scenes are kept outside the
repository; the values of Box3D are written in the tests with their date. The known gaps are
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
4. **Regressions** (`bench/regression.go`): 6 chaotic scenes, spinning tops, locked bodies, 2 scenes of queries
   (`bench/queries.go`: rays, sweeps and overlaps among 1000 bodies and on a terrain of 1025 x 1025 samples; their
   fingerprint is the hash of their results, their phases are their batches of 10000 queries) and the scenes of
   Solver2D, compared to `bench/baseline.json`:
   - the fingerprint of the final state (identical on the same GOARCH);
   - quality metrics, 0.5 mm of tolerance on a depth, 0.1 % on an energy gain;
   - the draws: 11 measures of 8 scenes are decided by the rounding (the depth of a pile, the speed left in the ball of
     "rush", the worst gap of the net, the far deviation of the pyramid...). Pushing every body by 1 µm/s moves them by
     more than their tolerance: any change of the engine draws them again, one value says nothing. Each is followed by
     its median over 32 variants of its scene (other seeds for the piles; a drop height, a radius, a push, a gravity or
     a gap across a finite range for the others), with a tolerance of 3 standard deviations of that median, measured in
     the ensemble (from the median to its upper quartile, so a few variants far in the tail don't widen it) and never
     under the tolerance of the unit. The worst variant is written in the reference to be read, it is not compared. The
     variants are not timed: they run in parallel after the timed scenes (`-check` takes about 80 s);
   - the time of a step (+20 %) and of its phases (+30 %, over 5 % of the step), each the best of 3 runs, only on the
     machine of the reference, and for the steps over 0.1 ms (under it, the noise of the timer dominates).

   The 3 runs start at 3 depths of the stack, a third of a page of 4 KiB apart (`bench/stack.go`). The time of a phase
   depends on where the stack lies in its page: `Tree.scan` copies AABBs on the stack, and at a few depths a write of
   16 bytes straddles two pages (the broad phase of "solver2d confined": 0.084 ms instead of 0.046, 10 depths out of
   512). The depth comes from the size of every frame above: 8 bytes more in a frame of the engine or of the bench
   doubled the time of this phase with the same code. With the best run of each phase over the 3 depths, the 512
   depths give 0.043 to 0.049 ms. `go run . -check -stack 2936` moves the stack of the scenes in its page (in bytes):
   the result must stay the same.

The queries are tested against a brute force (`query_ray_test.go`, `query_sweep_test.go`, `query_overlap_test.go`):
on 300 bodies of every shape and kind, a ray, a sweep and an overlap give what a walk of `World.Bodies` in their order
gives; the gap a sweep leaves is measured without the engine (the distance from a point to each shape is analytic).
`cd bench && go run . -queries` prints the cost of a query of each kind.

`World.Profile()` gives the time of each phase of the last step (broad phase, narrow phase, prepare, substeps,
restitution, continuous collision, islands), without allocation.
`World.parallelFrom` (tests only) runs the parallel paths under 256 bodies, for the determinism.

## Current limitations
- The broad phase is a pair of dynamic AABB trees (static and dynamic bodies), the dynamic AABBs enlarged by a margin:
  a sleeping body costs nothing (the planes & the heightfields are not in the trees, they are tested with every awake body).
- A heightfield is a surface: a body entirely under it is not pushed up.
- The contacts are computed once per step: on a rough terrain, a corner of a tumbling body can slide over another
  triangle during the step, and sink by a few mm before the next step.
- The friction around the normal comes from the lever arms of the points: a ball spinning on itself on its single point
  of contact never stops (no sleep), unless its material has a `SpinningResistance` (0 by default).
- A capsule resting across a bump of a terrain can stay a few mm in the terrain: the contact of a triangle comes from
  the feature of the body above the triangle, the middle of the capsule is missed.
- The restitution is applied once per step, with the approach velocity of the impact: a body not round (box,
  capsule), bouncy (`e` over 0.5) and spinning fast (10-20 rad/s) can bounce higher than it fell. Measured at 60 Hz
  with 8 substeps: up to +21 % of energy at `e = 1`, never up to `e = 0.5`. Jolt documents the same limit.
- No kinematic bodies (moving platforms): a body is static or dynamic.
- The queries test every plane and every heightfield of the world: they are in no tree. A terrain cut in dozens of
  heightfields would need one.
- `SyncQueries` costs a test per body, even for a static body which never moves: Feather doesn't know what the game
  wrote. The update of the trees at the end of a step knows what the step moved, and only reads these bodies: measured
  on a falling pile of 2 000 bodies, it adds 0.02 to 0.04 ms per step to the broad phase (7 to 12 %), and nothing once
  the pile is asleep.
- A sweep doesn't turn the shape, gives its first hit only, and no depth when the shape starts in a body. The back
  side of a heightfield is never hit.
- A fast body which is not a bullet goes through a dynamic body with all its axes locked: the continuous collision of
  the other bodies only looks at the static ones.
- The continuous collision stops the fast bodies against the static bodies (and the bullets against all the bodies),
  not the other pairs: 2 fast dynamic bodies rely on their speculative contacts (2 cm) and on the spring of the contact.
