# Feather - Algorithms

1. [GJK Algorithm](#gjk-algorithm)
2. [EPA Algorithm](#epa-algorithm)
3. [Contact points](#contact-points)
4. [Solver](#solver)
5. [Joints](#joints)
6. [Heightfield](#heightfield)
7. [Continuous collision](#continuous-collision)

## Broad phase

Two dynamic AABB trees (Catto, "Dynamic Bounding Volume Hierarchies", GDC 2019; the `b2DynamicTree` of Box2D, the
`btDbvt` of Bullet), one per kind of body as in Box2D v3: one for the static bodies, one for the dynamic bodies, awake
or asleep. A dynamic body is stored with its AABB enlarged by `AABBMargin` (0.1 m, as Box2D v2.4): while it moves
inside, the tree is not touched; a sleeping body never touches it. A leaf is inserted next to the sibling which
enlarges the tree the least (the surface area heuristic), found down the tree by the least reachable cost; the
ancestors are then refitted and each tries the rotation of a child with a grandchild which shrinks it the most (the
tree rotations of Catto's talk). The planes and the heightfields are not in the trees: they are tested against every
awake body.

The pairs of bodies whose stored AABBs overlap are kept from a step to the next (the persistent pairs of Box2D v3):
only a body put in a tree since the last step (a dynamic body out of its enlarged AABB, a static body moved by the
game, a body added) queries the trees for its pairs, and a pair is dropped when its stored AABBs no longer overlap.
A resting scene costs nothing, an awake one only pays for the bodies which left their enlarged AABB (2000 bodies
settling: 1.6 ms for a traversal of the trees against each other, 0.3 ms with the pairs kept). The pairs of the step
are those whose exact AABBs overlap, with an awake dynamic body, sorted by the index of their first body (a counting
sort): the list is the same as a search from scratch, and the same as the former uniform grid gave, so the solver keeps
its order and its results bit for bit.

## Collision filtering

The filter is the one of the 3 engines Feather follows, reduced to what they share.

**Layers and masks.** A body has a layer (one bit) and a mask; a pair collides if
`a.Mask & b.Layer != 0 && b.Mask & a.Layer != 0`. It is the test of Box2D (`categoryBits` & `maskBits`, `b2Filter`:
`include/box2d/types.h:268-277`; `b2ShouldShapesCollide`: `src/contact.c:469-478`, v3.1.0), of Bullet and of Jolt in its
mask mode ("Two layers can collide if Object1.Group & Object2.Mask is non-zero and Object2.Group & Object1.Mask is
non-zero", `ObjectLayerPairFilterMask.h:12` and `:45-49`, v5.3.0). PhysX leaves the test to the game (4 words of
`PxFilterData`, `PxFiltering.h:365`, read by a filter shader, `:594`); its sample shader does a group table and masks
(`ExtDefaultSimulationFilterShader.cpp:238-274`, 5.6.0). 32 layers, as the layers of Unity and the words of PhysX.

**Not taken.** The signed group of Box2D (`groupIndex`, `types.h:279-285`: the same negative group never collides, the
same positive group always does, before the masks): the ignored pairs and the joints do its job between 2 bodies, the
layers between families, and a third rule which overrides the masks makes a filter hard to read. The broad phase layers
of Jolt (`BroadPhaseLayer.h:12-17`: one tree per group of layers): Feather keeps its 2 trees, static and dynamic
(Box2D v3 has one tree per body type, 3 of them: `b2BroadPhase.trees`, `src/broad_phase.h:28`). The group filter of Jolt (`GroupFilter.h:17-26`, `GroupFilterTable.h:96-116`: a bit table of the sub groups
of a ragdoll, where `Ragdoll.cpp:192-204` disables each parent with its child): the table of pairs below gives the same
result without a group per ragdoll.

**Pairs which never collide.** One table of pairs of bodies, a map keyed by both pointers: O(1) per pair, and skipped
when it is empty. A pair is in it for 2 reasons, kept together in its entry:
- the game ignores it (`World.IgnoreCollision(a, b, ignore)`: `Physics.IgnoreCollision(collider1, collider2, ignore)`
  of Unity; Box2D v3.1 has a joint for it, the filter joint, `b2CreateFilterJoint`, `src/joint.c:472-496`);
- joints without `CollideConnected` link its bodies (`collideConnected` of the joints of Box2D, read by
  `b2ShouldBodiesCollide`, `src/body.c:1848-1884`, which walks the joints of the body with the fewest joints;
  `PxConstraintFlag::eCOLLISION_ENABLED` of PhysX, `PxConstraint.h:57`). The entry counts them.

A pair leaves the table when it has no reason left. `CollideConnected` is read when the joint is added: the joint
remembers it entered the table, and leaves it the same way.

**Where.** In the broad phase, when the pairs of the step are emitted from the pairs kept (`Tree.scan`), after the
tests of the AABBs and of the awake bodies: a resting pair doesn't reach the filter. Box2D filters when a pair is
created (`b2PairQueryCallback`, `src/broad_phase.c:257` and `:265`) and must then destroy the contacts and reset the
proxy when a filter changes (`b2Shape_SetFilter`, `src/shape.c:1209-1231`); Jolt filters at each step, in the search
of the pairs (`Body::sFindCollidingPairsCanCollide`, `Body.inl:75`). Feather does as Jolt: the filter is 2 masks and,
only when the table is not empty, a hash; in return a filter can change at any time, with nothing to invalidate.

**Triggers.** A trigger is a body like the others in the broad phase: the filter applies before, a filtered pair sends
no trigger event (the sensors of Box2D go through `b2ShouldShapesCollide` too, `src/sensor.c:78`). Unlike Box2D, the
table of pairs applies to the triggers too, as it did in v0.3.0 for the bodies of a joint.

**Continuous collision.** `stopAtImpact` queries the trees itself: it skips the bodies the fast body doesn't collide
with, by the same test (`World.ShouldCollide`), as Box2D (`b2ContinuousQueryCallback`, `src/solver.c:241` and `:260`).

**Queries only.** A body of empty mask collides with nothing, and still has its leaf in the tree with its layer. It is
the "Query Only" collision of Unreal ("This body is used only for spatial queries (raycasts, sweeps, and overlaps). It
cannot be used for simulation (rigid body, constraints)", Collision Response Reference) and a PhysX shape with
`eSCENE_QUERY_SHAPE` without `eSIMULATION_SHAPE` (`PxShape.h:80-85`). No flag is needed: it follows from the masks.

**Filter of the queries.** `QueryFilter`: a mask of layers and a few excluded bodies, as the `b2QueryFilter` of Box2D
(`types.h:296-304`) and the `PxQueryFilterData` of PhysX (`PxQueryFiltering.h:134-147`). `Accepts(body)` reads the layer
of the body, not its mask. It is a deviation, on purpose: Box2D tests both ways, the mask of the shape against the
category of the query too (`src/world.c:2083`), and PhysX compares the query to a word of the shape kept apart from
its simulation filter (`PxQueryFiltering.h:125-126`). In Feather the mask of a body only says what it collides with:
a body of empty mask is still seen by the queries on its layer, which is what makes the "queries only" bodies. It is
the `layerMask` of `Physics.Raycast` in Unity: the layer of the collider against the mask of the ray.
The excluded bodies are searched one by one, as `IgnoreMultipleBodiesFilter` of Jolt (`BodyFilter.h:55-80`).

## Pair cache and warm start

The contacts of a pair are kept with the pair of the broad phase. If a body moved less than 1 mm and turned less than
2° relative to the other since their contact points were computed, the points are moved with the bodies and their
separations measured again, instead of running GJK/EPA (the body pair cache of Jolt, with its thresholds). The
impulses of the previous step warm start the new points: each point takes the impulses of the closest previous point
in the local space of body A, within 2 cm (as the contact cache of Jolt; Box2D matches the points by feature id).

## GJK Algorithm
GJK tests if two convex shapes overlap: they overlap if their Minkowski difference `A - B` contains the origin.
The shapes only need a `Support(direction)` function, the farthest point in a direction.

````
direction ← center of B - center of A
simplex ← [Support(A - B, direction)]
direction ← -simplex[0]
loop
    point ← Support(A - B, direction)
    if point · direction <= 0 then return false   // the origin can't be reached
    simplex.add(point)
    if simplex contains the origin then return true  // only a tetrahedron can
    simplex ← feature of the simplex closest to the origin, direction ← towards the origin
end
````

- Each vertex keeps its support points on A and B, EPA uses them for the witness points.
- With a margin, A is inflated by a sphere: shapes closer than the margin overlap. EPA then gives `depth = margin - distance`.
- When the shapes are only touching (the origin on the simplex), the simplex is completed into a tetrahedron for EPA.

## EPA Algorithm
EPA starts from the tetrahedron of GJK, and grows it towards the surface of the Minkowski difference:

````
polytope ← tetrahedron of GJK
loop
    face ← face of the polytope closest to the origin
    point ← Support(A - B, face.normal)
    if point · face.normal - face.distance < tolerance then
        return face.normal, face.distance, witness points
    remove the faces visible from point, close the hole with new faces from the horizon to point
end
````

- The tolerance is 1e-7 m: it is the error on the penetration depth.
- **Ties**: the faces less deep than the closest one by less than 1 µm are as deep (a cube overlapping 2 others by the
  same amount). They all converge, then EPA takes the first in a fixed order in the local space of A, not the one the
  rounding found first: a scene moved by 1 µm or 100 km keeps the same normals, and the same motion. The triangles of a
  same feature (normals within 1°: a face, a rounded surface) are not tied, the closest is kept.
- The witness points come from the barycentric coordinates of the origin projected on the closest face.
- Tested against exact solutions: SAT for box-box, closest point for sphere-box (see `epa/epa_test.go`).

## Contact points
From the normal of EPA, each body gives the feature facing the other body (a face for a box, a line or a point for a capsule):

- **Face contact**: a face is aligned with the normal (0.5°). The other feature is clipped by the side planes of this face (Sutherland-Hodgman).
- **Parallel edges**: a box on an edge, a capsule along an edge. One edge is clipped by the other: 2 points.
- **Otherwise** (crossing edges, a vertex, a sphere): the witness point of EPA.

The deepest point has the separation of EPA, the other points are higher along the normal.
The points closer than the margin are kept, 4 at most: the deepest, the farthest from it, then the points adding the most
area. A point replaces the best one only if it is clearly better, deeper by 1 µm or a score higher by 1/0.95 (the pecking
order of Box3D): the choice between points as good doesn't depend on the rounding.
A box touches a plane (or the face of a triangle) with its supporting face, the face the most opposed to the normal
(as the incident face of Jolt): the corners behind it are never candidates. The contacts are reduced the same way.

Spheres and capsules don't use EPA: their contact comes from the closest points of their segments (Ericson 5.1.9).
Parallel capsules get 2 points.
Against the other shapes, a rounded shape is its **core** with a radius (the collision margin of Bullet, the convex
radius of Jolt, the rounded polygons of Box2D v3): a point
for a sphere, a segment for a capsule. GJK gives the distance and the closest points of the cores (exact against a
polytope, in 3 or 4 iterations), the radii and the margin are added along their direction, and the contact points are
clipped as above. EPA runs on the full shapes only if the cores overlap (the center of a sphere inside a box): on the
rounded shape, it would tessellate it (13 iterations and 7 µs for a sphere against a box, against 1.8 µs).

## Solver
TGS Soft, from Box2D v3 (Erin Catto, [Solver2D](https://box2d.org/posts/2024/02/solver2d/)), in 3D.

### Prepare (once per step)
For each contact point: the anchors `rA`, `rB` (from the centers of mass), the effective masses along the normal and both tangents,
the impulses of the previous step (warm starting), and the relative normal velocity (for the restitution).

### Substeps
````
for numSubsteps do
    IntegrateVelocities();   // gravity, forces, gyroscopic torque, damping
    WarmStart();             // apply the accumulated impulses
    Push();                  // soft constraint
    IntegratePositions();
    Relax();                 // rigid constraint, then friction
end
Restitution();
````

The contact points are not computed again during the substeps: the separation is updated from the motion of both anchors.
````
separation = baseSeparation + (ΔpB + ΔqB*coreB - ΔpA - ΔqA*coreA) · normal
````
The anchors are the points on the surface of each body, without the radius of the rounded shapes
(`core = surface ± radius * normal`): the center of a sphere, the axis of a capsule, the corner of a box.
A rolling sphere turns its surface, not its center: with the point of its surface, its contacts would open while it
rolls, and it would sink in the wall in front of it.

The rotation of a body is limited to `MaxRotation` (π/4) per substep. Box2D limits it per step (with an option for the
wheels), and keeps the anchors fixed during the step. Feather lets the bodies turn faster, so the lever arms of the
contacts turn with the bodies before each `Relax` (`turnAnchors`), when a body turned more than 0.01 rad since the
beginning of the step: a tumbling capsule at 30 rad/s turns by 0.5 rad per step, it would otherwise be pushed at the place
of its contact at the beginning of the step, and the solver would see the contact open while it sinks.
The inertia turns with the body too (`I⁻¹ = ΔR I⁻¹start ΔRᵀ`).
To our knowledge, the cores (the idea of the convex radius of Bullet & Jolt, applied to the separation) and `turnAnchors`
are Feather's own: without them, the bodies tumbling on a slope sink by 14 cm (`TestPileLandsWithoutSinking`).

### Soft constraint
From Erin Catto, [Soft Constraints](https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf) (GDC 2011): the
overlap is a spring of frequency `ω = 2π * hertz` and damping ratio `ζ`, whatever the mass (`k = m ω²`, `c = 2 m ζ ω`),
integrated implicitly over the substep `h`:
````
biasRate = k / (c + h k) = ω / (2ζ + h ω)
gamma    = m / (h (c + h k)) = 1 / (h ω (2ζ + h ω))       // the softness γ of the paper, times the mass

Push:
    if separation > 0 then bias = separation / h           // speculative: can get closer, not further than the gap
    else bias = max(biasRate * separation, -ContactSpeed)
    λ = -(m (vn + bias) + gamma * λ_total) / (1 + gamma)
    λ_total = max(λ_total + λ, 0)
````
`Relax` solves the same constraint rigid (`gamma = 0`, bias only for the speculative contacts): the spring adds energy.
The joints use the same soft rows. The contacts with a static body are twice as stiff, with half the damping ratio (as
Box3D).
The friction is solved in `Relax` only, after the normals (as Box3D): solved in `Push` before the normals, it pushed the
light bodies out from under a heavy one.

### Block solver
The points of a contact are solved together, exactly: their accumulated impulses are the solution of the linear
complementarity problem `w = (K + D) λ + r`, `λ ≥ 0`, `w ≥ 0`, `λ w = 0`, found by enumerating the sets of active
points, all of them first (the block solver of Box2D v2, for 4 points). `K` is the mass matrix of the points, `D` their
softness (`gamma K_ii`, the fixed point of the soft row), `r = vn + bias - K λ_total`.
Solved one after the other (Gauss-Seidel, as PhysX, Jolt & Box3D), the first point takes more than its share and turns
the body: a box landing flat on another starts to spin, and lands back on a corner (Box3D: 30 cm of drift).
4 rigid points on a face give 3 independent rows only: the share of the load between the points is not defined. A
proximal term `ε W (λ - λ₀)` (`W` the diagonal of `K`, `ε = 1e-3`) chooses the share closest to the impulses the rows
start from (the proximal point method: Rockafellar 1976; the proximal formulations of contact: Alart & Curnier 1991,
Acary & Brogliato 2008). Repeated at each pass, its bias vanishes. A term towards 0 (the solution of minimum norm) moved
the load between the points at each substep (30 % of it), and a house of cards fell.

### Friction
Solved in `Relax`, after the normals, at the friction center of the points of the contact (as Box3D, Jolt, and the
friction patches of PhysX), not at each point:
- along both tangents, with their 2x2 mass matrix: the impulse stays in a disk of radius `µ Σ λ_normal`;
- around the normal (the twist): up to `µ Σ (lever arm × λ_normal)`, the lever arm of a point being its distance to the
  center. A single point holds no twist.

The center is the average of the points, weighted by their separation (as Box3D: 1 up to the speculative distance, 0 at
twice). µ is the static friction when the center slides slower than 1 cm/s, the dynamic friction otherwise.

### Restitution
Applied once after the substeps, for the contacts hitting faster than 1 m/s, once their compression is over (the
point doesn't approach anymore). The bounce impulse goes towards the velocity `-e * vn_impact` (Newton), and is at most
`e` times the impulse of the compression (Poisson's hypothesis, W. J. Stronge, Impact Mechanics):
````
λ = max(0, min(-m (vn + e * vn_impact), e * λ_compression))
````
An impact starting at the very end of a step is compressed over 2 steps: the point keeps its approach velocity and its
compression impulse (`ImpactVelocity`, `CompressionImpulse`, warm started like the impulses) and bounces at the end of
the second step, with the whole impulse. Bounced at the end of the first step, a ball dropped from 1 m at 60 Hz gave
back 10 % of its height instead of 92 % (`TestBounceRestitution`, at every rate).
Both are needed: a pile of balls bouncing with `e = 1` gains energy with Newton alone (411 J) or Poisson alone
(3523 J), not with both (`TestRestitutionNeverAddsEnergy`). The bounce uses the approach velocity of the impact: a body not
round and spinning fast can turn its point away before the end of the step, and bounce higher than it fell over
`e = 0.5` (see ARCHITECTURE.md, as documented by Jolt). Newton's law is the one of the game engines (Box2D, Jolt).

### Gyroscopic torque
`ω × Iω` is integrated implicitly (1 Newton-Raphson iteration in body space), as described by Erin Catto
([GDC 2015](https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf)). Dropping it removes the tumbling
of long bodies, integrating it explicitly makes them gain energy.

### Parallel solver
The solver is a Gauss-Seidel: each constraint uses the velocities left by the previous one. To solve in parallel,
the contacts and the joints are colored (Box2D v3, `constraint_graph.c`): each takes the first color where both of its
dynamic bodies are free (the static bodies don't count). The constraints of a color don't share any body, the workers
solve them at the same time. The constraints without a free color (16 colors) are solved first, on a single goroutine.
A contact with a static body never takes the color 0 (as Box2D v3): it is solved after the contacts between dynamic
bodies, the ground has the last word. Solved first, a light body pressed by a heavy one leaves the step moving into
the ground.

### Default values
| Constant | Value |
|----------|-------|
| `DefaultContactHertz` | 30 Hz, as Box2D v3.1 (x2 against static bodies, capped to 1/8 of the substeps rate) |
| `ContactDampingRatio` | 10 |
| `ContactSpeed` | 3 m/s |
| `RestitutionThreshold` | 1 m/s |
| `SpeculativeDistance` | 2 cm |
| `LinearSlop` | 5 mm |
| `MaxRotation` | π/4 per substep |

## Joints
The joints are solved like the contacts (as in Box2D v3): warm starting, soft constraints in `Push` (60 Hz, damping ratio 2
by default), rigid constraints in `Relax`. They are colored with the contacts and solved with them, in parallel; the
articulations below are solved before the colors, on a single goroutine.

### Articulations
The point constraints (the anchors kept together) of the joints linking dynamic bodies are solved together, exactly, by
tree of joints: `K Δλ = -(ċ + bias)`, `K = J M⁻¹ Jᵀ` the mass matrix of all their anchors, factored from the leaves to the
root (block `LDLᵀ`, the linear time dynamics of Baraff, "Linear-Time Dynamics using Lagrange Multipliers", SIGGRAPH
1996). A joint is eliminated after the joints below it: a chain gives no fill, a body with `d` children a clique of `d`
blocks. The soft spring acts on the whole system, `Δλ = -(K⁻¹ (ċ + bias) + γ λ) / (1 + γ)`.
Solved one by one, the spring of a joint acts on the mass of its own bodies: a ball 670 times heavier than a link
stretched each joint by 7 cm (a chain of 20 m by 1.4 m). Solved together, 0.1 mm (`TestHeavyChainDoesNotStretch`).
- A chain taut between 2 fixed points has a redundant row: the proximal term of the contacts keeps `K` invertible.
- The joints of a net (a loop between dynamic bodies) are solved one by one, as the other rows of the joints (the axes
  of the hinges, the limits, the motors, the springs): the tree exact against the joints closing the loops solved alone
  converges slowly (a net of 60 x 60 opened by 357 mm instead of 320).
- A link turning by half a radian in a substep (the tip of a whip, 125 rad/s) opens its joint by `r (ω h)² / 2` for a few
  steps: the constraints are linear in the velocities.

Each joint has a frame on each body. The X axis of the frames is the axis of the hinge, and the twist axis of the ball
joint (as in PhysX).

- **Point** (ball, hinge, fixed): the anchors stay together, 3 rows solved together:
  `K = (mA + mB) I - [rA]x IA [rA]x - [rB]x IB [rB]x`
- **Hinge axis**: the X axis of B stays on the X axis of A: 2 rows along the Y & Z axes of A, the error is `xA × xB`.
- **Angle limits** (hinge, twist): like the contacts, speculative above the limit, soft under it.
- **Cone** (ball): the X axis of B seen in the frame A, `p`, must stay in the elliptic cone of the 2 half angles.
  `p` moves as `dp/dt = ω × p`: the rate of the cone function `f(p)` is `ω · (p × ∇f)`, so the constraint turns around
  `p × ∇f`. This axis is orthogonal to `p`: it doesn't twist B, and it also follows the cone when B slides along its
  border (the limit changes with the direction).
- **Twist** (ball): swing-twist decomposition of the rotation of B relative to A. The rate of the twist isn't exactly
  `ω · x`: it is measured numerically, to follow the twist when B also swings.
- **Distance**: 1 row along the axis between the anchors, rigid, or a spring with the limits.
- **Fixed & drive**: 3 angular rows, `K = IA + IB`, the error is the rotation vector between the frames.
- **Configurable**: each axis chooses its row. Linear: 1 row along the axis of A (locked, or 2 limits), all locked = the
  point. Angular: the twist row, the cone if both swings are limited, else 1 row per swing
  (`atan2(-p.z, p.x)` around Y, `atan2(p.y, p.x)` around Z), all locked = the 3 angular rows.

## Heightfield
The terrain is a grid of heights, split in 2 triangles per cell (along the diagonal from (x, z) to (x+1, z+1)).
The body is tested against the triangles under its AABB, one by one: the grid is cut in blocks of 16x16 cells with
their lowest and highest heights, to skip the blocks far from the body.

Each triangle is tested with GJK/EPA:
- **Face**: always, the contact of the body with the plane of the triangle (`CollideWithPlane`), limited to the
  points above the triangle. On a flat terrain, a body behaves exactly as on a plane.
- **Inner edges**: a body sliding on the terrain must not hit the edges between the triangles. Each edge is active if it
  is on a border or a hole, or if it bends down (convex) by more than 5° (like Jolt, `ActiveEdges.h`). A contact on an inactive
  edge (or vertex) takes the normal of its triangle. If the body is beside the triangle, above the edge, it keeps the
  witness point of EPA, only if no other triangle has a contact with this normal.
- **Active edges** (a ridge, a border): also the contact of EPA, with its normal. A capsule lying across a ridge touches
  the ridge, and its ends can fall on both faces.

The contacts are then grouped by normal: the contacts of triangles with less than 5° between their normals form a patch,
a manifold of 4 points. A body touches the terrain with 8 patches at most (`MaxManifoldsPerPair`): a box in a valley
gets one patch per slope. The patches of the deepest contacts are kept, the others are dropped (like Jolt).

## Continuous collision
**Speculative contacts**: against a static body, the contacts are created up to `SpeculativeDistance` + the relative
speed of the bodies * dt (the speculative CCD of PhysX, the "Continuous Speculative" mode of Unity): the solver stops the
bodies before they touch. Between 2 dynamic bodies, only up to `SpeculativeDistance` (as Box2D v3): a fast impact is
absorbed by the spring of the contact over a few substeps. A rigid stop in one substep throws the light body of a
sandwich (a heavy body falling on a light one resting on the ground) and turns both bodies. Their known limits: a contact can be found by a body which will not touch it (a ghost contact), and a body
accelerated by the solver during the step can go further than its margin.

**Time of impact** (as in Box2D v3): after the solver, a body which moved more than half of its smallest extent is moved
back to its first impact with a static body (a plane, a terrain...) along its motion, its velocity is kept. A bullet
(`IsBullet`) is also stopped by the dynamic bodies. The time of impact is found by conservative advancement
(Mirtich, as in Bullet): the body moves forward by its distance to the other body (GJK) divided by the fastest approach
of its points, until it is `LinearSlop` away. If it already touches at the start, only its core (a sphere of 1/4 of its
smallest extent, as in Box2D) is stopped.

**GJK distance**: the distance and the closest points of 2 convex shapes. The simplex is reduced to its feature closest
to the origin (Voronoi regions, Ericson 5.1 & 9.5).

A sleeping body touched by an awake body wakes up with its island during the collision detection, and gets its contacts
in the same step (like Jolt).
