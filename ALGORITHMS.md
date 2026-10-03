# Feather - Algorithms

1. [GJK Algorithm](#gjk-algorithm)
2. [EPA Algorithm](#epa-algorithm)
3. [Contact points](#contact-points)
4. [Solver](#solver)
5. [Joints](#joints)
6. [Heightfield](#heightfield)
7. [Continuous collision](#continuous-collision)
8. [Queries](#queries)
9. [Kinematic bodies](#kinematic-bodies)

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
A resting scene costs nothing (but for the bodies asleep in a trigger, [Triggers](#triggers)), an awake one only pays
for the bodies which left their enlarged AABB (2000 bodies settling: 1.6 ms for a traversal of the trees against each
other, 0.3 ms with the pairs kept). The pairs of the step are those whose exact AABBs overlap, with an awake dynamic
body, or a trigger and a dynamic or kinematic body awake or asleep ([Triggers](#triggers)), sorted by the index of
their first body (a counting sort), then the pairs of each body by the index of the other body (the sort of the
standard library, `slices.SortFunc`: pdqsort, an insertion sort under 12 elements; the keys of a body are unique, so
the order doesn't depend on the sort): the list is the same as a search from scratch, and the same as the former
uniform grid gave, so the solver keeps its order and its results bit for bit. The pairs of a body were sorted by
insertion up to v0.4.0, quadratic for a body with hundreds of pairs (a zone or a ground box under 400 bodies: their
pairs come from the records nearly in reverse order): 400 crates asleep in a zone cost 341 µs per step with it, 103 µs
with `slices.SortFunc` ([Triggers](#triggers)).

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

## Triggers

`tree.go` (`detectsTrigger`), `world.go` (`overlapTrigger`, `overlapKept`), `event.go`. A pair of a trigger and a body
ends when their shapes no longer overlap, or when one of them leaves the world: never because one of them falls
asleep. Up to v0.4.0 the pair of a static trigger and a body left the broad phase when the body fell asleep, and the
missing pair was taken for an exit (the body entered again when it woke up); a kinematic body never entered a static
trigger.

What the references do:

| Engine | A body asleep in a trigger | Removal |
|--------|----------------------------|---------|
| Box2D v3.1.1 | every sensor queries the trees at each step, "Sensors do not consider sleep" (`docs/simulation.md`, `b2SensorTask`): the overlap holds | `b2SensorEndTouchEvent` "if the sensor or visitor are destroyed" (`types.h`) |
| PhysX | a trigger pair is not tested while both actors sleep, "the overlap state can not change if both objects are sleeping"; it is tested again when an actor is created or its pose set (`ScTriggerInteraction.cpp`, `onActivate`, `onDeactivate`, `PROCESS_THIS_FRAME`) | `eNOTIFY_TOUCH_LOST` when an object is deleted (`PxTriggerPair`, since 3.1.1) |
| Jolt 5.6 | a static sensor loses the contact of a body which falls asleep (`OnContactRemoved`, `ContactListener.h`); an active kinematic or dynamic sensor never sleeps and detects the sleeping bodies (`Body::SetIsSensor`, `Body::UpdateSleepStateInternal`) | `OnContactRemoved` at the next update |
| Godot 4 | `GodotAreaPair3D` keeps its state, tested only when its area moves or its body is active (`godot_step_3d.cpp`); its Jolt module makes every `Area3D` a kinematic sensor, active while it monitors (`JoltArea3D::_get_motion_type`, `_should_sleep`) | `body_exited` when the pair is destroyed |
| Unity | `OnTriggerStay` "while a collider remains touching the trigger", `OnTriggerExit` "when a collider stops touching a trigger" | no `OnTriggerExit` when a collider is destroyed or deactivated |

Feather follows Box2D for the pairs and PhysX for their cost:
- The broad phase emits the pair of a trigger and a body which moves (dynamic or kinematic) whatever the sleep
  (`detectsTrigger`): a static trigger the game moves away from a sleeping body ends the pair, moved over it starts one.
  2 static bodies never pair.
- A kinematic body enters and leaves a static trigger, and a kinematic trigger detects the static and the kinematic
  bodies, without any contact: Box2D v3 tests every sensor against its 3 trees, static, kinematic and dynamic
  (`b2SensorTask`); Jolt: "These sensors will only detect collisions with active Dynamic or Kinematic bodies" for a
  static sensor (`Body.h`), a kinematic sensor detects the static bodies only with `SetCollideKinematicVsNonDynamic`;
  Unity: "A dynamic or kinematic trigger collider collides with any collider type. A static trigger collider collides
  with any dynamic or Kinematic collider" (Manual, Collider types interaction), the rule of Feather. A moved kinematic
  proxy queries the static tree and the planes too: its pairs without a trigger are kept in the records of the broad
  phase, never emitted, as a filtered pair.
- A pair with a trigger has no contact: its narrow phase only tells whether the shapes overlap, by the distance between
  their cores (GJK), not over the sum of their radii, as `World.Overlap` (Box2D: `b2ShapeDistance` with the radii, an
  overlap under `10 * FLT_EPSILON`, `b2SensorQueryCallback`). A body in contact with a trigger is in it. A plane, a
  heightfield or a triangle mesh keeps its real contacts: it has no core for GJK.
- A pair whose bodies both rest (static or asleep) keeps the overlap of its last test, while the AABB of each body is
  the same, bit for bit, since that test (`overlapKept`: the box and the step of its last change in the proxy of each
  body, the overlap in the record of the pair): the overlap can't have changed, as PhysX. A body moved by the game, its
  AABB updated, is tested again. A rotation which keeps the AABB of a body bit for bit is not seen, as the broad phase
  doesn't see it for a static body.
- The events: an enter when the pair starts, a stay at each step while one of the bodies is awake (none while both rest,
  as between 2 sleeping bodies), an exit when it ends. `RemoveBody` ends the pairs of the body: their exits are sent
  with the next events (at the end of the next step, or in the flush running if a listener removed the body), as Box2D,
  PhysX and Jolt.

**The contacts follow the same rules.** A contact whose bodies both rest, a body asleep on a static body (the ground,
a plane) or on another sleeping body, is no longer detected but its bodies still touch: the events keep it, with no
stay, and its exit comes when the bodies stop touching or one of them is removed. Up to v0.4.0 only a contact between 2
sleeping bodies was kept: a crate falling asleep on the ground got a `CollisionExit`, and a `CollisionEnter` when it
woke up. Box2D v3 keeps the touching contacts of an island which falls asleep in its sleeping set (`b2TrySleepIsland`,
`solver_set.c`) and sends `b2ContactEndTouchEvent` when a touching contact is destroyed (`b2DestroyContact`,
`contact.c`); PhysX sends `eNOTIFY_TOUCH_LOST` when an actor is removed; Unity: "Collision stay events are not sent for
sleeping Rigidbodies". Jolt alone documents the other way: "as soon as a body goes to sleep the contacts between that
body and all other bodies will receive an OnContactRemoved callback" (`ContactListener.h`). A pair of the events keeps
its kind (contact or trigger): a body made a trigger by the game ends its contacts and starts its trigger pairs.

Cost of a step, 1 worker, crates asleep in a static zone of 20 x 20 m and listened to (µs, median of 5, machine at
rest):

| Crates | With the overlap kept | A GJK per pair at each step | The manifolds of the former narrow phase | Before (out of the zone) |
|--------|-----------------------|-----------------------------|------------------------------------------|--------------------------|
| 25 | 8 | 26 | 91 | 4 |
| 100 | 29 | 104 | 363 | 8 |
| 400 | 120 | 429 | 1463 | 25 |

Before, the sleeping crates were out of the broad phase, and out of the zone too; their contact with the ground left
the events when they fell asleep. With the overlap kept, a crate at rest costs about 0.24 µs per step: its trigger
pair, its place in the pairs of the step (the scan of its record, the sort), `overlapKept`, and in the events its
trigger pair and its contact with the ground, kept, and their comparison with the previous step (0.04 µs of it for
the contact). The pairs of the zone sorted by insertion, as up to v0.4.0, cost 341 µs at 400 crates instead of 103
without the kept contacts. PhysX costs nothing for a pair whose actors both sleep (the interaction is deactivated),
Box2D a `b2ShapeDistance` per pair at each step (the column of a GJK per pair).

The kinematic bodies: 64 kinematic cubes going round over a floor of 400 static tiles (1 worker, median of 5), 95 µs
per step before and 96 µs with their pairs with the static bodies kept in the records for the triggers; with 16
static zones on the floor, 95 µs before (no kinematic cube entered them) and 118 µs: the cubes in the zones are tested
at each step and send their events.

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
To our knowledge, the cores (the idea of the collision margin of Bullet & of the convex radius of Jolt, applied to the
separation) and `turnAnchors` are Feather's own: without them, the bodies tumbling on a slope sink by 14 cm
(`TestPileLandsWithoutSinking`).

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

The lever arm is measured between two points of the same kind: the point and the center, both on the surface of A (the
average is the one of the points on the surface of A). A contact point is halfway between both surfaces: measured from
it to the center on the surface of A, the lever arm of a single point was half its separation instead of 0, and a ball
spinning on the ground lost 0.068 rad/s² without any resistance. A single point (a ball, the cap of a capsule, the
corner of a box) is its own center: its lever arm is exactly 0, whatever the rounding of its weight, and its twist
impulse stays 0 (`TestSinglePointHasNoLeverArm`, `TestTopKeepsItsSpinWithoutSpinningResistance`: 20 rad/s kept after
10 s). A box spinning flat on the ground, its 4 corners at the distance `d` of the center, brakes at `µ d m g / I`
(`TestSpinningBoxBrakesByItsLeverArms`).

Box3D measures the same way, between the anchor of the point and the center of the anchors
(`src/contact_solver.c:241-256`, the bound `:601` & `:615`, commit 9f998c8): its anchors are the contact points
(`src/contact.c:578-579`), Feather takes the points of the surface of A, where its friction center already is. Both
give 0 for a single point; for several points, they differ along the normal only (the half separations between the
contact points and the surface of A).
Jolt (v5.3.0) and Box2D (v3.1.0) have no friction center: the friction is solved at each contact point, bounded by µ
times the normal impulse of the point (`ContactConstraintManager.cpp:57-59`, `:140-141`, `:1600`;
`src/contact_solver.c:133-148`, `:348`), so a single point holds no twist by construction, and several points hold it
by their own lever arms.

### Rolling & spinning resistance
A sphere or a capsule touches by a single point: no lever arm, the friction holds neither its rolling nor its spin
around the normal. Two angular rows are added to the contact, solved in `Relax` after the normals and before the
friction, each warm started from its own impulse (`Manifold.RollingImpulse`, a vector along the tangents, and
`Manifold.SpinningImpulse`, a scalar around the normal: the same pair as `FrictionImpulse` & `TwistImpulse`), each
bounded by a length times the normal impulse of the contact:
- **rolling** (`solveRolling`), around both tangents: `|λ| ≤ RollingResistance * radius * Σ λ_normal`, the rolling
  resistance of Box2D v3 (`src/contact.c:497-502`, `src/contact_solver.c:361-367`, v3.1.0);
- **spinning** (`solveSpinning`), around the normal, after the rolling: `|λ| ≤ SpinningResistance * radius * Σ λ_normal`,
  with the mass of the twist `1 / (n · (IA⁻¹ + IB⁻¹) n)`.

`radius` is the largest radius of both shapes (0 for a box), the resistance the largest of both materials. The bounds
don't read the friction µ of the bodies: a ball without friction (µ = 0) brakes its spin at the same rate (12.26 rad/s²
measured, the case below). The largest radius wins, as in Box2D: a ball of radius 10 cm spinning on a static ball of
radius 1 m takes the radius of 1 m, and brakes 10 times faster than on a plane (122.7 rad/s² measured, 12.3 on the
plane), where the contact patch of Hertz follows the smallest radius. The rolling resistance has the same bias.

The spinning row is the spinning friction of Bullet (3.25): a row around the normal without linear part
(`btSequentialImpulseConstraintSolver.cpp:619-680`, `setupTorsionalFrictionConstraint`), bounded by its coefficient
times the normal impulse (`:1679-1687`), the coefficient set by `btCollisionObject::setSpinningFriction`
(`btCollisionObject.h:89`, `:339`). Feather differs from Bullet on these points:
- the coefficient of Bullet is a length (a torque over a force); Feather gives it as a ratio of the radius, as the
  rolling resistance of Box2D, so that a material fits balls of all sizes;
- Bullet combines `spinningA * µB + spinningB * µA` (`btManifoldResult.cpp:43-45`); Feather takes the largest, as its
  rolling resistance;
- Bullet adds a row per contact point, and only when the rolling friction is also set
  (`btSequentialImpulseConstraintSolver.cpp:1058-1061`); Feather has one row per contact, bounded by the normal impulse
  of all its points, whatever the rolling resistance;
- Bullet starts the row from 0 at each step (`:641`, the contact writes back its normal and both tangents only,
  `:1765-1775`) and caps its bound to the coefficient itself (`:1683-1684`); Feather warm starts it, without this cap.

The other engines: Box3D resists around the 3 axes with its rolling resistance (a vector impulse with the mass
`(IA⁻¹ + IB⁻¹)⁻¹`, `src/contact_solver.c:139`, `:625-639`, commit 9f998c8), its twist friction comes from the lever arms
(`:612-619`); PhysX has a torsional patch radius per shape, for a single anchor and its TGS solver only, the bound
`max(minRadius, sqrt(penetration * radius))` times the friction impulse (`PxShape.h:471-487`,
`DyTGSContactPrep.cpp:779-810`, 5.6.1, tag `107.3-physx-5.6.1`: both files are the same at 5.6.0); Jolt (v5.3.0, `ContactConstraintManager.cpp`) and Box2D v3 (2D) have none.

The length stands for the width of the contact. A patch of radius `a` under a load `N` spread uniformly
(`p = N / (π a²)`), sliding with a friction `µ`, holds the torque
````
τ = ∫₀ᵃ µ p r 2πr dr = 2/3 µ a N
````
(`3π/16 µ a N` with the pressure of Hertz, `p = 3N / (2π a²) * sqrt(1 - r²/a²)`), so
`SpinningResistance = 2/3 µ a / radius`.

A ball resting on the ground (`Σ λ_normal = m g h` per substep, `I = 2/5 m r²`) loses `Δω = c r m g h / I` per substep
while it spins: it slows down at
````
α = c r m g / (2/5 m r²) = c r g / (2/5 r²) = 5/2 c g / r
````
12.26 rad/s² for `c = 0.05`, `r = 0.1 m` (`TestSpinningResistanceStopsTheTop`: 12.2625 rad/s² measured, stopped from
20 rad/s after 1.63 s, asleep 0.5 s later). The resistance is the only torque around the normal: the single point of a
ball holds no twist (see Friction). Under the bound, the row holds the ball still: an applied torque smaller than
`c r m g` doesn't turn it (`TestSpinningResistanceHoldsATorque`). A capsule standing on its cap brakes at `c r m g / I`, `I` its inertia around
its axis (`TestSpinningResistanceStopsAStandingCapsule`).

The order of the rows is fixed by `TestRelaxSolvesTheSpinningAfterTheRolling`: for a ball the rolling and the spinning
rows don't see each other, for a tilted capsule they do (an impulse around a tangent changes the spin around the
normal).

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

### Axis locks
A body is locked along (translation) and around (rotation) world axes: `LinearLock` & `AngularLock`, 3 bits each.

**Mechanism.** The body has no inverse mass along a locked axis, and no inverse inertia around it, in the solver and in
the integration. It is the choice of Jolt (`EAllowedDOFs`, `Body/AllowedDOFs.h:10-21`, v5.3.0: "Body can move in world
space X axis"...), against the 2 other ways:
- Box3D and Box2D keep the masses and clear the velocities when the positions are integrated ("Motion locks - these
  can be viewed as a constraint that come last", `box3d/src/solver.c:192-198`, commit 9f998c8; `box2d/src/solver.c:134-137`,
  commit 956ce4e, after v3.1). Only a body with its 3 rotations locked loses its inertia (`box3d/src/body.c:1048-1054`).
  The solver computes its impulses for a body which still has its whole mass along the locked axis.
- PhysX clears the velocities too, before the solver (`DyRigidBodyToSolverBody.cpp:72-98`, 5.6.0) and when it
  integrates (`DyBodyCoreIntegrator.h:86-123`; TGS: `DyTGSDynamics.cpp:195-221` and `:1405-1420`), and keeps the
  inertia on purpose: "technically, we can zero the inertia columns and produce stiffer constraints. However, this can
  cause numerical issues with the joint solver" (`DyRigidBodyToSolverBody.cpp:81-83`).

Feather takes the stiffer constraints, and handles the joints (below).

**Translation.** The state of a body carries its inverse mass by axis, null along a locked axis (`invMassAxes`). An
impulse `λ d` changes the velocity by `λ M⁻¹ d`, component by component: a locked component never changes. Jolt does it
by clearing the locked components after each change (`LockTranslation`, `MotionProperties.h:163-166`, in
`AddLinearVelocityStep`, `:195-196`, and in `Body::AddPositionStep`, `Body.h:342`), and keeps the whole inverse mass in
the effective mass of its rows (`AxisConstraintPart.h:114` and `:127`): its rows are solved for a body lighter than it
is along a locked axis, the iterations make up for it. Feather uses the mass the row really moves,
`d·(MA⁻¹ + MB⁻¹)·d` (`linearMass`), as Rapier (v0.22.0: `effective_inv_mass` is a vector, null along the locked axes,
`rigid_body_components.rs:408-424`, projected on the direction of the row, `one_body_constraint.rs:135-136`), and `tx·M⁻¹·ty` between both tangents of the friction (`crossMass`): a rigid row is
exact after one pass (`TestLockedNormalRowIsExact`, `TestLockedNormalBlockIsExact`, `TestLockedJointRowsAreExact`), and
a bounce on a locked body gives back its approach velocity (`TestLockedBounceIsExact`).

**Rotation.** The inverse inertia in world space of a locked body is the inverse of the free block of its inertia
(F the free axes, L the locked ones, `K = R I⁻¹local Rᵀ` the inverse inertia of the free body):
````
I⁻¹locked = (I_FF)⁻¹ = K_FF - K_FL K_LL⁻¹ K_LF   on the free axes, null rows and columns around the locked ones
````
It is the rigid body dynamics of the textbooks: a body held by bearings turns with the inertia of its free axes, the
bearings give the torque which keeps the others still. For a single free axis `n`, it is the moment of inertia around
a fixed axis, `1 / (n·I·n)`. The matrix is written as the Schur complement of `K_LL` in K (`Axes.LockInverseInertia`):
for a body whose axes of inertia are the ones of the world, `K_FL` is null and the result is `K_FF` bit for bit.

**It is not what the other engines do**, and the gap is on purpose. They all keep `K_FF`, the free block of the
*inverse*, which is the answer of a free body whose locked velocities are cleared afterwards:
- Jolt masks the rows and the columns of the inverse (`MotionProperties::GetInverseInertiaForRotation`: "We need to
  mask out both the rows and columns of DOFs that are not allowed", `MotionProperties.inl:69-81`;
  `MultiplyWorldSpaceInverseInertiaByVector`, `:86-100`);
- Rapier masks the rows and the columns of the square root of the inverse (`effective_world_inv_inertia_sqrt`,
  `rigid_body_components.rs:432-450`, v0.22.0);
- PhysX and Box3D keep the whole inverse inertia and clear the locked components of the angular velocity (above): the
  answer on the free axes is `K_FF` too.

`K_FF` is exact when `K_FL` is null (an upright capsule, a box not leaning, a sphere), and too large otherwise: the
body is lighter than it is. Measured with a torque of 1 N·m during 1 s around Y, rotations locked around X and Z,
against `1 / (n·I·n)`: a box of 20 x 100 x 40 cm leaning by 0.5 rad turns 1.7 times too fast, by 45° 2 times; a rod of
2 m leaning by 10° 8 times, by 80° 33 times, by 45° more than 100 times. With the exact inertia they are all at 1
(`TestLockedBodyTurnsAsAroundAFixedAxis`). It costs 1 division and a few products per locked body which turns, and it
never makes the body lighter than the masked matrix does. Unity doesn't have the gap by construction: its rotation
locks follow the axes of inertia of the body ("position constraints are applied in World space, and rotation
constraints are applied in the inertia space", `Rigidbody.constraints`), where `K_FL` is null.

The locks are in world space and the inertia turns with the body: Jolt computes the matrix again from the rotation of
the body each time it needs it. Feather turns the inertia of a body during the step (`integratePosition`,
`I⁻¹ = ΔR I⁻¹start ΔRᵀ`): the inertia kept from the start of the step is the free one, and the turned matrix is locked
again at each substep. Turning the locked matrix would be wrong: a body locked around X turning around Y would get
back an inertia around X (`TestLockedInertiaFollowsTheBody`).

**What doesn't go through the inverse mass** is cleared along the locked axes after the velocities are integrated: the
gravity (as `LockTranslation` in `MotionProperties::ApplyForceTorqueAndDragInternal`, `MotionProperties.inl:137`), the
gyroscopic torque (PhysX: the torque, then the lock flags, `DyRigidBodyToSolverBody.cpp:55-98`) and a velocity written
by the game. A body which turns around a single world axis gets no gyroscopic torque at all: the torque has no
component along its angular velocity, the locks hold it whole (`TestLockedSpinIsKept`).

**Joints.** With locks, both bodies of a joint may not answer along a direction: a body which only turns around Y held
by a fixed joint (the 3 rotations solved together: `K = IA⁻¹ + IB⁻¹` has 2 null rows), a body which can't move pinned
to the world off its center (`K = -[r]× I⁻¹ [r]×`, null along `r`). K is singular: Jolt gives up the whole block
(`PointConstraintPart.h:124-125`, `RotationEulerConstraintPart.h:146-147`: `if (!mEffectiveMass.SetInversed3x3(...))
Deactivate()`), which is the "numerical issue" PhysX avoids. Feather solves the rows which answer: K is symmetric,
positive semidefinite, it is eliminated row by row without pivoting, and a row whose pivot is under 10⁻⁶ of the
stiffest row is left out, its impulse stays null (`lockedInverse`, a generalized inverse: `K G K = K`). With its 3 rows,
it is the inverse of K. The same for the diagonal blocks of an articulation: a chain of bodies locked in a plane is
still solved as a tree (`TestLockedChainHoldsInItsPlane`).

**Friction, 2 rows.** For the 2 rows of a hinge and both tangents of the friction (`lockedInverse2`), a block with a
single row which answers is of rank 1, `K = k u uᵀ`, and `u` is any direction between both rows: a plate which only
turns around Y, rubbing on a ball off its axis, answers along the circle around its axis, not along a tangent of the
contact. Its inverse is the one of least norm (the pseudo-inverse of Moore-Penrose), `K⁺ = u uᵀ / k = K / tr(K)²`: the
impulse is along `u`. Keeping one row (the other generalized inverses) adds an impulse along the direction the locks
absorb: it moves nothing, but it counts in the friction cone, and the body rubs less than Coulomb says, down to not at
all when `u` is nearly the second tangent (`TestLockedPlateRubsOffItsAxes`: the plate stops after the same 0.78 s
wherever the ball is). A box which can't turn nor move along X still rubs along Z (`TestLockedBoxRubsAlongItsFreeAxis`).

**All the axes locked.** The body stays dynamic: it never moves, carries the bodies resting on it, sleeps and wakes up
(`TestFullyLockedBodyStaysDynamic`). It is the behaviour of Unity (`RigidbodyConstraints.FreezeAll`), of PhysX and of
Box3D; Jolt forbids it ("No degrees of freedom are allowed. Note that this is not valid and will crash. Use a static
body instead", `AllowedDOFs.h:12`; the assertion of `MotionProperties::SetMassProperties`, `MotionProperties.cpp:60`).
It is a dynamic body for the continuous collision: a fast body is stopped by it as by any body (`findImpact`), from
their speculative contact when they had one, from the time of impact otherwise (`TestFastBodyStopsOnEveryBody`).

**Setting the locks.** `RigidBody.SetLocks` clears the velocities along the new locked axes (`b3Body_SetMotionLocks`,
`box3d/src/body.c:2379-2408`), returns at once without change (`:2362-2365`), and wakes the body up.

**Continuous collision.** A fast body moved back to its first impact is placed between its start and its end: a
locked coordinate is the same at both, it is kept. The rotation of a body locked around its 3 axes is kept as it is
(the interpolation of 2 equal rotations is not the rotation bit for bit); a body locked around some axes only still
turns, it gets the rotation of its impact (`TestContinuousTurnsAPartlyLockedBody`).

**Without lock**, the arithmetic is the one of a body which has no lock at all: `linearMass` returns the sum of both
inverse masses, the inverse of K is `Inv3`. The fingerprints of the bench are the same bit for bit.

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

## Heightfield and triangle mesh
A heightfield and a mesh are surfaces of triangles. They differ by the triangles they give under the AABB of a body (the
cells of a grid, the leaves of a tree) and by nothing else: one generator of contacts per triangle (`collideTriangle`)
and one grouping in patches (`collideTriangles`, `collision_triangles.go`) serve both, as `b3ComputeMeshManifolds` of
Box3D takes its triangles from `b3QueryMeshTriangles` or `b3QueryHeightFieldTriangles` (`src/mesh_contact.c:51-73`),
`CollideConvexVsTriangles` of Jolt is the visitor of its `MeshShape` and of its `HeightFieldShape`, and
`PCMConvexVsMeshContactGeneration` of PhysX serves its mesh and its height field. The sweeps (`sweepTriangle`), the
overlaps (`overlapsTriangle`) and the continuous collision (`triangleImpact`) share their test of a triangle the same
way. The edges of a mesh are classified as the ones of the heightfield (active, convex), by the triangles sharing them.
No interface of "source of triangles": the enumeration of the triangles is a `switch` on the shape, the loop over the
cells or over the leaves writes in the buffers of the scratch (an interface or a closure per triangle would cost an
indirect call per triangle, and could allocate).

The terrain is a grid of heights, split in 2 triangles per cell (along the diagonal from (x, z) to (x+1, z+1)).
The body is tested against the triangles under its AABB, one by one: the grid is cut in blocks of 16x16 cells with
their lowest and highest heights, to skip the blocks far from the body. Jolt keeps a hierarchy of such blocks
(`HeightFieldShape`, `RangeBlock`), PhysX and Box3D a flat grid (`b3QueryHeightField`); none uses a tree of triangles
for a terrain. The mesh is described below (Triangle mesh).

**Edges** (`Heightfield.cellEdges`). An edge is *convex* if the triangle on its other side bends down, or if there is
none (a border, a hole), and *active* if it is convex by more than 5° (`ActiveEdges::IsEdgeActive` of Jolt with
`mActiveEdgeCosThresholdAngle`, `HeightFieldShape.h:110`; `cos5Deg` of `b3CreateHeightField`, Box3D
`src/height_field.c:192`). Only an active edge can push a body sideways.

**Each triangle** (`collideTriangle`), with GJK/EPA as the only detection:
1. **Seen from below, it is ignored**: the center of the body is under its plane (Jolt
   `CollideConvexVsTriangles.cpp:52-54`, Box3D `triangle_manifold.c:380-385`, PhysX
   `GuPCMContactConvexCommon.cpp:238-240`). The terrain is a surface, without thickness (`CollidePoint` of the height
   field of Jolt is empty, `thickness = 0` in `GuHeightField.h:847` of PhysX): a body whose center went through it is
   not pushed back, it falls under the terrain. The continuous collision is what keeps a fast body above it.
2. **Face**: the vertices of the body over the triangle, along the normal of the triangle. The face of the body is the
   one it shows to a plane of this normal (`CollideWithPlane` without limit of distance): the 4 corners of a face of a
   box, both ends of a capsule. It is taken whatever its tilt, as the supporting face of Jolt
   (`BoxShape::GetSupportingFace`); a capsule gives both ends at any tilt, where Jolt asks them within 2 cm along the
   normal (`cCapsuleProjectionSlop`, `CapsuleShape.cpp:204-212`). Seen along the normal, the face is cut by the 3
   sides of the triangle (Sutherland-Hodgman): the part of the face over the triangle, the polygon
   `ManifoldBetweenTwoFaces` of Jolt gets by cutting the triangle by the face (`ManifoldBetweenTwoFaces.cpp:194`);
   a segment is cut the same way (`b3CollideTriangleAndCapsule` of Box3D: `ClipPolyVsEdge` of Jolt, `ClipPoly.h`, can
   give a point out of the triangle when the capsule ends beside it). Each point is at its distance to the plane of the
   triangle along the normal, kept under the margin of the pair (Jolt adds `mManifoldTolerance`, 1 mm). On a flat
   terrain these points are the points of the body on a plane.
3. **Flat ridges**: the points where the face of the body crosses a convex edge which is not active (a ridge folded by
   less than 5°), along the normal of the triangle. The side of a capsule, the face of a box laid across such a ridge
   touch it there, with none of their vertices: before, the ridge went in the body by `half length x tan(fold / 2)`,
   up to 10.7 mm for a body of 0.5 m. The points cut on a flat edge or on an edge bent inwards are not kept (Jolt keeps
   all the points of the polygon): they are between vertices of the body, and say nothing more.
4. **Closest point** (GJK/EPA: the closest points of the triangle and of the body, their normal, their depth). Its
   normal follows the rule of `ActiveEdges::FixNormal` of Jolt (`ActiveEdges.h:42-111`), without its part on the
   motion of the body: the normal of EPA is kept if the 3 edges of the triangle are active, if the triangle has an
   active edge and both normals are closer than 1°, or if the point is on an active edge or on a vertex of an active
   edge (barycentric coordinates under 1e-4); else the normal is the one of the triangle. The point and its depth are
   kept in both cases (Jolt), the separation is measured along the normal kept (`ContactConstraintManager.cpp:69`).
   Jolt also keeps the normal of EPA when it brakes the motion of the body over the terrain less than the normal of
   the triangle (the hint `mActiveEdgeMovementDirection` of `PhysicsSystem.cpp`: a body leaving a triangle is not held
   by its plane). Feather does not: the margin of Jolt is fixed (`mSpeculativeContactDistance`, 2 cm, `PhysicsSettings.h:50`, the
   `mMaxSeparationDistance` of its collisions, `PhysicsSystem.cpp:1108`),
   the speculative margin of Feather grows with the speed of the body (`reach`: 10 cm at 5 m/s), and brings the
   vertices and the edges of the triangles around within it, where every normal tilted from the vertical brakes a fall
   less than the plane. A box falling flat on a flat terrain, straight on a vertex of the grid, had 5 contacts at the
   same corner: one straight up, 4 on the edges of the triangles around, tilted by 10 to 60°; nearly parallel to the
   first, they took impulse from it over the sub-steps (9.4 N·s, their planes 2 mm under the ground), and the box landed
   2 mm deep (0.3 mm now, the deepest of 560 boxes dropped alone on a flat terrain). PhysX and Box3D have no such
   rule (`PCMContactConvexMesh`, `b3ComputeMeshManifolds`). What the rule gives in Jolt, a box sliding across an active
   ridge not held by the plane it leaves, comes from the points cut on an active edge, never points of the face
   (below).
   - With the normal of the triangle, the closest point is a point of its face (a sphere has no other), kept as the
     vertices of the body are, or a point of its edge, the body beside the triangle, above this edge: kept if it is
     deeper than the other points of the triangle (the end of a capsule right above a ridge is over neither of its
     triangles, along their normals).
   - With another normal, it is the contact of an **edge**: the points where the face of the body crosses the active
     edges of the triangle, along this normal, or the closest point alone (a sphere, an edge of a box across the
     ridge). The points cut on an active edge are never points of the face: the plane of a triangle stops at its edge,
     a box sliding across a ridge must not be held by the plane of the slope it leaves.

**Patches** (`collideHeightfield`). The contacts are sorted, the deepest first, and grouped by normal: the contacts whose
normals differ by less than 5° form a patch, a manifold of 4 points (`mContactNormalCosMaxDeltaRotation` of Jolt,
`PhysicsSettings.h:74`; `clusterThreshold` 0.996 of Box3D, `mesh_contact.c:856`). The normal of a patch is the one of
its deepest contact (Jolt takes the mean of the normals, `PhysicsSystem.cpp:1186`). 3 choices of Feather:
- **The vertices of the body and the edges of the terrain don't share a manifold**: one patch per normal for the points
  of the faces, one for the points on the edges (flat ridges, active edges). Jolt and Box3D group by normal alone and
  keep 4 points out of all of them (`PruneContactPoints`, `b3ReduceCluster`). With 4 points for both, a point on an
  edge takes the place of a corner: a plate resting flat on a slope lost 2 corners to 2 points of its face over an edge
  of the grid, tipped, and sank by 10 mm while the pair cache kept its contact; boxes dropped alone on a flat terrain
  landed up to 21 mm deep (0.3 mm with their corners, as on a plane).
- **A closest point on an edge is kept if it is the deepest point along its normal** (no patch within 5° has a point
  as deep). A body resting on a triangle near an edge also has a closest point with each triangle around, on this
  edge: one more point per triangle, which moves the friction center (a sphere would not roll on a flat terrain as on
  a plane). Box3D and PhysX drop such a point when a triangle with a contact of its face owns the edge
  (`b3ComputeMeshManifolds`, `mesh_contact.c:654-832`; `PCMConvexVsMeshContactGeneration::generateLastContacts`).
  Feather compares the depths: with the ownership alone, the deepest point of a body above a ridge is dropped when the
  rest of the body gives a contact to the faces around (the cause of the bodies found 5 to 19 cm in the terrain: a
  point 56 mm deep dropped for a point 0.1 mm deep). A closest point inside a triangle is a point of its face: kept
  whatever its depth, as the vertices of a body are (Box3D and PhysX keep it too, `mesh_contact.c:653-660`,
  `GuPCMContactSphereMesh.cpp:305-319`: a point of a face owns no edge). Dropped as the others, it left a sphere rolling
  to a triangle folded by less than 5° without a contact with it until it was the deepest, and the sphere sank by
  3 mm: 70 spheres out of 2800 dropped alone on the hills landed deeper than 2 mm, 22 now.
- **8 patches at most** per pair (`MaxManifoldsPerPair`; 32 manifolds in Jolt, `PhysicsSystem.cpp:1133`): the deepest
  are kept. Measured with 32: the same depths for the spheres and the capsules, 47 boxes out of 800 landing deeper
  than 5 mm instead of 52.

The 4 points of a patch (`reducePatch`) follow the steps of `Reduce`: the deepest, the furthest from it, the points
adding the most area. Among the points as deep as the deepest, the first one is an end of the contact (the furthest
along a tangent, as the first step of `b3ReduceManifoldPoints`), and a point is added only if it widens the contact by
`SpeculativeDistance`.

Measured (`bench`, the depth of a box counts the terrain in its faces): a capsule and a box across a ridge folded by 0
to 10° rest less than 0.1 mm in it (2.2 to 10.8 mm before, under 5°); 1000 piles of 60 bodies on hills rest at 0.28 mm
(median of the deepest body; 3.3 mm before) and land at 7.9 mm (15.9 mm before); the same piles on a flat terrain rest
at 0.13 mm and land at 1.7 mm, as on a plane. What is left at the landing is the limit of the contacts computed once
per step, as Jolt, Box3D and PhysX compute them (see the limitations in ARCHITECTURE.md): a point of a tumbling body
moves over another triangle during the step, and has no contact with it before the next step. With the contacts
computed at each of the 8 sub-steps, the deepest of 60 bodies dropped alone lands at 3.7 mm instead of 8.1 mm, and
no sphere deeper than 1 mm.

## Convex hull
`actor.ConvexHull`, `actor/convexhull.go`. Sources read: Jolt v5.3.0 (`Geometry/ConvexHullBuilder.cpp`,
`Physics/Collision/Shape/ConvexHullShape.cpp`), Box3D (commit `9f998c8`, `src/hull.c`), PhysX (`main`, 5.11.0,
`GuCookingQuickHullConvexHullLib.cpp`, `PxConvexMeshDesc.h`), Bullet 3.25 (`btConvexHullComputer`), Barber, Dobkin &
Huhdanpaa (1996), Dirk Gregorius, "Implementing QuickHull" (GDC 2014).

**Quickhull** (`hullBuilder`). The points are taken around the center of their bounds (Box3D shifts them to an origin
too, `b3HullBuilder_Construct`). The tolerance is the rounding error of the distance of a point to a plane, from the
size of the cloud: `3 (|x| + |y| + |z|)max ε` (Gregorius; `DetermineCoplanarDistance` of Jolt, `ConvexHullBuilder.cpp:240`;
`b3HullBuilder_ComputeTolerance` of Box3D, `hull.c:632`), with the ε of a float64 where both use the one of a float. A
point is out of the hull if it is further than 8 tolerances in front of a face (`minOutside` of Box3D, `hull.c:644-645`),
a face is seen by a point if the point is more than 4 tolerances in front of its plane (`minRadius`, `hull.c:868`).
1. **The initial tetrahedron**: the 2 points the furthest apart along the axis of the largest extent, the point the
   furthest from their line, the point the furthest from their plane (`b3HullBuilder_BuildInitialHull`, `hull.c:649`;
   `QuickHull::findSimplex` of PhysX). Fewer than 4 points, a line or a plane: `ErrHullDegenerate` (Jolt builds a flat
   hull of 2 faces, `ConvexHullBuilder2D`; Box3D fails as Feather). A hull is a volume: a flat one has no mass.
2. **The outside sets** (Barber et al.; the conflict lists of Jolt and Box3D): each point out of the hull is given to the
   face it is the furthest in front of, the furthest point of a face last (`AssignPointToFace`, `ConvexHullBuilder.cpp:196`).
3. **A point at a time**: the furthest point of the face with the furthest point (`b3HullBuilder_NextConflictVertex`,
   `hull.c:782`). The faces which see it, from its face through their neighbors (the horizon of Barber et al.,
   `b3HullBuilder_BuildHorizon`, `hull.c:839`), are removed, a cone of triangles is built from the horizon to the point
   (`b3HullBuilder_BuildCone`, `hull.c:881`; `AddPoint` of Jolt, `:622`), and the points of the removed faces are given to
   the cone. The horizon must be a loop, each vertex starting one edge: if the rounding breaks it, `ErrHullFailed`
   (Jolt returns an error when the hull is inconsistent, `ConvexHullShape.cpp:60-78`).
4. **The vertex limit**: the construction stops at `maxVertices` (`MaxHullVertices` = 256 at most, `cMaxPointsInHull` of
   Jolt, `B3_HULL_MAX_COUNT` of Box3D; 255 in PhysX, `PxConvexMeshDesc::vertexLimit`). The points left out are then
   outside the hull, the closest to it: the furthest were added first. Jolt accepts a hull stopped this way
   (`MaxVerticesReached`, `ConvexHullShape.cpp:61`), Box3D runs on a budget (`hull.c:1450-1458`); PhysX expands the
   hull by its planes instead (`expandHull`, plane shifting). Measured: 32 vertices out of a cloud of 500 points on a
   sphere keep 83 % of its volume.
5. **The faces**: once the triangulation is complete, 2 triangles across an edge are merged when the edge is not convex:
   the centroid of one is not behind the plane of the other by more than the tolerance (coplanar, or bent inwards by the
   rounding: `b3IsEdgeConvex` of Box3D, `hull.c:466`, both ways as `MergeCoplanarOrConcaveFaces` of Jolt, `:975`). The
   boundary of each group of triangles is chained in a polygon, its vertices on the segment of their neighbors within
   the tolerance are dropped (`b3HullBuilder_ResolveVertices`, `RemoveInvalidEdges` of Jolt), and its plane is Newell's
   normal through its centroid (`b3NewellPlane`, `hull.c:529`). Jolt and Box3D merge after each cone, on a hull still
   growing; Feather merges once, at the end: the triangulation is the quickhull of Barber et al. as it is, the merge a
   pass over it. The face of a cube is 1 polygon of 4 vertices: a cube resting on the ground has the manifold of the Box
   (4 corners), where 2 triangles would give 3.

**Mass properties** (`massProperties`): the volume and the center of mass are the sum of the tetrahedra between the
faces (fans of triangles) and a point inside (the mean of the vertices), as `GetCenterOfMassAndVolume` of Jolt
(`ConvexHullBuilder.cpp:1258`); the inertia is the covariance of the canonical tetrahedron carried by each tetrahedron
from the center of mass (Jolt, `ConvexHullShape.cpp:84-121`, after Blow & Binstock), for a density of 1, scaled to the
mass. The points are then translated so that the center of mass is at the origin of the hull: a body is at its center
of mass, `CenterOfMass()` gives it in the space of the points (Jolt centers its shapes the same way,
`mCenterOfMass`). Measured: the hull of the corners of a box has its mass and its inertia within 1e-6, its center of
mass within 1e-12.

**The shape**: the support point is the vertex the furthest along the direction, among all of them
(`HullNoConvex::GetSupport` of Jolt, `b3FindHullSupportVertex` of Box3D, `hull.c:1644`); the contact feature is the face
the most aligned with the direction (`GetSupportingFace` of Jolt, `ConvexHullShape.cpp:674`; `b3FindHullSupportFace`),
8 vertices at most, one out of k on a larger face (Jolt skips vertices the same way, `:703-706`); against a plane, the
vertices of this face, as the Box; the AABB is the one of the 8 corners of the local bounds (`b3ComputeHullAABB`,
`hull.c:2614`; the local bounds of Jolt), not of the vertices; the ray is clipped by the planes of the faces, in the hull
between the last plane it enters and the first one it leaves (`b3RayCastHull`, `hull.c:2641`; `CastRayHelper` of Jolt,
`:883`). Measured (a hull against a box, per pair, `Collide`): 8 vertices 5.1 µs, 32 vertices 6.9 µs, 64 vertices
7.3 µs, 256 vertices 16.3 µs; a box against a box 6.7 µs, the hull of a cube 7.2 µs. The build of a hull of 4000
points takes 0.5 ms.

## Triangle mesh
`actor.TriangleMesh`, `actor/trianglemesh.go`. Sources read: Box3D (`src/mesh.c`, `src/mesh_contact.c`), Jolt v5.3.0
(`MeshShape.cpp`, `AABBTree/AABBTreeBuilder.cpp`, `TriangleSplitter/TriangleSplitterBinning.cpp`, `Geometry/Indexify.h`,
`Geometry/RayTriangle.h`, `Physics/Collision/ActiveEdges.h`), PhysX (`GuBV4Build.h`, `GuEdgeList.cpp`), Bullet 3.25
(`btQuantizedBvh.cpp`), Ericson 6.2.1.

**Build** (`NewTriangleMesh`, outside `Step`; 100 000 triangles take 0.3 s). The vertices closer than 0.1 mm are
welded (`MeshWeldDistance`: the weld distance of `Indexify` of Jolt, `Indexify.h:14`; `weldTolerance` of `b3MeshDef`), in
a grid of cells of this size: a mesh exported as a soup of triangles has its edges shared again, else every edge would be
a border, and active. The triangles under 0.01 LinearSlop² of area are degenerate (`minArea` of `b3CreateMesh`,
`mesh.c:1590`): kept in `Indices` (the indices of the game don't move), out of the tree, without edge. The triangles
are counterclockwise seen from outside, the right-hand rule: the side of the normal is the side a body touches (Box3D
and Jolt take the same winding).

**Edges** (`findEdges`): each edge is looked up by its 2 vertices in a map (`b3IdentifyEdges`, `mesh.c:1065`;
`sFindActiveEdges` of Jolt, `MeshShape.cpp:248`). An edge of 1 triangle (a border) is convex and active; an edge of 2
triangles is convex if the opposite vertex of the other triangle is under the plane, and active if it is convex and bent
by more than 5° (the rule of the heightfield, `cellEdges`), or if the 2 triangles are back to back, closer than 1° to
opposite (`ActiveEdges::IsEdgeActive`, `ActiveEdges.h:21`: a sheet); an edge of 3 triangles or more is active (Jolt,
`MeshShape.cpp:313-319`). Bits per triangle as the heightfield: active, then convex.

**Tree** (`buildNode`, `sahSplit`): a binary tree of AABBs built top-down, the triangles of a node split in 2 by the
surface area heuristic over binned centroids (`b3SplitBinnedSah`, `mesh.c:570`; `TriangleSplitterBinning` of Jolt;
the `BV4_SAH` strategy of PhysX, `GuBV4Build.h:62-67`, whose default splits at the center): on each axis the centroids
of the triangles fall in 8 bins (`B3_BIN_COUNT`), and the cost of a split after a bin is the count of the triangles on
each side times the area of their AABB; the cheapest split of the 3 axes wins. A leaf holds 4 triangles at most when a
split is found (`B3_DESIRED_TRIANGLES_PER_LEAF`; 8 in Jolt), 8 when none separates them (the centroids are the same,
`B3_MAXIMUM_TRIANGLES_PER_LEAF`), halves above. Added to Box3D: a split leaving less than a third of the triangles on a
side is replaced by the median along its axis, the rule of `btQuantizedBvh::sortAndCalcSplittingIndex` of Bullet
("if the splitIndex causes unbalanced trees, fix this by using the center"): the height of the tree is then bounded by
`log(triangles) / log(1.5)`, 52 for 2^31 triangles, and the traversals keep their nodes in an array of 64 on the stack
(`meshStackSize`; Box3D keeps 256 and asserts). The nodes are in depth-first order, the left child follows its parent;
the triangles of a leaf are contiguous in `order`. Not taken: the quantized nodes of Jolt (`NodeCodecQuadTreeHalfFloat`)
and of Bullet, the 4-wide nodes of PhysX (BV4): less memory and SIMD, for another day. Measured: 20 000 triangles of
hills give 12 053 nodes, 6 027 leaves, a height of 14 (bound 26).

**Queries** (`OverlapTriangles`, `Cast`, `MeshCast`): the triangles whose AABB overlaps a box, by the nodes it overlaps
(`b3QueryMesh`, `mesh.c:2416`); the triangles a moving box meets, in the order of its motion: the segment of its center
against the AABBs of the nodes enlarged by its half sizes, by slabs, the closest child first, the nodes entered after
the limit dropped (`b3ShapeCastMesh`, `mesh.c:2070`, which orders the children by the axis of the split; the ray through
the trees of the broad phase of Feather, `aabbTree.cast`). `MeshCast` is an iterator with its stack inside, as `CellWalk`
of the heightfield: the sweeps and the rays run it without a callback.

**Ray** (`CastRay`): `MeshCast` without size, Möller-Trumbore on each triangle (`RayTriangle.h` of Jolt,
`b3RayCastMesh` of Box3D), the side of the normal only (the determinant positive), a point up to `footprintSlack` out
of its triangle in barycentric coordinates so that a ray on an edge hits the triangles on both sides, and at the very
same fraction the triangle of the lowest index, as the heightfield. Measured on 100 352 triangles of hills: 1.2 µs per
ray (100 000 slanted rays, 96 % hits), 0.85 µs for vertical rays, against the 5 µs asked.

**Collisions**: the triangles under the AABB of the body come from `OverlapTriangles`, then the generator of the
heightfield (above). Measured: 150 bodies (boxes, spheres, capsules) falling on the same hills as a heightfield of 41x41
samples and as a mesh of 3 200 triangles: the narrow phase takes 2.5 ms per step on the heightfield, 2.4 ms on the mesh
(the step 6.9 and 6.5 ms). A sphere rolls on a tilted grid mesh within 3 µm of a sphere on a plane; a cube on a tread of
a staircase mesh, a cube overhanging the edge of a tread and a plank lying on the edges of 3 treads (active edges alone)
rest for 10 s without drifting by a micrometer, and sleep.

## Continuous collision
Sources read: Box2D v3.1.0 (`src/solver.c`, `docs/simulation.md`, `src/constants.h`), Box3D (commit `9f998c8`,
`src/solver.c`, `include/box3d/types.h`), Jolt v5.3.0 (`Physics/PhysicsSystem.cpp`, `PhysicsSettings.h`,
`Body/MotionQuality.h`, `Docs/Architecture.md`), PhysX 5.6.0 (`ScCCD.cpp`, `PxsCCD.cpp`, `PxRigidBody.h`,
`PxSceneDesc.h`, the guide Advanced Collision Detection).

**Speculative contacts**: against a static or a kinematic body, and for every pair of a fast body (below), the contacts
are created up to `SpeculativeDistance` + the relative speed of the bodies * dt: the solver stops the bodies before they
touch. It is the speculative CCD of PhysX, where the contact distance of a body is its linear speed * dt + its contact
offset + its angular speed * dt * the radius of its bounds (`Sc::BodySim::updateContactDistance`, `ScCCD.cpp:64-96`),
and the "Continuous Speculative" mode of Unity. PhysX applies it to the bodies flagged `eENABLE_SPECULATIVE_CCD`
(`PxRigidBody.h:99-105`); Feather to the static and kinematic bodies, and to the fast bodies, without a flag. Between 2
dynamic bodies which are not fast, only up to `SpeculativeDistance` (as Box2D v3: `B2_SPECULATIVE_DISTANCE`, 4 linear
slops for every pair, `constants.h:38`): an impact is absorbed by the spring of the contact over a few substeps, where a
speculative row stops the bodies rigidly in the substep they touch. Their known limits: a contact can be found by a body
which will not touch it (a ghost contact), and a body accelerated by the solver during the step can go further than its
margin (PhysX documents it: "if the constraint solver accelerates an actor [...] such that the actor passes entirely
through objects during that time-step, speculative CCD can result in tunneling").

**Fast body**: a body which moves more than half of its smallest extent during the step, by the speed of its farthest
point (translation + rotation * its extent): the fast body of Box2D v3 (`maxVelocity * timeStep > 0.5f *
sim->minExtent`, `solver.c:584` & `:622`), of Box3D with its `safetyFactor` of 0.5 by default (`types.h:316-318`,
`solver.c:780`); Jolt casts a body which moves more than 0.75 of its inner radius (`mLinearCastThreshold`,
`PhysicsSettings.h:53`, `PhysicsSystem.cpp:1572-1573`). Feather judges it twice: from the velocities at the start of
the step for the margin of the narrow phase (`isFast`), from the motion the step made for the time of impact (as Box2D,
from its velocity at the end of the step).

**Time of impact**: after the solver, a fast body is moved back to its first impact along its motion, its velocity is
kept: the contact of the next step stops it (Box2D v3, `b2SolveContinuous`; Jolt applies an impulse at the impact at
once, `sSolveCCDContact`, `PhysicsSystem.cpp:2093`, not followed: the solver of the next step has the contact, with its
friction and its restitution). The time of impact is found by conservative advancement (Mirtich, as in Bullet): the body
moves forward by its distance to the other body (GJK) divided by the fastest approach of its points, until it is
`LinearSlop` away. If it already touches at the start, only its core (a sphere of 1/4 of its smallest extent, as in
Box2D) is stopped. The fast body is swept against every body it collides with: the static bodies (Box2D), the kinematic
and the dynamic bodies too (the `LinearCast` motion quality of Jolt casts against every body; Box2D only for its
bullets, "Bullets will perform CCD with all body types, but not other bullets", `simulation.md:377-378`), where they
are at the end of the step (Jolt: "Body has already moved", `PhysicsSystem.cpp:1724`). A dynamic or a kinematic body
the fast body has a contact with this step is left to the solver: the speculative row holds the pair where they touch,
and a sweep of a solved pair would lift the fast body off the overlap the soft contact allows, every step. Measured:
swept too, the links of a swinging chain of 20 capsules (`joint chain` of the bench) stretched its joints by 214 mm
instead of 9 mm, the small cubes under the slab of `high mass ratio 2` drifted by 34 mm instead of 18. A static body
is always swept (Box2D), its speculative contact being the one of the start of the step: a resting body touches it at
the start and is only stopped by its core. **Two fast bodies** are swept once, by the one with the smallest state
index, with their relative motion against the start pose of the other (Jolt: "Get relative movement of these two
bodies", `direction = mShapeCast.mDirection - sCalculateBodyMotion(body2, ...)`, `PhysicsSystem.cpp:1884`), the
rotation of the other during the step ignored (as the linear cast of Jolt); both are stopped at the fraction found
(Jolt: "the other body will shorten its distance traveled", `:2054-2060`). The fractions of all the fast bodies are
found before any body moves: the result doesn't depend on the order of the bodies (Jolt sorts its CCD bodies by
fraction for the same reason, `:2017`). Feather has no bullet flag (`IsBullet` removed): every fast body is stopped by
every body, as the motion quality of Jolt stops the bodies which have it; Box2D keeps `isBullet` because its continuous
collision is against the static bodies by default.

Measured (`TestFastSpheresFaceToFaceBounce`, `TestFastPlateNeverCrossesARestingPlate`,
`TestKickedBodyIsStoppedByTheContinuousCollision`): before, 2 spheres of 10 cm went through each other from 7.5 m/s of
relative speed (12.5 cm per step at 60 Hz: their diameter + the margin of 2 cm), 2 plates of 1 cm from 2.4 m/s (4 cm
per step: twice their thickness + the margin). With the margin of the fast pairs, 2 spheres at 20 m/s each come within
0.07 mm and bounce back at 10 m/s each (restitution 0.5), a plate at 30 m/s pushes the resting plate at 15 m/s without
overlap. The margin follows the speed of the start of the step: a light plate kicked at 40 m/s by a heavy ball during
the step goes through the plate 30 cm behind it (their pair had the margin of 2 resting bodies); the time of impact
stops it 0.8 mm from it. Cost: nothing measurable on the falling pile of 2 000 bodies of `BenchmarkWorldStep` (no body
of it is fast: 380 ms for 60 steps before and after, 1 worker, A/B interleaved); on the mixed scene of the determinism
tests (200 bodies, 42 of them fast at every step, bouncing in a pile), the step goes from 1.80 to 2.43 ms: the narrow
phase doubles (331 to 695 µs, the speculative contacts of the fast pairs), the continuous collision goes from 58 to
89 µs, the solver takes the rest (more contacts). PhysX documents the same price for its CCD: "As the objects'
velocities increase, the CCD overhead will increase, especially if there are a lot of high-speed objects in close
proximity". On the bench, the fingerprints of 13 scenes change (slope pile, terrain piles, rain on terrain, joint
chain, locked bodies, and card house, double domino, far chain, far stack, high mass ratio 2 and 3, joint grid, rush
of Solver2D): the worst stretch of the joint chain goes from 23.7 to 9.1 mm and the small cubes of high mass ratio 2
and 3 sink 12 mm less into the ground, while they drift by 17.8 mm instead of 16.5 and the least push of the locked
bodies goes from -0.054 to -0.051 m, accepted; the reference was regenerated machine at rest.

**GJK distance**: the distance and the closest points of 2 convex shapes. The simplex is reduced to its feature closest
to the origin (Voronoi regions, Ericson 5.1 & 9.5).

A sleeping body touched by an awake body wakes up with its island during the collision detection, and gets its contacts
in the same step (like Jolt).

## Queries
A ray, a moving shape and an overlap, on the trees of the broad phase. Sources read: Box3D (commit `9f998c8`), Box2D
v3.1.0, Jolt v5.3.0, PhysX 5.6.0, the ScriptReference of Unity, the API reference of Unreal.

**Convention.** A query is an origin and a translation, its hit is at a fraction of the translation, from 0 to 1: the
convention of Box3D (`b3World_CastRay`, `b3World_CastShape`, `include/box3d/box3d.h:83-107`; `b3RayResult`,
`types.h:1353-1386`) and of Jolt, where PhysX and Unity take a unit direction and a distance. No direction to
normalize, and a translation without length has a meaning: a point. The results come by value or in a buffer of the
caller, as the `NonAlloc` queries of Unity, where Box3D and Jolt call a function for each hit (`b3CastResultFcn`,
`types.h:100-117`): in Go, a closure given to a method escapes and allocates.

**The trees between 2 steps.** A step leaves the trees up to date for the queries which follow it, as the last stage of
a step of Box2D v3: "Refitting [...] is necessary to ensure the BVH is valid for subsequent queries, such as ray casts"
(`docs/simulation.md:1913-1917`; `b2Solve`, `src/solver.c:1881`). As in Box2D, which only enlarges the AABBs "for
shapes that have moved significantly", the end of a step only reads the bodies the step moved, the bodies of its
solver: neither the static bodies nor the sleeping ones (`World.syncMoved`). What the game writes is taken on demand by
`SyncQueries`: `Physics.SyncTransforms` of Unity ("Do use it after Transform changes [...] if you're immediately
performing a physics query"), `flushUpdates` of PhysX (`PxSceneQuerySystem.h:143-156`). Not taken: the lazy update of
PhysX at the first query, which writes the scene from a query and needs a lock; the update at each write of Box2D
(`b2Body_SetTransform`, `src/body.c:681-728`), the fields of a body of Feather being public.

**Ray through a tree** (`aabbTree.cast`), as `b3DynamicTree_RayCast` of Box3D (`src/dynamic_tree.c:1062-1188`): in
depth with a fixed stack, the segment tested by slabs against the AABB of both children of a node, the closest child
walked first (`:1124-1136`), and the segment shortened at each hit (`:1163-1171`), which drops the nodes it enters
further. Differences: the children are ordered by the fraction where the segment enters them, not by the distance of
their center; a node entered exactly at the best fraction is still walked, for the rule of the lowest index; when the
stack is full, a child is walked at once by a call instead of being dropped (Box3D asserts). The inverse of the
translation is kept finite: a segment starting in the plane of a face of an AABB would give 0 × ∞. For a moving shape,
the segment is the path of the center of its AABB and the AABBs of the nodes are enlarged by its half sizes
(`b3DynamicTree_BoxCast`, `dynamic_tree.c:1191`).

**Ray on a shape** (`CastRay`, in the local space of the shape), analytic on each:

| Shape | Method | Source |
|---|---|---|
| sphere | the closest point of the line to the center, then its distance to the surface along the line | `b3RayCastSphere`, Box3D `src/sphere.c:62-137` (after "Precision Improvements for Ray/Sphere Intersection", Ray Tracing Gems, 2019); the quadratic of Jolt `Geometry/RaySphere.h:17-40` loses digits far from the sphere; Ericson 5.3.2 |
| box | slabs: the ray is in the box while it is between its 3 pairs of faces | Jolt `Geometry/RayAABox.h`; PhysX `GuIntersectionRayBox.cpp:59`; Ericson 5.3.3 |
| capsule | the infinite cylinder, then the spheres of both ends if the ray enters the cylinder beside the segment | Jolt `Geometry/RayCapsule.h:19-34`, `RayCylinder.h`; Box3D `src/capsule.c:99` solves the closest points of the 2 segments instead; Ericson 5.3.7 |
| plane | the half space under the plane, in world space | Jolt `PlaneShape::CastRay`, `PlaneShape.cpp:167-190`; Ericson 5.3.1 |
| heightfield | a walk of the cells, then the planes of the 2 triangles of each cell | below |

The sections of Ericson (*Real-Time Collision Detection*, chapter 5, "Intersecting Lines, Rays, and (Directed)
Segments") are given by their titles in its table of contents.

**Start in a shape.** The shapes are solid: a ray which starts in a sphere, a box, a capsule or under a plane hits at
the fraction 0, the normal against its direction; so does a moving shape which overlaps a body. It is the choice of
PhysX (`GuRaycastTests.cpp:97-100`, `:147-150`; `hadInitialOverlap`, `PxGeometryHit.h:121`), of Jolt
(`mTreatConvexAsSolid = true`, `RayCast.h:83-84`), of Unreal (`FHitResult::bStartPenetrating`: "Whether the trace
started in penetration, i.e. with an initial blocking overlap") and of the raw casts of Box3D (`sphere.c:81-86`,
`distance.c:1084-1100`). Not taken: Unity ("Raycasts will not detect Colliders for which the Raycast origin is inside
the Collider") and `b3World_CastRayClosest` (`physics_world.c:3000`), where a camera arm starting against a wall goes
through it. The caller who wants to ignore the body excludes it.

**Start in exact contact.** A moving shape only "starts in" a body it penetrates. In exact contact, it hits at the
fraction 0, with the normal of the contact, if it moves towards the body, and hits nothing if it moves away or along
it: a camera arm or a character against a wall slides along it. It is the rule of Jolt, which only keeps the hit of a
shape cast when its normal faces the motion (`contact_normal.Dot(mDirection) > 0`, "Test if backfacing",
`ConvexShape.cpp:283-285`, the default mode). Not taken: Box3D, where any start closer than its slop is an "initial
overlap" whatever the direction, without normal (`distance.c:1084-1100`). Jolt extends its rule to the overlaps;
Feather doesn't: a shape which penetrates a body is stopped at the fraction 0 whatever its direction. What it means:
- **The exact contact is decided by the rounding.** A shape put in contact by computed coordinates (a center at the
  sum of the radii) is in contact, a hair apart or a hair in the body, as the last bit falls: measured on spheres put
  against a sphere, 1 in 8 to 1 in 3, depending on the coordinates, is seen in the body, and stopped at the fraction 0
  even when it moves away. A mover never puts its shape in contact: it starts again from the place given by `Sweep`
  (0.5 to 1 µm from the body), or keeps a skin.
- **A body within 1 µm is touched.** The shape is "on" a body closer than twice the gap (see the sweep, below): a
  shape which passes within 1 µm of a body, moving towards it by any amount, hits it.

**Back faces.** Only the top side of a heightfield is hit, by a ray and by a moving shape, as everywhere by default
(Box3D "Ignores back-side collision on meshes and height-fields", `box3d.h:85`; Jolt `IgnoreBackFaces`,
`RayCast.h:78-81`; Unity `Physics.queriesHitBackfaces`). `Overlap` sees both sides.

**Ray on a heightfield** (`Heightfield.CastRay`, `CellWalk`). The ray walks the cells under it, in the order it meets
them, then tests the 2 triangles of each cell: the traversal of a grid by a ray of Amanatides & Woo ("A Fast Voxel
Traversal Algorithm for Ray Tracing", Eurographics 1987), used by the ray and shape casts of the height fields of Box3D
(`b3RayCastHeightField` & `b3ShapeCastHeightField`, `src/height_field.c:593-601` and `:605`) and of PhysX
(`traceSegment`, `GuHeightFieldUtil.h:481`); Ericson 7.4.2 ("Uniform Grid Intersection Test"). Differences:
- **By columns.** Amanatides & Woo step from a cell to the next by the closest of the next 2 lines of the grid. Here
  the walk follows the axis the ray moves the most along, column by column, and takes in each column the range of rows
  between the points where the ray enters and leaves it: the same cells for a ray, and it widens to a box (a moving
  shape) by its half sizes, where Box3D walks the leading corner of the box and sweeps its front.
- **2 levels.** The columns are taken by blocks of 16 (the blocks of the narrow phase, with their lowest and highest
  heights): a block the ray flies over or under is skipped with its 256 cells. Jolt walks a min/max hierarchy instead
  (`HeightFieldShape.cpp`).
- **The triangle.** A triangle of a heightfield is a plane over half a cell: the height of the ray above it is linear
  along the ray, the hit is where it is 0, if this point is over the triangle and the ray comes from above. It is a
  ray against a plane (Ericson 5.3.1) and a test of 3 lines of the grid, in the place of the Möller-Trumbore
  intersection Jolt uses for its triangles (`Geometry/RayTriangle.h:8-9`).
- **On an edge.** A ray which falls on an edge or on a sample hits several triangles at the same fraction:
  `Hit.Triangle` names the lowest index, wherever the ray comes from, as a moving shape and as the bodies do. The rule
  only holds at the very same fraction (a flat terrain); elsewhere the rounding chooses, and both triangles are right.
- **No leak.** The point may be a billionth of a cell out of its triangle, and the walk is widened by twice as much: a
  ray on an edge, on a sample or on the border of a hole hits the triangles around it. Möller-Trumbore, tested on each
  triangle alone, leaks on the edges a ray lies on (the brute force of the tests needs the same slack there).

**Sweep** (`World.Sweep`, `query_sweep.go`): conservative advancement (Mirtich 1996) on the cores of the shapes, as
`b3ShapeCast` of Box3D (`src/distance.c:1044-1172`; `sphere.c:205-217`, `capsule.c:342-354`). At each iteration GJK
gives the distance of the cores (a point for a sphere, a segment for a capsule) and its direction; the shape moves
forward by this distance, less the radii and the gap, divided by the speed it closes at along this direction. The
distance is convex along the path: the advancement is a Newton iteration from the left, it never crosses the surface.
Measured on the tests: 4.3 iterations per hit, 9 at most; 32 at most (20 in Box3D). A sweep which would need more
gives its hit where the shape is at the 32nd iteration, before the contact: the shape never goes further than it
should. Box3D gives no hit then (`distance.c:1044-1172`).
- **The gap.** The shape stops 0.5 µm before the contact, and is on it within 1 µm (0.05 and 0.1 mm between 2 shapes
  without radius, where GJK is less sharp): the hit never overlaps the body, and the same sweep from the hit finds the
  body at the fraction 0. Box3D aims at `B3_LINEAR_SLOP` (5 mm) from the contact, within a quarter of it
  (`distance.c:1047-1052`), and the continuous collision of Feather stops 5 mm short (`ccd.go`, on the whole shapes,
  not changed): too far for a foot or a camera.
- **Plane**: the lowest point of the shape along the normal of the plane reaches it first: exact, no iteration.
- **Heightfield**: the triangles of the cells of `CellWalk`, a band of the width of the shape along its path, not the
  whole AABB of the motion; the walk ends when the band enters cells after the best hit. A triangle is skipped if the
  shape moves along its normal: it comes from behind, or leaves (`b3ShapeCastHeightField` skips the triangles whose
  plane is over the center of the shape instead).
- **Mesh**: the triangles of the leaves of `MeshCast`, the nodes entered after the best hit dropped
  (`b3ShapeCastMesh`); the same test of a triangle.
- **Not taken**: the GJK ray cast of van den Bergen, of Jolt (`GJKClosestPoint.h:506`) and PhysX (`GuGJKRaycast.h:55`):
  one loop instead of a GJK per iteration, faster, but a new algorithm in `gjk/`.

**Overlap** (`World.Overlap`): the candidates of the trees for the AABB of the shape, then the distance of the cores by
GJK against the sum of the radii; a plane by the lowest point of the shape; a heightfield or a mesh by its triangles
under the shape, from any side. The contact counts.

**Order.** At the same fraction, the body of the lowest index in `World.Bodies` is hit, and on a heightfield or a mesh
the triangle of the lowest index; `RaycastAll` is sorted by
fraction then by index, `Overlap` by index (Unity: "the order of the results is undefined"). The results don't depend
on the shape of the trees, nor on `Workers`.

**Triggers.** `QueryFilter.Triggers`, false by default: `QueryTriggerInteraction` of Unity (`Ignore`, `Collide`). In
Box2D and Box3D a sensor is a shape filtered by its category.

**Concurrency.** No lock: several readers or one writer, as the queries without lock of Jolt
(`GetNarrowPhaseQueryNoLock`, `PhysicsSystem.h:124-125`). A query during a step panics, as Box3D refuses a locked
world (`b3GetUnlockedWorldFromId`, `src/physics_world.c:95-109`).

## Kinematic bodies

A kinematic body goes where the game puts it, with the velocity of this motion, and pushes the dynamic bodies it meets;
nothing pushes it back. `kinematic.go`.

**A target per step.** `RigidBody.SetKinematicTarget` stores the pose the body reaches at the end of the next step. At
the start of the step, the body takes the velocity of the motion to its target (`World.moveKinematics`):
````
v = (target.position - position) / dt
ω = axis * angle / dt            // of the rotation target ⊗ rotation⁻¹, on the shortest arc (W ≥ 0)
````
It is the model of PhysX (`Sc::BodySim::calculateKinematicVelocity`, `ScKinematics.cpp:44-99`, 5.6.1: the same two
formulas, "we simply determine the distance moved since the last simulation frame and assign the appropriate delta to
the velocity. This vel will be used to shove dynamic objects in the solver"), whose kinematic actor has no velocity for
a step without target (`:97-98`) and is put on its target at the end of the step (`updateKinematicPose`, `:193-217`;
`PxRigidDynamic::setKinematicTarget`: "After the move is carried out during a single time step, the velocity is
returned to zero. Thus, you must continuously call this in every time step"). Jolt (`Body::MoveKinematic`,
`Body.cpp:81-95`, `MotionProperties::MoveKinematic`, `MotionProperties.inl:9-24`, v5.3.0) and Box2D
(`b2Body_SetTargetTransform`, `body.c:808-852`, v3.1.0) compute the same velocities, then keep them from a step to the
next and integrate the body with them: without a new call the body goes on, and a first order integration of the
rotation doesn't land exactly on the target. Feather follows PhysX: the target is dropped once reached, a body without
target stays, and the last sub-step writes the target bit for bit (`solver.finalizeBody`). A target on a body which is
not kinematic is refused with `actor.ErrNotKinematic`, never ignored (PhysX asserts "Body must be kinematic").

**Interpolation over the sub-steps.** The contacts are solved at each sub-step, and read the position of the body
(`currentSeparation`, from its `deltaPosition` and `deltaRotation`): at the sub-step k of n, the body is at
````
Δp = (target.position - position) * k / n
Δq = slerp(identity, target ⊗ rotation⁻¹, k / n) = rotation of angle * k / n around the axis
````
(`solver.moveKinematic`, `rotationFraction`). The slerp from the identity turns at a constant angular velocity: the
velocity ω the rows read is the velocity of the motion at every sub-step, and the body sweeps exactly the arc its
velocity says. The last sub-step takes the target itself, not the interpolation at k = n (cos and sin don't give the
quaternion back bit for bit). A body without target doesn't move: its deltas stay the identity.

**No mass.** The state of a kinematic body in the solver (`bodyState`, `dynamic` false) has no inverse mass, no inverse
inertia and no lock: an impulse changes its velocity by λ M⁻¹ d = 0 ("Static and kinematic sims have zero mass", Box2D
`body.c:535-536`; "static or kinematic bodies have infinite mass so should be treated as 1 / mass = 0",
`MotionProperties.h:94-95` of Jolt). The rows read its velocity and its motion (a contact against it is a contact
against a moving wall, a joint to it pulls the other body with it), and never write it: `jacobian.apply`,
`applyTwist`, `applyRolling`, `applyLinear` and `applyAngular` write a body only if it is dynamic, where they wrote
every body with a state before. Nothing else changes for a dynamic body: the arithmetic of a world without kinematic
bodies is the same bit for bit (the fingerprints of `bench/baseline.json` are unchanged).

Not written, a kinematic body can be shared by the rows of a color: the graph coloring takes it as a static body
(`solver.dynamicIndex`). Box2D v3 colors it as a dynamic body, for a reason Feather doesn't have: "Unlike static bodies,
we cannot use a dummy solver body for kinematic bodies. We cannot access a kinematic body from multiple threads
efficiently because the SIMD solver body scatter would write to the same kinematic body from multiple threads"
(`constraint_graph.c:20-23`, v3.1.0, the same in Box3D). A kinematic body touching 200 bodies would otherwise put its
200 contacts in 200 colors, 16 colors and the overflow here.

The trees of joints (`articulation.go`) take it as a static body too: a joint to a kinematic body is a leaf, and two
chains hanging from the same kinematic body are two trees (the kinematic body couples nothing: its mass terms are 0).
The contacts with a kinematic body keep the softness of the contacts between dynamic bodies, as Box2D v3.1
(`contact_solver.c:1514`: the stiffer softness is for a body without solver state) and Box3D (`b3_contactStaticFlag`,
`contact.c:241`, for a static body only).

**Pairs.** A kinematic body has no contact with a static or a kinematic body, as in every engine (Jolt
`Body::sFindCollidingPairsCanCollide`, `Body.inl:36-44`: "One of the bodies must be dynamic to collide"; Box2D
`simulation.md`: "A shape on a kinematic body can only collide with a dynamic body"; PhysX `PxRigidBody.h:56`:
"Kinematics will not collide with static or other kinematic objects"). The broad phase keeps it in the tree of the
dynamic bodies, with a proxy of its own kind (`proxyKinematic`): moved, it queries the dynamic tree, and the static
tree and the planes for the triggers only, and a pair is emitted if it has a dynamic body and an awake body which moves
(`needsSolving`), or a trigger ([Triggers](#triggers)). Box2D v3 gives the kinematic bodies a third tree for the same
rule ("Only dynamic proxies collide with kinematic and static proxies", `broad_phase.c:351`), its sensors query the 3
trees; two trees and a kind on the proxy do the same job here, and a body which changes type (below) keeps its leaf,
its pairs and their contacts.

**Speculative contacts from its speed.** The margin of a contact against a static body follows the relative speed of
the bodies (the speculative CCD of PhysX, see Continuous collision); the margin against a kinematic body does the same
(`World.collide`), where the pairs of dynamic bodies keep the 2 cm of Box2D. PhysX: "unlike the sweep-based CCD, it is
legal to enable speculative CCD on kinematic actors" (the guide, Advanced Collision Detection). A leg at 5 m/s moves
8 cm per step at 60 Hz: without it, the leg enters the tail it meets by 6 cm before the contact exists, and the spring
of the contact throws the tail. With it, the contact exists a step ahead, the speculative row brings the tail to the
speed of the leg within the sub-step they touch, and the leg enters it by 0.25 mm (`TestFastKinematicDoesNotGoThroughARestingBody`,
capsules and boxes). The kinematic body itself is never stopped by the continuous collision ("Kinematic bodies cannot be
stopped", Jolt `PhysicsSystem.cpp:1564`); a fast dynamic body meeting it is held by their speculative contact, and
stopped by the time of impact when they had none (see Continuous collision; Box2D sweeps its bullets only against the
kinematic bodies, `solver.c:440-446`).

**Sleep.** A kinematic body is in the island of the bodies it touches, as in Box2D ("dynamic and kinematic bodies that
are enabled need a island", `body.c:313-317`) and Jolt (`IslandBuilder::LinkBodies` for every contact constraint,
`PhysicsSystem.cpp:1278`). It rests when its velocity is exactly 0: no target, or a target at its pose. On its way it
keeps its island awake, however slowly it goes; stopped, its timer runs like the others and the island sleeps together;
a target away from its pose wakes it up (`SetKinematicTarget`), and the island with it at the step. Box2D and Jolt apply
their sleep threshold to the kinematic bodies too (`b2FinalizeBodiesTask`, `solver.c:617-651`;
`MotionProperties::AccumulateSleepTime`): a platform slower than 5 cm/s falls asleep with its riders, and stops, since
their kinematic body is moved by a velocity. With a target per step, the platform would wake up at the next step: a
sleep and a wake event at every half second, and the riders losing their velocity. PhysX keeps a kinematic actor
awake as long as a target is set ("A kinematic actor is asleep unless a target pose has been set",
`PxRigidDynamic.h:150-152`): so does Feather.

**SetBodyType**, between kinematic and dynamic, in place: the body keeps its index, its leaf in the tree (the proxy
changes kind at the next step, the pairs and their contacts stay), its joints, its filters and its island; it wakes up
with its island. It is `Body::SetMotionType` of Jolt (`Body.cpp:37-77`: the forces dropped for a kinematic body, the
velocities for a static one), which needs `mAllowDynamicOrKinematic` at the creation to switch a static body; PhysX
toggles `PxRigidBodyFlag::eKINEMATIC` on a `PxRigidDynamic`, a static actor being another class ("you do need to
provide a mass for the kinematic actor"); Box2D `b2Body_SetType` destroys the contacts of the body and recreates its
proxies (`body.c:1023-1260`). Feather refuses the static bodies (`ErrStaticBody`): a static body is the shape of the world. To
dynamic, the body keeps the velocity of its last motion (the ragdoll of a creature thrown by its animation) and needs
the mass of its shape (`ErrMasslessBody` without it): a kinematic body created with a density keeps it (`NewRigidBody`),
the solver never reads it while it is kinematic. To kinematic, the body stops and drops its forces; its locks wait for its next dynamic life.

**Teleport** places any body without velocity (`World.Teleport`): the target is dropped, a kinematic body stops, a
dynamic body keeps its velocity; the body and the sleeping bodies at the new place wake up, and the trees take it at
the next step or at `SyncQueries`. A kinematic body teleported against a resting body pushes nothing
(`TestTeleportGivesNoVelocity`), brought there by a target it pushes. PhysX on `setGlobalPose`: "the kinematic actor
would not push away other dynamic actors in its path, instead it would go right through them. The setGlobalPose()
function can still be used though, if one simply wants to teleport a kinematic actor to a new position".

**Measured** (`TestKinematicPushesABox`, `TestFastKinematicDoesNotGoThroughARestingBody`, 60 Hz, 8 sub-steps): a cube
pushed at 1 m/s on the ground (µ = 0.6) stays against the pusher within 0.00 mm, 0.12 mm deep at worst, at 1.000 m/s; a
leg at 5 m/s enters a resting capsule by 0.25 mm at worst, a box by 0.22 mm, and never goes through.
