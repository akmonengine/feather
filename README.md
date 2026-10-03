# Feather
![GitHub go.mod Go version](https://img.shields.io/github/go-mod/go-version/akmonengine/feather)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Go Reference](https://img.shields.io/badge/reference-%23007D9C?logo=go&logoColor=white&labelColor=gray)](https://pkg.go.dev/github.com/akmonengine/feather)
[![Go Report Card](https://goreportcard.com/badge/github.com/akmonengine/feather)](https://goreportcard.com/report/github.com/akmonengine/feather)
![Tests](https://img.shields.io/github/actions/workflow/status/akmonengine/feather/code_coverage.yml?label=tests)
![Codecov](https://img.shields.io/codecov/c/github/akmonengine/feather)
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/akmonengine/feather)
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues-pr/akmonengine/feather)

A Go physic library, based on the TGS Soft solver algorithm.

## Shapes
All shapes live in the `actor` package and implement `actor.ShapeInterface`.

| Shape | Definition | Narrow phase |
|---|---|---|
| `Sphere` | `Radius` | analytic against planes, spheres and capsules; its center against the other shapes (GJK distance + radius) |
| `Box` | `HalfExtents` | analytic against planes; GJK/EPA otherwise |
| `Plane` | `Normal`, `Distance` (static only) | analytic |
| `Capsule` | `HalfHeight`, `Radius`, axis along local Y | analytic against planes, spheres and capsules; its segment against the other shapes (GJK distance + radius) |
| `Heightfield` | a grid of heights (static only), 2 triangles per cell | GJK/EPA against each triangle under the body |
| `TriangleMesh` | triangles with a tree of AABBs (static only), for the decor: `NewTriangleMesh(vertices, indices)` | GJK/EPA against each triangle under the body, as the heightfield |
| `ConvexHull` | the convex hull of a cloud of points, by quickhull: `NewConvexHull(points, maxVertices)`, 256 vertices at most | GJK/EPA, its faces for the contact points; dynamic, with the mass and the inertia of its volume |

Each shape also casts a ray on itself, in its local space (`CastRay`): the world queries are built on it.

```go
// the decor: a mesh of triangles, counterclockwise seen from outside, built once (outside Step)
mesh, err := actor.NewTriangleMesh(vertices, indices) // vertices []mgl64.Vec3, indices []int32, 3 per triangle
rock := actor.NewRigidBody(actor.Transform{Rotation: mgl64.QuatIdent()}, mesh, actor.BodyTypeStatic, 0)

// a dynamic object: the hull of the vertices of its mesh, centered on its center of mass
hull, err := actor.NewConvexHull(vertices, 64)
crate := actor.NewRigidBody(actor.Transform{Position: hull.CenterOfMass(), Rotation: mgl64.QuatIdent()}, hull, actor.BodyTypeDynamic, 300)
```
A mesh and a heightfield are surfaces: their triangles are seen from the side of their normal, a body whose center went
through is not pushed back. See the [physics guide](PHYSICS_GUIDE.md#decor-meshes-and-convex-hulls) and the
[algorithms](ALGORITHMS.md#convex-hull).

```go
body := actor.NewRigidBody(
	actor.Transform{Position: mgl64.Vec3{0, 1, 0}, Rotation: mgl64.QuatIdent()},
	&actor.Capsule{HalfHeight: 0.6, Radius: 0.3},
	actor.BodyTypeDynamic,
	1000, // density
)
body.Material.StaticFriction = 0.6
body.Material.DynamicFriction = 0.5
world.AddBody(body)

body.AddForce(mgl64.Vec3{10, 0, 0}) // in N, during the next step
world.Step(1.0 / 60.0)
```

## Collision filtering
Each body has a layer (one bit among 32) and a mask, the layers it collides with. 2 bodies collide if each one has its
layer in the mask of the other. By default a body is on `actor.LayerDefault` and collides with every layer.

```go
const (
	layerWorld actor.Layers = 1 << iota
	layerPawn
	layerDebris
)
pawn.Layer, pawn.Mask = layerPawn, layerWorld|layerPawn  // the pawns don't collide with the debris
debris.Layer, debris.Mask = layerDebris, layerWorld      // the debris only collide with the world

world.IgnoreCollision(tail, pelvis, true) // this pair never collides, whatever its layers
world.AddJoint(joint)                     // neither do the 2 bodies of a joint, unless joint.CollideConnected

decor.Mask = actor.NoLayers               // "queries only": no contact, seen by the queries on its layer
filter := feather.QueryFilter{Mask: layerWorld | layerDebris, Excluded: []*actor.RigidBody{pawn}}
```
The filtered pairs leave in the broad phase: no narrow phase, no contact, no event, nobody wakes up. The triggers and
the continuous collision go through the same filter. See the [physics guide](PHYSICS_GUIDE.md#collision-filtering) and
the [algorithms](ALGORITHMS.md#collision-filtering).

## Queries
The world answers 3 questions about its bodies, static or dynamic, awake or asleep: what is on this segment (a ray),
what does this shape touch first if it moves there (a sweep), what is in this volume (an overlap). A query takes an
origin and a translation, and gives its hit at a fraction of the translation, from 0 to 1.

```go
filter := feather.DefaultQueryFilter() // every layer, without the triggers
filter.Excluded = []*actor.RigidBody{character}

// the ground under a foot: the first body on 2 m down
if hit, ok := world.Raycast(foot, mgl64.Vec3{0, -2, 0}, filter); ok {
	_ = hit.Body     // the body, hit.Triangle on a heightfield
	_ = hit.Point    // foot + hit.Fraction * translation
	_ = hit.Normal   // out of the body
}

// every body on a line of sight, sorted by fraction, in a buffer reused from a call to the next
hits = world.RaycastAll(eye, target.Sub(eye), filter, hits[:0])

// a camera arm: how far a sphere of 20 cm can move back before it touches something
ball := &actor.Sphere{Radius: 0.2}
hit, ok := world.Sweep(ball, actor.Transform{Position: head, Rotation: mgl64.QuatIdent()}, back, filter)

// the bodies in a blast of 3 m, in the order of world.Bodies
bodies = world.Overlap(&actor.Sphere{Radius: 3}, actor.Transform{Position: center, Rotation: mgl64.QuatIdent()}, filter, bodies[:0])

world.SyncQueries() // after the game added, moved or reshaped bodies, if a query must see them before the next Step
```
- A ray is exact on each shape. A moving shape (a sphere, a capsule, a box) stops within 1 µm of the body it hits
  (0.1 mm between 2 boxes), never in it.
- A ray or a shape which starts in a body hits it at the fraction 0, the normal against its direction.
- The top side of a heightfield only is hit; a hole is not. The side of the normal of the triangles of a mesh; its
  `hit.Triangle` is the index of the triangle.
- At the same fraction, the body of the lowest index in `World.Bodies` is hit: the results don't depend on the workers
  nor on the shape of the trees.
- No allocation, no write: any number of goroutines can run queries together while nothing writes the world. A query
  during `Step` panics.

See the [physics guide](PHYSICS_GUIDE.md#queries) and the [algorithms](ALGORITHMS.md#queries).

## Axis locks
A dynamic body can be locked along (translation) or around (rotation) the world axes X, Y and Z, as the
`Rigidbody.constraints` of Unity: a character which stays upright, a game in a plane, a platform on a rail.

```go
character.SetLocks(actor.NoAxes, actor.AxisX|actor.AxisZ)            // it only turns around Y: it stays upright
pawn.SetLocks(actor.AxisZ, actor.AxisX|actor.AxisY)                  // a game in 2D, in the plane XY
lift.LinearLock, lift.AngularLock = actor.AxisX|actor.AxisZ, actor.AllAxes // before the first step: the fields
```
A locked axis has no inverse mass (or inverse inertia) in the solver and in the integration: neither the gravity, the
forces, the impulses, the contacts nor the joints move it, and its coordinate is kept bit for bit. Around its free axes
a locked body has its exact inertia, the one of a body on an axle. A body without lock is simulated bit for bit as
before. See the [physics guide](PHYSICS_GUIDE.md#axis-locks) and the
[algorithms](ALGORITHMS.md#axis-locks).

## Kinematic bodies
A kinematic body goes where the game puts it and pushes the dynamic bodies it meets; nothing pushes it back: a moving
platform, a character, the bones of a creature which follow its animation while its tail is simulated. It is the
kinematic body of PhysX, Jolt, Box2D, Unity and Unreal.

```go
platform := actor.NewRigidBody(transform, &actor.Box{HalfExtents: mgl64.Vec3{2, 0.1, 2}}, actor.BodyTypeKinematic, 500)
world.AddBody(platform)

// each step: the pose the platform reaches at the end of the step; it goes there over the sub-steps
next := platform.Transform
next.Position = next.Position.Add(mgl64.Vec3{1, 0, 0}.Mul(dt))
err := platform.SetKinematicTarget(next)      // actor.ErrNotKinematic on a body which is not kinematic
world.Step(dt)                                   // the platform is at next, bit for bit; platform.Velocity is (next - previous) / dt

world.Teleport(platform, start)                  // placed without velocity: it pushes nothing
err = world.SetBodyType(bone, actor.BodyTypeDynamic)   // a ragdoll: the bone falls, with its contacts, its joints and its island
err = world.SetBodyType(bone, actor.BodyTypeKinematic) // and follows its targets again; ErrStaticBody, ErrMasslessBody refuse
```
- One target per step: it is reached at the end of the step and dropped. Without target the body stays, with no
  velocity (the kinematic actors of PhysX). Its velocity is the one of its motion: the contacts push with it.
- Infinite mass for the solver: no force, no gravity, no impulse, no contact moves it. A dynamic body it meets takes its
  velocity, friction included (a stack rides a platform, a leg pushes a tail). The contact exists before they touch,
  from the speed of the kinematic body: a leg at 5 m/s doesn't enter the resting body on its path.
- No contact between a kinematic body and a static or a kinematic body: no narrow phase, no collision event. A
  kinematic body enters and leaves the triggers, static or not, and a kinematic trigger detects every body
  ([Triggers](#triggers)).
- On its way, a kinematic body keeps awake the bodies it touches; stopped, it sleeps with them; its next target wakes
  them all up.
- The planes and the heightfields stay static.

See the [physics guide](PHYSICS_GUIDE.md#kinematic-bodies) and the [algorithms](ALGORITHMS.md#kinematic-bodies).

## Triggers
A trigger (`IsTrigger`) is not solved: it reports the bodies which overlap it, with the events of the world. A pair of
a trigger and a body enters once, stays, and exits once: when the shapes no longer overlap, when one of the bodies
leaves the world, never because one of them falls asleep. Counting the enters and the exits tells who is in a zone.

```go
zone := actor.NewRigidBody(transform, &actor.Box{HalfExtents: mgl64.Vec3{2, 1, 2}}, actor.BodyTypeStatic, 0)
zone.IsTrigger = true
world.AddBody(zone)

occupants := 0
world.Events.Subscribe(feather.EventTriggerEnter, func(feather.Event) { occupants++ })
world.Events.Subscribe(feather.EventTriggerExit, func(feather.Event) { occupants-- })
```
- A body in contact with a trigger is in it: the overlap is the one of `World.Overlap`.
- A body asleep in a trigger stays in it, without stay events while both rest (the overlap can't change). Its pair is
  kept in the broad phase whatever the sleep, as the sensors of Box2D v3, and is tested again when the game moves one of
  the bodies (its AABB changes), as the trigger pairs of PhysX.
- `RemoveBody` on a body in a trigger, or on the trigger, ends their pairs: the exits are sent with the events of the
  next step (or in the events running, if a listener removed the body).
- A kinematic body enters and leaves a static trigger, and a kinematic trigger detects the static and the kinematic
  bodies (as Box2D v3 and Unity), without any contact. 2 static bodies never pair.
- A trigger is filtered like any body ([Collision filtering](#collision-filtering)).
- The collision events follow the same rules: a contact ends when the bodies stop touching or one of them is removed
  (`EventCollisionExit` at the next events), never because a body falls asleep, on the ground or on another body, and
  sends no stay while both rest.

See the [physics guide](PHYSICS_GUIDE.md#triggers) and the [algorithms](ALGORITHMS.md#triggers).

## TGS Soft
TGS Soft (or "Soft Step") is the solver of Box2D v3, described by Erin Catto in Solver2D.
It is made of substeps, soft constraints, warm starting and relaxation:
````
while simulating do
    contacts ← CollectContacts();   // once per step, with speculative contacts
    h ← Δt/numSubsteps;
    PrepareContacts(contacts);      // anchors, effective masses, previous impulses

    for numSubsteps do
        for n bodies do
            v ← v + h*(g + f_ext/m);
            ω ← ω + h*I⁻¹(τ_ext - ω × Iω);
        end
        WarmStart(contacts);        // apply the impulses of the previous substep
        Push(contacts);             // soft constraint: remove the overlap
        for n bodies do
            x ← x + h*v;
            q ← q + h/2 * ω*q;
        end
        Relax(contacts);            // rigid constraint + friction, removes the energy of the soft constraint
    end

    ApplyRestitution(contacts);
    StoreImpulses(contacts);        // warm start of the next step
end
````

- The contacts are computed only once per step: during the substeps, the separation of each contact point is updated from the motion of both bodies.
- The soft constraint is a spring + damper, set with a frequency (`World.ContactHertz`, 30 Hz by default, as Box2D v3.1) and a damping ratio.
- Contacts exist before the bodies touch (speculative contacts), so fast bodies don't go through thin walls.
- Friction follows Coulomb's law: static friction when the contact sticks, dynamic friction when it slides.
- The simulation is deterministic: same result bit for bit, whatever the number of `Workers`.
- The solver is parallel: the contacts are split into colors (graph coloring), the contacts of a color don't share any body.
- A step doesn't allocate memory (after the first steps).
- The bodies touching each other sleep and wake up together (islands).

### Why not XPBD anymore
Up to v0.2.0, Feather used a simplified XPBD solver. The same scenes (`bench/`), each version at its own setting:
TGS Soft at 60 Hz with 8 substeps (the setting of Feather for the games), v0.2.0 at 50 Hz with 12 substeps (the
setting AkmonEngine ran it with; v0.2.0 has no default):

| Scene | v0.2.0 (XPBD) | TGS Soft | Expected |
|---|---|---|---|
| Pyramid of 55 boxes, 3 s | explodes (top box at 93 m) | stands (4.744 m) | 4.750 m |
| Box on a 20° slope, µ = 0.6 | slides 9.9 m | 0 m | 0 m |
| Box on a 35° slope, µ = 0.3 | slides 16.9 m | 9.657 m | 9.648 m |
| Bounce from 1 m, restitution 0.5 | 0.06 m | 0.23 m | 0.25 m |
| 10 N during 1 s on 32.7 kg | 15279 m/s | 0.306 m/s | 0.306 m/s |
| Same scene, run twice | 38/40 bodies differ | identical | identical |
| EPA sphere-box normal (p99) | 2.7° | 0.03° | 0° |
| Step, 10 / 100 / 500 bodies resting on the ground (one layer of boxes & spheres), 1 worker | 0.39 / 1.82 / 8.3 ms | 0.017 / 0.11 / 0.57 ms | |

```
cd bench
go run .                                            # current version
go run -tags v020 -modfile=go.v020.mod .            # v0.2.0
```

A heavier scene, 500 boxes & spheres falling on each other (`BenchmarkWorldStep`, 60 Hz, 8 substeps), takes ~2.1 ms
per step on 1 worker, ~0.8 ms on 8 workers; 2000 bodies awake, 6.5 ms and 2 ms.

### Constraints
- Contact: generated when a collision is detected between two rigid bodies, up to 4 points (manifold), with friction,
  rolling & spinning resistance and restitution.
- Distance: fixed length, a range [min, max] (a rope), or a spring. Usage: ropes, chains, springs
- Ball (ball and socket): the anchors stay together, with an optional elliptic cone for the swing and a range for the twist,
  and an optional drive towards a target rotation. Usage: ragdolls, physical bones, tails
- Hinge: rotation around one axis only, with an optional angle range, motor and spring. Usage: doors, wheels, knees
- Fixed: the position and the rotation of the 2 bodies are frozen together
- Configurable: each of the 6 axes is locked, limited or free, with optional drives. Usage: sliders, shoulders, vehicles,
  anything the other joints don't cover

```go
hinge := feather.NewHingeJoint(frame, door, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{0, 1, 0}) // anchor, axis (world space)
hinge.EnableLimit = true
hinge.LowerAngle, hinge.UpperAngle = -math.Pi/2, math.Pi/2
world.AddJoint(hinge)
```

## GJK
Detects if two convex shapes overlap. With a margin, it also detects the shapes closer than the margin (speculative contacts).

## EPA
Computes the penetration depth, the normal and the witness points. The contact points are then clipped between the faces
of both shapes (Sutherland-Hodgman), each point with its own separation.

See [ALGORITHMS.md](ALGORITHMS.md), [ARCHITECTURE.md](ARCHITECTURE.md) and the [physics guide](PHYSICS_GUIDE.md).

## Tests & benchmarks
- **Minimal scenes** (`scenes_test.go`): one mechanism each (a box landing on a corner, a capsule spinning like a top,
  a sphere in a V...), with a bound derived from the engine (`LinearSlop`), never from a measure.
- **Invariants** (`invariants_test.go`): 60 random scenes checked at every step: finite values, unit quaternions,
  1 and 8 workers giving the same bits, no body in a plane, no energy gained, momentum & angular momentum kept in
  free flight.
- **Scenes of Solver2D** (`bench/scenes`): the samples of Erin Catto's Solver2D in 3D, each checked, and compared to
  Box2D v3.1 on the same scenes (the reference: Feather must do at least as well; the known gaps are followed by #821).
- **Regressions** (`bench/`): 8 scenes (piles, pyramid, joint chain, rain on a terrain, spinning tops, locked bodies),
  2 scenes of queries and the scenes of Solver2D against a committed reference: fingerprint, quality and speed per phase
  (`World.Profile`). The measures decided by the rounding (the depth of a pile...) are followed by their median over
  32 variants of their scene.
- **Queries** (`query_*_test.go`): each query against a walk of every body, on 300 bodies of every kind.
````
go test ./...
cd bench && go run . -check     # exit 1 on a regression
cd bench && go run . -update    # after a wanted change
cd bench && go test ./...       # the scenes of Solver2D
cd bench && go run . -scenes    # the scenes at full size (-compare: with v0.2.0)
cd bench && go run . -queries   # the cost of a ray, a sweep, an overlap
````

## Sources
- https://box2d.org/posts/2024/02/solver2d/
- https://github.com/erincatto/box2d (v3)
- https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
- https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf (gyroscopic torque)
- https://github.com/bepu/bepuphysics2
- https://cse442-17f.github.io/Gilbert-Johnson-Keerthi-Distance-Algorithm/
- https://winter.dev/articles/epa-algorithm
- Christer Ericson, Real-Time Collision Detection (2004)
- https://github.com/jrouwe/JoltPhysics (active edges, contact patches, body pair cache)
- W. J. Stronge, Impact Mechanics (2000): Poisson's hypothesis for the restitution
- Brian Mirtich, Impulse-based Dynamic Simulation of Rigid Body Systems (1996): conservative advancement
- Solver2D, the samples of the reference scenes: https://github.com/erincatto/solver2d (MIT)
- Collision filtering: Box2D v3.1 (`b2Filter`, `b2ShouldShapesCollide`, the filter joint), Jolt 5.3
  (`ObjectLayerPairFilterMask`, `GroupFilterTable`), PhysX 5.6 (`PxFilterData`, `PxShapeFlag`,
  https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/RigidBodyCollision.html#collision-filtering),
  Unreal "Query Only" (https://dev.epicgames.com/documentation/en-us/unreal-engine/collision-response-reference-in-unreal-engine),
  Unity `Physics.IgnoreCollision` (https://docs.unity3d.com/ScriptReference/Physics.IgnoreCollision.html)
- Spinning resistance: Bullet 3.25 (`btCollisionObject::setSpinningFriction`, `setupTorsionalFrictionConstraint`;
  https://github.com/bulletphysics/bullet3), PhysX 5.6 (`PxShape::setTorsionalPatchRadius`), Box3D & Box2D v3.1 (the
  rolling resistance)
- Queries: Box3D (`b3World_CastRay`, `b3World_CastShape`, `b3World_OverlapShape`, `b3DynamicTree_RayCast`,
  `b3ShapeCast`, `b3ShapeCastHeightField`; https://github.com/erincatto/box3d, commit 9f998c8), Box2D v3.1 ("refit BVH",
  docs/simulation.md), Jolt 5.3 (`RaySphere.h`, `RayCapsule.h`, `RayAABox.h`, `PlaneShape::CastRay`, `RayCast.h`),
  PhysX 5.6 (`PxSceneQuerySystem.h`, `GuRaycastTests.cpp`), Unity (`Physics.Raycast`, `Physics.SyncTransforms`,
  `QueryTriggerInteraction`), Unreal (`FHitResult::bStartPenetrating`,
  https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Engine/FHitResult)
- John Amanatides & Andrew Woo, A Fast Voxel Traversal Algorithm for Ray Tracing (Eurographics 1987): the walk of a
  grid by a ray
- Triggers, contacts & sleep: Box2D v3.1.1 (`src/sensor.c`: `b2SensorTask`, `b2SensorQueryCallback`;
  `b2SensorEndTouchEvent`; docs/simulation.md, "Sensors do not consider sleep"; `src/solver_set.c`:
  `b2TrySleepIsland`; `src/contact.c`: `b2DestroyContact`, `b2ContactEndTouchEvent`), PhysX (`ScTriggerInteraction.cpp`:
  `onActivate`, `onDeactivate`, `PROCESS_THIS_FRAME`; `PxTriggerPair`, `eNOTIFY_TOUCH_LOST`), Jolt 5.6
  (`Body::SetIsSensor`, `Body::SetCollideKinematicVsNonDynamic`, `Body::UpdateSleepStateInternal`,
  `ContactListener::OnContactRemoved`), Godot 4 (`Area3D.body_exited`, `GodotAreaPair3D`, `JoltArea3D`), Unity
  (`MonoBehaviour.OnTriggerStay`, `MonoBehaviour.OnTriggerExit`, `MonoBehaviour.OnCollisionStay`; Manual, Collider types
  interaction)
- Axis locks: Jolt 5.3 (`EAllowedDOFs`, `MotionProperties::GetInverseInertiaForRotation`), Rapier 0.22 (the inverse
  mass by axis, `effective_inv_mass`), Box3D & Box2D (`b3MotionLocks`, `b2MotionLocks`), PhysX 5.6
  (`PxRigidDynamicLockFlag`), Unity `Rigidbody.constraints`
  (https://docs.unity3d.com/ScriptReference/Rigidbody-constraints.html)
- Friction center & lever arms: Box3D (`b3PrepareContacts_Mesh`: `centerA`, `leverArm`; commit 9f998c8), where Jolt 5.3
  (`ContactConstraintManager.cpp`) and Box2D v3.1 (`src/contact_solver.c`) solve the friction at each contact point
- Dynamic AABB tree, collision margin & conservative advancement: Bullet 3.25 (`btDbvt` & `btDbvtBroadphase`; the margin
  of `btSphereShape` & `btCapsuleShape`, a point & a segment with their radius, added by `btGjkPairDetector`;
  `btContinuousConvexCollision`)
- PhysX speculative CCD & Unity "Continuous Speculative": https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/AdvancedCollisionDetection.html,
  PhysX 5.6 (`Sc::BodySim::updateContactDistance` in `ScCCD.cpp`, `PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD`)
- Continuous collision of the fast bodies: Box2D v3.1 (`b2SolveContinuous`, the fast body of `b2FinalizeBodiesTask`,
  `src/solver.c`; the bullets of `docs/simulation.md`), Box3D (`b3SolveContinuous`, `safetyFactor`), Jolt 5.3
  (`EMotionQuality::LinearCast`, `JobFindCCDContacts` & `JobResolveCCDContacts` in `PhysicsSystem.cpp`,
  `mLinearCastThreshold`), PhysX 5.6 (the sweep-based CCD of `PxsCCD.cpp`, the guide Advanced Collision Detection)
- Convex hull: Barber, Dobkin & Huhdanpaa, The Quickhull Algorithm for Convex Hulls (ACM TOMS, 1996); Dirk
  Gregorius, Implementing QuickHull (GDC 2014: the tolerance, the merging of the faces); Jolt 5.3
  (`ConvexHullBuilder`, `ConvexHullShape`: the inertia by the covariance of the tetrahedra, `GetSupportingFace`), Box3D
  (`b3CreateHull`, `src/hull.c`: the initial tetrahedron, the thresholds, `b3RayCastHull`, commit 9f998c8), PhysX
  (`QuickHullConvexHullLib`, `PxConvexMeshDesc::vertexLimit`), Bullet 3.25 (`btConvexHullComputer`); Jonathan Blow &
  Atman Binstock, How to find the inertia tensor (or other mass properties) of a 3D solid body represented by a triangle
  mesh (the covariance of the canonical tetrahedron)
- Triangle mesh: Box3D (`b3CreateMesh`, `b3SplitBinnedSah`, `b3IdentifyEdges`, `b3RayCastMesh`, `b3ShapeCastMesh`,
  `b3QueryMesh`, `src/mesh.c`; `b3ComputeMeshManifolds`, `src/mesh_contact.c`), Jolt 5.3 (`MeshShape`,
  `AABBTreeBuilder`, `TriangleSplitterBinning`, `ActiveEdges`, `Indexify`, `RayTriangle.h`), PhysX (`BV4_AABBTree`,
  `EdgeList::computeActiveEdges`, `PCMConvexVsMeshContactGeneration`), Bullet 3.25 (`btQuantizedBvh::buildTree`: the
  balance of the split), Ericson 6.2.1 (top-down construction of a bounding volume hierarchy), Möller & Trumbore, Fast,
  Minimum Storage Ray/Triangle Intersection (1997)
- Kinematic bodies: PhysX 5.6 (`PxRigidDynamic::setKinematicTarget`, `Sc::BodySim::calculateKinematicVelocity` &
  `updateKinematicPose` in `ScKinematics.cpp`, the guide Rigid Body Dynamics > Kinematic Actors,
  https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/RigidBodyDynamics.html), Jolt 5.3 (`Body::MoveKinematic`,
  `Body::SetMotionType`, `Body::sFindCollidingPairsCanCollide`, `MotionProperties::MoveKinematic`), Box2D v3.1 & Box3D
  (`b2Body_SetTargetTransform`, `b2Body_SetType`, the kinematic tree of `broad_phase.c`, the kinematic bodies of
  `constraint_graph.c`, the sleep of `b2FinalizeBodiesTask`)

## Acknowledgements
Feather is written from the publications, the documentation and the source code of these projects:
- [Box2D](https://github.com/erincatto/box2d), by Erin Catto: the TGS Soft solver (Solver2D, Soft Constraints),
  the graph coloring, the continuous collision, the category & mask bits of the collision filter
- [Box3D](https://github.com/erincatto/box3d), by Erin Catto: the reference of the bench, the friction center and its
  lever arms, the queries (an origin and a translation, the ray through a tree, the ray on a sphere), the thresholds of
  quickhull and the ray on a hull, the tree of a mesh (the binned surface area heuristic, the leaves of 4 triangles,
  the casts through it) and one generator of contacts for the meshes and the height fields
- [Jolt Physics](https://github.com/jrouwe/JoltPhysics), by Jorrit Rouwe: the active edges of the terrains and of
  the meshes, the contact patches, the body pair cache, the allowed degrees of freedom (the axis locks), the cast of a
  fast body against every body and the relative cast of two fast bodies, the hull centered on its center of mass with
  the inertia of its volume, the supporting face of a hull, the weld of the vertices of a mesh
- [Bullet](https://github.com/bulletphysics/bullet3), by Erwin Coumans: the spinning friction (the spinning resistance),
  the dynamic AABB tree (`btDbvt`), the collision margin (the cores of the rounded shapes),
  the conservative advancement of the time of impact, the balance of the splits of the tree of a mesh
- [PhysX](https://github.com/NVIDIA-Omniverse/PhysX), by NVIDIA: the speculative CCD, the kinematic actors (a target
  per step, reached at the end of the step, no velocity without target), the vertex limit of a convex hull

## Contributing Guidelines

See [how to contribute](CONTRIBUTING.md).

## Licence
This project is distributed under the [Apache 2.0 licence](LICENCE.md).
