# Feather - Physics Guide

## Units
Feather uses the SI units: meters, kilograms, seconds, newtons.
- `AddForce` in N, `AddTorque` in N·m, both applied during the next `World.Step`.
- `Velocity` in m/s, `AngularVelocity` in rad/s (world space).
- `Gravity` in m/s².

## Materials

### Density (kg/m³)
The mass and inertia come from the density and the volume of the shape.

| Material | Density (kg/m³) |
|----------|----------------|
| Wood (Balsa) | 160 |
| Wood (Pine) | 500-600 |
| Wood (Oak) | 700-900 |
| Ice | 917 |
| Water | 1000 |
| Concrete | 2400 |
| Glass | 2500 |
| Stone (Granite) | 2750 |
| Steel | 7850 |
| Lead | 11340 |

### Friction
`StaticFriction` is used while the contact sticks, `DynamicFriction` while it slides (above 1 cm/s). The friction of
a contact acts at the center of its points, along the surface and around the normal (a box turning on the ground).
The friction of a contact is the geometric mean of both bodies: `sqrt(µA * µB)`.
Note: a body with a friction of 0 removes the friction of all its contacts, including with the ground.

A box stays on a slope when `tan(angle) < µ`.

| Surfaces | Friction |
|----------|----------|
| Ice | 0.05 |
| Wood on wood | 0.3 - 0.5 |
| Rubber on concrete | 0.8 - 1.0 |

### Restitution (0.0 to 1.0)
The restitution of a contact is the average of both bodies. A ball dropped from a height h bounces back to `e² * h`.
There is no bounce under 1 m/s of impact (`RestitutionThreshold`), so resting bodies don't jitter.

| Value | Behavior | Real Materials |
|-------|----------|----------------|
| 0.0 | No bounce | Clay, wet sand |
| 0.1-0.2 | Minimal bounce | Lead, wet wood |
| 0.3-0.4 | Slight bounce | Concrete, hard wood |
| 0.5-0.6 | Moderate bounce | Hard plastic, stone |
| 0.7-0.8 | High bounce | Rubber, basketballs |
| 0.9 | Very high bounce | Super balls |

### Rolling resistance
`RollingResistance` (usually 0 to 1, 0 by default) slows down the rolling spheres and capsules. Without it, a ball rolls forever
on a flat ground, and a scene with balls never sleeps. The contact uses the largest value of both bodies, times the largest
radius (0 for a box). A ball rolling at v stops after `v² / (2 * 5/7 * resistance * g)`.

### Spinning resistance
`SpinningResistance` (without unit, 0 by default) slows down the spheres and capsules spinning on their contact, like a
top. Without it, a ball spinning around the vertical on its single point of contact never stops, and never sleeps.
The contact uses the largest value of both bodies, times the largest radius (0 for a box, whose points already hold the
twist by their lever arms): it holds a torque of `resistance * radius * normal force` around its normal, and nothing
around the other axes (a rolling ball is not slowed down, see `RollingResistance`). It doesn't depend on the friction
of the bodies: a ball without friction brakes its spin at the same rate.

The largest radius of both shapes is used: a ball of radius 10 cm spinning on top of a static ball of radius 1 m brakes
10 times faster than on a flat ground (the same holds for the rolling resistance). Give such a large round body a
smaller resistance, or none.

A ball of radius r spinning at ω slows down at `resistance * r * g / (2/5 * r²) = 5/2 * resistance * g / r` (rad/s²),
and stops after `ω / (5/2 * resistance * g / r)`: 1.6 s at 20 rad/s for a ball of radius 10 cm with a resistance of
0.05. It falls asleep 0.5 s after it stopped.

The resistance stands for the width of the contact: a patch of radius `a` pressed uniformly holds a torque of
`2/3 * µ * a * normal force`, so `resistance = 2/3 * µ * a / r`. The µ of this formula is part of the value you choose:
the engine doesn't multiply the resistance by the friction of the materials.

| Value | Radius of the patch (µ = 0.6) |
|-------|-------------------------------|
| 0 | a point: spins forever |
| 0.01 | 2.5 % of the radius of the ball |
| 0.05 | 12.5 % |
| 0.2 | 50 % |

### Damping
`LinearDamping` and `AngularDamping` (1/s) slow the body down: `v = v / (1 + h * damping)` at each substep.

## Simulation

### Joints
```go
ball := feather.NewBallJoint(parent, child, anchor, twistAxis) // world space
ball.EnableSwingLimit, ball.SwingLimitY, ball.SwingLimitZ = true, 0.5, 0.3 // rad
ball.EnableTwistLimit, ball.TwistMin, ball.TwistMax = true, -0.2, 0.2
world.AddJoint(ball)
```
- `Hertz` & `DampingRatio`: the softness of the joint (60 Hz and 2 by default, capped at 1/4 of the substeps rate).
- `CollideConnected` (false by default): the 2 bodies of the joint don't collide with each other. Set it before
  `AddJoint`: it is read when the joint is added (see [Collision filtering](#collision-filtering)).
- The drive of the ball joint (`DriveTarget`, `DriveHertz`, `DriveDampingRatio`) brings the child to a target rotation,
  like a muscle: a damping ratio of 1 reaches it without overshoot.
- The motor of the hinge turns at `MotorSpeed` with at most `MaxMotorTorque`.
- A body removed from the world removes its joints.

```go
slider := feather.NewConfigurableJoint(frame, carriage, anchor, mgl64.Vec3{1, 0, 0}) // everything locked
slider.LinearMotion[0] = feather.MotionLimited
slider.LinearMin, slider.LinearMax = mgl64.Vec3{-1, 0, 0}, mgl64.Vec3{1, 0, 0}
world.AddJoint(slider)
```
- The configurable joint sets each axis: `MotionLocked`, `MotionLimited` or `MotionFree`. 3 linear axes, the twist
  and 2 swings (a cone if both are limited).
- Its drives bring B to `DriveTargetPosition` and `DriveTargetRotation` (in the frame A).
- The limits are soft: a huge force bends them a little (under 0.5° for 5 g at the end of an arm).

### Axis locks
A dynamic body can be locked along the world axes (`LinearLock`: it doesn't move along them) and around them
(`AngularLock`: it doesn't turn around them). It is the `constraints` of a `Rigidbody` in Unity (Freeze Position,
Freeze Rotation).

```go
body.LinearLock, body.AngularLock = actor.AxisX, actor.NoAxes   // before the first step
body.SetLocks(actor.NoAxes, actor.AxisX|actor.AxisZ)            // later: it clears the locked velocities & wakes the body up
```

| Recipe | `LinearLock` | `AngularLock` |
|--------|--------------|---------------|
| A character which stays upright | `NoAxes` | `AxisX \| AxisZ` |
| A crate which never tips over | `NoAxes` | `AllAxes` |
| A game in 2D, in the plane XY | `AxisZ` | `AxisX \| AxisY` |
| A lift, a platform on a vertical rail | `AxisX \| AxisZ` | `AllAxes` |

- The axes are the ones of the world, whatever the rotation of the body. A body locked around X and Z, created leaning,
  keeps its lean and turns around the vertical.
- A locked axis is kept bit for bit: neither the gravity, `AddForce`, `AddTorque`, the impulses, the contacts, the joints
  nor the continuous collision move it. A velocity written along a locked axis is cleared by the next step.
- The body has no mass along a locked axis: what hits it along this axis hits a wall. Along its free axes it keeps its
  mass, its friction and its bounce.
- `SetLocks` wakes the body up: a body freed in the air falls. Writing the fields of a sleeping body doesn't.
- A body with all its axes locked stays dynamic: it never moves, carries what rests on it, sleeps and wakes up, and
  collides with the static bodies (events). A static body costs less: prefer it for what never moves.
- A fast body is stopped by a body with all its axes locked as by any body (speculative contact, then time of impact).
  A static body still costs less for a wall which never moves.
- A static body has no lock: `SetLocks` does nothing on it.
- A joint which needs a locked axis can't be satisfied: its other rows are still solved (a body which only turns around
  Y, held by a fixed joint, doesn't turn), the impossible ones are left out. Nothing explodes.
- To move a locked body along its locked axis, write its `Transform` (then `UpdateAABB`), as for any body.
- The rotation locks are in world space, as Jolt and PhysX; Unity locks the rotation in the inertia space of the body.
  It is the same thing for a body whose inertia axes are the ones of the world (an upright capsule, a box not leaning).
- A leaning body locked around some axes turns as on an axle: with its moment of inertia around its free axes (a rod
  leaning by 45° which only turns around Y is as heavy to spin as it looks). Jolt, Rapier, PhysX and Box3D make it
  lighter than it is; see the [algorithms](ALGORITHMS.md#axis-locks).

### Kinematic bodies
A kinematic body is moved by the game, not by the forces: a lift, a moving platform, a door, a character, the bones of
a creature which follow its animation while its tail is simulated. It pushes the dynamic bodies it meets with the
velocity of its motion, carries what rests on it, and nothing pushes it back. It is the kinematic body of PhysX
(`setKinematicTarget`), Unity (`isKinematic` + `MovePosition`), Jolt (`MoveKinematic`) and Unreal.

```go
lift := actor.NewRigidBody(actor.Transform{Position: start, Rotation: mgl64.QuatIdent()}, &actor.Box{HalfExtents: mgl64.Vec3{1, 0.1, 1}}, actor.BodyTypeKinematic, 500)
lift.Material.StaticFriction, lift.Material.DynamicFriction = 0.6, 0.6 // the friction of its surface, for the riders
world.AddBody(lift)

// each frame, before Step: where the lift is at the end of the step
next := lift.Transform
next.Position = next.Position.Add(mgl64.Vec3{0, 0.5, 0}.Mul(dt))
lift.SetKinematicTarget(next)
world.Step(dt)
```
- **One target per step.** `SetKinematicTarget` gives the pose the body reaches at the end of the next step: the body
  goes there over the sub-steps (position linearly, rotation along the shortest arc), its velocity is the one of this
  motion (`Velocity`, `AngularVelocity`: (target - pose) / dt, read after the step), and the target is dropped. Give one
  before each step to move the body along a path; without target the body stays where it is, with no velocity. A
  velocity written on a kinematic body is ignored: move it by its target.
- **What it carries follows it.** A body resting on a platform moves with it, by friction (give the platform a
  friction); a body it meets is pushed at its speed. The contact exists before they touch, from the speed of the
  kinematic body: a leg at 5 m/s doesn't enter the tail on its path. A kinematic body pushing a body against a wall
  squeezes it: the body enters the wall, as in every engine.
- **No contact with the static and the kinematic bodies**: a kinematic body in the ground, through a wall or through
  another kinematic body is not pushed out, and sends no collision event. It enters and leaves the triggers, static or
  not, and a kinematic trigger detects every body ([Triggers](#triggers)).
- **Sleep.** On its way, a kinematic body keeps awake the bodies it touches, however slowly it goes. Stopped (no
  target), it falls asleep with them half a second later; its next target wakes them all up.
- **Teleport.** `world.Teleport(body, transform)` places a body without any velocity: a kinematic body brought there by
  a target pushes what is on its path, a teleported one goes through it (it overlaps it, and the contact pushes them
  apart at the next steps). For a cut scene, a respawn; `Teleport` works on every body, and wakes the sleeping bodies at
  the new place.
- **Forces, torques and impulses** are ignored, so is the gravity. The locks (`LinearLock`, `AngularLock`) are kept for
  the day the body becomes dynamic.
- **The animated bones of a creature.** Each bone is a kinematic body: at each step, give it the pose of its bone in the
  animation. The simulated parts (a tail, a bag, hair) are dynamic bodies linked by joints, which hit the kinematic
  bones and are pushed by them.
- **Ragdoll.** `world.SetBodyType(bone, actor.BodyTypeDynamic)`: the bone falls with the velocity of its last motion,
  and keeps its contacts, its joints, its layers and the pairs it ignores; `SetBodyType(bone, actor.BodyTypeKinematic)`
  stops it and gives it back to the animation. Create the bones with their density: a body without mass can't become
  dynamic (`SetBodyType` panics). A static body stays static (it panics too): it is the shape of the world.
- **The continuous collision** never stops a kinematic body. A fast dynamic body meeting it is held by their speculative
  contact (from their relative speed), and stopped by the time of impact when they had none: a thin moving wall is a
  kinematic body.
- A plane or a heightfield stays static: a moving terrain is not supported.

### Characters
A character is a capsule which walks: the player, a creature of the game. It is virtual (the `CharacterVirtual` of
Jolt, whose name it takes, the `CharacterController` of Unity, the mover of Box2D v3): the game gives its velocity and calls its update,
which slides the capsule along the walls, climbs the slopes and the steps, finds the ground and pushes the dynamic
bodies; no force, no contact of the solver moves it, so it goes exactly where the game decides, at once.

```go
hero := feather.NewCharacterVirtual(actor.Capsule{HalfHeight: 0.55, Radius: 0.3}, feet.Add(mgl64.Vec3{0, 0.85, 0}))
hero.MaxSlopeAngle, hero.StepHeight = 45*math.Pi/180, 0.3 // the defaults: 50°, 0.4 m, 100 N, 70 kg, StickToFloor 0.5 m (Jolt)
world.AddCharacter(hero)
world.SetFilter(hero.Body(), layerCharacters, actor.AllLayers)

// each frame
world.Step(dt)
velocity := mgl64.Vec3{0, hero.Velocity.Y(), 0}             // in the air: the vertical velocity of the fall is kept
if hero.GroundState() == feather.CharacterOnGround {        // on the ground: the velocity of the ground (a platform)
	hero.UpdateGroundVelocity()
	velocity = hero.GroundVelocity()
	if jump {
		velocity = velocity.Add(mgl64.Vec3{0, 4, 0})
	}
}
hero.Velocity = velocity.Add(world.Gravity.Mul(dt)).Add(input) // + the gravity of the step, + the input (m/s)
hero.Update(dt)
model.SetPosition(hero.Position.Sub(mgl64.Vec3{0, 0.85, 0})) // the feet are 0.85 m under the center of the capsule
```
- **The velocity is the game's.** The recipe above is the one of the samples of Jolt (`CharacterVirtualTest::HandleInput`):
  the gravity of the step is added every frame, so that the character presses on the ground and falls when it leaves
  it; on the ground the vertical velocity is reset to the one of the ground, in the air it accumulates. PhysX, Box2D
  and Godot leave the gravity to the game the same way. After the update, `Velocity` is what the character took: the
  part towards a slope too steep removed.
- **Update after the step.** The velocity of a platform is the one of the step just done: updated after it, the
  character follows the platform without lag (`UpdateGroundVelocity` reads the velocity of the platform again before
  the recipe, for the frame the platform starts). Updated before the step, it is one step behind the platform. During
  a step an update panics, as a query.
- **It walks on everything**: a plane, a terrain, a mesh, the boxes, the hulls, the capsules, the dynamic and the
  kinematic bodies. It keeps 2 cm from them (`CharacterPadding`): a model drawn at the capsule stands 2 cm above the
  ground. The contacts are gathered 10 cm around the capsule: no body is entered, whatever the speed.
- **Slopes.** Under `MaxSlopeAngle` a surface is a ground: the horizontal velocity is projected on it (the character
  climbs at `speed × cos(angle)` along the slope, as in Jolt; Godot keeps the speed with `floor_constant_speed`).
  Steeper, it is a wall: the character doesn't climb it, and slides down it under the gravity of the recipe
  (`CharacterOnSteepGround`; the `ePREVENT_CLIMBING_AND_FORCE_SLIDING` of PhysX). Standing still (no horizontal
  velocity) on a ground, the character doesn't creep down (the stop on slope of the samples of Jolt and of Godot).
- **Steps.** A step lower than `StepHeight` is walked without jumping; higher, it is a wall. A step is walked up,
  forward and down onto its floor, which must not be too steep where the character lands (the edge of a step is tested
  15 cm further). A step of 0.6 m blocks a character of radius 0.3: the sphere of its capsule stops against the riser.
- **Downhill**, the character sticks to the floor: leaving the ground without going up, it looks for the floor within
  `StickToFloor` under it (0.5 m by default; 0 sticks to nothing) and lands on it (a slope, a step down). It walks down
  a 30° slope at 3 m/s without a frame in the air. Over a crest it was going up: it leaves the ground for a frame, and
  lands at the next one. Off a drop longer than `StickToFloor`, it falls.
- **Platforms.** Standing on a kinematic body, `GroundVelocity` is the velocity of the point under the character over
  the last step (its arc when the platform turns, as `CalculateCharacterGroundVelocity` of Jolt); the recipe adds it. A
  platform going up lifts the character by its contact. Drive the platform by `SetKinematicTarget`: its velocity is
  the one of its motion.
- **Pushing.** A dynamic body met gets an impulse which brings it to the speed of the character along the contact (damped
  by 0.9), at most `MaxStrength × dt` (100 N: a ball of 11 kg or a crate of 22 kg is pushed at the walking speed), and
  never downwards; a crate of 8 t doesn't move and stops the character. A dynamic body which hits a character is
  stopped by its inner kinematic body: it never goes through it. The weight of the character (`Mass × g`) presses a
  dynamic body it stands on.
- **Characters** block each other head on (they stop 2 paddings apart), slide along each other when they cross, and
  never go through each other. The inner body of a character is the body a ray, a sweep or an overlap hits:
  `world.CharacterOf(hit.Body)` gives the character.
- **Filtering.** The character collides with the bodies its inner body collides with: its layer and its mask
  (`world.SetFilter(hero.Body(), ...)`), the pairs ignored (`IgnoreCollision`). Give the characters a layer of their
  own to let the rays of the game skip them. The inner body enters and leaves the triggers as a kinematic body does
  (`EventTriggerEnter` and `EventTriggerExit` with the body; `CharacterOf` gives the character); a trigger never
  blocks the character: its contacts and its sweeps skip the triggers.
- **Teleport** places the character with its inner body and reads its contacts there; the next update pushes it out of
  a body it was placed in. `RemoveCharacter` removes it with its inner body.
- **A crouch**: write `Capsule.HalfHeight` between 2 updates; the inner body shares the capsule.
- **Cost.** On a bumpy terrain, 11 µs per update for a character walking alone (24 characters: 0.27 ms per frame), 54 µs
  in a crowd pushing against walls and crates (24: 1.3 ms; 200: 9.8 ms), measured on one core, then 13 % less (the
  supports and the cores of GJK made once, see ALGORITHMS.md). An update is GJK on the 6 to 10 triangles under the
  capsule and the 2 or 3 its sweeps hit.

### Collision filtering
Who collides with whom is decided by 3 things, tested in the broad phase before any contact is computed.

**Layers and masks.** Each body has a `Layer` (one bit among 32) and a `Mask` (the layers it collides with). 2 bodies
collide if each one has its layer in the mask of the other: one refusal is enough. A new body is on
`actor.LayerDefault` with the mask `actor.AllLayers`: everything collides, as before the layers existed.
```go
const (
	layerWorld actor.Layers = 1 << iota // the layers of the game: one bit each
	layerPawn
	layerDebris
)
ground.Layer = layerWorld
pawn.Layer, pawn.Mask = layerPawn, layerWorld|layerPawn // not the debris
debris.Layer, debris.Mask = layerDebris, layerWorld     // only the world

world.SetFilter(pawn, layerPawn, layerWorld) // during the game: the same, and the bodies resting on it wake up
```
- Give each body one layer. A layer of 0 collides with nothing and is seen by no query.
- A layer of several bits is allowed, as in Box2D, Jolt and Bullet: the body is on each of these layers. It collides
  with a body whose mask has at least one of them, and a query sees it if its mask has at least one of them.
- `Layer` and `Mask` can be written at any time, they are read at each step. Written directly, nobody wakes up: a
  sleeping body resting on a body which doesn't collide with it anymore keeps floating until something wakes it up.
  `World.SetFilter` writes them and wakes up the body and the sleeping bodies it collided with. Called with the layer
  and the mask the body already has, it does nothing: it can be called at each frame.
- `World.SetFilter` doesn't wake up the bodies the body collides with from now on. A static body which becomes solid
  around a sleeping body leaves it asleep, inside: it is pushed out when something else wakes it up. It is the same
  for a body added to the world with `AddBody`, and in Box2D.

**Ignored pairs.** `world.IgnoreCollision(a, b, true)`: these 2 bodies never collide, whatever their layers; their
other pairs are not changed. `false` restores the pair. For the pair a layer can't describe: the tail and the pelvis it
starts from (the tail still hits the legs), a projectile and the body which fires it.
- The cost is a hash per pair of overlapping AABBs with an awake body, only when the world has ignored pairs or joints.
- Both bodies wake up when the pair changes. The pair is forgotten when one of its bodies is removed from the world.

**Linked bodies.** The 2 bodies of a joint don't collide, unless the joint has `CollideConnected`: the neighboring
capsules of a ragdoll overlap at their joints and must not push each other. It is the same table as the ignored pairs:
a pair ignored and linked stays filtered when the joint is removed, and collides again when it is restored too.

A filtered pair has no contact: it is not solved, sends no collision or trigger event, wakes nobody up, and a fast body
is not stopped by a body it doesn't collide with (continuous collision). A trigger is filtered like any body, and the
test goes both ways: the mask of the trigger must have the layer of the bodies it detects, and the mask of these
bodies must have the layer of the trigger. With only one of them, no event is sent.

#### Queries only
A body of empty mask collides with nothing, but stays in the broad phase with its layer: the world queries (ray,
sweep, overlap) see it. It is the recipe for a decor seen by the rays which costs nothing in the narrow phase nor in
the solver. In the broad phase it costs what any static body costs, about 24 ns per step even far from everything
(its AABB is computed and compared to its leaf at each step), plus about 10 ns per dynamic body overlapping it:
```go
decor := actor.NewRigidBody(transform, shape, actor.BodyTypeStatic, 0)
decor.Layer, decor.Mask = layerDecor, actor.NoLayers
world.AddBody(decor)

// a query which sees the decor and the world, but not the character casting it
filter := feather.QueryFilter{Mask: layerDecor | layerWorld, Excluded: []*actor.RigidBody{character}}
filter.Accepts(decor) // true: the layer of the body is in the mask of the query, its own mask is not read
```
- Make it static: no pair, no contact, no event, nothing in the solver. A dynamic body of empty mask is still
  integrated: it falls through everything under gravity.
- It wakes nobody up, neither when it moves nor when it is removed.
- `QueryFilter{}` (empty mask) sees nothing: start from `feather.DefaultQueryFilter()`, which sees every layer.

### Queries
A query asks the world about its bodies without moving them: `Raycast` (the first body on a segment), `RaycastAll`
(all of them), `Sweep` (the first body a moving shape touches), `Overlap` (the bodies in a volume). It sees the static
bodies, the dynamic bodies awake or asleep, and the bodies "queries only", at their place of the end of the last step.
It wakes nobody up and sends no event.

```go
filter := feather.DefaultQueryFilter()              // every layer, no trigger
filter.Mask = layerWorld | layerDecor               // or a few layers: the layer of the body must be in the mask
filter.Excluded = []*actor.RigidBody{character}     // never the body which asks
filter.Triggers = true                              // the triggers too
```
`feather.QueryFilter{}` sees nothing (its mask is empty): always start from `DefaultQueryFilter()`. The mask of a body
is not read: a body which collides with nothing is seen on its layer. A body of layer 0 is seen by no query.

**The ground under a foot.** A ray from the knee to under the sole: `Fraction` tells how far the ground is, `Normal`
how to tilt the foot.
```go
hit, ok := world.Raycast(knee, mgl64.Vec3{0, -0.8, 0}, filter)
if ok {
	ground := hit.Point // knee + hit.Fraction * translation
	slope := hit.Normal // out of the ground; hit.Triangle is the triangle of a terrain (2*cell + t) or of a mesh, else -1
}
```
A ray has no thickness: on stairs or on rubble, a sphere of the size of the sole gives a steadier answer.
`hit.Fraction` is then where the sphere stops, `hit.Point` the point of the ground it touches.
```go
sole := &actor.Sphere{Radius: 0.08}
hit, ok := world.Sweep(sole, actor.Transform{Position: knee, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{0, -0.8, 0}, filter)
```

**A camera arm.** A sphere moved from the head to the wanted place of the camera: the camera goes where the sphere
stops. A ray would let the camera see through the corner of a wall.
```go
lens := &actor.Sphere{Radius: 0.2}
arm := wanted.Sub(head)
if hit, ok := world.Sweep(lens, actor.Transform{Position: head, Rotation: mgl64.QuatIdent()}, arm, filter); ok {
	camera = head.Add(arm.Mul(hit.Fraction))
}
```
- The sphere stops between 0 and 1 µm from what it hits, never in it: the sweep of the next frame starts free.
- Against a wall, a sweep which moves away from it or along it hits nothing: the camera, or a character, slides along
  the wall. Start the next sweep from the place the last one gave, or keep a skin (1 mm is plenty) between the shape
  and the walls: a shape put in exact contact by your own computation is, by the rounding, in the wall once in a few
  times, and then stopped at the fraction 0 whatever its direction.
- A body resting on the ground rests a little in it: a sweep started from its pose hits the ground at the fraction 0.
  Exclude the ground, or start from above.
- If the head is already in a wall, the sweep hits at the fraction 0, the normal against the arm: the camera stays on
  the head. To go through this wall, exclude its body and sweep again.
- A sweep moves a sphere, a capsule or a box, without turning it. A plane or a heightfield cannot be moved: it panics.

**What is around.** The bodies in a volume, in the order of `World.Bodies`; a body touching the volume is in.
```go
blast := &actor.Sphere{Radius: 3}
bodies = world.Overlap(blast, actor.Transform{Position: center, Rotation: mgl64.QuatIdent()}, filter, bodies[:0])
```

**Decor "queries only".** A wall of leaves the rays must see and the bodies must not hit: static, an empty mask, a
layer (see [above](#queries-only)). It costs nothing in the narrow phase nor in the solver.

**Buffers.** `RaycastAll` and `Overlap` append to the slice they are given: keep it, and pass `hits[:0]`. With a
capacity large enough, no query allocates.

**Start in a body.** A ray which starts in a sphere, a box, a capsule or under a plane hits this body at the fraction
0, at its origin, the normal against its direction; so does a shape which overlaps a body. A ray without length is a
point: the bodies which hold it are hit, without normal. A terrain is a surface: only its top side is hit, a ray from
below goes through, as through a hole.

**When to call `SyncQueries`.** The queries see the world of the end of the last `Step`. What the game writes after it
is seen after the next `Step`, or after `SyncQueries`:
```go
world.AddBody(crate)
door.Transform.Position = open
door.UpdateAABB()        // after a write to a transform or to a shape, as for a step
world.SyncQueries()      // the rays of this frame see the crate, and the door open
```
- Not needed after `Step`, nor after `RemoveBody` (a removed body is never hit).
- Without it, a body added is not seen, and a body moved is seen at its old place.
- Call it once after all the writes, not after each: it costs about 30 ns per body of the world.

**Threads.** Queries only read: any number of goroutines can run them at the same time, the pose of hundreds of
characters on 8 goroutines for example. Nothing may write the world meanwhile: no `Step`, `SyncQueries`, `AddBody`,
`RemoveBody`, `UpdateHeightfield`, no write to a body. Feather takes no lock; a query during `Step` panics
(`feather: query during Step`). A listener of an event can run queries: it runs after the step.

**Cost**, on one core of a Ryzen 7 5800X, among 1000 bodies scattered on 100 x 100 m (`cd bench && go run . -queries`):
about 0.4 µs for a ray of 2 m down, 0.9 µs for a level ray of 50 m, 0.7 µs for a sphere moved 2 m down, 0.7 µs for an
overlap of 1 m; on a terrain of 1025 x 1025 samples, 0.3 µs for a ray down, 2.7 µs for a ray of 1000 m across it, 4 µs
for a sphere moved 2 m down.

### Terrain
```go
// the grid of the terrain: heights[x*zSamples+z], shared without copy
field := actor.NewHeightfield(xSamples, zSamples, heights, mgl64.Vec3{0.5, 20, 0.5}) // 0.5 m between samples, heights * 20
terrain := actor.NewRigidBody(actor.Transform{Rotation: mgl64.QuatIdent()}, field, actor.BodyTypeStatic, 0)
world.AddBody(terrain)

// after a change of the heights (or of field.Holes) in [minX, maxX] x [minZ, maxZ]
world.UpdateHeightfield(terrain, minX, minZ, maxX, maxZ)
```
- A heightfield is static. The body is at the center of the grid, the heights along its Y axis.
- The terrain is a surface, without thickness: a body is held while its center is above it. Placed with its center
  under the surface, it falls under the terrain.
- The terrain is made of triangles: a finer grid gives finer contacts (a 2048x2048 grid follows the ground better than
  512x512), the bodies slide on the flat parts without hitting the edges between the triangles.
- `Holes[x*(zSamples-1)+z]`: a cell without triangles (a cave, a tunnel entrance).
- `World.UpdateHeightfield` wakes up the bodies above the changed region, and computes their contacts again.

### Decor meshes and convex hulls
```go
// the decor: the triangles of a model (indices 3 per triangle, counterclockwise seen from outside), built once
mesh, err := actor.NewTriangleMesh(vertices, indices)
rock := actor.NewRigidBody(actor.Transform{Position: place, Rotation: orientation}, mesh, actor.BodyTypeStatic, 0)
world.AddBody(rock)

// a dynamic object: the convex hull of the vertices of its model, 64 vertices at most
hull, err := actor.NewConvexHull(vertices, 64)
// the hull is centered on its center of mass: the body is there, not at the origin of the model
crate := actor.NewRigidBody(actor.Transform{Position: place.Add(hull.CenterOfMass()), Rotation: mgl64.QuatIdent()}, hull, actor.BodyTypeDynamic, 300)
```
- A mesh is static, and a surface: its triangles are touched from the side of their normal (the right-hand rule on the
  winding), a body whose center went through a wall is not pushed back. Give the triangles their outward winding, and
  close the solids the bodies can fall into. The vertices closer than 0.1 mm are welded: a soup of triangles is fine.
  A degenerate triangle (no area) is kept out, at its index.
- A mesh is built outside `Step`: 100 000 triangles take 0.3 s. Build it once per model and share it between bodies (a
  shape has no state); a mesh never changes, build another one for another decor.
- `hit.Triangle` of a ray or a sweep on a mesh is the index of the triangle: `mesh.Triangle(hit.Triangle)` gives its
  vertices, `mesh.Indices[3*hit.Triangle:]` its vertices in the model, for a material per triangle on your side.
- A hull is a convex volume: the mass and the inertia come from its volume and the density. Its local space is
  centered on its center of mass, `hull.CenterOfMass()` is the offset from the origin of the points. The limit of
  vertices (256 at most) bounds the cost of a contact (a hull of 64 vertices costs as a box, 256 twice as much); when it
  is reached, the hull is the one of the vertices the furthest out, the points left out are the closest to it. A
  flat cloud (a quad, a line) has no hull: use a Box.
- A hull slides, rolls and rests on a mesh or a terrain as the other shapes, with the same edges rules.

### Moving a body
A shape has no state: several bodies can share the same shape. Each body keeps its AABB: after moving a body by hand
(its `Transform`), call `UpdateAABB`.

### Timestep & substeps
```go
world := feather.World{
	Gravity:     mgl64.Vec3{0, -9.81, 0},
	Substeps:    8,
	SpatialGrid: feather.NewSpatialGrid(2.0, 4096),
	Workers:     4,
	Events:      feather.NewEvents(),
}
world.Step(1.0 / 60.0) // fixed timestep
```

- Use a fixed timestep (e.g. 1/60 s), with an accumulator if the frame rate varies.
- The contacts are computed once per step, the solver runs once per substep.
- Usually 4 substeps for simple scenes, 8 to 12 for stacks and heavy bodies.

### Contact stiffness
`World.ContactHertz` (30 Hz by default, as Box2D v3.1) is the stiffness of the contacts. The contacts with a static body
are twice as stiff.
- Higher values = less overlap under load (stacks), but it is capped at 1/8 of the substeps rate: `substeps / dt / 8`.
- Lower values = softer contacts.

With 8 substeps at 60 Hz, a stack of 10 boxes of 50 cm sinks by ~32 mm (Box2D v3.1: 30 mm): a contact sinks by
(load / mass) g / (2π hertz)² under its load. A stiffer world sinks less, but a heavy body landing on a light one bounces
more.

### Fast bodies
The contacts with a static or a kinematic body are created before the body touches it (speculative contacts), from the
distance it can travel during the step; so are the contacts of a fast body (a body which can move more than half of its
smallest extent during the step) with every body. Between 2 slower dynamic bodies, from 2 cm only: a body can enter
another one by what it moves in a step, the spring of the contact pushes it out.
A fast body is also moved back to its first impact with any body it had no contact with (continuous collision):
nothing marks a projectile, there is no bullet flag.
A ball at 40 m/s does not go through a 4 cm wall at 60 Hz (nor at 80 m/s); 2 balls of 10 cm thrown at 20 m/s each
bounce off each other; a plate of 1 cm at 30 m/s pushes the resting plate it meets.

### Sleep
The bodies touching each other form an island. An island resting for 0.5 s (all its bodies under 0.05 m/s and 0.05 rad/s)
falls asleep: it is not simulated anymore.
The whole island wakes up with `AddForce`, `AddTorque` or `WakeUp` on one of its bodies, when a moving body touches it,
or when a body under it is removed. A body asleep in a trigger stays in it ([Triggers](#triggers)).

### Triggers
A trigger (`IsTrigger`) is not solved: it detects the bodies which overlap it. `EventTriggerEnter` when a body starts to
overlap it, `EventTriggerStay` at each step while one of them is awake, `EventTriggerExit` when they no longer overlap, or
when one of them is removed (`RemoveBody`: the exit is sent with the events of the next step).
- A body asleep in a trigger stays in it: no exit, and no stay while both rest. Counting the enters and the exits gives
  the bodies in a zone.
- A body in contact with a trigger is in it (as `World.Overlap`).
- The game moves a static trigger, or a sleeping body: set its `Transform` and call `UpdateAABB` (or `Teleport`). The
  overlap is tested again when the AABB of one of the bodies changed.
- A kinematic body enters and leaves a static trigger, and a kinematic trigger detects the static and the kinematic
  bodies: a door zone sees the platform which carries the player. 2 static bodies never pair.
- Each awake body in a trigger costs a test per step; a body asleep in a trigger costs its place in the pairs of the
  step, without test while nothing moves.

### Collision events
`EventCollisionEnter` when 2 bodies start to touch, `EventCollisionStay` at each step while one of them is awake,
`EventCollisionExit` when they stop touching, or when one of them is removed (`RemoveBody`: the exit is sent with the
events of the next step). A body asleep on the ground, or on another body, still touches it: no exit, and no stay
while both rest. A body made a trigger ends its contacts and starts its trigger pairs.

### Determinism & threads
The same scene gives the same result, bit for bit, whatever the number of `Workers`.
From 256 bodies, the collision detection and the solver run on `Workers` goroutines: set it to the number of cores.
Call `World.Close()` when the world is not used anymore, to stop its workers.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Bodies slide on slopes | Set `StaticFriction` & `DynamicFriction` on both bodies (the ground too) |
| Stacks sink | Increase `ContactHertz` or `Substeps` |
| Stacks wobble | Increase `Substeps` |
| No bounce | Restitution on both bodies, impact faster than 1 m/s |
| A body does not move | It may be asleep: call `WakeUp`. It may be locked along this axis: `LinearLock`, `AngularLock` |
| A character falls over | Use a `CharacterVirtual` (a capsule which never turns); for a dynamic body, lock its rotations around X and Z: `SetLocks(actor.NoAxes, actor.AxisX\|actor.AxisZ)` |
| A character falls through the floor, or flies | Add the gravity of the step to its velocity every frame, and reset the vertical velocity on the ground (the recipe of [Characters](#characters)) |
| A character creeps down a slope | It has a horizontal velocity: give it exactly 0 to stand still. Steeper than `MaxSlopeAngle`, a slope is slid down |
| A character stops at a small step | `StepHeight` is 0, or lower than the step |
| A character lags behind its platform | Update it after `World.Step`, and call `UpdateGroundVelocity` before reading `GroundVelocity` |
| A character can't push a crate | `MaxStrength` (N) × dt is the impulse of a frame: 100 N moves 20 kg; a heavy crate stops the character by design |
| A kinematic body doesn't move | Give it a target before each step: `SetKinematicTarget`. A velocity written on it is ignored |
| A kinematic body goes through the bodies | It was teleported or placed by hand: move it by `SetKinematicTarget`, which pushes with the velocity of the motion |
| A platform falls asleep with its riders | It had no target: a kinematic body without target rests. Give it a target each step while it must move |
