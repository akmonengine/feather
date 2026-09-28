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
- `CollideConnected` (false by default): the 2 bodies of the joint don't collide with each other.
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
- The terrain is made of triangles: a finer grid gives finer contacts (a 2048x2048 grid follows the ground better than
  512x512), the bodies slide on the flat parts without hitting the edges between the triangles.
- `Holes[x*(zSamples-1)+z]`: a cell without triangles (a cave, a tunnel entrance).
- `World.UpdateHeightfield` wakes up the bodies above the changed region, and computes their contacts again.

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
The contacts with a static body are created before the body touches it (speculative contacts), from the distance it can
travel during the step. Between 2 dynamic bodies, from 2 cm only: a fast body can enter another one during a step, the
spring of the contact pushes it out.
A fast body is also moved back to its first impact with a static body (continuous collision). Set `IsBullet` on a small
fast body (a projectile) to stop it on the dynamic bodies too.
A ball at 40 m/s does not go through a 4 cm wall at 60 Hz (nor at 80 m/s).

### Sleep
The bodies touching each other form an island. An island resting for 0.5 s (all its bodies under 0.05 m/s and 0.05 rad/s)
falls asleep: it is not simulated anymore.
The whole island wakes up with `AddForce`, `AddTorque` or `WakeUp` on one of its bodies, when a moving body touches it,
or when a body under it is removed.

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
| A body does not move | It may be asleep: call `WakeUp` |
