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
`StaticFriction` is used while the contact sticks, `DynamicFriction` while it slides (above 1 cm/s).
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

### Damping
`LinearDamping` and `AngularDamping` (1/s) slow the body down: `v = v / (1 + h * damping)` at each substep.

## Simulation

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
`World.ContactHertz` (60 Hz by default) is the stiffness of the contacts. The contacts with a static body are twice as stiff.
- Higher values = less overlap under load (stacks), but it is capped at 1/8 of the substeps rate: `substeps / dt / 8`.
- Lower values = softer contacts.

With 12 substeps at 50 Hz, a stack of 10 boxes of 50 cm sinks by ~5 mm.

### Fast bodies
The contacts are created before the bodies touch (speculative contacts), from the distance the bodies can travel during the step.
A ball at 40 m/s does not go through a 4 cm wall at 50 Hz.

### Sleep
A body resting for 0.5 s (under 0.05 m/s and 0.05 rad/s) falls asleep: it is not simulated anymore.
It wakes up with `AddForce`, `AddTorque`, `WakeUp`, or when a moving body touches it.

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
