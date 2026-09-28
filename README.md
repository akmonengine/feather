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
| `Sphere` | `Radius` | analytic against planes, spheres and capsules; GJK/EPA otherwise |
| `Box` | `HalfExtents` | analytic against planes; GJK/EPA otherwise |
| `Plane` | `Normal`, `Distance` (static only) | analytic |
| `Capsule` | `HalfHeight`, `Radius`, axis along local Y | analytic against planes, spheres and capsules; GJK/EPA otherwise |

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
- The soft constraint is a spring + damper, set with a frequency (`World.ContactHertz`, 60 Hz by default) and a damping ratio.
- Contacts exist before the bodies touch (speculative contacts), so fast bodies don't go through thin walls.
- Friction follows Coulomb's law: static friction when the contact sticks, dynamic friction when it slides.
- The simulation is deterministic: same result bit for bit, whatever the number of `Workers`.

### Why not XPBD anymore
Up to v0.2.0, Feather used a simplified XPBD solver. The same scenes (`bench/`, 50 Hz, 12 substeps):

| Scene | v0.2.0 (XPBD) | TGS Soft | Expected |
|---|---|---|---|
| Pyramid of 55 boxes, 3 s | explodes (top box at 134 m) | stands (4.748 m) | 4.750 m |
| Box on a 20° slope, µ = 0.6 | slides 9.9 m | 0 m | 0 m |
| Box on a 35° slope, µ = 0.3 | slides 16.9 m | 9.654 m | 9.648 m |
| Bounce from 1 m, restitution 0.5 | 0.06 m | 0.24 m | 0.25 m |
| 10 N during 1 s on 32.7 kg | 15279 m/s | 0.306 m/s | 0.306 m/s |
| Same scene, run twice | 39/40 bodies differ | identical | identical |
| EPA sphere-box normal (p99) | 2.7° | 0.03° | 0° |
| Step, 10 / 100 / 500 bodies | 0.41 / 1.94 / 8.8 ms | 0.06 / 0.52 / 2.6 ms | |

```
cd bench
go run .                                            # current version
go run -tags v020 -modfile=go.v020.mod .            # v0.2.0
```

### Constraints
- Contact: generated when a collision is detected between two rigid bodies, up to 4 points (manifold), with friction and restitution.

A not exhaustive list of possible constraints (not implemented yet):
- Distance: Maintains constant distance between two points. Usage: Ropes, chains, rigid connections, ragdoll bones
- Distance Range: Keeps distance within [min, max] range. Usage: Elastic ropes, springs with limits, telescopic joints
- Hinge: Allows rotation around one axis only (like a door). Usage: Doors, wheels, joints, rotating platforms
- Angular Range: limits rotation within [min/max]. Usage: articulation

## GJK
Detects if two convex shapes overlap. With a margin, it also detects the shapes closer than the margin (speculative contacts).

## EPA
Computes the penetration depth, the normal and the witness points. The contact points are then clipped between the faces
of both shapes (Sutherland-Hodgman), each point with its own separation.

See [ALGORITHMS.md](ALGORITHMS.md), [ARCHITECTURE.md](ARCHITECTURE.md) and the [physics guide](PHYSICS_GUIDE.md).

## Sources
- https://box2d.org/posts/2024/02/solver2d/
- https://github.com/erincatto/box2d (v3, contact_solver.c & solver.c)
- https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
- https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf (gyroscopic torque)
- https://github.com/bepu/bepuphysics2
- https://cse442-17f.github.io/Gilbert-Johnson-Keerthi-Distance-Algorithm/
- https://winter.dev/articles/epa-algorithm
- Christer Ericson, Real-Time Collision Detection (2004)

## Contributing Guidelines

See [how to contribute](CONTRIBUTING.md).

## Licence
This project is distributed under the [Apache 2.0 licence](LICENCE.md).
