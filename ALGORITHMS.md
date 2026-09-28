# Feather - Algorithms

1. [GJK Algorithm](#gjk-algorithm)
2. [EPA Algorithm](#epa-algorithm)
3. [Contact points](#contact-points)
4. [Solver](#solver)
5. [Joints](#joints)

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
- The witness points come from the barycentric coordinates of the origin projected on the closest face.
- Tested against exact solutions: SAT for box-box, closest point for sphere-box (see `epa/epa_test.go`).

## Contact points
From the normal of EPA, each body gives the feature facing the other body (a face for a box, a line or a point for a capsule):

- **Face contact**: a face is aligned with the normal (0.5°). The other feature is clipped by the side planes of this face (Sutherland-Hodgman).
- **Parallel edges**: a box on an edge, a capsule along an edge. One edge is clipped by the other: 2 points.
- **Otherwise** (crossing edges, a vertex, a sphere): the witness point of EPA.

The deepest point has the separation of EPA, the other points are higher along the normal.
The points closer than the margin are kept, 4 at most: the deepest, the farthest from it, then the points adding the most area.

Spheres and capsules don't use EPA: their contact comes from the closest points of their segments (Ericson 5.1.9).
Parallel capsules get 2 points.

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
    Relax();                 // rigid constraint + friction
end
Restitution();
````

The contact points are not computed again during the substeps: the separation is updated from the motion of both anchors.
````
separation = baseSeparation + (ΔpB + ΔqB*rB - ΔpA - ΔqA*rA) · normal
````

### Soft constraint
The contact is a spring + damper, with a frequency `ω = 2π * hertz` and a damping ratio `ζ`:
````
a1 = 2ζ + hω
a2 = hω * a1
a3 = 1 / (1 + a2)
biasRate = ω / a1,  massScale = a2 * a3,  impulseScale = a3

Push:
    if separation > 0 then bias = separation / h          // speculative: can get closer, not further than the gap
    else bias = max(massScale * biasRate * separation, -ContactSpeed)
    λ = -normalMass * (massScale * vn + bias) - impulseScale * λ_total
    λ_total = max(λ_total + λ, 0)
````
`Relax` solves the same constraint without the spring (bias only for the speculative contacts), which removes the energy added by the spring.

### Friction
Solved in `Relax`, along 2 tangents, with Coulomb's law: the tangent impulse stays in a disc of radius `µ * λ_normal`.
µ is the static friction when the contact point slides slower than 1 cm/s, the dynamic friction otherwise.

### Restitution
Applied after the substeps, for the contacts hitting faster than 1 m/s:
`λ = -normalMass * (vn + e * vn_before)`, limited so that the bounce never adds energy.

### Gyroscopic torque
`ω × Iω` is integrated implicitly (1 Newton-Raphson iteration in body space), as described by Erin Catto
([GDC 2015](https://box2d.org/files/ErinCatto_NumericalMethods_GDC2015.pdf)). Dropping it removes the tumbling
of long bodies, integrating it explicitly makes them gain energy.

### Parallel solver
The solver is a Gauss-Seidel: each contact uses the velocities left by the previous one. To solve in parallel,
the contacts are colored (Box2D v3, `constraint_graph.c`): each contact takes the first color where both of its dynamic
bodies are free (the static bodies don't count). The contacts of a color don't share any body, the workers solve them
at the same time. The contacts without a free color (16 colors) are solved first, on a single goroutine.

### Default values
| Constant | Value |
|----------|-------|
| `DefaultContactHertz` | 60 Hz (x2 against static bodies, capped to 1/8 of the substeps rate) |
| `ContactDampingRatio` | 10 |
| `ContactSpeed` | 3 m/s |
| `RestitutionThreshold` | 1 m/s |
| `SpeculativeDistance` | 2 cm |
| `LinearSlop` | 5 mm |

## Joints
The joints are solved like the contacts (as in Box2D v3): warm starting, soft constraints in `Push` (60 Hz, damping ratio 2
by default), rigid constraints in `Relax`. They are solved before the contacts, on a single goroutine.

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
