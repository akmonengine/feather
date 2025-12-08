# Feather Physics Engine - Algorithm Deep Dives

This document provides detailed explanations of the core algorithms used in Feather: GJK, EPA, manifold generation, and the XPBD constraint solver.

## Table of Contents

1. [GJK Algorithm](#gjk-algorithm-gilbert-johnson-keerthi)
2. [EPA Algorithm](#epa-algorithm-expanding-polytope-algorithm)
3. [Manifold Generation](#manifold-generation-sutherland-hodgman-clipping)
4. [Contact Constraint Solver](#contact-constraint-solver-xpbd)
5. [References & Resources](#references--resources)

---

## GJK Algorithm (Gilbert-Johnson-Keerthi)

The GJK algorithm detects collisions between convex shapes by operating on their **Minkowski difference**.

### High-Level Explanation

**Core Idea**: Two convex shapes A and B overlap if and only if their Minkowski difference (A - B) contains the origin.

**Minkowski Difference**: A - B = {a - b | a ∈ A, b ∈ B}

**GJK Strategy**:
1. Build a simplex (point, line, triangle, or tetrahedron) within the Minkowski difference
2. Check if the simplex contains the origin
3. If yes → collision detected
4. If no → refine the simplex toward the origin
5. Repeat until either origin is contained or shapes are proven separated

### Why This Works

- We never compute the full Minkowski difference (expensive!)
- Only query **support points**: the farthest point in a given direction
- Support points are easy to compute for most shapes
- Simplex refinement converges quickly (typically 3-6 iterations)

### Algorithm Pseudocode

```
function GJK(shapeA, shapeB):
    // Initialize with arbitrary direction
    direction = (1, 0, 0)

    // Get first support point in Minkowski difference
    support = Support(shapeA, direction) - Support(shapeB, -direction)
    simplex = [support]

    // Search direction toward origin
    direction = -support

    while true:
        // Get new support point
        support = Support(shapeA, direction) - Support(shapeB, -direction)

        // If we didn't pass the origin, shapes are separated
        if dot(support, direction) < 0:
            return NO_COLLISION

        // Add point to simplex
        simplex.add(support)

        // Check if simplex contains origin
        if NearestSimplex(simplex, direction):
            return COLLISION

        // Otherwise, continue with refined simplex and new direction
```

### Simplex Evolution: Dimension Cases

GJK builds up a simplex incrementally:

#### Case 1: Point Simplex (1 vertex)
```
        origin
          O
         /
        /
       A
```
**Action**: Search toward origin from point A
**Next Direction**: -A

#### Case 2: Line Simplex (2 vertices)
```
       origin
          O
         /|
        / |
       B--A
```
**Action**: Check if origin is in Voronoi region of line AB
- If yes: Keep both points, search perpendicular to line toward origin
- If no: Keep only closest point (A or B), search toward origin

#### Case 3: Triangle Simplex (3 vertices)
```
         C
        /|\
       / | \
      /  O  \  (origin inside)
     /   |   \
    B----+----A
```
**Action**: Check which Voronoi region contains origin
- Inside triangle: Check if origin is above/below triangle plane
- Outside: Reduce to closest edge

#### Case 4: Tetrahedron Simplex (4 vertices)
```
         D
        /|\
       / | \
      / (O) \  (origin inside)
     /   |   \
    C----+----B
     \   |   /
      \  |  /
       \ | /
        \|/
         A
```
**Action**: Check if origin is inside tetrahedron
- Yes: **COLLISION DETECTED**
- No: Reduce to closest face (triangle)

### Visual Example: 2D Collision

```
Step 0: Initial search direction
   A ┌────┐
     │    │
     └────┘
              ┌────┐
              │    │ B
              └────┘
     direction →

Step 1: Get first support point
   A ┌────┐
     │    │•─→ S1 (rightmost of A - leftmost of B)
     └────┘
              ┌────┐
              │    │ B
              └────┘

Step 2: Direction toward origin
        S1
         •
        ←─ direction (toward origin at O)
     O

Step 3: Get second support point
         S1
         •
        /
       / S2
      •
     O (origin)

Step 4: Build line simplex
         S1
         •
        /|  ← origin in Voronoi region
       / | O
      •--+
     S2

Step 5: Search perpendicular to line
         S1
    ↑    •
    │   /
 direction /
    │   /
    │  • S2

Step 6: Get third support point
      S3
       •
         S1
         •
        /|
       / | O ← origin inside triangle
      /  |
     •---+
    S2

Result: COLLISION (origin contained in simplex S1-S2-S3)
```

### Implementation Details (gjk/gjk.go)

**Key Functions**:

1. **`GJK(bodyA, bodyB *actor.RigidBody) bool`**
   - Main entry point
   - Returns true if collision detected
   - Iterates until collision or separation proven

2. **`getSupport(bodyA, bodyB, direction)`**
   - Computes support point in Minkowski difference
   - `supportA - supportB` where points are in opposite directions

3. **`nearestSimplex(simplex, direction)`**
   - Core simplex refinement logic
   - Handles 1D (line), 2D (triangle), 3D (tetrahedron) cases
   - Updates search direction toward origin
   - Returns true if origin contained

4. **`handleLineCase()`, `handleTriangleCase()`, `handleTetrahedronCase()`**
   - Dimension-specific logic
   - Voronoi region tests
   - Simplex reduction

### Edge Cases & Optimizations

**Early Exit**: If `dot(support, direction) < 0`, the support point didn't cross the origin → shapes are separated

**Degenerate Simplices**: If simplex becomes too small or flat, algorithm may stall. Implementation handles this with epsilon comparisons.

**Optimization Opportunities** (marked in code):
- Reuse backing arrays for simplex vertices
- Cache previous support points
- Warm-start with previous frame's simplex

### Limitations

- **Convex shapes only**: GJK requires convexity
- **No penetration depth**: Only detects IF collision, not HOW MUCH
- **Numerical precision**: Very deep penetrations may fail

---

## EPA Algorithm (Expanding Polytope Algorithm)

EPA computes **penetration depth** and **contact normal** for overlapping convex shapes. It's always run after GJK detects a collision.

### High-Level Explanation

**Input**: Final simplex from GJK (tetrahedron containing origin in Minkowski difference)

**Output**:
- Penetration depth (how far shapes overlap)
- Contact normal (direction to separate them)

**Strategy**:
1. Start with GJK's final simplex as initial polytope
2. Find the face of the polytope closest to the origin
3. Expand the polytope in that direction
4. Repeat until convergence or max iterations

**Result**: The closest face to origin gives us:
- **Normal**: Face normal = separation direction
- **Depth**: Distance from origin to face = penetration depth

### Why This Works

The Minkowski difference A - B is a convex polytope. The closest point on this polytope to the origin tells us the **Minimum Translation Vector** (MTV) to separate the shapes.

EPA builds this polytope incrementally, always expanding toward the origin until it can't get any closer.

### Algorithm Pseudocode

```
function EPA(simplex, shapeA, shapeB):
    // Initialize polytope with GJK's final simplex
    polytope = Polytope(simplex)

    for iteration = 0 to MAX_ITERATIONS:
        // Find face closest to origin
        face = polytope.GetClosestFace()

        // Get support point in direction of face normal
        support = Support(shapeA, face.normal) - Support(shapeB, -face.normal)

        // Distance from origin to support point
        distance = dot(support, face.normal)

        // Convergence check
        if abs(distance - face.distance) < TOLERANCE:
            // Found closest face!
            return {
                normal: face.normal,
                depth: distance
            }

        // Expand polytope by adding support point
        // Remove faces that can "see" the new point
        // Add new faces connecting to the new point
        polytope.Expand(support)

    // Max iterations reached
    return ERROR
```

### Polytope Expansion: Visual Example

```
Step 0: Initial tetrahedron from GJK
         D
        /|\
       / | \
      /  O  \  (O = origin)
     /   |   \
    C----+----B
     \   |   /
      \  |  /
       \ | /
        \|/
         A

Step 1: Find closest face (say, ABC)
         D
        /|\
       / | \
      /  O  \
     / [ABC] \  ← closest face
    C----+----B
     \   |   /
      \  |  /
       \|/
        A

Step 2: Get support point in face normal direction
         D
        /|\
       / | \
      /  O  \
     /   |   \
    C----+----B
     \   |   / \
      \  |  /   \ S (new support point)
       \|/       \
        A

Step 3: Expand polytope
         D
        /|\
       / | \____
      /  O  \   \
     /   |   \   S  ← new vertex
    C----+----B /
     \   |   / /
      \  |  / /
       \|/ /
        A

Step 4: Remove old faces that can see S
Step 5: Add new faces connecting S to remaining edges
Step 6: Repeat...

Convergence: When new support point is on (or very close to) closest face
```

### Face Management (epa/face.go)

**Face Structure**:
```go
type Face struct {
    vertices [3]mgl64.Vec3  // Triangle vertices
    normal   mgl64.Vec3     // Outward normal
    distance float64        // Distance to origin
}
```

**Key Operations**:
1. **ComputeNormal**: Cross product of edges, pointing away from polytope center
2. **ComputeDistance**: `dot(vertex, normal)` gives signed distance
3. **CanSee**: Checks if point is "in front of" face (for removal during expansion)

### Degenerate Case Handling

#### Problem 1: Coplanar Faces
When support point lands exactly on closest face → distance doesn't change

**Solution**: Convergence tolerance check: `abs(newDistance - oldDistance) < EPSILON`

#### Problem 2: Polytope Collapse
Numerical errors can cause polytope to become degenerate (zero-volume)

**Solution**:
- Validate all faces have non-zero area
- Check that normals point outward
- Reject support points too close to existing vertices

#### Problem 3: No Progress
Polytope fails to expand (support points don't add new information)

**Solution**: Return error after MAX_ITERATIONS (typically 100)

### Implementation Details (epa/epa.go)

**Key Constants**:
```go
const (
    EPAMaxIterations = 100      // Prevent infinite loops
    EPATolerance     = 1e-6     // Convergence threshold
    DefaultCompliance = 1e-9    // Soft constraint stiffness
)
```

**Key Functions**:

1. **`EPA(simplex, bodyA, bodyB) (normal, depth, error)`**
   - Main EPA algorithm
   - Returns contact normal and penetration depth
   - Error if convergence fails

2. **`getClosestFace(faces []Face) Face`**
   - Linear search for face nearest to origin
   - Could optimize with priority queue

3. **`expandPolytope(polytope, support)`**
   - Core polytope expansion logic
   - Removes faces visible from support point
   - Adds new faces connecting support to horizon edges

4. **`buildHorizon(faces, support) []Edge`**
   - Finds edges between visible and non-visible faces
   - These edges form the "horizon" around the new point

### Parameter Tuning

#### MAX_ITERATIONS (currently 100)
- **Lower** (20-50): Faster but may fail on complex shapes
- **Higher** (100-200): More robust but slower
- **Typical convergence**: 5-15 iterations for simple shapes

#### TOLERANCE (currently 1e-6)
- **Lower** (1e-8): More precise penetration depth
- **Higher** (1e-4): Faster convergence but less accurate
- **Trade-off**: Precision vs speed

#### DEFAULT_COMPLIANCE (currently 1e-9)
- **Lower** (1e-10): Stiffer contacts, less penetration, more jitter
- **Higher** (1e-6): Softer contacts, more penetration, less jitter
- **See PHYSICS_GUIDE.md** for tuning guidelines

### Limitations

- **Convergence not guaranteed**: Degenerate cases may fail (returns error)
- **Computational cost**: O(n) where n = polytope faces (typically 20-50)
- **Numerical precision**: Very shallow or very deep penetrations can be problematic

---

## Manifold Generation (Sutherland-Hodgman Clipping)

After EPA gives us the collision normal and depth, we need to find **contact points** where the shapes touch. Multiple contact points create a stable "manifold."

### Why Multi-Point Contacts?

**Single Point Contact Problems**:
- Unstable (boxes would balance on corners)
- Incorrect torque (rotation around wrong axis)
- Jittery (contact point jumps between features)

**Multi-Point Manifold Benefits**:
- Stability (objects rest naturally)
- Realistic torque distribution
- Smooth contact transitions

### High-Level Explanation

**Goal**: Find 1-4 contact points distributed across the contact area

**Strategy**:
1. Identify reference face on one shape (most aligned with contact normal)
2. Identify incident face on other shape (most opposing the contact normal)
3. Clip incident face against reference face's side planes (Sutherland-Hodgman)
4. Keep points behind reference face (penetrating)
5. Reduce to best 4 points if more remain

### Sutherland-Hodgman Algorithm

Classic polygon clipping algorithm that clips one polygon against a plane.

**Pseudocode**:
```
function ClipPolygon(polygon, plane):
    output = []

    for each edge (A, B) in polygon:
        if A is behind plane:
            output.add(A)
            if B is in front of plane:
                // Edge crosses plane, add intersection point
                intersection = IntersectEdgePlane(A, B, plane)
                output.add(intersection)
        else:  // A is in front of plane
            if B is behind plane:
                // Edge crosses plane, add intersection point
                intersection = IntersectEdgePlane(A, B, plane)
                output.add(intersection)
                output.add(B)

    return output
```

**Applied to Contact Manifold**:
1. Start with incident face vertices (4 points for box)
2. Clip against reference face's 4 side planes
3. Each clip may reduce point count or add intersection points
4. Final points are those inside all planes = contact region

### Visual Example: Box-Box Contact

```
Top view of two boxes colliding:

Box A (reference face):
    ┌─────────┐
    │    A    │
    │         │
    └─────────┘

Box B (incident face):
         ┌─────────┐
         │    B    │
         │         │
         └─────────┘

Step 1: Identify faces
Reference face: Bottom of A (normal points down)
Incident face: Top of B (normal points up, most opposing)

Step 2: Clip incident face against reference face side planes

Clip against left plane:
         ┌─────────┐
         │ B  │    │  → Keep right portion
         │    │    │
         └─────────┘
              ↑
         left plane of A

Clip against right plane:
         ┌──│──┐
         │  B  │  → Keep left portion
         │     │
         └────│┘
              ↑
         right plane of A

(Repeat for top/bottom planes)

Step 3: Result - 4 contact points at corners of overlap region
         ┌─────────┐
         │ • ─ ─ • │
         │ │  B  │ │
         │ • ─ ─ • │
         └─────────┘
           (4 contacts)
```

### Special Cases

#### Sphere-Sphere Contact
No need for clipping - analytical solution:
```
contactPoint = centerA + (centerB - centerA).normalize() * radiusA
```
Single contact point at the midpoint between surface points.

#### Sphere-Box Contact
Find closest feature on box (face, edge, or corner), project sphere center:
```
closestPointOnBox = ClampToBox(sphereCenter)
contactPoint = closestPointOnBox
```

#### Plane-Box Contact
Project box corners onto plane:
```
for each corner in box:
    if distance(corner, plane) < threshold:
        contactPoints.add(corner)
```
Can generate 1-4 contact points depending on box orientation.

### Contact Point Reduction

If clipping generates >4 points (rare but possible):

**Strategy**: Keep 4 most well-distributed points
1. Find point pair with maximum distance → keep both
2. Find point farthest from line connecting first pair → keep
3. Find point farthest from triangle of first 3 → keep

**Why 4 points?** Balance between stability and performance:
- <4: May not be stable (torque errors)
- =4: Optimal for most scenarios
- >4: Diminishing returns, more solver cost

### Implementation Details (epa/manifold.go)

**Key Functions**:

1. **`GenerateManifold(bodyA, bodyB, normal, depth) []ContactPoint`**
   - Main entry point
   - Delegates to shape-specific logic
   - Returns 1-4 contact points

2. **`clipFaceAgainstPlane(face, plane) []Vec3`**
   - Sutherland-Hodgman core implementation
   - Clips polygon vertices against a plane
   - Returns clipped polygon

3. **`findReferenceAndIncidentFaces(bodyA, bodyB, normal)`**
   - Identifies which face to use as reference (most aligned with normal)
   - Identifies incident face (most opposing normal)

4. **`reduceContactPoints(points []Vec3) []ContactPoint`**
   - Reduces >4 points to best 4
   - Greedy algorithm for maximum distribution

### Manifold Quality Metrics

**Good Manifold**:
- 3-4 contact points for large flat contacts
- 1-2 points for edge/corner contacts
- Well-distributed (not clustered)
- Consistent between frames (no jitter)

**Poor Manifold**:
- All points clustered at one corner
- Point count varies wildly between frames
- Points far from actual contact region

---

## Contact Constraint Solver (XPBD)

The constraint solver resolves contacts to prevent penetration and apply restitution (bounciness). Feather uses **XPBD** (Extended Position-Based Dynamics).

### XPBD Overview

**Key Idea**: Solve constraints directly in position space, then derive velocities

**Two-Phase Solving**:
1. **Position Correction**: Move bodies apart to fix penetration
2. **Velocity Correction**: Apply restitution (bounce) and friction (future)

### Position Constraint

**Goal**: Eliminate penetration (depth = 0)

**Constraint**: `C = dot(pB - pA, normal) >= 0`
- Where `pA`, `pB` are contact points on each body
- `normal` points from A to B
- `C < 0` means penetration

**XPBD Position Correction Formula**:
```
Δλ = -(C + compliance * λ) / (wA + wB + compliance)
pA += -Δλ * normal * wA
pB +=  Δλ * normal * wB
λ  += Δλ
```

Where:
- `C`: Constraint violation (penetration depth, negative)
- `λ`: Lagrange multiplier (accumulated impulse)
- `compliance`: Soft constraint parameter (inverse stiffness)
- `wA, wB`: Inverse masses (0 for static bodies)

**Compliance Interpretation**:
- `compliance = 0`: Infinitely stiff (hard constraint)
- `compliance > 0`: Soft constraint (allows some penetration)
- Typical value: `1e-9` (very stiff but numerically stable)

### Velocity Constraint (Restitution)

**Goal**: Apply bounciness at contact

**Restitution Coefficient** `e`:
- `e = 0`: Perfectly inelastic (no bounce)
- `e = 1`: Perfectly elastic (full bounce)
- Typical: `0.3-0.8` for most materials

**Relative Velocity**:
```
vRel = dot(vB - vA, normal)
```

**Restitution Formula**:
```
if vRel < 0:  // Bodies approaching
    targetVel = -e * vRel  // Reverse with restitution
    Δv = (targetVel - vRel) / (wA + wB)
    vA += -Δv * normal * wA
    vB +=  Δv * normal * wB
```

### XPBD Substep Approach (NOT Iterations)

**CRITICAL**: XPBD uses **substeps** with **ONE solver pass per substep**, NOT multiple iterations.

**Why substeps instead of iterations?**
- Each substep uses a smaller timestep (h = dt / substeps)
- Better integration accuracy → fewer convergence issues
- Simpler: no iteration loops needed
- More stable for stiff constraints

**XPBD Pattern** (what Feather uses):
```
for substep = 0 to NUM_SUBSTEPS:
    h = dt / NUM_SUBSTEPS

    // Apply forces & integrate velocities

    // Solve constraints - SINGLE pass only!
    for each contact:
        SolvePositionConstraint(contact, h)

    for each contact:
        SolveVelocityConstraint(contact, h)

    // Integrate positions
```

**Typical Substep Counts**:
- Standard scenes: 1-2 substeps
- Fast objects / tall stacks: 4 substeps
- Extreme precision: 8+ substeps

**Contrast with traditional solvers**:
- **Traditional PBD/SI**: 1 step, 10-20 iterations
- **XPBD**: 2-4 substeps, 1 iteration each
- Same total solver passes, but better accuracy!

### Mathematical Derivation (Simplified)

**Starting Point**: Newton's law `F = ma`

**Constraint Force**: `F = λ * normal` (along contact normal)

**Position-Based Dynamics**: Instead of forces, directly compute position changes

**XPBD Extension**: Add compliance for soft constraints
- Standard PBD: `Δλ = -C / (wA + wB)`
- XPBD: `Δλ = -(C + α * λ) / (wA + wB + α)`
- Where `α = compliance / dt²`

**Benefit**: Compliance makes constraints "soft" without instability

### Implementation Details (constraint/contact.go)

**Contact Constraint Structure**:
```go
type ContactConstraint struct {
    bodyA, bodyB *actor.RigidBody
    normal       mgl64.Vec3
    depth        float64
    points       []mgl64.Vec3  // Contact manifold
    lambda       float64       // Accumulated impulse
}
```

**Key Functions**:

1. **`SolvePosition(dt float64, compliance float64)`**
   - Applies position correction to separate bodies
   - Uses XPBD formula with compliance
   - Updates body positions directly

2. **`SolveVelocity(dt float64, restitution float64)`**
   - Applies velocity correction for restitution
   - Only affects separating velocity (no stick)
   - Updates body velocities directly

3. **`Solve(dt float64) error`**
   - Main entry point (convenience wrapper)
   - Calls position then velocity solver
   - Uses default compliance and combined restitution

### Parameter Tuning

#### Compliance
- **Too low** (<1e-10): Stiff, jittery, potential instability
- **Too high** (>1e-6): Soft, excessive penetration, "mushy"
- **Recommended**: `1e-9` to `1e-8` for rigid bodies

#### Restitution
- **0.0**: Clay, putty (no bounce)
- **0.3-0.5**: Wood, concrete (typical solids)
- **0.7-0.9**: Rubber ball (bouncy)
- **0.95+**: Super ball (very bouncy)

**Combined Restitution**: When two materials collide
- Average: `(eA + eB) / 2`
- Maximum: `max(eA, eB)` ← **Feather uses this**
- Multiply: `eA * eB`

#### Substeps
- **Standard**: 1-2 (sufficient for most scenes)
- **Complex**: 4 (for tall stacks, fast objects)
- **Trade-off**: Accuracy vs performance (each substep = 1 full solver pass)

### Solver Stability

**Sources of Instability**:
1. Very small timesteps (dt < 1ms)
2. Very high mass ratios (heavy vs light)
3. Very stiff constraints (low compliance)
4. Too few substeps

**Stability Techniques in XPBD**:
1. **Compliance**: Softens constraints (numerical damping)
2. **Substeps**: Smaller timesteps improve integration
3. **Warm starting**: Reuse λ from previous frame (future)
4. **Mass clamping**: Limit effective mass ratios

---

## References & Resources

### Academic Papers

**GJK Algorithm**:
- Gilbert, Johnson, Keerthi: "A Fast Procedure for Computing the Distance Between Complex Objects in Three-Dimensional Space" (1988)
- Van den Bergen: "Efficient Collision Detection of Complex Deformable Models using AABB Trees" (1997)

**EPA Algorithm**:
- Van den Bergen: "Proximity Queries and Penetration Depth Computation on 3D Game Objects" (2001)

**XPBD Solver**:
- Macklin, Müller, Chentanez: "XPBD: Position-Based Simulation of Compliant Constrained Dynamics" (2016)
- Müller, Heidelberger, Hennix, Ratcliff: "Position Based Dynamics" (2007)

**Contact Manifolds**:
- Catto: "Contact Manifolds" (GDC 2007) - Box2D approach
- Gregorius: "Robust Contact Creation for Physics Simulations" (GDC 2015)

### Online Resources

**Tutorials**:
- Casey Muratori's Handmade Hero (GJK explanation): https://www.youtube.com/watch?v=Qupqu1xe7Io
- Winter Dev: GJK & EPA visualization: https://blog.winter.dev/2020/gjk-algorithm/
- Randy Gaul's Game Physics series: https://www.randygaul.net/

**Interactive Demos**:
- GJK Algorithm Visualizer: http://www.cs.ox.ac.uk/people/stephen.cameron/distances/
- EPA 2D Demo: https://observablehq.com/@esperanc/gjk-and-epa

**Reference Implementations**:
- **Box2D** (C++): Industry standard 2D physics - excellent collision code
- **Bullet Physics** (C++): 3D physics with GJK/EPA
- **ReactPhysics3D** (C++): Clean, educational implementation

### Books

- **"Real-Time Collision Detection"** by Christer Ericson
  - Chapter 5: Basic Primitive Tests
  - Chapter 9: Convex Objects (GJK)

- **"Game Physics Engine Development"** by Ian Millington
  - Chapters on collision detection and resolution

- **"Physics for Game Developers"** by David M. Bourg & Bryan Bywalec
  - Practical physics implementation

### Code Study Recommendations

For deeper understanding, study these files in order:

1. **gjk/gjk.go**: Start here - GJK is the foundation
2. **epa/epa.go**: EPA builds on GJK's output
3. **epa/face.go**: Understand polytope face management
4. **epa/manifold.go**: See how contact points are generated
5. **constraint/contact.go**: Finally, constraint solving

Each file has been documented with inline comments referencing this guide.

---

## Summary

### Algorithm Selection Rationale

| Algorithm | Why Chosen | Alternatives |
|-----------|-----------|-------------|
| **GJK** | Versatile for all convex shapes, fast convergence | SAT (less general) |
| **EPA** | Precise penetration depth from GJK simplex | MPR (less precise) |
| **Sutherland-Hodgman** | Robust polygon clipping, multi-point contacts | Discrete sampling (less accurate) |
| **XPBD** | Stable stacking, intuitive compliance parameter | Sequential Impulse (less stable), Penalty methods (hard to tune) |

### Computational Complexity Summary

| Algorithm | Typical Count | Worst Case | Per Frame Cost |
|-----------|--------------|------------|----------------|
| GJK | 3-6 iterations | 20 iterations | O(1) per pair |
| EPA | 5-15 iterations | 100 iterations | O(1) per collision |
| Manifold | 1 pass | 1 pass | O(1) per collision |
| Position Solver | 1 pass/substep | 1 pass/substep | O(contacts * substeps) |
| Velocity Solver | 1 pass/substep | 1 pass/substep | O(contacts * substeps) |

### Performance Tips

1. **Minimize contact count**: Use broad phase effectively
2. **Tune substeps**: Start with 1-2, increase only if needed
3. **Adjust compliance**: Higher = faster but softer
4. **Cache manifolds**: Reuse contact points between frames (future optimization)
5. **Warm start solver**: Reuse λ from previous frame (future optimization)

---

For architectural context and design decisions, see [ARCHITECTURE.md](ARCHITECTURE.md).
For practical parameter tuning and usage examples, see [PHYSICS_GUIDE.md](PHYSICS_GUIDE.md).
