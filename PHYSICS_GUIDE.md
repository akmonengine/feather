# Feather Physics Engine - Physics Parameter Guide

This guide helps you configure realistic physics simulations by choosing appropriate material properties, simulation parameters, and troubleshooting common issues.

## Table of Contents

1. [Material Parameters](#material-parameters)
2. [Simulation Parameters](#simulation-parameters)
3. [Common Scenarios & Troubleshooting](#common-scenarios--troubleshooting)
4. [Code Examples](#code-examples)

---

## Material Parameters

### Density (kg/m³)

Density determines an object's mass and inertia based on its volume. Higher density = heavier object = harder to move.

#### Realistic Density Values

| Material | Density (kg/m³) | Use Cases |
|----------|----------------|-----------|
| **Air** | 1.2 | Balloons, airships |
| **Wood (Balsa)** | 160 | Lightweight props |
| **Wood (Pine)** | 500-600 | Furniture, crates |
| **Wood (Oak)** | 700-900 | Heavy furniture, beams |
| **Ice** | 917 | Frozen objects |
| **Water** | 1000 | Reference value, liquids |
| **Concrete** | 2400 | Buildings, structures |
| **Glass** | 2500 | Windows, bottles |
| **Aluminum** | 2700 | Lightweight metal objects |
| **Stone (Granite)** | 2750 | Rocks, monuments |
| **Steel** | 7850 | Heavy machinery, I-beams |
| **Copper** | 8960 | Wires, pipes |
| **Lead** | 11340 | Very heavy objects |
| **Gold** | 19320 | Treasure, jewelry |

#### How Density Affects Simulation

```go
// Example: Two boxes with same size, different densities
lightBox := actor.NewBox(
    transform,
    mgl64.Vec3{1, 1, 1},  // 1m x 1m x 1m
    500,                   // Wood density
)
// mass = density * volume = 500 * (1*1*1) = 500 kg

heavyBox := actor.NewBox(
    transform,
    mgl64.Vec3{1, 1, 1},  // Same size
    7850,                  // Steel density
)
// mass = 7850 * (1*1*1) = 7850 kg

// The steel box is 15.7x heavier!
// - Requires 15.7x more force to accelerate
// - Falls at same speed (gravity affects all masses equally)
// - Has 15.7x more momentum when moving
```

#### Choosing Density

**For realistic simulation**: Use real-world values from table above

**For gameplay**:
- **Heavy objects** (800-10000 kg/m³): Obstacles, boss enemies, wrecking balls
- **Medium objects** (100-800 kg/m³): Player, props, crates
- **Light objects** (10-100 kg/m³): Debris, decorations, pickups

**Pro tip**: Mass ratio between interacting objects should ideally be <10:1 for stable simulation. If you need a massive object (planet), make it static instead.

---

### Restitution (Coefficient of Restitution)

Restitution controls "bounciness" - how much kinetic energy is retained after collision.

#### Restitution Scale (0.0 to 1.0)

| Value | Behavior | Real Materials | Game Use |
|-------|----------|----------------|----------|
| **0.0** | No bounce (perfectly inelastic) | Clay, putty, wet sand | Sticky surfaces, dampening |
| **0.1-0.2** | Minimal bounce | Lead, wet wood | Realistic ground |
| **0.3-0.4** | Slight bounce | Concrete, hard wood | Standard surfaces |
| **0.5-0.6** | Moderate bounce | Hard plastic, stone | Slightly bouncy |
| **0.7-0.8** | High bounce | Rubber, basketballs | Bouncy surfaces |
| **0.9** | Very high bounce | Super balls | Game power-ups |
| **0.95-1.0** | Nearly perfect bounce | Steel on steel | Pinball, special effects |
| **>1.0** | Gains energy (unphysical!) | N/A | Avoid (causes instability) |

#### Restitution Formula

When two objects collide, Feather combines their restitution values:

```go
// Current implementation: Maximum of the two
combinedRestitution = max(bodyA.Restitution, bodyB.Restitution)

// Alternative approaches (not currently used):
// Average: (bodyA.Restitution + bodyB.Restitution) / 2
// Multiply: bodyA.Restitution * bodyB.Restitution
```

**Why maximum?** A bouncy ball (e=0.9) should bounce on any surface, even clay (e=0.0).

#### How Restitution Affects Simulation

```go
// Example: Dropping a ball from 10m height

// Clay ball (e = 0.0)
ball1 := actor.NewSphere(transform, 0.5, 1000)
ball1.Restitution = 0.0
// Hits ground at ~14 m/s, bounces at 0 m/s → stops dead

// Basketball (e = 0.8)
ball2 := actor.NewSphere(transform, 0.5, 1000)
ball2.Restitution = 0.8
// Hits ground at ~14 m/s, bounces at 11.2 m/s → bounces to 6.4m

// Super ball (e = 0.95)
ball3 := actor.NewSphere(transform, 0.5, 1000)
ball3.Restitution = 0.95
// Hits ground at ~14 m/s, bounces at 13.3 m/s → bounces to 9m
```

#### Choosing Restitution

**For realism**: Use values from table (most materials: 0.2-0.6)

**For gameplay**:
- **Sticky platforms** (0.0): Player shouldn't bounce when landing
- **Standard ground** (0.1-0.3): Slight bounce, feels natural
- **Bouncy obstacles** (0.7-0.9): Fun gameplay mechanic
- **Hyper-bounce** (0.95+): Special power-up zones

**Pro tip**: Extremely high restitution (>0.95) can cause jitter. If objects won't settle, reduce restitution or increase compliance.

---

### Compliance (Soft Constraint Parameter)

Compliance controls constraint "softness" - how much constraints are allowed to violate before being enforced.

**Physics Meaning**: Compliance = 1 / Stiffness

#### Compliance Scale

| Value | Behavior | Visual Effect | Use Cases |
|-------|----------|---------------|-----------|
| **0** | Infinitely stiff (hard constraint) | No penetration, potential jitter | Ideal (but may be unstable) |
| **1e-10** | Extremely stiff | Tiny penetration, may jitter | Very rigid contacts |
| **1e-9** | Very stiff (default) | Barely visible penetration | Standard rigid bodies |
| **1e-8** | Stiff | Slight penetration, smoother | Stable rigid bodies |
| **1e-7** | Moderate | Noticeable soft contact | Slightly squishy objects |
| **1e-6** | Soft | Visible squishing | Soft bodies, cushions |
| **1e-5** | Very soft | Significant deformation | Jello, very soft materials |

#### How Compliance Affects Simulation

```
Low Compliance (1e-10):
┌────┐
│    │  ← Object barely penetrates surface
└────┘
═══════  Ground

High Compliance (1e-6):
┌────┐
│    │
└─ ┬ ┴┘  ← Object visibly sinks into surface
══╧═══  Ground
```

**Trade-off**:
- **Lower compliance** → Stiffer contacts → Less penetration → More jitter/instability
- **Higher compliance** → Softer contacts → More penetration → More stable/smooth

#### Compliance Tuning Process

1. **Start with default**: `1e-9` (very stiff)
2. **If jittery/vibrating**: Increase compliance by 10x (`1e-8`)
3. **If too much penetration**: Decrease compliance by 10x (`1e-10`)
4. **If still unstable**: Increase substeps or decrease timestep
5. **Iterate until satisfied**

#### Code Example

```go
// Current: Compliance is hardcoded in epa/epa.go
// Future: Will be a material property

// Temporary workaround: Modify epa/epa.go constant
const DefaultCompliance = 1e-9  // Adjust this value

// Ideal future API:
material := actor.Material{
    Density:     500,
    Restitution: 0.3,
    Compliance:  1e-8,  // Per-material compliance
}
box := actor.NewBoxWithMaterial(transform, halfExtents, material)
```

---

## Simulation Parameters

These parameters affect the global simulation quality and performance.

### Timestep (dt)

Timestep is how much simulated time passes per physics update.

#### Common Timestep Values

| Timestep | FPS Equivalent | Use Case |
|----------|----------------|----------|
| **1/30 (0.0333s)** | 30 FPS | Slow-paced games, low-end devices |
| **1/60 (0.0167s)** | 60 FPS | **Standard for most games** |
| **1/120 (0.0083s)** | 120 FPS | High-precision simulation |
| **1/240 (0.0042s)** | 240 FPS | Very fast objects, high accuracy |

#### Fixed vs Variable Timestep

**Fixed Timestep (Recommended)**:
```go
const physicsTimestep = 1.0 / 60.0  // 60 FPS

func GameLoop() {
    accumulator := 0.0
    for {
        frameTime := GetFrameTime()
        accumulator += frameTime

        // Update physics in fixed timesteps
        for accumulator >= physicsTimestep {
            world.Step(physicsTimestep)
            accumulator -= physicsTimestep
        }

        Render()
    }
}
```

**Benefits**:
- Deterministic (same input → same output)
- Stable (physics tuned for one timestep)
- Prevents physics explosions from frame rate drops

**Variable Timestep (Not Recommended)**:
```go
func GameLoop() {
    for {
        dt := GetFrameTime()  // Variable!
        world.Step(dt)        // Unstable
        Render()
    }
}
```

**Problems**:
- Non-deterministic
- Unstable (large dt can cause explosions)
- Difficult to tune

#### Choosing Timestep

**For most games**: `1/60` (60 FPS physics)

**Use smaller timestep if**:
- Fast-moving objects tunnel through walls
- High restitution causes instability
- Simulation feels "floaty" or imprecise

**Trade-off**: Smaller timestep = more accurate but more CPU cost

---

### Substeps

Substeps divide each physics step into smaller internal steps for better accuracy.

#### How Substeps Work

```go
func Step(dt float64, substeps int) {
    subDt := dt / float64(substeps)

    for i := 0; i < substeps; i++ {
        // Run full physics pipeline
        ApplyForces(subDt)
        DetectCollisions()
        SolveConstraints(subDt)
        IntegrateVelocities(subDt)
    }
}

// Example: dt=1/60, substeps=4
// Each substep processes 1/240 of a second
// Reduces tunneling and improves stability
```

#### Substep Guidelines

| Substeps | Use Case | CPU Cost |
|----------|----------|----------|
| **1** | Standard scenes, medium speeds | 1x (baseline) |
| **2** | Fast-moving objects, bouncy materials | 2x |
| **4** | Very fast projectiles, high stacks | 4x |
| **8+** | Extreme accuracy requirements | 8x+ |

**When to increase substeps**:
- Fast objects tunnel through thin walls
- Bouncy objects (high restitution) jitter or explode
- Tall stacks collapse unrealistically
- Constraints feel "soft" even with low compliance

#### Code Example

```go
// Currently: Substeps are internal to World.Step()
// Check world.go for substep implementation

world := NewWorld()
world.Substeps = 4  // If this property exists
world.Step(1.0 / 60.0)
```

---

### Substeps (NOT Solver Iterations)

**IMPORTANT**: XPBD uses **substeps** with **ONE solver iteration per substep**, not multiple solver iterations per step.

The key difference from traditional solvers:
- **Traditional solvers**: 1 step, many iterations
- **XPBD**: Many substeps, 1 iteration per substep

```go
// XPBD approach (what Feather uses)
func Step(dt float64, substeps int) {
    h := dt / float64(substeps)

    for i := 0; i < substeps; i++ {
        // Apply forces & integrate velocities
        // Detect collisions

        // Solve constraints - SINGLE iteration only!
        for each contact {
            SolvePositionConstraint(h)
        }
        for each contact {
            SolveVelocityConstraint(h)
        }

        // Integrate positions
    }
}
```

#### Substep Guidelines

| Substeps | Quality | Use Case | CPU Cost |
|----------|---------|----------|----------|
| **1** | Standard | Most scenes | 1x (baseline) |
| **2** | Better | Fast objects, bouncy | 2x |
| **4** | High | Very fast, tall stacks | 4x |
| **8+** | Very high | Extreme accuracy | 8x+ |

**When to increase substeps** (NOT iterations):
- Fast-moving objects tunnel through walls
- Tall stacks collapse unrealistically
- High restitution causes instability
- Need more precise integration

**Trade-off**: More substeps = more accurate but slower

**Why this works**: Each substep operates on a smaller timestep, improving integration accuracy and constraint stability without needing multiple iterations.

#### Code Example

```go
// Current implementation (world.go)
world := NewWorld()
world.Substeps = 2  // 2 substeps, 1 iteration each
world.Step(1.0 / 60.0)  // Total: 2 solver passes
```

---

## Common Scenarios & Troubleshooting

### Scenario 1: Stack of Boxes (Stability Test)

**Goal**: Build a stable tower of boxes

#### Recommended Setup

```go
ground := actor.NewPlane(
    actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
    mgl64.Vec3{0, 1, 0},  // Normal pointing up
    0,                     // Distance from origin
)
ground.Static = true
ground.Restitution = 0.1  // Minimal bounce

boxes := make([]*actor.RigidBody, 10)
for i := 0; i < 10; i++ {
    box := actor.NewBox(
        actor.Transform{Position: mgl64.Vec3{0, float64(i)*2 + 1, 0}},
        mgl64.Vec3{0.5, 1, 0.5},  // 1m x 2m x 1m boxes
        700,                       // Wood density
    )
    box.Restitution = 0.1    // Low bounce
    boxes[i] = box
    world.AddBody(box)
}

// Simulation settings
dt := 1.0 / 60.0
substeps := 2  // XPBD uses substeps, not solver iterations
```

#### Common Problems & Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Stack collapses immediately | Boxes spawned overlapping | Space boxes further apart initially |
| Stack wobbles and falls | Too few substeps | Increase substeps to 4 |
| Boxes vibrate/jitter | Compliance too low | Increase compliance to 1e-8 |
| Boxes sink into each other | Compliance too high | Decrease compliance to 1e-9 |
| Stack slowly tips over | Numerical drift | Increase substeps to 2-4 |

---

### Scenario 2: Bouncing Ball

**Goal**: Realistic bouncy ball that loses energy gradually

#### Recommended Setup

```go
ground := actor.NewPlane(
    actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
    mgl64.Vec3{0, 1, 0},
    0,
)
ground.Static = true
ground.Restitution = 0.8  // Bouncy surface

ball := actor.NewSphere(
    actor.Transform{Position: mgl64.Vec3{0, 10, 0}},  // 10m high
    0.5,    // 0.5m radius
    1100,   // Rubber density
)
ball.Restitution = 0.85  // High restitution

world.AddBody(ground)
world.AddBody(ball)

// Combined restitution: max(0.8, 0.85) = 0.85
// Ball will bounce to ~72% of previous height each bounce
```

#### Expected Behavior

```
Drop height: 10m
Bounce 1: ~7.2m   (0.85² ≈ 0.72)
Bounce 2: ~5.2m
Bounce 3: ~3.7m
Bounce 4: ~2.7m
...eventually settles
```

#### Common Problems & Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Ball doesn't bounce | Restitution too low | Increase to 0.7-0.9 |
| Ball bounces forever | Restitution too high (>1.0) | Reduce to ≤0.95 |
| Ball bounces higher each time | Restitution >1.0 or solver bug | Check restitution value |
| Ball vibrates on ground | High restitution + low compliance | Increase compliance or reduce restitution |
| Energy loss too fast | Combined restitution low | Increase restitution on both objects |

---

### Scenario 3: Resting Contacts (Jitter Prevention)

**Goal**: Objects at rest shouldn't vibrate or jitter

#### Recommended Setup

```go
ground := actor.NewPlane(
    actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
    mgl64.Vec3{0, 1, 0},
    0,
)
ground.Static = true
ground.Restitution = 0.0  // No bounce for resting

box := actor.NewBox(
    actor.Transform{Position: mgl64.Vec3{0, 1, 0}},
    mgl64.Vec3{1, 1, 1},
    500,  // Wood
)
box.Restitution = 0.0  // No bounce
// Let it fall and settle

// Tuning parameters
compliance := 1e-8      // Slightly soft for stability
velocityThreshold := 0.01  // Sleep threshold (future feature)
```

#### Common Problems & Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Object vibrates up/down | Compliance too low | Increase compliance to 1e-8 or 1e-7 |
| Object slowly sinks | Compliance too high | Decrease compliance to 1e-9 |
| Object drifts sideways | Friction not implemented | Wait for friction feature, or increase compliance |
| Vibration at high FPS | Timestep too small for compliance | Adjust compliance proportionally |

---

### Scenario 4: Fast-Moving Objects (Tunneling Prevention)

**Goal**: Prevent fast projectiles from passing through thin walls

#### The Tunneling Problem

```
Frame 1:       Frame 2:
   •
 (ball)
              |    |•
              | wall  (ball passed through!)
              |    |
```

When object moves >1 thickness per frame, it can "teleport" through walls.

#### Solutions

**Solution 1: Increase Substeps** (Recommended)
```go
world.Substeps = 4  // Check collision 4x per frame
// Effective speed limit = wallThickness * substeps / dt
```

**Solution 2: Decrease Timestep**
```go
dt = 1.0 / 120.0  // 120 FPS physics (2x more CPU)
```

**Solution 3: Thicken Walls**
```go
wall := actor.NewBox(
    transform,
    mgl64.Vec3{5, 10, 2},  // 4m thick instead of 0.5m
    2400,  // Concrete
)
```

**Solution 4: Continuous Collision Detection (Future Feature)**
```go
bullet.CCD = true  // Will detect collision along swept path
```

#### Speed Limit Calculation

```
Maximum safe speed = wallThickness / (dt / substeps)

Example:
- Wall thickness: 0.5m
- Timestep: 1/60 = 0.0167s
- Substeps: 4
- Safe speed: 0.5 / (0.0167/4) = 120 m/s

For faster speeds, increase substeps or decrease dt
```

---

## Code Examples

### Example 1: Simple Scene Setup

```go
package main

import (
    "github.com/akmonengine/feather"
    "github.com/akmonengine/feather/actor"
    "github.com/go-gl/mathgl/mgl64"
)

func main() {
    // Create world with gravity
    world := feather.NewWorld()
    world.Gravity = mgl64.Vec3{0, -9.81, 0}  // Earth gravity

    // Create ground plane
    ground := actor.NewPlane(
        actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
        mgl64.Vec3{0, 1, 0},  // Normal (up)
        0,
    )
    ground.Static = true
    ground.Restitution = 0.3
    world.AddBody(ground)

    // Create falling box
    box := actor.NewBox(
        actor.Transform{Position: mgl64.Vec3{0, 10, 0}},
        mgl64.Vec3{0.5, 0.5, 0.5},  // 1m cube
        700,                          // Wood density
    )
    box.Restitution = 0.4
    world.AddBody(box)

    // Simulation loop
    dt := 1.0 / 60.0
    for i := 0; i < 600; i++ {  // 10 seconds
        world.Step(dt)
        // Render or log positions
    }
}
```

### Example 2: Tower of Boxes

```go
func CreateTower(world *feather.World, height int) {
    for i := 0; i < height; i++ {
        box := actor.NewBox(
            actor.Transform{
                Position: mgl64.Vec3{0, float64(i)*2.0 + 1.0, 0},
            },
            mgl64.Vec3{0.5, 1.0, 0.5},  // 1m x 2m x 1m
            700,                          // Wood
        )
        box.Restitution = 0.1  // Low bounce for stability
        world.AddBody(box)
    }
}

func main() {
    world := feather.NewWorld()
    world.Gravity = mgl64.Vec3{0, -9.81, 0}

    ground := actor.NewPlane(
        actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
        mgl64.Vec3{0, 1, 0},
        0,
    )
    ground.Static = true
    ground.Restitution = 0.1
    world.AddBody(ground)

    CreateTower(world, 10)  // 10-box tower

    // Stable simulation settings
    dt := 1.0 / 60.0
    substeps := 2  // XPBD: 2 substeps = 2 solver passes total

    for i := 0; i < 1200; i++ {  // 20 seconds
        world.Step(dt)
    }
}
```

### Example 3: Bouncing Balls with Different Materials

```go
func main() {
    world := feather.NewWorld()
    world.Gravity = mgl64.Vec3{0, -9.81, 0}

    ground := actor.NewPlane(
        actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
        mgl64.Vec3{0, 1, 0},
        0,
    )
    ground.Static = true
    ground.Restitution = 0.8  // Bouncy floor
    world.AddBody(ground)

    // Different ball materials
    materials := []struct {
        name        string
        restitution float64
        density     float64
        x           float64
    }{
        {"Clay", 0.0, 1500, -3},
        {"Wood", 0.3, 700, -1},
        {"Rubber", 0.85, 1100, 1},
        {"Super Ball", 0.95, 1100, 3},
    }

    for _, mat := range materials {
        ball := actor.NewSphere(
            actor.Transform{Position: mgl64.Vec3{mat.x, 10, 0}},
            0.5,
            mat.density,
        )
        ball.Restitution = mat.restitution
        world.AddBody(ball)
    }

    // Run simulation and observe different bounce behaviors
    dt := 1.0 / 60.0
    for i := 0; i < 600; i++ {
        world.Step(dt)
    }
}
```

### Example 4: Newton's Cradle (Contact Chain)

```go
func CreateNewtonsCradle(world *feather.World) {
    // Five spheres in a row
    for i := 0; i < 5; i++ {
        sphere := actor.NewSphere(
            actor.Transform{
                Position: mgl64.Vec3{float64(i) * 1.1, 5, 0},
            },
            0.5,   // Radius
            7850,  // Steel density
        )
        sphere.Restitution = 0.95  // Nearly elastic
        world.AddBody(sphere)
    }

    // Pull first sphere back and release
    // (requires distance constraint - not yet implemented)
    // For now, just give it initial velocity:
    spheres := world.GetBodies()
    spheres[0].Velocity = mgl64.Vec3{5, 0, 0}  // Push rightward
}

// Expected behavior: Energy transfers through chain
// First ball stops, last ball swings out
```

---

## Quick Reference Tables

### Material Presets

```go
// Define common material presets
type MaterialPreset struct {
    Density     float64
    Restitution float64
}

var Materials = map[string]MaterialPreset{
    "Wood":     {700, 0.3},
    "Stone":    {2750, 0.2},
    "Steel":    {7850, 0.6},
    "Rubber":   {1100, 0.85},
    "Glass":    {2500, 0.4},
    "Concrete": {2400, 0.2},
    "Ice":      {917, 0.05},
}

// Usage:
mat := Materials["Wood"]
box := actor.NewBox(transform, halfExtents, mat.Density)
box.Restitution = mat.Restitution
```

### Troubleshooting Checklist

| Symptom | Check | Typical Fix |
|---------|-------|-------------|
| Objects jitter/vibrate | Compliance | Increase to 1e-8 |
| Objects sink into ground | Compliance | Decrease to 1e-9 |
| Objects bounce forever | Restitution | Reduce to <0.95 |
| No bounce at all | Restitution | Increase to >0.3 |
| Stack collapses | Substeps | Increase to 4 |
| Fast objects tunnel | Substeps | Increase to 4+ |
| Simulation too slow | Substeps | Reduce to 1 |
| Unrealistic movement | Density | Use real-world values |

---

## Performance Optimization

### Tips for Large Scenes

1. **Use static bodies for immovable objects** (ground, walls)
2. **Implement sleep system** (deactivate resting bodies) - future feature
3. **Use minimum substeps** (usually 1-2 is sufficient)
4. **Use spatial acceleration** for broad phase - future feature
5. **Optimize contact manifolds** (cache between frames) - future feature

### Performance Budget Example

```
Target: 60 FPS (16.67ms per frame)
Physics budget: 5ms

Rough estimates:
- 50 bodies: ~2ms (comfortable)
- 100 bodies: ~5ms (at limit)
- 200 bodies: ~12ms (need optimization)

If over budget:
1. Reduce substeps (2 → 1)
2. Increase timestep (1/60 → 1/30)
3. Implement spatial grid (future)
4. Implement sleep/islands (future)
```

---

For architectural decisions and design rationale, see [ARCHITECTURE.md](ARCHITECTURE.md).
For detailed algorithm explanations, see [ALGORITHMS.md](ALGORITHMS.md).
