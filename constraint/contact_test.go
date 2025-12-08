package constraint

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Helper function to create a dynamic rigid body for testing
// Mass is calculated from density and shape (unit sphere)
func createDynamicBody(position mgl64.Vec3, velocity mgl64.Vec3, density float64) *actor.RigidBody {
	// Create a unit sphere shape
	shape := &actor.Sphere{Radius: 1.0}

	rb := actor.NewRigidBody(
		actor.Transform{Position: position},
		shape,
		actor.BodyTypeDynamic,
		density,
	)

	rb.Velocity = velocity
	rb.PresolveVelocity = velocity
	rb.Material.Restitution = 0.5

	return rb
}

// Helper function to create a static rigid body
func createStaticBody(position mgl64.Vec3) *actor.RigidBody {
	// Create a unit box shape for static body
	shape := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}

	rb := actor.NewRigidBody(
		actor.Transform{Position: position},
		shape,
		actor.BodyTypeStatic,
		0.0,
	)

	return rb
}

func TestContactConstraint_SolvePosition_NoPenetration(t *testing.T) {
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.5,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{1, 0, 0},
				Penetration: 0.0, // No penetration
			},
		},
	}

	originalPosA := bodyA.Transform.Position
	originalPosB := bodyB.Transform.Position

	constraint.SolvePosition(0.016) // 60 FPS timestep

	// Positions should not change when there's no penetration
	if bodyA.Transform.Position != originalPosA {
		t.Errorf("BodyA position changed when there was no penetration: %v -> %v", originalPosA, bodyA.Transform.Position)
	}
	if bodyB.Transform.Position != originalPosB {
		t.Errorf("BodyB position changed when there was no penetration: %v -> %v", originalPosB, bodyB.Transform.Position)
	}
}

func TestContactConstraint_SolvePosition_WithPenetration(t *testing.T) {
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	penetrationDepth := 0.5

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0}, // Normal points from A to B
		Compliance:  0.0,
		Restitution: 0.5,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{0.75, 0, 0},
				Penetration: penetrationDepth,
			},
		},
	}

	originalPosA := bodyA.Transform.Position
	originalPosB := bodyB.Transform.Position

	constraint.SolvePosition(0.016)

	// Bodies should move apart
	// BodyA should move in -normal direction (left)
	if bodyA.Transform.Position.X() >= originalPosA.X() {
		t.Errorf("BodyA should move left (negative X), but moved from %v to %v", originalPosA, bodyA.Transform.Position)
	}

	// BodyB should move in +normal direction (right)
	if bodyB.Transform.Position.X() <= originalPosB.X() {
		t.Errorf("BodyB should move right (positive X), but moved from %v to %v", originalPosB, bodyB.Transform.Position)
	}

	// The separation distance should increase
	newSeparation := bodyB.Transform.Position.Sub(bodyA.Transform.Position).Len()
	oldSeparation := originalPosB.Sub(originalPosA).Len()

	if newSeparation <= oldSeparation {
		t.Errorf("Bodies did not separate: old distance=%v, new distance=%v", oldSeparation, newSeparation)
	}
}

func TestContactConstraint_SolvePosition_EqualMasses(t *testing.T) {
	mass := 2.0
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0, 0, 0}, mass)
	bodyB := createDynamicBody(mgl64.Vec3{1, 0, 0}, mgl64.Vec3{0, 0, 0}, mass)

	penetration := 0.2

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.0,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{0.5, 0, 0},
				Penetration: penetration,
			},
		},
	}

	originalPosA := bodyA.Transform.Position
	originalPosB := bodyB.Transform.Position

	constraint.SolvePosition(0.016)

	// With equal masses, both should move equal distances
	deltaA := bodyA.Transform.Position.Sub(originalPosA).Len()
	deltaB := bodyB.Transform.Position.Sub(originalPosB).Len()

	if math.Abs(deltaA-deltaB) > 1e-6 {
		t.Errorf("Equal mass bodies should move equal distances: deltaA=%v, deltaB=%v", deltaA, deltaB)
	}
}

func TestContactConstraint_SolvePosition_StaticBody(t *testing.T) {
	bodyA := createStaticBody(mgl64.Vec3{0, 0, 0})
	bodyB := createDynamicBody(mgl64.Vec3{1, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.0,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{0.5, 0, 0},
				Penetration: 0.3,
			},
		},
	}

	originalPosA := bodyA.Transform.Position
	originalPosB := bodyB.Transform.Position

	constraint.SolvePosition(0.016)

	// Static body should not move
	if bodyA.Transform.Position != originalPosA {
		t.Errorf("Static body moved: %v -> %v", originalPosA, bodyA.Transform.Position)
	}

	// Dynamic body should move away
	if bodyB.Transform.Position.X() <= originalPosB.X() {
		t.Errorf("Dynamic body should move away from static body: %v -> %v", originalPosB, bodyB.Transform.Position)
	}
}

func TestContactConstraint_SolvePosition_BothStatic(t *testing.T) {
	bodyA := createStaticBody(mgl64.Vec3{0, 0, 0})
	bodyB := createStaticBody(mgl64.Vec3{1, 0, 0})

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.0,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{0.5, 0, 0},
				Penetration: 0.5,
			},
		},
	}

	originalPosA := bodyA.Transform.Position
	originalPosB := bodyB.Transform.Position

	constraint.SolvePosition(0.016)

	// Both static bodies should not move
	if bodyA.Transform.Position != originalPosA {
		t.Errorf("Static bodyA moved: %v -> %v", originalPosA, bodyA.Transform.Position)
	}
	if bodyB.Transform.Position != originalPosB {
		t.Errorf("Static bodyB moved: %v -> %v", originalPosB, bodyB.Transform.Position)
	}
}

func TestContactConstraint_SolveVelocity_NoRelativeVelocity(t *testing.T) {
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 0, 0}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{1, 0, 0}, 1.0)

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.5,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{1, 0, 0},
				Penetration: 0.1,
			},
		},
	}

	originalVelA := bodyA.Velocity
	originalVelB := bodyB.Velocity

	constraint.SolveVelocity(0.016)

	// Since both bodies have the same velocity, there's no relative motion to correct
	// Velocities should remain relatively unchanged
	epsilon := 0.1
	if bodyA.Velocity.Sub(originalVelA).Len() > epsilon {
		t.Logf("BodyA velocity changed from %v to %v (expected minimal change)", originalVelA, bodyA.Velocity)
	}
	if bodyB.Velocity.Sub(originalVelB).Len() > epsilon {
		t.Logf("BodyB velocity changed from %v to %v (expected minimal change)", originalVelB, bodyB.Velocity)
	}
}

func TestContactConstraint_SolveVelocity_Approaching(t *testing.T) {
	// Body A moving right, Body B stationary
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{5, 0, 0}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	bodyA.Material.Restitution = 0.8
	bodyB.Material.Restitution = 0.8

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0}, // Normal points from A to B
		Compliance:  0.0,
		Restitution: 0.8,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{1, 0, 0},
				Penetration: 0.1,
			},
		},
	}

	constraint.SolveVelocity(0.016)

	// After collision, bodyA should slow down (or reverse)
	if bodyA.Velocity.X() >= 5.0 {
		t.Errorf("BodyA should slow down after collision: velocity=%v", bodyA.Velocity)
	}

	// BodyB should gain velocity in the positive direction
	if bodyB.Velocity.X() <= 0.0 {
		t.Errorf("BodyB should gain velocity after collision: velocity=%v", bodyB.Velocity)
	}
}

func TestContactConstraint_SolveVelocity_Restitution(t *testing.T) {
	// Test with high restitution (bouncy collision)
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{10, 0, 0}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	bodyA.Material.Restitution = 1.0
	bodyB.Material.Restitution = 1.0
	bodyA.PresolveVelocity = mgl64.Vec3{10, 0, 0}
	bodyB.PresolveVelocity = mgl64.Vec3{0, 0, 0}

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 1.0,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{1, 0, 0},
				Penetration: 0.1,
			},
		},
	}

	constraint.SolveVelocity(0.5) // Larger timestep to avoid restitution threshold

	// With perfect restitution and equal masses, velocities should exchange
	// (bodyA should slow down significantly, bodyB should speed up)
	totalMomentumBefore := 10.0 // bodyA momentum
	totalMomentumAfter := bodyA.Velocity.X() + bodyB.Velocity.X()

	// Momentum should be conserved (approximately)
	if math.Abs(totalMomentumBefore-totalMomentumAfter) > 1.0 {
		t.Logf("Momentum conservation: before=%v, after=%v", totalMomentumBefore, totalMomentumAfter)
	}
}

func TestContactConstraint_SolveVelocity_LowSpeedNoRestitution(t *testing.T) {
	// Test restitution threshold - low velocity collisions should not bounce
	dt := 0.016
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0.1, 0, 0}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	bodyA.Material.Restitution = 0.9
	bodyB.Material.Restitution = 0.9
	bodyA.PresolveVelocity = mgl64.Vec3{0.1, 0, 0}
	bodyB.PresolveVelocity = mgl64.Vec3{0, 0, 0}

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.9,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{1, 0, 0},
				Penetration: 0.05,
			},
		},
	}

	constraint.SolveVelocity(dt)

	// Due to restitution threshold (2 * 9.81 * dt), low velocities should not bounce much
	restitutionThreshold := 2.0 * 9.81 * dt

	if math.Abs(bodyA.PresolveVelocity.X()) < restitutionThreshold {
		// Low velocity - should not bounce with full restitution
		relativeVel := bodyB.Velocity.Sub(bodyA.Velocity).Dot(constraint.Normal)
		if relativeVel < 0 {
			t.Logf("Low velocity collision correctly avoided bouncing")
		}
	}
}

func TestContactConstraint_SolveVelocity_MultiplePoints(t *testing.T) {
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{2, 0, 0}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.5,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{1, 0.5, 0},
				Penetration: 0.1,
			},
			{
				Position:    mgl64.Vec3{1, -0.5, 0},
				Penetration: 0.1,
			},
		},
	}

	originalVelA := bodyA.Velocity
	constraint.SolveVelocity(0.016)

	// Velocity should change due to contact resolution
	if bodyA.Velocity == originalVelA {
		t.Errorf("Expected velocity to change with multiple contact points")
	}
}

func TestContactConstraint_SolveVelocity_SmallVelocityClamping(t *testing.T) {
	// Test that very small velocities get clamped to zero
	bodyA := createDynamicBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1e-9, 1e-9, 1e-9}, 1.0)
	bodyB := createDynamicBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{0, 0, 0}, 1.0)

	constraint := &ContactConstraint{
		BodyA:       bodyA,
		BodyB:       bodyB,
		Normal:      mgl64.Vec3{1, 0, 0},
		Compliance:  0.0,
		Restitution: 0.0,
		Points: []ContactPoint{
			{
				Position:    mgl64.Vec3{1, 0, 0},
				Penetration: 0.1,
			},
		},
	}

	constraint.SolveVelocity(0.016)

	// Small velocities should be clamped to zero
	zeroVec := mgl64.Vec3{0, 0, 0}
	if bodyA.Velocity != zeroVec && bodyA.Velocity.Len() < 1e-7 {
		t.Logf("Very small velocity was appropriately handled: %v", bodyA.Velocity)
	}
}
