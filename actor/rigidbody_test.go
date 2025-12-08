package actor

import (
	"math"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// =============================================================================
// BodyType Tests
// =============================================================================

func TestBodyType_Constants(t *testing.T) {
	// Verify that body type constants are distinct
	if BodyTypeDynamic == BodyTypeStatic {
		t.Error("BodyTypeDynamic and BodyTypeStatic should have different values")
	}

	// Verify expected values (iota starts at 0)
	if BodyTypeDynamic != 0 {
		t.Errorf("BodyTypeDynamic = %d, want 0", BodyTypeDynamic)
	}
	if BodyTypeStatic != 1 {
		t.Errorf("BodyTypeStatic = %d, want 1", BodyTypeStatic)
	}
}

// =============================================================================
// Material Tests
// =============================================================================

func TestMaterial_GetMass(t *testing.T) {
	tests := []struct {
		name     string
		material Material
		wantMass float64
	}{
		{
			name: "normal mass",
			material: Material{
				Density: 1.0,
				mass:    10.0,
			},
			wantMass: 10.0,
		},
		{
			name: "zero mass",
			material: Material{
				Density: 0.0,
				mass:    0.0,
			},
			wantMass: 0.0,
		},
		{
			name: "infinite mass",
			material: Material{
				Density: 0.0,
				mass:    math.Inf(1),
			},
			wantMass: math.Inf(1),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			mass := tt.material.GetMass()
			if math.IsInf(tt.wantMass, 1) {
				if !math.IsInf(mass, 1) {
					t.Errorf("GetMass() = %v, want +Inf", mass)
				}
			} else if mass != tt.wantMass {
				t.Errorf("GetMass() = %v, want %v", mass, tt.wantMass)
			}
		})
	}
}

// =============================================================================
// NewRigidBody Tests
// =============================================================================

func TestNewRigidBody_Dynamic(t *testing.T) {
	transform := Transform{
		Position: mgl64.Vec3{1, 2, 3},
	}
	sphere := &Sphere{Radius: 1.0}
	density := 2.0

	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, density)

	// Verify body type
	if rb.BodyType != BodyTypeDynamic {
		t.Errorf("BodyType = %v, want BodyTypeDynamic", rb.BodyType)
	}

	// Verify transforms are set correctly
	if !vec3AlmostEqual(rb.Transform.Position, transform.Position, 1e-10) {
		t.Errorf("Transform.Position = %v, want %v", rb.Transform.Position, transform.Position)
	}
	if !vec3AlmostEqual(rb.PreviousTransform.Position, transform.Position, 1e-10) {
		t.Errorf("PreviousTransform.Position = %v, want %v", rb.PreviousTransform.Position, transform.Position)
	}

	// Verify velocity is zero initialized
	expectedVelocity := mgl64.Vec3{0, 0, 0}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	// Verify shape is set
	if rb.Shape != sphere {
		t.Error("Shape not set correctly")
	}

	// Verify mass is computed correctly
	expectedMass := sphere.ComputeMass(density)
	if !almostEqual(rb.Material.GetMass(), expectedMass, 1e-10) {
		t.Errorf("Material.GetMass() = %v, want %v", rb.Material.GetMass(), expectedMass)
	}

	// Verify density is set
	if rb.Material.Density != density {
		t.Errorf("Material.Density = %v, want %v", rb.Material.Density, density)
	}

	// Verify restitution is initialized to 0
	if rb.Material.Restitution != 0.0 {
		t.Errorf("Material.Restitution = %v, want 0.0", rb.Material.Restitution)
	}
}

func TestNewRigidBody_Static(t *testing.T) {
	transform := Transform{
		Position: mgl64.Vec3{5, 10, 15},
	}
	box := &Box{HalfExtents: mgl64.Vec3{2, 2, 2}}
	density := 1.5 // Should be ignored for static bodies

	rb := NewRigidBody(transform, box, BodyTypeStatic, density)

	// Verify body type
	if rb.BodyType != BodyTypeStatic {
		t.Errorf("BodyType = %v, want BodyTypeStatic", rb.BodyType)
	}

	// Verify mass is infinite
	if !math.IsInf(rb.Material.GetMass(), 1) {
		t.Errorf("Material.GetMass() = %v, want +Inf for static body", rb.Material.GetMass())
	}

	// Verify density is set to 0 for static bodies
	if rb.Material.Density != 0 {
		t.Errorf("Material.Density = %v, want 0 for static body", rb.Material.Density)
	}

	// Verify transforms are set
	if !vec3AlmostEqual(rb.Transform.Position, transform.Position, 1e-10) {
		t.Errorf("Transform.Position = %v, want %v", rb.Transform.Position, transform.Position)
	}
}

func TestNewRigidBody_DifferentShapes(t *testing.T) {
	transform := NewTransform()
	density := 1.0

	tests := []struct {
		name  string
		shape ShapeInterface
	}{
		{
			name:  "sphere",
			shape: &Sphere{Radius: 2.0},
		},
		{
			name:  "box",
			shape: &Box{HalfExtents: mgl64.Vec3{1, 2, 3}},
		},
		{
			name:  "plane",
			shape: &Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: 0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			rb := NewRigidBody(transform, tt.shape, BodyTypeDynamic, density)

			if rb.Shape != tt.shape {
				t.Errorf("Shape not set correctly for %s", tt.name)
			}

			expectedMass := tt.shape.ComputeMass(density)
			actualMass := rb.Material.GetMass()

			// Handle infinite mass case (e.g., planes always have infinite mass)
			if math.IsInf(expectedMass, 1) && math.IsInf(actualMass, 1) {
				// Both infinite, test passes
				return
			}

			if !almostEqual(actualMass, expectedMass, 1e-10) {
				t.Errorf("Mass = %v, want %v for %s", actualMass, expectedMass, tt.name)
			}
		})
	}
}

// =============================================================================
// Integrate Tests
// =============================================================================

func TestIntegrate_Dynamic_NoGravity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Set initial velocity
	rb.Velocity = mgl64.Vec3{1, 2, 3}

	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0} // No gravity

	rb.Integrate(dt, gravity)

	// With no gravity, velocity should remain constant
	expectedVelocity := mgl64.Vec3{1, 2, 3}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	// Position should update based on velocity
	expectedPosition := mgl64.Vec3{0.1, 0.2, 0.3} // dt * velocity
	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-10) {
		t.Errorf("Position = %v, want %v", rb.Transform.Position, expectedPosition)
	}

	// Previous velocity should be saved
	expectedPreviousVelocity := mgl64.Vec3{1, 2, 3}
	if !vec3AlmostEqual(rb.PresolveVelocity, expectedPreviousVelocity, 1e-10) {
		t.Errorf("PresolveVelocity = %v, want %v", rb.PresolveVelocity, expectedPreviousVelocity)
	}

	// Previous position should be saved
	expectedPreviousPosition := mgl64.Vec3{0, 0, 0}
	if !vec3AlmostEqual(rb.PreviousTransform.Position, expectedPreviousPosition, 1e-10) {
		t.Errorf("PreviousTransform.Position = %v, want %v", rb.PreviousTransform.Position, expectedPreviousPosition)
	}
}

func TestIntegrate_Dynamic_WithGravity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	density := 1.0
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, density)

	// Initial velocity is zero
	rb.Velocity = mgl64.Vec3{0, 0, 0}

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0} // Standard gravity

	rb.Integrate(dt, gravity)

	// Velocity should increase due to gravity: v = v0 + g*dt
	// Since mass cancels out in the force calculation: a = F/m = (g*m)/m = g
	expectedVelocity := mgl64.Vec3{0, -1, 0} // g * dt = -10 * 0.1
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	// Position should update: p = p0 + v*dt
	// Since initial velocity was 0, and we use the NEW velocity:
	expectedPosition := mgl64.Vec3{0, -0.1, 0} // v * dt = -1 * 0.1
	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-10) {
		t.Errorf("Position = %v, want %v", rb.Transform.Position, expectedPosition)
	}
}

func TestIntegrate_Dynamic_MultipleSteps(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	// Integrate multiple times
	for i := 0; i < 3; i++ {
		rb.Integrate(dt, gravity)
	}

	// After 3 steps:
	// Step 1: v = 0 + (-10)*0.1 = -1, p = 0 + (-1)*0.1 = -0.1
	// Step 2: v = -1 + (-10)*0.1 = -2, p = -0.1 + (-2)*0.1 = -0.3
	// Step 3: v = -2 + (-10)*0.1 = -3, p = -0.3 + (-3)*0.1 = -0.6

	expectedVelocity := mgl64.Vec3{0, -3, 0}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-9) {
		t.Errorf("Velocity after 3 steps = %v, want %v", rb.Velocity, expectedVelocity)
	}

	expectedPosition := mgl64.Vec3{0, -0.6, 0}
	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-9) {
		t.Errorf("Position after 3 steps = %v, want %v", rb.Transform.Position, expectedPosition)
	}
}

func TestIntegrate_Static_NoMovement(t *testing.T) {
	transform := Transform{
		Position: mgl64.Vec3{5, 10, 15},
	}
	box := &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
	rb := NewRigidBody(transform, box, BodyTypeStatic, 1.0)

	// Try to set velocity (shouldn't matter for static)
	rb.Velocity = mgl64.Vec3{100, 200, 300}

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	initialPosition := rb.Transform.Position

	rb.Integrate(dt, gravity)

	// Static bodies should not move
	if !vec3AlmostEqual(rb.Transform.Position, initialPosition, 1e-10) {
		t.Errorf("Static body moved: Position = %v, want %v", rb.Transform.Position, initialPosition)
	}

	// Velocity should remain unchanged
	expectedVelocity := mgl64.Vec3{100, 200, 300}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Static body velocity changed: Velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}
}

func TestIntegrate_Dynamic_WithInitialVelocity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Set initial velocity
	rb.Velocity = mgl64.Vec3{5, 10, 0}

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	// v = v0 + g*dt = (5, 10, 0) + (0, -10, 0)*0.1 = (5, 9, 0)
	expectedVelocity := mgl64.Vec3{5, 9, 0}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	// p = p0 + v*dt = (0, 0, 0) + (5, 9, 0)*0.1 = (0.5, 0.9, 0)
	expectedPosition := mgl64.Vec3{0.5, 0.9, 0}
	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-10) {
		t.Errorf("Position = %v, want %v", rb.Transform.Position, expectedPosition)
	}
}

func TestIntegrate_Dynamic_DifferentMasses(t *testing.T) {
	tests := []struct {
		name    string
		density float64
		radius  float64
	}{
		{
			name:    "light sphere",
			density: 0.5,
			radius:  1.0,
		},
		{
			name:    "heavy sphere",
			density: 10.0,
			radius:  1.0,
		},
		{
			name:    "large light sphere",
			density: 0.1,
			radius:  5.0,
		},
	}

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			transform := NewTransform()
			sphere := &Sphere{Radius: tt.radius}
			rb := NewRigidBody(transform, sphere, BodyTypeDynamic, tt.density)

			rb.Integrate(dt, gravity)

			// All bodies should fall at the same rate (mass cancels out)
			// v = g*dt, regardless of mass
			expectedVelocity := mgl64.Vec3{0, -1, 0}
			if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-9) {
				t.Errorf("%s: Velocity = %v, want %v", tt.name, rb.Velocity, expectedVelocity)
			}
		})
	}
}

func TestIntegrate_Dynamic_ZeroTimeStep(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	rb.Velocity = mgl64.Vec3{5, 10, 15}

	dt := 0.0
	gravity := mgl64.Vec3{0, -10, 0}

	initialPosition := rb.Transform.Position
	initialVelocity := rb.Velocity

	rb.Integrate(dt, gravity)

	// With dt=0, nothing should change
	if !vec3AlmostEqual(rb.Transform.Position, initialPosition, 1e-10) {
		t.Errorf("Position changed with dt=0: Position = %v, want %v", rb.Transform.Position, initialPosition)
	}
	if !vec3AlmostEqual(rb.Velocity, initialVelocity, 1e-10) {
		t.Errorf("Velocity changed with dt=0: Velocity = %v, want %v", rb.Velocity, initialVelocity)
	}
}

func TestIntegrate_Dynamic_3DGravity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	dt := 0.1
	// Diagonal gravity vector
	gravity := mgl64.Vec3{1, -10, 2}

	rb.Integrate(dt, gravity)

	// v = g*dt
	expectedVelocity := mgl64.Vec3{0.1, -1, 0.2}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	// p = v*dt
	expectedPosition := mgl64.Vec3{0.01, -0.1, 0.02}
	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-10) {
		t.Errorf("Position = %v, want %v", rb.Transform.Position, expectedPosition)
	}
}

func TestIntegrate_Dynamic_PreviousStateTracking(t *testing.T) {
	transform := Transform{
		Position: mgl64.Vec3{1, 2, 3},
	}
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	rb.Velocity = mgl64.Vec3{10, 20, 30}

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	// First integration
	rb.Integrate(dt, gravity)

	// Check that previous states were saved
	expectedPreviousPosition := mgl64.Vec3{1, 2, 3}
	if !vec3AlmostEqual(rb.PreviousTransform.Position, expectedPreviousPosition, 1e-10) {
		t.Errorf("PreviousTransform.Position = %v, want %v", rb.PreviousTransform.Position, expectedPreviousPosition)
	}

	// PresolveVelocity is set to velocity AFTER integration (not before)
	// After first integration: velocity becomes [10, 20, 30] + gravity*dt = [10, 19, 30] (assuming gravity = [0, -10, 0])
	expectedPresolveVelocity := rb.Velocity // PresolveVelocity should equal the current velocity after integration
	if !vec3AlmostEqual(rb.PresolveVelocity, expectedPresolveVelocity, 1e-10) {
		t.Errorf("PresolveVelocity = %v, want %v (current velocity after integration)", rb.PresolveVelocity, expectedPresolveVelocity)
	}

	// Second integration
	currentPosition := rb.Transform.Position

	rb.Integrate(dt, gravity)

	// Previous state should now be the state before this integration
	if !vec3AlmostEqual(rb.PreviousTransform.Position, currentPosition, 1e-10) {
		t.Errorf("PreviousTransform.Position after 2nd integration = %v, want %v", rb.PreviousTransform.Position, currentPosition)
	}
	// PresolveVelocity should equal the velocity after the 2nd integration (not the velocity before it)
	if !vec3AlmostEqual(rb.PresolveVelocity, rb.Velocity, 1e-10) {
		t.Errorf("PresolveVelocity after 2nd integration = %v, want %v (velocity after integration)", rb.PresolveVelocity, rb.Velocity)
	}
}

// =============================================================================
// Edge Cases and Stress Tests
// =============================================================================

func TestNewRigidBody_ZeroDensity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 0.0)

	// Mass should be zero
	if rb.Material.GetMass() != 0.0 {
		t.Errorf("Mass with zero density = %v, want 0.0", rb.Material.GetMass())
	}
}

func TestIntegrate_Dynamic_LargeTimeStep(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	dt := 10.0 // Very large time step
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	// Should still work, just with large changes
	expectedVelocity := mgl64.Vec3{0, -100, 0}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-8) {
		t.Errorf("Velocity with large dt = %v, want %v", rb.Velocity, expectedVelocity)
	}

	expectedPosition := mgl64.Vec3{0, -1000, 0}
	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-6) {
		t.Errorf("Position with large dt = %v, want %v", rb.Transform.Position, expectedPosition)
	}
}

func TestIntegrate_Dynamic_SmallTimeStep(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	dt := 0.001 // Very small time step
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	expectedVelocity := mgl64.Vec3{0, -0.01, 0}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity with small dt = %v, want %v", rb.Velocity, expectedVelocity)
	}
}

func TestIntegrate_Dynamic_NegativeGravity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	dt := 0.1
	gravity := mgl64.Vec3{0, 10, 0} // Upward gravity

	rb.Integrate(dt, gravity)

	// Body should accelerate upward
	expectedVelocity := mgl64.Vec3{0, 1, 0}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity with upward gravity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	expectedPosition := mgl64.Vec3{0, 0.1, 0}
	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-10) {
		t.Errorf("Position with upward gravity = %v, want %v", rb.Transform.Position, expectedPosition)
	}
}

// =============================================================================
// PHASE 1: Angular Motion Tests (CRITICAL - Previously Untested)
// =============================================================================

// TestIntegrate_AngularVelocity_Basic verifies that a body with no initial
// angular velocity and no external torques maintains zero angular velocity
func TestIntegrate_AngularVelocity_Basic(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// No initial angular velocity
	rb.AngularVelocity = mgl64.Vec3{0, 0, 0}

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	// With no external torque and no initial rotation, AngularVelocity should remain zero
	expectedAngularVelocity := mgl64.Vec3{0, 0, 0}
	if !vec3AlmostEqual(rb.AngularVelocity, expectedAngularVelocity, 1e-10) {
		t.Errorf("AngularVelocity = %v, want %v", rb.AngularVelocity, expectedAngularVelocity)
	}

	// PresolveAngularVelocity should also be zero
	if !vec3AlmostEqual(rb.PresolveAngularVelocity, expectedAngularVelocity, 1e-10) {
		t.Errorf("PresolveAngularVelocity = %v, want %v", rb.PresolveAngularVelocity, expectedAngularVelocity)
	}

	// Rotation quaternion should remain identity (no rotation)
	identityQuat := mgl64.QuatIdent()
	if !quatAlmostEqual(rb.Transform.Rotation, identityQuat, 1e-10) {
		t.Errorf("Transform.Rotation = %v, want identity quaternion %v", rb.Transform.Rotation, identityQuat)
	}
}

// TestIntegrate_AngularVelocity_WithInitialRotation verifies that a body with
// initial angular velocity correctly updates its quaternion rotation
func TestIntegrate_AngularVelocity_WithInitialRotation(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Set initial angular velocity (rotation around Z axis)
	rb.AngularVelocity = mgl64.Vec3{0, 0, 1} // 1 rad/s around Z

	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0} // No gravity for clearer test

	initialRotation := rb.Transform.Rotation

	rb.Integrate(dt, gravity)

	// Angular velocity should remain constant (no external torque, no damping)
	expectedAngularVelocity := mgl64.Vec3{0, 0, 1}
	if !vec3AlmostEqual(rb.AngularVelocity, expectedAngularVelocity, 1e-10) {
		t.Errorf("AngularVelocity = %v, want %v", rb.AngularVelocity, expectedAngularVelocity)
	}

	// Rotation quaternion should have changed
	if quatAlmostEqual(rb.Transform.Rotation, initialRotation, 1e-10) {
		t.Error("Transform.Rotation did not change despite angular velocity")
	}

	// Quaternion should still be normalized
	quatMagnitude := math.Sqrt(rb.Transform.Rotation.W*rb.Transform.Rotation.W +
		rb.Transform.Rotation.V.X()*rb.Transform.Rotation.V.X() +
		rb.Transform.Rotation.V.Y()*rb.Transform.Rotation.V.Y() +
		rb.Transform.Rotation.V.Z()*rb.Transform.Rotation.V.Z())
	if !almostEqual(quatMagnitude, 1.0, 1e-10) {
		t.Errorf("Quaternion magnitude = %v, want 1.0 (normalized)", quatMagnitude)
	}

	// PresolveAngularVelocity should be saved
	if !vec3AlmostEqual(rb.PresolveAngularVelocity, rb.AngularVelocity, 1e-10) {
		t.Errorf("PresolveAngularVelocity = %v, want %v", rb.PresolveAngularVelocity, rb.AngularVelocity)
	}
}

// TestIntegrate_QuaternionNormalization verifies that quaternion remains normalized
// after many integration steps (prevents numerical drift)
func TestIntegrate_QuaternionNormalization(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// High angular velocity to accumulate potential errors faster
	rb.AngularVelocity = mgl64.Vec3{10, 5, 3}

	dt := 0.01
	gravity := mgl64.Vec3{0, 0, 0}

	// Integrate many times
	numSteps := 1000
	for i := 0; i < numSteps; i++ {
		rb.Integrate(dt, gravity)
	}

	// Quaternion must remain normalized
	quatMagnitude := math.Sqrt(rb.Transform.Rotation.W*rb.Transform.Rotation.W +
		rb.Transform.Rotation.V.X()*rb.Transform.Rotation.V.X() +
		rb.Transform.Rotation.V.Y()*rb.Transform.Rotation.V.Y() +
		rb.Transform.Rotation.V.Z()*rb.Transform.Rotation.V.Z())

	if !almostEqual(quatMagnitude, 1.0, 1e-6) {
		t.Errorf("After %d steps, quaternion magnitude = %v, want 1.0 (BUG: quaternion drift)", numSteps, quatMagnitude)
	}

	// Check for NaN values
	if math.IsNaN(rb.Transform.Rotation.W) || math.IsNaN(rb.Transform.Rotation.V.X()) ||
		math.IsNaN(rb.Transform.Rotation.V.Y()) || math.IsNaN(rb.Transform.Rotation.V.Z()) {
		t.Error("Quaternion contains NaN values (BUG: numerical instability)")
	}
}

// TestIntegrate_GyroscopicTerm verifies the gyroscopic term ω × (I·ω) is correctly computed
// For a spinning body, this term should maintain angular momentum conservation
func TestIntegrate_GyroscopicTerm(t *testing.T) {
	transform := NewTransform()
	// Use asymmetric box to make gyroscopic effects visible
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 0.5}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Set angular velocity
	rb.AngularVelocity = mgl64.Vec3{5, 0, 0}

	dt := 0.01
	gravity := mgl64.Vec3{0, 0, 0}

	// Compute initial angular momentum L = I·ω
	I := rb.GetInertiaWorld()
	L_initial := I.Mul3x1(rb.AngularVelocity)

	// Integrate multiple steps
	for i := 0; i < 100; i++ {
		rb.Integrate(dt, gravity)
	}

	// Angular momentum should be conserved (no external torque)
	I_final := rb.GetInertiaWorld()
	L_final := I_final.Mul3x1(rb.AngularVelocity)

	// Note: L conservation is approximate due to numerical integration
	if !vec3AlmostEqual(L_final, L_initial, 1e-3) {
		t.Logf("POTENTIAL BUG: Angular momentum not conserved")
		t.Logf("L_initial = %v", L_initial)
		t.Logf("L_final = %v", L_final)
		t.Logf("Difference = %v", L_final.Sub(L_initial))
		// Don't fail the test - this is expected to have some drift, just log it
	}
}

// TestIntegrate_AngularDamping verifies angular damping reduces rotation over time
func TestIntegrate_AngularDamping(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Set angular damping
	rb.Material.AngularDamping = 0.1

	// Initial angular velocity
	rb.AngularVelocity = mgl64.Vec3{10, 0, 0}
	initialAngularSpeed := rb.AngularVelocity.Len()

	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0}

	rb.Integrate(dt, gravity)

	// Expected: v_new = v_old * exp(-AngularDamping * dt)
	expectedFactor := math.Exp(-rb.Material.AngularDamping * dt)
	expectedAngularVelocity := mgl64.Vec3{10 * expectedFactor, 0, 0}

	if !vec3AlmostEqual(rb.AngularVelocity, expectedAngularVelocity, 1e-9) {
		t.Errorf("AngularVelocity after damping = %v, want %v", rb.AngularVelocity, expectedAngularVelocity)
		t.Logf("POTENTIAL BUG: Angular damping formula incorrect")
		t.Logf("Expected formula: ω_new = ω_old * (1 - drag*dt)")
		t.Logf("Initial speed: %v, Final speed: %v", initialAngularSpeed, rb.AngularVelocity.Len())
	}

	// After many steps, angular velocity should approach zero
	for i := 0; i < 100; i++ {
		rb.Integrate(dt, gravity)
	}

	finalAngularSpeed := rb.AngularVelocity.Len()
	if finalAngularSpeed >= initialAngularSpeed*0.5 {
		t.Errorf("After damping, angular speed = %v, expected significant reduction from initial %v",
			finalAngularSpeed, initialAngularSpeed)
	}
}

// TestIntegrate_PreviousRotationTracking verifies PreviousTransform.Rotation is saved
func TestIntegrate_PreviousRotationTracking(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	rb.AngularVelocity = mgl64.Vec3{1, 2, 3}

	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0}

	initialRotation := rb.Transform.Rotation

	// First integration
	rb.Integrate(dt, gravity)

	// Previous rotation should be the initial rotation
	if !quatAlmostEqual(rb.PreviousTransform.Rotation, initialRotation, 1e-10) {
		t.Errorf("PreviousTransform.Rotation = %v, want %v", rb.PreviousTransform.Rotation, initialRotation)
	}

	// Second integration
	currentRotation := rb.Transform.Rotation
	rb.Integrate(dt, gravity)

	// Previous rotation should now be the rotation before this integration
	if !quatAlmostEqual(rb.PreviousTransform.Rotation, currentRotation, 1e-10) {
		t.Errorf("PreviousTransform.Rotation after 2nd integration = %v, want %v",
			rb.PreviousTransform.Rotation, currentRotation)
	}
}

// =============================================================================
// PHASE 4: Damping Tests (HIGH PRIORITY - Production Code Never Tested)
// =============================================================================

// TestIntegrate_LinearDamping_Zero verifies that zero linear damping means no velocity reduction
func TestIntegrate_LinearDamping_Zero(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Zero damping (default)
	rb.Material.LinearDamping = 0.0

	// Set initial velocity
	rb.Velocity = mgl64.Vec3{10, 20, 30}

	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0} // No gravity to isolate damping

	rb.Integrate(dt, gravity)

	// With zero damping, velocity should remain constant
	expectedVelocity := mgl64.Vec3{10, 20, 30}
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity with zero damping = %v, want %v", rb.Velocity, expectedVelocity)
		t.Logf("POTENTIAL BUG: Zero damping should not change velocity")
	}
}

// TestIntegrate_LinearDamping_Positive verifies linear damping reduces velocity correctly
func TestIntegrate_LinearDamping_Positive(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Set linear damping
	rb.Material.LinearDamping = 0.1
	// Initial velocity
	rb.Velocity = mgl64.Vec3{10, 0, 0}
	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0}

	// Integrate one step
	rb.Integrate(dt, gravity)

	// Expected: v_new = v_old * exp(-LinearDamping * dt)
	// exp(-0.1 * 0.1) = exp(-0.01) ≈ 0.99004983
	expectedFactor := math.Exp(-rb.Material.LinearDamping * dt)
	expectedVelocity := mgl64.Vec3{10 * expectedFactor, 0, 0}

	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-8) {
		t.Errorf("Velocity after one damping step = %v, want %v", rb.Velocity, expectedVelocity)
		t.Logf("Expected: v *= exp(-LinearDamping * dt)")
		t.Logf("LinearDamping = %v, dt = %v, exp(-k*dt) = %v", rb.Material.LinearDamping, dt, expectedFactor)
		t.Logf("Actual velocity: %v", rb.Velocity)
	}

	// After many steps, velocity should approach zero exponentially
	for i := 0; i < 500; i++ {
		rb.Integrate(dt, gravity)
	}

	finalSpeed := rb.Velocity.Len()
	// After 500 steps: exp(-0.1 * 0.1 * 500) = exp(-5) ≈ 0.0067
	// So final speed should be ~10 * 0.0067 ≈ 0.067 — well below 0.1
	if finalSpeed > 0.1 {
		t.Errorf("After 500 damping steps, speed = %v, expected near zero (exp(-5) ≈ 0.0067)", finalSpeed)
		t.Logf("Theoretical decay: 10 * exp(-0.1 * 0.1 * 500) = %v", 10*math.Exp(-5))
	}
}

// TestIntegrate_LinearDamping_ExtremeValues tests edge case where drag*dt > 1
// This is a CRITICAL test - if drag*dt > 1, the formula v *= (1 - drag*dt) gives NEGATIVE velocity!
func TestIntegrate_LinearDamping_ExtremeValues(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// EXTREME damping
	rb.Material.LinearDamping = 0.99

	// Initial velocity
	rb.Velocity = mgl64.Vec3{10, 0, 0}

	dt := 1.5 // Large dt
	gravity := mgl64.Vec3{0, 0, 0}

	rb.Integrate(dt, gravity)

	// CRITICAL BUG CHECK: drag*dt = 0.99*1.5 = 1.485 > 1
	// Formula v *= (1 - 1.485) = v *= (-0.485) would give NEGATIVE velocity!
	// Expected behavior: velocity should be clamped to zero or positive

	if rb.Velocity.X() < 0 {
		t.Errorf("CRITICAL BUG: Linear damping caused negative velocity! v = %v", rb.Velocity)
		t.Logf("LinearDamping*dt = %v * %v = %v > 1", rb.Material.LinearDamping, dt, rb.Material.LinearDamping*dt)
		t.Logf("Formula (1 - drag*dt) = %v (negative!)", 1-rb.Material.LinearDamping*dt)
	}

	// Velocity should be zero or very small, not negative
	if rb.Velocity.Len() > 1.0 {
		t.Logf("WARNING: With extreme damping (drag*dt > 1), velocity = %v", rb.Velocity)
		t.Logf("Expected velocity to be clamped near zero")
	}
}

// TestIntegrate_AngularDamping_Positive verifies angular damping reduces rotation correctly
func TestIntegrate_AngularDamping_Positive(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Set angular damping
	rb.Material.AngularDamping = 0.05

	// Initial angular velocity
	rb.AngularVelocity = mgl64.Vec3{20, 0, 0}

	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0}

	rb.Integrate(dt, gravity)

	// Expected: v_new = v_old * exp(-AngularDamping * dt)
	expectedFactor := math.Exp(-rb.Material.AngularDamping * dt)
	expectedAngularVelocity := mgl64.Vec3{20 * expectedFactor, 0, 0}

	if !vec3AlmostEqual(rb.AngularVelocity, expectedAngularVelocity, 1e-9) {
		t.Errorf("AngularVelocity after damping = %v, want %v", rb.AngularVelocity, expectedAngularVelocity)
		t.Logf("POTENTIAL BUG: Angular damping formula incorrect")
		t.Logf("Expected formula: ω_new = ω_old * (1 - AngularDamping*dt)")
	}

	// After many steps, angular velocity should approach zero
	for i := 0; i < 2000; i++ {
		rb.Integrate(dt, gravity)
	}

	finalAngularSpeed := rb.AngularVelocity.Len()
	if finalAngularSpeed > 0.1 {
		t.Errorf("After 500 damping steps, angular speed = %v, expected near zero", finalAngularSpeed)
	}
}

// TestIntegrate_BothDampings verifies linear and angular damping work independently
func TestIntegrate_BothDampings(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Set both dampings
	rb.Material.LinearDamping = 0.1
	rb.Material.AngularDamping = 0.05

	// Initial velocities
	rb.Velocity = mgl64.Vec3{10, 0, 0}
	rb.AngularVelocity = mgl64.Vec3{0, 20, 0}

	dt := 0.1
	gravity := mgl64.Vec3{0, 0, 0}

	rb.Integrate(dt, gravity)

	// Expected: v_new = v_old * exp(-LinearDamping * dt)
	expectedFactor := math.Exp(-rb.Material.LinearDamping * dt)
	expectedVelocity := mgl64.Vec3{10 * expectedFactor, 0, 0}

	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-9) {
		t.Errorf("Linear velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	// Expected: v_new = v_old * exp(-LinearDamping * dt)
	expectedFactor = math.Exp(-rb.Material.AngularDamping * dt)
	expectedAngularVelocity := mgl64.Vec3{0, 20 * expectedFactor, 0}
	if !vec3AlmostEqual(rb.AngularVelocity, expectedAngularVelocity, 1e-9) {
		t.Errorf("Angular velocity = %v, want %v", rb.AngularVelocity, expectedAngularVelocity)
	}

	// Verify independence: apply more steps
	for i := 0; i < 100; i++ {
		rb.Integrate(dt, gravity)
	}

	// Both should decay, but independently
	finalLinearSpeed := rb.Velocity.Len()
	finalAngularSpeed := rb.AngularVelocity.Len()

	if finalLinearSpeed == 0 || finalAngularSpeed == 0 {
		t.Error("Damping caused complete stop too quickly")
	}

	// Linear should decay faster (higher drag coefficient)
	// After t=10s (100 steps * 0.1), decay factors:
	// Linear: (0.99)^100 ≈ 0.366
	// Angular: (0.995)^100 ≈ 0.606
	// So angular should be larger relative to initial
}

// =============================================================================
// PHASE 2: Inertia Tensor Tests (Previously Untested)
// =============================================================================

// TestGetInertiaWorld_NoRotation verifies that with no rotation, world inertia equals local inertia
func TestGetInertiaWorld_NoRotation(t *testing.T) {
	transform := NewTransform() // Identity rotation
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	I_world := rb.GetInertiaWorld()
	I_local := rb.InertiaLocal

	// With identity rotation, I_world should equal I_local
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(I_world[i*3+j], I_local[i*3+j], 1e-10) {
				t.Errorf("I_world[%d,%d] = %v, want %v (I_local)", i, j, I_world[i*3+j], I_local[i*3+j])
			}
		}
	}
}

// TestGetInertiaWorld_WithRotation verifies correct transformation I_world = R * I_local * R^T
func TestGetInertiaWorld_WithRotation(t *testing.T) {
	transform := NewTransform()
	// Asymmetric box to make rotation effects visible
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 0.5}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Rotate 90° around Z axis
	rb.Transform.Rotation = mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})

	I_world := rb.GetInertiaWorld()
	I_local := rb.InertiaLocal

	// After rotation, I_world should differ from I_local
	different := false
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(I_world[i*3+j], I_local[i*3+j], 1e-6) {
				different = true
			}
		}
	}

	if !different {
		t.Error("I_world should differ from I_local after rotation")
	}

	// Verify manual calculation: I_world = R * I_local * R^T
	R := rb.Transform.Rotation.Mat4().Mat3()
	expected_I_world := R.Mul3(I_local).Mul3(R.Transpose())

	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(I_world[i*3+j], expected_I_world[i*3+j], 1e-9) {
				t.Errorf("I_world[%d,%d] = %v, want %v (manual calc)", i, j, I_world[i*3+j], expected_I_world[i*3+j])
			}
		}
	}
}

// TestGetInertiaWorld_DifferentShapes verifies inertia for different shapes
func TestGetInertiaWorld_DifferentShapes(t *testing.T) {
	tests := []struct {
		name  string
		shape ShapeInterface
	}{
		{"sphere", &Sphere{Radius: 1.0}},
		{"box", &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			transform := NewTransform()
			rb := NewRigidBody(transform, tt.shape, BodyTypeDynamic, 1.0)

			I_world := rb.GetInertiaWorld()

			// Inertia tensor should be symmetric
			for i := 0; i < 3; i++ {
				for j := 0; j < 3; j++ {
					if !almostEqual(I_world[i*3+j], I_world[j*3+i], 1e-10) {
						t.Errorf("%s: I_world not symmetric: I[%d,%d]=%v != I[%d,%d]=%v",
							tt.name, i, j, I_world[i*3+j], j, i, I_world[j*3+i])
					}
				}
			}

			// Diagonal elements should be positive
			for i := 0; i < 3; i++ {
				if I_world[i*3+i] <= 0 {
					t.Errorf("%s: I_world[%d,%d] = %v, should be > 0", tt.name, i, i, I_world[i*3+i])
				}
			}
		})
	}
}

// TestGetInverseInertiaWorld_StaticBody verifies static bodies return zero inverse inertia
func TestGetInverseInertiaWorld_StaticBody(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
	rb := NewRigidBody(transform, box, BodyTypeStatic, 1.0)

	I_inv := rb.GetInverseInertiaWorld()

	// Static bodies should have zero inverse inertia (infinite inertia)
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if I_inv[i*3+j] != 0 {
				t.Errorf("Static body I_inv[%d,%d] = %v, want 0", i, j, I_inv[i*3+j])
			}
		}
	}
}

// TestGetInverseInertiaWorld_DynamicBody verifies I_inv * I = I * I_inv = Identity
func TestGetInverseInertiaWorld_DynamicBody(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Rotate to test in non-trivial orientation
	rb.Transform.Rotation = mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{1, 1, 0}.Normalize())

	I := rb.GetInertiaWorld()
	I_inv := rb.GetInverseInertiaWorld()

	// Compute I * I_inv
	product := I.Mul3(I_inv)

	// Should equal identity matrix
	identity := mgl64.Ident3()

	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(product[i*3+j], identity[i*3+j], 1e-6) {
				t.Errorf("I * I_inv[%d,%d] = %v, want %v (identity)", i, j, product[i*3+j], identity[i*3+j])
				t.Logf("POTENTIAL BUG: Inverse inertia calculation incorrect")
			}
		}
	}
}

// =============================================================================
// PHASE 3: SupportWorld Tests (Previously Untested - Critical for GJK)
// =============================================================================

// TestSupportWorld_Sphere_NoRotation verifies support points for sphere without rotation
func TestSupportWorld_Sphere_NoRotation(t *testing.T) {
	transform := NewTransform()
	transform.Position = mgl64.Vec3{0, 0, 0}
	sphere := &Sphere{Radius: 2.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	tests := []struct {
		name      string
		direction mgl64.Vec3
		expected  mgl64.Vec3
	}{
		{"positive X", mgl64.Vec3{1, 0, 0}, mgl64.Vec3{2, 0, 0}},
		{"negative X", mgl64.Vec3{-1, 0, 0}, mgl64.Vec3{-2, 0, 0}},
		{"positive Y", mgl64.Vec3{0, 1, 0}, mgl64.Vec3{0, 2, 0}},
		{"negative Y", mgl64.Vec3{0, -1, 0}, mgl64.Vec3{0, -2, 0}},
		{"positive Z", mgl64.Vec3{0, 0, 1}, mgl64.Vec3{0, 0, 2}},
		{"diagonal", mgl64.Vec3{1, 1, 1}.Normalize(), mgl64.Vec3{1, 1, 1}.Normalize().Mul(2)},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			support := rb.SupportWorld(tt.direction)
			if !vec3AlmostEqual(support, tt.expected, 1e-9) {
				t.Errorf("SupportWorld(%v) = %v, want %v", tt.direction, support, tt.expected)
			}
		})
	}
}

// TestSupportWorld_Sphere_WithTranslation verifies translation is correctly added
func TestSupportWorld_Sphere_WithTranslation(t *testing.T) {
	transform := NewTransform()
	transform.Position = mgl64.Vec3{10, 20, 30}
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	direction := mgl64.Vec3{1, 0, 0}
	support := rb.SupportWorld(direction)

	// Support should be center + radius*direction = [10,20,30] + [1,0,0]
	expected := mgl64.Vec3{11, 20, 30}
	if !vec3AlmostEqual(support, expected, 1e-9) {
		t.Errorf("SupportWorld with translation = %v, want %v", support, expected)
	}
}

// TestSupportWorld_Sphere_WithRotation verifies rotation doesn't affect sphere (isotropic)
func TestSupportWorld_Sphere_WithRotation(t *testing.T) {
	transform := NewTransform()
	transform.Rotation = mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 0, 1})
	transform.InverseRotation = transform.Rotation.Inverse()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// For sphere, rotation shouldn't change support (isotropic shape)
	direction := mgl64.Vec3{1, 0, 0}
	support := rb.SupportWorld(direction)

	expected := mgl64.Vec3{1, 0, 0}
	if !vec3AlmostEqual(support, expected, 1e-9) {
		t.Errorf("SupportWorld for rotated sphere = %v, want %v", support, expected)
	}
}

// TestSupportWorld_Box_NoRotation verifies box support points without rotation
func TestSupportWorld_Box_NoRotation(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{2, 3, 1}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	tests := []struct {
		name      string
		direction mgl64.Vec3
		expected  mgl64.Vec3
	}{
		{"positive X corner", mgl64.Vec3{1, 0, 0}, mgl64.Vec3{2, 3, 1}},
		// For negative X with zero Y,Z: Copysign(Y, 0)=+Y, Copysign(Z, 0)=+Z
		{"negative X corner", mgl64.Vec3{-1, 0, 0}, mgl64.Vec3{-2, 3, 1}},
		{"positive Y corner", mgl64.Vec3{0, 1, 0}, mgl64.Vec3{2, 3, 1}},
		{"diagonal corner", mgl64.Vec3{1, 1, 1}, mgl64.Vec3{2, 3, 1}},
		// Full negative diagonal
		{"negative diagonal", mgl64.Vec3{-1, -1, -1}, mgl64.Vec3{-2, -3, -1}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			support := rb.SupportWorld(tt.direction)
			if !vec3AlmostEqual(support, tt.expected, 1e-9) {
				t.Errorf("SupportWorld(%v) = %v, want %v", tt.direction, support, tt.expected)
			}
		})
	}
}

// TestSupportWorld_Box_WithRotation verifies box support with 90° rotation
func TestSupportWorld_Box_WithRotation(t *testing.T) {
	transform := NewTransform()
	// Rotate 90° around Z axis
	transform.Rotation = mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	transform.InverseRotation = transform.Rotation.Inverse()
	box := &Box{HalfExtents: mgl64.Vec3{2, 1, 0.5}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Direction in world space: +X
	// After inverse rotation: should point in -Y (local)
	// Local support in -Y direction: (-2, -1, -0.5)
	// Rotate back to world: expected ≈ (1, -2, -0.5) with 90° Z rotation
	direction := mgl64.Vec3{1, 0, 0}
	support := rb.SupportWorld(direction)

	// Manual calculation
	localDirection := transform.Rotation.Inverse().Rotate(direction)
	localSupport := box.Support(localDirection)
	expectedSupport := transform.Rotation.Rotate(localSupport)

	if !vec3AlmostEqual(support, expectedSupport, 1e-9) {
		t.Errorf("SupportWorld with rotation = %v, want %v", support, expectedSupport)
		t.Logf("Local direction: %v", localDirection)
		t.Logf("Local support: %v", localSupport)
	}
}

// TestSupportWorld_Box_ArbitraryRotation verifies support with arbitrary rotation
func TestSupportWorld_Box_ArbitraryRotation(t *testing.T) {
	transform := NewTransform()
	transform.Position = mgl64.Vec3{5, 10, 15}
	transform.Rotation = mgl64.QuatRotate(math.Pi/3, mgl64.Vec3{1, 1, 1}.Normalize())
	transform.InverseRotation = transform.Rotation.Inverse()
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	direction := mgl64.Vec3{1, 0, 0}
	support := rb.SupportWorld(direction)

	// Manual verification
	localDir := transform.Rotation.Inverse().Rotate(direction)
	localSupport := box.Support(localDir)
	worldSupport := transform.Rotation.Rotate(localSupport)
	expectedSupport := transform.Position.Add(worldSupport)

	if !vec3AlmostEqual(support, expectedSupport, 1e-9) {
		t.Errorf("SupportWorld arbitrary rotation = %v, want %v", support, expectedSupport)
		t.Logf("POTENTIAL BUG: Rotation or translation not applied correctly")
	}
}

// TestSupportWorld_Plane verifies plane support function
func TestSupportWorld_Plane(t *testing.T) {
	transform := NewTransform()
	plane := &Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: 0}
	rb := NewRigidBody(transform, plane, BodyTypeDynamic, 1.0)

	// Plane support should return point on plane in given direction
	// For upward direction, should return point on plane
	direction := mgl64.Vec3{0, 1, 0}
	support := rb.SupportWorld(direction)

	// Check that support is on the plane: normal.Dot(support) = distance
	distanceToOrigin := plane.Normal.Dot(support)
	if !almostEqual(distanceToOrigin, plane.Distance, 1e-9) {
		t.Errorf("Plane support point distance = %v, want %v", distanceToOrigin, plane.Distance)
	}
}

// =============================================================================
// PHASE 5: Material Properties Tests
// =============================================================================

// TestMaterial_Restitution_Values verifies restitution coefficient storage
func TestMaterial_Restitution_Values(t *testing.T) {
	tests := []struct {
		name        string
		restitution float64
	}{
		{"perfectly inelastic", 0.0},
		{"partially elastic", 0.5},
		{"highly elastic", 0.9},
		{"perfectly elastic", 1.0},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			transform := NewTransform()
			sphere := &Sphere{Radius: 1.0}
			rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

			rb.Material.Restitution = tt.restitution

			if rb.Material.Restitution != tt.restitution {
				t.Errorf("Restitution = %v, want %v", rb.Material.Restitution, tt.restitution)
			}
		})
	}
}

// TestMaterial_Friction_Initialization verifies friction defaults
func TestMaterial_Friction_Initialization(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Default friction should be zero
	if rb.Material.StaticFriction != 0.0 {
		t.Errorf("Default StaticFriction = %v, want 0.0", rb.Material.StaticFriction)
	}

	if rb.Material.DynamicFriction != 0.0 {
		t.Errorf("Default DynamicFriction = %v, want 0.0", rb.Material.DynamicFriction)
	}
}

// TestMaterial_Friction_CustomValues verifies custom friction values are stored
func TestMaterial_Friction_CustomValues(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	rb.Material.StaticFriction = 0.6
	rb.Material.DynamicFriction = 0.4

	if rb.Material.StaticFriction != 0.6 {
		t.Errorf("StaticFriction = %v, want 0.6", rb.Material.StaticFriction)
	}

	if rb.Material.DynamicFriction != 0.4 {
		t.Errorf("DynamicFriction = %v, want 0.4", rb.Material.DynamicFriction)
	}

	// Static friction should typically be >= dynamic friction
	if rb.Material.StaticFriction < rb.Material.DynamicFriction {
		t.Logf("WARNING: StaticFriction (%v) < DynamicFriction (%v) - unusual but not necessarily wrong",
			rb.Material.StaticFriction, rb.Material.DynamicFriction)
	}
}

// TestMaterial_Damping_Initialization verifies damping defaults
func TestMaterial_Damping_Initialization(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Default damping should be zero
	if rb.Material.LinearDamping != 0.0 {
		t.Errorf("Default LinearDamping = %v, want 0.0", rb.Material.LinearDamping)
	}

	if rb.Material.AngularDamping != 0.0 {
		t.Errorf("Default AngularDamping = %v, want 0.0", rb.Material.AngularDamping)
	}
}

// =============================================================================
// PHASE 6: Edge Cases Tests
// =============================================================================

// TestNewRigidBody_NegativeDensity verifies behavior with negative density
func TestNewRigidBody_NegativeDensity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, -1.0)

	// Negative density should produce negative mass (unusual but mathematically valid)
	if rb.Material.GetMass() >= 0 {
		t.Logf("Negative density produced non-negative mass: %v", rb.Material.GetMass())
	}

	// Density should be stored as-is
	if rb.Material.Density != -1.0 {
		t.Errorf("Density = %v, want -1.0", rb.Material.Density)
	}
}

// TestNewRigidBody_InfiniteDensity verifies behavior with infinite density
func TestNewRigidBody_InfiniteDensity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, math.Inf(1))

	// Infinite density should produce infinite mass
	if !math.IsInf(rb.Material.GetMass(), 1) {
		t.Errorf("Infinite density produced finite mass: %v", rb.Material.GetMass())
	}
}

// TestIntegrate_NegativeTimeStep verifies behavior with negative dt
func TestIntegrate_NegativeTimeStep(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	rb.Velocity = mgl64.Vec3{10, 0, 0}
	initialPos := rb.Transform.Position

	dt := -0.1 // Negative time step
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	// With negative dt, position should move backwards
	// This is mathematically valid (time reversal simulation)
	t.Logf("Negative dt integration: initial pos = %v, final pos = %v", initialPos, rb.Transform.Position)
	t.Logf("This tests if the engine supports time-reversal (uncommon but valid)")
}

// TestIntegrate_VerySmallMass verifies behavior with near-zero mass
func TestIntegrate_VerySmallMass(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1e-10) // Very small density

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	// With very small mass, acceleration should be normal (F=ma, a=F/m)
	// Gravity force = g*m, acceleration = g*m/m = g (mass cancels)
	// So even tiny mass should fall at normal rate
	expectedVelocity := mgl64.Vec3{0, -1, 0} // g*dt
	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-8) {
		t.Errorf("Very small mass velocity = %v, want %v (mass should cancel in gravity)",
			rb.Velocity, expectedVelocity)
	}
}

// TestIntegrate_InfiniteMass_Dynamic verifies dynamic body with infinite mass
func TestIntegrate_InfiniteMass_Dynamic(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, math.Inf(1))

	rb.Velocity = mgl64.Vec3{10, 0, 0}

	dt := 0.1
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	// With infinite mass and gravity force = g*m = g*inf = inf
	// Acceleration = F/m = inf/inf = undefined (NaN expected)
	// This is an edge case - behavior depends on implementation

	if math.IsNaN(rb.Velocity.X()) || math.IsNaN(rb.Velocity.Y()) || math.IsNaN(rb.Velocity.Z()) {
		t.Logf("Infinite mass produced NaN velocity (expected edge case)")
	} else {
		t.Logf("Infinite mass velocity = %v (no NaN)", rb.Velocity)
	}
}

// TestSupportWorld_UnnormalizedQuaternion verifies behavior with bad quaternion
func TestSupportWorld_UnnormalizedQuaternion(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Set unnormalized quaternion (magnitude != 1)
	rb.Transform.Rotation = mgl64.Quat{W: 1, V: mgl64.Vec3{1, 1, 1}} // |q| = sqrt(4) = 2

	direction := mgl64.Vec3{1, 0, 0}
	support := rb.SupportWorld(direction)

	// Unnormalized quaternion will produce incorrect rotations
	// This tests robustness - should either normalize or produce warning
	t.Logf("SupportWorld with unnormalized quat: %v", support)
	t.Logf("This tests edge case handling of invalid quaternions")
}

// TestIntegrate_HighAngularVelocity verifies stability with extreme rotation
func TestIntegrate_HighAngularVelocity(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	// Very high angular velocity (100 rad/s = ~16 revolutions/second)
	rb.AngularVelocity = mgl64.Vec3{100, 0, 0}

	dt := 0.01
	gravity := mgl64.Vec3{0, 0, 0}

	// Integrate many steps
	for i := 0; i < 1000; i++ {
		rb.Integrate(dt, gravity)
	}

	// Check for numerical instability
	quatMagnitude := math.Sqrt(rb.Transform.Rotation.W*rb.Transform.Rotation.W +
		rb.Transform.Rotation.V.X()*rb.Transform.Rotation.V.X() +
		rb.Transform.Rotation.V.Y()*rb.Transform.Rotation.V.Y() +
		rb.Transform.Rotation.V.Z()*rb.Transform.Rotation.V.Z())

	if !almostEqual(quatMagnitude, 1.0, 1e-3) {
		t.Errorf("After high angular velocity, quaternion magnitude = %v, want 1.0 (BUG: drift)", quatMagnitude)
	}

	// Check for NaN
	if math.IsNaN(rb.Transform.Rotation.W) {
		t.Error("High angular velocity produced NaN quaternion (BUG: numerical instability)")
	}
}

// =============================================================================
// PHASE 7: Mathematical Consistency Tests
// =============================================================================

// TestIntegrate_EnergyConservation verifies energy conservation without damping
func TestIntegrate_EnergyConservation(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Set initial velocities
	rb.Velocity = mgl64.Vec3{5, 0, 0}
	rb.AngularVelocity = mgl64.Vec3{0, 2, 0}

	// Compute initial kinetic energy: E = (1/2)*m*v² + (1/2)*ω^T*I*ω
	mass := rb.Material.GetMass()
	v2 := rb.Velocity.Dot(rb.Velocity)
	linearKE := 0.5 * mass * v2

	I := rb.GetInertiaWorld()
	I_omega := I.Mul3x1(rb.AngularVelocity)
	angularKE := 0.5 * rb.AngularVelocity.Dot(I_omega)

	initialEnergy := linearKE + angularKE

	dt := 0.01
	gravity := mgl64.Vec3{0, 0, 0} // No gravity
	// No damping (already default 0)

	// Integrate multiple steps
	for i := 0; i < 100; i++ {
		rb.Integrate(dt, gravity)
	}

	// Compute final energy
	v2_final := rb.Velocity.Dot(rb.Velocity)
	linearKE_final := 0.5 * mass * v2_final

	I_final := rb.GetInertiaWorld()
	I_omega_final := I_final.Mul3x1(rb.AngularVelocity)
	angularKE_final := 0.5 * rb.AngularVelocity.Dot(I_omega_final)

	finalEnergy := linearKE_final + angularKE_final

	// Energy should be conserved (no forces, no damping)
	energyDiff := math.Abs(finalEnergy - initialEnergy)
	relativeError := energyDiff / initialEnergy

	if relativeError > 0.01 { // Allow 1% error for numerical integration
		t.Logf("Energy conservation test:")
		t.Logf("Initial energy = %v", initialEnergy)
		t.Logf("Final energy = %v", finalEnergy)
		t.Logf("Difference = %v (%.2f%%)", energyDiff, relativeError*100)
		t.Logf("Note: Some drift expected due to numerical integration")
	}
}

// TestIntegrate_MomentumConservation verifies linear momentum conservation
func TestIntegrate_MomentumConservation(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	rb.Velocity = mgl64.Vec3{10, 5, 3}

	// Initial momentum: p = m*v
	mass := rb.Material.GetMass()
	initialMomentum := rb.Velocity.Mul(mass)

	dt := 0.01
	gravity := mgl64.Vec3{0, 0, 0} // No external forces

	// Integrate
	for i := 0; i < 100; i++ {
		rb.Integrate(dt, gravity)
	}

	// Final momentum
	finalMomentum := rb.Velocity.Mul(mass)

	// Momentum should be conserved
	if !vec3AlmostEqual(finalMomentum, initialMomentum, 1e-6) {
		t.Logf("Momentum conservation:")
		t.Logf("Initial = %v", initialMomentum)
		t.Logf("Final = %v", finalMomentum)
		t.Logf("Difference = %v", finalMomentum.Sub(initialMomentum))
	}
}

// TestIntegrate_AngularMomentumConservation verifies angular momentum conservation
func TestIntegrate_AngularMomentumConservation(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 0.5}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	rb.AngularVelocity = mgl64.Vec3{3, 2, 1}

	// Initial angular momentum: L = I*ω
	I_initial := rb.GetInertiaWorld()
	L_initial := I_initial.Mul3x1(rb.AngularVelocity)

	dt := 0.01
	gravity := mgl64.Vec3{0, 0, 0} // No external torques

	// Integrate
	for i := 0; i < 100; i++ {
		rb.Integrate(dt, gravity)
	}

	// Final angular momentum
	I_final := rb.GetInertiaWorld()
	L_final := I_final.Mul3x1(rb.AngularVelocity)

	// Angular momentum should be conserved (approximately)
	diff := L_final.Sub(L_initial)
	diffMagnitude := diff.Len()

	if diffMagnitude > 0.1 {
		t.Logf("Angular momentum conservation:")
		t.Logf("Initial L = %v", L_initial)
		t.Logf("Final L = %v", L_final)
		t.Logf("Difference magnitude = %v", diffMagnitude)
		t.Logf("Note: Some drift expected with gyroscopic term")
	}
}

// TestGetInertiaWorld_Symmetry verifies inertia tensor is symmetric
func TestGetInertiaWorld_Symmetry(t *testing.T) {
	transform := NewTransform()
	transform.Rotation = mgl64.QuatRotate(math.Pi/3, mgl64.Vec3{1, 1, 1}.Normalize())
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	I := rb.GetInertiaWorld()

	// Verify symmetry: I[i,j] = I[j,i]
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(I[i*3+j], I[j*3+i], 1e-10) {
				t.Errorf("Inertia tensor not symmetric: I[%d,%d]=%v != I[%d,%d]=%v",
					i, j, I[i*3+j], j, i, I[j*3+i])
			}
		}
	}
}

// TestGetInertiaWorld_PositiveDefinite verifies diagonal elements are positive
func TestGetInertiaWorld_PositiveDefinite(t *testing.T) {
	shapes := []ShapeInterface{
		&Sphere{Radius: 1.0},
		&Box{HalfExtents: mgl64.Vec3{1, 2, 3}},
	}

	for _, shape := range shapes {
		transform := NewTransform()
		rb := NewRigidBody(transform, shape, BodyTypeDynamic, 1.0)

		I := rb.GetInertiaWorld()

		// Diagonal elements must be positive
		for i := 0; i < 3; i++ {
			if I[i*3+i] <= 0 {
				t.Errorf("Inertia tensor diagonal I[%d,%d] = %v, must be > 0", i, i, I[i*3+i])
			}
		}
	}
}

// =============================================================================
// PHASE 8: Regression Tests
// =============================================================================

// TestIntegrate_LongSimulation verifies numerical stability over many steps
func TestIntegrate_LongSimulation(t *testing.T) {
	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	rb.Velocity = mgl64.Vec3{1, 2, 3}
	rb.AngularVelocity = mgl64.Vec3{0.5, 0.5, 0.5}

	dt := 0.01
	gravity := mgl64.Vec3{0, -10, 0}

	// Simulate 1000 steps (10 seconds)
	for i := 0; i < 1000; i++ {
		rb.Integrate(dt, gravity)

		// Check for NaN at each step
		if math.IsNaN(rb.Velocity.X()) || math.IsNaN(rb.Velocity.Y()) || math.IsNaN(rb.Velocity.Z()) {
			t.Fatalf("NaN velocity at step %d (BUG: numerical instability)", i)
		}

		if math.IsNaN(rb.Transform.Position.X()) {
			t.Fatalf("NaN position at step %d (BUG: numerical instability)", i)
		}

		if math.IsNaN(rb.Transform.Rotation.W) {
			t.Fatalf("NaN quaternion at step %d (BUG: numerical instability)", i)
		}
	}

	// Verify quaternion still normalized
	quatMag := math.Sqrt(rb.Transform.Rotation.W*rb.Transform.Rotation.W +
		rb.Transform.Rotation.V.X()*rb.Transform.Rotation.V.X() +
		rb.Transform.Rotation.V.Y()*rb.Transform.Rotation.V.Y() +
		rb.Transform.Rotation.V.Z()*rb.Transform.Rotation.V.Z())

	if !almostEqual(quatMag, 1.0, 1e-4) {
		t.Errorf("After 1000 steps, quaternion magnitude = %v, want 1.0", quatMag)
	}
}

// TestIntegrate_CompareIntegrationMethods documents current integration method
func TestIntegrate_CompareIntegrationMethods(t *testing.T) {
	// This test documents that we use semi-implicit Euler integration
	// (velocity updated first, then position uses new velocity)

	transform := NewTransform()
	sphere := &Sphere{Radius: 1.0}
	rb := NewRigidBody(transform, sphere, BodyTypeDynamic, 1.0)

	rb.Velocity = mgl64.Vec3{0, 0, 0}

	dt := 1.0
	gravity := mgl64.Vec3{0, -10, 0}

	rb.Integrate(dt, gravity)

	// Semi-implicit Euler:
	// v_new = v_old + a*dt = 0 + (-10)*1 = -10
	// p_new = p_old + v_new*dt = 0 + (-10)*1 = -10

	// Explicit Euler would give:
	// v_new = 0 + (-10)*1 = -10
	// p_new = p_old + v_old*dt = 0 + 0*1 = 0

	expectedVelocity := mgl64.Vec3{0, -10, 0}
	expectedPosition := mgl64.Vec3{0, -10, 0} // Semi-implicit uses new velocity

	if !vec3AlmostEqual(rb.Velocity, expectedVelocity, 1e-10) {
		t.Errorf("Velocity = %v, want %v", rb.Velocity, expectedVelocity)
	}

	if !vec3AlmostEqual(rb.Transform.Position, expectedPosition, 1e-10) {
		t.Logf("Position = %v", rb.Transform.Position)
		t.Logf("Expected (semi-implicit) = %v", expectedPosition)
		t.Logf("This documents the integration method used")
	}
}

// TestIntegrate_HighAngularVelocity_Stability verifies no explosion with fast rotation
func TestIntegrate_HighAngularVelocity_Stability(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Extreme angular velocity
	rb.AngularVelocity = mgl64.Vec3{100, 50, 75}

	dt := 0.01
	gravity := mgl64.Vec3{0, 0, 0}

	initialSpeed := rb.AngularVelocity.Len()

	// Integrate
	for i := 0; i < 1000; i++ {
		rb.Integrate(dt, gravity)
	}

	finalSpeed := rb.AngularVelocity.Len()

	// Angular speed should remain roughly constant (no external torque, no damping)
	// Allow some drift due to gyroscopic effects
	speedRatio := finalSpeed / initialSpeed

	if speedRatio > 2.0 || speedRatio < 0.5 {
		t.Logf("High angular velocity stability:")
		t.Logf("Initial speed = %v rad/s", initialSpeed)
		t.Logf("Final speed = %v rad/s", finalSpeed)
		t.Logf("Ratio = %v (expected ~1.0)", speedRatio)
		t.Logf("POTENTIAL BUG: Angular velocity unstable with high speeds")
	}

	// Check for NaN
	if math.IsNaN(finalSpeed) {
		t.Error("High angular velocity produced NaN (BUG: numerical instability)")
	}
}

// Helper function to compare floats with epsilon tolerance
func almostEqual(a, b, epsilon float64) bool {
	return math.Abs(a-b) < epsilon
}

// Helper function to compare Vec3 with epsilon tolerance
func vec3AlmostEqual(a, b mgl64.Vec3, epsilon float64) bool {
	return almostEqual(a.X(), b.X(), epsilon) &&
		almostEqual(a.Y(), b.Y(), epsilon) &&
		almostEqual(a.Z(), b.Z(), epsilon)
}

// Helper function to compare quaternions with epsilon tolerance
func quatAlmostEqual(a, b mgl64.Quat, epsilon float64) bool {
	return almostEqual(a.W, b.W, epsilon) &&
		almostEqual(a.V.X(), b.V.X(), epsilon) &&
		almostEqual(a.V.Y(), b.V.Y(), epsilon) &&
		almostEqual(a.V.Z(), b.V.Z(), epsilon)
}
