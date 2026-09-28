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

// =============================================================================
// PHASE 1: Angular Motion Tests (CRITICAL - Previously Untested)
// =============================================================================

// =============================================================================
// PHASE 4: Damping Tests (HIGH PRIORITY - Production Code Never Tested)
// =============================================================================

// =============================================================================
// PHASE 2: Inertia Tensor Tests (Previously Untested)
// =============================================================================

// TestGetInertiaWorld_NoRotation verifies that with no rotation, world inertia equals local inertia
func TestGetInertiaWorld_NoRotation(t *testing.T) {
	transform := NewTransform() // Identity rotation
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	inertiaWorld := rb.GetInertiaWorld()
	inertiaLocal := rb.InertiaLocal

	// With identity rotation, inertiaWorld should equal inertiaLocal
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(inertiaWorld[i*3+j], inertiaLocal[i*3+j], 1e-10) {
				t.Errorf("inertiaWorld[%d,%d] = %v, want %v (inertiaLocal)", i, j, inertiaWorld[i*3+j], inertiaLocal[i*3+j])
			}
		}
	}
}

// TestGetInertiaWorld_WithRotation verifies correct transformation inertiaWorld = R * inertiaLocal * R^T
func TestGetInertiaWorld_WithRotation(t *testing.T) {
	transform := NewTransform()
	// Asymmetric box to make rotation effects visible
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 0.5}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Rotate 90° around Z axis
	rb.Transform.Rotation = mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})

	inertiaWorld := rb.GetInertiaWorld()
	inertiaLocal := rb.InertiaLocal

	// After rotation, inertiaWorld should differ from inertiaLocal
	different := false
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(inertiaWorld[i*3+j], inertiaLocal[i*3+j], 1e-6) {
				different = true
			}
		}
	}

	if !different {
		t.Error("inertiaWorld should differ from inertiaLocal after rotation")
	}

	// Verify manual calculation: inertiaWorld = R * inertiaLocal * R^T
	R := rb.Transform.Rotation.Mat4().Mat3()
	expectedInertiaWorld := R.Mul3(inertiaLocal).Mul3(R.Transpose())

	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(inertiaWorld[i*3+j], expectedInertiaWorld[i*3+j], 1e-9) {
				t.Errorf("inertiaWorld[%d,%d] = %v, want %v (manual calc)", i, j, inertiaWorld[i*3+j], expectedInertiaWorld[i*3+j])
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

			inertiaWorld := rb.GetInertiaWorld()

			// Inertia tensor should be symmetric
			for i := 0; i < 3; i++ {
				for j := 0; j < 3; j++ {
					if !almostEqual(inertiaWorld[i*3+j], inertiaWorld[j*3+i], 1e-10) {
						t.Errorf("%s: inertiaWorld not symmetric: I[%d,%d]=%v != I[%d,%d]=%v",
							tt.name, i, j, inertiaWorld[i*3+j], j, i, inertiaWorld[j*3+i])
					}
				}
			}

			// Diagonal elements should be positive
			for i := 0; i < 3; i++ {
				if inertiaWorld[i*3+i] <= 0 {
					t.Errorf("%s: inertiaWorld[%d,%d] = %v, should be > 0", tt.name, i, i, inertiaWorld[i*3+i])
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

	inverseInertia := rb.GetInverseInertiaWorld()

	// Static bodies should have zero inverse inertia (infinite inertia)
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if inverseInertia[i*3+j] != 0 {
				t.Errorf("Static body inverseInertia[%d,%d] = %v, want 0", i, j, inverseInertia[i*3+j])
			}
		}
	}
}

// TestGetInverseInertiaWorld_DynamicBody verifies inverseInertia * I = I * inverseInertia = Identity
func TestGetInverseInertiaWorld_DynamicBody(t *testing.T) {
	transform := NewTransform()
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
	rb := NewRigidBody(transform, box, BodyTypeDynamic, 1.0)

	// Rotate to test in non-trivial orientation
	rb.Transform.Rotation = mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{1, 1, 0}.Normalize())

	I := rb.GetInertiaWorld()
	inverseInertia := rb.GetInverseInertiaWorld()

	// Compute I * inverseInertia
	product := I.Mul3(inverseInertia)

	// Should equal identity matrix
	identity := mgl64.Ident3()

	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !almostEqual(product[i*3+j], identity[i*3+j], 1e-6) {
				t.Errorf("I * inverseInertia[%d,%d] = %v, want %v (identity)", i, j, product[i*3+j], identity[i*3+j])
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

// =============================================================================
// PHASE 7: Mathematical Consistency Tests
// =============================================================================

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

// A static body has no inertia: no infinite or NaN value, and a unique serial like every body
func TestStaticBodyInertia(t *testing.T) {
	static := NewRigidBody(NewTransform(), &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}, BodyTypeStatic, 0)
	for i := 0; i < 9; i++ {
		if static.InertiaLocal[i] != 0 || static.InverseInertiaLocal[i] != 0 {
			t.Fatalf("static inertia %v, inverse %v: want 0", static.InertiaLocal, static.InverseInertiaLocal)
		}
	}
	other := NewRigidBody(NewTransform(), &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}, BodyTypeStatic, 0)
	if static.Serial() == 0 || static.Serial() == other.Serial() {
		t.Errorf("serials %d and %d", static.Serial(), other.Serial())
	}
}
