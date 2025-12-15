package epa

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// TestSnapNormalToAxis tests the normal snapping function for numerical stability
func TestSnapNormalToAxis(t *testing.T) {
	tests := []struct {
		name     string
		input    mgl64.Vec3
		expected mgl64.Vec3
	}{
		{
			name:     "small_x_component",
			input:    mgl64.Vec3{1e-9, 1.0, 0.0},
			expected: mgl64.Vec3{0.0, 1.0, 0.0},
		},
		{
			name:     "small_y_component",
			input:    mgl64.Vec3{1.0, 1e-9, 0.0},
			expected: mgl64.Vec3{1.0, 0.0, 0.0},
		},
		{
			name:     "small_z_component",
			input:    mgl64.Vec3{0.0, 1.0, 1e-9},
			expected: mgl64.Vec3{0.0, 1.0, 0.0},
		},
		{
			name:     "already_axis_aligned_x",
			input:    mgl64.Vec3{1.0, 0.0, 0.0},
			expected: mgl64.Vec3{1.0, 0.0, 0.0},
		},
		{
			name:     "diagonal_normal",
			input:    mgl64.Vec3{1.0, 1.0, 1.0}.Normalize(),
			expected: mgl64.Vec3{1.0, 1.0, 1.0}.Normalize(),
		},
		{
			name:     "near_zero_vector",
			input:    mgl64.Vec3{1e-9, 1e-9, 1e-9},
			expected: mgl64.Vec3{0.0, 1.0, 0.0}, // Default fallback
		},
		{
			name:     "multiple_small_components",
			input:    mgl64.Vec3{1e-8, 1e-8, 1.0},
			expected: mgl64.Vec3{0.0, 0.0, 1.0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := snapNormalToAxis(tt.input)

			if !vec3ApproxEqual(result, tt.expected, 1e-6) {
				t.Errorf("snapNormalToAxis(%v) = %v, want %v", tt.input, result, tt.expected)
			}

			// Verify result is normalized
			if !isNormalized(result, 1e-6) {
				t.Errorf("result is not normalized: length = %v", result.Len())
			}
		})
	}
}

// TestHandleDegenerateSimplex tests the handling of degenerate GJK simplex cases
func TestHandleDegenerateSimplex(t *testing.T) {
	// Create mock rigid bodies with shapes
	bodyA := &actor.RigidBody{
		Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
		Transform: actor.Transform{
			Position: mgl64.Vec3{0, 0, 0},
			Rotation: mgl64.QuatIdent(),
		},
	}

	bodyB := &actor.RigidBody{
		Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
		Transform: actor.Transform{
			Position: mgl64.Vec3{0, 1.0, 0},
			Rotation: mgl64.QuatIdent(),
		},
	}

	t.Run("two_points_simplex", func(t *testing.T) {
		simplex := &gjk.Simplex{}
		simplex.Points[0] = mgl64.Vec3{0, 0.5, 0}
		simplex.Points[1] = mgl64.Vec3{0, 0.6, 0}
		simplex.Count = 2

		result := handleDegenerateSimplex(bodyA, bodyB, simplex)

		// Should return a valid contact constraint
		if result.Normal.Len() == 0 {
			t.Error("normal should not be zero vector")
		}

		// Check that we have contact points (the actual penetration is in the points)
		if len(result.Points) == 0 {
			t.Errorf("should have at least one contact point")
		}

		// Normal should be approximately in the direction from A to B
		expectedDir := mgl64.Vec3{0, 1, 0}
		if result.Normal.Dot(expectedDir) <= 0 {
			t.Errorf("normal should point upward, got %v", result.Normal)
		}
	})

	t.Run("one_point_simplex", func(t *testing.T) {
		simplex := &gjk.Simplex{}
		simplex.Points[0] = mgl64.Vec3{0, 0.5, 0}
		simplex.Count = 1

		result := handleDegenerateSimplex(bodyA, bodyB, simplex)

		// Should use center-based estimation
		if result.Normal.Len() == 0 {
			t.Error("normal should not be zero vector")
		}

		// For degenerate cases, we just check that we have a valid result
		if len(result.Points) == 0 {
			t.Errorf("should have at least one contact point even in degenerate case")
		}
	})

	t.Run("aligned_centers", func(t *testing.T) {
		// Same position bodies
		bodyA.Transform.Position = mgl64.Vec3{0, 0, 0}
		bodyB.Transform.Position = mgl64.Vec3{0, 0, 0}

		simplex := &gjk.Simplex{}
		simplex.Count = 1

		result := handleDegenerateSimplex(bodyA, bodyB, simplex)

		// Should use default upward normal
		expectedNormal := mgl64.Vec3{0, 1, 0}
		if !vec3ApproxEqual(result.Normal, expectedNormal, 1e-6) {
			t.Errorf("normal = %v, want %v for aligned centers", result.Normal, expectedNormal)
		}
	})

	t.Run("close_centers", func(t *testing.T) {
		// Very close but not identical centers
		bodyA.Transform.Position = mgl64.Vec3{0, 0, 0}
		bodyB.Transform.Position = mgl64.Vec3{1e-8, 1e-8, 1e-8}

		simplex := &gjk.Simplex{}
		simplex.Count = 1

		result := handleDegenerateSimplex(bodyA, bodyB, simplex)

		// Should still work and return a valid normal
		if result.Normal.Len() == 0 {
			t.Error("normal should not be zero vector")
		}
	})
}

// TestEPA tests the main EPA function
func TestEPA(t *testing.T) {
	t.Run("convergence_success", func(t *testing.T) {
		// Create two overlapping boxes
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.5, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		// Create a valid tetrahedron simplex
		simplex := &gjk.Simplex{}
		simplex.Points[0] = mgl64.Vec3{0.5, 0.5, 0.5}
		simplex.Points[1] = mgl64.Vec3{-0.5, 0.5, 0.5}
		simplex.Points[2] = mgl64.Vec3{0.5, -0.5, 0.5}
		simplex.Points[3] = mgl64.Vec3{0.5, 0.5, -0.5}
		simplex.Count = 4

		result, err := EPA(bodyA, bodyB, simplex)

		if err != nil {
			t.Fatalf("EPA failed: %v", err)
		}

		// Verify result
		if result.Normal.Len() == 0 {
			t.Error("normal should not be zero vector")
		}

		if len(result.Points) == 0 {
			t.Error("should have at least one contact point")
		}

		// Normal should point from A to B (upward)
		if result.Normal.Y() <= 0 {
			t.Errorf("normal should point upward, got %v", result.Normal)
		}

		// Check that we have reasonable contact points
		if len(result.Points) == 0 {
			t.Errorf("should have at least one contact point")
		}
	})

	t.Run("degenerate_simplex", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		// Degenerate simplex with only 2 points
		simplex := &gjk.Simplex{}
		simplex.Points[0] = mgl64.Vec3{0, 0.5, 0}
		simplex.Points[1] = mgl64.Vec3{0, 0.6, 0}
		simplex.Count = 2

		result, err := EPA(bodyA, bodyB, simplex)

		if err != nil {
			t.Fatalf("EPA failed: %v", err)
		}

		// Should handle degenerate case gracefully
		if result.Normal.Len() == 0 {
			t.Error("normal should not be zero vector")
		}

		if len(result.Points) == 0 {
			t.Error("should have at least one contact point even with degenerate simplex")
		}
	})

	t.Run("single_point_simplex", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		// Single point simplex
		simplex := &gjk.Simplex{}
		simplex.Points[0] = mgl64.Vec3{0, 0.5, 0}
		simplex.Count = 1

		result, err := EPA(bodyA, bodyB, simplex)

		if err != nil {
			t.Fatalf("EPA failed: %v", err)
		}

		// Should handle single point case
		if result.Normal.Len() == 0 {
			t.Error("normal should not be zero vector")
		}
	})

	t.Run("convergence_with_rotation", func(t *testing.T) {
		// Test with rotated boxes
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0}),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.5, 0},
				Rotation: mgl64.QuatRotate(math.Pi/6, mgl64.Vec3{0, 1, 0}),
			},
		}

		// Create a valid simplex
		simplex := &gjk.Simplex{}
		simplex.Points[0] = mgl64.Vec3{0.5, 0.5, 0.5}
		simplex.Points[1] = mgl64.Vec3{-0.5, 0.5, 0.5}
		simplex.Points[2] = mgl64.Vec3{0.5, -0.5, 0.5}
		simplex.Points[3] = mgl64.Vec3{0.5, 0.5, -0.5}
		simplex.Count = 4

		result, err := EPA(bodyA, bodyB, simplex)

		if err != nil {
			t.Fatalf("EPA failed with rotation: %v", err)
		}

		// Should still converge
		if result.Normal.Len() == 0 {
			t.Error("normal should not be zero vector with rotation")
		}

		if len(result.Points) == 0 {
			t.Error("should have contact points with rotation")
		}
	})
}

// TestEPAIntegration tests the integration between GJK and EPA
func TestEPAIntegration(t *testing.T) {
	t.Run("box_box_collision", func(t *testing.T) {
		// Create two boxes that are clearly overlapping
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.5, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		// First run GJK to get simplex
		simplex := &gjk.Simplex{}
		if !gjk.GJK(bodyA, bodyB, simplex) {
			t.Skip("GJK did not detect collision, skipping EPA test")
		}

		if simplex.Count < 4 {
			t.Skip("GJK returned degenerate simplex, skipping")
		}

		// Then run EPA
		epaResult, err := EPA(bodyA, bodyB, simplex)

		if err != nil {
			t.Fatalf("EPA failed: %v", err)
		}

		// Verify integration results
		if epaResult.Normal.Len() == 0 {
			t.Error("EPA result normal should not be zero")
		}

		if len(epaResult.Points) == 0 {
			t.Error("EPA should return at least one contact point")
		}

		// The normal should be consistent with collision direction
		expectedNormal := mgl64.Vec3{0, 1, 0}
		if epaResult.Normal.Dot(expectedNormal) <= 0 {
			t.Errorf("EPA normal %v should be in same direction as expected %v",
				epaResult.Normal, expectedNormal)
		}

		// Check penetration in contact points
		if len(epaResult.Points) > 0 {
			for _, point := range epaResult.Points {
				if point.Penetration <= 0 || point.Penetration > 2.0 {
					t.Errorf("penetration should be reasonable, got %v", point.Penetration)
				}
			}
		}
	})

	t.Run("sphere_sphere_collision", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.9, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		// Run GJK
		simplex := &gjk.Simplex{}
		if !gjk.GJK(bodyA, bodyB, simplex) {
			t.Skip("GJK did not detect collision")
		}

		// Run EPA
		epaResult, err := EPA(bodyA, bodyB, simplex)

		if err != nil {
			t.Fatalf("EPA failed: %v", err)
		}

		// Spheres should have single contact point
		if len(epaResult.Points) != 1 {
			t.Errorf("Expected 1 contact point for spheres, got %d", len(epaResult.Points))
		}

		// Normal should be in the correct direction
		expectedNormal := mgl64.Vec3{0, 1, 0}
		if epaResult.Normal.Dot(expectedNormal) <= 0 {
			t.Errorf("EPA normal %v should be in same direction as expected %v",
				epaResult.Normal, expectedNormal)
		}
	})

	t.Run("rotated_boxes_collision", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(math.Pi/6, mgl64.Vec3{0, 1, 0}),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.8, 0},
				Rotation: mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0}),
			},
		}

		// Run GJK
		simplex := &gjk.Simplex{}
		if !gjk.GJK(bodyA, bodyB, simplex) {
			t.Skip("GJK did not detect collision")
		}

		// Run EPA
		epaResult, err := EPA(bodyA, bodyB, simplex)

		if err != nil {
			t.Fatalf("EPA failed: %v", err)
		}

		// Should work with rotation
		if len(epaResult.Points) == 0 {
			t.Error("should have contact points with rotation")
		}
	})
}
