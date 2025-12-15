package constraint

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

func TestComputeRestitution(t *testing.T) {
	tests := []struct {
		name     string
		matA     actor.Material
		matB     actor.Material
		expected float64
	}{
		{
			name: "both zero restitution",
			matA: actor.Material{
				Restitution: 0.0,
			},
			matB: actor.Material{
				Restitution: 0.0,
			},
			expected: 0.0,
		},
		{
			name: "one zero, one high restitution - returns max",
			matA: actor.Material{
				Restitution: 0.0,
			},
			matB: actor.Material{
				Restitution: 0.8,
			},
			expected: 0.4,
		},
		{
			name: "both same restitution",
			matA: actor.Material{
				Restitution: 0.5,
			},
			matB: actor.Material{
				Restitution: 0.5,
			},
			expected: 0.5,
		},
		{
			name: "different restitutions - returns max",
			matA: actor.Material{
				Restitution: 0.3,
			},
			matB: actor.Material{
				Restitution: 0.7,
			},
			expected: 0.5,
		},
		{
			name: "both perfect restitution",
			matA: actor.Material{
				Restitution: 1.0,
			},
			matB: actor.Material{
				Restitution: 1.0,
			},
			expected: 1.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := ComputeRestitution(tt.matA, tt.matB)
			if math.Abs(result-tt.expected) > 1e-10 {
				t.Errorf("ComputeRestitution() = %v, want %v", result, tt.expected)
			}
		})
	}
}

func TestClampSmallVelocities(t *testing.T) {
	tests := []struct {
		name             string
		initialVelocity  mgl64.Vec3
		expectedVelocity mgl64.Vec3
		shouldClamp      bool
	}{
		{
			name:             "zero velocity stays zero",
			initialVelocity:  mgl64.Vec3{0, 0, 0},
			expectedVelocity: mgl64.Vec3{0, 0, 0},
			shouldClamp:      true,
		},
		{
			name:             "very small velocity gets clamped",
			initialVelocity:  mgl64.Vec3{1e-9, 1e-9, 1e-9},
			expectedVelocity: mgl64.Vec3{0, 0, 0},
			shouldClamp:      true,
		},
		{
			name:             "velocity at threshold gets clamped",
			initialVelocity:  mgl64.Vec3{5e-9, 5e-9, 0},
			expectedVelocity: mgl64.Vec3{0, 0, 0},
			shouldClamp:      true,
		},
		{
			name:             "normal velocity is not clamped",
			initialVelocity:  mgl64.Vec3{1.0, 2.0, 3.0},
			expectedVelocity: mgl64.Vec3{1.0, 2.0, 3.0},
			shouldClamp:      false,
		},
		{
			name:             "small but above threshold velocity is not clamped",
			initialVelocity:  mgl64.Vec3{2e-5, 0, 0},
			expectedVelocity: mgl64.Vec3{2e-5, 0, 0},
			shouldClamp:      false,
		},
		{
			name:             "negative velocity gets clamped if small enough",
			initialVelocity:  mgl64.Vec3{-1e-9, -1e-9, -1e-9},
			expectedVelocity: mgl64.Vec3{0, 0, 0},
			shouldClamp:      true,
		},
		{
			name:             "large negative velocity is not clamped",
			initialVelocity:  mgl64.Vec3{-5.0, -2.0, -1.0},
			expectedVelocity: mgl64.Vec3{-5.0, -2.0, -1.0},
			shouldClamp:      false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			rb := &actor.RigidBody{
				Velocity: tt.initialVelocity,
			}

			clampSmallVelocities(rb)

			// Check if velocity was clamped to zero
			if tt.shouldClamp {
				if rb.Velocity != tt.expectedVelocity {
					t.Errorf("clampSmallVelocities() velocity = %v, want %v", rb.Velocity, tt.expectedVelocity)
				}
			} else {
				// Check if velocity remained unchanged
				epsilon := 1e-10
				if math.Abs(rb.Velocity.X()-tt.expectedVelocity.X()) > epsilon ||
					math.Abs(rb.Velocity.Y()-tt.expectedVelocity.Y()) > epsilon ||
					math.Abs(rb.Velocity.Z()-tt.expectedVelocity.Z()) > epsilon {
					t.Errorf("clampSmallVelocities() velocity = %v, want %v", rb.Velocity, tt.expectedVelocity)
				}
			}
		})
	}
}
