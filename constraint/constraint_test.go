package constraint

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
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
