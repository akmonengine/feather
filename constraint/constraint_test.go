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

// The combined resistance: the largest of both materials, times the largest radius (as the rolling resistance)
func TestComputeSpinningResistance(t *testing.T) {
	cases := []struct {
		a, b, radiusA, radiusB, want float64
	}{
		{0, 0, 0.1, 0.2, 0},
		{0.05, 0, 0.1, 0, 0.005},
		{0, 0.05, 0.1, 0, 0.005},
		{0.05, 0.2, 0.1, 0.3, 0.06},
		{0.2, 0.05, 0.3, 0.1, 0.06},
		{0.05, 0.05, 0, 0, 0},
	}
	for _, c := range cases {
		got := ComputeSpinningResistance(actor.Material{SpinningResistance: c.a}, actor.Material{SpinningResistance: c.b}, c.radiusA, c.radiusB)
		if math.Abs(got-c.want) > 1e-15 {
			t.Errorf("resistances %v & %v, radii %v & %v: %v, want %v", c.a, c.b, c.radiusA, c.radiusB, got, c.want)
		}
	}
	// the rolling resistance is not the spinning one
	if got := ComputeSpinningResistance(actor.Material{RollingResistance: 1}, actor.Material{RollingResistance: 1}, 1, 1); got != 0 {
		t.Errorf("the rolling resistance gave a spinning resistance of %v", got)
	}
}
