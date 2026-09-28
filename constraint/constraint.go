package constraint

import (
	"math"

	"github.com/akmonengine/feather/actor"
)

var posInf = math.Inf(1)

func ComputeRestitution(matA, matB actor.Material) float64 {
	// Average
	return (matA.Restitution + matB.Restitution) / 2.0
}

func ComputeStaticFriction(matA, matB actor.Material) float64 {
	// Geometric mean
	return math.Sqrt(matA.StaticFriction * matB.StaticFriction)
}

func ComputeDynamicFriction(matA, matB actor.Material) float64 {
	// Geometric mean
	return math.Sqrt(matA.DynamicFriction * matB.DynamicFriction)
}

// ComputeRollingResistance is the largest rolling resistance of both materials, times the largest radius
// of both shapes (0 for a box): it limits the torque that stops the rolling (as in Box2D)
func ComputeRollingResistance(matA, matB actor.Material, radiusA, radiusB float64) float64 {
	return math.Max(matA.RollingResistance, matB.RollingResistance) * math.Max(radiusA, radiusB)
}
