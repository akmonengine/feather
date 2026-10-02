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

// ComputeSpinningResistance is the largest spinning resistance of both materials, times the largest radius of both
// shapes (0 for a box), as the rolling resistance: it limits the torque that stops the spin around the normal (the
// spinning friction of Bullet, whose coefficient is this length)
func ComputeSpinningResistance(matA, matB actor.Material, radiusA, radiusB float64) float64 {
	return math.Max(matA.SpinningResistance, matB.SpinningResistance) * math.Max(radiusA, radiusB)
}
