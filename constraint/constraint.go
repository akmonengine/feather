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
