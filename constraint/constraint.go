package constraint

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

type Constraint interface {
	SolvePosition(dt float64)
	SolveVelocity(dt float64)
}

func ComputeRestitution(matA, matB actor.Material) float64 {
	// Option 1: Average (more realistic)
	return (matA.Restitution + matB.Restitution) / 2.0

	// Option 2: Maximum (if one bounces, it bounces)
	//return math.Max(matA.Restitution, matB.Restitution)

	// Option 3: Geometric mean (Box2D approach)
	// return math.Sqrt(matA.Restitution * matB.Restitution)
}

func ComputeStaticFriction(matA, matB actor.Material) float64 {
	// Moyenne géométrique (standard en physique)
	return math.Sqrt(matA.StaticFriction * matB.StaticFriction)
}

func ComputeDynamicFriction(matA, matB actor.Material) float64 {
	return math.Sqrt(matA.DynamicFriction * matB.DynamicFriction)
}

func clampSmallVelocities(rb *actor.RigidBody) {
	const velocityThreshold = 1e-5

	if rb.Velocity.Len() < velocityThreshold {
		rb.Velocity = mgl64.Vec3{0, 0, 0}
	}
	if rb.AngularVelocity.Len() < velocityThreshold {
		rb.AngularVelocity = mgl64.Vec3{0, 0, 0}
	}
}
