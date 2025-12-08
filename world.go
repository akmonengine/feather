package feather

import (
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

type World struct {
	// List of all rigid bodies in the world
	Bodies []*actor.RigidBody
	// Gravity acceleration (m/s², or N/kg)
	Gravity  mgl64.Vec3
	Substeps int
}

// AddBody adds a rigid body to the world
func (w *World) AddBody(body *actor.RigidBody) {
	w.Bodies = append(w.Bodies, body)
}

// RemoveBody removes a rigid body from the world
func (w *World) RemoveBody(body *actor.RigidBody) {
	k := -1
	for i, b := range w.Bodies {
		if b == body {
			k = i
			break
		}
	}

	if k != -1 {
		w.Bodies = append(w.Bodies[:k], w.Bodies[k+1:]...)
	}
}

func (w *World) Step(dt float64) {
	h := dt / float64(w.Substeps)

	for range w.Substeps {
		// Phase 1: Integrate new position
		for _, body := range w.Bodies {
			body.Integrate(h, w.Gravity)
		}
		//Phase pre-2.0: compute all aabbs once per substep
		for _, body := range w.Bodies {
			body.Shape.ComputeAABB(body.Transform)
		}

		// Phase 2.0: Collision pair finding - Broad phase
		collisionPairs := BroadPhase(w.Bodies)
		// Phase 2.1: Collision pair finding - narrow phase
		constraints := NarrowPhase(collisionPairs)

		// Phase 3: Solver, only one iteration is required thanks to substeps
		for _, c := range constraints {
			c.SolvePosition(h)
		}

		// Phase 4: Update Position & Velocity
		// Calculate final velocities and commit positions
		for _, body := range w.Bodies {
			body.Update(h)
		}

		// Phase 5: Velocity
		for _, c := range constraints {
			c.SolveVelocity(h)
		}

		for _, body := range w.Bodies {
			body.TrySleep(h, 10*h, 1e-2) // Seuil de vitesse pour le sleeping
		}
	}
}
