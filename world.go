package feather

import (
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

const DEFAULT_WORKERS = 1

type World struct {
	// List of all rigid bodies in the world
	Bodies []*actor.RigidBody
	// Gravity acceleration (m/s², or N/kg)
	Gravity     mgl64.Vec3
	Substeps    int
	SpatialGrid *SpatialGrid
	Workers     int

	Events Events
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

	delete(w.Events.sleepStates, body)
	for pair := range w.Events.previousActivePairs {
		if pair.bodyA == body || pair.bodyB == body {
			delete(w.Events.previousActivePairs, pair)
		}
	}
}

func (w *World) Step(dt float64) {
	w.Workers = max(DEFAULT_WORKERS, w.Workers)
	h := dt / float64(w.Substeps)

	for range w.Substeps {
		w.integrate(h)

		// Phase 2.0: Collision pair finding - Broad phase
		// Phase 2.1: Collision pair finding - narrow phase
		constraints := w.detectCollision()

		constraints = w.Events.recordCollisions(constraints)

		// Phase 3: Solver, only one iteration is required thanks to substeps
		w.solvePosition(h, constraints)

		// Phase 4: Update Position & Velocity
		// Calculate final velocities and commit positions
		w.update(h)

		// Phase 5: Velocity
		w.solveVelocity(h, constraints)

		w.trySleep(h)
	}

	w.Events.processSleepEvents(w.Bodies)
	w.Events.flush()
}

func (w *World) integrate(h float64) {
	task(w.Workers, w.Bodies, func(body *actor.RigidBody) {
		body.Integrate(h, w.Gravity)
	})
}

func (w *World) detectCollision() []*constraint.ContactConstraint {
	return NarrowPhase(BroadPhase(w.SpatialGrid, w.Bodies, w.Workers), w.Workers)
}

func (w *World) solvePosition(h float64, constraints []*constraint.ContactConstraint) {
	task(w.Workers, constraints, func(constraint *constraint.ContactConstraint) {
		constraint.SolvePosition(h)
	})
}

func (w *World) update(h float64) {
	task(w.Workers, w.Bodies, func(body *actor.RigidBody) {
		body.Update(h)
	})
}

func (w *World) solveVelocity(h float64, constraints []*constraint.ContactConstraint) {
	task(w.Workers, constraints, func(constraint *constraint.ContactConstraint) {
		constraint.SolveVelocity(h)
	})
}

// trySleep sets the body to sleep if its velocity is lower than the threshold, for a given duration
// this method is too simple to use a task, it slows down in multiple goroutines
func (w *World) trySleep(h float64) {
	for _, body := range w.Bodies {
		body.TrySleep(h, 0.1, 0.05)
	}
}
