package feather

import "time"

// ========== PROFILE ==========

// Profile is the time spent in each phase of the last step, as the b2Profile of Box2D. The phases follow each other:
// their sum is the step, but for the bookkeeping between them
type Profile struct {
	Step time.Duration
	// BroadPhase: the AABBs of the bodies and the pairs of the spatial grid
	BroadPhase time.Duration
	// NarrowPhase: the contacts of the pairs
	NarrowPhase time.Duration
	// Prepare: the events of the contacts, the warm start and the constraints of the solver
	Prepare time.Duration
	// Substeps: the velocities, the constraints and the positions, for all the substeps
	Substeps time.Duration
	// Restitution: the bounces and the impulses stored for the next step
	Restitution time.Duration
	// Continuous: the continuous collision of the fast bodies
	Continuous time.Duration
	// Islands: the sleep of the bodies, and the events
	Islands time.Duration
}

// Profile of the last step
func (w *World) Profile() Profile {
	return w.profile
}
