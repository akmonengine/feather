package feather

import (
	"sync"

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
	// Workers is the number of goroutines for the collision detection.
	// The result is exactly the same whatever the value.
	Workers int
	// ContactHertz is the stiffness of the contacts (0 = DefaultContactHertz)
	// Higher values = less overlap under load, lower values = softer contacts.
	// It is capped to 1/8 of the sub-steps rate.
	ContactHertz float64

	Events Events

	solver solver
	// contacts of the previous step, to warm start the solver
	contacts      []constraint.Manifold
	contactsIndex map[pairKey]int
	previous      []constraint.Manifold
	aabbs         []actor.AABB
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

	w.Events.forget(body)
	n := 0
	for _, contact := range w.contacts {
		if contact.BodyA != body && contact.BodyB != body {
			w.contacts[n] = contact
			n++
		}
	}
	w.contacts = w.contacts[:n]
	w.indexContacts()
}

// Contacts returns the contacts of the last step, with the impulses applied by the solver
func (w *World) Contacts() []constraint.Manifold {
	return w.contacts
}

func (w *World) Step(dt float64) {
	if dt <= 0 {
		return
	}
	workers := max(DEFAULT_WORKERS, w.Workers)
	substeps := max(1, w.Substeps)
	contactHertz := w.ContactHertz
	if contactHertz <= 0 {
		contactHertz = DefaultContactHertz
	}

	w.wakeTouchedBodies()

	// Phase 1: Collision detection, once per step - broad phase & narrow phase
	manifolds := w.detectCollision(dt, workers)
	manifolds = w.Events.recordCollisions(manifolds)
	w.warmStart(manifolds)

	// Phase 2: Solver, with substeps
	s := &w.solver
	s.prepare(w.Bodies, manifolds, dt, substeps, contactHertz)
	for range substeps {
		s.integrateVelocities(w.Gravity)
		s.warmStart()
		s.push()
		s.integratePositions(dt)
		s.relax()
	}
	for range restitutionIterations {
		s.restitution()
	}
	s.storeImpulses()
	s.finalize()

	w.contacts = manifolds
	w.indexContacts()

	// Phase 3: Sleep & events
	for _, body := range w.Bodies {
		body.TrySleep(dt, actor.DefaultTimeToSleep, actor.DefaultSleepSpeed)
	}

	w.Events.processSleepEvents(w.Bodies)
	w.Events.flush()
}

// detectCollision: the AABBs are enlarged by the distance the bodies can travel during the step,
// so that the contacts exist before the bodies touch (speculative contacts)
func (w *World) detectCollision(dt float64, workers int) []constraint.Manifold {
	w.aabbs = w.aabbs[:0]
	for _, body := range w.Bodies {
		aabb := body.Shape.GetAABB()
		if _, isPlane := body.Shape.(*actor.Plane); !isPlane {
			margin := reach(body, aabb, dt)
			aabb = actor.AABB{Min: aabb.Min.Sub(mgl64.Vec3{margin, margin, margin}), Max: aabb.Max.Add(mgl64.Vec3{margin, margin, margin})}
		}
		w.aabbs = append(w.aabbs, aabb)
	}

	w.SpatialGrid.Clear()
	for i, body := range w.Bodies {
		w.SpatialGrid.InsertAABB(i, body, w.aabbs[i])
	}
	pairs := w.SpatialGrid.FindPairs(w.Bodies, w.aabbs, workers)

	w.previous = w.contacts
	return narrowPhase(pairs, workers, func(a, b *actor.RigidBody) float64 {
		// triggers only need the real overlaps
		if a.IsTrigger || b.IsTrigger {
			return 0
		}
		return SpeculativeDistance + relativeSpeed(a, b)*dt
	})
}

// reach is the distance a body can travel during dt, plus the speculative distance
func reach(body *actor.RigidBody, aabb actor.AABB, dt float64) float64 {
	if body.BodyType == actor.BodyTypeStatic || body.IsSleeping {
		return SpeculativeDistance
	}
	radius := aabb.Max.Sub(aabb.Min).Len() / 2
	return SpeculativeDistance + (body.Velocity.Len()+body.AngularVelocity.Len()*radius)*dt
}

// relativeSpeed is the maximum speed at which the surfaces of both bodies can get closer
func relativeSpeed(a, b *actor.RigidBody) float64 {
	speed := b.Velocity.Sub(a.Velocity).Len()
	for _, body := range [2]*actor.RigidBody{a, b} {
		if _, isPlane := body.Shape.(*actor.Plane); isPlane {
			continue
		}
		aabb := body.Shape.GetAABB()
		speed += body.AngularVelocity.Len() * aabb.Max.Sub(aabb.Min).Len() / 2
	}
	return speed
}

// warmStart: a contact point takes the impulses of the closest point of the previous step
// (in the local space of body A)
func (w *World) warmStart(manifolds []constraint.Manifold) {
	for i := range manifolds {
		manifold := &manifolds[i]
		k, ok := w.contactsIndex[makePairKey(manifold.BodyA, manifold.BodyB)]
		if !ok {
			continue
		}
		previous := &w.previous[k]
		if previous.BodyA != manifold.BodyA {
			continue
		}

		used := [constraint.MaxContactPoints]bool{}
		for j := 0; j < manifold.Count; j++ {
			local := manifold.BodyA.Transform.ToLocal(manifold.Points[j].Position)
			closest, closestDistance := -1, contactMatchDistance*contactMatchDistance
			for o := 0; o < previous.Count; o++ {
				if used[o] {
					continue
				}
				distance := previous.Points[o].LocalAnchorA.Sub(local).LenSqr()
				if distance <= closestDistance {
					closest, closestDistance = o, distance
				}
			}

			if closest >= 0 {
				used[closest] = true
				manifold.Points[j].NormalImpulse = previous.Points[closest].NormalImpulse
				manifold.Points[j].TangentImpulse = previous.Points[closest].TangentImpulse
			}
		}
	}
}

func (w *World) indexContacts() {
	if w.contactsIndex == nil {
		w.contactsIndex = make(map[pairKey]int)
	}
	clear(w.contactsIndex)
	for i := range w.contacts {
		w.contactsIndex[makePairKey(w.contacts[i].BodyA, w.contacts[i].BodyB)] = i
	}
}

// wakeTouchedBodies: a sleeping body touched by a moving body wakes up,
// otherwise it would be pushed without moving
func (w *World) wakeTouchedBodies() {
	for i := range w.contacts {
		bodyA, bodyB := w.contacts[i].BodyA, w.contacts[i].BodyB
		if bodyA.IsSleeping && isMoving(bodyB) {
			bodyA.WakeUp()
		} else if bodyB.IsSleeping && isMoving(bodyA) {
			bodyB.WakeUp()
		}
	}
}

func isMoving(body *actor.RigidBody) bool {
	return isAwakeDynamic(body) &&
		(body.Velocity.Len() >= actor.DefaultSleepSpeed || body.AngularVelocity.Len() >= actor.DefaultSleepSpeed)
}

// parallelFor calls fn(i) for each i in [0, n), split between the workers.
// Each i writes only its own result, so the order of execution does not matter.
func parallelFor(n, workers int, fn func(i int)) {
	workers = min(workers, n)
	if workers <= 1 {
		for i := 0; i < n; i++ {
			fn(i)
		}
		return
	}

	var wg sync.WaitGroup
	chunkSize := (n + workers - 1) / workers
	for start := 0; start < n; start += chunkSize {
		end := min(start+chunkSize, n)
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := start; i < end; i++ {
				fn(i)
			}
		}()
	}
	wg.Wait()
}
