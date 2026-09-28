package feather

import (
	"runtime"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

const DEFAULT_WORKERS = 1

type World struct {
	// List of all rigid bodies in the world
	Bodies []*actor.RigidBody
	// Joints between the bodies
	Joints []Joint
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

	solver  solver
	islands sleepIslands
	// contacts of the previous step, to warm start the solver
	contacts      []constraint.Manifold
	contactsIndex map[pairKey]contactsRange
	previous      []constraint.Manifold
	aabbs         []actor.AABB
	// pairs of bodies linked by a joint that must not collide
	jointPairs   map[pairKey]int
	solverJoints []Joint
	// 2 buffers: one for the contacts of this step, one for the previous step
	buffers [2][]constraint.Manifold
	buffer  int
	// the manifolds of the pair i are at offsets[i], counts[i] of them
	offsets []int
	counts  []int
	// heightfields changed during this step: their contacts are computed again
	changed []*actor.RigidBody

	// workers of the step, and the parameters of the narrow phase job
	workers    *workersHandle
	pairs      []Pair
	manifolds  []constraint.Manifold
	dt         float64
	collideJob func(i int)
	aabbJob    func(i int)
}

// AddBody adds a rigid body to the world
func (w *World) AddBody(body *actor.RigidBody) {
	w.Bodies = append(w.Bodies, body)
}

// AddJoint adds a joint between 2 bodies, and wakes them up
func (w *World) AddJoint(joint Joint) {
	w.Joints = append(w.Joints, joint)
	base := joint.base()
	w.islands.wake(base.BodyA)
	w.islands.wake(base.BodyB)
	if !base.CollideConnected {
		if w.jointPairs == nil {
			w.jointPairs = make(map[pairKey]int)
		}
		w.jointPairs[makePairKey(base.BodyA, base.BodyB)]++
	}
}

// RemoveJoint removes a joint, and wakes its bodies up
func (w *World) RemoveJoint(joint Joint) {
	for i, other := range w.Joints {
		if other != joint {
			continue
		}
		w.Joints = append(w.Joints[:i], w.Joints[i+1:]...)
		base := joint.base()
		w.islands.wake(base.BodyA)
		w.islands.wake(base.BodyB)
		if !base.CollideConnected {
			key := makePairKey(base.BodyA, base.BodyB)
			if w.jointPairs[key]--; w.jointPairs[key] <= 0 {
				delete(w.jointPairs, key)
			}
		}
		return
	}
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

	// the joints of the body are removed too
	for i := len(w.Joints) - 1; i >= 0; i-- {
		if base := w.Joints[i].base(); base.BodyA == body || base.BodyB == body {
			w.RemoveJoint(w.Joints[i])
		}
	}

	w.Events.forget(body)
	// the bodies touching the removed body wake up (with their islands): they may have to fall.
	// The sleeping bodies have no contact anymore: their AABB is used
	w.islands.remove(body)
	aabb := body.Shape.GetAABB()
	margin := mgl64.Vec3{SpeculativeDistance, SpeculativeDistance, SpeculativeDistance}
	aabb = actor.AABB{Min: aabb.Min.Sub(margin), Max: aabb.Max.Add(margin)}
	for _, other := range w.Bodies {
		if other.IsSleeping && aabb.Overlaps(other.Shape.GetAABB()) {
			w.islands.wake(other)
		}
	}
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

// workersHandle owns the workers of a World. When the World is not used anymore, the handle is collected
// and its finalizer stops the workers (they only reference the pool, not the World)
type workersHandle struct {
	pool *workerPool
}

func (w *World) workerPool() *workerPool {
	if w.workers == nil {
		w.workers = &workersHandle{pool: &workerPool{}}
		runtime.SetFinalizer(w.workers, func(handle *workersHandle) {
			handle.pool.close()
		})
	}
	return w.workers.pool
}

// Close stops the workers of the world. The world can still be used, the workers are created again if needed.
func (w *World) Close() {
	if w.workers != nil {
		w.workers.pool.close()
	}
}

// UpdateHeightfield after a change of the heights or of the holes of the samples [minX, maxX] x [minZ, maxZ]
// of a heightfield body: the terrain is updated, the sleeping bodies above the region wake up,
// and the contacts with the terrain are computed again
func (w *World) UpdateHeightfield(body *actor.RigidBody, minX, minZ, maxX, maxZ int) {
	field := body.Shape.(*actor.Heightfield)
	field.Update(minX, minZ, maxX, maxZ)
	body.Shape.ComputeAABB(body.Transform)
	w.changed = append(w.changed, body)

	// the region in the local space of the terrain, around the changed samples
	halfX, halfZ := float64(field.XSamples-1)/2, float64(field.ZSamples-1)/2
	regionMinX, regionMaxX := (float64(minX-1)-halfX)*field.Scale.X(), (float64(maxX+1)-halfX)*field.Scale.X()
	regionMinZ, regionMaxZ := (float64(minZ-1)-halfZ)*field.Scale.Z(), (float64(maxZ+1)-halfZ)*field.Scale.Z()
	for _, other := range w.Bodies {
		if !other.IsSleeping {
			continue
		}
		bounds := localBounds(body.Transform, other.Shape.GetAABB())
		if bounds.Max.X() >= regionMinX && bounds.Min.X() <= regionMaxX && bounds.Max.Z() >= regionMinZ && bounds.Min.Z() <= regionMaxZ {
			w.islands.wake(other)
		}
	}
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
	pool := w.workerPool()
	if workers > 1 && len(w.Bodies) >= minParallelBodies {
		pool.begin(workers)
	}

	// Phase 1: Collision detection, once per step - broad phase & narrow phase.
	// The buffer of the previous step is kept for the warm start
	w.previous = w.contacts
	w.buffer = 1 - w.buffer
	manifolds := w.detectCollision(dt, pool)
	if w.wakeTouched(manifolds) {
		// the woken bodies get their contacts in this step (as in Jolt)
		manifolds = w.detectCollision(dt, pool)
	}
	w.changed = w.changed[:0]
	manifolds = w.Events.recordCollisions(manifolds)
	w.warmStart(manifolds)

	// Phase 2: Solver, with substeps
	s := &w.solver
	s.joints = w.activeJoints()
	s.prepare(w.Bodies, manifolds, dt, substeps, contactHertz, pool)
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
	pool.end()
	w.continuous(s, dt)

	w.contacts = manifolds
	w.indexContacts()

	// Phase 3: Sleep & events
	w.islands.update(s, dt)

	w.Events.processSleepEvents(w.Bodies)
	w.Events.flush()
}

// detectCollision: the AABBs are enlarged by the distance the bodies can travel during the step, so that the contacts
// exist before the bodies touch (speculative contacts: the speculative CCD of PhysX, the "Continuous Speculative" mode
// of Unity)
func (w *World) detectCollision(dt float64, pool *workerPool) []constraint.Manifold {
	if cap(w.aabbs) < len(w.Bodies) {
		w.aabbs = make([]actor.AABB, len(w.Bodies))
	}
	w.aabbs = w.aabbs[:len(w.Bodies)]
	w.dt = dt
	if w.aabbJob == nil {
		w.aabbJob = w.computeAABB
	}
	pool.run(len(w.Bodies), bodiesChunk, w.aabbJob)

	w.SpatialGrid.Clear()
	for i, body := range w.Bodies {
		w.SpatialGrid.InsertAABB(i, body, w.aabbs[i])
	}
	w.pairs = w.SpatialGrid.findPairsPool(w.Bodies, w.aabbs, pool)

	// Narrow phase, in a buffer reused every 2 steps (the previous step is needed for the warm start).
	// Each pair has its own place: 1 manifold, MaxManifoldsPerPair against a heightfield
	if cap(w.offsets) < len(w.pairs)+1 {
		w.offsets = make([]int, len(w.pairs)+1)
		w.counts = make([]int, len(w.pairs))
	}
	w.offsets, w.counts = w.offsets[:len(w.pairs)+1], w.counts[:len(w.pairs)]
	for i, pair := range w.pairs {
		w.offsets[i+1] = w.offsets[i] + manifoldsOf(pair)
	}
	total := w.offsets[len(w.pairs)]
	if cap(w.buffers[w.buffer]) < total {
		w.buffers[w.buffer] = make([]constraint.Manifold, total)
	}
	w.manifolds = w.buffers[w.buffer][:total]
	if w.collideJob == nil {
		w.collideJob = w.collide
	}
	pool.run(len(w.pairs), pairsPerChunk, w.collideJob)

	return compactManifolds(w.manifolds, w.offsets, w.counts)
}

// wakeTouched: a sleeping body touched by an awake dynamic body wakes up with its island (as in Box2D & Jolt).
// Returns true if a body woke up: its contacts must be found in this step
func (w *World) wakeTouched(manifolds []constraint.Manifold) bool {
	woke := false
	for i := range manifolds {
		a, b := manifolds[i].BodyA, manifolds[i].BodyB
		if a.IsSleeping && isAwakeDynamic(b) {
			w.islands.wake(a)
			woke = true
		} else if b.IsSleeping && isAwakeDynamic(a) {
			w.islands.wake(b)
			woke = true
		}
	}
	return woke
}

// activeJoints: the joints with at least one awake dynamic body
func (w *World) activeJoints() []Joint {
	w.solverJoints = w.solverJoints[:0]
	for _, joint := range w.Joints {
		base := joint.base()
		if isAwakeDynamic(base.BodyA) || isAwakeDynamic(base.BodyB) {
			w.solverJoints = append(w.solverJoints, joint)
		}
	}
	return w.solverJoints
}

// computeAABB of the body i, enlarged by the distance it can travel during the step
func (w *World) computeAABB(i int) {
	body := w.Bodies[i]
	aabb := body.Shape.GetAABB()
	if _, isPlane := body.Shape.(*actor.Plane); !isPlane {
		margin := reach(body, aabb, w.dt)
		aabb = actor.AABB{Min: aabb.Min.Sub(mgl64.Vec3{margin, margin, margin}), Max: aabb.Max.Add(mgl64.Vec3{margin, margin, margin})}
	}
	w.aabbs[i] = aabb
}

// collide the pair i. The triggers only need the real overlaps, the other pairs get speculative contacts
func (w *World) collide(i int) {
	pair := w.pairs[i]
	out := w.manifolds[w.offsets[i]:w.offsets[i+1]]
	if w.jointPairs[makePairKey(pair.BodyA, pair.BodyB)] > 0 {
		w.counts[i] = 0
		return
	}
	margin := 0.0
	if !pair.BodyA.IsTrigger && !pair.BodyB.IsTrigger {
		margin = SpeculativeDistance + relativeSpeed(pair.BodyA, pair.BodyB)*w.dt

		// pair cache: the contacts of the previous step, if the bodies barely moved relative to each other
		if r, ok := w.contactsIndex[makePairKey(pair.BodyA, pair.BodyB)]; ok && w.previous[r.first].BodyA == pair.BodyA && !w.isChanged(pair) {
			count := 0
			for k := r.first; k < r.first+r.count && count < len(out); k++ {
				if reuseManifold(&w.previous[k], margin, &out[count]) {
					count++
				}
			}
			if count > 0 {
				w.counts[i] = count
				return
			}
		}
	}
	w.counts[i] = collidePair(pair, margin, out)
}

// isChanged: a body of the pair is a heightfield changed during this step
func (w *World) isChanged(pair Pair) bool {
	for _, body := range w.changed {
		if body == pair.BodyA || body == pair.BodyB {
			return true
		}
	}
	return false
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
// (in the local space of body A), among the manifolds of the same pair
func (w *World) warmStart(manifolds []constraint.Manifold) {
	for i := range manifolds {
		manifold := &manifolds[i]
		r, ok := w.contactsIndex[makePairKey(manifold.BodyA, manifold.BodyB)]
		if !ok || w.previous[r.first].BodyA != manifold.BodyA {
			continue
		}

		used := [MaxManifoldsPerPair][constraint.MaxContactPoints]bool{}
		for j := 0; j < manifold.Count; j++ {
			local := manifold.Points[j].LocalAnchorA
			closestManifold, closest, closestDistance := -1, -1, contactMatchDistance*contactMatchDistance
			for k := 0; k < r.count; k++ {
				previous := &w.previous[r.first+k]
				for o := 0; o < previous.Count; o++ {
					if used[k][o] {
						continue
					}
					distance := previous.Points[o].LocalAnchorA.Sub(local).LenSqr()
					if distance <= closestDistance {
						closestManifold, closest, closestDistance = k, o, distance
					}
				}
			}

			if closest >= 0 {
				used[closestManifold][closest] = true
				point := &w.previous[r.first+closestManifold].Points[closest]
				manifold.Points[j].NormalImpulse = point.NormalImpulse
				manifold.Points[j].TangentImpulse = point.TangentImpulse
			}
		}
	}
}

// contactsRange: the manifolds of a pair follow each other in the contacts
type contactsRange struct {
	first int
	count int
}

func (w *World) indexContacts() {
	if w.contactsIndex == nil {
		w.contactsIndex = make(map[pairKey]contactsRange)
	}
	clear(w.contactsIndex)
	for i := range w.contacts {
		key := makePairKey(w.contacts[i].BodyA, w.contacts[i].BodyB)
		r, ok := w.contactsIndex[key]
		if !ok {
			r.first = i
		}
		r.count++
		w.contactsIndex[key] = r
	}
}

// wakeTouchedBodies: a sleeping body touched by a moving body wakes up with its island,
// otherwise it would be pushed without moving
func (w *World) wakeTouchedBodies() {
	w.islands.wakeWoken()
	for _, joint := range w.Joints {
		base := joint.base()
		if base.BodyA.IsSleeping && isAwakeDynamic(base.BodyB) {
			w.islands.wake(base.BodyA)
		} else if base.BodyB.IsSleeping && isAwakeDynamic(base.BodyA) {
			w.islands.wake(base.BodyB)
		}
	}
	for i := range w.contacts {
		bodyA, bodyB := w.contacts[i].BodyA, w.contacts[i].BodyB
		if bodyA.IsSleeping && isMoving(bodyB) {
			w.islands.wake(bodyA)
		} else if bodyB.IsSleeping && isMoving(bodyA) {
			w.islands.wake(bodyB)
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
