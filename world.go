package feather

import (
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// DefaultWorkers: the collision detection and the solver run on one goroutine by default
const DefaultWorkers = 1

const ()

type World struct {
	// List of all rigid bodies in the world
	Bodies []*actor.RigidBody
	// Joints between the bodies
	Joints []Joint
	// Gravity acceleration (m/s², or N/kg)
	Gravity  mgl64.Vec3
	Substeps int
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
	contacts []constraint.Manifold
	previous []constraint.Manifold
	aabbs    []actor.AABB
	// fast: the body i can move more than half of its smallest extent during the step (isFast): its pairs get the
	// speculative margin of its speed
	fast []bool
	// step: the count of steps, the stamp of the contacts kept by the pairs of the broad phase
	step  uint32
	shift []int32 // buffer of RemoveBody
	// the broad phase
	tree Tree
	// pairs of bodies which never collide: ignored, or linked by a joint
	filter       pairFilter
	solverJoints []Joint
	// 2 buffers: one for the contacts of this step, one for the previous step
	buffers [2][]constraint.Manifold
	buffer  int
	// the manifolds of the pair i are at offsets[i], counts[i] of them
	offsets []int
	counts  []int
	// heightfields changed during this step: their contacts are computed again
	changed []*actor.RigidBody

	// profile of the last step
	profile Profile

	// stepping: a step is running, the queries panic (query.go)
	stepping atomic.Bool

	// parallelFrom: the step runs on several goroutines from this count of bodies (minParallelBodies if 0). The tests
	// lower it, to run the parallel paths on small scenes
	parallelFrom int

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
	base.filtering = !base.CollideConnected
	if base.filtering {
		w.filter.link(base.BodyA, base.BodyB, 1)
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
		if base.filtering {
			w.filter.link(base.BodyA, base.BodyB, -1)
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
		w.tree.removed(k)
	}

	// the joints of the body are removed too
	for i := len(w.Joints) - 1; i >= 0; i-- {
		if base := w.Joints[i].base(); base.BodyA == body || base.BodyB == body {
			w.RemoveJoint(w.Joints[i])
		}
	}

	w.Events.forget(body)
	// the bodies touching the removed body wake up (with their islands): they may have to fall
	w.islands.remove(body)
	w.wakeNeighbors(body)
	w.filter.forget(body)
	// the contacts of the body leave: the contacts kept by the other pairs move down
	n := 0
	for i := range w.contacts {
		w.shift = append(w.shift, int32(i-n))
		contact := &w.contacts[i]
		if contact.BodyA != body && contact.BodyB != body {
			w.contacts[n] = *contact
			n++
		}
	}
	w.contacts = w.contacts[:n]
	w.tree.shiftContacts(w.shift, w.step)
	w.shift = w.shift[:0]
}

// wakeNeighbors: the sleeping bodies which collide with the body wake up, with their islands. The sleeping bodies have
// no contact anymore: their AABB is used
func (w *World) wakeNeighbors(body *actor.RigidBody) {
	aabb := body.AABB()
	margin := mgl64.Vec3{SpeculativeDistance, SpeculativeDistance, SpeculativeDistance}
	aabb = actor.AABB{Min: aabb.Min.Sub(margin), Max: aabb.Max.Add(margin)}
	for _, other := range w.Bodies {
		if other.IsSleeping && aabb.Overlaps(other.AABB()) && w.ShouldCollide(body, other) {
			w.islands.wake(other)
		}
	}
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
	body.UpdateAABB()
	w.changed = append(w.changed, body)

	// the region in the local space of the terrain, around the changed samples
	halfX, halfZ := float64(field.XSamples-1)/2, float64(field.ZSamples-1)/2
	regionMinX, regionMaxX := (float64(minX-1)-halfX)*field.Scale.X(), (float64(maxX+1)-halfX)*field.Scale.X()
	regionMinZ, regionMaxZ := (float64(minZ-1)-halfZ)*field.Scale.Z(), (float64(maxZ+1)-halfZ)*field.Scale.Z()
	for _, other := range w.Bodies {
		if !other.IsSleeping || !w.ShouldCollide(body, other) {
			continue
		}
		bounds := localBounds(body.Transform, other.AABB())
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
	w.stepping.Store(true)
	workers := max(DefaultWorkers, w.Workers)
	substeps := max(1, w.Substeps)
	contactHertz := w.ContactHertz
	if contactHertz <= 0 {
		contactHertz = DefaultContactHertz
	}

	start := time.Now()
	w.profile = Profile{}
	w.moveKinematics(dt)
	w.wakeTouchedBodies()
	pool := w.workerPool()
	parallelFrom := w.parallelFrom
	if parallelFrom <= 0 {
		parallelFrom = minParallelBodies
	}
	if workers > 1 && len(w.Bodies) >= parallelFrom {
		pool.begin(workers)
	}

	// Phase 1: Collision detection, once per step - broad phase & narrow phase.
	// The buffer of the previous step is kept for the warm start
	w.previous = w.contacts
	w.buffer = 1 - w.buffer
	w.step++
	manifolds := w.detectCollision(dt, pool)
	if w.wakeTouched(manifolds) {
		// the woken bodies get their contacts in this step (as in Jolt)
		manifolds = w.detectCollision(dt, pool)
	}
	mark := time.Now()
	w.changed = w.changed[:0]
	manifolds = w.Events.recordCollisions(manifolds)

	// Phase 2: Solver, with substeps
	s := &w.solver
	s.joints = w.activeJoints()
	s.prepare(w.Bodies, manifolds, dt, substeps, contactHertz, pool)
	mark = w.lap(&w.profile.Prepare, mark)
	for k := range substeps {
		s.substep = k + 1
		s.integrateVelocities(w.Gravity)
		s.warmStart()
		s.push()
		s.integratePositions(dt)
		s.relax()
	}
	mark = w.lap(&w.profile.Substeps, mark)
	s.restitution()
	s.storeImpulses()
	s.finalize()
	pool.end()
	mark = w.lap(&w.profile.Restitution, mark)
	w.continuous(s, manifolds, dt)

	w.contacts = manifolds
	w.recordContacts()
	mark = w.lap(&w.profile.Continuous, mark)

	// Phase 3: Sleep & events
	w.islands.update(s, dt)
	mark = w.lap(&w.profile.Islands, mark)

	// the trees follow the bodies moved by the step, for the queries: the events are sent after, their listeners can
	// run queries
	w.syncMoved()
	mark = w.lap(&w.profile.BroadPhase, mark)
	w.stepping.Store(false)

	w.Events.processSleepEvents(w.Bodies)
	w.Events.flush()
	w.lap(&w.profile.Islands, mark)
	w.profile.Step = time.Since(start)
}

// lap adds the time since mark to the phase, and returns now
func (w *World) lap(phase *time.Duration, mark time.Time) time.Time {
	now := time.Now()
	*phase += now.Sub(mark)
	return now
}

// detectCollision: the AABBs are enlarged by the distance the bodies can travel during the step, so that the contacts
// with the static bodies exist before the bodies touch (speculative contacts: the speculative CCD of PhysX, the
// "Continuous Speculative" mode of Unity)
func (w *World) detectCollision(dt float64, pool *workerPool) []constraint.Manifold {
	w.dt = dt
	mark := time.Now()
	w.syncTrees(pool)
	w.pairs = w.tree.findPairs(w.Bodies, w.aabbs, pool)
	mark = w.lap(&w.profile.BroadPhase, mark)

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

	manifolds := compactManifolds(w.manifolds, w.offsets, w.counts)
	w.lap(&w.profile.NarrowPhase, mark)
	return manifolds
}

// syncTrees: the trees of the broad phase take the bodies of the world at their AABBs, enlarged by the distance they
// can travel during a step of w.dt. It runs at the start of a step, for its pairs, and in SyncQueries, for the queries:
// Feather doesn't know what the game wrote, every body is tested. With syncMoved at the end of a step, all three give
// the same AABBs for the bodies which didn't change, so that a body is put back in its tree once (Tree.update compares
// the AABB of a static body bit for bit)
func (w *World) syncTrees(pool *workerPool) {
	if cap(w.aabbs) < len(w.Bodies) {
		w.aabbs = make([]actor.AABB, len(w.Bodies))
		w.fast = make([]bool, len(w.Bodies))
	}
	w.aabbs, w.fast = w.aabbs[:len(w.Bodies)], w.fast[:len(w.Bodies)]
	if w.aabbJob == nil {
		w.aabbJob = w.computeAABB
	}
	pool.run(len(w.Bodies), bodiesChunk, w.aabbJob)

	w.tree.filter = &w.filter
	w.tree.sync(w.Bodies, w.aabbs)
}

// syncMoved: at the end of a step, the trees take the bodies the step moved, at the AABBs of syncTrees: the bodies of
// the solver, awake when the step started or woken by it (the continuous collision and the sleep only write these).
// The static and the sleeping bodies are not read: a world asleep pays nothing. As the last stage of a step of Box2D
// v3, which only enlarges the AABBs of the bodies which left them ("for shapes that have moved significantly",
// docs/simulation.md)
func (w *World) syncMoved() {
	for _, i := range w.solver.stateBody {
		w.computeAABB(int(i))
		w.tree.update(i, w.Bodies[i], w.aabbs[i])
	}
}

// wakeTouched: a sleeping body touched by an awake dynamic or kinematic body wakes up with its island (as in Box2D &
// Jolt). Returns true if a body woke up: its contacts must be found in this step
func (w *World) wakeTouched(manifolds []constraint.Manifold) bool {
	woke := false
	for i := range manifolds {
		a, b := manifolds[i].BodyA, manifolds[i].BodyB
		if a.IsSleeping && isAwakeMover(b) {
			w.islands.wake(a)
			woke = true
		} else if b.IsSleeping && isAwakeMover(a) {
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

// computeAABB of the body i, enlarged by the distance it can travel during the step; and whether the body is fast
func (w *World) computeAABB(i int) {
	body := w.Bodies[i]
	w.fast[i] = isFast(body, w.dt)
	aabb := body.AABB()
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
	margin := 0.0
	if !pair.BodyA.IsTrigger && !pair.BodyB.IsTrigger {
		// against a static or a kinematic body, the contact exists before the body touches it, from their relative speed
		// (the speculative CCD of PhysX, legal on its kinematic actors): it doesn't sink into the ground, a leg at 5 m/s
		// doesn't enter the tail it meets. The pairs of a fast dynamic body (isFast) get the same margin: 2 fast spheres
		// don't go through each other. Between 2 dynamic bodies which are not fast, only within SpeculativeDistance (as in
		// Box2D v3): an impact is then absorbed by the spring of the contact over a few substeps, a rigid stop in one
		// substep would throw the lighter body and turn both
		margin = SpeculativeDistance
		if pair.BodyA.BodyType != actor.BodyTypeDynamic || pair.BodyB.BodyType != actor.BodyTypeDynamic ||
			w.fast[pair.IndexA] || w.fast[pair.IndexB] {
			margin += relativeSpeed(pair.BodyA, pair.BodyB) * w.dt
		}

		// pair cache: the contacts of the previous step, if the bodies barely moved relative to each other
		previous := w.previousContacts(pair)
		if len(previous) > 0 && !w.isChanged(pair) {
			count := 0
			for k := range previous {
				if count < len(out) && reuseManifold(&previous[k], margin, &out[count]) {
					count++
				}
			}
			if count > 0 {
				w.counts[i] = count
				indexManifolds(out[:count], pair)
				warmStartPair(out[:count], previous)
				return
			}
		}
		w.counts[i] = collidePair(pair, margin, out)
		indexManifolds(out[:w.counts[i]], pair)
		warmStartPair(out[:w.counts[i]], previous)
		return
	}
	w.counts[i] = collidePair(pair, margin, out)
	indexManifolds(out[:w.counts[i]], pair)
}

// previousContacts of the pair: the manifolds it had in the previous step (none if it had none, or if the bodies
// changed)
func (w *World) previousContacts(pair Pair) []constraint.Manifold {
	record := &w.tree.fat[pair.slot]
	if record.stamp != w.step-1 || record.count == 0 || w.previous[record.first].BodyA != pair.BodyA {
		return nil
	}
	return w.previous[record.first : record.first+record.count]
}

// recordContacts: each pair keeps where its contacts of this step are, for the next step. The contacts with a trigger
// are not kept (they are not in the contacts)
func (w *World) recordContacts() {
	first := int32(0)
	for i := range w.pairs {
		pair := &w.pairs[i]
		count := int32(w.counts[i])
		if pair.BodyA.IsTrigger || pair.BodyB.IsTrigger {
			count = 0
		}
		record := &w.tree.fat[pair.slot]
		record.first, record.count, record.stamp = first, count, w.step
		first += count
	}
}

// indexManifolds: the manifolds of the pair carry the indices of its bodies, for the solver
func indexManifolds(manifolds []constraint.Manifold, pair Pair) {
	for k := range manifolds {
		manifolds[k].IndexA, manifolds[k].IndexB = pair.IndexA, pair.IndexB
	}
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

// reach is the distance a body can travel during dt, plus the speculative distance. A kinematic body travels at the
// velocity of the motion to its target
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
		aabb := body.AABB()
		speed += body.AngularVelocity.Len() * aabb.Max.Sub(aabb.Min).Len() / 2
	}
	return speed
}

// warmStartPair: a contact point takes the impulses of the closest point of the previous step (in the local space of
// body A, as the contact cache of Jolt matches its points; Box2D matches them by feature id), among the previous
// manifolds of the same pair. The contact takes the friction, twist, rolling & spinning impulses of the previous
// contact of its first matched point
func warmStartPair(manifolds, previous []constraint.Manifold) {
	if len(previous) == 0 {
		return
	}
	for i := range manifolds {
		manifold := &manifolds[i]
		used := [MaxManifoldsPerPair][constraint.MaxContactPoints]bool{}
		source := -1
		for j := 0; j < manifold.Count; j++ {
			local := manifold.Points[j].LocalAnchorA
			closestManifold, closest, closestDistance := -1, -1, contactMatchDistance*contactMatchDistance
			for k := range previous {
				for o := 0; o < previous[k].Count; o++ {
					if used[k][o] {
						continue
					}
					distance := previous[k].Points[o].LocalAnchorA.Sub(local).LenSqr()
					if distance <= closestDistance {
						closestManifold, closest, closestDistance = k, o, distance
					}
				}
			}

			if closest >= 0 {
				used[closestManifold][closest] = true
				point := &previous[closestManifold].Points[closest]
				manifold.Points[j].NormalImpulse = point.NormalImpulse
				manifold.Points[j].ImpactVelocity, manifold.Points[j].CompressionImpulse = point.ImpactVelocity, point.CompressionImpulse
				if source < 0 {
					source = closestManifold
				}
			}
		}
		// the impulses of the whole contact come from the previous contact of its first point
		if source >= 0 {
			manifold.FrictionImpulse, manifold.TwistImpulse = previous[source].FrictionImpulse, previous[source].TwistImpulse
			manifold.RollingImpulse, manifold.SpinningImpulse = previous[source].RollingImpulse, previous[source].SpinningImpulse
		}
	}
}

// wakeTouchedBodies: a sleeping body touched by a moving body (dynamic, or a kinematic body on its way to a target)
// wakes up with its island, otherwise it would be pushed without moving
func (w *World) wakeTouchedBodies() {
	w.islands.wakeWoken()
	for _, joint := range w.Joints {
		base := joint.base()
		if base.BodyA.IsSleeping && (isAwakeDynamic(base.BodyB) || isMoving(base.BodyB)) {
			w.islands.wake(base.BodyA)
		} else if base.BodyB.IsSleeping && (isAwakeDynamic(base.BodyA) || isMoving(base.BodyA)) {
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
