package feather

import (
	"math"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== CONTINUOUS COLLISION ==========
// The speculative contacts stop most of the fast bodies. They can miss a body accelerated by the solver during the step:
// after the solver, a fast body is moved back to its first impact with any body, found along its motion (the time of
// impact of Box2D v3, against every body as the LinearCast motion quality of Jolt). Its velocity is kept: the contact of
// the next step stops it. A body is only stopped by the bodies it collides with (World.ShouldCollide, as the continuous
// collision of Box2D skips the filtered shapes and bodies).
//
// A static or a kinematic body, and a dynamic body which is not fast, are where the step left them: the fast body is
// swept against their final pose (Box2D, Jolt: "Body has already moved"). Two fast bodies are swept once, by their
// relative motion against the start pose of the other (Jolt: the cast of a body against a body which casts too takes
// "the relative movement of these two bodies", PhysicsSystem.cpp:1884), and both are stopped at the fraction found
// (Jolt: "the other body will shorten its distance traveled", :2054-2060). The fractions are all found before a body
// moves: the result doesn't depend on the order of the bodies.

const (
	// continuousSafetyFactor: a body is fast when it moves more than half of its smallest extent during a step (Box2D)
	continuousSafetyFactor = 0.5

	// toiTarget: the moving body is stopped at this distance from the other body (m)
	toiTarget = LinearSlop

	// toiTolerance around the target (m)
	toiTolerance = 0.25 * LinearSlop

	// toiIterations of the conservative advancement
	toiIterations = 32

	// coreFraction: if the body already touches the other body at the start, only its core (a sphere of this fraction of
	// its smallest extent, at its center) is stopped (B2_CORE_FRACTION of Box2D main, after v3.1)
	coreFraction = 0.25
)

// sweep: the motion of a body during the step
type sweep struct {
	start actor.Transform
	end   actor.Transform
	// angle of the rotation from start to end (rad)
	angle float64
}

// at: the transform at the fraction t of the motion (position lerp, rotation nlerp)
func (s *sweep) at(t float64) actor.Transform {
	rotation := s.end.Rotation
	if s.start.Rotation.Dot(rotation) < 0 {
		rotation = rotation.Scale(-1)
	}
	return actor.Transform{
		Position: s.start.Position.Add(s.end.Position.Sub(s.start.Position).Mul(t)),
		Rotation: s.start.Rotation.Scale(1 - t).Add(rotation.Scale(t)).Normalize(),
	}
}

// fastBody: a body which moved more than half of its smallest extent during the step, to be stopped at its first impact
type fastBody struct {
	// index of its state in the solver
	state  int32
	motion sweep
	// the distance of the farthest point of the body from its center (m), and the fraction of its motion kept (1: all)
	radius   float64
	fraction float64
}

// ccdScratch: the buffers of the continuous collision, reused to avoid the allocations
type ccdScratch struct {
	stack      []int32
	candidates []int32
	cells      []int32
	triangles  []int32
	shape      triangleShape
	core       actor.Sphere
	// fast: the fast bodies of the step, in the order of the states; fastOf[state] is the index of a state in fast, -1
	// if the body is not fast
	fast   []fastBody
	fastOf []int32
	// the bodies in contact with each fast body this step: partners[offsets[k]:offsets[k+1]] for the fast body k;
	// touched[body] is k+1 while the fast body k is swept
	offsets  []int32
	partners []int32
	touched  []int32
}

var ccdPool = sync.Pool{New: func() any { return &ccdScratch{} }}

// continuous collision of the fast bodies: their fractions are all found first, against the bodies where the step left
// them (the start pose of another fast body), then the bodies are moved. The result doesn't depend on the order of the
// bodies. The manifolds are the contacts of the step
func (w *World) continuous(s *solver, manifolds []constraint.Manifold, dt float64) {
	scratch := ccdPool.Get().(*ccdScratch)
	defer ccdPool.Put(scratch)
	scratch.fast = scratch.fast[:0]
	if cap(scratch.fastOf) < len(s.states) {
		scratch.fastOf = make([]int32, len(s.states))
	}
	scratch.fastOf = scratch.fastOf[:len(s.states)]
	for i := range s.states {
		scratch.fastOf[i] = -1
		state := &s.states[i]
		body := state.body
		// a kinematic body is never stopped ("Kinematic bodies cannot be stopped", Jolt PhysicsSystem.cpp:1564)
		if !state.dynamic || body.IsTrigger {
			continue
		}
		minExtent, maxExtent := shapeExtents(body.Shape)
		motion := sweep{
			start: actor.Transform{Position: body.Transform.Position.Sub(state.deltaPosition), Rotation: s.starts[i].rotation},
			end:   body.Transform,
		}
		motion.angle = rotationAngle(state.deltaRotation)
		// the farthest point of the body moves at most by the translation + the rotation * its extent
		if state.deltaPosition.Len()+motion.angle*maxExtent <= continuousSafetyFactor*minExtent {
			continue
		}
		scratch.fastOf[i] = int32(len(scratch.fast))
		scratch.fast = append(scratch.fast, fastBody{state: int32(i), motion: motion, radius: maxExtent, fraction: 1})
	}
	if len(scratch.fast) == 0 {
		return
	}
	w.contactPartners(s, manifolds, scratch)
	for k := range scratch.fast {
		fast := &scratch.fast[k]
		scratch.core.Radius = coreFraction * shapeMinExtent(s.states[fast.state].body.Shape)
		w.findImpact(s, k, scratch)
	}
	for k := range scratch.fast {
		fast := &scratch.fast[k]
		if fast.fraction < 1 {
			body := s.states[fast.state].body
			body.Transform = fast.motion.at(fast.fraction)
			if body.AngularLock == actor.AllAxes {
				// it didn't turn: the interpolation of 2 equal rotations is not the rotation bit for bit
				body.Transform.Rotation = fast.motion.end.Rotation
			}
			body.UpdateAABB()
		}
	}
}

// contactPartners lists, for each fast body, the bodies it has a contact with this step (the manifolds of the pair, found
// by the narrow phase up to the speculative margin)
func (w *World) contactPartners(s *solver, manifolds []constraint.Manifold, scratch *ccdScratch) {
	if cap(scratch.offsets) < len(scratch.fast)+1 {
		scratch.offsets = make([]int32, len(scratch.fast)+1)
	}
	scratch.offsets = scratch.offsets[:len(scratch.fast)+1]
	clear(scratch.offsets)
	fastOfBody := func(index int32) int32 {
		if state := s.stateIndex[index]; state >= 0 {
			return scratch.fastOf[state]
		}
		return -1
	}
	// the count of partners of each fast body, then the offsets
	for i := range manifolds {
		m := &manifolds[i]
		if k := fastOfBody(m.IndexA); k >= 0 {
			scratch.offsets[k+1]++
		}
		if k := fastOfBody(m.IndexB); k >= 0 {
			scratch.offsets[k+1]++
		}
	}
	for k := 1; k < len(scratch.offsets); k++ {
		scratch.offsets[k] += scratch.offsets[k-1]
	}
	total := int(scratch.offsets[len(scratch.offsets)-1])
	if cap(scratch.partners) < total {
		scratch.partners = make([]int32, total)
	}
	scratch.partners = scratch.partners[:total]
	if cap(scratch.touched) < len(w.Bodies) {
		scratch.touched = make([]int32, len(w.Bodies))
	}
	scratch.touched = scratch.touched[:len(w.Bodies)]
	clear(scratch.touched)
	// the partners, each fast body filling its range from its offset
	for i := range manifolds {
		m := &manifolds[i]
		if k := fastOfBody(m.IndexA); k >= 0 {
			scratch.partners[scratch.offsets[k]] = m.IndexB
			scratch.offsets[k]++
		}
		if k := fastOfBody(m.IndexB); k >= 0 {
			scratch.partners[scratch.offsets[k]] = m.IndexA
			scratch.offsets[k]++
		}
	}
	// the offsets are back to the start of each range
	for k := len(scratch.offsets) - 1; k > 0; k-- {
		scratch.offsets[k] = scratch.offsets[k-1]
	}
	scratch.offsets[0] = 0
}

// findImpact finds the first impact of the fast body k during its motion, with every body it collides with, and
// shortens its fraction (and the one of another fast body it meets). A dynamic or a kinematic body the fast body has
// a contact with this step is left to the solver: the contact already holds them apart, and the sweep would undo the
// overlap the soft contact allows
func (w *World) findImpact(s *solver, k int, scratch *ccdScratch) {
	fast := &scratch.fast[k]
	body := s.states[fast.state].body
	motion := &fast.motion
	// the bodies around the motion
	swept := sweptAABB(body.Shape, motion)
	scratch.stack, scratch.candidates = w.tree.queryCandidates(swept, scratch.stack, scratch.candidates[:0])
	for _, partner := range scratch.partners[scratch.offsets[k]:scratch.offsets[k+1]] {
		scratch.touched[partner] = int32(k + 1)
	}

	for _, index := range scratch.candidates {
		other := w.Bodies[index]
		if other == body || other.IsTrigger || !w.ShouldCollide(body, other) {
			continue
		}
		if other.BodyType != actor.BodyTypeStatic && scratch.touched[index] == int32(k+1) {
			continue
		}
		otherFast := -1
		if state := s.stateIndex[index]; state >= 0 {
			otherFast = int(scratch.fastOf[state])
		}
		if otherFast >= 0 {
			// 2 fast bodies are swept once, by the one with the smallest index
			if otherFast < k {
				continue
			}
			w.fastPairImpact(s, k, otherFast, scratch)
			continue
		}
		if !swept.Overlaps(other.AABB()) {
			continue
		}
		switch shape := other.Shape.(type) {
		case *actor.Plane:
			fast.fraction = math.Min(fast.fraction, impact(body.Shape, motion, fast.radius, nil, shape, fast.fraction, scratch))
		case *actor.Heightfield, *actor.TriangleMesh:
			fast.fraction = math.Min(fast.fraction, surfaceImpact(body.Shape, motion, fast.radius, other, swept, fast.fraction, scratch))
		default:
			proxy := gjk.NewProxy(other)
			fast.fraction = math.Min(fast.fraction, impact(body.Shape, motion, fast.radius, &proxy, nil, fast.fraction, scratch))
		}
	}
}

// fastPairImpact: the first impact of 2 fast bodies, found by sweeping the body k with their relative motion against
// the start pose of the body j, as Jolt (PhysicsSystem.cpp:1884). The fraction is given to both, so that both are
// stopped where they meet (Jolt, :2054-2060)
func (w *World) fastPairImpact(s *solver, k, j int, scratch *ccdScratch) {
	fast, other := &scratch.fast[k], &scratch.fast[j]
	bodyA, bodyB := s.states[fast.state].body, s.states[other.state].body
	relative := sweep{
		start: fast.motion.start,
		end: actor.Transform{
			Position: fast.motion.end.Position.Sub(other.motion.end.Position.Sub(other.motion.start.Position)),
			Rotation: fast.motion.end.Rotation,
		},
		angle: fast.motion.angle,
	}
	if !sweptAABB(bodyA.Shape, &relative).Overlaps(bodyB.Shape.ComputeAABB(other.motion.start)) {
		return
	}
	proxy := gjk.NewProxyAt(other.motion.start, bodyB.Shape)
	maxFraction := math.Max(fast.fraction, other.fraction)
	fraction := impact(bodyA.Shape, &relative, fast.radius, &proxy, nil, maxFraction, scratch)
	fast.fraction = math.Min(fast.fraction, fraction)
	other.fraction = math.Min(other.fraction, fraction)
}

// sweptAABB: the AABB of the shape at the start and at the end of the motion
func sweptAABB(shape actor.ShapeInterface, motion *sweep) actor.AABB {
	swept := shape.ComputeAABB(motion.start)
	end := shape.ComputeAABB(motion.end)
	for k := 0; k < 3; k++ {
		swept.Min[k] = math.Min(swept.Min[k], end.Min[k])
		swept.Max[k] = math.Max(swept.Max[k], end.Max[k])
	}
	return swept
}

// impact: the fraction of the motion at the first impact with the convex shape or the plane, 1 if there is none.
// If the body already touches at the start, its core is used (as in Box2D): a body resting on the ground is not stopped,
// a body going through is
func impact(shape actor.ShapeInterface, motion *sweep, radius float64, other *gjk.Proxy, plane *actor.Plane, maxFraction float64, scratch *ccdScratch) float64 {
	t := timeOfImpact(shape, motion, radius, other, plane, maxFraction)
	if t == 0 {
		t = timeOfImpact(&scratch.core, motion, scratch.core.Radius, other, plane, maxFraction)
		if t == 0 {
			return 1
		}
	}
	return t
}

// timeOfImpact: the fraction of the motion where the moving shape gets to toiTarget from the static shape (or plane),
// found by conservative advancement (Mirtich 1996, as in Bullet): at each iteration, the shape moves forward by the
// distance divided by the fastest approach speed of its points (translation along the normal + rotation * radius),
// so it never goes through. Returns 1 if there is no impact before maxFraction, 0 if the shapes touch at the start
func timeOfImpact(shape actor.ShapeInterface, motion *sweep, radius float64, other *gjk.Proxy, plane *actor.Plane, maxFraction float64) float64 {
	translation := motion.end.Position.Sub(motion.start.Position)
	t := 0.0
	for i := 0; i < toiIterations; i++ {
		transform := motion.at(t)
		var distance float64
		var normal mgl64.Vec3
		if plane != nil {
			lowest := transform.ToWorld(shape.Support(transform.Rotation.Conjugate().Rotate(plane.Normal.Mul(-1))))
			distance, normal = lowest.Dot(plane.Normal)+plane.Distance, plane.Normal.Mul(-1)
		} else {
			proxy := gjk.NewProxyAt(transform, shape)
			result := gjk.Distance(&proxy, other)
			distance, normal = result.Distance, result.Normal
			if result.Overlap {
				distance = 0
			}
		}
		if distance <= toiTarget+toiTolerance {
			return t
		}
		approach := translation.Dot(normal) + motion.angle*radius
		if approach <= 0 {
			return 1
		}
		t += (distance - toiTarget) / approach
		if t >= maxFraction {
			return 1
		}
	}
	return t
}

// surfaceImpact: the first impact with the triangles of the surface (a heightfield, a mesh) under the motion
func surfaceImpact(shape actor.ShapeInterface, motion *sweep, radius float64, surface *actor.RigidBody, swept actor.AABB, maxFraction float64, scratch *ccdScratch) float64 {
	local := localBounds(surface.Transform, swept)
	fraction := maxFraction
	switch other := surface.Shape.(type) {
	case *actor.Heightfield:
		scratch.cells = other.OverlapCells(local, scratch.cells[:0])
		cellsZ := other.ZSamples - 1
		for _, cell := range scratch.cells {
			x, z := int(cell)/cellsZ, int(cell)%cellsZ
			for t := 0; t < 2; t++ {
				vertices, _ := other.Triangle(x, z, t)
				fraction = triangleImpact(shape, motion, radius, surface, vertices, swept, fraction, scratch)
			}
		}
	case *actor.TriangleMesh:
		scratch.triangles = other.OverlapTriangles(local, scratch.triangles[:0])
		for _, t := range scratch.triangles {
			vertices, _ := other.Triangle(t)
			fraction = triangleImpact(shape, motion, radius, surface, vertices, swept, fraction, scratch)
		}
	}
	return fraction
}

// triangleImpact: the impact with the triangle of the surface, given in its local space, before the fraction
func triangleImpact(shape actor.ShapeInterface, motion *sweep, radius float64, surface *actor.RigidBody, vertices [3]mgl64.Vec3, swept actor.AABB, fraction float64, scratch *ccdScratch) float64 {
	for i := range vertices {
		scratch.shape.vertices[i] = surface.Transform.ToWorld(vertices[i])
	}
	if !triangleAABB(scratch.shape.vertices).Overlaps(swept) {
		return fraction
	}
	proxy := gjk.NewProxyAt(actor.NewTransform(), &scratch.shape)
	return math.Min(fraction, impact(shape, motion, radius, &proxy, nil, fraction, scratch))
}

// isFast: the body can move more than half of its smallest extent during the step, from its velocities at the start
// of the step (the fast body of Box2D v3: maxVelocity * timeStep > 0.5 * minExtent, with the velocity of the farthest
// point of the body, src/solver.c:584 & :622). The narrow phase gives the pairs of a fast body the speculative margin
// of its speed; the continuous collision judges the motion the step made
func isFast(body *actor.RigidBody, dt float64) bool {
	if body.BodyType != actor.BodyTypeDynamic || body.IsSleeping {
		return false
	}
	minExtent, maxExtent := shapeExtents(body.Shape)
	return (body.Velocity.Len()+body.AngularVelocity.Len()*maxExtent)*dt > continuousSafetyFactor*minExtent
}

// shapeExtents: the smallest half size of the shape, and the distance of its farthest point from its center
func shapeExtents(shape actor.ShapeInterface) (float64, float64) {
	switch shape := shape.(type) {
	case *actor.Sphere:
		return shape.Radius, shape.Radius
	case *actor.Capsule:
		return shape.Radius, shape.HalfHeight + shape.Radius
	case *actor.Box:
		h := shape.HalfExtents
		return math.Min(h.X(), math.Min(h.Y(), h.Z())), h.Len()
	}
	aabb := shape.ComputeAABB(actor.NewTransform())
	size := aabb.Max.Sub(aabb.Min).Mul(0.5)
	return math.Min(size.X(), math.Min(size.Y(), size.Z())), size.Len()
}

// shapeMinExtent: the smallest half size of the shape
func shapeMinExtent(shape actor.ShapeInterface) float64 {
	minExtent, _ := shapeExtents(shape)
	return minExtent
}

// rotationAngle of a unit quaternion (rad)
func rotationAngle(q mgl64.Quat) float64 {
	return 2 * math.Acos(math.Min(1, math.Abs(q.W)))
}
