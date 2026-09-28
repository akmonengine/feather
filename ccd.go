package feather

import (
	"math"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== CONTINUOUS COLLISION ==========
// The speculative contacts stop most of the fast bodies. They can miss a body accelerated by the solver during the step:
// as in Box2D v3, after the solver, a fast body is moved back to its first impact with a static body (or with any body
// for a bullet), found along its motion. Its velocity is kept: the contact of the next step stops it.

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
	// its smallest extent, at its center) is stopped (Box2D B2_CORE_FRACTION)
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

// ccdScratch: the buffers of the continuous collision, reused to avoid the allocations
type ccdScratch struct {
	stack      []int32
	candidates []int32
	cells      []int32
	shape      triangleShape
	core       actor.Sphere
}

var ccdPool = sync.Pool{New: func() any { return &ccdScratch{} }}

// continuous collision of the fast bodies: first the bodies against the static bodies, then the bullets against all
// the bodies (at their final position). The result doesn't depend on the order of the bodies
func (w *World) continuous(s *solver, dt float64) {
	scratch := ccdPool.Get().(*ccdScratch)
	defer ccdPool.Put(scratch)
	for _, bullets := range [2]bool{false, true} {
		for i := range s.states {
			state := &s.states[i]
			body := state.body
			if body.IsBullet != bullets || body.IsTrigger {
				continue
			}
			minExtent, maxExtent := shapeExtents(body.Shape)
			motion := sweep{
				start: actor.Transform{Position: body.Transform.Position.Sub(state.deltaPosition), Rotation: state.rotation},
				end:   body.Transform,
			}
			motion.angle = rotationAngle(state.deltaRotation)
			// the farthest point of the body moves at most by the translation + the rotation * its extent
			if state.deltaPosition.Len()+motion.angle*maxExtent <= continuousSafetyFactor*minExtent {
				continue
			}
			scratch.core.Radius = coreFraction * minExtent
			w.stopAtImpact(body, &motion, maxExtent, scratch)
		}
	}
}

// stopAtImpact moves the body back to its first impact during its motion
func (w *World) stopAtImpact(body *actor.RigidBody, motion *sweep, radius float64, scratch *ccdScratch) {
	// the bodies around the motion
	swept := body.Shape.ComputeAABB(motion.start)
	end := body.Shape.ComputeAABB(motion.end)
	for k := 0; k < 3; k++ {
		swept.Min[k] = math.Min(swept.Min[k], end.Min[k])
		swept.Max[k] = math.Max(swept.Max[k], end.Max[k])
	}
	scratch.stack, scratch.candidates = w.tree.queryCandidates(swept, scratch.stack, scratch.candidates[:0])

	fraction := 1.0
	for _, index := range scratch.candidates {
		other := w.Bodies[index]
		if other == body || other.IsTrigger || w.jointPairs[makePairKey(body, other)] > 0 {
			continue
		}
		// the bullets against all the bodies, the other bodies against the static bodies only
		if other.BodyType != actor.BodyTypeStatic && (!body.IsBullet || other.IsBullet) {
			continue
		}
		if !swept.Overlaps(other.AABB()) {
			continue
		}
		switch shape := other.Shape.(type) {
		case *actor.Plane:
			fraction = math.Min(fraction, impact(body.Shape, motion, radius, nil, shape, fraction, scratch))
		case *actor.Heightfield:
			fraction = math.Min(fraction, w.heightfieldImpact(body.Shape, motion, radius, other, shape, swept, fraction, scratch))
		default:
			proxy := gjk.NewProxy(other)
			fraction = math.Min(fraction, impact(body.Shape, motion, radius, &proxy, nil, fraction, scratch))
		}
	}

	if fraction < 1 {
		body.Transform = motion.at(fraction)
	}
	body.UpdateAABB()
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

// heightfieldImpact: the first impact with the triangles under the motion
func (w *World) heightfieldImpact(shape actor.ShapeInterface, motion *sweep, radius float64, terrain *actor.RigidBody, field *actor.Heightfield, swept actor.AABB, maxFraction float64, scratch *ccdScratch) float64 {
	scratch.cells = field.OverlapCells(localBounds(terrain.Transform, swept), scratch.cells[:0])
	fraction := maxFraction
	cellsZ := field.ZSamples - 1
	for _, cell := range scratch.cells {
		x, z := int(cell)/cellsZ, int(cell)%cellsZ
		for t := 0; t < 2; t++ {
			local, _ := field.Triangle(x, z, t)
			for i := range local {
				scratch.shape.vertices[i] = terrain.Transform.ToWorld(local[i])
			}
			if !triangleAABB(scratch.shape.vertices).Overlaps(swept) {
				continue
			}
			proxy := gjk.NewProxyAt(actor.NewTransform(), &scratch.shape)
			fraction = math.Min(fraction, impact(shape, motion, radius, &proxy, nil, fraction, scratch))
		}
	}
	return fraction
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

// rotationAngle of a unit quaternion (rad)
func rotationAngle(q mgl64.Quat) float64 {
	return 2 * math.Acos(math.Min(1, math.Abs(q.W)))
}
