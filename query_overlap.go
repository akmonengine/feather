package feather

import (
	"slices"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== OVERLAP ==========
// The bodies a convex shape touches: the candidates of the trees for its AABB (Tree.queryCandidates), then the
// distance between the cores of both shapes by GJK, as the sweep: the shapes overlap when it is not over the sum of
// their radii. The contact counts.

// Overlap appends to bodies the bodies which overlap the convex shape (a sphere, a capsule, a box) at the transform,
// among the bodies the filter accepts, in the order of World.Bodies, and returns the slice: no allocation if its
// capacity is enough. A body in contact with the shape overlaps it. A heightfield or a mesh overlaps the shape when one
// of its triangles does, from any side. A plane, a heightfield or a mesh as the shape panics
func (w *World) Overlap(shape actor.ShapeInterface, at actor.Transform, filter QueryFilter, bodies []*actor.RigidBody) []*actor.RigidBody {
	w.guard()
	mustBeConvex(shape)
	if !finiteSegment(at.Position, mgl64.Vec3{}) {
		return bodies
	}
	scratch := queryPool.Get().(*queryScratch)
	bounds := shape.ComputeAABB(at)
	scratch.stack, scratch.candidates = w.tree.queryCandidates(bounds, scratch.stack, scratch.candidates[:0])
	// the trees give the bodies in the order of their leaves
	slices.Sort(scratch.candidates)
	core, radius := gjk.NewCoreProxyAt(at, shape)
	for _, index := range scratch.candidates {
		body := w.Bodies[index]
		if !filter.Accepts(body) || !bounds.Overlaps(body.AABB()) {
			continue
		}
		var overlaps bool
		switch other := body.Shape.(type) {
		case *actor.Plane:
			// the lowest point of the shape along the normal of the plane
			rotation := &at.Rotation
			lowest := at.Position.Add(actor.Rotate(rotation, shape.Support(actor.RotateInverse(rotation, other.Normal.Mul(-1)))))
			overlaps = lowest.Dot(other.Normal)+other.Distance <= 0
		case *actor.Heightfield:
			overlaps = scratch.overlapsHeightfield(&core, radius, bounds, body, other)
		case *actor.TriangleMesh:
			overlaps = scratch.overlapsMesh(&core, radius, bounds, body, other)
		default:
			otherCore, otherRadius := gjk.NewCoreProxy(body)
			overlaps = coresOverlap(&core, radius, &otherCore, otherRadius)
		}
		if overlaps {
			bodies = append(bodies, body)
		}
	}
	scratch.release()
	return bodies
}

// coresOverlap: the cores are not further than the sum of their radii
func coresOverlap(a *gjk.Proxy, radiusA float64, b *gjk.Proxy, radiusB float64) bool {
	closest := gjk.Distance(a, b)
	return closest.Overlap || closest.Distance <= radiusA+radiusB
}
