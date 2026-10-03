package feather

import (
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== SWEEP ==========
// A convex shape moved along a segment, without rotation, stops where it first touches a body: found by conservative
// advancement (Mirtich 1996), as b3ShapeCast of Box3D. At each iteration GJK gives the distance between the cores of
// both shapes (a point for a sphere, a segment for a capsule: gjk.NewCoreProxy) and its direction; the shape then
// moves forward by this distance, less the radii, divided by the speed it closes at along this direction: no other
// point of it can reach the body sooner, it never goes through. The distance of the cores is exact where the distance
// of the rounded shapes is not: the sweep ends within a micrometer, where the continuous collision (ccd.go), on the
// whole shapes, stops LinearSlop short.
//
// The shape is stopped sweepGap before the contact, not on it: its hit never overlaps the body, by any rounding, and
// the same sweep started from the hit finds the body again at the fraction 0.

const (
	// sweepIterations of the conservative advancement at most (20 in Box3D)
	sweepIterations = 32

	// sweepGap: the moving shape stops this far from the body (m), and is on it within twice this distance, when one
	// of both shapes is rounded
	sweepGap = 0.5e-6

	// sweepFlatGap: the same between 2 shapes without radius (boxes, triangles): GJK is less sharp when both shapes
	// are polytopes almost in contact
	sweepFlatGap = 0.5e-4

	// notConvex: the panic of a sweep or an overlap of a shape which is not convex
	notConvex = "feather: a query needs a convex shape, not a plane, a heightfield nor a mesh"
)

// queryScratch: the buffers of a query, reused to avoid the allocations
type queryScratch struct {
	// mover: the moving shape as a body, for the overlaps (penetration needs bodies)
	mover actor.RigidBody
	// a triangle of a surface, in world space, and its body
	shape    triangleShape
	triangle actor.RigidBody
	simplex  gjk.Simplex
	// sweep: the query of a moving shape. It points to itself (treeQuery.sweep): it cannot live on a stack
	sweep sweepQuery
	// buffers of Overlap: the stack and the candidates of the trees, the cells of a terrain, the triangles of a mesh
	stack      []int32
	candidates []int32
	cells      []int32
	triangles  []int32
}

var queryPool = sync.Pool{New: func() any {
	s := &queryScratch{}
	s.mover = actor.RigidBody{BodyType: actor.BodyTypeStatic}
	s.triangle = actor.RigidBody{Transform: actor.NewTransform(), BodyType: actor.BodyTypeStatic, Shape: &s.shape}
	return s
}}

// sweepQuery: a shape moving through a world
type sweepQuery struct {
	treeQuery
	shape       actor.ShapeInterface
	start       actor.Transform
	translation mgl64.Vec3
	// the core of the shape at its start, and its radius
	core    gjk.Proxy
	radius  float64
	scratch *queryScratch
}

// mustBeConvex: a plane, a heightfield and a mesh have no support point, they cannot be moved nor overlapped
func mustBeConvex(shape actor.ShapeInterface) {
	switch shape.(type) {
	case *actor.Plane, *actor.Heightfield, *actor.TriangleMesh:
		panic(notConvex)
	}
}

// newSweepQuery of the shape moved from start by translation. Through the trees, the shape is the center of its AABB,
// and the AABBs of the nodes are enlarged by its half sizes
func newSweepQuery(w *World, shape actor.ShapeInterface, start actor.Transform, translation mgl64.Vec3, filter QueryFilter, scratch *queryScratch) *sweepQuery {
	q := &scratch.sweep
	*q = sweepQuery{shape: shape, start: start, translation: translation, scratch: scratch}
	q.core, q.radius = gjk.NewCoreProxyAt(start, shape)
	bounds := shape.ComputeAABB(start)
	reach := mgl64.Vec3{2 * sweepFlatGap, 2 * sweepFlatGap, 2 * sweepFlatGap}
	q.treeQuery = treeQuery{
		world: w, filter: filter, limit: 1, sweep: q,
		ray: newTreeRay(bounds.Min.Add(bounds.Max).Mul(0.5), translation, bounds.Max.Sub(bounds.Min).Mul(0.5).Add(reach)),
	}
	scratch.mover.Transform, scratch.mover.Shape = start, shape
	return q
}

// Sweep: the first body touched by the convex shape (a sphere, a capsule, a box) moved from start by translation,
// without rotation, among the bodies the filter accepts. The shape stops between 0 and 1 µm from the body (0.1 mm
// between 2 shapes without radius), never in it: Fraction is where it stops, Point the point of the body it touches,
// Normal the direction from this point to the shape. A shape which starts in a body hits it at the fraction 0, the
// normal against its motion, at a point in both. The top side of a heightfield only is hit, the side of the normal of
// the triangles of a mesh.
// A shape which starts in exact contact with a body, or within 1 µm of it, hits it at the fraction 0 with the normal of
// the contact if it moves towards the body, and doesn't hit it if it moves away or along it (the back facing test of
// the shape casts of Jolt, ConvexShape.cpp; Box3D reports an initial overlap whatever the direction). The exact
// contact is decided by the rounding: a shape put in contact by computed coordinates can be seen in the body, and
// stopped whatever its direction. Start from the place given by a Sweep, or keep a skin.
// After sweepIterations the hit is given where the shape is, before the contact (Box3D gives no hit).
// A plane, a heightfield or a mesh as the moving shape panics
func (w *World) Sweep(shape actor.ShapeInterface, start actor.Transform, translation mgl64.Vec3, filter QueryFilter) (Hit, bool) {
	w.guard()
	mustBeConvex(shape)
	if !finiteSegment(start.Position, translation) {
		return Hit{}, false
	}
	scratch := queryPool.Get().(*queryScratch)
	query := newSweepQuery(w, shape, start, translation, filter, scratch)
	query.run()
	best, found := query.best, query.found
	scratch.release()
	return best, found
}

// release the scratch: it keeps no body nor shape of the world alive
func (s *queryScratch) release() {
	s.sweep = sweepQuery{}
	s.mover.Shape = nil
	queryPool.Put(s)
}

// sweepBody: the hit of the moving shape on the body, before the limit of the query
func (q *sweepQuery) sweepBody(body *actor.RigidBody) (Hit, bool) {
	var hit Hit
	var ok bool
	switch shape := body.Shape.(type) {
	case *actor.Plane:
		hit, ok = q.sweepPlane(shape)
	case *actor.Heightfield:
		hit, ok = q.sweepHeightfield(body, shape)
	case *actor.TriangleMesh:
		hit, ok = q.sweepMesh(body, shape)
	default:
		// the exact AABB of the body, after the AABB of its leaf
		box := body.AABB()
		if _, enters := q.ray.enters(&box, q.limit); !enters {
			return Hit{}, false
		}
		core, radius := gjk.NewCoreProxy(body)
		var overlap bool
		hit, overlap, ok = q.sweepConvex(&core, radius, q.limit)
		if overlap {
			hit.Point = q.overlapPoint(body)
		}
		hit.Triangle = actor.NoTriangle
	}
	hit.Body = body
	return hit, ok
}

// against: the hit of a shape which starts in a body: at once, against its motion
func (q *sweepQuery) against() mgl64.Vec3 {
	if length := q.translation.Len(); length > 0 {
		return q.translation.Mul(-1 / length)
	}
	return mgl64.Vec3{}
}

// sweepConvex: the conservative advancement of the moving core towards the core of a convex shape, up to the limit.
// Returns the hit, and whether the shapes overlap at the start (the hit has no point then)
func (q *sweepQuery) sweepConvex(other *gjk.Proxy, otherRadius, limit float64) (Hit, bool, bool) {
	radii := q.radius + otherRadius
	gap := sweepGap
	if radii == 0 {
		gap = sweepFlatGap
	}
	moving := q.core
	var hit Hit
	for i := 0; ; i++ {
		moving.Position = q.core.Position.Add(q.translation.Mul(hit.Fraction))
		closest := gjk.Distance(&moving, other)
		separation := closest.Distance - radii
		if closest.Overlap || separation < 0 {
			if i == 0 {
				return Hit{Normal: q.against()}, true, true
			}
			// GJK lost the shapes it left apart (they are too close for it): the shape stops where it was
			return hit, false, true
		}
		// the speed the shapes close at, along the direction of their closest points
		approach := q.translation.Dot(closest.Normal)
		if approach <= 0 {
			return Hit{}, false, false
		}
		hit.Point, hit.Normal = closest.PointB.Sub(closest.Normal.Mul(otherRadius)), closest.Normal.Mul(-1)
		if separation <= 2*gap || i == sweepIterations-1 {
			return hit, false, true
		}
		hit.Fraction += (separation - gap) / approach
		if hit.Fraction > limit {
			return Hit{}, false, false
		}
	}
}

// overlapPoint: a point in both the moving shape at its start and the body: the middle of their deepest points
func (q *sweepQuery) overlapPoint(body *actor.RigidBody) mgl64.Vec3 {
	result, ok := penetration(&q.scratch.mover, body, 0, &q.scratch.simplex)
	if !ok {
		return q.start.Position
	}
	return result.WitnessA.Add(result.WitnessB).Mul(0.5)
}

// sweepPlane: the lowest point of the shape along the normal of the plane reaches it first: exact
func (q *sweepQuery) sweepPlane(plane *actor.Plane) (Hit, bool) {
	rotation := &q.start.Rotation
	lowest := q.start.Position.Add(actor.Rotate(rotation, q.shape.Support(actor.RotateInverse(rotation, plane.Normal.Mul(-1)))))
	separation := lowest.Dot(plane.Normal) + plane.Distance
	if separation < 0 {
		return Hit{Point: lowest, Normal: q.against(), Triangle: actor.NoTriangle}, true
	}
	approach := -q.translation.Dot(plane.Normal)
	if approach <= 0 {
		return Hit{}, false
	}
	gap := sweepGap
	if q.radius == 0 {
		gap = sweepFlatGap
	}
	fraction := 0.0
	if separation > 2*gap {
		fraction = (separation - gap) / approach
		if fraction > q.limit {
			return Hit{}, false
		}
	}
	touching := lowest.Add(q.translation.Mul(fraction))
	onPlane := touching.Sub(plane.Normal.Mul(touching.Dot(plane.Normal) + plane.Distance))
	return Hit{Fraction: fraction, Point: onPlane, Normal: plane.Normal, Triangle: actor.NoTriangle}, true
}

// sweepTriangle: the hit of the moving shape on a triangle in world space, its vertices turning around its normal,
// before the limit. Only the side of the normal is hit: a triangle is skipped if the shape moves along its normal (it
// comes from behind, or leaves). The triangles of a heightfield and of a mesh are swept this way
func (q *sweepQuery) sweepTriangle(vertices [3]mgl64.Vec3, limit float64) (Hit, bool) {
	normal := vertices[1].Sub(vertices[0]).Cross(vertices[2].Sub(vertices[0]))
	if q.translation.Dot(normal) > 0 {
		return Hit{}, false
	}
	box := triangleAABB(vertices)
	if _, enters := q.ray.enters(&box, limit); !enters {
		return Hit{}, false
	}
	s := q.scratch
	s.shape.vertices, s.shape.aabb = vertices, box
	proxy := gjk.NewProxyAt(s.triangle.Transform, &s.shape)
	hit, overlap, ok := q.sweepConvex(&proxy, 0, limit)
	if overlap {
		hit.Point = q.overlapPoint(&s.triangle)
	}
	return hit, ok
}
