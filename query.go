package feather

import (
	"math"
	"slices"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== QUERIES ==========
// The queries read the trees of the broad phase (tree.go). A step leaves them up to date: it updates them once more
// at its end, after the continuous collision, as the last stage of a step of Box2D v3 refits its tree for the queries
// which follow ("refit BVH", docs/simulation.md; b2Solve in solver.c). What the game writes between 2 steps
// is taken by SyncQueries, as Physics.SyncTransforms of Unity and PxSceneQuerySystem::flushUpdates of PhysX.
//
// A query writes nothing: any number of goroutines can run queries together, while nothing writes the world (Step,
// SyncQueries, AddBody, RemoveBody, a write to a body...). Feather takes no lock, the caller orders the readers and
// the writer; a query started during a step panics, as Box3D refuses a locked world.

//
// A ray or a moving shape is a segment: an origin and a translation, the hit at a fraction of the translation (the
// convention of Box3D: b3World_CastRay, b3RayResult). The planes and the heightfields are tested first: they are in no
// tree, and their hit shortens the segment for the trees (Tree.cast). The result never depends on the shape of the
// trees: at the same fraction, the body of the lowest index in World.Bodies is hit.

// queryDuringStep: the panic of a query started while Step runs
const queryDuringStep = "feather: query during Step"

// guard: a query during a step would read bodies and trees being written
func (w *World) guard() {
	if w.stepping.Load() {
		panic(queryDuringStep)
	}
}

// SyncQueries: the queries see the bodies added, moved or reshaped by the game since the last Step. A body is seen
// at its AABB: after a write to its transform or to its shape, call its UpdateAABB first. Without SyncQueries, these
// changes are seen after the next Step only (RemoveBody is seen at once).
// On a world which didn't change it touches no tree: it costs a test per body
func (w *World) SyncQueries() {
	w.guard()
	w.syncTrees(w.workerPool())
}

// Hit: where a ray or a moving shape first touches a body
type Hit struct {
	Body     *actor.RigidBody
	Point    mgl64.Vec3 // world space, on the surface of Body
	Normal   mgl64.Vec3 // unit, out of Body, at Point
	Fraction float64    // of the translation, in [0, 1]
	// heightfield: 2*cell + t (cell = x*(ZSamples-1)+z), the lowest index if several triangles are hit at the same
	// fraction (a ray on an edge); else actor.NoTriangle
	Triangle int32
	// index of Body in World.Bodies when it was hit: the order of the hits at the same fraction
	index int32
}

// before: the order of the hits, by fraction then by index
func (h *Hit) before(other *Hit) bool {
	return h.Fraction < other.Fraction || (h.Fraction == other.Fraction && h.index < other.index)
}

// treeQuery: a ray or a moving shape through the planes and the trees of a world. It lives on the stack of its query
type treeQuery struct {
	world  *World
	filter QueryFilter
	ray    treeRay
	// limit: the fraction after which a hit is not looked for (the best hit so far, or 1 for all the hits)
	limit float64
	best  Hit
	found bool
	// all: every hit is appended to hits, else the first one is kept in best
	all  bool
	hits []Hit
	// sweep: the query moves a shape (query_sweep.go), else it is a ray
	sweep *sweepQuery
}

// run the query on the large bodies, then on both trees
func (q *treeQuery) run() {
	tree := &q.world.tree
	for _, index := range tree.planes {
		q.visit(index)
	}
	tree.statics.cast(tree.statics.root, q)
	tree.dynamics.cast(tree.dynamics.root, q)
}

// visit the body of a leaf, or a large body: its hit is kept if it comes first
func (q *treeQuery) visit(index int32) {
	body := q.world.Bodies[index]
	if !q.filter.Accepts(body) {
		return
	}
	var found Hit
	if q.sweep != nil {
		var ok bool
		if found, ok = q.sweep.sweepBody(body); !ok {
			return
		}
	} else {
		hit, ok := castRay(body, q.ray.origin, q.ray.translation, q.limit)
		if !ok {
			return
		}
		found = Hit{Body: body, Point: q.ray.origin.Add(q.ray.translation.Mul(hit.Fraction)), Normal: hit.Normal, Fraction: hit.Fraction, Triangle: hit.Triangle}
	}
	found.index = index
	if q.all {
		q.hits = append(q.hits, found)
		return
	}
	if !q.found || found.before(&q.best) {
		q.best, q.found, q.limit = found, true, found.Fraction
	}
}

// castRay on the body, in world space: the ray goes to the local space of the shape, its normal comes back
func castRay(body *actor.RigidBody, origin, translation mgl64.Vec3, maxFraction float64) (actor.RayHit, bool) {
	switch shape := body.Shape.(type) {
	case *actor.Plane:
		// a plane is in world space, whatever the transform of its body
		return shape.CastRay(origin, translation, maxFraction)
	case *actor.Sphere:
		// a sphere turned is the same sphere
		return shape.CastRay(origin.Sub(body.Transform.Position), translation, maxFraction)
	}
	rotation := &body.Transform.Rotation
	if rotation.V == (mgl64.Vec3{}) {
		// not turned: most of the terrains and of the decor
		return body.Shape.CastRay(origin.Sub(body.Transform.Position), translation, maxFraction)
	}
	hit, ok := body.Shape.CastRay(actor.RotateInverse(rotation, origin.Sub(body.Transform.Position)), actor.RotateInverse(rotation, translation), maxFraction)
	if ok {
		hit.Normal = actor.Rotate(rotation, hit.Normal)
	}
	return hit, ok
}

// finiteSegment: a query with a NaN or an infinite coordinate hits nothing (x - x is 0 for a finite x only)
func finiteSegment(origin, translation mgl64.Vec3) bool {
	sum := 0.0
	for k := 0; k < 3; k++ {
		sum += (origin[k] - origin[k]) + (translation[k] - translation[k])
	}
	return sum == 0
}

// Raycast: the first body on the segment from origin to origin + translation, among the bodies the filter accepts.
// A ray which starts in a body hits it at the fraction 0, the normal against its direction; a ray without length is a
// point, its hit has no normal. The top side of a heightfield only is hit. A ray which is not finite hits nothing
func (w *World) Raycast(origin, translation mgl64.Vec3, filter QueryFilter) (Hit, bool) {
	w.guard()
	if !finiteSegment(origin, translation) {
		return Hit{}, false
	}
	query := treeQuery{world: w, filter: filter, ray: newTreeRay(origin, translation, mgl64.Vec3{}), limit: 1}
	query.run()
	return query.best, query.found
}

// RaycastAll appends to hits one hit per body on the segment, its first one, sorted by fraction then by the index of
// the body in World.Bodies, and returns the slice: no allocation if its capacity is enough
// (hits = w.RaycastAll(origin, translation, filter, hits[:0]))
func (w *World) RaycastAll(origin, translation mgl64.Vec3, filter QueryFilter, hits []Hit) []Hit {
	w.guard()
	if !finiteSegment(origin, translation) {
		return hits
	}
	query := treeQuery{world: w, filter: filter, ray: newTreeRay(origin, translation, mgl64.Vec3{}), limit: 1, all: true, hits: hits}
	query.run()
	slices.SortFunc(query.hits[len(hits):], func(a, b Hit) int {
		if a.before(&b) {
			return -1
		}
		return 1
	})
	return query.hits
}

// ========== THE TREES ==========

// treeStackSize: the nodes a traversal keeps for later. A tree is rarely higher than 2 log2(bodies): 64 is enough for
// any world, and a higher tree is still walked (aabbTree.cast)
const treeStackSize = 64

// treeRay: a segment against the AABBs of the nodes, by slabs. For a moving shape, the segment is the path of the
// center of its AABB, and the AABBs of the nodes are enlarged by its half sizes (b3DynamicTree_BoxCast of Box3D)
type treeRay struct {
	origin, translation mgl64.Vec3
	// inverse of the translation, finite: 0 x infinity would be a NaN for a ray starting in the plane of a face
	inverse mgl64.Vec3
	extents mgl64.Vec3
}

func newTreeRay(origin, translation, extents mgl64.Vec3) treeRay {
	ray := treeRay{origin: origin, translation: translation, extents: extents}
	for k := 0; k < 3; k++ {
		if translation[k] != 0 {
			ray.inverse[k] = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, 1/translation[k]))
		}
	}
	return ray
}

// enters: the fraction where the segment, cut at limit, enters the AABB, false if it doesn't. The faces belong to the
// AABB: a segment lying in the plane of a face, or stopping on it, enters
func (r *treeRay) enters(box *actor.AABB, limit float64) (float64, bool) {
	enter, exit := 0.0, limit
	for k := 0; k < 3; k++ {
		low, high := box.Min[k]-r.extents[k]-r.origin[k], box.Max[k]+r.extents[k]-r.origin[k]
		if r.translation[k] == 0 {
			if low > 0 || high < 0 {
				return 0, false
			}
			continue
		}
		near, far := low*r.inverse[k], high*r.inverse[k]
		if near > far {
			near, far = far, near
		}
		enter, exit = max(enter, near), min(exit, far)
	}
	return enter, enter <= exit
}

// castEntry: a node kept for later, with the fraction where the segment enters it
type castEntry struct {
	node  int32
	enter float64
}

// cast the query through the nodes under root, in depth: the segment is tested against the AABB of both children, the
// child it enters first is walked first, and the nodes it enters after the limit of the query, which shrinks at each
// hit, are dropped (b3DynamicTree_RayCast of Box3D). The stack is an array on the stack of the goroutine: when it is
// full, a child is walked at once by a call instead of being kept, and no leaf is lost
func (t *aabbTree) cast(root int32, q *treeQuery) {
	if root == nullNode || t.empty() {
		return
	}
	enter, ok := q.ray.enters(&t.nodes[root].aabb, q.limit)
	if !ok {
		return
	}
	var stack [treeStackSize]castEntry
	stack[0] = castEntry{root, enter}
	for count := 1; count > 0; {
		count--
		entry := stack[count]
		// the limit may have shrunk since the node was kept. A node entered at the limit is walked: a body of a lower
		// index can be hit at the same fraction
		if entry.enter > q.limit {
			continue
		}
		node := &t.nodes[entry.node]
		if node.height == 0 {
			q.visit(node.body)
			continue
		}
		near, far := castEntry{node: node.child1}, castEntry{node: node.child2}
		var nearOk, farOk bool
		near.enter, nearOk = q.ray.enters(&t.nodes[near.node].aabb, q.limit)
		far.enter, farOk = q.ray.enters(&t.nodes[far.node].aabb, q.limit)
		if farOk && (!nearOk || far.enter < near.enter) {
			near, far, nearOk, farOk = far, near, farOk, nearOk
		}
		// the further child is kept under the closer one (the node just left the stack: the first child fits)
		if farOk {
			stack[count] = far
			count++
		}
		if nearOk {
			if count < len(stack) {
				stack[count] = near
				count++
			} else {
				t.cast(near.node, q)
			}
		}
	}
}
