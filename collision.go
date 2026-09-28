package feather

import (
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
)

// BroadPhase returns the pairs of bodies whose AABBs overlap, always in the same order
func BroadPhase(spatialGrid *SpatialGrid, bodies []*actor.RigidBody, workersCount int) []Pair {
	boxes := make([]actor.AABB, len(bodies))
	for i, body := range bodies {
		boxes[i] = body.Shape.GetAABB()
	}
	spatialGrid.Clear()
	for i, body := range bodies {
		spatialGrid.InsertAABB(i, body, boxes[i])
	}
	return spatialGrid.FindPairs(bodies, boxes, workersCount)
}

// NarrowPhase returns the contacts of the overlapping pairs (without speculative contacts), in the order of the pairs
func NarrowPhase(pairs []Pair, workersCount int) []constraint.Manifold {
	return narrowPhase(pairs, workersCount, func(a, b *actor.RigidBody) float64 { return 0 })
}

// narrowPhase runs Collide on each pair in parallel.
// Each result is written at the index of its pair, so the order never depends on the workers
func narrowPhase(pairs []Pair, workersCount int, margin func(a, b *actor.RigidBody) float64) []constraint.Manifold {
	manifolds := make([]constraint.Manifold, len(pairs))
	found := make([]bool, len(pairs))

	parallelFor(len(pairs), workersCount, func(i int) {
		a, b := pairs[i].BodyA, pairs[i].BodyB
		m := &manifolds[i]
		found[i] = Collide(a, b, margin(a, b), m)
		if found[i] && (a.IsTrigger || b.IsTrigger) {
			found[i] = m.MinSeparation() < 0
		}
		for j := 0; j < m.Count; j++ {
			m.Points[j].LocalAnchorA = m.BodyA.Transform.ToLocal(m.Points[j].Position)
		}
	})

	n := 0
	for i := range manifolds {
		if found[i] {
			manifolds[n] = manifolds[i]
			n++
		}
	}
	return manifolds[:n]
}

// Collide computes the contact between a and b, including the points closer than margin.
// The normal points from a to b.
// - planes: CollideWithPlane of the shape
// - spheres & capsules: closest points of their segments (collision_capsule.go)
// - other shapes: GJK/EPA, then the contact points are clipped (epa/manifold.go)
func Collide(a, b *actor.RigidBody, margin float64, m *constraint.Manifold) bool {
	m.Reset(a, b)

	if plane, ok := a.Shape.(*actor.Plane); ok {
		return collidePlane(plane, b, margin, false, m)
	}
	if plane, ok := b.Shape.(*actor.Plane); ok {
		return collidePlane(plane, a, margin, true, m)
	}

	if isAnalyticPair(a.Shape, b.Shape) {
		return collideAnalyticPair(a, b, margin, m)
	}

	simplex := gjk.SimplexPool.Get().(*gjk.Simplex)
	defer gjk.SimplexPool.Put(simplex)
	simplex.Reset()

	if !gjk.GJKMargin(a, b, margin, simplex) {
		return false
	}
	result, err := epa.EPA(a, b, simplex, margin)
	if err != nil {
		return false
	}
	epa.Manifold(a, b, result, margin, m)
	return m.Count > 0
}

// collidePlane keeps the order of the pair: if the plane is body B, the normal is reversed
func collidePlane(plane *actor.Plane, object *actor.RigidBody, margin float64, planeIsB bool, m *constraint.Manifold) bool {
	collision, points := object.Shape.CollideWithPlane(plane.Normal, plane.Distance, object.Transform, margin)
	if !collision {
		return false
	}

	m.Normal = plane.Normal
	if planeIsB {
		m.Normal = plane.Normal.Mul(-1)
	}
	for _, p := range points {
		m.Add(p.Position, p.Separation)
	}
	return m.Count > 0
}
