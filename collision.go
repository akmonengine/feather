package feather

import (
	"math"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
)

const (
	// pairCacheMaxDeltaPosition: the contact of a pair is computed again if B moved more than 1 mm relative to A (m)
	pairCacheMaxDeltaPosition = 0.001

	// pairCacheCosMaxDeltaRotationDiv2: or if B turned more than 2° relative to A, cos(2° / 2)
	pairCacheCosMaxDeltaRotationDiv2 = 0.99984769515639123915701155881391
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
		found[i] = collidePair(pairs[i], margin(pairs[i].BodyA, pairs[i].BodyB), &manifolds[i])
	})
	return compactManifolds(manifolds, found)
}

// collidePair: triggers keep only the real overlaps
func collidePair(pair Pair, margin float64, m *constraint.Manifold) bool {
	a, b := pair.BodyA, pair.BodyB
	found := Collide(a, b, margin, m)
	if found && (a.IsTrigger || b.IsTrigger) {
		found = m.MinSeparation() < 0
	}
	if found {
		setLocalAnchors(m)
	}
	return found
}

// setLocalAnchors stores the contact in the local spaces of the bodies, for the next step
func setLocalAnchors(m *constraint.Manifold) {
	transformA, transformB := m.BodyA.Transform, m.BodyB.Transform
	for j := 0; j < m.Count; j++ {
		point := &m.Points[j]
		// Position is halfway between both surfaces, the normal goes from A to B
		halfSeparation := m.Normal.Mul(point.Separation / 2)
		point.LocalAnchorA = transformA.ToLocal(point.Position.Sub(halfSeparation))
		point.LocalAnchorB = transformB.ToLocal(point.Position.Add(halfSeparation))
	}
	m.LocalNormal = transformA.Rotation.Conjugate().Rotate(m.Normal)
	m.RelativePosition = transformA.ToLocal(transformB.Position)
	m.RelativeRotation = transformA.Rotation.Conjugate().Mul(transformB.Rotation)
}

// reuseManifold: if B moved less than 1 mm and 2° relative to A since the contact points were computed,
// the previous contact points are moved with the bodies instead of running the collision detection again
// (like the body pair cache of Jolt). The separation of each point is measured again.
func reuseManifold(previous *constraint.Manifold, margin float64, m *constraint.Manifold) bool {
	transformA, transformB := previous.BodyA.Transform, previous.BodyB.Transform

	relativePosition := transformA.ToLocal(transformB.Position)
	if relativePosition.Sub(previous.RelativePosition).LenSqr() > pairCacheMaxDeltaPosition*pairCacheMaxDeltaPosition {
		return false
	}
	relativeRotation := transformA.Rotation.Conjugate().Mul(transformB.Rotation)
	if math.Abs(relativeRotation.Dot(previous.RelativeRotation)) < pairCacheCosMaxDeltaRotationDiv2 {
		return false
	}

	m.Reset(previous.BodyA, previous.BodyB)
	m.Normal = transformA.Rotation.Rotate(previous.LocalNormal)
	m.LocalNormal = previous.LocalNormal
	m.RelativePosition = previous.RelativePosition
	m.RelativeRotation = previous.RelativeRotation
	for j := 0; j < previous.Count; j++ {
		point := &previous.Points[j]
		onA := transformA.ToWorld(point.LocalAnchorA)
		onB := transformB.ToWorld(point.LocalAnchorB)
		separation := onB.Sub(onA).Dot(m.Normal)
		if separation > margin {
			continue
		}
		m.Points[m.Count] = constraint.ContactPoint{
			Position:     onA.Add(onB).Mul(0.5),
			Separation:   separation,
			LocalAnchorA: point.LocalAnchorA,
			LocalAnchorB: point.LocalAnchorB,
		}
		m.Count++
	}
	return m.Count > 0
}

// compactManifolds keeps the manifolds found, in the same order
func compactManifolds(manifolds []constraint.Manifold, found []bool) []constraint.Manifold {
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

	proxyA, proxyB := gjk.NewProxy(a), gjk.NewProxy(b)
	if !gjk.GJKProxies(&proxyA, &proxyB, margin, simplex) {
		return false
	}
	result, err := epa.EPAProxies(&proxyA, &proxyB, simplex, margin)
	if err != nil {
		return false
	}
	epa.Manifold(a, b, result, margin, m)
	return m.Count > 0
}

// planeContactsPool: the buffers given to CollideWithPlane, reused to avoid the allocations
var planeContactsPool = sync.Pool{New: func() any {
	contacts := make(actor.PlaneContact, 0, 8)
	return &contacts
}}

// collidePlane keeps the order of the pair: if the plane is body B, the normal is reversed
func collidePlane(plane *actor.Plane, object *actor.RigidBody, margin float64, planeIsB bool, m *constraint.Manifold) bool {
	buffer := planeContactsPool.Get().(*actor.PlaneContact)
	defer planeContactsPool.Put(buffer)
	points := object.Shape.CollideWithPlane(plane.Normal, plane.Distance, object.Transform, margin, (*buffer)[:0])
	*buffer = points
	if len(points) == 0 {
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
