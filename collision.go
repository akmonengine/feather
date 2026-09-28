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
		boxes[i] = body.AABB()
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
// Each pair writes its manifolds at its own offset, so the order never depends on the workers
func narrowPhase(pairs []Pair, workersCount int, margin func(a, b *actor.RigidBody) float64) []constraint.Manifold {
	offsets := make([]int, len(pairs)+1)
	for i, pair := range pairs {
		offsets[i+1] = offsets[i] + manifoldsOf(pair)
	}
	manifolds := make([]constraint.Manifold, offsets[len(pairs)])
	counts := make([]int, len(pairs))
	parallelFor(len(pairs), workersCount, func(i int) {
		counts[i] = collidePair(pairs[i], margin(pairs[i].BodyA, pairs[i].BodyB), manifolds[offsets[i]:offsets[i+1]])
	})
	return compactManifolds(manifolds, offsets, counts)
}

// manifoldsOf: the count of manifolds a pair can have, MaxManifoldsPerPair against a heightfield
func manifoldsOf(pair Pair) int {
	if isHeightfield(pair.BodyA) || isHeightfield(pair.BodyB) {
		return MaxManifoldsPerPair
	}
	return 1
}

func isHeightfield(body *actor.RigidBody) bool {
	_, ok := body.Shape.(*actor.Heightfield)
	return ok
}

// collidePair: triggers keep only the real overlaps
func collidePair(pair Pair, margin float64, out []constraint.Manifold) int {
	a, b := pair.BodyA, pair.BodyB
	count := CollideAll(a, b, margin, out)
	if a.IsTrigger || b.IsTrigger {
		n := 0
		for k := 0; k < count; k++ {
			if out[k].MinSeparation() < 0 {
				out[n] = out[k]
				n++
			}
		}
		count = n
	}
	for k := 0; k < count; k++ {
		setLocalAnchors(&out[k])
	}
	return count
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

// compactManifolds keeps the manifolds found, in the same order: counts[i] manifolds at offsets[i] for the pair i
func compactManifolds(manifolds []constraint.Manifold, offsets, counts []int) []constraint.Manifold {
	n := 0
	for i, count := range counts {
		for k := 0; k < count; k++ {
			manifolds[n] = manifolds[offsets[i]+k]
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
//
// Against a heightfield, a body can touch the terrain with several normals: Collide keeps the deepest patch,
// CollideAll returns all of them
func Collide(a, b *actor.RigidBody, margin float64, m *constraint.Manifold) bool {
	var manifolds [1]constraint.Manifold
	found := CollideAll(a, b, margin, manifolds[:]) > 0
	*m = manifolds[0]
	return found
}

// CollideAll writes in manifolds the contacts between a and b (MaxManifoldsPerPair at most), and returns their count
func CollideAll(a, b *actor.RigidBody, margin float64, manifolds []constraint.Manifold) int {
	if len(manifolds) == 0 {
		return 0
	}
	m := &manifolds[0]
	m.Reset(a, b)

	if field, ok := a.Shape.(*actor.Heightfield); ok {
		return collideHeightfield(a, field, b, margin, false, manifolds)
	}
	if field, ok := b.Shape.(*actor.Heightfield); ok {
		return collideHeightfield(b, field, a, margin, true, manifolds)
	}
	if collide(a, b, margin, m) {
		return 1
	}
	return 0
}

// collide the convex shapes a & b
func collide(a, b *actor.RigidBody, margin float64, m *constraint.Manifold) bool {
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

// planeBuffers: the buffers of collidePlane, reused to avoid the allocations
type planeBuffers struct {
	plane  actor.PlaneContact
	points []constraint.ContactPoint
}

var planeContactsPool = sync.Pool{New: func() any {
	return &planeBuffers{plane: make(actor.PlaneContact, 0, 8), points: make([]constraint.ContactPoint, 0, 8)}
}}

// collidePlane keeps the order of the pair: if the plane is body B, the normal is reversed.
// The points of the shape are reduced to 4 like the other contacts: the deepest first
func collidePlane(plane *actor.Plane, object *actor.RigidBody, margin float64, planeIsB bool, m *constraint.Manifold) bool {
	buffers := planeContactsPool.Get().(*planeBuffers)
	defer planeContactsPool.Put(buffers)
	buffers.plane = object.Shape.CollideWithPlane(plane.Normal, plane.Distance, object.Transform, margin, buffers.plane[:0])
	if len(buffers.plane) == 0 {
		return false
	}

	m.Normal = plane.Normal
	if planeIsB {
		m.Normal = plane.Normal.Mul(-1)
	}
	buffers.points = buffers.points[:0]
	for _, p := range buffers.plane {
		buffers.points = append(buffers.points, constraint.ContactPoint{Position: p.Position, Separation: p.Separation})
	}
	epa.Reduce(buffers.points, m.Normal, m)
	return m.Count > 0
}
