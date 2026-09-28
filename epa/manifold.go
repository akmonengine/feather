package epa

import (
	"math"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

const (
	// maxBufferSize: a quad clipped by 4 planes has at most 8 vertices
	maxBufferSize = 8

	// minFaceAlignment: a face is in contact if its normal is aligned with the contact normal (0.5°).
	// Otherwise the contact is an edge or a vertex
	minFaceAlignment = 0.99996

	// edgeTolerance: 2 points at the same height along the normal (relative to the size of the feature) form an edge
	edgeTolerance = 1e-4

	// parallelSin: 2 edges closer to parallel than ~1.1° touch along a line
	parallelSin = 0.02

	// faceTieTolerance: if both faces are aligned, the face of A is the reference (the choice must not change between 2 steps)
	faceTieTolerance = 1e-3

	// epsilonDistance of the Sutherland-Hodgman clipping
	epsilonDistance = 1e-9

	epsilonLength = 1e-12
)

type polygon struct {
	points [maxBufferSize]mgl64.Vec3
	count  int
}

func (p *polygon) add(v mgl64.Vec3) {
	if p.count < maxBufferSize {
		p.points[p.count] = v
		p.count++
	}
}

// Manifold generates the contact points of A and B, from the result of EPA:
//   - face contact (a face aligned with the normal): the other feature is clipped by the sides of this face (Sutherland-Hodgman)
//   - parallel edges (a box on an edge, a capsule along an edge): one edge is clipped by the other
//   - otherwise (crossing edges, vertex, sphere): the witness point of EPA
//
// The deepest point has the separation of EPA, the other points are higher along the normal.
// Points further than the margin are removed, and 4 points are kept at most
func Manifold(a, b *actor.RigidBody, result Result, margin float64, m *constraint.Manifold) {
	m.Reset(a, b)
	normal := result.Normal
	m.Normal = normal
	separation := margin - result.Depth

	buffers := featuresPool.Get().(*features)
	defer featuresPool.Put(buffers)
	featureA, featureB := &buffers.a, &buffers.b
	feature(a, normal, featureA)
	feature(b, normal.Mul(-1), featureB)

	if referenceIsA, ok := chooseReference(featureA, featureB, normal); ok {
		reference, incident := featureA, featureB
		direction := normal // from the reference body towards the incident one
		if !referenceIsA {
			reference, incident = featureB, featureA
			direction = normal.Mul(-1)
		}
		clipFeatures(reference, incident, direction, separation, margin, m)
	} else {
		edgeA := deepest(featureA, normal)
		edgeB := deepest(featureB, normal.Mul(-1))
		if edgeA.count == 2 && edgeB.count == 2 && parallel(&edgeA, &edgeB) {
			clipped := edgeB
			clipToSlab(&clipped, edgeA.points[0], edgeA.points[1])
			keepPoints(&clipped, normal, separation, margin, m)
		}
	}

	if m.Count == 0 {
		// Witness point, halfway between A and B. WitnessA is on A + margin
		onA := result.WitnessA.Sub(normal.Mul(margin))
		m.Add(onA.Add(result.WitnessB).Mul(0.5), separation)
	}
}

// deepest keeps the points of the feature the furthest along the direction: the deepest edge or vertex of a face
func deepest(p *polygon, direction mgl64.Vec3) polygon {
	var out polygon
	if p.count == 0 {
		return out
	}
	size := 0.0
	top := math.Inf(-1)
	for i := 0; i < p.count; i++ {
		top = math.Max(top, p.points[i].Dot(direction))
		size = math.Max(size, p.points[i].Sub(p.points[0]).Len())
	}
	for i := 0; i < p.count; i++ {
		if top-p.points[i].Dot(direction) <= edgeTolerance*size {
			out.add(p.points[i])
		}
	}
	return out
}

func parallel(a, b *polygon) bool {
	da := a.points[1].Sub(a.points[0])
	db := b.points[1].Sub(b.points[0])
	lengths := da.Len() * db.Len()
	return lengths > epsilonLength && da.Cross(db).Len() <= parallelSin*lengths
}

// clipToSlab keeps the part of the segment between the planes at both ends of [start, end]
func clipToSlab(segment *polygon, start, end mgl64.Vec3) {
	axis := end.Sub(start)
	length := axis.Len()
	if length < epsilonLength {
		return
	}
	axis = axis.Mul(1 / length)
	var scratch polygon
	clipAgainstPlane(segment, start, axis, &scratch)
	clipAgainstPlane(&scratch, end, axis.Mul(-1), segment)
}

// feature returns the feature of the body facing the direction, in world space
func feature(body *actor.RigidBody, direction mgl64.Vec3, out *polygon) {
	body.Shape.GetContactFeature(body.Transform.Rotation.Conjugate().Rotate(direction), &out.points, &out.count)
	for i := 0; i < out.count; i++ {
		out.points[i] = body.Transform.ToWorld(out.points[i])
	}
}

// features are the buffers of Manifold: they escape to the heap through the interface of the shapes,
// so they are reused
type features struct {
	a polygon
	b polygon
}

var featuresPool = sync.Pool{New: func() any { return &features{} }}

// chooseReference returns the reference face: the face aligned with the normal (the face of A if both are)
func chooseReference(featureA, featureB *polygon, normal mgl64.Vec3) (bool, bool) {
	alignA := -1.0
	if featureA.count >= 3 {
		alignA = math.Abs(faceNormal(featureA).Dot(normal))
	}
	alignB := -1.0
	if featureB.count >= 3 {
		alignB = math.Abs(faceNormal(featureB).Dot(normal))
	}

	switch {
	case alignA >= minFaceAlignment && alignA >= alignB-faceTieTolerance:
		return true, true
	case alignB >= minFaceAlignment:
		return false, true
	}
	return false, false
}

// faceNormal returns the normal of the polygon, in any orientation
func faceNormal(p *polygon) mgl64.Vec3 {
	n := p.points[1].Sub(p.points[0]).Cross(p.points[2].Sub(p.points[0]))
	length := n.Len()
	if length < epsilonLength {
		return mgl64.Vec3{}
	}
	return n.Mul(1 / length)
}

// clipFeatures clips the incident feature with the side planes of the reference face
func clipFeatures(reference, incident *polygon, direction mgl64.Vec3, separation, margin float64, m *constraint.Manifold) {
	refNormal := faceNormal(reference)
	if refNormal.Dot(direction) < 0 {
		refNormal = refNormal.Mul(-1)
	}

	center := mgl64.Vec3{}
	for i := 0; i < reference.count; i++ {
		center = center.Add(reference.points[i])
	}
	center = center.Mul(1 / float64(reference.count))

	clipped := *incident
	var scratch polygon
	for i := 0; i < reference.count && clipped.count > 0; i++ {
		v1 := reference.points[i]
		v2 := reference.points[(i+1)%reference.count]
		sideNormal := v2.Sub(v1).Cross(refNormal)
		length := sideNormal.Len()
		if length < epsilonLength {
			continue
		}
		sideNormal = sideNormal.Mul(1 / length)
		if sideNormal.Dot(center.Sub(v1)) < 0 {
			sideNormal = sideNormal.Mul(-1)
		}
		clipAgainstPlane(&clipped, v1, sideNormal, &scratch)
		clipped, scratch = scratch, clipped
	}

	keepPoints(&clipped, direction, separation, margin, m)
}

// keepPoints converts the clipped points into contact points.
// The deepest point has the separation of EPA, the others are higher along the direction (from the reference towards the incident body)
func keepPoints(clipped *polygon, direction mgl64.Vec3, separation, margin float64, m *constraint.Manifold) {
	if clipped.count == 0 {
		return
	}
	lowest := clipped.points[0].Dot(direction)
	for i := 1; i < clipped.count; i++ {
		lowest = math.Min(lowest, clipped.points[i].Dot(direction))
	}

	var candidates [maxBufferSize]constraint.ContactPoint
	count := 0
	for i := 0; i < clipped.count; i++ {
		p := clipped.points[i]
		pointSeparation := separation + p.Dot(direction) - lowest
		if pointSeparation > margin {
			continue
		}
		candidates[count] = constraint.ContactPoint{
			Position:   p.Sub(direction.Mul(pointSeparation / 2)),
			Separation: pointSeparation,
		}
		count++
	}

	reduce(candidates[:count], direction, m)
}

// clipAgainstPlane keeps the part of the polygon (or segment) in front of the plane
func clipAgainstPlane(in *polygon, point, normal mgl64.Vec3, out *polygon) {
	out.count = 0
	if in.count == 1 {
		if in.points[0].Sub(point).Dot(normal) >= -epsilonDistance {
			out.add(in.points[0])
		}
		return
	}

	edges := in.count
	if in.count == 2 {
		edges = 1 // an open segment, not a closed polygon
	}

	for i := 0; i < edges; i++ {
		current := in.points[i]
		next := in.points[(i+1)%in.count]
		dc := current.Sub(point).Dot(normal)
		dn := next.Sub(point).Dot(normal)

		if dc >= -epsilonDistance {
			out.add(current)
		}
		if (dc >= -epsilonDistance) != (dn >= -epsilonDistance) {
			t := dc / (dc - dn)
			out.add(current.Add(next.Sub(current).Mul(t)))
		}
		if in.count == 2 && dn >= -epsilonDistance {
			out.add(next)
		}
	}
}

// reduce keeps 4 points: the deepest, the furthest from it, then the points adding the most area to the contact polygon
func reduce(points []constraint.ContactPoint, normal mgl64.Vec3, m *constraint.Manifold) {
	if len(points) <= constraint.MaxContactPoints {
		for _, p := range points {
			m.Add(p.Position, p.Separation)
		}
		return
	}

	chosen := [constraint.MaxContactPoints]int{}
	deepest := 0
	for i, p := range points {
		if p.Separation < points[deepest].Separation {
			deepest = i
		}
	}
	chosen[0] = deepest

	farthest, best := -1, -1.0
	for i, p := range points {
		d := planar(p.Position.Sub(points[deepest].Position), normal).LenSqr()
		if d > best {
			farthest, best = i, d
		}
	}
	chosen[1] = farthest

	third, best := -1, -1.0
	for i, p := range points {
		area := math.Abs(signedArea(points[deepest].Position, points[farthest].Position, p.Position, normal))
		if area > best {
			third, best = i, area
		}
	}
	chosen[2] = third

	orientation := math.Copysign(1, signedArea(points[deepest].Position, points[farthest].Position, points[third].Position, normal))
	fourth, best := -1, 0.0
	triangle := [3]int{deepest, farthest, third}
	for i, p := range points {
		for e := 0; e < 3; e++ {
			// area added outside the edge e
			added := -orientation * signedArea(points[triangle[e]].Position, points[triangle[(e+1)%3]].Position, p.Position, normal)
			if added > best {
				fourth, best = i, added
			}
		}
	}

	for k := 0; k < 3; k++ {
		m.Add(points[chosen[k]].Position, points[chosen[k]].Separation)
	}
	if fourth >= 0 {
		m.Add(points[fourth].Position, points[fourth].Separation)
	}
}

func planar(v, normal mgl64.Vec3) mgl64.Vec3 {
	return v.Sub(normal.Mul(v.Dot(normal)))
}

// signedArea returns 2x the signed area of the triangle, seen along the normal
func signedArea(a, b, c, normal mgl64.Vec3) float64 {
	return b.Sub(a).Cross(c.Sub(a)).Dot(normal)
}
