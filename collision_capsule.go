package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

const (
	// segmentEpsilon: under this squared length, a segment is a point (also used to detect parallel segments)
	segmentEpsilon = 1e-12

	// parallelSinSquared: two capsules closer to parallel than ~1.1° touch along a line (2 points)
	parallelSinSquared = 4e-4

	// contactMergeDistance: under this overlap length, both points of a parallel contact are merged
	contactMergeDistance = 1e-6

	// normalEpsilon: under this distance between the closest points, their direction is not reliable for the normal
	normalEpsilon = 1e-9
)

// CollideCapsuleCapsule uses the closest points of both segments.
// Parallel capsules get 2 contact points (both ends of the overlap), so they don't roll
func CollideCapsuleCapsule(a, b *actor.RigidBody, margin float64, manifold *constraint.Manifold) bool {
	capsuleA, okA := a.Shape.(*actor.Capsule)
	capsuleB, okB := b.Shape.(*actor.Capsule)
	if !okA || !okB {
		return false
	}

	a0, a1 := capsuleA.Segment(a.Transform)
	b0, b1 := capsuleB.Segment(b.Transform)

	manifold.Reset(a, b)
	return collideSegments(a0, a1, capsuleA.Radius, b0, b1, capsuleB.Radius,
		b.Transform.Position.Sub(a.Transform.Position), margin, manifold)
}

// CollideCapsuleSphere uses the closest point of the segment to the center of the sphere
func CollideCapsuleSphere(capsule, sphere *actor.RigidBody, margin float64, manifold *constraint.Manifold) bool {
	if _, ok := capsule.Shape.(*actor.Capsule); !ok {
		return false
	}
	if _, ok := sphere.Shape.(*actor.Sphere); !ok {
		return false
	}
	return collideAnalyticPair(capsule, sphere, margin, manifold)
}

// segmentOf returns the segment and radius of a capsule, or of a sphere (a segment of length 0)
func segmentOf(body *actor.RigidBody) (mgl64.Vec3, mgl64.Vec3, float64, bool) {
	switch shape := body.Shape.(type) {
	case *actor.Capsule:
		p0, p1 := shape.Segment(body.Transform)
		return p0, p1, shape.Radius, true
	case *actor.Sphere:
		return body.Transform.Position, body.Transform.Position, shape.Radius, true
	}
	return mgl64.Vec3{}, mgl64.Vec3{}, 0, false
}

// isAnalyticPair: capsules and spheres have an exact solution, no need for GJK/EPA
func isAnalyticPair(a, b actor.ShapeInterface) bool {
	_, aIsCapsule := a.(*actor.Capsule)
	_, bIsCapsule := b.(*actor.Capsule)
	_, aIsSphere := a.(*actor.Sphere)
	_, bIsSphere := b.(*actor.Sphere)

	return (aIsCapsule || aIsSphere) && (bIsCapsule || bIsSphere)
}

func collideAnalyticPair(a, b *actor.RigidBody, margin float64, manifold *constraint.Manifold) bool {
	a0, a1, radiusA, okA := segmentOf(a)
	b0, b1, radiusB, okB := segmentOf(b)
	if !okA || !okB {
		return false
	}
	manifold.Reset(a, b)
	return collideSegments(a0, a1, radiusA, b0, b1, radiusB, b.Transform.Position.Sub(a.Transform.Position), margin, manifold)
}

// collideSegments is used for capsules and spheres: 2 segments with a radius.
// centerOffset (B - A) gives the normal when the segments intersect
func collideSegments(a0, a1 mgl64.Vec3, radiusA float64, b0, b1 mgl64.Vec3, radiusB float64,
	centerOffset mgl64.Vec3, margin float64, manifold *constraint.Manifold) bool {
	directionA := a1.Sub(a0)
	directionB := b1.Sub(b0)
	radii := radiusA + radiusB

	s, t := closestSegmentParameters(a0, directionA, b0, directionB)
	closestA := a0.Add(directionA.Mul(s))
	closestB := b0.Add(directionB.Mul(t))

	delta := closestB.Sub(closestA)
	distanceSquared := delta.LenSqr()
	if reach := radii + margin; distanceSquared > reach*reach {
		return false
	}

	distance := math.Sqrt(distanceSquared)
	var normal mgl64.Vec3
	if distance > normalEpsilon {
		normal = delta.Mul(1 / distance)
	} else {
		normal = fallbackNormal(directionA, directionB, centerOffset)
	}

	manifold.Normal = normal
	manifold.Count = 0

	if areParallel(directionA, directionB) {
		addParallelContacts(a0, directionA, radiusA, b0, directionB, radiusB, margin, manifold)
		if manifold.Count > 0 {
			return true
		}
	}

	addContact(manifold, closestA, closestB, normal, radiusA, radiusB, distance-radii)
	return true
}

// addParallelContacts adds a point at each end of the overlap of 2 parallel segments.
// Nothing if the overlap is too short (end to end): the caller adds the closest point
func addParallelContacts(a0, directionA mgl64.Vec3, radiusA float64, b0, directionB mgl64.Vec3, radiusB float64,
	margin float64, manifold *constraint.Manifold) {
	lengthSquaredA := directionA.LenSqr()
	start := b0.Sub(a0).Dot(directionA) / lengthSquaredA
	end := b0.Add(directionB).Sub(a0).Dot(directionA) / lengthSquaredA

	low := math.Max(0, math.Min(start, end))
	high := math.Min(1, math.Max(start, end))
	if (high-low)*math.Sqrt(lengthSquaredA) <= contactMergeDistance {
		return
	}

	radii := radiusA + radiusB
	for _, parameter := range [2]float64{low, high} {
		onA := a0.Add(directionA.Mul(parameter))
		onB := b0.Add(directionB.Mul(closestPointParameter(b0, directionB, onA)))

		separation := onB.Sub(onA).Dot(manifold.Normal) - radii
		if separation <= margin {
			addContact(manifold, onA, onB, manifold.Normal, radiusA, radiusB, separation)
		}
	}
}

// addContact adds a point halfway between the surface of A and the surface of B
func addContact(manifold *constraint.Manifold, onA, onB, normal mgl64.Vec3, radiusA, radiusB, separation float64) {
	position := onA.Add(onB).Mul(0.5).Add(normal.Mul((radiusA - radiusB) / 2))
	manifold.Add(position, separation)
}

func areParallel(directionA, directionB mgl64.Vec3) bool {
	lengthsSquared := directionA.LenSqr() * directionB.LenSqr()
	if lengthsSquared <= segmentEpsilon {
		return false
	}
	return directionA.Cross(directionB).LenSqr() <= parallelSinSquared*lengthsSquared
}

// fallbackNormal when the segments touch or intersect: perpendicular to both axes if they cross,
// otherwise perpendicular to the axis of A, towards B
func fallbackNormal(directionA, directionB, centerOffset mgl64.Vec3) mgl64.Vec3 {
	normal := directionA.Cross(directionB)
	if normal.LenSqr() <= segmentEpsilon*directionA.LenSqr()*directionB.LenSqr() {
		// Parallel axes, or a point: remove the part of the offset along the axis
		normal = centerOffset
		if lengthSquaredA := directionA.LenSqr(); lengthSquaredA > segmentEpsilon {
			normal = normal.Sub(directionA.Mul(normal.Dot(directionA) / lengthSquaredA))
		}
		if normal.LenSqr() <= segmentEpsilon {
			normal = anyPerpendicular(directionA)
		}
	}
	if normal.Dot(centerOffset) < 0 {
		normal = normal.Mul(-1)
	}
	return normal.Normalize()
}

// anyPerpendicular returns a unit vector orthogonal to v (+Y if v is null)
func anyPerpendicular(v mgl64.Vec3) mgl64.Vec3 {
	if v.LenSqr() <= segmentEpsilon {
		return mgl64.Vec3{0, 1, 0}
	}
	axis := mgl64.Vec3{1, 0, 0}
	if math.Abs(v.X()) > math.Abs(v.Z()) {
		axis = mgl64.Vec3{0, 0, 1}
	}
	return v.Cross(axis).Normalize()
}

// closestPointParameter returns t in [0,1] of the point origin + t*direction closest to p
func closestPointParameter(origin, direction, p mgl64.Vec3) float64 {
	lengthSquared := direction.LenSqr()
	if lengthSquared <= segmentEpsilon {
		return 0
	}
	return clamp01(p.Sub(origin).Dot(direction) / lengthSquared)
}

// closestSegmentParameters returns s & t in [0,1] of the closest points of p1 + s*d1 and p2 + t*d2
// See Ericson, Real-Time Collision Detection, 5.1.9
func closestSegmentParameters(p1, d1, p2, d2 mgl64.Vec3) (float64, float64) {
	r := p1.Sub(p2)
	a := d1.Dot(d1)
	e := d2.Dot(d2)
	f := d2.Dot(r)

	if a <= segmentEpsilon && e <= segmentEpsilon {
		return 0, 0
	}
	if a <= segmentEpsilon {
		return 0, clamp01(f / e)
	}

	c := d1.Dot(r)
	if e <= segmentEpsilon {
		return clamp01(-c / a), 0
	}

	b := d1.Dot(d2)
	denominator := a*e - b*b

	// Parallel segments: any s is valid, we take the first end of A
	s := 0.0
	if denominator > segmentEpsilon*a*e {
		s = clamp01((b*f - c*e) / denominator)
	}

	t := (b*s + f) / e
	if t < 0 {
		return clamp01(-c / a), 0
	}
	if t > 1 {
		return clamp01((b - c) / a), 1
	}
	return s, t
}

func clamp01(value float64) float64 {
	return math.Max(0, math.Min(1, value))
}
