package epa

import (
	"math"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// Manifold generation configuration constants
const (
	// maxContactPoints is the maximum number of contact points in a manifold.
	// Limited to 4 for constraint solver stability (see Erin Catto, GDC 2007).
	maxContactPoints = 4

	// maxBufferSize is the size of pre-allocated working buffers.
	// Must be >= maxContactPoints * 2 to handle worst-case Sutherland-Hodgman clipping.
	maxBufferSize = 8
)

// Numerical tolerance constants for geometric computation stability
const (
	// epsilonColinear is the tolerance for detecting colinear edges.
	// If |edge.Cross(normal)| < epsilonColinear, the edge is parallel to the normal.
	epsilonColinear = 1e-6

	// epsilonDistance is the distance tolerance for Sutherland-Hodgman clipping.
	// Points at distance >= -epsilonDistance from the plane are considered "inside".
	epsilonDistance = 1e-6

	// epsilonParallel is the tolerance for detecting a line parallel to a plane.
	// If |direction.Dot(planeNormal)| < epsilonParallel, the line is parallel.
	epsilonParallel = 1e-10

	// tangentBasisThreshold determines which axis to use for building the tangent basis.
	// If |normal.X()| > tangentBasisThreshold, use Y instead of X as the first tangent.
	tangentBasisThreshold = 0.9
)

// ManifoldBuilder contains all working buffers with fixed-size arrays to avoid allocations.
type ManifoldBuilder struct {
	// Fixed-size arrays to avoid allocations
	localFeatureA [maxBufferSize]mgl64.Vec3
	localFeatureB [maxBufferSize]mgl64.Vec3
	worldFeatureA [maxBufferSize]mgl64.Vec3
	worldFeatureB [maxBufferSize]mgl64.Vec3
	clipBuffer1   [maxBufferSize]mgl64.Vec3
	clipBuffer2   [maxBufferSize]mgl64.Vec3
	clippedResult [maxBufferSize]mgl64.Vec3 // Dedicated buffer for final clipping result
	tempPoints    [maxBufferSize]constraint.ContactPoint

	// Counters
	localFeatureACount int
	localFeatureBCount int
	worldFeatureACount int
	worldFeatureBCount int
	clipBuffer1Count   int
	clipBuffer2Count   int
	clippedResultCount int
	tempPointsCount    int
}

// Pool of builders for reuse
var manifoldBuilderPool = sync.Pool{
	New: func() interface{} {
		return &ManifoldBuilder{}
	},
}

// Reset prepares the builder for a new use
func (b *ManifoldBuilder) Reset() {
	b.localFeatureACount = 0
	b.localFeatureBCount = 0
	b.worldFeatureACount = 0
	b.worldFeatureBCount = 0
	b.clipBuffer1Count = 0
	b.clipBuffer2Count = 0
	b.clippedResultCount = 0
	b.tempPointsCount = 0
}

// GenerateManifold is the main entry point
func GenerateManifold(bodyA, bodyB *actor.RigidBody, normal mgl64.Vec3, depth float64) []constraint.ContactPoint {
	builder := manifoldBuilderPool.Get().(*ManifoldBuilder)
	defer manifoldBuilderPool.Put(builder)

	builder.Reset()

	return builder.Generate(bodyA, bodyB, normal, depth)
}

// Generate generates the manifold using internal buffers
func (b *ManifoldBuilder) Generate(bodyA, bodyB *actor.RigidBody, normal mgl64.Vec3, depth float64) []constraint.ContactPoint {
	// Convert normal to local space
	localNormalA := bodyA.Transform.Rotation.Conjugate().Rotate(normal)
	localNormalB := bodyB.Transform.Rotation.Conjugate().Rotate(normal.Mul(-1))

	// Get features into buffers
	bodyA.Shape.GetContactFeature(localNormalA, &b.localFeatureA, &b.localFeatureACount)
	bodyB.Shape.GetContactFeature(localNormalB, &b.localFeatureB, &b.localFeatureBCount)

	// Transform into buffers
	b.transformFeature(&b.localFeatureA, b.localFeatureACount, bodyA.Transform, bodyA.Shape, &b.worldFeatureA, &b.worldFeatureACount)
	b.transformFeature(&b.localFeatureB, b.localFeatureBCount, bodyB.Transform, bodyB.Shape, &b.worldFeatureB, &b.worldFeatureBCount)

	// Determine incident and reference
	var incident *[8]mgl64.Vec3
	var incidentCount int
	var reference *[8]mgl64.Vec3
	var referenceCount int

	if b.worldFeatureBCount <= b.worldFeatureACount {
		incident = &b.worldFeatureB
		incidentCount = b.worldFeatureBCount
		reference = &b.worldFeatureA
		referenceCount = b.worldFeatureACount
	} else {
		incident = &b.worldFeatureA
		incidentCount = b.worldFeatureACount
		reference = &b.worldFeatureB
		referenceCount = b.worldFeatureBCount
	}

	// Trivial case: single incident point
	if incidentCount == 1 {
		b.tempPoints[0] = constraint.ContactPoint{
			Position:    incident[0],
			Penetration: depth,
		}
		b.tempPointsCount = 1
		return b.buildResult()
	}

	// Clip incident against reference
	clippedCount := b.clipIncidentAgainstReference(incident, incidentCount, reference, referenceCount, normal)

	// Final clip against reference plane
	if clippedCount > 0 && referenceCount > 0 {
		b.clipAgainstReferencePlane(clippedCount, reference, referenceCount, normal, depth)
	}

	// Fallback
	if b.tempPointsCount == 0 {
		deepest := bodyB.SupportWorld(normal.Mul(-1))
		b.tempPoints[0] = constraint.ContactPoint{
			Position:    deepest,
			Penetration: depth,
		}
		b.tempPointsCount = 1
	}

	// Limit to maxContactPoints
	if b.tempPointsCount > maxContactPoints {
		b.reduceTo4Points(normal)
	}

	return b.buildResult()
}

// transformFeature transforms features to world space
func (b *ManifoldBuilder) transformFeature(input *[8]mgl64.Vec3, inputCount int, transform actor.Transform, shape actor.ShapeInterface, output *[8]mgl64.Vec3, outputCount *int) {
	*outputCount = 0

	// Transform points from local to world space
	for i := 0; i < inputCount; i++ {
		rotated := transform.Rotation.Rotate(input[i])
		output[i] = transform.Position.Add(rotated)
	}
	*outputCount = inputCount
}

// clipIncidentAgainstReference clips the incident feature against the reference feature.
// Always returns the result in clipBuffer1 for consistent downstream consumption.
func (b *ManifoldBuilder) clipIncidentAgainstReference(incident *[8]mgl64.Vec3, incidentCount int, reference *[8]mgl64.Vec3, referenceCount int, normal mgl64.Vec3) int {
	// Handle insufficient reference (need at least 2 points for edges)
	if referenceCount < 2 {
		for i := 0; i < incidentCount; i++ {
			b.clipBuffer1[i] = incident[i]
		}
		b.clipBuffer1Count = incidentCount
		return incidentCount
	}

	// Copy incident to clipBuffer1
	for i := 0; i < incidentCount; i++ {
		b.clipBuffer1[i] = incident[i]
	}
	b.clipBuffer1Count = incidentCount
	b.clipBuffer2Count = 0

	useBuffer1 := true

	// Clip against each edge
	for i := 0; i < referenceCount; i++ {
		var inputBuffer *[8]mgl64.Vec3
		var inputCount int
		var outputBuffer *[8]mgl64.Vec3
		var outputCount *int

		if useBuffer1 {
			inputBuffer = &b.clipBuffer1
			inputCount = b.clipBuffer1Count
			outputBuffer = &b.clipBuffer2
			outputCount = &b.clipBuffer2Count
		} else {
			inputBuffer = &b.clipBuffer2
			inputCount = b.clipBuffer2Count
			outputBuffer = &b.clipBuffer1
			outputCount = &b.clipBuffer1Count
		}

		*outputCount = 0

		if inputCount == 0 {
			break
		}

		v1 := reference[i]
		v2 := reference[(i+1)%referenceCount]

		edge := v2.Sub(v1)
		edgeCrossNormal := edge.Cross(normal)

		// Skip if edge is colinear with normal (no lateral clipping needed)
		edgeCrossLen := edgeCrossNormal.Len()
		if edgeCrossLen < epsilonColinear {
			continue
		}

		clipNormal := edgeCrossNormal.Mul(1.0 / edgeCrossLen)

		// Verify direction
		center := b.computeCenter(reference, referenceCount)
		toCenter := center.Sub(v1)
		if toCenter.Dot(clipNormal) < 0 {
			clipNormal = clipNormal.Mul(-1)
		}

		// Clip
		b.clipPolygonAgainstPlane(inputBuffer, inputCount, v1, clipNormal, outputBuffer, outputCount)

		useBuffer1 = !useBuffer1
	}

	// Always put the result in clipBuffer1
	var finalCount int
	if useBuffer1 {
		// Result already in clipBuffer1
		finalCount = b.clipBuffer1Count
	} else {
		// Result in clipBuffer2, copy to clipBuffer1
		finalCount = b.clipBuffer2Count
		for i := 0; i < finalCount; i++ {
			b.clipBuffer1[i] = b.clipBuffer2[i]
		}
		b.clipBuffer1Count = finalCount
	}

	return finalCount
}

// clipPolygonAgainstPlane clips a polygon against a plane using the Sutherland-Hodgman algorithm
func (b *ManifoldBuilder) clipPolygonAgainstPlane(input *[8]mgl64.Vec3, inputCount int, planePoint, planeNormal mgl64.Vec3, output *[8]mgl64.Vec3, outputCount *int) {
	if inputCount == 0 {
		*outputCount = 0
		return
	}

	*outputCount = 0

	for i := 0; i < inputCount; i++ {
		current := input[i]
		next := input[(i+1)%inputCount]

		currentDist := current.Sub(planePoint).Dot(planeNormal)
		nextDist := next.Sub(planePoint).Dot(planeNormal)

		if currentDist >= -epsilonDistance {
			if *outputCount < maxBufferSize {
				output[*outputCount] = current
				*outputCount++
			}

			if nextDist < -epsilonDistance && *outputCount < maxBufferSize {
				intersection := lineIntersectPlane(current, next, planePoint, planeNormal)
				output[*outputCount] = intersection
				*outputCount++
			}
		} else {
			if nextDist >= -epsilonDistance && *outputCount < maxBufferSize {
				intersection := lineIntersectPlane(current, next, planePoint, planeNormal)
				output[*outputCount] = intersection
				*outputCount++
			}
		}
	}
}

// clipAgainstReferencePlane performs final clipping against the reference plane.
// Reads from clipBuffer1 and writes results to tempPoints.
func (b *ManifoldBuilder) clipAgainstReferencePlane(clippedCount int, reference *[8]mgl64.Vec3, referenceCount int, normal mgl64.Vec3, depth float64) {
	b.tempPointsCount = 0

	// Compute reference normal
	edge1 := reference[1].Sub(reference[0])
	edge2 := reference[2].Sub(reference[0])
	refNormal := edge1.Cross(edge2).Normalize()

	if refNormal.Dot(normal) < 0 {
		refNormal = refNormal.Mul(-1)
	}

	refPoint := reference[0]
	offset := refPoint.Dot(refNormal)

	// Always read from clipBuffer1
	for i := 0; i < clippedCount && b.tempPointsCount < maxBufferSize; i++ {
		point := b.clipBuffer1[i]
		distance := point.Dot(refNormal) - offset

		if distance <= 0.0 {
			b.tempPoints[b.tempPointsCount] = constraint.ContactPoint{
				Position:    point,
				Penetration: depth,
			}
			b.tempPointsCount++
		}
	}
}

// reduceTo4Points reduces the contact points to maxContactPoints by keeping the 4 extreme points
func (b *ManifoldBuilder) reduceTo4Points(normal mgl64.Vec3) {
	if b.tempPointsCount <= maxContactPoints {
		return
	}

	tangent1, tangent2 := getTangentBasis(normal)

	minX, maxX, minY, maxY := 0, 0, 0, 0
	minXval, maxXval := math.Inf(1), math.Inf(-1)
	minYval, maxYval := math.Inf(1), math.Inf(-1)

	for i := 0; i < b.tempPointsCount; i++ {
		p := b.tempPoints[i]
		x := p.Position.Dot(tangent1)
		y := p.Position.Dot(tangent2)

		if x < minXval {
			minXval, minX = x, i
		}
		if x > maxXval {
			maxXval, maxX = x, i
		}
		if y < minYval {
			minYval, minY = y, i
		}
		if y > maxYval {
			maxYval, maxY = y, i
		}
	}

	// Collect unique indices
	indices := [maxContactPoints]int{minX, maxX, minY, maxY}
	seen := [maxBufferSize]bool{}

	// Compact to the beginning of the buffer
	newCount := 0
	for _, idx := range indices {
		if !seen[idx] {
			seen[idx] = true
			b.tempPoints[newCount] = b.tempPoints[idx]
			newCount++
		}
	}

	b.tempPointsCount = newCount
}

// buildResult is the ONLY function that allocates (final copy)
func (b *ManifoldBuilder) buildResult() []constraint.ContactPoint {
	result := make([]constraint.ContactPoint, b.tempPointsCount)
	for i := 0; i < b.tempPointsCount; i++ {
		result[i] = b.tempPoints[i]
	}
	return result
}

// computeCenter computes the centroid of a set of points
func (b *ManifoldBuilder) computeCenter(points *[8]mgl64.Vec3, count int) mgl64.Vec3 {
	if count == 0 {
		return mgl64.Vec3{0, 0, 0}
	}

	sum := mgl64.Vec3{0, 0, 0}
	for i := 0; i < count; i++ {
		sum = sum.Add(points[i])
	}
	return sum.Mul(1.0 / float64(count))
}

// lineIntersectPlane computes the intersection point between a line segment and a plane.
// Returns p1 if the line is parallel to the plane. Clamps t to [0,1].
func lineIntersectPlane(p1, p2, planePoint, planeNormal mgl64.Vec3) mgl64.Vec3 {
	dir := p2.Sub(p1)
	dist := p1.Sub(planePoint).Dot(planeNormal)
	denom := dir.Dot(planeNormal)

	if math.Abs(denom) < epsilonParallel {
		return p1
	}

	t := -dist / denom
	t = math.Max(0, math.Min(1, t))

	return p1.Add(dir.Mul(t))
}

// getTangentBasis constructs an orthonormal tangent basis from a normal vector.
// Returns two tangent vectors perpendicular to the normal and to each other.
func getTangentBasis(normal mgl64.Vec3) (mgl64.Vec3, mgl64.Vec3) {
	tangent1 := mgl64.Vec3{1, 0, 0}
	if math.Abs(normal.X()) > tangentBasisThreshold {
		tangent1 = mgl64.Vec3{0, 1, 0}
	}

	tangent1 = tangent1.Sub(normal.Mul(tangent1.Dot(normal))).Normalize()
	tangent2 := normal.Cross(tangent1).Normalize()

	return tangent1, tangent2
}
