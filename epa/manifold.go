package epa

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// GenerateManifold creates contact points for a collision using Sutherland-Hodgman clipping.
//
// A contact manifold is a set of 1-4 contact points that represent where two shapes touch.
// Multiple points provide stability (preventing rotation/jitter) and distribute forces realistically.
//
// Algorithm:
//  1. Get contact features from each shape (point, edge, or face)
//  2. Transform features to world space
//  3. Determine incident (fewer points) and reference (more points) features
//  4. Clip incident feature against reference feature's side planes
//  5. Keep points that are penetrating
//  6. Reduce to max 4 points if needed (for performance)
//
// Special cases:
//   - Sphere-Sphere: Single contact point at midpoint
//   - Sphere-Box/Plane: Project sphere center onto closest feature
//   - Box-Box: Sutherland-Hodgman clipping (1-4 points)
//
// For detailed explanation with visual examples, see:
// ALGORITHMS.md - "Manifold Generation" section
//
// Parameters:
//   - bodyA, bodyB: The two colliding bodies
//   - normal: Contact normal (from A toward B)
//   - depth: Penetration depth (positive value)
//
// Returns:
//
//	Slice of 1-4 ContactPoints with positions and penetration depth
func GenerateManifold(bodyA, bodyB *actor.RigidBody, normal mgl64.Vec3, depth float64) []constraint.ContactPoint {
	// Convertir la normale en espace local pour chaque body
	localNormalA := bodyA.Transform.Rotation.Conjugate().Rotate(normal)
	localNormalB := bodyB.Transform.Rotation.Conjugate().Rotate(normal.Mul(-1))

	// Get features avec les directions locales
	featureA := bodyA.Shape.GetContactFeature(localNormalA)
	featureB := bodyB.Shape.GetContactFeature(localNormalB)

	// Transform to world space
	worldFeatureA := transformFeature(featureA, bodyA.Transform, bodyA.Shape)
	worldFeatureB := transformFeature(featureB, bodyB.Transform, bodyB.Shape)

	// Determine incident and reference
	var incident, reference []mgl64.Vec3

	if len(worldFeatureB) <= len(worldFeatureA) {
		incident = worldFeatureB
		reference = worldFeatureA
	} else {
		incident = worldFeatureA
		reference = worldFeatureB
	}

	// Cas trivial : single incident point
	if len(incident) == 1 {
		return []constraint.ContactPoint{{
			Position:    incident[0],
			Penetration: depth,
		}}
	}

	// Clip incident against reference (plans latéraux)
	clipped := clipIncidentAgainstReference(incident, reference, normal)
	// ÉTAPE 3 : Clip final contre le plan de référence
	var contactPoints []constraint.ContactPoint

	if len(clipped) > 0 && len(reference) > 0 {
		// Calculer la normale de la face de référence
		// (devrait être proche de 'normal' mais calculée depuis la géométrie)
		edge1 := reference[1].Sub(reference[0])
		edge2 := reference[2].Sub(reference[0])
		refNormal := edge1.Cross(edge2).Normalize()

		// S'assurer que la normale pointe dans la bonne direction
		if refNormal.Dot(normal) < 0 {
			refNormal = refNormal.Mul(-1)
		}

		// Le plan passe par n'importe quel point de la référence
		refPoint := reference[0]
		offset := refPoint.Dot(refNormal)

		for _, point := range clipped {
			// Distance signée au plan de référence
			distance := point.Dot(refNormal) - offset

			// Garder les points "derrière" ou sur le plan (distance <= 0)
			if distance <= 0.0 {
				contactPoints = append(contactPoints, constraint.ContactPoint{
					Position:    point,
					Penetration: depth,
				})
			}
		}
	}

	// Fallback si aucun point après clipping
	if len(contactPoints) == 0 {
		deepest := bodyB.SupportWorld(normal.Mul(-1))
		contactPoints = append(contactPoints, constraint.ContactPoint{
			Position:    deepest,
			Penetration: depth,
		})
	}

	// Limit to 4 points
	if len(contactPoints) > 4 {
		contactPoints = reduceTo4Points(contactPoints, normal)
	}

	return contactPoints
}

// clipIncidentAgainstReference performs Sutherland-Hodgman polygon clipping.
//
// Clips the incident feature (polygon) against the side planes of the reference feature.
// This finds the intersection of the two features, which gives us the contact region.
//
// The algorithm clips the incident polygon against each edge plane of the reference polygon:
//  1. For each reference edge, create a clipping plane perpendicular to contact normal
//  2. Clip incident polygon against this plane (keep points "inside")
//  3. Result is the incident polygon trimmed to the reference feature's bounds
//
// Special handling:
//   - Plane reference features: No lateral clipping (infinite extent)
//   - Point/edge references: Skip clipping (insufficient geometry)
//
// Parameters:
//   - incident: Polygon to be clipped (vertices in world space)
//   - reference: Reference polygon defining clipping planes
//   - normal: Contact normal (defines "inside" direction)
//
// Returns:
//
//	Clipped polygon vertices (0-N points, typically 1-4 for boxes)
func clipIncidentAgainstReference(incident, reference []mgl64.Vec3, normal mgl64.Vec3) []mgl64.Vec3 {
	// If reference is a large plane (Plane), no lateral clipping is necessary
	if isLargePlane(reference) {
		return incident
	}

	// If reference has less than 2 points, clipping is not possible
	if len(reference) < 2 {
		return incident
	}

	output := incident

	var edge, clipNormal, toCenter, center mgl64.Vec3
	// Clip against each edge of the reference
	for i := 0; i < len(reference); i++ {
		if len(output) == 0 {
			break
		}

		// Current edge
		v1 := reference[i]
		v2 := reference[(i+1)%len(reference)]

		// Clipping plane normal (perpendicular to the edge, pointing inward)
		edge = v2.Sub(v1)
		clipNormal = edge.Cross(normal).Normalize()

		// Verify that the normal points inward
		// (toward the center of the reference feature)
		center = computeCenter(reference)
		toCenter = center.Sub(v1)
		if toCenter.Dot(clipNormal) < 0 {
			clipNormal = clipNormal.Mul(-1)
		}

		// Clip against this plane
		output = clipPolygonAgainstPlane(output, v1, clipNormal)
	}

	return output
}

// clipPolygonAgainstPlane implements Sutherland-Hodgman for a single plane
func clipPolygonAgainstPlane(polygon []mgl64.Vec3, planePoint, planeNormal mgl64.Vec3) []mgl64.Vec3 {
	if len(polygon) == 0 {
		return polygon
	}

	var output []mgl64.Vec3
	var intersection mgl64.Vec3
	for i := 0; i < len(polygon); i++ {
		current := polygon[i]
		next := polygon[(i+1)%len(polygon)]

		currentDist := current.Sub(planePoint).Dot(planeNormal)
		nextDist := next.Sub(planePoint).Dot(planeNormal)

		const tolerance = 1e-6

		// Current is inside
		if currentDist >= -tolerance {
			output = append(output, current)

			// Next is outside → add intersection
			if nextDist < -tolerance {
				intersection = lineIntersectPlane(current, next, planePoint, planeNormal)
				output = append(output, intersection)
			}
		} else {
			// Current is outside, next is inside → add intersection
			if nextDist >= -tolerance {
				intersection = lineIntersectPlane(current, next, planePoint, planeNormal)
				output = append(output, intersection)
			}
		}
	}

	return output
}

// lineIntersectPlane calculates the intersection between a line segment and a plane
func lineIntersectPlane(p1, p2, planePoint, planeNormal mgl64.Vec3) mgl64.Vec3 {
	dir := p2.Sub(p1)
	dist := p1.Sub(planePoint).Dot(planeNormal)
	denom := dir.Dot(planeNormal)

	if math.Abs(denom) < 1e-10 {
		return p1 // Segment parallel to plane
	}

	t := -dist / denom
	t = math.Max(0, math.Min(1, t)) // Clamp to segment

	return p1.Add(dir.Mul(t))
}

// computeCenter calculates the centroid of a set of points
func computeCenter(points []mgl64.Vec3) mgl64.Vec3 {
	if len(points) == 0 {
		return mgl64.Vec3{0, 0, 0}
	}

	sum := mgl64.Vec3{0, 0, 0}
	for _, p := range points {
		sum = sum.Add(p)
	}
	return sum.Mul(1.0 / float64(len(points)))
}

// isLargePlane detects if a feature represents an infinite plane
func isLargePlane(feature []mgl64.Vec3) bool {
	if len(feature) != 4 {
		return false
	}

	// Check if distances between points are very large (> 100)
	for i := 0; i < len(feature); i++ {
		for j := i + 1; j < len(feature); j++ {
			if feature[i].Sub(feature[j]).Len() > 100 {
				return true
			}
		}
	}
	return false
}

func transformFeature(feature []mgl64.Vec3, transform actor.Transform, shape actor.ShapeInterface) []mgl64.Vec3 {
	if plane, ok := shape.(*actor.Plane); ok {
		tangent1, tangent2 := getTangentBasis(plane.Normal)
		center := plane.Normal.Mul(-plane.Distance)
		const size = 1000.0

		return []mgl64.Vec3{
			center.Add(tangent1.Mul(-size)).Add(tangent2.Mul(-size)),
			center.Add(tangent1.Mul(-size)).Add(tangent2.Mul(size)),
			center.Add(tangent1.Mul(size)).Add(tangent2.Mul(size)),
			center.Add(tangent1.Mul(size)).Add(tangent2.Mul(-size)),
		}
	}

	result := make([]mgl64.Vec3, len(feature))
	for i, point := range feature {
		rotated := transform.Rotation.Rotate(point)
		result[i] = transform.Position.Add(rotated)
	}
	return result
}

func getTangentBasis(normal mgl64.Vec3) (mgl64.Vec3, mgl64.Vec3) {
	tangent1 := mgl64.Vec3{1, 0, 0}
	if math.Abs(normal.X()) > 0.9 {
		tangent1 = mgl64.Vec3{0, 1, 0}
	}

	tangent1 = tangent1.Sub(normal.Mul(tangent1.Dot(normal))).Normalize()
	tangent2 := normal.Cross(tangent1).Normalize()

	return tangent1, tangent2
}

func reduceTo4Points(points []constraint.ContactPoint, normal mgl64.Vec3) []constraint.ContactPoint {
	tangent1, tangent2 := getTangentBasis(normal)

	minX, maxX, minY, maxY := 0, 0, 0, 0
	minXval, maxXval := math.Inf(1), math.Inf(-1)
	minYval, maxYval := math.Inf(1), math.Inf(-1)

	for i, p := range points {
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

	indices := map[int]bool{minX: true, maxX: true, minY: true, maxY: true}

	result := make([]constraint.ContactPoint, 0, 4)
	for idx := range indices {
		result = append(result, points[idx])
	}

	return result
}
