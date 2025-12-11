package epa

import (
	"math"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// ManifoldBuilder - Contient TOUS les buffers de travail avec arrays fixes
type ManifoldBuilder struct {
	// Arrays fixes pour éviter les allocations
	localFeatureA [8]mgl64.Vec3
	localFeatureB [8]mgl64.Vec3
	worldFeatureA [8]mgl64.Vec3
	worldFeatureB [8]mgl64.Vec3
	clipBuffer1   [8]mgl64.Vec3
	clipBuffer2   [8]mgl64.Vec3
	clippedResult [8]mgl64.Vec3 // NOUVEAU : buffer dédié pour le résultat final du clipping
	tempPoints    [8]constraint.ContactPoint

	// Compteurs
	localFeatureACount int
	localFeatureBCount int
	worldFeatureACount int
	worldFeatureBCount int
	clipBuffer1Count   int
	clipBuffer2Count   int
	clippedResultCount int // NOUVEAU
	tempPointsCount    int
}

// Pool de builders
var manifoldBuilderPool = sync.Pool{
	New: func() interface{} {
		return &ManifoldBuilder{}
	},
}

// Reset - Prépare le builder pour une nouvelle utilisation
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

// GenerateManifold - Point d'entrée principal (API publique)
func GenerateManifold(bodyA, bodyB *actor.RigidBody, normal mgl64.Vec3, depth float64) []constraint.ContactPoint {
	builder := manifoldBuilderPool.Get().(*ManifoldBuilder)
	defer manifoldBuilderPool.Put(builder)

	builder.Reset()

	return builder.Generate(bodyA, bodyB, normal, depth)
}

// Generate - Génère le manifold en utilisant les buffers internes
func (b *ManifoldBuilder) Generate(bodyA, bodyB *actor.RigidBody, normal mgl64.Vec3, depth float64) []constraint.ContactPoint {
	// Convertir la normale en espace local
	localNormalA := bodyA.Transform.Rotation.Conjugate().Rotate(normal)
	localNormalB := bodyB.Transform.Rotation.Conjugate().Rotate(normal.Mul(-1))

	// Get features DANS les buffers
	bodyA.Shape.GetContactFeature(localNormalA, &b.localFeatureA, &b.localFeatureACount)
	bodyB.Shape.GetContactFeature(localNormalB, &b.localFeatureB, &b.localFeatureBCount)

	// Transform DANS les buffers
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

	// Cas trivial : single incident point
	if incidentCount == 1 {
		b.tempPoints[0] = constraint.ContactPoint{
			Position:    incident[0],
			Penetration: depth,
		}
		b.tempPointsCount = 1
		return b.buildResult()
	}

	// Clip incident against reference - résultat dans clippedResult
	clippedCount := b.clipIncidentAgainstReference(incident, incidentCount, reference, referenceCount, normal)

	// Clip final contre le plan de référence
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

	// Limit to 4 points
	if b.tempPointsCount > 4 {
		b.reduceTo4Points(normal)
	}

	return b.buildResult()
}

// transformFeature - Transforme vers world space
func (b *ManifoldBuilder) transformFeature(input *[8]mgl64.Vec3, inputCount int, transform actor.Transform, shape actor.ShapeInterface, output *[8]mgl64.Vec3, outputCount *int) {
	*outputCount = 0

	// Cas spécial : plan
	if plane, ok := shape.(*actor.Plane); ok {
		tangent1, tangent2 := getTangentBasis(plane.Normal)
		center := plane.Normal.Mul(-plane.Distance)
		const size = 1000.0

		output[0] = center.Add(tangent1.Mul(-size)).Add(tangent2.Mul(-size))
		output[1] = center.Add(tangent1.Mul(-size)).Add(tangent2.Mul(size))
		output[2] = center.Add(tangent1.Mul(size)).Add(tangent2.Mul(size))
		output[3] = center.Add(tangent1.Mul(size)).Add(tangent2.Mul(-size))
		*outputCount = 4
		return
	}

	// Transform normal
	for i := 0; i < inputCount; i++ {
		rotated := transform.Rotation.Rotate(input[i])
		output[i] = transform.Position.Add(rotated)
	}
	*outputCount = inputCount
}

// clipIncidentAgainstReference - MODIFIÉ pour toujours retourner dans clipBuffer1
func (b *ManifoldBuilder) clipIncidentAgainstReference(incident *[8]mgl64.Vec3, incidentCount int, reference *[8]mgl64.Vec3, referenceCount int, normal mgl64.Vec3) int {
	// Check si c'est un grand plan
	if b.isLargePlane(reference, referenceCount) {
		// Copier incident dans clipBuffer1
		for i := 0; i < incidentCount; i++ {
			b.clipBuffer1[i] = incident[i]
		}
		b.clipBuffer1Count = incidentCount
		return incidentCount
	}

	if referenceCount < 2 {
		for i := 0; i < incidentCount; i++ {
			b.clipBuffer1[i] = incident[i]
		}
		b.clipBuffer1Count = incidentCount
		return incidentCount
	}

	// Copier incident dans clipBuffer1
	for i := 0; i < incidentCount; i++ {
		b.clipBuffer1[i] = incident[i]
	}
	b.clipBuffer1Count = incidentCount
	b.clipBuffer2Count = 0

	useBuffer1 := true

	// Clip contre chaque edge
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

		// ========== FIX : Skip si edge est colinéaire avec normal ==========
		edgeCrossLen := edgeCrossNormal.Len()
		if edgeCrossLen < 1e-6 {
			// Edge et normal sont colinéaires, skip cet edge
			// (pas de clipping latéral nécessaire)
			continue
		}

		clipNormal := edgeCrossNormal.Mul(1.0 / edgeCrossLen)

		// Vérifier direction
		center := b.computeCenter(reference, referenceCount)
		toCenter := center.Sub(v1)
		if toCenter.Dot(clipNormal) < 0 {
			clipNormal = clipNormal.Mul(-1)
		}

		// Clip
		b.clipPolygonAgainstPlane(inputBuffer, inputCount, v1, clipNormal, outputBuffer, outputCount)

		useBuffer1 = !useBuffer1
	}

	// ========== FIX : Toujours mettre le résultat dans clipBuffer1 ==========
	var finalCount int
	if useBuffer1 {
		// Résultat déjà dans clipBuffer1
		finalCount = b.clipBuffer1Count
	} else {
		// Résultat dans clipBuffer2, copier vers clipBuffer1
		finalCount = b.clipBuffer2Count
		for i := 0; i < finalCount; i++ {
			b.clipBuffer1[i] = b.clipBuffer2[i]
		}
		b.clipBuffer1Count = finalCount
	}

	return finalCount
}

// clipPolygonAgainstPlane - Sutherland-Hodgman pour un plan
func (b *ManifoldBuilder) clipPolygonAgainstPlane(input *[8]mgl64.Vec3, inputCount int, planePoint, planeNormal mgl64.Vec3, output *[8]mgl64.Vec3, outputCount *int) {
	if inputCount == 0 {
		*outputCount = 0
		return
	}

	*outputCount = 0
	const tolerance = 1e-6

	for i := 0; i < inputCount; i++ {
		current := input[i]
		next := input[(i+1)%inputCount]

		currentDist := current.Sub(planePoint).Dot(planeNormal)
		nextDist := next.Sub(planePoint).Dot(planeNormal)

		if currentDist >= -tolerance {
			if *outputCount < 8 {
				output[*outputCount] = current
				*outputCount++
			}

			if nextDist < -tolerance && *outputCount < 8 {
				intersection := lineIntersectPlane(current, next, planePoint, planeNormal)
				output[*outputCount] = intersection
				*outputCount++
			}
		} else {
			if nextDist >= -tolerance && *outputCount < 8 {
				intersection := lineIntersectPlane(current, next, planePoint, planeNormal)
				output[*outputCount] = intersection
				*outputCount++
			}
		}
	}
}

// clipAgainstReferencePlane - Clip final (utilise clippedResult directement)
// clipAgainstReferencePlane - SIMPLIFIÉ : toujours lire clipBuffer1
func (b *ManifoldBuilder) clipAgainstReferencePlane(clippedCount int, reference *[8]mgl64.Vec3, referenceCount int, normal mgl64.Vec3, depth float64) {
	b.tempPointsCount = 0

	// Calculer normale de référence
	edge1 := reference[1].Sub(reference[0])
	edge2 := reference[2].Sub(reference[0])
	refNormal := edge1.Cross(edge2).Normalize()

	if refNormal.Dot(normal) < 0 {
		refNormal = refNormal.Mul(-1)
	}

	refPoint := reference[0]
	offset := refPoint.Dot(refNormal)

	// ========== FIX : Toujours lire clipBuffer1 ==========
	for i := 0; i < clippedCount && b.tempPointsCount < 8; i++ {
		point := b.clipBuffer1[i] // Toujours clipBuffer1
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

// reduceTo4Points - Réduit à 4 points maximum
func (b *ManifoldBuilder) reduceTo4Points(normal mgl64.Vec3) {
	if b.tempPointsCount <= 4 {
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

	// Collecter les indices uniques
	indices := [4]int{minX, maxX, minY, maxY}
	seen := [8]bool{}

	// Compacter vers le début du buffer
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

// buildResult - SEULE fonction qui alloue (copie finale)
func (b *ManifoldBuilder) buildResult() []constraint.ContactPoint {
	result := make([]constraint.ContactPoint, b.tempPointsCount)
	for i := 0; i < b.tempPointsCount; i++ {
		result[i] = b.tempPoints[i]
	}
	return result
}

// ========== Fonctions helper ==========

func (b *ManifoldBuilder) isLargePlane(feature *[8]mgl64.Vec3, count int) bool {
	if count != 4 {
		return false
	}

	for i := 0; i < count; i++ {
		for j := i + 1; j < count; j++ {
			if feature[i].Sub(feature[j]).Len() > 100 {
				return true
			}
		}
	}
	return false
}

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

func lineIntersectPlane(p1, p2, planePoint, planeNormal mgl64.Vec3) mgl64.Vec3 {
	dir := p2.Sub(p1)
	dist := p1.Sub(planePoint).Dot(planeNormal)
	denom := dir.Dot(planeNormal)

	if math.Abs(denom) < 1e-10 {
		return p1
	}

	t := -dist / denom
	t = math.Max(0, math.Min(1, t))

	return p1.Add(dir.Mul(t))
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
