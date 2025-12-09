package epa

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

type Face struct {
	Points   [3]mgl64.Vec3 // Les 3 sommets du triangle
	Normal   mgl64.Vec3    // Normale pointant vers l'extérieur
	Distance float64       // Distance de l'origine au plan de la face
}

func createFaceOutward(a, b, c, oppositePoint mgl64.Vec3) *Face {
	face := facePool.Get().(*Face)
	face.Points = [3]mgl64.Vec3{a, b, c}

	// Calculate two edges of the triangle
	ab := b.Sub(a)
	ac := c.Sub(a)

	// Normal via cross product (right-hand rule)
	normal := ab.Cross(ac)

	// Normalize the normal first (important!)
	normalLength := math.Sqrt(normal.Dot(normal))
	if normalLength < 1e-8 {
		// Degenerate triangle (zero area)
		face.Normal = mgl64.Vec3{0, 1, 0}
		face.Distance = 0.0001
		return face
	}
	normal = normal.Normalize()

	// === CRITICAL PART: Ensure normal points OUTWARD ===

	// Vector from face point A to the opposite point
	toOpposite := oppositePoint.Sub(a)

	// If normal points TOWARDS the opposite point, it's pointing INWARD
	// We need to flip it to point OUTWARD
	if normal.Dot(toOpposite) > 0 {
		normal = normal.Mul(-1)
	}

	// Calculate distance from origin to the plane
	distance := a.Dot(normal)

	// Distance should be positive (normal points away from origin)
	// If negative, the origin is on the "wrong side" of the face
	if distance < 0 {
		normal = normal.Mul(-1)
		distance = -distance
	}

	// Force minimum distance to avoid degenerate cases
	if distance < 0.0001 {
		distance = 0.0001
	}

	face.Normal = snapNormalToAxis(normal)
	face.Distance = distance

	return face
}

func buildInitialFaces(simplex []mgl64.Vec3) []*Face {
	a, b, c, d := simplex[0], simplex[1], simplex[2], simplex[3]

	// Create the 4 triangular faces
	// Each face is defined by 3 points + the opposite point for reference
	candidateFaces := []*Face{
		createFaceOutward(a, b, c, d), // Face ABC, opposite point is D
		createFaceOutward(a, c, d, b), // Face ACD, opposite point is B
		createFaceOutward(a, d, b, c), // Face ADB, opposite point is C
		createFaceOutward(b, d, c, a), // Face BDC, opposite point is A
	}

	// Filter out degenerate faces (too close to origin)
	var faces []*Face
	for _, face := range candidateFaces {
		if face.Distance >= 0.0001 {
			faces = append(faces, face)
		} else {
			facePool.Put(face) // Libère les faces invalides
		}
	}

	// Safety: need at least 3 faces
	if len(faces) < 3 {
		return candidateFaces
	}

	return faces
}

func findClosestFaceIndex(faces []*Face) int {
	closestIndex := 0
	minDistance := faces[0].Distance

	for i := 1; i < len(faces); i++ {
		if faces[i].Distance < minDistance {
			closestIndex = i
			minDistance = faces[i].Distance
		}
	}

	return closestIndex
}

type Edge struct {
	A, B mgl64.Vec3
}

func addPointAndRebuildFaces(faces *[]*Face, support mgl64.Vec3, closestIndex int) {
	// Calculate the centroid of the current polytope (average of all points)
	var centroid mgl64.Vec3
	pointSet := make(map[mgl64.Vec3]bool)

	for _, face := range *faces {
		for _, p := range face.Points {
			pointSet[p] = true
		}
	}

	count := 0
	for point := range pointSet {
		centroid = centroid.Add(point)
		count++
	}
	if count > 0 {
		centroid = centroid.Mul(1.0 / float64(count))
	}

	// Find visible faces
	var visibleFaces []int
	for i := 0; i < len(*faces); i++ {
		face := (*faces)[i]
		toSupport := support.Sub(face.Points[0])
		if toSupport.Dot(face.Normal) > 0 {
			visibleFaces = append(visibleFaces, i)
		}
	}

	// Safety: don't remove all faces
	if len(visibleFaces) >= len(*faces) {
		visibleFaces = []int{closestIndex}
	}

	// Find boundary edges
	edges := findBoundaryEdges(*faces, visibleFaces)

	// Remove visible faces
	for i := len(visibleFaces) - 1; i >= 0; i-- {
		index := visibleFaces[i]
		facePool.Put((*faces)[index])
		*faces = append((*faces)[:index], (*faces)[index+1:]...)
	}

	// Create new faces using the centroid as reference
	for _, edge := range edges {
		newFace := createFaceOutward(edge.A, edge.B, support, centroid)
		*faces = append(*faces, newFace)
	}

	// Safety check
	if len(*faces) == 0 {
		*faces = []*Face{
			{
				Points:   [3]mgl64.Vec3{support, support, support},
				Normal:   mgl64.Vec3{0, 1, 0},
				Distance: 0.0001,
			},
		}
	}
}

func findBoundaryEdges(faces []*Face, visibleIndices []int) []Edge {
	// Create a set of visible face indices for quick lookup
	visibleSet := make(map[int]bool)
	for _, idx := range visibleIndices {
		visibleSet[idx] = true
	}

	// Count how many times each edge appears in visible faces
	edgeCount := make(map[Edge]int)

	for _, idx := range visibleIndices {
		face := faces[idx]

		// Each triangular face has 3 edges
		edges := [3]Edge{
			{face.Points[0], face.Points[1]},
			{face.Points[1], face.Points[2]},
			{face.Points[2], face.Points[0]},
		}

		for _, edge := range edges {
			// Normalize edge (ensure consistent ordering)
			normalizedEdge := normalizeEdge(edge)
			edgeCount[normalizedEdge]++
		}
	}

	// Boundary edges appear exactly once (shared with only one visible face)
	var boundaryEdges []Edge
	for edge, count := range edgeCount {
		if count == 1 {
			boundaryEdges = append(boundaryEdges, edge)
		}
	}

	return boundaryEdges
}

func normalizeEdge(edge Edge) Edge {
	// Ensure consistent edge representation (A < B lexicographically)
	// This allows us to detect duplicate edges regardless of order
	if compareVec3(edge.A, edge.B) > 0 {
		return Edge{edge.B, edge.A}
	}
	return edge
}

func compareVec3(a, b mgl64.Vec3) int {
	// Compare vectors lexicographically (x, then y, then z)
	if a[0] != b[0] {
		if a[0] < b[0] {
			return -1
		}
		return 1
	}
	if a[1] != b[1] {
		if a[1] < b[1] {
			return -1
		}
		return 1
	}
	if a[2] != b[2] {
		if a[2] < b[2] {
			return -1
		}
		return 1
	}
	return 0
}
