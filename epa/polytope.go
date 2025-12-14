package epa

import (
	"fmt"
	"math"
	"sync"

	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// PolytopeBuilder manages polytope expansion with dynamic buffers and initial capacity.
type PolytopeBuilder struct {
	// Face storage - dynamic with initial capacity
	// Stores all faces in the current polytope
	faces []Face

	// Point deduplication buffer for centroid calculation
	// Uses sorted slice with binary search for deduplication
	uniquePoints []mgl64.Vec3

	// Edge tracking for boundary detection
	// Normalized edges (A < B) with occurrence count
	edges []EdgeEntry

	// Visible face tracking
	visibleIndices []int

	// Temporary workspace for face construction
	tempFace Face
}

// EdgeEntry represents an edge with occurrence counting for boundary detection.
// An edge is a boundary edge if it appears exactly once (count == 1).
// Edges are normalized so A < B lexicographically for consistent deduplication.
type EdgeEntry struct {
	A, B  mgl64.Vec3 // Edge vertices (normalized: A < B)
	Count int        // Occurrence count (1 = boundary edge, 2+ = internal edge)
}

// polytopeBuilderPool is the single sync.Pool for PolytopeBuilder instances.
// This eliminates allocation of builder structures during EPA iterations.
var polytopeBuilderPool = sync.Pool{
	New: func() interface{} {
		return &PolytopeBuilder{
			faces:          make([]Face, 0, polytopeInitialCapacity),
			uniquePoints:   make([]mgl64.Vec3, 0, polytopeInitialCapacity),
			edges:          make([]EdgeEntry, 0, polytopeInitialCapacity),
			visibleIndices: make([]int, 0, polytopeInitialCapacity),
		}
	},
}

// Reset prepares the builder for reuse by clearing all slices.
// This allows the builder to be reused from the pool without reallocation.
func (b *PolytopeBuilder) Reset() {
	b.faces = b.faces[:0]
	b.uniquePoints = b.uniquePoints[:0]
	b.edges = b.edges[:0]
	b.visibleIndices = b.visibleIndices[:0]
}

// BuildInitialFaces creates the initial polytope from a GJK tetrahedron simplex.
// Creates 4 triangular faces from the 4 simplex points, filtering degenerate faces.
//
// Returns error if simplex is invalid (count != 4).
func (b *PolytopeBuilder) BuildInitialFaces(simplex *gjk.Simplex) error {
	if simplex.Count != 4 {
		return fmt.Errorf("invalid simplex count: %d (expected 4)", simplex.Count)
	}

	p0, p1, p2, p3 := simplex.Points[0], simplex.Points[1], simplex.Points[2], simplex.Points[3]

	// Create 4 candidate faces (one for each tetrahedron face)
	// Each face is defined by 3 points + the opposite point for normal orientation
	candidateFaces := [4]Face{
		b.createFaceOutward(p0, p1, p2, p3), // Face ABC, opposite point is D
		b.createFaceOutward(p0, p2, p3, p1), // Face ACD, opposite point is B
		b.createFaceOutward(p0, p3, p1, p2), // Face ADB, opposite point is C
		b.createFaceOutward(p1, p3, p2, p0), // Face BDC, opposite point is A
	}

	// Filter valid faces (distance >= EPAMinFaceDistance)
	for i := 0; i < 4; i++ {
		if candidateFaces[i].Distance >= EPAMinFaceDistance {
			b.faces = append(b.faces, candidateFaces[i])
		}
	}

	// Safety: need at least 3 faces for valid polytope
	if len(b.faces) < 3 {
		// Keep all faces for degenerate case
		b.faces = b.faces[:0]
		for i := 0; i < 4; i++ {
			b.faces = append(b.faces, candidateFaces[i])
		}
	}

	return nil
}

// createFaceOutward creates a Face with normal pointing outward from the polytope.
// Uses the opposite point as a reference to determine the correct normal orientation.
//
// Algorithm:
//  1. Compute normal via cross product: (b-a) × (c-a)
//  2. Check if normal points toward opposite point (inward) → flip if needed
//  3. Ensure distance is positive (normal away from origin)
//  4. Snap near-zero components for numerical stability
func (b *PolytopeBuilder) createFaceOutward(p0, p1, p2, oppositePoint mgl64.Vec3) Face {
	var face Face
	face.Points = [3]mgl64.Vec3{p0, p1, p2}

	// Calculate two edges of the triangle
	edge1 := p1.Sub(p0)
	edge2 := p2.Sub(p0)

	// Normal via cross product (right-hand rule)
	normal := edge1.Cross(edge2)

	// Normalize the normal
	normalLength := math.Sqrt(normal.Dot(normal))
	if normalLength < 1e-8 {
		// Degenerate triangle (zero area)
		face.Normal = mgl64.Vec3{0, 1, 0}
		face.Distance = EPAMinFaceDistance
		return face
	}
	normal = normal.Mul(1.0 / normalLength)

	// === CRITICAL: Ensure normal points OUTWARD ===
	// Vector from face point p0 to the opposite point
	toOpposite := oppositePoint.Sub(p0)

	// If normal points TOWARDS the opposite point, it's pointing INWARD
	// We need to flip it to point OUTWARD
	if normal.Dot(toOpposite) > 0 {
		normal = normal.Mul(-1)
	}

	// Calculate distance from origin to the plane
	distance := p0.Dot(normal)

	// Distance should be positive (normal points away from origin)
	if distance < 0 {
		normal = normal.Mul(-1)
		distance = -distance
	}

	// Force minimum distance to avoid degenerate cases
	if distance < EPAMinFaceDistance {
		distance = EPAMinFaceDistance
	}

	face.Normal = snapNormalToAxis(normal)
	face.Distance = distance

	return face
}

// FindClosestFaceIndex returns the index of the face closest to the origin.
// Returns -1 if no faces exist.
func (b *PolytopeBuilder) FindClosestFaceIndex() int {
	if len(b.faces) == 0 {
		return -1
	}

	closestIndex := 0
	minDistance := b.faces[0].Distance

	for i := 1; i < len(b.faces); i++ {
		if b.faces[i].Distance < minDistance {
			closestIndex = i
			minDistance = b.faces[i].Distance
		}
	}

	return closestIndex
}

// calculateCentroid computes the centroid (average position) of all unique points
// in the current polytope. Uses dynamic slice with binary search for
// point deduplication.
func (b *PolytopeBuilder) calculateCentroid() mgl64.Vec3 {
	// Collect unique points using sorted slice
	b.uniquePoints = b.uniquePoints[:0] // Clear existing points

	for i := 0; i < len(b.faces); i++ {
		face := &b.faces[i]
		for j := 0; j < 3; j++ {
			point := face.Points[j]

			// Binary search for insertion point
			insertIdx := b.findPointInsertionIndex(point)

			// Check if point already exists
			if insertIdx < len(b.uniquePoints) && vec3Equal(b.uniquePoints[insertIdx], point) {
				continue // Already have this point
			}

			// Insert point (shift slice right)
			if insertIdx < len(b.uniquePoints) {
				// Make sure we have enough capacity
				if cap(b.uniquePoints) == len(b.uniquePoints) {
					newCap := len(b.uniquePoints) * 2
					if newCap == 0 {
						newCap = polytopeInitialCapacity
					}
					newPoints := make([]mgl64.Vec3, len(b.uniquePoints), newCap)
					copy(newPoints, b.uniquePoints)
					b.uniquePoints = newPoints
				}

				// Shift elements to make space
				b.uniquePoints = append(b.uniquePoints, mgl64.Vec3{}) // Add space
				copy(b.uniquePoints[insertIdx+1:], b.uniquePoints[insertIdx:])
				b.uniquePoints[insertIdx] = point
			}
		}
	}

	// Calculate average
	if len(b.uniquePoints) == 0 {
		return mgl64.Vec3{0, 0, 0}
	}

	sum := mgl64.Vec3{0, 0, 0}
	for i := 0; i < len(b.uniquePoints); i++ {
		sum = sum.Add(b.uniquePoints[i])
	}

	return sum.Mul(1.0 / float64(len(b.uniquePoints)))
}

// findPointInsertionIndex performs binary search to find the correct insertion
// index for a point in the sorted uniquePoints array.
func (b *PolytopeBuilder) findPointInsertionIndex(point mgl64.Vec3) int {
	left, right := 0, len(b.uniquePoints)

	for left < right {
		mid := (left + right) / 2
		cmp := compareVec3(b.uniquePoints[mid], point)

		if cmp < 0 {
			left = mid + 1
		} else {
			right = mid
		}
	}

	return left
}

// findBoundaryEdges identifies boundary edges from visible faces.
// A boundary edge appears exactly once (count == 1), while internal edges
// appear twice and are filtered out.
//
// Uses dynamic slice with linear search for edge tracking.
func (b *PolytopeBuilder) findBoundaryEdges() error {
	b.edges = b.edges[:0] // Clear existing edges

	// Collect all edges from visible faces
	for i := 0; i < len(b.visibleIndices); i++ {
		faceIdx := b.visibleIndices[i]
		face := &b.faces[faceIdx]

		// Three edges per triangle
		edges := [3][2]mgl64.Vec3{
			{face.Points[0], face.Points[1]},
			{face.Points[1], face.Points[2]},
			{face.Points[2], face.Points[0]},
		}

		for _, edge := range edges {
			// Normalize edge (A < B lexicographically)
			edgeA, edgeB := edge[0], edge[1]
			if compareVec3(edgeA, edgeB) > 0 {
				edgeA, edgeB = edgeB, edgeA
			}

			// Find or insert edge
			edgeIdx := b.findEdgeIndex(edgeA, edgeB)

			if edgeIdx >= 0 {
				// Edge exists, increment count
				b.edges[edgeIdx].Count++
			} else {
				// New edge - no buffer overflow possible with dynamic slices
				b.edges = append(b.edges, EdgeEntry{
					A:     edgeA,
					B:     edgeB,
					Count: 1,
				})
			}
		}
	}

	return nil
}

// findEdgeIndex performs linear search for an edge in the edges buffer.
// Returns the index if found, -1 otherwise.
// Linear search is efficient for small edge counts (typically < 30).
func (b *PolytopeBuilder) findEdgeIndex(edgeA, edgeB mgl64.Vec3) int {
	for i := 0; i < len(b.edges); i++ {
		edge := &b.edges[i]
		if vec3Equal(edge.A, edgeA) && vec3Equal(edge.B, edgeB) {
			return i
		}
	}
	return -1
}

// findVisibleFaces populates visibleIndices with faces visible from the support point.
// A face is visible if the vector from the face to the support point points in the
// same direction as the face normal (dot product > 0).
func (b *PolytopeBuilder) findVisibleFaces(support mgl64.Vec3) {
	b.visibleIndices = b.visibleIndices[:0] // Clear existing indices

	for i := 0; i < len(b.faces); i++ {
		face := &b.faces[i]
		toSupport := support.Sub(face.Points[0])

		if toSupport.Dot(face.Normal) > 0 {
			b.visibleIndices = append(b.visibleIndices, i)
		}
	}
}

// removeVisibleFaces removes faces marked in visibleIndices using swap-with-last pattern.
// Indices are sorted descending to prevent index invalidation during removal.
func (b *PolytopeBuilder) removeVisibleFaces() {
	// Sort indices descending to remove from end first
	for i := 0; i < len(b.visibleIndices)-1; i++ {
		for j := i + 1; j < len(b.visibleIndices); j++ {
			if b.visibleIndices[i] < b.visibleIndices[j] {
				b.visibleIndices[i], b.visibleIndices[j] = b.visibleIndices[j], b.visibleIndices[i]
			}
		}
	}

	// Remove faces using swap-with-last
	for i := 0; i < len(b.visibleIndices); i++ {
		idx := b.visibleIndices[i]

		if idx < len(b.faces) {
			// Swap with last element
			b.faces[idx] = b.faces[len(b.faces)-1]
			b.faces = b.faces[:len(b.faces)-1]
		}
	}
}

// addBoundaryFaces creates new faces connecting boundary edges to the support point.
// Only processes edges with count == 1 (boundary edges).
func (b *PolytopeBuilder) addBoundaryFaces(support mgl64.Vec3, centroid mgl64.Vec3) error {
	// Iterate through edges with count == 1 (boundary)
	for i := 0; i < len(b.edges); i++ {
		edge := &b.edges[i]

		if edge.Count != 1 {
			continue // Not a boundary edge
		}

		// Create new face
		newFace := b.createFaceOutward(edge.A, edge.B, support, centroid)

		// Add to slice - no buffer overflow possible with dynamic slices
		b.faces = append(b.faces, newFace)
	}

	return nil
}

// AddPointAndRebuildFaces expands the polytope by adding a support point.
// This is the main EPA expansion step that:
//  1. Finds visible faces from the support point
//  2. Identifies boundary edges of the visible region
//  3. Removes visible faces
//  4. Creates new faces connecting boundary edges to the support point
//
// All operations use fixed buffers for zero allocations.
func (b *PolytopeBuilder) AddPointAndRebuildFaces(support mgl64.Vec3, closestIndex int) error {
	// Calculate centroid (zero allocations)
	centroid := b.calculateCentroid()

	// Find visible faces
	b.findVisibleFaces(support)

	// Safety: don't remove all faces
	if len(b.visibleIndices) >= len(b.faces) {
		b.visibleIndices = b.visibleIndices[:0]
		b.visibleIndices = append(b.visibleIndices, closestIndex)
	}

	// Find boundary edges (zero allocations)
	if err := b.findBoundaryEdges(); err != nil {
		return err
	}

	// Remove visible faces
	b.removeVisibleFaces()

	// Add new faces from boundary
	if err := b.addBoundaryFaces(support, centroid); err != nil {
		return err
	}

	// Safety check: ensure at least one face exists
	if len(b.faces) == 0 {
		// Create fallback face
		b.faces = append(b.faces, Face{
			Points:   [3]mgl64.Vec3{support, support, support},
			Normal:   mgl64.Vec3{0, 1, 0},
			Distance: EPAMinFaceDistance,
		})
	}

	return nil
}

// GetClosestFace returns a pointer to the closest face for EPA result.
// Returns nil if no faces exist.
func (b *PolytopeBuilder) GetClosestFace() *Face {
	if len(b.faces) == 0 {
		return nil
	}
	idx := b.FindClosestFaceIndex()
	return &b.faces[idx]
}

// vec3Equal performs exact equality check for point deduplication.
// Uses exact float comparison (no epsilon) since we need exact deduplication.
func vec3Equal(a, b mgl64.Vec3) bool {
	return a[0] == b[0] && a[1] == b[1] && a[2] == b[2]
}
