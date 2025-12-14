package epa

import (
	"github.com/go-gl/mathgl/mgl64"
)

// Face represents a triangular face of the polytope in EPA.
// Each face has 3 vertices, an outward-pointing normal, and distance to origin.
//
// This struct is now used by PolytopeBuilder for zero-allocation EPA.
// The old pointer-based approach with facePool has been replaced.
type Face struct {
	Points   [3]mgl64.Vec3 // The 3 vertices of the triangle
	Normal   mgl64.Vec3    // Outward-pointing normal
	Distance float64       // Distance from origin to the face plane
}

// Edge represents an edge between two vertices.
// Kept for backward compatibility, but no longer used with PolytopeBuilder.
// PolytopeBuilder uses EdgeEntry instead with occurrence counting.
type Edge struct {
	A, B mgl64.Vec3
}

// compareVec3 compares two vectors lexicographically (x, then y, then z).
// Returns:
//
//	-1 if a < b
//	 0 if a == b
//	+1 if a > b
//
// Used by PolytopeBuilder for edge normalization and point deduplication.
func compareVec3(a, b mgl64.Vec3) int {
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
