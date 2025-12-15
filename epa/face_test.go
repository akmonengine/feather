package epa

import (
	"fmt"
	"math"
	"testing"

	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// Helper functions for testing
func vec3ApproxEqual(a, b mgl64.Vec3, tolerance float64) bool {
	return math.Abs(a.X()-b.X()) < tolerance &&
		math.Abs(a.Y()-b.Y()) < tolerance &&
		math.Abs(a.Z()-b.Z()) < tolerance
}

func isNormalized(v mgl64.Vec3, tolerance float64) bool {
	length := v.Len()
	return math.Abs(length-1.0) < tolerance
}

// normalizeEdge normalizes an edge so that A < B lexicographically
// This is the same logic used in PolytopeBuilder.findBoundaryEdges
func normalizeEdge(edge Edge) Edge {
	a, b := edge.A, edge.B
	if compareVec3(a, b) > 0 {
		return Edge{A: b, B: a}
	}
	return Edge{A: a, B: b}
}

// TestCompareVec3 tests lexicographic comparison of vectors
func TestCompareVec3(t *testing.T) {
	tests := []struct {
		name     string
		a        mgl64.Vec3
		b        mgl64.Vec3
		expected int
	}{
		{
			name:     "equal vectors",
			a:        mgl64.Vec3{1, 2, 3},
			b:        mgl64.Vec3{1, 2, 3},
			expected: 0,
		},
		{
			name:     "a < b on x",
			a:        mgl64.Vec3{1, 2, 3},
			b:        mgl64.Vec3{2, 2, 3},
			expected: -1,
		},
		{
			name:     "a > b on x",
			a:        mgl64.Vec3{2, 2, 3},
			b:        mgl64.Vec3{1, 2, 3},
			expected: 1,
		},
		{
			name:     "a < b on y (x equal)",
			a:        mgl64.Vec3{1, 1, 3},
			b:        mgl64.Vec3{1, 2, 3},
			expected: -1,
		},
		{
			name:     "a > b on y (x equal)",
			a:        mgl64.Vec3{1, 3, 3},
			b:        mgl64.Vec3{1, 2, 3},
			expected: 1,
		},
		{
			name:     "a < b on z (x,y equal)",
			a:        mgl64.Vec3{1, 2, 2},
			b:        mgl64.Vec3{1, 2, 3},
			expected: -1,
		},
		{
			name:     "a > b on z (x,y equal)",
			a:        mgl64.Vec3{1, 2, 4},
			b:        mgl64.Vec3{1, 2, 3},
			expected: 1,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := compareVec3(tt.a, tt.b)
			if result != tt.expected {
				t.Errorf("compareVec3(%v, %v) = %d, want %d", tt.a, tt.b, result, tt.expected)
			}
		})
	}
}

// TestNormalizeEdge tests edge normalization
func TestNormalizeEdge(t *testing.T) {
	tests := []struct {
		name     string
		edge     Edge
		expected Edge
	}{
		{
			name:     "already normalized (A < B)",
			edge:     Edge{A: mgl64.Vec3{0, 0, 0}, B: mgl64.Vec3{1, 0, 0}},
			expected: Edge{A: mgl64.Vec3{0, 0, 0}, B: mgl64.Vec3{1, 0, 0}},
		},
		{
			name:     "needs swap (A > B)",
			edge:     Edge{A: mgl64.Vec3{1, 0, 0}, B: mgl64.Vec3{0, 0, 0}},
			expected: Edge{A: mgl64.Vec3{0, 0, 0}, B: mgl64.Vec3{1, 0, 0}},
		},
		{
			name:     "same point",
			edge:     Edge{A: mgl64.Vec3{1, 1, 1}, B: mgl64.Vec3{1, 1, 1}},
			expected: Edge{A: mgl64.Vec3{1, 1, 1}, B: mgl64.Vec3{1, 1, 1}},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := normalizeEdge(tt.edge)
			if !vec3ApproxEqual(result.A, tt.expected.A, 1e-9) || !vec3ApproxEqual(result.B, tt.expected.B, 1e-9) {
				t.Errorf("normalizeEdge(%v) = %v, want %v", tt.edge, result, tt.expected)
			}
		})
	}
}

// TestCreateFaceOutward tests face creation with outward normal
func TestCreateFaceOutward(t *testing.T) {
	tests := []struct {
		name          string
		a, b, c       mgl64.Vec3
		oppositePoint mgl64.Vec3
		checkNormal   bool // whether to check normal direction
	}{
		{
			name:          "triangle on xy plane, opposite below",
			a:             mgl64.Vec3{1, 0, 0},
			b:             mgl64.Vec3{0, 1, 0},
			c:             mgl64.Vec3{0, 0, 0},
			oppositePoint: mgl64.Vec3{0, 0, -1},
			checkNormal:   true,
		},
		{
			name:          "triangle on xz plane",
			a:             mgl64.Vec3{1, 0, 0},
			b:             mgl64.Vec3{0, 0, 1},
			c:             mgl64.Vec3{0, 0, 0},
			oppositePoint: mgl64.Vec3{0, -1, 0},
			checkNormal:   true,
		},
		{
			name:          "degenerate triangle (collinear points)",
			a:             mgl64.Vec3{0, 0, 0},
			b:             mgl64.Vec3{1, 0, 0},
			c:             mgl64.Vec3{2, 0, 0},
			oppositePoint: mgl64.Vec3{0, 1, 0},
			checkNormal:   false, // degenerate case
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Use PolytopeBuilder to create face
			builder := &PolytopeBuilder{}
			face := builder.createFaceOutward(tt.a, tt.b, tt.c, tt.oppositePoint)

			// Check that points are stored correctly
			if !vec3ApproxEqual(face.Points[0], tt.a, 1e-9) {
				t.Errorf("face.Points[0] = %v, want %v", face.Points[0], tt.a)
			}
			if !vec3ApproxEqual(face.Points[1], tt.b, 1e-9) {
				t.Errorf("face.Points[1] = %v, want %v", face.Points[1], tt.b)
			}
			if !vec3ApproxEqual(face.Points[2], tt.c, 1e-9) {
				t.Errorf("face.Points[2] = %v, want %v", face.Points[2], tt.c)
			}

			if tt.checkNormal {
				// Check that normal is normalized
				if !isNormalized(face.Normal, 1e-6) {
					t.Errorf("normal is not normalized: length = %v", face.Normal.Len())
				}

				// Check that normal points away from opposite point
				toOpposite := tt.oppositePoint.Sub(tt.a)
				dotProduct := face.Normal.Dot(toOpposite)
				if dotProduct > 0 {
					t.Errorf("normal points toward opposite point: dot = %v (should be <= 0)", dotProduct)
				}

				// Check that distance is positive
				if face.Distance < 0 {
					t.Errorf("distance is negative: %v", face.Distance)
				}

				// Distance should be at least the minimum threshold
				if face.Distance < 0.0001 {
					t.Logf("distance clamped to minimum: %v", face.Distance)
				}
			} else {
				// Degenerate case should have default values
				if face.Distance < 0.0001 {
					t.Logf("degenerate triangle detected, distance set to minimum")
				}
			}
		})
	}
}

// TestBuildInitialFaces tests initial tetrahedron face creation
func TestBuildInitialFaces(t *testing.T) {
	tests := []struct {
		name         string
		simplex      []mgl64.Vec3
		minFaces     int
		maxFaces     int
		expectFilter bool // whether we expect filtering of degenerate faces
	}{
		{
			name: "regular tetrahedron",
			simplex: []mgl64.Vec3{
				{1, 0, 0},
				{0, 1, 0},
				{0, 0, 1},
				{0, 0, 0},
			},
			minFaces:     3,
			maxFaces:     4,
			expectFilter: false,
		},
		{
			name: "flat tetrahedron (4 coplanar points)",
			simplex: []mgl64.Vec3{
				{0, 0, 0},
				{1, 0, 0},
				{0, 1, 0},
				{0.5, 0.5, 0},
			},
			minFaces:     3, // Safety returns all 4 if < 3 after filtering
			maxFaces:     4,
			expectFilter: true,
		},
		{
			name: "origin-centered tetrahedron",
			simplex: []mgl64.Vec3{
				{1, 1, 1},
				{-1, -1, 1},
				{-1, 1, -1},
				{1, -1, -1},
			},
			minFaces:     3,
			maxFaces:     4,
			expectFilter: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Create a simplex for testing
			simplex := &gjk.Simplex{}
			// Copy points to simplex
			for i, point := range tt.simplex {
				if i < len(simplex.Points) {
					simplex.Points[i] = point
				}
			}
			simplex.Count = len(tt.simplex)

			// Use PolytopeBuilder to build initial faces
			builder := &PolytopeBuilder{}
			err := builder.BuildInitialFaces(simplex)
			if err != nil {
				t.Fatalf("BuildInitialFaces failed: %v", err)
			}

			// Get the faces from the builder
			faces := builder.faces[:len(builder.faces)]

			// Check number of faces
			if len(faces) < tt.minFaces || len(faces) > tt.maxFaces {
				t.Errorf("BuildInitialFaces() returned %d faces, want between %d and %d",
					len(faces), tt.minFaces, tt.maxFaces)
			}

			// All faces should have valid distance
			for i, face := range faces {
				if face.Distance < 0 {
					t.Errorf("face %d has negative distance: %v", i, face.Distance)
				}

				// Check that normal is normalized (unless degenerate)
				if face.Distance >= 0.0001 && !isNormalized(face.Normal, 1e-6) {
					t.Errorf("face %d has non-normalized normal: length = %v", i, face.Normal.Len())
				}
			}
		})
	}
}

// TestFindClosestFaceIndex tests finding the face closest to origin
func TestFindClosestFaceIndex(t *testing.T) {
	tests := []struct {
		name          string
		faces         []Face
		expectedIndex int
	}{
		{
			name: "single face",
			faces: []Face{
				{Distance: 1.0},
			},
			expectedIndex: 0,
		},
		{
			name: "closest is first",
			faces: []Face{
				{Distance: 0.5},
				{Distance: 1.0},
				{Distance: 2.0},
			},
			expectedIndex: 0,
		},
		{
			name: "closest is middle",
			faces: []Face{
				{Distance: 2.0},
				{Distance: 0.3},
				{Distance: 1.0},
			},
			expectedIndex: 1,
		},
		{
			name: "closest is last",
			faces: []Face{
				{Distance: 2.0},
				{Distance: 1.0},
				{Distance: 0.1},
			},
			expectedIndex: 2,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Use PolytopeBuilder to find closest face
			builder := &PolytopeBuilder{}
			// Copy faces to builder using append
			builder.faces = append(builder.faces, tt.faces...)

			result := builder.FindClosestFaceIndex()
			if result != tt.expectedIndex {
				t.Errorf("FindClosestFaceIndex() = %d, want %d", result, tt.expectedIndex)
			}
		})
	}
}

// TestFindBoundaryEdges tests boundary edge detection
func TestFindBoundaryEdges(t *testing.T) {
	tests := []struct {
		name           string
		faces          []Face
		visibleIndices []int
		minEdges       int
		maxEdges       int
	}{
		{
			name: "single visible triangle",
			faces: []Face{
				{
					Points: [3]mgl64.Vec3{
						{0, 0, 0},
						{1, 0, 0},
						{0, 1, 0},
					},
				},
			},
			visibleIndices: []int{0},
			minEdges:       3, // All 3 edges are boundary
			maxEdges:       3,
		},
		{
			name: "two adjacent triangles, one visible",
			faces: []Face{
				{
					Points: [3]mgl64.Vec3{
						{0, 0, 0},
						{1, 0, 0},
						{0, 1, 0},
					},
				},
				{
					Points: [3]mgl64.Vec3{
						{0, 0, 0},
						{0, 1, 0},
						{0, 0, 1},
					},
				},
			},
			visibleIndices: []int{0},
			minEdges:       2, // Two edges are unique to face 0
			maxEdges:       3,
		},
		{
			name: "tetrahedron, two opposite faces visible",
			faces: []Face{
				{
					Points: [3]mgl64.Vec3{
						{0, 0, 0},
						{1, 0, 0},
						{0, 1, 0},
					},
				},
				{
					Points: [3]mgl64.Vec3{
						{0, 0, 1},
						{1, 0, 1},
						{0, 1, 1},
					},
				},
			},
			visibleIndices: []int{0, 1},
			minEdges:       6, // All edges are boundary (no shared edges)
			maxEdges:       6,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Use PolytopeBuilder to find boundary edges
			builder := &PolytopeBuilder{}

			// Copy faces to builder using append
			builder.faces = append(builder.faces, tt.faces...)
			// Copy visible indices to builder using append
			builder.visibleIndices = append(builder.visibleIndices, tt.visibleIndices...)

			// Find boundary edges
			err := builder.findBoundaryEdges()
			if err != nil {
				t.Fatalf("findBoundaryEdges failed: %v", err)
			}

			// Get edges from builder
			edges := builder.edges[:len(builder.edges)]

			if len(edges) < tt.minEdges || len(edges) > tt.maxEdges {
				t.Errorf("findBoundaryEdges() returned %d edges, want between %d and %d",
					len(edges), tt.minEdges, tt.maxEdges)
			}

			// All boundary edges should be normalized
			for _, edge := range edges {
				normalized := normalizeEdge(Edge{A: edge.A, B: edge.B})
				if !vec3ApproxEqual(edge.A, normalized.A, 1e-9) || !vec3ApproxEqual(edge.B, normalized.B, 1e-9) {
					t.Logf("edge not in normalized form: %v (normalized: %v)", Edge{A: edge.A, B: edge.B}, normalized)
				}
			}
		})
	}
}

// TestAddPointAndRebuildFaces tests polytope expansion
func TestAddPointAndRebuildFaces(t *testing.T) {
	t.Run("add point to tetrahedron", func(t *testing.T) {
		// Start with a simple tetrahedron
		initialFaces := []Face{
			{
				Points:   [3]mgl64.Vec3{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}},
				Normal:   mgl64.Vec3{0, 0, 1},
				Distance: 0.1,
			},
			{
				Points:   [3]mgl64.Vec3{{0, 0, 1}, {1, 0, 1}, {0, 1, 1}},
				Normal:   mgl64.Vec3{0, 0, -1},
				Distance: 0.1,
			},
			{
				Points:   [3]mgl64.Vec3{{0, 0, 0}, {0, 0, 1}, {1, 0, 0}},
				Normal:   mgl64.Vec3{0, 1, 0},
				Distance: 0.1,
			},
			{
				Points:   [3]mgl64.Vec3{{0, 1, 0}, {0, 1, 1}, {1, 1, 0}},
				Normal:   mgl64.Vec3{0, -1, 0},
				Distance: 0.1,
			},
		}

		// Use PolytopeBuilder for polytope expansion
		builder := &PolytopeBuilder{}

		// Copy initial faces to builder using append
		builder.faces = append(builder.faces, initialFaces...)

		support := mgl64.Vec3{2, 0.5, 0.5}
		closestIndex := 0

		err := builder.AddPointAndRebuildFaces(support, closestIndex)
		if err != nil {
			t.Fatalf("AddPointAndRebuildFaces failed: %v", err)
		}

		// Get faces from builder
		faces := builder.faces[:len(builder.faces)]

		// Should still have faces after rebuild
		if len(faces) == 0 {
			t.Error("AddPointAndRebuildFaces() resulted in no faces (safety check failed)")
		}

		// Check that all faces have valid normals and distances
		for i, face := range faces {
			if face.Distance < 0 {
				t.Errorf("face %d has negative distance after rebuild: %v", i, face.Distance)
			}
		}
	})

	t.Run("remove all faces safety check", func(t *testing.T) {
		// Single face that would be removed
		initialFaces := []Face{
			{
				Points:   [3]mgl64.Vec3{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}},
				Normal:   mgl64.Vec3{0, 0, 1},
				Distance: 0.5,
			},
		}

		// Use PolytopeBuilder for polytope expansion
		builder := &PolytopeBuilder{}

		// Copy initial faces to builder using append
		builder.faces = append(builder.faces, initialFaces...)

		// Point that would make all faces visible
		support := mgl64.Vec3{0, 0, 2}
		closestIndex := 0

		err := builder.AddPointAndRebuildFaces(support, closestIndex)
		if err != nil {
			t.Fatalf("AddPointAndRebuildFaces failed: %v", err)
		}

		// Get faces from builder
		faces := builder.faces[:len(builder.faces)]

		// Safety check should ensure at least one face remains
		if len(faces) == 0 {
			t.Error("safety check failed: no faces remain after rebuild")
		}
	})

	t.Run("no visible faces case", func(t *testing.T) {
		initialFaces := []Face{
			{
				Points:   [3]mgl64.Vec3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
				Normal:   mgl64.Vec3{1, 1, 1}.Normalize(),
				Distance: 1.0,
			},
		}

		// Use PolytopeBuilder for polytope expansion
		builder := &PolytopeBuilder{}

		// Copy initial faces to builder using append
		builder.faces = append(builder.faces, initialFaces...)
		// Point behind the face (not visible)
		support := mgl64.Vec3{-1, -1, -1}
		closestIndex := 0

		initialLen := len(builder.faces)
		err := builder.AddPointAndRebuildFaces(support, closestIndex)
		if err != nil {
			t.Fatalf("AddPointAndRebuildFaces failed: %v", err)
		}

		// Get faces from builder
		faces := builder.faces[:len(builder.faces)]

		// Should have modified the polytope
		if len(faces) == 0 {
			t.Error("no faces after rebuild")
		}

		// Verify we have at least as many faces as before (or the safety fallback)
		if len(faces) < initialLen && len(faces) != 1 {
			t.Logf("face count changed from %d to %d", initialLen, len(faces))
		}
	})
}

// Benchmark tests
func BenchmarkCreateFaceOutward(b *testing.B) {
	a := mgl64.Vec3{1, 0, 0}
	c := mgl64.Vec3{0, 1, 0}
	d := mgl64.Vec3{0, 0, 0}
	opposite := mgl64.Vec3{0, 0, 1}
	builder := &PolytopeBuilder{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		builder.createFaceOutward(a, c, d, opposite)
	}
}

func BenchmarkBuildInitialFaces(b *testing.B) {
	simplex := &gjk.Simplex{}
	simplex.Points[0] = mgl64.Vec3{1, 0, 0}
	simplex.Points[1] = mgl64.Vec3{0, 1, 0}
	simplex.Points[2] = mgl64.Vec3{0, 0, 1}
	simplex.Points[3] = mgl64.Vec3{0, 0, 0}
	simplex.Count = 4
	builder := &PolytopeBuilder{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		builder.Reset()
		err := builder.BuildInitialFaces(simplex)

		if err != nil {
			fmt.Printf("error building initial faces: %v", err)
		}
	}
}

func BenchmarkFindBoundaryEdges(b *testing.B) {
	faces := []Face{
		{Points: [3]mgl64.Vec3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}},
		{Points: [3]mgl64.Vec3{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
		{Points: [3]mgl64.Vec3{{0, 0, 0}, {0, 0, 1}, {1, 0, 0}}},
		{Points: [3]mgl64.Vec3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
	}
	visibleIndices := []int{0, 1}
	builder := &PolytopeBuilder{}

	// Setup builder
	for i, face := range faces {
		if i < len(builder.faces) {
			builder.faces[i] = face
		}
	}
	for i, idx := range visibleIndices {
		if i < len(builder.visibleIndices) {
			builder.visibleIndices[i] = idx
		}
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		err := builder.findBoundaryEdges()
		if err != nil {
			fmt.Printf("error finding boundary edges: %v", err)
		}
	}
}

func BenchmarkAddPointAndRebuildFaces(b *testing.B) {
	support := mgl64.Vec3{2, 0.5, 0.5}
	closestIndex := 0
	builder := &PolytopeBuilder{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		// Setup initial faces
		builder.Reset()
		builder.faces = append(builder.faces,
			Face{Points: [3]mgl64.Vec3{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}, Normal: mgl64.Vec3{0, 0, 1}, Distance: 0.1},
			Face{Points: [3]mgl64.Vec3{{0, 0, 1}, {1, 0, 1}, {0, 1, 1}}, Normal: mgl64.Vec3{0, 0, -1}, Distance: 0.1},
			Face{Points: [3]mgl64.Vec3{{0, 0, 0}, {0, 0, 1}, {1, 0, 0}}, Normal: mgl64.Vec3{0, 1, 0}, Distance: 0.1})
		b.StartTimer()

		err := builder.AddPointAndRebuildFaces(support, closestIndex)
		if err != nil {
			fmt.Printf("error adding faces: %v", err)
		}
	}
}
