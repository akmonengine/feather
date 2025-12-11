package epa

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// TestGetTangentBasis tests the creation of orthogonal tangent basis
func TestGetTangentBasis(t *testing.T) {
	tests := []struct {
		name   string
		normal mgl64.Vec3
	}{
		{
			name:   "normal_aligned_with_x",
			normal: mgl64.Vec3{1, 0, 0},
		},
		{
			name:   "normal_aligned_with_neg_x",
			normal: mgl64.Vec3{-1, 0, 0},
		},
		{
			name:   "normal_aligned_with_y",
			normal: mgl64.Vec3{0, 1, 0},
		},
		{
			name:   "normal_aligned_with_z",
			normal: mgl64.Vec3{0, 0, 1},
		},
		{
			name:   "normal_at_threshold",
			normal: mgl64.Vec3{0.9, 0.436, 0}.Normalize(),
		},
		{
			name:   "normal_diagonal",
			normal: mgl64.Vec3{1, 1, 1}.Normalize(),
		},
		{
			name:   "normal_negative_diagonal",
			normal: mgl64.Vec3{-1, -1, -1}.Normalize(),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tangent1, tangent2 := getTangentBasis(tt.normal)

			// Both tangents should be normalized
			if !isNormalized(tangent1, 1e-6) {
				t.Errorf("tangent1 is not normalized: length = %v", tangent1.Len())
			}
			if !isNormalized(tangent2, 1e-6) {
				t.Errorf("tangent2 is not normalized: length = %v", tangent2.Len())
			}

			// tangent1 perpendicular to normal
			dot1 := tangent1.Dot(tt.normal)
			if math.Abs(dot1) > 1e-6 {
				t.Errorf("tangent1 not perpendicular to normal: dot = %v", dot1)
			}

			// tangent2 perpendicular to normal
			dot2 := tangent2.Dot(tt.normal)
			if math.Abs(dot2) > 1e-6 {
				t.Errorf("tangent2 not perpendicular to normal: dot = %v", dot2)
			}

			// tangent1 perpendicular to tangent2
			dot12 := tangent1.Dot(tangent2)
			if math.Abs(dot12) > 1e-6 {
				t.Errorf("tangent1 not perpendicular to tangent2: dot = %v", dot12)
			}

			// Verify cross product: normal x tangent1 ≈ tangent2
			cross := tt.normal.Cross(tangent1)
			if !vec3ApproxEqual(cross, tangent2, 1e-6) {
				t.Errorf("normal.Cross(tangent1) = %v, want %v", cross, tangent2)
			}
		})
	}
}

// TestLineIntersectPlane tests line-plane intersection with clamping
func TestLineIntersectPlane(t *testing.T) {
	tests := []struct {
		name        string
		p1          mgl64.Vec3
		p2          mgl64.Vec3
		planePoint  mgl64.Vec3
		planeNormal mgl64.Vec3
		expected    mgl64.Vec3
	}{
		{
			name:        "perpendicular_intersection",
			p1:          mgl64.Vec3{0, -1, 0},
			p2:          mgl64.Vec3{0, 1, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 0, 0},
		},
		{
			name:        "parallel_line",
			p1:          mgl64.Vec3{0, 1, 0},
			p2:          mgl64.Vec3{1, 1, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 1, 0}, // Should return p1
		},
		{
			name:        "intersection_at_p1",
			p1:          mgl64.Vec3{0, 0, 0},
			p2:          mgl64.Vec3{0, 2, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 0, 0},
		},
		{
			name:        "intersection_at_p2",
			p1:          mgl64.Vec3{0, -1, 0},
			p2:          mgl64.Vec3{0, 0, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 0, 0},
		},
		{
			name:        "clamping_below_zero",
			p1:          mgl64.Vec3{0, 1, 0},
			p2:          mgl64.Vec3{0, 2, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 1, 0}, // t clamped to 0
		},
		{
			name:        "clamping_above_one",
			p1:          mgl64.Vec3{0, -2, 0},
			p2:          mgl64.Vec3{0, -1, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, -1, 0}, // t clamped to 1
		},
		{
			name:        "diagonal_intersection",
			p1:          mgl64.Vec3{-1, -1, -1},
			p2:          mgl64.Vec3{1, 1, 1},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 0, 0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := lineIntersectPlane(tt.p1, tt.p2, tt.planePoint, tt.planeNormal)
			if !vec3ApproxEqual(result, tt.expected, 1e-6) {
				t.Errorf("lineIntersectPlane() = %v, want %v", result, tt.expected)
			}
		})
	}
}

// TestIsLargePlane tests large plane detection
func TestIsLargePlane(t *testing.T) {
	builder := &ManifoldBuilder{}

	tests := []struct {
		name     string
		feature  [8]mgl64.Vec3
		count    int
		expected bool
	}{
		{
			name:     "not_4_points",
			feature:  [8]mgl64.Vec3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
			count:    3,
			expected: false,
		},
		{
			name:     "zero_points",
			feature:  [8]mgl64.Vec3{},
			count:    0,
			expected: false,
		},
		{
			name:     "small_feature",
			feature:  [8]mgl64.Vec3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}},
			count:    4,
			expected: false,
		},
		{
			name:     "large_plane_detected",
			feature:  [8]mgl64.Vec3{{0, 0, 0}, {200, 0, 0}, {0, 200, 0}, {200, 200, 0}},
			count:    4,
			expected: true,
		},
		{
			name:     "boundary_case_exactly_100",
			feature:  [8]mgl64.Vec3{{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
			count:    4,
			expected: false, // Should be > 100, not >= 100
		},
		{
			name:     "boundary_case_just_over_100",
			feature:  [8]mgl64.Vec3{{0, 0, 0}, {100.01, 0, 0}, {0, 0, 0}, {0, 0, 0}},
			count:    4,
			expected: true,
		},
		{
			name:     "first_pair_large",
			feature:  [8]mgl64.Vec3{{0, 0, 0}, {150, 0, 0}, {1, 0, 0}, {1, 0, 0}},
			count:    4,
			expected: true, // Early exit on i=0, j=1
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := builder.isLargePlane(&tt.feature, tt.count)
			if result != tt.expected {
				t.Errorf("isLargePlane() = %v, want %v", result, tt.expected)
			}
		})
	}
}

// TestComputeCenter tests center computation
func TestComputeCenter(t *testing.T) {
	builder := &ManifoldBuilder{}

	tests := []struct {
		name     string
		points   [8]mgl64.Vec3
		count    int
		expected mgl64.Vec3
	}{
		{
			name:     "zero_count",
			points:   [8]mgl64.Vec3{},
			count:    0,
			expected: mgl64.Vec3{0, 0, 0},
		},
		{
			name:     "single_point",
			points:   [8]mgl64.Vec3{{5, 10, 15}},
			count:    1,
			expected: mgl64.Vec3{5, 10, 15},
		},
		{
			name:     "two_points",
			points:   [8]mgl64.Vec3{{0, 0, 0}, {4, 0, 0}},
			count:    2,
			expected: mgl64.Vec3{2, 0, 0},
		},
		{
			name:     "four_points_square",
			points:   [8]mgl64.Vec3{{0, 0, 0}, {4, 0, 0}, {0, 4, 0}, {4, 4, 0}},
			count:    4,
			expected: mgl64.Vec3{2, 2, 0},
		},
		{
			name:     "four_points_3d",
			points:   [8]mgl64.Vec3{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {0, 0, 2}},
			count:    4,
			expected: mgl64.Vec3{0.5, 0.5, 0.5},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := builder.computeCenter(&tt.points, tt.count)
			if !vec3ApproxEqual(result, tt.expected, 1e-6) {
				t.Errorf("computeCenter() = %v, want %v", result, tt.expected)
			}
		})
	}
}

// TestTransformFeatureNormalShapes tests transformation for Box and Sphere
func TestTransformFeatureNormalShapes(t *testing.T) {
	builder := &ManifoldBuilder{}

	t.Run("box_transformation", func(t *testing.T) {
		// Create a box with 4 points on top face (local space)
		input := [8]mgl64.Vec3{
			{-1, 1, -1},
			{1, 1, -1},
			{1, 1, 1},
			{-1, 1, 1},
		}
		inputCount := 4

		// Transform: rotation 45° around Y, translation {5, 10, 15}
		angleY := math.Pi / 4 // 45 degrees
		transform := actor.Transform{
			Position: mgl64.Vec3{5, 10, 15},
			Rotation: mgl64.QuatRotate(angleY, mgl64.Vec3{0, 1, 0}),
		}

		box := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}

		var output [8]mgl64.Vec3
		var outputCount int

		builder.transformFeature(&input, inputCount, transform, box, &output, &outputCount)

		// Should have 4 output points
		if outputCount != 4 {
			t.Errorf("outputCount = %d, want 4", outputCount)
		}

		// Each point should be rotated and translated
		for i := 0; i < outputCount; i++ {
			rotated := transform.Rotation.Rotate(input[i])
			expected := transform.Position.Add(rotated)
			if !vec3ApproxEqual(output[i], expected, 1e-6) {
				t.Errorf("output[%d] = %v, want %v", i, output[i], expected)
			}
		}
	})

	t.Run("sphere_transformation", func(t *testing.T) {
		// Sphere has 1 support point
		input := [8]mgl64.Vec3{{0, 1, 0}}
		inputCount := 1

		transform := actor.Transform{
			Position: mgl64.Vec3{-3, 2, 7},
			Rotation: mgl64.QuatIdent(),
		}

		sphere := &actor.Sphere{Radius: 1.0}

		var output [8]mgl64.Vec3
		var outputCount int

		builder.transformFeature(&input, inputCount, transform, sphere, &output, &outputCount)

		// Should have 1 output point
		if outputCount != 1 {
			t.Errorf("outputCount = %d, want 1", outputCount)
		}

		// Point should be translated
		expected := mgl64.Vec3{-3, 3, 7}
		if !vec3ApproxEqual(output[0], expected, 1e-6) {
			t.Errorf("output[0] = %v, want %v", output[0], expected)
		}
	})
}

// TestTransformFeaturePlane tests plane special case with large corners
func TestTransformFeaturePlane(t *testing.T) {
	builder := &ManifoldBuilder{}

	t.Run("horizontal_plane", func(t *testing.T) {
		input := [8]mgl64.Vec3{} // Input ignored for Plane
		inputCount := 0

		transform := actor.Transform{
			Position: mgl64.Vec3{0, 0, 0},
			Rotation: mgl64.QuatIdent(),
		}

		plane := &actor.Plane{
			Normal:   mgl64.Vec3{0, 1, 0},
			Distance: 0,
		}

		var output [8]mgl64.Vec3
		var outputCount int

		builder.transformFeature(&input, inputCount, transform, plane, &output, &outputCount)

		// Should generate exactly 4 corners
		if outputCount != 4 {
			t.Errorf("outputCount = %d, want 4", outputCount)
		}

		// Points should form a large square (size 1000.0)
		// Center should be at plane.Normal * -plane.Distance = {0,0,0}
		// Corners should be ±1000 in tangent directions

		// Check all points are roughly 1000 units from center
		center := mgl64.Vec3{0, 0, 0}
		for i := 0; i < outputCount; i++ {
			dist := output[i].Sub(center).Len()
			expectedDist := 1000.0 * math.Sqrt(2) // Diagonal of square
			if math.Abs(dist-expectedDist) > 1.0 {
				t.Errorf("point[%d] distance from center = %v, want ~%v", i, dist, expectedDist)
			}

			// All points should be on the plane (Y=0)
			if math.Abs(output[i].Y()) > 1e-6 {
				t.Errorf("point[%d].Y = %v, should be on plane Y=0", i, output[i].Y())
			}
		}
	})

	t.Run("vertical_plane_x", func(t *testing.T) {
		input := [8]mgl64.Vec3{}
		inputCount := 0

		transform := actor.Transform{
			Position: mgl64.Vec3{0, 0, 0},
			Rotation: mgl64.QuatIdent(),
		}

		plane := &actor.Plane{
			Normal:   mgl64.Vec3{1, 0, 0},
			Distance: 0,
		}

		var output [8]mgl64.Vec3
		var outputCount int

		builder.transformFeature(&input, inputCount, transform, plane, &output, &outputCount)

		if outputCount != 4 {
			t.Errorf("outputCount = %d, want 4", outputCount)
		}

		// All points should be on the plane (X=0)
		for i := 0; i < outputCount; i++ {
			if math.Abs(output[i].X()) > 1e-6 {
				t.Errorf("point[%d].X = %v, should be on plane X=0", i, output[i].X())
			}
		}
	})

	t.Run("diagonal_plane", func(t *testing.T) {
		input := [8]mgl64.Vec3{}
		inputCount := 0

		normal := mgl64.Vec3{1, 1, 1}.Normalize()

		transform := actor.Transform{
			Position: mgl64.Vec3{0, 0, 0},
			Rotation: mgl64.QuatIdent(),
		}

		plane := &actor.Plane{
			Normal:   normal,
			Distance: 0,
		}

		var output [8]mgl64.Vec3
		var outputCount int

		builder.transformFeature(&input, inputCount, transform, plane, &output, &outputCount)

		if outputCount != 4 {
			t.Errorf("outputCount = %d, want 4", outputCount)
		}

		// All points should be on the plane: dot(point, normal) = 0
		for i := 0; i < outputCount; i++ {
			dotProduct := output[i].Dot(normal)
			if math.Abs(dotProduct) > 1e-4 {
				t.Errorf("point[%d] not on plane: dot = %v", i, dotProduct)
			}
		}

		// Points should be large (~ 1000 units from origin)
		for i := 0; i < outputCount; i++ {
			dist := output[i].Len()
			if dist < 1000.0 {
				t.Errorf("point[%d] distance = %v, expected > 1000", i, dist)
			}
		}
	})
}

// TestClipPolygonAgainstPlane tests Sutherland-Hodgman single plane clipping
func TestClipPolygonAgainstPlane(t *testing.T) {
	builder := &ManifoldBuilder{}

	tests := []struct {
		name          string
		input         []mgl64.Vec3
		planePoint    mgl64.Vec3
		planeNormal   mgl64.Vec3
		expectedCount int
		checkPoints   bool
		expectedOut   []mgl64.Vec3
	}{
		{
			name:          "empty_input",
			input:         []mgl64.Vec3{},
			planePoint:    mgl64.Vec3{0, 0, 0},
			planeNormal:   mgl64.Vec3{0, 1, 0},
			expectedCount: 0,
		},
		{
			name: "all_inside",
			input: []mgl64.Vec3{
				{-1, 1, -1},
				{1, 1, -1},
				{1, 1, 1},
				{-1, 1, 1},
			},
			planePoint:    mgl64.Vec3{0, 0, 0},
			planeNormal:   mgl64.Vec3{0, 1, 0},
			expectedCount: 4,
		},
		{
			name: "all_outside",
			input: []mgl64.Vec3{
				{-1, -2, -1},
				{1, -2, -1},
				{1, -2, 1},
				{-1, -2, 1},
			},
			planePoint:    mgl64.Vec3{0, 0, 0},
			planeNormal:   mgl64.Vec3{0, 1, 0},
			expectedCount: 0,
		},
		{
			name: "partial_clip",
			input: []mgl64.Vec3{
				{-1, 0, 1},  // inside
				{1, 0, 1},   // inside
				{1, 0, -1},  // outside
				{-1, 0, -1}, // outside
			},
			planePoint:    mgl64.Vec3{0, 0, 0},
			planeNormal:   mgl64.Vec3{0, 0, 1},
			expectedCount: 4, // 2 original + 2 intersections
		},
		{
			name: "boundary_tolerance",
			input: []mgl64.Vec3{
				{0, 0, -1e-6}, // Exactly at tolerance, should be included
			},
			planePoint:    mgl64.Vec3{0, 0, 0},
			planeNormal:   mgl64.Vec3{0, 0, 1},
			expectedCount: 1,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Copy input to buffer
			var inputBuf [8]mgl64.Vec3
			for i, p := range tt.input {
				inputBuf[i] = p
			}

			var outputBuf [8]mgl64.Vec3
			var outputCount int

			builder.clipPolygonAgainstPlane(&inputBuf, len(tt.input), tt.planePoint, tt.planeNormal, &outputBuf, &outputCount)

			if outputCount != tt.expectedCount {
				t.Errorf("outputCount = %d, want %d", outputCount, tt.expectedCount)
			}

			if tt.checkPoints && len(tt.expectedOut) > 0 {
				for i := 0; i < outputCount; i++ {
					if !vec3ApproxEqual(outputBuf[i], tt.expectedOut[i], 1e-6) {
						t.Errorf("output[%d] = %v, want %v", i, outputBuf[i], tt.expectedOut[i])
					}
				}
			}
		})
	}
}

// TestClipIncidentAgainstReference tests multi-edge Sutherland-Hodgman with buffer ping-pong
func TestClipIncidentAgainstReference(t *testing.T) {
	builder := &ManifoldBuilder{}

	t.Run("large_plane_detected", func(t *testing.T) {
		builder.Reset()

		// Create a large plane reference
		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{0, 0, 0}
		reference[1] = mgl64.Vec3{200, 0, 0}
		reference[2] = mgl64.Vec3{200, 200, 0}
		reference[3] = mgl64.Vec3{0, 200, 0}
		referenceCount := 4

		// Incident polygon
		var incident [8]mgl64.Vec3
		incident[0] = mgl64.Vec3{1, 0, 0}
		incident[1] = mgl64.Vec3{2, 0, 0}
		incidentCount := 2

		normal := mgl64.Vec3{0, 0, 1}

		count := builder.clipIncidentAgainstReference(&incident, incidentCount, &reference, referenceCount, normal)

		// Should copy incident to clipBuffer1 unchanged
		if count != incidentCount {
			t.Errorf("count = %d, want %d", count, incidentCount)
		}

		// Verify clipBuffer1 has the incident points
		for i := 0; i < incidentCount; i++ {
			if !vec3ApproxEqual(builder.clipBuffer1[i], incident[i], 1e-6) {
				t.Errorf("clipBuffer1[%d] = %v, want %v", i, builder.clipBuffer1[i], incident[i])
			}
		}
	})

	t.Run("insufficient_reference", func(t *testing.T) {
		builder.Reset()

		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{0, 0, 0}
		referenceCount := 1 // < 2

		var incident [8]mgl64.Vec3
		incident[0] = mgl64.Vec3{1, 0, 0}
		incidentCount := 1

		normal := mgl64.Vec3{0, 0, 1}

		count := builder.clipIncidentAgainstReference(&incident, incidentCount, &reference, referenceCount, normal)

		// Should copy incident directly
		if count != incidentCount {
			t.Errorf("count = %d, want %d", count, incidentCount)
		}
	})

	t.Run("colinear_edge_skip", func(t *testing.T) {
		builder.Reset()

		// Create reference with one edge parallel to normal
		var reference [8]mgl64.Vec3
		normal := mgl64.Vec3{0, 0, 1}

		// Edge from (0,0,0) to (0,0,1) is parallel to normal
		reference[0] = mgl64.Vec3{0, 0, 0}
		reference[1] = mgl64.Vec3{0, 0, 1} // Colinear edge
		reference[2] = mgl64.Vec3{1, 0, 1}
		reference[3] = mgl64.Vec3{1, 0, 0}
		referenceCount := 4

		var incident [8]mgl64.Vec3
		incident[0] = mgl64.Vec3{0.5, 0, 0.5}
		incidentCount := 1

		count := builder.clipIncidentAgainstReference(&incident, incidentCount, &reference, referenceCount, normal)

		// Should still return some result (colinear edge skipped)
		if count == 0 {
			t.Error("count = 0, colinear edge should be skipped but not fail")
		}
	})

	t.Run("normal_clipping_square", func(t *testing.T) {
		builder.Reset()

		// Create a square reference face
		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{-1, 0, -1}
		reference[1] = mgl64.Vec3{1, 0, -1}
		reference[2] = mgl64.Vec3{1, 0, 1}
		reference[3] = mgl64.Vec3{-1, 0, 1}
		referenceCount := 4

		// Incident polygon (slightly overlapping)
		var incident [8]mgl64.Vec3
		incident[0] = mgl64.Vec3{-0.5, 0, -0.5}
		incident[1] = mgl64.Vec3{0.5, 0, -0.5}
		incident[2] = mgl64.Vec3{0.5, 0, 0.5}
		incident[3] = mgl64.Vec3{-0.5, 0, 0.5}
		incidentCount := 4

		normal := mgl64.Vec3{0, 1, 0}

		count := builder.clipIncidentAgainstReference(&incident, incidentCount, &reference, referenceCount, normal)

		// Should clip successfully (exact count depends on geometry)
		if count == 0 {
			t.Error("count = 0, expected some points after clipping")
		}

		// Result should be in clipBuffer1 (even number of edges = 4)
		if builder.clipBuffer1Count == 0 {
			t.Error("clipBuffer1Count = 0, expected result in clipBuffer1")
		}
	})

	t.Run("clip_normal_inversion", func(t *testing.T) {
		builder.Reset()

		// Create reference where center is on the opposite side
		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{10, 0, 10}
		reference[1] = mgl64.Vec3{11, 0, 10}
		reference[2] = mgl64.Vec3{11, 0, 11}
		reference[3] = mgl64.Vec3{10, 0, 11}
		referenceCount := 4

		// Incident at origin
		var incident [8]mgl64.Vec3
		incident[0] = mgl64.Vec3{0, 0, 0}
		incident[1] = mgl64.Vec3{1, 0, 0}
		incident[2] = mgl64.Vec3{1, 0, 1}
		incident[3] = mgl64.Vec3{0, 0, 1}
		incidentCount := 4

		normal := mgl64.Vec3{0, 1, 0}

		count := builder.clipIncidentAgainstReference(&incident, incidentCount, &reference, referenceCount, normal)

		// Should handle clip normal inversion
		if count < 0 {
			t.Errorf("count = %d, should be >= 0", count)
		}
	})

	t.Run("odd_number_of_edges", func(t *testing.T) {
		builder.Reset()

		// Create a triangular reference (3 edges)
		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{-1, 0, -1}
		reference[1] = mgl64.Vec3{1, 0, -1}
		reference[2] = mgl64.Vec3{0, 0, 1}
		referenceCount := 3

		// Incident polygon
		var incident [8]mgl64.Vec3
		incident[0] = mgl64.Vec3{-0.5, 0, -0.5}
		incident[1] = mgl64.Vec3{0.5, 0, -0.5}
		incident[2] = mgl64.Vec3{0.5, 0, 0.5}
		incident[3] = mgl64.Vec3{-0.5, 0, 0.5}
		incidentCount := 4

		normal := mgl64.Vec3{0, 1, 0}

		count := builder.clipIncidentAgainstReference(&incident, incidentCount, &reference, referenceCount, normal)

		// With odd number of edges, result should be copied to clipBuffer1
		if count == 0 {
			t.Error("count = 0, expected some points after clipping")
		}

		// Verify result is in clipBuffer1
		if builder.clipBuffer1Count == 0 {
			t.Error("clipBuffer1Count = 0, expected result in clipBuffer1 after odd edges")
		}
	})
}

// TestClipAgainstReferencePlane tests final clipping against reference plane
func TestClipAgainstReferencePlane(t *testing.T) {
	builder := &ManifoldBuilder{}

	t.Run("points_behind_plane", func(t *testing.T) {
		builder.Reset()

		// Set up clipBuffer1 with 4 points
		builder.clipBuffer1[0] = mgl64.Vec3{0, 1, 0}  // Above plane (behind)
		builder.clipBuffer1[1] = mgl64.Vec3{0, -1, 0} // Below plane (in front)
		builder.clipBuffer1[2] = mgl64.Vec3{1, -1, 0} // Below plane (in front)
		builder.clipBuffer1[3] = mgl64.Vec3{1, 1, 0}  // Above plane (behind)
		clippedCount := 4

		// Reference triangle defining plane at Y=0
		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{0, 0, 0}
		reference[1] = mgl64.Vec3{1, 0, 0}
		reference[2] = mgl64.Vec3{0, 0, 1}
		referenceCount := 3

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		builder.clipAgainstReferencePlane(clippedCount, &reference, referenceCount, normal, depth)

		// Should keep only points with distance <= 0 (below plane)
		if builder.tempPointsCount != 2 {
			t.Errorf("tempPointsCount = %d, want 2", builder.tempPointsCount)
		}

		// Verify the kept points are the ones below the plane
		for i := 0; i < builder.tempPointsCount; i++ {
			if builder.tempPoints[i].Position.Y() > 0 {
				t.Errorf("tempPoints[%d].Y = %v, should be <= 0", i, builder.tempPoints[i].Position.Y())
			}
		}
	})

	t.Run("all_points_pass", func(t *testing.T) {
		builder.Reset()

		// All points below plane
		builder.clipBuffer1[0] = mgl64.Vec3{0, -1, 0}
		builder.clipBuffer1[1] = mgl64.Vec3{1, -1, 0}
		builder.clipBuffer1[2] = mgl64.Vec3{1, -1, 1}
		builder.clipBuffer1[3] = mgl64.Vec3{0, -1, 1}
		clippedCount := 4

		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{0, 0, 0}
		reference[1] = mgl64.Vec3{1, 0, 0}
		reference[2] = mgl64.Vec3{0, 0, 1}
		referenceCount := 3

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		builder.clipAgainstReferencePlane(clippedCount, &reference, referenceCount, normal, depth)

		// All points should pass
		if builder.tempPointsCount != 4 {
			t.Errorf("tempPointsCount = %d, want 4", builder.tempPointsCount)
		}
	})

	t.Run("buffer_limit", func(t *testing.T) {
		builder.Reset()

		// Fill clipBuffer1 with 8 points all below plane
		for i := 0; i < 8; i++ {
			builder.clipBuffer1[i] = mgl64.Vec3{float64(i), -1, 0}
		}
		clippedCount := 8

		// Add 4 more to tempPoints first (to test limit)
		for i := 0; i < 4; i++ {
			builder.tempPoints[i] = constraint.ContactPoint{
				Position:    mgl64.Vec3{float64(i), -1, 0},
				Penetration: 0.1,
			}
		}
		builder.tempPointsCount = 4

		var reference [8]mgl64.Vec3
		reference[0] = mgl64.Vec3{0, 0, 0}
		reference[1] = mgl64.Vec3{10, 0, 0}
		reference[2] = mgl64.Vec3{0, 0, 10}
		referenceCount := 3

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		builder.clipAgainstReferencePlane(clippedCount, &reference, referenceCount, normal, depth)

		// Should stop at 8 total (4 existing + 4 new)
		if builder.tempPointsCount > 8 {
			t.Errorf("tempPointsCount = %d, should not exceed 8", builder.tempPointsCount)
		}
	})
}

// TestReduceTo4Points tests point reduction algorithm
func TestReduceTo4Points(t *testing.T) {
	builder := &ManifoldBuilder{}

	t.Run("already_4_or_fewer", func(t *testing.T) {
		builder.Reset()

		// Create 4 points
		for i := 0; i < 4; i++ {
			builder.tempPoints[i] = constraint.ContactPoint{
				Position:    mgl64.Vec3{float64(i), 0, 0},
				Penetration: 0.1,
			}
		}
		builder.tempPointsCount = 4

		normal := mgl64.Vec3{0, 1, 0}
		builder.reduceTo4Points(normal)

		// Should not change
		if builder.tempPointsCount != 4 {
			t.Errorf("tempPointsCount = %d, want 4", builder.tempPointsCount)
		}
	})

	t.Run("reduce_from_8", func(t *testing.T) {
		builder.Reset()

		// Create 8 points forming an octagon in XY plane
		for i := 0; i < 8; i++ {
			angle := float64(i) * math.Pi / 4
			builder.tempPoints[i] = constraint.ContactPoint{
				Position:    mgl64.Vec3{math.Cos(angle), math.Sin(angle), 0},
				Penetration: 0.1,
			}
		}
		builder.tempPointsCount = 8

		normal := mgl64.Vec3{0, 0, 1}
		builder.reduceTo4Points(normal)

		// Should reduce to 4 extremes
		if builder.tempPointsCount > 4 {
			t.Errorf("tempPointsCount = %d, want <= 4", builder.tempPointsCount)
		}
		if builder.tempPointsCount < 1 {
			t.Errorf("tempPointsCount = %d, want >= 1", builder.tempPointsCount)
		}
	})
}

// TestManifoldGenerate tests the main Generate orchestrator
func TestManifoldGenerate(t *testing.T) {
	t.Run("trivial_case_single_incident", func(t *testing.T) {
		// Sphere vs Box: Sphere has 1 point, Box has 4
		bodyA := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.9, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Should return 1 point (trivial case)
		if len(points) != 1 {
			t.Errorf("len(points) = %d, want 1", len(points))
		}

		if len(points) > 0 && points[0].Penetration != depth {
			t.Errorf("points[0].Penetration = %v, want %v", points[0].Penetration, depth)
		}
	})

	t.Run("fallback_case_empty_clipping", func(t *testing.T) {
		// Create a scenario where all points get clipped away
		// Box-Box with rotations that produce difficult clipping
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.1, 0.1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0}),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.1, 0.1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0.15, 0},
				Rotation: mgl64.QuatRotate(math.Pi/3, mgl64.Vec3{0, 1, 0}),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.05

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Should use fallback and still produce at least 1 point
		if len(points) == 0 {
			t.Error("len(points) = 0, fallback should produce at least 1 point")
		}
	})

	t.Run("reduction_case_more_than_4", func(t *testing.T) {
		// Box-Box aligned to produce maximum contact points (potentially 8)
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0.99, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.01

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Should reduce to max 4 points
		if len(points) > 4 {
			t.Errorf("len(points) = %d, should reduce to max 4", len(points))
		}

		if len(points) == 0 {
			t.Error("len(points) = 0, expected contact points")
		}
	})

	t.Run("clippedCount_zero_skip_reference_plane", func(t *testing.T) {
		// Create scenario where clipping produces 0 points
		// This will skip clipAgainstReferencePlane (line 110 condition false)
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{0.01, 0.01, 0.01}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{1, 0, 0}),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{0.01, 0.01, 0.01}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{10, 10, 10},
				Rotation: mgl64.QuatRotate(math.Pi/3, mgl64.Vec3{0, 0, 1}),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.001

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// With clippedCount == 0, should use fallback
		if len(points) == 0 {
			t.Error("len(points) = 0, fallback should produce at least 1 point")
		}
	})

	t.Run("both_features_equal_count", func(t *testing.T) {
		// Test exact equality case (worldFeatureBCount == worldFeatureACount)
		// This ensures the <= branch (line 84) is properly tested
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.5, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.5

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Both boxes should have same feature count (4 points each)
		// Should still produce valid manifold
		if len(points) == 0 {
			t.Error("len(points) = 0, expected contact points")
		}
	})

	t.Run("skip_reduction_exactly_4", func(t *testing.T) {
		// Test case where tempPointsCount == 4 exactly
		// This ensures the > 4 check (line 125) is false
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0.99, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.01

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Should produce points without needing reduction
		if len(points) > 4 {
			t.Errorf("len(points) = %d, should not exceed 4", len(points))
		}
	})

	t.Run("normal_clipping_path", func(t *testing.T) {
		// Ensure normal path where clippedCount > 0 AND referenceCount > 0
		// This makes line 110 condition TRUE
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.5, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.5

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Normal clipping should work
		if len(points) == 0 {
			t.Error("len(points) = 0, expected contact points from normal clipping")
		}
	})
}

// TestGenerateManifold tests public API
func TestGenerateManifold(t *testing.T) {
	t.Run("box_box_typical", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1.5, 1.5, 1.5}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1.5, 1.5, 1.5}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 2.9, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		if len(points) == 0 {
			t.Error("len(points) = 0, expected at least 1")
		}

		if len(points) > 4 {
			t.Errorf("len(points) = %d, should not exceed 4", len(points))
		}
	})
}

// TestManifoldBuilderReset tests Reset method
func TestManifoldBuilderReset(t *testing.T) {
	builder := &ManifoldBuilder{}

	// Set non-zero counts
	builder.localFeatureACount = 5
	builder.localFeatureBCount = 3
	builder.worldFeatureACount = 4
	builder.worldFeatureBCount = 2
	builder.clipBuffer1Count = 1
	builder.clipBuffer2Count = 6
	builder.clippedResultCount = 7
	builder.tempPointsCount = 8

	builder.Reset()

	// All should be zero
	if builder.localFeatureACount != 0 {
		t.Errorf("localFeatureACount = %d, want 0", builder.localFeatureACount)
	}
	if builder.localFeatureBCount != 0 {
		t.Errorf("localFeatureBCount = %d, want 0", builder.localFeatureBCount)
	}
	if builder.worldFeatureACount != 0 {
		t.Errorf("worldFeatureACount = %d, want 0", builder.worldFeatureACount)
	}
	if builder.worldFeatureBCount != 0 {
		t.Errorf("worldFeatureBCount = %d, want 0", builder.worldFeatureBCount)
	}
	if builder.clipBuffer1Count != 0 {
		t.Errorf("clipBuffer1Count = %d, want 0", builder.clipBuffer1Count)
	}
	if builder.clipBuffer2Count != 0 {
		t.Errorf("clipBuffer2Count = %d, want 0", builder.clipBuffer2Count)
	}
	if builder.clippedResultCount != 0 {
		t.Errorf("clippedResultCount = %d, want 0", builder.clippedResultCount)
	}
	if builder.tempPointsCount != 0 {
		t.Errorf("tempPointsCount = %d, want 0", builder.tempPointsCount)
	}
}

// TestBuildResult tests final result building
func TestBuildResult(t *testing.T) {
	builder := &ManifoldBuilder{}

	t.Run("zero_points", func(t *testing.T) {
		builder.Reset()
		result := builder.buildResult()

		if len(result) != 0 {
			t.Errorf("len(result) = %d, want 0", len(result))
		}
	})

	t.Run("four_points", func(t *testing.T) {
		builder.Reset()

		for i := 0; i < 4; i++ {
			builder.tempPoints[i] = constraint.ContactPoint{
				Position:    mgl64.Vec3{float64(i), 0, 0},
				Penetration: 0.1,
			}
		}
		builder.tempPointsCount = 4

		result := builder.buildResult()

		if len(result) != 4 {
			t.Errorf("len(result) = %d, want 4", len(result))
		}

		// Verify values copied correctly
		for i := 0; i < 4; i++ {
			if !vec3ApproxEqual(result[i].Position, mgl64.Vec3{float64(i), 0, 0}, 1e-6) {
				t.Errorf("result[%d].Position = %v, want %v", i, result[i].Position, mgl64.Vec3{float64(i), 0, 0})
			}
		}
	})
}

// TestManifoldShapeCombinations tests different shape pairs
func TestManifoldShapeCombinations(t *testing.T) {
	t.Run("box_sphere", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 0.5},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.4, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		if len(points) == 0 {
			t.Error("len(points) = 0, expected at least 1")
		}
	})

	t.Run("sphere_sphere", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.9, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Should return 1 point (both spheres have 1 point)
		if len(points) != 1 {
			t.Errorf("len(points) = %d, want 1", len(points))
		}
	})

	t.Run("box_plane", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0.9, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Plane{
				Normal:   mgl64.Vec3{0, 1, 0},
				Distance: 0,
			},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Plane generates 4 large corners, box has 4 points
		// Should produce contact points
		if len(points) == 0 {
			t.Error("len(points) = 0, expected contact points for box-plane")
		}
	})

	t.Run("sphere_plane", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0.9, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Plane{
				Normal:   mgl64.Vec3{0, 1, 0},
				Distance: 0,
			},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.1

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Sphere has 1 point, should use trivial case
		if len(points) != 1 {
			t.Errorf("len(points) = %d, want 1", len(points))
		}
	})

	t.Run("rotated_box_box", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(math.Pi/6, mgl64.Vec3{0, 1, 0}),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.8, 0},
				Rotation: mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0}),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.2

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		if len(points) == 0 {
			t.Error("len(points) = 0, expected contact points for rotated boxes")
		}

		if len(points) > 4 {
			t.Errorf("len(points) = %d, should not exceed 4", len(points))
		}
	})
}

// TestManifoldEdgeCases tests edge cases and numerical stability
func TestManifoldEdgeCases(t *testing.T) {
	t.Run("zero_depth", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 2, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.0

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		// Should still work with zero depth
		if len(points) == 0 {
			t.Error("len(points) = 0, should handle zero depth")
		}

		for i, p := range points {
			if p.Penetration != 0.0 {
				t.Errorf("points[%d].Penetration = %v, want 0", i, p.Penetration)
			}
		}
	})

	t.Run("tiny_penetration", func(t *testing.T) {
		bodyA := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		bodyB := &actor.RigidBody{
			Shape: &actor.Sphere{Radius: 1.0},
			Transform: actor.Transform{
				Position: mgl64.Vec3{0, 1.999999, 0},
				Rotation: mgl64.QuatIdent(),
			},
		}

		normal := mgl64.Vec3{0, 1, 0}
		depth := 1e-12

		points := GenerateManifold(bodyA, bodyB, normal, depth)

		if len(points) == 0 {
			t.Error("len(points) = 0, should handle tiny penetration")
		}
	})
}

// BenchmarkManifoldBoxBox-16    	 1000000	      1030 ns/op	     128 B/op	       1 allocs/op
func BenchmarkManifoldBoxBox(b *testing.B) {
	bodyA := &actor.RigidBody{
		Shape: &actor.Box{
			HalfExtents: mgl64.Vec3{1.5, 1.5, 1.5},
		},
		Transform: actor.Transform{
			Position: mgl64.Vec3{0, 0, 0},
			Rotation: mgl64.QuatIdent(),
		},
	}
	bodyA.Transform.InverseRotation = bodyA.Transform.Rotation.Inverse()

	bodyB := &actor.RigidBody{
		Shape: &actor.Box{
			HalfExtents: mgl64.Vec3{1.5, 1.5, 1.5},
		},
		Transform: actor.Transform{
			Position: mgl64.Vec3{0, 3, 0},
			Rotation: mgl64.QuatIdent(),
		},
	}
	bodyB.Transform.InverseRotation = bodyB.Transform.Rotation.Inverse()

	normal := mgl64.Vec3{0, 1, 0}
	depth := 0.01

	b.ResetTimer()
	b.ReportAllocs()

	for i := 0; i < b.N; i++ {
		points := GenerateManifold(bodyA, bodyB, normal, depth)
		if len(points) == 0 {
			b.Fatal("No contact points generated")
		}
	}
}
