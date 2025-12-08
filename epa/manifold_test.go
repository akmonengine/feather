package epa

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// Test helpers
func vec3Equal(a, b mgl64.Vec3, tolerance float64) bool {
	return math.Abs(a.X()-b.X()) < tolerance &&
		math.Abs(a.Y()-b.Y()) < tolerance &&
		math.Abs(a.Z()-b.Z()) < tolerance
}

// TestComputeCenter tests the centroid calculation
func TestComputeCenter(t *testing.T) {
	tests := []struct {
		name     string
		points   []mgl64.Vec3
		expected mgl64.Vec3
	}{
		{
			name:     "empty slice",
			points:   []mgl64.Vec3{},
			expected: mgl64.Vec3{0, 0, 0},
		},
		{
			name:     "single point",
			points:   []mgl64.Vec3{{1, 2, 3}},
			expected: mgl64.Vec3{1, 2, 3},
		},
		{
			name:     "two points",
			points:   []mgl64.Vec3{{0, 0, 0}, {2, 4, 6}},
			expected: mgl64.Vec3{1, 2, 3},
		},
		{
			name:     "square corners",
			points:   []mgl64.Vec3{{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}},
			expected: mgl64.Vec3{0, 0, 0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := computeCenter(tt.points)
			if !vec3Equal(result, tt.expected, 1e-9) {
				t.Errorf("computeCenter() = %v, want %v", result, tt.expected)
			}
		})
	}
}

// TestIsLargePlane tests the large plane detection
func TestIsLargePlane(t *testing.T) {
	tests := []struct {
		name     string
		feature  []mgl64.Vec3
		expected bool
	}{
		{
			name:     "not 4 points",
			feature:  []mgl64.Vec3{{0, 0, 0}, {1, 0, 0}},
			expected: false,
		},
		{
			name:     "small quad",
			feature:  []mgl64.Vec3{{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}},
			expected: false,
		},
		{
			name: "large quad",
			feature: []mgl64.Vec3{
				{-500, -500, 0},
				{500, -500, 0},
				{500, 500, 0},
				{-500, 500, 0},
			},
			expected: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := isLargePlane(tt.feature)
			if result != tt.expected {
				t.Errorf("isLargePlane() = %v, want %v", result, tt.expected)
			}
		})
	}
}

// TestGetTangentBasis tests tangent basis generation
func TestGetTangentBasis(t *testing.T) {
	tests := []struct {
		name   string
		normal mgl64.Vec3
	}{
		{
			name:   "Y-axis normal",
			normal: mgl64.Vec3{0, 1, 0},
		},
		{
			name:   "X-axis normal",
			normal: mgl64.Vec3{1, 0, 0},
		},
		{
			name:   "Z-axis normal",
			normal: mgl64.Vec3{0, 0, 1},
		},
		{
			name:   "diagonal normal",
			normal: mgl64.Vec3{1, 1, 1}.Normalize(),
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t1, t2 := getTangentBasis(tt.normal)

			// Check that tangents are orthogonal to normal
			if math.Abs(t1.Dot(tt.normal)) > 1e-9 {
				t.Errorf("tangent1 not orthogonal to normal: dot = %v", t1.Dot(tt.normal))
			}
			if math.Abs(t2.Dot(tt.normal)) > 1e-9 {
				t.Errorf("tangent2 not orthogonal to normal: dot = %v", t2.Dot(tt.normal))
			}

			// Check that tangents are orthogonal to each other
			if math.Abs(t1.Dot(t2)) > 1e-9 {
				t.Errorf("tangents not orthogonal: dot = %v", t1.Dot(t2))
			}

			// Check that tangents are normalized
			if math.Abs(t1.Len()-1.0) > 1e-9 {
				t.Errorf("tangent1 not normalized: len = %v", t1.Len())
			}
			if math.Abs(t2.Len()-1.0) > 1e-9 {
				t.Errorf("tangent2 not normalized: len = %v", t2.Len())
			}
		})
	}
}

// TestLineIntersectPlane tests line-plane intersection
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
			name:        "perpendicular intersection",
			p1:          mgl64.Vec3{0, -1, 0},
			p2:          mgl64.Vec3{0, 1, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 0, 0},
		},
		{
			name:        "diagonal intersection",
			p1:          mgl64.Vec3{-1, -1, 0},
			p2:          mgl64.Vec3{1, 1, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 0, 0},
		},
		{
			name:        "parallel segment (returns p1)",
			p1:          mgl64.Vec3{0, 0, 0},
			p2:          mgl64.Vec3{1, 0, 0},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			expected:    mgl64.Vec3{0, 0, 0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := lineIntersectPlane(tt.p1, tt.p2, tt.planePoint, tt.planeNormal)
			if !vec3Equal(result, tt.expected, 1e-9) {
				t.Errorf("lineIntersectPlane() = %v, want %v", result, tt.expected)
			}
		})
	}
}

// TestClipPolygonAgainstPlane tests Sutherland-Hodgman clipping
func TestClipPolygonAgainstPlane(t *testing.T) {
	tests := []struct {
		name        string
		polygon     []mgl64.Vec3
		planePoint  mgl64.Vec3
		planeNormal mgl64.Vec3
		minExpected int // minimum expected points
		maxExpected int // maximum expected points
	}{
		{
			name:        "empty polygon",
			polygon:     []mgl64.Vec3{},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			minExpected: 0,
			maxExpected: 0,
		},
		{
			name: "square fully inside",
			polygon: []mgl64.Vec3{
				{-1, 1, -1}, {1, 1, -1}, {1, 1, 1}, {-1, 1, 1},
			},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			minExpected: 4,
			maxExpected: 4,
		},
		{
			name: "square fully outside",
			polygon: []mgl64.Vec3{
				{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1},
			},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			minExpected: 0,
			maxExpected: 0,
		},
		{
			name: "square partially clipped",
			polygon: []mgl64.Vec3{
				{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0},
			},
			planePoint:  mgl64.Vec3{0, 0, 0},
			planeNormal: mgl64.Vec3{0, 1, 0},
			minExpected: 3,
			maxExpected: 4,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := clipPolygonAgainstPlane(tt.polygon, tt.planePoint, tt.planeNormal)
			if len(result) < tt.minExpected || len(result) > tt.maxExpected {
				t.Errorf("clipPolygonAgainstPlane() returned %d points, want between %d and %d",
					len(result), tt.minExpected, tt.maxExpected)
			}
		})
	}
}

// TestTransformFeature tests feature transformation
func TestTransformFeature(t *testing.T) {
	t.Run("box feature", func(t *testing.T) {
		box := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
		transform := actor.Transform{Position: mgl64.Vec3{5, 10, 15}}
		feature := []mgl64.Vec3{{1, 1, 1}, {-1, 1, 1}}

		result := transformFeature(feature, transform, box)

		if len(result) != len(feature) {
			t.Fatalf("expected %d points, got %d", len(feature), len(result))
		}

		// Points should be translated by transform position
		expected0 := mgl64.Vec3{6, 11, 16}
		expected1 := mgl64.Vec3{4, 11, 16}

		if !vec3Equal(result[0], expected0, 1e-9) {
			t.Errorf("point 0 = %v, want %v", result[0], expected0)
		}
		if !vec3Equal(result[1], expected1, 1e-9) {
			t.Errorf("point 1 = %v, want %v", result[1], expected1)
		}
	})

	t.Run("plane feature", func(t *testing.T) {
		plane := &actor.Plane{
			Normal:   mgl64.Vec3{0, 1, 0},
			Distance: 0,
		}
		transform := actor.Transform{Position: mgl64.Vec3{0, 0, 0}}
		feature := []mgl64.Vec3{} // Not used for planes

		result := transformFeature(feature, transform, plane)

		// Should return 4 large points
		if len(result) != 4 {
			t.Errorf("expected 4 points for plane, got %d", len(result))
		}

		// Check that the quad is large
		for i := 0; i < len(result); i++ {
			for j := i + 1; j < len(result); j++ {
				dist := result[i].Sub(result[j]).Len()
				if dist < 100 {
					t.Errorf("plane feature too small: distance = %v", dist)
				}
			}
		}
	})
}

// TestReduceTo4Points tests contact point reduction
func TestReduceTo4Points(t *testing.T) {
	tests := []struct {
		name   string
		points []constraint.ContactPoint
		normal mgl64.Vec3
	}{
		{
			name: "5 points in a square pattern",
			points: []constraint.ContactPoint{
				{Position: mgl64.Vec3{-1, 0, -1}, Penetration: 0.1},
				{Position: mgl64.Vec3{1, 0, -1}, Penetration: 0.1},
				{Position: mgl64.Vec3{1, 0, 1}, Penetration: 0.1},
				{Position: mgl64.Vec3{-1, 0, 1}, Penetration: 0.1},
				{Position: mgl64.Vec3{0, 0, 0}, Penetration: 0.1}, // center point
			},
			normal: mgl64.Vec3{0, 1, 0},
		},
		{
			name: "6 points in a line",
			points: []constraint.ContactPoint{
				{Position: mgl64.Vec3{-2, 0, 0}, Penetration: 0.1},
				{Position: mgl64.Vec3{-1, 0, 0}, Penetration: 0.1},
				{Position: mgl64.Vec3{0, 0, 0}, Penetration: 0.1},
				{Position: mgl64.Vec3{1, 0, 0}, Penetration: 0.1},
				{Position: mgl64.Vec3{2, 0, 0}, Penetration: 0.1},
				{Position: mgl64.Vec3{0, 0, 1}, Penetration: 0.1},
			},
			normal: mgl64.Vec3{0, 1, 0},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := reduceTo4Points(tt.points, tt.normal)

			// Should return exactly 4 points
			if len(result) > 4 {
				t.Errorf("reduceTo4Points() returned %d points, want <= 4", len(result))
			}

			// All returned points should be from the original set
			for _, rp := range result {
				found := false
				for _, op := range tt.points {
					if vec3Equal(rp.Position, op.Position, 1e-9) {
						found = true
						break
					}
				}
				if !found {
					t.Errorf("result contains point not in original set: %v", rp.Position)
				}
			}
		})
	}
}

// TestClipIncidentAgainstReference tests the main clipping function
func TestClipIncidentAgainstReference(t *testing.T) {
	t.Run("large plane reference - no clipping", func(t *testing.T) {
		incident := []mgl64.Vec3{{0, 0, 0}, {1, 0, 0}}
		reference := []mgl64.Vec3{
			{-500, 0, -500}, {500, 0, -500}, {500, 0, 500}, {-500, 0, 500},
		}
		normal := mgl64.Vec3{0, 1, 0}

		result := clipIncidentAgainstReference(incident, reference, normal)

		if len(result) != len(incident) {
			t.Errorf("expected %d points (no clipping), got %d", len(incident), len(result))
		}
	})

	t.Run("reference with less than 2 points", func(t *testing.T) {
		incident := []mgl64.Vec3{{0, 0, 0}, {1, 0, 0}}
		reference := []mgl64.Vec3{{0, 1, 0}}
		normal := mgl64.Vec3{0, 1, 0}

		result := clipIncidentAgainstReference(incident, reference, normal)

		if len(result) != len(incident) {
			t.Errorf("expected %d points (no clipping), got %d", len(incident), len(result))
		}
	})

	t.Run("normal clipping", func(t *testing.T) {
		// Incident edge
		incident := []mgl64.Vec3{{-2, 0, 0}, {2, 0, 0}}

		// Reference square
		reference := []mgl64.Vec3{
			{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1},
		}
		normal := mgl64.Vec3{0, 1, 0}

		result := clipIncidentAgainstReference(incident, reference, normal)

		// The incident edge should be clipped to fit within the reference square
		// Result should have at least 1 point
		if len(result) < 1 {
			t.Errorf("expected at least 1 point after clipping, got %d", len(result))
		}
	})
}

// TestGenerateManifold tests the main manifold generation function
func TestGenerateManifold(t *testing.T) {
	t.Run("sphere-sphere contact", func(t *testing.T) {
		sphereA := &actor.Sphere{Radius: 1.0}
		sphereB := &actor.Sphere{Radius: 1.0}

		bodyA := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
			sphereA,
			actor.BodyTypeDynamic,
			1.0,
		)

		bodyB := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{1.5, 0, 0}},
			sphereB,
			actor.BodyTypeDynamic,
			1.0,
		)

		normal := mgl64.Vec3{1, 0, 0}
		depth := 0.5

		result := GenerateManifold(bodyA, bodyB, normal, depth)

		// Sphere-sphere should produce 1 contact point
		if len(result) != 1 {
			t.Errorf("expected 1 contact point for sphere-sphere, got %d", len(result))
		}

		if len(result) > 0 && result[0].Penetration != depth {
			t.Errorf("expected penetration %v, got %v", depth, result[0].Penetration)
		}
	})

	t.Run("box-box contact", func(t *testing.T) {
		boxA := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
		boxB := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}

		bodyA := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
			boxA,
			actor.BodyTypeDynamic,
			1.0,
		)

		bodyB := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{0, 1.8, 0}},
			boxB,
			actor.BodyTypeDynamic,
			1.0,
		)

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.2

		result := GenerateManifold(bodyA, bodyB, normal, depth)

		// Box-box face contact should produce multiple contact points
		if len(result) == 0 {
			t.Error("expected at least 1 contact point for box-box")
		}

		// Should not exceed 4 contact points
		if len(result) > 4 {
			t.Errorf("expected at most 4 contact points, got %d", len(result))
		}

		// All points should have the correct penetration depth
		for i, cp := range result {
			if cp.Penetration != depth {
				t.Errorf("contact point %d: expected penetration %v, got %v", i, depth, cp.Penetration)
			}
		}
	})

	t.Run("box-plane contact", func(t *testing.T) {
		box := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
		plane := &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: 0}

		bodyA := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{0, 0.5, 0}},
			box,
			actor.BodyTypeDynamic,
			1.0,
		)

		bodyB := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
			plane,
			actor.BodyTypeStatic,
			0.0,
		)

		normal := mgl64.Vec3{0, 1, 0}
		depth := 0.5

		result := GenerateManifold(bodyA, bodyB, normal, depth)

		// Box on plane should produce contact points
		if len(result) == 0 {
			t.Error("expected at least 1 contact point for box-plane")
		}

		// Should not exceed 4 contact points
		if len(result) > 4 {
			t.Errorf("expected at most 4 contact points, got %d", len(result))
		}
	})

	t.Run("fallback case - empty clipping result", func(t *testing.T) {
		// This test simulates a case where clipping might produce no points
		// The function should create a fallback contact point

		box := &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.1, 0.1}}
		sphere := &actor.Sphere{Radius: 0.1}

		bodyA := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
			box,
			actor.BodyTypeDynamic,
			1.0,
		)

		bodyB := actor.NewRigidBody(
			actor.Transform{Position: mgl64.Vec3{0.15, 0, 0}},
			sphere,
			actor.BodyTypeDynamic,
			1.0,
		)

		normal := mgl64.Vec3{1, 0, 0}
		depth := 0.05

		result := GenerateManifold(bodyA, bodyB, normal, depth)

		// Should always produce at least 1 contact point (fallback)
		if len(result) == 0 {
			t.Error("expected at least 1 contact point (fallback should trigger)")
		}
	})
}

// Benchmark tests
func BenchmarkGenerateManifold(b *testing.B) {
	boxA := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
	boxB := &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}

	bodyA := actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
		boxA,
		actor.BodyTypeDynamic,
		1.0,
	)

	bodyB := actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{0, 1.8, 0}},
		boxB,
		actor.BodyTypeDynamic,
		1.0,
	)

	normal := mgl64.Vec3{0, 1, 0}
	depth := 0.2

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		GenerateManifold(bodyA, bodyB, normal, depth)
	}
}

func BenchmarkClipPolygonAgainstPlane(b *testing.B) {
	polygon := []mgl64.Vec3{
		{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0},
	}
	planePoint := mgl64.Vec3{0, 0, 0}
	planeNormal := mgl64.Vec3{0, 1, 0}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		clipPolygonAgainstPlane(polygon, planePoint, planeNormal)
	}
}
