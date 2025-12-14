package gjk

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Test helper functions

func createBoxBody(position mgl64.Vec3, halfExtents mgl64.Vec3) *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: position, Rotation: mgl64.QuatIdent()},
		&actor.Box{HalfExtents: halfExtents},
		actor.BodyTypeDynamic,
		1.0,
	)
}

func createSphereBody(position mgl64.Vec3, radius float64) *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: position, Rotation: mgl64.QuatIdent()},
		&actor.Sphere{Radius: radius},
		actor.BodyTypeDynamic,
		1.0,
	)
}

// MinkowskiSupport tests

func TestMinkowskiSupport(t *testing.T) {
	t.Run("two separated spheres along x-axis", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{3, 0, 0}, 1.0)

		direction := mgl64.Vec3{1, 0, 0}
		support := MinkowskiSupport(a, b, direction)

		// For separated spheres (B to the right of A):
		// A - B is shifted left, so support should be negative
		// Specifically: max(A.x) - min(B.x) = 1 - 2 = -1
		if support.X() >= 0 {
			t.Errorf("Expected support.X < 0 for separated shapes, got %v", support.X())
		}

		// Check the exact expected value
		expectedX := -1.0
		if support.X() != expectedX {
			t.Errorf("Expected support.X = %v, got %v", expectedX, support.X())
		}
	})

	t.Run("two overlapping spheres", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{1.5, 0, 0}, 1.0)

		direction := mgl64.Vec3{1, 0, 0}
		support := MinkowskiSupport(a, b, direction)

		// For overlapping spheres:
		// The Minkowski difference should contain the origin
		// Support in +X should be positive: max(A.x) - min(B.x) = 1 - 0.5 = 0.5
		if support.X() <= 0 {
			t.Errorf("Expected support.X > 0 for overlapping shapes, got %v", support.X())
		}

		expectedX := 0.5
		if support.X() != expectedX {
			t.Errorf("Expected support.X = %v, got %v", expectedX, support.X())
		}
	})

	t.Run("opposite directions give different supports", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{5, 0, 0}, 1.0)

		direction := mgl64.Vec3{1, 0, 0}
		support1 := MinkowskiSupport(a, b, direction)

		direction = mgl64.Vec3{-1, 0, 0}
		support2 := MinkowskiSupport(a, b, direction)

		// Supports in opposite directions should have different X values
		// For +X: max(A.x) - min(B.x) = 1 - 4 = -3
		// For -X: min(A.x) - max(B.x) = -1 - 6 = -7
		// So support1.X should be > support2.X
		if support1.X() <= support2.X() {
			t.Errorf("Expected support1.X > support2.X, got %v <= %v", support1.X(), support2.X())
		}
	})
}

// GJK collision detection tests - Spheres

func TestGJK_Spheres_Intersecting(t *testing.T) {
	t.Run("overlapping spheres", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{1.5, 0, 0}, 1.0)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision between overlapping spheres")
		}
	})

	t.Run("touching spheres", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{2.0, 0, 0}, 1.0)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		// Touching should be detected as collision
		if !result {
			t.Error("Expected collision for touching spheres")
		}
	})

	t.Run("identical position spheres", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for spheres at identical positions")
		}
	})
}

func TestGJK_Spheres_Separated(t *testing.T) {
	t.Run("far apart spheres", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{10, 0, 0}, 1.0)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if result {
			t.Error("Expected no collision between separated spheres")
		}
	})

	t.Run("barely separated spheres", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{2.1, 0, 0}, 1.0)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if result {
			t.Error("Expected no collision for barely separated spheres")
		}
	})

	t.Run("spheres separated on different axes", func(t *testing.T) {
		testCases := []struct {
			name      string
			positionB mgl64.Vec3
		}{
			{"separated on Y", mgl64.Vec3{0, 5, 0}},
			{"separated on Z", mgl64.Vec3{0, 0, 5}},
			{"separated diagonally", mgl64.Vec3{3, 3, 3}},
		}

		for _, tc := range testCases {
			t.Run(tc.name, func(t *testing.T) {
				a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
				b := createSphereBody(tc.positionB, 1.0)
				simplex := &Simplex{}

				result := GJK(a, b, simplex)
				if result {
					t.Errorf("Expected no collision for %s", tc.name)
				}
			})
		}
	})
}

// GJK collision detection tests - Boxes

func TestGJK_Boxes_Intersecting(t *testing.T) {
	t.Run("overlapping boxes", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		b := createBoxBody(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision between overlapping boxes")
		}
	})

	t.Run("touching boxes", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		b := createBoxBody(mgl64.Vec3{2.0, 0, 0}, mgl64.Vec3{1, 1, 1})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for touching boxes")
		}
	})

	t.Run("box completely inside another", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{2, 2, 2})
		b := createBoxBody(mgl64.Vec3{0, 1, 1}, mgl64.Vec3{1, 1, 1})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for box inside another box")
		}
	})
}

func TestGJK_Boxes_Separated(t *testing.T) {
	t.Run("far apart boxes", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		b := createBoxBody(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if result {
			t.Error("Expected no collision between separated boxes")
		}
	})

	t.Run("barely separated boxes", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		b := createBoxBody(mgl64.Vec3{2.1, 0, 0}, mgl64.Vec3{1, 1, 1})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if result {
			t.Error("Expected no collision for barely separated boxes")
		}
	})
}

// GJK collision detection tests - Mixed shapes

func TestGJK_MixedShapes_Intersecting(t *testing.T) {
	t.Run("sphere inside box", func(t *testing.T) {
		box := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{2, 2, 2})
		sphere := createSphereBody(mgl64.Vec3{0, 0, 0}, 0.5)
		simplex := &Simplex{}

		result := GJK(box, sphere, simplex)
		if !result {
			t.Error("Expected collision for sphere inside box")
		}
	})

	t.Run("sphere overlapping box corner", func(t *testing.T) {
		box := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		sphere := createSphereBody(mgl64.Vec3{1.5, 1.5, 1.5}, 1.0)
		simplex := &Simplex{}

		result := GJK(box, sphere, simplex)
		if !result {
			t.Error("Expected collision for sphere overlapping box corner")
		}
	})
}

func TestGJK_MixedShapes_Separated(t *testing.T) {
	t.Run("sphere outside box", func(t *testing.T) {
		box := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		sphere := createSphereBody(mgl64.Vec3{5, 0, 0}, 1.0)
		simplex := &Simplex{}

		result := GJK(box, sphere, simplex)
		if result {
			t.Error("Expected no collision for sphere outside box")
		}
	})

	t.Run("sphere near box edge but not touching", func(t *testing.T) {
		box := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		sphere := createSphereBody(mgl64.Vec3{2.5, 0, 0}, 0.4)
		simplex := &Simplex{}

		result := GJK(box, sphere, simplex)
		if result {
			t.Error("Expected no collision for sphere near but not touching box")
		}
	})
}

// Edge case tests

func TestGJK_EdgeCases(t *testing.T) {
	t.Run("very small spheres overlapping", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 0.001)
		b := createSphereBody(mgl64.Vec3{0.0015, 0, 0}, 0.001)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for very small overlapping spheres")
		}
	})

	t.Run("very large spheres", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1000.0)
		b := createSphereBody(mgl64.Vec3{1500, 0, 0}, 1000.0)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for very large overlapping spheres")
		}
	})

	t.Run("different sized boxes overlapping", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
		b := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{5, 5, 5})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for different sized boxes at same position")
		}
	})
}

// Zero-vector direction handling

func TestGJK_ZeroVectorDirection(t *testing.T) {
	t.Run("identical positions trigger fallback", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		simplex := &Simplex{}

		// This should trigger the fallback direction (1,0,0)
		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for spheres at identical positions with zero initial direction")
		}
	})

	t.Run("extremely close positions trigger fallback", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		b := createSphereBody(mgl64.Vec3{1e-15, 0, 0}, 1.0)
		simplex := &Simplex{}

		// This should trigger the fallback direction due to near-zero initial direction
		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for spheres with extremely close positions")
		}
	})
}

// Extreme precision edge cases
func TestGJK_ExtremePrecision(t *testing.T) {
	t.Run("separation at tolerance boundary (1e-7)", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
		// Separation distance should be exactly at tolerance threshold
		b := createSphereBody(mgl64.Vec3{2.0000001, 0, 0}, 1.0) // 2.0 + 1e-7
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if result {
			t.Error("Expected no collision for spheres separated by exactly 1e-8")
		}
	})

	t.Run("extremely large shapes (1e10)", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1e10)
		b := createSphereBody(mgl64.Vec3{1.5e10, 0, 0}, 1e10)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for extremely large overlapping spheres")
		}
	})

	t.Run("extremely small shapes (1e-10)", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1e-10)
		b := createSphereBody(mgl64.Vec3{1.5e-10, 0, 0}, 1e-10)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for extremely small overlapping spheres")
		}
	})
}

// Degenerate simplex cases
func TestGJK_DegenerateSimplex(t *testing.T) {
	t.Run("colinear points in tetrahedron", func(t *testing.T) {
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{0, 0, 0},
				{1, 0, 0},
				{2, 0, 0},
				{3, 0, 0},
			},
			Count: 4,
		}
		direction := mgl64.Vec3{0, 1, 0}

		// This should be reduced to a line and eventually return false
		result := tetrahedron(&simplex, &direction)
		if result {
			t.Error("Expected tetrahedron with colinear points to not contain origin (origin not on any segment)")
		}
	})

	t.Run("identical points in simplex", func(t *testing.T) {
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{0, 0, 0},
				{0, 0, 0},
				{1, 0, 0},
				{0, 1, 0},
			},
			Count: 4,
		}
		direction := mgl64.Vec3{0, 0, 1}

		// This should be handled gracefully and not cause panic
		result := tetrahedron(&simplex, &direction)
		if result {
			t.Error("Expected tetrahedron with identical points to not contain origin")
		}
	})

	t.Run("zero-length edge in line", func(t *testing.T) {
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{1e-15, 0, 0},
				{1e-15, 1e-15, 0},
				{0, 0, 0},
				{0, 0, 0},
			},
			Count: 2,
		}
		direction := mgl64.Vec3{0, 1, 0}

		// This should be handled as a degenerate line
		result := line(&simplex, &direction)
		if !result {
			t.Error("Expected degenerate line with near-identical points to contain origin")
		}
	})
}

// Tetrahedron face normal orientation

func TestGJK_TetrahedronFaceNormal(t *testing.T) {
	t.Run("origin nearly on face (distance < 1e-12)", func(t *testing.T) {
		// Move origin extremely close to the face (ABC) but outside
		// The face ABC is the triangle with points A, B, C
		// The normal should point away from D (0,0,0)
		// Origin is at (0,0,0) which is point D, so we need to move the tetrahedron
		// so origin is near face ABC but not inside

		// Create a tetrahedron with face ABC at z=1e-12 and origin at (0,0,0)
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{1, 1, -1e-12}, // D
				{1, 0, 1e-12},  // C
				{0, 1, 1e-12},  // B
				{0, 0, 1e-12},  // A
			},
			Count: 4,
		}
		direction := mgl64.Vec3{0, 0, 1}

		result := tetrahedron(&simplex, &direction)
		if result {
			t.Error("Expected origin outside tetrahedron near face to not contain origin")
		}
	})

	t.Run("face normal with near-zero magnitude", func(t *testing.T) {
		// Create a tetrahedron where one face has a normal with near-zero magnitude
		// This can happen when three points are nearly colinear
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{0, 0, 0},
				{1, 0, 0},
				{1, 1e-15, 0},
				{0, 0, 1},
			},
			Count: 4,
		}
		direction := mgl64.Vec3{0, 0, 1}

		// This should be handled gracefully and not cause division by zero
		result := tetrahedron(&simplex, &direction)
		if result {
			t.Error("Expected tetrahedron with near-zero face normal to not contain origin")
		}
	})
}

// GJK with zero-volume shapes
func TestGJK_ZeroVolumeShapes(t *testing.T) {
	t.Run("zero-radius sphere (point)", func(t *testing.T) {
		a := createSphereBody(mgl64.Vec3{0, 0, 0}, 0.0)
		b := createSphereBody(mgl64.Vec3{0, 0, 0}, 0.0)
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for two points at same position")
		}
	})

	t.Run("zero-extent box (point)", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0, 0, 0})
		b := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0, 0, 0})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for two zero-extent boxes at same position")
		}
	})

	t.Run("zero-extent box (line)", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 0, 0})
		b := createBoxBody(mgl64.Vec3{1, 0, 0}, mgl64.Vec3{1, 0, 0})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for two lines touching at endpoint")
		}
	})

	t.Run("zero-extent box (plane)", func(t *testing.T) {
		a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 0})
		b := createBoxBody(mgl64.Vec3{0.5, 0.5, 0}, mgl64.Vec3{1, 1, 0})
		simplex := &Simplex{}

		result := GJK(a, b, simplex)
		if !result {
			t.Error("Expected collision for two zero-thickness planes overlapping")
		}
	})
}

func Inf() float64 {
	return math.Inf(1)
}

// Simplex helper function tests
func TestLine(t *testing.T) {
	t.Run("origin near line (normal case)", func(t *testing.T) {
		// Normal case: origin is near the line but not on it
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{-1, 1, 0}, // B (old point)
				{1, 1, 0},  // A (most recent point)
				{0, 0, 0},
				{0, 0, 0},
			},
			Count: 2,
		}
		direction := mgl64.Vec3{0, 1, 0}

		result := line(&simplex, &direction)

		if result {
			t.Error("Line not passing through origin should not detect collision")
		}
		// Origin is in direction of B, so both points should be kept
		if simplex.Count != 2 {
			t.Errorf("Expected simplex length 2, got %d", simplex.Count)
		}
	})

	t.Run("origin ON line segment (degenerate)", func(t *testing.T) {
		// Special case: origin is exactly on the line segment AB
		// This is a degenerate case that indicates collision
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{-1, 0, 0}, // B (old point)
				{1, 0, 0},  // A (most recent point)
				{0, 0, 0},
				{0, 0, 0},
			},
			Count: 2,
		}
		direction := mgl64.Vec3{0, 1, 0}

		result := line(&simplex, &direction)

		if !result {
			t.Error("Line passing through origin should detect collision")
		}
	})

	t.Run("origin on line segment", func(t *testing.T) {
		// Test that origin is detected as on segment when t is between 0 and 1
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{2, 0, 0}, // B
				{0, 0, 0}, // A
				{0, 0, 0},
				{0, 0, 0},
			},
			Count: 2,
		}
		direction := mgl64.Vec3{0, 1, 0}

		// Origin is at (0,0,0) which is exactly point A (t=0)
		// Correctly identifies this as Voronoi region A
		// and reduces to point A, returning false (no collision in this case)
		result := line(&simplex, &direction)
		if result {
			t.Error("Expected no collision when origin is exactly at point A (Voronoi region A)")
		}
	})

	t.Run("origin on line segment middle", func(t *testing.T) {
		// Test that origin is detected as on segment when t is between 0 and 1
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{2, 0, 0}, // B
				{0, 0, 0}, // A
				{0, 0, 0},
				{0, 0, 0},
			},
			Count: 2,
		}
		direction := mgl64.Vec3{0, 1, 0}

		// Move simplex so origin is in the middle of segment AB
		simplex.Points[1] = mgl64.Vec3{1, 0, 0}  // A
		simplex.Points[0] = mgl64.Vec3{-1, 0, 0} // B
		// Origin (0,0,0) is exactly in the middle (t=0.5)
		result := line(&simplex, &direction)
		if !result {
			t.Error("Expected collision when origin is in the middle of segment (t=0.5)")
		}
	})

	t.Run("origin on infinite line but not on segment", func(t *testing.T) {
		// Test that origin on infinite line but outside segment returns false
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{1, 0, 0}, // B
				{2, 0, 0}, // A
				{0, 0, 0},
				{0, 0, 0},
			},
			Count: 2,
		}
		direction := mgl64.Vec3{0, 1, 0}

		// Origin (0,0,0) is on the infinite line but not on segment [A,B]
		// Segment is from (2,0,0) to (1,0,0), origin is at (0,0,0) which is outside
		result := line(&simplex, &direction)
		if result {
			t.Error("Expected no collision when origin is on infinite line but not on segment")
		}
	})

	t.Run("origin behind point A", func(t *testing.T) {
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{3, 0, 0}, // B
				{1, 0, 0}, // A
				{0, 0, 0},
				{0, 0, 0},
			},
			Count: 2,
		}
		direction := mgl64.Vec3{-1, 0, 0}
		result := line(&simplex, &direction)
		if result {
			t.Error("Line should not contain origin")
		}
		// When origin is behind point A, simplex should be reduced to point A only
		if simplex.Count != 1 {
			t.Errorf("Expected simplex to be reduced to 1 point, got %d", simplex.Count)
		}
		// Direction should point from A toward origin
		if direction.Dot(mgl64.Vec3{-1, 0, 0}) != 1.0 {
			t.Errorf("Expected direction to be (-1,0,0), got %v", direction)
		}
	})
}

func TestTriangle(t *testing.T) {
	t.Run("origin above triangle", func(t *testing.T) {
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{1, 0, 0},   // C (oldest)
				{0, 1, 0},   // B
				{0, 0, 0.5}, // A (most recent)
				{0, 0, 0},
			},
			Count: 3,
		}
		direction := mgl64.Vec3{0, 0, 1}

		result := triangle(&simplex, &direction)

		if result {
			t.Error("Triangle should never contain origin in 3D")
		}
		// Simplex should remain a triangle (3 points)
		if simplex.Count != 3 {
			t.Errorf("Expected simplex to remain triangle (3 points), got %d", simplex.Count)
		}
	})

	t.Run("origin in AB edge region", func(t *testing.T) {
		// Create a proper triangle (not degenerate)
		// Triangle vertices: A=(2,0,0), B=(0,2,0), C=(3,3,0)
		// Origin should be in the Voronoi region of edge AB
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{3, 3, 0}, // C (oldest)
				{0, 2, 0}, // B
				{2, 0, 0}, // A (most recent)
				{0, 0, 0},
			},
			Count: 3,
		}
		direction := mgl64.Vec3{0, 0, 1}

		result := triangle(&simplex, &direction)

		if result {
			t.Error("Triangle should never contain origin in 3D")
		}
		// Origin is in AB region, so simplex should be reduced to edge AB (2 points)
		if simplex.Count != 2 {
			t.Errorf("Expected simplex reduced to edge (2 points), got %d", simplex.Count)
		}
	})

	t.Run("origin in AC edge region", func(t *testing.T) {
		// Create a proper triangle where origin is in AC edge region
		// Triangle vertices: A=(2,0,0), B=(3,3,0), C=(0,2,0)
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{0, 2, 0}, // C (oldest)
				{3, 3, 0}, // B
				{2, 0, 0}, // A (most recent)
				{0, 0, 0},
			},
			Count: 3,
		}
		direction := mgl64.Vec3{0, 0, 1}

		result := triangle(&simplex, &direction)

		if result {
			t.Error("Triangle should never contain origin in 3D")
		}
		// Origin is in AC region, so simplex should be reduced to edge AC (2 points)
		if simplex.Count != 2 {
			t.Errorf("Expected simplex reduced to edge (2 points), got %d", simplex.Count)
		}
	})
}

func TestTetrahedron(t *testing.T) {
	t.Run("origin inside tetrahedron", func(t *testing.T) {
		// Create a tetrahedron that actually contains the origin
		// Using a regular tetrahedron centered near origin
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{-1, -1, -1}, // D (oldest)
				{1, 1, -1},   // C
				{1, -1, 1},   // B
				{-1, 1, 1},   // A (most recent)
			},
			Count: 4,
		}
		direction := mgl64.Vec3{0, 0, 1}

		result := tetrahedron(&simplex, &direction)

		if !result {
			t.Error("Expected tetrahedron to contain origin")
		}
	})

	t.Run("origin outside ABC face", func(t *testing.T) {
		// Tetrahedron with origin clearly outside
		simplex := Simplex{
			Points: [4]mgl64.Vec3{
				{5, 5, 5}, // D (oldest)
				{6, 5, 5}, // C
				{5, 6, 5}, // B
				{5, 5, 6}, // A (most recent)
			},
			Count: 4,
		}
		direction := mgl64.Vec3{0, 0, 1}

		result := tetrahedron(&simplex, &direction)

		if result {
			t.Error("Expected origin to be outside tetrahedron")
		}
		if simplex.Count > 3 {
			t.Errorf("Expected simplex reduced to triangle (3 points), got %d", simplex.Count)
		}
	})
}

// Benchmark tests

func BenchmarkGJK_Spheres_Intersecting(b *testing.B) {
	a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
	body := createSphereBody(mgl64.Vec3{1.5, 0, 0}, 1.0)
	simplex := &Simplex{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		GJK(a, body, simplex)
	}
}

func BenchmarkGJK_Spheres_Separated(b *testing.B) {
	a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1.0)
	body := createSphereBody(mgl64.Vec3{10, 0, 0}, 1.0)
	simplex := &Simplex{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		GJK(a, body, simplex)
	}
}

func BenchmarkGJK_Boxes_Intersecting(b *testing.B) {
	a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
	box := createBoxBody(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1})
	simplex := &Simplex{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		GJK(a, box, simplex)
	}
}

func BenchmarkGJK_MixedShapes(b *testing.B) {
	box := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
	sphere := createSphereBody(mgl64.Vec3{1.5, 1.5, 1.5}, 1.0)
	simplex := &Simplex{}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		GJK(box, sphere, simplex)
	}
}
