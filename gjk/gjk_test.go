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

// Tetrahedron face normal orientation

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

// With a margin, body A is inflated: shapes closer than the margin overlap, farther ones
// do not.
func TestGJKMargin(t *testing.T) {
	a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
	b := createBoxBody(mgl64.Vec3{0, 2.01, 0}, mgl64.Vec3{1, 1, 1})
	simplex := &Simplex{}
	if GJKMargin(a, b, 0, simplex) {
		t.Error("boxes 1 cm apart overlap without margin")
	}
	simplex.Reset()
	if !GJKMargin(a, b, 0.02, simplex) {
		t.Error("boxes 1 cm apart do not overlap with a 2 cm margin")
	}
	if simplex.Count != 4 {
		t.Errorf("simplex has %d points, want a tetrahedron for EPA", simplex.Count)
	}
	simplex.Reset()
	if GJKMargin(a, b, 0.005, simplex) {
		t.Error("boxes 1 cm apart overlap with a 5 mm margin")
	}
}

// Every vertex of the final simplex remembers its support points: W = A - B, A on body A
// (inflated), B on body B.
func TestGJKSimplexKeepsSupports(t *testing.T) {
	a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1)
	b := createBoxBody(mgl64.Vec3{0.5, 1.2, 0.3}, mgl64.Vec3{0.5, 0.5, 0.5})
	simplex := &Simplex{}
	if !GJK(a, b, simplex) {
		t.Fatal("no overlap")
	}
	for i := 0; i < simplex.Count; i++ {
		v := simplex.Vertex(i)
		if v.W.Sub(v.A.Sub(v.B)).Len() > 1e-12 {
			t.Errorf("vertex %d: W %v != A - B %v", i, v.W, v.A.Sub(v.B))
		}
		if math.Abs(v.A.Len()-1) > 1e-9 {
			t.Errorf("vertex %d: A %v is not on the sphere", i, v.A)
		}
	}
}

// Shapes exactly touching (the origin on the Minkowski boundary) are an overlap, and the
// simplex is grown into a tetrahedron so EPA can start.
func TestGJKTouchingFillsTetrahedron(t *testing.T) {
	a := createBoxBody(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1})
	b := createBoxBody(mgl64.Vec3{2, 0, 0}, mgl64.Vec3{1, 1, 1})
	simplex := &Simplex{}
	if !GJK(a, b, simplex) {
		t.Fatal("touching boxes not reported")
	}
	if simplex.Count != 4 {
		t.Errorf("simplex has %d points, want 4", simplex.Count)
	}
}

// A direction of zero length gives a finite support point (the sphere used to return NaN).
func TestSupportZeroDirection(t *testing.T) {
	a := createSphereBody(mgl64.Vec3{0, 0, 0}, 1)
	b := createSphereBody(mgl64.Vec3{0, 0, 0}, 1)
	v := Support(a, b, mgl64.Vec3{}, 0.1)
	for _, x := range v.W {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			t.Fatalf("support %v is not finite", v)
		}
	}
}
