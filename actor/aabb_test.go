package actor

import (
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// =============================================================================
// AABB Utility Function Tests
// =============================================================================

func TestAABBOverlaps_Separated(t *testing.T) {
	tests := []struct {
		name  string
		aabb1 AABB
		aabb2 AABB
	}{
		{
			name:  "Separated on X axis (positive)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{2, 0, 0}, Max: mgl64.Vec3{3, 1, 1}},
		},
		{
			name:  "Separated on X axis (negative)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{-2, 0, 0}, Max: mgl64.Vec3{-1, 1, 1}},
		},
		{
			name:  "Separated on Y axis (positive)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{0, 2, 0}, Max: mgl64.Vec3{1, 3, 1}},
		},
		{
			name:  "Separated on Y axis (negative)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{0, -2, 0}, Max: mgl64.Vec3{1, -1, 1}},
		},
		{
			name:  "Separated on Z axis (positive)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{0, 0, 2}, Max: mgl64.Vec3{1, 1, 3}},
		},
		{
			name:  "Separated on Z axis (negative)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{0, 0, -2}, Max: mgl64.Vec3{1, 1, -1}},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if tt.aabb1.Overlaps(tt.aabb2) {
				t.Errorf("AABBs should not overlap")
			}
			// Test symmetry
			if tt.aabb2.Overlaps(tt.aabb1) {
				t.Errorf("AABBs should not overlap (symmetry test)")
			}
		})
	}
}

func TestAABBOverlaps_Overlapping(t *testing.T) {
	tests := []struct {
		name  string
		aabb1 AABB
		aabb2 AABB
	}{
		{
			name:  "Complete overlap (identical)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
		},
		{
			name:  "Partial overlap on X axis",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 1, 1}},
			aabb2: AABB{Min: mgl64.Vec3{1, 0, 0}, Max: mgl64.Vec3{3, 1, 1}},
		},
		{
			name:  "Partial overlap on Y axis",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 2, 1}},
			aabb2: AABB{Min: mgl64.Vec3{0, 1, 0}, Max: mgl64.Vec3{1, 3, 1}},
		},
		{
			name:  "Partial overlap on Z axis",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 2}},
			aabb2: AABB{Min: mgl64.Vec3{0, 0, 1}, Max: mgl64.Vec3{1, 1, 3}},
		},
		{
			name:  "Complete containment (aabb2 inside aabb1)",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{10, 10, 10}},
			aabb2: AABB{Min: mgl64.Vec3{2, 2, 2}, Max: mgl64.Vec3{3, 3, 3}},
		},
		{
			name:  "Partial overlap on all axes",
			aabb1: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}},
			aabb2: AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{3, 3, 3}},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if !tt.aabb1.Overlaps(tt.aabb2) {
				t.Errorf("AABBs should overlap")
			}
			// Test symmetry
			if !tt.aabb2.Overlaps(tt.aabb1) {
				t.Errorf("AABBs should overlap (symmetry test)")
			}
		})
	}
}

func TestAABBOverlaps_EdgeTouching(t *testing.T) {
	tests := []struct {
		name          string
		aabb1         AABB
		aabb2         AABB
		shouldOverlap bool
	}{
		{
			name:          "Edge touching on X axis",
			aabb1:         AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2:         AABB{Min: mgl64.Vec3{1, 0, 0}, Max: mgl64.Vec3{2, 1, 1}},
			shouldOverlap: true, // Touching edges should be considered overlapping
		},
		{
			name:          "Edge touching on Y axis",
			aabb1:         AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2:         AABB{Min: mgl64.Vec3{0, 1, 0}, Max: mgl64.Vec3{1, 2, 1}},
			shouldOverlap: true,
		},
		{
			name:          "Edge touching on Z axis",
			aabb1:         AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
			aabb2:         AABB{Min: mgl64.Vec3{0, 0, 1}, Max: mgl64.Vec3{1, 1, 2}},
			shouldOverlap: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.aabb1.Overlaps(tt.aabb2)
			if result != tt.shouldOverlap {
				t.Errorf("Expected overlap=%v, got %v", tt.shouldOverlap, result)
			}
		})
	}
}

func TestAABBContainsPoint(t *testing.T) {
	aabb := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}

	tests := []struct {
		name     string
		point    mgl64.Vec3
		expected bool
	}{
		{"Center point", mgl64.Vec3{1, 1, 1}, true},
		{"Min corner", mgl64.Vec3{0, 0, 0}, true},
		{"Max corner", mgl64.Vec3{2, 2, 2}, true},
		{"Outside (X too large)", mgl64.Vec3{3, 1, 1}, false},
		{"Outside (X too small)", mgl64.Vec3{-1, 1, 1}, false},
		{"Outside (Y too large)", mgl64.Vec3{1, 3, 1}, false},
		{"Outside (Y too small)", mgl64.Vec3{1, -1, 1}, false},
		{"Outside (Z too large)", mgl64.Vec3{1, 1, 3}, false},
		{"Outside (Z too small)", mgl64.Vec3{1, 1, -1}, false},
		{"Edge point (X)", mgl64.Vec3{2, 1, 1}, true},
		{"Edge point (Y)", mgl64.Vec3{1, 2, 1}, true},
		{"Edge point (Z)", mgl64.Vec3{1, 1, 2}, true},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := aabb.ContainsPoint(tt.point)
			if result != tt.expected {
				t.Errorf("ContainsPoint(%v) = %v, expected %v", tt.point, result, tt.expected)
			}
		})
	}
}

func TestAABBOverlaps_ZeroVolume(t *testing.T) {
	t.Run("Point AABB vs Point AABB (same position)", func(t *testing.T) {
		pointAABB1 := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{1, 1, 1}}
		pointAABB2 := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{1, 1, 1}}
		if !pointAABB1.Overlaps(pointAABB2) {
			t.Error("Point AABBs at same position should overlap")
		}
	})

	t.Run("Point AABB vs Point AABB (different positions)", func(t *testing.T) {
		pointAABB1 := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{1, 1, 1}}
		pointAABB2 := AABB{Min: mgl64.Vec3{2, 2, 2}, Max: mgl64.Vec3{2, 2, 2}}
		if pointAABB1.Overlaps(pointAABB2) {
			t.Error("Point AABBs at different positions should not overlap")
		}
	})

	t.Run("Point AABB inside normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		pointAABB := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{1, 1, 1}}
		if !normalAABB.Overlaps(pointAABB) {
			t.Error("Point AABB inside normal AABB should overlap")
		}
		if !pointAABB.Overlaps(normalAABB) {
			t.Error("Symmetry: Point AABB should overlap normal AABB")
		}
	})

	t.Run("Point AABB on edge of normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		pointAABB := AABB{Min: mgl64.Vec3{2, 1, 1}, Max: mgl64.Vec3{2, 1, 1}}
		if !normalAABB.Overlaps(pointAABB) {
			t.Error("Point AABB on edge of normal AABB should overlap")
		}
	})

	t.Run("Point AABB outside normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		pointAABB := AABB{Min: mgl64.Vec3{3, 3, 3}, Max: mgl64.Vec3{3, 3, 3}}
		if normalAABB.Overlaps(pointAABB) {
			t.Error("Point AABB outside normal AABB should not overlap")
		}
	})

	t.Run("Line AABB on X axis intersecting normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		lineAABB := AABB{Min: mgl64.Vec3{-1, 1, 1}, Max: mgl64.Vec3{3, 1, 1}}
		if !normalAABB.Overlaps(lineAABB) {
			t.Error("Line AABB intersecting normal AABB should overlap")
		}
	})

	t.Run("Line AABB on Y axis intersecting normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		lineAABB := AABB{Min: mgl64.Vec3{1, -1, 1}, Max: mgl64.Vec3{1, 3, 1}}
		if !normalAABB.Overlaps(lineAABB) {
			t.Error("Line AABB intersecting normal AABB should overlap")
		}
	})

	t.Run("Line AABB on Z axis intersecting normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		lineAABB := AABB{Min: mgl64.Vec3{1, 1, -1}, Max: mgl64.Vec3{1, 1, 3}}
		if !normalAABB.Overlaps(lineAABB) {
			t.Error("Line AABB intersecting normal AABB should overlap")
		}
	})

	t.Run("Line AABB parallel to normal AABB edge (not touching)", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		lineAABB := AABB{Min: mgl64.Vec3{3, 0, 0}, Max: mgl64.Vec3{3, 2, 0}}
		if normalAABB.Overlaps(lineAABB) {
			t.Error("Line AABB parallel but not touching should not overlap")
		}
	})

	t.Run("Plane AABB XY intersecting normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		planeAABB := AABB{Min: mgl64.Vec3{-1, -1, 1}, Max: mgl64.Vec3{3, 3, 1}}
		if !normalAABB.Overlaps(planeAABB) {
			t.Error("Plane AABB intersecting normal AABB should overlap")
		}
	})

	t.Run("Plane AABB XZ intersecting normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		planeAABB := AABB{Min: mgl64.Vec3{-1, 1, -1}, Max: mgl64.Vec3{3, 1, 3}}
		if !normalAABB.Overlaps(planeAABB) {
			t.Error("Plane AABB intersecting normal AABB should overlap")
		}
	})

	t.Run("Plane AABB YZ intersecting normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		planeAABB := AABB{Min: mgl64.Vec3{1, -1, -1}, Max: mgl64.Vec3{1, 3, 3}}
		if !normalAABB.Overlaps(planeAABB) {
			t.Error("Plane AABB intersecting normal AABB should overlap")
		}
	})

	t.Run("Plane AABB not intersecting normal AABB", func(t *testing.T) {
		normalAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		planeAABB := AABB{Min: mgl64.Vec3{-1, -1, 3}, Max: mgl64.Vec3{3, 3, 3}}
		if normalAABB.Overlaps(planeAABB) {
			t.Error("Plane AABB not intersecting should not overlap")
		}
	})
}

func TestAABBOverlaps_CornerTouching(t *testing.T) {
	t.Run("Single corner touching (max to min)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{2, 2, 2}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs with touching corners should overlap")
		}
		if !aabb2.Overlaps(aabb1) {
			t.Error("Symmetry: AABBs with touching corners should overlap")
		}
	})

	t.Run("Diagonal corner touching", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 1, 0}, Max: mgl64.Vec3{2, 2, 1}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs with diagonal corner touching should overlap")
		}
	})

	t.Run("Corner near but not touching", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{1.01, 1.01, 1.01}, Max: mgl64.Vec3{2, 2, 2}}
		if aabb1.Overlaps(aabb2) {
			t.Error("AABBs with corners not touching should not overlap")
		}
	})
}

func TestAABBOverlaps_MultiAxisSeparation(t *testing.T) {
	t.Run("Separated on X and Y, overlapping on Z", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 2}}
		aabb2 := AABB{Min: mgl64.Vec3{2, 2, 1}, Max: mgl64.Vec3{3, 3, 3}}
		if aabb1.Overlaps(aabb2) {
			t.Error("AABBs separated on X and Y should not overlap (even if overlapping on Z)")
		}
	})

	t.Run("Separated on X and Z, overlapping on Y", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 2, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{2, 1, 2}, Max: mgl64.Vec3{3, 3, 3}}
		if aabb1.Overlaps(aabb2) {
			t.Error("AABBs separated on X and Z should not overlap (even if overlapping on Y)")
		}
	})

	t.Run("Separated on Y and Z, overlapping on X", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 1, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 2, 2}, Max: mgl64.Vec3{3, 3, 3}}
		if aabb1.Overlaps(aabb2) {
			t.Error("AABBs separated on Y and Z should not overlap (even if overlapping on X)")
		}
	})

	t.Run("Separated on all three axes", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{2, 2, 2}, Max: mgl64.Vec3{3, 3, 3}}
		if aabb1.Overlaps(aabb2) {
			t.Error("AABBs separated on all axes should not overlap")
		}
	})
}

func TestAABBOverlaps_Reflexivity(t *testing.T) {
	tests := []struct {
		name string
		aabb AABB
	}{
		{
			name: "Normal AABB",
			aabb: AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}},
		},
		{
			name: "Point AABB",
			aabb: AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{1, 1, 1}},
		},
		{
			name: "Large AABB",
			aabb: AABB{Min: mgl64.Vec3{-1000, -1000, -1000}, Max: mgl64.Vec3{1000, 1000, 1000}},
		},
		{
			name: "Negative space AABB",
			aabb: AABB{Min: mgl64.Vec3{-5, -5, -5}, Max: mgl64.Vec3{-1, -1, -1}},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if !tt.aabb.Overlaps(tt.aabb) {
				t.Errorf("AABB should always overlap with itself (reflexivity)")
			}
		})
	}
}

func TestAABBOverlaps_FaceTouching(t *testing.T) {
	t.Run("Face touching on X axis (same size)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 2, 2}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs with touching faces should overlap")
		}
	})

	t.Run("Face touching on Y axis (same size)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 1, 2}}
		aabb2 := AABB{Min: mgl64.Vec3{0, 1, 0}, Max: mgl64.Vec3{2, 2, 2}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs with touching faces should overlap")
		}
	})

	t.Run("Face touching on Z axis (same size)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{0, 0, 1}, Max: mgl64.Vec3{2, 2, 2}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs with touching faces should overlap")
		}
	})

	t.Run("Face touching (different sizes)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 3, 3}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 0, 0}, Max: mgl64.Vec3{2, 1, 1}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs with touching faces (different sizes) should overlap")
		}
	})
}

func TestAABBOverlaps_NegativeCoordinates(t *testing.T) {
	t.Run("Both AABBs in negative space (overlapping)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{-5, -5, -5}, Max: mgl64.Vec3{-3, -3, -3}}
		aabb2 := AABB{Min: mgl64.Vec3{-4, -4, -4}, Max: mgl64.Vec3{-2, -2, -2}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs in negative space should overlap if they intersect")
		}
	})

	t.Run("Both AABBs in negative space (separated)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{-10, -10, -10}, Max: mgl64.Vec3{-8, -8, -8}}
		aabb2 := AABB{Min: mgl64.Vec3{-5, -5, -5}, Max: mgl64.Vec3{-3, -3, -3}}
		if aabb1.Overlaps(aabb2) {
			t.Error("Separated AABBs in negative space should not overlap")
		}
	})

	t.Run("One in negative, one in positive (separated)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{-5, -5, -5}, Max: mgl64.Vec3{-1, -1, -1}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{5, 5, 5}}
		if aabb1.Overlaps(aabb2) {
			t.Error("AABBs on opposite sides of origin should not overlap")
		}
	})

	t.Run("Spanning origin (overlapping)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{-2, -2, -2}, Max: mgl64.Vec3{2, 2, 2}}
		aabb2 := AABB{Min: mgl64.Vec3{-1, -1, -1}, Max: mgl64.Vec3{1, 1, 1}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("AABBs spanning origin should overlap")
		}
	})
}

func TestAABBOverlaps_LargeSizeDifferences(t *testing.T) {
	t.Run("Very large AABB vs very small AABB (contained)", func(t *testing.T) {
		largeAABB := AABB{Min: mgl64.Vec3{-1000, -1000, -1000}, Max: mgl64.Vec3{1000, 1000, 1000}}
		smallAABB := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{0.001, 0.001, 0.001}}
		if !largeAABB.Overlaps(smallAABB) {
			t.Error("Small AABB inside large AABB should overlap")
		}
		if !smallAABB.Overlaps(largeAABB) {
			t.Error("Symmetry: should overlap")
		}
	})

	t.Run("Very large AABB vs very small AABB (separated)", func(t *testing.T) {
		largeAABB := AABB{Min: mgl64.Vec3{-1000, -1000, -1000}, Max: mgl64.Vec3{-500, -500, -500}}
		smallAABB := AABB{Min: mgl64.Vec3{500, 500, 500}, Max: mgl64.Vec3{500.001, 500.001, 500.001}}
		if largeAABB.Overlaps(smallAABB) {
			t.Error("Large and small AABBs far apart should not overlap")
		}
	})
}

func TestAABBOverlaps_DiagonalConfigurations(t *testing.T) {
	t.Run("Diagonal offset in 3D (overlapping)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{3, 3, 3}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("Diagonally overlapping AABBs should overlap")
		}
	})

	t.Run("Diagonal offset in 3D (separated)", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1, 1, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{2, 2, 2}, Max: mgl64.Vec3{3, 3, 3}}
		if aabb1.Overlaps(aabb2) {
			t.Error("Diagonally separated AABBs should not overlap")
		}
	})

	t.Run("Complex diagonal configuration", func(t *testing.T) {
		aabb1 := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{3, 1, 1}}
		aabb2 := AABB{Min: mgl64.Vec3{1, 0.5, 0.5}, Max: mgl64.Vec3{2, 2, 2}}
		if !aabb1.Overlaps(aabb2) {
			t.Error("Complex diagonal overlap should be detected")
		}
	})
}

func TestAABBOverlaps_NonTransitivity(t *testing.T) {
	// Démonstration que Overlaps n'est PAS transitif
	// A overlaps B, B overlaps C, mais A ne overlaps pas C
	t.Run("Non-transitivity demonstration", func(t *testing.T) {
		aabbA := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{2, 2, 2}}
		aabbB := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{3, 3, 3}}
		aabbC := AABB{Min: mgl64.Vec3{2.5, 2.5, 2.5}, Max: mgl64.Vec3{4, 4, 4}}

		// A overlaps B
		if !aabbA.Overlaps(aabbB) {
			t.Error("A should overlap B")
		}

		// B overlaps C
		if !aabbB.Overlaps(aabbC) {
			t.Error("B should overlap C")
		}

		// A does NOT overlap C (non-transitivity)
		if aabbA.Overlaps(aabbC) {
			t.Error("A should NOT overlap C (demonstrating non-transitivity)")
		}
	})
}

func TestAABBContainsPoint_AllCorners(t *testing.T) {
	aabb := AABB{Min: mgl64.Vec3{1, 2, 3}, Max: mgl64.Vec3{4, 5, 6}}

	// Test des 8 coins d'un cube
	corners := []struct {
		name  string
		point mgl64.Vec3
	}{
		{"Corner (min.x, min.y, min.z)", mgl64.Vec3{1, 2, 3}},
		{"Corner (max.x, max.y, max.z)", mgl64.Vec3{4, 5, 6}},
		{"Corner (min.x, min.y, max.z)", mgl64.Vec3{1, 2, 6}},
		{"Corner (min.x, max.y, min.z)", mgl64.Vec3{1, 5, 3}},
		{"Corner (max.x, min.y, min.z)", mgl64.Vec3{4, 2, 3}},
		{"Corner (min.x, max.y, max.z)", mgl64.Vec3{1, 5, 6}},
		{"Corner (max.x, min.y, max.z)", mgl64.Vec3{4, 2, 6}},
		{"Corner (max.x, max.y, min.z)", mgl64.Vec3{4, 5, 3}},
	}

	for _, tc := range corners {
		t.Run(tc.name, func(t *testing.T) {
			if !aabb.ContainsPoint(tc.point) {
				t.Errorf("Corner %v should be contained in AABB", tc.point)
			}
		})
	}
}

func TestAABBContainsPoint_EdgeMidpoints(t *testing.T) {
	aabb := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{4, 4, 4}}

	// Un cube a 12 arêtes, on teste le midpoint de chacune
	edges := []struct {
		name  string
		point mgl64.Vec3
	}{
		// 4 arêtes parallèles à X (y et z constants)
		{"Edge X (min.y, min.z)", mgl64.Vec3{2, 0, 0}},
		{"Edge X (max.y, min.z)", mgl64.Vec3{2, 4, 0}},
		{"Edge X (min.y, max.z)", mgl64.Vec3{2, 0, 4}},
		{"Edge X (max.y, max.z)", mgl64.Vec3{2, 4, 4}},

		// 4 arêtes parallèles à Y (x et z constants)
		{"Edge Y (min.x, min.z)", mgl64.Vec3{0, 2, 0}},
		{"Edge Y (max.x, min.z)", mgl64.Vec3{4, 2, 0}},
		{"Edge Y (min.x, max.z)", mgl64.Vec3{0, 2, 4}},
		{"Edge Y (max.x, max.z)", mgl64.Vec3{4, 2, 4}},

		// 4 arêtes parallèles à Z (x et y constants)
		{"Edge Z (min.x, min.y)", mgl64.Vec3{0, 0, 2}},
		{"Edge Z (max.x, min.y)", mgl64.Vec3{4, 0, 2}},
		{"Edge Z (min.x, max.y)", mgl64.Vec3{0, 4, 2}},
		{"Edge Z (max.x, max.y)", mgl64.Vec3{4, 4, 2}},
	}

	for _, tc := range edges {
		t.Run(tc.name, func(t *testing.T) {
			if !aabb.ContainsPoint(tc.point) {
				t.Errorf("Edge midpoint %v should be contained in AABB", tc.point)
			}
		})
	}
}

func TestAABBContainsPoint_FaceCenters(t *testing.T) {
	aabb := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{4, 4, 4}}

	// Un cube a 6 faces, on teste le centre de chacune
	faces := []struct {
		name  string
		point mgl64.Vec3
	}{
		{"Face -X (x=min)", mgl64.Vec3{0, 2, 2}},
		{"Face +X (x=max)", mgl64.Vec3{4, 2, 2}},
		{"Face -Y (y=min)", mgl64.Vec3{2, 0, 2}},
		{"Face +Y (y=max)", mgl64.Vec3{2, 4, 2}},
		{"Face -Z (z=min)", mgl64.Vec3{2, 2, 0}},
		{"Face +Z (z=max)", mgl64.Vec3{2, 2, 4}},
	}

	for _, tc := range faces {
		t.Run(tc.name, func(t *testing.T) {
			if !aabb.ContainsPoint(tc.point) {
				t.Errorf("Face center %v should be contained in AABB", tc.point)
			}
		})
	}
}

func TestAABBContainsPoint_ZeroVolumeAABB(t *testing.T) {
	t.Run("Point AABB contains exact point", func(t *testing.T) {
		pointAABB := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{1, 1, 1}}
		if !pointAABB.ContainsPoint(mgl64.Vec3{1, 1, 1}) {
			t.Error("Point AABB should contain its exact position")
		}
	})

	t.Run("Point AABB does not contain different point", func(t *testing.T) {
		pointAABB := AABB{Min: mgl64.Vec3{1, 1, 1}, Max: mgl64.Vec3{1, 1, 1}}
		if pointAABB.ContainsPoint(mgl64.Vec3{2, 2, 2}) {
			t.Error("Point AABB should not contain a different point")
		}
	})

	t.Run("Line AABB on X contains point on line", func(t *testing.T) {
		lineAABB := AABB{Min: mgl64.Vec3{0, 1, 1}, Max: mgl64.Vec3{5, 1, 1}}
		if !lineAABB.ContainsPoint(mgl64.Vec3{2.5, 1, 1}) {
			t.Error("Line AABB should contain point on the line")
		}
	})

	t.Run("Line AABB does not contain point off line", func(t *testing.T) {
		lineAABB := AABB{Min: mgl64.Vec3{0, 1, 1}, Max: mgl64.Vec3{5, 1, 1}}
		if lineAABB.ContainsPoint(mgl64.Vec3{2.5, 2, 1}) {
			t.Error("Line AABB should not contain point off the line")
		}
	})

	t.Run("Plane AABB XY contains point on plane", func(t *testing.T) {
		planeAABB := AABB{Min: mgl64.Vec3{0, 0, 5}, Max: mgl64.Vec3{10, 10, 5}}
		if !planeAABB.ContainsPoint(mgl64.Vec3{3, 7, 5}) {
			t.Error("Plane AABB should contain point on the plane")
		}
	})

	t.Run("Plane AABB does not contain point off plane", func(t *testing.T) {
		planeAABB := AABB{Min: mgl64.Vec3{0, 0, 5}, Max: mgl64.Vec3{10, 10, 5}}
		if planeAABB.ContainsPoint(mgl64.Vec3{3, 7, 6}) {
			t.Error("Plane AABB should not contain point off the plane")
		}
	})
}

func TestAABBContainsPoint_NegativeCoordinates(t *testing.T) {
	t.Run("AABB in negative space contains negative point", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{-5, -5, -5}, Max: mgl64.Vec3{-1, -1, -1}}
		point := mgl64.Vec3{-3, -3, -3}
		if !aabb.ContainsPoint(point) {
			t.Error("AABB in negative space should contain point inside it")
		}
	})

	t.Run("AABB in negative space does not contain positive point", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{-5, -5, -5}, Max: mgl64.Vec3{-1, -1, -1}}
		point := mgl64.Vec3{3, 3, 3}
		if aabb.ContainsPoint(point) {
			t.Error("AABB in negative space should not contain point in positive space")
		}
	})

	t.Run("AABB spanning origin contains origin", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{-2, -2, -2}, Max: mgl64.Vec3{2, 2, 2}}
		origin := mgl64.Vec3{0, 0, 0}
		if !aabb.ContainsPoint(origin) {
			t.Error("AABB spanning origin should contain origin")
		}
	})

	t.Run("AABB spanning origin contains negative point", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{-2, -2, -2}, Max: mgl64.Vec3{2, 2, 2}}
		point := mgl64.Vec3{-1, -1, -1}
		if !aabb.ContainsPoint(point) {
			t.Error("AABB spanning origin should contain negative point inside it")
		}
	})
}

func TestAABBContainsPoint_ExtremeValues(t *testing.T) {
	t.Run("Very large AABB with large point inside", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{-1e10, -1e10, -1e10}, Max: mgl64.Vec3{1e10, 1e10, 1e10}}
		point := mgl64.Vec3{5e9, 5e9, 5e9}
		if !aabb.ContainsPoint(point) {
			t.Error("Very large AABB should contain large point inside it")
		}
	})

	t.Run("Very small AABB with small point inside", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1e-10, 1e-10, 1e-10}}
		point := mgl64.Vec3{5e-11, 5e-11, 5e-11}
		if !aabb.ContainsPoint(point) {
			t.Error("Very small AABB should contain small point inside it")
		}
	})

	t.Run("Very small AABB at boundary (1e-10)", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1e-10, 1e-10, 1e-10}}
		point := mgl64.Vec3{1e-10, 1e-10, 1e-10}
		if !aabb.ContainsPoint(point) {
			t.Error("Small AABB should contain point at boundary")
		}
	})

	t.Run("Very small epsilon (1e-15)", func(t *testing.T) {
		aabb := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{1e-15, 1e-15, 1e-15}}
		point := mgl64.Vec3{5e-16, 5e-16, 5e-16}
		if !aabb.ContainsPoint(point) {
			t.Error("Tiny AABB should contain point inside it")
		}
	})
}

func TestAABBContainsPoint_BoundaryPrecision(t *testing.T) {
	aabb := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{10, 10, 10}}

	t.Run("Point just inside boundary (Min + epsilon)", func(t *testing.T) {
		epsilon := 1e-10
		point := mgl64.Vec3{0 + epsilon, 0 + epsilon, 0 + epsilon}
		if !aabb.ContainsPoint(point) {
			t.Error("Point just inside boundary should be contained")
		}
	})

	t.Run("Point just outside boundary (Min - epsilon)", func(t *testing.T) {
		epsilon := 1e-10
		point := mgl64.Vec3{0 - epsilon, 5, 5}
		if aabb.ContainsPoint(point) {
			t.Error("Point just outside boundary should not be contained")
		}
	})

	t.Run("Point just inside max boundary (Max - epsilon)", func(t *testing.T) {
		epsilon := 1e-10
		point := mgl64.Vec3{10 - epsilon, 10 - epsilon, 10 - epsilon}
		if !aabb.ContainsPoint(point) {
			t.Error("Point just inside max boundary should be contained")
		}
	})

	t.Run("Point just outside max boundary (Max + epsilon)", func(t *testing.T) {
		epsilon := 1e-10
		point := mgl64.Vec3{10 + epsilon, 5, 5}
		if aabb.ContainsPoint(point) {
			t.Error("Point just outside max boundary should not be contained")
		}
	})

	t.Run("Very small epsilon precision (1e-15)", func(t *testing.T) {
		epsilon := 1e-15
		point := mgl64.Vec3{0 + epsilon, 5, 5}
		if !aabb.ContainsPoint(point) {
			t.Error("Point with tiny epsilon inside should be contained")
		}
	})
}

func TestAABBContainsPoint_ContainmentHierarchy(t *testing.T) {
	t.Run("Point in nested AABBs", func(t *testing.T) {
		// Hiérarchie: AABB_large contient AABB_medium qui contient AABB_small
		aabbLarge := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{10, 10, 10}}
		aabbMedium := AABB{Min: mgl64.Vec3{2, 2, 2}, Max: mgl64.Vec3{8, 8, 8}}
		aabbSmall := AABB{Min: mgl64.Vec3{4, 4, 4}, Max: mgl64.Vec3{6, 6, 6}}
		point := mgl64.Vec3{5, 5, 5}

		// Point devrait être contenu dans les trois
		if !aabbSmall.ContainsPoint(point) {
			t.Error("Point should be in smallest AABB")
		}
		if !aabbMedium.ContainsPoint(point) {
			t.Error("Point should be in medium AABB")
		}
		if !aabbLarge.ContainsPoint(point) {
			t.Error("Point should be in largest AABB")
		}
	})

	t.Run("Point in outer AABB but not in inner", func(t *testing.T) {
		aabbOuter := AABB{Min: mgl64.Vec3{0, 0, 0}, Max: mgl64.Vec3{10, 10, 10}}
		aabbInner := AABB{Min: mgl64.Vec3{4, 4, 4}, Max: mgl64.Vec3{6, 6, 6}}
		point := mgl64.Vec3{1, 1, 1} // Dans outer, pas dans inner

		if !aabbOuter.ContainsPoint(point) {
			t.Error("Point should be in outer AABB")
		}
		if aabbInner.ContainsPoint(point) {
			t.Error("Point should not be in inner AABB")
		}
	})
}
