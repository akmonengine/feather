package actor

import (
	"math"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// Helper functions
func vec3Equal(a, b mgl64.Vec3, tolerance float64) bool {
	return math.Abs(a.X()-b.X()) < tolerance &&
		math.Abs(a.Y()-b.Y()) < tolerance &&
		math.Abs(a.Z()-b.Z()) < tolerance
}

func floatEqual(a, b, tolerance float64) bool {
	return math.Abs(a-b) < tolerance
}

// Helper function pour comparer les matrices 3x3
func mat3Equal(a, b mgl64.Mat3, tolerance float64) bool {
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if math.Abs(a.At(i, j)-b.At(i, j)) >= tolerance {
				return false
			}
		}
	}
	return true
}

// ========== INERTIA MATRIX TESTS (NOUVEAUX - CRITIQUES) ==========
func TestBoxComputeInertia(t *testing.T) {
	tests := []struct {
		name         string
		box          *Box
		mass         float64
		expectedDiag mgl64.Vec3 // diagonal elements (ix, iy, iz)
	}{
		{
			name:         "unit cube",
			box:          &Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			mass:         12.0,                // m/12 = 1.0
			expectedDiag: mgl64.Vec3{8, 8, 8}, // (2*2 + 2*2, 2*2 + 2*2, 2*2 + 2*2)
		},
		{
			name:         "rectangular box 2x3x4",
			box:          &Box{HalfExtents: mgl64.Vec3{2, 3, 4}},
			mass:         12.0,
			expectedDiag: mgl64.Vec3{100, 80, 52}, // (m/12)*(3²+4²), (m/12)*(2²+4²), (m/12)*(2²+3²)
		},
		{
			name:         "thin box",
			box:          &Box{HalfExtents: mgl64.Vec3{0.1, 5, 0.1}},
			mass:         60.0,
			expectedDiag: mgl64.Vec3{500.2, 0.4, 500.2},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.box.ComputeInertia(tt.mass)

			// Vérifier que c'est une matrice diagonale (hors diagonale = 0)
			if !floatEqual(result.At(0, 1), 0.0, 1e-9) || !floatEqual(result.At(0, 2), 0.0, 1e-9) ||
				!floatEqual(result.At(1, 0), 0.0, 1e-9) || !floatEqual(result.At(1, 2), 0.0, 1e-9) ||
				!floatEqual(result.At(2, 0), 0.0, 1e-9) || !floatEqual(result.At(2, 1), 0.0, 1e-9) {
				t.Errorf("ComputeInertia() returned non-diagonal matrix: %v", result)
			}

			// Vérifier les éléments diagonaux
			if !vec3Equal(result.Diag(), tt.expectedDiag, 1e-6) {
				t.Errorf("ComputeInertia() diagonal = %v, want %v", result.Diag(), tt.expectedDiag)
			}
		})
	}
}

func TestSphereComputeInertia(t *testing.T) {
	tests := []struct {
		name      string
		sphere    *Sphere
		mass      float64
		expectedI float64 // tous les éléments doivent être égaux pour une sphère
	}{
		{
			name:      "unit sphere",
			sphere:    &Sphere{Radius: 1.0},
			mass:      5.0,
			expectedI: (2.0 / 5.0) * 5.0 * 1.0 * 1.0, // 2
		},
		{
			name:      "sphere radius 2",
			sphere:    &Sphere{Radius: 2.0},
			mass:      10.0,
			expectedI: (2.0 / 5.0) * 10.0 * 4.0, // 16
		},
		{
			name:      "small sphere",
			sphere:    &Sphere{Radius: 0.5},
			mass:      1.0,
			expectedI: (2.0 / 5.0) * 1.0 * 0.25, // 0.1
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := tt.sphere.ComputeInertia(tt.mass)

			// Vérifier que c'est une matrice diagonale avec tous les éléments égaux
			expectedMat := mgl64.Mat3{
				tt.expectedI, 0, 0,
				0, tt.expectedI, 0,
				0, 0, tt.expectedI,
			}

			for i := 0; i < 3; i++ {
				for j := 0; j < 3; j++ {
					if !floatEqual(result.At(i, j), expectedMat.At(i, j), 1e-9) {
						t.Errorf("ComputeInertia()[%d][%d] = %v, want %v", i, j, result.At(i, j), expectedMat.At(i, j))
					}
				}
			}
		})
	}
}

func TestPlaneComputeInertia(t *testing.T) {
	plane := &Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: 0}
	mass := 1.0

	result := plane.ComputeInertia(mass)

	// Un plan doit avoir une inertie infinie
	if !result.ApproxEqual(mgl64.Mat3{}) {
		t.Errorf("ComputeInertia() = %v, want a 0 matrix to simulate an infinite inertia", result)
	}
}

// ========== ROTATION TESTS (PRIORITÉ CRITIQUE) ==========
func TestBoxComputeAABBWithRotation(t *testing.T) {
	tests := []struct {
		name        string
		box         *Box
		transform   Transform
		expectedMin mgl64.Vec3
		expectedMax mgl64.Vec3
	}{
		{
			name: "rotation 90° around Z-axis",
			box:  &Box{HalfExtents: mgl64.Vec3{1, 2, 3}},
			transform: Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(mgl64.DegToRad(90), mgl64.Vec3{0, 0, 1}),
			},
			expectedMin: mgl64.Vec3{-2, -1, -3},
			expectedMax: mgl64.Vec3{2, 1, 3},
		},
		{
			name: "rotation 45° around Y-axis",
			box:  &Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			transform: Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(mgl64.DegToRad(45), mgl64.Vec3{0, 1, 0}),
			},
			expectedMin: mgl64.Vec3{-1.4142, -1, -1.4142}, // approx sqrt(2)
			expectedMax: mgl64.Vec3{1.4142, 1, 1.4142},
		},
		{
			name: "rotation 180° around X-axis",
			box:  &Box{HalfExtents: mgl64.Vec3{1, 2, 3}},
			transform: Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(mgl64.DegToRad(180), mgl64.Vec3{1, 0, 0}),
			},
			expectedMin: mgl64.Vec3{-1, -2, -3}, // 180° rotation preserves AABB
			expectedMax: mgl64.Vec3{1, 2, 3},
		},
		{
			name: "rotation with offset position",
			box:  &Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
			transform: Transform{
				Position: mgl64.Vec3{5, 10, -3},
				Rotation: mgl64.QuatRotate(mgl64.DegToRad(90), mgl64.Vec3{0, 0, 1}),
			},
			expectedMin: mgl64.Vec3{4, 9, -4},
			expectedMax: mgl64.Vec3{6, 11, -2},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tt.box.ComputeAABB(tt.transform)
			aabb := tt.box.GetAABB()

			// Vérifications de base
			if !vec3Equal(aabb.Min, tt.expectedMin, 1e-3) {
				t.Errorf("Min = %v, want %v (tolerance 1e-3)", aabb.Min, tt.expectedMin)
			}
			if !vec3Equal(aabb.Max, tt.expectedMax, 1e-3) {
				t.Errorf("Max = %v, want %v (tolerance 1e-3)", aabb.Max, tt.expectedMax)
			}

			// Vérification que l'AABB est valide (Min <= Max sur tous les axes)
			if aabb.Min[0] > aabb.Max[0] || aabb.Min[1] > aabb.Max[1] || aabb.Min[2] > aabb.Max[2] {
				t.Errorf("Invalid AABB: Min %v > Max %v", aabb.Min, aabb.Max)
			}
		})
	}
}

func TestBoxSupportWithRotation(t *testing.T) {
	box := &Box{HalfExtents: mgl64.Vec3{2, 3, 4}}

	// La méthode Support ne dépend pas de la rotation du Transform
	// Elle est toujours calculée dans l'espace local de la forme
	direction := mgl64.Vec3{1, 0, 0}
	support := box.Support(direction)

	// Doit retourner le point dans la direction +X dans l'espace local
	expected := mgl64.Vec3{2, 3, 4} // (hx, hy, hz) pour direction +X

	if !vec3Equal(support, expected, 1e-9) {
		t.Errorf("Support with direction %v = %v, want %v", direction, support, expected)
	}
}

func TestBoxGetContactFeatureWithRotation(t *testing.T) {
	box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}

	direction := mgl64.Vec3{1, 0, 0}
	var features [8]mgl64.Vec3
	var featureCount int
	box.GetContactFeature(direction, &features, &featureCount)

	// GetContactFeature retourne toujours les points en espace local
	// Doit retourner 4 points (une face)
	if featureCount != 4 {
		t.Errorf("GetContactFeature() returned %d points, want 4", featureCount)
	}

	// Vérifier que tous les points sont des vertices valides du cube
	for i, vertex := range features {
		if math.Abs(vertex.X()) > 1.0001 || math.Abs(vertex.Y()) > 2.0001 || math.Abs(vertex.Z()) > 3.0001 {
			t.Errorf("Vertex %d = %v, exceeds box bounds (±1, ±2, ±3)", i, vertex)
		}
	}
}

// ========== EDGE CASES & ROBUSTNESS TESTS ==========

func TestShapeEdgeCases(t *testing.T) {
	t.Run("Box with zero dimensions", func(t *testing.T) {
		box := &Box{HalfExtents: mgl64.Vec3{0, 0, 0}}

		// Mass should be zero
		mass := box.ComputeMass(1.0)
		if !floatEqual(mass, 0.0, 1e-9) {
			t.Errorf("Zero box mass = %v, want 0", mass)
		}

		// Support should be zero vector
		support := box.Support(mgl64.Vec3{1, 0, 0})
		if !vec3Equal(support, mgl64.Vec3{0, 0, 0}, 1e-9) {
			t.Errorf("Zero box support = %v, want (0,0,0)", support)
		}
	})

	t.Run("Sphere with zero radius", func(t *testing.T) {
		sphere := &Sphere{Radius: 0.0}

		mass := sphere.ComputeMass(1.0)
		if !floatEqual(mass, 0.0, 1e-9) {
			t.Errorf("Zero radius sphere mass = %v, want 0", mass)
		}

		inertia := sphere.ComputeInertia(1.0)
		expected := mgl64.Mat3{0, 0, 0, 0, 0, 0, 0, 0, 0}
		if !mat3Equal(inertia, expected, 1e-9) {
			t.Errorf("Zero radius sphere inertia = %v, want zero matrix", inertia)
		}
	})

	t.Run("Zero density", func(t *testing.T) {
		box := &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}
		sphere := &Sphere{Radius: 1.0}

		boxMass := box.ComputeMass(0.0)
		sphereMass := sphere.ComputeMass(0.0)

		if !floatEqual(boxMass, 0.0, 1e-9) || !floatEqual(sphereMass, 0.0, 1e-9) {
			t.Errorf("Zero density masses: box=%v, sphere=%v, want 0", boxMass, sphereMass)
		}
	})
}

func TestShapeConsistency(t *testing.T) {
	t.Run("Support vs GetContactFeature consistency", func(t *testing.T) {
		box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
		sphere := &Sphere{Radius: 2.0}

		directions := []mgl64.Vec3{
			{1, 0, 0}, {-1, 0, 0},
			{0, 1, 0}, {0, -1, 0},
			{0, 0, 1}, {0, 0, -1},
			{1, 1, 1}, mgl64.Vec3{-1, -1, -1}.Normalize(),
		}

		for _, dir := range directions {
			var features [8]mgl64.Vec3
			var featureCount int
			// Box test
			boxSupport := box.Support(dir)
			box.GetContactFeature(dir, &features, &featureCount)

			// Le support doit être l'un des points de contact
			found := false
			for _, feature := range features {
				if vec3Equal(boxSupport, feature, 1e-6) {
					found = true
					break
				}
			}
			if !found {
				t.Errorf("Box support point %v not found in contact features %v", boxSupport, features)
			}

			// Sphere test
			sphereSupport := sphere.Support(dir)
			sphere.GetContactFeature(dir, &features, &featureCount)

			// Pour une sphère, GetContactFeature doit retourner exactement le point de support
			if featureCount != 1 || !vec3Equal(sphereSupport, features[0], 1e-9) {
				t.Errorf("Sphere support/contact feature mismatch: support=%v, feature=%v", sphereSupport, features)
			}
		}
	})

	t.Run("AABB consistency", func(t *testing.T) {
		box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
		transform := Transform{
			Position: mgl64.Vec3{5, 10, 15},
			Rotation: mgl64.QuatRotate(mgl64.DegToRad(45), mgl64.Vec3{0, 0, 1}),
		}

		box.ComputeAABB(transform)
		aabb := box.GetAABB()

		// L'AABB doit contenir tous les coins transformés
		corners := [8]mgl64.Vec3{
			{-1, -2, -3}, {1, -2, -3}, {-1, 2, -3}, {1, 2, -3},
			{-1, -2, 3}, {1, -2, 3}, {-1, 2, 3}, {1, 2, 3},
		}

		for i, corner := range corners {
			worldCorner := transform.Rotation.Rotate(corner).Add(transform.Position)

			// Vérifier que chaque coin est dans l'AABB
			if worldCorner[0] < aabb.Min[0] || worldCorner[0] > aabb.Max[0] ||
				worldCorner[1] < aabb.Min[1] || worldCorner[1] > aabb.Max[1] ||
				worldCorner[2] < aabb.Min[2] || worldCorner[2] > aabb.Max[2] {
				t.Errorf("Corner %d = %v is outside AABB [Min:%v, Max:%v]", i, worldCorner, aabb.Min, aabb.Max)
			}
		}
	})

	t.Run("Mass-Inertia consistency", func(t *testing.T) {
		box := &Box{HalfExtents: mgl64.Vec3{1, 2, 3}}
		density := 2.5
		mass := box.ComputeMass(density)
		inertia := box.ComputeInertia(mass)

		diag := inertia.Diag()
		if diag[0] <= 0 || diag[1] <= 0 || diag[2] <= 0 {
			t.Errorf("Box inertia matrix has non-positive diagonal elements: %v", diag)
		}

		// Dimensions complètes
		wx, wy, wz := 2*box.HalfExtents.X(), 2*box.HalfExtents.Y(), 2*box.HalfExtents.Z()
		// Formule : trace = m/12 * (wy²+wz² + wx²+wz² + wx²+wy²) = m/12 * 2(wx² + wy² + wz²)
		expectedTrace := mass * (wx*wx + wy*wy + wz*wz) * 2 / 12.0

		if !floatEqual(diag[0]+diag[1]+diag[2], expectedTrace, 1e-6) {
			t.Errorf("Inertia trace = %v, want %v", diag[0]+diag[1]+diag[2], expectedTrace)
		}
	})
}

func TestSphereComputeAABB(t *testing.T) {
	tests := []struct {
		name        string
		sphere      *Sphere
		transform   Transform
		expectedMin mgl64.Vec3
		expectedMax mgl64.Vec3
	}{
		{
			name:   "sphere at origin",
			sphere: &Sphere{Radius: 2.0},
			transform: Transform{
				Position: mgl64.Vec3{0, 0, 0},
				Rotation: mgl64.QuatRotate(mgl64.DegToRad(45), mgl64.Vec3{1, 0, 0}),
			},
			expectedMin: mgl64.Vec3{-2, -2, -2},
			expectedMax: mgl64.Vec3{2, 2, 2},
		},
		{
			name:   "sphere with offset position",
			sphere: &Sphere{Radius: 1.5},
			transform: Transform{
				Position: mgl64.Vec3{3, -2, 5},
				Rotation: mgl64.QuatRotate(mgl64.DegToRad(90), mgl64.Vec3{0, 0, 1}),
			},
			expectedMin: mgl64.Vec3{1.5, -3.5, 3.5},
			expectedMax: mgl64.Vec3{4.5, -0.5, 6.5},
		},
		{
			name:   "small sphere",
			sphere: &Sphere{Radius: 0.1},
			transform: Transform{
				Position: mgl64.Vec3{10, 20, 30},
				Rotation: mgl64.QuatRotate(mgl64.DegToRad(180), mgl64.Vec3{1, 1, 1}),
			},
			expectedMin: mgl64.Vec3{9.9, 19.9, 29.9},
			expectedMax: mgl64.Vec3{10.1, 20.1, 30.1},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tt.sphere.ComputeAABB(tt.transform)
			aabb := tt.sphere.GetAABB()

			// Vérifications de base
			if !vec3Equal(aabb.Min, tt.expectedMin, 1e-9) {
				t.Errorf("Min = %v, want %v", aabb.Min, tt.expectedMin)
			}
			if !vec3Equal(aabb.Max, tt.expectedMax, 1e-9) {
				t.Errorf("Max = %v, want %v", aabb.Max, tt.expectedMax)
			}

			// Vérification que l'AABB est valide (Min <= Max sur tous les axes)
			if aabb.Min[0] > aabb.Max[0] || aabb.Min[1] > aabb.Max[1] || aabb.Min[2] > aabb.Max[2] {
				t.Errorf("Invalid AABB: Min %v > Max %v", aabb.Min, aabb.Max)
			}

			// Vérifier que la rotation n'affecte pas l'AABB de la sphère
			// (ce qui différencie la sphère de la boîte)
			transformNoRotation := Transform{
				Position: tt.transform.Position,
				Rotation: mgl64.QuatIdent(),
			}

			tt.sphere.ComputeAABB(transformNoRotation)
			aabbNoRotation := tt.sphere.GetAABB()
			if !aabb.Min.ApproxEqual(aabbNoRotation.Min) || !aabb.Max.ApproxEqual(aabbNoRotation.Max) {
				t.Errorf("Sphere AABB affected by rotation, but should not be")
			}
		})
	}
}

func TestGetTangentBasis(t *testing.T) {
	tests := []struct {
		name            string
		normal          mgl64.Vec3
		expectedLengths [2]float64 // longueur attendue des deux tangents
	}{
		{
			name:            "X-axis normal",
			normal:          mgl64.Vec3{1, 0, 0},
			expectedLengths: [2]float64{1, 1},
		},
		{
			name:            "Y-axis normal",
			normal:          mgl64.Vec3{0, 1, 0},
			expectedLengths: [2]float64{1, 1},
		},
		{
			name:            "Z-axis normal",
			normal:          mgl64.Vec3{0, 0, 1},
			expectedLengths: [2]float64{1, 1},
		},
		{
			name:            "diagonal normal",
			normal:          mgl64.Vec3{1, 1, 1}.Normalize(),
			expectedLengths: [2]float64{1, 1},
		},
		{
			name:            "arbitrary normal",
			normal:          mgl64.Vec3{0.5, 0.8, 0.3}.Normalize(),
			expectedLengths: [2]float64{1, 1},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			tangent1, tangent2 := getTangentBasis(tt.normal)

			// Les tangents doivent avoir une longueur de 1
			if !floatEqual(tangent1.Len(), tt.expectedLengths[0], 1e-6) {
				t.Errorf("Tangent1 length = %v, want %v", tangent1.Len(), tt.expectedLengths[0])
			}
			if !floatEqual(tangent2.Len(), tt.expectedLengths[1], 1e-6) {
				t.Errorf("Tangent2 length = %v, want %v", tangent2.Len(), tt.expectedLengths[1])
			}

			// Les tangents doivent être perpendiculaires à la normale
			if math.Abs(tangent1.Dot(tt.normal)) > 1e-6 {
				t.Errorf("Tangent1 not perpendicular to normal: dot = %v", tangent1.Dot(tt.normal))
			}
			if math.Abs(tangent2.Dot(tt.normal)) > 1e-6 {
				t.Errorf("Tangent2 not perpendicular to normal: dot = %v", tangent2.Dot(tt.normal))
			}

			// Les deux tangents doivent être perpendiculaires entre elles
			if math.Abs(tangent1.Dot(tangent2)) > 1e-6 {
				t.Errorf("Tangents not perpendicular to each other: dot = %v", tangent1.Dot(tangent2))
			}

			// Le produit vectoriel normal x tangent1 doit donner tangent2 (ou son opposé)
			cross := tt.normal.Cross(tangent1)
			if !vec3Equal(cross, tangent2, 1e-6) && !vec3Equal(cross, tangent2.Mul(-1), 1e-6) {
				t.Errorf("Cross product not equal to tangent2: cross=%v, tangent2=%v", cross, tangent2)
			}
		})
	}
}
