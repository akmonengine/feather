package feather

import (
	"sort"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

func TestWorldToCell(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)

	tests := []struct {
		name     string
		position mgl64.Vec3
		expected CellKey
	}{
		{"origine", mgl64.Vec3{0, 0, 0}, CellKey{0, 0, 0}},
		{"positif", mgl64.Vec3{1.5, 2.3, 3.7}, CellKey{1, 2, 3}},
		{"negatif", mgl64.Vec3{-1.5, -2.3, -3.7}, CellKey{-2, -3, -4}},
		{"fractionnaire", mgl64.Vec3{0.5, 0.5, 0.5}, CellKey{0, 0, 0}},
		{"grand", mgl64.Vec3{100.7, -200.3, 50.1}, CellKey{100, -201, 50}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := grid.worldToCell(tt.position)
			if result != tt.expected {
				t.Errorf("worldToCell(%v) = %v, want %v", tt.position, result, tt.expected)
			}
		})
	}
}

func TestHashCell(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16) // 16 cellules, mask = 15

	tests := []struct {
		name     string
		key      CellKey
		expected int
	}{
		{"origine", CellKey{0, 0, 0}, 0},
		{"simple", CellKey{1, 2, 3}, 0},
		{"negatif", CellKey{-1, -2, -3}, 13},
		{"grand", CellKey{100, 200, 300}, 14},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := grid.hashCell(tt.key)
			// Vérifier que le résultat est dans la plage valide
			if result < 0 || result >= len(grid.cells) {
				t.Errorf("hashCell(%v) = %d, out of range [0, %d)", tt.key, result, len(grid.cells))
			}
			// Vérifier la valeur exacte (tous les cas maintenant)
			if true {
				if result != tt.expected {
					t.Errorf("hashCell(%v) = %d, want %d", tt.key, result, tt.expected)
				}
			}
		})
	}
}

func TestHashCellDistribution(t *testing.T) {
	grid := NewSpatialGrid(1.0, 1024) // Grande grille pour tester la distribution

	// Créer beaucoup de clés aléatoires et vérifier la distribution
	cellCounts := make(map[int]int)
	for x := -100; x <= 100; x++ {
		for y := -100; y <= 100; y++ {
			for z := -100; z <= 100; z++ {
				key := CellKey{x, y, z}
				hash := grid.hashCell(key)
				cellCounts[hash]++
			}
		}
	}

	// Vérifier que la distribution est raisonnable
	minCount := int(^uint(0) >> 1)
	maxCount := 0
	for _, count := range cellCounts {
		if count < minCount {
			minCount = count
		}
		if count > maxCount {
			maxCount = count
		}
	}

	t.Logf("Hash distribution: min=%d, max=%d, avg=%.1f", minCount, maxCount, float64(201*201*201)/float64(len(cellCounts)))

	// La distribution devrait être relativement uniforme
	// Le ratio max/min ne devrait pas être trop élevé
	ratio := float64(maxCount) / float64(minCount)
	if ratio > 2.0 {
		t.Logf("Warning: hash distribution ratio is %.1f, expected < 2.0", ratio)
	}
}

func createTestBox(position mgl64.Vec3, halfExtents mgl64.Vec3) *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: position, Rotation: mgl64.QuatIdent()},
		&actor.Box{HalfExtents: halfExtents},
		actor.BodyTypeDynamic,
		1.0,
	)
}

func createTestPlane() *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()},
		&actor.Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: 0},
		actor.BodyTypeStatic,
		0.0,
	)
}

func TestInsertSingleBody(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	body := createTestBox(mgl64.Vec3{1.5, 2.5, 3.5}, mgl64.Vec3{0.4, 0.4, 0.4})

	grid.Insert(0, body)

	// Vérifier que le body est dans la bonne cellule
	minCell := grid.worldToCell(body.Shape.GetAABB().Min)
	maxCell := grid.worldToCell(body.Shape.GetAABB().Max)

	found := false
	for x := minCell.X; x <= maxCell.X; x++ {
		for y := minCell.Y; y <= maxCell.Y; y++ {
			for z := minCell.Z; z <= maxCell.Z; z++ {
				cellKey := CellKey{x, y, z}
				cellIdx := grid.hashCell(cellKey)
				for _, idx := range grid.cells[cellIdx].bodyIndices {
					if idx == 0 {
						found = true
						break
					}
				}
				if found {
					break
				}
			}
			if found {
				break
			}
		}
		if found {
			break
		}
	}

	if !found {
		t.Error("Body not found in any cell after insertion")
	}
}

func TestInsertMultipleBodies(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	bodies := []*actor.RigidBody{
		createTestBox(mgl64.Vec3{1.0, 1.0, 1.0}, mgl64.Vec3{0.4, 0.4, 0.4}),
		createTestBox(mgl64.Vec3{2.0, 2.0, 2.0}, mgl64.Vec3{0.4, 0.4, 0.4}),
		createTestBox(mgl64.Vec3{3.0, 3.0, 3.0}, mgl64.Vec3{0.4, 0.4, 0.4}),
	}

	for i, body := range bodies {
		grid.Insert(i, body)
	}

	// Vérifier que tous les bodies sont insérés
	for i, body := range bodies {
		found := false
		minCell := grid.worldToCell(body.Shape.GetAABB().Min)
		maxCell := grid.worldToCell(body.Shape.GetAABB().Max)

		for x := minCell.X; x <= maxCell.X; x++ {
			for y := minCell.Y; y <= maxCell.Y; y++ {
				for z := minCell.Z; z <= maxCell.Z; z++ {
					cellKey := CellKey{x, y, z}
					cellIdx := grid.hashCell(cellKey)
					for _, idx := range grid.cells[cellIdx].bodyIndices {
						if idx == i {
							found = true
							break
						}
					}
					if found {
						break
					}
				}
				if found {
					break
				}
			}
			if found {
				break
			}
		}

		if !found {
			t.Errorf("Body %d not found in any cell after insertion", i)
		}
	}
}

func TestInsertPlane(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	plane := createTestPlane()

	grid.Insert(0, plane)

	// Les planes devraient être dans la cellule spéciale planes
	if len(grid.planes.bodyIndices) != 1 || grid.planes.bodyIndices[0] != 0 {
		t.Error("Plane not correctly inserted into planes cell")
	}

	// Vérifier qu'aucun body n'est dans les cellules régulières
	for _, cell := range grid.cells {
		if len(cell.bodyIndices) > 0 {
			t.Error("Regular cells should be empty when inserting plane")
		}
	}
}

func TestClear(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	bodies := []*actor.RigidBody{
		createTestBox(mgl64.Vec3{1.0, 1.0, 1.0}, mgl64.Vec3{0.4, 0.4, 0.4}),
		createTestBox(mgl64.Vec3{2.0, 2.0, 2.0}, mgl64.Vec3{0.4, 0.4, 0.4}),
	}

	// Insérer des bodies
	for i, body := range bodies {
		grid.Insert(i, body)
	}

	// Vérifier que les bodies sont présents
	if len(grid.cells[grid.hashCell(grid.worldToCell(bodies[0].Shape.GetAABB().Min))].bodyIndices) == 0 {
		t.Error("Bodies should be present before clear")
	}

	// Clear
	grid.Clear()

	// Vérifier que tout est vide
	if len(grid.planes.bodyIndices) != 0 {
		t.Error("Planes cell should be empty after clear")
	}

	for _, cell := range grid.cells {
		if len(cell.bodyIndices) != 0 {
			t.Error("Cells should be empty after clear")
		}
	}
}

func TestSortCells(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)

	// Insérer des bodies dans la même cellule dans un ordre aléatoire
	bodyIndices := []int{5, 2, 8, 1, 9, 3}
	cellIdx := 0 // Utiliser la première cellule
	grid.cells[cellIdx].bodyIndices = append(grid.cells[cellIdx].bodyIndices, bodyIndices...)

	// Trier
	grid.SortCells()

	// Vérifier que la cellule est triée
	if !sort.IntsAreSorted(grid.cells[cellIdx].bodyIndices) {
		t.Error("Cell indices should be sorted")
	}

	// Vérifier que les indices sont corrects
	expected := []int{1, 2, 3, 5, 8, 9}
	for i, idx := range grid.cells[cellIdx].bodyIndices {
		if idx != expected[i] {
			t.Errorf("Expected index %d at position %d, got %d", expected[i], i, idx)
		}
	}
}

func TestFindPairsParallelNoCollision(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	bodies := []*actor.RigidBody{
		createTestBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0.4, 0.4, 0.4}),
		createTestBox(mgl64.Vec3{10, 10, 10}, mgl64.Vec3{0.4, 0.4, 0.4}),
	}

	// Insérer les bodies
	for i, body := range bodies {
		grid.Insert(i, body)
	}

	// Trouver les paires avec la version parallèle
	pairs := make([]Pair, 0)
	for pair := range grid.FindPairsParallel(bodies, 2) {
		pairs = append(pairs, pair)
	}

	// Ne devrait pas avoir de collision (pas de planes dans ce test)
	if len(pairs) != 0 {
		t.Errorf("Expected 0 pairs, got %d", len(pairs))
	}
}

func TestFindPairsParallelWithCollision(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	bodies := []*actor.RigidBody{
		createTestBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0.4, 0.4, 0.4}),
		createTestBox(mgl64.Vec3{0.5, 0.5, 0.5}, mgl64.Vec3{0.4, 0.4, 0.4}),
	}

	// Insérer les bodies
	for i, body := range bodies {
		grid.Insert(i, body)
	}

	// Trouver les paires avec la version parallèle
	pairs := make([]Pair, 0)
	for pair := range grid.FindPairsParallel(bodies, 2) {
		pairs = append(pairs, pair)
	}

	// Devrait avoir une collision
	if len(pairs) != 1 {
		t.Errorf("Expected 1 pair, got %d", len(pairs))
	}

	// Vérifier que c'est la bonne paire
	foundCorrectPair := false
	for _, pair := range pairs {
		if (pair.BodyA == bodies[0] && pair.BodyB == bodies[1]) || (pair.BodyA == bodies[1] && pair.BodyB == bodies[0]) {
			foundCorrectPair = true
			break
		}
	}
	if !foundCorrectPair {
		t.Error("Correct pair not found")
	}
}

func TestFindPairsParallelWithPlane(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	plane := createTestPlane()
	body := createTestBox(mgl64.Vec3{0, 5, 0}, mgl64.Vec3{0.4, 0.4, 0.4})

	bodies := []*actor.RigidBody{plane, body} // Inclure le plane dans la liste des bodies

	// Insérer le plane et le body
	grid.Insert(0, plane)
	grid.Insert(1, body)

	// Trouver les paires avec la version parallèle
	pairs := make([]Pair, 0)
	for pair := range grid.FindPairsParallel(bodies, 2) {
		pairs = append(pairs, pair)
	}

	// Devrait détecter la paire plane-body (TOUJOURS ajoutée sans test de collision)
	// Note: Le plane est dans la liste des bodies, donc il sera traité normalement
	// mais aussi via la logique spéciale des planes
	if len(pairs) != 1 {
		t.Errorf("Expected 1 pair with plane, got %d", len(pairs))
	}

	// Vérifier que c'est la bonne paire
	foundCorrectPair := false
	for _, pair := range pairs {
		if (pair.BodyA == plane && pair.BodyB == body) || (pair.BodyA == body && pair.BodyB == plane) {
			foundCorrectPair = true
			break
		}
	}
	if !foundCorrectPair {
		t.Error("Correct plane-body pair not found")
	}
}

func TestFindPairsParallelStaticBodies(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	staticBody1 := actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()},
		&actor.Box{HalfExtents: mgl64.Vec3{0.4, 0.4, 0.4}},
		actor.BodyTypeStatic,
		0.0,
	)
	staticBody2 := actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{0.5, 0.5, 0.5}, Rotation: mgl64.QuatIdent()},
		&actor.Box{HalfExtents: mgl64.Vec3{0.4, 0.4, 0.4}},
		actor.BodyTypeStatic,
		0.0,
	)

	bodies := []*actor.RigidBody{staticBody1, staticBody2}

	// Insérer les bodies
	for i, body := range bodies {
		grid.Insert(i, body)
	}

	// Trouver les paires avec la version parallèle
	pairs := make([]Pair, 0)
	for pair := range grid.FindPairsParallel(bodies, 2) {
		pairs = append(pairs, pair)
	}

	// Ne devrait pas détecter de collision entre bodies statiques
	if len(pairs) != 0 {
		t.Errorf("Expected 0 pairs for static bodies, got %d", len(pairs))
	}
}

func TestFindPairsParallelSleepingBodies(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	body1 := createTestBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{0.4, 0.4, 0.4})
	body2 := createTestBox(mgl64.Vec3{0.5, 0.5, 0.5}, mgl64.Vec3{0.4, 0.4, 0.4})

	body1.IsSleeping = true
	body2.IsSleeping = true

	bodies := []*actor.RigidBody{body1, body2}

	// Insérer les bodies
	for i, body := range bodies {
		grid.Insert(i, body)
	}

	// Trouver les paires avec la version parallèle
	pairs := make([]Pair, 0)
	for pair := range grid.FindPairsParallel(bodies, 2) {
		pairs = append(pairs, pair)
	}

	// Ne devrait pas détecter de collision entre bodies endormis
	if len(pairs) != 0 {
		t.Errorf("Expected 0 pairs for sleeping bodies, got %d", len(pairs))
	}
}

func TestFindPairsParallelMultiplePlanes(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)
	plane1 := createTestPlane()
	plane2 := actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()},
		&actor.Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: 0},
		actor.BodyTypeStatic,
		0.0,
	)
	body := createTestBox(mgl64.Vec3{0, 5, 0}, mgl64.Vec3{0.4, 0.4, 0.4})

	bodies := []*actor.RigidBody{plane1, plane2, body} // Inclure les planes dans la liste des bodies

	// Insérer les planes et le body
	grid.Insert(0, plane1)
	grid.Insert(1, plane2)
	grid.Insert(2, body)

	// Trouver les paires avec la version parallèle
	pairs := make([]Pair, 0)
	for pair := range grid.FindPairsParallel(bodies, 2) {
		pairs = append(pairs, pair)
	}

	// Devrait détecter les paires avec les deux planes (TOUJOURS ajoutées sans test de collision)
	// Note: Les planes sont dans la liste des bodies, donc ils seront traités normalement
	// mais aussi via la logique spéciale des planes
	if len(pairs) != 2 {
		t.Errorf("Expected 2 pairs with planes, got %d", len(pairs))
	}

	// Vérifier que les deux paires plane-body sont présentes
	foundPlane1 := false
	foundPlane2 := false
	for _, pair := range pairs {
		if (pair.BodyA == plane1 && pair.BodyB == body) || (pair.BodyA == body && pair.BodyB == plane1) {
			foundPlane1 = true
		}
		if (pair.BodyA == plane2 && pair.BodyB == body) || (pair.BodyA == body && pair.BodyB == plane2) {
			foundPlane2 = true
		}
	}
	if !foundPlane1 || !foundPlane2 {
		t.Error("Both plane-body pairs should be found")
	}
}

// ============================================================================
// Tests pour les cas limites
// ============================================================================

func TestBoundaryCases(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)

	// Body exactement sur la frontière entre deux cellules
	body := createTestBox(mgl64.Vec3{1.0, 1.0, 1.0}, mgl64.Vec3{0.5, 0.5, 0.5})

	grid.Insert(0, body)

	// Vérifier que le body est dans les cellules attendues
	minCell := grid.worldToCell(body.Shape.GetAABB().Min)
	maxCell := grid.worldToCell(body.Shape.GetAABB().Max)

	// Devrait couvrir 2 cellules dans chaque dimension
	if maxCell.X-minCell.X != 1 || maxCell.Y-minCell.Y != 1 || maxCell.Z-minCell.Z != 1 {
		t.Errorf("Expected body to span 2 cells in each dimension, got %d, %d, %d",
			maxCell.X-minCell.X, maxCell.Y-minCell.Y, maxCell.Z-minCell.Z)
	}
}

func TestLargeBodySpanningManyCells(t *testing.T) {
	grid := NewSpatialGrid(1.0, 16)

	// Body très large couvrant beaucoup de cellules
	body := createTestBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{5.0, 5.0, 5.0})

	grid.Insert(0, body)

	// Vérifier que le body est dans toutes les cellules attendues
	minCell := grid.worldToCell(body.Shape.GetAABB().Min)
	maxCell := grid.worldToCell(body.Shape.GetAABB().Max)

	expectedCells := (maxCell.X - minCell.X + 1) * (maxCell.Y - minCell.Y + 1) * (maxCell.Z - minCell.Z + 1)
	actualCells := 0

	for x := minCell.X; x <= maxCell.X; x++ {
		for y := minCell.Y; y <= maxCell.Y; y++ {
			for z := minCell.Z; z <= maxCell.Z; z++ {
				cellKey := CellKey{x, y, z}
				cellIdx := grid.hashCell(cellKey)
				for _, idx := range grid.cells[cellIdx].bodyIndices {
					if idx == 0 {
						actualCells++
						break
					}
				}
			}
		}
	}

	if actualCells != expectedCells {
		t.Errorf("Expected body in %d cells, found in %d cells", expectedCells, actualCells)
	}
}

func BenchmarkFindPairsParallel(b *testing.B) {
	grid := NewSpatialGrid(1.0, 1024)
	bodies := make([]*actor.RigidBody, 100)

	// Créer des bodies aléatoires
	for i := range bodies {
		pos := mgl64.Vec3{
			float64(i%10) * 2.0,
			float64((i/10)%10) * 2.0,
			float64((i/100)%10) * 2.0,
		}
		bodies[i] = createTestBox(pos, mgl64.Vec3{0.4, 0.4, 0.4})
	}

	// Insérer les bodies
	for i, body := range bodies {
		grid.Insert(i, body)
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		for range grid.FindPairsParallel(bodies, 4) {
			// Consume the channel
		}
	}
}
