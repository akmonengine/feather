package feather

import (
	"math"
	"sort"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ============================================================================
// Types
// ============================================================================

// CellKey - Coordonnées d'une cellule dans l'espace 3D
type CellKey struct {
	X, Y, Z int
}

// Cell - Conteneur d'indices de bodies dans une cellule
type Cell struct {
	bodyIndices []int
}

// Pair - Paire de bodies potentiellement en collision
type Pair struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

// SpatialGrid - Grille spatiale uniforme avec hashing pour broad phase
type SpatialGrid struct {
	cellSize float64
	cells    []Cell
	cellMask int
}

// ============================================================================
// Constructeur
// ============================================================================

// NewSpatialGrid - Crée une nouvelle grille spatiale
func NewSpatialGrid(cellSize float64, numCells int) *SpatialGrid {
	numCells = nextPowerOfTwo(numCells)

	cells := make([]Cell, numCells)
	for i := range cells {
		cells[i].bodyIndices = make([]int, 0, 8)
	}

	return &SpatialGrid{
		cellSize: cellSize,
		cells:    cells,
		cellMask: numCells - 1,
	}
}

// nextPowerOfTwo - Arrondit à la puissance de 2 supérieure
func nextPowerOfTwo(n int) int {
	if n <= 0 {
		return 1
	}
	n--
	n |= n >> 1
	n |= n >> 2
	n |= n >> 4
	n |= n >> 8
	n |= n >> 16
	n++
	return n
}

// Insert - Insère un body dans toutes les cellules qu'il occupe
func (sg *SpatialGrid) Insert(bodyIndex int, body *actor.RigidBody) {
	aabb := body.Shape.GetAABB()
	minCell := sg.worldToCell(aabb.Min)
	maxCell := sg.worldToCell(aabb.Max)

	for x := minCell.X; x <= maxCell.X; x++ {
		for y := minCell.Y; y <= maxCell.Y; y++ {
			for z := minCell.Z; z <= maxCell.Z; z++ {
				cellKey := CellKey{x, y, z}
				cellIdx := sg.hashCell(cellKey)

				sg.cells[cellIdx].bodyIndices = append(
					sg.cells[cellIdx].bodyIndices,
					bodyIndex,
				)
			}
		}
	}
}

func (sg *SpatialGrid) Clear() {
	for i := range sg.cells {
		sg.cells[i].bodyIndices = sg.cells[i].bodyIndices[:0]
	}
}

func (sg *SpatialGrid) SortCells() {
	for i := range sg.cells {
		if len(sg.cells[i].bodyIndices) > 1 {
			sort.Ints(sg.cells[i].bodyIndices)
		}
	}
}

// FindPairs - Version séquentielle
func (sg *SpatialGrid) FindPairs(bodies []*actor.RigidBody) []Pair {
	pairs := make([]Pair, 0, len(bodies)/2)

	// ========== BOUCLE SUR BODIES ==========
	for bodyIdx := 0; bodyIdx < len(bodies); bodyIdx++ {
		bodyA := bodies[bodyIdx]

		// Trouver cellules occupées par bodyA
		minCell := sg.worldToCell(bodyA.Shape.GetAABB().Min)
		maxCell := sg.worldToCell(bodyA.Shape.GetAABB().Max)

		// Parcourir ces cellules
		for x := minCell.X; x <= maxCell.X; x++ {
			for y := minCell.Y; y <= maxCell.Y; y++ {
				for z := minCell.Z; z <= maxCell.Z; z++ {
					cellKey := CellKey{x, y, z}
					cellIdx := sg.hashCell(cellKey)

					// Tester contre tous les bodies dans cette cellule
					for _, otherIdx := range sg.cells[cellIdx].bodyIndices {
						// ========== ORDRE DÉTERMINISTE ==========
						if otherIdx <= bodyIdx {
							continue // Évite doublons (A,B) et (B,A)
						}

						bodyB := bodies[otherIdx]

						// Checks
						if bodyA.BodyType == actor.BodyTypeStatic && bodyB.BodyType == actor.BodyTypeStatic {
							continue
						}
						if bodyA.IsSleeping && bodyB.IsSleeping {
							continue
						}

						_, aIsPlane := bodyA.Shape.(*actor.Plane)
						_, bIsPlane := bodyB.Shape.(*actor.Plane)
						if aIsPlane || bIsPlane {
							pairs = append(pairs, Pair{BodyA: bodyA, BodyB: bodyB})
							continue
						}
						if bodyA.Shape.GetAABB().Overlaps(bodyB.Shape.GetAABB()) {
							pairs = append(pairs, Pair{BodyA: bodyA, BodyB: bodyB})
						}
					}
				}
			}
		}
	}

	return pairs
}

// FindPairsParallel - Version parallèle retournant un channel
func (sg *SpatialGrid) FindPairsParallel(bodies []*actor.RigidBody, numWorkers int) <-chan Pair {
	var wg sync.WaitGroup
	pairsChan := make(chan Pair, numWorkers*10)

	bodiesPerWorker := len(bodies) / numWorkers
	if bodiesPerWorker == 0 {
		bodiesPerWorker = 1
	}

	clearSeen := make([]bool, len(bodies))
	for w := 0; w < numWorkers; w++ {
		wg.Add(1)

		startIdx := w * bodiesPerWorker
		endIdx := startIdx + bodiesPerWorker
		if w == numWorkers-1 {
			endIdx = len(bodies)
		}

		go func(start, end int) {
			defer wg.Done()

			seen := make([]bool, len(bodies))
			for bodyIdx := start; bodyIdx < end; bodyIdx++ {
				copy(seen, clearSeen)

				bodyA := bodies[bodyIdx]

				// Trouver cellules occupées par bodyA
				minCell := sg.worldToCell(bodyA.Shape.GetAABB().Min)
				maxCell := sg.worldToCell(bodyA.Shape.GetAABB().Max)

				// Parcourir ces cellules
				for x := minCell.X; x <= maxCell.X; x++ {
					for y := minCell.Y; y <= maxCell.Y; y++ {
						for z := minCell.Z; z <= maxCell.Z; z++ {
							cellKey := CellKey{x, y, z}
							cellIdx := sg.hashCell(cellKey)

							// Tester contre tous les bodies dans cette cellule
							for _, otherIdx := range sg.cells[cellIdx].bodyIndices {
								// Avoid duplicates
								if otherIdx <= bodyIdx || seen[otherIdx] {
									continue
								}
								seen[otherIdx] = true

								bodyB := bodies[otherIdx]
								if bodyA.BodyType == actor.BodyTypeStatic && bodyB.BodyType == actor.BodyTypeStatic {
									continue
								}
								if bodyA.IsSleeping && bodyB.IsSleeping {
									continue
								}

								_, aIsPlane := bodyA.Shape.(*actor.Plane)
								_, bIsPlane := bodyB.Shape.(*actor.Plane)
								if aIsPlane || bIsPlane {
									pairsChan <- Pair{BodyA: bodyA, BodyB: bodyB}
									continue
								}

								if bodyA.Shape.GetAABB().Overlaps(bodyB.Shape.GetAABB()) {
									pairsChan <- Pair{BodyA: bodyA, BodyB: bodyB}
								}
							}
						}
					}
				}
			}
		}(startIdx, endIdx)
	}

	go func() {
		wg.Wait()
		close(pairsChan)
	}()

	return pairsChan
}

// worldToCell - Convertit une position monde en coordonnées de cellule
func (sg *SpatialGrid) worldToCell(pos mgl64.Vec3) CellKey {
	return CellKey{
		X: int(math.Floor(pos.X() / sg.cellSize)),
		Y: int(math.Floor(pos.Y() / sg.cellSize)),
		Z: int(math.Floor(pos.Z() / sg.cellSize)),
	}
}

// hashCell - Hash une cellule vers un index dans l'array
func (sg *SpatialGrid) hashCell(key CellKey) int {
	h := (key.X * 73856093) ^ (key.Y * 19349663) ^ (key.Z * 83492791)
	return h & sg.cellMask
}
