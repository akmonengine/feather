package feather

import (
	"math"
	"sort"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// CellKey - Coordinates of a cell in 3D space
type CellKey struct {
	X, Y, Z int
}

// Cell - Container of body indices in a cell
type Cell struct {
	bodyIndices []int
}

// Pair - Pair of bodies potentially in collision
type Pair struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

// SpatialGrid - Uniform spatial grid with hashing for broad phase
type SpatialGrid struct {
	cellSize float64
	cells    []Cell
	planes   Cell
}

// NewSpatialGrid - Creates a new spatial grid
func NewSpatialGrid(cellSize float64, numCells int) *SpatialGrid {
	cells := make([]Cell, numCells)
	for i := range cells {
		cells[i].bodyIndices = make([]int, 0, 8)
	}

	return &SpatialGrid{
		cellSize: cellSize,
		cells:    cells,
	}
}

// Insert - Inserts a body into all cells it occupies
func (sg *SpatialGrid) Insert(bodyIndex int, body *actor.RigidBody) {
	if _, ok := body.Shape.(*actor.Plane); ok {
		sg.planes.bodyIndices = append(sg.planes.bodyIndices, bodyIndex)
		return
	}

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

// Clear - Resets the spatial grid by clearing all body indices from cells and planes
func (sg *SpatialGrid) Clear() {
	sg.planes.bodyIndices = sg.planes.bodyIndices[:0]

	for i := range sg.cells {
		sg.cells[i].bodyIndices = sg.cells[i].bodyIndices[:0]
	}
}

// SortCells - Sorts body indices within each cell for optimized collision detection
func (sg *SpatialGrid) SortCells() {
	for i := range sg.cells {
		if len(sg.cells[i].bodyIndices) > 1 {
			sort.Ints(sg.cells[i].bodyIndices)
		}
	}
}

// FindPairsParallel - Parallel version returning a channel
func (sg *SpatialGrid) FindPairsParallel(bodies []*actor.RigidBody, workersCount int) <-chan Pair {
	var wg sync.WaitGroup
	pairsChan := make(chan Pair, workersCount*10)
	clearSeen := make([]bool, len(bodies))

	dataSize := len(bodies)
	chunkSize := (dataSize + workersCount - 1) / workersCount
	for workerID := 0; workerID < workersCount; workerID++ {
		wg.Add(1)

		go func(start, end int) {
			defer wg.Done()

			seen := make([]bool, len(bodies))
			for bodyIdx := start; bodyIdx < end; bodyIdx++ {
				if _, isPlane := bodies[bodyIdx].Shape.(*actor.Plane); isPlane {
					continue
				}
				bodyA := bodies[bodyIdx]

				// write all planes/body collisions
				for _, planeId := range sg.planes.bodyIndices {
					pairsChan <- Pair{BodyA: bodies[planeId], BodyB: bodyA}
				}

				copy(seen, clearSeen)

				// Find cells occupied by bodyA
				minCell := sg.worldToCell(bodyA.Shape.GetAABB().Min)
				maxCell := sg.worldToCell(bodyA.Shape.GetAABB().Max)

				// Iterate through these cells
				for x := minCell.X; x <= maxCell.X; x++ {
					for y := minCell.Y; y <= maxCell.Y; y++ {
						for z := minCell.Z; z <= maxCell.Z; z++ {
							cellKey := CellKey{x, y, z}
							cellIdx := sg.hashCell(cellKey)

							// Test against all bodies in this cell
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

								if bodyA.Shape.GetAABB().Overlaps(bodyB.Shape.GetAABB()) {
									pairsChan <- Pair{BodyA: bodyA, BodyB: bodyB}
								}
							}
						}
					}
				}
			}
		}(workerID*chunkSize, min((workerID+1)*chunkSize, dataSize))
	}

	go func() {
		wg.Wait()
		close(pairsChan)
	}()

	return pairsChan
}

// worldToCell - Converts a world position to cell coordinates
func (sg *SpatialGrid) worldToCell(pos mgl64.Vec3) CellKey {
	return CellKey{
		X: int(math.Floor(pos.X() / sg.cellSize)),
		Y: int(math.Floor(pos.Y() / sg.cellSize)),
		Z: int(math.Floor(pos.Z() / sg.cellSize)),
	}
}

// hashCell - Hashes a cell to an index in the array
// Uses a hash function inspired by MurmurHash3 for better distribution
// and to reduce collisions. The constants used are known prime numbers
// for their good bit mixing properties.
func (sg *SpatialGrid) hashCell(key CellKey) int {
	// Mixing constants inspired by MurmurHash3
	// These values were chosen empirically for their diffusion properties
	const (
		prime1 = uint32(16777619)   // First prime number for initial mixing
		prime2 = uint32(2166136261) // Second prime number for mixing
		prime3 = uint32(1681692777) // Third prime number for mixing

		// Constants for final mixing (avalanche effect)
		mix1 = uint32(0x85ebca6b) // Mixing constant for bit diffusion
		mix2 = uint32(0xc2b2ae35) // Second mixing constant
	)

	// Conversion to uint32 to avoid unexpected overflows
	h := uint32(key.X) * prime1
	h = (h ^ uint32(key.Y)) * prime2
	h = (h ^ uint32(key.Z)) * prime3

	// Final mixing to improve distribution (avalanche effect)
	// This sequence creates complete bit diffusion to reduce collisions
	h ^= h >> 16
	h *= mix1
	h ^= h >> 13
	h *= mix2
	h ^= h >> 16

	return int(h) % len(sg.cells)
}
