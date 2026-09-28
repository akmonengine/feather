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
	sg.InsertAABB(bodyIndex, body, body.Shape.GetAABB())
}

// InsertAABB - Inserts a body into all cells of the given AABB (e.g. an enlarged AABB)
func (sg *SpatialGrid) InsertAABB(bodyIndex int, body *actor.RigidBody, aabb actor.AABB) {
	if _, ok := body.Shape.(*actor.Plane); ok {
		sg.planes.bodyIndices = append(sg.planes.bodyIndices, bodyIndex)
		return
	}

	minCell := sg.worldToCell(aabb.Min)
	maxCell := sg.worldToCell(aabb.Max)

	for x := minCell.X; x <= maxCell.X; x++ {
		for y := minCell.Y; y <= maxCell.Y; y++ {
			for z := minCell.Z; z <= maxCell.Z; z++ {
				cellIdx := sg.hashCell(CellKey{x, y, z})
				sg.cells[cellIdx].bodyIndices = append(sg.cells[cellIdx].bodyIndices, bodyIndex)
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

// FindPairs - Finds the pairs of bodies with overlapping AABBs, always in the same order:
// sorted by index of the first body, then of the second body, planes first.
// Pairs without any awake dynamic body are ignored.
func (sg *SpatialGrid) FindPairs(bodies []*actor.RigidBody, boxes []actor.AABB, workersCount int) []Pair {
	workersCount = max(1, min(workersCount, len(bodies)))
	chunks := make([][]Pair, workersCount)
	chunkSize := (len(bodies) + workersCount - 1) / workersCount

	var wg sync.WaitGroup
	for workerID := 0; workerID < workersCount; workerID++ {
		start, end := workerID*chunkSize, min((workerID+1)*chunkSize, len(bodies))
		work := func() {
			chunks[workerID] = sg.findPairsRange(bodies, boxes, start, end)
		}
		if workersCount == 1 {
			work()
			continue
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			work()
		}()
	}
	wg.Wait()

	count := 0
	for _, c := range chunks {
		count += len(c)
	}
	pairs := make([]Pair, 0, count)
	for _, c := range chunks {
		pairs = append(pairs, c...)
	}
	return pairs
}

func (sg *SpatialGrid) findPairsRange(bodies []*actor.RigidBody, boxes []actor.AABB, start, end int) []Pair {
	var pairs []Pair
	seen := make([]bool, len(bodies))
	var found []int

	for bodyIdx := start; bodyIdx < end; bodyIdx++ {
		bodyA := bodies[bodyIdx]
		if _, isPlane := bodyA.Shape.(*actor.Plane); isPlane {
			continue
		}

		for _, planeIdx := range sg.planes.bodyIndices {
			if needsSolving(bodies[planeIdx], bodyA) {
				pairs = append(pairs, Pair{BodyA: bodies[planeIdx], BodyB: bodyA})
			}
		}

		found = found[:0]
		minCell := sg.worldToCell(boxes[bodyIdx].Min)
		maxCell := sg.worldToCell(boxes[bodyIdx].Max)
		for x := minCell.X; x <= maxCell.X; x++ {
			for y := minCell.Y; y <= maxCell.Y; y++ {
				for z := minCell.Z; z <= maxCell.Z; z++ {
					for _, otherIdx := range sg.cells[sg.hashCell(CellKey{x, y, z})].bodyIndices {
						if otherIdx <= bodyIdx || seen[otherIdx] {
							continue
						}
						seen[otherIdx] = true
						found = append(found, otherIdx)
					}
				}
			}
		}

		sort.Ints(found)
		for _, otherIdx := range found {
			seen[otherIdx] = false
			bodyB := bodies[otherIdx]
			if needsSolving(bodyA, bodyB) && boxes[bodyIdx].Overlaps(boxes[otherIdx]) {
				pairs = append(pairs, Pair{BodyA: bodyA, BodyB: bodyB})
			}
		}
	}
	return pairs
}

// needsSolving - At least one body must be dynamic and awake
func needsSolving(a, b *actor.RigidBody) bool {
	return isAwakeDynamic(a) || isAwakeDynamic(b)
}

func isAwakeDynamic(body *actor.RigidBody) bool {
	return body.BodyType == actor.BodyTypeDynamic && !body.IsSleeping
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
