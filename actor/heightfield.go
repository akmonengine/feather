package actor

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

const (
	// HeightfieldBlockSize: the cells are grouped by blocks of 16x16, with their lowest & highest heights,
	// to skip the blocks far from a body
	HeightfieldBlockSize = 16

	// activeEdgeCos: an edge between 2 triangles bent less than 5°, or bent inwards, is inactive: cos(5°)
	activeEdgeCos = 0.99619469809174553229501040247389
)

// heightfieldNeighbors of the edges of the 2 triangles of the cell (x, z): the cell of the triangle on the other side
// of the edge, and the vertex of this triangle opposite to the edge (offsets from the sample (x, z))
var heightfieldNeighbors = [2][3]struct{ cellX, cellZ, triangle, vertexX, vertexZ int }{
	{{-1, 0, 1, -1, 0}, {0, 1, 1, 1, 2}, {0, 0, 1, 1, 0}},
	{{0, 0, 0, 0, 1}, {1, 0, 0, 2, 1}, {0, -1, 0, 0, -1}},
}

// heightfieldTriangles: the vertices of the 2 triangles of a cell, as offsets from the sample (x, z).
// Both are split along the diagonal from (x, z) to (x+1, z+1), their normals point up (+Y)
var heightfieldTriangles = [2][3][2]int{
	{{0, 0}, {0, 1}, {1, 1}},
	{{0, 0}, {1, 1}, {1, 0}},
}

// Heightfield is a static terrain: a grid of heights, 2 triangles per cell.
// Heights[x*ZSamples+z] is the height of the sample (x, z): the grid of the terrains of AkmonEngine, shared without copy.
// In the local space, the sample (x, z) is at
//
//	((x - (XSamples-1)/2) * Scale.X, height * Scale.Y, (z - (ZSamples-1)/2) * Scale.Z)
//
// the body is at the center of the terrain. The terrain is a surface: the bodies collide with its top side.
// After a change of Heights or Holes, call Update on the changed samples.
type Heightfield struct {
	XSamples int
	ZSamples int
	Heights  []float32
	// Scale: X & Z are the distances between 2 samples (m), Y multiplies the heights
	Scale mgl64.Vec3
	// Holes of the cells (optional), Holes[x*(ZSamples-1)+z]: a hole has no triangle
	Holes []bool

	blocksZ int
	blocks  []heightBlock
	// edges of the cells, 6 bits per triangle: active, then convex
	edges     []uint16
	minHeight float64
	maxHeight float64
}

// heightBlock: lowest & highest heights of the samples of a block (local)
type heightBlock struct {
	min float64
	max float64
}

// NewHeightfield: xSamples*zSamples heights, at least 2x2 samples
func NewHeightfield(xSamples, zSamples int, heights []float32, scale mgl64.Vec3) *Heightfield {
	if xSamples < 2 || zSamples < 2 || len(heights) != xSamples*zSamples {
		panic("feather: a heightfield needs xSamples*zSamples heights, at least 2x2")
	}
	h := &Heightfield{XSamples: xSamples, ZSamples: zSamples, Heights: heights, Scale: scale}
	h.Update(0, 0, xSamples-1, zSamples-1)
	return h
}

// Update the blocks & the active edges around the samples [minX, maxX] x [minZ, maxZ], after a change of the heights or
// of the holes. The bodies resting on the terrain must be woken up (World.UpdateHeightfield does both)
func (h *Heightfield) Update(minX, minZ, maxX, maxZ int) {
	cellsX, cellsZ := h.XSamples-1, h.ZSamples-1
	blocksX := (cellsX + HeightfieldBlockSize - 1) / HeightfieldBlockSize
	h.blocksZ = (cellsZ + HeightfieldBlockSize - 1) / HeightfieldBlockSize
	if len(h.blocks) != blocksX*h.blocksZ {
		h.blocks = make([]heightBlock, blocksX*h.blocksZ)
		h.edges = make([]uint16, cellsX*cellsZ)
		minX, minZ, maxX, maxZ = 0, 0, h.XSamples-1, h.ZSamples-1
	}
	minX, minZ = max(0, minX), max(0, minZ)
	maxX, maxZ = min(h.XSamples-1, maxX), min(h.ZSamples-1, maxZ)

	// ========== BLOCKS ==========
	// a sample is shared by the blocks around it
	for bx := max(0, (minX-1)/HeightfieldBlockSize); bx <= min(blocksX-1, maxX/HeightfieldBlockSize); bx++ {
		for bz := max(0, (minZ-1)/HeightfieldBlockSize); bz <= min(h.blocksZ-1, maxZ/HeightfieldBlockSize); bz++ {
			block := heightBlock{min: math.Inf(1), max: math.Inf(-1)}
			for x := bx * HeightfieldBlockSize; x <= min(h.XSamples-1, (bx+1)*HeightfieldBlockSize); x++ {
				for z := bz * HeightfieldBlockSize; z <= min(h.ZSamples-1, (bz+1)*HeightfieldBlockSize); z++ {
					height := h.height(x, z)
					block.min = math.Min(block.min, height)
					block.max = math.Max(block.max, height)
				}
			}
			h.blocks[bx*h.blocksZ+bz] = block
		}
	}
	h.minHeight, h.maxHeight = math.Inf(1), math.Inf(-1)
	for _, block := range h.blocks {
		h.minHeight = math.Min(h.minHeight, block.min)
		h.maxHeight = math.Max(h.maxHeight, block.max)
	}

	// ========== ACTIVE EDGES ==========
	// the edges of the cells around the changed samples, and of their neighbors
	for x := max(0, minX-2); x <= min(cellsX-1, maxX+1); x++ {
		for z := max(0, minZ-2); z <= min(cellsZ-1, maxZ+1); z++ {
			h.edges[x*cellsZ+z] = h.cellEdges(x, z)
		}
	}
}

// cellEdges: an edge is convex if the neighbor triangle bends down, or if there is no neighbor on this side
// (border, hole). A convex edge is active if it bends by more than 5°: only the active edges can push a body sideways
func (h *Heightfield) cellEdges(x, z int) uint16 {
	var edges uint16
	for t := 0; t < 2; t++ {
		triangle := h.localTriangle(x, z, t)
		normal := triangleNormal(triangle)
		for e, neighbor := range heightfieldNeighbors[t] {
			active, convex := uint16(1)<<(t*6+e), uint16(1)<<(t*6+3+e)
			cellX, cellZ := x+neighbor.cellX, z+neighbor.cellZ
			if !h.hasCell(cellX, cellZ) {
				edges |= active | convex
				continue
			}
			opposite := h.localVertex(x+neighbor.vertexX, z+neighbor.vertexZ)
			if opposite.Sub(triangle[e]).Dot(normal) >= 0 {
				continue
			}
			edges |= convex
			if normal.Dot(triangleNormal(h.localTriangle(cellX, cellZ, neighbor.triangle))) < activeEdgeCos {
				edges |= active
			}
		}
	}
	return edges
}

func (h *Heightfield) height(x, z int) float64 {
	return float64(h.Heights[x*h.ZSamples+z]) * h.Scale.Y()
}

// hasCell: the cell exists and is not a hole
func (h *Heightfield) hasCell(x, z int) bool {
	if x < 0 || z < 0 || x >= h.XSamples-1 || z >= h.ZSamples-1 {
		return false
	}
	return h.Holes == nil || !h.Holes[x*(h.ZSamples-1)+z]
}

func (h *Heightfield) localVertex(x, z int) mgl64.Vec3 {
	return mgl64.Vec3{
		(float64(x) - float64(h.XSamples-1)/2) * h.Scale.X(),
		h.height(x, z),
		(float64(z) - float64(h.ZSamples-1)/2) * h.Scale.Z(),
	}
}

func (h *Heightfield) localTriangle(x, z, t int) [3]mgl64.Vec3 {
	var triangle [3]mgl64.Vec3
	for i, offset := range heightfieldTriangles[t] {
		triangle[i] = h.localVertex(x+offset[0], z+offset[1])
	}
	return triangle
}

func triangleNormal(triangle [3]mgl64.Vec3) mgl64.Vec3 {
	return triangle[1].Sub(triangle[0]).Cross(triangle[2].Sub(triangle[0])).Normalize()
}

// Triangle t (0 or 1) of the cell (x, z), in the local space, with its edges: bit e if the edge from the vertex e
// to the vertex e+1 is active, bit 3+e if it is convex
func (h *Heightfield) Triangle(x, z, t int) ([3]mgl64.Vec3, uint8) {
	return h.localTriangle(x, z, t), uint8(h.edges[x*(h.ZSamples-1)+z]>>(t*6)) & 0b111111
}

// OverlapCells appends to cells the index x*(ZSamples-1)+z of the cells which may touch the local bounds:
// the cells under the bounds, without the holes, whose block and heights overlap the bounds
func (h *Heightfield) OverlapCells(bounds AABB, cells []int32) []int32 {
	halfX, halfZ := float64(h.XSamples-1)/2, float64(h.ZSamples-1)/2
	minX := max(0, int(math.Floor(bounds.Min.X()/h.Scale.X()+halfX)))
	maxX := min(h.XSamples-2, int(math.Floor(bounds.Max.X()/h.Scale.X()+halfX)))
	minZ := max(0, int(math.Floor(bounds.Min.Z()/h.Scale.Z()+halfZ)))
	maxZ := min(h.ZSamples-2, int(math.Floor(bounds.Max.Z()/h.Scale.Z()+halfZ)))
	if minX > maxX || minZ > maxZ || bounds.Min.Y() > h.maxHeight || bounds.Max.Y() < h.minHeight {
		return cells
	}

	cellsZ := h.ZSamples - 1
	for bx := minX / HeightfieldBlockSize; bx <= maxX/HeightfieldBlockSize; bx++ {
		for bz := minZ / HeightfieldBlockSize; bz <= maxZ/HeightfieldBlockSize; bz++ {
			block := h.blocks[bx*h.blocksZ+bz]
			if bounds.Min.Y() > block.max || bounds.Max.Y() < block.min {
				continue
			}
			for x := max(minX, bx*HeightfieldBlockSize); x <= min(maxX, (bx+1)*HeightfieldBlockSize-1); x++ {
				for z := max(minZ, bz*HeightfieldBlockSize); z <= min(maxZ, (bz+1)*HeightfieldBlockSize-1); z++ {
					if !h.hasCell(x, z) {
						continue
					}
					h00, h01, h10, h11 := h.height(x, z), h.height(x, z+1), h.height(x+1, z), h.height(x+1, z+1)
					if bounds.Min.Y() > max(h00, h01, h10, h11) || bounds.Max.Y() < min(h00, h01, h10, h11) {
						continue
					}
					cells = append(cells, int32(x*cellsZ+z))
				}
			}
		}
	}
	return cells
}

// HeightAt returns the height of the triangles at the local position (x, z), false outside the terrain or in a hole
func (h *Heightfield) HeightAt(x, z float64) (float64, bool) {
	fx := x/h.Scale.X() + float64(h.XSamples-1)/2
	fz := z/h.Scale.Z() + float64(h.ZSamples-1)/2
	cellX, cellZ := int(math.Floor(fx)), int(math.Floor(fz))
	// the last samples belong to the last cells
	if fx == float64(h.XSamples-1) {
		cellX--
	}
	if fz == float64(h.ZSamples-1) {
		cellZ--
	}
	if !h.hasCell(cellX, cellZ) {
		return 0, false
	}
	u, v := fx-float64(cellX), fz-float64(cellZ)
	h00, h01, h10, h11 := h.height(cellX, cellZ), h.height(cellX, cellZ+1), h.height(cellX+1, cellZ), h.height(cellX+1, cellZ+1)
	if v >= u {
		// triangle 0: (0,0), (0,1), (1,1)
		return h00 + (h01-h00)*(v-u) + (h11-h00)*u, true
	}
	// triangle 1: (0,0), (1,1), (1,0)
	return h00 + (h10-h00)*(u-v) + (h11-h00)*v, true
}

func (h *Heightfield) ComputeAABB(transform Transform) AABB {
	halfX := float64(h.XSamples-1) / 2 * h.Scale.X()
	halfZ := float64(h.ZSamples-1) / 2 * h.Scale.Z()
	min := mgl64.Vec3{math.Inf(1), math.Inf(1), math.Inf(1)}
	max := mgl64.Vec3{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for i := 0; i < 8; i++ {
		corner := mgl64.Vec3{-halfX, h.minHeight, -halfZ}
		if i&1 != 0 {
			corner[0] = halfX
		}
		if i&2 != 0 {
			corner[1] = h.maxHeight
		}
		if i&4 != 0 {
			corner[2] = halfZ
		}
		world := transform.ToWorld(corner)
		for k := 0; k < 3; k++ {
			min[k] = math.Min(min[k], world[k])
			max[k] = math.Max(max[k], world[k])
		}
	}
	return AABB{Min: min, Max: max}
}

// ComputeMass: a heightfield is static
func (h *Heightfield) ComputeMass(density float64) float64 {
	return math.Inf(1)
}

func (h *Heightfield) ComputeInertia(mass float64) mgl64.Mat3 {
	return mgl64.Mat3{}
}

// Support: a heightfield is not convex, its triangles are tested one by one
func (h *Heightfield) Support(direction mgl64.Vec3) mgl64.Vec3 {
	return mgl64.Vec3{}
}

func (h *Heightfield) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	output[0] = mgl64.Vec3{}
	*count = 1
}

// CollideWithPlane - Heightfield/Plane collision (not supported)
func (h *Heightfield) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact {
	return contacts
}

// CastRay: the ray walks the cells under it (WalkCells: the blocks it flies over are skipped), and in each cell meets
// the planes of both triangles. A triangle is a plane over half a cell: the height of the ray above it is linear along
// the ray, and the ray hits where it is 0, if this point is over the triangle. Only the top side is hit: a ray coming
// from below goes through, as through a hole. The point may be footprintSlack out of the triangle: a ray on an edge or
// on a sample hits the triangles around it, and never leaks between them. At the very same fraction, the triangle of
// the lowest index is named
func (h *Heightfield) CastRay(origin, translation mgl64.Vec3, maxFraction float64) (RayHit, bool) {
	if !finiteRay(origin, translation) {
		return RayHit{}, false
	}
	// the ray in the grid: a cell is a unit square, the sample (x, z) at (x, z)
	startX, startZ := origin.X()/h.Scale.X()+float64(h.XSamples-1)/2, origin.Z()/h.Scale.Z()+float64(h.ZSamples-1)/2
	alongX, alongZ := translation.X()/h.Scale.X(), translation.Z()/h.Scale.Z()

	hit, found := RayHit{Fraction: maxFraction, Triangle: NoTriangle}, false
	slopeX, slopeZ := 0.0, 0.0
	var walk CellWalk
	walk.start(h, origin, translation, mgl64.Vec3{}, maxFraction)
	for {
		x, z, ok := walk.Next(hit.Fraction)
		if !ok {
			break
		}
		u, v := startX-float64(x), startZ-float64(z)
		h00, h01, h10, h11 := h.height(x, z), h.height(x, z+1), h.height(x+1, z), h.height(x+1, z+1)
		// the slopes of both triangles along x & z (in the cell: per cell, not per meter)
		slopes := [2][2]float64{{h11 - h01, h01 - h00}, {h10 - h00, h11 - h10}}
		for t, slope := range slopes {
			above := origin.Y() - (h00 + slope[0]*u + slope[1]*v)
			descent := slope[0]*alongX + slope[1]*alongZ - translation.Y()
			// from below, along the plane, or further than the best hit
			if above < 0 || descent <= 0 || above > hit.Fraction*descent {
				continue
			}
			fraction := above / descent
			hitU, hitV := u+fraction*alongX, v+fraction*alongZ
			// triangle 0 is the half of the cell where v >= u, triangle 1 the other half
			across := hitV - hitU
			if t == 1 {
				hitU, hitV, across = hitV, hitU, -across
			}
			if hitU < -footprintSlack || hitV > 1+footprintSlack || across < -footprintSlack {
				continue
			}
			// at the same fraction (a ray on an edge or on a sample), the triangle of the lowest index is kept,
			// wherever the ray comes from
			triangle := int32(2*(x*(h.ZSamples-1)+z) + t)
			if found && fraction == hit.Fraction && triangle > hit.Triangle {
				continue
			}
			hit.Fraction, hit.Triangle, found = fraction, triangle, true
			slopeX, slopeZ = slope[0], slope[1]
		}
	}
	if found {
		hit.Normal = mgl64.Vec3{-slopeX / h.Scale.X(), 1, -slopeZ / h.Scale.Z()}.Normalize()
	}
	return hit, found
}
