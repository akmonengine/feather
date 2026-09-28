package actor

import (
	"math"
	"math/rand"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

func randomHeightfield(seed int64, xSamples, zSamples int) *Heightfield {
	r := rand.New(rand.NewSource(seed))
	heights := make([]float32, xSamples*zSamples)
	for i := range heights {
		heights[i] = float32(r.Float64())
	}
	return NewHeightfield(xSamples, zSamples, heights, mgl64.Vec3{0.5, 2, 0.25})
}

// The height of the triangles: the samples at the samples, a plane inside each triangle
func TestHeightfieldHeightAt(t *testing.T) {
	h := randomHeightfield(1, 7, 5)
	for x := 0; x < h.XSamples; x++ {
		for z := 0; z < h.ZSamples; z++ {
			vertex := h.localVertex(x, z)
			height, ok := h.HeightAt(vertex.X(), vertex.Z())
			if !ok || math.Abs(height-vertex.Y()) > 1e-12 {
				t.Fatalf("sample (%d, %d): %v %v, want %v", x, z, height, ok, vertex.Y())
			}
		}
	}

	r := rand.New(rand.NewSource(2))
	for i := 0; i < 1000; i++ {
		x, z := r.Intn(h.XSamples-1), r.Intn(h.ZSamples-1)
		triangle := h.localTriangle(x, z, r.Intn(2))
		// a random point of the triangle
		a, b := r.Float64(), r.Float64()
		if a+b > 1 {
			a, b = 1-a, 1-b
		}
		p := triangle[0].Add(triangle[1].Sub(triangle[0]).Mul(a)).Add(triangle[2].Sub(triangle[0]).Mul(b))
		if height, ok := h.HeightAt(p.X(), p.Z()); !ok || math.Abs(height-p.Y()) > 1e-9 {
			t.Fatalf("point %v of the triangle: height %v", p, height)
		}
	}

	if _, ok := h.HeightAt(10, 0); ok {
		t.Error("a height outside the terrain")
	}
	h.Holes = make([]bool, (h.XSamples-1)*(h.ZSamples-1))
	h.Holes[2*(h.ZSamples-1)+1] = true
	center := h.localVertex(2, 1).Add(h.localVertex(3, 2)).Mul(0.5)
	if _, ok := h.HeightAt(center.X(), center.Z()); ok {
		t.Error("a height in a hole")
	}
}

// edgesOf the triangle t of the cell (x, z): active, convex
func edgesOf(h *Heightfield, x, z, t, e int) (bool, bool) {
	_, edges := h.Triangle(x, z, t)
	return edges&(1<<e) != 0, edges&(1<<(3+e)) != 0
}

func TestHeightfieldEdges(t *testing.T) {
	heights := make([]float32, 5*5)
	flat := NewHeightfield(5, 5, heights, mgl64.Vec3{1, 1, 1})
	for x := 0; x < 4; x++ {
		for z := 0; z < 4; z++ {
			// the diagonal: never active on a flat terrain
			if active, _ := edgesOf(flat, x, z, 0, 2); active {
				t.Fatal("an inner edge of a flat terrain is active")
			}
		}
	}
	// the borders are active
	if active, _ := edgesOf(flat, 0, 1, 0, 0); !active {
		t.Error("the border x = 0 is not active")
	}
	if active, _ := edgesOf(flat, 1, 1, 0, 0); active {
		t.Error("the inner edge x = 1 is active")
	}

	// a ridge along z at x = 2: convex and active, a valley: concave and inactive
	for _, ridge := range []bool{true, false} {
		for x := 0; x < 5; x++ {
			for z := 0; z < 5; z++ {
				height := -math.Abs(float64(x)-2) * 0.5
				if !ridge {
					height = -height
				}
				heights[x*5+z] = float32(height)
			}
		}
		field := NewHeightfield(5, 5, heights, mgl64.Vec3{1, 1, 1})
		// the edge x = 2 is the edge 0 of the triangle 0 of the cell (2, z)
		active, convex := edgesOf(field, 2, 1, 0, 0)
		if active != ridge || convex != ridge {
			t.Errorf("ridge %v: active %v, convex %v", ridge, active, convex)
		}
	}

	// bent by 3° only (1.5° on each side): convex but inactive
	for x := 0; x < 5; x++ {
		for z := 0; z < 5; z++ {
			heights[x*5+z] = float32(-math.Abs(float64(x)-2) * math.Tan(1.5*math.Pi/180))
		}
	}
	gentle := NewHeightfield(5, 5, heights, mgl64.Vec3{1, 1, 1})
	if active, convex := edgesOf(gentle, 2, 1, 0, 0); active || !convex {
		t.Errorf("gentle ridge: active %v, convex %v", active, convex)
	}
}

// OverlapCells returns exactly the cells under the bounds whose heights overlap the bounds
func TestHeightfieldOverlapCells(t *testing.T) {
	h := randomHeightfield(3, 40, 37)
	h.Holes = make([]bool, 39*36)
	h.Holes[5*36+7] = true
	h.Update(0, 0, 39, 36)
	r := rand.New(rand.NewSource(4))
	for i := 0; i < 300; i++ {
		center := mgl64.Vec3{r.Float64()*24 - 12, r.Float64()*3 - 0.5, r.Float64()*12 - 6}
		size := mgl64.Vec3{r.Float64() * 3, r.Float64(), r.Float64() * 3}
		bounds := AABB{Min: center.Sub(size), Max: center.Add(size)}
		cells := h.OverlapCells(bounds, nil)

		want := map[int32]bool{}
		for x := 0; x < h.XSamples-1; x++ {
			for z := 0; z < h.ZSamples-1; z++ {
				if !h.hasCell(x, z) {
					continue
				}
				low, high := h.localVertex(x, z), h.localVertex(x+1, z+1)
				minHeight := math.Min(math.Min(h.height(x, z), h.height(x+1, z)), math.Min(h.height(x, z+1), h.height(x+1, z+1)))
				maxHeight := math.Max(math.Max(h.height(x, z), h.height(x+1, z)), math.Max(h.height(x, z+1), h.height(x+1, z+1)))
				cell := AABB{Min: mgl64.Vec3{low.X(), minHeight, low.Z()}, Max: mgl64.Vec3{high.X(), maxHeight, high.Z()}}
				// the cells touching the bounds only on their far side belong to the next cell
				if cell.Overlaps(bounds) && bounds.Max.X() >= low.X() && bounds.Min.X() < high.X() && bounds.Min.Z() < high.Z() {
					want[int32(x*(h.ZSamples-1)+z)] = true
				}
			}
		}
		if len(cells) != len(want) {
			t.Fatalf("bounds %v: %d cells, want %d", bounds, len(cells), len(want))
		}
		for _, cell := range cells {
			if !want[cell] {
				t.Fatalf("bounds %v: cell %d is not under the bounds", bounds, cell)
			}
		}
	}
}

// Update of a region gives the same terrain as a new terrain
func TestHeightfieldUpdate(t *testing.T) {
	h := randomHeightfield(5, 50, 45)
	r := rand.New(rand.NewSource(6))
	for i := 0; i < 20; i++ {
		minX, minZ := r.Intn(50), r.Intn(45)
		maxX, maxZ := min(49, minX+r.Intn(6)), min(44, minZ+r.Intn(6))
		for x := minX; x <= maxX; x++ {
			for z := minZ; z <= maxZ; z++ {
				h.Heights[x*45+z] = float32(r.Float64()*3 - 1)
			}
		}
		h.Update(minX, minZ, maxX, maxZ)

		fresh := NewHeightfield(50, 45, h.Heights, h.Scale)
		for k := range fresh.blocks {
			if fresh.blocks[k] != h.blocks[k] {
				t.Fatalf("update %d: block %d is %v, want %v", i, k, h.blocks[k], fresh.blocks[k])
			}
		}
		for k := range fresh.edges {
			if fresh.edges[k] != h.edges[k] {
				t.Fatalf("update %d: edges of the cell %d are %b, want %b", i, k, h.edges[k], fresh.edges[k])
			}
		}
		if fresh.minHeight != h.minHeight || fresh.maxHeight != h.maxHeight {
			t.Fatalf("update %d: heights [%v, %v], want [%v, %v]", i, h.minHeight, h.maxHeight, fresh.minHeight, fresh.maxHeight)
		}
	}
}

func TestHeightfieldAABB(t *testing.T) {
	h := randomHeightfield(7, 9, 5)
	transform := Transform{Position: mgl64.Vec3{1, 2, 3}, Rotation: mgl64.QuatRotate(0.7, mgl64.Vec3{0, 1, 0})}
	aabb := h.ComputeAABB(transform)
	for x := 0; x < h.XSamples; x++ {
		for z := 0; z < h.ZSamples; z++ {
			if !aabb.ContainsPoint(transform.ToWorld(h.localVertex(x, z))) {
				t.Fatalf("the sample (%d, %d) is outside the AABB", x, z)
			}
		}
	}
}

func TestHeightfieldNeedsSamples(t *testing.T) {
	defer func() {
		if recover() == nil {
			t.Error("no panic")
		}
	}()
	NewHeightfield(3, 3, make([]float32, 8), mgl64.Vec3{1, 1, 1})
}
