package actor

import (
	"math"
	"math/rand"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// hillsHeightfield: smooth hills of a few meters, with samples every 0.5 m along x and 0.75 m along z
func hillsHeightfield(xSamples, zSamples int) *Heightfield {
	heights := make([]float32, xSamples*zSamples)
	for x := 0; x < xSamples; x++ {
		for z := 0; z < zSamples; z++ {
			heights[x*zSamples+z] = float32(1.5*math.Sin(float64(x)*0.21)*math.Cos(float64(z)*0.17) + 0.4*math.Sin(float64(x+2*z)*0.9))
		}
	}
	return NewHeightfield(xSamples, zSamples, heights, mgl64.Vec3{0.5, 2, 0.75})
}

// bruteForceRay: the first triangle hit from above, by Möller-Trumbore on every triangle of the terrain. A point may be
// slack out of a triangle (barycentric): Möller-Trumbore leaks along the edges a ray lies on
func bruteForceRay(h *Heightfield, origin, translation mgl64.Vec3, maxFraction, slack float64) (float64, int32, bool) {
	best, triangle, found := maxFraction, NoTriangle, false
	for x := 0; x < h.XSamples-1; x++ {
		for z := 0; z < h.ZSamples-1; z++ {
			if !h.hasCell(x, z) {
				continue
			}
			for t := 0; t < 2; t++ {
				vertices := h.localTriangle(x, z, t)
				edge1, edge2 := vertices[1].Sub(vertices[0]), vertices[2].Sub(vertices[0])
				p := translation.Cross(edge2)
				// the determinant is positive for a ray coming from the side of the normal
				determinant := edge1.Dot(p)
				if determinant <= 0 {
					continue
				}
				s := origin.Sub(vertices[0])
				u := s.Dot(p) / determinant
				q := s.Cross(edge1)
				v := translation.Dot(q) / determinant
				fraction := edge2.Dot(q) / determinant
				if u < -slack || v < -slack || u+v > 1+slack || fraction < 0 || fraction >= best {
					continue
				}
				best, triangle, found = fraction, int32(2*(x*(h.ZSamples-1)+z)+t), true
			}
		}
	}
	return best, triangle, found
}

// triangleOf the index of a hit
func triangleOf(h *Heightfield, index int32) [3]mgl64.Vec3 {
	cell := int(index) / 2
	return h.localTriangle(cell/(h.ZSamples-1), cell%(h.ZSamples-1), int(index)%2)
}

// Criterion 8: a vertical ray hits the terrain at the height of HeightAt, with the triangle under it and its normal
func TestHeightfieldCastRayVertical(t *testing.T) {
	h := hillsHeightfield(40, 30)
	r := rand.New(rand.NewSource(3))
	halfX, halfZ := float64(h.XSamples-1)/2*h.Scale.X(), float64(h.ZSamples-1)/2*h.Scale.Z()
	for i := 0; i < 2000; i++ {
		x, z := (r.Float64()*2-1)*halfX, (r.Float64()*2-1)*halfZ
		height, _ := h.HeightAt(x, z)
		hit, ok := h.CastRay(mgl64.Vec3{x, 30, z}, mgl64.Vec3{0, -80, 0}, 1)
		if !ok || math.Abs(30-80*hit.Fraction-height) > 1e-9 {
			t.Fatalf("ray %d at (%.3f, %.3f): hit %v at the height %.12f, HeightAt gives %.12f", i, x, z, ok, 30-80*hit.Fraction, height)
		}
		fx, fz := x/h.Scale.X()+float64(h.XSamples-1)/2, z/h.Scale.Z()+float64(h.ZSamples-1)/2
		cellX, cellZ := int(fx), int(fz)
		triangle := int32(2*(cellX*(h.ZSamples-1)+cellZ) + 1)
		if fz-float64(cellZ) >= fx-float64(cellX) {
			triangle--
		}
		if hit.Triangle != triangle {
			t.Fatalf("ray %d at (%.3f, %.3f): triangle %d, want %d", i, x, z, hit.Triangle, triangle)
		}
		if normal := triangleNormal(triangleOf(h, hit.Triangle)); !near(hit.Normal, normal, 1e-9) {
			t.Fatalf("ray %d: normal %v, the triangle has %v", i, hit.Normal, normal)
		}
		// cut right before the hit
		if _, ok := h.CastRay(mgl64.Vec3{x, 30, z}, mgl64.Vec3{0, -80, 0}, hit.Fraction-1e-9); ok {
			t.Fatalf("ray %d: a hit before its fraction", i)
		}
	}
}

// Criterion 8: a ray thrown exactly at a sample, on an edge or on the diagonal of a cell hits the terrain, on the
// borders of the terrain too: a point on an edge belongs to the triangles around it
func TestHeightfieldCastRayOnEdgesAndVertices(t *testing.T) {
	h := hillsHeightfield(12, 9)
	check := func(what string, p mgl64.Vec3) {
		t.Helper()
		directions := []mgl64.Vec3{{0, -40, 0}, {0.3, -40, 0}, {0, -40, -0.2}, {0.25, -40, 0.25}, {-0.4, -40, 0.1}}
		for _, direction := range directions {
			// the ray goes through the point p of the surface
			origin := p.Sub(direction.Mul(0.5))
			hit, ok := h.CastRay(origin, direction, 1)
			if !ok {
				t.Errorf("%s at %v along %v: no hit", what, p, direction)
				continue
			}
			if point := origin.Add(direction.Mul(hit.Fraction)); !near(point, p, 1e-9) {
				t.Errorf("%s at %v along %v: hit at %v", what, p, direction, point)
			}
			vertices := triangleOf(h, hit.Triangle)
			if distance := math.Abs(p.Sub(vertices[0]).Dot(triangleNormal(vertices))); distance > 1e-9 {
				t.Errorf("%s at %v along %v: the triangle %d is %.3g m from the point", what, p, direction, hit.Triangle, distance)
			}
		}
	}
	for x := 0; x < h.XSamples; x++ {
		for z := 0; z < h.ZSamples; z++ {
			vertex := h.localVertex(x, z)
			check("sample", vertex)
			if x+1 < h.XSamples {
				check("edge along x", vertex.Add(h.localVertex(x+1, z)).Mul(0.5))
			}
			if z+1 < h.ZSamples {
				check("edge along z", vertex.Add(h.localVertex(x, z+1)).Mul(0.5))
			}
			if x+1 < h.XSamples && z+1 < h.ZSamples {
				check("diagonal", vertex.Add(h.localVertex(x+1, z+1)).Mul(0.5))
				check("diagonal", vertex.Mul(0.25).Add(h.localVertex(x+1, z+1).Mul(0.75)))
			}
		}
	}
}

// A ray which falls on an edge or on a sample hits several triangles at the same fraction: the triangle of the lowest
// index is named, wherever the ray comes from. (The rule only holds at the very same fraction, as here on a flat
// terrain: elsewhere the rounding chooses, and both triangles are right)
func TestHeightfieldCastRayOnAnEdgeNamesTheLowestTriangle(t *testing.T) {
	const samples = 6
	h := NewHeightfield(samples, samples, make([]float32, samples*samples), mgl64.Vec3{1, 1, 1})
	cells := samples - 1
	triangle := func(x, z, half int) int32 { return int32(2*(x*cells+z) + half) }
	cases := []struct {
		what string
		// the point, in the grid (the sample (x, z) at (x, z))
		x, z float64
		want int32
	}{
		{"the edge between 2 columns", 2, 1.5, triangle(1, 1, 1)},
		{"the edge between 2 rows", 2.5, 2, triangle(2, 1, 0)},
		{"the diagonal of a cell", 2.5, 2.5, triangle(2, 2, 0)},
		{"a sample", 2, 2, triangle(1, 1, 0)},
	}
	directions := []mgl64.Vec3{{1, -2, 0}, {-1, -2, 0}, {0, -2, 1}, {0, -2, -1}, {1, -2, 0.5}, {-1, -2, -0.5}, {0.5, -2, -1}, {-0.5, -2, 1}, {0, -2, 0}}
	for _, c := range cases {
		p := mgl64.Vec3{c.x - float64(cells)/2, 0, c.z - float64(cells)/2}
		for _, direction := range directions {
			hit, ok := h.CastRay(p.Sub(direction.Mul(0.5)), direction, 1)
			if !ok || hit.Fraction != 0.5 {
				t.Errorf("%s along %v: hit %v at the fraction %v, want 0.5", c.what, direction, ok, hit.Fraction)
				continue
			}
			if hit.Triangle != c.want {
				t.Errorf("%s along %v: triangle %d, want %d, the lowest index", c.what, direction, hit.Triangle, c.want)
			}
		}
	}
}

// Criterion 8: a ray goes through a hole, through the terrain from below, and hits nothing out of the terrain
func TestHeightfieldCastRayHolesAndBackFaces(t *testing.T) {
	h := hillsHeightfield(12, 9)
	cellsZ := h.ZSamples - 1
	h.Holes = make([]bool, (h.XSamples-1)*cellsZ)
	h.Holes[4*cellsZ+3], h.Holes[5*cellsZ+3] = true, true
	h.Update(0, 0, h.XSamples-1, h.ZSamples-1)
	down, up := mgl64.Vec3{0, -40, 0}, mgl64.Vec3{0, 40, 0}

	inHole := h.localVertex(4, 3).Add(h.localVertex(5, 4)).Mul(0.5)
	if _, ok := h.CastRay(inHole.Add(mgl64.Vec3{0.05, 20, 0.02}), down, 1); ok {
		t.Error("a ray through a hole hits the terrain")
	}
	// the edge between both holes has no triangle, the edges around the holes belong to the triangles beside
	between := h.localVertex(5, 3).Add(h.localVertex(5, 4)).Mul(0.5)
	if _, ok := h.CastRay(between.Add(mgl64.Vec3{0, 20, 0}), down, 1); ok {
		t.Error("a ray on the edge between 2 holes hits the terrain")
	}
	for _, edge := range [][2][2]int{{{4, 3}, {4, 4}}, {{6, 3}, {6, 4}}, {{4, 3}, {5, 3}}, {{5, 4}, {6, 4}}} {
		p := h.localVertex(edge[0][0], edge[0][1]).Add(h.localVertex(edge[1][0], edge[1][1])).Mul(0.5)
		hit, ok := h.CastRay(p.Add(mgl64.Vec3{0, 20, 0}), down, 1)
		if !ok || math.Abs(20-40*hit.Fraction) > 1e-9 {
			t.Errorf("a ray on the edge %v of a hole: hit %v at the fraction %g", edge, ok, hit.Fraction)
		}
		if cell := int(hit.Triangle) / 2; ok && h.Holes[cell] {
			t.Errorf("a ray on the edge %v of a hole hits the triangle %d of the hole", edge, hit.Triangle)
		}
	}

	r := rand.New(rand.NewSource(4))
	halfX, halfZ := float64(h.XSamples-1)/2*h.Scale.X(), float64(h.ZSamples-1)/2*h.Scale.Z()
	for i := 0; i < 500; i++ {
		x, z := (r.Float64()*2-1)*halfX, (r.Float64()*2-1)*halfZ
		if _, ok := h.CastRay(mgl64.Vec3{x, -20, z}, up.Add(mgl64.Vec3{r.Float64() - 0.5, 0, r.Float64() - 0.5}), 1); ok {
			t.Fatalf("ray %d from below (%.3f, %.3f) hits the terrain", i, x, z)
		}
		// beside the terrain
		side := mgl64.Vec3{halfX + 1e-6 + r.Float64(), 20, z}
		if i%2 == 0 {
			side = mgl64.Vec3{x, 20, -halfZ - 1e-6 - r.Float64()}
		}
		if _, ok := h.CastRay(side, down, 1); ok {
			t.Fatalf("ray %d beside the terrain at %v hits it", i, side)
		}
	}
	// a point of the surface is not in the terrain: it has no inside
	surface := h.localVertex(2, 2)
	if _, ok := h.CastRay(surface, mgl64.Vec3{}, 1); ok {
		t.Error("a ray without length on the surface hits the terrain")
	}
	if _, ok := h.CastRay(surface.Sub(mgl64.Vec3{0, 5, 0}), mgl64.Vec3{}, 1); ok {
		t.Error("a ray without length under the surface hits the terrain")
	}
	for _, value := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for k := 0; k < 3; k++ {
			origin, translation := mgl64.Vec3{0, 20, 0}, down
			origin[k] = value
			if _, ok := h.CastRay(origin, translation, 1); ok {
				t.Errorf("a hit with the origin %v", origin)
			}
			origin, translation = mgl64.Vec3{0, 20, 0}, down
			translation[k] = value
			if _, ok := h.CastRay(origin, translation, 1); ok {
				t.Errorf("a hit with the translation %v", translation)
			}
		}
	}
}

// randomTerrainRay: a ray around the terrain: steep, slanted, grazing the hills, along an axis or along a line of the
// grid
func randomTerrainRay(r *rand.Rand, h *Heightfield, i int) (mgl64.Vec3, mgl64.Vec3) {
	halfX, halfZ := float64(h.XSamples-1)/2*h.Scale.X(), float64(h.ZSamples-1)/2*h.Scale.Z()
	around := func(height float64) mgl64.Vec3 {
		return mgl64.Vec3{(r.Float64()*2.4 - 1.2) * halfX, height, (r.Float64()*2.4 - 1.2) * halfZ}
	}
	origin, target := around(4+6*r.Float64()), around(-4)
	switch i % 5 {
	case 1:
		// grazing: towards the middle of a triangle, 1 to 4° from its plane
		triangle := h.localTriangle(r.Intn(h.XSamples-1), r.Intn(h.ZSamples-1), r.Intn(2))
		normal := triangleNormal(triangle)
		tangent := normal.Cross(mgl64.Vec3{r.Float64() - 0.5, r.Float64() - 0.5, r.Float64() - 0.5}).Normalize()
		angle := (1 + 3*r.Float64()) * math.Pi / 180
		direction := tangent.Mul(math.Cos(angle)).Sub(normal.Mul(math.Sin(angle)))
		target = triangle[0].Add(triangle[1]).Add(triangle[2]).Mul(1.0 / 3)
		origin = target.Sub(direction.Mul(1 + 8*r.Float64()))
		target = target.Add(direction)
	case 2:
		// almost level
		origin = around(4*r.Float64() - 2)
		target = around(origin.Y() - 0.3*r.Float64())
	case 3:
		// along an axis
		target = origin
		target[r.Intn(3)] -= 5 + 40*r.Float64()
	case 4:
		// along a line of the grid, level or slanted
		line := h.localVertex(r.Intn(h.XSamples), r.Intn(h.ZSamples))
		origin, target = around(3*r.Float64()), around(3*r.Float64()-3)
		axis := 2 * r.Intn(2)
		origin[axis], target[axis] = line[axis], line[axis]
	}
	return origin, target.Sub(origin)
}

// Criterion 8: 1000 random rays, grazing ones too, hit the same triangle at the same fraction as a brute force on every
// triangle of the terrain
func TestHeightfieldCastRayIsTheBruteForce(t *testing.T) {
	for _, holes := range []bool{false, true} {
		h := hillsHeightfield(70, 45)
		if holes {
			r := rand.New(rand.NewSource(6))
			h.Holes = make([]bool, (h.XSamples-1)*(h.ZSamples-1))
			for i := range h.Holes {
				h.Holes[i] = r.Intn(10) == 0
			}
			h.Update(0, 0, h.XSamples-1, h.ZSamples-1)
		}
		r := rand.New(rand.NewSource(5))
		hits, misses, grazing := 0, 0, 0
		for i := 0; i < 3000; i++ {
			origin, translation := randomTerrainRay(r, h, i)
			maxFraction := 1.0
			if i%3 == 0 {
				maxFraction = r.Float64()
			}
			slack := 0.0
			if i%5 == 4 {
				slack = 1e-12
			}
			wantFraction, wantTriangle, want := bruteForceRay(h, origin, translation, maxFraction, slack)
			hit, ok := h.CastRay(origin, translation, maxFraction)
			if i%5 == 4 {
				// along a line of the grid, the ray is on the edges of 2 rows of triangles: both rows hold the hit
				if ok != want || (ok && math.Abs(hit.Fraction-wantFraction) > 1e-9) {
					t.Fatalf("holes %v, ray %d along a line: hit %v at %.12f, the brute force has %v at %.12f", holes, i, ok, hit.Fraction, want, wantFraction)
				}
				continue
			}
			if ok != want || (ok && (math.Abs(hit.Fraction-wantFraction) > 1e-12 || hit.Triangle != wantTriangle)) {
				t.Fatalf("holes %v, ray %d from %v along %v: hit %v at %.15f on the triangle %d, the brute force has %v at %.15f on %d",
					holes, i, origin, translation, ok, hit.Fraction, hit.Triangle, want, wantFraction, wantTriangle)
			}
			if !ok {
				misses++
				continue
			}
			hits++
			normal := triangleNormal(triangleOf(h, hit.Triangle))
			if !near(hit.Normal, normal, 1e-9) {
				t.Fatalf("holes %v, ray %d: normal %v, the triangle has %v", holes, i, hit.Normal, normal)
			}
			if -translation.Normalize().Dot(normal) < 0.1 {
				grazing++
			}
		}
		if hits < 800 || misses < 300 || grazing < 50 {
			t.Errorf("holes %v: %d hits (%d grazing) and %d misses, the rays don't cover them all", holes, hits, grazing, misses)
		}
	}
}

// touchedCells: the cells whose square the box touches during its motion, by sampling the motion
func touchedCells(h *Heightfield, origin, translation, extents mgl64.Vec3, maxFraction float64) map[[2]int]bool {
	cells := map[[2]int]bool{}
	const samples = 4000
	for k := 0; k <= samples; k++ {
		center := origin.Add(translation.Mul(maxFraction * float64(k) / samples))
		low := [2]float64{(center.X()-extents.X())/h.Scale.X() + float64(h.XSamples-1)/2, (center.Z()-extents.Z())/h.Scale.Z() + float64(h.ZSamples-1)/2}
		high := [2]float64{(center.X()+extents.X())/h.Scale.X() + float64(h.XSamples-1)/2, (center.Z()+extents.Z())/h.Scale.Z() + float64(h.ZSamples-1)/2}
		for x := max(0, int(math.Floor(low[0]))); x <= min(h.XSamples-2, int(math.Floor(high[0]))); x++ {
			for z := max(0, int(math.Floor(low[1]))); z <= min(h.ZSamples-2, int(math.Floor(high[1]))); z++ {
				// the heights of the box against the heights of the cell
				lowest, highest := math.Inf(1), math.Inf(-1)
				for _, corner := range [4][2]int{{0, 0}, {0, 1}, {1, 0}, {1, 1}} {
					height := h.height(x+corner[0], z+corner[1])
					lowest, highest = math.Min(lowest, height), math.Max(highest, height)
				}
				if h.hasCell(x, z) && center.Y()-extents.Y() <= highest && center.Y()+extents.Y() >= lowest {
					cells[[2]int{x, z}] = true
				}
			}
		}
	}
	return cells
}

// The walk gives every cell the moving box can touch, once, none far from its path, and stops at its limit
func TestHeightfieldWalkCells(t *testing.T) {
	h := hillsHeightfield(70, 45)
	h.Holes = make([]bool, (h.XSamples-1)*(h.ZSamples-1))
	h.Holes[10*(h.ZSamples-1)+10] = true
	r := rand.New(rand.NewSource(8))
	visitedTotal, neededTotal := 0, 0
	for i := 0; i < 400; i++ {
		origin, translation := randomTerrainRay(r, h, i)
		extents := mgl64.Vec3{}
		if i%2 == 0 {
			extents = mgl64.Vec3{2 * r.Float64(), r.Float64(), 2 * r.Float64()}
		}
		maxFraction := 0.2 + 0.8*r.Float64()
		needed := touchedCells(h, origin, translation, extents, maxFraction)

		walk := h.WalkCells(origin, translation, extents, maxFraction)
		visited := map[[2]int]bool{}
		for {
			x, z, ok := walk.Next(maxFraction)
			if !ok {
				break
			}
			if visited[[2]int{x, z}] {
				t.Fatalf("walk %d: the cell (%d, %d) twice", i, x, z)
			}
			if !h.hasCell(x, z) {
				t.Fatalf("walk %d: the cell (%d, %d) is a hole, or out of the terrain", i, x, z)
			}
			visited[[2]int{x, z}] = true
		}
		for cell := range needed {
			if !visited[cell] {
				t.Fatalf("walk %d from %v along %v, extents %v: the cell %v is touched and not visited", i, origin, translation, extents, cell)
			}
		}
		// no cell further than a cell from the path of the box
		wide := touchedCells(h, origin, translation, extents.Add(mgl64.Vec3{h.Scale.X(), 1e9, h.Scale.Z()}), maxFraction)
		for cell := range visited {
			if !wide[cell] {
				t.Fatalf("walk %d from %v along %v, extents %v: the cell %v is visited, far from the path", i, origin, translation, extents, cell)
			}
		}
		visitedTotal, neededTotal = visitedTotal+len(visited), neededTotal+len(needed)

		// a walk limited to the half of the motion never visits more, and visits what the half motion touches
		half := h.WalkCells(origin, translation, extents, maxFraction)
		early := touchedCells(h, origin, translation, extents, maxFraction/2)
		count := 0
		for {
			x, z, ok := half.Next(maxFraction / 2)
			if !ok {
				break
			}
			count++
			delete(early, [2]int{x, z})
		}
		if len(early) > 0 || count > len(visited) {
			t.Fatalf("walk %d: limited to its half, %d cells of the half motion are missing, %d cells for %d of the whole", i, len(early), count, len(visited))
		}
	}
	if neededTotal < 2000 || visitedTotal > 3*neededTotal {
		t.Errorf("%d cells visited for %d touched: the walk is too wide, or the motions too short", visitedTotal, neededTotal)
	}
}

// countCells: the cells the walk gives up to the limit
func countCells(walk CellWalk, limit float64) int {
	cells := 0
	for {
		if _, _, ok := walk.Next(limit); !ok {
			return cells
		}
		cells++
	}
}

// The exact number of cells of a walk: the walk stops at its limit, only covers the part of the motion between the
// lowest and the highest heights of the terrain, and skips the blocks the box flies over
func TestHeightfieldWalkCellCounts(t *testing.T) {
	const samples = 65
	// in the grid (the sample (x, z) at (x, z)) for a scale of 1
	local := func(x, y, z float64) mgl64.Vec3 { return mgl64.Vec3{x - (samples-1)/2, y, z - (samples-1)/2} }

	// heights of 0 & 1 on a checkerboard: every cell, and every block, goes from 0 to 1
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32((x + z) % 2)
		}
	}
	checkerboard := NewHeightfield(samples, samples, heights, mgl64.Vec3{1, 1, 1})

	// a level ray through 41 columns, entered at the fraction (column - 0.5) / 40
	level := func() CellWalk {
		return checkerboard.WalkCells(local(0.5, 0.5, 20.5), mgl64.Vec3{40, 0, 0}, mgl64.Vec3{}, 1)
	}
	for _, c := range []struct {
		limit float64
		want  int
	}{{1, 41}, {0.25, 11}, {0.38, 16}, {0, 1}} {
		if cells := countCells(level(), c.limit); cells != c.want {
			t.Errorf("level ray limited to %v: %d cells, want %d", c.limit, cells, c.want)
		}
	}

	// a ray going down by half a meter per column, and across 0.9 row: it is between the heights 1 and 0 from the
	// fraction 0.5 to 0.6 only, over the columns 10 to 12, entered and left in their middle: the rows 14, 14 & 15, 15
	falling := checkerboard.WalkCells(local(0.5, 6, 5.05), mgl64.Vec3{20, -10, 18}, mgl64.Vec3{}, 1)
	if cells := countCells(falling, 1); cells != 4 {
		t.Errorf("falling ray: %d cells, want the 4 cells it is over between the heights of the terrain", cells)
	}

	// a ridge 5 m high along x, in the rows 16 to 31 (a block of rows), the rest flat: a box 40 rows wide, flying
	// 2.5 m over the flat part across 4 columns, only gets the rows of the block of the ridge
	heights = make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := HeightfieldBlockSize + 1; z < 2*HeightfieldBlockSize; z++ {
			heights[x*samples+z] = 5
		}
	}
	ridge := NewHeightfield(samples, samples, heights, mgl64.Vec3{1, 1, 1})
	wide := ridge.WalkCells(local(2.5, 3, 24), mgl64.Vec3{3, 0, 0}, mgl64.Vec3{0, 0.5, 20}, 1)
	if cells := countCells(wide, 1); cells != 4*HeightfieldBlockSize {
		t.Errorf("wide box over the ridge: %d cells, want %d (4 columns of the %d rows of the block of the ridge)", cells, 4*HeightfieldBlockSize, HeightfieldBlockSize)
	}
}

// The walk skips the blocks the ray flies over: a long level ray over hills visits a few cells only
func TestHeightfieldWalkSkipsBlocks(t *testing.T) {
	const samples = 513
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32(2 * math.Sin(float64(x)*0.02) * math.Sin(float64(z)*0.02))
		}
	}
	// a peak in the last blocks
	heights[500*samples+250] = 30
	h := NewHeightfield(samples, samples, heights, mgl64.Vec3{1, 1, 1})
	origin, translation := mgl64.Vec3{-255, 5, -6.3}, mgl64.Vec3{510, 0, 0.4}
	walk := h.WalkCells(origin, translation, mgl64.Vec3{}, 1)
	cells := 0
	for {
		if _, _, ok := walk.Next(1); !ok {
			break
		}
		cells++
	}
	if cells == 0 || cells > 3*HeightfieldBlockSize {
		t.Errorf("%d cells visited by a ray of 510 m over the hills, want the cells of the block of the peak only", cells)
	}
	hit, ok := h.CastRay(origin, translation, 1)
	if !ok || int(hit.Triangle)/2/(samples-1) < 499 || int(hit.Triangle)/2/(samples-1) > 500 {
		t.Errorf("the ray over the hills: hit %v on the triangle %d, want a hit on the peak", ok, hit.Triangle)
	}
}

// A ray against a heightfield doesn't allocate
func TestHeightfieldCastRayDoesNotAllocate(t *testing.T) {
	h := hillsHeightfield(70, 45)
	hits := 0
	allocs := testing.AllocsPerRun(100, func() {
		if _, ok := h.CastRay(mgl64.Vec3{-15, 3, -12}, mgl64.Vec3{30, -4, 25}, 1); ok {
			hits++
		}
	})
	if allocs > 0 || hits == 0 {
		t.Errorf("%.1f allocations per ray (%d hits), want 0", allocs, hits)
	}
}
