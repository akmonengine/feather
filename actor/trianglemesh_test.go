package actor

import (
	"errors"
	"math"
	"math/rand"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// gridMesh: n x n cells of size cell, at the height of the function, the vertices shared
func gridMesh(n int, cell float64, height func(x, z float64) float64) *TriangleMesh {
	vertices := make([]mgl64.Vec3, 0, (n+1)*(n+1))
	for x := 0; x <= n; x++ {
		for z := 0; z <= n; z++ {
			px, pz := (float64(x)-float64(n)/2)*cell, (float64(z)-float64(n)/2)*cell
			vertices = append(vertices, mgl64.Vec3{px, height(px, pz), pz})
		}
	}
	indices := make([]int32, 0, 6*n*n)
	for x := 0; x < n; x++ {
		for z := 0; z < n; z++ {
			a, b, c, d := int32(x*(n+1)+z), int32(x*(n+1)+z+1), int32((x+1)*(n+1)+z+1), int32((x+1)*(n+1)+z)
			indices = append(indices, a, b, c, a, c, d)
		}
	}
	m, err := NewTriangleMesh(vertices, indices)
	if err != nil {
		panic(err)
	}
	return m
}

// cubeSoup: the 12 triangles of a cube as a soup of 36 vertices, counterclockwise seen from outside
func cubeSoup(half float64) ([]mgl64.Vec3, []int32) {
	faces := [6][4]mgl64.Vec3{
		{{-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1}, {-1, 1, -1}}, // -x
		{{1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {1, -1, 1}},     // +x
		{{-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1}}, // -y
		{{-1, 1, -1}, {-1, 1, 1}, {1, 1, 1}, {1, 1, -1}},     // +y
		{{-1, -1, -1}, {-1, 1, -1}, {1, 1, -1}, {1, -1, -1}}, // -z
		{{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}},     // +z
	}
	var vertices []mgl64.Vec3
	var indices []int32
	for _, f := range faces {
		i := int32(len(vertices))
		for _, v := range f {
			vertices = append(vertices, v.Mul(half))
		}
		indices = append(indices, i, i+1, i+2, i, i+2, i+3)
	}
	return vertices, indices
}

// A cube given as a soup of triangles is welded to 8 vertices; its 12 edges are convex and active, the diagonals of
// its faces are neither; a flat grid has inactive inner edges and active borders; a fold inwards is not convex; an
// edge shared by 3 triangles is active; a sheet of 2 triangles back to back has active edges
func TestTriangleMeshWeldsAndClassifiesEdges(t *testing.T) {
	vertices, indices := cubeSoup(0.5)
	m, err := NewTriangleMesh(vertices, indices)
	if err != nil {
		t.Fatal(err)
	}
	distinct := map[int32]bool{}
	for _, i := range m.Indices {
		distinct[i] = true
	}
	if len(distinct) != 8 {
		t.Fatalf("%d distinct vertices after the weld, want 8", len(distinct))
	}
	for i := 0; i < m.TriangleCount(); i++ {
		triangle, edges := m.Triangle(int32(i))
		for e := 0; e < 3; e++ {
			diagonal := triangle[e].Sub(triangle[(e+1)%3]).LenSqr() > 1.5
			active, convex := edges&(1<<e) != 0, edges&(1<<(3+e)) != 0
			if diagonal && (active || convex) {
				t.Errorf("triangle %d: the diagonal %d is active %v convex %v", i, e, active, convex)
			}
			if !diagonal && (!active || !convex) {
				t.Errorf("triangle %d: the edge %d of the cube is active %v convex %v", i, e, active, convex)
			}
		}
	}
	if m.Bounds() != (AABB{Min: mgl64.Vec3{-0.5, -0.5, -0.5}, Max: mgl64.Vec3{0.5, 0.5, 0.5}}) {
		t.Errorf("bounds %v", m.Bounds())
	}

	// a flat grid: borders active, inner edges inactive and not convex
	grid := gridMesh(4, 1, func(x, z float64) float64 { return 0 })
	for i := 0; i < grid.TriangleCount(); i++ {
		triangle, edges := grid.Triangle(int32(i))
		for e := 0; e < 3; e++ {
			a, c := triangle[e], triangle[(e+1)%3]
			border := (a[0] == c[0] && math.Abs(a[0]) == 2) || (a[2] == c[2] && math.Abs(a[2]) == 2)
			if active := edges&(1<<e) != 0; active != border {
				t.Errorf("grid triangle %d edge %d (%v-%v): active %v, border %v", i, e, a, c, active, border)
			}
			if edges&(1<<(3+e)) != 0 && !border {
				t.Errorf("grid triangle %d edge %d is convex", i, e)
			}
		}
	}

	// a V: 2 triangles folded inwards (seen from above): the fold is neither convex nor active
	v, err := NewTriangleMesh([]mgl64.Vec3{{-1, 1, -1}, {-1, 1, 1}, {0, 0, 1}, {0, 0, -1}, {1, 1, 1}, {1, 1, -1}}, []int32{0, 1, 2, 0, 2, 3, 3, 2, 4, 3, 4, 5})
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 4; i++ {
		triangle, edges := v.Triangle(int32(i))
		for e := 0; e < 3; e++ {
			if a, c := triangle[e], triangle[(e+1)%3]; a[0] == 0 && c[0] == 0 && edges&((1|1<<3)<<e) != 0 {
				t.Errorf("the fold of the V is active or convex (triangle %d)", i)
			}
		}
	}
	// a ridge: the fold is convex and active
	ridge, err := NewTriangleMesh([]mgl64.Vec3{{-1, 0, -1}, {-1, 0, 1}, {0, 1, 1}, {0, 1, -1}, {1, 0, 1}, {1, 0, -1}}, []int32{0, 1, 2, 0, 2, 3, 3, 2, 4, 3, 4, 5})
	if err != nil {
		t.Fatal(err)
	}
	folds := 0
	for i := 0; i < 4; i++ {
		triangle, edges := ridge.Triangle(int32(i))
		for e := 0; e < 3; e++ {
			if a, c := triangle[e], triangle[(e+1)%3]; a[0] == 0 && c[0] == 0 {
				folds++
				if edges&((1|1<<3)<<e) != (1|1<<3)<<e {
					t.Errorf("the fold of the ridge is not active and convex (triangle %d)", i)
				}
			}
		}
	}
	if folds != 2 {
		t.Errorf("%d triangles on the fold, want 2", folds)
	}
	// 3 triangles on an edge, and 2 triangles back to back
	fan, err := NewTriangleMesh([]mgl64.Vec3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, -1, 0}}, []int32{0, 1, 2, 0, 3, 1, 0, 1, 4})
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 3; i++ {
		if _, edges := fan.Triangle(int32(i)); edges&(1|1<<3) != 1|1<<3 {
			t.Errorf("triangle %d of the fan: its shared edge is not active", i)
		}
	}
	sheet, err := NewTriangleMesh([]mgl64.Vec3{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}}, []int32{0, 2, 1, 0, 1, 2})
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 2; i++ {
		if _, edges := sheet.Triangle(int32(i)); edges != 0b111111 {
			t.Errorf("triangle %d of the sheet: edges %06b, want all active", i, edges)
		}
	}
}

// The indices are checked, the degenerate triangles are kept out of the tree, without edge, and a mesh of degenerate
// triangles only is refused
func TestTriangleMeshDegenerate(t *testing.T) {
	vertices := []mgl64.Vec3{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}, {0.5, 0, 0}}
	for _, indices := range [][]int32{{0, 1}, {0, 1, 2, 3}, {0, 1, 5}, {0, 1, -1}, nil} {
		if _, err := NewTriangleMesh(vertices, indices); !errors.Is(err, ErrMeshIndices) {
			t.Errorf("indices %v: %v, want ErrMeshIndices", indices, err)
		}
	}
	if _, err := NewTriangleMesh(vertices, []int32{0, 1, 4, 0, 0, 1}); !errors.Is(err, ErrMeshDegenerate) {
		t.Errorf("degenerate triangles only: %v, want ErrMeshDegenerate", err)
	}
	m, err := NewTriangleMesh(vertices, []int32{0, 1, 4, 0, 2, 1, 1, 2, 3})
	if err != nil {
		t.Fatal(err)
	}
	if m.TriangleCount() != 3 {
		t.Fatalf("%d triangles, want 3 (the degenerate one kept, out of the tree)", m.TriangleCount())
	}
	if _, edges := m.Triangle(0); edges != 0 {
		t.Errorf("the degenerate triangle has edges %06b", edges)
	}
	found := m.OverlapTriangles(AABB{Min: mgl64.Vec3{-1, -1, -1}, Max: mgl64.Vec3{2, 1, 2}}, nil)
	if len(found) != 2 || found[0] == 0 || found[1] == 0 {
		t.Errorf("the tree gives %v, want the triangles 1 and 2", found)
	}
	// the border of the mesh along the degenerate triangle is a border: active
	if _, edges := m.Triangle(1); edges&(1<<2) == 0 {
		t.Errorf("the edge 2-0 of the triangle 1 (under the degenerate triangle) is not active: %06b", edges)
	}
}

// degenerate: the triangle has no area (it is out of the tree)
func degenerate(triangle [3]mgl64.Vec3) bool {
	return triangle[1].Sub(triangle[0]).Cross(triangle[2].Sub(triangle[0])).Len() < 2*meshMinTriangleArea
}

// bruteForceTriangles: the triangles whose AABB overlaps the bounds
func bruteForceTriangles(m *TriangleMesh, bounds AABB) map[int32]bool {
	found := map[int32]bool{}
	for i := 0; i < m.TriangleCount(); i++ {
		if triangle, _ := m.Triangle(int32(i)); !degenerate(triangle) && triangleBounds(triangle).Overlaps(bounds) {
			found[int32(i)] = true
		}
	}
	return found
}

// hillsMesh: 100 x 100 cells of 0.25 m on hills, 20 000 triangles
func hillsMesh() *TriangleMesh {
	return gridMesh(100, 0.25, func(x, z float64) float64 { return 2*math.Sin(x*0.3)*math.Cos(z*0.2) + 0.3*math.Sin(x*2+z) })
}

// The tree gives the triangles whose AABB overlaps a box, exactly the ones of the brute force, each triangle is in one
// leaf of at most 8 triangles, every node contains its children, and the height is bounded by log(triangles)/log(1.5)
func TestTriangleMeshOverlapTrianglesAndTree(t *testing.T) {
	m := hillsMesh()
	r := rand.New(rand.NewSource(5))
	found := 0
	for i := 0; i < 500; i++ {
		center := mgl64.Vec3{(r.Float64() - 0.5) * 26, r.NormFloat64() * 2, (r.Float64() - 0.5) * 26}
		half := mgl64.Vec3{r.Float64() * 2, r.Float64() * 2, r.Float64() * 2}
		bounds := AABB{Min: center.Sub(half), Max: center.Add(half)}
		want := bruteForceTriangles(m, bounds)
		got := m.OverlapTriangles(bounds, nil)
		seen := map[int32]bool{}
		for _, tr := range got {
			if !want[tr] || seen[tr] {
				t.Fatalf("box %d: the triangle %d is given, not by the brute force, or twice", i, tr)
			}
			seen[tr] = true
		}
		if len(got) != len(want) {
			t.Fatalf("box %d: %d triangles, the brute force gives %d", i, len(got), len(want))
		}
		found += len(got)
	}
	if found < 5000 {
		t.Fatalf("only %d triangles found", found)
	}

	leaves, inLeaf := 0, make([]int, m.TriangleCount())
	for i := range m.nodes {
		node := &m.nodes[i]
		box := AABB{Min: node.min, Max: node.max}
		if node.right < 0 {
			leaves++
			if node.count < 1 || node.count > meshLeafMaxTriangles {
				t.Fatalf("leaf %d has %d triangles", i, node.count)
			}
			for _, tr := range m.order[node.first : node.first+node.count] {
				inLeaf[tr]++
				triangle, _ := m.Triangle(tr)
				for _, v := range triangle {
					if !box.ContainsPoint(v) {
						t.Fatalf("leaf %d doesn't contain its triangle %d", i, tr)
					}
				}
			}
			continue
		}
		for _, child := range [2]int32{int32(i) + 1, node.right} {
			c := &m.nodes[child]
			for k := 0; k < 3; k++ {
				if c.min[k] < node.min[k] || c.max[k] > node.max[k] {
					t.Fatalf("node %d doesn't contain its child %d", i, child)
				}
			}
		}
	}
	for tr, count := range inLeaf {
		if count != 1 {
			t.Fatalf("triangle %d is in %d leaves", tr, count)
		}
	}
	bound := int(math.Ceil(math.Log(float64(m.TriangleCount()))/math.Log(1.5))) + 1
	t.Logf("%d triangles, %d nodes, %d leaves, height %d (bound %d)", m.TriangleCount(), len(m.nodes), leaves, m.Height(), bound)
	if m.Height() > bound || m.Height() > meshStackSize-2 {
		t.Errorf("height %d over the bound %d", m.Height(), bound)
	}
}

// bruteForceMeshRay: the first triangle hit, seen from the side of its normal, by Möller-Trumbore on every triangle
func bruteForceMeshRay(m *TriangleMesh, origin, translation mgl64.Vec3, slack float64) (float64, int32, bool) {
	best, triangle, found := 1.0, NoTriangle, false
	for i := 0; i < m.TriangleCount(); i++ {
		vertices, _ := m.Triangle(int32(i))
		if degenerate(vertices) {
			continue
		}
		edge1, edge2 := vertices[1].Sub(vertices[0]), vertices[2].Sub(vertices[0])
		p := translation.Cross(edge2)
		determinant := edge1.Dot(p)
		if determinant <= 0 {
			continue
		}
		s := origin.Sub(vertices[0])
		u := s.Dot(p) / determinant
		q := s.Cross(edge1)
		v := translation.Dot(q) / determinant
		fraction := edge2.Dot(q) / determinant
		if u < -slack || v < -slack || u+v > 1+slack || fraction < 0 || fraction > best || (found && fraction == best && int32(i) > triangle) {
			continue
		}
		best, triangle, found = fraction, int32(i), true
	}
	return best, triangle, found
}

// A ray on a mesh gives the triangle of the brute force, at its fraction, with the normal of the triangle; a ray from
// behind goes through; a ray on an edge of the grid hits the triangle of the lowest index; a ray without length hits
// nothing
func TestTriangleMeshCastRay(t *testing.T) {
	m := hillsMesh()
	r := rand.New(rand.NewSource(6))
	hits := 0
	for i := 0; i < 3000; i++ {
		origin := mgl64.Vec3{(r.Float64() - 0.5) * 30, r.Float64()*6 - 1, (r.Float64() - 0.5) * 30}
		translation := mgl64.Vec3{r.NormFloat64() * 5, -r.Float64() * 10, r.NormFloat64() * 5}
		if i%7 == 0 {
			translation[1] = -translation[1] // from below: most of them miss
		}
		wantFraction, wantTriangle, wantOk := bruteForceMeshRay(m, origin, translation, footprintSlack)
		hit, ok := m.CastRay(origin, translation, 1)
		if ok != wantOk {
			t.Fatalf("ray %d from %v: hit %v, the brute force says %v (triangle %d)", i, origin, ok, wantOk, wantTriangle)
		}
		if !ok {
			continue
		}
		hits++
		if hit.Triangle != wantTriangle || math.Abs(hit.Fraction-wantFraction) > 1e-12 {
			t.Fatalf("ray %d: triangle %d at %.12f, the brute force says %d at %.12f", i, hit.Triangle, hit.Fraction, wantTriangle, wantFraction)
		}
		vertices, _ := m.Triangle(hit.Triangle)
		if normal := triangleNormal(vertices); hit.Normal.Sub(normal).Len() > 1e-12 {
			t.Fatalf("ray %d: normal %v, want %v", i, hit.Normal, normal)
		}
	}
	if hits < 800 {
		t.Fatalf("only %d hits", hits)
	}

	// on the lines of a flat grid: a hit, the lowest triangle at the same fraction
	flat := gridMesh(10, 1, func(x, z float64) float64 { return 0 })
	for i := 0; i < 200; i++ {
		x, z := float64(r.Intn(11)-5), (r.Float64()-0.5)*10
		if i%2 == 0 {
			x, z = z, x
		}
		if i%3 == 0 {
			x = math.Round(x)
			z = math.Round(z)
		}
		hit, ok := flat.CastRay(mgl64.Vec3{x, 1, z}, mgl64.Vec3{0, -2, 0}, 1)
		if !ok || math.Abs(hit.Fraction-0.5) > 1e-12 {
			t.Fatalf("ray on the line (%v, %v): %v %v", x, z, hit, ok)
		}
		// the rule of the lowest index holds at the very same fraction: the brute force finds the lowest triangle at
		// its best fraction, the mesh must give it, or a triangle at a fraction closer by the rounding
		fraction, want, _ := bruteForceMeshRay(flat, mgl64.Vec3{x, 1, z}, mgl64.Vec3{0, -2, 0}, footprintSlack)
		if hit.Triangle != want && hit.Fraction >= fraction {
			t.Fatalf("ray on the line (%v, %v): triangle %d at %.17g, want the lowest %d at %.17g", x, z, hit.Triangle, hit.Fraction, want, fraction)
		}
	}
	if _, ok := flat.CastRay(mgl64.Vec3{0.3, -1, 0.3}, mgl64.Vec3{0, 2, 0}, 1); ok {
		t.Errorf("a ray from below hits the grid")
	}
	if _, ok := flat.CastRay(mgl64.Vec3{0.3, 0, 0.3}, mgl64.Vec3{}, 1); ok {
		t.Errorf("a ray without length hits the grid")
	}
	if _, ok := flat.CastRay(mgl64.Vec3{0.3, 1, 0.3}, mgl64.Vec3{0, -2, 0}, 0.4); ok {
		t.Errorf("a ray cut before the grid hits it")
	}
	if _, ok := flat.CastRay(mgl64.Vec3{math.NaN(), 1, 0.3}, mgl64.Vec3{0, -2, 0}, 1); ok {
		t.Errorf("a ray which is not finite hits the grid")
	}
}

// segmentEntersBox: the segment, cut at limit, enters the AABB enlarged by the extents (by slabs)
func segmentEntersBox(box AABB, origin, translation, extents mgl64.Vec3, limit float64) bool {
	enter, exit := 0.0, limit
	for k := 0; k < 3; k++ {
		low, high := box.Min[k]-extents[k]-origin[k], box.Max[k]+extents[k]-origin[k]
		if translation[k] == 0 {
			if low > 0 || high < 0 {
				return false
			}
			continue
		}
		near, far := low/translation[k], high/translation[k]
		if near > far {
			near, far = far, near
		}
		enter, exit = math.Max(enter, near), math.Min(exit, far)
	}
	return enter <= exit
}

// The cast of a box through the tree gives every triangle whose AABB, enlarged by the box, is entered by the segment
// of its center, each once; cut at a limit, every triangle entered before the limit
func TestTriangleMeshCast(t *testing.T) {
	m := hillsMesh()
	r := rand.New(rand.NewSource(8))
	given := map[int32]bool{}
	total := 0
	for i := 0; i < 300; i++ {
		origin := mgl64.Vec3{(r.Float64() - 0.5) * 30, r.Float64()*6 - 1, (r.Float64() - 0.5) * 30}
		translation := mgl64.Vec3{r.NormFloat64() * 5, -r.Float64() * 10, r.NormFloat64() * 5}
		extents := mgl64.Vec3{r.Float64() * 0.5, r.Float64() * 0.5, r.Float64() * 0.5}
		if i%4 == 0 {
			extents = mgl64.Vec3{}
		}
		limit := 1.0
		if i%2 == 1 {
			limit = r.Float64()
		}
		cast := m.Cast(origin, translation, extents, 1)
		clear(given)
		for {
			tr, ok := cast.Next(limit)
			if !ok {
				break
			}
			if given[tr] {
				t.Fatalf("cast %d: the triangle %d is given twice", i, tr)
			}
			given[tr] = true
		}
		for tr := 0; tr < m.TriangleCount(); tr++ {
			triangle, _ := m.Triangle(int32(tr))
			if !degenerate(triangle) && segmentEntersBox(triangleBounds(triangle), origin, translation, extents, limit) && !given[int32(tr)] {
				t.Fatalf("cast %d cut at %.3f: the triangle %d is not given", i, limit, tr)
			}
		}
		total += len(given)
	}
	if total < 3000 {
		t.Fatalf("only %d triangles given", total)
	}
}

// The shape of a mesh: static (infinite mass, no inertia), its AABB turned and moved
func TestTriangleMeshShape(t *testing.T) {
	vertices, indices := cubeSoup(0.5)
	m, err := NewTriangleMesh(vertices, indices)
	if err != nil {
		t.Fatal(err)
	}
	if !math.IsInf(m.ComputeMass(1000), 1) || m.ComputeInertia(1) != (mgl64.Mat3{}) {
		t.Errorf("a mesh has a mass or an inertia")
	}
	transform := Transform{Position: mgl64.Vec3{1, 2, 3}, Rotation: mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0})}
	aabb := m.ComputeAABB(transform)
	half := math.Sqrt(0.5)
	if aabb.Min.Sub(mgl64.Vec3{1 - half, 1.5, 3 - half}).Len() > 1e-12 || aabb.Max.Sub(mgl64.Vec3{1 + half, 2.5, 3 + half}).Len() > 1e-12 {
		t.Errorf("AABB %v", aabb)
	}
	body := NewRigidBody(transform, m, BodyTypeStatic, 0)
	if body.AABB() != aabb {
		t.Errorf("the body has the AABB %v", body.AABB())
	}
}
