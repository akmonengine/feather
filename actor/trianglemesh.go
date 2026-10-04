package actor

import (
	"errors"
	"math"
	"slices"

	"github.com/go-gl/mathgl/mgl64"
)

// ========== TRIANGLE MESH ==========
// A static decor of triangles (a rock, a staircase, a building), with a bounding volume hierarchy: a binary tree of
// AABBs, a few triangles at each leaf, built once, top-down, by the surface area heuristic over binned centroids
// (b3CreateMesh of Box3D, src/mesh.c; TriangleSplitterBinning & AABBTreeBuilder of Jolt; Ericson 6.2.1, "Top-down
// Construction"). A body or a query takes the triangles under its AABB from the tree (OverlapTriangles), a ray or a
// moving shape walks the tree front to back (Cast), as b3QueryMesh, b3RayCastMesh and b3ShapeCastMesh of Box3D.
//
// The mesh is a surface, as the heightfield: its triangles are seen from the side of their normal (the vertices
// counterclockwise seen from outside, the right-hand rule), a body whose center went through is not pushed back. The
// edges between its triangles are classified as the ones of the heightfield (active, convex): a body slides over the
// edges of a flat part without hitting them.

const (
	// MeshWeldDistance: 2 vertices closer than this distance (m) are the same vertex (the weld distance of Indexify
	// of Jolt, Geometry/Indexify.h:14). A mesh exported as a soup of triangles has its edges shared again: without it,
	// every edge would be a border, and active
	MeshWeldDistance = 1e-4

	// meshLeafTriangles: a leaf of the tree holds this many triangles at most, when a split is found
	// (B3_DESIRED_TRIANGLES_PER_LEAF of Box3D, src/mesh.c:24; 8 in Jolt, mMaxTrianglesPerLeaf)
	meshLeafTriangles = 4

	// meshLeafMaxTriangles: a leaf holds up to this many triangles when no split separates them (their centroids are
	// the same); above, they are split in halves (B3_MAXIMUM_TRIANGLES_PER_LEAF, src/mesh.c:26)
	meshLeafMaxTriangles = 8

	// meshBins of the surface area heuristic, per axis (B3_BIN_COUNT of Box3D, src/mesh.c:23)
	meshBins = 8

	// meshBalance: a split leaving less than a third of the triangles on a side is replaced by the median, so that the
	// height of the tree is bounded by log(triangles)/log(1.5) (the rule of btQuantizedBvh::sortAndCalcSplittingIndex
	// of Bullet: "if the splitIndex causes unbalanced trees, fix this by using the center")
	meshBalance = 3

	// meshStackSize: the nodes a traversal keeps for later. The height of the tree is bounded by meshBalance: 52 for
	// 2^31 triangles
	meshStackSize = 64

	// meshMinTriangleArea: a triangle of a smaller area (m²) is degenerate, out of the tree (the minArea of
	// b3CreateMesh of Box3D, src/mesh.c: 0.01 x B3_LINEAR_SLOP², with the LinearSlop of Feather, 5 mm)
	meshMinTriangleArea = 0.01 * 0.005 * 0.005

	// backToBackCos: 2 triangles whose normals are closer than 1° to opposite are back to back (a sheet): their edge
	// is active (cos(179°), ActiveEdges::IsEdgeActive of Jolt)
	backToBackCos = -0.999848
)

var (
	// ErrMeshIndices: the indices are not 3 per triangle, or out of the vertices
	ErrMeshIndices = errors.New("feather: a triangle mesh needs 3 indices per triangle, within its vertices")

	// ErrMeshDegenerate: no triangle of the mesh has an area
	ErrMeshDegenerate = errors.New("feather: a triangle mesh needs a triangle which is not degenerate")
)

// TriangleMesh is a static surface of triangles with a tree of AABBs, built by NewTriangleMesh. Hit.Triangle of a
// query on a mesh is the index of the triangle (Indices[3*i:3*i+3])
type TriangleMesh struct {
	// Vertices given to NewTriangleMesh, shared without copy
	Vertices []mgl64.Vec3
	// Indices of the vertices of each triangle, 3 per triangle, in the order given to NewTriangleMesh: the vertices
	// closer than MeshWeldDistance share one index
	Indices []int32

	// edges of each triangle: bit e if the edge from the vertex e to the vertex e+1 is active, bit 3+e if it is
	// convex (0 for a degenerate triangle)
	edges []uint8
	nodes []meshNode
	// order: the triangles of the leaves, the ones of a leaf from its first
	order  []int32
	bounds AABB
}

// meshNode: a node of the tree: its AABB and its 2 children (the left one follows it), or its triangles (a leaf)
type meshNode struct {
	min, max mgl64.Vec3
	right    int32 // index of the right child, -1 for a leaf
	first    int32 // leaf: index in order of its first triangle
	count    int32
}

// meshPrimitive: a triangle being sorted in the tree
type meshPrimitive struct {
	aabb   AABB
	center mgl64.Vec3
	index  int32
}

// NewTriangleMesh builds the mesh of the triangles (indices[3*i], indices[3*i+1], indices[3*i+2] are the vertices of the
// triangle i, counterclockwise seen from outside) and its tree. The vertices closer than MeshWeldDistance are welded,
// the degenerate triangles are kept out of the tree (Triangle gives them without edge). The vertices are shared, the
// indices are copied
func NewTriangleMesh(vertices []mgl64.Vec3, indices []int32) (*TriangleMesh, error) {
	if len(indices) == 0 || len(indices)%3 != 0 {
		return nil, ErrMeshIndices
	}
	for _, index := range indices {
		if index < 0 || int(index) >= len(vertices) {
			return nil, ErrMeshIndices
		}
	}
	m := &TriangleMesh{Vertices: vertices, Indices: weldIndices(vertices, indices)}
	count := len(indices) / 3
	m.edges = make([]uint8, count)

	primitives := make([]meshPrimitive, 0, count)
	for i := 0; i < count; i++ {
		triangle := m.vertices(int32(i))
		if triangle[1].Sub(triangle[0]).Cross(triangle[2].Sub(triangle[0])).Len() < 2*meshMinTriangleArea {
			continue
		}
		bounds := triangleBounds(triangle)
		primitives = append(primitives, meshPrimitive{aabb: bounds, center: bounds.Min.Add(bounds.Max).Mul(0.5), index: int32(i)})
	}
	if len(primitives) == 0 {
		return nil, ErrMeshDegenerate
	}
	m.findEdges(primitives)
	m.nodes = make([]meshNode, 0, 2*len(primitives))
	m.order = make([]int32, 0, len(primitives))
	m.buildNode(primitives)
	m.bounds = AABB{Min: m.nodes[0].min, Max: m.nodes[0].max}
	return m, nil
}

// weldIndices: the indices with the vertices closer than MeshWeldDistance merged on the first of them, found in a grid
// of cells of the weld distance (a vertex is compared to the vertices of the 27 cells around it)
func weldIndices(vertices []mgl64.Vec3, indices []int32) []int32 {
	type cell [3]int64
	cells := map[cell][]int32{}
	welded := make([]int32, len(vertices))
	for i := range welded {
		welded[i] = -1
	}
	cellOf := func(p mgl64.Vec3) cell {
		return cell{int64(math.Floor(p[0] / MeshWeldDistance)), int64(math.Floor(p[1] / MeshWeldDistance)), int64(math.Floor(p[2] / MeshWeldDistance))}
	}
	out := make([]int32, len(indices))
	for k, index := range indices {
		if welded[index] >= 0 {
			out[k] = welded[index]
			continue
		}
		p := vertices[index]
		c := cellOf(p)
		found := int32(-1)
		for dx := int64(-1); dx <= 1 && found < 0; dx++ {
			for dy := int64(-1); dy <= 1 && found < 0; dy++ {
				for dz := int64(-1); dz <= 1 && found < 0; dz++ {
					for _, other := range cells[cell{c[0] + dx, c[1] + dy, c[2] + dz}] {
						if vertices[other].Sub(p).LenSqr() <= MeshWeldDistance*MeshWeldDistance {
							found = other
							break
						}
					}
				}
			}
		}
		if found < 0 {
			found = index
			cells[c] = append(cells[c], index)
		}
		welded[index] = found
		out[k] = found
	}
	return out
}

// vertices of the triangle i
func (m *TriangleMesh) vertices(i int32) [3]mgl64.Vec3 {
	return [3]mgl64.Vec3{m.Vertices[m.Indices[3*i]], m.Vertices[m.Indices[3*i+1]], m.Vertices[m.Indices[3*i+2]]}
}

func triangleBounds(triangle [3]mgl64.Vec3) AABB {
	aabb := AABB{Min: triangle[0], Max: triangle[0]}
	for _, v := range triangle[1:] {
		for k := 0; k < 3; k++ {
			aabb.Min[k] = math.Min(aabb.Min[k], v[k])
			aabb.Max[k] = math.Max(aabb.Max[k], v[k])
		}
	}
	return aabb
}

// findEdges classifies the edges of the triangles in the tree, by the triangles sharing each edge (the edge map of
// sFindActiveEdges of Jolt, of b3IdentifyEdges of Box3D):
//   - 1 triangle (a border): convex and active
//   - 2 triangles: convex if the opposite vertex of the other triangle is under the plane, active if convex and bent
//     by more than 5° (the rule of the heightfield, cellEdges), or if the triangles are back to back (Jolt)
//   - 3 triangles or more: active (Jolt)
func (m *TriangleMesh) findEdges(primitives []meshPrimitive) {
	type side struct {
		triangle int32
		edge     uint8
	}
	type shared struct {
		count int
		sides [2]side
	}
	edges := make(map[[2]int32]*shared, 3*len(primitives))
	for _, primitive := range primitives {
		t := primitive.index
		for e := int32(0); e < 3; e++ {
			a, c := m.Indices[3*t+e], m.Indices[3*t+(e+1)%3]
			key := [2]int32{min(a, c), max(a, c)}
			s := edges[key]
			if s == nil {
				s = &shared{}
				edges[key] = s
			}
			if s.count < 2 {
				s.sides[s.count] = side{t, uint8(e)}
			} else {
				// a 3rd triangle on the edge: active for it, and for the 2 others below
				m.edges[t] |= (1 | 1<<3) << e
			}
			s.count++
		}
	}
	for _, s := range edges {
		if s.count != 2 {
			for _, side := range s.sides[:min(s.count, 2)] {
				m.edges[side.triangle] |= (1 | 1<<3) << side.edge
			}
			continue
		}
		for k, side := range s.sides {
			triangle, other := m.vertices(side.triangle), s.sides[1-k]
			normal := triangleNormal(triangle)
			otherTriangle := m.vertices(other.triangle)
			opposite := otherTriangle[(other.edge+2)%3]
			cos := normal.Dot(triangleNormal(otherTriangle))
			if cos < backToBackCos {
				m.edges[side.triangle] |= (1 | 1<<3) << side.edge
				continue
			}
			if opposite.Sub(triangle[side.edge]).Dot(normal) >= 0 {
				continue
			}
			m.edges[side.triangle] |= 1 << (3 + side.edge)
			if cos < activeEdgeCos {
				m.edges[side.triangle] |= 1 << side.edge
			}
		}
	}
}

// buildNode builds the node of the primitives and its subtree, and returns its index. The primitives are sorted in
// place: the ones of a leaf are contiguous in order
func (m *TriangleMesh) buildNode(primitives []meshPrimitive) int32 {
	index := int32(len(m.nodes))
	bounds := primitives[0].aabb
	for _, p := range primitives[1:] {
		for k := 0; k < 3; k++ {
			bounds.Min[k] = math.Min(bounds.Min[k], p.aabb.Min[k])
			bounds.Max[k] = math.Max(bounds.Max[k], p.aabb.Max[k])
		}
	}
	m.nodes = append(m.nodes, meshNode{min: bounds.Min, max: bounds.Max, right: -1})

	split := 0
	if len(primitives) > meshLeafTriangles {
		split = sahSplit(primitives)
		if split == 0 && len(primitives) > meshLeafMaxTriangles {
			// no split by the heuristic (the centroids are the same): halves (b3SplitHalf of Box3D)
			split = len(primitives) / 2
		}
	}
	if split == 0 {
		m.nodes[index].first, m.nodes[index].count = int32(len(m.order)), int32(len(primitives))
		for _, p := range primitives {
			m.order = append(m.order, p.index)
		}
		return index
	}
	m.buildNode(primitives[:split])
	m.nodes[index].right = m.buildNode(primitives[split:])
	return index
}

// sahSplit partitions the primitives in place by the surface area heuristic over binned centroids, and returns the
// count of the left part, 0 if no split separates them. On each axis the centroids fall in meshBins bins, and the cost
// of a split after a bin is the count of the triangles on each side times the area of their AABB (b3SplitBinnedSah of
// Box3D, src/mesh.c:570-681; TriangleSplitterBinning of Jolt). A split leaving less than a third of the triangles on
// a side is replaced by the median along its axis (meshBalance)
func sahSplit(primitives []meshPrimitive) int {
	centroids := AABB{Min: primitives[0].center, Max: primitives[0].center}
	for _, p := range primitives[1:] {
		for k := 0; k < 3; k++ {
			centroids.Min[k] = math.Min(centroids.Min[k], p.center[k])
			centroids.Max[k] = math.Max(centroids.Max[k], p.center[k])
		}
	}
	type bin struct {
		count  int
		bounds AABB
	}
	bestAxis, bestBin, bestCost := -1, -1, math.Inf(1)
	for axis := 0; axis < 3; axis++ {
		extent := centroids.Max[axis] - centroids.Min[axis]
		if extent <= 0 {
			continue
		}
		var bins [meshBins]bin
		factor := meshBins * (1 - epsilon64) / extent
		for _, p := range primitives {
			b := &bins[int(factor*(p.center[axis]-centroids.Min[axis]))]
			if b.count == 0 {
				b.bounds = p.aabb
			} else {
				b.bounds = unionAABB(b.bounds, p.aabb)
			}
			b.count++
		}
		for i := 0; i < meshBins-1; i++ {
			leftCount, rightCount := 0, 0
			var left, right AABB
			for k := 0; k <= i; k++ {
				if bins[k].count > 0 {
					if leftCount == 0 {
						left = bins[k].bounds
					} else {
						left = unionAABB(left, bins[k].bounds)
					}
					leftCount += bins[k].count
				}
			}
			for k := i + 1; k < meshBins; k++ {
				if bins[k].count > 0 {
					if rightCount == 0 {
						right = bins[k].bounds
					} else {
						right = unionAABB(right, bins[k].bounds)
					}
					rightCount += bins[k].count
				}
			}
			if leftCount == 0 || rightCount == 0 {
				continue
			}
			if cost := float64(leftCount)*area(left) + float64(rightCount)*area(right); cost < bestCost {
				bestAxis, bestBin, bestCost = axis, i, cost
			}
		}
	}
	if bestAxis < 0 {
		return 0
	}
	// the partition: the primitives of the bins up to bestBin first
	factor := meshBins * (1 - epsilon64) / (centroids.Max[bestAxis] - centroids.Min[bestAxis])
	split := 0
	for i := range primitives {
		if int(factor*(primitives[i].center[bestAxis]-centroids.Min[bestAxis])) <= bestBin {
			primitives[i], primitives[split] = primitives[split], primitives[i]
			split++
		}
	}
	if split < len(primitives)/meshBalance || len(primitives)-split < len(primitives)/meshBalance {
		slices.SortStableFunc(primitives, func(a, b meshPrimitive) int {
			switch {
			case a.center[bestAxis] < b.center[bestAxis]:
				return -1
			case a.center[bestAxis] > b.center[bestAxis]:
				return 1
			}
			return int(a.index - b.index)
		})
		split = len(primitives) / 2
	}
	return split
}

func unionAABB(a, b AABB) AABB {
	for k := 0; k < 3; k++ {
		a.Min[k] = math.Min(a.Min[k], b.Min[k])
		a.Max[k] = math.Max(a.Max[k], b.Max[k])
	}
	return a
}

// area: the surface area of the AABB
func area(a AABB) float64 {
	d := a.Max.Sub(a.Min)
	return 2 * (d[0]*d[1] + d[1]*d[2] + d[2]*d[0])
}

// TriangleCount: the triangles of the mesh, degenerate or not
func (m *TriangleMesh) TriangleCount() int {
	return len(m.Indices) / 3
}

// Triangle i, in the local space, with its edges: bit e if the edge from the vertex e to the vertex e+1 is active, bit
// 3+e if it is convex. A degenerate triangle has no edge
func (m *TriangleMesh) Triangle(i int32) ([3]mgl64.Vec3, uint8) {
	return m.vertices(i), m.edges[i]
}

// Bounds of the mesh, in its local space
func (m *TriangleMesh) Bounds() AABB {
	return m.bounds
}

// Height of the tree (0 for a single leaf)
func (m *TriangleMesh) Height() int {
	return m.height(0)
}

func (m *TriangleMesh) height(node int32) int {
	if m.nodes[node].right < 0 {
		return 0
	}
	return 1 + max(m.height(node+1), m.height(m.nodes[node].right))
}

// OverlapTriangles appends to triangles the indices of the triangles whose AABB overlaps the local bounds, in the order
// of the tree, and returns the slice: no allocation if its capacity is enough
func (m *TriangleMesh) OverlapTriangles(bounds AABB, triangles []int32) []int32 {
	return m.overlapNode(0, bounds, triangles)
}

func (m *TriangleMesh) overlapNode(index int32, bounds AABB, out []int32) []int32 {
	node := &m.nodes[index]
	if !bounds.Overlaps(AABB{Min: node.min, Max: node.max}) {
		return out
	}
	if node.right < 0 {
		for _, t := range m.order[node.first : node.first+node.count] {
			if triangleBounds(m.vertices(t)).Overlaps(bounds) {
				out = append(out, t)
			}
		}
		return out
	}
	out = m.overlapNode(index+1, bounds, out)
	return m.overlapNode(node.right, bounds, out)
}

// ========== CAST ==========

// castNode: a node kept for later by a cast, with the fraction where the segment enters it
type castNode struct {
	node  int32
	enter float64
}

// MeshCast: the triangles a moving box meets, from TriangleMesh.Cast: the nodes of the tree in the order the segment
// of the center enters them (the closest child first, the nodes entered after the limit dropped), the triangles of
// each leaf in their order
type MeshCast struct {
	mesh                *TriangleMesh
	origin, translation mgl64.Vec3
	// inverse of the translation, finite: 0 x infinity would be a NaN for a segment starting in the plane of a face
	inverse mgl64.Vec3
	extents mgl64.Vec3
	stack   [meshStackSize]castNode
	count   int
	// the triangles left in the leaf being walked
	next, end int32
}

// Cast: the triangles a box of half sizes extents can meet while its center moves from origin by maxFraction *
// translation, in the local space of the mesh, in the order of the motion (a ray is a box without size). The AABBs of
// the nodes are enlarged by the extents (b3ShapeCastMesh of Box3D)
func (m *TriangleMesh) Cast(origin, translation, extents mgl64.Vec3, maxFraction float64) MeshCast {
	var c MeshCast
	c.start(m, origin, translation, extents, maxFraction)
	return c
}

func (c *MeshCast) start(m *TriangleMesh, origin, translation, extents mgl64.Vec3, maxFraction float64) {
	*c = MeshCast{mesh: m, origin: origin, translation: translation, extents: extents}
	for k := 0; k < 3; k++ {
		if translation[k] != 0 {
			c.inverse[k] = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, 1/translation[k]))
		}
	}
	if enter, ok := c.enters(&m.nodes[0], maxFraction); ok {
		c.stack[0] = castNode{0, enter}
		c.count = 1
	}
}

// enters: the fraction where the segment, cut at limit, enters the AABB of the node enlarged by the extents, false if
// it doesn't. The faces belong to the AABB
func (c *MeshCast) enters(node *meshNode, limit float64) (float64, bool) {
	enter, exit := 0.0, limit
	for k := 0; k < 3; k++ {
		low, high := node.min[k]-c.extents[k]-c.origin[k], node.max[k]+c.extents[k]-c.origin[k]
		if c.translation[k] == 0 {
			if low > 0 || high < 0 {
				return 0, false
			}
			continue
		}
		near, far := low*c.inverse[k], high*c.inverse[k]
		if near > far {
			near, far = far, near
		}
		enter, exit = math.Max(enter, near), math.Min(exit, far)
	}
	return enter, enter <= exit
}

// Next triangle of the cast, false after the last one. The nodes entered after the limit are dropped: the caller
// lowers the limit to its best hit. A node entered at the limit is walked: a triangle of a lower index can be hit at
// the same fraction
func (c *MeshCast) Next(limit float64) (int32, bool) {
	m := c.mesh
	for {
		if c.next < c.end {
			t := m.order[c.next]
			c.next++
			return t, true
		}
		if c.count == 0 {
			return 0, false
		}
		c.count--
		entry := c.stack[c.count]
		if entry.enter > limit {
			continue
		}
		node := &m.nodes[entry.node]
		if node.right < 0 {
			c.next, c.end = node.first, node.first+node.count
			continue
		}
		near, far := castNode{node: entry.node + 1}, castNode{node: node.right}
		var nearOk, farOk bool
		near.enter, nearOk = c.enters(&m.nodes[near.node], limit)
		far.enter, farOk = c.enters(&m.nodes[far.node], limit)
		if farOk && (!nearOk || far.enter < near.enter) {
			near, far, nearOk, farOk = far, near, farOk, nearOk
		}
		// the further child under the closer one (the node just left the stack: 2 children fit, the height of the
		// tree is bounded by meshBalance)
		if farOk {
			c.stack[c.count] = far
			c.count++
		}
		if nearOk {
			c.stack[c.count] = near
			c.count++
		}
	}
}

// ========== SHAPE ==========

// ComputeAABB: the bounds of the mesh turned and moved: the AABB of their 8 corners
func (m *TriangleMesh) ComputeAABB(transform Transform) AABB {
	q := &transform.Rotation
	min := mgl64.Vec3{math.Inf(1), math.Inf(1), math.Inf(1)}
	max := mgl64.Vec3{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for i := 0; i < 8; i++ {
		corner := m.bounds.Min
		if i&1 != 0 {
			corner[0] = m.bounds.Max[0]
		}
		if i&2 != 0 {
			corner[1] = m.bounds.Max[1]
		}
		if i&4 != 0 {
			corner[2] = m.bounds.Max[2]
		}
		world := Rotate(q, corner).Add(transform.Position)
		for k := 0; k < 3; k++ {
			min[k] = math.Min(min[k], world[k])
			max[k] = math.Max(max[k], world[k])
		}
	}
	return AABB{Min: min, Max: max}
}

// ComputeMass: a mesh is static
func (m *TriangleMesh) ComputeMass(density float64) float64 {
	return math.Inf(1)
}

func (m *TriangleMesh) ComputeInertia(mass float64) mgl64.Mat3 {
	return mgl64.Mat3{}
}

// Support: a mesh is not convex, its triangles are tested one by one
func (m *TriangleMesh) Support(direction mgl64.Vec3) mgl64.Vec3 {
	return mgl64.Vec3{}
}

func (m *TriangleMesh) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	output[0] = mgl64.Vec3{}
	*count = 1
}

// CollideWithPlane - Mesh/Plane collision (not supported)
func (m *TriangleMesh) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact {
	return contacts
}

// CastRay: the ray walks the tree front to back (Cast) and meets each triangle by Möller-Trumbore (the ray cast of the
// meshes of Jolt, RayTriangle.h, and of Box3D, b3RayCastMesh). Only the side of the normal is hit: a ray from behind
// goes through. A point may be footprintSlack out of its triangle (in barycentric coordinates): a ray on an edge hits
// the triangles on both sides, and never leaks between them. At the very same fraction, the triangle of the lowest
// index is named. A ray without length hits nothing: a mesh has no inside
func (m *TriangleMesh) CastRay(origin, translation mgl64.Vec3, maxFraction float64) (RayHit, bool) {
	if !finiteRay(origin, translation) {
		return RayHit{}, false
	}
	hit, found := RayHit{Fraction: maxFraction, Triangle: NoTriangle}, false
	var normal mgl64.Vec3
	var cast MeshCast
	cast.start(m, origin, translation, mgl64.Vec3{}, maxFraction)
	for {
		t, ok := cast.Next(hit.Fraction)
		if !ok {
			break
		}
		triangle := m.vertices(t)
		edge1, edge2 := triangle[1].Sub(triangle[0]), triangle[2].Sub(triangle[0])
		p := translation.Cross(edge2)
		// the determinant is positive for a ray coming from the side of the normal
		determinant := edge1.Dot(p)
		if determinant <= 0 {
			continue
		}
		s := origin.Sub(triangle[0])
		u := s.Dot(p) / determinant
		if u < -footprintSlack || u > 1+footprintSlack {
			continue
		}
		q := s.Cross(edge1)
		v := translation.Dot(q) / determinant
		if v < -footprintSlack || u+v > 1+footprintSlack {
			continue
		}
		fraction := edge2.Dot(q) / determinant
		if fraction < 0 || fraction > hit.Fraction || (found && fraction == hit.Fraction && t > hit.Triangle) {
			continue
		}
		hit.Fraction, hit.Triangle, found = fraction, t, true
		normal = edge1.Cross(edge2)
	}
	if found {
		hit.Normal = normal.Normalize()
	}
	return hit, found
}
