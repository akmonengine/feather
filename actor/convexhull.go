package actor

import (
	"errors"
	"math"
	"slices"

	"github.com/go-gl/mathgl/mgl64"
)

// ========== CONVEX HULL ==========
// The convex hull of a cloud of points (the vertices of a mesh), built by quickhull (Barber, Dobkin & Huhdanpaa, "The
// Quickhull Algorithm for Convex Hulls", 1996; Dirk Gregorius, "Implementing QuickHull", GDC 2014): a tetrahedron of
// the points the furthest apart, then the point the furthest out of the hull is added at each step, the faces it sees
// are replaced by a cone of triangles from the horizon to this point, and the points out of the removed faces are given
// to the new faces. The ConvexHullBuilder of Jolt and b3CreateHull of Box3D (src/hull.c) build their hulls this way.
// Once every point is in the hull, the triangles coplanar within the tolerance are merged in polygons (the faces),
// each with its plane: the face of a cube is 1 polygon of 4 vertices, not 2 triangles, and the manifold of a cube
// resting on the ground has its 4 corners. Jolt and Box3D merge the faces during the construction (MergeCoplanarOrConcaveFaces,
// b3HullBuilder_MergeFaces); Feather merges once at the end, on a complete triangulation.
//
// The hull is a convex polytope: GJK/EPA takes it by its support point (the vertex the furthest along a direction,
// found among all the vertices, as HullNoConvex::GetSupport of Jolt and b3FindHullSupportVertex of Box3D), the contact
// clipping by its faces. Its mass, its center of mass and its inertia come from the integral of its volume: the sum of
// the tetrahedra between its faces and a point inside (ConvexHullShape of Jolt, after Blow & Binstock, "How to find the
// inertia tensor of a 3D solid body represented by a triangle mesh"). The points are then translated so that the center
// of mass is at the origin of the local space of the hull, where a body has its center of mass (Jolt does the same:
// its shapes are centered on their center of mass).

const (
	// MaxHullVertices: a hull has at most this many vertices (256 in Jolt, cMaxPointsInHull, and in Box3D,
	// B3_HULL_MAX_COUNT; 255 in PhysX, PxConvexMeshDesc::vertexLimit). Its support point is found among all of them
	MaxHullVertices = 256

	// hullToleranceFactor: the points within tolerance of a plane are on it. The tolerance is the rounding error of
	// the distance of a point to a plane, from the size of the cloud: 3 x (|x| + |y| + |z|)max x epsilon (Gregorius,
	// "Implementing QuickHull"; DetermineCoplanarDistance of Jolt, b3HullBuilder_ComputeTolerance of Box3D, in
	// single precision; here in double)
	hullToleranceFactor = 3

	// hullMinOutsideFactor: a point is out of the hull if it is further than this many tolerances in front of a face
	// (minOutside of Box3D: 8 tolerances, src/hull.c:644-645; points closer are on the hull, within its tolerance)
	hullMinOutsideFactor = 8

	// hullMinRadiusFactor: a face is seen by a point if the point is more than this many tolerances in front of its
	// plane (minRadius of Box3D: 4 tolerances, src/hull.c:644, :868)
	hullMinRadiusFactor = 4

	// hullFeatureVertices: a face returned by GetContactFeature has at most this many vertices, the size of the
	// buffer of the contact clipping. A face with more vertices gives one vertex out of k (GetSupportingFace of Jolt
	// skips vertices the same way)
	hullFeatureVertices = 8
)

var (
	// ErrHullDegenerate: the points are fewer than 4, or on a plane: no hull
	ErrHullDegenerate = errors.New("feather: a convex hull needs 4 points not on a plane")

	// ErrHullVertexLimit: maxVertices is not in [4, MaxHullVertices]
	ErrHullVertexLimit = errors.New("feather: the vertex limit of a convex hull is in [4, MaxHullVertices]")

	// ErrHullFailed: the construction lost the shape of the hull (the horizon of a point is not a loop): the points
	// are too close to coplanar for the tolerance
	ErrHullFailed = errors.New("feather: the convex hull could not be built from these points")
)

// ConvexHull is a convex polytope: the convex hull of a cloud of points, built by NewConvexHull. In its local space
// the center of mass is at the origin: a body carrying the hull is at the center of mass of the cloud (CenterOfMass).
// A hull can be dynamic
type ConvexHull struct {
	// Points: the vertices of the hull, in its local space (the center of mass at the origin)
	Points []mgl64.Vec3

	// faces: the polygons, their vertices in faceVertices (indices in Points, counterclockwise seen from outside)
	faces        []hullFace
	faceVertices []int32
	// centerOfMass of the hull in the space of the points given to NewConvexHull
	centerOfMass mgl64.Vec3
	volume       float64
	// inertia for a density of 1, about the center of mass, in the local space
	inertia mgl64.Mat3
	bounds  AABB
}

// hullFace: a polygon of the hull and its plane (Normal·p + Distance = 0, the normal out of the hull)
type hullFace struct {
	normal   mgl64.Vec3
	distance float64
	first    int32
	count    int32
}

// NewConvexHull builds the convex hull of the points, with maxVertices vertices at most (in [4, MaxHullVertices]).
// The points inside the hull, or on its surface within the rounding, are not vertices. If the limit is reached, the
// hull stops growing: the points left out are the closest to the hull, since the furthest point is added first
// (Jolt and Box3D stop the same way, PhysX expands the hull by its planes instead). The points are read, not kept
func NewConvexHull(points []mgl64.Vec3, maxVertices int) (*ConvexHull, error) {
	if maxVertices < 4 || maxVertices > MaxHullVertices {
		return nil, ErrHullVertexLimit
	}
	if len(points) < 4 {
		return nil, ErrHullDegenerate
	}
	var b hullBuilder
	if err := b.build(points, maxVertices); err != nil {
		return nil, err
	}
	return b.finish()
}

// CenterOfMass of the hull in the space of the points given to NewConvexHull. The local space of the hull is centered
// on it: a body carrying the hull of a mesh is at transform.ToWorld(CenterOfMass()) of the mesh
func (h *ConvexHull) CenterOfMass() mgl64.Vec3 {
	return h.centerOfMass
}

// Volume of the hull (m³)
func (h *ConvexHull) Volume() float64 {
	return h.volume
}

// FaceCount: the number of faces (polygons) of the hull
func (h *ConvexHull) FaceCount() int {
	return len(h.faces)
}

// Face i: its plane (Normal·p + Distance = 0, the normal out of the hull) and the indices of its vertices in Points,
// counterclockwise seen from outside. The slice is read-only
func (h *ConvexHull) Face(i int) (normal mgl64.Vec3, distance float64, vertices []int32) {
	face := &h.faces[i]
	return face.normal, face.distance, h.faceVertices[face.first : face.first+face.count]
}

// ComputeAABB: the local bounds of the hull turned and moved: the AABB of their 8 corners, as b3ComputeHullAABB of
// Box3D (b3AABB_Transform of the local AABB) and the local bounds of ConvexHullShape of Jolt, not the AABB of the
// vertices (tighter, but a rotation of every vertex at each step)
func (h *ConvexHull) ComputeAABB(transform Transform) AABB {
	q := &transform.Rotation
	min := mgl64.Vec3{math.Inf(1), math.Inf(1), math.Inf(1)}
	max := mgl64.Vec3{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for i := 0; i < 8; i++ {
		corner := h.bounds.Min
		if i&1 != 0 {
			corner[0] = h.bounds.Max[0]
		}
		if i&2 != 0 {
			corner[1] = h.bounds.Max[1]
		}
		if i&4 != 0 {
			corner[2] = h.bounds.Max[2]
		}
		world := Rotate(q, corner).Add(transform.Position)
		for k := 0; k < 3; k++ {
			min[k] = math.Min(min[k], world[k])
			max[k] = math.Max(max[k], world[k])
		}
	}
	return AABB{Min: min, Max: max}
}

// ComputeMass: the density times the volume of the hull
func (h *ConvexHull) ComputeMass(density float64) float64 {
	return density * h.volume
}

// ComputeInertia about the center of mass, in the local space: the inertia of the volume, scaled to the mass
func (h *ConvexHull) ComputeInertia(mass float64) mgl64.Mat3 {
	if h.volume <= 0 {
		return mgl64.Mat3{}
	}
	scale := mass / h.volume
	var inertia mgl64.Mat3
	for i := range inertia {
		inertia[i] = h.inertia[i] * scale
	}
	return inertia
}

// Support: the vertex the furthest along the direction, among all of them (HullNoConvex::GetSupport of Jolt,
// b3FindHullSupportVertex of Box3D). At the same distance, the first one
func (h *ConvexHull) Support(direction mgl64.Vec3) mgl64.Vec3 {
	best, bestDot := 0, math.Inf(-1)
	for i, p := range h.Points {
		if dot := p.Dot(direction); dot > bestDot {
			best, bestDot = i, dot
		}
	}
	return h.Points[best]
}

// supportingFace: the face whose normal is the most aligned with the direction (GetSupportingFace of Jolt,
// b3FindHullSupportFace of Box3D)
func (h *ConvexHull) supportingFace(direction mgl64.Vec3) int {
	best, bestDot := 0, math.Inf(-1)
	for i := range h.faces {
		if dot := h.faces[i].normal.Dot(direction); dot > bestDot {
			best, bestDot = i, dot
		}
	}
	return best
}

// GetContactFeature: the vertices of the face the most aligned with the direction, hullFeatureVertices at most: a face
// with more vertices gives one out of k, around the polygon (GetSupportingFace of Jolt)
func (h *ConvexHull) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	face := &h.faces[h.supportingFace(direction)]
	vertices := h.faceVertices[face.first : face.first+face.count]
	step := (len(vertices) + hullFeatureVertices - 1) / hullFeatureVertices
	*count = 0
	for i := 0; i < len(vertices) && *count < hullFeatureVertices; i += step {
		output[*count] = h.Points[vertices[i]]
		*count++
	}
}

// CollideWithPlane returns the vertices of the supporting face of the hull (the face the most opposed to the normal of
// the plane) closer to the plane than the margin, as the Box does
func (h *ConvexHull) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact {
	var face [8]mgl64.Vec3
	var count int
	q := &myTransform.Rotation
	h.GetContactFeature(RotateInverse(q, planeNormal.Mul(-1)), &face, &count)
	for _, vertex := range face[:count] {
		worldVertex := myTransform.Position.Add(Rotate(q, vertex))
		separation := worldVertex.Dot(planeNormal) + planeDistance
		if separation > margin {
			continue
		}
		contacts = append(contacts, ContactPoint{
			Position:   worldVertex.Sub(planeNormal.Mul(separation / 2)),
			Separation: separation,
		})
	}
	return contacts
}

// CastRay against the planes of the faces: the ray is in the hull between the last plane it enters and the first one
// it leaves (b3RayCastHull of Box3D, CastRayHelper of ConvexHullShape of Jolt). The normal is the one of the face it
// enters by
func (h *ConvexHull) CastRay(origin, translation mgl64.Vec3, maxFraction float64) (RayHit, bool) {
	if !finiteRay(origin, translation) {
		return RayHit{}, false
	}
	enter, exit, face := 0.0, maxFraction, -1
	inside := true
	for i := range h.faces {
		plane := &h.faces[i]
		distance := origin.Dot(plane.normal) + plane.distance
		if distance > 0 {
			inside = false
		}
		descent := translation.Dot(plane.normal)
		if descent == 0 {
			// along the plane: in front of it all the way, or never
			if distance > 0 {
				return RayHit{}, false
			}
			continue
		}
		fraction := -distance / descent
		if descent < 0 {
			// the ray enters this plane
			if fraction > enter {
				enter, face = fraction, i
			}
		} else if fraction < exit {
			exit = fraction
		}
		if enter > exit {
			return RayHit{}, false
		}
	}
	if inside {
		return startInside(translation), true
	}
	if face < 0 {
		return RayHit{}, false
	}
	return RayHit{Fraction: enter, Normal: h.faces[face].normal, Triangle: NoTriangle}, true
}

// ========== QUICKHULL ==========

// qhFace: a triangle of the hull being built
type qhFace struct {
	vertices [3]int32
	// neighbors[e]: the face across the edge from vertices[e] to vertices[e+1]
	neighbors [3]int32
	normal    mgl64.Vec3 // unit
	distance  float64    // normal·p + distance = 0
	centroid  mgl64.Vec3
	// conflicts: the points in front of the face, further than minOutside, the furthest last
	conflicts []int32
	furthest  float64
	alive     bool
	// visited stamp of the horizon search
	visited uint32
}

// hullBuilder: the state of quickhull
type hullBuilder struct {
	points []mgl64.Vec3
	// origin: the center of the cloud, taken out of the points for the precision
	origin     mgl64.Vec3
	tolerance  float64
	minRadius  float64
	minOutside float64
	faces      []qhFace
	// vertices in the hull
	count int
	stamp uint32
	// buffers of a step
	visible []int32
	horizon []horizonEdge
	cone    []int32
}

// horizonEdge: an edge of a visible face whose other face is not visible: from a to b, seen from the visible face
type horizonEdge struct {
	a, b    int32
	outside int32 // the face across, not visible
	edge    int32 // the edge of outside from b to a
}

// build the triangulated hull
func (b *hullBuilder) build(input []mgl64.Vec3, maxVertices int) error {
	// the points around their center: the tolerance follows the size of the cloud, not its distance to the origin
	bounds := AABB{Min: input[0], Max: input[0]}
	for _, p := range input[1:] {
		for k := 0; k < 3; k++ {
			bounds.Min[k] = math.Min(bounds.Min[k], p[k])
			bounds.Max[k] = math.Max(bounds.Max[k], p[k])
		}
	}
	b.origin = bounds.Min.Add(bounds.Max).Mul(0.5)
	b.points = make([]mgl64.Vec3, len(input))
	largest := mgl64.Vec3{}
	for i, p := range input {
		b.points[i] = p.Sub(b.origin)
		for k := 0; k < 3; k++ {
			largest[k] = math.Max(largest[k], math.Abs(b.points[i][k]))
		}
	}
	b.tolerance = hullToleranceFactor * (largest[0] + largest[1] + largest[2]) * epsilon64
	b.minRadius = hullMinRadiusFactor * b.tolerance
	b.minOutside = hullMinOutsideFactor * b.tolerance

	if !b.initialTetrahedron() {
		return ErrHullDegenerate
	}
	for i := range b.points {
		b.assign(int32(i), b.faces)
	}

	for b.count < maxVertices {
		face := b.furthestFace()
		if face < 0 {
			break
		}
		apex := b.faces[face].conflicts[len(b.faces[face].conflicts)-1]
		b.faces[face].conflicts = b.faces[face].conflicts[:len(b.faces[face].conflicts)-1]
		if !b.addPoint(int32(face), apex) {
			return ErrHullFailed
		}
	}
	return nil
}

// epsilon64: the relative rounding error of a float64 (DBL_EPSILON)
const epsilon64 = 0x1p-52

// initialTetrahedron: the 2 points the furthest apart along the axis of the largest extent, the point the furthest
// from their line, the point the furthest from their plane (b3HullBuilder_BuildInitialHull of Box3D, the initial
// simplex of PhysX, QuickHull::findSimplex). False if the cloud is flat, within the tolerance
func (b *hullBuilder) initialTetrahedron() bool {
	var low, high [3]int
	for i, p := range b.points {
		for k := 0; k < 3; k++ {
			if p[k] < b.points[low[k]][k] {
				low[k] = i
			}
			if p[k] > b.points[high[k]][k] {
				high[k] = i
			}
		}
	}
	axis := 0
	for k := 1; k < 3; k++ {
		if b.points[high[k]][k]-b.points[low[k]][k] > b.points[high[axis]][axis]-b.points[low[axis]][axis] {
			axis = k
		}
	}
	i1, i2 := low[axis], high[axis]
	if b.points[i2][axis]-b.points[i1][axis] <= 2*b.tolerance {
		return false
	}

	// the furthest from the line
	a, line := b.points[i1], b.points[i2].Sub(b.points[i1])
	i3, best := -1, 4*b.tolerance*b.tolerance*line.LenSqr()
	for i, p := range b.points {
		if i == i1 || i == i2 {
			continue
		}
		if cross := p.Sub(a).Cross(line).LenSqr(); cross > best {
			i3, best = i, cross
		}
	}
	if i3 < 0 {
		return false
	}

	// the furthest from the plane
	normal := line.Cross(b.points[i3].Sub(a)).Normalize()
	i4, bestDistance := -1, 2*b.tolerance
	for i, p := range b.points {
		if i == i1 || i == i2 || i == i3 {
			continue
		}
		if distance := math.Abs(p.Sub(a).Dot(normal)); distance > bestDistance {
			i4, bestDistance = i, distance
		}
	}
	if i4 < 0 {
		return false
	}
	// the 4th point behind the face (i1, i2, i3): its normal points out
	if b.points[i4].Sub(a).Dot(normal) > 0 {
		i2, i3 = i3, i2
	}
	v1, v2, v3, v4 := int32(i1), int32(i2), int32(i3), int32(i4)
	b.faces = append(b.faces,
		b.triangle(v1, v2, v3, [3]int32{1, 2, 3}),
		b.triangle(v2, v1, v4, [3]int32{0, 3, 2}),
		b.triangle(v3, v2, v4, [3]int32{0, 1, 3}),
		b.triangle(v1, v3, v4, [3]int32{0, 2, 1}),
	)
	b.count = 4
	return true
}

// triangle: a face from 3 vertices, counterclockwise seen from outside, with its neighbors across its 3 edges
func (b *hullBuilder) triangle(v0, v1, v2 int32, neighbors [3]int32) qhFace {
	p0, p1, p2 := b.points[v0], b.points[v1], b.points[v2]
	normal := p1.Sub(p0).Cross(p2.Sub(p0)).Normalize()
	centroid := p0.Add(p1).Add(p2).Mul(1.0 / 3)
	return qhFace{vertices: [3]int32{v0, v1, v2}, neighbors: neighbors, normal: normal, distance: -normal.Dot(centroid), centroid: centroid, alive: true}
}

// separation of the point from the plane of the face, positive in front
func (f *qhFace) separation(p mgl64.Vec3) float64 {
	return p.Dot(f.normal) + f.distance
}

// assign the point to the face it is the furthest in front of, among faces, if it is out of the hull (further than
// minOutside): the conflict lists of Barber et al. (the outside sets), kept with the furthest point last
func (b *hullBuilder) assign(point int32, faces []qhFace) {
	p := b.points[point]
	best, bestSeparation := -1, b.minOutside
	for i := range faces {
		if !faces[i].alive {
			continue
		}
		if separation := faces[i].separation(p); separation > bestSeparation {
			best, bestSeparation = i, separation
		}
	}
	if best < 0 {
		return
	}
	face := &faces[best]
	if bestSeparation > face.furthest {
		face.furthest = bestSeparation
		face.conflicts = append(face.conflicts, point)
		return
	}
	// not the furthest: before the last one
	face.conflicts = append(face.conflicts, 0)
	copy(face.conflicts[len(face.conflicts)-1:], face.conflicts[len(face.conflicts)-2:])
	face.conflicts[len(face.conflicts)-2] = point
}

// furthestFace: the face with the furthest point in front of it, -1 if no point is out of the hull
func (b *hullBuilder) furthestFace() int {
	best, furthest := -1, 0.0
	for i := range b.faces {
		face := &b.faces[i]
		if face.alive && len(face.conflicts) > 0 && face.furthest > furthest {
			best, furthest = i, face.furthest
		}
	}
	return best
}

// addPoint adds the apex to the hull: the faces which see it (from the face it is in front of, through their
// neighbors) are removed, a cone of triangles is built from their horizon to the apex, and the points of the removed
// faces are given to the cone. False if the horizon is not a loop: the hull is lost
func (b *hullBuilder) addPoint(seed, apex int32) bool {
	p := b.points[apex]
	b.stamp++
	b.visible, b.horizon = b.visible[:0], b.horizon[:0]
	b.faces[seed].visited = b.stamp
	b.visible = append(b.visible, seed)
	for i := 0; i < len(b.visible); i++ {
		face := &b.faces[b.visible[i]]
		for e := 0; e < 3; e++ {
			other := face.neighbors[e]
			neighbor := &b.faces[other]
			if neighbor.visited == b.stamp {
				continue
			}
			if neighbor.separation(p) > b.minRadius {
				neighbor.visited = b.stamp
				b.visible = append(b.visible, other)
				continue
			}
			// the edge of the neighbor from vertices[e+1] to vertices[e]
			a, c := face.vertices[e], face.vertices[(e+1)%3]
			edge := int32(-1)
			for k := 0; k < 3; k++ {
				if neighbor.vertices[k] == c && neighbor.vertices[(k+1)%3] == a {
					edge = int32(k)
				}
			}
			if edge < 0 {
				return false
			}
			b.horizon = append(b.horizon, horizonEdge{a: a, b: c, outside: other, edge: edge})
		}
	}
	// a face which is not visible is tested from each visible face beside it, and gives one horizon edge each time

	// the horizon as a loop: each vertex starts one edge
	b.cone = b.cone[:0]
	for range b.horizon {
		b.cone = append(b.cone, -1)
	}
	for i := range b.horizon {
		h := &b.horizon[i]
		for j := range b.horizon {
			if b.horizon[j].a == h.b {
				if b.cone[i] >= 0 {
					return false
				}
				b.cone[i] = int32(j)
			}
		}
		if b.cone[i] < 0 {
			return false
		}
	}

	// the cone: a triangle per horizon edge, in the slots of the visible faces, then appended
	first := len(b.faces)
	for range b.horizon {
		b.faces = append(b.faces, qhFace{})
	}
	for i, h := range b.horizon {
		index := int32(first + i)
		face := b.triangle(h.a, h.b, apex, [3]int32{h.outside, int32(first) + b.cone[i], 0})
		b.faces[index] = face
		b.faces[h.outside].neighbors[h.edge] = index
	}
	for i := range b.horizon {
		// the previous edge of the loop ends at this one: across the edge apex -> a
		next := int32(first) + b.cone[i]
		b.faces[next].neighbors[2] = int32(first + i)
	}
	for i := range b.horizon {
		if normal := b.faces[first+i].normal; math.IsNaN(normal[0]) || normal == (mgl64.Vec3{}) {
			return false
		}
	}

	// the points of the removed faces go to the cone
	cone := b.faces[first:]
	for _, index := range b.visible {
		face := &b.faces[index]
		face.alive = false
		for _, point := range face.conflicts {
			if point != apex {
				b.assign(point, cone)
			}
		}
		face.conflicts = nil
	}
	b.count++
	return true
}

// ========== THE FACES ==========

// finish merges the coplanar triangles in polygons, keeps the vertices used, computes the mass properties and
// centers the hull on its center of mass
func (b *hullBuilder) finish() (*ConvexHull, error) {
	// the alive faces, by index; group[i]: the group of the face i (union-find)
	group := make([]int32, len(b.faces))
	for i := range group {
		group[i] = int32(i)
	}
	find := func(i int32) int32 {
		for group[i] != i {
			group[i] = group[group[i]]
			i = group[i]
		}
		return i
	}
	// 2 faces across an edge are merged when the edge is not convex: the centroid of one is not behind the plane of
	// the other by more than the tolerance (coplanar, or bent inwards by the rounding: b3IsEdgeConvex of Box3D, both
	// ways as MergeCoplanarOrConcaveFaces of Jolt)
	for i := range b.faces {
		face := &b.faces[i]
		if !face.alive {
			continue
		}
		for e := 0; e < 3; e++ {
			other := &b.faces[face.neighbors[e]]
			if face.separation(other.centroid) >= -b.tolerance || other.separation(face.centroid) >= -b.tolerance {
				a, c := find(int32(i)), find(face.neighbors[e])
				if a != c {
					group[a] = c
				}
			}
		}
	}

	// the polygon of each group: its boundary edges, chained
	h := &ConvexHull{}
	used := make([]int32, len(b.points))
	for i := range used {
		used[i] = -1
	}
	type boundary struct{ a, c int32 }
	var edges []boundary
	var loop []int32
	for i := range b.faces {
		if !b.faces[i].alive || find(int32(i)) != int32(i) {
			continue
		}
		edges = edges[:0]
		for j := range b.faces {
			face := &b.faces[j]
			if !face.alive || find(int32(j)) != int32(i) {
				continue
			}
			for e := 0; e < 3; e++ {
				if find(face.neighbors[e]) != int32(i) {
					edges = append(edges, boundary{face.vertices[e], face.vertices[(e+1)%3]})
				}
			}
		}
		if len(edges) < 3 {
			return nil, ErrHullFailed
		}
		loop = loop[:0]
		loop = append(loop, edges[0].a)
		for next := edges[0].c; next != edges[0].a; {
			found := false
			for _, edge := range edges {
				if edge.a == next {
					loop = append(loop, next)
					next, found = edge.c, true
					break
				}
			}
			if !found || len(loop) > len(edges) {
				return nil, ErrHullFailed
			}
		}
		if len(loop) != len(edges) {
			return nil, ErrHullFailed
		}
		loop = b.dropCollinear(loop)
		if len(loop) < 3 {
			return nil, ErrHullFailed
		}
		// the plane of the polygon: Newell's normal, through its centroid
		normal, centroid := mgl64.Vec3{}, mgl64.Vec3{}
		for k, v := range loop {
			p, q := b.points[v], b.points[loop[(k+1)%len(loop)]]
			normal = normal.Add(mgl64.Vec3{(p[1] - q[1]) * (p[2] + q[2]), (p[2] - q[2]) * (p[0] + q[0]), (p[0] - q[0]) * (p[1] + q[1])})
			centroid = centroid.Add(p)
		}
		if normal.Len() == 0 {
			return nil, ErrHullFailed
		}
		normal = normal.Normalize()
		centroid = centroid.Mul(1 / float64(len(loop)))
		face := hullFace{normal: normal, distance: -normal.Dot(centroid), first: int32(len(h.faceVertices)), count: int32(len(loop))}
		for _, v := range loop {
			if used[v] < 0 {
				used[v] = int32(len(h.Points))
				h.Points = append(h.Points, b.points[v])
			}
			h.faceVertices = append(h.faceVertices, used[v])
		}
		h.faces = append(h.faces, face)
	}
	if len(h.faces) < 4 || len(h.Points) < 4 {
		return nil, ErrHullFailed
	}

	h.massProperties()
	h.centerOfMass = h.centerOfMass.Add(b.origin)
	h.bounds = AABB{Min: h.Points[0], Max: h.Points[0]}
	for _, p := range h.Points[1:] {
		for k := 0; k < 3; k++ {
			h.bounds.Min[k] = math.Min(h.bounds.Min[k], p[k])
			h.bounds.Max[k] = math.Max(h.bounds.Max[k], p[k])
		}
	}
	return h, nil
}

// dropCollinear removes from the loop the vertices on the segment between their neighbors, within the tolerance (a
// vertex of merged triangles in the middle of a side of the polygon), and the repeated vertices
func (b *hullBuilder) dropCollinear(loop []int32) []int32 {
	for changed := true; changed && len(loop) > 3; {
		changed = false
		for k := 0; k < len(loop); k++ {
			previous, current, next := b.points[loop[(k+len(loop)-1)%len(loop)]], b.points[loop[k]], b.points[loop[(k+1)%len(loop)]]
			side := next.Sub(previous)
			if loop[k] == loop[(k+1)%len(loop)] || current.Sub(previous).Cross(side).Len() <= b.tolerance*side.Len() {
				loop = slices.Delete(loop, k, k+1)
				changed = true
				break
			}
		}
	}
	return loop
}

// massProperties: the volume, the center of mass and the inertia of the hull, from the tetrahedra between its faces
// (fans of triangles) and a point inside (the mean of the vertices). The inertia for a density of 1 is the covariance
// of the canonical tetrahedron carried by each tetrahedron (ConvexHullShape of Jolt, after Blow & Binstock). The
// points are then centered on the center of mass
func (h *ConvexHull) massProperties() {
	inside := mgl64.Vec3{}
	for _, p := range h.Points {
		inside = inside.Add(p)
	}
	inside = inside.Mul(1 / float64(len(h.Points)))

	volume, center := 0.0, mgl64.Vec3{}
	for _, face := range h.faces {
		vertices := h.faceVertices[face.first : face.first+face.count]
		v1 := h.Points[vertices[0]].Sub(inside)
		for k := 1; k+1 < len(vertices); k++ {
			v2, v3 := h.Points[vertices[k]].Sub(inside), h.Points[vertices[k+1]].Sub(inside)
			// 6 times the volume, 4 times the centroid: divided at the end
			tetra := v1.Dot(v2.Cross(v3))
			volume += tetra
			center = center.Add(v1.Add(v2).Add(v3).Mul(tetra))
		}
	}
	if volume > 0 {
		center = center.Mul(1 / (4 * volume))
	}
	center = center.Add(inside)
	h.volume = volume / 6
	h.centerOfMass = center
	for i := range h.Points {
		h.Points[i] = h.Points[i].Sub(center)
	}
	for i := range h.faces {
		h.faces[i].distance += h.faces[i].normal.Dot(center)
	}

	// the covariance of the canonical tetrahedron (0, e1, e2, e3): 1/60 on the diagonal, 1/120 elsewhere, for a
	// density of 1. A tetrahedron (0, v1, v2, v3) is its image by A = [v1 v2 v3]: covariance det(A) A C Aᵀ
	canonical := mgl64.Mat3{1.0 / 60, 1.0 / 120, 1.0 / 120, 1.0 / 120, 1.0 / 60, 1.0 / 120, 1.0 / 120, 1.0 / 120, 1.0 / 60}
	var covariance mgl64.Mat3
	for _, face := range h.faces {
		vertices := h.faceVertices[face.first : face.first+face.count]
		v1 := h.Points[vertices[0]]
		for k := 1; k+1 < len(vertices); k++ {
			v2, v3 := h.Points[vertices[k]], h.Points[vertices[k+1]]
			a := mgl64.Mat3{v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]}
			det := Det3(&a)
			ac := Mul3(&a, &canonical)
			at := Transpose3(&a)
			c := Mul3(&ac, &at)
			for i := range covariance {
				covariance[i] += det * c[i]
			}
		}
	}
	trace := covariance[0] + covariance[4] + covariance[8]
	h.inertia = mgl64.Mat3{
		trace - covariance[0], -covariance[1], -covariance[2],
		-covariance[3], trace - covariance[4], -covariance[5],
		-covariance[6], -covariance[7], trace - covariance[8],
	}
}
