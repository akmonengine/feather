package feather

import (
	"math"
	"slices"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

const (
	// MaxManifoldsPerPair: a body on a terrain touches it with 8 normals at most (8 patches):
	// on a rough terrain, both ends of a capsule can touch 4 triangles each
	MaxManifoldsPerPair = 8

	// patchCos: the contacts of 2 triangles whose normals differ by less than 5° are in the same patch: cos(5°)
	patchCos = 0.99619469809174553229501040247389

	// triangleFaceCos: the contact of a triangle is a face contact if its normal is the normal of the triangle (0.5°)
	triangleFaceCos = 0.99996

	// edgeBarycentric: a point on a triangle with a barycentric coordinate under this value is on an edge
	edgeBarycentric = 1e-3

	// weldDistance: 2 points of a patch closer than this distance are the same point (m)
	weldDistance = 1e-4

	// insideTriangle: a point on a side of a triangle is inside (barycentric coordinate)
	insideTriangle = -1e-9
)

// triangleShape is a triangle of a heightfield, in world space: its body has the identity transform
type triangleShape struct {
	vertices [3]mgl64.Vec3
	aabb     actor.AABB
}

func (t *triangleShape) ComputeAABB(transform actor.Transform) {}

func (t *triangleShape) GetAABB() actor.AABB {
	return t.aabb
}

func (t *triangleShape) ComputeMass(density float64) float64 {
	return math.Inf(1)
}

func (t *triangleShape) ComputeInertia(mass float64) mgl64.Mat3 {
	return mgl64.Mat3{}
}

func (t *triangleShape) Support(direction mgl64.Vec3) mgl64.Vec3 {
	best := 0
	for i := 1; i < 3; i++ {
		if t.vertices[i].Dot(direction) > t.vertices[best].Dot(direction) {
			best = i
		}
	}
	return t.vertices[best]
}

// GetContactFeature: the face, the manifold keeps its deepest edge or vertex if it is not aligned with the normal
func (t *triangleShape) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	copy(output[:3], t.vertices[:])
	*count = 3
}

func (t *triangleShape) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform actor.Transform, margin float64, contacts actor.PlaneContact) actor.PlaneContact {
	return contacts
}

// triangleContact: the points of a triangle, in heightfieldScratch.points, with their normal (from the terrain to the body).
// A witness contact is only used if its patch has no other contact
type triangleContact struct {
	normal     mgl64.Vec3
	first      int
	count      int
	separation float64
	witness    bool
}

// heightfieldScratch: the buffers of a collision with a heightfield, reused to avoid the allocations
type heightfieldScratch struct {
	shape    triangleShape
	triangle actor.RigidBody
	simplex  gjk.Simplex
	manifold constraint.Manifold
	cells    []int32
	plane    actor.PlaneContact
	contacts []triangleContact
	points   []constraint.ContactPoint
	patches  [MaxManifoldsPerPair][]constraint.ContactPoint
}

var heightfieldPool = sync.Pool{New: func() any {
	s := &heightfieldScratch{}
	s.triangle = actor.RigidBody{Transform: actor.NewTransform(), BodyType: actor.BodyTypeStatic, Shape: &s.shape}
	return s
}}

// collideHeightfield writes in out the patches of contact between the terrain and the body, and returns their count.
// Each triangle under the body is tested with GJK/EPA. A contact on an inactive edge (between 2 triangles almost flat,
// or bent inwards) takes the normal of its triangle: a body sliding on the terrain doesn't hit the inner edges.
// A contact with the face of a triangle is the contact with its plane (CollideWithPlane of the shape), limited to
// the triangle: on a flat terrain, the bodies behave exactly as on a plane.
// The contacts are then grouped by normal: one patch (a manifold of 4 points) per normal.
// The order of the pair is kept: if the terrain is B, the normals point from the body to the terrain
func collideHeightfield(terrain *actor.RigidBody, field *actor.Heightfield, object *actor.RigidBody, margin float64, terrainIsB bool, out []constraint.Manifold) int {
	s := heightfieldPool.Get().(*heightfieldScratch)
	defer heightfieldPool.Put(s)

	bounds := object.Shape.GetAABB()
	bounds = actor.AABB{Min: bounds.Min.Sub(mgl64.Vec3{margin, margin, margin}), Max: bounds.Max.Add(mgl64.Vec3{margin, margin, margin})}
	s.cells = field.OverlapCells(localBounds(terrain.Transform, bounds), s.cells[:0])
	s.contacts, s.points = s.contacts[:0], s.points[:0]

	cellsZ := field.ZSamples - 1
	for _, cell := range s.cells {
		x, z := int(cell)/cellsZ, int(cell)%cellsZ
		for t := 0; t < 2; t++ {
			local, edges := field.Triangle(x, z, t)
			for i := range local {
				s.shape.vertices[i] = terrain.Transform.ToWorld(local[i])
			}
			s.shape.aabb = triangleAABB(s.shape.vertices)
			if s.shape.aabb.Overlaps(bounds) {
				s.collideTriangle(object, edges, margin)
			}
		}
	}
	if len(s.contacts) == 0 {
		return 0
	}

	// ========== PATCHES ==========
	// the deepest contacts first: they give the normal of their patch, the shallowest are dropped (as in Jolt)
	slices.SortStableFunc(s.contacts, func(a, b triangleContact) int {
		switch {
		case a.separation < b.separation:
			return -1
		case a.separation > b.separation:
			return 1
		}
		return 0
	})
	var normals [MaxManifoldsPerPair]mgl64.Vec3
	var witnesses [MaxManifoldsPerPair]int
	patches := 0
	for c, contact := range s.contacts {
		best, bestCos := -1, math.Inf(-1)
		for k := 0; k < patches; k++ {
			if cos := normals[k].Dot(contact.normal); cos > bestCos {
				best, bestCos = k, cos
			}
		}
		if bestCos < patchCos {
			// a new normal: a new patch, unless the deepest patches are already found
			if patches == min(len(out), MaxManifoldsPerPair) {
				continue
			}
			best = patches
			normals[best] = contact.normal
			s.patches[best] = s.patches[best][:0]
			witnesses[best] = -1
			patches++
		}
		if contact.witness {
			// the deepest witness contact of the patch, in case it has no other contact
			if witnesses[best] < 0 {
				witnesses[best] = c
			}
			continue
		}
		for _, point := range s.points[contact.first : contact.first+contact.count] {
			s.patches[best] = weld(s.patches[best], point)
		}
	}
	for k := 0; k < patches; k++ {
		if len(s.patches[k]) == 0 {
			witness := s.contacts[witnesses[k]]
			s.patches[k] = append(s.patches[k], s.points[witness.first])
		}
	}

	for k := 0; k < patches; k++ {
		m := &out[k]
		m.Reset(terrain, object)
		m.Normal = normals[k]
		if terrainIsB {
			m.Reset(object, terrain)
			m.Normal = normals[k].Mul(-1)
		}
		epa.Reduce(s.patches[k], normals[k], m)
	}
	return patches
}

// collideTriangle adds the contact of the body with the triangle of s.shape
func (s *heightfieldScratch) collideTriangle(object *actor.RigidBody, edges uint8, margin float64) {
	s.simplex.Reset()
	proxyA, proxyB := gjk.NewProxy(&s.triangle), gjk.NewProxy(object)
	if !gjk.GJKProxies(&proxyA, &proxyB, margin, &s.simplex) {
		return
	}
	result, err := epa.EPAProxies(&proxyA, &proxyB, &s.simplex, margin)
	if err != nil {
		return
	}

	vertices := s.shape.vertices
	faceNormal := vertices[1].Sub(vertices[0]).Cross(vertices[2].Sub(vertices[0])).Normalize()

	// ========== FACE ==========
	// the points of the body above the triangle, closer to its plane than the margin
	first := len(s.points)
	s.plane = object.Shape.CollideWithPlane(faceNormal, -faceNormal.Dot(vertices[0]), object.Transform, margin, s.plane[:0])
	for _, point := range s.plane {
		if u, v, w := barycentric(point.Position, vertices[0], vertices[1], vertices[2]); u >= insideTriangle && v >= insideTriangle && w >= insideTriangle {
			s.points = append(s.points, constraint.ContactPoint{Position: point.Position, Separation: point.Separation})
		}
	}
	faceFound := s.addContact(faceNormal, first, false)

	// ========== EDGE ==========
	// the body touches an active edge or vertex from above: the contact of EPA, with its normal
	onTriangle := result.WitnessA.Sub(result.Normal.Mul(margin))
	cos := result.Normal.Dot(faceNormal)
	if cos < triangleFaceCos && cos > 0 && touchesEdge(vertices, onTriangle, edges&0b111) {
		first = len(s.points)
		epa.Manifold(&s.triangle, object, result, margin, &s.manifold)
		s.points = append(s.points, s.manifold.Points[:s.manifold.Count]...)
		s.addContact(result.Normal, first, false)
		return
	}

	if !faceFound {
		// the body is beside the triangle, above an inactive edge: the witness point, with the normal of the face,
		// only used if no other triangle has a contact with this normal
		first = len(s.points)
		onB := result.WitnessB
		s.points = append(s.points, constraint.ContactPoint{Position: onTriangle.Add(onB).Mul(0.5), Separation: margin - result.Depth})
		s.addContact(faceNormal, first, true)
	}
}

// addContact: the points from first to the end of s.points, with their normal. Returns false if there is no point
func (s *heightfieldScratch) addContact(normal mgl64.Vec3, first int, witness bool) bool {
	count := len(s.points) - first
	if count == 0 {
		return false
	}
	deepest := math.Inf(1)
	for _, point := range s.points[first:] {
		deepest = math.Min(deepest, point.Separation)
	}
	s.contacts = append(s.contacts, triangleContact{normal: normal, first: first, count: count, separation: deepest, witness: witness})
	return true
}

// touchesEdge: the point is on one of the edges of the triangle (bit e for the edge from the vertex e to the vertex e+1),
// or on a vertex of these edges
func touchesEdge(vertices [3]mgl64.Vec3, p mgl64.Vec3, edges uint8) bool {
	u, v, w := barycentric(p, vertices[0], vertices[1], vertices[2])
	weights := [3]float64{u, v, w}
	for e := 0; e < 3; e++ {
		// the edge e goes from the vertex e to the vertex e+1: the weight of the opposite vertex is 0
		if weights[(e+2)%3] <= edgeBarycentric && edges&(1<<e) != 0 {
			return true
		}
	}
	return false
}

// barycentric coordinates of the projection of p on the triangle
func barycentric(p, a, b, c mgl64.Vec3) (float64, float64, float64) {
	v0, v1, v2 := b.Sub(a), c.Sub(a), p.Sub(a)
	d00, d01, d11 := v0.Dot(v0), v0.Dot(v1), v1.Dot(v1)
	d20, d21 := v2.Dot(v0), v2.Dot(v1)
	denominator := d00*d11 - d01*d01
	if denominator == 0 {
		return 1, 0, 0
	}
	v := (d11*d20 - d01*d21) / denominator
	w := (d00*d21 - d01*d20) / denominator
	return 1 - v - w, v, w
}

// weld adds the point, unless the patch already has it
func weld(points []constraint.ContactPoint, point constraint.ContactPoint) []constraint.ContactPoint {
	for i := range points {
		if points[i].Position.Sub(point.Position).LenSqr() < weldDistance*weldDistance {
			points[i].Separation = math.Min(points[i].Separation, point.Separation)
			return points
		}
	}
	return append(points, point)
}

func triangleAABB(vertices [3]mgl64.Vec3) actor.AABB {
	aabb := actor.AABB{Min: vertices[0], Max: vertices[0]}
	for _, v := range vertices[1:] {
		for k := 0; k < 3; k++ {
			aabb.Min[k] = math.Min(aabb.Min[k], v[k])
			aabb.Max[k] = math.Max(aabb.Max[k], v[k])
		}
	}
	return aabb
}

// localBounds: the world AABB seen in the local space of the transform (the AABB of its 8 corners)
func localBounds(transform actor.Transform, aabb actor.AABB) actor.AABB {
	min := mgl64.Vec3{math.Inf(1), math.Inf(1), math.Inf(1)}
	max := mgl64.Vec3{math.Inf(-1), math.Inf(-1), math.Inf(-1)}
	for i := 0; i < 8; i++ {
		corner := aabb.Min
		if i&1 != 0 {
			corner[0] = aabb.Max[0]
		}
		if i&2 != 0 {
			corner[1] = aabb.Max[1]
		}
		if i&4 != 0 {
			corner[2] = aabb.Max[2]
		}
		local := transform.ToLocal(corner)
		for k := 0; k < 3; k++ {
			min[k] = math.Min(min[k], local[k])
			max[k] = math.Max(max[k], local[k])
		}
	}
	return actor.AABB{Min: min, Max: max}
}
