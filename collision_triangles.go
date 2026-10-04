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
	// MaxManifoldsPerPair: a body on a terrain touches it with 8 normals at most (8 patches): on a rough terrain, both
	// ends of a capsule can touch 4 triangles each. Jolt keeps 32 manifolds per pair (PhysicsSystem.cpp), PhysX 6
	// (GU_MAX_MANIFOLD_SIZE); the deepest patches are kept, the ones left out are the shallowest
	MaxManifoldsPerPair = 8

	// patchCos: the contacts of 2 triangles whose normals differ by less than 5° are in the same patch: cos(5°)
	// (mContactNormalCosMaxDeltaRotation of Jolt)
	patchCos = 0.99619469809174553229501040247389

	// nearFaceCos: a contact normal closer than 1° to the normal of its triangle is kept as it is: cos(1°), the value
	// of ActiveEdges::FixNormal of Jolt
	nearFaceCos = 0.999848

	// edgeBarycentric: a point of a triangle with a barycentric coordinate under this value is on an edge, over 1 minus
	// this value on a vertex (cEpsilon of ActiveEdges::FixNormal of Jolt)
	edgeBarycentric = 1e-4

	// weldDistance: 2 points of a patch closer than this distance are the same point (m)
	weldDistance = 1e-4

	// clipEpsilon: a point on a side of a triangle is over the triangle (m)
	clipEpsilon = 1e-9

	// maxClipVertices: the face of a body has 8 vertices at most (ShapeInterface.GetContactFeature), each of the 3 sides
	// of a triangle adds one
	maxClipVertices = 11
)

// triangleShape is a triangle of a surface (a heightfield, a mesh), in world space: its body has the identity transform
type triangleShape struct {
	vertices [3]mgl64.Vec3
	aabb     actor.AABB
}

// ComputeAABB: the vertices are in world space
func (t *triangleShape) ComputeAABB(transform actor.Transform) actor.AABB {
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

// CastRay: a ray is cast on the surface, not on its triangles
func (t *triangleShape) CastRay(origin, translation mgl64.Vec3, maxFraction float64) (actor.RayHit, bool) {
	return actor.RayHit{}, false
}

// triangleContact: points of a triangle, in triangleScratch.points, with their normal (from the surface to the
// body) and the separation of the deepest one
type triangleContact struct {
	normal     mgl64.Vec3
	first      int
	count      int
	separation float64
	// edge: the points of the body on the edges of the triangle, else its vertices over the face of the triangle
	edge bool
	// closest: the closest point of the body alone
	closest bool
}

// triangleScratch: the buffers of a collision with a surface of triangles, reused to avoid the allocations
type triangleScratch struct {
	shape    triangleShape
	triangle actor.RigidBody
	simplex  gjk.Simplex
	// epa: the buffers of EPA of the character which owns the scratch (contactScratch), nil for those of the pool of EPA
	epa *epa.Scratch
	// the cells of a heightfield, the triangles of a mesh, under the body
	cells     []int32
	triangles []int32
	plane     actor.PlaneContact
	contacts  []triangleContact
	points    []constraint.ContactPoint
	patches   [MaxManifoldsPerPair][]constraint.ContactPoint
	clip      [2][maxClipVertices]mgl64.Vec3
	// sides: the sides of the triangle each vertex of clip was cut on (bit e), 0 for a vertex of the body
	sides [2][maxClipVertices]uint8
	// movement: the direction the body moves in, for the normals of the contacts on the inactive edges (contactNormal);
	// zero for the pairs of a step
	movement mgl64.Vec3
	// the cores of the triangle (made once: the shape is the one of the scratch, at the identity) and of the body
	// (made once per pair), for the GJK of every triangle
	triangleCore, objectCore gjk.Proxy
	objectRadius             float64
}

// init the triangle of the scratch and its core: they point to the scratch, which must not be copied
func (s *triangleScratch) init() {
	s.triangle = actor.RigidBody{Transform: actor.NewTransform(), BodyType: actor.BodyTypeStatic, Shape: &s.shape}
	s.triangleCore, _ = gjk.NewCoreProxy(&s.triangle)
}

var trianglePool = sync.Pool{New: func() any {
	s := &triangleScratch{}
	s.init()
	return s
}}

// isSurface: the body is a heightfield or a triangle mesh: a static surface of triangles, which a body touches with
// several normals (MaxManifoldsPerPair manifolds)
func isSurface(body *actor.RigidBody) bool {
	switch body.Shape.(type) {
	case *actor.Heightfield, *actor.TriangleMesh:
		return true
	}
	return false
}

// collideTriangles writes in out the patches of contact between the surface (a heightfield or a triangle mesh, the
// shape of surface) and the body, and returns their count. The surfaces differ by the triangles they give under the
// AABB of the body (the cells of a heightfield, the leaves of the tree of a mesh) and by nothing else: one generator of
// contacts per triangle (collideTriangle) and one grouping in patches serve both, as b3ComputeMeshManifolds of Box3D
// takes the triangles of b3QueryMeshTriangles or of b3QueryHeightFieldTriangles (src/mesh_contact.c:51-73), as
// CollideConvexVsTriangles of Jolt is the visitor of its mesh and of its height field, and PCMConvexVsMeshContactGeneration
// of PhysX serves its mesh and its height field.
// Each triangle under the body, seen from the side of its normal, is tested with GJK/EPA (collideTriangle): it gives
// the vertices of the body over its face, and the points of the body on its edges (a ridge in the face of a box, an
// active edge).
// The contacts are then grouped by normal: one patch (a manifold of 4 points) per normal, for the vertices of the body,
// and one for the edges of the surface. On a flat surface a body has the manifold it has on a plane: its vertices.
// Jolt and Box3D group by normal alone, and keep 4 points out of all of them (PruneContactPoints, b3ReduceCluster).
// In one manifold of 4 points, the points on the edges of the surface take the place of a corner of the body: a plate
// resting flat lost 2 corners to 2 points of its face over an edge of the grid, and sank by 10 mm on this side while
// the pair cache kept its contact.
// The order of the pair is kept: if the surface is B, the normals point from the body to the surface. The buffers are
// those of owner, those of the pool if nil
func collideTriangles(owner *contactScratch, surface, object *actor.RigidBody, margin float64, surfaceIsB bool, movement mgl64.Vec3, out []constraint.Manifold) int {
	var s *triangleScratch
	if owner != nil {
		s = &owner.triangles
	} else {
		s = trianglePool.Get().(*triangleScratch)
		defer trianglePool.Put(s)
	}
	s.movement = movement
	s.objectCore, s.objectRadius = gjk.NewCoreProxy(object)

	bounds := object.AABB()
	bounds = actor.AABB{Min: bounds.Min.Sub(mgl64.Vec3{margin, margin, margin}), Max: bounds.Max.Add(mgl64.Vec3{margin, margin, margin})}
	local := localBounds(surface.Transform, bounds)
	s.contacts, s.points = s.contacts[:0], s.points[:0]

	switch shape := surface.Shape.(type) {
	case *actor.Heightfield:
		s.cells = shape.OverlapCells(local, s.cells[:0])
		cellsZ := shape.ZSamples - 1
		for _, cell := range s.cells {
			x, z := int(cell)/cellsZ, int(cell)%cellsZ
			for t := 0; t < 2; t++ {
				vertices, edges := shape.Triangle(x, z, t)
				s.collideLocalTriangle(surface, object, vertices, edges, bounds, margin)
			}
		}
	case *actor.TriangleMesh:
		s.triangles = shape.OverlapTriangles(local, s.triangles[:0])
		for _, t := range s.triangles {
			vertices, edges := shape.Triangle(t)
			s.collideLocalTriangle(surface, object, vertices, edges, bounds, margin)
		}
	}
	if len(s.contacts) == 0 {
		return 0
	}

	// ========== PATCHES ==========
	// the deepest contacts first: they give the normal of their patch, the shallowest are dropped (Jolt replaces its
	// shallowest manifold, and gives a manifold the mean of its normals). A closest point comes after the points as
	// deep as it
	slices.SortStableFunc(s.contacts, func(a, b triangleContact) int {
		depthA, depthB := a.separation, b.separation
		if a.closest {
			depthA += epa.EPATieTolerance
		}
		if b.closest {
			depthB += epa.EPATieTolerance
		}
		switch {
		case depthA < depthB:
			return -1
		case depthA > depthB:
			return 1
		}
		return 0
	})
	var normals [MaxManifoldsPerPair]mgl64.Vec3
	var edges [MaxManifoldsPerPair]bool
	patches := 0
	for _, contact := range s.contacts {
		best, bestCos, held := -1, math.Inf(-1), false
		for k := 0; k < patches; k++ {
			cos := normals[k].Dot(contact.normal)
			held = held || cos >= patchCos
			if edges[k] == contact.edge && cos > bestCos {
				best, bestCos = k, cos
			}
		}
		if contact.closest && contact.edge && held {
			// a closest point on an edge is the point of the terrain several triangles see, on the edge they share: it
			// is kept if it is the deepest point along its normal. A body resting on a triangle near an edge would have
			// one more point per triangle around, moving its friction center (a sphere would not roll on a flat terrain
			// as on a plane): Box3D and PhysX drop such a point when a triangle with a contact of its face owns the edge
			// (b3ComputeMeshManifolds, generateLastContacts). The depth is compared instead: the end of a capsule right
			// above a ridge is over neither of its triangles, along their normals, while the rest of the capsule gives
			// them a contact of their face. Its deepest point was dropped this way, and bodies sank by centimetres.
			// A closest point inside the triangle is a point of its face (a sphere): kept whatever its depth, as the
			// vertices of a body are (Box3D and PhysX keep it too: a point of a face owns no edge). It was dropped when
			// a patch within 5° had a deeper point: a sphere rolling to a triangle folded by less than 5° had no contact
			// with it until it was the deepest, and sank by 3 mm
			continue
		}
		if bestCos < patchCos {
			// a new normal: a new patch, unless the deepest patches are already found
			if patches == min(len(out), MaxManifoldsPerPair) {
				continue
			}
			best = patches
			normals[best], edges[best] = contact.normal, contact.edge
			s.patches[best] = s.patches[best][:0]
			patches++
		}
		for _, point := range s.points[contact.first : contact.first+contact.count] {
			s.patches[best] = weld(s.patches[best], point)
		}
	}

	for k := 0; k < patches; k++ {
		m := &out[k]
		m.Reset(surface, object)
		m.Normal = normals[k]
		if surfaceIsB {
			m.Reset(object, surface)
			m.Normal = normals[k].Mul(-1)
		}
		reducePatch(s.patches[k], normals[k], m)
	}
	return patches
}

// collideLocalTriangle: the triangle of the surface, given in its local space with its edges (bit e active, bit 3+e
// convex), is taken to world space and tested if its AABB overlaps the bounds of the body
func (s *triangleScratch) collideLocalTriangle(surface, object *actor.RigidBody, vertices [3]mgl64.Vec3, edges uint8, bounds actor.AABB, margin float64) {
	for i := range vertices {
		s.shape.vertices[i] = surface.Transform.ToWorld(vertices[i])
	}
	s.shape.aabb = triangleAABB(s.shape.vertices)
	if s.shape.aabb.Overlaps(bounds) {
		s.collideTriangle(object, edges&0b111, edges>>3, margin)
	}
}

// collideTriangle adds the contacts of the body with the triangle of s.shape:
//   - a triangle seen from below is ignored: the center of the body is under its plane (CollideConvexVsTriangles of
//     Jolt, b3ComputeMeshManifolds of Box3D, PCMConvexVsMeshContactGeneration of PhysX). The terrain is a surface: a
//     body whose center went through it is not pushed back
//   - the face: the vertices of the body over the triangle, along the normal of the triangle (clipFace), as against a
//     plane. Whatever the closest point is: a corner which turns towards the face of a triangle during the step has
//     its contact
//   - the ridges too flat to be active edges (inactive, bent outwards): the points where the face of the body crosses
//     them, along the normal of the triangle. The side of a capsule, the face of a box laid across such a ridge touch
//     it there, with no vertex
//   - GJK/EPA gives the closest points of the triangle and of the body, their normal and their depth. Their normal is
//     the one of EPA, or the one of the triangle on an inactive edge (contactNormal)
//   - with the normal of the triangle, the closest point is one more point of the triangle, if it is deeper than the
//     others: a point of its face (a sphere), or of its edge with the body beside the triangle, above this edge
//   - with another normal (an active edge), the points where the body crosses the active edges, along this normal, or
//     the closest point alone (a sphere, an edge of a box across the edge)
//
// Every separation is measured along the normal of its contact.
// activeEdges, convexEdges: bit e for the edge from the vertex e to the vertex e+1
func (s *triangleScratch) collideTriangle(object *actor.RigidBody, activeEdges, convexEdges uint8, margin float64) {
	vertices := &s.shape.vertices
	faceNormal := vertices[1].Sub(vertices[0]).Cross(vertices[2].Sub(vertices[0])).Normalize()
	if faceNormal.Dot(object.Transform.Position.Sub(vertices[0])) < 0 {
		return
	}
	result, ok := penetrationOfCores(&s.triangle, object, &s.triangleCore, 0, &s.objectCore, s.objectRadius, margin, &s.simplex, s.epa)
	if !ok {
		return
	}

	// ========== FACE ==========
	first := len(s.points)
	s.clipFace(object, faceNormal, faceNormal, true, 0, margin)
	faceDeepest := s.addContact(faceNormal, first, false, false)

	// ========== FLAT RIDGES ==========
	// across a flat edge, or one bent inwards, the vertices of the body on both sides say everything. Across an active
	// edge, the point belongs to the contact of this edge: the plane of the triangle stops at its edge, a face sliding
	// across a ridge must not be held by the plane of the slope it leaves
	if ridges := convexEdges &^ activeEdges; ridges != 0 {
		first = len(s.points)
		s.clipFace(object, faceNormal, faceNormal, false, ridges, margin)
		faceDeepest = math.Min(faceDeepest, s.addContact(faceNormal, first, true, false))
	}

	// ========== CLOSEST POINT ==========
	// WitnessA is on the triangle grown by the margin
	onTriangle, onBody := result.WitnessA.Sub(result.Normal.Mul(margin)), result.WitnessB
	touched := touchedEdges(vertices, onTriangle)
	normal := contactNormal(faceNormal, activeEdges, touched, result.Normal, s.movement)
	separation := onBody.Sub(onTriangle).Dot(normal)
	closest := constraint.ContactPoint{Position: onBody.Sub(normal.Mul(separation / 2)), Separation: separation}
	first = len(s.points)
	if normal == faceNormal || normal.Dot(faceNormal) > nearFaceCos {
		// a point of the face, inside the triangle (a sphere), or a point on an inactive edge, the body beside the
		// triangle: with the points of the edges
		if separation < faceDeepest-epa.EPATieTolerance {
			s.points = append(s.points, closest)
			s.addContact(normal, first, touched != 0, true)
		}
		return
	}

	// ========== EDGE ==========
	// the normal of EPA: on an active edge
	s.clipFace(object, normal, faceNormal, false, activeEdges, margin)
	if len(s.points) > first {
		s.addContact(normal, first, true, false)
		return
	}
	s.points = append(s.points, closest)
	s.addContact(normal, first, true, true)
}

// addContact: the points from first to the end of s.points are a contact. Returns the separation of its deepest point,
// +Inf without point
func (s *triangleScratch) addContact(normal mgl64.Vec3, first int, edge, closest bool) float64 {
	deepest := math.Inf(1)
	for _, point := range s.points[first:] {
		deepest = math.Min(deepest, point.Separation)
	}
	if len(s.points) > first {
		s.contacts = append(s.contacts, triangleContact{normal: normal, first: first, count: len(s.points) - first, separation: deepest, edge: edge, closest: closest})
	}
	return deepest
}

// touchedEdges: the edges of the triangle the point is on (bit e for the edge from the vertex e to the vertex e+1): one
// edge, the 2 edges of a vertex, 0 inside the triangle. The barycentric coordinates of ActiveEdges::FixNormal of Jolt:
// the edge e is opposite to the vertex e+2
func touchedEdges(vertices *[3]mgl64.Vec3, point mgl64.Vec3) uint8 {
	u, v, w := barycentric(point, vertices[0], vertices[1], vertices[2])
	switch {
	case u > 1-edgeBarycentric:
		return 0b101
	case v > 1-edgeBarycentric:
		return 0b011
	case w > 1-edgeBarycentric:
		return 0b110
	case u < edgeBarycentric:
		return 0b010
	case v < edgeBarycentric:
		return 0b100
	case w < edgeBarycentric:
		return 0b001
	}
	return 0
}

// contactNormal: the normal of the contact of a body with a triangle, given the normal of EPA and the edges of the
// triangle its point is on (touchedEdges). A body sliding on the terrain must not hit the edges between its triangles:
// on an inactive edge (between 2 triangles almost flat, or bent inwards) the normal is the one of the triangle. The rule
// of ActiveEdges::FixNormal of Jolt (ActiveEdges.h). The normal of EPA is kept:
//   - if the body moves (movement, a unit direction, or zero), and the normal of EPA brakes its motion less than the
//     normal of the triangle: the body grazes a triangle it can't reach by its face (a capsule level with the top edge
//     of a riser, whose closest point is on the flat diagonal of the riser: with the normal of the riser it would be a
//     wall at a distance of 0, where its real distance is along the normal of EPA), or leaves a triangle. The hint of
//     Jolt, "to make a distinction between sliding over a horizontal triangulated grid and hitting an edge ... and
//     grazing a vertical triangle with an inactive edge"
//   - if the 3 edges of the triangle are active
//   - if the triangle has an active edge, and the normal of EPA is closer than 1° to its normal
//   - if the point is on an active edge, or on a vertex of an active edge
//
// The pairs of a step give no movement: Jolt keeps the normal of EPA when it brakes the motion of the body less than
// the normal of the triangle (a body leaving a triangle is not held by its plane): its margin is fixed (2 cm,
// mSpeculativeContactDistance), the speculative margin of Feather grows with the speed of the body (reach), and
// brings the vertices and the edges of the triangles around within it. A box falling flat on a flat terrain, straight
// on a vertex of the grid, had 5 contacts at the same corner: one straight up, 4 on the edges of the triangles around,
// tilted by 10 to 60°, each braking a vertical fall less than the plane. The tilted ones, nearly parallel, took impulse
// from the vertical one over the sub-steps, and the box landed 2 mm deep. The character (character_move.go) gives the
// direction of its velocity: its contacts are gathered within 10 cm whatever its speed, and the rule is the one of
// CharacterVirtual (CheckCollision passes the movement direction). PhysX and Box3D have no such rule
// (PCMContactConvexMesh, b3ComputeMeshManifolds). A box sliding across an active ridge is not held by the plane it
// leaves: the points cut on an active edge are never points of the face (clipFace)
//
// activeEdges: bit e for the edge from the vertex e to the vertex e+1. The normals point from the triangle to the body:
// the ones of Jolt point into the triangle, its test reads the other way
func contactNormal(faceNormal mgl64.Vec3, activeEdges, touched uint8, normal, movement mgl64.Vec3) mgl64.Vec3 {
	if movement.Dot(normal) > movement.Dot(faceNormal) {
		return normal
	}
	if activeEdges == 0b111 {
		return normal
	}
	if activeEdges == 0 {
		return faceNormal
	}
	if normal.Dot(faceNormal) > nearFaceCos || activeEdges&touched != 0 {
		return normal
	}
	return faceNormal
}

// clipFace adds to s.points the points of the face of the body over the triangle of s.shape, closer to it than the
// margin along the normal. The face is the one the body shows to a plane of this normal (CollideWithPlane, whatever
// the distance): the 4 corners of a face of a box, both ends of a capsule. It is taken without any threshold on its
// alignment with the normal, as the supporting face of Jolt (BoxShape::GetSupportingFace); a capsule gives both its
// ends whatever its tilt, where Jolt asks them within 2 cm along the normal (cCapsuleProjectionSlop): the points under
// its segment are on its surface at any tilt. Seen along the normal, the face is cut by the 3 sides of the triangle
// (Sutherland-Hodgman): what is left is the part of the face over the triangle, the same polygon as the triangle cut
// by the sides of the face (ManifoldBetweenTwoFaces of Jolt, ClipPolyVsPoly). A segment is cut the same way, as
// b3CollideTriangleAndCapsule of Box3D: ClipPolyVsEdge of Jolt can give a point out of the triangle when the capsule
// ends beside it. Each vertex left is a point of the body, at its distance to the plane of the triangle along the
// normal.
//
// bodyVertices: the vertices of the face over the triangle are kept: the points of the body against a plane.
// cutSides: the points cut on these sides of the triangle are kept (bit e for the edge from the vertex e to the vertex
// e+1): a ridge of the terrain in the face of a box, across the side of a capsule, is found where it crosses them.
// Jolt keeps all the points of the polygon. The points cut on a flat edge are between vertices of the body, on the
// same plane: they say nothing more, and take the place of a corner among the 4 points of a manifold
func (s *triangleScratch) clipFace(object *actor.RigidBody, normal, faceNormal mgl64.Vec3, bodyVertices bool, cutSides uint8, margin float64) {
	vertices := &s.shape.vertices
	alignment := normal.Dot(faceNormal)
	if alignment <= 0 {
		// the body touches the triangle from its side: no face over it
		return
	}
	s.plane = object.Shape.CollideWithPlane(normal, -normal.Dot(vertices[0]), object.Transform, math.Inf(1), s.plane[:0])
	if len(s.plane) < 2 || len(s.plane) > maxClipVertices-3 {
		return
	}
	face, sides, count := &s.clip[0], &s.sides[0], len(s.plane)
	for i, point := range s.plane {
		// Position is halfway between the body and the plane
		face[i], sides[i] = point.Position.Add(normal.Mul(point.Separation/2)), 0
	}
	clipped, clippedSides := &s.clip[1], &s.sides[1]
	for e := 0; e < 3 && count > 0; e++ {
		// the side of the triangle along the normal, its inner side
		inward := normal.Cross(vertices[(e+1)%3].Sub(vertices[e]))
		count = clipAgainstSide(face[:count], sides, vertices[e], inward, 1<<e, clipped, clippedSides)
		face, clipped, sides, clippedSides = clipped, face, clippedSides, sides
	}
	for i, vertex := range face[:count] {
		if (sides[i] == 0 && !bodyVertices) || (sides[i] != 0 && sides[i]&cutSides == 0) {
			continue
		}
		separation := vertex.Sub(vertices[0]).Dot(faceNormal) / alignment
		if separation <= margin {
			s.points = append(s.points, constraint.ContactPoint{Position: vertex.Sub(normal.Mul(separation / 2)), Separation: separation})
		}
	}
}

// clipAgainstSide writes in out the part of the polygon (2 points: a segment) on the inner side of the plane, and
// returns its count of vertices. sides follows the vertices: a vertex cut on this plane is on the side (its bit), and
// on the sides both its neighbors are on
func clipAgainstSide(in []mgl64.Vec3, inSides *[maxClipVertices]uint8, point, inward mgl64.Vec3, side uint8, out *[maxClipVertices]mgl64.Vec3, outSides *[maxClipVertices]uint8) int {
	length := inward.Len()
	if length == 0 {
		return 0
	}
	epsilon := clipEpsilon * length
	count := 0
	edges := len(in)
	if edges == 2 {
		edges = 1 // an open segment, not a closed polygon
	}
	if len(in) == 1 {
		if in[0].Sub(point).Dot(inward) >= -epsilon {
			out[0], outSides[0], count = in[0], inSides[0], 1
		}
		return count
	}
	for i := 0; i < edges; i++ {
		j := (i + 1) % len(in)
		current, next := in[i], in[j]
		distance, nextDistance := current.Sub(point).Dot(inward), next.Sub(point).Dot(inward)
		if distance >= -epsilon {
			out[count], outSides[count] = current, inSides[i]
			count++
		}
		if (distance >= -epsilon) != (nextDistance >= -epsilon) {
			out[count] = current.Add(next.Sub(current).Mul(distance / (distance - nextDistance)))
			outSides[count] = side | (inSides[i] & inSides[j])
			count++
		}
		if len(in) == 2 && nextDistance >= -epsilon {
			out[count], outSides[count] = next, inSides[j]
			count++
		}
	}
	return count
}

// reducePatch adds to m 4 points of the patch at most. The points of a patch come from several triangles. The steps of
// Reduce (the ones of b3ReduceManifoldPoints of Box3D): the deepest point, the furthest from it, then the points adding
// the most area. 2 differences, for the points of several triangles:
//   - among the points as deep as the deepest (EPATieTolerance), the first one is the furthest along a tangent, then
//     along the other (the first step of Box3D looks for an extreme point along a tangent): an end of the contact,
//     never a point inside it
//   - a point is added if it widens the contact by SpeculativeDistance at least (Box3D has the same tolerance, on the
//     area), and the best one wins, without the bias Reduce gives to the order of its points: the order of the points
//     of a patch is the order of its triangles
func reducePatch(points []constraint.ContactPoint, normal mgl64.Vec3, m *constraint.Manifold) {
	if len(points) <= constraint.MaxContactPoints {
		for _, p := range points {
			m.Add(p.Position, p.Separation)
		}
		return
	}

	// ========== THE DEEPEST, AT AN END ==========
	tangent := normal.Cross(mgl64.Vec3{1, 0, 0})
	if math.Abs(normal.X()) > 0.5 {
		tangent = normal.Cross(mgl64.Vec3{0, 0, 1})
	}
	tangent = tangent.Normalize()
	bitangent := normal.Cross(tangent)
	lowest := math.Inf(1)
	for _, p := range points {
		lowest = math.Min(lowest, p.Separation)
	}
	first := -1
	for i, p := range points {
		if p.Separation > lowest+epa.EPATieTolerance {
			continue
		}
		if first < 0 {
			first = i
			continue
		}
		along := p.Position.Sub(points[first].Position).Dot(tangent)
		if along > weldDistance || (along >= -weldDistance && p.Position.Sub(points[first].Position).Dot(bitangent) > 0) {
			first = i
		}
	}
	m.Add(points[first].Position, points[first].Separation)

	// ========== THE WIDEST CONTACT ==========
	planar := func(a, b int) mgl64.Vec3 {
		offset := points[b].Position.Sub(points[a].Position)
		return offset.Sub(normal.Mul(offset.Dot(normal)))
	}
	// area: twice the area of the triangle, seen along the normal
	area := func(a, b, c int) float64 { return planar(a, b).Cross(planar(a, c)).Dot(normal) }
	kept := [constraint.MaxContactPoints]int{first}
	orientation := 0.0
	for m.Count < constraint.MaxContactPoints {
		chosen, highest := -1, 0.0
		for i := range points {
			// how much the point widens the contact of the points kept, and if it does by SpeculativeDistance
			value, widens := 0.0, false
			switch m.Count {
			case 1:
				value = planar(first, i).LenSqr()
				widens = value > SpeculativeDistance*SpeculativeDistance
			case 2:
				// its distance to the line of the 2 points
				value = math.Abs(area(kept[0], kept[1], i))
				widens = value > SpeculativeDistance*planar(kept[0], kept[1]).Len()
			default:
				// the area added outside an edge of the triangle, by a point further than the tolerance from this edge
				for e := 0; e < 3; e++ {
					from, to := kept[e], kept[(e+1)%3]
					outside := -orientation * area(from, to, i)
					value = math.Max(value, outside)
					widens = widens || outside > SpeculativeDistance*planar(from, to).Len()
				}
			}
			if widens && value > highest {
				chosen, highest = i, value
			}
		}
		if chosen < 0 {
			return
		}
		kept[m.Count] = chosen
		m.Add(points[chosen].Position, points[chosen].Separation)
		if m.Count == 3 {
			orientation = math.Copysign(1, area(kept[0], kept[1], kept[2]))
		}
	}
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
