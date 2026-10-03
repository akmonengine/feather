package feather

import (
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== QUERIES ON A SURFACE ==========
// A heightfield and a triangle mesh are swept and overlapped triangle by triangle: the triangles come from the cells
// under the path of the shape (Heightfield.WalkCells), or from the tree of the mesh (TriangleMesh.Cast,
// OverlapTriangles); the test of a triangle is the same (sweepTriangle, overlapsTriangle)

// localMotion: the transform of the moving shape and its translation in the local space of the surface
func (q *sweepQuery) localMotion(surface *actor.RigidBody) (actor.Transform, mgl64.Vec3) {
	rotation := &surface.Transform.Rotation
	conjugate := mgl64.Quat{W: rotation.W, V: rotation.V.Mul(-1)}
	local := actor.Transform{
		Position: actor.RotateInverse(rotation, q.start.Position.Sub(surface.Transform.Position)),
		Rotation: actor.MulQuat(&conjugate, &q.start.Rotation),
	}
	return local, actor.RotateInverse(rotation, q.translation)
}

// sweepHeightfield: the moving shape against the triangles of the cells under its path, in the order of its motion
// (Heightfield.WalkCells, as b3ShapeCastHeightField of Box3D): a band of the width of the shape, not the whole AABB
// of the motion. The walk ends when the shape enters the next cells after its best hit
func (q *sweepQuery) sweepHeightfield(terrain *actor.RigidBody, field *actor.Heightfield) (Hit, bool) {
	// the motion in the local space of the terrain: the AABB of the shape there, moved along the translation
	local, translation := q.localMotion(terrain)
	bounds := q.shape.ComputeAABB(local)
	reach := mgl64.Vec3{2 * sweepFlatGap, 2 * sweepFlatGap, 2 * sweepFlatGap}
	walk := field.WalkCells(bounds.Min.Add(bounds.Max).Mul(0.5), translation, bounds.Max.Sub(bounds.Min).Mul(0.5).Add(reach), q.limit)

	best, found := Hit{Fraction: q.limit}, false
	cellsZ := field.ZSamples - 1
	for {
		x, z, ok := walk.Next(best.Fraction)
		if !ok {
			break
		}
		for t := 0; t < 2; t++ {
			vertices, _ := field.Triangle(x, z, t)
			q.sweepLocalTriangle(terrain, vertices, int32(2*(x*cellsZ+z)+t), &best, &found)
		}
	}
	return best, found
}

// sweepMesh: the moving shape against the triangles of the leaves of the tree along its path, in the order of its
// motion (TriangleMesh.Cast, as b3ShapeCastMesh of Box3D); the nodes entered after the best hit are dropped
func (q *sweepQuery) sweepMesh(body *actor.RigidBody, mesh *actor.TriangleMesh) (Hit, bool) {
	local, translation := q.localMotion(body)
	bounds := q.shape.ComputeAABB(local)
	reach := mgl64.Vec3{2 * sweepFlatGap, 2 * sweepFlatGap, 2 * sweepFlatGap}
	cast := mesh.Cast(bounds.Min.Add(bounds.Max).Mul(0.5), translation, bounds.Max.Sub(bounds.Min).Mul(0.5).Add(reach), q.limit)

	best, found := Hit{Fraction: q.limit}, false
	for {
		t, ok := cast.Next(best.Fraction)
		if !ok {
			break
		}
		vertices, _ := mesh.Triangle(t)
		q.sweepLocalTriangle(body, vertices, t, &best, &found)
	}
	return best, found
}

// sweepLocalTriangle: the triangle of the surface, in its local space, taken to world space and swept; the hit is
// kept if it comes first (at the same fraction, the triangle of the lowest index)
func (q *sweepQuery) sweepLocalTriangle(surface *actor.RigidBody, vertices [3]mgl64.Vec3, triangle int32, best *Hit, found *bool) {
	rotation := &surface.Transform.Rotation
	for i := range vertices {
		vertices[i] = surface.Transform.Position.Add(actor.Rotate(rotation, vertices[i]))
	}
	hit, ok := q.sweepTriangle(vertices, best.Fraction)
	hit.Triangle = triangle
	if ok && (!*found || hit.Fraction < best.Fraction || (hit.Fraction == best.Fraction && hit.Triangle < best.Triangle)) {
		*best, *found = hit, true
	}
}

// overlapsHeightfield: a triangle of the cells under the bounds of the shape overlaps it, from any side. A hole has
// no triangle
func (s *queryScratch) overlapsHeightfield(core *gjk.Proxy, radius float64, bounds actor.AABB, terrain *actor.RigidBody, field *actor.Heightfield) bool {
	s.cells = field.OverlapCells(localBounds(terrain.Transform, bounds), s.cells[:0])
	cellsZ := field.ZSamples - 1
	for _, cell := range s.cells {
		for t := 0; t < 2; t++ {
			vertices, _ := field.Triangle(int(cell)/cellsZ, int(cell)%cellsZ, t)
			if s.overlapsTriangle(core, radius, bounds, terrain, vertices) {
				return true
			}
		}
	}
	return false
}

// overlapsMesh: a triangle of the leaves under the bounds of the shape overlaps it, from any side
func (s *queryScratch) overlapsMesh(core *gjk.Proxy, radius float64, bounds actor.AABB, body *actor.RigidBody, mesh *actor.TriangleMesh) bool {
	s.triangles = mesh.OverlapTriangles(localBounds(body.Transform, bounds), s.triangles[:0])
	for _, t := range s.triangles {
		vertices, _ := mesh.Triangle(t)
		if s.overlapsTriangle(core, radius, bounds, body, vertices) {
			return true
		}
	}
	return false
}

// overlapsTriangle: the triangle of the surface, in its local space, overlaps the core of the shape within its radius
func (s *queryScratch) overlapsTriangle(core *gjk.Proxy, radius float64, bounds actor.AABB, surface *actor.RigidBody, vertices [3]mgl64.Vec3) bool {
	for i := range vertices {
		vertices[i] = surface.Transform.ToWorld(vertices[i])
	}
	s.shape.vertices, s.shape.aabb = vertices, triangleAABB(vertices)
	if !s.shape.aabb.Overlaps(bounds) {
		return false
	}
	return coresOverlap(core, radius, &s.triangleProxy, 0)
}
