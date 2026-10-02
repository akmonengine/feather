package feather

import (
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== QUERIES ON A HEIGHTFIELD ==========

// sweepHeightfield: the moving shape against the triangles of the cells under its path, in the order of its motion
// (Heightfield.WalkCells, as b3ShapeCastHeightField of Box3D): a band of the width of the shape, not the whole AABB
// of the motion. The walk ends when the shape enters the next cells after its best hit
func (q *sweepQuery) sweepHeightfield(terrain *actor.RigidBody, field *actor.Heightfield) (Hit, bool) {
	// the motion in the local space of the terrain: the AABB of the shape there, moved along the translation
	rotation := &terrain.Transform.Rotation
	conjugate := mgl64.Quat{W: rotation.W, V: rotation.V.Mul(-1)}
	local := actor.Transform{
		Position: actor.RotateInverse(rotation, q.start.Position.Sub(terrain.Transform.Position)),
		Rotation: actor.MulQuat(&conjugate, &q.start.Rotation),
	}
	bounds := q.shape.ComputeAABB(local)
	reach := mgl64.Vec3{2 * sweepFlatGap, 2 * sweepFlatGap, 2 * sweepFlatGap}
	walk := field.WalkCells(bounds.Min.Add(bounds.Max).Mul(0.5), actor.RotateInverse(rotation, q.translation), bounds.Max.Sub(bounds.Min).Mul(0.5).Add(reach), q.limit)

	best, found := Hit{Fraction: q.limit}, false
	cellsZ := field.ZSamples - 1
	for {
		x, z, ok := walk.Next(best.Fraction)
		if !ok {
			break
		}
		for t := 0; t < 2; t++ {
			vertices, _ := field.Triangle(x, z, t)
			for i := range vertices {
				vertices[i] = terrain.Transform.Position.Add(actor.Rotate(rotation, vertices[i]))
			}
			hit, ok := q.sweepTriangle(vertices, best.Fraction)
			hit.Triangle = int32(2*(x*cellsZ+z) + t)
			// at the same fraction, the triangle of the lowest index
			if ok && (!found || hit.Fraction < best.Fraction || (hit.Fraction == best.Fraction && hit.Triangle < best.Triangle)) {
				best, found = hit, true
			}
		}
	}
	return best, found
}

// overlapsHeightfield: a triangle of the cells under the bounds of the shape overlaps it, from any side. A hole has
// no triangle
func (s *queryScratch) overlapsHeightfield(core *gjk.Proxy, radius float64, bounds actor.AABB, terrain *actor.RigidBody, field *actor.Heightfield) bool {
	s.cells = field.OverlapCells(localBounds(terrain.Transform, bounds), s.cells[:0])
	cellsZ := field.ZSamples - 1
	for _, cell := range s.cells {
		for t := 0; t < 2; t++ {
			vertices, _ := field.Triangle(int(cell)/cellsZ, int(cell)%cellsZ, t)
			for i := range vertices {
				vertices[i] = terrain.Transform.ToWorld(vertices[i])
			}
			s.shape.vertices, s.shape.aabb = vertices, triangleAABB(vertices)
			if !s.shape.aabb.Overlaps(bounds) {
				continue
			}
			proxy := gjk.NewProxyAt(s.triangle.Transform, &s.shape)
			if coresOverlap(core, radius, &proxy, 0) {
				return true
			}
		}
	}
	return false
}
