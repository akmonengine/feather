package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== MEASURES ==========
// Exact measures of the overlaps, independent of the collision detection of the engine

// surfaceDepth: how deep the body is in the static body (plane, terrain or box), 0 outside it
func surfaceDepth(surface, body *actor.RigidBody) float64 {
	switch shape := surface.Shape.(type) {
	case *actor.Plane:
		lowest := body.SupportWorld(shape.Normal.Mul(-1))
		return math.Max(0, -(lowest.Dot(shape.Normal) + shape.Distance))
	case *actor.Heightfield:
		return terrainDepth(shape, surface.Transform, body)
	}
	return math.Max(0, satOverlap(surface, body))
}

// terrainDepth: the deepest point of the body under the terrain, 0 above it
func terrainDepth(field *actor.Heightfield, transform actor.Transform, body *actor.RigidBody) float64 {
	// the distance of a world point to the terrain, negative under it
	distance := func(p mgl64.Vec3) float64 {
		return terrainDistance(field, transform.Rotation.Conjugate().Rotate(p.Sub(transform.Position)))
	}
	depth := 0.0
	switch shape := body.Shape.(type) {
	case *actor.Box:
		for c := 0; c < 8; c++ {
			corner := shape.HalfExtents
			for k := 0; k < 3; k++ {
				if c&(1<<k) != 0 {
					corner[k] = -corner[k]
				}
			}
			depth = math.Max(depth, -distance(body.Transform.ToWorld(corner)))
		}
	case *actor.Sphere:
		depth = shape.Radius - distance(body.Transform.Position)
	case *actor.Capsule:
		// the distance to the terrain is not convex along the segment: sampled every cm
		bottom, top := shape.Segment(body.Transform)
		samples := int(math.Ceil(top.Sub(bottom).Len() / 0.01))
		for k := 0; k <= samples; k++ {
			depth = math.Max(depth, shape.Radius-distance(bottom.Add(top.Sub(bottom).Mul(float64(k)/float64(samples)))))
		}
	}
	return math.Max(0, depth)
}

// satOverlap: the overlap of 2 bodies along the axis separating them best (separating axis theorem), negative when
// they are apart. Exact for 2 boxes (the 15 axes) and for a sphere and a box (the axis from the center of the sphere
// to the closest point of the box is added)
func satOverlap(a, b *actor.RigidBody) float64 {
	var axes [16]mgl64.Vec3
	count := 0
	add := func(axis mgl64.Vec3) {
		if length := axis.Len(); length > 1e-9 {
			axes[count] = axis.Mul(1 / length)
			count++
		}
	}
	directions := func(body *actor.RigidBody) []mgl64.Vec3 {
		if _, ok := body.Shape.(*actor.Box); !ok {
			return nil
		}
		r := body.Transform.Rotation
		return []mgl64.Vec3{r.Rotate(mgl64.Vec3{1, 0, 0}), r.Rotate(mgl64.Vec3{0, 1, 0}), r.Rotate(mgl64.Vec3{0, 0, 1})}
	}
	axesA, axesB := directions(a), directions(b)
	for _, axis := range axesA {
		add(axis)
	}
	for _, axis := range axesB {
		add(axis)
	}
	for _, x := range axesA {
		for _, y := range axesB {
			add(x.Cross(y))
		}
	}
	for _, pair := range [2][2]*actor.RigidBody{{a, b}, {b, a}} {
		if _, ok := pair[0].Shape.(*actor.Sphere); ok {
			add(pair[0].Transform.Position.Sub(closestOnBox(pair[1], pair[0].Transform.Position)))
		}
	}

	overlap := math.Inf(1)
	for _, n := range axes[:count] {
		maxA, minA := a.SupportWorld(n).Dot(n), a.SupportWorld(n.Mul(-1)).Dot(n)
		maxB, minB := b.SupportWorld(n).Dot(n), b.SupportWorld(n.Mul(-1)).Dot(n)
		overlap = math.Min(overlap, math.Min(maxA-minB, maxB-minA))
	}
	return overlap
}

// closestOnBox: the point of the box closest to p
func closestOnBox(body *actor.RigidBody, p mgl64.Vec3) mgl64.Vec3 {
	box, ok := body.Shape.(*actor.Box)
	if !ok {
		return body.Transform.Position
	}
	local := body.Transform.Rotation.Conjugate().Rotate(p.Sub(body.Transform.Position))
	for k := 0; k < 3; k++ {
		local[k] = math.Max(-box.HalfExtents[k], math.Min(box.HalfExtents[k], local[k]))
	}
	return body.Transform.ToWorld(local)
}

// boxOverlap: how much 2 boxes overlap, 0 apart
func boxOverlap(a, b *actor.RigidBody) float64 {
	return math.Max(0, satOverlap(a, b))
}

// terrainDistance: the distance from the point to the triangles, negative under the terrain
func terrainDistance(field *actor.Heightfield, p mgl64.Vec3) float64 {
	best := math.Inf(1)
	around := mgl64.Vec3{1, 100, 1}
	for _, cell := range field.OverlapCells(actor.AABB{Min: p.Sub(around), Max: p.Add(around)}, nil) {
		x, z := int(cell)/(field.ZSamples-1), int(cell)%(field.ZSamples-1)
		for t := 0; t < 2; t++ {
			triangle, _ := field.Triangle(x, z, t)
			best = math.Min(best, p.Sub(closestOnTriangle(p, triangle)).Len())
		}
	}
	if height, ok := field.HeightAt(p.X(), p.Z()); ok && p.Y() < height {
		return -best
	}
	return best
}

// closestOnTriangle: the closest point of the triangle (Ericson 5.1.5)
func closestOnTriangle(p mgl64.Vec3, triangle [3]mgl64.Vec3) mgl64.Vec3 {
	a, b, c := triangle[0], triangle[1], triangle[2]
	ab, ac, ap := b.Sub(a), c.Sub(a), p.Sub(a)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return a
	}
	bp := p.Sub(b)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return b
	}
	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		return a.Add(ab.Mul(d1 / (d1 - d3)))
	}
	cp := p.Sub(c)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return c
	}
	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		return a.Add(ac.Mul(d2 / (d2 - d6)))
	}
	va := d3*d6 - d5*d4
	if va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		return b.Add(c.Sub(b).Mul((d4 - d3) / ((d4 - d3) + (d5 - d6))))
	}
	denominator := 1 / (va + vb + vc)
	return a.Add(ab.Mul(vb * denominator)).Add(ac.Mul(vc * denominator))
}
