//go:build !v020

package main

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// exactCapsuleBox: PD(capsule, box) = r + PD(segment, box) when the segment enters the box
// (SAT over the box normals and segment × box axes, exact for these polytopes); otherwise
// r - distance(segment, box), a convex problem solved by local refinement.
func exactCapsuleBox(a, b *actor.RigidBody) (float64, mgl64.Vec3, bool) {
	flip := false
	c, okC := a.Shape.(*actor.Capsule)
	_, okB := b.Shape.(*actor.Box)
	if !okC || !okB {
		c2, okC2 := b.Shape.(*actor.Capsule)
		_, okB2 := a.Shape.(*actor.Box)
		if !okC2 || !okB2 {
			return 0, mgl64.Vec3{}, false
		}
		c, flip = c2, true
		a, b = b, a
	}
	// a: capsule, b: box. Segment support: h_seg(n) = h_capsule(n) - r.
	axes := [3]mgl64.Vec3{b.Transform.Rotation.Rotate(mgl64.Vec3{1, 0, 0}), b.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}), b.Transform.Rotation.Rotate(mgl64.Vec3{0, 0, 1})}
	d := a.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0})
	cands := axes[:]
	for _, x := range axes {
		if cr := d.Cross(x); cr.Len() > 1e-9 {
			cands = append(cands, cr.Normalize())
		}
	}
	segDepth := func(n mgl64.Vec3) float64 { return depthAlong(a, b, n) - c.Radius }
	best, bn := math.Inf(1), mgl64.Vec3{}
	for _, n := range cands {
		for _, s := range []float64{1, -1} {
			if v := segDepth(n.Mul(s)); v < best {
				best, bn = v, n.Mul(s)
			}
		}
	}
	if best < 0 {
		// Segment outside the box: exact distance, minimised along the segment (convex).
		box := b.Shape.(*actor.Box)
		p0 := a.Transform.Position.Sub(d.Mul(c.HalfHeight))
		p1 := a.Transform.Position.Add(d.Mul(c.HalfHeight))
		closest := func(t float64) (mgl64.Vec3, mgl64.Vec3) {
			p := p0.Add(p1.Sub(p0).Mul(t))
			l := b.Transform.Rotation.Conjugate().Rotate(p.Sub(b.Transform.Position))
			h := box.HalfExtents
			q := mgl64.Vec3{math.Max(-h[0], math.Min(h[0], l[0])), math.Max(-h[1], math.Min(h[1], l[1])), math.Max(-h[2], math.Min(h[2], l[2]))}
			return p, b.Transform.Position.Add(b.Transform.Rotation.Rotate(q))
		}
		dist := func(t float64) float64 { p, q := closest(t); return p.Sub(q).Len() }
		lo, hi := 0.0, 1.0
		for k := 0; k < 200; k++ {
			m1, m2 := lo+(hi-lo)/3, hi-(hi-lo)/3
			if dist(m1) < dist(m2) {
				hi = m2
			} else {
				lo = m1
			}
		}
		p, q := closest((lo + hi) / 2)
		best, bn = -p.Sub(q).Len(), q.Sub(p).Normalize()
	}
	depth := best + c.Radius
	if flip {
		bn = bn.Mul(-1)
	}
	return depth, bn, true
}
