package main

import (
	"fmt"
	"math"
	"math/rand"
	"sort"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// h is the support function of the body: max over its points of p·n.
func h(b *actor.RigidBody, n mgl64.Vec3) float64 { return b.SupportWorld(n).Dot(n) }

// depthAlong is how far B must move along n (A→B) to separate.
func depthAlong(a, b *actor.RigidBody, n mgl64.Vec3) float64 { return h(a, n) + h(b, n.Mul(-1)) }

// reference estimates the penetration (minimum over all directions) and its direction,
// by dense sampling then local refinement, for pairs without an exact formula: the
// sampling can miss the minimum, which is why exact() is preferred wherever it exists.
func reference(a, b *actor.RigidBody, seeds []mgl64.Vec3) (float64, mgl64.Vec3) {
	type cand struct {
		d float64
		n mgl64.Vec3
	}
	var cs []cand
	for _, n := range seeds {
		cs = append(cs, cand{depthAlong(a, b, n), n})
	}
	sort.Slice(cs, func(i, j int) bool { return cs[i].d < cs[j].d })
	best, bn := math.Inf(1), mgl64.Vec3{}
	for k := 0; k < len(cs) && k < 4; k++ {
		if d, n := refine(a, b, cs[k].n); d < best {
			best, bn = d, n
		}
	}
	return best, bn
}

func refine(a, b *actor.RigidBody, bn mgl64.Vec3) (float64, mgl64.Vec3) {
	best := depthAlong(a, b, bn)
	step := 0.05
	for step > 1e-10 {
		improved := false
		t1 := bn.Cross(mgl64.Vec3{1, 0, 0})
		if t1.Len() < 0.5 {
			t1 = bn.Cross(mgl64.Vec3{0, 1, 0})
		}
		t1 = t1.Normalize()
		t2 := bn.Cross(t1)
		for _, dir := range []mgl64.Vec3{t1, t1.Mul(-1), t2, t2.Mul(-1), t1.Add(t2).Normalize(), t1.Sub(t2).Normalize(), t2.Sub(t1).Normalize(), t1.Add(t2).Mul(-1).Normalize()} {
			n := bn.Add(dir.Mul(step)).Normalize()
			if d := depthAlong(a, b, n); d < best {
				best, bn, improved = d, n, true
			}
		}
		if !improved {
			step /= 2
		}
	}
	return best, bn
}

func fibonacci(n int) []mgl64.Vec3 {
	var out []mgl64.Vec3
	ga := math.Pi * (3 - math.Sqrt(5))
	for i := 0; i < n; i++ {
		y := 1 - 2*(float64(i)+0.5)/float64(n)
		r := math.Sqrt(1 - y*y)
		out = append(out, mgl64.Vec3{math.Cos(ga*float64(i)) * r, y, math.Sin(ga*float64(i)) * r})
	}
	return out
}

func randQuat(r *rand.Rand) mgl64.Quat {
	u1, u2, u3 := r.Float64(), r.Float64(), r.Float64()
	return mgl64.Quat{W: math.Sqrt(1-u1) * math.Sin(2*math.Pi*u2), V: mgl64.Vec3{math.Sqrt(1-u1) * math.Cos(2*math.Pi*u2), math.Sqrt(u1) * math.Sin(2*math.Pi*u3), math.Sqrt(u1) * math.Cos(2*math.Pi*u3)}}.Normalize()
}

type shapeMaker func(r *rand.Rand) actor.ShapeInterface

func boxMaker(r *rand.Rand) actor.ShapeInterface {
	return &actor.Box{HalfExtents: mgl64.Vec3{0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64()}}
}
func sphereMaker(r *rand.Rand) actor.ShapeInterface {
	return &actor.Sphere{Radius: 0.1 + 0.5*r.Float64()}
}

func pct(v []float64, p float64) float64 {
	if len(v) == 0 {
		return math.NaN()
	}
	s := append([]float64(nil), v...)
	sort.Float64s(s)
	return s[int(math.Min(float64(len(s)-1), p*float64(len(s))))]
}

// exact returns the exact penetration depth and normal (A→B) when a closed form exists:
// SAT over the 15 axes for two boxes, the closest point for a sphere and a box.
func exact(a, b *actor.RigidBody) (float64, mgl64.Vec3, bool) {
	ba, okA := a.Shape.(*actor.Box)
	bb, okB := b.Shape.(*actor.Box)
	if okA && okB {
		axesA := [3]mgl64.Vec3{a.Transform.Rotation.Rotate(mgl64.Vec3{1, 0, 0}), a.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}), a.Transform.Rotation.Rotate(mgl64.Vec3{0, 0, 1})}
		axesB := [3]mgl64.Vec3{b.Transform.Rotation.Rotate(mgl64.Vec3{1, 0, 0}), b.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}), b.Transform.Rotation.Rotate(mgl64.Vec3{0, 0, 1})}
		cands := append([]mgl64.Vec3{}, axesA[:]...)
		cands = append(cands, axesB[:]...)
		for _, x := range axesA {
			for _, y := range axesB {
				if c := x.Cross(y); c.Len() > 1e-9 {
					cands = append(cands, c.Normalize())
				}
			}
		}
		_ = ba
		_ = bb
		best, bn := math.Inf(1), mgl64.Vec3{}
		for _, n := range cands {
			for _, s := range []float64{1, -1} {
				m := n.Mul(s)
				if d := depthAlong(a, b, m); d < best {
					best, bn = d, m
				}
			}
		}
		return best, bn, true
	}
	if d, n, ok := exactCapsuleBox(a, b); ok {
		return d, n, true
	}
	sa, okS := a.Shape.(*actor.Sphere)
	bx, okX := b.Shape.(*actor.Box)
	flip := false
	if !okS || !okX {
		sb, okS2 := b.Shape.(*actor.Sphere)
		ax, okX2 := a.Shape.(*actor.Box)
		if !okS2 || !okX2 {
			return 0, mgl64.Vec3{}, false
		}
		sa, bx, flip = sb, ax, true
		a, b = b, a
	}
	// a: sphere, b: box
	c := b.Transform.Rotation.Conjugate().Rotate(a.Transform.Position.Sub(b.Transform.Position))
	h := bx.HalfExtents
	q := mgl64.Vec3{math.Max(-h[0], math.Min(h[0], c[0])), math.Max(-h[1], math.Min(h[1], c[1])), math.Max(-h[2], math.Min(h[2], c[2]))}
	var depth float64
	var nLocal mgl64.Vec3 // from box to sphere
	if q != c {
		d := c.Sub(q)
		depth = sa.Radius - d.Len()
		nLocal = d.Normalize()
	} else {
		best := math.Inf(1)
		for i := 0; i < 3; i++ {
			for _, s := range []float64{1, -1} {
				dist := h[i] - s*c[i]
				if dist < best {
					best = dist
					nLocal = mgl64.Vec3{}
					nLocal[i] = s
				}
			}
		}
		depth = sa.Radius + best
	}
	n := b.Transform.Rotation.Rotate(nLocal) // box → sphere
	if !flip {
		n = n.Mul(-1) // sphere is A: A→B = sphere → box
	}
	return depth, n, true
}

// accuracy places pairs at a chosen reference depth and compares the narrow phase.
func accuracy(name string, ma, mb shapeMaker, minDepth, maxDepth float64, n int) {
	r := rand.New(rand.NewSource(7))
	seeds := fibonacci(1500)
	var angErr, depErr []float64
	missed, wrongSide := 0, 0
	for i := 0; i < n; i++ {
		sa, sb := ma(r), mb(r)
		qa, qb := randQuat(r), randQuat(r)
		dir := mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize()
		target := minDepth + (maxDepth-minDepth)*r.Float64()
		a := actor.NewRigidBody(tr(mgl64.Vec3{}, qa), sa, actor.BodyTypeDynamic, 1)
		b := actor.NewRigidBody(tr(mgl64.Vec3{}, qb), sb, actor.BodyTypeDynamic, 1)
		b.Transform.Position = dir.Mul(0.3)
		d0, n0 := reference(a, b, seeds[:500])
		b.Transform.Position = b.Transform.Position.Add(n0.Mul(d0 - target))
		a.Shape.ComputeAABB(a.Transform)
		b.Shape.ComputeAABB(b.Transform)
		want, wn := reference(a, b, seeds)
		if d, n, ok := exact(a, b); ok {
			want, wn = d, n
		}
		if want <= 0 {
			continue
		}
		ok, got, depth, _ := narrow(a, b)
		if !ok {
			missed++
			continue
		}
		ang := math.Acos(math.Max(-1, math.Min(1, got.Dot(wn)))) * 180 / math.Pi
		if ang > 90 {
			wrongSide++
		}
		angErr = append(angErr, ang)
		depErr = append(depErr, math.Abs(depth-want)*1000)
	}
	fmt.Printf("%-12s depth %4.1f-%4.0f mm  n=%d  missed=%d wrongSide=%d  normal err° median %.3f p99 %.2f max %.1f | depth err mm median %.3f p99 %.2f max %.1f\n",
		name, minDepth*1000, maxDepth*1000, n, missed, wrongSide, pct(angErr, 0.5), pct(angErr, 0.99), pct(angErr, 1), pct(depErr, 0.5), pct(depErr, 0.99), pct(depErr, 1))
}

func epaAccuracy() {
	accuracy("box-box", boxMaker, boxMaker, 0.0001, 0.02, 200)
	accuracy("box-box", boxMaker, boxMaker, 0.02, 0.2, 200)
	accuracy("sphere-box", sphereMaker, boxMaker, 0.0001, 0.02, 200)
	accuracy("box-sphere", boxMaker, sphereMaker, 0.0001, 0.02, 200)
	accuracy("sphere-sphr", sphereMaker, sphereMaker, 0.0001, 0.02, 100)
	if capsuleMaker != nil {
		accuracy("capsule-box", capsuleMaker, boxMaker, 0.0001, 0.02, 200)
		accuracy("box-capsule", boxMaker, capsuleMaker, 0.0001, 0.02, 200)
		accuracy("capsule-box", capsuleMaker, boxMaker, 0.02, 0.2, 200)
	}
}
