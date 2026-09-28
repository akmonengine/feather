package epa

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

func body(position mgl64.Vec3, rotation mgl64.Quat, shape actor.ShapeInterface) *actor.RigidBody {
	return actor.NewRigidBody(actor.Transform{Position: position, Rotation: rotation}, shape, actor.BodyTypeDynamic, 1)
}

func randomRotation(r *rand.Rand) mgl64.Quat {
	u1, u2, u3 := r.Float64(), r.Float64(), r.Float64()
	return mgl64.Quat{W: math.Sqrt(1-u1) * math.Sin(2*math.Pi*u2), V: mgl64.Vec3{
		math.Sqrt(1-u1) * math.Cos(2*math.Pi*u2), math.Sqrt(u1) * math.Sin(2*math.Pi*u3), math.Sqrt(u1) * math.Cos(2*math.Pi*u3),
	}}.Normalize()
}

// depthAlong is how far B must move along n to leave A: h_A(n) + h_B(-n).
func depthAlong(a, b *actor.RigidBody, n mgl64.Vec3) float64 {
	return a.SupportWorld(n).Dot(n) - b.SupportWorld(n.Mul(-1)).Dot(n)
}

// satBoxBox is the exact penetration of two boxes: the minimum over the 15 separating axes
// (the face normals and edge cross products span every face of their Minkowski difference).
func satBoxBox(a, b *actor.RigidBody) (float64, mgl64.Vec3) {
	var axes []mgl64.Vec3
	var edgesA, edgesB [3]mgl64.Vec3
	for i := 0; i < 3; i++ {
		var e mgl64.Vec3
		e[i] = 1
		edgesA[i] = a.Transform.Rotation.Rotate(e)
		edgesB[i] = b.Transform.Rotation.Rotate(e)
		axes = append(axes, edgesA[i], edgesB[i])
	}
	for _, x := range edgesA {
		for _, y := range edgesB {
			if c := x.Cross(y); c.Len() > 1e-9 {
				axes = append(axes, c.Normalize())
			}
		}
	}
	best, normal := math.Inf(1), mgl64.Vec3{}
	for _, axis := range axes {
		for _, n := range [2]mgl64.Vec3{axis, axis.Mul(-1)} {
			if d := depthAlong(a, b, n); d < best {
				best, normal = d, n
			}
		}
	}
	return best, normal
}

// closestSphereBox is the exact penetration of a sphere (A) into a box (B).
func closestSphereBox(sphere, box *actor.RigidBody) (float64, mgl64.Vec3) {
	radius := sphere.Shape.(*actor.Sphere).Radius
	h := box.Shape.(*actor.Box).HalfExtents
	c := box.Transform.ToLocal(sphere.Transform.Position)
	q := mgl64.Vec3{math.Max(-h[0], math.Min(h[0], c[0])), math.Max(-h[1], math.Min(h[1], c[1])), math.Max(-h[2], math.Min(h[2], c[2]))}
	var depth float64
	var outward mgl64.Vec3 // from the box towards the sphere, local
	if q != c {
		depth, outward = radius-c.Sub(q).Len(), c.Sub(q).Normalize()
	} else {
		best := math.Inf(1)
		for i := 0; i < 3; i++ {
			for _, s := range [2]float64{1, -1} {
				if d := h[i] - s*c[i]; d < best {
					best = d
					outward = mgl64.Vec3{}
					outward[i] = s
				}
			}
		}
		depth = radius + best
	}
	return depth, box.Transform.Rotation.Rotate(outward).Mul(-1)
}

// place moves b along a random direction until the exact penetration is target.
func place(a, b *actor.RigidBody, direction mgl64.Vec3, target float64, exact func(a, b *actor.RigidBody) (float64, mgl64.Vec3)) bool {
	lo, hi := 0.0, 5.0
	for i := 0; i < 80; i++ {
		mid := (lo + hi) / 2
		b.Transform.Position = direction.Mul(mid)
		if d, _ := exact(a, b); d > target {
			lo = mid
		} else {
			hi = mid
		}
	}
	b.Transform.Position = direction.Mul(hi)
	d, _ := exact(a, b)
	return d > 0
}

func runEPA(t *testing.T, a, b *actor.RigidBody, margin float64) Result {
	t.Helper()
	simplex := &gjk.Simplex{}
	if !gjk.GJKMargin(a, b, margin, simplex) {
		t.Fatalf("GJK found no overlap: a=%v b=%v", a.Transform, b.Transform)
	}
	result, err := EPA(a, b, simplex, margin)
	if err != nil {
		t.Fatalf("EPA: %v", err)
	}
	return result
}

// EPA is exact against SAT on random box pairs, shallow and deep. v0.2.0 was off by up to
// 0.3 mm on the depth.
func TestEPABoxBoxMatchesSAT(t *testing.T) {
	r := rand.New(rand.NewSource(11))
	for i := 0; i < 400; i++ {
		a := body(mgl64.Vec3{}, randomRotation(r), &actor.Box{HalfExtents: mgl64.Vec3{0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64()}})
		b := body(mgl64.Vec3{}, randomRotation(r), &actor.Box{HalfExtents: mgl64.Vec3{0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64()}})
		direction := mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize()
		target := 1e-4 + 0.2*r.Float64()*r.Float64()
		if !place(a, b, direction, target, satBoxBox) {
			continue
		}
		want, wantNormal := satBoxBox(a, b)
		got := runEPA(t, a, b, 0)
		if math.Abs(got.Depth-want) > 1e-6 {
			t.Fatalf("pair %d: depth %.9f, SAT %.9f", i, got.Depth, want)
		}
		// Depth along EPA's normal is the depth itself: the normal separates the boxes.
		if d := depthAlong(a, b, got.Normal); math.Abs(d-want) > 1e-6 {
			t.Fatalf("pair %d: moving B by %.9f along %v leaves %.2e of overlap (SAT normal %v)", i, got.Depth, got.Normal, d-want, wantNormal)
		}
	}
}

// Sphere against box is exact too (v0.2.0: up to 2.9° and 0.9 mm off).
func TestEPASphereBoxMatchesClosestPoint(t *testing.T) {
	r := rand.New(rand.NewSource(12))
	for i := 0; i < 400; i++ {
		sphere := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1 + 0.5*r.Float64()})
		box := body(mgl64.Vec3{}, randomRotation(r), &actor.Box{HalfExtents: mgl64.Vec3{0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64()}})
		direction := mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize()
		if !place(sphere, box, direction, 1e-4+0.02*r.Float64(), closestSphereBox) {
			continue
		}
		want, wantNormal := closestSphereBox(sphere, box)
		got := runEPA(t, sphere, box, 0)
		if math.Abs(got.Depth-want) > 1e-6 {
			t.Fatalf("pair %d: depth %.9f, want %.9f", i, got.Depth, want)
		}
		if angle := math.Acos(math.Min(1, got.Normal.Dot(wantNormal))) * 180 / math.Pi; angle > 0.1 {
			t.Fatalf("pair %d: normal %.4f° off", i, angle)
		}
	}
}

// Capsule against box (a segment against a box, rounded by the radius) goes through EPA:
// the depth along its normal is the penetration, within the convergence tolerance.
func TestEPACapsuleBoxIsMinimal(t *testing.T) {
	r := rand.New(rand.NewSource(13))
	for i := 0; i < 300; i++ {
		capsule := body(mgl64.Vec3{}, randomRotation(r), &actor.Capsule{HalfHeight: 0.05 + 0.5*r.Float64(), Radius: 0.05 + 0.3*r.Float64()})
		box := body(mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Mul(0.3), randomRotation(r), &actor.Box{HalfExtents: mgl64.Vec3{0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64()}})
		simplex := &gjk.Simplex{}
		if !gjk.GJK(capsule, box, simplex) {
			continue
		}
		got, err := EPA(capsule, box, simplex, 0)
		if err != nil {
			t.Fatal(err)
		}
		if d := depthAlong(capsule, box, got.Normal); math.Abs(d-got.Depth) > 1e-6 {
			t.Fatalf("pair %d: depth %.9f but %.9f along its normal", i, got.Depth, d)
		}
		// No direction separates them with less: sample around the normal.
		for k := 0; k < 64; k++ {
			n := got.Normal.Add(mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Mul(0.2)).Normalize()
			if d := depthAlong(capsule, box, n); d < got.Depth-1e-6 {
				t.Fatalf("pair %d: direction %v separates with %.9f < EPA %.9f", i, n, d, got.Depth)
			}
		}
	}
}

// The witness points are the deepest points of each shape: WitnessA - WitnessB = depth·n,
// WitnessA on A's surface along n, WitnessB on B's surface along -n.
func TestEPAWitnessPoints(t *testing.T) {
	a := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}})
	b := body(mgl64.Vec3{0.3, 1.9, -0.2}, mgl64.QuatIdent(), &actor.Sphere{Radius: 1})
	got := runEPA(t, a, b, 0)
	if !near(got.Normal, mgl64.Vec3{0, 1, 0}, 1e-6) || math.Abs(got.Depth-0.1) > 1e-6 {
		t.Fatalf("normal %v depth %f, want +Y 0.1", got.Normal, got.Depth)
	}
	// On a curved surface the witness converges like sqrt(tolerance): a few micrometres.
	if !near(got.WitnessB, mgl64.Vec3{0.3, 0.9, -0.2}, 1e-5) {
		t.Errorf("witness on the sphere %v, want its lowest point (0.3, 0.9, -0.2)", got.WitnessB)
	}
	if d := got.WitnessA.Sub(got.WitnessB).Sub(got.Normal.Mul(got.Depth)).Len(); d > 1e-6 {
		t.Errorf("WitnessA - WitnessB is %.2e from depth·normal", d)
	}
}

// With a margin, shapes up to margin apart overlap: depth = margin - distance.
func TestEPAMargin(t *testing.T) {
	a := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}})
	b := body(mgl64.Vec3{0, 2.005, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}})
	simplex := &gjk.Simplex{}
	if gjk.GJK(a, b, simplex) {
		t.Fatal("boxes 5 mm apart overlap")
	}
	got := runEPA(t, a, b, 0.02)
	if math.Abs(got.Depth-0.015) > 1e-6 || !near(got.Normal, mgl64.Vec3{0, 1, 0}, 1e-6) {
		t.Errorf("depth %f normal %v, want 0.015 along +Y", got.Depth, got.Normal)
	}
}

// Exactly touching shapes (the origin on the Minkowski boundary) still produce a result.
func TestEPATouching(t *testing.T) {
	a := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}})
	b := body(mgl64.Vec3{0.5, 2, 0.5}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}})
	got := runEPA(t, a, b, 0.01)
	if math.Abs(got.Depth-0.01) > 1e-6 {
		t.Errorf("touching boxes with a 1 cm margin: depth %f, want 0.01", got.Depth)
	}
}

func TestEPARejectsIncompleteSimplex(t *testing.T) {
	a := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: 1})
	simplex := &gjk.Simplex{Count: 2}
	if _, err := EPA(a, a, simplex, 0); err == nil {
		t.Error("EPA accepted a 2-point simplex")
	}
}

func TestBarycentric(t *testing.T) {
	a, b, c := mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 0, 0}, mgl64.Vec3{0, 1, 0}
	for _, tc := range []struct {
		p       mgl64.Vec3
		u, v, w float64
	}{
		{mgl64.Vec3{0, 0, 0}, 1, 0, 0},
		{mgl64.Vec3{0.25, 0.25, 0}, 0.5, 0.25, 0.25},
		{mgl64.Vec3{2, 0, 0}, 0, 1, 0},   // clamped outside
		{mgl64.Vec3{-1, -1, 0}, 1, 0, 0}, // clamped outside
	} {
		u, v, w := barycentric(tc.p, a, b, c)
		if math.Abs(u-tc.u) > 1e-12 || math.Abs(v-tc.v) > 1e-12 || math.Abs(w-tc.w) > 1e-12 {
			t.Errorf("barycentric(%v) = %v %v %v, want %v %v %v", tc.p, u, v, w, tc.u, tc.v, tc.w)
		}
	}
}

func BenchmarkEPABoxBox(b *testing.B) {
	boxA := body(mgl64.Vec3{}, mgl64.QuatRotate(0.3, mgl64.Vec3{1, 1, 0}.Normalize()), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}})
	boxB := body(mgl64.Vec3{0.2, 0.9, 0.1}, mgl64.QuatRotate(0.7, mgl64.Vec3{0, 1, 1}.Normalize()), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}})
	simplex := &gjk.Simplex{}
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		simplex.Reset()
		if gjk.GJKMargin(boxA, boxB, 0.02, simplex) {
			_, _ = EPA(boxA, boxB, simplex, 0.02)
		}
	}
}

func near(a, b mgl64.Vec3, tolerance float64) bool { return a.Sub(b).Len() <= tolerance }
