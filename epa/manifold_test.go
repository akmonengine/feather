package epa

import (
	"math"
	"math/rand"
	"sort"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

func manifold(t *testing.T, a, b *actor.RigidBody, margin float64) constraint.Manifold {
	t.Helper()
	result := runEPA(t, a, b, margin)
	var m constraint.Manifold
	Manifold(a, b, result, margin, &m)
	return m
}

func sortedPositions(m constraint.Manifold) []mgl64.Vec3 {
	points := make([]mgl64.Vec3, m.Count)
	for i := 0; i < m.Count; i++ {
		points[i] = m.Points[i].Position
	}
	sort.Slice(points, func(i, j int) bool {
		if points[i].X() != points[j].X() {
			return points[i].X() < points[j].X()
		}
		return points[i].Z() < points[j].Z()
	})
	return points
}

func unitBox(position mgl64.Vec3, rotation mgl64.Quat) *actor.RigidBody {
	return body(position, rotation, &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}})
}

// A box resting flat on a larger box: four points at its corners, halfway into the overlap.
func TestManifoldFaceOnFace(t *testing.T) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}})
	box := unitBox(mgl64.Vec3{0.3, 0.99, -0.4}, mgl64.QuatIdent())
	m := manifold(t, ground, box, 0)
	if m.Count != 4 || !near(m.Normal, mgl64.Vec3{0, 1, 0}, 1e-9) {
		t.Fatalf("manifold %+v, want 4 points along +Y", m)
	}
	want := []mgl64.Vec3{{-0.2, 0.495, -0.9}, {-0.2, 0.495, 0.1}, {0.8, 0.495, -0.9}, {0.8, 0.495, 0.1}}
	for i, p := range sortedPositions(m) {
		if !near(p, want[i], 1e-9) {
			t.Errorf("point %d = %v, want %v", i, p, want[i])
		}
	}
	for i := 0; i < m.Count; i++ {
		if math.Abs(m.Points[i].Separation+0.01) > 1e-9 {
			t.Errorf("separation %d = %.9f, want -0.01", i, m.Points[i].Separation)
		}
	}
}

// A box hanging over the edge of its support: the points are clipped to the support face.
func TestManifoldClippedToReference(t *testing.T) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 0.5, 1}})
	box := unitBox(mgl64.Vec3{0.8, 0.99, 0}, mgl64.QuatIdent())
	m := manifold(t, ground, box, 0)
	if m.Count != 4 {
		t.Fatalf("got %d points, want 4", m.Count)
	}
	for i := 0; i < m.Count; i++ {
		if x := m.Points[i].Position.X(); x < 0.3-1e-9 || x > 1+1e-9 {
			t.Errorf("point %v outside the support face (x in [0.3, 1])", m.Points[i].Position)
		}
	}
}

// A slightly tilted box: each corner gets its own separation (v0.2.0 gave every point the
// same depth, which put the wrong torque on the box).
func TestManifoldPerPointSeparation(t *testing.T) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}})
	tilt := mgl64.QuatRotate(0.02, mgl64.Vec3{0, 0, 1})
	box := unitBox(mgl64.Vec3{0, 1, 0}, tilt)
	// Lowest corner of the box, to sink it by 5 mm.
	lowest := math.Inf(1)
	for _, c := range [4]mgl64.Vec3{{-0.5, -0.5, 0}, {0.5, -0.5, 0}} {
		lowest = math.Min(lowest, box.Transform.ToWorld(c).Y())
	}
	box.Transform.Position = box.Transform.Position.Add(mgl64.Vec3{0, 0.5 - lowest - 0.005, 0})

	m := manifold(t, ground, box, 0.02)
	if m.Count != 4 {
		t.Fatalf("got %d points, want 4", m.Count)
	}
	for i := 0; i < m.Count; i++ {
		p := m.Points[i]
		// The corner lies at p + separation/2 above the ground face y=0.5.
		corner := p.Position.Y() + p.Separation/2
		if math.Abs(corner-0.5-p.Separation) > 1e-6 {
			t.Errorf("point %d: separation %.6f but corner %.6f above the face", i, p.Separation, corner-0.5)
		}
	}
	if min := m.MinSeparation(); math.Abs(min+0.005) > 1e-6 {
		t.Errorf("deepest separation %.6f, want -0.005", min)
	}
}

// Two boxes a few millimetres apart, within the margin: speculative points with a positive
// separation.
func TestManifoldSpeculative(t *testing.T) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}})
	box := unitBox(mgl64.Vec3{0, 1.005, 0}, mgl64.QuatIdent())
	m := manifold(t, ground, box, 0.02)
	if m.Count != 4 {
		t.Fatalf("got %d points, want 4", m.Count)
	}
	for i := 0; i < m.Count; i++ {
		if math.Abs(m.Points[i].Separation-0.005) > 1e-6 {
			t.Errorf("separation %.6f, want +0.005", m.Points[i].Separation)
		}
	}
}

// A box balanced on one of its edges: the contact is that edge, two points.
func TestManifoldEdgeOnFace(t *testing.T) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}})
	box := unitBox(mgl64.Vec3{0, 0.5 + math.Sqrt2/2 - 0.01, 0}, mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 0, 1}))
	m := manifold(t, ground, box, 0)
	if m.Count != 2 {
		t.Fatalf("got %d points %+v, want the 2 ends of the edge", m.Count, m.Points[:m.Count])
	}
	zs := []float64{m.Points[0].Position.Z(), m.Points[1].Position.Z()}
	sort.Float64s(zs)
	if math.Abs(zs[0]+0.5) > 1e-6 || math.Abs(zs[1]-0.5) > 1e-6 || math.Abs(m.MinSeparation()+0.01) > 1e-6 {
		t.Errorf("edge points z=%v separation %.6f, want z=±0.5 and -0.01", zs, m.MinSeparation())
	}
}

// Two boxes crossing edge to edge: no face is involved, the contact is the single witness
// point on the edges.
func TestManifoldEdgeOnEdge(t *testing.T) {
	a := unitBox(mgl64.Vec3{}, mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 0, 1}))
	b := unitBox(mgl64.Vec3{0, math.Sqrt2 - 0.01, 0}, mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{1, 0, 0}))
	m := manifold(t, a, b, 0)
	if m.Count != 1 {
		t.Fatalf("got %d points, want 1", m.Count)
	}
	if !near(m.Points[0].Position, mgl64.Vec3{0, math.Sqrt2/2 - 0.005, 0}, 1e-5) || math.Abs(m.Points[0].Separation+0.01) > 1e-6 {
		t.Errorf("point %+v, want (0, %.4f, 0) separation -0.01", m.Points[0], math.Sqrt2/2-0.005)
	}
}

// Two squares turned by 45°: their overlap is an octagon, reduced to four points that keep
// the deepest one and span most of the area.
func TestManifoldReducedToFour(t *testing.T) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}})
	box := body(mgl64.Vec3{0, 0.99, 0}, mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0}), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}})
	m := manifold(t, ground, box, 0)
	if m.Count != 4 {
		t.Fatalf("got %d points, want 4", m.Count)
	}
	// Area of the kept quadrilateral against the octagon's (2(√2-1)... = 0.828 for unit squares).
	pts := sortedByAngle(m)
	area := 0.0
	for i := range pts {
		j := (i + 1) % len(pts)
		area += pts[i].X()*pts[j].Z() - pts[j].X()*pts[i].Z()
	}
	area = math.Abs(area) / 2
	octagon := 2 * (math.Sqrt2 - 1)
	if area < 0.6*octagon {
		t.Errorf("kept area %.3f of the %.3f octagon", area, octagon)
	}
}

// Random points reduced to 4 at most (3 if the others are inside their triangle): the deepest is always kept
func TestReduceKeepsDeepest(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	normal := mgl64.Vec3{0, 1, 0}
	for i := 0; i < 500; i++ {
		points := make([]constraint.ContactPoint, 5+r.Intn(12))
		deepest := math.Inf(1)
		for k := range points {
			points[k] = constraint.ContactPoint{Position: mgl64.Vec3{r.Float64() - 0.5, 0, r.Float64() - 0.5}, Separation: r.Float64()*0.1 - 0.05}
			deepest = math.Min(deepest, points[k].Separation)
		}
		var m constraint.Manifold
		Reduce(points, normal, &m)
		if m.Count < 3 || m.MinSeparation() != deepest {
			t.Fatalf("%d points kept, the deepest at %.4f, want %.4f", m.Count, m.MinSeparation(), deepest)
		}
	}
}

func sortedByAngle(m constraint.Manifold) []mgl64.Vec3 {
	pts := sortedPositions(m)
	center := mgl64.Vec3{}
	for _, p := range pts {
		center = center.Add(p)
	}
	center = center.Mul(1 / float64(len(pts)))
	sort.Slice(pts, func(i, j int) bool {
		return math.Atan2(pts[i].Z()-center.Z(), pts[i].X()-center.X()) < math.Atan2(pts[j].Z()-center.Z(), pts[j].X()-center.X())
	})
	return pts
}

// A capsule lying on a box face: its side line, clipped to the face.
func TestManifoldCapsuleOnFace(t *testing.T) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 0.5, 1}})
	capsule := body(mgl64.Vec3{0.5, 0.79, 0}, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1}), &actor.Capsule{HalfHeight: 1, Radius: 0.3})
	m := manifold(t, ground, capsule, 0)
	if m.Count != 2 {
		t.Fatalf("got %d points, want 2", m.Count)
	}
	xs := []float64{m.Points[0].Position.X(), m.Points[1].Position.X()}
	sort.Float64s(xs)
	// Capsule line x in [-0.5, 1.5], face x in [-1, 1]: clipped to [-0.5, 1].
	if math.Abs(xs[0]+0.5) > 1e-6 || math.Abs(xs[1]-1) > 1e-6 {
		t.Errorf("points x=%v, want [-0.5 1]", xs)
	}
}

func TestClipAgainstPlane(t *testing.T) {
	square := polygon{points: [maxBufferSize]mgl64.Vec3{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, count: 4}
	var out polygon
	clipAgainstPlane(&square, mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 0, 0}, &out)
	if out.count != 4 {
		t.Fatalf("half square: %d points, want 4", out.count)
	}
	for i := 0; i < out.count; i++ {
		if out.points[i].X() < -1e-12 {
			t.Errorf("point %v kept on the wrong side", out.points[i])
		}
	}

	segment := polygon{points: [maxBufferSize]mgl64.Vec3{{-1, 0, 0}, {1, 0, 0}}, count: 2}
	clipAgainstPlane(&segment, mgl64.Vec3{0.5, 0, 0}, mgl64.Vec3{-1, 0, 0}, &out)
	if out.count != 2 || !near(out.points[0], mgl64.Vec3{-1, 0, 0}, 1e-12) || !near(out.points[1], mgl64.Vec3{0.5, 0, 0}, 1e-12) {
		t.Errorf("clipped segment %v, want [-1, 0.5] once (no duplicate)", out.points[:out.count])
	}
}

func BenchmarkManifoldBoxBox(b *testing.B) {
	ground := body(mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}})
	box := unitBox(mgl64.Vec3{0.3, 0.99, -0.4}, mgl64.QuatRotate(0.3, mgl64.Vec3{0, 1, 0}))
	simplex := &gjk.Simplex{}
	gjk.GJKMargin(ground, box, 0.02, simplex)
	result, _ := EPA(ground, box, simplex, 0.02)
	var m constraint.Manifold
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		Manifold(ground, box, result, 0.02, &m)
	}
}

// The reference face is whichever face lies flat against the normal, A's or B's.
func TestChooseReference(t *testing.T) {
	flat := polygon{points: [maxBufferSize]mgl64.Vec3{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, count: 4}
	tilted := polygon{points: [maxBufferSize]mgl64.Vec3{{-1, -0.1, -1}, {1, 0.1, -1}, {1, 0.1, 1}, {-1, -0.1, 1}}, count: 4}
	segment := polygon{points: [maxBufferSize]mgl64.Vec3{{-1, 0, 0}, {1, 0, 0}}, count: 2}
	up := mgl64.Vec3{0, 1, 0}
	for _, tc := range []struct {
		name       string
		a, b       *polygon
		isA, found bool
	}{
		{"both flat: A", &flat, &flat, true, true},
		{"only B flat", &tilted, &flat, false, true},
		{"only A flat", &flat, &tilted, true, true},
		{"segment on a face", &segment, &flat, false, true},
		{"no face in contact", &tilted, &segment, false, false},
	} {
		isA, found := chooseReference(tc.a, tc.b, up)
		if found != tc.found || (found && isA != tc.isA) {
			t.Errorf("%s: reference A=%v found=%v, want A=%v found=%v", tc.name, isA, found, tc.isA, tc.found)
		}
	}
}

// A capsule lying along a box edge touches it along a line: two points, not one.
func TestManifoldCapsuleAlongEdge(t *testing.T) {
	box := unitBox(mgl64.Vec3{}, mgl64.QuatIdent())
	direction := mgl64.Vec3{1, 1, 0}.Normalize()
	center := mgl64.Vec3{0.5, 0.5, 0}.Add(direction.Mul(0.29))
	capsule := body(center, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{1, 0, 0}), &actor.Capsule{HalfHeight: 1, Radius: 0.3})
	m := manifold(t, box, capsule, 0)
	if m.Count != 2 {
		t.Fatalf("got %d points, want the edge z in [-0.5, 0.5]", m.Count)
	}
	for i := 0; i < m.Count; i++ {
		if math.Abs(math.Abs(m.Points[i].Position.Z())-0.5) > 1e-6 || math.Abs(m.Points[i].Separation+0.01) > 1e-6 {
			t.Errorf("point %+v, want z=±0.5 separation -0.01", m.Points[i])
		}
	}
}
