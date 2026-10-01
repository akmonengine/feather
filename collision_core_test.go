package feather

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// closestSphereBox: the exact contact of a sphere with a box whose center is outside the box, the separation and the
// normal from the sphere towards the box
func closestSphereBox(sphere, box *actor.RigidBody) (float64, mgl64.Vec3, bool) {
	radius := sphere.Shape.(*actor.Sphere).Radius
	h := box.Shape.(*actor.Box).HalfExtents
	c := box.Transform.ToLocal(sphere.Transform.Position)
	q := mgl64.Vec3{math.Max(-h[0], math.Min(h[0], c[0])), math.Max(-h[1], math.Min(h[1], c[1])), math.Max(-h[2], math.Min(h[2], c[2]))}
	if q == c {
		return 0, mgl64.Vec3{}, false
	}
	return c.Sub(q).Len() - radius, box.Transform.Rotation.Rotate(q.Sub(c).Normalize()), true
}

// A sphere against a box is its center (a core) with a radius: the separation and the normal are those of the closest
// point of the box to the center, exact to the rounding, on the faces, the edges and the corners, within the margin
func TestSphereBoxIsExact(t *testing.T) {
	r := rand.New(rand.NewSource(12))
	var m constraint.Manifold
	tested := 0
	for i := 0; i < 400; i++ {
		box := actor.NewRigidBody(actor.Transform{Rotation: mgl64.QuatRotate(r.Float64()*math.Pi, mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize())},
			&actor.Box{HalfExtents: mgl64.Vec3{0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64(), 0.1 + 0.5*r.Float64()}}, actor.BodyTypeDynamic, 1)
		direction := mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize()
		sphere := createSphere(direction.Mul(0.5+0.6*r.Float64()), 0.1+0.3*r.Float64(), actor.BodyTypeDynamic)
		want, wantNormal, outside := closestSphereBox(sphere, box)
		if !outside || want > SpeculativeDistance {
			continue
		}
		tested++
		if !Collide(sphere, box, SpeculativeDistance, &m) || m.Count != 1 {
			t.Fatalf("pair %d: no contact, separation %.4f", i, want)
		}
		if math.Abs(m.Points[0].Separation-want) > 1e-9 {
			t.Fatalf("pair %d: separation %.12f, want %.12f", i, m.Points[0].Separation, want)
		}
		if m.Normal.Sub(wantNormal).Len() > 1e-9 {
			t.Fatalf("pair %d: normal %v, want %v", i, m.Normal, wantNormal)
		}
	}
	if tested < 100 {
		t.Fatalf("only %d pairs tested", tested)
	}
}

// The center of the sphere inside the box: the cores overlap, EPA gives the shallowest way out
func TestSphereInsideBox(t *testing.T) {
	box := createBox(mgl64.Vec3{}, mgl64.Vec3{1, 0.5, 1}, actor.BodyTypeStatic)
	sphere := createSphere(mgl64.Vec3{0.2, 0.3, -0.1}, 0.25, actor.BodyTypeDynamic)
	var m constraint.Manifold
	if !Collide(sphere, box, 0, &m) {
		t.Fatal("no contact")
	}
	if want := -(0.5 - 0.3 + 0.25); math.Abs(m.Points[0].Separation-want) > 1e-6 || m.Normal.Sub(mgl64.Vec3{0, -1, 0}).Len() > 1e-6 {
		t.Errorf("separation %.6f normal %v, want %.6f and (0 -1 0)", m.Points[0].Separation, m.Normal, want)
	}
}

// roundedShapesOnBox: a box with a sphere and a lying capsule resting on it
func roundedShapesOnBox() (box, sphere, capsule *actor.RigidBody) {
	box = createBox(mgl64.Vec3{}, mgl64.Vec3{0.5, 0.5, 0.5}, actor.BodyTypeStatic)
	sphere = createSphere(mgl64.Vec3{0.3, 0.74, 0.2}, 0.25, actor.BodyTypeDynamic)
	capsule = createCapsule(mgl64.Vec3{0, 0.8, 0}, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1}), 0.4, 0.3, actor.BodyTypeDynamic)
	return box, sphere, capsule
}

// The cores of the rounded shapes don't allocate: the core is the shape itself, seen through another type
func TestCoresDoNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	box, sphere, capsule := roundedShapesOnBox()
	var m constraint.Manifold
	allocs := testing.AllocsPerRun(100, func() {
		Collide(sphere, box, SpeculativeDistance, &m)
		Collide(box, capsule, SpeculativeDistance, &m)
	})
	if allocs != 0 {
		t.Errorf("the rounded shapes against a box allocate %.1f times per run, want 0", allocs)
	}
}

// A capsule lying on a box touches it by its segment: 2 points
func TestLyingCapsuleOnBoxHasTwoPoints(t *testing.T) {
	box, _, capsule := roundedShapesOnBox()
	var m constraint.Manifold
	if !Collide(box, capsule, SpeculativeDistance, &m) || m.Count != 2 {
		t.Errorf("the capsule lying on the box has %d points, want 2", m.Count)
	}
}
