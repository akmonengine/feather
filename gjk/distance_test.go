package gjk

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

func randomRotation(r *rand.Rand) mgl64.Quat {
	return mgl64.QuatRotate(r.Float64()*2*math.Pi, mgl64.Vec3{r.Float64() - 0.5, r.Float64() - 0.5, r.Float64() - 0.5}.Normalize())
}

func distanceOf(a, b *actor.RigidBody) DistanceResult {
	proxyA, proxyB := NewProxy(a), NewProxy(b)
	return Distance(&proxyA, &proxyB)
}

// A box against a sphere: the exact distance is the distance from the center of the sphere to the box, minus its radius
func TestDistanceBoxSphere(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	halfExtents := mgl64.Vec3{0.5, 0.3, 0.8}
	for i := 0; i < 1000; i++ {
		rotation := randomRotation(r)
		box := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0.2, -0.1, 0.3}, Rotation: rotation}, &actor.Box{HalfExtents: halfExtents}, actor.BodyTypeDynamic, 1)
		center := mgl64.Vec3{r.Float64()*4 - 2, r.Float64()*4 - 2, r.Float64()*4 - 2}
		sphere := actor.NewRigidBody(actor.Transform{Position: center, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 0.25}, actor.BodyTypeDynamic, 1)

		local := box.Transform.ToLocal(center)
		clamped := mgl64.Vec3{}
		for k := 0; k < 3; k++ {
			clamped[k] = math.Max(-halfExtents[k], math.Min(halfExtents[k], local[k]))
		}
		want := local.Sub(clamped).Len() - 0.25

		result := distanceOf(box, sphere)
		if want <= 0 {
			if !result.Overlap && want < -1e-6 {
				t.Fatalf("case %d: overlap %.4f not found (distance %.4f)", i, want, result.Distance)
			}
			continue
		}
		if result.Overlap || math.Abs(result.Distance-want) > 1e-6 {
			t.Fatalf("case %d: distance %.8f, want %.8f (overlap %v)", i, result.Distance, want, result.Overlap)
		}
		// the closest points: on the box and on the sphere, along the normal
		if math.Abs(result.PointB.Sub(result.PointA).Len()-want) > 1e-6 || math.Abs(result.PointB.Sub(center).Len()-0.25) > 1e-6 {
			t.Fatalf("case %d: wrong closest points", i)
		}
	}
}

// Capsules: the exact distance is the distance between their segments, minus both radii (sampled)
func TestDistanceCapsules(t *testing.T) {
	r := rand.New(rand.NewSource(2))
	for i := 0; i < 300; i++ {
		a := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{}, Rotation: randomRotation(r)}, &actor.Capsule{HalfHeight: 0.5, Radius: 0.1}, actor.BodyTypeDynamic, 1)
		b := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{r.Float64()*3 - 1.5, r.Float64()*3 - 1.5, r.Float64()*3 - 1.5}, Rotation: randomRotation(r)}, &actor.Capsule{HalfHeight: 0.3, Radius: 0.2}, actor.BodyTypeDynamic, 1)
		a0, a1 := a.Shape.(*actor.Capsule).Segment(a.Transform)
		b0, b1 := b.Shape.(*actor.Capsule).Segment(b.Transform)
		best := math.Inf(1)
		const samples = 400
		for p := 0; p <= samples; p++ {
			pa := a0.Add(a1.Sub(a0).Mul(float64(p) / samples))
			// closest point of the segment b
			s := math.Max(0, math.Min(1, pa.Sub(b0).Dot(b1.Sub(b0))/b1.Sub(b0).LenSqr()))
			best = math.Min(best, pa.Sub(b0.Add(b1.Sub(b0).Mul(s))).Len())
		}
		want := best - 0.3
		result := distanceOf(a, b)
		if want < -1e-3 {
			if !result.Overlap {
				t.Fatalf("case %d: overlap not found", i)
			}
			continue
		}
		if want > 1e-3 && (result.Overlap || math.Abs(result.Distance-want) > 1e-4) {
			t.Fatalf("case %d: distance %.6f, want %.6f", i, result.Distance, want)
		}
	}
}

func TestDistanceSpheres(t *testing.T) {
	a := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{1, 2, 3}, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 0.5}, actor.BodyTypeDynamic, 1)
	b := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{4, 6, 3}, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 1}, actor.BodyTypeDynamic, 1)
	result := distanceOf(a, b)
	if math.Abs(result.Distance-3.5) > 1e-9 || result.Normal.Sub(mgl64.Vec3{0.6, 0.8, 0}).Len() > 1e-9 {
		t.Errorf("distance %v, normal %v", result.Distance, result.Normal)
	}
}
