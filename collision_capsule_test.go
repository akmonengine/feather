package feather

import (
	"math"
	"sort"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// Rotations mapping the capsule's local Y axis onto a world axis.
var (
	capsuleAlongX = mgl64.QuatRotate(-math.Pi/2, mgl64.Vec3{0, 0, 1})
	capsuleAlongZ = mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{1, 0, 0})
)

func createCapsule(position mgl64.Vec3, rotation mgl64.Quat, halfHeight, radius float64, bodyType actor.BodyType) *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: position, Rotation: rotation},
		&actor.Capsule{HalfHeight: halfHeight, Radius: radius},
		bodyType,
		1.0,
	)
}

func nearlyEqualVec(a, b mgl64.Vec3, tolerance float64) bool {
	return a.Sub(b).Len() <= tolerance
}

// sortedPoints orders contact points along an axis so tests do not depend on emission order.
func sortedPoints(points []constraint.ContactPoint, axis mgl64.Vec3) []constraint.ContactPoint {
	sorted := append([]constraint.ContactPoint(nil), points...)
	sort.Slice(sorted, func(i, j int) bool {
		return sorted[i].Position.Dot(axis) < sorted[j].Position.Dot(axis)
	})
	return sorted
}

type expectedContact struct {
	normal mgl64.Vec3
	points []constraint.ContactPoint // ordered along sortAxis
}

func checkManifold(t *testing.T, m *constraint.Manifold, want expectedContact, sortAxis mgl64.Vec3, tolerance float64) {
	t.Helper()
	checkContact(t, m.Normal, m.Points[:m.Count], want, sortAxis, tolerance)
}

func checkContact(t *testing.T, normal mgl64.Vec3, points []constraint.ContactPoint, want expectedContact, sortAxis mgl64.Vec3, tolerance float64) {
	t.Helper()
	if !nearlyEqualVec(normal, want.normal, tolerance) {
		t.Errorf("normal = %v, want %v", normal, want.normal)
	}
	if len(points) != len(want.points) {
		t.Fatalf("got %d points %v, want %d %v", len(points), points, len(want.points), want.points)
	}
	got := sortedPoints(points, sortAxis)
	for i := range got {
		if !nearlyEqualVec(got[i].Position, want.points[i].Position, tolerance) {
			t.Errorf("point[%d] = %v, want %v", i, got[i].Position, want.points[i].Position)
		}
		if math.Abs(got[i].Separation-want.points[i].Separation) > tolerance {
			t.Errorf("point[%d] separation = %.9f, want %.9f", i, got[i].Separation, want.points[i].Separation)
		}
	}
}

func point(x, y, z, depth float64) constraint.ContactPoint {
	return constraint.ContactPoint{Position: mgl64.Vec3{x, y, z}, Separation: -depth}
}

// Contact points of the analytic paths sit halfway between the two surfaces.
func TestCollideCapsuleCapsule(t *testing.T) {
	const tol = 1e-12
	yAxis := mgl64.Vec3{0, 1, 0}

	t.Run("side by side, parallel", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{0.9, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) {
			t.Fatal("no collision")
		}
		checkManifold(t, &m, expectedContact{
			normal: mgl64.Vec3{1, 0, 0},
			points: []constraint.ContactPoint{point(0.45, -1, 0, 0.1), point(0.45, 1, 0, 0.1)},
		}, yAxis, tol)
	})

	t.Run("parallel, partial overlap and different radii", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{0, 1.5, -0.7}, mgl64.QuatIdent(), 1, 0.3, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) {
			t.Fatal("no collision")
		}
		// Surfaces at z=-0.5 (A) and z=-0.4 (B): midpoint z=-0.45. Axial overlap y ∈ [0.5, 1].
		checkManifold(t, &m, expectedContact{
			normal: mgl64.Vec3{0, 0, -1},
			points: []constraint.ContactPoint{point(0, 0.5, -0.45, 0.1), point(0, 1, -0.45, 0.1)},
		}, yAxis, tol)
	})

	t.Run("crossed", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{0.9, 0.3, 0}, capsuleAlongZ, 1, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) {
			t.Fatal("no collision")
		}
		checkManifold(t, &m, expectedContact{
			normal: mgl64.Vec3{1, 0, 0},
			points: []constraint.ContactPoint{point(0.45, 0.3, 0, 0.1)},
		}, yAxis, tol)
	})

	t.Run("end to end", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{0, 2.9, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) {
			t.Fatal("no collision")
		}
		checkManifold(t, &m, expectedContact{
			normal: mgl64.Vec3{0, 1, 0},
			points: []constraint.ContactPoint{point(0, 1.45, 0, 0.1)},
		}, yAxis, tol)
	})

	t.Run("end against side (T)", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, capsuleAlongX, 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{0.2, 1.3, 0}, mgl64.QuatIdent(), 0.5, 0.4, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) {
			t.Fatal("no collision")
		}
		// B's lower end (0.2,0.8,0) is 0.8 above A's axis; radii sum 0.9. Surfaces at y=0.5 and y=0.4.
		checkManifold(t, &m, expectedContact{
			normal: mgl64.Vec3{0, 1, 0},
			points: []constraint.ContactPoint{point(0.2, 0.45, 0, 0.1)},
		}, yAxis, tol)
	})

	t.Run("arbitrary pose, parallel", func(t *testing.T) {
		rotation := mgl64.QuatRotate(0.7, mgl64.Vec3{1, 2, 3}.Normalize())
		offset := mgl64.Vec3{3, -2, 5}
		a := createCapsule(offset, rotation, 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(offset.Add(rotation.Rotate(mgl64.Vec3{0.9, 0, 0})), rotation, 1, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) {
			t.Fatal("no collision")
		}
		world := func(x, y, z, depth float64) constraint.ContactPoint {
			return constraint.ContactPoint{Position: offset.Add(rotation.Rotate(mgl64.Vec3{x, y, z})), Separation: -depth}
		}
		checkManifold(t, &m, expectedContact{
			normal: rotation.Rotate(mgl64.Vec3{1, 0, 0}),
			points: []constraint.ContactPoint{world(0.45, -1, 0, 0.1), world(0.45, 1, 0, 0.1)},
		}, rotation.Rotate(yAxis), 1e-9)
	})

	t.Run("axes intersect", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{0, 0, 0}, capsuleAlongZ, 1, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) {
			t.Fatal("no collision")
		}
		if m.Count != 1 || math.Abs(-m.Points[0].Separation-1) > tol {
			t.Errorf("manifold = %+v, want one point of depth 1", m)
		}
		if math.Abs(m.Normal.Len()-1) > tol || math.Abs(m.Normal.Y()) > tol || math.Abs(m.Normal.Z()) > tol {
			t.Errorf("normal = %v, want a unit vector orthogonal to both axes", m.Normal)
		}
	})

	t.Run("separated", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		for _, b := range []*actor.RigidBody{
			createCapsule(mgl64.Vec3{1.001, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic), // 1 mm apart
			createCapsule(mgl64.Vec3{0, 3.1, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic),
			createCapsule(mgl64.Vec3{1.2, 0, 0}, capsuleAlongZ, 1, 0.5, actor.BodyTypeDynamic),
		} {
			var m constraint.Manifold
			if CollideCapsuleCapsule(a, b, 0, &m) {
				t.Errorf("capsule at %v: unexpected collision %+v", b.Transform.Position, m)
			}
		}
	})

	t.Run("touching: zero separation", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{1.0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0, &m) || m.Count != 2 || m.MinSeparation() != 0 {
			t.Errorf("touching capsules: %+v, want 2 points at separation 0", m)
		}
	})

	t.Run("speculative: within the margin only", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{1.01, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleCapsule(a, b, 0.02, &m) || math.Abs(m.MinSeparation()-0.01) > 1e-12 {
			t.Errorf("capsules 1 cm apart with a 2 cm margin: %+v, want separation 0.01", m)
		}
		if CollideCapsuleCapsule(a, b, 0.005, &m) {
			t.Errorf("capsules 1 cm apart with a 5 mm margin: unexpected contact %+v", m)
		}
	})
}

func TestCollideCapsuleSphere(t *testing.T) {
	const tol = 1e-12
	yAxis := mgl64.Vec3{0, 1, 0}
	capsule := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)

	t.Run("against the side", func(t *testing.T) {
		sphere := createSphere(mgl64.Vec3{0.9, 0.3, 0}, 0.5, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleSphere(capsule, sphere, 0, &m) {
			t.Fatal("no collision")
		}
		checkManifold(t, &m, expectedContact{
			normal: mgl64.Vec3{1, 0, 0},
			points: []constraint.ContactPoint{point(0.45, 0.3, 0, 0.1)},
		}, yAxis, tol)
	})

	t.Run("against the cap", func(t *testing.T) {
		sphere := createSphere(mgl64.Vec3{0, -1.7, 0}, 0.3, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleSphere(capsule, sphere, 0, &m) {
			t.Fatal("no collision")
		}
		// Surfaces at y=-1.5 (capsule) and y=-1.4 (sphere).
		checkManifold(t, &m, expectedContact{
			normal: mgl64.Vec3{0, -1, 0},
			points: []constraint.ContactPoint{point(0, -1.45, 0, 0.1)},
		}, yAxis, tol)
	})

	t.Run("oblique on the cap", func(t *testing.T) {
		direction := mgl64.Vec3{1, 1, 1}.Normalize()
		sphere := createSphere(mgl64.Vec3{0, 1, 0}.Add(direction.Mul(0.8)), 0.4, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleSphere(capsule, sphere, 0, &m) {
			t.Fatal("no collision")
		}
		mid := mgl64.Vec3{0, 1, 0}.Add(direction.Mul(0.45))
		checkManifold(t, &m, expectedContact{
			normal: direction,
			points: []constraint.ContactPoint{{Position: mid, Separation: -0.1}},
		}, yAxis, 1e-12)
	})

	t.Run("centre on the axis", func(t *testing.T) {
		sphere := createSphere(mgl64.Vec3{0, 0.2, 0}, 0.3, actor.BodyTypeDynamic)
		var m constraint.Manifold
		if !CollideCapsuleSphere(capsule, sphere, 0, &m) {
			t.Fatal("no collision")
		}
		if m.Count != 1 || math.Abs(-m.Points[0].Separation-0.8) > tol {
			t.Errorf("manifold = %+v, want one point of depth 0.8", m)
		}
		if math.Abs(m.Normal.Len()-1) > tol || math.Abs(m.Normal.Y()) > tol {
			t.Errorf("normal = %v, want a unit vector orthogonal to the axis", m.Normal)
		}
	})

	t.Run("separated", func(t *testing.T) {
		sphere := createSphere(mgl64.Vec3{0, 2.001, 0}, 0.5, actor.BodyTypeDynamic) // 1 mm apart
		var m constraint.Manifold
		if CollideCapsuleSphere(capsule, sphere, 0, &m) {
			t.Errorf("unexpected collision %+v", m)
		}
	})
}

// narrowPhaseOne runs the public narrow phase on a single pair.
func narrowPhaseOne(a, b *actor.RigidBody) []constraint.Manifold {
	return NarrowPhase([]Pair{{BodyA: a, BodyB: b}}, 2)
}

func TestNarrowPhaseCapsulePairs(t *testing.T) {
	yAxis := mgl64.Vec3{0, 1, 0}

	t.Run("capsule-capsule uses the analytic path", func(t *testing.T) {
		a := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		b := createCapsule(mgl64.Vec3{0.9, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		contacts := narrowPhaseOne(a, b)
		if len(contacts) != 1 {
			t.Fatalf("got %d contacts, want 1", len(contacts))
		}
		c := contacts[0]
		if c.BodyA != a || c.BodyB != b {
			t.Errorf("bodies not preserved")
		}
		checkContact(t, c.Normal, c.Points[:c.Count], expectedContact{
			normal: mgl64.Vec3{1, 0, 0},
			points: []constraint.ContactPoint{point(0.45, -1, 0, 0.1), point(0.45, 1, 0, 0.1)},
		}, yAxis, 1e-12)
	})

	t.Run("sphere-capsule keeps the A to B normal", func(t *testing.T) {
		sphere := createSphere(mgl64.Vec3{0.9, 0.3, 0}, 0.5, actor.BodyTypeDynamic)
		capsule := createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		contacts := narrowPhaseOne(sphere, capsule)
		if len(contacts) != 1 {
			t.Fatalf("got %d contacts, want 1", len(contacts))
		}
		c := contacts[0]
		if c.BodyA != sphere || c.BodyB != capsule {
			t.Errorf("bodies not preserved")
		}
		checkContact(t, c.Normal, c.Points[:c.Count], expectedContact{
			normal: mgl64.Vec3{-1, 0, 0},
			points: []constraint.ContactPoint{point(0.45, 0.3, 0, 0.1)},
		}, yAxis, 1e-12)
	})

	t.Run("capsule lying on a plane", func(t *testing.T) {
		// The spatial grid always emits the plane as BodyA.
		plane := createPlane(mgl64.Vec3{0, 1, 0}, 0)
		capsule := createCapsule(mgl64.Vec3{0, 0.45, 0}, capsuleAlongX, 1, 0.5, actor.BodyTypeDynamic)
		contacts := narrowPhaseOne(plane, capsule)
		if len(contacts) != 1 {
			t.Fatalf("got %d contacts, want 1", len(contacts))
		}
		checkContact(t, contacts[0].Normal, contacts[0].Points[:contacts[0].Count], expectedContact{
			normal: mgl64.Vec3{0, 1, 0}, // from the plane (A) to the capsule (B)
			// Halfway between the capsule surface (y=-0.05) and the plane.
			points: []constraint.ContactPoint{point(-1, -0.025, 0, 0.05), point(1, -0.025, 0, 0.05)},
		}, mgl64.Vec3{1, 0, 0}, 1e-12)
	})
}

// Capsule against box goes through GJK/EPA: EPA converges to EPAConvergenceTolerance.
func TestNarrowPhaseCapsuleBox(t *testing.T) {
	const tol = 1e-6
	xAxis := mgl64.Vec3{1, 0, 0}

	t.Run("lying on the top face", func(t *testing.T) {
		box := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{2, 0.5, 2}, actor.BodyTypeStatic)
		capsule := createCapsule(mgl64.Vec3{0.3, 0.9, 0.2}, capsuleAlongX, 1, 0.5, actor.BodyTypeDynamic)
		contacts := narrowPhaseOne(box, capsule)
		if len(contacts) != 1 {
			t.Fatalf("got %d contacts, want 1", len(contacts))
		}
		// Deepest line of the capsule: y=0.4, x ∈ [-0.7, 1.3]; the points lie halfway to the
		// box face (y=0.5).
		checkContact(t, contacts[0].Normal, contacts[0].Points[:contacts[0].Count], expectedContact{
			normal: mgl64.Vec3{0, 1, 0},
			points: []constraint.ContactPoint{point(-0.7, 0.45, 0.2, 0.1), point(1.3, 0.45, 0.2, 0.1)},
		}, xAxis, tol)
	})

	t.Run("standing on the top face, capsule first", func(t *testing.T) {
		capsule := createCapsule(mgl64.Vec3{0.5, 1.9, -0.5}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)
		box := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{2, 0.5, 2}, actor.BodyTypeStatic)
		contacts := narrowPhaseOne(capsule, box)
		if len(contacts) != 1 {
			t.Fatalf("got %d contacts, want 1", len(contacts))
		}
		checkContact(t, contacts[0].Normal, contacts[0].Points[:contacts[0].Count], expectedContact{
			normal: mgl64.Vec3{0, -1, 0},
			points: []constraint.ContactPoint{point(0.5, 0.45, -0.5, 0.1)},
		}, xAxis, tol)
	})

	t.Run("end against a side face", func(t *testing.T) {
		box := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic)
		capsule := createCapsule(mgl64.Vec3{2.4, 0.2, 0.1}, capsuleAlongX, 1, 0.5, actor.BodyTypeDynamic)
		contacts := narrowPhaseOne(box, capsule)
		if len(contacts) != 1 {
			t.Fatalf("got %d contacts, want 1", len(contacts))
		}
		checkContact(t, contacts[0].Normal, contacts[0].Points[:contacts[0].Count], expectedContact{
			normal: mgl64.Vec3{1, 0, 0},
			points: []constraint.ContactPoint{point(0.95, 0.2, 0.1, 0.1)},
		}, xAxis, tol)
	})

	t.Run("crossed over an edge", func(t *testing.T) {
		// Capsule along Z resting across the top-right edge of a box, at 45°.
		box := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic)
		direction := mgl64.Vec3{1, 1, 0}.Normalize()
		center := mgl64.Vec3{1, 1, 0}.Add(direction.Mul(0.4))
		capsule := createCapsule(center, capsuleAlongZ, 2, 0.5, actor.BodyTypeDynamic)
		contacts := narrowPhaseOne(box, capsule)
		if len(contacts) != 1 {
			t.Fatalf("got %d contacts, want 1", len(contacts))
		}
		c := contacts[0]
		// On a rounded surface, EPA's distance tolerance bounds the normal error to
		// acos(1 - tol/radius), well under 0.1° here.
		maxAngle := math.Acos(1 - epa.EPAConvergenceTolerance/0.5)
		if mgl64.RadToDeg(maxAngle) > 0.1 {
			t.Fatalf("EPA tolerance allows %.3f°, want < 0.1°", mgl64.RadToDeg(maxAngle))
		}
		if angle := math.Acos(math.Min(1, c.Normal.Dot(direction))); angle > maxAngle {
			t.Errorf("normal = %v, %.2f° from %v (max %.2f°)", c.Normal, mgl64.RadToDeg(angle), direction, mgl64.RadToDeg(maxAngle))
		}
		if c.Count != 2 {
			t.Fatalf("got %d points %v, want the capsule line clipped to the box (2 points)", c.Count, c.Points)
		}
		zs := []float64{c.Points[0].Position.Z(), c.Points[1].Position.Z()}
		sort.Float64s(zs)
		if math.Abs(zs[0]+1) > 1e-6 || math.Abs(zs[1]-1) > 1e-6 {
			t.Errorf("points z = %v, want the capsule line clipped to the box, [-1 1]", zs)
		}
		for _, p := range c.Points[:c.Count] {
			if math.Abs(p.Separation+0.1) > tol {
				t.Errorf("separation = %f, want -0.1", p.Separation)
			}
			// The points lie on the capsule's deepest line, within the depth of the edge x=y=1.
			edgeDistance := mgl64.Vec3{p.Position.X() - 1, p.Position.Y() - 1, 0}.Len()
			if edgeDistance > 0.1+tol {
				t.Errorf("point %v is %.4f from the box edge, want <= 0.1", p.Position, edgeDistance)
			}
		}
	})
}

// The GJK/EPA general path must agree with the analytic kernels.
func TestCapsuleCapsuleAnalyticMatchesGJK(t *testing.T) {
	poses := capsuleBenchPoses()
	for i, pose := range poses {
		var m constraint.Manifold
		analytic := CollideCapsuleCapsule(pose.a, pose.b, 0, &m)

		simplex := &gjk.Simplex{}
		general := gjk.GJK(pose.a, pose.b, simplex)
		if analytic != general {
			t.Errorf("pose %d: analytic collision %v, GJK %v", i, analytic, general)
			continue
		}
		if !general {
			continue
		}
		result, err := epa.EPA(pose.a, pose.b, simplex, 0)
		if err != nil {
			t.Errorf("pose %d: EPA error %v", i, err)
			continue
		}
		if !nearlyEqualVec(result.Normal, m.Normal, 1e-3) {
			t.Errorf("pose %d: EPA normal %v, analytic %v", i, result.Normal, m.Normal)
		}
		if math.Abs(result.Depth+m.MinSeparation()) > 2*epa.EPAConvergenceTolerance {
			t.Errorf("pose %d: EPA depth %f, analytic %f", i, result.Depth, -m.MinSeparation())
		}
	}
}

func TestCapsuleAnalyticDoesNotAllocate(t *testing.T) {
	poses := capsuleBenchPoses()
	sphere := createSphere(mgl64.Vec3{0.9, 0.3, 0}, 0.5, actor.BodyTypeDynamic)
	var m constraint.Manifold

	allocs := testing.AllocsPerRun(100, func() {
		for _, pose := range poses {
			CollideCapsuleCapsule(pose.a, pose.b, 0, &m)
			CollideCapsuleSphere(pose.a, sphere, 0, &m)
		}
	})
	if allocs != 0 {
		t.Errorf("analytic capsule kernels allocate %.1f times per run, want 0", allocs)
	}
}

// simulateCapsuleOnPlane drops nothing: the capsule starts exactly resting on the ground
// and the maximum displacement from that pose over the duration is returned.
func simulateCapsuleOnPlane(t *testing.T, rotation mgl64.Quat, restingHeight float64, seconds float64) (maxDrift float64, finalAxis mgl64.Vec3) {
	t.Helper()
	world := World{
		Gravity:     mgl64.Vec3{0, -9.81, 0},
		Substeps:    10,
		SpatialGrid: NewSpatialGrid(2.0, 1024),
		Workers:     1,
		Events:      NewEvents(),
	}
	world.AddBody(createPlane(mgl64.Vec3{0, 1, 0}, 0))
	start := mgl64.Vec3{0.25, restingHeight, -0.5}
	capsule := createCapsule(start, rotation, 0.6, 0.3, actor.BodyTypeDynamic)
	world.AddBody(capsule)

	const dt = 1.0 / 60.0
	for step := 0; step < int(seconds/dt); step++ {
		world.Step(dt)
		maxDrift = math.Max(maxDrift, capsule.Transform.Position.Sub(start).Len())
	}
	return maxDrift, capsule.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0})
}

func TestCapsuleRestsOnPlane(t *testing.T) {
	const maxDrift = 1e-4 // 0.1 mm

	t.Run("upright", func(t *testing.T) {
		drift, axis := simulateCapsuleOnPlane(t, mgl64.QuatIdent(), 0.6+0.3, 10)
		if !(drift < maxDrift) { // also rejects NaN
			t.Errorf("upright capsule drifted %.3e m over 10 s, want < %.0e", drift, maxDrift)
		}
		if !(axis.Y() >= 1-1e-9) {
			t.Errorf("upright capsule tilted: axis %v", axis)
		}
	})

	t.Run("lying", func(t *testing.T) {
		drift, axis := simulateCapsuleOnPlane(t, capsuleAlongX, 0.3, 10)
		if !(drift < maxDrift) { // also rejects NaN
			t.Errorf("lying capsule drifted %.3e m over 10 s, want < %.0e", drift, maxDrift)
		}
		if !(math.Abs(axis.Y()) <= 1e-6) {
			t.Errorf("lying capsule tilted: axis %v", axis)
		}
	})
}

type capsulePose struct {
	name string
	a, b *actor.RigidBody
}

// capsuleBenchPoses covers the contact configurations of the acceptance criteria.
func capsuleBenchPoses() []capsulePose {
	tilted := mgl64.QuatRotate(0.4, mgl64.Vec3{1, 0, 1}.Normalize())
	return []capsulePose{
		{"parallel",
			createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic),
			createCapsule(mgl64.Vec3{0.9, 0.2, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)},
		{"crossed",
			createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic),
			createCapsule(mgl64.Vec3{0.9, 0.3, 0}, capsuleAlongZ, 1, 0.5, actor.BodyTypeDynamic)},
		{"end to end",
			createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic),
			createCapsule(mgl64.Vec3{0.05, 2.9, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic)},
		{"tilted",
			createCapsule(mgl64.Vec3{0, 0, 0}, capsuleAlongX, 1, 0.4, actor.BodyTypeDynamic),
			createCapsule(mgl64.Vec3{0.3, 0.7, 0.1}, tilted, 0.8, 0.4, actor.BodyTypeDynamic)},
		{"separated",
			createCapsule(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), 1, 0.5, actor.BodyTypeDynamic),
			createCapsule(mgl64.Vec3{1.5, 0, 0}, capsuleAlongZ, 1, 0.5, actor.BodyTypeDynamic)},
	}
}

func BenchmarkCapsuleCapsule(b *testing.B) {
	for _, pose := range capsuleBenchPoses() {
		b.Run("analytic/"+pose.name, func(b *testing.B) {
			var m constraint.Manifold
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				CollideCapsuleCapsule(pose.a, pose.b, 0, &m)
			}
		})
		b.Run("gjk-epa/"+pose.name, func(b *testing.B) {
			simplex := &gjk.Simplex{}
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				simplex.Reset()
				if gjk.GJK(pose.a, pose.b, simplex) {
					_, _ = epa.EPA(pose.a, pose.b, simplex, 0)
				}
			}
		})
	}
}
