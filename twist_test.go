package feather

import (
	"fmt"
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// Twist friction: the friction of a contact around its normal. It is held by the lever arms of the points of the
// contact, the distance of each point to the friction center, both taken on the surface of A. A single point is its own
// center: it holds no twist
const (
	// leverArmTolerance: a lever arm is a distance between points given to the solver, exact to the rounding (m)
	leverArmTolerance = 1e-12

	// boxTwistTolerance: a box spinning flat on the ground brakes at the rate of its lever arms at ± 0.1 %. Its contact
	// points are found again at each step, on corners which turned: the load moves between them, the bound doesn't
	// (the 4 corners are at the same distance of the center)
	boxTwistTolerance = 0.001
)

// twistPoint: a contact point given by its place on the surface of A and its separation
type twistPoint struct {
	onA        mgl64.Vec3
	separation float64
}

// preparedContact: the constraint the solver prepares for a contact between both bodies of the World (A the first one),
// along the normal. Position is halfway between both surfaces
func preparedContact(w *World, normal mgl64.Vec3, points ...twistPoint) *contactConstraint {
	manifold := constraint.Manifold{BodyA: w.Bodies[0], BodyB: w.Bodies[1], IndexA: 0, IndexB: 1, Normal: normal}
	for _, point := range points {
		manifold.Add(point.onA.Add(normal.Mul(point.separation/2)), point.separation)
	}
	s := &w.solver
	s.prepare(w.Bodies, []constraint.Manifold{manifold}, sceneDt, sceneSubsteps, DefaultContactHertz, w.workerPool())
	return &s.constraints[0]
}

// frictionWeight of a point in the friction center: 1 up to SpeculativeDistance, 0 at twice
func frictionWeight(separation float64) float64 {
	return math.Min(1, math.Max(minFrictionWeight, 2-separation/SpeculativeDistance))
}

// ========== LEVER ARMS ==========
// The single point of a ball or of the cap of a capsule is the friction center of its contact: its lever arm is exactly
// 0, whatever its overlap (or its distance, for a speculative point), the body it belongs to and the normal
func TestSinglePointHasNoLeverArm(t *testing.T) {
	shapes := []struct {
		name  string
		shape actor.ShapeInterface
	}{
		{"ball", &actor.Sphere{Radius: spinRadius}},
		{"capsule", &actor.Capsule{HalfHeight: 0.2, Radius: spinRadius}},
	}
	others := []struct {
		name  string
		shape actor.ShapeInterface
	}{
		{"box", cube()},
		{"ball", &actor.Sphere{Radius: 3 * spinRadius}},
	}
	normals := []mgl64.Vec3{{0, 1, 0}, {0.36, 0.48, 0.8}}
	separations := []float64{-0.01, -70e-6, 0, 0.5 * SpeculativeDistance, 1.25 * SpeculativeDistance, 3 * SpeculativeDistance}
	for _, round := range shapes {
		for _, other := range others {
			for _, normal := range normals {
				for _, separation := range separations {
					for _, roundFirst := range []bool{false, true} {
						name := fmt.Sprintf("%s on a %s, normal %v, separation %g, round first %v", round.name, other.name, normal, separation, roundFirst)
						// the point on the surface of the other body, the round body above it along the normal
						onOther := mgl64.Vec3{0.3, 0.7, -0.2}
						onRound := onOther.Add(normal.Mul(separation))
						otherAt, roundAt := onOther.Sub(normal.Mul(cubeHalf)), onRound.Add(normal.Mul(spinRadius))
						w := newScene(1)
						var c *contactConstraint
						if roundFirst {
							// A is the round body: the normal goes down to B
							addBody(w, roundAt, mgl64.QuatIdent(), round.shape, actor.BodyTypeDynamic, 0.8, 0)
							addBody(w, otherAt, mgl64.QuatIdent(), other.shape, actor.BodyTypeDynamic, 0.8, 0)
							c = preparedContact(w, normal.Mul(-1), twistPoint{onRound, separation})
						} else {
							addBody(w, otherAt, mgl64.QuatIdent(), other.shape, actor.BodyTypeDynamic, 0.8, 0)
							addBody(w, roundAt, mgl64.QuatIdent(), round.shape, actor.BodyTypeDynamic, 0.8, 0)
							c = preparedContact(w, normal, twistPoint{onOther, separation})
						}
						if c.pointsCount != 1 || c.points[0].leverArm != 0 {
							t.Errorf("%s: lever arm %g m, want 0", name, c.points[0].leverArm)
						}
					}
				}
			}
		}
	}
}

// The lever arm of a point is its distance to the friction center, both on the surface of A: the overlap of the points
// is not part of it
func TestLeverArmsAreTakenFromTheFrictionCenter(t *testing.T) {
	half := mgl64.Vec3{0.4, 0.1, 0.3}
	up := mgl64.Vec3{0, 1, 0}
	plate := func(w *World) {
		addBody(w, mgl64.Vec3{0, -1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{5, 1, 5}}, actor.BodyTypeStatic, 0.5, 0)
		addBody(w, mgl64.Vec3{0, half.Y(), 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: half}, actor.BodyTypeDynamic, 0.5, 0)
	}

	// a box flat on the ground: its 4 corners are at half a diagonal of the center, whatever their overlap
	for _, separation := range []float64{-0.01, -70e-6, 0, 0.01} {
		w := newScene(1)
		plate(w)
		c := preparedContact(w, up,
			twistPoint{mgl64.Vec3{half.X(), 0, half.Z()}, separation}, twistPoint{mgl64.Vec3{-half.X(), 0, half.Z()}, separation},
			twistPoint{mgl64.Vec3{-half.X(), 0, -half.Z()}, separation}, twistPoint{mgl64.Vec3{half.X(), 0, -half.Z()}, separation})
		for j := 0; j < c.pointsCount; j++ {
			if got, want := c.points[j].leverArm, math.Hypot(half.X(), half.Z()); math.Abs(got-want) > leverArmTolerance {
				t.Errorf("flat box, separation %g: lever arm %d is %.15f m, want %.15f", separation, j, got, want)
			}
		}
	}

	// a capsule lying on the ground: both ends of its axis are at half its length of the center
	const halfHeight = 0.2
	for _, separation := range []float64{-0.01, 0, 0.01} {
		w := newScene(1)
		addGround(w, 0.5)
		addBody(w, mgl64.Vec3{0, spinRadius + separation, 0}, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1}),
			&actor.Capsule{HalfHeight: halfHeight, Radius: spinRadius}, actor.BodyTypeDynamic, 0.5, 0)
		c := preparedContact(w, up, twistPoint{mgl64.Vec3{halfHeight, 0, 0}, separation}, twistPoint{mgl64.Vec3{-halfHeight, 0, 0}, separation})
		for j := 0; j < c.pointsCount; j++ {
			if got := c.points[j].leverArm; math.Abs(got-halfHeight) > leverArmTolerance {
				t.Errorf("lying capsule, separation %g: lever arm %d is %.15f m, want %.15f", separation, j, got, halfHeight)
			}
		}
	}

	// a box leaning over the ground: each point has its own separation, and the farthest ones count less in the center
	// (the weights of prepareFriction). The points of A (the ground) are in its plane: the lever arms are their
	// distances to their weighted center, in the plane
	points := []twistPoint{
		{mgl64.Vec3{half.X(), 0, half.Z()}, -0.004},
		{mgl64.Vec3{-half.X(), 0, half.Z()}, 0.012},
		{mgl64.Vec3{-half.X(), 0, -half.Z()}, 0.026},
		{mgl64.Vec3{half.X(), 0, -half.Z()}, 0.034},
	}
	var center mgl64.Vec3
	total := 0.0
	for _, point := range points {
		center = center.Add(point.onA.Mul(frictionWeight(point.separation)))
		total += frictionWeight(point.separation)
	}
	center = center.Mul(1 / total)
	if center.Len() < 0.1 {
		t.Fatalf("the center %v is the middle of the box: the weights test nothing", center)
	}
	w := newScene(1)
	plate(w)
	c := preparedContact(w, up, points...)
	for j, point := range points {
		if got, want := c.points[j].leverArm, point.onA.Sub(center).Len(); math.Abs(got-want) > leverArmTolerance {
			t.Errorf("leaning box: lever arm %d is %.15f m, want %.15f", j, got, want)
		}
	}
}

// ========== SOLVER ==========
// The twist is an angular impulse around the normal, opposite on both bodies, bounded by the friction times the lever
// arm of each point times its normal impulse: µ Σ (lever arm × λ_normal)
func TestSolveFrictionBoundsTheTwistByTheLeverArms(t *testing.T) {
	normal := mgl64.Vec3{0, 1, 0}
	bodyA, bodyB := &actor.RigidBody{}, &actor.RigidBody{}
	newStates := func(spinA, spinB float64) (*bodyState, *bodyState) {
		return &bodyState{body: bodyA, dynamic: true, angularVelocity: mgl64.Vec3{0.3, spinA, 0}, inverseInertia: mgl64.Diag3(mgl64.Vec3{2, 2, 2})},
			&bodyState{body: bodyB, dynamic: true, angularVelocity: mgl64.Vec3{0, spinB, -0.7}, inverseInertia: mgl64.Diag3(mgl64.Vec3{4, 4, 4})}
	}
	// the tangent rows have no mass here: the twist alone is solved
	newConstraint := func(friction float64) *contactConstraint {
		c := &contactConstraint{normal: normal, pointsCount: 2, friction: friction, twistMass: 1.0 / (2 + 4)}
		c.tangents[0], c.tangents[1] = tangentBasis(normal)
		c.points[0].leverArm, c.points[0].normalImpulse = 0.2, 3
		c.points[1].leverArm, c.points[1].normalImpulse = 0.5, 5
		return c
	}
	const limit = 0.4 * (0.2*3 + 0.5*5)

	// sliding: the impulse is at its bound, against the relative spin
	stateA, stateB := newStates(1, 11)
	c := newConstraint(0.4)
	c.solveFriction(stateA, stateB)
	if math.Abs(c.twistImpulse+limit) > 1e-15 {
		t.Errorf("sliding: impulse %v, want %v", c.twistImpulse, -limit)
	}
	if got, want := stateA.angularVelocity, (mgl64.Vec3{0.3, 1 + 2*limit, 0}); got.Sub(want).Len() > 1e-12 {
		t.Errorf("sliding: A spins at %v, want %v", got, want)
	}
	if got, want := stateB.angularVelocity, (mgl64.Vec3{0, 11 - 4*limit, -0.7}); got.Sub(want).Len() > 1e-12 {
		t.Errorf("sliding: B spins at %v, want %v", got, want)
	}
	// the other way
	stateA, stateB = newStates(11, 1)
	c = newConstraint(0.4)
	c.solveFriction(stateA, stateB)
	if math.Abs(c.twistImpulse-limit) > 1e-15 {
		t.Errorf("sliding the other way: impulse %v, want %v", c.twistImpulse, limit)
	}

	// sticking: both bodies end with the same spin, the one which keeps their angular momentum
	stateA, stateB = newStates(1, 4)
	c = newConstraint(10)
	c.solveFriction(stateA, stateB)
	shared := (1/2.0 + 4/4.0) / (1/2.0 + 1/4.0)
	if math.Abs(stateA.angularVelocity.Y()-shared) > 1e-12 || math.Abs(stateB.angularVelocity.Y()-shared) > 1e-12 {
		t.Errorf("sticking: spins %v & %v, want %v", stateA.angularVelocity.Y(), stateB.angularVelocity.Y(), shared)
	}
	// the accumulated impulse is bounded, not each of its increments
	stateB.angularVelocity[1] += 100
	c.friction = 0.4
	c.solveFriction(stateA, stateB)
	if math.Abs(c.twistImpulse+limit) > 1e-15 {
		t.Errorf("accumulated: impulse %v, want %v", c.twistImpulse, -limit)
	}

	// a point without normal impulse holds nothing: the bound is the one of the other point
	stateA, stateB = newStates(1, 11)
	c = newConstraint(0.4)
	c.points[1].normalImpulse = 0
	c.solveFriction(stateA, stateB)
	if want := -0.4 * 0.2 * 3; math.Abs(c.twistImpulse-want) > 1e-15 {
		t.Errorf("a point without load: impulse %v, want %v", c.twistImpulse, want)
	}

	// a single point, without lever arm: no twist, the spins are kept bit for bit
	stateA, stateB = newStates(1, 11)
	c = newConstraint(0.4)
	c.pointsCount, c.points[0].leverArm = 1, 0
	c.solveFriction(stateA, stateB)
	if c.twistImpulse != 0 {
		t.Errorf("single point: twist impulse %v, want 0", c.twistImpulse)
	}
	if stateA.angularVelocity != (mgl64.Vec3{0.3, 1, 0}) || stateB.angularVelocity != (mgl64.Vec3{0, 11, -0.7}) {
		t.Errorf("single point: spins %v & %v, want them kept", stateA.angularVelocity, stateB.angularVelocity)
	}
}

// ========== A BOX ==========
// A box spinning flat on the ground brakes by its lever arms. Its 4 corners are at the same distance d of the center
// (half a diagonal): however the load is shared between them, the contact holds the torque µ·d·m·g, and the box
// (I = m·d²/3 around the vertical) slows down at µ·d·m·g / I = 3·µ·g / d
func TestSpinningBoxBrakesByItsLeverArms(t *testing.T) {
	const friction, spin = 0.1, 10.0
	for _, half := range []mgl64.Vec3{{0.4, 0.1, 0.3}, {cubeHalf, cubeHalf, cubeHalf}} {
		w := newScene(1)
		addGround(w, friction)
		box := addBody(w, mgl64.Vec3{0, half.Y(), 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: half}, actor.BodyTypeDynamic, friction, 0)
		simulate(w, 0.25, nil)
		box.AngularVelocity = mgl64.Vec3{0, spin, 0}

		want := 3 * friction * sceneGravity / math.Hypot(half.X(), half.Z())
		// measured between 1/4 and 3/4 of the braking
		from, to := 0.25*spin/want, 0.75*spin/want
		elapsed, stopped := 0.0, -1.0
		var speedFrom, speedTo, timeFrom, timeTo float64
		simulate(w, 2*spin/want, func() {
			elapsed += sceneDt
			speed := box.AngularVelocity.Y()
			if elapsed <= from {
				speedFrom, timeFrom = speed, elapsed
			}
			if elapsed <= to {
				speedTo, timeTo = speed, elapsed
			}
			if stopped < 0 && math.Abs(speed) < actor.DefaultSleepSpeed {
				stopped = elapsed
			}
		})
		got := (speedFrom - speedTo) / (timeTo - timeFrom)
		t.Logf("box %v: deceleration %.6f rad/s², analytic %.6f; stopped at %.4f s, analytic %.4f s", half, got, want, stopped, spin/want)
		if math.Abs(got-want) > boxTwistTolerance*want {
			t.Errorf("box %v: deceleration %.4f rad/s², want %.4f", half, got, want)
		}
		if math.Abs(stopped-spin/want) > 2*sceneDt {
			t.Errorf("box %v: stopped at %.4f s, want %.4f s", half, stopped, spin/want)
		}
		if drift := box.Transform.Position.Sub(mgl64.Vec3{0, half.Y(), 0}).Len(); drift > 1e-3 {
			t.Errorf("box %v: moved by %.2g m", half, drift)
		}
	}
}
