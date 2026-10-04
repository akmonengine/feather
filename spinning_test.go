package feather

import (
	"math"
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Spinning resistance: a ball spinning around the normal of its contact (a top). A contact patch of radius a under a
// load N holds the torque 2/3·µ·a·N (uniform pressure); the material gives it as c·R·N, c without unit, R the radius.
const (
	spinRadius     = 0.1
	spinResistance = 0.05
	spinSpeed      = 20.0

	// spinTolerance: the deceleration is the analytic one at ± 0.1 %. The resistance is the only torque around the
	// normal (the single point of a ball holds no twist): what is left is the rounding of the measure
	spinTolerance = 0.001

	// spinKept: without spinning resistance a top keeps its spin, to the rounding (rad/s lost over 10 s at 20 rad/s).
	// Nothing acts around the normal of its single point: the twist has no lever arm, the friction is applied on the
	// axis of the spin
	spinKept = 1e-9
)

// spinDeceleration of a solid sphere (I = 2/5·m·R²) under the torque c·R·m·g: c·R·g / (2/5·R²)
func spinDeceleration(resistance, radius float64) float64 {
	return resistance * radius * sceneGravity / (2.0 / 5.0 * radius * radius)
}

// spinScene: a ball resting on the ground. The contact takes the largest resistance of both bodies
func spinScene(workers int, radius, ballResistance, groundResistance float64, groundFirst bool) (*World, *actor.RigidBody) {
	w := newScene(workers)
	if groundFirst {
		addGround(w, 0.8).Material.SpinningResistance = groundResistance
	}
	ball := addBody(w, mgl64.Vec3{0, radius, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0.8, 0)
	ball.Material.SpinningResistance = ballResistance
	if !groundFirst {
		addGround(w, 0.8).Material.SpinningResistance = groundResistance
	}
	// the ball settles on its contact, still awake
	simulate(w, 0.25, nil)
	return w, ball
}

// spinSceneOn: a ball resting on a plane of this normal, the gravity along the normal (the same scene as on the
// ground, turned)
func spinSceneOn(normal mgl64.Vec3, resistance float64) (*World, *actor.RigidBody) {
	w := newScene(1)
	w.Gravity = normal.Mul(-sceneGravity)
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Plane{Normal: normal}, actor.BodyTypeStatic, 0.8, 0)
	ball := addBody(w, normal.Mul(spinRadius), mgl64.QuatIdent(), &actor.Sphere{Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
	ball.Material.SpinningResistance = resistance
	simulate(w, 0.25, nil)
	return w, ball
}

// turnAngle of the rotation, in radians
func turnAngle(q mgl64.Quat) float64 {
	return 2 * math.Atan2(q.V.Len(), math.Abs(q.W))
}

// ========== ACCEPTANCE ==========
// A ball of radius 10 cm spinning at 20 rad/s around the vertical, resistance 0.05: it slows down at
// c·R·g / (2/5·R²) = 12.26 rad/s² (± spinTolerance), stops within 3 s and falls asleep DefaultTimeToSleep after it
// stopped
func TestSpinningResistanceStopsTheTop(t *testing.T) {
	cases := []struct {
		name                 string
		radius               float64
		ball, ground, speed  float64
		groundFirst          bool
		resistance, wantStop float64
	}{
		{"ball", spinRadius, spinResistance, 0, spinSpeed, true, spinResistance, 3},
		{"other way", spinRadius, spinResistance, 0, -spinSpeed, true, spinResistance, 3},
		{"ground", spinRadius, 0, spinResistance, spinSpeed, true, spinResistance, 3},
		{"ball before the ground", spinRadius, spinResistance, 0, spinSpeed, false, spinResistance, 3},
		{"largest of both", spinRadius, spinResistance, 2 * spinResistance, spinSpeed, true, 2 * spinResistance, 3},
		{"diameter of 10 cm", spinRadius / 2, spinResistance, 0, spinSpeed, true, spinResistance, 3},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			w, ball := spinScene(1, c.radius, c.ball, c.ground, c.groundFirst)
			ball.AngularVelocity = mgl64.Vec3{0, c.speed, 0}
			want := spinDeceleration(c.resistance, c.radius)
			// measured between 1/4 and 3/4 of the braking
			from, to := 0.25*spinSpeed/want, 0.75*spinSpeed/want
			elapsed, stopped, asleep := 0.0, -1.0, -1.0
			var speedFrom, speedTo, timeFrom, timeTo float64
			simulate(w, 3, func() {
				elapsed += sceneDt
				speed := ball.AngularVelocity.Len()
				if elapsed <= from {
					speedFrom, timeFrom = speed, elapsed
				}
				if elapsed <= to {
					speedTo, timeTo = speed, elapsed
				}
				if stopped < 0 && speed < actor.DefaultSleepSpeed {
					stopped = elapsed
				}
				if asleep < 0 && ball.IsSleeping {
					asleep = elapsed
				}
				if side := math.Hypot(ball.AngularVelocity.X(), ball.AngularVelocity.Z()); side > 1e-9 {
					t.Fatalf("at %.2f s the ball turns around a tangent: %v", elapsed, ball.AngularVelocity)
				}
			})
			got := (speedFrom - speedTo) / (timeTo - timeFrom)
			t.Logf("deceleration %.4f rad/s², analytic %.4f; stopped at %.4f s, analytic %.4f s; asleep at %.4f s",
				got, want, stopped, spinSpeed/want, asleep)
			if math.Abs(got-want) > spinTolerance*want {
				t.Errorf("deceleration %.4f rad/s², want %.4f", got, want)
			}
			if stopped < 0 || stopped >= c.wantStop {
				t.Errorf("stopped at %.3f s, want under %.0f s", stopped, c.wantStop)
			}
			if math.Abs(stopped-spinSpeed/want) > 0.05*spinSpeed/want {
				t.Errorf("stopped at %.3f s, want %.3f s", stopped, spinSpeed/want)
			}
			if !ball.IsSleeping {
				t.Errorf("the ball is still awake after 3 s, at %v rad/s", ball.AngularVelocity)
			}
			// the timer of the sleep starts when the ball is under the sleep speed: nothing wakes it up, nothing delays it
			if want := stopped + actor.DefaultTimeToSleep; math.Abs(asleep-want) > sceneDt*1.01 {
				t.Errorf("asleep at %.4f s, want %.4f s (stopped at %.4f s + %.1f s)", asleep, want, stopped, actor.DefaultTimeToSleep)
			}
			if drift := ball.Transform.Position.Sub(mgl64.Vec3{0, c.radius, 0}).Len(); drift > 1e-3 {
				t.Errorf("the ball moved by %.2g m", drift)
			}
		})
	}
}

// Without spinning resistance, a top keeps its spin: a ball of radius 10 cm at 20 rad/s around the vertical still spins
// at 20 rad/s after 10 s (± spinKept), awake, and its single point never holds any twist. The same for a smaller ball,
// for the ball added before the ground (the other body of the contact), on a tilted plane, on a box resting on the
// ground, and for a capsule standing on its cap
func TestTopKeepsItsSpinWithoutSpinningResistance(t *testing.T) {
	tilted := mgl64.Vec3{0.36, 0.48, 0.8}
	cases := []struct {
		name  string
		axis  mgl64.Vec3
		scene func() (*World, *actor.RigidBody)
	}{
		{"ball", mgl64.Vec3{0, 1, 0}, func() (*World, *actor.RigidBody) { return spinScene(1, spinRadius, 0, 0, true) }},
		{"ball before the ground", mgl64.Vec3{0, 1, 0}, func() (*World, *actor.RigidBody) { return spinScene(1, spinRadius, 0, 0, false) }},
		{"diameter of 10 cm", mgl64.Vec3{0, 1, 0}, func() (*World, *actor.RigidBody) { return spinScene(1, spinRadius/2, 0, 0, true) }},
		{"tilted plane", tilted, func() (*World, *actor.RigidBody) { return spinSceneOn(tilted, 0) }},
		{"ball on a box", mgl64.Vec3{0, 1, 0}, func() (*World, *actor.RigidBody) {
			w := newScene(1)
			addGround(w, 0.8)
			addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.8, 0)
			ball := addBody(w, mgl64.Vec3{0, 2*cubeHalf + spinRadius, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
			simulate(w, 0.25, nil)
			return w, ball
		}},
		{"standing capsule", mgl64.Vec3{0, 1, 0}, func() (*World, *actor.RigidBody) {
			const halfHeight = 0.2
			w := newScene(1)
			addGround(w, 0.8)
			capsule := addBody(w, mgl64.Vec3{0, halfHeight + spinRadius, 0}, mgl64.QuatIdent(),
				&actor.Capsule{HalfHeight: halfHeight, Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
			simulate(w, 0.25, nil)
			return w, capsule
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			w, top := c.scene()
			top.AngularVelocity = c.axis.Mul(spinSpeed)
			single := 0
			simulate(w, 10, func() {
				for i := range w.contacts {
					if w.contacts[i].Count != 1 {
						continue
					}
					single++
					if w.contacts[i].TwistImpulse != 0 {
						t.Fatalf("the single point of the top holds a twist of %.3g N·m·s", w.contacts[i].TwistImpulse)
					}
				}
			})
			if single < 600 {
				t.Fatalf("the top rested on a single point during %d steps of 600: the scene tests nothing", single)
			}
			speed := top.AngularVelocity.Dot(c.axis)
			t.Logf("spinning at %.12f rad/s after 10 s", speed)
			if math.Abs(speed-spinSpeed) > spinKept {
				t.Errorf("spinning at %.12f rad/s after 10 s, want %v (lost %.3g rad/s)", speed, spinSpeed, spinSpeed-speed)
			}
			if top.IsSleeping {
				t.Error("the spinning top fell asleep")
			}
		})
	}
}

// The spinning resistance acts around the normal only: a rolling ball is not slowed down by it, with or without
// rolling resistance
func TestSpinningResistanceDoesNotBrakeRolling(t *testing.T) {
	roll := func(rolling, spinning, seconds float64) (float64, float64) {
		w := newScene(1)
		ground := addGround(w, 0.8)
		ground.Material.RollingResistance, ground.Material.SpinningResistance = rolling, spinning
		ball := addBody(w, mgl64.Vec3{0, spinRadius, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
		ball.Material.SpinningResistance = spinning
		ball.Velocity = mgl64.Vec3{2, 0, 0}
		ball.AngularVelocity = mgl64.Vec3{0, 0, -2 / spinRadius}
		simulate(w, seconds, nil)
		return ball.Velocity.X(), ball.Transform.Position.X()
	}
	for _, c := range []struct{ rolling, seconds float64 }{{0, 6}, {0.1, 1}, {0.1, 6}} {
		speed, distance := roll(c.rolling, 0, c.seconds)
		speedWith, distanceWith := roll(c.rolling, spinResistance, c.seconds)
		t.Logf("rolling resistance %.1f, %.0f s: %.6f m/s at %.4f m, with spinning resistance %.6f m/s at %.4f m",
			c.rolling, c.seconds, speed, distance, speedWith, distanceWith)
		if math.Abs(speedWith-speed) > 1e-3 {
			t.Errorf("rolling resistance %.1f, %.0f s: %.6f m/s with spinning resistance, %.6f without", c.rolling, c.seconds, speedWith, speed)
		}
		if math.Abs(distanceWith-distance) > 1e-3 {
			t.Errorf("rolling resistance %.1f, %.0f s: rolled %.4f m with spinning resistance, %.4f without", c.rolling, c.seconds, distanceWith, distance)
		}
	}
}

// A box has no radius: its twist friction comes from the lever arms of its points, the resistance changes nothing
func TestSpinningResistanceLeavesBoxesUnchanged(t *testing.T) {
	run := func(resistance float64) []mgl64.Vec3 {
		w := newScene(1)
		addGround(w, 0.6).Material.SpinningResistance = resistance
		box := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		box.Material.SpinningResistance = resistance
		box.AngularVelocity = mgl64.Vec3{0, 5, 0}
		var out []mgl64.Vec3
		simulate(w, 2, func() {
			out = append(out, box.Transform.Position, box.Transform.Rotation.V, box.Velocity, box.AngularVelocity)
		})
		return out
	}
	reference, got := run(0), run(spinResistance)
	for i := range reference {
		if got[i] != reference[i] {
			t.Fatalf("value %d is %v with a spinning resistance, %v without", i, got[i], reference[i])
		}
	}
}

// A ball rolling and spinning at once: both resistances brake it, each on its axes, and the contact keeps each impulse
// (the rolling one along the tangents, the spinning one around the normal)
func TestRollingAndSpinningResistances(t *testing.T) {
	const rolling = 0.1
	w := newScene(1)
	ground := addGround(w, 0.8)
	ground.Material.RollingResistance, ground.Material.SpinningResistance = rolling, spinResistance
	ball := addBody(w, mgl64.Vec3{0, spinRadius, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
	ball.Velocity = mgl64.Vec3{2, 0, 0}
	ball.AngularVelocity = mgl64.Vec3{0, spinSpeed, -2 / spinRadius}
	simulate(w, 1, nil)

	// rolling without slipping, braked at 5/7·c·g; spinning, braked at c·R·g / (2/5·R²)
	if got, want := ball.Velocity.X(), 2-5.0/7.0*rolling*sceneGravity; math.Abs(got-want) > 0.01*want {
		t.Errorf("rolls at %.4f m/s after 1 s, want %.4f", got, want)
	}
	if got, want := ball.AngularVelocity.Y(), spinSpeed-spinDeceleration(spinResistance, spinRadius); math.Abs(got-want) > 0.02*want {
		t.Errorf("spins at %.4f rad/s after 1 s, want %.4f", got, want)
	}
	if len(w.contacts) != 1 {
		t.Fatalf("%d contacts, want 1", len(w.contacts))
	}
	// both impulses are at their bound: the resistance × the radius × the normal impulse of a sub-step
	manifold := &w.contacts[0]
	normalImpulse := ball.Material.GetMass() * sceneGravity * sceneDt / sceneSubsteps
	if got, want := math.Abs(manifold.SpinningImpulse), spinResistance*spinRadius*normalImpulse; math.Abs(got-want) > 0.01*want {
		t.Errorf("stored spinning impulse %.6g N·m·s, want %.6g", got, want)
	}
	if got, want := manifold.RollingImpulse.Len(), rolling*spinRadius*normalImpulse; math.Abs(got-want) > 0.01*want {
		t.Errorf("stored rolling impulse %.6g N·m·s, want %.6g", got, want)
	}
	// the rolling impulse lies along the tangents: the spinning impulse is not in it
	if around := manifold.RollingImpulse.Dot(manifold.Normal); math.Abs(around) > 1e-12*normalImpulse {
		t.Errorf("the rolling impulse has %.3g N·m·s around the normal", around)
	}
}

// A top with a rolling resistance too: the spinning impulse stays out of the rolling rows, the top brakes in place
// without rolling at all
func TestTopWithRollingResistanceStaysInPlace(t *testing.T) {
	w, ball := spinScene(1, spinRadius, spinResistance, 0, true)
	ball.Material.RollingResistance = 0.1
	ball.AngularVelocity = mgl64.Vec3{0, spinSpeed, 0}
	simulate(w, 1, func() {
		if len(w.contacts) != 1 {
			t.Fatalf("%d contacts, want 1", len(w.contacts))
		}
		if rolling := w.contacts[0].RollingImpulse.Len(); rolling > 1e-15 {
			t.Fatalf("the top has a rolling impulse of %.3g N·m·s", rolling)
		}
	})
	if got, want := ball.AngularVelocity.Y(), spinSpeed-spinDeceleration(spinResistance, spinRadius); math.Abs(got-want) > 0.02*want {
		t.Errorf("spins at %.4f rad/s after 1 s, want %.4f", got, want)
	}
	if side := math.Hypot(ball.Transform.Position.X(), ball.Transform.Position.Z()); side > 1e-12 {
		t.Errorf("the top moved by %.3g m", side)
	}
	// it turned around the vertical only
	if tilt := math.Hypot(ball.Transform.Rotation.V.X(), ball.Transform.Rotation.V.Z()); tilt > 1e-12 {
		t.Errorf("the axis of the top tilted: its rotation is %v", ball.Transform.Rotation)
	}
}

// On a tilted plane (the gravity tilted with it), the top brakes as on the ground: the spin is taken around the normal
// of the contact, whatever its axes. The normal has its 3 components, the largest along Z
func TestSpinningResistanceOnATiltedPlane(t *testing.T) {
	normal := mgl64.Vec3{0.36, 0.48, 0.8}
	want := spinDeceleration(spinResistance, spinRadius)

	w, ball := spinSceneOn(normal, spinResistance)
	ball.AngularVelocity = normal.Mul(spinSpeed)
	simulate(w, 0.5, func() {
		spin := ball.AngularVelocity.Dot(normal)
		if side := ball.AngularVelocity.Sub(normal.Mul(spin)).Len(); side > 1e-9 {
			t.Fatalf("the ball turns around a tangent: %v", ball.AngularVelocity)
		}
	})
	got := (spinSpeed - ball.AngularVelocity.Dot(normal)) / 0.5
	t.Logf("deceleration %.4f rad/s² on the tilted plane, analytic %.4f", got, want)
	if math.Abs(got-want) > spinTolerance*want {
		t.Errorf("deceleration %.4f rad/s² on the tilted plane, want %.4f", got, want)
	}
	// the same as on the ground, to the rounding of the turned scene
	flat, flatBall := spinScene(1, spinRadius, spinResistance, 0, true)
	flatBall.AngularVelocity = mgl64.Vec3{0, spinSpeed, 0}
	simulate(flat, 0.5, nil)
	if onGround := (spinSpeed - flatBall.AngularVelocity.Y()) / 0.5; math.Abs(got-onGround) > 1e-6*onGround {
		t.Errorf("deceleration %.9f rad/s² on the tilted plane, %.9f on the ground", got, onGround)
	}
	simulate(w, 2.5, nil)
	if !ball.IsSleeping {
		t.Errorf("the ball is still awake after 3 s, at %v rad/s", ball.AngularVelocity)
	}

	// a torque under the limit is held: the whole spin is seen by the row, the ball turns during the very first
	// sub-step only (τ h² / I, see TestSpinningResistanceHoldsATorque)
	w, ball = spinSceneOn(normal, spinResistance)
	limit := spinResistance * spinRadius * ball.Material.GetMass() * sceneGravity
	inertia := 2.0 / 5.0 * ball.Material.GetMass() * spinRadius * spinRadius
	h := sceneDt / sceneSubsteps
	for i := 0; i < 60; i++ {
		ball.AddTorque(normal.Mul(0.5 * limit))
		w.Step(sceneDt)
	}
	if angle, want := turnAngle(ball.Transform.Rotation), 0.5*limit*h*h/inertia; math.Abs(angle-want) > 0.01*want {
		t.Errorf("held torque: the ball turned by %.4g rad, want %.4g", angle, want)
	}
	if speed := ball.AngularVelocity.Len(); speed > 1e-9 {
		t.Errorf("held torque: the ball spins at %.3g rad/s", speed)
	}
}

// A capsule standing on its cap, spinning around its axis, is a top too: it brakes at c·R·m·g / I, I the inertia
// around its axis (a cylinder m·R²/2 and both halves of a sphere 2/5·m·R²), and falls asleep standing
func TestSpinningResistanceStopsAStandingCapsule(t *testing.T) {
	const halfHeight = 0.2
	w := newScene(1)
	addGround(w, 0.8)
	capsule := addBody(w, mgl64.Vec3{0, halfHeight + spinRadius, 0}, mgl64.QuatIdent(),
		&actor.Capsule{HalfHeight: halfHeight, Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
	capsule.Material.SpinningResistance = spinResistance
	simulate(w, 0.25, nil)
	capsule.AngularVelocity = mgl64.Vec3{0, spinSpeed, 0}

	// by unit of density
	cylinder, sphere := math.Pi*spinRadius*spinRadius*2*halfHeight, 4.0/3.0*math.Pi*spinRadius*spinRadius*spinRadius
	inertia := cylinder*spinRadius*spinRadius/2 + sphere*2.0/5.0*spinRadius*spinRadius
	want := spinResistance * spinRadius * (cylinder + sphere) * sceneGravity / inertia

	simulate(w, 0.5, nil)
	got := (spinSpeed - capsule.AngularVelocity.Y()) / 0.5
	elapsed, stopped := 0.5, -1.0
	simulate(w, 2.5, func() {
		elapsed += sceneDt
		if stopped < 0 && capsule.AngularVelocity.Len() < actor.DefaultSleepSpeed {
			stopped = elapsed
		}
	})
	t.Logf("deceleration %.4f rad/s², analytic %.4f; stopped at %.3f s, analytic %.3f s", got, want, stopped, spinSpeed/want)
	if math.Abs(got-want) > spinTolerance*want {
		t.Errorf("deceleration %.4f rad/s², want %.4f", got, want)
	}
	if math.Abs(stopped-spinSpeed/want) > 0.05*spinSpeed/want {
		t.Errorf("stopped at %.3f s, want %.3f s", stopped, spinSpeed/want)
	}
	if !capsule.IsSleeping {
		t.Errorf("the capsule is still awake after 3 s, at %v rad/s", capsule.AngularVelocity)
	}
	if axis := capsule.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}); axis.Y() < 1-1e-6 {
		t.Errorf("the capsule doesn't stand anymore: its axis is %v", axis)
	}
}

// The resistance removed while the contact keeps an impulse: the impulse is neither read nor applied, the step is the
// one of a contact which never had any, and the contact keeps none
func TestSpinningImpulseWithoutResistance(t *testing.T) {
	run := func(clear bool) (*actor.RigidBody, float64, float64) {
		w, ball := spinScene(1, spinRadius, spinResistance, 0, true)
		ball.AngularVelocity = mgl64.Vec3{0, spinSpeed, 0}
		simulate(w, 0.5, nil)
		if len(w.contacts) != 1 || w.contacts[0].SpinningImpulse == 0 {
			t.Fatalf("the braking ball has no spinning impulse: the scene tests nothing (%d contacts)", len(w.contacts))
		}
		before := ball.AngularVelocity.Y()
		ball.Material.SpinningResistance = 0
		if clear {
			w.contacts[0].SpinningImpulse = 0
		}
		w.Step(sceneDt)
		if len(w.contacts) != 1 {
			t.Fatalf("%d contacts, want 1", len(w.contacts))
		}
		return ball, before, w.contacts[0].SpinningImpulse
	}
	ball, before, stored := run(false)
	reference, _, _ := run(true)
	if ball.AngularVelocity != reference.AngularVelocity || ball.Velocity != reference.Velocity ||
		ball.Transform.Rotation != reference.Transform.Rotation || ball.Transform.Position != reference.Transform.Position {
		t.Errorf("the stored impulse moved the ball: it spins at %v, %v without it", ball.AngularVelocity, reference.AngularVelocity)
	}
	if stored != 0 {
		t.Errorf("the contact keeps a spinning impulse of %.3g N·m·s without resistance", stored)
	}
	// nothing brakes the step: neither the resistance, nor the twist of the single point
	if lost := before - ball.AngularVelocity.Y(); math.Abs(lost) > spinKept {
		t.Errorf("the ball lost %.4g rad/s in a step without resistance", lost)
	}
}

// ========== COULOMB ==========
// The resistance holds a torque under c·R·N without turning (the impulse is warm started: the ball doesn't creep
// between the sub-steps), and lets the rest of a larger torque through
func TestSpinningResistanceHoldsATorque(t *testing.T) {
	hold := func(factor float64) (*World, *actor.RigidBody, float64) {
		w, ball := spinScene(1, spinRadius, spinResistance, 0, true)
		limit := spinResistance * spinRadius * ball.Material.GetMass() * sceneGravity
		first := 0.0
		for i := 0; i < 60; i++ {
			ball.AddTorque(mgl64.Vec3{0, factor * limit, 0})
			w.Step(sceneDt)
			if i == 0 {
				first = turnAngle(ball.Transform.Rotation)
			}
		}
		return w, ball, first
	}

	w, ball, first := hold(0.5)
	limit := spinResistance * spinRadius * ball.Material.GetMass() * sceneGravity
	inertia := 2.0 / 5.0 * ball.Material.GetMass() * spinRadius * spinRadius
	if speed := ball.AngularVelocity.Len(); speed > 1e-9 {
		t.Errorf("held torque: the ball spins at %.3g rad/s", speed)
	}
	// the torque turns the ball during the very first sub-step only (no impulse to start from yet): τ h² / I
	h := sceneDt / sceneSubsteps
	if want := 0.5 * limit * h * h / inertia; math.Abs(first-want) > 0.01*want {
		t.Errorf("held torque: the ball turned by %.3g rad in the first step, want %.3g", first, want)
	}
	if angle := turnAngle(ball.Transform.Rotation); math.Abs(angle-first) > 1e-9 {
		t.Errorf("held torque: the ball turned by %.3g rad after the first step", angle-first)
	}
	if len(w.contacts) != 1 {
		t.Fatalf("%d contacts, want 1", len(w.contacts))
	}
	// the impulse of a sub-step holds the torque during this sub-step
	want := 0.5 * limit * h
	if got := math.Abs(w.contacts[0].SpinningImpulse); math.Abs(got-want) > 0.01*want {
		t.Errorf("stored spinning impulse %.6g N·m·s, want %.6g", got, want)
	}

	_, ball, _ = hold(2)
	want = (2*limit - limit) / inertia // after 1 s
	if got := ball.AngularVelocity.Y(); math.Abs(got-want) > 0.02*want {
		t.Errorf("torque of twice the limit: %.4f rad/s after 1 s, want %.4f", got, want)
	}
}

// ========== SOLVER ==========
// The row is an angular impulse around the normal, opposite on both bodies (the angular momentum is kept), bounded by
// the resistance times the normal impulse of the points
func TestSolveSpinning(t *testing.T) {
	normal := mgl64.Vec3{0, 1, 0}
	bodyA, bodyB := &actor.RigidBody{}, &actor.RigidBody{}
	newStates := func(spinA, spinB float64) (*bodyState, *bodyState) {
		return &bodyState{body: bodyA, dynamic: true, angularVelocity: mgl64.Vec3{0.3, spinA, 0}, inverseInertia: mgl64.Diag3(mgl64.Vec3{2, 2, 2})},
			&bodyState{body: bodyB, dynamic: true, angularVelocity: mgl64.Vec3{0, spinB, -0.7}, inverseInertia: mgl64.Diag3(mgl64.Vec3{4, 4, 4})}
	}
	newConstraint := func(resistance float64) *contactConstraint {
		c := &contactConstraint{normal: normal, pointsCount: 2, spinningResistance: resistance, twistMass: 1.0 / (2 + 4)}
		c.points[0].normalImpulse, c.points[1].normalImpulse = 3, 5
		return c
	}

	// sliding: the impulse is at its bound, resistance × Σ normal impulse, against the relative spin
	stateA, stateB := newStates(1, 11)
	c := newConstraint(0.1)
	c.solveSpinning(stateA, stateB)
	if want := -0.1 * (3 + 5); math.Abs(c.spinningImpulse-want) > 1e-15 {
		t.Errorf("sliding: impulse %v, want %v", c.spinningImpulse, want)
	}
	if got, want := stateA.angularVelocity, (mgl64.Vec3{0.3, 1 + 2*0.8, 0}); got.Sub(want).Len() > 1e-12 {
		t.Errorf("sliding: A spins at %v, want %v", got, want)
	}
	if got, want := stateB.angularVelocity, (mgl64.Vec3{0, 11 - 4*0.8, -0.7}); got.Sub(want).Len() > 1e-12 {
		t.Errorf("sliding: B spins at %v, want %v", got, want)
	}
	// the other way
	stateA, stateB = newStates(11, 1)
	c = newConstraint(0.1)
	c.solveSpinning(stateA, stateB)
	if want := 0.1 * (3 + 5); math.Abs(c.spinningImpulse-want) > 1e-15 {
		t.Errorf("sliding the other way: impulse %v, want %v", c.spinningImpulse, want)
	}

	// sticking: both bodies end with the same spin, the one which keeps their angular momentum
	stateA, stateB = newStates(1, 4)
	c = newConstraint(1)
	c.solveSpinning(stateA, stateB)
	shared := (1/2.0 + 4/4.0) / (1/2.0 + 1/4.0)
	if math.Abs(stateA.angularVelocity.Y()-shared) > 1e-12 || math.Abs(stateB.angularVelocity.Y()-shared) > 1e-12 {
		t.Errorf("sticking: spins %v & %v, want %v", stateA.angularVelocity.Y(), stateB.angularVelocity.Y(), shared)
	}
	// the accumulated impulse is bounded, not each of its increments
	stateB.angularVelocity[1] += 100
	c.spinningResistance = 0.1
	c.solveSpinning(stateA, stateB)
	if want := -0.1 * (3 + 5); math.Abs(c.spinningImpulse-want) > 1e-15 {
		t.Errorf("accumulated: impulse %v, want %v", c.spinningImpulse, want)
	}

	// no resistance: nothing moves, neither solved nor warm started
	stateA, stateB = newStates(1, 11)
	c = newConstraint(0)
	c.spinningImpulse = 0.5
	c.solveSpinning(stateA, stateB)
	if stateA.angularVelocity.Y() != 1 || stateB.angularVelocity.Y() != 11 {
		t.Errorf("no resistance: spins %v & %v, want 1 & 11", stateA.angularVelocity.Y(), stateB.angularVelocity.Y())
	}
	s := &solver{states: []bodyState{*stateA, *stateB}}
	c.indexA, c.indexB, c.pointsCount = 0, 1, 0
	s.warmStartConstraint(c)
	if s.states[0].angularVelocity.Y() != 1 || s.states[1].angularVelocity.Y() != 11 {
		t.Errorf("no resistance: warm started to %v & %v, want 1 & 11", s.states[0].angularVelocity.Y(), s.states[1].angularVelocity.Y())
	}
	// with a resistance, the warm start applies the impulse: -λ on A, +λ on B
	c.spinningResistance = 0.1
	s.warmStartConstraint(c)
	if got, want := s.states[0].angularVelocity.Y(), 1-2*0.5; math.Abs(got-want) > 1e-12 {
		t.Errorf("warm start: A spins at %v, want %v", got, want)
	}
	if got, want := s.states[1].angularVelocity.Y(), 11+4*0.5; math.Abs(got-want) > 1e-12 {
		t.Errorf("warm start: B spins at %v, want %v", got, want)
	}

	// a normal with its 3 components: the spin is the whole relative angular velocity along it. Sticking, both bodies
	// end with the same spin around the normal, and their angular velocity along the tangents is kept
	tilted := mgl64.Vec3{0.36, 0.48, 0.8}
	side := mgl64.Vec3{0.8, 0, -0.36}
	stateA = &bodyState{body: bodyA, dynamic: true, angularVelocity: tilted.Mul(1).Add(side), inverseInertia: mgl64.Diag3(mgl64.Vec3{2, 2, 2})}
	stateB = &bodyState{body: bodyB, dynamic: true, angularVelocity: tilted.Mul(4), inverseInertia: mgl64.Diag3(mgl64.Vec3{4, 4, 4})}
	c = newConstraint(1)
	c.normal = tilted
	c.solveSpinning(stateA, stateB)
	if got := stateA.angularVelocity.Dot(tilted); math.Abs(got-shared) > 1e-12 {
		t.Errorf("tilted normal: A spins at %v around it, want %v", got, shared)
	}
	if got := stateB.angularVelocity.Dot(tilted); math.Abs(got-shared) > 1e-12 {
		t.Errorf("tilted normal: B spins at %v around it, want %v", got, shared)
	}
	if got, want := stateA.angularVelocity.Sub(tilted.Mul(shared)), side; got.Sub(want).Len() > 1e-12 {
		t.Errorf("tilted normal: A turns at %v along the tangents, want %v", got, want)
	}
}

// relax solves the rows of a contact in this order: the normals, the rolling, the spinning, then the friction. The
// bound of the spinning comes from the normal impulses just solved, and the friction sees what both resistances left.
// For a ball, the rolling & the spinning don't see each other; for a tilted capsule they do: an impulse around a
// tangent changes its spin around the normal
func TestRelaxSolvesTheSpinningAfterTheRolling(t *testing.T) {
	w := newScene(1)
	ground := addGround(w, 0.8)
	ground.Material.RollingResistance, ground.Material.SpinningResistance = 0.1, spinResistance
	const halfHeight = 0.2
	lean := mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 0, 1})
	capsule := addBody(w, mgl64.Vec3{0, halfHeight*math.Cos(math.Pi/4) + spinRadius, 0}, lean,
		&actor.Capsule{HalfHeight: halfHeight, Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
	capsule.AngularVelocity = mgl64.Vec3{3, spinSpeed, -2}
	// until the capsule lands on its cap, tumbling
	s := &w.solver
	landed := func() bool { return len(s.constraints) == 1 && s.constraints[0].points[0].normalImpulse > 0 }
	for step := 0; step < 60 && !landed(); step++ {
		w.Step(sceneDt)
	}
	if !landed() || s.constraints[0].pointsCount != 1 {
		t.Fatalf("the capsule didn't land on its cap: %d contacts", len(s.constraints))
	}
	if axis := capsule.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}); math.Abs(axis.Y()) < 0.1 || math.Abs(axis.Y()) > 0.98 {
		t.Fatalf("the capsule is not tilted: its axis is %v", axis)
	}

	// the same contact and the same bodies, their velocities disturbed, solved once by each order
	prepared := s.constraints[0]
	states := slices.Clone(s.states)
	solve := func(rows func(c *contactConstraint, stateA, stateB *bodyState)) (mgl64.Vec3, mgl64.Vec3) {
		copy(s.states, states)
		c := prepared
		stateA, stateB := s.state(c.indexA), s.state(c.indexB)
		moving := stateB
		if stateA.body != nil {
			moving = stateA
		}
		moving.velocity = moving.velocity.Add(mgl64.Vec3{0.2, -0.3, 0.1})
		moving.angularVelocity = moving.angularVelocity.Add(mgl64.Vec3{1, 5, -2})
		rows(&c, stateA, stateB)
		return moving.velocity, moving.angularVelocity
	}
	defer copy(s.states, states)

	velocity, angular := solve(func(c *contactConstraint, _, _ *bodyState) { s.relaxConstraint(c) })
	orders := []struct {
		name string
		same bool
		rows func(c *contactConstraint, stateA, stateB *bodyState)
	}{
		{"normals, rolling, spinning, friction", true, func(c *contactConstraint, stateA, stateB *bodyState) {
			c.turnAnchors(stateA, stateB)
			s.solveNormals(c, stateA, stateB, false)
			c.solveRolling(stateA, stateB)
			c.solveSpinning(stateA, stateB)
			c.solveFriction(stateA, stateB)
		}},
		{"spinning before the rolling", false, func(c *contactConstraint, stateA, stateB *bodyState) {
			c.turnAnchors(stateA, stateB)
			s.solveNormals(c, stateA, stateB, false)
			c.solveSpinning(stateA, stateB)
			c.solveRolling(stateA, stateB)
			c.solveFriction(stateA, stateB)
		}},
		{"spinning before the normals", false, func(c *contactConstraint, stateA, stateB *bodyState) {
			c.turnAnchors(stateA, stateB)
			c.solveSpinning(stateA, stateB)
			s.solveNormals(c, stateA, stateB, false)
			c.solveRolling(stateA, stateB)
			c.solveFriction(stateA, stateB)
		}},
		{"spinning after the friction", false, func(c *contactConstraint, stateA, stateB *bodyState) {
			c.turnAnchors(stateA, stateB)
			s.solveNormals(c, stateA, stateB, false)
			c.solveRolling(stateA, stateB)
			c.solveFriction(stateA, stateB)
			c.solveSpinning(stateA, stateB)
		}},
	}
	for _, order := range orders {
		gotVelocity, gotAngular := solve(order.rows)
		if same := gotVelocity == velocity && gotAngular == angular; same != order.same {
			t.Errorf("%s: velocity %v, angular velocity %v; relax gives %v & %v (the same: %v, want %v)",
				order.name, gotVelocity, gotAngular, velocity, angular, same, order.same)
		}
	}
}

// ========== DETERMINISM & ALLOCATIONS ==========
// spinPile: a pile of balls, capsules and boxes thrown spinning, with rolling & spinning resistance
func spinPile(count, workers int) *World {
	w := benchScene(count, workers)
	w.parallelFrom = 1
	for i, body := range w.Bodies {
		body.Material.SpinningResistance = spinResistance
		body.Material.RollingResistance = 0.1
		if body.BodyType == actor.BodyTypeDynamic {
			body.AngularVelocity = mgl64.Vec3{0, float64(i%7) - 3, 0}.Mul(spinSpeed / 3)
		}
	}
	return w
}

func TestSpinningResistanceIsDeterministic(t *testing.T) {
	run := func(workers int) []mgl64.Vec3 {
		w := spinPile(200, workers)
		simulate(w, 2, nil)
		var out []mgl64.Vec3
		for _, b := range w.Bodies {
			out = append(out, b.Transform.Position, b.Transform.Rotation.V, b.Velocity, b.AngularVelocity)
		}
		for i := range w.contacts {
			out = append(out, mgl64.Vec3{w.contacts[i].SpinningImpulse, w.contacts[i].TwistImpulse, 0})
		}
		return out
	}
	reference := run(1)
	spinning := 0
	for _, value := range reference[4*201:] {
		if value[0] != 0 {
			spinning++
		}
	}
	if spinning == 0 {
		t.Fatal("no contact resists a spin: the scene tests nothing")
	}
	for _, workers := range []int{2, 8} {
		got := run(workers)
		if len(got) != len(reference) {
			t.Fatalf("workers=%d: %d values, want %d", workers, len(got), len(reference))
		}
		for i := range reference {
			if got[i] != reference[i] {
				t.Fatalf("workers=%d: value %d is %v, want %v", workers, i, got[i], reference[i])
			}
		}
	}
}

// 400 tops braking on the ground, each on its own contact: the count of bodies & contacts doesn't change, no buffer
// grows during the measure (a pile still moving grows its buffers, with or without spinning resistance)
func TestSpinningResistanceDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	const side = 20
	for _, workers := range []int{1, 8} {
		w := newScene(workers)
		w.parallelFrom = 1
		addGround(w, 0.8)
		for i := 0; i < side*side; i++ {
			ball := addBody(w, mgl64.Vec3{float64(i%side) * 0.5, spinRadius, float64(i/side) * 0.5}, mgl64.QuatIdent(),
				&actor.Sphere{Radius: spinRadius}, actor.BodyTypeDynamic, 0.8, 0)
			ball.Material.SpinningResistance = spinResistance
			ball.AngularVelocity = mgl64.Vec3{0, spinSpeed, 0}
		}
		simulate(w, 0.4, nil)
		allocs := testing.AllocsPerRun(10, func() { w.Step(sceneDt) })
		if allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
		// the steps measured did brake the tops
		braking := 0
		for i := range w.contacts {
			if w.contacts[i].SpinningImpulse != 0 {
				braking++
			}
		}
		if braking != side*side {
			t.Errorf("workers=%d: %d contacts brake a top after the measure, want %d", workers, braking, side*side)
		}
		for _, body := range w.Bodies[1:] {
			if speed := body.AngularVelocity.Y(); speed <= 1 || speed >= spinSpeed-5 {
				t.Fatalf("workers=%d: a top spins at %.2f rad/s after the measure, it should be braking", workers, speed)
			}
		}
	}
}
