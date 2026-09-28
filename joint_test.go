package feather

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

func anchorBody(w *World, position mgl64.Vec3) *actor.RigidBody {
	return addBody(w, position, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.02}, actor.BodyTypeStatic, 0, 0)
}

func degrees(radians float64) float64 { return radians * 180 / math.Pi }

// A pendulum: the period of a physical pendulum, 2π sqrt(I / (m g L)), within 1% (small angles)
func TestJointPendulumPeriod(t *testing.T) {
	const length, radius, angle = 1.0, 0.1, 0.1
	w := newScene(1)
	pivot := mgl64.Vec3{0, 2, 0}
	anchor := anchorBody(w, pivot)
	bob := addBody(w, pivot.Add(mgl64.Vec3{length * math.Sin(angle), -length * math.Cos(angle), 0}), mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 0)
	w.AddJoint(NewBallJoint(anchor, bob, pivot, mgl64.Vec3{0, -1, 0}))

	// zero crossings of x, going to the right
	var crossings []float64
	elapsed, previous := 0.0, bob.Transform.Position.X()
	simulate(w, 12, func() {
		elapsed += sceneDt
		x := bob.Transform.Position.X()
		if previous < 0 && x >= 0 {
			crossings = append(crossings, elapsed-sceneDt*x/(x-previous))
		}
		previous = x
	})
	if len(crossings) < 4 {
		t.Fatalf("only %d crossings", len(crossings))
	}
	period := (crossings[len(crossings)-1] - crossings[0]) / float64(len(crossings)-1)
	inertia := 0.4*radius*radius + length*length
	want := 2 * math.Pi * math.Sqrt(inertia/(sceneGravity*length)) * (1 + angle*angle/16)
	t.Logf("period %.4f s, theory %.4f s", period, want)
	if math.Abs(period-want) > 0.01*want {
		t.Errorf("period %.4f s, want %.4f s (1%%)", period, want)
	}
	if d := bob.Transform.Position.Sub(pivot).Len(); math.Abs(d-length) > 0.001 {
		t.Errorf("the bob is %.4f m from the pivot, want %.4f", d, length)
	}
}

// chain of capsules hanging from an anchor, linked by ball joints
func hangingChain(w *World, links int) (*actor.RigidBody, []*actor.RigidBody, []*BallJoint) {
	const halfHeight, radius = 0.2, 0.05
	top := mgl64.Vec3{0, 5, 0}
	anchor := anchorBody(w, top)
	previous := anchor
	var capsules []*actor.RigidBody
	var joints []*BallJoint
	for i := 0; i < links; i++ {
		joint := top.Sub(mgl64.Vec3{0, float64(i) * 2 * halfHeight, 0})
		capsule := addBody(w, joint.Sub(mgl64.Vec3{0, halfHeight, 0}), mgl64.QuatIdent(), &actor.Capsule{HalfHeight: halfHeight, Radius: radius}, actor.BodyTypeDynamic, 0.5, 0)
		ball := NewBallJoint(previous, capsule, joint, mgl64.Vec3{0, -1, 0})
		w.AddJoint(ball)
		joints = append(joints, ball)
		capsules = append(capsules, capsule)
		previous = capsule
	}
	return anchor, capsules, joints
}

// jointGap: distance between the anchors of both bodies
func jointGap(j *JointBase) float64 {
	return j.BodyA.Transform.ToWorld(j.LocalFrameA.Position).Sub(j.BodyB.Transform.ToWorld(j.LocalFrameB.Position)).Len()
}

// A chain of 5 capsules stays at rest, and doesn't explode when it is shaken
func TestJointChain(t *testing.T) {
	w := newScene(1)
	_, capsules, joints := hangingChain(w, 5)
	simulate(w, 2, nil)
	start := make([]mgl64.Vec3, len(capsules))
	for i, c := range capsules {
		start[i] = c.Transform.Position
	}
	simulate(w, 10, nil)
	for i, c := range capsules {
		if d := c.Transform.Position.Sub(start[i]).Len(); d > 0.0001 {
			t.Errorf("capsule %d drifted %.3f mm at rest", i, d*1000)
		}
	}

	// shaking: random impulses on the whole chain
	r := rand.New(rand.NewSource(3))
	worstGap := 0.0
	simulate(w, 3, func() {
		for _, c := range capsules {
			c.AddImpulse(mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Mul(0.3))
		}
		for _, j := range joints {
			worstGap = math.Max(worstGap, jointGap(&j.JointBase))
		}
	})
	for i, c := range capsules {
		if !finite(c.Transform.Position) || c.Velocity.Len() > 50 {
			t.Fatalf("capsule %d exploded: position %v velocity %v", i, c.Transform.Position, c.Velocity)
		}
	}
	t.Logf("worst gap in the joints while shaking: %.2f mm", worstGap*1000)
	if worstGap > 0.01 {
		t.Errorf("the joints opened by %.1f mm while shaking", worstGap*1000)
	}
	// after the shaking, no damping: the chain keeps swinging, but its energy must not grow
	energy := func() float64 {
		e := 0.0
		for _, c := range capsules {
			m := c.Material.GetMass()
			e += 0.5*m*c.Velocity.LenSqr() + 0.5*c.AngularVelocity.Dot(c.GetInertiaWorld().Mul3x1(c.AngularVelocity)) + m*sceneGravity*c.Transform.Position.Y()
		}
		return e
	}
	start0 := energy()
	worstEnergy := start0
	simulate(w, 5, func() { worstEnergy = math.Max(worstEnergy, energy()) })
	t.Logf("energy after the shaking %.3f J, highest in the next 5 s %.3f J", start0, worstEnergy)
	if worstEnergy > start0+0.01*math.Abs(start0) {
		t.Errorf("the chain gained energy: %.3f J -> %.3f J", start0, worstEnergy)
	}
}

// A chain of 10 links carrying a ball 100 times heavier, released horizontal: while it swings, no joint opens by more
// than 1 % of a link. Solved one by one, the spring of each joint acts on the mass of a link, and the ball stretches the
// chain (articulation.go)
func TestHeavyChainDoesNotStretch(t *testing.T) {
	const links, halfHeight, radius, ratio = 10, 0.2, 0.05, 100
	w := newScene(1)
	top := mgl64.Vec3{0, 6, 0}
	previous := anchorBody(w, top)
	lying := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	var joints []*BallJoint
	for i := 0; i < links; i++ {
		joint := top.Add(mgl64.Vec3{float64(i) * 2 * halfHeight, 0, 0})
		link := addBody(w, joint.Add(mgl64.Vec3{halfHeight, 0, 0}), lying, &actor.Capsule{HalfHeight: halfHeight, Radius: radius}, actor.BodyTypeDynamic, 0.5, 0)
		ball := NewBallJoint(previous, link, joint, mgl64.Vec3{1, 0, 0})
		w.AddJoint(ball)
		joints = append(joints, ball)
		previous = link
	}
	end := top.Add(mgl64.Vec3{float64(links) * 2 * halfHeight, 0, 0})
	linkMass := previous.Material.GetMass()
	const ballRadius = 0.2
	density := ratio * linkMass / (4.0 / 3 * math.Pi * ballRadius * ballRadius * ballRadius)
	ball := actor.NewRigidBody(actor.Transform{Position: end.Add(mgl64.Vec3{ballRadius, 0, 0}), Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: ballRadius}, actor.BodyTypeDynamic, density)
	w.AddBody(ball)
	w.AddJoint(NewBallJoint(previous, ball, end, mgl64.Vec3{1, 0, 0}))
	joints = append(joints, w.Joints[len(w.Joints)-1].(*BallJoint))

	worstGap := 0.0
	simulate(w, 4, func() {
		for _, j := range joints {
			worstGap = math.Max(worstGap, jointGap(&j.JointBase))
		}
	})
	t.Logf("ball %.1f kg, link %.2f kg: worst gap %.3f mm", ball.Material.GetMass(), linkMass, worstGap*1000)
	if worstGap > 0.01*2*halfHeight {
		t.Errorf("a joint opened by %.2f mm, more than 1 %% of a link (%.1f mm)", worstGap*1000, 0.01*2*halfHeight*1000)
	}
}

// A door on a hinge: it only turns around its axis, and stays within its limits, even pushed hard
func TestJointHingeLimits(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	frame := anchorBody(w, mgl64.Vec3{0, 1, 0})
	door := addBody(w, mgl64.Vec3{0.5, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 0.05}}, actor.BodyTypeDynamic, 0.5, 0)
	hinge := NewHingeJoint(frame, door, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{0, 1, 0})
	hinge.EnableLimit, hinge.LowerAngle, hinge.UpperAngle = true, -0.5, 1.0
	w.AddJoint(hinge)

	worstLimit, worstAxis := 0.0, 0.0
	check := func() {
		angle := hinge.Angle()
		worstLimit = math.Max(worstLimit, math.Max(hinge.LowerAngle-angle, angle-hinge.UpperAngle))
		axis := door.Transform.Rotation.Mul(hinge.LocalFrameB.Rotation).Rotate(mgl64.Vec3{1, 0, 0})
		worstAxis = math.Max(worstAxis, math.Acos(math.Min(1, axis.Dot(mgl64.Vec3{0, 1, 0}))))
	}
	for _, torque := range []mgl64.Vec3{{0, 200, 0}, {0, -200, 0}, {150, 0, 150}} {
		simulate(w, 1.5, func() {
			door.AddTorque(torque)
			check()
		})
	}
	t.Logf("worst limit overshoot %.3f°, worst axis tilt %.3f°", degrees(worstLimit), degrees(worstAxis))
	if degrees(worstLimit) > 0.5 {
		t.Errorf("the door went %.2f° beyond its limits", degrees(worstLimit))
	}
	if degrees(worstAxis) > 0.5 {
		t.Errorf("the hinge axis tilted by %.2f°", degrees(worstAxis))
	}
}

// A ball joint with an elliptic cone and a twist range: never exceeded by more than 0.5°
func TestJointBallLimits(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	pivot := mgl64.Vec3{0, 2, 0}
	anchor := anchorBody(w, pivot)
	arm := addBody(w, pivot.Add(mgl64.Vec3{0, -0.5, 0}), mgl64.QuatIdent(), &actor.Capsule{HalfHeight: 0.4, Radius: 0.05}, actor.BodyTypeDynamic, 0.5, 0)
	ball := NewBallJoint(anchor, arm, pivot, mgl64.Vec3{0, -1, 0})
	ball.EnableSwingLimit, ball.SwingLimitY, ball.SwingLimitZ = true, 20*math.Pi/180, 40*math.Pi/180
	ball.EnableTwistLimit, ball.TwistMin, ball.TwistMax = true, -10*math.Pi/180, 10*math.Pi/180
	w.AddJoint(ball)

	worstSwing, worstTwist := 0.0, 0.0
	check := func() {
		frameA := anchor.Transform.Rotation.Mul(ball.LocalFrameA.Rotation)
		frameB := arm.Transform.Rotation.Mul(ball.LocalFrameB.Rotation)
		twist := twistAngle(frameA, frameB)
		// the X axis of B, in the frame A: its angle beyond the ellipse, at its direction
		p := frameA.Conjugate().Rotate(frameB.Rotate(mgl64.Vec3{1, 0, 0}))
		if r := math.Hypot(p.Y(), p.Z()); r > 1e-9 {
			angle := math.Atan2(r, p.X())
			limit := 1 / math.Hypot(p.Y()/r/ball.SwingLimitZ, p.Z()/r/ball.SwingLimitY)
			worstSwing = math.Max(worstSwing, angle-limit)
		}
		worstTwist = math.Max(worstTwist, math.Max(ball.TwistMin-twist, twist-ball.TwistMax))
	}
	bottom := func() mgl64.Vec3 { return arm.Transform.ToWorld(mgl64.Vec3{0, -0.45, 0}) }
	for _, push := range []mgl64.Vec3{{50, 0, 0}, {-50, 0, 0}, {0, 0, 50}, {0, 0, -50}, {35, 0, 35}} {
		simulate(w, 1, func() {
			arm.AddForceAtPoint(push, bottom())
			arm.AddTorque(arm.Transform.Rotation.Rotate(mgl64.Vec3{0, 3, 0}))
			check()
		})
	}
	simulate(w, 1, func() {
		arm.AddTorque(arm.Transform.Rotation.Rotate(mgl64.Vec3{0, -3, 0}))
		check()
	})
	t.Logf("worst swing overshoot %.3f°, worst twist overshoot %.3f°", degrees(worstSwing), degrees(worstTwist))
	if degrees(worstSwing) > 0.5 || degrees(worstTwist) > 0.5 {
		t.Errorf("limits exceeded: swing %.2f°, twist %.2f°", degrees(worstSwing), degrees(worstTwist))
	}
}

// A distance joint with a spring: the body oscillates at the frequency of the spring
func TestJointSpringFrequency(t *testing.T) {
	const hertz = 2.0
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	anchor := anchorBody(w, mgl64.Vec3{0, 0, 0})
	body := addBody(w, mgl64.Vec3{1, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0, 0)
	spring := NewDistanceJoint(anchor, body, mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 0, 0})
	spring.EnableSpring, spring.SpringHertz, spring.SpringDampingRatio = true, hertz, 0
	w.AddJoint(spring)
	body.Transform.Position = mgl64.Vec3{1.2, 0, 0} // stretched by 20 cm

	var crossings []float64
	elapsed, previous := 0.0, body.Transform.Position.X()-1
	simulate(w, 5, func() {
		elapsed += sceneDt
		x := body.Transform.Position.X() - 1
		if previous > 0 && x <= 0 {
			crossings = append(crossings, elapsed-sceneDt*x/(x-previous))
		}
		previous = x
	})
	if len(crossings) < 3 {
		t.Fatalf("only %d crossings", len(crossings))
	}
	frequency := float64(len(crossings)-1) / (crossings[len(crossings)-1] - crossings[0])
	t.Logf("frequency %.4f Hz, spring %.1f Hz", frequency, hertz)
	if math.Abs(frequency-hertz) > 0.03*hertz {
		t.Errorf("frequency %.3f Hz, want %.1f Hz", frequency, hertz)
	}
}

// A rope (distance in [0, L]): the ball falls freely, then hangs at L
func TestJointRope(t *testing.T) {
	w := newScene(1)
	anchor := anchorBody(w, mgl64.Vec3{0, 5, 0})
	ball := addBody(w, mgl64.Vec3{0.5, 5, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0, 0)
	rope := NewDistanceJoint(anchor, ball, mgl64.Vec3{0, 5, 0}, mgl64.Vec3{0.5, 5, 0})
	rope.EnableSpring, rope.EnableLimit, rope.MinLength, rope.MaxLength = true, true, 0, 2
	w.AddJoint(rope)
	simulate(w, 0.2, nil)
	if y := ball.Transform.Position.Y(); math.Abs(y-(5-0.5*sceneGravity*0.04)) > 0.01 {
		t.Errorf("free fall: y=%.3f, want %.3f", y, 5-0.5*sceneGravity*0.04)
	}
	simulate(w, 8, nil)
	if d := ball.Transform.Position.Sub(mgl64.Vec3{0, 5, 0}).Len(); math.Abs(d-2) > 0.002 {
		t.Errorf("the rope is %.4f m long, want 2", d)
	}
}

// A fixed joint: 2 boxes stay welded, even hit
func TestJointFixed(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	a := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	b := addBody(w, mgl64.Vec3{0.5, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	w.AddJoint(NewFixedJoint(a, b, mgl64.Vec3{0.25, 0, 0}))
	a.AddImpulseAtPoint(mgl64.Vec3{0, 20, 0}, mgl64.Vec3{-0.25, 0, 0.2})
	worstPosition, worstAngle := 0.0, 0.0
	simulate(w, 3, func() {
		relative := a.Transform.ToLocal(b.Transform.Position)
		worstPosition = math.Max(worstPosition, relative.Sub(mgl64.Vec3{0.5, 0, 0}).Len())
		rotation := a.Transform.Rotation.Conjugate().Mul(b.Transform.Rotation)
		worstAngle = math.Max(worstAngle, 2*math.Acos(math.Min(1, math.Abs(rotation.W))))
	})
	t.Logf("worst %.3f mm, %.3f°", worstPosition*1000, degrees(worstAngle))
	if worstPosition > 0.001 || degrees(worstAngle) > 0.5 {
		t.Errorf("the weld moved by %.2f mm and %.2f°", worstPosition*1000, degrees(worstAngle))
	}
	if a.AngularVelocity.Len() < 1 {
		t.Error("the welded pair did not spin")
	}
}

// Motors: the hinge motor reaches its speed, the ball drive brings the body to its target without overshoot
// (critical damping)
func TestJointMotors(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	axle := anchorBody(w, mgl64.Vec3{})
	wheel := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Capsule{HalfHeight: 0.3, Radius: 0.2}, actor.BodyTypeDynamic, 0.5, 0)
	hinge := NewHingeJoint(axle, wheel, mgl64.Vec3{}, mgl64.Vec3{0, 1, 0})
	hinge.EnableMotor, hinge.MotorSpeed, hinge.MaxMotorTorque = true, 3, 100
	w.AddJoint(hinge)
	simulate(w, 1, nil)
	if speed := wheel.AngularVelocity.Y(); math.Abs(speed-3) > 0.01 {
		t.Errorf("motor speed %.3f rad/s, want 3", speed)
	}

	w2 := newScene(1)
	w2.Gravity = mgl64.Vec3{}
	pivot := anchorBody(w2, mgl64.Vec3{})
	arm := addBody(w2, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.1, 0.1}}, actor.BodyTypeDynamic, 0.5, 0)
	drive := NewBallJoint(pivot, arm, mgl64.Vec3{}, mgl64.Vec3{1, 0, 0})
	target := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	drive.EnableDrive, drive.DriveTarget, drive.DriveHertz, drive.DriveDampingRatio = true, target, 2, 1
	w2.AddJoint(drive)
	overshoot := 0.0
	simulate(w2, 3, func() {
		angle := 2 * math.Atan2(arm.Transform.Rotation.V.Z(), arm.Transform.Rotation.W)
		overshoot = math.Max(overshoot, angle-math.Pi/2)
	})
	finalError := degrees(2 * math.Acos(math.Min(1, math.Abs(arm.Transform.Rotation.Dot(target)))))
	t.Logf("drive: final error %.3f°, overshoot %.3f°", finalError, degrees(overshoot))
	if finalError > 0.5 {
		t.Errorf("the drive did not reach its target: %.2f° away", finalError)
	}
	if degrees(overshoot) > 1 {
		t.Errorf("critically damped drive overshot by %.2f°", degrees(overshoot))
	}
}

// 2 bodies linked by a joint don't collide by default; with CollideConnected they do
func TestJointCollideConnected(t *testing.T) {
	for _, collide := range []bool{false, true} {
		w := newScene(1)
		w.Gravity = mgl64.Vec3{}
		a := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
		b := addBody(w, mgl64.Vec3{0.4, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
		joint := NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position)
		joint.CollideConnected = collide
		w.AddJoint(joint)
		w.Step(sceneDt)
		if contacts := len(w.Contacts()); (contacts > 0) != collide {
			t.Errorf("CollideConnected=%v: %d contacts", collide, contacts)
		}
	}
}

// Joints don't allocate after the first steps
func TestJointsDoNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	w := newScene(1)
	hangingChain(w, 10)
	simulate(w, 0.2, nil)
	if allocs := testing.AllocsPerRun(10, func() { w.Step(sceneDt) }); allocs > 0 {
		t.Errorf("%.1f allocations per step", allocs)
	}
}
