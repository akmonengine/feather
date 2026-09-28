package feather

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// A slider: the block only slides along X of the frame (on a 30° slope, pulled by gravity), and stops at the limits
func TestConfigurableSlider(t *testing.T) {
	w := newScene(1)
	rail := anchorBody(w, mgl64.Vec3{0, 3, 0})
	slope := mgl64.Vec3{math.Cos(math.Pi / 6), -math.Sin(math.Pi / 6), 0}
	block := addBody(w, mgl64.Vec3{0, 3, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	slider := NewConfigurableJoint(rail, block, mgl64.Vec3{0, 3, 0}, slope)
	slider.LinearMotion = [3]Motion{MotionLimited, MotionLocked, MotionLocked}
	slider.LinearMin, slider.LinearMax = mgl64.Vec3{-0.5, 0, 0}, mgl64.Vec3{1.5, 0, 0}
	w.AddJoint(slider)

	worstOff, worstBeyond, worstTurn := 0.0, 0.0, 0.0
	simulate(w, 3, func() {
		offset := block.Transform.Position.Sub(mgl64.Vec3{0, 3, 0})
		along := offset.Dot(slope)
		worstOff = math.Max(worstOff, offset.Sub(slope.Mul(along)).Len())
		worstBeyond = math.Max(worstBeyond, along-1.5)
		worstTurn = math.Max(worstTurn, 2*math.Acos(math.Min(1, math.Abs(block.Transform.Rotation.W))))
	})
	along := block.Transform.Position.Sub(mgl64.Vec3{0, 3, 0}).Dot(slope)
	t.Logf("stopped at %.4f m (limit 1.5), off the axis %.3f mm, beyond the limit %.3f mm, turned %.3f°", along, worstOff*1000, worstBeyond*1000, degrees(worstTurn))
	if math.Abs(along-1.5) > 0.002 || worstOff > 0.001 || worstBeyond > 0.002 || degrees(worstTurn) > 0.5 {
		t.Error("the slider did not hold")
	}
}

// The configurable joint set as a hinge (only the twist free) keeps its axis like HingeJoint
func TestConfigurableHinge(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	frame := anchorBody(w, mgl64.Vec3{0, 1, 0})
	door := addBody(w, mgl64.Vec3{0.5, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 0.05}}, actor.BodyTypeDynamic, 0.5, 0)
	hinge := NewConfigurableJoint(frame, door, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{0, 1, 0})
	hinge.TwistMotion = MotionFree
	w.AddJoint(hinge)

	worstAxis := 0.0
	simulate(w, 2, func() {
		door.AddTorque(mgl64.Vec3{100, 60, 100})
		axis := door.Transform.Rotation.Mul(hinge.LocalFrameB.Rotation).Rotate(mgl64.Vec3{1, 0, 0})
		worstAxis = math.Max(worstAxis, math.Acos(math.Min(1, axis.Dot(mgl64.Vec3{0, 1, 0}))))
	})
	t.Logf("axis tilt %.3f°, spin %.2f rad/s", degrees(worstAxis), door.AngularVelocity.Y())
	if degrees(worstAxis) > 0.5 || door.AngularVelocity.Y() < 1 {
		t.Errorf("axis tilt %.2f°, spin %.2f rad/s", degrees(worstAxis), door.AngularVelocity.Y())
	}
}

// One swing limited, the other locked, the twist locked: the arm only swings around Y, within its limit
func TestConfigurableSwingAxes(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	pivot := mgl64.Vec3{0, 2, 0}
	anchor := anchorBody(w, pivot)
	arm := addBody(w, pivot.Add(mgl64.Vec3{0, -0.5, 0}), mgl64.QuatIdent(), &actor.Capsule{HalfHeight: 0.4, Radius: 0.05}, actor.BodyTypeDynamic, 0.5, 0)
	joint := NewConfigurableJoint(anchor, arm, pivot, mgl64.Vec3{0, -1, 0})
	joint.SwingYMotion, joint.SwingLimitY = MotionLimited, 20*math.Pi/180
	w.AddJoint(joint)

	worstY, worstZ, worstTwist := 0.0, 0.0, 0.0
	for _, push := range []mgl64.Vec3{{40, 0, 0}, {-40, 0, 0}, {0, 0, 40}, {0, 0, -40}, {30, 0, 30}} {
		simulate(w, 1, func() {
			arm.AddForceAtPoint(push, arm.Transform.ToWorld(mgl64.Vec3{0, -0.45, 0}))
			arm.AddTorque(arm.Transform.Rotation.Rotate(mgl64.Vec3{0, 2, 0}))
			frameA := anchor.Transform.Rotation.Mul(joint.LocalFrameA.Rotation)
			frameB := arm.Transform.Rotation.Mul(joint.LocalFrameB.Rotation)
			p := frameA.Conjugate().Rotate(frameB.Rotate(mgl64.Vec3{1, 0, 0}))
			worstY = math.Max(worstY, math.Abs(swingAngles[0](p))-joint.SwingLimitY)
			worstZ = math.Max(worstZ, math.Abs(swingAngles[1](p)))
			worstTwist = math.Max(worstTwist, math.Abs(twistAngle(frameA, frameB)))
		})
	}
	t.Logf("swing Y beyond its limit %.3f°, swing Z %.3f° (locked), twist %.3f° (locked)", degrees(worstY), degrees(worstZ), degrees(worstTwist))
	if degrees(worstY) > 0.5 || degrees(worstZ) > 0.5 || degrees(worstTwist) > 0.5 {
		t.Error("the swing axes did not hold")
	}
}

// Both swings limited: the elliptic cone, like the ball joint
func TestConfigurableCone(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	pivot := mgl64.Vec3{0, 2, 0}
	anchor := anchorBody(w, pivot)
	arm := addBody(w, pivot.Add(mgl64.Vec3{0, -0.5, 0}), mgl64.QuatIdent(), &actor.Capsule{HalfHeight: 0.4, Radius: 0.05}, actor.BodyTypeDynamic, 0.5, 0)
	joint := NewConfigurableJoint(anchor, arm, pivot, mgl64.Vec3{0, -1, 0})
	joint.SwingYMotion, joint.SwingZMotion, joint.SwingLimitY, joint.SwingLimitZ = MotionLimited, MotionLimited, 20*math.Pi/180, 40*math.Pi/180
	joint.TwistMotion, joint.TwistMin, joint.TwistMax = MotionLimited, -0.2, 0.2
	w.AddJoint(joint)

	// 25 N at the tip is about 5 g: harder pushes bend the soft limits further
	worstSwing, worstTwist := 0.0, 0.0
	for _, push := range []mgl64.Vec3{{25, 0, 0}, {-25, 0, 0}, {0, 0, 25}, {18, 0, 18}} {
		simulate(w, 1, func() {
			arm.AddForceAtPoint(push, arm.Transform.ToWorld(mgl64.Vec3{0, -0.45, 0}))
			arm.AddTorque(arm.Transform.Rotation.Rotate(mgl64.Vec3{0, 3, 0}))
			frameA := anchor.Transform.Rotation.Mul(joint.LocalFrameA.Rotation)
			frameB := arm.Transform.Rotation.Mul(joint.LocalFrameB.Rotation)
			p := frameA.Conjugate().Rotate(frameB.Rotate(mgl64.Vec3{1, 0, 0}))
			if r := math.Hypot(p.Y(), p.Z()); r > 1e-9 {
				worstSwing = math.Max(worstSwing, math.Atan2(r, p.X())-1/math.Hypot(p.Y()/r/joint.SwingLimitZ, p.Z()/r/joint.SwingLimitY))
			}
			twist := twistAngle(frameA, frameB)
			worstTwist = math.Max(worstTwist, math.Max(joint.TwistMin-twist, twist-joint.TwistMax))
		})
	}
	t.Logf("cone overshoot %.3f°, twist overshoot %.3f°", degrees(worstSwing), degrees(worstTwist))
	if degrees(worstSwing) > 0.5 || degrees(worstTwist) > 0.5 {
		t.Error("the cone did not hold")
	}
}

// Everything locked: a fixed joint
func TestConfigurableLocked(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	a := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	b := addBody(w, mgl64.Vec3{0.5, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	w.AddJoint(NewConfigurableJoint(a, b, mgl64.Vec3{0.25, 0, 0}, mgl64.Vec3{1, 0, 0}))
	a.AddImpulseAtPoint(mgl64.Vec3{0, 20, 0}, mgl64.Vec3{-0.25, 0, 0.2})
	worstPosition, worstAngle := 0.0, 0.0
	simulate(w, 3, func() {
		worstPosition = math.Max(worstPosition, a.Transform.ToLocal(b.Transform.Position).Sub(mgl64.Vec3{0.5, 0, 0}).Len())
		rotation := a.Transform.Rotation.Conjugate().Mul(b.Transform.Rotation)
		worstAngle = math.Max(worstAngle, 2*math.Acos(math.Min(1, math.Abs(rotation.W))))
	})
	t.Logf("worst %.3f mm, %.3f°", worstPosition*1000, degrees(worstAngle))
	if worstPosition > 0.001 || degrees(worstAngle) > 0.5 {
		t.Errorf("moved by %.2f mm and %.2f°", worstPosition*1000, degrees(worstAngle))
	}
}

// Drives: the linear drive brings the body to its target position, the angular drive to its target rotation
func TestConfigurableDrives(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	base := anchorBody(w, mgl64.Vec3{})
	body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	joint := NewConfigurableJoint(base, body, mgl64.Vec3{}, mgl64.Vec3{1, 0, 0})
	joint.LinearMotion = [3]Motion{MotionFree, MotionFree, MotionFree}
	joint.TwistMotion, joint.SwingYMotion, joint.SwingZMotion = MotionFree, MotionFree, MotionFree
	joint.EnableLinearDrive, joint.DriveTargetPosition, joint.LinearDriveHertz, joint.LinearDriveDampingRatio = true, mgl64.Vec3{0.3, -0.2, 0.1}, 2, 1
	target := mgl64.QuatRotate(1, mgl64.Vec3{1, 1, 0}.Normalize())
	joint.EnableAngularDrive, joint.DriveTargetRotation, joint.AngularDriveHertz, joint.AngularDriveDampingRatio = true, target, 2, 1
	w.AddJoint(joint)
	simulate(w, 3, nil)
	positionError := body.Transform.Position.Sub(mgl64.Vec3{0.3, -0.2, 0.1}).Len()
	angleError := 2 * math.Acos(math.Min(1, math.Abs(body.Transform.Rotation.Dot(target))))
	t.Logf("position error %.3f mm, rotation error %.3f°", positionError*1000, degrees(angleError))
	if positionError > 0.001 || degrees(angleError) > 0.5 {
		t.Error("the drives did not reach their targets")
	}
}

func TestConfigurableDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	w := newScene(1)
	pivot := mgl64.Vec3{0, 2, 0}
	previous := anchorBody(w, pivot)
	for i := 0; i < 5; i++ {
		link := addBody(w, pivot.Sub(mgl64.Vec3{0, float64(i)*0.5 + 0.25, 0}), mgl64.QuatIdent(), &actor.Capsule{HalfHeight: 0.2, Radius: 0.05}, actor.BodyTypeDynamic, 0.5, 0)
		joint := NewConfigurableJoint(previous, link, pivot.Sub(mgl64.Vec3{0, float64(i) * 0.5, 0}), mgl64.Vec3{0, -1, 0})
		joint.SwingYMotion, joint.SwingZMotion, joint.SwingLimitY, joint.SwingLimitZ = MotionLimited, MotionLimited, 0.5, 0.5
		joint.TwistMotion, joint.TwistMin, joint.TwistMax = MotionLimited, -0.2, 0.2
		w.AddJoint(joint)
		previous = link
	}
	simulate(w, 0.2, nil)
	if allocs := testing.AllocsPerRun(10, func() { w.Step(sceneDt) }); allocs > 0 {
		t.Errorf("%.1f allocations per step", allocs)
	}
}
