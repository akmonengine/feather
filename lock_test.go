package feather

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Axis locks: a dynamic body can't move along, or turn around, the locked world axes. It has no inverse mass (or
// inverse inertia) along them, in the solver and in the integration.
const (
	// lockSteps: the criterion of #865, 600 steps (10 s)
	lockSteps = 600

	lockBallRadius = 0.2
)

// lockSeconds: the duration of lockSteps
const lockSeconds = lockSteps * sceneDt

// throwBall at a target: a ball of 16.7 kg at this velocity
func throwBall(w *World, position, velocity mgl64.Vec3) *actor.RigidBody {
	ball := addBody(w, position, mgl64.QuatIdent(), &actor.Sphere{Radius: lockBallRadius}, actor.BodyTypeDynamic, 0.5, 0)
	ball.Velocity = velocity
	return ball
}

// hitBoxScene: a box falls from 50 cm on the ground, a ball hits its side along X (and rubs it along Z), then another
// ball hits it along Z. Returns the positions and the velocities of the box at each step
func hitBoxScene(linear actor.Axes) (*actor.RigidBody, []mgl64.Vec3, []mgl64.Vec3) {
	w := newScene(1)
	addGround(w, 0.5)
	box := addBody(w, mgl64.Vec3{0.3, cubeHalf + 0.5, 0.7}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	box.LinearLock = linear
	var positions, velocities []mgl64.Vec3
	step := 0
	simulate(w, lockSeconds, func() {
		step++
		switch step {
		case 60:
			throwBall(w, mgl64.Vec3{-1.2, cubeHalf, -0.2}, mgl64.Vec3{10, 0, 8})
		case 200:
			throwBall(w, mgl64.Vec3{0.3, cubeHalf, -0.8}, mgl64.Vec3{0, 0, 10})
		}
		positions = append(positions, box.Transform.Position)
		velocities = append(velocities, box.Velocity)
	})
	return box, positions, velocities
}

// ========== ACCEPTANCE ==========
// A box with its translation locked along X, hit from the side: its X is the same bit for bit during 600 steps, it moves
// freely along Y (it fell) and Z (the second ball pushed it). The same box without lock is pushed along X
func TestLinearLockKeepsTheAxisBitForBit(t *testing.T) {
	box, positions, velocities := hitBoxScene(actor.AxisX)
	if len(positions) != lockSteps {
		t.Fatalf("%d steps, want %d", len(positions), lockSteps)
	}
	const startX, startY, startZ = 0.3, cubeHalf + 0.5, 0.7
	for i := range positions {
		if positions[i].X() != startX || velocities[i].X() != 0 {
			t.Fatalf("step %d: x = %v (velocity %v), want %v bit for bit", i+1, positions[i].X(), velocities[i].X(), startX)
		}
	}
	if fall := startY - box.Transform.Position.Y(); math.Abs(fall-0.5) > 0.01 {
		t.Errorf("the locked box fell by %.3f m along Y, want 0.5", fall)
	}
	if slide := box.Transform.Position.Z() - startZ; slide < 0.2 {
		t.Errorf("the locked box moved by %.4f m along Z, want it pushed by the ball", slide)
	}

	free, _, _ := hitBoxScene(actor.NoAxes)
	if push := free.Transform.Position.X() - startX; push < 0.2 {
		t.Errorf("the free box moved by %.3f m along X, want it pushed by the ball", push)
	}
}

// edgeBoxScene: a box resting on an edge, leaning by 15° from its balance
func edgeBoxScene(angular actor.Axes) (*World, *actor.RigidBody) {
	w := newScene(1)
	addGround(w, 0.5)
	lean := 30 * math.Pi / 180
	height := cubeHalf * (math.Cos(lean) + math.Sin(lean))
	box := addBody(w, mgl64.Vec3{0, height, 0}, mgl64.QuatRotate(lean, mgl64.Vec3{0, 0, 1}), cube(), actor.BodyTypeDynamic, 0.5, 0)
	box.AngularLock = angular
	return w, box
}

// A box with all its rotations locked, resting on an edge, doesn't tip over: its rotation is the same bit for bit, and it
// stays on its edge. The same box without lock falls flat
func TestAngularLockHoldsTheBoxOnItsEdge(t *testing.T) {
	w, box := edgeBoxScene(actor.AllAxes)
	start := box.Transform
	simulate(w, lockSeconds, func() {
		if box.Transform.Rotation != start.Rotation || box.AngularVelocity != (mgl64.Vec3{}) {
			t.Fatalf("the locked box turned: rotation %v (angular velocity %v), want %v", box.Transform.Rotation, box.AngularVelocity, start.Rotation)
		}
	})
	if sink := math.Abs(box.Transform.Position.Y() - start.Position.Y()); sink > 0.005 {
		t.Errorf("the locked box moved by %.2f mm along Y, want it resting on its edge", sink*1000)
	}
	if !box.IsSleeping {
		t.Error("the locked box resting on its edge doesn't sleep")
	}

	w, free := edgeBoxScene(actor.NoAxes)
	simulate(w, lockSeconds, nil)
	if angle := turnAngle(free.Transform.Rotation.Mul(start.Rotation.Inverse())); angle < 0.4 {
		t.Errorf("the free box turned by %.3f rad, want it fallen flat", angle)
	}
}

// capsuleScene: a character, an upright capsule of 1.8 m, hit by 3 balls at its head, from 3 sides
func capsuleScene(angular actor.Axes) (*World, *actor.RigidBody, func()) {
	const radius, halfHeight = 0.3, 0.6
	w := newScene(1)
	addGround(w, 0.5)
	capsule := addBody(w, mgl64.Vec3{0, halfHeight + radius, 0}, mgl64.QuatIdent(), &actor.Capsule{HalfHeight: halfHeight, Radius: radius}, actor.BodyTypeDynamic, 0.5, 0)
	capsule.AngularLock = angular
	step := 0
	head := 2*halfHeight + radius
	return w, capsule, func() {
		step++
		center := capsule.Transform.Position
		switch step {
		case 30:
			throwBall(w, mgl64.Vec3{center.X() - 1.5, head, center.Z()}, mgl64.Vec3{12, 0, 0})
		case 150:
			throwBall(w, mgl64.Vec3{center.X(), head, center.Z() + 1.5}, mgl64.Vec3{0, 0, -12})
		case 270:
			throwBall(w, mgl64.Vec3{center.X() + 1.5, head, center.Z() + 0.15}, mgl64.Vec3{-12, 0, 0})
		}
	}
}

// A capsule with its rotations locked around X and Z stays upright under the hits: its axis is the vertical bit for
// bit, the hits push it, and it still turns around Y. The same capsule without lock falls over
func TestAngularLockKeepsTheCapsuleUpright(t *testing.T) {
	up := mgl64.Vec3{0, 1, 0}
	w, capsule, hit := capsuleScene(actor.AxisX | actor.AxisZ)
	start := capsule.Transform.Position
	simulate(w, lockSeconds, func() {
		hit()
		rotation, spin := capsule.Transform.Rotation, capsule.AngularVelocity
		if rotation.V[0] != 0 || rotation.V[2] != 0 || spin[0] != 0 || spin[2] != 0 || rotation.Rotate(up) != up {
			t.Fatalf("the locked capsule leans: rotation %v, angular velocity %v", rotation, spin)
		}
	})
	if push := capsule.Transform.Position.Sub(start).Len(); push < 0.05 {
		t.Errorf("the locked capsule moved by %.3f m, want it pushed by the balls", push)
	}
	if lift := math.Abs(capsule.Transform.Position.Y() - start.Y()); lift > 0.005 {
		t.Errorf("the locked capsule is %.2f mm off the ground", lift*1000)
	}
	before := capsule.AngularVelocity.Y()
	capsule.AddAngularImpulse(mgl64.Vec3{3, 3, 3})
	if spin := capsule.AngularVelocity; spin[0] != 0 || spin[1] <= before || spin[2] != 0 {
		t.Errorf("an angular impulse gives the locked capsule the angular velocity %v, want around Y only (it was %v)", spin, before)
	}

	w, free, hit := capsuleScene(actor.NoAxes)
	simulate(w, lockSeconds, hit)
	if lean := free.Transform.Rotation.Rotate(up).Y(); lean > 0.5 {
		t.Errorf("the free capsule still stands (its axis is at %.2f of the vertical), want it fallen", lean)
	}
}

// ========== INTEGRATION ==========
// A body locked along Y doesn't fall, a body locked around all its axes doesn't turn: the gravity, the forces, the
// torques and the impulses don't move a locked axis
func TestLocksResistGravityForcesAndImpulses(t *testing.T) {
	w := newScene(1)
	body := addBody(w, mgl64.Vec3{1, 2, 3}, mgl64.QuatRotate(0.4, mgl64.Vec3{1, 2, 3}.Normalize()), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.3, 0.2}}, actor.BodyTypeDynamic, 0.5, 0)
	body.LinearLock, body.AngularLock = actor.AxisY, actor.AxisX|actor.AxisZ
	start := body.Transform
	simulate(w, 2, func() {
		body.AddForce(mgl64.Vec3{20, 500, -10})
		body.AddTorque(mgl64.Vec3{5, 1, -5})
		body.AddImpulse(mgl64.Vec3{0.5, 30, -0.5})
		body.AddAngularImpulse(mgl64.Vec3{1, 0.01, 1})
		if body.Transform.Position.Y() != start.Position.Y() || body.Velocity.Y() != 0 {
			t.Fatalf("the body locked along Y moved: y = %v, velocity %v", body.Transform.Position.Y(), body.Velocity)
		}
		if body.AngularVelocity[0] != 0 || body.AngularVelocity[2] != 0 {
			t.Fatalf("the body locked around X and Z turns around them: %v", body.AngularVelocity)
		}
	})
	if body.Transform.Position.X() <= start.Position.X() || body.Transform.Position.Z() >= start.Position.Z() {
		t.Errorf("the body didn't follow the forces along its free axes: %v from %v", body.Transform.Position, start.Position)
	}
	if body.AngularVelocity.Y() <= 0 {
		t.Errorf("the body doesn't turn around its free axis: %v", body.AngularVelocity)
	}
}

// A velocity written along a locked axis is cleared by the step: the body doesn't drift
func TestLockClearsAWrittenVelocity(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	body.LinearLock, body.AngularLock = actor.AxisX, actor.AxisZ
	body.Velocity, body.AngularVelocity = mgl64.Vec3{3, 2, 1}, mgl64.Vec3{1, 2, 3}
	w.Step(sceneDt)
	if body.Velocity != (mgl64.Vec3{0, 2, 1}) || body.AngularVelocity != (mgl64.Vec3{1, 2, 0}) {
		t.Errorf("velocity %v & angular velocity %v, want the locked axes cleared", body.Velocity, body.AngularVelocity)
	}
	if body.Transform.Position.X() != 0 {
		t.Errorf("the body moved along its locked axis: %v", body.Transform.Position)
	}
}

// The mass of a contact along its normal is the one of the free axes: a ball hitting a locked ball at an angle doesn't
// sink into it, and the momentum is kept along the free axes (the lock holds the other one)
func TestLockedContactKeepsTheMomentumOfTheFreeAxes(t *testing.T) {
	const radius = 0.25
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	locked := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 0)
	locked.LinearLock = actor.AxisX
	direction := mgl64.Vec3{1, 0, 1}.Normalize()
	ball := addBody(w, direction.Mul(-1.5), mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 0)
	ball.Velocity = direction.Mul(4)
	mass := ball.Material.GetMass()
	momentum := ball.Velocity.Z() * mass
	worst := 0.0
	simulate(w, 1, func() {
		worst = math.Max(worst, 2*radius-ball.Transform.Position.Sub(locked.Transform.Position).Len())
		if locked.Transform.Position.X() != 0 {
			t.Fatalf("the locked ball moved along X: %v", locked.Transform.Position)
		}
	})
	if worst > 0.01 {
		t.Errorf("the ball sank by %.1f mm into the locked ball", worst*1000)
	}
	if locked.Velocity.Z() < 0.5 {
		t.Errorf("the locked ball has the velocity %v, want it pushed along Z", locked.Velocity)
	}
	if after := (ball.Velocity.Z() + locked.Velocity.Z()) * mass; math.Abs(after-momentum) > 1e-9*momentum {
		t.Errorf("the momentum along Z is %.12f kg·m/s, want %.12f", after, momentum)
	}
}

// ========== LOCKS IN WORLD SPACE ==========
// The inverse inertia is locked in world space: when the body turns during the step, its turned inertia is locked
// again (the locked inertia is not turned). A leaning box locked around X only, spinning around Y
func TestLockedInertiaFollowsTheBody(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	body := addBody(w, mgl64.Vec3{}, mgl64.QuatRotate(0.5, mgl64.Vec3{0, 0, 1}), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.5, 0.2}}, actor.BodyTypeDynamic, 0, 0)
	body.AngularLock = actor.AxisX
	body.AngularVelocity = mgl64.Vec3{0, 30, 0}

	s := &w.solver
	s.prepare(w.Bodies, nil, sceneDt, sceneSubsteps, DefaultContactHertz, w.workerPool())
	state := &s.states[0]
	for range sceneSubsteps {
		s.integratePositions(sceneDt)
		turned := *body
		turned.Transform.Rotation = state.deltaRotation.Mul(s.starts[0].rotation).Normalize()
		want := turned.GetInverseInertiaWorld()
		for k := range want {
			if math.Abs(state.inverseInertia[k]-want[k]) > 1e-9*want[4] {
				t.Fatalf("inverse inertia %v, want the one of the turned body %v", state.inverseInertia, want)
			}
		}
		for _, k := range []int{0, 1, 2, 3, 6} {
			if state.inverseInertia[k] != 0 {
				t.Fatalf("inverse inertia %v, want null rows & columns around X", state.inverseInertia)
			}
		}
	}
	if angle := rotationAngle(state.deltaRotation); angle < 0.4 {
		t.Fatalf("the body turned by %.3f rad during the step, the test needs it to turn", angle)
	}
}

// A leaning body which can only turn around Y keeps its spin: the gyroscopic torque is held by the locks. Locked around
// X only, it still tumbles, and never gains speed
func TestLockedSpinIsKept(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	body := addBody(w, mgl64.Vec3{}, mgl64.QuatRotate(0.5, mgl64.Vec3{0, 0, 1}), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.5, 0.2}}, actor.BodyTypeDynamic, 0, 0)
	body.AngularLock = actor.AxisX | actor.AxisZ
	body.AngularVelocity = mgl64.Vec3{0, 8, 0}
	simulate(w, lockSeconds, func() {
		if spin := body.AngularVelocity; spin[0] != 0 || spin[2] != 0 || math.Abs(spin[1]-8) > 1e-9 {
			t.Fatalf("the spin is %v, want [0 8 0]", spin)
		}
	})

	body.SetLocks(actor.NoAxes, actor.AxisX)
	body.AngularVelocity = mgl64.Vec3{0, 8, 3}
	tumbles := false
	simulate(w, lockSeconds, func() {
		spin := body.AngularVelocity
		// the speed of a tumbling body changes (the free body: from 6.75 to 8.72 rad/s; this one up to 9.03)
		if spin[0] != 0 || spin.Len() > 1.1*math.Hypot(8, 3) {
			t.Fatalf("the spin is %v, want it null around X and bounded", spin)
		}
		tumbles = tumbles || math.Abs(spin[2]-3) > 0.5
	})
	if !tumbles {
		t.Error("the body locked around X keeps its angular velocity, want the gyroscopic torque on its free axes")
	}
}

// ========== JOINTS ==========
// A body locked along X, pulled by a joint: its X is kept bit for bit, and the joint holds
func TestLockResistsAJoint(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	locked := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	locked.LinearLock = actor.AxisX
	ball := addBody(w, mgl64.Vec3{1, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.2}, actor.BodyTypeDynamic, 0.5, 0)
	ball.Velocity = mgl64.Vec3{6, 0, 3}
	joint := NewBallJoint(locked, ball, mgl64.Vec3{0.5, 0, 0}, mgl64.Vec3{1, 0, 0})
	w.AddJoint(joint)
	worst := 0.0
	simulate(w, 5, func() {
		if locked.Transform.Position.X() != 0 || locked.Velocity.X() != 0 {
			t.Fatalf("the locked body moved along X: %v (velocity %v)", locked.Transform.Position, locked.Velocity)
		}
		worst = math.Max(worst, jointGap(&joint.JointBase))
	})
	if worst > 0.005 {
		t.Errorf("the joint opened by %.2f mm", worst*1000)
	}
	if math.Abs(locked.Transform.Position.Z()) < 0.5 {
		t.Errorf("the locked body is at %v, want it pulled along Z", locked.Transform.Position)
	}
}

// The bodies of a joint may not answer along a direction: the other rows are still solved. A body which only turns
// around Y, held to the world by a fixed joint, doesn't turn; by a hinge around X, it doesn't turn either (it can't
// turn around X); without joint, it does
func TestJointHoldsTheFreeAxesOfALockedBody(t *testing.T) {
	cases := []struct {
		name  string
		joint func(anchor, body *actor.RigidBody) Joint
		turns bool
	}{
		{"no joint", nil, true},
		{"fixed", func(anchor, body *actor.RigidBody) Joint { return NewFixedJoint(anchor, body, mgl64.Vec3{0, 1, 0}) }, false},
		{"fixed, the locked body first", func(anchor, body *actor.RigidBody) Joint { return NewFixedJoint(body, anchor, mgl64.Vec3{0, 1, 0}) }, false},
		{"hinge, the locked body first", func(anchor, body *actor.RigidBody) Joint {
			return NewHingeJoint(body, anchor, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{1, 0, 0})
		}, false},
		{"hinge", func(anchor, body *actor.RigidBody) Joint {
			return NewHingeJoint(anchor, body, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{1, 0, 0})
		}, false},
		{"ball drive", func(anchor, body *actor.RigidBody) Joint {
			joint := NewBallJoint(anchor, body, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{1, 0, 0})
			joint.EnableDrive, joint.DriveHertz, joint.DriveDampingRatio = true, 20, 1
			return joint
		}, false},
		{"configurable", func(anchor, body *actor.RigidBody) Joint {
			return NewConfigurableJoint(anchor, body, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{1, 0, 0})
		}, false},
	}
	for _, c := range cases {
		w := newScene(1)
		anchor := anchorBody(w, mgl64.Vec3{0, 1, 0})
		body := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.1, 0.2}}, actor.BodyTypeDynamic, 0.5, 0)
		body.LinearLock, body.AngularLock = actor.AllAxes, actor.AxisX|actor.AxisZ
		if c.joint != nil {
			w.AddJoint(c.joint(anchor, body))
		}
		simulate(w, 2, func() {
			body.AddTorque(mgl64.Vec3{0, 2, 0})
		})
		angle := turnAngle(body.Transform.Rotation)
		if !finite(body.Transform.Position) || !finite(body.AngularVelocity) || math.IsNaN(angle) {
			t.Fatalf("%s: the body exploded: %v, %v", c.name, body.Transform, body.AngularVelocity)
		}
		if turns := angle > 0.05; turns != c.turns {
			t.Errorf("%s: the body turned by %.4f rad, turns %v, want %v", c.name, angle, turns, c.turns)
		}
		if body.Transform.Position != (mgl64.Vec3{0, 1, 0}) {
			t.Errorf("%s: the body moved to %v", c.name, body.Transform.Position)
		}
	}
}

// A body which can't move, pinned to the world off its center: it can't turn either (its anchor would move). The joint
// has no row to solve, nothing explodes
func TestPinnedLockedBodyStays(t *testing.T) {
	w := newScene(1)
	anchor := anchorBody(w, mgl64.Vec3{0.5, 1, 0})
	body := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	body.LinearLock = actor.AllAxes
	joint := NewBallJoint(anchor, body, mgl64.Vec3{0.5, 1, 0}, mgl64.Vec3{1, 0, 0})
	w.AddJoint(joint)
	simulate(w, 2, func() {
		body.AddTorque(mgl64.Vec3{0, 0, 5})
	})
	if gap := jointGap(&joint.JointBase); !(gap < 0.002) {
		t.Errorf("the joint opened by %.3f mm", gap*1000)
	}
	if body.Transform.Position != (mgl64.Vec3{0, 1, 0}) || !finite(body.AngularVelocity) || !finite(joint.linearImpulse) {
		t.Errorf("the body is at %v, angular velocity %v, impulse %v", body.Transform.Position, body.AngularVelocity, joint.linearImpulse)
	}
	if angle := turnAngle(body.Transform.Rotation); !(angle < 0.01) {
		t.Errorf("the pinned body turned by %.4f rad", angle)
	}
}

// planarChain: a chain hanging from the world, thrown sideways. Locked in its plane (no translation along Z, no
// rotation around X and Y), it is a chain in 2D
func planarChain(lock bool) (*World, []*actor.RigidBody, []*BallJoint) {
	w := newScene(1)
	_, capsules, joints := hangingChain(w, 8)
	for i, capsule := range capsules {
		if lock {
			capsule.LinearLock, capsule.AngularLock = actor.AxisZ, actor.AxisX|actor.AxisY
		}
		capsule.Velocity = mgl64.Vec3{float64(i+1) * 0.5, 0, 0}
	}
	return w, capsules, joints
}

// A chain of locked bodies is an articulation whose joints don't answer along Z: it holds as well as the free chain,
// stays in its plane bit for bit, and swings like it
func TestLockedChainHoldsInItsPlane(t *testing.T) {
	gaps := [2]float64{}
	var ends [2]mgl64.Vec3
	for k, lock := range []bool{false, true} {
		w, capsules, joints := planarChain(lock)
		// a free body, after the locked ones
		addBody(w, mgl64.Vec3{5, 5, 5}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
		if lock {
			// a push out of the plane
			capsules[3].Velocity = capsules[3].Velocity.Add(mgl64.Vec3{0, 0, 2})
		}
		simulate(w, 3, func() {
			for _, joint := range joints {
				gaps[k] = math.Max(gaps[k], jointGap(&joint.JointBase))
			}
			for i, capsule := range capsules {
				if !lock {
					continue
				}
				rotation := capsule.Transform.Rotation
				if capsule.Transform.Position.Z() != 0 || rotation.V[0] != 0 || rotation.V[1] != 0 {
					t.Fatalf("capsule %d left its plane: %v, %v", i, capsule.Transform.Position, rotation)
				}
			}
		})
		ends[k] = capsules[len(capsules)-1].Transform.Position
	}
	t.Logf("worst gap: free %.3f mm, locked %.3f mm", gaps[0]*1000, gaps[1]*1000)
	if gaps[1] > math.Max(2*gaps[0], 0.001) {
		t.Errorf("the locked chain opened by %.3f mm, the free one by %.3f mm", gaps[1]*1000, gaps[0]*1000)
	}
	if distance := ends[1].Sub(ends[0]).Len(); distance > 0.05 {
		t.Errorf("the end of the locked chain is %.1f mm from the end of the free chain", distance*1000)
	}
}

// ========== SLEEP ==========
// A locked body sleeps as the others; SetLocks wakes it up: freed in the air, it falls
func TestSetLocksWakesTheBody(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.5)
	body := addBody(w, mgl64.Vec3{0, 2, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	body.SetLocks(actor.AxisY, actor.NoAxes)
	simulate(w, 1, nil)
	if !body.IsSleeping || body.Transform.Position.Y() != 2 {
		t.Fatalf("the body locked in the air: sleeping %v, at %v", body.IsSleeping, body.Transform.Position)
	}
	body.SetLocks(actor.AxisY, actor.NoAxes)
	if !body.IsSleeping {
		t.Error("SetLocks without change woke the body up")
	}
	body.SetLocks(actor.NoAxes, actor.NoAxes)
	if body.IsSleeping {
		t.Error("SetLocks didn't wake the body up")
	}
	simulate(w, 2, nil)
	if y := body.Transform.Position.Y(); math.Abs(y-cubeHalf) > 0.005 {
		t.Errorf("the freed body is at y=%.3f, want it on the ground", y)
	}
}

// A body with all its axes locked stays dynamic (as a Rigidbody with FreezeAll in Unity, a PxRigidDynamic with all its
// lock flags): it never moves, carries the bodies resting on it, sleeps and wakes up. Jolt forbids it (EAllowedDOFs::None)
func TestFullyLockedBodyStaysDynamic(t *testing.T) {
	w := newScene(1)
	shelf := addBody(w, mgl64.Vec3{0, 2, 0}, mgl64.QuatRotate(0.3, mgl64.Vec3{0, 1, 0}), &actor.Box{HalfExtents: mgl64.Vec3{1, 0.1, 1}}, actor.BodyTypeDynamic, 0.5, 0)
	shelf.SetLocks(actor.AllAxes, actor.AllAxes)
	twin := addBody(w, mgl64.Vec3{0, 2.05, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.1, 0.5}}, actor.BodyTypeDynamic, 0.5, 0)
	twin.SetLocks(actor.AllAxes, actor.AllAxes)
	box := addBody(w, mgl64.Vec3{0.4, 2.9, 0.3}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	start, twinStart := shelf.Transform, twin.Transform
	check := func() {
		if shelf.Transform != start || twin.Transform != twinStart || shelf.Velocity != (mgl64.Vec3{}) || shelf.AngularVelocity != (mgl64.Vec3{}) {
			t.Fatalf("a fully locked body moved: %v (%v %v), %v", shelf.Transform, shelf.Velocity, shelf.AngularVelocity, twin.Transform)
		}
	}
	simulate(w, 3, check)
	if y := box.Transform.Position.Y(); math.Abs(y-(2.15+cubeHalf)) > 0.005 || !finite(box.Velocity) {
		t.Errorf("the box is at y=%.4f (velocity %v), want it resting on the locked bodies at %.2f", y, box.Velocity, 2.15+cubeHalf)
	}
	if shelf.BodyType != actor.BodyTypeDynamic || !shelf.IsSleeping || !box.IsSleeping {
		t.Errorf("the locked shelf: type %v, sleeping %v (box %v), want dynamic & asleep", shelf.BodyType, shelf.IsSleeping, box.IsSleeping)
	}
	throwBall(w, mgl64.Vec3{-2, 2.4, 0.3}, mgl64.Vec3{15, 0, 0})
	awake := false
	simulate(w, 2, func() {
		check()
		awake = awake || !shelf.IsSleeping
	})
	if !awake {
		t.Error("the locked shelf didn't wake up when the ball hit the box it carries")
	}
}

// ========== CONTINUOUS COLLISION ==========
// A fast locked body stopped at its first impact keeps its locked axes bit for bit: its X, and its rotation
func TestContinuousKeepsTheLockedAxes(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.5)
	rotation := mgl64.QuatRotate(0.7, mgl64.Vec3{1, 1, 0}.Normalize())
	box := addBody(w, mgl64.Vec3{0.3, 30, 0.4}, rotation, cube(), actor.BodyTypeDynamic, 0.5, 0)
	box.LinearLock, box.AngularLock = actor.AxisX, actor.AllAxes
	start := box.Transform
	box.Velocity = mgl64.Vec3{0, -300, 40}

	// motions of a step through the ground, as if the solver had accelerated the box
	w.Step(sceneDt)
	scratch := ccdPool.Get().(*ccdScratch)
	scratch.core.Radius = coreFraction * cubeHalf
	for i := 0; i < 20; i++ {
		turned := mgl64.QuatRotate(0.3*float64(i), mgl64.Vec3{1, float64(i), 2}.Normalize()).Normalize()
		height := 3 + 0.1*float64(i)
		motion := sweep{start: actor.Transform{Position: mgl64.Vec3{0.3, height, 0.4}, Rotation: turned}, end: actor.Transform{Position: mgl64.Vec3{0.3, -3, 1}, Rotation: turned}}
		box.Transform = motion.end
		w.stopAtImpact(box, &motion, cube().HalfExtents.Len(), scratch)
		if y := box.Transform.Position.Y(); y < 0 || y > height {
			t.Fatalf("the box is at y=%.3f, want it stopped above the ground", y)
		}
		if box.Transform.Position.X() != start.Position.X() || box.Transform.Rotation != turned {
			t.Errorf("the stopped box is at %v, want x=%v and the rotation %v bit for bit", box.Transform, start.Position.X(), turned)
		}
	}
	ccdPool.Put(scratch)
	box.Transform = start

	simulate(w, 2, func() {
		if box.Transform.Position.X() != start.Position.X() || box.Transform.Rotation != start.Rotation {
			t.Fatalf("the fast box is at %v, want x=%v and the rotation %v bit for bit", box.Transform, start.Position.X(), start.Rotation)
		}
	})
	if y := box.Transform.Position.Y(); y < 0 || y > 1 {
		t.Errorf("the box is at y=%.3f, want it on the ground", y)
	}
}

// ========== TERRAIN ==========
// On a slope of terrain without friction, a box locked along X and Z doesn't slide, and rests on the terrain; the free
// box slides down
func TestLockedBoxStaysOnTheSlope(t *testing.T) {
	angle := 20 * math.Pi / 180
	positions := [2]mgl64.Vec3{}
	start := mgl64.Vec3{3.1, 3.1*math.Tan(angle) + 0.6, 0.37}
	for k, lock := range []actor.Axes{actor.NoAxes, actor.AxisX | actor.AxisZ} {
		w := newScene(1)
		terrain := slopeTerrain(w, angle, 0)
		box := addBody(w, start, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0, 0)
		box.LinearLock = lock
		worst := 0.0
		simulate(w, 3, func() {
			if lock != actor.NoAxes && (box.Transform.Position.X() != start.X() || box.Transform.Position.Z() != start.Z()) {
				t.Fatalf("the locked box slid to %v", box.Transform.Position)
			}
			worst = math.Max(worst, terrainDepth(terrain.Shape.(*actor.Heightfield), terrain.Transform, box))
		})
		if worst > 0.01 {
			t.Errorf("lock %v: the box sank by %.1f mm into the terrain", lock, worst*1000)
		}
		positions[k] = box.Transform.Position
	}
	if slide := start.X() - positions[0].X(); slide < 1 {
		t.Errorf("the free box slid by %.3f m, want it down the slope", slide)
	}
	if fall := start.Y() - positions[1].Y(); fall < 0.05 || fall > 0.6 {
		t.Errorf("the locked box fell by %.3f m, want it resting on the terrain", fall)
	}
}

// ========== FILTERING ==========
// The locks and the filter are independent: a locked body goes through what its mask refuses, SetFilter changes it
func TestLockedBodyRespectsTheFilter(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	box := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	box.LinearLock = actor.AxisX
	box.Mask = actor.AllLayers &^ layerDebris
	first := throwBall(w, mgl64.Vec3{-1, 0, 0.3}, mgl64.Vec3{5, 0, 0})
	first.Layer = layerDebris
	simulate(w, 1, nil)
	if first.Transform.Position.X() < 3 || box.Transform.Position != (mgl64.Vec3{}) {
		t.Fatalf("the filtered ball is at %v, the box at %v: want the ball through the box", first.Transform.Position, box.Transform.Position)
	}
	w.SetFilter(box, box.Layer, actor.AllLayers)
	second := throwBall(w, mgl64.Vec3{-1, 0, 0.1}, mgl64.Vec3{5, 0, 1})
	second.Layer = layerDebris
	simulate(w, 1, nil)
	if second.Transform.Position.X() > 1 || box.Transform.Position.X() != 0 {
		t.Errorf("the ball is at %v, the box at %v: want the ball stopped by the box, the box still at x=0", second.Transform.Position, box.Transform.Position)
	}
}

// ========== SPINNING RESISTANCE ==========
// A top locked around X and Z slows down as the free top (the spinning resistance is around the normal, a free axis),
// and stays upright; locked around Y, it can't spin
func TestLockedTopSpinsDown(t *testing.T) {
	want := spinDeceleration(spinResistance, spinRadius)
	for _, lock := range []actor.Axes{actor.NoAxes, actor.AxisX | actor.AxisZ} {
		w, ball := spinScene(1, spinRadius, spinResistance, 0, true)
		ball.SetLocks(actor.NoAxes, lock)
		ball.AngularVelocity = mgl64.Vec3{0, spinSpeed, 0}
		simulate(w, 0.5, nil)
		before := ball.AngularVelocity.Y()
		simulate(w, 0.5, nil)
		deceleration := (before - ball.AngularVelocity.Y()) / 0.5
		if math.Abs(deceleration-want) > spinTolerance*want {
			t.Errorf("lock %v: the top slows down at %.3f rad/s², want %.3f", lock, deceleration, want)
		}
		if lock != actor.NoAxes && (ball.AngularVelocity[0] != 0 || ball.AngularVelocity[2] != 0) {
			t.Errorf("the locked top leans: %v", ball.AngularVelocity)
		}
	}

	w, ball := spinScene(1, spinRadius, spinResistance, 0, true)
	ball.SetLocks(actor.NoAxes, actor.AxisY)
	simulate(w, 0.5, func() {
		ball.AddAngularImpulse(mgl64.Vec3{0, 1, 0})
		ball.AddTorque(mgl64.Vec3{0, 10, 0})
		if ball.AngularVelocity.Y() != 0 {
			t.Fatalf("the top locked around Y spins: %v", ball.AngularVelocity)
		}
	})
}

// ========== DETERMINISM & ALLOCATIONS ==========
// lockedPile: a pile where 2 bodies out of 3 have locks, and a locked chain
func lockedPile(count, workers int) *World {
	w := benchScene(count, workers)
	w.parallelFrom = 1
	for i, body := range w.Bodies[1:] {
		switch i % 3 {
		case 0:
			body.LinearLock, body.AngularLock = actor.Axes(i/3%8), actor.Axes(i/24%8)
		case 1:
			body.AngularLock = actor.AxisX | actor.AxisZ
		}
		body.AngularVelocity = mgl64.Vec3{float64(i%5) - 2, float64(i%7) - 3, float64(i%3) - 1}
	}
	_, capsules, _ := hangingChain(w, 10)
	for _, capsule := range capsules {
		capsule.LinearLock, capsule.AngularLock = actor.AxisZ, actor.AxisX|actor.AxisY
	}
	return w
}

func TestLocksAreDeterministic(t *testing.T) {
	run := func(workers int) []mgl64.Vec3 {
		w := lockedPile(200, workers)
		simulate(w, 1, nil)
		var out []mgl64.Vec3
		for _, body := range w.Bodies {
			out = append(out, body.Transform.Position, body.Transform.Rotation.V, body.Velocity, body.AngularVelocity)
		}
		return out
	}
	reference := run(1)
	for _, value := range reference {
		if !finite(value) {
			t.Fatalf("the pile exploded: %v", value)
		}
	}
	for _, workers := range []int{2, 8} {
		got := run(workers)
		for i := range reference {
			if got[i] != reference[i] {
				t.Fatalf("workers=%d: value %d is %v, want %v", workers, i, got[i], reference[i])
			}
		}
	}
}

// The locked axes of each body of the pile don't move during the step, whatever touches it
func TestLockedPileKeepsItsAxes(t *testing.T) {
	w := lockedPile(200, 1)
	type pose struct{ position, rotation mgl64.Vec3 }
	starts := make([]pose, len(w.Bodies))
	for i, body := range w.Bodies {
		starts[i] = pose{body.Transform.Position, body.Transform.Rotation.V}
	}
	simulate(w, 2, func() {
		for i, body := range w.Bodies {
			for k := 0; k < 3; k++ {
				if body.LinearLock.Has(k) && (body.Transform.Position[k] != starts[i].position[k] || body.Velocity[k] != 0) {
					t.Fatalf("body %d moved along its locked axis %d: %v from %v", i, k, body.Transform.Position, starts[i].position)
				}
				if body.AngularLock.Has(k) && body.AngularVelocity[k] != 0 {
					t.Fatalf("body %d turns around its locked axis %d: %v", i, k, body.AngularVelocity)
				}
			}
		}
	})
}

func TestLockedStepDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 8} {
		w := lockedPile(500, workers)
		// once the pile has landed: the buffers of the contacts grow with their count, with or without locks
		simulate(w, 1.5, nil)
		awake := 0
		for _, body := range w.Bodies {
			if !body.IsSleeping && body.LinearLock|body.AngularLock != actor.NoAxes {
				awake++
			}
		}
		if len(w.Contacts()) == 0 || awake < 100 {
			t.Fatalf("%d contacts, %d locked bodies awake: the scene doesn't cover the solver", len(w.Contacts()), awake)
		}
		allocs := testing.AllocsPerRun(10, func() {
			w.Step(sceneDt)
		})
		if allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
	}
}

// ========== MASS MATRIX ==========
// The rows are solved with the mass of the free axes: a rigid row is exact after one pass, the bodies don't get closer
// anymore. With the whole mass on a locked axis (as Jolt computes its effective mass), a part of the velocity is left.
// One step of one sub-step, without friction nor gravity
func exactScene() *World {
	w := newScene(1)
	w.Gravity, w.Substeps = mgl64.Vec3{}, 1
	return w
}

// 1 point: 2 balls touching at an angle, the first one locked along X
func TestLockedNormalRowIsExact(t *testing.T) {
	const radius = 0.25
	for _, lockedFirst := range []bool{true, false} {
		w := exactScene()
		direction := mgl64.Vec3{1, 0, 1}.Normalize()
		positions := [2]mgl64.Vec3{{}, direction.Mul(-2*radius + 0.001)}
		if !lockedFirst {
			positions[0], positions[1] = positions[1], positions[0]
		}
		a := addBody(w, positions[0], mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 0)
		b := addBody(w, positions[1], mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 0)
		locked, ball := a, b
		if !lockedFirst {
			locked, ball = b, a
		}
		locked.LinearLock = actor.AxisX
		ball.Velocity = direction.Mul(2)
		w.Step(sceneDt)
		// the soft pass pushes the overlap out (1 mm): the balls may leave each other, they don't get closer anymore
		// (within the proximal term of the row: 0.1 mm/s)
		if approach := ball.Velocity.Sub(locked.Velocity).Dot(direction); approach > 0.002 || approach < -0.1 {
			t.Errorf("locked first %v: the balls get closer at %.6f m/s after a rigid pass, want them stopped", lockedFirst, approach)
		}
		if locked.Velocity.Z() < 0.1 || locked.Velocity.X() != 0 {
			t.Errorf("locked first %v: the locked ball has the velocity %v, want it pushed along Z only", lockedFirst, locked.Velocity)
		}
	}
}

// 4 points: a box which can't turn nor move along X, lying on a slope of 30° (a static box): the gravity pushes it into
// the slope, a pass stops it
func TestLockedNormalBlockIsExact(t *testing.T) {
	w := exactScene()
	w.Gravity = mgl64.Vec3{0, -sceneGravity, 0}
	angle := 30 * math.Pi / 180
	turned := mgl64.QuatRotate(angle, mgl64.Vec3{0, 0, 1})
	normal := turned.Rotate(mgl64.Vec3{0, 1, 0})
	addBody(w, normal.Mul(-0.5), turned, &actor.Box{HalfExtents: mgl64.Vec3{3, 0.5, 3}}, actor.BodyTypeStatic, 0, 0)
	box := addBody(w, normal.Mul(cubeHalf-0.0005), turned, cube(), actor.BodyTypeDynamic, 0, 0)
	box.LinearLock, box.AngularLock = actor.AxisX, actor.AllAxes
	w.Step(sceneDt)
	if len(w.Contacts()) != 1 || w.Contacts()[0].Count != 4 {
		t.Fatalf("the box on the slope has %d contacts, want 1 of 4 points", len(w.Contacts()))
	}
	// the soft pass pushed the overlap out, the rigid pass keeps what leaves the slope
	if box.Velocity.Y() < -1e-9 || box.Velocity.Y() > ContactSpeed || box.Velocity.X() != 0 {
		t.Errorf("the box has the velocity %v after a rigid pass, want it stopped by the slope", box.Velocity)
	}
}

// A restitution of 1 gives back the approach velocity: the impulse of the bounce is the one of the mass of the free
// axes
func TestLockedBounceIsExact(t *testing.T) {
	const radius = 0.25
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	direction := mgl64.Vec3{1, 0, 1}.Normalize()
	locked := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 1)
	locked.LinearLock = actor.AxisX
	ball := addBody(w, direction.Mul(-1), mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 1)
	ball.Velocity = direction.Mul(4)
	simulate(w, 0.5, nil)
	if leaving := locked.Velocity.Sub(ball.Velocity).Dot(direction); math.Abs(leaving-4) > 1e-6 {
		t.Errorf("the balls leave each other at %.9f m/s along the normal of their contact, want the approach velocity 4", leaving)
	}
	if energy := ball.Velocity.LenSqr() + locked.Velocity.LenSqr(); math.Abs(energy-16) > 1e-6 {
		t.Errorf("the kinetic energy is %.9f times m/2, want 16: the bounce on a locked body keeps it", energy)
	}
}

// anchorVelocity: the velocity of B relative to A at the anchor of their joint
func anchorVelocity(j *JointBase) mgl64.Vec3 {
	rA := j.BodyA.Transform.Rotation.Rotate(j.LocalFrameA.Position)
	rB := j.BodyB.Transform.Rotation.Rotate(j.LocalFrameB.Position)
	return j.BodyB.Velocity.Add(j.BodyB.AngularVelocity.Cross(rB)).Sub(j.BodyA.Velocity.Add(j.BodyA.AngularVelocity.Cross(rA)))
}

// The point of a joint between a locked body and a free one: its anchors move together after one pass. Alone (a ball
// joint), and in an articulation (a chain of 3 joints, within its proximal term: 5 mm/s are left, as without lock)
func TestLockedJointRowsAreExact(t *testing.T) {
	cases := []struct {
		name      string
		links     int
		tolerance float64
	}{
		{"alone", 1, 1e-9},
		{"articulation", 3, 0.01},
	}
	for _, c := range cases {
		w := exactScene()
		var bodies []*actor.RigidBody
		var joints []*BallJoint
		for i := 0; i <= c.links; i++ {
			body := addBody(w, mgl64.Vec3{float64(i), 0, 0.3 * float64(i)}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0, 0)
			if i%2 == 0 {
				body.LinearLock, body.AngularLock = actor.AxisX, actor.AxisZ
			}
			body.Velocity = mgl64.Vec3{float64(i%2) * 2, 1, float64(i) - 1}
			if i > 0 {
				joint := NewBallJoint(bodies[i-1], body, mgl64.Vec3{float64(i) - 0.5, 0.2, 0.3*float64(i) - 0.1}, mgl64.Vec3{1, 0, 0})
				w.AddJoint(joint)
				joints = append(joints, joint)
			}
			bodies = append(bodies, body)
		}
		w.Step(sceneDt)
		for i, joint := range joints {
			if joint.inArticulation != (c.links > 1) {
				t.Fatalf("%s: joint %d in an articulation: %v", c.name, i, joint.inArticulation)
			}
			if velocity := anchorVelocity(&joint.JointBase).Len(); velocity > c.tolerance {
				t.Errorf("%s: the anchors of the joint %d move apart at %.3g m/s after a rigid pass, want less than %.3g", c.name, i, velocity, c.tolerance)
			}
		}
		for i, body := range bodies {
			if i%2 == 0 && (body.Velocity.X() != 0 || body.AngularVelocity.Z() != 0) {
				t.Errorf("%s: the locked body %d moves along its locked axes: %v %v", c.name, i, body.Velocity, body.AngularVelocity)
			}
		}
	}
}

// A rigid distance joint between a locked body and a free one: its length doesn't change anymore after one pass
func TestLockedDistanceRowIsExact(t *testing.T) {
	w := exactScene()
	locked := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0, 0)
	locked.LinearLock = actor.AxisX
	ball := addBody(w, mgl64.Vec3{1, 0.5, 1}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.2}, actor.BodyTypeDynamic, 0, 0)
	ball.Velocity = mgl64.Vec3{3, 1, 2}
	joint := NewDistanceJoint(locked, ball, mgl64.Vec3{}, mgl64.Vec3{1, 0.5, 1})
	w.AddJoint(joint)
	w.Step(sceneDt)
	axis := ball.Transform.Position.Sub(locked.Transform.Position).Normalize()
	if stretching := ball.Velocity.Sub(locked.Velocity).Dot(axis); math.Abs(stretching) > 1e-6 {
		t.Errorf("the joint still stretches at %.3g m/s after a rigid pass", stretching)
	}
	if locked.Velocity.X() != 0 || locked.Velocity.Len() < 0.1 {
		t.Errorf("the locked body has the velocity %v, want it pulled along its free axes", locked.Velocity)
	}
}

// A hinge around a leaning axis of the world holds a body which can't move: after one pass it only turns around the axis
// of the hinge (both rows of the hinge are coupled by the inertia of the leaning box)
func TestLockedHingeRowsAreExact(t *testing.T) {
	w := exactScene()
	anchor := anchorBody(w, mgl64.Vec3{0, 1, 0})
	body := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatRotate(0.5, mgl64.Vec3{0, 0, 1}), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.5, 0.2}}, actor.BodyTypeDynamic, 0, 0)
	body.LinearLock = actor.AllAxes
	body.AngularVelocity = mgl64.Vec3{0, 3, -1}
	axis := mgl64.Vec3{0, 1, 1}.Normalize()
	w.AddJoint(NewHingeJoint(anchor, body, mgl64.Vec3{0, 1, 0}, axis))
	w.Step(sceneDt)
	spin := body.AngularVelocity
	if off := spin.Sub(axis.Mul(spin.Dot(axis))).Len(); off > 1e-9 || math.Abs(spin.Dot(axis)) < 0.1 {
		t.Errorf("the angular velocity %v is %.3g rad/s off the axis of the hinge", spin, off)
	}
}

// lockedInverse: the inverse of a regular matrix, and a generalized inverse (K G K = K) of a singular one, whose rows
// which don't answer are null
func TestLockedInverse(t *testing.T) {
	regular := mgl64.Mat3{4, 1, 0.5, 1, 3, 0.2, 0.5, 0.2, 2}
	if inverse, ok := lockedInverse(&regular); !ok || inverse != actor.Inv3(&regular) {
		t.Errorf("the inverse of a regular matrix is %v (%v), want %v", inverse, ok, actor.Inv3(&regular))
	}
	if _, ok := lockedInverse(&mgl64.Mat3{}); ok {
		t.Error("the null matrix has an inverse")
	}

	// r rᵀ-like blocks: rank 2 (no answer along an axis), rank 2 (no answer along a direction), rank 1
	direction := mgl64.Vec3{1, 2, -1}.Normalize()
	skewed := skew(direction)
	projector := skewed.Mul3(skewed).Mul(-1)
	// the same within the rounding of a mass matrix: its third row still doesn't answer
	rounded := projector
	rounded[8] += 1e-10
	cases := []struct {
		name string
		k    mgl64.Mat3
		rank int
	}{
		{"locked axis", mgl64.Mat3{2, 0.5, 0, 0.5, 3, 0, 0, 0, 0}, 2},
		{"locked first axis", mgl64.Mat3{0, 0, 0, 0, 3, 0.5, 0, 0.5, 2}, 2},
		{"direction", projector, 2},
		{"direction, rounded", rounded, 2},
		{"one row", mgl64.Mat3{0, 0, 0, 0, 5, 0, 0, 0, 0}, 1},
		// a row answering 10⁴ times less than the stiffest one still answers, 10¹⁰ times less it doesn't
		{"soft row", mgl64.Mat3{2, 0, 0, 0, 2e-4, 0, 0, 0, 2}, 3},
		{"row which doesn't answer", mgl64.Mat3{2, 0, 0, 0, 2e-10, 0, 0, 0, 2}, 2},
	}
	for _, c := range cases {
		g, ok := lockedInverse(&c.k)
		if !ok {
			t.Errorf("%s: no inverse", c.name)
			continue
		}
		kg := c.k.Mul3(g)
		kgk := kg.Mul3(c.k)
		rows := 0
		for i := 0; i < 3; i++ {
			if g[4*i] != 0 {
				rows++
			}
		}
		if rows != c.rank {
			t.Errorf("%s: %d rows solved, want %d: %v", c.name, rows, c.rank, g)
		}
		for i := range kgk {
			if math.Abs(kgk[i]-c.k[i]) > 1e-9 || math.Abs(g[i]-g.Transpose()[i]) > 1e-12 {
				t.Errorf("%s: K G K = %v, want K = %v (G = %v)", c.name, kgk, c.k, g)
				break
			}
		}
	}
}

// lockedInverse2: the inverse of a regular block; of a block of rank 1, K = k u uᵀ, the inverse of least norm u uᵀ / k
func TestLockedInverse2(t *testing.T) {
	cases := []struct {
		k11, k12, k22 float64
		want          [3]float64
		ok            bool
	}{
		{2, 0, 4, [3]float64{0.5, 0, 0.25}, true},
		{2, 1, 3, [3]float64{0.6, -0.2, 0.4}, true},
		{0, 0, 4, [3]float64{0, 0, 0.25}, true},
		{2, 0, 0, [3]float64{0.5, 0, 0}, true},
		// rank 1 off the rows: k = 4 along u = (1, 1) / √2, and along u = (0.6, 0.8)
		{2, 2, 2, [3]float64{0.125, 0.125, 0.125}, true},
		{2, 2, 2 + 1e-10, [3]float64{0.125, 0.125, 0.125}, true},
		{4 * 0.36, 4 * 0.48, 4 * 0.64, [3]float64{0.36 / 4, 0.48 / 4, 0.64 / 4}, true},
		// a row answering 10⁴ times less than the stiffest one still answers, 10¹⁰ times less it doesn't
		{2, 0, 2e-4, [3]float64{0.5, 0, 5000}, true},
		{2e-4, 0, 2, [3]float64{5000, 0, 0.5}, true},
		{2, 0, 2e-10, [3]float64{0.5, 0, 0}, true},
		{0, 0, 0, [3]float64{}, false},
	}
	for _, c := range cases {
		got, ok := lockedInverse2(c.k11, c.k12, c.k22)
		for i := range got {
			if !(math.Abs(got[i]-c.want[i]) <= 1e-9*math.Max(1, math.Abs(c.want[i]))) || ok != c.ok {
				t.Errorf("lockedInverse2(%v %v %v) = %v %v, want %v %v", c.k11, c.k12, c.k22, got, ok, c.want, c.ok)
				break
			}
		}
	}
}

// The friction of a locked body: the mass of both tangents is the one of its free axes, with the coupling of both
// tangents by the locked axis. frictionMass is the inverse of K = Tᵀ (MA⁻¹ + MB⁻¹) T (the bodies don't turn here)
func TestLockedFrictionMass(t *testing.T) {
	const inverseMass = 0.25
	locked := bodyState{body: &actor.RigidBody{}, invMass: inverseMass, invMassAxes: mgl64.Vec3{inverseMass, inverseMass, 0}, linearLock: actor.AxisZ}
	static := bodyState{}
	for _, normal := range []mgl64.Vec3{{0.3, 0.8, 0.52}, {0.7, 0.1, -0.7}, {0, 1, 0}} {
		for _, order := range [][2]*bodyState{{&locked, &static}, {&static, &locked}} {
			c := contactConstraint{normal: normal.Normalize()}
			c.tangents[0], c.tangents[1] = tangentBasis(c.normal)
			c.makeFrictionRows(order[0], order[1], mgl64.Vec3{}, mgl64.Vec3{})
			var k [2][2]float64
			for i := range k {
				for j := range k {
					k[i][j] = inverseMass * (c.tangents[i][0]*c.tangents[j][0] + c.tangents[i][1]*c.tangents[j][1])
				}
			}
			m := c.frictionMass
			if normal.X() == 0 {
				// the first tangent is Z: it doesn't answer, the second one rubs alone
				if m[0] != 0 || m[1] != 0 || math.Abs(m[2]-1/inverseMass) > 1e-12 {
					t.Errorf("normal %v: friction mass %v, want [0 0 %v]", normal, m, 1/inverseMass)
				}
				continue
			}
			product := [4]float64{m[0]*k[0][0] + m[1]*k[1][0], m[0]*k[0][1] + m[1]*k[1][1], m[1]*k[0][0] + m[2]*k[1][0], m[1]*k[0][1] + m[2]*k[1][1]}
			for i, want := range [4]float64{1, 0, 0, 1} {
				if math.Abs(product[i]-want) > 1e-9 {
					t.Errorf("normal %v: friction mass %v times K %v = %v, want the identity", normal, m, k, product)
					break
				}
			}
			if math.Abs(k[0][1]) < 1e-3 {
				t.Fatalf("normal %v: the tangents are not coupled, the test needs them to be", normal)
			}
		}
	}
}

// A box which can't turn, nor move along X, still rubs along Z: it stops at v² / (2 µ g), as the free box
func TestLockedBoxRubsAlongItsFreeAxis(t *testing.T) {
	const friction, speed = 0.5, 3.0
	want := speed * speed / (2 * friction * sceneGravity)
	for _, lock := range [][2]actor.Axes{{actor.NoAxes, actor.NoAxes}, {actor.AxisX, actor.AllAxes}} {
		w := newScene(1)
		addGround(w, friction)
		box := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, friction, 0)
		box.LinearLock, box.AngularLock = lock[0], lock[1]
		simulate(w, 0.2, nil)
		box.Velocity = mgl64.Vec3{0, 0, speed}
		start := box.Transform.Position.Z()
		simulate(w, 3, nil)
		if slide := box.Transform.Position.Z() - start; math.Abs(slide-want) > 0.05*want {
			t.Errorf("locks %v: the box slid by %.3f m, want %.3f", lock, slide, want)
		}
	}
}

// The mass of 2 bodies along a direction: without lock the sum of the inverse masses, the same bits as a body which has
// no lock at all; with a lock, the inverse masses of the free axes
func TestLinearMass(t *testing.T) {
	free := bodyState{invMass: 0.25, invMassAxes: mgl64.Vec3{0.25, 0.25, 0.25}}
	other := bodyState{invMass: 0.1, invMassAxes: mgl64.Vec3{0.1, 0.1, 0.1}}
	locked := bodyState{invMass: 0.25, invMassAxes: mgl64.Vec3{0, 0.25, 0.25}, linearLock: actor.AxisX}
	direction := mgl64.Vec3{0.6, 0, 0.8}
	if mass := linearMass(&free, &other, direction); mass != 0.25+0.1 {
		t.Errorf("free bodies: %v, want the sum of the inverse masses %v", mass, 0.25+0.1)
	}
	want := 0.36*0.1 + 0.64*0.35
	if mass := linearMass(&locked, &other, direction); math.Abs(mass-want) > 1e-15 {
		t.Errorf("locked along X: %v, want %v", mass, want)
	}
	if mass := linearMass(&other, &locked, direction); math.Abs(mass-want) > 1e-15 {
		t.Errorf("locked along X (body B): %v, want %v", mass, want)
	}
	if cross := crossMass(&locked, &other, direction, mgl64.Vec3{0.8, 0, -0.6}); math.Abs(cross-(0.48*0.1-0.48*0.35)) > 1e-15 {
		t.Errorf("cross mass %v, want %v", cross, 0.48*0.1-0.48*0.35)
	}
	matrix := linearMassMatrix(&locked, &other)
	if matrix != (mgl64.Mat3{0.1, 0, 0, 0, 0.35, 0, 0, 0, 0.35}) {
		t.Errorf("linear mass matrix %v", matrix)
	}
}

// ========== EXACT INERTIA ==========
// A leaning body which only turns around Y answers to a torque as a body on an axle: with the moment of inertia around
// this axis, 1 / (n·I·n). The free block of its inverse inertia (Jolt, Rapier, PhysX, Box3D) makes the box 1.7 times
// too fast, and the rod 8 to more than 100 times
func TestLockedBodyTurnsAsAroundAFixedAxis(t *testing.T) {
	const torque, seconds = 1.0, 1.0
	box, rod := mgl64.Vec3{0.1, 0.5, 0.2}, mgl64.Vec3{0.025, 1, 0.025}
	cases := []struct {
		name        string
		halfExtents mgl64.Vec3
		lean        float64
	}{
		{"upright box", box, 0},
		{"box leaning by 0.5 rad", box, 0.5},
		{"box leaning by 45°", box, math.Pi / 4},
		{"rod leaning by 10°", rod, 10 * math.Pi / 180},
		{"rod leaning by 45°", rod, math.Pi / 4},
		{"rod leaning by 80°", rod, 80 * math.Pi / 180},
	}
	for _, c := range cases {
		w := newScene(1)
		w.Gravity = mgl64.Vec3{}
		body := addBody(w, mgl64.Vec3{}, mgl64.QuatRotate(c.lean, mgl64.Vec3{0, 0, 1}), &actor.Box{HalfExtents: c.halfExtents}, actor.BodyTypeDynamic, 0, 0)
		body.AngularLock = actor.AxisX | actor.AxisZ
		body.Material.AngularDamping = 0
		inertia := body.GetInertiaWorld()
		want := torque * seconds / inertia[4]
		simulate(w, seconds, func() { body.AddTorque(mgl64.Vec3{0, torque, 0}) })
		// the torque of the last callback is not integrated yet, the one of the first step is missing
		simulate(w, sceneDt, nil)
		spin := body.AngularVelocity
		if spin.X() != 0 || spin.Z() != 0 {
			t.Errorf("%s: the body turns around its locked axes: %v", c.name, spin)
		}
		if math.Abs(spin.Y()-want) > 0.01*want {
			t.Errorf("%s: %.4f rad/s around Y after %v N·m during %v s, want %.4f (%.2f times the body on its axle)", c.name, spin.Y(), torque, seconds, want, spin.Y()/want)
		}
	}
}

// ========== FRICTION OFF THE AXES ==========
// A plate which only moves along Y and turns around Y, spinning on a static ball off its axis: the friction stops it
// as Coulomb says, t = ω I / (µ m g r), wherever the ball is around the axis. The plate answers to the friction along
// a single direction, which is not a tangent of the contact
func TestLockedPlateRubsOffItsAxes(t *testing.T) {
	const friction, spin, distance, ballRadius = 0.5, 4.0, 0.7, 0.1
	half := mgl64.Vec3{1, 0.1, 1}
	// I / m of a box around Y
	want := spin * (half.X()*half.X() + half.Z()*half.Z()) / 3 / (friction * sceneGravity * distance)
	for _, azimuth := range []float64{0, 30, 60, 89} {
		angle := azimuth * math.Pi / 180
		w := newScene(1)
		addBody(w, mgl64.Vec3{distance * math.Cos(angle), 0, distance * math.Sin(angle)}, mgl64.QuatIdent(), &actor.Sphere{Radius: ballRadius}, actor.BodyTypeStatic, friction, 0)
		plate := addBody(w, mgl64.Vec3{0, ballRadius + half.Y(), 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: half}, actor.BodyTypeDynamic, friction, 0)
		plate.LinearLock, plate.AngularLock = actor.AxisX|actor.AxisZ, actor.AxisX|actor.AxisZ
		plate.Material.AngularDamping = 0
		simulate(w, 0.2, nil)
		plate.AngularVelocity = mgl64.Vec3{0, spin, 0}
		stopped, elapsed := -1.0, 0.0
		simulate(w, 3*want, func() {
			elapsed += sceneDt
			if stopped < 0 && math.Abs(plate.AngularVelocity.Y()) < 0.01 {
				stopped = elapsed
			}
		})
		if math.Abs(stopped-want) > 0.05*want {
			t.Errorf("azimuth %v°: the plate stopped after %.2f s (-1: never), want %.2f", azimuth, stopped, want)
		}
	}
}

// ========== A JOINT TO THE WORLD, EACH LOCKED AXIS ==========
// A body locked along one axis, pinned to the world at a point off its center: after one pass its anchor doesn't move
// anymore, and neither does its locked axis. The inverse mass of the point is the one of each axis
func TestLockedBodyPinnedToTheWorldIsExact(t *testing.T) {
	for axis, lock := range []actor.Axes{actor.AxisX, actor.AxisY, actor.AxisZ} {
		w := exactScene()
		anchor := anchorBody(w, mgl64.Vec3{0.2, 0.1, -0.15})
		body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0, 0)
		body.LinearLock = lock
		body.Velocity = mgl64.Vec3{1, 2, 3}
		joint := NewBallJoint(anchor, body, mgl64.Vec3{0.2, 0.1, -0.15}, mgl64.Vec3{1, 0, 0})
		w.AddJoint(joint)
		w.Step(sceneDt)
		if velocity := anchorVelocity(&joint.JointBase).Len(); velocity > 1e-9 {
			t.Errorf("locked along %d: the anchor moves at %.3g m/s after a rigid pass, want it held", axis, velocity)
		}
		if body.Velocity[axis] != 0 || body.AngularVelocity.Len() < 0.1 {
			t.Errorf("locked along %d: velocity %v, angular velocity %v, want the locked axis still and the body turning around its anchor", axis, body.Velocity, body.AngularVelocity)
		}
	}
}

// ========== A LOCKED BODY TURNING DURING THE STEP ==========
// The lever arms of a contact turn with a body which spins fast: the mass of its normal row is computed again, along
// the normal, with the inverse masses of the free axes
func TestTurnedContactKeepsTheLockedMass(t *testing.T) {
	const radius = 0.25
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	normal := mgl64.Vec3{1, 0, 2}.Normalize()
	locked := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 0)
	locked.LinearLock = actor.AxisX
	locked.AngularVelocity = mgl64.Vec3{0, 60, 0}
	ball := addBody(w, normal.Mul(2*radius-0.001), mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0, 0)
	w.Step(sceneDt)

	s := &w.solver
	if len(s.constraints) != 1 || s.constraints[0].pointsCount != 1 {
		t.Fatalf("%d contacts, want 1 of 1 point", len(s.constraints))
	}
	c := &s.constraints[0]
	state := s.state(c.indexA)
	if state.body != locked {
		state = s.state(c.indexB)
	}
	if state.body != locked || math.Abs(state.deltaRotation.W) >= turnAnchorsCos {
		t.Fatalf("the locked ball turned by %.3f rad, the test needs its anchors turned", rotationAngle(state.deltaRotation))
	}
	// the arms of 2 balls are along the normal: no angular term
	inverseMass := ball.InverseMass()
	want := 1 / (c.normal.X()*c.normal.X()*inverseMass + c.normal.Y()*c.normal.Y()*2*inverseMass + c.normal.Z()*c.normal.Z()*2*inverseMass)
	if mass := c.points[0].normal.mass; math.Abs(mass-want) > 1e-9*want {
		t.Errorf("the mass of the turned normal row is %v, want %v: 1 / (n·(MA⁻¹ + MB⁻¹)·n)", mass, want)
	}
}

// ========== CONTINUOUS COLLISION OF A BODY WHICH STILL TURNS ==========
// A body locked around some axes only turns during its motion: stopped at its first impact, it has the rotation of the
// impact, not the one of the end of its motion
func TestContinuousTurnsAPartlyLockedBody(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.5)
	box := addBody(w, mgl64.Vec3{0, 3, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	w.Step(sceneDt)
	scratch := ccdPool.Get().(*ccdScratch)
	defer ccdPool.Put(scratch)
	scratch.core.Radius = coreFraction * cubeHalf
	const turn = 1.2
	for _, lock := range []actor.Axes{actor.AxisX, actor.AxisX | actor.AxisZ} {
		box.AngularLock = lock
		motion := sweep{
			start: actor.Transform{Position: mgl64.Vec3{0, 3, 0}, Rotation: mgl64.QuatIdent()},
			end:   actor.Transform{Position: mgl64.Vec3{0, -3, 0}, Rotation: mgl64.QuatRotate(turn, mgl64.Vec3{0, 1, 0})},
		}
		box.Transform = motion.end
		w.stopAtImpact(box, &motion, cube().HalfExtents.Len(), scratch)
		y := box.Transform.Position.Y()
		if y < 0 || y > 3 {
			t.Fatalf("locks %d: the box is at y=%.3f, want it stopped above the ground", lock, y)
		}
		// it fell by (3 - y) of the 6 m of its motion
		want := turn * (3 - y) / 6
		if angle := rotationAngle(box.Transform.Rotation); math.Abs(angle-want) > 0.01 {
			t.Errorf("locks %d: the stopped box turned by %.3f rad, want %.3f (%.3f at the end of its motion)", lock, angle, want, turn)
		}
	}
}
