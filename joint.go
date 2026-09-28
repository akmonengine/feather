package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Joints are solved like the contacts (TGS Soft, as in Box2D v3): warm starting, soft constraints in the push,
// then rigid constraints in the relax. Each joint links 2 bodies with a frame on each body.
// The X axis of the frames is the axis of the hinge, and the twist axis of the ball joint (as in PhysX).
const (
	// DefaultJointHertz is the stiffness of the joints
	DefaultJointHertz = 60.0

	// DefaultJointDampingRatio of the joints
	DefaultJointDampingRatio = 2.0

	// the joint hertz can't exceed 1/4 of the sub-steps rate
	jointHertzPerSubstepRate = 0.25

	// under this length, the axis of a distance joint is not reliable (m)
	jointMinLength = 1e-9
)

// Joint links 2 bodies (see DistanceJoint, BallJoint, HingeJoint, FixedJoint)
type Joint interface {
	base() *JointBase
	prepare(s *solver)
	warmStart(s *solver)
	solve(s *solver, useBias bool)
}

// JointBase holds the settings common to all the joints
type JointBase struct {
	BodyA *actor.RigidBody // can be static: the joint is attached to the world
	BodyB *actor.RigidBody
	// LocalFrameA & LocalFrameB: the anchor and the orientation of the joint in the local space of each body
	LocalFrameA actor.Transform
	LocalFrameB actor.Transform
	// CollideConnected: if false, the 2 bodies don't collide with each other
	CollideConnected bool
	// Hertz & DampingRatio: the softness of the joint. 0 hertz = DefaultJointHertz
	Hertz        float64
	DampingRatio float64

	// ========== solver ==========
	indexA, indexB int
	frameA, frameB mgl64.Quat // world rotation of the frames, at the beginning of the step
	anchorA        mgl64.Vec3 // anchors from the centers of mass, world orientation, at the beginning of the step
	anchorB        mgl64.Vec3
	deltaCenter    mgl64.Vec3
	spring         spring

	linearImpulse mgl64.Vec3
	// inArticulation: the point constraint is solved with the other joints of its tree (articulation.go)
	inArticulation bool
}

func (j *JointBase) base() *JointBase { return j }

// prepareBase computes the world frames of the joint, at the beginning of the step
func (j *JointBase) prepareBase(s *solver) {
	j.indexA, j.indexB = s.indexOf(j.BodyA), s.indexOf(j.BodyB)
	transformA, transformB := j.BodyA.Transform, j.BodyB.Transform
	j.frameA = transformA.Rotation.Mul(j.LocalFrameA.Rotation).Normalize()
	j.frameB = transformB.Rotation.Mul(j.LocalFrameB.Rotation).Normalize()
	j.anchorA = transformA.Rotation.Rotate(j.LocalFrameA.Position)
	j.anchorB = transformB.Rotation.Rotate(j.LocalFrameB.Position)
	j.deltaCenter = transformB.Position.Sub(transformA.Position)

	hertz := j.Hertz
	if hertz <= 0 {
		hertz = DefaultJointHertz
	}
	j.spring = newSpring(math.Min(hertz, jointHertzPerSubstepRate*s.invH), j.DampingRatio, s.h)
}

// currentAnchors during the substeps
func (j *JointBase) currentAnchors(stateA, stateB *bodyState) (mgl64.Vec3, mgl64.Vec3) {
	return stateA.deltaMatrix.Mul3x1(j.anchorA), stateB.deltaMatrix.Mul3x1(j.anchorB)
}

// currentFrames: world rotation of both frames during the substeps
func (j *JointBase) currentFrames(stateA, stateB *bodyState) (mgl64.Quat, mgl64.Quat) {
	return stateA.deltaRotation.Mul(j.frameA), stateB.deltaRotation.Mul(j.frameB)
}

// ========== Point constraint ==========
// The anchors of both bodies stay at the same place (3 rows)

func (j *JointBase) solvePoint(s *solver, stateA, stateB *bodyState, useBias bool) {
	if j.inArticulation {
		return
	}
	rA, rB := j.currentAnchors(stateA, stateB)
	cdot := stateB.velocity.Add(stateB.angularVelocity.Cross(rB)).Sub(stateA.velocity.Add(stateA.angularVelocity.Cross(rA)))

	bias, row := mgl64.Vec3{}, rigid
	if useBias {
		separation := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(rB.Sub(rA)).Add(j.deltaCenter)
		row = j.spring
		bias = separation.Mul(row.biasRate)
	}

	// K = (mA + mB) I - [rA]x IA [rA]x - [rB]x IB [rB]x
	skewA, skewB := skew(rA), skew(rB)
	k := mgl64.Ident3().Mul(stateA.invMass + stateB.invMass).Sub(skewA.Mul3(stateA.inverseInertia).Mul3(skewA)).Sub(skewB.Mul3(stateB.inverseInertia).Mul3(skewB))
	if math.Abs(k.Det()) < 1e-30 {
		return
	}
	impulse := row.impulse3(k.Inv(), cdot, bias, j.linearImpulse)
	j.linearImpulse = j.linearImpulse.Add(impulse)
	applyLinear(stateA, stateB, rA, rB, impulse)
}

// applyLinear: -impulse at rA on A, +impulse at rB on B
func applyLinear(stateA, stateB *bodyState, rA, rB, impulse mgl64.Vec3) {
	if stateA.body != nil {
		stateA.velocity = stateA.velocity.Sub(impulse.Mul(stateA.invMass))
		stateA.angularVelocity = stateA.angularVelocity.Sub(stateA.inverseInertia.Mul3x1(rA.Cross(impulse)))
	}
	if stateB.body != nil {
		stateB.velocity = stateB.velocity.Add(impulse.Mul(stateB.invMass))
		stateB.angularVelocity = stateB.angularVelocity.Add(stateB.inverseInertia.Mul3x1(rB.Cross(impulse)))
	}
}

// applyAngular: -impulse on A, +impulse on B
func applyAngular(stateA, stateB *bodyState, impulse mgl64.Vec3) {
	if stateA.body != nil {
		stateA.angularVelocity = stateA.angularVelocity.Sub(stateA.inverseInertia.Mul3x1(impulse))
	}
	if stateB.body != nil {
		stateB.angularVelocity = stateB.angularVelocity.Add(stateB.inverseInertia.Mul3x1(impulse))
	}
}

// axialMass for an angular impulse around the axis
func axialMass(stateA, stateB *bodyState, axis mgl64.Vec3) float64 {
	k := axis.Dot(stateA.inverseInertia.Mul3x1(axis)) + axis.Dot(stateB.inverseInertia.Mul3x1(axis))
	if k <= 0 {
		return 0
	}
	return 1 / k
}

// solveAngularLimit: C >= 0 around the axis, with C = direction * (angle of B around the axis) + offset.
// Returns the new accumulated impulse. Speculative when C > 0, soft when useBias.
func (j *JointBase) solveAngularLimit(s *solver, stateA, stateB *bodyState, axis mgl64.Vec3, c float64, direction float64, accumulated float64, useBias bool) float64 {
	bias, row := j.limitRow(s, c, useBias)
	cdot := direction * axis.Dot(stateB.angularVelocity.Sub(stateA.angularVelocity))
	impulse := row.impulse(axialMass(stateA, stateB, axis), cdot, bias, accumulated)
	newImpulse := math.Max(accumulated+impulse, 0)
	applyAngular(stateA, stateB, axis.Mul(direction*(newImpulse-accumulated)))
	return newImpulse
}

// limitRow: the bias and the spring of a limit C >= 0. Speculative when C > 0 (the bodies can get closer by C during
// the substep), soft when useBias, rigid otherwise
func (j *JointBase) limitRow(s *solver, c float64, useBias bool) (float64, spring) {
	switch {
	case c > 0:
		return c * s.invH, rigid
	case useBias:
		return j.spring.biasRate * c, j.spring
	}
	return 0, rigid
}

// rotationError is the rotation vector (world space) from the target to the current rotation, for small errors
func rotationError(current, target mgl64.Quat) mgl64.Vec3 {
	q := current.Mul(target.Conjugate())
	if q.W < 0 {
		q = q.Scale(-1)
	}
	return q.V.Mul(2)
}

// twistAngle: the rotation of B relative to A around the X axis, after the swing (swing-twist decomposition)
func twistAngle(frameA, frameB mgl64.Quat) float64 {
	relative := frameA.Conjugate().Mul(frameB)
	if relative.W < 0 {
		relative = relative.Scale(-1)
	}
	return 2 * math.Atan2(relative.V.X(), relative.W)
}

// angleRow: a rotation constraint on a function f(p) of the X axis of the frame B seen in the frame A (p, unit vector).
// p moves as dp/dt = ω × p, so the rate of f is ω · (p × ∇f): returns f(p), the unit rotation axis p × ∇f
// (orthogonal to p: no twist), and the rate of f per unit of angular velocity around this axis
func angleRow(p mgl64.Vec3, f func(p mgl64.Vec3) float64) (float64, mgl64.Vec3, float64, bool) {
	// gradient of f in the tangent plane of p
	const epsilon = 1e-6
	tangent1 := anyPerpendicular(p)
	tangent2 := p.Cross(tangent1)
	d1 := (f(p.Add(tangent1.Mul(epsilon)).Normalize()) - f(p.Sub(tangent1.Mul(epsilon)).Normalize())) / (2 * epsilon)
	d2 := (f(p.Add(tangent2.Mul(epsilon)).Normalize()) - f(p.Sub(tangent2.Mul(epsilon)).Normalize())) / (2 * epsilon)
	axis := p.Cross(tangent1.Mul(d1).Add(tangent2.Mul(d2)))
	rate := axis.Len()
	if rate < 1e-12 {
		return 0, mgl64.Vec3{}, 0, false
	}
	return f(p), axis.Mul(1 / rate), rate, true
}

// swingLimit: p must stay in the elliptic cone of the 2 half angles.
// Returns the distance to the cone (rad, > 0 inside) and the rotation axis moving p out of the cone
func swingLimit(p mgl64.Vec3, limitY, limitZ float64) (float64, mgl64.Vec3, bool) {
	cone := func(p mgl64.Vec3) float64 {
		r := math.Hypot(p.Y(), p.Z())
		if r < 1e-12 {
			return 0
		}
		// rotation around Z moves X towards Y, rotation around Y moves X towards -Z
		return math.Atan2(r, p.X()) / r * math.Hypot(p.Y()/limitZ, p.Z()/limitY)
	}
	if cone(p) == 0 {
		return 0, mgl64.Vec3{}, false
	}
	f, axis, rate, ok := angleRow(p, cone)
	if !ok {
		return 0, mgl64.Vec3{}, false
	}
	return (1 - f) / rate, axis, true
}

// twistRow: the twist angle, and its exact rate: d(twist)/dt = rate * (axis · (wB - wA)).
// The rate of the twist is not along the axes X when B swings: it is measured by turning B a little around each axis.
// A rotation of both frames together doesn't change the twist, so the rate depends on wB - wA only
func twistRow(frameA, frameB mgl64.Quat) (float64, mgl64.Vec3, float64) {
	const epsilon = 1e-6
	twist := twistAngle(frameA, frameB)
	var gradient mgl64.Vec3
	for k := 0; k < 3; k++ {
		var delta mgl64.Vec3
		delta[k] = epsilon
		plus := twistAngle(frameA, integrateRotation(frameB, delta))
		minus := twistAngle(frameA, integrateRotation(frameB, delta.Mul(-1)))
		gradient[k] = math.Remainder(plus-minus, 2*math.Pi) / (2 * epsilon)
	}
	rate := gradient.Len()
	if rate < 1e-12 {
		return twist, frameA.Rotate(mgl64.Vec3{1, 0, 0}), 1
	}
	return twist, gradient.Mul(1 / rate), rate
}

// frameFromAxis returns a rotation turning X onto the axis
func frameFromAxis(axis mgl64.Vec3) mgl64.Quat {
	return mgl64.QuatBetweenVectors(mgl64.Vec3{1, 0, 0}, axis.Normalize())
}

// newJointBase: the joint frames from an anchor and an orientation in world space
func newJointBase(bodyA, bodyB *actor.RigidBody, anchor mgl64.Vec3, frame mgl64.Quat) JointBase {
	local := func(body *actor.RigidBody) actor.Transform {
		return actor.Transform{Position: body.Transform.ToLocal(anchor), Rotation: body.Transform.Rotation.Conjugate().Mul(frame).Normalize()}
	}
	return JointBase{
		BodyA:        bodyA,
		BodyB:        bodyB,
		LocalFrameA:  local(bodyA),
		LocalFrameB:  local(bodyB),
		Hertz:        DefaultJointHertz,
		DampingRatio: DefaultJointDampingRatio,
	}
}

// ========== Distance ==========

// DistanceJoint keeps the anchors at a distance: fixed (Length), in a range [MinLength, MaxLength] (a rope),
// or with a spring
type DistanceJoint struct {
	JointBase
	Length float64

	EnableLimit bool
	MinLength   float64
	MaxLength   float64

	EnableSpring       bool
	SpringHertz        float64
	SpringDampingRatio float64

	springRow    spring
	impulse      float64
	lowerImpulse float64
	upperImpulse float64
}

// NewDistanceJoint links 2 anchors (world space), at their current distance
func NewDistanceJoint(bodyA, bodyB *actor.RigidBody, anchorA, anchorB mgl64.Vec3) *DistanceJoint {
	j := &DistanceJoint{JointBase: newJointBase(bodyA, bodyB, anchorA, mgl64.QuatIdent())}
	j.LocalFrameB.Position = bodyB.Transform.ToLocal(anchorB)
	j.Length = anchorB.Sub(anchorA).Len()
	j.MinLength, j.MaxLength = j.Length, j.Length
	return j
}

func (j *DistanceJoint) prepare(s *solver) {
	j.prepareBase(s)
	if j.SpringHertz > 0 {
		j.springRow = newSpring(j.SpringHertz, j.SpringDampingRatio, s.h)
	}
}

func (j *DistanceJoint) axis(stateA, stateB *bodyState) (mgl64.Vec3, mgl64.Vec3, mgl64.Vec3, float64) {
	rA, rB := j.currentAnchors(stateA, stateB)
	separation := j.deltaCenter.Add(stateB.deltaPosition.Sub(stateA.deltaPosition)).Add(rB.Sub(rA))
	length := separation.Len()
	if length < jointMinLength {
		return rA, rB, mgl64.Vec3{0, 1, 0}, length
	}
	return rA, rB, separation.Mul(1 / length), length
}

func (j *DistanceJoint) warmStart(s *solver) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	rA, rB, axis, _ := j.axis(stateA, stateB)
	applyLinear(stateA, stateB, rA, rB, axis.Mul(j.impulse+j.lowerImpulse-j.upperImpulse))
}

// solveLinearAxis solves an impulse along the axis, applied at rA on A and rB on B: returns the new accumulated impulse
func solveLinearAxis(stateA, stateB *bodyState, rA, rB, axis mgl64.Vec3, bias float64, row spring, accumulated, low, high float64) float64 {
	cdot := axis.Dot(stateB.velocity.Add(stateB.angularVelocity.Cross(rB)).Sub(stateA.velocity.Add(stateA.angularVelocity.Cross(rA))))
	rnA, rnB := rA.Cross(axis), rB.Cross(axis)
	k := stateA.invMass + stateB.invMass + rnA.Dot(stateA.inverseInertia.Mul3x1(rnA)) + rnB.Dot(stateB.inverseInertia.Mul3x1(rnB))
	if k <= 0 {
		return accumulated
	}
	impulse := row.impulse(1/k, cdot, bias, accumulated)
	newImpulse := math.Max(low, math.Min(high, accumulated+impulse))
	applyLinear(stateA, stateB, rA, rB, axis.Mul(newImpulse-accumulated))
	return newImpulse
}

func (j *DistanceJoint) solve(s *solver, useBias bool) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	rA, rB, axis, length := j.axis(stateA, stateB)
	infinite := math.Inf(1)

	if j.EnableSpring && (j.MinLength < j.MaxLength || !j.EnableLimit) {
		// ========== SPRING ==========
		if j.SpringHertz > 0 {
			bias := j.springRow.biasRate * (length - j.Length)
			j.impulse = solveLinearAxis(stateA, stateB, rA, rB, axis, bias, j.springRow, j.impulse, -infinite, infinite)
		}

		// ========== LIMITS ==========
		if j.EnableLimit {
			j.lowerImpulse = j.solveLimit(s, stateA, stateB, rA, rB, axis, length-j.MinLength, j.lowerImpulse, useBias)
			j.upperImpulse = j.solveLimit(s, stateA, stateB, rA, rB, axis.Mul(-1), j.MaxLength-length, j.upperImpulse, useBias)
		}
		return
	}

	// ========== RIGID ==========
	bias, row := 0.0, rigid
	if useBias {
		row = j.spring
		bias = row.biasRate * (length - j.Length)
	}
	j.impulse = solveLinearAxis(stateA, stateB, rA, rB, axis, bias, row, j.impulse, -infinite, infinite)
}

// solveLimit: C >= 0 along the axis
func (j *DistanceJoint) solveLimit(s *solver, stateA, stateB *bodyState, rA, rB, axis mgl64.Vec3, c, accumulated float64, useBias bool) float64 {
	bias, row := j.limitRow(s, c, useBias)
	return solveLinearAxis(stateA, stateB, rA, rB, axis, bias, row, accumulated, 0, math.Inf(1))
}

// ========== Ball ==========

// BallJoint (ball and socket) keeps the anchors together, the bodies rotate freely.
// Optional limits: a cone for the swing of the X axis (half angles around Y and Z), and a range for the twist
// around X. Optional drive: a spring towards a target rotation of the frame B relative to the frame A.
type BallJoint struct {
	JointBase

	EnableSwingLimit bool
	SwingLimitY      float64 // rad, rotation of the X axis around Y
	SwingLimitZ      float64 // rad, rotation of the X axis around Z

	EnableTwistLimit bool
	TwistMin         float64 // rad
	TwistMax         float64 // rad

	EnableDrive       bool
	DriveTarget       mgl64.Quat // rotation of the frame B relative to the frame A
	DriveHertz        float64
	DriveDampingRatio float64

	driveRow          spring
	swingImpulse      float64
	twistLowerImpulse float64
	twistUpperImpulse float64
	driveImpulse      mgl64.Vec3
	swingAxis         mgl64.Vec3
	twistAxis         mgl64.Vec3
}

// NewBallJoint links 2 bodies at an anchor (world space). The twist axis is X of the frames: twistAxis in world space
func NewBallJoint(bodyA, bodyB *actor.RigidBody, anchor, twistAxis mgl64.Vec3) *BallJoint {
	return &BallJoint{JointBase: newJointBase(bodyA, bodyB, anchor, frameFromAxis(twistAxis)), DriveTarget: mgl64.QuatIdent()}
}

func (j *BallJoint) prepare(s *solver) {
	j.prepareBase(s)
	if j.DriveHertz > 0 {
		j.driveRow = newSpring(j.DriveHertz, j.DriveDampingRatio, s.h)
	}
}

func (j *BallJoint) warmStart(s *solver) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	rA, rB := j.currentAnchors(stateA, stateB)
	applyLinear(stateA, stateB, rA, rB, j.linearImpulse)
	applyAngular(stateA, stateB, j.driveImpulse.Add(j.twistAxis.Mul(j.twistLowerImpulse-j.twistUpperImpulse)).Sub(j.swingAxis.Mul(j.swingImpulse)))
}

func (j *BallJoint) solve(s *solver, useBias bool) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	frameA, frameB := j.currentFrames(stateA, stateB)

	// ========== DRIVE ==========
	if j.EnableDrive && j.DriveHertz > 0 {
		c := rotationError(frameB, frameA.Mul(j.DriveTarget))
		cdot := stateB.angularVelocity.Sub(stateA.angularVelocity)
		k := stateA.inverseInertia.Add(stateB.inverseInertia)
		if math.Abs(k.Det()) > 1e-30 {
			impulse := j.driveRow.impulse3(k.Inv(), cdot, c.Mul(j.driveRow.biasRate), j.driveImpulse)
			j.driveImpulse = j.driveImpulse.Add(impulse)
			applyAngular(stateA, stateB, impulse)
		}
	}

	// ========== TWIST LIMITS ==========
	if j.EnableTwistLimit {
		twist, axis, rate := twistRow(frameA, frameB)
		j.twistAxis = axis
		j.twistLowerImpulse = j.solveAngularLimit(s, stateA, stateB, j.twistAxis, (twist-j.TwistMin)/rate, 1, j.twistLowerImpulse, useBias)
		j.twistUpperImpulse = j.solveAngularLimit(s, stateA, stateB, j.twistAxis, (j.TwistMax-twist)/rate, -1, j.twistUpperImpulse, useBias)
	}

	// ========== SWING LIMIT (elliptic cone) ==========
	if j.EnableSwingLimit {
		p := frameA.Conjugate().Rotate(frameB.Rotate(mgl64.Vec3{1, 0, 0}))
		if c, axis, ok := swingLimit(p, j.SwingLimitY, j.SwingLimitZ); ok {
			j.swingAxis = frameA.Rotate(axis)
			j.swingImpulse = j.solveAngularLimit(s, stateA, stateB, j.swingAxis, c, -1, j.swingImpulse, useBias)
		}
	}

	// ========== POINT ==========
	j.solvePoint(s, stateA, stateB, useBias)
}

// ========== Hinge ==========

// HingeJoint keeps the anchors together, the bodies rotate around the X axis of the frames only (a door, a wheel).
// Optional: an angle range, a motor (speed and max torque), a spring towards a target angle.
type HingeJoint struct {
	JointBase

	EnableLimit bool
	LowerAngle  float64 // rad
	UpperAngle  float64 // rad

	EnableMotor    bool
	MotorSpeed     float64 // rad/s
	MaxMotorTorque float64 // N·m

	EnableSpring       bool
	TargetAngle        float64 // rad
	SpringHertz        float64
	SpringDampingRatio float64

	springRow      spring
	angularImpulse mgl64.Vec3 // keeps the axes aligned
	lowerImpulse   float64
	upperImpulse   float64
	motorImpulse   float64
	springImpulse  float64
	axis           mgl64.Vec3
}

// NewHingeJoint links 2 bodies at an anchor, rotating around an axis (world space)
func NewHingeJoint(bodyA, bodyB *actor.RigidBody, anchor, axis mgl64.Vec3) *HingeJoint {
	return &HingeJoint{JointBase: newJointBase(bodyA, bodyB, anchor, frameFromAxis(axis))}
}

func (j *HingeJoint) prepare(s *solver) {
	j.prepareBase(s)
	if j.SpringHertz > 0 {
		j.springRow = newSpring(j.SpringHertz, j.SpringDampingRatio, s.h)
	}
	j.axis = j.frameA.Rotate(mgl64.Vec3{1, 0, 0})
}

// Angle of the frame B around the axis, relative to the frame A
func (j *HingeJoint) Angle() float64 {
	frameA := j.BodyA.Transform.Rotation.Mul(j.LocalFrameA.Rotation)
	frameB := j.BodyB.Transform.Rotation.Mul(j.LocalFrameB.Rotation)
	return hingeAngle(frameA, frameB)
}

func hingeAngle(frameA, frameB mgl64.Quat) float64 {
	axis := frameA.Rotate(mgl64.Vec3{1, 0, 0})
	yA, yB := frameA.Rotate(mgl64.Vec3{0, 1, 0}), frameB.Rotate(mgl64.Vec3{0, 1, 0})
	return math.Atan2(axis.Dot(yA.Cross(yB)), yA.Dot(yB))
}

func (j *HingeJoint) warmStart(s *solver) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	rA, rB := j.currentAnchors(stateA, stateB)
	applyLinear(stateA, stateB, rA, rB, j.linearImpulse)
	axial := j.springImpulse + j.motorImpulse + j.lowerImpulse - j.upperImpulse
	applyAngular(stateA, stateB, j.angularImpulse.Add(j.axis.Mul(axial)))
}

func (j *HingeJoint) solve(s *solver, useBias bool) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	frameA, frameB := j.currentFrames(stateA, stateB)
	j.axis = frameA.Rotate(mgl64.Vec3{1, 0, 0})
	angle := hingeAngle(frameA, frameB)
	mass := axialMass(stateA, stateB, j.axis)
	cdot := func() float64 { return j.axis.Dot(stateB.angularVelocity.Sub(stateA.angularVelocity)) }

	// ========== SPRING ==========
	if j.EnableSpring && j.SpringHertz > 0 {
		c := math.Remainder(angle-j.TargetAngle, 2*math.Pi)
		impulse := j.springRow.impulse(mass, cdot(), j.springRow.biasRate*c, j.springImpulse)
		j.springImpulse += impulse
		applyAngular(stateA, stateB, j.axis.Mul(impulse))
	}

	// ========== MOTOR ==========
	if j.EnableMotor {
		impulse := -mass * (cdot() - j.MotorSpeed)
		maxImpulse := s.h * j.MaxMotorTorque
		old := j.motorImpulse
		j.motorImpulse = math.Max(-maxImpulse, math.Min(maxImpulse, old+impulse))
		applyAngular(stateA, stateB, j.axis.Mul(j.motorImpulse-old))
	}

	// ========== LIMITS ==========
	if j.EnableLimit {
		j.lowerImpulse = j.solveAngularLimit(s, stateA, stateB, j.axis, angle-j.LowerAngle, 1, j.lowerImpulse, useBias)
		j.upperImpulse = j.solveAngularLimit(s, stateA, stateB, j.axis, j.UpperAngle-angle, -1, j.upperImpulse, useBias)
	}

	// ========== AXIS: B turns only around the axis of A ==========
	{
		u1, u2 := frameA.Rotate(mgl64.Vec3{0, 1, 0}), frameA.Rotate(mgl64.Vec3{0, 0, 1})
		axisError := j.axis.Cross(frameB.Rotate(mgl64.Vec3{1, 0, 0}))
		relative := stateB.angularVelocity.Sub(stateA.angularVelocity)
		k := stateA.inverseInertia.Add(stateB.inverseInertia)
		k11, k12, k22 := u1.Dot(k.Mul3x1(u1)), u1.Dot(k.Mul3x1(u2)), u2.Dot(k.Mul3x1(u2))
		det := k11*k22 - k12*k12
		if det > 1e-30 {
			bias1, bias2, row := 0.0, 0.0, rigid
			if useBias {
				row = j.spring
				bias1, bias2 = row.biasRate*u1.Dot(axisError), row.biasRate*u2.Dot(axisError)
			}
			b1, b2 := u1.Dot(relative)+bias1, u2.Dot(relative)+bias2
			// solve the 2x2 system
			l1 := (k22*b1 - k12*b2) / det
			l2 := (k11*b2 - k12*b1) / det
			accumulated1, accumulated2 := j.angularImpulse.Dot(u1), j.angularImpulse.Dot(u2)
			// λ = -(K⁻¹ (v + b) + gamma * accumulated) / (1 + gamma), on both axes
			scale := -1 / (1 + row.gamma)
			impulse := u1.Mul(scale * (l1 + row.gamma*accumulated1)).Add(u2.Mul(scale * (l2 + row.gamma*accumulated2)))
			j.angularImpulse = j.angularImpulse.Add(impulse)
			applyAngular(stateA, stateB, impulse)
		}
	}

	// ========== POINT ==========
	j.solvePoint(s, stateA, stateB, useBias)
}

// ========== Fixed ==========

// FixedJoint freezes the position and the rotation of B relative to A
type FixedJoint struct {
	JointBase
	angularImpulse mgl64.Vec3
}

// NewFixedJoint welds 2 bodies at an anchor (world space)
func NewFixedJoint(bodyA, bodyB *actor.RigidBody, anchor mgl64.Vec3) *FixedJoint {
	return &FixedJoint{JointBase: newJointBase(bodyA, bodyB, anchor, mgl64.QuatIdent())}
}

func (j *FixedJoint) prepare(s *solver) { j.prepareBase(s) }

func (j *FixedJoint) warmStart(s *solver) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	rA, rB := j.currentAnchors(stateA, stateB)
	applyLinear(stateA, stateB, rA, rB, j.linearImpulse)
	applyAngular(stateA, stateB, j.angularImpulse)
}

func (j *FixedJoint) solve(s *solver, useBias bool) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)

	// ========== ANGULAR ==========
	frameA, frameB := j.currentFrames(stateA, stateB)
	cdot := stateB.angularVelocity.Sub(stateA.angularVelocity)
	bias := mgl64.Vec3{}
	row := rigid
	if useBias {
		row = j.spring
		bias = rotationError(frameB, frameA).Mul(row.biasRate)
	}
	k := stateA.inverseInertia.Add(stateB.inverseInertia)
	if math.Abs(k.Det()) > 1e-30 {
		impulse := row.impulse3(k.Inv(), cdot, bias, j.angularImpulse)
		j.angularImpulse = j.angularImpulse.Add(impulse)
		applyAngular(stateA, stateB, impulse)
	}

	// ========== POINT ==========
	j.solvePoint(s, stateA, stateB, useBias)
}
