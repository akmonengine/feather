package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Motion of an axis of a ConfigurableJoint
type Motion int

const (
	MotionLocked Motion = iota
	MotionLimited
	MotionFree
)

// ConfigurableJoint sets each of the 6 axes of the frame A (like the D6 joint of PhysX): locked, limited or free.
// - linear X, Y, Z: the position of the anchor B along the axes of the frame A
// - twist: rotation around X, swing Y & swing Z: rotation of the X axis around Y and Z.
// If both swings are limited, they form an elliptic cone.
// Optional drives: towards a target position (in the frame A) and a target rotation (of the frame B relative to A).
// With everything locked it is a fixed joint, with only the twist free a hinge, with the linear locked a ball joint...
type ConfigurableJoint struct {
	JointBase

	LinearMotion [3]Motion
	LinearMin    mgl64.Vec3 // m, along X, Y, Z of the frame A
	LinearMax    mgl64.Vec3

	TwistMotion Motion
	TwistMin    float64 // rad
	TwistMax    float64

	SwingYMotion Motion
	SwingZMotion Motion
	SwingLimitY  float64 // rad, half angle of the rotation of X around Y
	SwingLimitZ  float64 // rad, half angle of the rotation of X around Z

	EnableLinearDrive       bool
	DriveTargetPosition     mgl64.Vec3 // in the frame A
	LinearDriveHertz        float64
	LinearDriveDampingRatio float64

	EnableAngularDrive       bool
	DriveTargetRotation      mgl64.Quat // rotation of the frame B relative to the frame A
	AngularDriveHertz        float64
	AngularDriveDampingRatio float64

	linearDriveRow  spring
	angularDriveRow spring
	// accumulated impulses, per axis: [0] for a locked axis or a lower limit, [1] for an upper limit
	linearImpulses      [3][2]float64
	linearDriveImpulses [3]float64
	twistImpulses       [2]float64
	swingImpulses       [2][2]float64
	coneImpulse         float64
	angularImpulse      mgl64.Vec3
	angularDriveImpulse mgl64.Vec3
	// axes of the last solve, for the warm starting
	linearAxes [3]mgl64.Vec3
	twistAxis  mgl64.Vec3
	swingAxes  [2]mgl64.Vec3
	coneAxis   mgl64.Vec3
}

// NewConfigurableJoint links 2 bodies at an anchor (world space), X of the frames along the axis.
// Everything is locked by default: set the motion of each axis
func NewConfigurableJoint(bodyA, bodyB *actor.RigidBody, anchor, axis mgl64.Vec3) *ConfigurableJoint {
	return &ConfigurableJoint{
		JointBase:           newJointBase(bodyA, bodyB, anchor, frameFromAxis(axis)),
		DriveTargetRotation: mgl64.QuatIdent(),
	}
}

func (j *ConfigurableJoint) prepare(s *solver) {
	j.prepareBase(s)
	if j.EnableLinearDrive && j.LinearDriveHertz > 0 {
		j.linearDriveRow = newSpring(j.LinearDriveHertz, j.LinearDriveDampingRatio, s.h)
	}
	if j.EnableAngularDrive && j.AngularDriveHertz > 0 {
		j.angularDriveRow = newSpring(j.AngularDriveHertz, j.AngularDriveDampingRatio, s.h)
	}
}

func (j *ConfigurableJoint) allLinearLocked() bool {
	return j.LinearMotion[0] == MotionLocked && j.LinearMotion[1] == MotionLocked && j.LinearMotion[2] == MotionLocked
}

func (j *ConfigurableJoint) allAngularLocked() bool {
	return j.TwistMotion == MotionLocked && j.SwingYMotion == MotionLocked && j.SwingZMotion == MotionLocked
}

// linearState: the anchors, and the offset of the anchor B from the anchor A (world space)
func (j *ConfigurableJoint) linearState(stateA, stateB *bodyState) (mgl64.Vec3, mgl64.Vec3, mgl64.Vec3) {
	rA, rB := j.currentAnchors(stateA, stateB)
	offset := j.deltaCenter.Add(stateB.deltaPosition.Sub(stateA.deltaPosition)).Add(rB.Sub(rA))
	return rA, rB, offset
}

func (j *ConfigurableJoint) warmStart(s *solver) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	rA, rB, offset := j.linearState(stateA, stateB)

	// ========== LINEAR ==========
	if j.allLinearLocked() {
		applyLinear(stateA, stateB, rA, rB, j.linearImpulse)
	} else {
		for k := range j.linearAxes {
			impulse := j.linearImpulses[k][0] - j.linearImpulses[k][1] + j.linearDriveImpulses[k]
			applyLinear(stateA, stateB, rA.Add(offset), rB, j.linearAxes[k].Mul(impulse))
		}
	}

	// ========== ANGULAR ==========
	angular := j.angularDriveImpulse.Add(j.angularImpulse).Add(j.twistAxis.Mul(j.twistImpulses[0] - j.twistImpulses[1]))
	angular = angular.Sub(j.coneAxis.Mul(j.coneImpulse))
	for k := range j.swingAxes {
		angular = angular.Add(j.swingAxes[k].Mul(j.swingImpulses[k][0] - j.swingImpulses[k][1]))
	}
	applyAngular(stateA, stateB, angular)
}

func (j *ConfigurableJoint) solve(s *solver, useBias bool) {
	stateA, stateB := s.state(j.indexA), s.state(j.indexB)
	frameA, frameB := j.currentFrames(stateA, stateB)

	// ========== ANGULAR DRIVE ==========
	if j.EnableAngularDrive && j.AngularDriveHertz > 0 {
		c := rotationError(frameB, actor.MulQuat(&frameA, &j.DriveTargetRotation))
		j.angularDriveImpulse = solveAngular3(stateA, stateB, c, j.angularDriveRow, true, j.angularDriveImpulse)
	}

	// ========== ANGULAR ==========
	if j.allAngularLocked() {
		j.angularImpulse = solveAngular3(stateA, stateB, rotationError(frameB, frameA), j.spring, useBias, j.angularImpulse)
	} else {
		j.solveTwist(s, stateA, stateB, frameA, frameB, useBias)
		j.solveSwing(s, stateA, stateB, frameA, frameB, useBias)
	}

	// ========== LINEAR DRIVE ==========
	rA, rB, offset := j.linearState(stateA, stateB)
	if j.EnableLinearDrive && j.LinearDriveHertz > 0 {
		for k := 0; k < 3; k++ {
			if j.LinearMotion[k] == MotionLocked {
				continue
			}
			axis := actor.Rotate(&frameA, unitAxes[k])
			c := offset.Dot(axis) - j.DriveTargetPosition[k]
			drive := j.linearDriveRow
			j.linearDriveImpulses[k] = solveLinearAxis(stateA, stateB, rA.Add(offset), rB, axis, drive.biasRate*c, drive, j.linearDriveImpulses[k], math.Inf(-1), math.Inf(1))
		}
	}

	// ========== LINEAR ==========
	if j.allLinearLocked() {
		j.solvePoint(s, stateA, stateB, useBias)
		return
	}
	for k := 0; k < 3; k++ {
		axis := actor.Rotate(&frameA, unitAxes[k])
		j.linearAxes[k] = axis
		position := offset.Dot(axis)
		switch j.LinearMotion[k] {
		case MotionLocked:
			bias, row := j.equalityRow(useBias, position)
			j.linearImpulses[k][0] = solveLinearAxis(stateA, stateB, rA.Add(offset), rB, axis, bias, row, j.linearImpulses[k][0], math.Inf(-1), math.Inf(1))
		case MotionLimited:
			// lower: position - min >= 0, upper: max - position >= 0 (along -axis)
			bias, row := j.limitRow(s, position-j.LinearMin[k], useBias)
			j.linearImpulses[k][0] = solveLinearAxis(stateA, stateB, rA.Add(offset), rB, axis, bias, row, j.linearImpulses[k][0], 0, math.Inf(1))
			bias, row = j.limitRow(s, j.LinearMax[k]-position, useBias)
			j.linearImpulses[k][1] = solveLinearAxis(stateA, stateB, rA.Add(offset), rB, axis.Mul(-1), bias, row, j.linearImpulses[k][1], 0, math.Inf(1))
		}
	}
}

var unitAxes = [3]mgl64.Vec3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}

// equalityRow: the bias and the spring of an equality constraint C = 0, soft when useBias
func (j *JointBase) equalityRow(useBias bool, c float64) (float64, spring) {
	if !useBias {
		return 0, rigid
	}
	return j.spring.biasRate * c, j.spring
}

func (j *ConfigurableJoint) solveTwist(s *solver, stateA, stateB *bodyState, frameA, frameB mgl64.Quat, useBias bool) {
	if j.TwistMotion == MotionFree {
		return
	}
	twist, axis, rate := twistRow(frameA, frameB)
	j.twistAxis = axis
	if j.TwistMotion == MotionLocked {
		j.twistImpulses[0] = j.solveAngularEquality(stateA, stateB, j.twistAxis, twist/rate, 1, j.twistImpulses[0], useBias)
		return
	}
	j.twistImpulses[0] = j.solveAngularLimit(s, stateA, stateB, j.twistAxis, (twist-j.TwistMin)/rate, 1, j.twistImpulses[0], useBias)
	j.twistImpulses[1] = j.solveAngularLimit(s, stateA, stateB, j.twistAxis, (j.TwistMax-twist)/rate, -1, j.twistImpulses[1], useBias)
}

// swingAngles: rotation of the X axis of B around Y and around Z, from p (X of B seen in the frame A)
var swingAngles = [2]func(p mgl64.Vec3) float64{
	func(p mgl64.Vec3) float64 { return math.Atan2(-p.Z(), p.X()) },
	func(p mgl64.Vec3) float64 { return math.Atan2(p.Y(), p.X()) },
}

func (j *ConfigurableJoint) solveSwing(s *solver, stateA, stateB *bodyState, frameA, frameB mgl64.Quat, useBias bool) {
	p := actor.RotateInverse(&frameA, actor.Rotate(&frameB, mgl64.Vec3{1, 0, 0}))

	// both limited: elliptic cone
	if j.SwingYMotion == MotionLimited && j.SwingZMotion == MotionLimited {
		if c, axis, ok := swingLimit(p, j.SwingLimitY, j.SwingLimitZ); ok {
			j.coneAxis = actor.Rotate(&frameA, axis)
			j.coneImpulse = j.solveAngularLimit(s, stateA, stateB, j.coneAxis, c, -1, j.coneImpulse, useBias)
		}
		return
	}

	motions := [2]Motion{j.SwingYMotion, j.SwingZMotion}
	limits := [2]float64{j.SwingLimitY, j.SwingLimitZ}
	for k := 0; k < 2; k++ {
		if motions[k] == MotionFree {
			continue
		}
		angle, axis, rate, ok := angleRow(p, swingAngles[k])
		if !ok {
			continue
		}
		j.swingAxes[k] = actor.Rotate(&frameA, axis)
		// the angle changes by rate per unit of angular velocity around the axis: the constraints are in angle / rate
		if motions[k] == MotionLocked {
			j.swingImpulses[k][0] = j.solveAngularEquality(stateA, stateB, j.swingAxes[k], angle/rate, 1, j.swingImpulses[k][0], useBias)
			continue
		}
		j.swingImpulses[k][0] = j.solveAngularLimit(s, stateA, stateB, j.swingAxes[k], (angle+limits[k])/rate, 1, j.swingImpulses[k][0], useBias)
		j.swingImpulses[k][1] = j.solveAngularLimit(s, stateA, stateB, j.swingAxes[k], (limits[k]-angle)/rate, -1, j.swingImpulses[k][1], useBias)
	}
}

// solveAngularEquality: C = 0 around the axis (a locked rotation), soft when useBias
func (j *JointBase) solveAngularEquality(stateA, stateB *bodyState, axis mgl64.Vec3, c float64, direction float64, accumulated float64, useBias bool) float64 {
	bias, row := j.equalityRow(useBias, c)
	cdot := direction * axis.Dot(stateB.angularVelocity.Sub(stateA.angularVelocity))
	impulse := row.impulse(axialMass(stateA, stateB, axis), cdot, bias, accumulated)
	applyAngular(stateA, stateB, axis.Mul(direction*impulse))
	return accumulated + impulse
}

// solveAngular3: the 3 rotations together (K = IA + IB), towards the error c (rotation vector, world space)
func solveAngular3(stateA, stateB *bodyState, c mgl64.Vec3, soft spring, useBias bool, accumulated mgl64.Vec3) mgl64.Vec3 {
	cdot := stateB.angularVelocity.Sub(stateA.angularVelocity)
	bias, row := mgl64.Vec3{}, rigid
	if useBias {
		bias, row = c.Mul(soft.biasRate), soft
	}
	k := actor.Add3(&stateA.inverseInertia, &stateB.inverseInertia)
	inverse, ok := massInverse(stateA, stateB, &k)
	if !ok {
		return accumulated
	}
	impulse := row.impulse3(&inverse, cdot, bias, accumulated)
	applyAngular(stateA, stateB, impulse)
	return accumulated.Add(impulse)
}
