package feather

import (
	"errors"
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== KINEMATIC BODIES ==========
// A kinematic body goes where the game puts it: a target pose per step (RigidBody.SetKinematicTarget), reached at the
// end of the step. Its velocity is the one of this motion, (target - pose) / dt and the axis of the rotation to the
// target times its angle / dt, deduced at the start of the step: the model of PhysX (PxRigidDynamic::setKinematicTarget,
// Sc::BodySim::calculateKinematicVelocity in ScKinematics.cpp, 5.6.1; the velocity is 0 for a step without target,
// and the pose is set to the target at the end of the step, updateKinematicPose), the same velocities as
// Body::MoveKinematic of Jolt (Body.cpp, MotionProperties.inl, v5.3.0) and b2Body_SetTargetTransform of Box2D
// (body.c, v3.1.0), which keep them until the next call.
// During the sub-steps, the body goes to its target by interpolation: its position linearly, its rotation along the
// shortest arc at a constant angular velocity (a slerp from the identity: the velocity the contacts read is the velocity
// of the motion at every sub-step), and the last sub-step puts it on its target bit for bit.
//
// For the solver, a kinematic body has no inverse mass nor inverse inertia ("Static and kinematic sims have zero
// mass", Box2D body.c:535; MotionProperties.h:94-95 of Jolt): the contacts and the joints read its velocity and its
// motion, and never write it. A dynamic body it meets takes its velocity (a platform carries, a leg pushes); nothing
// pushes it back. It doesn't collide with the static and the kinematic bodies (Jolt Body.inl:36-44, Box2D
// simulation.md "A shape on a kinematic body can only collide with a dynamic body", PhysX PxRigidBody.h:56).
// Teleport (World.Teleport) puts a body somewhere without any velocity: PxRigidActor::setGlobalPose on a kinematic
// actor ("it would go right through them", the PhysX guide, Kinematic Actors).

// kinematicMotion: the velocities which bring a body from a pose to a target in dt: the translation over dt, and the
// axis of the rotation from the pose to the target times its angle over dt (the shortest way)
func kinematicMotion(from, to actor.Transform, dt float64) (mgl64.Vec3, mgl64.Vec3) {
	velocity := to.Position.Sub(from.Position).Mul(1 / dt)
	axis, angle := axisAngle(rotationDelta(from.Rotation, to.Rotation))
	return velocity, axis.Mul(angle / dt)
}

// rotationDelta: the rotation from a to b in world space, b ⊗ a⁻¹, on the shortest way (W ≥ 0)
func rotationDelta(a, b mgl64.Quat) mgl64.Quat {
	conjugate := mgl64.Quat{W: a.W, V: a.V.Mul(-1)}
	delta := actor.MulQuat(&b, &conjugate)
	if delta.W < 0 {
		delta = delta.Scale(-1)
	}
	return delta
}

// axisAngle of a unit quaternion with W ≥ 0: a unit axis and an angle in [0, π], the axis is null without rotation
func axisAngle(q mgl64.Quat) (mgl64.Vec3, float64) {
	sine := q.V.Len()
	if sine == 0 {
		return mgl64.Vec3{}, 0
	}
	return q.V.Mul(1 / sine), 2 * math.Atan2(sine, q.W)
}

// rotationFraction: the fraction t of the rotation delta (W ≥ 0), around its axis: the slerp from the identity
func rotationFraction(delta mgl64.Quat, t float64) mgl64.Quat {
	axis, angle := axisAngle(delta)
	if angle == 0 {
		return mgl64.QuatIdent()
	}
	half := 0.5 * t * angle
	return mgl64.Quat{W: math.Cos(half), V: axis.Mul(math.Sin(half))}
}

// moveKinematics, at the start of a step: the kinematic bodies take the velocity of the motion to their target. A body
// without target (or whose target is its pose) has no velocity. The velocity is read by the broad phase (the AABB
// enlarged by the distance it travels), the speculative margin of its contacts, the bodies it wakes up and the solver
func (w *World) moveKinematics(dt float64) {
	for _, body := range w.Bodies {
		if body.BodyType != actor.BodyTypeKinematic {
			continue
		}
		target, ok := body.KinematicTarget()
		if !ok {
			body.Velocity, body.AngularVelocity = mgl64.Vec3{}, mgl64.Vec3{}
			continue
		}
		body.Velocity, body.AngularVelocity = kinematicMotion(body.Transform, target, dt)
	}
}

// moveKinematic: the state i (a kinematic body) at the sub-step s.substep of s.substeps, on the way to its target:
// its position linearly, its rotation along the shortest arc. The last sub-step is the target itself
func (s *solver) moveKinematic(i int) {
	state := &s.states[i]
	target, ok := state.body.KinematicTarget()
	if !ok {
		return
	}
	start := state.body.Transform
	translation, delta := target.Position.Sub(start.Position), rotationDelta(start.Rotation, target.Rotation)
	if s.substep == s.substeps {
		state.deltaPosition, state.deltaRotation = translation, delta
	} else {
		t := float64(s.substep) / float64(s.substeps)
		state.deltaPosition, state.deltaRotation = translation.Mul(t), rotationFraction(delta, t)
	}
	state.deltaMatrix = rotationMatrix(&state.deltaRotation)
}

// isAwakeMover: an awake body which moves by itself, dynamic or kinematic. The pairs of the step, the states of the
// solver and the bodies woken by a touch follow it: a dynamic body is needed in a pair (needsSolving)
func isAwakeMover(body *actor.RigidBody) bool {
	return body.BodyType != actor.BodyTypeStatic && !body.IsSleeping
}

// isMoving: an awake body moving fast enough to keep what it touches awake. A dynamic body under the sleep speed is
// resting; a kinematic body goes exactly where it is told, there is no jitter to filter: any motion of its target moves
// it. Box2D and Jolt apply their sleep threshold to the kinematic bodies too (b2FinalizeBodiesTask, solver.c:617-651;
// MotionProperties::AccumulateSleepTime), with a velocity kept from a step to the next, where PhysX keeps a kinematic
// actor awake as long as a target is set (PxRigidDynamic.h:150-152): so does Feather
func isMoving(body *actor.RigidBody) bool {
	if body.IsSleeping {
		return false
	}
	switch body.BodyType {
	case actor.BodyTypeDynamic:
		return body.Velocity.Len() >= actor.DefaultSleepSpeed || body.AngularVelocity.Len() >= actor.DefaultSleepSpeed
	case actor.BodyTypeKinematic:
		return body.Velocity != (mgl64.Vec3{}) || body.AngularVelocity != (mgl64.Vec3{})
	}
	return false
}

// isResting: the body is still enough for its sleep timer to run (the opposite of isMoving, for an awake body)
func isResting(body *actor.RigidBody) bool {
	if body.BodyType == actor.BodyTypeKinematic {
		return body.Velocity == (mgl64.Vec3{}) && body.AngularVelocity == (mgl64.Vec3{})
	}
	return body.Velocity.Len() < actor.DefaultSleepSpeed && body.AngularVelocity.Len() < actor.DefaultSleepSpeed
}

// dynamicIndex: the state index i if it is the one of a dynamic body, else -1: for what a kinematic body must not
// enter (the graph coloring, the trees of joints), as a static body
func (s *solver) dynamicIndex(i int) int {
	if i >= 0 && s.states[i].dynamic {
		return i
	}
	return -1
}

// ErrStaticBody: SetBodyType was asked to change a body from or to static. The body is left as it is
var ErrStaticBody = errors.New("feather: SetBodyType changes a body between kinematic and dynamic, a static body stays static")

// ErrMasslessBody: SetBodyType was asked to make dynamic a body without mass. The body is left as it is
var ErrMasslessBody = errors.New("feather: a dynamic body needs a mass: create the kinematic body with a density")

// SetBodyType changes a body between kinematic and dynamic, in place: the body keeps its index in World.Bodies and
// its place in the broad phase (and so the pairs and the contacts it had, with their warm start), its joints, its
// layers and the pairs it ignores, and its island: the bodies resting on it keep resting on it. As Body::SetMotionType
// of Jolt (Body.cpp:37-77, v5.3.0), where Box2D destroys the contacts of the body (b2Body_SetType, body.c:1044).
// The body and its island wake up (Box2D wakes the body and the bodies of its joints).
//   - To dynamic: the body keeps the velocity of its last motion (a ragdoll thrown by its animation), and needs the
//     mass of its shape: a kinematic body created with a density (as PhysX: "you do need to provide a mass for the
//     kinematic actor"). Without mass, the body is refused: ErrMasslessBody.
//   - To kinematic: the body stops (no velocity until a target), its forces are dropped (Jolt), its locks are kept for
//     the day it becomes dynamic again.
//
// A static body is another kind of body (PxRigidStatic in PhysX; Jolt needs mAllowDynamicOrKinematic at the creation):
// a body is not changed from or to static, it is refused: ErrStaticBody. A refused body is left as it is. A body
// already of the type does nothing. The call allocates nothing
func (w *World) SetBodyType(body *actor.RigidBody, bodyType actor.BodyType) error {
	if body.BodyType == bodyType {
		return nil
	}
	if body.BodyType == actor.BodyTypeStatic || bodyType == actor.BodyTypeStatic {
		return ErrStaticBody
	}
	if mass := body.Material.GetMass(); bodyType == actor.BodyTypeDynamic && !(mass > 0 && !math.IsInf(mass, 1)) {
		return ErrMasslessBody
	}
	body.BodyType = bodyType
	body.ClearKinematicTarget()
	if bodyType == actor.BodyTypeKinematic {
		body.Velocity, body.AngularVelocity = mgl64.Vec3{}, mgl64.Vec3{}
		body.ClearForces()
	}
	w.islands.wake(body)
	return nil
}

// Teleport puts a body at a transform without any velocity: what it lands against is not pushed (a kinematic body
// brought there by SetKinematicTarget pushes with the velocity of its motion). The target of a kinematic body is
// dropped and it stops; a dynamic body keeps its velocity. The body wakes up with its island, and the sleeping bodies
// at the new place wake up: a body teleported into another one overlaps it, the spring of their contact pushes them
// apart at the next steps (at ContactSpeed at most). The trees see the body at its new place at the next step, or at
// SyncQueries. As PxRigidActor::setGlobalPose on a kinematic actor (PhysX: "teleport a kinematic actor to a new
// position"), Body::SetPositionAndRotation of Jolt, b2Body_SetTransform of Box2D
func (w *World) Teleport(body *actor.RigidBody, transform actor.Transform) {
	transform.Rotation = transform.Rotation.Normalize()
	body.Transform = transform
	body.ClearKinematicTarget()
	if body.BodyType == actor.BodyTypeKinematic {
		body.Velocity, body.AngularVelocity = mgl64.Vec3{}, mgl64.Vec3{}
	}
	body.UpdateAABB()
	w.islands.wake(body)
	w.wakeNeighbors(body)
}
