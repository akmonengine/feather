package actor

import (
	"errors"
	"math"
	"sync/atomic"

	"github.com/go-gl/mathgl/mgl64"
)

// BodyType represents the type of rigid body
type BodyType int

const (
	// BodyTypeDynamic bodies are affected by forces, gravity, and collisions
	// They have finite mass and can move freely
	BodyTypeDynamic BodyType = iota

	// BodyTypeStatic bodies are immovable and have infinite mass
	// They are not affected by forces or gravity (e.g., ground, walls)
	BodyTypeStatic

	// BodyTypeKinematic bodies go where the game puts them (SetKinematicTarget: a pose per step), with the velocity
	// of this motion, and push the dynamic bodies they meet: infinite mass for the solver, no force, no gravity (a
	// moving platform, a character, the animated bones of a creature). They don't collide with the static and the
	// kinematic bodies. World.SetBodyType changes a body between kinematic and dynamic, World.Teleport places it
	// without velocity
	BodyTypeKinematic
)

const (
	// DefaultSleepSpeed: under this linear (m/s) and angular (rad/s) speed, a body is resting
	DefaultSleepSpeed = 0.05

	// DefaultTimeToSleep: a body resting for this duration (s) falls asleep
	DefaultTimeToSleep = 0.5
)

// Layers is a set of collision layers, one bit per layer (32 layers)
type Layers uint32

const (
	// LayerDefault: the layer of a body created by NewRigidBody
	LayerDefault Layers = 1

	// AllLayers: the mask of a body created by NewRigidBody, it collides with every layer
	AllLayers Layers = math.MaxUint32

	// NoLayers: the mask of a body which collides with nothing (see the "queries only" recipe of PHYSICS_GUIDE.md)
	NoLayers Layers = 0
)

type Material struct {
	Density     float64
	mass        float64
	Restitution float64 // 0= no rebound, 1= perfect restitution

	// StaticFriction when the surfaces stick, DynamicFriction when they slide
	StaticFriction  float64
	DynamicFriction float64
	// RollingResistance slows down the rolling spheres and capsules, usually in the range [0,1]
	RollingResistance float64
	// SpinningResistance slows down the spheres and capsules spinning on their contact (a top), without unit: the
	// contact holds a torque of SpinningResistance * radius * its normal force around its normal. 0: it spins forever
	SpinningResistance float64
	LinearDamping      float64 // 0.0 - 1.0, typical: 0.01
	AngularDamping     float64 // 0.0 - 1.0, typical: 0.05
}

func (material Material) GetMass() float64 {
	return material.mass
}

// RigidBody represents a rigid body in the physics simulation
type RigidBody struct {
	// Useful to map to user data (e.g. entity id)
	Id any

	// Spatial properties
	Transform Transform

	// Linear motion
	Velocity mgl64.Vec3 // Linear velocity (m/s)

	// Angular motion
	AngularVelocity mgl64.Vec3 // Angular velocity (rad/s)
	// Inertia
	InertiaLocal        mgl64.Mat3 // Inertia tensor, in the local space
	InverseInertiaLocal mgl64.Mat3

	// Force (N) & torque (N·m) applied during the next step
	accumulatedForce  mgl64.Vec3
	accumulatedTorque mgl64.Vec3

	IsTrigger  bool
	IsSleeping bool
	SleepTimer float64

	// Physical properties
	Material Material
	BodyType BodyType // Dynamic or Static

	// Layer of the body (one bit), and Mask: the layers it collides with. 2 bodies collide if each one has its layer
	// in the mask of the other
	Layer Layers
	Mask  Layers

	// LinearLock: the world axes a dynamic body can't move along. AngularLock: the world axes it can't turn around.
	// Along a locked axis the body has no inverse mass (or inverse inertia): neither the gravity, the forces, the
	// impulses, the contacts nor the joints move it. Set them before the first step, then by SetLocks
	LinearLock  Axes
	AngularLock Axes

	// Collision shape
	Shape ShapeInterface // The collision shape
	aabb  AABB
	// serial: a unique number, given by NewRigidBody
	serial uint64

	// target: the pose a kinematic body reaches at the end of the next step (SetKinematicTarget), if targeted
	target   Transform
	targeted bool
}

// serials of the bodies created by NewRigidBody
var serials atomic.Uint64

// NewRigidBody creates a new rigid body with the given properties
// density is used to calculate mass for dynamic and kinematic bodies (ignored for static): a kinematic body keeps the
// mass of its shape for the day it becomes dynamic (World.SetBodyType), the solver never reads it
func NewRigidBody(transform Transform, shape ShapeInterface, bodyType BodyType, density float64) *RigidBody {
	transform.Rotation = transform.Rotation.Normalize()
	rb := &RigidBody{
		serial:    serials.Add(1),
		Transform: transform,
		Shape:     shape,
		BodyType:  bodyType,
		Velocity:  mgl64.Vec3{0, 0, 0},
		Layer:     LayerDefault,
		Mask:      AllLayers,
	}

	// Calculate mass data based on body type
	if bodyType == BodyTypeStatic {
		// Static bodies have infinite mass
		rb.Material = Material{
			Density:         0,
			mass:            math.Inf(1),
			StaticFriction:  0.0,
			DynamicFriction: 0.0,
		}
	} else {
		// Dynamic and kinematic bodies compute mass from shape and density
		rb.Material = Material{
			Density:         density,
			mass:            shape.ComputeMass(density),
			Restitution:     0.0,
			StaticFriction:  0.0,
			DynamicFriction: 0.0,
			LinearDamping:   0.0,
			AngularDamping:  0.0,
		}
	}

	// a static body has no inertia (its inverse inertia is 0, it never turns); a kinematic body keeps the one of its
	// shape for a switch to dynamic (GetInverseInertiaWorld gives 0 while it is kinematic)
	if bodyType != BodyTypeStatic {
		rb.InertiaLocal = shape.ComputeInertia(rb.Material.mass)
		rb.InverseInertiaLocal = rb.InertiaLocal.Inv()
	}
	rb.UpdateAABB()

	return rb
}

// Serial is a unique number of the body, given by NewRigidBody
func (rb *RigidBody) Serial() uint64 {
	return rb.serial
}

// AABB of the body, at its transform
func (rb *RigidBody) AABB() AABB {
	return rb.aabb
}

// UpdateAABB after a change of the transform (the World updates it after each step)
func (rb *RigidBody) UpdateAABB() {
	rb.aabb = rb.Shape.ComputeAABB(rb.Transform)
}

// InverseMass: 1 / mass for a dynamic body, 0 for a static or a kinematic body (nothing moves them)
func (rb *RigidBody) InverseMass() float64 {
	if rb.BodyType != BodyTypeDynamic {
		return 0
	}
	return 1 / rb.Material.mass
}

// InverseMassAxes: the inverse mass along each world axis, null along a locked axis
func (rb *RigidBody) InverseMassAxes() mgl64.Vec3 {
	inverseMass := rb.InverseMass()
	return rb.LinearLock.LockVector(mgl64.Vec3{inverseMass, inverseMass, inverseMass})
}

// SetLocks changes the locked axes of a dynamic body: its velocities along the locked axes are cleared, and it wakes
// up (a body freed in the air must fall). A static body never moves: it has no lock. A kinematic body goes where its
// target is: it keeps the locks for the day it becomes dynamic (World.SetBodyType)
func (rb *RigidBody) SetLocks(linear, angular Axes) {
	linear, angular = linear&AllAxes, angular&AllAxes
	if rb.BodyType == BodyTypeStatic || (linear == rb.LinearLock && angular == rb.AngularLock) {
		return
	}
	rb.LinearLock, rb.AngularLock = linear, angular
	if rb.BodyType != BodyTypeDynamic {
		return
	}
	rb.Velocity = linear.LockVector(rb.Velocity)
	rb.AngularVelocity = angular.LockVector(rb.AngularVelocity)
	rb.WakeUp()
}

func (rb *RigidBody) Sleep() {
	rb.IsSleeping = true
	rb.SleepTimer = 0.0

	rb.UpdateAABB()
	rb.ClearForces()
	rb.Velocity = mgl64.Vec3{}
	rb.AngularVelocity = mgl64.Vec3{}
}

func (rb *RigidBody) WakeUp() {
	rb.IsSleeping = false
	rb.SleepTimer = 0.0
}

// AddForce in N, during the next step. A static or a kinematic body ignores the forces, the torques and the impulses
// (PhysX refuses them on a kinematic actor, Jolt drops them: "Cancel forces" in Body::SetMotionType)
func (rb *RigidBody) AddForce(force mgl64.Vec3) {
	if rb.BodyType == BodyTypeDynamic {
		rb.WakeUp()
		rb.accumulatedForce = rb.accumulatedForce.Add(force)
	}
}

// AddTorque in N·m (world space), during the next step
func (rb *RigidBody) AddTorque(torque mgl64.Vec3) {
	if rb.BodyType == BodyTypeDynamic {
		rb.WakeUp()
		rb.accumulatedTorque = rb.accumulatedTorque.Add(torque)
	}
}

// AddForceAtPoint in N, applied at a point in world space: it also adds the torque (point - center) × force
func (rb *RigidBody) AddForceAtPoint(force mgl64.Vec3, point mgl64.Vec3) {
	rb.AddForce(force)
	rb.AddTorque(point.Sub(rb.Transform.Position).Cross(force))
}

// AddImpulse in N·s: the velocity changes immediately (a hit, a jump)
func (rb *RigidBody) AddImpulse(impulse mgl64.Vec3) {
	if rb.BodyType == BodyTypeDynamic {
		rb.WakeUp()
		rb.Velocity = rb.LinearLock.LockVector(rb.Velocity.Add(impulse.Mul(rb.InverseMass())))
	}
}

// AddImpulseAtPoint in N·s, applied at a point in world space: the body also starts to spin
func (rb *RigidBody) AddImpulseAtPoint(impulse mgl64.Vec3, point mgl64.Vec3) {
	rb.AddImpulse(impulse)
	rb.AddAngularImpulse(point.Sub(rb.Transform.Position).Cross(impulse))
}

// AddAngularImpulse in N·m·s (world space): the angular velocity changes immediately
func (rb *RigidBody) AddAngularImpulse(impulse mgl64.Vec3) {
	if rb.BodyType == BodyTypeDynamic {
		rb.WakeUp()
		rb.AngularVelocity = rb.AngularVelocity.Add(rb.GetInverseInertiaWorld().Mul3x1(impulse))
	}
}

func (rb *RigidBody) Force() mgl64.Vec3 { return rb.accumulatedForce }

func (rb *RigidBody) Torque() mgl64.Vec3 { return rb.accumulatedTorque }

func (rb *RigidBody) ClearForces() {
	rb.accumulatedForce = mgl64.Vec3{0, 0, 0}
	rb.accumulatedTorque = mgl64.Vec3{0, 0, 0}
}

func (rb *RigidBody) SupportWorld(direction mgl64.Vec3) mgl64.Vec3 {
	localDirection := rb.Transform.Rotation.Conjugate().Rotate(direction)
	localSupport := rb.Shape.Support(localDirection)
	return rb.Transform.Position.Add(rb.Transform.Rotation.Rotate(localSupport))
}

// Inertia in world space: R * inertiaLocal * R^T
func (rb *RigidBody) GetInertiaWorld() mgl64.Mat3 {
	R := rb.Transform.Rotation.Mat4().Mat3()
	return R.Mul3(rb.InertiaLocal).Mul3(R.Transpose())
}

// Inverse inertia in world space: R * inertiaLocal^-1 * R^T, of the body held around its locked axes (the inverse of
// the free block of its inertia, see Axes.LockInverseInertia). The locks are in world space, the matrix is locked again
// each time the body turns
func (rb *RigidBody) GetInverseInertiaWorld() mgl64.Mat3 {
	inverseInertia := rb.GetFreeInverseInertiaWorld()
	if rb.AngularLock != NoAxes {
		rb.AngularLock.LockInverseInertia(&inverseInertia)
	}
	return inverseInertia
}

// GetFreeInverseInertiaWorld: the inverse inertia in world space of the body without its locks, 0 for a static or a
// kinematic body
func (rb *RigidBody) GetFreeInverseInertiaWorld() mgl64.Mat3 {
	if rb.BodyType != BodyTypeDynamic {
		return mgl64.Mat3{}
	}
	// the rotation matrix of mgl64 (Quat.Mat4), and R I⁻¹ Rᵀ without copying the matrices: the same arithmetic
	q := rb.Transform.Rotation
	w, x, y, z := q.W, q.V[0], q.V[1], q.V[2]
	r := mgl64.Mat3{
		1 - 2*y*y - 2*z*z, 2*x*y + 2*w*z, 2*x*z - 2*w*y,
		2*x*y - 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z + 2*w*x,
		2*x*z + 2*w*y, 2*y*z - 2*w*x, 1 - 2*x*x - 2*y*y,
	}
	ri := Mul3(&r, &rb.InverseInertiaLocal)
	rt := Transpose3(&r)
	return Mul3(&ri, &rt)
}

// ErrNotKinematic: SetKinematicTarget was called on a body which is not kinematic. The target is not stored
var ErrNotKinematic = errors.New("actor: SetKinematicTarget on a body which is not kinematic")

// SetKinematicTarget: the pose a kinematic body reaches at the end of the next step. The step brings it there over its
// sub-steps (its position linearly, its rotation along the shortest arc), with the velocity of this motion, and drops
// the target: give one per step to move the body along a path, as PxRigidDynamic::setKinematicTarget of PhysX. Without
// target the body stays, with no velocity. A target away from the pose of the body wakes it up, with its island at the
// step. On a body which is not kinematic the target is refused: ErrNotKinematic, the only error (PhysX refuses it too,
// "Body must be kinematic"); the call allocates nothing. To place the body without pushing anything, see World.Teleport
func (rb *RigidBody) SetKinematicTarget(target Transform) error {
	if rb.BodyType != BodyTypeKinematic {
		return ErrNotKinematic
	}
	target.Rotation = target.Rotation.Normalize()
	rb.target, rb.targeted = target, true
	if target != rb.Transform {
		rb.WakeUp()
	}
	return nil
}

// KinematicTarget: the target of the next step, if the body has one (the step drops it once reached)
func (rb *RigidBody) KinematicTarget() (Transform, bool) {
	return rb.target, rb.targeted
}

// ClearKinematicTarget: the body has no target anymore, it stays where it is at the next step. The World calls it when
// the body reaches its target, and when the body is teleported or changes type
func (rb *RigidBody) ClearKinematicTarget() {
	rb.target, rb.targeted = Transform{}, false
}
