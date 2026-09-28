package actor

import (
	"math"

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
)

const (
	// DefaultSleepSpeed: under this linear (m/s) and angular (rad/s) speed, a body is resting
	DefaultSleepSpeed = 0.05

	// DefaultTimeToSleep: a body resting for this duration (s) falls asleep
	DefaultTimeToSleep = 0.5
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
	LinearDamping     float64 // 0.0 - 1.0, typique : 0.01
	AngularDamping    float64 // 0.0 - 1.0, typique : 0.05
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
	AngularVelocity mgl64.Vec3 // Vitesse de rotation (rad/s)
	// Inertia
	InertiaLocal        mgl64.Mat3 // Tenseur d'inertie en espace local
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

	// Collision shape
	Shape ShapeInterface // The collision shape
}

// NewRigidBody creates a new rigid body with the given properties
// density is used to calculate mass for dynamic bodies (ignored for static)
func NewRigidBody(transform Transform, shape ShapeInterface, bodyType BodyType, density float64) *RigidBody {
	transform.Rotation = transform.Rotation.Normalize()
	rb := &RigidBody{
		Transform: transform,
		Shape:     shape,
		BodyType:  bodyType,
		Velocity:  mgl64.Vec3{0, 0, 0},
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
		// Dynamic bodies compute mass from shape and density
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

	rb.InertiaLocal = shape.ComputeInertia(rb.Material.mass)
	rb.InverseInertiaLocal = rb.InertiaLocal.Inv()
	rb.Shape.ComputeAABB(rb.Transform)

	return rb
}

func (rb *RigidBody) InverseMass() float64 {
	if rb.BodyType == BodyTypeStatic {
		return 0
	}
	return 1 / rb.Material.mass
}

// TrySleep check if a body can be set to sleep.
// returns 0 if no changes, 1 if set to sleep, 2 if waken
func (rb *RigidBody) TrySleep(dt float64, timethreshold float64, velocityThreshold float64) uint8 {
	if rb.BodyType == BodyTypeStatic {
		return 0
	}
	if rb.Velocity.Len() < velocityThreshold && rb.AngularVelocity.Len() < velocityThreshold {
		rb.SleepTimer += dt // Incrémente le timer
		if !rb.IsSleeping && rb.SleepTimer >= timethreshold {
			rb.Sleep()

			return 1
		}
		return 0
	}

	wasSleeping := rb.IsSleeping
	rb.WakeUp()
	if wasSleeping {
		return 2
	}
	return 0
}

func (rb *RigidBody) Sleep() {
	rb.IsSleeping = true
	rb.SleepTimer = 0.0

	rb.Shape.ComputeAABB(rb.Transform)
	rb.ClearForces()
	rb.Velocity = mgl64.Vec3{}
	rb.AngularVelocity = mgl64.Vec3{}
}

func (rb *RigidBody) WakeUp() {
	rb.IsSleeping = false
	rb.SleepTimer = 0.0
}

// AddForce in N, during the next step
func (rb *RigidBody) AddForce(force mgl64.Vec3) {
	if rb.BodyType != BodyTypeStatic {
		rb.WakeUp()
		rb.accumulatedForce = rb.accumulatedForce.Add(force)
	}
}

// AddTorque in N·m (world space), during the next step
func (rb *RigidBody) AddTorque(torque mgl64.Vec3) {
	if rb.BodyType != BodyTypeStatic {
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
	if rb.BodyType != BodyTypeStatic {
		rb.WakeUp()
		rb.Velocity = rb.Velocity.Add(impulse.Mul(rb.InverseMass()))
	}
}

// AddImpulseAtPoint in N·s, applied at a point in world space: the body also starts to spin
func (rb *RigidBody) AddImpulseAtPoint(impulse mgl64.Vec3, point mgl64.Vec3) {
	rb.AddImpulse(impulse)
	rb.AddAngularImpulse(point.Sub(rb.Transform.Position).Cross(impulse))
}

// AddAngularImpulse in N·m·s (world space): the angular velocity changes immediately
func (rb *RigidBody) AddAngularImpulse(impulse mgl64.Vec3) {
	if rb.BodyType != BodyTypeStatic {
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

// Inverse inertia in world space: R * inertiaLocal^-1 * R^T
func (rb *RigidBody) GetInverseInertiaWorld() mgl64.Mat3 {
	if rb.BodyType == BodyTypeStatic {
		return mgl64.Mat3{}
	}
	R := rb.Transform.Rotation.Mat4().Mat3()
	return R.Mul3(rb.InverseInertiaLocal).Mul3(R.Transpose())
}
