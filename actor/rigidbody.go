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

type Material struct {
	Density     float64
	mass        float64
	Restitution float64 // 0= no rebound, 1= perfect restitution

	StaticFriction  float64
	DynamicFriction float64
	LinearDamping   float64 // 0.0 - 1.0, typique : 0.01
	AngularDamping  float64 // 0.0 - 1.0, typique : 0.05
}

func (material Material) GetMass() float64 {
	return material.mass
}

// RigidBody represents a rigid body in the physics simulation
type RigidBody struct {
	// Spatial properties
	PreviousTransform Transform
	Transform         Transform

	// Linear motion
	PresolveVelocity mgl64.Vec3
	Velocity         mgl64.Vec3 // Linear velocity (m/s)

	// Angular motion (NOUVEAU)
	PresolveAngularVelocity mgl64.Vec3
	AngularVelocity         mgl64.Vec3 // Vitesse de rotation (rad/s)
	// Inertia (NOUVEAU)
	InertiaLocal        mgl64.Mat3 // Tenseur d'inertie en espace local
	InverseInertiaLocal mgl64.Mat3

	accumulatedForce  mgl64.Vec3
	accumulatedTorque mgl64.Vec3

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
	rb := &RigidBody{
		PreviousTransform: transform,
		Transform:         transform,
		Shape:             shape,
		BodyType:          bodyType,
		Velocity:          mgl64.Vec3{0, 0, 0},
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

func (rb *RigidBody) TrySleep(dt float64, timethreshold float64, velocityThreshold float64) {
	if rb.Velocity.Len() < velocityThreshold && rb.AngularVelocity.Len() < velocityThreshold {
		rb.SleepTimer += dt // Incrémente le timer
		if rb.SleepTimer >= timethreshold {
			rb.Sleep()
		}
	} else {
		rb.Awake()
	}
}

func (rb *RigidBody) Sleep() {
	rb.IsSleeping = true
	rb.SleepTimer = 0.0

	rb.Shape.ComputeAABB(rb.Transform)
	rb.ClearForces()
	rb.Velocity = mgl64.Vec3{}
	rb.AngularVelocity = mgl64.Vec3{}
}

func (rb *RigidBody) Awake() {
	rb.IsSleeping = false
	rb.SleepTimer = 0.0
}

func (rb *RigidBody) Integrate(dt float64, gravity mgl64.Vec3) {
	if rb.BodyType == BodyTypeStatic || rb.IsSleeping {
		return
	}

	// Stockage état précédent
	rb.PreviousTransform.Position = rb.Transform.Position
	rb.PreviousTransform.Rotation = rb.Transform.Rotation

	// ========== INTÉGRATION LINÉAIRE ==========
	forces := gravity.Mul(rb.Material.mass).Mul(dt * (1.0 / rb.Material.GetMass()))
	forces = forces.Add(rb.accumulatedForce.Mul(1.0 / rb.Material.GetMass()))
	rb.Velocity = rb.Velocity.Add(forces)

	// ========== LINEAR DAMPING ==========
	rb.Velocity = rb.Velocity.Mul(math.Exp(-rb.Material.LinearDamping * dt))
	rb.Transform.Position = rb.Transform.Position.Add(rb.Velocity.Mul(dt))

	// ========== INTÉGRATION ANGULAIRE ==========
	I_inv := rb.GetInverseInertiaWorld()
	torques := rb.accumulatedTorque.Mul(1.0 / dt)
	angularAccel := I_inv.Mul3x1(torques)
	rb.AngularVelocity = rb.AngularVelocity.Add(angularAccel.Mul(dt))

	// ========== ANGULAR DAMPING ==========
	rb.AngularVelocity = rb.AngularVelocity.Mul(math.Exp(-rb.Material.AngularDamping * dt))

	// ========== UPDATE QUATERNION ==========
	omegaQuat := mgl64.Quat{V: rb.AngularVelocity, W: 0}
	q_dot := omegaQuat.Mul(rb.Transform.Rotation).Scale(0.5)
	rb.Transform.Rotation = rb.Transform.Rotation.Add(q_dot.Scale(dt)).Normalize()
	rb.Transform.InverseRotation = rb.Transform.Rotation.Inverse()

	rb.PresolveVelocity = rb.Velocity
	rb.PresolveAngularVelocity = rb.AngularVelocity

	rb.Shape.ComputeAABB(rb.Transform)
	rb.ClearForces()
}

func (rb *RigidBody) Update(dt float64) {
	if rb.BodyType == BodyTypeStatic || rb.IsSleeping {
		return
	}

	// Commit predicted position to actual position
	rb.Velocity = rb.Transform.Position.Sub(rb.PreviousTransform.Position).Mul(1.0 / dt)
	qDelta := rb.Transform.Rotation.Mul(rb.PreviousTransform.Rotation.Conjugate())
	qDelta = qDelta.Normalize()
	if qDelta.W >= 0.0 {
		rb.AngularVelocity = qDelta.V.Mul(2.0 / dt)
	} else {
		rb.AngularVelocity = qDelta.V.Mul(-2.0 / dt)
	}
}

// AddForce in 1000N (1000 * kg⋅m/s²)
func (rb *RigidBody) AddForce(force mgl64.Vec3) {
	if rb.BodyType != BodyTypeStatic {
		rb.Awake()

		rb.accumulatedForce = rb.accumulatedForce.Add(force.Mul(1000))
	}
}

// AddTorque in 1000N⋅m
func (rb *RigidBody) AddTorque(torque mgl64.Vec3) {
	if rb.BodyType != BodyTypeStatic {
		rb.Awake()

		rb.accumulatedTorque = rb.accumulatedTorque.Add(torque.Mul(1000))
	}
}

// Méthodes optionnelles pour reset
func (rb *RigidBody) ClearForces() {
	rb.accumulatedForce = mgl64.Vec3{0, 0, 0}
	rb.accumulatedTorque = mgl64.Vec3{0, 0, 0}
}

func (rb *RigidBody) SupportWorld(direction mgl64.Vec3) mgl64.Vec3 {
	// 1. Transformer la direction en espace local (rotation inverse)
	localDirection := rb.Transform.InverseRotation.Rotate(direction)

	// 2. Trouver le support en espace local
	localSupport := rb.Shape.Support(localDirection)

	// 3. Transformer le point support en espace monde (rotation + translation)
	worldSupport := rb.Transform.Rotation.Rotate(localSupport)
	return rb.Transform.Position.Add(worldSupport)
}

// Inertie en espace monde
func (rb *RigidBody) GetInertiaWorld() mgl64.Mat3 {
	// I_world = R * I_local * R^T
	R := rb.Transform.Rotation.Mat4().Mat3()
	return R.Mul3(rb.InertiaLocal).Mul3(R.Transpose())
}

// Inverse de l'inertie en espace monde
func (rb *RigidBody) GetInverseInertiaWorld() mgl64.Mat3 {
	if rb.BodyType == BodyTypeStatic {
		return mgl64.Mat3{0, 0, 0, 0, 0, 0, 0, 0, 0}
	}

	// I_world^(-1) = R * I_local^(-1) * R^T
	R := rb.Transform.Rotation.Mat4().Mat3()
	return R.Mul3(rb.InverseInertiaLocal).Mul3(R.Transpose())
}
