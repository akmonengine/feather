package constraint

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

const (
	// DefaultCompliance controls soft constraint stiffness for contact resolution.
	// Lower values = stiffer contacts (less penetration, potential jitter)
	// Higher values = softer contacts (more penetration, smoother)
	// Typical range: 1e-10 (very stiff) to 1e-6 (soft)
	// See PHYSICS_GUIDE.md for tuning guidelines.
	DefaultCompliance = 1e-7
)

type ContactPoint struct {
	Position    mgl64.Vec3
	Penetration float64
}

type ContactConstraint struct {
	BodyA  *actor.RigidBody
	BodyB  *actor.RigidBody
	Points []ContactPoint
	Normal mgl64.Vec3
}

// SolvePosition resolves penetration (PBD style, no lambda accumulation)
func (c *ContactConstraint) SolvePosition(dt float64) {
	if len(c.Points) == 0 {
		return
	}
	if c.BodyA.IsSleeping && c.BodyB.IsSleeping {
		return
	}

	bodyA := c.BodyA
	bodyB := c.BodyB

	bodyA.Mutex.Lock()
	bodyB.Mutex.Lock()
	defer bodyA.Mutex.Unlock()
	defer bodyB.Mutex.Unlock()

	// ========== 1. Calculate total effective weight ==========
	invMassA := 1.0 / bodyA.Material.GetMass()
	invMassB := 1.0 / bodyB.Material.GetMass()
	IA_inv := bodyA.GetInverseInertiaWorld()
	IB_inv := bodyB.GetInverseInertiaWorld()

	var totalWeight float64
	var totalPenetration float64

	for _, point := range c.Points {
		penetration := point.Penetration
		if penetration <= 1e-8 {
			continue
		}

		rA := point.Position.Sub(bodyA.Transform.Position)
		rB := point.Position.Sub(bodyB.Transform.Position)

		// Calculate effective inertia for this point
		rA_cross_n := rA.Cross(c.Normal)
		rB_cross_n := rB.Cross(c.Normal)

		angularInertiaA := IA_inv.Mul3x1(rA_cross_n).Dot(rA_cross_n)
		angularInertiaB := IB_inv.Mul3x1(rB_cross_n).Dot(rB_cross_n)

		wA := invMassA + angularInertiaA
		wB := invMassB + angularInertiaB
		totalWeight += wA + wB

		totalPenetration += penetration
	}

	// ========== 2. Calculate deltaLambda (global correction) ==========
	if totalWeight <= 1e-8 {
		return
	}

	compliance := DefaultCompliance
	alphaTilde := compliance / (dt * dt)
	deltaLambda := -totalPenetration / (totalWeight + alphaTilde)

	// ========== 3. Apply linear corrections ==========
	totalImpulse := c.Normal.Mul(deltaLambda)

	if bodyA.BodyType != actor.BodyTypeStatic {
		bodyA.Transform.Position = bodyA.Transform.Position.Add(totalImpulse.Mul(invMassA))
	}
	if bodyB.BodyType != actor.BodyTypeStatic {
		bodyB.Transform.Position = bodyB.Transform.Position.Sub(totalImpulse.Mul(invMassB))
	}

	// ========== 4. Apply angular corrections ==========
	// Accumulate torques from all points, then apply ONE SINGLE correction
	var totalTorqueA, totalTorqueB mgl64.Vec3

	for _, point := range c.Points {
		if point.Penetration <= 1e-8 {
			continue
		}

		rA := point.Position.Sub(bodyA.Transform.Position)
		rB := point.Position.Sub(bodyB.Transform.Position)

		// Accumulate angular moments
		// Body A receives +totalImpulse → torque_A = rA × (+totalImpulse)
		// Body B receives -totalImpulse → torque_B = rB × (-totalImpulse)
		totalTorqueA = totalTorqueA.Add(rA.Cross(totalImpulse))
		totalTorqueB = totalTorqueB.Add(rB.Cross(totalImpulse.Mul(-1)))
	}

	// Calculate total angular correction
	// In XPBD: Δθ = I_inv * (Σ torque)
	deltaRotA := IA_inv.Mul3x1(totalTorqueA)
	deltaRotB := IB_inv.Mul3x1(totalTorqueB)

	// Apply ONE SINGLE rotation correction via quaternions
	// For a small angle δθ, the rotation quaternion is q_delta ≈ [1, δθ/2]
	if bodyA.BodyType != actor.BodyTypeStatic && deltaRotA.Len() > 1e-10 {
		qDelta := mgl64.Quat{W: 1.0, V: deltaRotA.Mul(0.5)}
		qDelta = qDelta.Normalize()
		bodyA.Transform.Rotation = qDelta.Mul(bodyA.Transform.Rotation).Normalize()
		bodyA.Transform.InverseRotation = bodyA.Transform.Rotation.Inverse()
	}

	if bodyB.BodyType != actor.BodyTypeStatic && deltaRotB.Len() > 1e-10 {
		qDelta := mgl64.Quat{W: 1.0, V: deltaRotB.Mul(0.5)}
		qDelta = qDelta.Normalize()
		bodyB.Transform.Rotation = qDelta.Mul(bodyB.Transform.Rotation).Normalize()
		bodyB.Transform.InverseRotation = bodyB.Transform.Rotation.Inverse()
	}
}

// SolveVelocity applies restitution
func (c *ContactConstraint) SolveVelocity(dt float64) {
	if len(c.Points) == 0 {
		return
	}
	if c.BodyA.IsSleeping && c.BodyB.IsSleeping {
		return
	}

	bodyA := c.BodyA
	bodyB := c.BodyB

	bodyA.Mutex.Lock()
	bodyB.Mutex.Lock()
	defer bodyA.Mutex.Unlock()
	defer bodyB.Mutex.Unlock()

	invMassA := 1.0 / bodyA.Material.GetMass()
	invMassB := 1.0 / bodyB.Material.GetMass()
	IA_inv := bodyA.GetInverseInertiaWorld()
	IB_inv := bodyB.GetInverseInertiaWorld()

	restitution := ComputeRestitution(bodyA.Material, bodyB.Material)
	staticFriction := ComputeStaticFriction(bodyA.Material, bodyB.Material)
	dynamicFriction := ComputeDynamicFriction(bodyA.Material, bodyB.Material)

	// ========== ACCUMULATE all impulses ==========
	var totalLinearImpulseA mgl64.Vec3
	var totalLinearImpulseB mgl64.Vec3
	var totalAngularImpulseA mgl64.Vec3
	var totalAngularImpulseB mgl64.Vec3

	for _, point := range c.Points {
		rA := point.Position.Sub(bodyA.Transform.Position)
		rB := point.Position.Sub(bodyB.Transform.Position)

		// ========== Velocities ==========
		vA := bodyA.Velocity.Add(bodyA.AngularVelocity.Cross(rA))
		vB := bodyB.Velocity.Add(bodyB.AngularVelocity.Cross(rB))
		relativeVel := vB.Sub(vA)
		normalVel := relativeVel.Dot(c.Normal)

		// ========== Pre-resolution velocity ==========
		vA_prev := bodyA.PresolveVelocity.Add(bodyA.PresolveAngularVelocity.Cross(rA))
		vB_prev := bodyB.PresolveVelocity.Add(bodyB.PresolveAngularVelocity.Cross(rB))
		relativeVelPrev := vB_prev.Sub(vA_prev)
		normalVelPrev := relativeVelPrev.Dot(c.Normal)

		// ========== NORMAL IMPULSE (restitution) ==========
		rA_cross_n := rA.Cross(c.Normal)
		rB_cross_n := rB.Cross(c.Normal)

		angularInertiaA := IA_inv.Mul3x1(rA_cross_n).Dot(rA_cross_n)
		angularInertiaB := IB_inv.Mul3x1(rB_cross_n).Dot(rB_cross_n)

		effectiveMassNormal := invMassA + invMassB + angularInertiaA + angularInertiaB

		if effectiveMassNormal < 1e-10 {
			continue
		}

		// ========== Impulse for this point ==========
		targetVel := -restitution * normalVelPrev
		deltaV := targetVel - normalVel
		lambdaNormal := deltaV / effectiveMassNormal

		// ========== CRITICAL: Prevent attractive impulses ==========
		if lambdaNormal < 0 {
			lambdaNormal = 0
		}

		normalImpulse := c.Normal.Mul(lambdaNormal)

		// Accumulate normal impulse
		totalLinearImpulseA = totalLinearImpulseA.Sub(normalImpulse.Mul(invMassA))
		totalLinearImpulseB = totalLinearImpulseB.Add(normalImpulse.Mul(invMassB))

		torqueA := rA.Cross(normalImpulse.Mul(-1))
		torqueB := rB.Cross(normalImpulse)

		totalAngularImpulseA = totalAngularImpulseA.Add(IA_inv.Mul3x1(torqueA))
		totalAngularImpulseB = totalAngularImpulseB.Add(IB_inv.Mul3x1(torqueB))

		// ========== TANGENTIAL IMPULSE (friction) ==========
		// Only if there is a normal force
		if lambdaNormal > 0 {
			// Tangential velocity (component perpendicular to normal)
			tangentVel := relativeVel.Sub(c.Normal.Mul(normalVel))
			tangentSpeed := tangentVel.Len()

			if tangentSpeed > 1e-6 {
				// Tangential direction
				tangentDir := tangentVel.Mul(1.0 / tangentSpeed)

				// Effective mass in tangential direction
				rA_cross_t := rA.Cross(tangentDir)
				rB_cross_t := rB.Cross(tangentDir)
				angularInertiaA_t := IA_inv.Mul3x1(rA_cross_t).Dot(rA_cross_t)
				angularInertiaB_t := IB_inv.Mul3x1(rB_cross_t).Dot(rB_cross_t)

				effectiveMassTangent := invMassA + invMassB + angularInertiaA_t + angularInertiaB_t

				if effectiveMassTangent < 1e-10 {
					continue
				}

				// Impulse to cancel tangential velocity
				lambdaTangent := -tangentSpeed / effectiveMassTangent

				// Coulomb's law: |F_friction| ≤ μ * |F_normal|
				maxStaticFriction := staticFriction * math.Abs(lambdaNormal)

				var frictionImpulse mgl64.Vec3

				if math.Abs(lambdaTangent) <= maxStaticFriction {
					// Static friction: completely cancels tangential velocity
					frictionImpulse = tangentDir.Mul(lambdaTangent)
				} else {
					// Dynamic friction: limited by μ_dynamic
					maxDynamicFriction := dynamicFriction * math.Abs(lambdaNormal)
					frictionImpulse = tangentDir.Mul(-math.Copysign(maxDynamicFriction, tangentSpeed))
				}

				// Accumulate friction impulse
				totalLinearImpulseA = totalLinearImpulseA.Sub(frictionImpulse.Mul(invMassA))
				totalLinearImpulseB = totalLinearImpulseB.Add(frictionImpulse.Mul(invMassB))

				torqueA_friction := rA.Cross(frictionImpulse.Mul(-1))
				torqueB_friction := rB.Cross(frictionImpulse)

				totalAngularImpulseA = totalAngularImpulseA.Add(IA_inv.Mul3x1(torqueA_friction))
				totalAngularImpulseB = totalAngularImpulseB.Add(IB_inv.Mul3x1(torqueB_friction))
			}
		}
	}

	// ========== APPLY all impulses ==========
	bodyA.Velocity = bodyA.Velocity.Add(totalLinearImpulseA)
	bodyB.Velocity = bodyB.Velocity.Add(totalLinearImpulseB)
	bodyA.AngularVelocity = bodyA.AngularVelocity.Add(totalAngularImpulseA)
	bodyB.AngularVelocity = bodyB.AngularVelocity.Add(totalAngularImpulseB)

	clampSmallVelocities(bodyA)
	clampSmallVelocities(bodyB)
}
