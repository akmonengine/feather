package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// The solver is a TGS Soft solver (sub-stepping + soft constraints + warm starting + relax).
// See ALGORITHMS.md - "Solver" section.
const (
	// LinearSlop is the length tolerance of the collision detection (m)
	LinearSlop = 0.005

	// SpeculativeDistance: contacts are created before the shapes touch, up to this distance (m),
	// plus the distance the bodies can travel during the step
	SpeculativeDistance = 4 * LinearSlop

	// DefaultContactHertz is the stiffness of the contacts between dynamic bodies.
	// Contacts with a static body are twice as stiff.
	// Higher values = less overlap under load, lower values = softer contacts
	DefaultContactHertz = 60.0

	// ContactDampingRatio of the contacts: > 1 means no oscillation
	ContactDampingRatio = 10.0

	// ContactSpeed is the maximum speed (m/s) used to push overlapping bodies apart
	ContactSpeed = 3.0

	// RestitutionThreshold: no bounce under this relative velocity (m/s)
	RestitutionThreshold = 1.0

	// MaxLinearSpeed of a body (m/s)
	MaxLinearSpeed = 400.0

	// MaxRotation of a body during one step (rad)
	MaxRotation = 0.25 * math.Pi

	// StaticFrictionSpeed: under this sliding speed (m/s), a contact point uses the static friction
	StaticFrictionSpeed = 0.01

	// the contact hertz can't exceed 1/8 of the sub-steps rate, otherwise it becomes unstable
	hertzPerSubstepRate = 0.125

	restitutionIterations = 2

	// a new contact point takes the impulses of an old point closer than this distance (m)
	contactMatchDistance = 4 * LinearSlop
)

// softness is a soft constraint (spring + damper), from its frequency and damping ratio
type softness struct {
	biasRate     float64
	massScale    float64
	impulseScale float64
}

func makeSoft(hertz, zeta, h float64) softness {
	if hertz == 0 {
		return softness{}
	}

	omega := 2 * math.Pi * hertz
	a1 := 2*zeta + h*omega
	a2 := h * omega * a1
	a3 := 1 / (1 + a2)

	return softness{biasRate: omega / a1, massScale: a2 * a3, impulseScale: a3}
}

// bodyState is the copy of a dynamic body used by the solver during a step
type bodyState struct {
	body            *actor.RigidBody
	velocity        mgl64.Vec3
	angularVelocity mgl64.Vec3
	deltaPosition   mgl64.Vec3 // since the beginning of the step
	deltaRotation   mgl64.Quat // since the beginning of the step
	invMass         float64
	inverseInertia  mgl64.Mat3 // inverse inertia in world space, at the beginning of the step
	rotation        mgl64.Quat // at the beginning of the step
}

type contactPoint struct {
	rA                 mgl64.Vec3 // from the center of mass of A
	rB                 mgl64.Vec3 // from the center of mass of B
	baseSeparation     float64
	normalMass         float64
	tangentMass        [2]float64
	normalImpulse      float64
	tangentImpulse     [2]float64
	totalNormalImpulse float64
	restitutionImpulse float64
	normalVelocity     float64 // before the solver, for the restitution
	friction           float64
}

type contactConstraint struct {
	manifold    *constraint.Manifold
	indexA      int // -1 for a static or sleeping body
	indexB      int
	normal      mgl64.Vec3
	tangents    [2]mgl64.Vec3
	restitution float64
	softness    softness
	points      [constraint.MaxContactPoints]contactPoint
	pointsCount int
}

type solver struct {
	states      []bodyState
	constraints []contactConstraint
	indices     map[*actor.RigidBody]int
	h           float64
	invH        float64

	// static bodies (and sleeping ones) share this state: no mass, they never move
	static bodyState
}

func (s *solver) state(index int) *bodyState {
	if index < 0 {
		return &s.static
	}
	return &s.states[index]
}

func (s *solver) prepare(bodies []*actor.RigidBody, manifolds []constraint.Manifold, dt float64, substeps int, contactHertz float64) {
	s.h = dt / float64(substeps)
	s.invH = 1 / s.h
	s.static = bodyState{deltaRotation: mgl64.QuatIdent()}

	// ========== 1. Body states ==========
	s.states = s.states[:0]
	if s.indices == nil {
		s.indices = make(map[*actor.RigidBody]int)
	}
	clear(s.indices)
	for _, body := range bodies {
		if !isAwakeDynamic(body) {
			continue
		}
		s.indices[body] = len(s.states)
		s.states = append(s.states, bodyState{
			body:            body,
			velocity:        body.Velocity,
			angularVelocity: body.AngularVelocity,
			deltaRotation:   mgl64.QuatIdent(),
			invMass:         body.InverseMass(),
			inverseInertia:  body.GetInverseInertiaWorld(),
			rotation:        body.Transform.Rotation,
		})
	}

	// ========== 2. Contact constraints ==========
	hertz := math.Min(contactHertz, hertzPerSubstepRate*s.invH)
	contactSoftness := makeSoft(hertz, ContactDampingRatio, s.h)
	staticSoftness := makeSoft(2*hertz, ContactDampingRatio, s.h)

	s.constraints = s.constraints[:0]
	for i := range manifolds {
		manifold := &manifolds[i]
		c := contactConstraint{
			manifold:    manifold,
			indexA:      s.indexOf(manifold.BodyA),
			indexB:      s.indexOf(manifold.BodyB),
			normal:      manifold.Normal,
			pointsCount: manifold.Count,
		}
		if c.indexA < 0 && c.indexB < 0 {
			continue
		}

		c.softness = contactSoftness
		if c.indexA < 0 || c.indexB < 0 {
			c.softness = staticSoftness
		}
		c.tangents[0], c.tangents[1] = tangentBasis(c.normal)
		c.restitution = constraint.ComputeRestitution(manifold.BodyA.Material, manifold.BodyB.Material)
		staticFriction := constraint.ComputeStaticFriction(manifold.BodyA.Material, manifold.BodyB.Material)
		dynamicFriction := constraint.ComputeDynamicFriction(manifold.BodyA.Material, manifold.BodyB.Material)

		stateA, stateB := s.state(c.indexA), s.state(c.indexB)
		for j := 0; j < manifold.Count; j++ {
			point := &manifold.Points[j]
			cp := &c.points[j]

			cp.rA = point.Position.Sub(manifold.BodyA.Transform.Position)
			cp.rB = point.Position.Sub(manifold.BodyB.Transform.Position)
			cp.baseSeparation = point.Separation - cp.rB.Sub(cp.rA).Dot(c.normal)
			cp.normalMass = effectiveMass(stateA, stateB, cp.rA, cp.rB, c.normal)
			cp.tangentMass[0] = effectiveMass(stateA, stateB, cp.rA, cp.rB, c.tangents[0])
			cp.tangentMass[1] = effectiveMass(stateA, stateB, cp.rA, cp.rB, c.tangents[1])

			// Warm starting: the impulses of the previous step
			cp.normalImpulse = point.NormalImpulse
			cp.tangentImpulse[0] = point.TangentImpulse.Dot(c.tangents[0])
			cp.tangentImpulse[1] = point.TangentImpulse.Dot(c.tangents[1])

			relativeVel := relativeVelocity(stateA, stateB, cp.rA, cp.rB)
			cp.normalVelocity = relativeVel.Dot(c.normal)
			tangentSpeed := relativeVel.Sub(c.normal.Mul(cp.normalVelocity)).Len()
			cp.friction = dynamicFriction
			if tangentSpeed < StaticFrictionSpeed {
				cp.friction = staticFriction
			}
		}
		s.constraints = append(s.constraints, c)
	}
}

func (s *solver) indexOf(body *actor.RigidBody) int {
	if index, ok := s.indices[body]; ok {
		return index
	}
	return -1
}

// effectiveMass for an impulse along the direction, applied at rA and rB
func effectiveMass(stateA, stateB *bodyState, rA, rB, direction mgl64.Vec3) float64 {
	rACrossD := rA.Cross(direction)
	rBCrossD := rB.Cross(direction)
	k := stateA.invMass + stateB.invMass + stateA.inverseInertia.Mul3x1(rACrossD).Dot(rACrossD) + stateB.inverseInertia.Mul3x1(rBCrossD).Dot(rBCrossD)
	if k <= 0 {
		return 0
	}
	return 1 / k
}

// relativeVelocity of B relative to A, at the contact point
func relativeVelocity(stateA, stateB *bodyState, rA, rB mgl64.Vec3) mgl64.Vec3 {
	vA := stateA.velocity.Add(stateA.angularVelocity.Cross(rA))
	vB := stateB.velocity.Add(stateB.angularVelocity.Cross(rB))
	return vB.Sub(vA)
}

// applyImpulse: -impulse on A, +impulse on B
func applyImpulse(stateA, stateB *bodyState, rA, rB, impulse mgl64.Vec3) {
	stateA.velocity = stateA.velocity.Sub(impulse.Mul(stateA.invMass))
	stateA.angularVelocity = stateA.angularVelocity.Sub(stateA.inverseInertia.Mul3x1(rA.Cross(impulse)))
	stateB.velocity = stateB.velocity.Add(impulse.Mul(stateB.invMass))
	stateB.angularVelocity = stateB.angularVelocity.Add(stateB.inverseInertia.Mul3x1(rB.Cross(impulse)))
}

// currentSeparation: the contact points are not computed again during the sub-steps,
// the separation is updated from the motion of both bodies
func currentSeparation(stateA, stateB *bodyState, cp *contactPoint, normal mgl64.Vec3) float64 {
	delta := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(stateB.deltaRotation.Rotate(cp.rB)).Sub(stateA.deltaRotation.Rotate(cp.rA))
	return cp.baseSeparation + delta.Dot(normal)
}

func tangentBasis(normal mgl64.Vec3) (mgl64.Vec3, mgl64.Vec3) {
	axis := mgl64.Vec3{1, 0, 0}
	if math.Abs(normal.X()) > 0.57735 {
		axis = mgl64.Vec3{0, 1, 0}
	}
	tangent1 := normal.Cross(axis).Normalize()
	return tangent1, normal.Cross(tangent1)
}

func (s *solver) integrateVelocities(gravity mgl64.Vec3) {
	h := s.h
	for i := range s.states {
		state := &s.states[i]
		body := state.body

		linearDamping := 1 / (1 + h*body.Material.LinearDamping)
		angularDamping := 1 / (1 + h*body.Material.AngularDamping)

		// ========== LINEAR ==========
		state.velocity = state.velocity.Mul(linearDamping).Add(gravity.Add(body.Force().Mul(state.invMass)).Mul(h))

		// ========== ANGULAR ==========
		angularVelocity := gyroscopic(state.angularVelocity, state.deltaRotation.Mul(state.rotation).Normalize(), body.InertiaLocal, h)
		state.angularVelocity = angularVelocity.Mul(angularDamping).Add(state.inverseInertia.Mul3x1(body.Torque()).Mul(h))
	}
}

// gyroscopic applies the gyroscopic torque -ω × Iω, implicitly (1 Newton iteration in body space).
// Without it, a spinning body does not keep its angular momentum.
func gyroscopic(angularVelocity mgl64.Vec3, rotation mgl64.Quat, inertia mgl64.Mat3, h float64) mgl64.Vec3 {
	omega := rotation.Conjugate().Rotate(angularVelocity)
	inertiaOmega := inertia.Mul3x1(omega)
	f := omega.Cross(inertiaOmega).Mul(h)
	jacobian := inertia.Add(skew(omega).Mul3(inertia).Sub(skew(inertiaOmega)).Mul(h))
	if math.Abs(jacobian.Det()) < 1e-30 {
		return angularVelocity
	}
	omega = omega.Sub(jacobian.Inv().Mul3x1(f))

	return rotation.Rotate(omega)
}

// skew returns the matrix of the cross product v × _
func skew(v mgl64.Vec3) mgl64.Mat3 {
	return mgl64.Mat3{0, v.Z(), -v.Y(), -v.Z(), 0, v.X(), v.Y(), -v.X(), 0}
}

func (s *solver) integratePositions(dt float64) {
	h := s.h
	maxAngularSpeed := MaxRotation / dt
	for i := range s.states {
		state := &s.states[i]
		if speed := state.velocity.Len(); speed > MaxLinearSpeed {
			state.velocity = state.velocity.Mul(MaxLinearSpeed / speed)
		}
		if speed := state.angularVelocity.Len(); speed > maxAngularSpeed {
			state.angularVelocity = state.angularVelocity.Mul(maxAngularSpeed / speed)
		}

		state.deltaPosition = state.deltaPosition.Add(state.velocity.Mul(h))
		state.deltaRotation = integrateRotation(state.deltaRotation, state.angularVelocity.Mul(h))
	}
}

// integrateRotation for a small rotation vector: q + 0.5 * θ * q
func integrateRotation(q mgl64.Quat, theta mgl64.Vec3) mgl64.Quat {
	qDot := mgl64.Quat{W: 0, V: theta}.Mul(q).Scale(0.5)
	return q.Add(qDot).Normalize()
}

func (s *solver) warmStart() {
	for i := range s.constraints {
		c := &s.constraints[i]
		stateA, stateB := s.state(c.indexA), s.state(c.indexB)
		for j := 0; j < c.pointsCount; j++ {
			cp := &c.points[j]
			impulse := c.normal.Mul(cp.normalImpulse).Add(c.tangents[0].Mul(cp.tangentImpulse[0])).Add(c.tangents[1].Mul(cp.tangentImpulse[1]))
			cp.totalNormalImpulse += cp.normalImpulse
			applyImpulse(stateA, stateB, cp.rA, cp.rB, impulse)
		}
	}
}

// push solves the contacts with the soft constraint, to remove the overlap. No friction here.
func (s *solver) push() {
	for i := range s.constraints {
		c := &s.constraints[i]
		stateA, stateB := s.state(c.indexA), s.state(c.indexB)
		for j := 0; j < c.pointsCount; j++ {
			cp := &c.points[j]
			separation := currentSeparation(stateA, stateB, cp, c.normal)

			var bias, massScale, impulseScale float64
			if separation > 0 {
				// speculative contact: the bodies can move closer, but not further than the gap
				bias = separation * s.invH
				massScale = 1
			} else {
				bias = math.Max(c.softness.massScale*c.softness.biasRate*separation, -ContactSpeed)
				massScale = c.softness.massScale
				impulseScale = c.softness.impulseScale
			}

			normalVel := relativeVelocity(stateA, stateB, cp.rA, cp.rB).Dot(c.normal)
			lambda := -cp.normalMass*(massScale*normalVel+bias) - impulseScale*cp.normalImpulse

			// the total impulse can't be attractive
			newImpulse := math.Max(cp.normalImpulse+lambda, 0)
			lambda = newImpulse - cp.normalImpulse
			cp.normalImpulse = newImpulse
			cp.totalNormalImpulse += lambda
			applyImpulse(stateA, stateB, cp.rA, cp.rB, c.normal.Mul(lambda))
		}
	}
}

// relax solves the contacts again without the soft constraint (it adds energy), then the friction
func (s *solver) relax() {
	for i := range s.constraints {
		c := &s.constraints[i]
		stateA, stateB := s.state(c.indexA), s.state(c.indexB)

		// ========== NORMAL ==========
		for j := 0; j < c.pointsCount; j++ {
			cp := &c.points[j]
			separation := currentSeparation(stateA, stateB, cp, c.normal)
			bias := 0.0
			if separation > 0 {
				bias = separation * s.invH
			}

			normalVel := relativeVelocity(stateA, stateB, cp.rA, cp.rB).Dot(c.normal)
			lambda := -cp.normalMass * (normalVel + bias)
			newImpulse := math.Max(cp.normalImpulse+lambda, 0)
			lambda = newImpulse - cp.normalImpulse
			cp.normalImpulse = newImpulse
			cp.totalNormalImpulse += lambda
			applyImpulse(stateA, stateB, cp.rA, cp.rB, c.normal.Mul(lambda))
		}

		// ========== FRICTION ==========
		for j := 0; j < c.pointsCount; j++ {
			cp := &c.points[j]
			relativeVel := relativeVelocity(stateA, stateB, cp.rA, cp.rB)
			previous := cp.tangentImpulse
			tangentImpulse := [2]float64{
				previous[0] - cp.tangentMass[0]*relativeVel.Dot(c.tangents[0]),
				previous[1] - cp.tangentMass[1]*relativeVel.Dot(c.tangents[1]),
			}

			// Coulomb's law: |friction| <= µ * normal impulse
			maxFriction := cp.friction * cp.normalImpulse
			if length := math.Hypot(tangentImpulse[0], tangentImpulse[1]); length > maxFriction {
				scale := 0.0
				if length > 0 {
					scale = maxFriction / length
				}
				tangentImpulse[0] *= scale
				tangentImpulse[1] *= scale
			}
			cp.tangentImpulse = tangentImpulse

			impulse := c.tangents[0].Mul(tangentImpulse[0] - previous[0]).Add(c.tangents[1].Mul(tangentImpulse[1] - previous[1]))
			applyImpulse(stateA, stateB, cp.rA, cp.rB, impulse)
		}
	}
}

// restitution is applied once, after the sub-steps. The bounce can't add energy.
func (s *solver) restitution() {
	for i := range s.constraints {
		c := &s.constraints[i]
		if c.restitution == 0 {
			continue
		}

		stateA, stateB := s.state(c.indexA), s.state(c.indexB)
		for j := 0; j < c.pointsCount; j++ {
			cp := &c.points[j]
			compressionImpulse := cp.totalNormalImpulse - cp.restitutionImpulse
			bouncing := cp.normalVelocity < -RestitutionThreshold && compressionImpulse > 0

			var bias float64
			if bouncing {
				bias = c.restitution * cp.normalVelocity
			} else if separation := currentSeparation(stateA, stateB, cp, c.normal); separation > 0 {
				bias = separation * s.invH
			}

			normalVel := relativeVelocity(stateA, stateB, cp.rA, cp.rB).Dot(c.normal)
			lambda := -cp.normalMass * (normalVel + bias)
			newImpulse := math.Max(cp.normalImpulse+lambda, 0)
			lambda = newImpulse - cp.normalImpulse

			approachImpulse := math.Min(math.Max(-cp.normalMass*normalVel, 0), math.Max(lambda, 0))
			if bouncing {
				allowance := c.restitution*(compressionImpulse+approachImpulse) - cp.restitutionImpulse
				lambda = math.Min(lambda, approachImpulse+math.Max(allowance, 0))
			}

			cp.normalImpulse += lambda
			cp.restitutionImpulse += lambda - approachImpulse
			cp.totalNormalImpulse += lambda
			applyImpulse(stateA, stateB, cp.rA, cp.rB, c.normal.Mul(lambda))
		}
	}
}

// storeImpulses in the manifolds, for the warm starting of the next step
func (s *solver) storeImpulses() {
	for i := range s.constraints {
		c := &s.constraints[i]
		for j := 0; j < c.pointsCount; j++ {
			point := &c.manifold.Points[j]
			cp := &c.points[j]
			point.NormalImpulse = cp.normalImpulse
			point.TangentImpulse = c.tangents[0].Mul(cp.tangentImpulse[0]).Add(c.tangents[1].Mul(cp.tangentImpulse[1]))
		}
	}
}

// finalize writes the new transform and velocities into the bodies
func (s *solver) finalize() {
	for i := range s.states {
		state := &s.states[i]
		body := state.body
		body.Transform.Position = body.Transform.Position.Add(state.deltaPosition)
		body.Transform.Rotation = state.deltaRotation.Mul(state.rotation).Normalize()
		body.Velocity = state.velocity
		body.AngularVelocity = state.angularVelocity
		body.ClearForces()
		body.Shape.ComputeAABB(body.Transform)
	}
}
