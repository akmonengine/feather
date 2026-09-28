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

	// DefaultContactHertz is the stiffness of the contacts between dynamic bodies, as in Box2D v3.1.
	// Contacts with a static body are twice as stiff, with half the damping ratio (as Box3D).
	// Higher values = less overlap under load, lower values = softer contacts
	DefaultContactHertz = 30.0

	// ContactDampingRatio of the contacts: > 1 means no oscillation
	ContactDampingRatio = 10.0

	// ContactSpeed is the maximum speed (m/s) used to push overlapping bodies apart
	ContactSpeed = 3.0

	// RestitutionThreshold: no bounce under this relative velocity (m/s)
	RestitutionThreshold = 1.0

	// MaxLinearSpeed of a body (m/s)
	MaxLinearSpeed = 400.0

	// MaxRotation of a body during one substep (rad). Box2D limits it per step: Feather lets the bodies turn faster
	// (a wheel, a ball), the contacts follow the rotation during the step (turnAnchors)
	MaxRotation = 0.25 * math.Pi

	// StaticFrictionSpeed: under this sliding speed (m/s), a contact point uses the static friction
	StaticFrictionSpeed = 0.01

	// turnAnchorsCos: the anchors of a body turn if it turned more than 0.01 rad since the beginning of the step,
	// cos(0.01 / 2). Under it, the error of a lever arm of 50 cm is 0.5 mm
	turnAnchorsCos = 0.99998750002604166

	// minFrictionWeight: the weight of a point far from touching in the friction center (Box3D)
	minFrictionWeight = 1e-10

	// minFrictionDeterminant: under it, the tangents can't turn the bodies apart (no mass): no friction
	minFrictionDeterminant = 1e-30

	// the contact hertz can't exceed 1/8 of the sub-steps rate, otherwise it becomes unstable
	hertzPerSubstepRate = 0.125

	// a new contact point takes the impulses of an old point closer than this distance (m)
	contactMatchDistance = 4 * LinearSlop
)

// spring is a soft constraint (Erin Catto, "Soft Constraints", GDC 2011): the error of the constraint is a spring
// of frequency ω and damping ratio ζ, whatever the mass (k = m ω², c = 2 m ζ ω), integrated implicitly over h:
//
//	biasRate = k / (c + h k) = ω / (2ζ + h ω)                     // the part of the error removed per second
//	gamma    = m / (h (c + h k)) = 1 / (h ω (2ζ + h ω))            // the softness γ of the paper, times the mass
//
// A row of effective mass m, relative velocity v and bias b (biasRate * error) gets the impulse
//
//	λ = -(m (v + b) + gamma * accumulated) / (1 + gamma)
//
// A rigid row has gamma = 0: λ = -m (v + b)
type spring struct {
	biasRate float64
	gamma    float64
}

// rigid: a constraint without softness
var rigid = spring{}

// newSpring for a frequency and a damping ratio, over the substep h. A spring of 0 hertz doesn't exist (it would be
// infinitely soft): the callers check the frequency first
func newSpring(hertz, dampingRatio, h float64) spring {
	if hertz <= 0 {
		panic("feather: a spring needs a frequency > 0")
	}
	omega := 2 * math.Pi * hertz
	return spring{biasRate: omega / (2*dampingRatio + h*omega), gamma: 1 / (h * omega * (2*dampingRatio + h*omega))}
}

// impulse of a row of effective mass m
func (s spring) impulse(mass, velocity, bias, accumulated float64) float64 {
	return -(mass*(velocity+bias) + s.gamma*accumulated) / (1 + s.gamma)
}

// impulse3 of 3 rows solved together, with the inverse of their mass matrix
func (s spring) impulse3(inverseMass mgl64.Mat3, velocity, bias, accumulated mgl64.Vec3) mgl64.Vec3 {
	return inverseMass.Mul3x1(velocity.Add(bias)).Add(accumulated.Mul(s.gamma)).Mul(-1 / (1 + s.gamma))
}

// bodyState is the copy of a dynamic body used by the solver during a step
type bodyState struct {
	body            *actor.RigidBody
	velocity        mgl64.Vec3
	angularVelocity mgl64.Vec3
	deltaPosition   mgl64.Vec3 // since the beginning of the step
	deltaRotation   mgl64.Quat // since the beginning of the step
	deltaMatrix     mgl64.Mat3 // deltaRotation as a matrix, updated once per substep
	invMass         float64
	inverseInertia  mgl64.Mat3 // inverse inertia in world space, turned with the body during the step
	startInertia    mgl64.Mat3 // inverse inertia in world space, at the beginning of the step
	rotation        mgl64.Quat // at the beginning of the step
	anisotropic     bool       // false if the inertia is the same on all axes: no gyroscopic torque, it doesn't turn
}

// jacobian of a contact direction d: the angular part rA × d and rB × d,
// and the angular velocity given by a unit impulse, I⁻¹ * (r × d)
type jacobian struct {
	angularA mgl64.Vec3
	angularB mgl64.Vec3
	impulseA mgl64.Vec3
	impulseB mgl64.Vec3
	mass     float64 // effective mass
}

type contactPoint struct {
	rA mgl64.Vec3 // from the center of mass of A
	rB mgl64.Vec3 // from the center of mass of B
	// coreA & coreB: the contact point without the radius of the rounded shapes (the center of a sphere, the axis of
	// a capsule). A rolling sphere turns its surface, not its center: the separation follows the cores
	coreA              mgl64.Vec3
	coreB              mgl64.Vec3
	baseSeparation     float64
	normal             jacobian
	normalImpulse      float64
	totalNormalImpulse float64 // the normal impulse of the step: the impulse which stopped the point, for the restitution
	normalVelocity     float64 // before the solver, for the restitution
	leverArm           float64 // distance to the friction center: the twist friction it can hold
}

type contactConstraint struct {
	manifold    *constraint.Manifold
	indexA      int // -1 for a static or sleeping body
	indexB      int
	normal      mgl64.Vec3
	tangents    [2]mgl64.Vec3
	restitution float64
	spring      spring
	points      [constraint.MaxContactPoints]contactPoint
	pointsCount int

	// radius of the rounded shapes: the anchors turn with their cores
	radiusA float64
	radiusB float64

	// friction of the contact, at the friction center of its points (as Box3D, Jolt, the friction patch of PhysX): along
	// both tangents, and the twist around the normal. The centers without the radius turn with the bodies (turnAnchors)
	friction        float64
	centerCoreA     mgl64.Vec3
	centerCoreB     mgl64.Vec3
	frictionRows    [2]jacobian
	frictionMass    [3]float64 // the inverse of the 2x2 mass matrix of both tangents: xx, xy, yy
	frictionImpulse [2]float64
	twistMass       float64
	twistImpulse    float64

	// rolling resistance, around both tangents
	rollingResistance float64
	rollingMass       [2]float64
	rollingImpulse    [2]float64
	rollingA          [2]mgl64.Vec3 // angular velocity of A given by a unit impulse
	rollingB          [2]mgl64.Vec3
}

type solver struct {
	states      []bodyState
	constraints []contactConstraint
	joints      []Joint
	// articulations: the trees of joints, solved together
	articulations articulations
	graph         constraintGraph
	pool          *workerPool
	jobs          solverJobs

	// parameters of the current stage, for the jobs
	manifolds       []constraint.Manifold
	contactSpring   spring
	staticSpring    spring
	gravity         mgl64.Vec3
	maxAngularSpeed float64
	stage           func(c *contactConstraint)
	color           []int
	// stateIndex: the state of each body of the World (-1 if not awake); stateBody: the body of each state
	stateIndex []int32
	stateBody  []int32
	bodies     []*actor.RigidBody
	// indices: the states of the bodies of the joints only (a map, for the few bodies of the joints)
	indices map[*actor.RigidBody]int
	h       float64
	invH    float64

	// static bodies (and sleeping ones) share this state: no mass, they never move
	static bodyState
}

// solverJobs are the functions run by the workers. They are created once: a closure created at each stage
// would allocate
type solverJobs struct {
	integrateVelocity func(i int)
	integratePosition func(i int)
	warmStart         func(c *contactConstraint)
	push              func(c *contactConstraint)
	relax             func(c *contactConstraint)
	restitution       func(c *contactConstraint)
	color             func(i int)
	prepareConstraint func(i int)
	storeImpulses     func(i int)
	finalize          func(i int)
	state             func(i int)
}

func (s *solver) initJobs() {
	if s.jobs.color != nil {
		return
	}
	s.jobs = solverJobs{
		integrateVelocity: s.integrateVelocity,
		integratePosition: s.integratePosition,
		warmStart:         s.warmStartConstraint,
		push:              s.pushConstraint,
		relax:             s.relaxConstraint,
		restitution:       s.restitutionConstraint,
		prepareConstraint: s.prepareConstraint,
		storeImpulses:     s.storeImpulsesConstraint,
		finalize:          s.finalizeBody,
		state:             s.stateOf,
		color: func(i int) {
			s.stage(&s.constraints[s.color[i]])
		},
	}
}

func (s *solver) state(index int) *bodyState {
	if index < 0 {
		return &s.static
	}
	return &s.states[index]
}

func (s *solver) prepare(bodies []*actor.RigidBody, manifolds []constraint.Manifold, dt float64, substeps int, contactHertz float64, pool *workerPool) {
	s.pool = pool
	s.initJobs()
	s.h = dt / float64(substeps)
	s.invH = 1 / s.h
	s.static = bodyState{deltaRotation: mgl64.QuatIdent(), deltaMatrix: mgl64.Ident3()}

	// ========== 1. Body states ==========
	// the awake dynamic bodies get a state, numbered in the order of the World; the states are filled in parallel
	if s.indices == nil {
		s.indices = make(map[*actor.RigidBody]int)
	}
	clear(s.indices)
	for _, joint := range s.joints {
		base := joint.base()
		s.indices[base.BodyA], s.indices[base.BodyB] = -1, -1
	}
	if cap(s.stateIndex) < len(bodies) {
		s.stateIndex = make([]int32, len(bodies))
	}
	s.stateIndex = s.stateIndex[:len(bodies)]
	s.stateBody = s.stateBody[:0]
	for i, body := range bodies {
		s.stateIndex[i] = -1
		if !isAwakeDynamic(body) {
			continue
		}
		s.stateIndex[i] = int32(len(s.stateBody))
		if len(s.indices) > 0 {
			if _, ok := s.indices[body]; ok {
				s.indices[body] = len(s.stateBody)
			}
		}
		s.stateBody = append(s.stateBody, int32(i))
	}
	if cap(s.states) < len(s.stateBody) {
		s.states = make([]bodyState, len(s.stateBody))
	}
	s.states = s.states[:len(s.stateBody)]
	s.bodies = bodies
	s.pool.run(len(s.states), bodiesChunk, s.jobs.state)
	s.bodies = nil

	// ========== 2. Contact constraints ==========
	hertz := math.Min(contactHertz, hertzPerSubstepRate*s.invH)
	s.contactSpring = newSpring(hertz, ContactDampingRatio, s.h)
	s.staticSpring = newSpring(2*hertz, 0.5*ContactDampingRatio, s.h)

	s.manifolds = manifolds
	if cap(s.constraints) < len(manifolds) {
		s.constraints = make([]contactConstraint, len(manifolds))
	}
	s.constraints = s.constraints[:len(manifolds)]
	s.pool.run(len(manifolds), constraintsChunk, s.jobs.prepareConstraint)
	s.manifolds = nil

	// ========== 3. Joints ==========
	for _, joint := range s.joints {
		joint.prepare(s)
	}
	s.buildArticulations()

	// ========== 4. Graph coloring ==========
	s.graph.color(s.constraints, len(s.states))
}

// isIsotropic: the same inertia on all axes (sphere, cube), the gyroscopic torque ω × Iω is null
func isIsotropic(inertia mgl64.Mat3) bool {
	return inertia[0] == inertia[4] && inertia[0] == inertia[8] &&
		inertia[1] == 0 && inertia[2] == 0 && inertia[3] == 0 && inertia[5] == 0 && inertia[6] == 0 && inertia[7] == 0
}

// prepareConstraint i, from the manifold i
func (s *solver) prepareConstraint(i int) {
	manifold := &s.manifolds[i]
	c := &s.constraints[i]
	*c = contactConstraint{
		manifold:    manifold,
		indexA:      int(s.stateIndex[manifold.IndexA]),
		indexB:      int(s.stateIndex[manifold.IndexB]),
		normal:      manifold.Normal,
		pointsCount: manifold.Count,
	}
	if c.indexA < 0 && c.indexB < 0 {
		// nothing to solve
		c.pointsCount = 0
		return
	}

	c.spring = s.contactSpring
	if c.indexA < 0 || c.indexB < 0 {
		c.spring = s.staticSpring
	}
	c.tangents[0], c.tangents[1] = tangentBasis(c.normal)
	c.restitution = constraint.ComputeRestitution(manifold.BodyA.Material, manifold.BodyB.Material)
	staticFriction := constraint.ComputeStaticFriction(manifold.BodyA.Material, manifold.BodyB.Material)
	dynamicFriction := constraint.ComputeDynamicFriction(manifold.BodyA.Material, manifold.BodyB.Material)

	stateA, stateB := s.state(c.indexA), s.state(c.indexB)
	radiusA, radiusB := shapeRadius(manifold.BodyA.Shape), shapeRadius(manifold.BodyB.Shape)
	c.radiusA, c.radiusB = radiusA, radiusB
	c.rollingResistance = constraint.ComputeRollingResistance(manifold.BodyA.Material, manifold.BodyB.Material, radiusA, radiusB)
	if c.rollingResistance > 0 {
		c.prepareRolling(stateA, stateB)
		for k := range c.tangents {
			c.rollingImpulse[k] = manifold.RollingImpulse.Dot(c.tangents[k])
		}
	}
	for j := 0; j < manifold.Count; j++ {
		point := &manifold.Points[j]
		cp := &c.points[j]

		cp.rA = point.Position.Sub(manifold.BodyA.Transform.Position)
		cp.rB = point.Position.Sub(manifold.BodyB.Transform.Position)
		// the point on the surface of each body (Position is halfway), then its core
		half := c.normal.Mul(point.Separation / 2)
		cp.coreA = cp.rA.Sub(half).Sub(c.normal.Mul(radiusA))
		cp.coreB = cp.rB.Add(half).Add(c.normal.Mul(radiusB))
		cp.baseSeparation = point.Separation - cp.coreB.Sub(cp.coreA).Dot(c.normal)
		cp.normal = makeJacobian(stateA, stateB, cp.rA, cp.rB, c.normal)

		// Warm starting: the impulses of the previous step
		cp.normalImpulse = point.NormalImpulse
		cp.normalVelocity = relativeVelocity(stateA, stateB, cp.rA, cp.rB).Dot(c.normal)
	}
	c.prepareFriction(stateA, stateB, staticFriction, dynamicFriction)
}

// prepareFriction: the friction center is the average of the points, weighted by their separation as in Box3D (a
// speculative point far from touching barely counts: 1 up to SpeculativeDistance, 0 at twice). The friction is the
// static one if the center slides slower than StaticFrictionSpeed
func (c *contactConstraint) prepareFriction(stateA, stateB *bodyState, staticFriction, dynamicFriction float64) {
	manifold := c.manifold
	var centerA, centerB mgl64.Vec3
	total := 0.0
	for j := 0; j < c.pointsCount; j++ {
		cp := &c.points[j]
		weight := math.Min(math.Max(2-manifold.Points[j].Separation/SpeculativeDistance, minFrictionWeight), 1)
		centerA = centerA.Add(cp.coreA.Mul(weight))
		centerB = centerB.Add(cp.coreB.Mul(weight))
		total += weight
	}
	c.centerCoreA, c.centerCoreB = centerA.Mul(1/total), centerB.Mul(1/total)
	rA, rB := c.frictionArms(c.centerCoreA, c.centerCoreB)
	for j := 0; j < c.pointsCount; j++ {
		c.points[j].leverArm = c.points[j].rA.Sub(rA).Len()
	}
	c.makeFrictionRows(stateA, stateB, rA, rB)

	c.frictionImpulse = [2]float64{manifold.FrictionImpulse.Dot(c.tangents[0]), manifold.FrictionImpulse.Dot(c.tangents[1])}
	c.twistImpulse = manifold.TwistImpulse
	relativeVel := relativeVelocity(stateA, stateB, rA, rB)
	c.friction = dynamicFriction
	if relativeVel.Sub(c.normal.Mul(relativeVel.Dot(c.normal))).Len() < StaticFrictionSpeed {
		c.friction = staticFriction
	}
}

// frictionArms: the lever arms of the friction center, from its cores (the radius of the rounded shapes along the normal)
func (c *contactConstraint) frictionArms(coreA, coreB mgl64.Vec3) (mgl64.Vec3, mgl64.Vec3) {
	return coreA.Add(c.normal.Mul(c.radiusA)), coreB.Sub(c.normal.Mul(c.radiusB))
}

// makeFrictionRows: both tangents at the friction center, their 2x2 mass matrix (coupled by the rotation), and the
// mass of the twist around the normal
func (c *contactConstraint) makeFrictionRows(stateA, stateB *bodyState, rA, rB mgl64.Vec3) {
	for k := range c.frictionRows {
		c.frictionRows[k] = makeJacobian(stateA, stateB, rA, rB, c.tangents[k])
	}
	t0, t1 := &c.frictionRows[0], &c.frictionRows[1]
	linear := stateA.invMass + stateB.invMass
	kxx := linear + t0.angularA.Dot(t0.impulseA) + t0.angularB.Dot(t0.impulseB)
	kyy := linear + t1.angularA.Dot(t1.impulseA) + t1.angularB.Dot(t1.impulseB)
	kxy := t0.angularA.Dot(t1.impulseA) + t0.angularB.Dot(t1.impulseB)
	c.frictionMass = [3]float64{}
	if det := kxx*kyy - kxy*kxy; det > minFrictionDeterminant {
		c.frictionMass = [3]float64{kyy / det, -kxy / det, kxx / det}
	}
	c.twistMass = 0
	if k := c.normal.Dot(stateA.inverseInertia.Mul3x1(c.normal)) + c.normal.Dot(stateB.inverseInertia.Mul3x1(c.normal)); k > 0 {
		c.twistMass = 1 / k
	}
}

// prepareRolling: the angular velocity given by a unit rolling impulse around both tangents, and its mass
func (c *contactConstraint) prepareRolling(stateA, stateB *bodyState) {
	for k := range c.tangents {
		c.rollingA[k] = stateA.inverseInertia.Mul3x1(c.tangents[k])
		c.rollingB[k] = stateB.inverseInertia.Mul3x1(c.tangents[k])
		c.rollingMass[k] = 0
		if mass := c.rollingA[k].Dot(c.tangents[k]) + c.rollingB[k].Dot(c.tangents[k]); mass > 0 {
			c.rollingMass[k] = 1 / mass
		}
	}
}

// shapeRadius is the radius of the rounded shapes, for the rolling resistance and the separation
func shapeRadius(shape actor.ShapeInterface) float64 {
	switch shape := shape.(type) {
	case *actor.Sphere:
		return shape.Radius
	case *actor.Capsule:
		return shape.Radius
	}
	return 0
}

// applyRolling applies the rolling impulses λ around both tangents: -λ on A, +λ on B
func (c *contactConstraint) applyRolling(stateA, stateB *bodyState, lambda [2]float64) {
	if stateA.body != nil {
		stateA.angularVelocity = stateA.angularVelocity.Sub(c.rollingA[0].Mul(lambda[0])).Sub(c.rollingA[1].Mul(lambda[1]))
	}
	if stateB.body != nil {
		stateB.angularVelocity = stateB.angularVelocity.Add(c.rollingB[0].Mul(lambda[0])).Add(c.rollingB[1].Mul(lambda[1]))
	}
}

// indexOf: the state of a body of a joint
func (s *solver) indexOf(body *actor.RigidBody) int {
	if index, ok := s.indices[body]; ok {
		return index
	}
	return -1
}

// stateOf fills the state of the body i
func (s *solver) stateOf(i int) {
	body := s.bodies[s.stateBody[i]]
	s.states[i] = bodyState{
		body:            body,
		velocity:        body.Velocity,
		angularVelocity: body.AngularVelocity,
		deltaRotation:   mgl64.QuatIdent(),
		deltaMatrix:     mgl64.Ident3(),
		invMass:         body.InverseMass(),
		inverseInertia:  body.GetInverseInertiaWorld(),
		startInertia:    body.GetInverseInertiaWorld(),
		rotation:        body.Transform.Rotation,
		anisotropic:     !isIsotropic(body.InertiaLocal),
	}
}

// makeJacobian for an impulse along the direction, applied at rA and rB
func makeJacobian(stateA, stateB *bodyState, rA, rB, direction mgl64.Vec3) jacobian {
	j := jacobian{angularA: rA.Cross(direction), angularB: rB.Cross(direction)}
	j.impulseA = stateA.inverseInertia.Mul3x1(j.angularA)
	j.impulseB = stateB.inverseInertia.Mul3x1(j.angularB)

	k := stateA.invMass + stateB.invMass + j.impulseA.Dot(j.angularA) + j.impulseB.Dot(j.angularB)
	if k > 0 {
		j.mass = 1 / k
	}
	return j
}

// velocity of B relative to A along the direction of the jacobian
func (j *jacobian) velocity(stateA, stateB *bodyState, direction mgl64.Vec3) float64 {
	return stateB.velocity.Sub(stateA.velocity).Dot(direction) + stateB.angularVelocity.Dot(j.angularB) - stateA.angularVelocity.Dot(j.angularA)
}

// apply the impulse λ along the direction: -λ on A, +λ on B
// The static state is shared by all the static bodies: it is never written (it has no mass anyway)
func (j *jacobian) apply(stateA, stateB *bodyState, direction mgl64.Vec3, lambda float64) {
	if stateA.body != nil {
		stateA.velocity = stateA.velocity.Sub(direction.Mul(lambda * stateA.invMass))
		stateA.angularVelocity = stateA.angularVelocity.Sub(j.impulseA.Mul(lambda))
	}
	if stateB.body != nil {
		stateB.velocity = stateB.velocity.Add(direction.Mul(lambda * stateB.invMass))
		stateB.angularVelocity = stateB.angularVelocity.Add(j.impulseB.Mul(lambda))
	}
}

// relativeVelocity of B relative to A, at the contact point
func relativeVelocity(stateA, stateB *bodyState, rA, rB mgl64.Vec3) mgl64.Vec3 {
	vA := stateA.velocity.Add(stateA.angularVelocity.Cross(rA))
	vB := stateB.velocity.Add(stateB.angularVelocity.Cross(rB))
	return vB.Sub(vA)
}

// currentSeparation: the contact points are not computed again during the sub-steps,
// the separation is updated from the motion of both bodies
func currentSeparation(stateA, stateB *bodyState, cp *contactPoint, normal mgl64.Vec3) float64 {
	delta := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(stateB.deltaMatrix.Mul3x1(cp.coreB)).Sub(stateA.deltaMatrix.Mul3x1(cp.coreA))
	return cp.baseSeparation + delta.Dot(normal)
}

// turnAnchors: the lever arms of the contacts turn with the bodies, once per substep (before Relax).
// A body turning fast (a tumbling capsule) would otherwise be pushed at the place its contact had at the beginning
// of the step: the solver would see the contact open while the body sinks.
// The core of a rounded shape turns, its radius stays along the normal. A static body doesn't turn
func (c *contactConstraint) turnAnchors(stateA, stateB *bodyState) {
	turnA := stateA.body != nil && math.Abs(stateA.deltaRotation.W) < turnAnchorsCos
	turnB := stateB.body != nil && math.Abs(stateB.deltaRotation.W) < turnAnchorsCos
	if !turnA && !turnB {
		return
	}
	for j := 0; j < c.pointsCount; j++ {
		cp := &c.points[j]
		if turnA {
			cp.normal.turnA(stateA, stateA.deltaMatrix.Mul3x1(cp.coreA).Add(c.normal.Mul(c.radiusA)), c.normal)
		}
		if turnB {
			cp.normal.turnB(stateB, stateB.deltaMatrix.Mul3x1(cp.coreB).Sub(c.normal.Mul(c.radiusB)), c.normal)
		}
		cp.normal.updateMass(stateA, stateB)
	}
	// the friction center turns with its bodies
	coreA, coreB := c.centerCoreA, c.centerCoreB
	if turnA {
		coreA = stateA.deltaMatrix.Mul3x1(coreA)
	}
	if turnB {
		coreB = stateB.deltaMatrix.Mul3x1(coreB)
	}
	rA, rB := c.frictionArms(coreA, coreB)
	c.makeFrictionRows(stateA, stateB, rA, rB)
	if c.rollingResistance > 0 {
		c.prepareRolling(stateA, stateB)
	}
}

// turnA: the lever arm of A is rA
func (j *jacobian) turnA(stateA *bodyState, rA, direction mgl64.Vec3) {
	j.angularA = rA.Cross(direction)
	j.impulseA = stateA.inverseInertia.Mul3x1(j.angularA)
}

// turnB: the lever arm of B is rB
func (j *jacobian) turnB(stateB *bodyState, rB, direction mgl64.Vec3) {
	j.angularB = rB.Cross(direction)
	j.impulseB = stateB.inverseInertia.Mul3x1(j.angularB)
}

func (j *jacobian) updateMass(stateA, stateB *bodyState) {
	j.mass = 0
	if k := stateA.invMass + stateB.invMass + j.impulseA.Dot(j.angularA) + j.impulseB.Dot(j.angularB); k > 0 {
		j.mass = 1 / k
	}
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
	s.gravity = gravity
	s.forEachBody(s.jobs.integrateVelocity)
}

func (s *solver) integrateVelocity(i int) {
	h, gravity := s.h, s.gravity
	state := &s.states[i]
	body := state.body

	linearDamping := 1 / (1 + h*body.Material.LinearDamping)
	angularDamping := 1 / (1 + h*body.Material.AngularDamping)

	// ========== LINEAR ==========
	state.velocity = state.velocity.Mul(linearDamping).Add(gravity.Add(body.Force().Mul(state.invMass)).Mul(h))

	// ========== ANGULAR ==========
	angularVelocity := state.angularVelocity
	if state.anisotropic {
		angularVelocity = gyroscopic(angularVelocity, state.deltaRotation.Mul(state.rotation).Normalize(), body.InertiaLocal, h)
	}
	state.angularVelocity = angularVelocity.Mul(angularDamping)
	if torque := body.Torque(); torque != (mgl64.Vec3{}) {
		state.angularVelocity = state.angularVelocity.Add(state.inverseInertia.Mul3x1(torque).Mul(h))
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
	s.maxAngularSpeed = MaxRotation * s.invH
	s.forEachBody(s.jobs.integratePosition)
}

func (s *solver) integratePosition(i int) {
	h, maxAngularSpeed := s.h, s.maxAngularSpeed
	state := &s.states[i]
	if speed := state.velocity.Len(); speed > MaxLinearSpeed {
		state.velocity = state.velocity.Mul(MaxLinearSpeed / speed)
	}
	if speed := state.angularVelocity.Len(); speed > maxAngularSpeed {
		state.angularVelocity = state.angularVelocity.Mul(maxAngularSpeed / speed)
	}

	state.deltaPosition = state.deltaPosition.Add(state.velocity.Mul(h))
	state.deltaRotation = integrateRotation(state.deltaRotation, state.angularVelocity.Mul(h))
	state.deltaMatrix = rotationMatrix(state.deltaRotation)

	// the inertia turns with the body, as the anchors of its contacts (turnAnchors): I⁻¹ = ΔR I⁻¹start ΔRᵀ
	if state.anisotropic && math.Abs(state.deltaRotation.W) < turnAnchorsCos {
		state.inverseInertia = state.deltaMatrix.Mul3(state.startInertia).Mul3(state.deltaMatrix.Transpose())
	}
}

// rotationMatrix of a unit quaternion (column major)
func rotationMatrix(q mgl64.Quat) mgl64.Mat3 {
	w, x, y, z := q.W, q.V[0], q.V[1], q.V[2]
	return mgl64.Mat3{
		1 - 2*(y*y+z*z), 2 * (x*y + w*z), 2 * (x*z - w*y),
		2 * (x*y - w*z), 1 - 2*(x*x+z*z), 2 * (y*z + w*x),
		2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2*(x*x+y*y),
	}
}

// integrateRotation for a small rotation vector: q + 0.5 * θ * q
func integrateRotation(q mgl64.Quat, theta mgl64.Vec3) mgl64.Quat {
	qDot := mgl64.Quat{W: 0, V: theta}.Mul(q).Scale(0.5)
	return q.Add(qDot).Normalize()
}

// The joints are solved before the contacts, on a single goroutine
func (s *solver) warmStart() {
	for _, joint := range s.joints {
		joint.warmStart(s)
	}
	s.solveConstraints(s.jobs.warmStart)
}

func (s *solver) warmStartConstraint(c *contactConstraint) {
	stateA, stateB := s.state(c.indexA), s.state(c.indexB)
	for j := 0; j < c.pointsCount; j++ {
		cp := &c.points[j]
		cp.totalNormalImpulse += cp.normalImpulse
		cp.normal.apply(stateA, stateB, c.normal, cp.normalImpulse)
	}
	c.frictionRows[0].apply(stateA, stateB, c.tangents[0], c.frictionImpulse[0])
	c.frictionRows[1].apply(stateA, stateB, c.tangents[1], c.frictionImpulse[1])
	c.applyTwist(stateA, stateB, c.twistImpulse)
	if c.rollingResistance > 0 {
		c.applyRolling(stateA, stateB, c.rollingImpulse)
	}
}

// push solves the contacts with their spring, to push the overlap out. No friction (as Box3D): solved there, before
// the normals, it pushed the light bodies out from under a heavy one (at 4 substeps, 2 cubes 1 m out from under a slab
// 400 times heavier)
func (s *solver) push() {
	for _, joint := range s.joints {
		joint.solve(s, true)
	}
	s.solveArticulations(true)
	s.solveConstraints(s.jobs.push)
}

func (s *solver) pushConstraint(c *contactConstraint) {
	stateA, stateB := s.state(c.indexA), s.state(c.indexB)
	s.solveNormals(c, stateA, stateB, true)
}

// relax solves the contacts again as rigid constraints (pushing the overlap out adds energy), then the friction
func (s *solver) relax() {
	for _, joint := range s.joints {
		joint.solve(s, false)
	}
	s.solveArticulations(false)
	s.solveConstraints(s.jobs.relax)
}

func (s *solver) relaxConstraint(c *contactConstraint) {
	stateA, stateB := s.state(c.indexA), s.state(c.indexB)
	c.turnAnchors(stateA, stateB)
	s.solveNormals(c, stateA, stateB, false)
	c.solveRolling(stateA, stateB)
	c.solveFriction(stateA, stateB)
}

// ========== NORMAL ==========
// solveNormals: the points of the contact must not overlap. A speculative point (separation > 0) can get closer by its
// separation during the substep, not further. An overlapping point is pushed out by the spring of the contact (soft,
// at ContactSpeed at most), or only stopped (rigid).
// The points of a contact are solved together, exactly (block Gauss-Seidel: the block solver of Box2D v2, for 4 points).
// Solved one after the other, the first point takes more than its share and turns the body: a box landing flat starts
// to tip, and the rounding decides which way
func (s *solver) solveNormals(c *contactConstraint, stateA, stateB *bodyState, soft bool) {
	n := c.pointsCount
	var block normalBlock
	block.count = n
	for i := 0; i < n; i++ {
		cp := &c.points[i]
		separation := currentSeparation(stateA, stateB, cp, c.normal)
		bias, gamma := 0.0, 0.0
		if separation > 0 {
			bias = separation * s.invH
		} else if soft {
			bias = math.Max(c.spring.biasRate*separation, -ContactSpeed)
			gamma = c.spring.gamma
		}
		for j := 0; j <= i; j++ {
			k := stateA.invMass + stateB.invMass + cp.normal.angularA.Dot(c.points[j].normal.impulseA) +
				cp.normal.angularB.Dot(c.points[j].normal.impulseB)
			block.matrix[i][j], block.matrix[j][i] = k, k
		}
		// the softness of the row: the fixed point of its own impulse is v + b + γ K_ii λ = 0
		block.softness[i] = gamma * block.matrix[i][i]
		block.previous[i] = cp.normalImpulse
		block.offset[i] = cp.normal.velocity(stateA, stateB, c.normal) + bias
	}
	impulses := block.solve()
	for i := 0; i < n; i++ {
		cp := &c.points[i]
		cp.addNormalImpulse(stateA, stateB, c.normal, impulses[i]-cp.normalImpulse)
	}
}

// normalBlock: the normal rows of a contact. Their accumulated impulses λ are the solution of the linear
// complementarity problem w = A λ + r, λ ≥ 0, w ≥ 0, λ w = 0, with A = K + D:
//   - K the mass matrix of the rows (the relative velocity of the point i given by a unit impulse at the point j)
//   - D the softness of the rows
//   - r = v + b - K λ₀, from the velocities v and the biases b with the impulses λ₀ of the rows
type normalBlock struct {
	count    int
	matrix   [constraint.MaxContactPoints][constraint.MaxContactPoints]float64
	softness [constraint.MaxContactPoints]float64
	previous [constraint.MaxContactPoints]float64
	offset   [constraint.MaxContactPoints]float64
}

const (
	// blockRegularization: 4 rigid points on a face give 3 independent rows only (a translation, 2 rotations): K is
	// singular, the share of the load between the points is not defined. A proximal term ε W (λ - λ₀) chooses the share
	// closest to the impulses the rows start from (the proximal point method: Rockafellar 1976; the proximal
	// formulations of contact of Alart & Curnier 1991, Acary & Brogliato 2008). Repeated at each pass, its bias
	// vanishes. ε = 1e-3 keeps A well conditioned (~1000, the bound of the block solver of Box2D v2)
	blockRegularization = 1e-3

	// blockTolerance: an impulse (N·s) or a relative velocity (m/s) within the rounding of 0 is 0
	blockTolerance = 1e-12
)

// solve the problem by enumerating the sets of active points, all of them first (Murty's total enumeration, as the
// block solver of Box2D v2). A is positive definite: the solution is unique, the first set found is the only one
func (b *normalBlock) solve() [constraint.MaxContactPoints]float64 {
	n := b.count
	var a [constraint.MaxContactPoints][constraint.MaxContactPoints]float64
	var r [constraint.MaxContactPoints]float64
	for i := 0; i < n; i++ {
		r[i] = b.offset[i]
		for j := 0; j < n; j++ {
			a[i][j] = b.matrix[i][j]
			r[i] -= b.matrix[i][j] * b.previous[j]
		}
		// the proximal term, towards the impulses λ₀ the rows start from
		a[i][i] += b.softness[i] + blockRegularization*b.matrix[i][i]
		r[i] -= blockRegularization * b.matrix[i][i] * b.previous[i]
	}

	for set := (1 << n) - 1; set > 0; set-- {
		if lambda, ok := solveActive(&a, &r, n, set); ok {
			return lambda
		}
	}
	// no point pushes
	return [constraint.MaxContactPoints]float64{}
}

// solveActive solves the rows of the set (a bit per point) with the others at 0, and checks the solution: the active
// impulses push, the inactive points don't get closer
func solveActive(a *[constraint.MaxContactPoints][constraint.MaxContactPoints]float64, r *[constraint.MaxContactPoints]float64, n, set int) ([constraint.MaxContactPoints]float64, bool) {
	var index [constraint.MaxContactPoints]int
	m := 0
	for i := 0; i < n; i++ {
		if set&(1<<i) != 0 {
			index[m] = i
			m++
		}
	}
	// Gaussian elimination of the active rows (A is positive definite: no pivoting)
	var sub [constraint.MaxContactPoints][constraint.MaxContactPoints + 1]float64
	for i := 0; i < m; i++ {
		for j := 0; j < m; j++ {
			sub[i][j] = a[index[i]][index[j]]
		}
		sub[i][m] = -r[index[i]]
	}
	for col := 0; col < m; col++ {
		if sub[col][col] <= 0 {
			return [constraint.MaxContactPoints]float64{}, false
		}
		for row := col + 1; row < m; row++ {
			f := sub[row][col] / sub[col][col]
			for k := col; k <= m; k++ {
				sub[row][k] -= f * sub[col][k]
			}
		}
	}
	var lambda [constraint.MaxContactPoints]float64
	for i := m - 1; i >= 0; i-- {
		sum := sub[i][m]
		for j := i + 1; j < m; j++ {
			sum -= sub[i][j] * lambda[index[j]]
		}
		value := sum / sub[i][i]
		if value < -blockTolerance {
			return lambda, false
		}
		lambda[index[i]] = math.Max(value, 0)
	}
	for i := 0; i < n; i++ {
		if set&(1<<i) != 0 {
			continue
		}
		w := r[i]
		for j := 0; j < n; j++ {
			w += a[i][j] * lambda[j]
		}
		if w < -blockTolerance {
			return lambda, false
		}
	}
	return lambda, true
}

// addNormalImpulse: the accumulated impulse of a contact stays positive (it pushes, never pulls).
// Returns the impulse applied
func (cp *contactPoint) addNormalImpulse(stateA, stateB *bodyState, normal mgl64.Vec3, impulse float64) float64 {
	accumulated := math.Max(cp.normalImpulse+impulse, 0)
	impulse = accumulated - cp.normalImpulse
	cp.normalImpulse = accumulated
	cp.totalNormalImpulse += impulse
	cp.normal.apply(stateA, stateB, normal, impulse)
	return impulse
}

// ========== ROLLING RESISTANCE ==========
// solveRolling: a torque against the rolling, up to rollingResistance * the normal impulse
func (c *contactConstraint) solveRolling(stateA, stateB *bodyState) {
	if c.rollingResistance <= 0 {
		return
	}
	normalImpulse := 0.0
	for j := 0; j < c.pointsCount; j++ {
		normalImpulse += c.points[j].normalImpulse
	}
	rolling := stateB.angularVelocity.Sub(stateA.angularVelocity)
	previous := c.rollingImpulse
	impulse := [2]float64{
		previous[0] - c.rollingMass[0]*rolling.Dot(c.tangents[0]),
		previous[1] - c.rollingMass[1]*rolling.Dot(c.tangents[1]),
	}
	clampDisk(&impulse, c.rollingResistance*normalImpulse)
	c.rollingImpulse = impulse
	c.applyRolling(stateA, stateB, [2]float64{impulse[0] - previous[0], impulse[1] - previous[1]})
}

// ========== FRICTION ==========
// solveFriction: Coulomb's law at the friction center (as Box3D & Jolt). The twist around the normal is held up to
// µ Σ (lever arm × normal impulse) of the points, then the tangent impulse stays in a disk of radius µ Σ normal impulse
func (c *contactConstraint) solveFriction(stateA, stateB *bodyState) {
	normalImpulse, twistLimit := 0.0, 0.0
	for j := 0; j < c.pointsCount; j++ {
		normalImpulse += c.points[j].normalImpulse
		twistLimit += c.points[j].leverArm * c.points[j].normalImpulse
	}

	// twist
	twistSpeed := c.normal.Dot(stateB.angularVelocity.Sub(stateA.angularVelocity))
	previousTwist := c.twistImpulse
	c.twistImpulse = math.Max(-c.friction*twistLimit, math.Min(c.friction*twistLimit, previousTwist-c.twistMass*twistSpeed))
	c.applyTwist(stateA, stateB, c.twistImpulse-previousTwist)

	// both tangents, together
	v0 := c.frictionRows[0].velocity(stateA, stateB, c.tangents[0])
	v1 := c.frictionRows[1].velocity(stateA, stateB, c.tangents[1])
	m := c.frictionMass
	previous := c.frictionImpulse
	impulse := [2]float64{previous[0] - (m[0]*v0 + m[1]*v1), previous[1] - (m[1]*v0 + m[2]*v1)}
	clampDisk(&impulse, c.friction*normalImpulse)
	c.frictionImpulse = impulse
	c.frictionRows[0].apply(stateA, stateB, c.tangents[0], impulse[0]-previous[0])
	c.frictionRows[1].apply(stateA, stateB, c.tangents[1], impulse[1]-previous[1])
}

// applyTwist: the angular impulse around the normal, -λ on A, +λ on B
func (c *contactConstraint) applyTwist(stateA, stateB *bodyState, lambda float64) {
	if stateA.body != nil {
		stateA.angularVelocity = stateA.angularVelocity.Sub(stateA.inverseInertia.Mul3x1(c.normal.Mul(lambda)))
	}
	if stateB.body != nil {
		stateB.angularVelocity = stateB.angularVelocity.Add(stateB.inverseInertia.Mul3x1(c.normal.Mul(lambda)))
	}
}

// clampDisk scales the 2D impulse down to the radius
func clampDisk(impulse *[2]float64, radius float64) {
	length := math.Hypot(impulse[0], impulse[1])
	if length <= radius {
		return
	}
	scale := 0.0
	if length > 0 {
		scale = radius / length
	}
	impulse[0] *= scale
	impulse[1] *= scale
}

// ========== RESTITUTION ==========
// restitution, after the substeps: a point which hit faster than RestitutionThreshold bounces. Its impulse goes towards
// the velocity -restitution * its velocity before the step (Newton), and is at most restitution times the impulse which
// stopped it, its normal impulse of the step (Poisson's hypothesis, W. J. Stronge, Impact Mechanics): a pile of bodies
// doesn't give back more than it absorbed
func (s *solver) restitution() {
	s.solveConstraints(s.jobs.restitution)
}

func (s *solver) restitutionConstraint(c *contactConstraint) {
	if c.restitution == 0 {
		return
	}
	stateA, stateB := s.state(c.indexA), s.state(c.indexB)
	for j := 0; j < c.pointsCount; j++ {
		cp := &c.points[j]
		if cp.normalVelocity >= -RestitutionThreshold || cp.totalNormalImpulse <= 0 {
			continue
		}
		velocity := cp.normal.velocity(stateA, stateB, c.normal)
		newton := -cp.normal.mass * (velocity + c.restitution*cp.normalVelocity)
		poisson := c.restitution * cp.totalNormalImpulse
		if impulse := math.Min(newton, poisson); impulse > 0 {
			cp.addNormalImpulse(stateA, stateB, c.normal, impulse)
		}
	}
}

// storeImpulses in the manifolds, for the warm starting of the next step
func (s *solver) storeImpulses() {
	s.pool.run(len(s.constraints), constraintsChunk, s.jobs.storeImpulses)
}

func (s *solver) storeImpulsesConstraint(i int) {
	c := &s.constraints[i]
	c.manifold.RollingImpulse = c.tangents[0].Mul(c.rollingImpulse[0]).Add(c.tangents[1].Mul(c.rollingImpulse[1]))
	for j := 0; j < c.pointsCount; j++ {
		point := &c.manifold.Points[j]
		cp := &c.points[j]
		point.NormalImpulse = cp.normalImpulse
	}
	c.manifold.FrictionImpulse = c.tangents[0].Mul(c.frictionImpulse[0]).Add(c.tangents[1].Mul(c.frictionImpulse[1]))
	c.manifold.TwistImpulse = c.twistImpulse
}

// finalize writes the new transform and velocities into the bodies
func (s *solver) finalize() {
	s.forEachBody(s.jobs.finalize)
}

func (s *solver) finalizeBody(i int) {
	state := &s.states[i]
	body := state.body
	body.Transform.Position = body.Transform.Position.Add(state.deltaPosition)
	body.Transform.Rotation = state.deltaRotation.Mul(state.rotation).Normalize()
	body.Velocity = state.velocity
	body.AngularVelocity = state.angularVelocity
	body.ClearForces()
	body.UpdateAABB()
}
