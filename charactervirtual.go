package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== CHARACTER ==========
// A character is a capsule the game moves at a velocity, which walks on the ground and the decor, slides along the
// walls, climbs the slopes under its limit and the steps under its height, stays on the floor downhill, is carried by
// a moving platform, pushes the dynamic bodies with a bounded force and is blocked by the heavy ones and by the other
// characters. It is virtual, as CharacterVirtual of Jolt (Jolt/Physics/Character/CharacterVirtual.cpp, v5.3.0) and
// the mover of Box2D v3 (src/mover.c, samples/sample_character.cpp, v3.1.0): no rigid body moves it, it moves itself
// between 2 steps with the queries of the world (the contacts of its capsule and the sweeps), so that it goes exactly
// where the game decides. PxController of PhysX (physxcharacterkinematic/src/CctCharacterController.cpp, 5.11) and
// CharacterBody3D of Godot (scene/3d/physics/character_body_3d.cpp, 4.4) are of the same kind.
//
// An inner kinematic body stands in the world at the capsule of the character (World.AddCharacter), teleported there
// after each Update without velocity (the inner body of CharacterVirtual, UpdateInnerBodyTransform; the kinematic
// actor of PxController): the rays and the sweeps see it, a dynamic body which runs into it is stopped by the solver
// (it never goes through a character), and the other characters find it in their contacts and their sweeps, so that
// 2 characters block each other or slide along each other (the CharacterVsCharacterCollision list of Jolt is not
// needed). It pushes nothing by itself: the character pushes the dynamic bodies with impulses it bounds.
//
// The game gives the velocity (CharacterVirtual.Velocity) and calls Update after the step of the world. The gravity is the
// game's, as in all four references: the recipe of the samples of Jolt (CharacterVirtualTest::HandleInput) is, on the
// ground, velocity = the velocity of the ground + the input, else the vertical velocity kept + the horizontal input,
// plus gravity * dt in both cases. See PHYSICS_GUIDE.md.
//
// Update follows CharacterVirtual::ExtendedUpdate: the velocity towards the slopes too steep is cancelled, the
// capsule is moved (character_move.go: the contacts around it become planes, a solver by time of impact slides the
// velocity along them, a sweep checks the path), the ground is found among the contacts, the character sticks to the
// floor when it leaves it downhill, and walks the stairs when a step blocked it. Up is +Y: Feather is Y-up.

const (
	// CharacterPadding: the character keeps this distance from everything (m): its capsule is inflated by it for its
	// contacts and its sweeps, so that the sweeps rarely hit and the character never gets stuck in the rounding of a
	// contact (mCharacterPadding of Jolt, 0.02; contactOffset of PhysX plays the same role, 0.1)
	CharacterPadding = 0.02

	// DefaultCharacterMaxSlopeAngle: the slope a character climbs, in rad (50° in Jolt, CharacterBaseSettings; 45° in
	// PhysX, slopeLimit 0.707, and in Godot, floor_max_angle)
	DefaultCharacterMaxSlopeAngle = 50 * math.Pi / 180

	// DefaultCharacterStepHeight: the step a character climbs without jumping, in m (mWalkStairsStepUp of Jolt, 0.4;
	// stepOffset of PhysX, 0.5)
	DefaultCharacterStepHeight = 0.4

	// DefaultCharacterMaxStrength: the force a character pushes the dynamic bodies with, in N (mMaxStrength of Jolt)
	DefaultCharacterMaxStrength = 100.0

	// DefaultCharacterMass: the mass of a character, in kg, its weight on the dynamic body it stands on (mMass of Jolt)
	DefaultCharacterMass = 70.0

	// characterPredictiveDistance: the contacts are gathered this far around the character (m), so that the planes it
	// slides along are known before it touches them (mPredictiveContactDistance of Jolt, 0.1: "A value of 0 will most
	// likely cause the character to get stuck as it cannot properly calculate a sliding direction")
	characterPredictiveDistance = 0.1

	// characterCollisionTolerance: how deep the character accepts to enter a body (m): a contact closer than this is
	// a collision, a sweep which would enter by less is ignored (mCollisionTolerance of Jolt)
	characterCollisionTolerance = 1e-3

	// characterAcceptedPenetration: a plane the velocity would enter by less than this (m) during the time left is not
	// solved (the "too little penetration" of SolveConstraints of Jolt)
	characterAcceptedPenetration = 1e-4

	// characterMaxCollisionIterations: the loops of contacts, solve and sweep of a move (mMaxCollisionIterations)
	characterMaxCollisionIterations = 5

	// characterMaxConstraintIterations: the planes solved at most during a move (mMaxConstraintIterations of Jolt)
	characterMaxConstraintIterations = 15

	// characterMinTimeRemaining: a move stops when less than this remains to simulate (s, mMinTimeRemaining of Jolt)
	characterMinTimeRemaining = 1e-4

	// characterPenetrationRecovery: the part of a penetration resolved in one update (mPenetrationRecoverySpeed of
	// Jolt, 1: everything)
	characterPenetrationRecovery = 1.0

	// DefaultCharacterStickToFloor: how far down the floor is looked for when the character leaves the ground without
	// going up, in m (mStickToFloorStepDown of the ExtendedUpdateSettings of Jolt, 0.5; floor_snap_length of Godot is
	// 0.1)
	DefaultCharacterStickToFloor = 0.5

	// characterMinStepForward: the least horizontal step of a stair walk, whatever the frame rate (m,
	// mWalkStairsMinStepForward of Jolt)
	characterMinStepForward = 0.02

	// characterStepForwardTest: how far ahead the floor of a step is tested when the first test lands on its edge (m,
	// mWalkStairsStepForwardTest of Jolt)
	characterStepForwardTest = 0.15

	// characterSteepContactPenetration: the conflicting contacts of a body (opposed normals, both this deep) are
	// reduced to the deepest one, so that a character squeezed in a body can get out (1.25 * padding, Jolt
	// RemoveConflictingContacts)
	characterConflictDepth = 1.25 * CharacterPadding

	// characterPushDamping and characterPushPenetration: the impulse on a dynamic body brings it to the velocity of
	// the character damped by 0.9, plus 0.4 of the penetration per update (cDamping & cPenetrationResolution of
	// CharacterVirtual::HandleContact)
	characterPushDamping     = 0.9
	characterPushPenetration = 0.4

	// characterGroundNormalCos: a contact tilted by less than 85° from up counts in the normal and the velocity of the
	// ground (0.08 in CharacterVirtual::UpdateSupportingContact)
	characterGroundNormalCos = 0.08

	// characterParallelCos: 2 planes closer than about 10° are not slid along their edge (0.984 in
	// CharacterVirtual::SolveConstraints)
	characterParallelCos = 0.984

	// characterVerticalEpsilon: a steep contact whose normal has a horizontal part shorter than this (the rounding of
	// a vertical normal under a slope limit of 0) gets no vertical plane
	characterVerticalEpsilon = 1e-6

	// characterProgressFraction: a stair walk which moved the character against the steep contact by less than this
	// part of the step forward made no progress and is cancelled (5 % in CharacterVirtual::WalkStairs)
	characterProgressFraction = 0.05
)

// characterCosAngleForwardContact: the ground normal is used for the forward test of a stair walk when it is within
// 75° of the walk, else the walk itself (mWalkStairsCosAngleForwardContact of Jolt)
var characterCosAngleForwardContact = math.Cos(75 * math.Pi / 180)

// characterUp: the up direction of every character: Feather is Y-up
var characterUp = mgl64.Vec3{0, 1, 0}

// CharacterGroundState: what a character stands on (EGroundState of Jolt)
type CharacterGroundState uint8

const (
	// CharacterInAir: the character touches nothing
	CharacterInAir CharacterGroundState = iota
	// CharacterNotSupported: the character touches a body, but it doesn't hold it (a wall, a ceiling): it falls
	CharacterNotSupported
	// CharacterOnSteepGround: the character stands on a slope too steep: it can't climb it, it slides down under its
	// gravity
	CharacterOnSteepGround
	// CharacterOnGround: the character stands on the ground and can walk
	CharacterOnGround
)

func (s CharacterGroundState) String() string {
	switch s {
	case CharacterInAir:
		return "in the air"
	case CharacterNotSupported:
		return "not supported"
	case CharacterOnSteepGround:
		return "on steep ground"
	}
	return "on the ground"
}

// CharacterVirtual: a capsule which walks. Create it with NewCharacterVirtual, add it to a world with World.AddCharacter,
// give it its Velocity and call Update after each step. Position is the center of the capsule. The name is the one of
// Jolt, which keeps Character for its character on a rigid body, moved by the solver; Feather has the virtual one
// only (PxController of PhysX, the mover of Box2D and CharacterBody3D of Godot are virtual too)
type CharacterVirtual struct {
	// Id is for the game (an entity id); the inner body keeps its own
	Id any

	// Capsule of the character, standing (its axis is up). Its radius and its half height can change between 2
	// updates (a crouch)
	Capsule actor.Capsule

	// Position of the center of the capsule, moved by Update. Write it through Teleport
	Position mgl64.Vec3

	// Velocity the character wants, written by the game before Update (its input, the gravity of the step and the
	// velocity of the ground: see the recipe in PHYSICS_GUIDE.md). Update cancels the part of it towards the slopes
	// too steep, and leaves the rest: the vertical velocity of a fall accumulates in it
	Velocity mgl64.Vec3

	// MaxSlopeAngle: the slope the character climbs (rad). Steeper, a surface is a wall: it is not climbed, the
	// character slides down it
	MaxSlopeAngle float64

	// StepHeight: the step the character climbs without jumping (m), 0 to climb no step
	StepHeight float64

	// MaxStrength: the force the character pushes the dynamic bodies with (N): the impulse of an update is at most
	// MaxStrength * dt. 0 pushes nothing
	MaxStrength float64

	// Mass of the character (kg): its weight on the dynamic body it stands on. 0 weighs nothing
	Mass float64

	// StickToFloor: how far down the floor is looked for when the character leaves the ground without going up (m):
	// it is moved down onto it, instead of flying off a step or a slope it walks down. 0 sticks to nothing
	StickToFloor float64

	world *World
	// body: the inner kinematic body, at the capsule of the character; index: its index in World.Bodies, checked
	// before use (RemoveBody moves the bodies)
	body  *actor.RigidBody
	index int

	// padded: the capsule inflated by the padding, the shape of the contacts and the sweeps; mover: a body for the
	// contact generator, at the padded capsule
	padded actor.Capsule
	mover  actor.RigidBody
	// cosMaxSlope: cos(MaxSlopeAngle), computed at each update
	cosMaxSlope float64
	// allowSliding: the character may slide along a walkable surface during this update: it has a horizontal
	// velocity, or it is not supported. Without it, it stops dead on the walkable surfaces it hits (the character
	// of the samples of Jolt, CharacterVirtualTest::OnContactSolve: no creeping down a slope while standing; the
	// floor_stop_on_slope of Godot). The velocity of the game then holds the gravity of the step alone
	allowSliding bool

	// the ground, after the last update, and the dt of the last update (the velocity of a turning platform is the one
	// of its arc over dt)
	ground characterGround
	lastDt float64

	// contacts: the active contacts of the last move (the contacts of the capsule, those which blocked it marked);
	// gathered: the contacts of the current position during a move
	contacts, gathered []characterContact
	// the buffers of a move (character_move.go)
	constraints []characterConstraint
	sorted      []int32
	previous    []int32
	excluded    []*actor.RigidBody
	steep       []mgl64.Vec3
	candidates  []int32
	stack       []int32
	manifolds   [MaxManifoldsPerPair]constraint.Manifold
	// the buffers of its contacts and of its sweeps, its own and not those of the pools of the world: a pool emptied by
	// the garbage collector gives new buffers, which grow again, and an update would allocate (made by AddCharacter)
	contactBuffers *contactScratch
	queryBuffers   *queryScratch
}

// characterGround: what the character stands on (the ground properties of CharacterVirtual)
type characterGround struct {
	state    CharacterGroundState
	normal   mgl64.Vec3
	velocity mgl64.Vec3
	body     *actor.RigidBody
	position mgl64.Vec3
}

// NewCharacterVirtual: a character of the capsule, standing at the position (the center of the capsule), with the
// defaults of Jolt: a slope of 50°, a step of 0.4 m, a strength of 100 N, a mass of 70 kg, the floor kept within 0.5 m.
// Add it to a world with World.AddCharacter
func NewCharacterVirtual(capsule actor.Capsule, position mgl64.Vec3) *CharacterVirtual {
	return &CharacterVirtual{
		Capsule:       capsule,
		Position:      position,
		MaxSlopeAngle: DefaultCharacterMaxSlopeAngle,
		StepHeight:    DefaultCharacterStepHeight,
		MaxStrength:   DefaultCharacterMaxStrength,
		Mass:          DefaultCharacterMass,
		StickToFloor:  DefaultCharacterStickToFloor,
	}
}

// AddCharacter adds the character to the world: its inner kinematic body is created at its capsule (Body), on the
// default layer with every layer in its mask (World.SetFilter changes them: the character collides with the bodies
// the pair rules let it collide with, as #800). A character of a zero capsule panics. The character is seen by the
// queries at the next Step or SyncQueries, as any body added
func (w *World) AddCharacter(c *CharacterVirtual) {
	if c.world != nil {
		panic("feather: the character is already in a world")
	}
	if !(c.Capsule.Radius > 0) {
		panic("feather: a character needs a capsule of a positive radius")
	}
	density := 0.0
	if volume := c.Capsule.ComputeMass(1); volume > 0 {
		density = c.Mass / volume
	}
	c.body = actor.NewRigidBody(actor.Transform{Position: c.Position, Rotation: mgl64.QuatIdent()}, &c.Capsule, actor.BodyTypeKinematic, density)
	c.world = w
	c.index = len(w.Bodies)
	c.mover = actor.RigidBody{Transform: actor.NewTransform(), BodyType: actor.BodyTypeStatic, Shape: &c.padded}
	c.ground = characterGround{}
	c.contacts = c.contacts[:0]
	if c.contactBuffers == nil {
		c.contactBuffers, c.queryBuffers = newContactScratch(), newQueryScratch()
		c.queryBuffers.epa = &c.contactBuffers.epa
	}
	w.AddBody(c.body)
	if w.characters == nil {
		w.characters = make(map[*actor.RigidBody]*CharacterVirtual)
	}
	w.characters[c.body] = c
}

// RemoveCharacter removes the character and its inner body from the world
func (w *World) RemoveCharacter(c *CharacterVirtual) {
	if c.world != w {
		return
	}
	w.RemoveBody(c.body)
	c.world, c.body = nil, nil
}

// CharacterOf: the character whose inner body this is, nil for any other body (the body hit by a ray)
func (w *World) CharacterOf(body *actor.RigidBody) *CharacterVirtual {
	return w.characters[body]
}

// Body: the inner kinematic body of the character, nil before AddCharacter. Its layer and its mask are the ones of
// the character (World.SetFilter); don't move it, the character does
func (c *CharacterVirtual) Body() *actor.RigidBody {
	return c.body
}

// GroundState after the last update
func (c *CharacterVirtual) GroundState() CharacterGroundState {
	return c.ground.state
}

// IsSupported: the character stands on something, the ground or a slope too steep (IsSupported of Jolt)
func (c *CharacterVirtual) IsSupported() bool {
	return c.ground.state == CharacterOnGround || c.ground.state == CharacterOnSteepGround
}

// GroundNormal: the normal of the ground, the average of the contacts which hold the character, zero in the air
func (c *CharacterVirtual) GroundNormal() mgl64.Vec3 {
	return c.ground.normal
}

// GroundVelocity: the velocity of the ground under the character (a platform), zero on a static ground or in the air.
// The game adds it to the velocity of the character to be carried
func (c *CharacterVirtual) GroundVelocity() mgl64.Vec3 {
	return c.ground.velocity
}

// UpdateGroundVelocity reads again the velocity of the ground from its body, without any collision test: the platform
// the character stands on may have changed its velocity since the last update (UpdateGroundVelocity of Jolt, called
// by its samples before they compute the velocity of the character: the character then follows a platform which
// starts without a frame of lag)
func (c *CharacterVirtual) UpdateGroundVelocity() {
	ground := c.ground.body
	if ground == nil || ground.BodyType == actor.BodyTypeStatic {
		return
	}
	if other := c.world.characters[ground]; other != nil {
		c.ground.velocity = other.Velocity
		return
	}
	c.ground.velocity = c.groundVelocityOn(ground, c.lastDt)
}

// GroundBody: the body the character stands on (or touches, when not supported), nil in the air
func (c *CharacterVirtual) GroundBody() *actor.RigidBody {
	return c.ground.body
}

// GroundPosition: the point of the ground the character stands on
func (c *CharacterVirtual) GroundPosition() mgl64.Vec3 {
	return c.ground.position
}

// IsSlopeTooSteep: a surface of this normal is steeper than MaxSlopeAngle: a wall for the character
func (c *CharacterVirtual) IsSlopeTooSteep(normal mgl64.Vec3) bool {
	return normal.Dot(characterUp) < math.Cos(c.MaxSlopeAngle)
}

// mustBeInWorld: the character was added to a world
func (c *CharacterVirtual) mustBeInWorld() {
	if c.world == nil {
		panic("feather: the character is in no world: World.AddCharacter first")
	}
}

// prepare the capsules and the cosine of the update
func (c *CharacterVirtual) prepare() {
	c.cosMaxSlope = math.Cos(c.MaxSlopeAngle)
	c.padded = actor.Capsule{HalfHeight: c.Capsule.HalfHeight, Radius: c.Capsule.Radius + CharacterPadding}
}

// Update moves the character for dt at its Velocity, between 2 steps of the world (during a step it panics, as a
// query). Call it after World.Step: the velocity of a platform is the one of the step just done, and the character
// follows it without lag.
//
// As CharacterVirtual::ExtendedUpdate of Jolt: the part of the velocity towards the slopes too steep is cancelled;
// the capsule slides along the contacts and sweeps; the ground is read from the contacts, and the weight of the
// character is put on a dynamic ground; if it left the ground without going up it sticks to the floor within
// StickToFloor; if the horizontal step it wanted was blocked by a steep contact, it walks a stair of StepHeight at
// most. Then the inner body is placed at the capsule, without velocity, awake when it moved so that
// the bodies which rest on it follow
func (c *CharacterVirtual) Update(dt float64) {
	c.mustBeInWorld()
	c.world.guard()
	if dt <= 0 {
		return
	}
	c.prepare()
	c.lastDt = dt

	desired := c.Velocity
	c.allowSliding = horizontal(desired).LenSqr() > 0 || !c.IsSupported()
	velocity := c.cancelVelocityTowardsSteepSlopes(desired)
	c.Velocity = velocity
	old := c.Position
	groundToAir := c.IsSupported()

	// the move, and the ground after it
	c.moveShape(&c.Position, velocity, dt, true)
	c.updateSupportingContact(false, dt)
	c.weighOnTheGround(dt)
	if c.IsSupported() {
		groundToAir = false
	}

	// stick to the floor when it left the ground without going up (ExtendedUpdate of Jolt)
	if groundToAir && c.StickToFloor > 0 {
		if vertical := c.Position.Sub(old).Dot(characterUp) / dt; vertical <= 1e-6 {
			c.stickToFloor(characterUp.Mul(-c.StickToFloor), dt)
		}
	}

	// walk a stair when the step wanted was blocked
	if c.StepHeight > 0 {
		wanted := horizontal(desired.Mul(dt))
		if wantedLength := wanted.Len(); wantedLength > 0 {
			forward := wanted.Mul(1 / wantedLength)
			// only the motion towards the step counts: sliding downhill while climbing is not progress
			achieved := math.Max(0, horizontal(c.Position.Sub(old)).Dot(forward))
			if achieved+characterMinTimeRemaining < wantedLength && c.canWalkStairs(desired) {
				stepForward := forward.Mul(math.Max(characterMinStepForward, wantedLength-achieved))
				// the floor ahead is tested along the ground normal in the horizontal plane, or along the walk when
				// they differ by more than 75°
				test := horizontal(c.ground.normal.Mul(-1))
				if length := test.Len(); length > 0 && test.Dot(forward)/length >= characterCosAngleForwardContact {
					test = test.Mul(characterStepForwardTest / length)
				} else {
					test = forward.Mul(characterStepForwardTest)
				}
				c.walkStairs(dt, characterUp.Mul(c.StepHeight), stepForward, test)
			}
		}
	}

	c.placeBody(c.Position != old)
}

// Teleport puts the character at the position, with its inner body, and reads its contacts there: nothing is pushed,
// the character placed in a body is pushed out at the next updates (as SetPosition then RefreshContacts of Jolt)
func (c *CharacterVirtual) Teleport(position mgl64.Vec3) {
	c.mustBeInWorld()
	c.world.guard()
	c.prepare()
	c.Position = position
	c.placeBody(true)
	c.refreshContacts()
}

// refreshContacts: the contacts at the position, and the ground among them (RefreshContacts of Jolt)
func (c *CharacterVirtual) refreshContacts() {
	direction := c.Velocity
	if length := direction.Len(); length > 0 {
		direction = direction.Mul(1 / length)
	}
	c.collectContacts(c.Position, direction)
	c.contacts = append(c.contacts[:0], c.gathered...)
	c.updateSupportingContact(true, 0)
}

// placeBody: the inner body takes the capsule of the character, without velocity (the inner body of Jolt is placed
// with SetPositionAndRotation, DontActivate). Its leaf in the tree of the broad phase follows at once, for the sweeps
// of the other characters. When it moved it wakes up with its island: the bodies resting on it must follow it
func (c *CharacterVirtual) placeBody(moved bool) {
	w, body := c.world, c.body
	body.Transform.Position = c.Position
	body.Velocity, body.AngularVelocity = mgl64.Vec3{}, mgl64.Vec3{}
	body.ClearKinematicTarget()
	body.UpdateAABB()
	if c.index >= len(w.Bodies) || w.Bodies[c.index] != body {
		c.index = -1
		for i, other := range w.Bodies {
			if other == body {
				c.index = i
				break
			}
		}
	}
	if c.index >= 0 && c.index < len(w.tree.proxies) && w.tree.bodies[c.index] == body {
		aabb := body.AABB()
		margin := mgl64.Vec3{SpeculativeDistance, SpeculativeDistance, SpeculativeDistance}
		w.tree.update(int32(c.index), body, actor.AABB{Min: aabb.Min.Sub(margin), Max: aabb.Max.Add(margin)})
	}
	if moved {
		w.islands.wake(body)
	}
}

// weighOnTheGround: the weight of the character during dt goes to the dynamic body it stands on, as an impulse at the
// ground point (CharacterVirtual::Update of Jolt)
func (c *CharacterVirtual) weighOnTheGround(dt float64) {
	ground := c.ground.body
	if ground == nil || ground.BodyType != actor.BodyTypeDynamic || !(c.Mass > 0) {
		return
	}
	gravity := c.world.Gravity
	length := gravity.Len()
	if length == 0 {
		return
	}
	along := c.ground.normal.Dot(gravity)
	if along >= 0 {
		return
	}
	ground.AddImpulseAtPoint(gravity.Mul(-c.Mass*along/length*dt), c.ground.position)
}

// horizontal: the vector without its vertical component
func horizontal(v mgl64.Vec3) mgl64.Vec3 {
	return v.Sub(characterUp.Mul(v.Dot(characterUp)))
}

// cancelVelocityTowardsSteepSlopes: on a slope too steep, the horizontal velocity towards the steep contacts is
// removed, so that the character doesn't try to climb (CancelVelocityTowardsSteepSlopes of Jolt)
func (c *CharacterVirtual) cancelVelocityTowardsSteepSlopes(desired mgl64.Vec3) mgl64.Vec3 {
	if c.ground.state == CharacterOnGround || c.ground.state == CharacterInAir {
		return desired
	}
	velocity := desired
	for i := range c.contacts {
		contact := &c.contacts[i]
		if !contact.hadCollision || contact.wasDiscarded || !c.tooSteep(contact.normal) {
			continue
		}
		normal := horizontal(contact.normal)
		if dot := normal.Dot(velocity); dot < 0 {
			velocity = velocity.Sub(normal.Mul(dot / normal.LenSqr()))
		}
	}
	return velocity
}

// tooSteep: the normal is the one of a surface the character can't climb, with the cosine of the update
func (c *CharacterVirtual) tooSteep(normal mgl64.Vec3) bool {
	return normal.Dot(characterUp) < c.cosMaxSlope
}

// updateSupportingContact: the ground among the active contacts (UpdateSupportingContact of Jolt). A contact close
// enough and not left (or any contact, with skipVelocityCheck) is a collision; a contact holds the character if its
// point is in the lower sphere of the capsule (the supporting volume of the samples of Jolt: Plane(Y, -radius)); the
// character is on the ground if a holding contact is under the slope limit, on steep ground if it only has steep
// ones, unless they block it together (a crevice), not supported if it touches without being held, else in the air.
// The ground normal and velocity are the average of the contacts within 85° of up
func (c *CharacterVirtual) updateSupportingContact(skipVelocityCheck bool, dt float64) {
	for i := range c.contacts {
		contact := &c.contacts[i]
		if !contact.wasDiscarded && !contact.hadCollision && contact.distance < characterCollisionTolerance &&
			(skipVelocityCheck || contact.normal.Dot(c.Velocity.Sub(contact.velocity)) <= 1e-4) {
			contact.hadCollision = true
		}
	}

	supported, sliding, averaged := 0, 0, 0
	var normal, velocity mgl64.Vec3
	supporting, deepest := -1, -1
	maxCos, smallest := math.Inf(-1), math.Inf(1)
	for i := range c.contacts {
		contact := &c.contacts[i]
		if !contact.hadCollision || contact.wasDiscarded {
			continue
		}
		cos := contact.normal.Dot(characterUp)
		if contact.distance < smallest {
			deepest, smallest = i, contact.distance
		}
		// a contact above the center of the lower sphere can't hold the character
		if contact.position.Sub(c.Position).Dot(characterUp) > -c.Capsule.HalfHeight {
			continue
		}
		if cos > maxCos {
			supporting, maxCos = i, cos
		}
		holds := cos >= c.cosMaxSlope
		if holds {
			supported++
		} else {
			sliding++
		}
		if cos >= characterGroundNormalCos {
			normal = normal.Add(contact.normal)
			averaged++
			if contact.body != nil && contact.body.BodyType == actor.BodyTypeKinematic && holds && contact.character == nil && dt > 0 {
				// a kinematic platform: the velocity of the point of the platform under the character, on its arc
				velocity = velocity.Add(c.groundVelocityOn(contact.body, dt))
			} else {
				velocity = velocity.Add(contact.velocity)
			}
		}
	}

	best := supporting
	if best < 0 {
		best = deepest
	}
	switch {
	case averaged >= 1:
		c.ground.normal, c.ground.velocity = normal.Normalize(), velocity.Mul(1/float64(averaged))
	case best >= 0:
		c.ground.normal, c.ground.velocity = c.contacts[best].normal, c.contacts[best].velocity
	default:
		c.ground.normal, c.ground.velocity = mgl64.Vec3{}, mgl64.Vec3{}
	}
	if best >= 0 {
		c.ground.body, c.ground.position = c.contacts[best].body, c.contacts[best].position
	} else {
		c.ground.body, c.ground.position = nil, mgl64.Vec3{}
	}

	switch {
	case supported > 0:
		c.ground.state = CharacterOnGround
	case sliding > 0:
		if c.Velocity.Sub(c.contacts[deepest].velocity).Dot(characterUp) > 1e-4 {
			// going up relative to the ground: not on it
			c.ground.state = CharacterOnSteepGround
		} else {
			// sliding down between several steep contacts which block each other: held
			c.gathered = append(c.gathered[:0], c.contacts...)
			c.determineConstraints(1)
			displacement, simulated := c.solveConstraints(characterUp.Mul(-1), 1, 1)
			if simulated < 0.001 || displacement.LenSqr() < (0.6*dt)*(0.6*dt) {
				c.ground.state = CharacterOnGround
			} else {
				c.ground.state = CharacterOnSteepGround
			}
		}
	case best >= 0:
		c.ground.state = CharacterNotSupported
	default:
		c.ground.state = CharacterInAir
	}
}

// groundVelocityOn: the velocity of the point of the body under the character, over dt: where the rotation of the
// body during dt brings this point, so that the character follows a turning platform without drifting
// (CalculateCharacterGroundVelocity of Jolt)
func (c *CharacterVirtual) groundVelocityOn(body *actor.RigidBody, dt float64) mgl64.Vec3 {
	angular := body.AngularVelocity
	rate := angular.Len()
	if rate*rate < 1e-12 || dt <= 0 {
		return body.Velocity
	}
	rotation := mgl64.QuatRotate(rate*dt, angular.Mul(1/rate))
	center := body.Transform.Position
	moved := center.Add(rotation.Rotate(c.Position.Sub(center)))
	return body.Velocity.Add(moved.Sub(c.Position).Mul(1 / dt))
}

// stickToFloor: the floor is looked for under the character, and the character moved onto it (StickToFloor of Jolt)
func (c *CharacterVirtual) stickToFloor(down mgl64.Vec3, dt float64) bool {
	c.excluded = append(c.excluded[:0], c.body)
	hit, ok := c.sweep(c.Position, down)
	if !ok {
		return false
	}
	c.moveToContact(c.Position.Add(down.Mul(hit.Fraction)))
	return true
}

// moveToContact: the character is placed, and its contacts and its ground read there (MoveToContact of Jolt)
func (c *CharacterVirtual) moveToContact(position mgl64.Vec3) {
	c.Position = position
	c.refreshContacts()
}

// canWalkStairs: the character is supported, moves horizontally and pushes into a contact too steep (CanWalkStairs)
func (c *CharacterVirtual) canWalkStairs(velocity mgl64.Vec3) bool {
	if !c.IsSupported() {
		return false
	}
	flat := horizontal(velocity)
	if flat.LenSqr() < 1e-12 {
		return false
	}
	for i := range c.contacts {
		contact := &c.contacts[i]
		if contact.hadCollision && !contact.wasDiscarded && contact.normal.Dot(flat.Sub(contact.velocity)) < 0 && c.tooSteep(contact.normal) {
			return true
		}
	}
	return false
}

// walkStairs: the character goes up by stepUp at most, forward by stepForward, and down onto the floor; the walk is
// cancelled if nothing was gained against the steep contacts, if there is no floor, or if the floor is too steep
// where it lands and at stepForwardTest further (WalkStairs of Jolt). Returns true if the character walked the stair
func (c *CharacterVirtual) walkStairs(dt float64, stepUp, stepForward, stepForwardTest mgl64.Vec3) bool {
	c.excluded = append(c.excluded[:0], c.body)
	up := stepUp
	if hit, ok := c.sweep(c.Position, up); ok {
		if hit.Fraction < 1e-6 {
			return false
		}
		up = up.Mul(hit.Fraction)
	}
	upPosition := c.Position.Add(up)

	// the steep contacts the character pushes into, before the move changes the contacts
	velocity := stepForward.Mul(1 / dt)
	flat := horizontal(velocity)
	c.steep = c.steep[:0]
	for i := range c.contacts {
		contact := &c.contacts[i]
		if contact.hadCollision && !contact.wasDiscarded && contact.normal.Dot(flat.Sub(contact.velocity)) < 0 && c.tooSteep(contact.normal) {
			c.steep = append(c.steep, contact.normal)
		}
	}
	if len(c.steep) == 0 {
		return false
	}

	// forward, from the top of the step
	newPosition := upPosition
	c.moveShape(&newPosition, velocity, dt, false)
	movement := newPosition.Sub(upPosition)
	movementSqr := movement.LenSqr()
	if movementSqr < 1e-8 {
		return false
	}
	progress := false
	for _, normal := range c.steep {
		if normal.Dot(movement) < -characterProgressFraction*stepForward.Len() {
			progress = true
			break
		}
	}
	if !progress {
		return false
	}

	// down onto the floor, as much as the character went up
	down := up.Mul(-1)
	c.excluded = append(c.excluded[:0], c.body)
	hit, ok := c.sweep(newPosition, down)
	if !ok {
		return false
	}
	if c.tooSteep(hit.Normal) {
		// the edge of the step may be hit with a normal too horizontal: the floor is tested further along the step
		if stepForwardTest.LenSqr() < 1e-12 {
			return false
		}
		testPosition := upPosition
		c.moveShape(&testPosition, stepForwardTest.Mul(1/dt), dt, false)
		if testPosition.Sub(upPosition).LenSqr() <= movementSqr+1e-8 {
			return false
		}
		c.excluded = append(c.excluded[:0], c.body)
		testHit, ok := c.sweep(testPosition, down)
		if !ok || c.tooSteep(testHit.Normal) {
			return false
		}
	}
	c.moveToContact(newPosition.Add(down.Mul(hit.Fraction)))
	// the floor further along is not too steep: the character is on the ground
	c.ground.state = CharacterOnGround
	return true
}
