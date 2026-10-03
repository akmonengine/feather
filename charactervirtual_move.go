package feather

import (
	"math"
	"slices"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== THE MOVE OF A CHARACTER ==========
// The move of a character is the one of CharacterVirtual::MoveShape of Jolt: the contacts of the padded capsule with
// everything within the predictive distance (collectContacts: the contact generator of the world, GJK/EPA and the
// patches of the surfaces, one plane per manifold), the planes made constraints with the velocity of their body
// (determineConstraints, a vertical plane added for a slope too steep), the velocity slid along the planes by time
// of impact (solveConstraints), a sweep to check that nothing was missed (sweep), and again while time remains, 5
// times at most. A dynamic body met receives an impulse (handleContact), bounded by the strength of the character.

// characterContact: a contact of the capsule of the character with a body (the Contact of CharacterVirtual)
type characterContact struct {
	// body: the body touched; character: the other character when the body is its inner body
	body      *actor.RigidBody
	character *CharacterVirtual
	// position on the body; normal from the body towards the character; distance between them, the padding removed
	// (negative when the padded capsule enters the body)
	position mgl64.Vec3
	normal   mgl64.Vec3
	distance float64
	// velocity of the body at the point (the velocity of the other character)
	velocity mgl64.Vec3
	// hadCollision: the contact blocked the character; wasDiscarded: a conflicting contact, ignored by the sweeps;
	// canPush: the body can push the character (always: the setting of Jolt is left to its listener)
	hadCollision, wasDiscarded bool
}

// characterConstraint: a plane the character can't cross, with its velocity (the Constraint of CharacterVirtual).
// The plane is normal · x + distance = 0 at the position of the character
type characterConstraint struct {
	contact  int32
	velocity mgl64.Vec3
	normal   mgl64.Vec3
	distance float64
	// steep: the plane is a slope too steep, a vertical constraint follows it
	steep bool
	// toi: the time of impact of the velocity on the plane, projected: the speed of approach (solveConstraints)
	toi, projected float64
}

// collectContacts: the contacts of the padded capsule at the position with the bodies within the predictive
// distance (GetContactsAtPosition of Jolt): the candidates of the trees for its AABB, filtered as the pairs of the
// world (World.ShouldCollide: the layers, the ignored pairs), the triggers skipped, then the contact generator of the
// world with the predictive distance as margin, one contact per manifold. The contacts are sorted by the index of
// their body: the same contacts whatever the trees
func (c *CharacterVirtual) collectContacts(position, direction mgl64.Vec3) {
	w := c.world
	c.gathered = c.gathered[:0]
	c.mover.Transform.Position = position
	c.mover.UpdateAABB()
	bounds := c.mover.AABB()
	margin := mgl64.Vec3{characterPredictiveDistance, characterPredictiveDistance, characterPredictiveDistance}
	bounds = actor.AABB{Min: bounds.Min.Sub(margin), Max: bounds.Max.Add(margin)}
	c.stack, c.candidates = w.tree.queryCandidates(bounds, c.stack, c.candidates[:0])
	slices.Sort(c.candidates)
	for _, index := range c.candidates {
		body := w.Bodies[index]
		if body == c.body || body.IsTrigger || !w.ShouldCollide(c.body, body) || !bounds.Overlaps(body.AABB()) {
			continue
		}
		count := collideAllMoving(c.contactBuffers, &c.mover, body, characterPredictiveDistance, direction, c.manifolds[:])
		for k := 0; k < count; k++ {
			c.addContact(body, &c.manifolds[k])
		}
	}
}

// addContact: the deepest point of the manifold, its normal towards the character, the velocity of the body there
func (c *CharacterVirtual) addContact(body *actor.RigidBody, m *constraint.Manifold) {
	deepest := 0
	for j := 1; j < m.Count; j++ {
		if m.Points[j].Separation < m.Points[deepest].Separation {
			deepest = j
		}
	}
	point := &m.Points[deepest]
	// the manifold goes from the mover to the body: the point on the body is half the separation further
	position := point.Position.Add(m.Normal.Mul(point.Separation / 2))
	contact := characterContact{body: body, position: position, normal: m.Normal.Mul(-1), distance: point.Separation}
	if other := c.world.characters[body]; other != nil {
		// the other character moves by itself: its velocity is the one it wants (Jolt sFillCharacterContactProperties)
		contact.character, contact.velocity = other, other.Velocity
	} else if body.BodyType != actor.BodyTypeStatic {
		contact.velocity = body.Velocity.Add(body.AngularVelocity.Cross(position.Sub(body.Transform.Position)))
	}
	c.gathered = append(c.gathered, contact)
}

// removeConflictingContacts: 2 contacts of the same body which both enter it deeper than the conflict depth with
// opposed normals can't both be solved (the character is squeezed in the body): the shallower one is discarded,
// and ignored by the sweeps (RemoveConflictingContacts of Jolt)
func (c *CharacterVirtual) removeConflictingContacts() {
	c.excluded = append(c.excluded[:0], c.body)
	for i := range c.gathered {
		first := &c.gathered[i]
		if first.wasDiscarded || first.distance > -characterConflictDepth {
			continue
		}
		for j := i + 1; j < len(c.gathered); j++ {
			second := &c.gathered[j]
			if second.wasDiscarded || second.body != first.body || second.distance > -characterConflictDepth || first.normal.Dot(second.normal) >= 0 {
				continue
			}
			if first.distance < second.distance {
				second.wasDiscarded = true
				c.excluded = append(c.excluded, second.body)
			} else {
				first.wasDiscarded = true
				c.excluded = append(c.excluded, first.body)
				break
			}
		}
	}
}

// determineConstraints: the planes of the gathered contacts (DetermineConstraints of Jolt). A contact which enters
// the body gets a velocity out of it (the penetration recovery); a slope too steep gets a second, vertical plane
// which blocks the way up it
func (c *CharacterVirtual) determineConstraints(dt float64) {
	c.constraints = c.constraints[:0]
	for i := range c.gathered {
		contact := &c.gathered[i]
		if contact.wasDiscarded {
			continue
		}
		velocity := contact.velocity
		if contact.distance < 0 {
			velocity = velocity.Sub(contact.normal.Mul(contact.distance * characterPenetrationRecovery / dt))
		}
		c.constraints = append(c.constraints, characterConstraint{contact: int32(i), velocity: velocity, normal: contact.normal, distance: contact.distance})
		if !c.tooSteep(contact.normal) {
			continue
		}
		// a plane which points up: a vertical plane blocks the way up the slope (a horizontal one is already vertical;
		// a normal within the rounding of up has no horizontal part to make a vertical plane of)
		dot := contact.normal.Dot(characterUp)
		normal := contact.normal.Sub(characterUp.Mul(dot))
		if dot <= 1e-3 || normal.LenSqr() < characterVerticalEpsilon*characterVerticalEpsilon {
			continue
		}
		c.constraints[len(c.constraints)-1].steep = true
		normal = normal.Normalize()
		// the velocity projected on the vertical normal: both planes push at the same rate; the distance is the one
		// to travel horizontally to reach the contact plane
		c.constraints = append(c.constraints, characterConstraint{
			contact: int32(i), velocity: normal.Mul(velocity.Dot(normal)), normal: normal, distance: contact.distance / normal.Dot(contact.normal),
		})
	}
}

// handleContact: the character hits the contact. A dynamic body receives an impulse which brings it to the velocity
// of the character along the normal (damped, plus a part of the penetration), bounded by the strength of the
// character, never downwards (HandleContact of Jolt; the gravity is the world's job). Returns false if the contact
// is not to be solved (never here: Jolt leaves it to its listener)
func (c *CharacterVirtual) handleContact(velocity mgl64.Vec3, constraint *characterConstraint, dt float64) bool {
	contact := &c.gathered[constraint.contact]
	contact.hadCollision = true
	body := contact.body
	if body.BodyType != actor.BodyTypeDynamic || !(c.MaxStrength > 0) {
		return true
	}
	relative := velocity.Sub(contact.velocity)
	projected := relative.Dot(contact.normal)
	delta := -projected*characterPushDamping - math.Min(contact.distance, 0)*characterPushPenetration/dt
	if delta < 0 {
		// separating
		return true
	}
	// the inverse mass of the body at the point along the normal
	lever := contact.position.Sub(body.Transform.Position).Cross(contact.normal)
	inverseMasses := body.InverseMassAxes()
	inverseMass := 0.0
	for k := 0; k < 3; k++ {
		inverseMass += inverseMasses[k] * contact.normal[k] * contact.normal[k]
	}
	inverseInertia := body.GetInverseInertiaWorld()
	effective := inverseInertia.Mul3x1(lever).Dot(lever) + inverseMass
	if effective <= 0 {
		return true
	}
	impulse := math.Min(delta/effective, c.MaxStrength*dt)
	world := contact.normal.Mul(-impulse)
	if down := world.Dot(characterUp); down < 0 {
		world = world.Sub(characterUp.Mul(down))
	}
	body.AddImpulseAtPoint(world, contact.position)
	return true
}

// solveConstraints: the displacement of the character at the velocity during remaining, slid along the constraints
// (SolveConstraints of Jolt). At each of the iterations the constraints are ordered by their time of impact (the
// pushing ones first at t = 0, the static bodies first at a tie), the character goes to the first one, its velocity
// is projected on the plane (a steep slope first loses the velocity towards it), and if the new velocity would enter
// a plane hit earlier the character slides along the edge of both. It stops when no velocity is left, or when the
// velocity reversed. Returns the displacement and the time simulated
func (c *CharacterVirtual) solveConstraints(velocity mgl64.Vec3, dt, remaining float64) (mgl64.Vec3, float64) {
	if len(c.constraints) == 0 {
		return velocity.Mul(remaining), remaining
	}
	c.sorted = c.sorted[:0]
	for i := range c.constraints {
		c.sorted = append(c.sorted, int32(i))
	}
	last := velocity
	var displacement mgl64.Vec3
	simulated := 0.0
	c.previous = c.previous[:0]

	for iteration := 0; iteration < characterMaxConstraintIterations; iteration++ {
		for i := range c.constraints {
			k := &c.constraints[i]
			k.projected = k.normal.Dot(k.velocity.Sub(velocity))
			if k.projected < 1e-6 {
				k.toi = math.Inf(1)
				continue
			}
			distance := k.normal.Dot(displacement) + k.distance
			if distance-k.projected*remaining > -characterAcceptedPenetration {
				// too little penetration: the movement is accepted
				k.toi = math.Inf(1)
			} else {
				k.toi = math.Max(0, distance/k.projected)
			}
		}
		slices.SortFunc(c.sorted, func(a, b int32) int {
			ka, kb := &c.constraints[a], &c.constraints[b]
			// both at t = 0: the one which pushes the most first (the deepest penetrations too)
			if ka.toi <= 0 && kb.toi <= 0 {
				return compare(kb.projected, ka.projected)
			}
			if ka.toi != kb.toi {
				return compare(ka.toi, kb.toi)
			}
			// a static body first: it has the most influence
			return compare(bodyTypeRank(c.gathered[kb.contact].body), bodyTypeRank(c.gathered[ka.contact].body))
		})

		// the first constraint reached
		var hit *characterConstraint
		hitIndex := int32(-1)
		for _, index := range c.sorted {
			k := &c.constraints[index]
			if k.toi >= remaining {
				// the goal is reached
				return displacement.Add(velocity.Mul(remaining)), simulated + remaining
			}
			contact := &c.gathered[k.contact]
			if contact.wasDiscarded {
				continue
			}
			if !contact.hadCollision && !c.handleContact(velocity, k, dt) {
				contact.wasDiscarded = true
				c.excluded = append(c.excluded, contact.body)
				continue
			}
			hit, hitIndex = k, index
			break
		}
		if hit == nil {
			return displacement.Add(velocity.Mul(remaining)), simulated + remaining
		}

		displacement = displacement.Add(velocity.Mul(hit.toi))
		remaining -= hit.toi
		simulated += hit.toi
		if remaining < characterMinTimeRemaining {
			return displacement, simulated
		}
		if hit.toi > 1e-4 {
			c.previous = c.previous[:0]
		}

		normal := hit.normal
		if hit.steep {
			// a steep slope: the velocity towards it goes first, else the character would climb a little and jitter
			vertical := normal.Sub(characterUp.Mul(normal.Dot(characterUp)))
			relative := velocity.Sub(hit.velocity)
			velocity = velocity.Sub(vertical.Mul(math.Min(0, relative.Dot(vertical)) / vertical.LenSqr()))
		}
		relative := velocity.Sub(hit.velocity)
		next := velocity.Sub(normal.Mul(relative.Dot(normal)))

		// the earlier plane the new velocity would enter the most
		var other *characterConstraint
		highest := 0.0
		for _, index := range c.previous {
			k := &c.constraints[index]
			if k == hit {
				continue
			}
			penetration := k.velocity.Sub(next).Dot(k.normal)
			if penetration > highest {
				if dot := k.normal.Dot(normal); dot < characterParallelCos && dot > -characterParallelCos {
					highest, other = penetration, k
				}
			}
		}
		if other != nil {
			// slide along the edge of both planes
			slide := normal.Cross(other.normal).Normalize()
			alongSlide := slide.Mul(next.Dot(slide))
			// neither plane pushes into the other anymore: no ping-pong
			hit.velocity = hit.velocity.Sub(other.normal.Mul(math.Min(0, hit.velocity.Dot(other.normal))))
			other.velocity = other.velocity.Sub(normal.Mul(math.Min(0, other.velocity.Dot(normal))))
			perpendicular := hit.velocity.Sub(slide.Mul(hit.velocity.Dot(slide)))
			otherPerpendicular := other.velocity.Sub(slide.Mul(other.velocity.Dot(slide)))
			next = alongSlide.Add(perpendicular).Add(otherPerpendicular)
		}
		if !c.allowSliding && hit.velocity.LenSqr() < 1e-16 && !c.tooSteep(hit.normal) {
			// standing without input on a walkable, still surface: the character doesn't creep down it
			next = mgl64.Vec3{}
		}
		velocity = next
		c.previous = append(c.previous, hitIndex)

		if hit.projected < 1e-8 && velocity.LenSqr() < 1e-8 {
			// nothing pushes, no velocity left
			return displacement, simulated
		}
		if hit.velocity.LenSqr() >= 1e-16 {
			last = hit.velocity
		} else if velocity.Dot(last) < 0 {
			// the velocity reversed
			return displacement, simulated
		}
	}
	return displacement, simulated
}

// compare for the sorts: -1, 0, 1
func compare(a, b float64) int {
	if a < b {
		return -1
	}
	if a > b {
		return 1
	}
	return 0
}

// bodyTypeRank: a static body first in a tie of the solver, then a kinematic one, then a dynamic one (the motion
// types of Jolt in order)
func bodyTypeRank(body *actor.RigidBody) float64 {
	switch body.BodyType {
	case actor.BodyTypeStatic:
		return 2
	case actor.BodyTypeKinematic:
		return 1
	}
	return 0
}

// moveShape: the character at position moves at the velocity for dt: contacts, constraints, solve, sweep, and
// again while time remains and the character moves (MoveShape of Jolt). With store, the contacts of the move are
// kept as the active contacts of the character
func (c *CharacterVirtual) moveShape(position *mgl64.Vec3, velocity mgl64.Vec3, dt float64, store bool) {
	direction := velocity
	if length := direction.Len(); length > 0 {
		direction = direction.Mul(1 / length)
	}
	remaining := dt
	for iteration := 0; iteration < characterMaxCollisionIterations && remaining >= characterMinTimeRemaining; iteration++ {
		c.collectContacts(*position, direction)
		c.removeConflictingContacts()
		c.determineConstraints(dt)
		displacement, simulated := c.solveConstraints(velocity, dt, remaining)
		if store {
			c.contacts = append(c.contacts[:0], c.gathered...)
		}
		if hit, ok := c.sweep(*position, displacement); ok {
			displacement = displacement.Mul(hit.Fraction)
			simulated *= hit.Fraction
		}
		*position = position.Add(displacement)
		remaining -= simulated
		if displacement.LenSqr() < 1e-8 {
			break
		}
	}
}

// sweep: the first body the padded capsule meets from the position along the displacement, among the bodies the
// character collides with and which are not excluded (GetFirstContactForSweep of Jolt). The bodies the capsule
// starts in or in contact with are passed through: their planes are solved, the sweep only checks what the contacts
// missed (the hits at the fraction 0 are ignored, as in ContactCastCollector of Jolt and b2World_CastMover of
// Box2D); a hit which would enter the body by less than the collision tolerance is ignored too
func (c *CharacterVirtual) sweep(position, displacement mgl64.Vec3) (Hit, bool) {
	if displacement.LenSqr() < 1e-8 {
		return Hit{}, false
	}
	scratch := c.queryBuffers
	filter := QueryFilter{Mask: c.body.Mask, Excluded: c.excluded}
	query := newSweepQuery(c.world, &c.padded, actor.Transform{Position: position, Rotation: mgl64.QuatIdent()}, displacement, filter, scratch)
	query.character = c
	query.run()
	best, found := query.best, query.found
	scratch.clear()
	return best, found
}

// sweepAccepts: the body is one the character collides with (its layer in the mask of the body: the other way of the
// pair rule, the mask of the character is in the filter of the query; and the pair not ignored)
func (c *CharacterVirtual) sweepAccepts(body *actor.RigidBody) bool {
	return !body.IsTrigger && body.Mask&c.body.Layer != 0 && c.world.filter.allows(c.body, body)
}

// sweepKeeps: the hit of a sweep of the character is kept: not at the start, and entering the body deeper than the
// collision tolerance by the end of the displacement
func (c *CharacterVirtual) sweepKeeps(hit *Hit, displacement mgl64.Vec3) bool {
	if hit.Fraction <= 0 {
		return false
	}
	return -hit.Normal.Dot(displacement)*(1-hit.Fraction) >= characterCollisionTolerance
}
