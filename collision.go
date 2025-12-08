package feather

import (
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
)

const STIFF_COMPLIANCE = CONCRETE_COMPLIANCE

const (
	CONCRETE_COMPLIANCE = 0.04e-9
	WOOD_COMPLIANCE     = 0.16e-9
	LEATHER_COMPLIANCE  = 14e-8
	TENDON_COMPLIANCE   = 0.2e-7
	RUBBER_COMPLIANCE   = 1e-6
	MUSCLE_COMPLIANCE   = 0.2e-3
	FAT_COMPLIANCE      = 1e-3
)

// CollisionPair represents a pair of rigid bodies that potentially collide
type CollisionPair struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

// BroadPhase performs broad-phase collision detection using AABB overlap tests
// It returns pairs of bodies whose AABBs overlap and might be colliding
// This is an O(n²) brute-force approach suitable for small numbers of bodies
func BroadPhase(bodies []*actor.RigidBody) []CollisionPair {
	pairs := make([]CollisionPair, 0)

	// Brute force: test all pairs
	for i := 0; i < len(bodies); i++ {
		for j := i + 1; j < len(bodies); j++ {
			bodyA := bodies[i]
			bodyB := bodies[j]

			// Skip if both bodies are static (static-static collisions don't matter)
			if bodyA.BodyType == actor.BodyTypeStatic && bodyB.BodyType == actor.BodyTypeStatic {
				continue
			}
			if bodyA.IsSleeping && bodyB.IsSleeping {
				continue
			}

			// Compute AABBs for both bodies
			aabbA := bodyA.Shape.GetAABB()
			aabbB := bodyB.Shape.GetAABB()

			// Check if AABBs overlap
			if aabbA.Overlaps(aabbB) {
				pairs = append(pairs, CollisionPair{bodyA, bodyB})
			}
		}
	}

	return pairs
}

func NarrowPhase(pairs []CollisionPair) []constraint.ContactConstraint {
	contacts := make([]constraint.ContactConstraint, 0)

	for _, pair := range pairs {
		if collision, simplex := gjk.GJK(pair.BodyA, pair.BodyB); collision {
			// Use EPA to get detailed contact information
			contact, err := epa.EPA(pair.BodyA, pair.BodyB, simplex)
			if err != nil {
				continue
			}

			//fmt.Printf("Contact normal: %v\n", contact.Normal)
			//fmt.Printf("Contact penetration: %.6f\n", contact.Point.Penetration)
			contacts = append(contacts, contact)
		}
	}

	return contacts
}
