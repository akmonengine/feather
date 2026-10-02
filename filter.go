package feather

import (
	"slices"

	"github.com/akmonengine/feather/actor"
)

// ========== COLLISION FILTERING ==========
// A pair of bodies is tested by the narrow phase if it passes 2 filters, both applied by the broad phase when it emits
// the pairs of the step (Tree.scan): a filtered pair costs no GJK, makes no contact, no event, and wakes nobody up.
//
//   - the layers: each body has a layer (one bit) and a mask, the layers it collides with. The pair collides if each
//     body has its layer in the mask of the other: the categoryBits & maskBits of Box2D (b2ShouldShapesCollide), the
//     group & mask bits of Bullet and of Jolt (ObjectLayerPairFilterMask). The signed group of Box2D (groupIndex) is not
//     taken: the ignored pairs do its job for 2 bodies, the layers for more
//   - the ignored pairs: a table of pairs of bodies which never collide, O(1) per pair (a hash of both pointers). A pair
//     is in the table when the game ignores it (World.IgnoreCollision: Physics.IgnoreCollision of Unity, the filter
//     joint of Box2D v3.1) or when a joint without CollideConnected links its bodies (collideConnected of Box2D,
//     eCOLLISION_ENABLED of PhysX). Both reasons share the table: a pair leaves it with its last reason
//
// The triggers and the continuous collision go through the same filters (World.ShouldCollide).
// A body of empty mask collides with nothing but stays in the tree of the broad phase with its layer, for the queries:
// the "Query Only" collision of Unreal, eSCENE_QUERY_SHAPE without eSIMULATION_SHAPE in PhysX

// ShouldCollide: each body has its layer in the mask of the other
func ShouldCollide(a, b *actor.RigidBody) bool {
	return a.Mask&b.Layer != 0 && b.Mask&a.Layer != 0
}

// pairRule: why a pair of bodies never collides
type pairRule struct {
	// joints without CollideConnected between the bodies
	joints int32
	// ignored by World.IgnoreCollision
	ignored bool
}

// pairFilter: the pairs of bodies which never collide. Only the pairs with a reason are in the table
type pairFilter struct {
	rules map[pairKey]pairRule
}

// allows the pair: it is not in the table. A nil filter allows every pair
func (f *pairFilter) allows(a, b *actor.RigidBody) bool {
	if f == nil || len(f.rules) == 0 {
		return true
	}
	_, filtered := f.rules[makePairKey(a, b)]
	return !filtered
}

// collides: the pair passes both filters, the layers of its bodies then the table
func (f *pairFilter) collides(a, b *actor.RigidBody) bool {
	return ShouldCollide(a, b) && f.allows(a, b)
}

// set the rule of the pair: a rule without any reason leaves the table
func (f *pairFilter) set(key pairKey, rule pairRule) {
	if rule.joints <= 0 && !rule.ignored {
		delete(f.rules, key)
		return
	}
	if f.rules == nil {
		f.rules = make(map[pairKey]pairRule)
	}
	f.rules[key] = rule
}

// ignore the pair (or not anymore), returns true if it changed
func (f *pairFilter) ignore(a, b *actor.RigidBody, ignored bool) bool {
	key := makePairKey(a, b)
	rule := f.rules[key]
	if rule.ignored == ignored {
		return false
	}
	rule.ignored = ignored
	f.set(key, rule)
	return true
}

// link the pair by count joints which filter it (negative when the joints leave)
func (f *pairFilter) link(a, b *actor.RigidBody, count int32) {
	key := makePairKey(a, b)
	rule := f.rules[key]
	rule.joints += count
	f.set(key, rule)
}

// forget the pairs of a removed body
func (f *pairFilter) forget(body *actor.RigidBody) {
	for key := range f.rules {
		if key.bodyA == body || key.bodyB == body {
			delete(f.rules, key)
		}
	}
}

// ShouldCollide: the layers of the bodies collide, the pair is not ignored and no joint filters it. The broad phase,
// the triggers and the continuous collision all use this test
func (w *World) ShouldCollide(a, b *actor.RigidBody) bool {
	return w.filter.collides(a, b)
}

// IgnoreCollision between 2 bodies, whatever their layers (or not anymore, with ignore false). The bodies wake up: a
// body resting on the other one falls. The pair is forgotten when one of its bodies is removed from the world
func (w *World) IgnoreCollision(a, b *actor.RigidBody, ignore bool) {
	if w.filter.ignore(a, b, ignore) {
		w.islands.wake(a)
		w.islands.wake(b)
	}
}

// SetFilter changes the layer and the mask of a body, and wakes it up with the sleeping bodies it collided with: they
// may have to fall (b2Shape_SetFilter of Box2D wakes the bodies too). Without any change nobody wakes up: it can be
// called at each frame. Writing Layer and Mask directly is the same without the wake up.
// The bodies the body collides with from now on are not woken up, as for a body added to the world: a sleeping body
// inside a static body which becomes solid stays asleep, and is pushed out when something else wakes it up
func (w *World) SetFilter(body *actor.RigidBody, layer, mask actor.Layers) {
	if body.Layer == layer && body.Mask == mask {
		return
	}
	w.wakeNeighbors(body)
	body.Layer, body.Mask = layer, mask
	w.islands.wake(body)
}

// QueryFilter of the world queries (ray, sweep, overlap): the layers they see, and the bodies they skip. The mask of a
// body is not read: a body which collides with nothing is still seen on its layer. The zero value sees nothing: start
// from DefaultQueryFilter
type QueryFilter struct {
	// Mask: the layers seen by the query (actor.AllLayers for all of them)
	Mask actor.Layers
	// Excluded bodies, skipped whatever their layer: the body which casts the ray. A few bodies, searched one by one
	// (IgnoreMultipleBodiesFilter of Jolt)
	Excluded []*actor.RigidBody
	// Triggers: the query also sees the triggers. The zero value ignores them
	Triggers bool
}

// DefaultQueryFilter sees every layer, and ignores the triggers (b3DefaultQueryFilter of Box3D; the triggers as
// QueryTriggerInteraction.Ignore of Unity)
func DefaultQueryFilter() QueryFilter {
	return QueryFilter{Mask: actor.AllLayers}
}

// Accepts: the query sees the body
func (f QueryFilter) Accepts(body *actor.RigidBody) bool {
	return f.Mask&body.Layer != 0 && (f.Triggers || !body.IsTrigger) && !slices.Contains(f.Excluded, body)
}
