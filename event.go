package feather

import (
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
)

const (
	EventTriggerEnter EventType = iota
	EventCollisionEnter
	EventTriggerStay
	EventCollisionStay
	EventTriggerExit
	EventCollisionExit
	EventSleep
	EventWake
)

type pairKey struct {
	bodyA *actor.RigidBody
	bodyB *actor.RigidBody
}

// makePairKey creates a normalized pair key with consistent ordering (the serials of the bodies).
// The key is only used to find a pair, never to order the computations
func makePairKey(bodyA, bodyB *actor.RigidBody) pairKey {
	if bodyB.Serial() < bodyA.Serial() {
		bodyA, bodyB = bodyB, bodyA
	}

	return pairKey{bodyA: bodyA, bodyB: bodyB}
}

// eventPair: a pair of the events, its bodies and its kind (a trigger pair or a contact) when it was recorded: a
// contact whose body is made a trigger by the game ends, and the trigger pair starts
type eventPair struct {
	pairKey
	trigger bool
}

func makeEventPair(bodyA, bodyB *actor.RigidBody) eventPair {
	return eventPair{pairKey: makePairKey(bodyA, bodyB), trigger: bodyA.IsTrigger || bodyB.IsTrigger}
}

type EventType uint8

// Event interface - all events implement this
type Event interface {
	Type() EventType
}

// Trigger events
type TriggerEnterEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e TriggerEnterEvent) Type() EventType { return EventTriggerEnter }

type TriggerStayEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e TriggerStayEvent) Type() EventType { return EventTriggerStay }

type TriggerExitEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e TriggerExitEvent) Type() EventType { return EventTriggerExit }

// Collision events
type CollisionEnterEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e CollisionEnterEvent) Type() EventType { return EventCollisionEnter }

type CollisionStayEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e CollisionStayEvent) Type() EventType { return EventCollisionStay }

type CollisionExitEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e CollisionExitEvent) Type() EventType { return EventCollisionExit }

// Sleep/Wake events
type SleepEvent struct {
	Body *actor.RigidBody
}

func (e SleepEvent) Type() EventType { return EventSleep }

type WakeEvent struct {
	Body *actor.RigidBody
}

func (e WakeEvent) Type() EventType { return EventWake }

// EventListener - callback for events
type EventListener func(event Event)

// Events manager
type Events struct {
	// Listeners by event type
	listeners map[EventType][]EventListener

	// Event buffer to send at flush
	buffer []Event

	// Collision tracking for Enter/Stay/Exit detection
	// The slices keep the order of the pairs, so the events are always sent in the same order
	previousPairs       []eventPair
	currentPairs        []eventPair
	previousActivePairs map[eventPair]bool
	currentActivePairs  map[eventPair]bool

	sleepStates map[*actor.RigidBody]bool
}

func NewEvents() Events {
	return Events{
		listeners:           make(map[EventType][]EventListener),
		buffer:              make([]Event, 0, 256),
		previousActivePairs: make(map[eventPair]bool),
		currentActivePairs:  make(map[eventPair]bool),
		sleepStates:         make(map[*actor.RigidBody]bool),
	}
}

// Subscribe adds a listener for an event type
func (e *Events) Subscribe(eventType EventType, listener EventListener) {
	e.listeners[eventType] = append(e.listeners[eventType], listener)
}

// touchingDistance: bodies closer than this distance are touching (for the collision events)
const touchingDistance = LinearSlop

// recordCollisions records the pairs in contact, and returns the manifolds to solve (triggers are removed).
// Speculative contacts are solved but do not send events
func (e *Events) recordCollisions(manifolds []constraint.Manifold) []constraint.Manifold {
	tracked := e.tracksCollisions()
	n := 0
	for i := range manifolds {
		m := &manifolds[i]
		isTrigger := m.BodyA.IsTrigger || m.BodyB.IsTrigger
		if tracked && (isTrigger || m.MinSeparation() <= touchingDistance) {
			e.record(makeEventPair(m.BodyA, m.BodyB))
		}
		if !isTrigger {
			manifolds[n] = *m
			n++
		}
	}
	return manifolds[:n]
}

func (e *Events) record(pair eventPair) {
	if e.currentActivePairs == nil {
		*e = NewEvents()
	}
	if !e.currentActivePairs[pair] {
		e.currentActivePairs[pair] = true
		e.currentPairs = append(e.currentPairs, pair)
	}
}

// forget a removed body. Its pairs end: their exit is sent with the next events (at the end of the next step, or in
// the flush running if a listener removed the body), as Box2D v3 sends a b2SensorEndTouchEvent when the sensor or its
// visitor is destroyed and a b2ContactEndTouchEvent when a touching contact is destroyed (b2DestroyContact), PhysX an
// eNOTIFY_TOUCH_LOST (PxTriggerPair, PxContactPair) and Jolt an OnContactRemoved
func (e *Events) forget(body *actor.RigidBody) {
	delete(e.sleepStates, body)
	n := 0
	for _, pair := range e.previousPairs {
		if pair.bodyA == body || pair.bodyB == body {
			delete(e.previousActivePairs, pair)
			if pair.trigger {
				if e.hasListeners(EventTriggerExit) {
					e.buffer = append(e.buffer, TriggerExitEvent{BodyA: pair.bodyA, BodyB: pair.bodyB})
				}
			} else if e.hasListeners(EventCollisionExit) {
				e.buffer = append(e.buffer, CollisionExitEvent{BodyA: pair.bodyA, BodyB: pair.bodyB})
			}
			continue
		}
		e.previousPairs[n] = pair
		n++
	}
	e.previousPairs = e.previousPairs[:n]
}

// processCollisionEvents compares current and previous pairs to detect Enter/Stay/Exit
// Should be called after all substeps
func (e *Events) processCollisionEvents() {
	// Detect Enter and Stay events
	for _, pair := range e.currentPairs {
		isTrigger := pair.trigger
		if e.previousActivePairs[pair] {
			// Pair was active before and still is, Stay. No stay while both bodies rest (static or asleep): nothing
			// changes, as Unity sends no collision stay for a sleeping rigidbody and PhysX tests no trigger pair whose
			// actors sleep
			if isAtRest(pair) {
				continue
			}
			if isTrigger {
				if e.hasListeners(EventTriggerStay) {
					e.buffer = append(e.buffer, TriggerStayEvent{
						BodyA: pair.bodyA,
						BodyB: pair.bodyB,
					})
				}
			} else {
				if e.hasListeners(EventCollisionStay) {
					e.buffer = append(e.buffer, CollisionStayEvent{
						BodyA: pair.bodyA,
						BodyB: pair.bodyB,
					})
				}
			}
		} else {
			// New pair, Enter
			if isTrigger {
				if e.hasListeners(EventTriggerEnter) {
					e.buffer = append(e.buffer, TriggerEnterEvent{
						BodyA: pair.bodyA,
						BodyB: pair.bodyB,
					})
				}
			} else {
				if e.hasListeners(EventCollisionEnter) {
					e.buffer = append(e.buffer, CollisionEnterEvent{
						BodyA: pair.bodyA,
						BodyB: pair.bodyB,
					})
				}
			}
		}
	}

	// Detect Exit events
	for _, pair := range e.previousPairs {
		isTrigger := pair.trigger
		// A contact whose bodies both rest (a body asleep on a static body, or on another sleeping body) is not detected
		// anymore, but its bodies still touch: it is kept, as Box2D v3 keeps the contacts of a sleeping body with their
		// touching flag (b2ContactEndTouchEvent only when they stop touching or are destroyed), unless one of its bodies
		// was made a trigger. A trigger pair is tested whatever the sleep: gone, its shapes no longer overlap
		if !isTrigger && !e.currentActivePairs[pair] && isAtRest(pair) && !isTriggerPair(pair.pairKey) {
			e.record(pair)
			continue
		}
		if !e.currentActivePairs[pair] {
			// Pair was active but is no longer, Exit

			if isTrigger {
				if e.hasListeners(EventTriggerExit) {
					e.buffer = append(e.buffer, TriggerExitEvent{
						BodyA: pair.bodyA,
						BodyB: pair.bodyB,
					})
				}
			} else {
				if e.hasListeners(EventCollisionExit) {
					e.buffer = append(e.buffer, CollisionExitEvent{
						BodyA: pair.bodyA,
						BodyB: pair.bodyB,
					})
				}
			}
		}
	}

	// Swap for next frame and clear current
	e.previousActivePairs, e.currentActivePairs = e.currentActivePairs, e.previousActivePairs
	e.previousPairs, e.currentPairs = e.currentPairs, e.previousPairs[:0]
	clear(e.currentActivePairs)
}

func (e *Events) processSleepEvents(bodies []*actor.RigidBody) {
	if e.sleepStates == nil {
		*e = NewEvents()
	}
	if !e.hasListeners(EventSleep) && !e.hasListeners(EventWake) {
		return
	}
	for _, body := range bodies {
		trackedState, exists := e.sleepStates[body]
		if !exists {
			e.sleepStates[body] = body.IsSleeping
			continue
		}

		if !trackedState && body.IsSleeping {
			if e.hasListeners(EventSleep) {
				e.buffer = append(e.buffer, SleepEvent{Body: body})
			}
			e.sleepStates[body] = true
		} else if trackedState && !body.IsSleeping {
			if e.hasListeners(EventWake) {
				e.buffer = append(e.buffer, WakeEvent{Body: body})
			}
			e.sleepStates[body] = false
		}
	}
}

// isAtRest: both bodies of the pair rest, static or asleep
func isAtRest(pair eventPair) bool {
	return !isAwakeMover(pair.bodyA) && !isAwakeMover(pair.bodyB)
}

// isTriggerPair: one of the bodies is a trigger
func isTriggerPair(pair pairKey) bool {
	return pair.bodyA.IsTrigger || pair.bodyB.IsTrigger
}

// hasListeners: an event is only created if somebody listens to it (creating an event allocates)
func (e *Events) hasListeners(eventType EventType) bool {
	return len(e.listeners[eventType]) > 0
}

// tracksCollisions: the pairs in contact are recorded only if somebody listens to the collisions or the triggers
func (e *Events) tracksCollisions() bool {
	for _, t := range [6]EventType{EventTriggerEnter, EventCollisionEnter, EventTriggerStay, EventCollisionStay, EventTriggerExit, EventCollisionExit} {
		if e.hasListeners(t) {
			return true
		}
	}
	return false
}

// flush sends all buffered events and clears the buffer. A listener can remove a body: the exits of its triggers are
// added to the buffer, and sent in this flush
func (e *Events) flush() {
	e.processCollisionEvents()

	for i := 0; i < len(e.buffer); i++ {
		event := e.buffer[i]
		if listeners, ok := e.listeners[event.Type()]; ok {
			for _, listener := range listeners {
				listener(event)
			}
		}
	}
	e.buffer = e.buffer[:0]
}
