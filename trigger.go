package feather

import (
	"unsafe"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
)

const (
	TRIGGER_ENTER EventType = iota
	COLLISION_ENTER
	TRIGGER_STAY
	COLLISION_STAY
	TRIGGER_EXIT
	COLLISION_EXIT
	ON_SLEEP
	ON_WAKE
)

type pairKey struct {
	bodyA *actor.RigidBody
	bodyB *actor.RigidBody
}

// makePairKey creates a normalized pair key with consistent ordering
func makePairKey(bodyA, bodyB *actor.RigidBody) pairKey {
	ptrA := uintptr(unsafe.Pointer(bodyA))
	ptrB := uintptr(unsafe.Pointer(bodyB))

	if ptrB < ptrA {
		bodyA, bodyB = bodyB, bodyA
	}

	return pairKey{bodyA: bodyA, bodyB: bodyB}
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

func (e TriggerEnterEvent) Type() EventType { return TRIGGER_ENTER }

type TriggerStayEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e TriggerStayEvent) Type() EventType { return TRIGGER_STAY }

type TriggerExitEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e TriggerExitEvent) Type() EventType { return TRIGGER_EXIT }

// Collision events
type CollisionEnterEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e CollisionEnterEvent) Type() EventType { return COLLISION_ENTER }

type CollisionStayEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e CollisionStayEvent) Type() EventType { return COLLISION_STAY }

type CollisionExitEvent struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
}

func (e CollisionExitEvent) Type() EventType { return COLLISION_EXIT }

// Sleep/Wake events
type SleepEvent struct {
	Body *actor.RigidBody
}

func (e SleepEvent) Type() EventType { return ON_SLEEP }

type WakeEvent struct {
	Body *actor.RigidBody
}

func (e WakeEvent) Type() EventType { return ON_WAKE }

// EventListener - callback for events
type EventListener func(event Event)

// Events manager
type Events struct {
	// Listeners by event type
	listeners map[EventType][]EventListener

	// Event buffer to send at flush
	buffer []Event

	// Collision tracking for Enter/Stay/Exit detection
	previousActivePairs map[pairKey]bool
	currentActivePairs  map[pairKey]bool

	sleepStates map[*actor.RigidBody]bool
}

func NewEvents() Events {
	return Events{
		listeners:           make(map[EventType][]EventListener),
		buffer:              make([]Event, 0, 256),
		previousActivePairs: make(map[pairKey]bool),
		currentActivePairs:  make(map[pairKey]bool),
		sleepStates:         make(map[*actor.RigidBody]bool),
	}
}

// Subscribe adds a listener for an event type
func (e *Events) Subscribe(eventType EventType, listener EventListener) {
	e.listeners[eventType] = append(e.listeners[eventType], listener)
}

// recordCollision is called during substeps to record a collision/trigger
func (e *Events) recordCollisions(constraints []*constraint.ContactConstraint) []*constraint.ContactConstraint {
	n := 0
	for _, c := range constraints {
		pair := makePairKey(c.BodyA, c.BodyB)
		e.currentActivePairs[pair] = true

		if c.BodyA.IsTrigger == false && c.BodyB.IsTrigger == false {
			constraints[n] = c
			n++
		}
	}
	constraints = constraints[:n]

	return constraints
}

// emitSleep emits a sleep event (called from trySleep)
func (e *Events) emitSleep(body *actor.RigidBody) {
	e.buffer = append(e.buffer, SleepEvent{Body: body})
}

// emitWake emits a wake event (called from WakeUp)
func (e *Events) emitWake(body *actor.RigidBody) {
	e.buffer = append(e.buffer, WakeEvent{Body: body})
}

// processCollisionEvents compares current and previous pairs to detect Enter/Stay/Exit
// Should be called after all substeps
func (e *Events) processCollisionEvents() {
	// Detect Enter and Stay events
	for pair := range e.currentActivePairs {
		// Skip if both bodies are sleeping, to avoid spamming events
		if pair.bodyA.IsSleeping && pair.bodyB.IsSleeping {
			continue
		}

		isTrigger := pair.bodyA.IsTrigger || pair.bodyB.IsTrigger

		if e.previousActivePairs[pair] {
			// Pair was active before and still is, Stay
			if isTrigger {
				e.buffer = append(e.buffer, TriggerStayEvent{
					BodyA: pair.bodyA,
					BodyB: pair.bodyB,
				})
			} else {
				e.buffer = append(e.buffer, CollisionStayEvent{
					BodyA: pair.bodyA,
					BodyB: pair.bodyB,
				})
			}
		} else {
			// New pair, Enter
			if isTrigger {
				e.buffer = append(e.buffer, TriggerEnterEvent{
					BodyA: pair.bodyA,
					BodyB: pair.bodyB,
				})
			} else {
				e.buffer = append(e.buffer, CollisionEnterEvent{
					BodyA: pair.bodyA,
					BodyB: pair.bodyB,
				})
			}
		}
	}

	// Detect Exit events
	for pair := range e.previousActivePairs {
		if !e.currentActivePairs[pair] {
			// Pair was active but is no longer, Exit
			isTrigger := pair.bodyA.IsTrigger || pair.bodyB.IsTrigger

			if isTrigger {
				e.buffer = append(e.buffer, TriggerExitEvent{
					BodyA: pair.bodyA,
					BodyB: pair.bodyB,
				})
			} else {
				e.buffer = append(e.buffer, CollisionExitEvent{
					BodyA: pair.bodyA,
					BodyB: pair.bodyB,
				})
			}
		}
	}

	// Swap for next frame and clear current
	e.previousActivePairs, e.currentActivePairs = e.currentActivePairs, e.previousActivePairs
	clear(e.currentActivePairs)
}

func (e *Events) processSleepEvents(bodies []*actor.RigidBody) {
	for _, body := range bodies {
		trackedState, exists := e.sleepStates[body]
		if !exists {
			e.sleepStates[body] = body.IsSleeping
			continue
		}

		if !trackedState && body.IsSleeping {
			e.buffer = append(e.buffer, SleepEvent{Body: body})
			e.sleepStates[body] = true
		} else if trackedState && !body.IsSleeping {
			e.buffer = append(e.buffer, WakeEvent{Body: body})
			e.sleepStates[body] = false
		}
	}
}

// flush sends all buffered events and clears the buffer
func (e *Events) flush() {
	e.processCollisionEvents()

	for _, event := range e.buffer {
		if listeners, ok := e.listeners[event.Type()]; ok {
			for _, listener := range listeners {
				listener(event)
			}
		}
	}
	e.buffer = e.buffer[:0]
}
