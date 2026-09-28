package feather

import (
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// createTestBody creates a minimal RigidBody for event testing
func createTestBody(id interface{}, isTrigger, isSleeping bool) *actor.RigidBody {
	shape := &actor.Sphere{Radius: 1.0}
	rb := actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{0, 0, 0}},
		shape,
		actor.BodyTypeDynamic,
		1.0,
	)
	rb.Id = id
	rb.IsTrigger = isTrigger
	rb.IsSleeping = isSleeping
	return rb
}

// createTestConstraint creates a touching manifold for testing
func createTestConstraint(bodyA, bodyB *actor.RigidBody) constraint.Manifold {
	m := constraint.Manifold{BodyA: bodyA, BodyB: bodyB, Normal: mgl64.Vec3{1, 0, 0}}
	m.Add(mgl64.Vec3{0, 0, 0}, -0.1)
	return m
}

type eventCapture struct {
	events []Event
}

func (ec *eventCapture) capture(event Event) {
	ec.events = append(ec.events, event)
}

func (ec *eventCapture) reset() {
	ec.events = ec.events[:0]
}

func (ec *eventCapture) count() int {
	return len(ec.events)
}

func (ec *eventCapture) hasEventType(eventType EventType) bool {
	for _, e := range ec.events {
		if e.Type() == eventType {
			return true
		}
	}
	return false
}

// =============================================================================
// Subscribe and Listeners Tests
// =============================================================================

func TestEvents_Subscribe(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}

	events.Subscribe(EventCollisionEnter, capture.capture)

	// Verify listener is registered
	if len(events.listeners[EventCollisionEnter]) != 1 {
		t.Errorf("Expected 1 listener for EventCollisionEnter, got %d", len(events.listeners[EventCollisionEnter]))
	}
}

func TestEvents_MultipleListeners(t *testing.T) {
	events := NewEvents()
	capture1 := &eventCapture{}
	capture2 := &eventCapture{}
	capture3 := &eventCapture{}

	// Subscribe multiple listeners to the same event type
	events.Subscribe(EventCollisionEnter, capture1.capture)
	events.Subscribe(EventCollisionEnter, capture2.capture)
	events.Subscribe(EventCollisionEnter, capture3.capture)

	// Verify all listeners are registered
	if len(events.listeners[EventCollisionEnter]) != 3 {
		t.Errorf("Expected 3 listeners for EventCollisionEnter, got %d", len(events.listeners[EventCollisionEnter]))
	}

	// Trigger an event
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// All listeners should have received the event
	if capture1.count() != 1 {
		t.Errorf("Capture1 expected 1 event, got %d", capture1.count())
	}
	if capture2.count() != 1 {
		t.Errorf("Capture2 expected 1 event, got %d", capture2.count())
	}
	if capture3.count() != 1 {
		t.Errorf("Capture3 expected 1 event, got %d", capture3.count())
	}
}

func TestEvents_DifferentEventTypes(t *testing.T) {
	events := NewEvents()
	captureCollision := &eventCapture{}
	captureTrigger := &eventCapture{}

	events.Subscribe(EventCollisionEnter, captureCollision.capture)
	events.Subscribe(EventTriggerEnter, captureTrigger.capture)

	// Trigger a collision event
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Only collision listener should receive event
	if captureCollision.count() != 1 {
		t.Errorf("Collision capture expected 1 event, got %d", captureCollision.count())
	}
	if captureTrigger.count() != 0 {
		t.Errorf("Trigger capture expected 0 events, got %d", captureTrigger.count())
	}
}

// =============================================================================
// makePairKey Tests
// =============================================================================

func TestMakePairKey_Normalization(t *testing.T) {
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)

	// Create pairs in both orders
	pairAB := makePairKey(bodyA, bodyB)
	pairBA := makePairKey(bodyB, bodyA)

	// Pairs should be identical (normalized)
	if pairAB.bodyA != pairBA.bodyA || pairAB.bodyB != pairBA.bodyB {
		t.Error("makePairKey should normalize pairs to consistent ordering")
	}
}

func TestMakePairKey_SamePair(t *testing.T) {
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)

	pair1 := makePairKey(bodyA, bodyB)
	pair2 := makePairKey(bodyA, bodyB)

	// Same input should produce identical keys
	if pair1.bodyA != pair2.bodyA || pair1.bodyB != pair2.bodyB {
		t.Error("makePairKey should produce consistent keys for same input")
	}
}

func TestMakePairKey_DifferentPairs(t *testing.T) {
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	bodyC := createTestBody("C", false, false)

	pairAB := makePairKey(bodyA, bodyB)
	pairAC := makePairKey(bodyA, bodyC)

	// Different pairs should have different keys
	isDifferent := (pairAB.bodyA != pairAC.bodyA || pairAB.bodyB != pairAC.bodyB)
	if !isDifferent {
		t.Error("makePairKey should produce different keys for different pairs")
	}
}

// =============================================================================
// recordCollisions Tests
// =============================================================================

func TestEvents_RecordCollisions_NormalCollision(t *testing.T) {
	events := NewEvents()
	// the pairs are recorded only if somebody listens
	events.Subscribe(EventCollisionEnter, func(Event) {})

	// Two normal bodies
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	constraints := []constraint.Manifold{c}
	result := events.recordCollisions(constraints)

	// Normal collision should remain in constraints
	if len(result) != 1 {
		t.Errorf("Expected 1 constraint to remain, got %d", len(result))
	}

	// Pair should be recorded
	pair := makePairKey(bodyA, bodyB)
	if !events.currentActivePairs[pair] {
		t.Error("Normal collision pair should be recorded in currentActivePairs")
	}
}

func TestEvents_RecordCollisions_TriggerCollision(t *testing.T) {
	events := NewEvents()
	// the pairs are recorded only if somebody listens
	events.Subscribe(EventCollisionEnter, func(Event) {})

	// One trigger body
	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	constraints := []constraint.Manifold{c}
	result := events.recordCollisions(constraints)

	// Trigger collision should be filtered out
	if len(result) != 0 {
		t.Errorf("Expected 0 constraints (trigger filtered), got %d", len(result))
	}

	// Pair should still be recorded for event generation
	pair := makePairKey(bodyA, bodyB)
	if !events.currentActivePairs[pair] {
		t.Error("Trigger pair should be recorded in currentActivePairs")
	}
}

func TestEvents_RecordCollisions_Mixed(t *testing.T) {
	events := NewEvents()
	// the pairs are recorded only if somebody listens
	events.Subscribe(EventCollisionEnter, func(Event) {})

	// Setup: 1 normal collision + 1 trigger collision
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	bodyC := createTestBody("C", true, false)
	bodyD := createTestBody("D", false, false)

	c1 := createTestConstraint(bodyA, bodyB) // Normal
	c2 := createTestConstraint(bodyC, bodyD) // Trigger

	constraints := []constraint.Manifold{c1, c2}
	result := events.recordCollisions(constraints)

	// Only normal collision should remain
	if len(result) != 1 {
		t.Errorf("Expected 1 normal constraint, got %d", len(result))
	}

	// Both pairs should be recorded
	if len(events.currentActivePairs) != 2 {
		t.Errorf("Expected 2 pairs recorded, got %d", len(events.currentActivePairs))
	}
}

// =============================================================================
// TRIGGER Events Tests
// =============================================================================

func TestEvents_TriggerEnter(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventTriggerEnter, capture.capture)

	// First frame: trigger collision
	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Should receive EventTriggerEnter event
	if !capture.hasEventType(EventTriggerEnter) {
		t.Error("Expected EventTriggerEnter event")
	}

	if capture.count() != 1 {
		t.Errorf("Expected 1 event, got %d", capture.count())
	}

	// Verify event contents
	event := capture.events[0].(TriggerEnterEvent)
	if event.BodyA == nil || event.BodyB == nil {
		t.Error("TriggerEnterEvent should have both bodies")
	}
}

func TestEvents_TriggerStay(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventTriggerStay, capture.capture)

	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter (should not trigger STAY)
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	if capture.hasEventType(EventTriggerStay) {
		t.Error("EventTriggerStay should not occur on first frame")
	}

	capture.reset()

	// Frame 2: Stay
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Should receive EventTriggerStay event
	if !capture.hasEventType(EventTriggerStay) {
		t.Error("Expected EventTriggerStay event on second frame")
	}
}

func TestEvents_TriggerExit(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventTriggerExit, capture.capture)

	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	capture.reset()

	// Frame 2: Exit (no collision)
	events.recordCollisions([]constraint.Manifold{})
	events.flush()

	// Should receive EventTriggerExit event
	if !capture.hasEventType(EventTriggerExit) {
		t.Error("Expected EventTriggerExit event")
	}
}

func TestEvents_TriggerStay_SleepingBodies(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventTriggerStay, capture.capture)

	// Both bodies sleeping
	bodyA := createTestBody("A", true, true)
	bodyB := createTestBody("B", false, true)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	capture.reset()

	// Frame 2: Stay (but both sleeping)
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Should NOT receive EventTriggerStay when both bodies are sleeping
	if capture.hasEventType(EventTriggerStay) {
		t.Error("EventTriggerStay should not occur when both bodies are sleeping")
	}
}

// =============================================================================
// COLLISION Events Tests
// =============================================================================

func TestEvents_CollisionEnter(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventCollisionEnter, capture.capture)

	// First frame: normal collision
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Should receive EventCollisionEnter event
	if !capture.hasEventType(EventCollisionEnter) {
		t.Error("Expected EventCollisionEnter event")
	}

	if capture.count() != 1 {
		t.Errorf("Expected 1 event, got %d", capture.count())
	}

	// Verify event contents
	event := capture.events[0].(CollisionEnterEvent)
	if event.BodyA == nil || event.BodyB == nil {
		t.Error("CollisionEnterEvent should have both bodies")
	}
}

func TestEvents_CollisionStay(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventCollisionStay, capture.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter (should not trigger STAY)
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	if capture.hasEventType(EventCollisionStay) {
		t.Error("EventCollisionStay should not occur on first frame")
	}

	capture.reset()

	// Frame 2: Stay
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Should receive EventCollisionStay event
	if !capture.hasEventType(EventCollisionStay) {
		t.Error("Expected EventCollisionStay event on second frame")
	}
}

func TestEvents_CollisionExit(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventCollisionExit, capture.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	capture.reset()

	// Frame 2: Exit (no collision)
	events.recordCollisions([]constraint.Manifold{})
	events.flush()

	// Should receive EventCollisionExit event
	if !capture.hasEventType(EventCollisionExit) {
		t.Error("Expected EventCollisionExit event")
	}
}

func TestEvents_CollisionStay_SleepingBodies(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventCollisionStay, capture.capture)

	// Both bodies sleeping
	bodyA := createTestBody("A", false, true)
	bodyB := createTestBody("B", false, true)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	capture.reset()

	// Frame 2: Stay (but both sleeping)
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Should NOT receive EventCollisionStay when both bodies are sleeping
	if capture.hasEventType(EventCollisionStay) {
		t.Error("EventCollisionStay should not occur when both bodies are sleeping")
	}
}

// =============================================================================
// Sleep/Wake Events Tests
// =============================================================================

func TestEvents_OnSleep(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventSleep, capture.capture)

	// Body starts awake
	body := createTestBody("A", false, false)
	bodies := []*actor.RigidBody{body}

	// Frame 1: Initialize state
	events.processSleepEvents(bodies)
	events.flush()

	// No event on initialization
	if capture.count() != 0 {
		t.Errorf("Expected no events on initialization, got %d", capture.count())
	}

	// Frame 2: Body goes to sleep
	body.IsSleeping = true
	events.processSleepEvents(bodies)
	events.flush()

	// Should receive EventSleep event
	if !capture.hasEventType(EventSleep) {
		t.Error("Expected EventSleep event")
	}

	if capture.count() != 1 {
		t.Errorf("Expected 1 event, got %d", capture.count())
	}

	// Verify event contents
	event := capture.events[0].(SleepEvent)
	if event.Body != body {
		t.Error("SleepEvent should contain the correct body")
	}
}

func TestEvents_OnWake(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventWake, capture.capture)

	// Body starts sleeping
	body := createTestBody("A", false, true)
	bodies := []*actor.RigidBody{body}

	// Frame 1: Initialize state
	events.processSleepEvents(bodies)
	events.flush()

	// No event on initialization
	if capture.count() != 0 {
		t.Errorf("Expected no events on initialization, got %d", capture.count())
	}

	// Frame 2: Body wakes up
	body.IsSleeping = false
	events.processSleepEvents(bodies)
	events.flush()

	// Should receive EventWake event
	if !capture.hasEventType(EventWake) {
		t.Error("Expected EventWake event")
	}

	if capture.count() != 1 {
		t.Errorf("Expected 1 event, got %d", capture.count())
	}

	// Verify event contents
	event := capture.events[0].(WakeEvent)
	if event.Body != body {
		t.Error("WakeEvent should contain the correct body")
	}
}

func TestEvents_NoSleepEvent_AlreadySleeping(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventSleep, capture.capture)

	// Body starts sleeping
	body := createTestBody("A", false, true)
	bodies := []*actor.RigidBody{body}

	// Frame 1: Initialize
	events.processSleepEvents(bodies)
	events.flush()

	capture.reset()

	// Frame 2: Still sleeping
	events.processSleepEvents(bodies)
	events.flush()

	// Should NOT receive EventSleep event (already sleeping)
	if capture.hasEventType(EventSleep) {
		t.Error("Should not receive EventSleep when body is already sleeping")
	}
}

func TestEvents_NoWakeEvent_AlreadyAwake(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventWake, capture.capture)

	// Body starts awake
	body := createTestBody("A", false, false)
	bodies := []*actor.RigidBody{body}

	// Frame 1: Initialize
	events.processSleepEvents(bodies)
	events.flush()

	capture.reset()

	// Frame 2: Still awake
	events.processSleepEvents(bodies)
	events.flush()

	// Should NOT receive EventWake event (already awake)
	if capture.hasEventType(EventWake) {
		t.Error("Should not receive EventWake when body is already awake")
	}
}

// =============================================================================
// Integration Tests
// =============================================================================

func TestEvents_CompleteWorkflow(t *testing.T) {
	events := NewEvents()
	captureEnter := &eventCapture{}
	captureStay := &eventCapture{}
	captureExit := &eventCapture{}

	events.Subscribe(EventCollisionEnter, captureEnter.capture)
	events.Subscribe(EventCollisionStay, captureStay.capture)
	events.Subscribe(EventCollisionExit, captureExit.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	if captureEnter.count() != 1 {
		t.Errorf("Frame 1: Expected 1 ENTER event, got %d", captureEnter.count())
	}
	if captureStay.count() != 0 {
		t.Errorf("Frame 1: Expected 0 STAY events, got %d", captureStay.count())
	}
	if captureExit.count() != 0 {
		t.Errorf("Frame 1: Expected 0 EXIT events, got %d", captureExit.count())
	}

	// Frame 2: Stay
	captureEnter.reset()
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	if captureEnter.count() != 0 {
		t.Errorf("Frame 2: Expected 0 ENTER events, got %d", captureEnter.count())
	}
	if captureStay.count() != 1 {
		t.Errorf("Frame 2: Expected 1 STAY event, got %d", captureStay.count())
	}
	if captureExit.count() != 0 {
		t.Errorf("Frame 2: Expected 0 EXIT events, got %d", captureExit.count())
	}

	// Frame 3: Exit
	captureStay.reset()
	events.recordCollisions([]constraint.Manifold{})
	events.flush()

	if captureEnter.count() != 0 {
		t.Errorf("Frame 3: Expected 0 ENTER events, got %d", captureEnter.count())
	}
	if captureStay.count() != 0 {
		t.Errorf("Frame 3: Expected 0 STAY events, got %d", captureStay.count())
	}
	if captureExit.count() != 1 {
		t.Errorf("Frame 3: Expected 1 EXIT event, got %d", captureExit.count())
	}
}

func TestEvents_MixedTriggerAndCollision(t *testing.T) {
	events := NewEvents()
	captureTrigger := &eventCapture{}
	captureCollision := &eventCapture{}

	events.Subscribe(EventTriggerEnter, captureTrigger.capture)
	events.Subscribe(EventCollisionEnter, captureCollision.capture)

	// Setup: 1 normal collision + 1 trigger collision
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	bodyC := createTestBody("C", true, false)
	bodyD := createTestBody("D", false, false)

	c1 := createTestConstraint(bodyA, bodyB) // Normal
	c2 := createTestConstraint(bodyC, bodyD) // Trigger

	events.recordCollisions([]constraint.Manifold{c1, c2})
	events.flush()

	// Should receive both event types
	if captureCollision.count() != 1 {
		t.Errorf("Expected 1 EventCollisionEnter, got %d", captureCollision.count())
	}
	if captureTrigger.count() != 1 {
		t.Errorf("Expected 1 EventTriggerEnter, got %d", captureTrigger.count())
	}
}

func TestEvents_SleepWakeWorkflow(t *testing.T) {
	events := NewEvents()
	captureSleep := &eventCapture{}
	captureWake := &eventCapture{}

	events.Subscribe(EventSleep, captureSleep.capture)
	events.Subscribe(EventWake, captureWake.capture)

	body := createTestBody("A", false, false)
	bodies := []*actor.RigidBody{body}

	// Frame 1: Initialize (awake)
	events.processSleepEvents(bodies)
	events.flush()

	if captureSleep.count() != 0 || captureWake.count() != 0 {
		t.Error("Expected no events on initialization")
	}

	// Frame 2: Go to sleep
	body.IsSleeping = true
	events.processSleepEvents(bodies)
	events.flush()

	if captureSleep.count() != 1 {
		t.Errorf("Expected 1 EventSleep event, got %d", captureSleep.count())
	}

	// Frame 3: Wake up
	captureSleep.reset()
	body.IsSleeping = false
	events.processSleepEvents(bodies)
	events.flush()

	if captureWake.count() != 1 {
		t.Errorf("Expected 1 EventWake event, got %d", captureWake.count())
	}
}

func TestEvents_Flush_ClearsBuffer(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(EventCollisionEnter, capture.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Add events to buffer
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Buffer should be cleared after flush
	if len(events.buffer) != 0 {
		t.Errorf("Expected buffer to be empty after flush, got %d events", len(events.buffer))
	}

	// Listener should have received the event
	if capture.count() != 1 {
		t.Errorf("Expected 1 event received, got %d", capture.count())
	}
}

// =============================================================================
// Edge Cases Tests
// =============================================================================

func TestEvents_EmptyBuffer_Flush(t *testing.T) {
	events := NewEvents()

	// Flush with empty buffer should not crash
	events.flush()

	// Should succeed without error
}

func TestEvents_NoListeners(t *testing.T) {
	events := NewEvents()

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Process events without any listeners
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	// Should succeed without error
}

func TestEvents_MultipleFrames_EnterExitEnter(t *testing.T) {
	events := NewEvents()
	captureEnter := &eventCapture{}
	captureExit := &eventCapture{}

	events.Subscribe(EventCollisionEnter, captureEnter.capture)
	events.Subscribe(EventCollisionExit, captureExit.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	if captureEnter.count() != 1 {
		t.Error("Expected ENTER on frame 1")
	}

	// Frame 2: Exit
	captureEnter.reset()
	events.recordCollisions([]constraint.Manifold{})
	events.flush()

	if captureExit.count() != 1 {
		t.Error("Expected EXIT on frame 2")
	}

	// Frame 3: Enter again
	captureExit.reset()
	events.recordCollisions([]constraint.Manifold{c})
	events.flush()

	if captureEnter.count() != 1 {
		t.Error("Expected ENTER again on frame 3")
	}
}
