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

// createTestConstraint creates a ContactConstraint for testing
func createTestConstraint(bodyA, bodyB *actor.RigidBody) *constraint.ContactConstraint {
	return &constraint.ContactConstraint{
		BodyA:  bodyA,
		BodyB:  bodyB,
		Normal: mgl64.Vec3{1, 0, 0},
		Points: []constraint.ContactPoint{
			{
				Position:    mgl64.Vec3{0, 0, 0},
				Penetration: 0.1,
			},
		},
	}
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

	events.Subscribe(COLLISION_ENTER, capture.capture)

	// Verify listener is registered
	if len(events.listeners[COLLISION_ENTER]) != 1 {
		t.Errorf("Expected 1 listener for COLLISION_ENTER, got %d", len(events.listeners[COLLISION_ENTER]))
	}
}

func TestEvents_MultipleListeners(t *testing.T) {
	events := NewEvents()
	capture1 := &eventCapture{}
	capture2 := &eventCapture{}
	capture3 := &eventCapture{}

	// Subscribe multiple listeners to the same event type
	events.Subscribe(COLLISION_ENTER, capture1.capture)
	events.Subscribe(COLLISION_ENTER, capture2.capture)
	events.Subscribe(COLLISION_ENTER, capture3.capture)

	// Verify all listeners are registered
	if len(events.listeners[COLLISION_ENTER]) != 3 {
		t.Errorf("Expected 3 listeners for COLLISION_ENTER, got %d", len(events.listeners[COLLISION_ENTER]))
	}

	// Trigger an event
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]*constraint.ContactConstraint{c})
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

	events.Subscribe(COLLISION_ENTER, captureCollision.capture)
	events.Subscribe(TRIGGER_ENTER, captureTrigger.capture)

	// Trigger a collision event
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]*constraint.ContactConstraint{c})
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

	// Two normal bodies
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	constraints := []*constraint.ContactConstraint{c}
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

	// One trigger body
	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	constraints := []*constraint.ContactConstraint{c}
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

	// Setup: 1 normal collision + 1 trigger collision
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	bodyC := createTestBody("C", true, false)
	bodyD := createTestBody("D", false, false)

	c1 := createTestConstraint(bodyA, bodyB) // Normal
	c2 := createTestConstraint(bodyC, bodyD) // Trigger

	constraints := []*constraint.ContactConstraint{c1, c2}
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
	events.Subscribe(TRIGGER_ENTER, capture.capture)

	// First frame: trigger collision
	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	// Should receive TRIGGER_ENTER event
	if !capture.hasEventType(TRIGGER_ENTER) {
		t.Error("Expected TRIGGER_ENTER event")
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
	events.Subscribe(TRIGGER_STAY, capture.capture)

	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter (should not trigger STAY)
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	if capture.hasEventType(TRIGGER_STAY) {
		t.Error("TRIGGER_STAY should not occur on first frame")
	}

	capture.reset()

	// Frame 2: Stay
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	// Should receive TRIGGER_STAY event
	if !capture.hasEventType(TRIGGER_STAY) {
		t.Error("Expected TRIGGER_STAY event on second frame")
	}
}

func TestEvents_TriggerExit(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(TRIGGER_EXIT, capture.capture)

	bodyA := createTestBody("A", true, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	capture.reset()

	// Frame 2: Exit (no collision)
	events.recordCollisions([]*constraint.ContactConstraint{})
	events.flush()

	// Should receive TRIGGER_EXIT event
	if !capture.hasEventType(TRIGGER_EXIT) {
		t.Error("Expected TRIGGER_EXIT event")
	}
}

func TestEvents_TriggerStay_SleepingBodies(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(TRIGGER_STAY, capture.capture)

	// Both bodies sleeping
	bodyA := createTestBody("A", true, true)
	bodyB := createTestBody("B", false, true)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	capture.reset()

	// Frame 2: Stay (but both sleeping)
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	// Should NOT receive TRIGGER_STAY when both bodies are sleeping
	if capture.hasEventType(TRIGGER_STAY) {
		t.Error("TRIGGER_STAY should not occur when both bodies are sleeping")
	}
}

// =============================================================================
// COLLISION Events Tests
// =============================================================================

func TestEvents_CollisionEnter(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(COLLISION_ENTER, capture.capture)

	// First frame: normal collision
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	// Should receive COLLISION_ENTER event
	if !capture.hasEventType(COLLISION_ENTER) {
		t.Error("Expected COLLISION_ENTER event")
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
	events.Subscribe(COLLISION_STAY, capture.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter (should not trigger STAY)
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	if capture.hasEventType(COLLISION_STAY) {
		t.Error("COLLISION_STAY should not occur on first frame")
	}

	capture.reset()

	// Frame 2: Stay
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	// Should receive COLLISION_STAY event
	if !capture.hasEventType(COLLISION_STAY) {
		t.Error("Expected COLLISION_STAY event on second frame")
	}
}

func TestEvents_CollisionExit(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(COLLISION_EXIT, capture.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	capture.reset()

	// Frame 2: Exit (no collision)
	events.recordCollisions([]*constraint.ContactConstraint{})
	events.flush()

	// Should receive COLLISION_EXIT event
	if !capture.hasEventType(COLLISION_EXIT) {
		t.Error("Expected COLLISION_EXIT event")
	}
}

func TestEvents_CollisionStay_SleepingBodies(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(COLLISION_STAY, capture.capture)

	// Both bodies sleeping
	bodyA := createTestBody("A", false, true)
	bodyB := createTestBody("B", false, true)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	capture.reset()

	// Frame 2: Stay (but both sleeping)
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	// Should NOT receive COLLISION_STAY when both bodies are sleeping
	if capture.hasEventType(COLLISION_STAY) {
		t.Error("COLLISION_STAY should not occur when both bodies are sleeping")
	}
}

// =============================================================================
// Sleep/Wake Events Tests
// =============================================================================

func TestEvents_OnSleep(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(ON_SLEEP, capture.capture)

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

	// Should receive ON_SLEEP event
	if !capture.hasEventType(ON_SLEEP) {
		t.Error("Expected ON_SLEEP event")
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
	events.Subscribe(ON_WAKE, capture.capture)

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

	// Should receive ON_WAKE event
	if !capture.hasEventType(ON_WAKE) {
		t.Error("Expected ON_WAKE event")
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
	events.Subscribe(ON_SLEEP, capture.capture)

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

	// Should NOT receive ON_SLEEP event (already sleeping)
	if capture.hasEventType(ON_SLEEP) {
		t.Error("Should not receive ON_SLEEP when body is already sleeping")
	}
}

func TestEvents_NoWakeEvent_AlreadyAwake(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(ON_WAKE, capture.capture)

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

	// Should NOT receive ON_WAKE event (already awake)
	if capture.hasEventType(ON_WAKE) {
		t.Error("Should not receive ON_WAKE when body is already awake")
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

	events.Subscribe(COLLISION_ENTER, captureEnter.capture)
	events.Subscribe(COLLISION_STAY, captureStay.capture)
	events.Subscribe(COLLISION_EXIT, captureExit.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]*constraint.ContactConstraint{c})
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
	events.recordCollisions([]*constraint.ContactConstraint{c})
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
	events.recordCollisions([]*constraint.ContactConstraint{})
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

	events.Subscribe(TRIGGER_ENTER, captureTrigger.capture)
	events.Subscribe(COLLISION_ENTER, captureCollision.capture)

	// Setup: 1 normal collision + 1 trigger collision
	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	bodyC := createTestBody("C", true, false)
	bodyD := createTestBody("D", false, false)

	c1 := createTestConstraint(bodyA, bodyB) // Normal
	c2 := createTestConstraint(bodyC, bodyD) // Trigger

	events.recordCollisions([]*constraint.ContactConstraint{c1, c2})
	events.flush()

	// Should receive both event types
	if captureCollision.count() != 1 {
		t.Errorf("Expected 1 COLLISION_ENTER, got %d", captureCollision.count())
	}
	if captureTrigger.count() != 1 {
		t.Errorf("Expected 1 TRIGGER_ENTER, got %d", captureTrigger.count())
	}
}

func TestEvents_SleepWakeWorkflow(t *testing.T) {
	events := NewEvents()
	captureSleep := &eventCapture{}
	captureWake := &eventCapture{}

	events.Subscribe(ON_SLEEP, captureSleep.capture)
	events.Subscribe(ON_WAKE, captureWake.capture)

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
		t.Errorf("Expected 1 ON_SLEEP event, got %d", captureSleep.count())
	}

	// Frame 3: Wake up
	captureSleep.reset()
	body.IsSleeping = false
	events.processSleepEvents(bodies)
	events.flush()

	if captureWake.count() != 1 {
		t.Errorf("Expected 1 ON_WAKE event, got %d", captureWake.count())
	}
}

func TestEvents_Flush_ClearsBuffer(t *testing.T) {
	events := NewEvents()
	capture := &eventCapture{}
	events.Subscribe(COLLISION_ENTER, capture.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Add events to buffer
	events.recordCollisions([]*constraint.ContactConstraint{c})
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
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	// Should succeed without error
}

func TestEvents_MultipleFrames_EnterExitEnter(t *testing.T) {
	events := NewEvents()
	captureEnter := &eventCapture{}
	captureExit := &eventCapture{}

	events.Subscribe(COLLISION_ENTER, captureEnter.capture)
	events.Subscribe(COLLISION_EXIT, captureExit.capture)

	bodyA := createTestBody("A", false, false)
	bodyB := createTestBody("B", false, false)
	c := createTestConstraint(bodyA, bodyB)

	// Frame 1: Enter
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	if captureEnter.count() != 1 {
		t.Error("Expected ENTER on frame 1")
	}

	// Frame 2: Exit
	captureEnter.reset()
	events.recordCollisions([]*constraint.ContactConstraint{})
	events.flush()

	if captureExit.count() != 1 {
		t.Error("Expected EXIT on frame 2")
	}

	// Frame 3: Enter again
	captureExit.reset()
	events.recordCollisions([]*constraint.ContactConstraint{c})
	events.flush()

	if captureEnter.count() != 1 {
		t.Error("Expected ENTER again on frame 3")
	}
}
