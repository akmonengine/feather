package feather

import (
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// The collision events, the sleep and the removal: a contact ends when the bodies no longer touch, or when one of them
// leaves the world, never because a body falls asleep on another one, static or not (Box2D v3 keeps the contacts of a
// sleeping body touching and ends them in b2DestroyContact with a b2ContactEndTouchEvent, contact.c; PhysX sends
// eNOTIFY_TOUCH_LOST when an actor is removed; Unity sends no collision event while a body sleeps)

// liftSpeed: a crate thrown up at this speed leaves the ground (m/s)
const liftSpeed = 3.0

// groundKinds: a crate lands on a plane, or on a static box
var groundKinds = []string{"plane", "box"}

// groundScene: a ground of the kind, and a crate dropped on it
func groundScene(kind string) (*World, *actor.RigidBody, *actor.RigidBody) {
	w := newScene(1)
	var ground *actor.RigidBody
	if kind == "plane" {
		ground = addGround(w, 0.5)
	} else {
		ground = addBody(w, mgl64.Vec3{0, -0.5, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}}, actor.BodyTypeStatic, 0.5, 0)
	}
	crate := addBody(w, mgl64.Vec3{0, crateDrop, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	return w, ground, crate
}

// A crate asleep on a static ground keeps its contact: one enter, no exit and no stay during 10 s of sleep; woken up
// and thrown up it leaves once, and the waking sends no enter
func TestACrateAsleepOnTheGroundKeepsItsContact(t *testing.T) {
	for _, kind := range groundKinds {
		t.Run(kind, func(t *testing.T) {
			w, ground, crate := groundScene(kind)
			log := listenTriggers(w).listenCollisions()
			simulate(w, settleTime, nil)
			if !crate.IsSleeping {
				t.Fatalf("the crate is awake after %v s: the scene doesn't test the sleep", settleTime)
			}
			stays := log.count(EventCollisionStay, ground, crate)
			simulate(w, stayTime, nil)
			if enter := log.count(EventCollisionEnter, ground, crate); enter != 1 {
				t.Errorf("%d enters, want 1", enter)
			}
			if exit := log.count(EventCollisionExit, ground, crate); exit != 0 {
				t.Errorf("%d exits while the crate sleeps on the ground, want 0", exit)
			}
			if more := log.count(EventCollisionStay, ground, crate) - stays; more != 0 {
				t.Errorf("%d stays while the crate sleeps on the ground, want 0", more)
			}

			crate.WakeUp()
			crate.Velocity = mgl64.Vec3{0, liftSpeed, 0}
			simulate(w, 0.1, nil)
			if enter := log.count(EventCollisionEnter, ground, crate); enter != 1 {
				t.Errorf("%d enters once the crate woke up, want 1: the contact was kept", enter)
			}
			if exit := log.count(EventCollisionExit, ground, crate); exit != 1 {
				t.Errorf("%d exits once the crate was thrown up, want 1", exit)
			}
		})
	}
}

// A crate removed from the world while it rests on the ground ends its contact: the exit is sent at the next step,
// once, asleep or awake
func TestARemovedCrateEndsItsContact(t *testing.T) {
	for _, asleep := range []bool{true, false} {
		name := "awake"
		if asleep {
			name = "asleep"
		}
		t.Run(name, func(t *testing.T) {
			w, ground, crate := groundScene("box")
			log := listenTriggers(w).listenCollisions()
			seconds := settleTime
			if !asleep {
				seconds = 1
			}
			simulate(w, seconds, nil)
			if crate.IsSleeping != asleep || log.count(EventCollisionEnter, ground, crate) != 1 {
				t.Fatalf("the crate is not on the ground (asleep %v, want %v)", crate.IsSleeping, asleep)
			}
			before := log.count(EventCollisionExit, ground, crate)
			w.RemoveBody(crate)
			w.Step(sceneDt)
			if exit := log.count(EventCollisionExit, ground, crate) - before; exit != 1 {
				t.Errorf("%d exits at the step after the removal of the crate, want 1", exit)
			}
			simulate(w, 1, nil)
			if exit := log.count(EventCollisionExit, ground, crate) - before; exit != 1 {
				t.Errorf("%d exits a second later, want 1", exit)
			}
		})
	}
}

// The ground removed from the world ends the contact of each crate asleep on it, once; a crate in the air has none
func TestARemovedGroundEndsItsContacts(t *testing.T) {
	w, ground, crate := groundScene("box")
	other := addBody(w, mgl64.Vec3{1, crateDrop, 1}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	log := listenTriggers(w).listenCollisions()
	simulate(w, settleTime, nil)
	flying := addBody(w, mgl64.Vec3{0, 10, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	w.Step(sceneDt)
	before := map[*actor.RigidBody]int{}
	for _, body := range []*actor.RigidBody{crate, other} {
		before[body] = log.count(EventCollisionExit, ground, body)
	}
	w.RemoveBody(ground)
	w.Step(sceneDt)
	for _, body := range []*actor.RigidBody{crate, other} {
		if exit := log.count(EventCollisionExit, ground, body) - before[body]; exit != 1 {
			t.Errorf("%d exits of a crate after the removal of the ground, want 1", exit)
		}
	}
	if exit := log.count(EventCollisionExit, ground, flying); exit != 0 {
		t.Errorf("%d exits of a crate which never touched the ground, want 0", exit)
	}
}

// A listener which removes the crate as it lands gets its exit in the same flush
func TestARemovalDuringTheCollisionEventsIsSent(t *testing.T) {
	w, ground, crate := groundScene("box")
	log := listenTriggers(w).listenCollisions()
	w.Events.Subscribe(EventCollisionEnter, func(Event) { w.RemoveBody(crate) })
	enterStep := uint32(0)
	simulate(w, settleTime, func() {
		if enterStep == 0 && log.count(EventCollisionEnter, ground, crate) > 0 {
			enterStep = w.step
		}
	})
	if enterStep == 0 {
		t.Fatalf("the crate never landed")
	}
	exits := 0
	for _, record := range log.events {
		if record.eventType == EventCollisionExit && pairOf(record, ground, crate) {
			exits++
			if record.step != enterStep {
				t.Errorf("the exit was sent at the step %d, the crate was removed at the step %d", record.step, enterStep)
			}
		}
	}
	if exits != 1 {
		t.Errorf("%d exits, want 1", exits)
	}
}
