package feather

import (
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// The pairs of a trigger and the sleep: a trigger reports the end of an overlap when the shapes no longer overlap, or
// when one of the bodies leaves the world, never because one of them falls asleep (b2SensorEndTouchEvent of Box2D v3,
// whose sensors "do not consider sleep"; the trigger pairs of PhysX, kept while both actors sleep)

// triggerLog records the trigger events of a world, by step (and its collision events, listenCollisions)
type triggerLog struct {
	w      *World
	events []triggerRecord
	// counting: the events are only counted, without allocation
	counting bool
	counted  int
}

type triggerRecord struct {
	step      uint32
	eventType EventType
	a, b      uint64 // the serials of the bodies, in the order of the event
}

func listenTriggers(w *World) *triggerLog {
	return listenEvents(w, EventTriggerEnter, EventTriggerStay, EventTriggerExit)
}

// listenEvents: a log of the events of the types
func listenEvents(w *World, eventTypes ...EventType) *triggerLog {
	log := &triggerLog{w: w}
	for _, eventType := range eventTypes {
		w.Events.Subscribe(eventType, log.record)
	}
	return log
}

func (log *triggerLog) record(event Event) {
	if log.counting {
		log.counted++
		return
	}
	var a, b *actor.RigidBody
	switch e := event.(type) {
	case TriggerEnterEvent:
		a, b = e.BodyA, e.BodyB
	case TriggerStayEvent:
		a, b = e.BodyA, e.BodyB
	case TriggerExitEvent:
		a, b = e.BodyA, e.BodyB
	case CollisionEnterEvent:
		a, b = e.BodyA, e.BodyB
	case CollisionStayEvent:
		a, b = e.BodyA, e.BodyB
	case CollisionExitEvent:
		a, b = e.BodyA, e.BodyB
	}
	log.events = append(log.events, triggerRecord{step: log.w.step, eventType: event.Type(), a: a.Serial(), b: b.Serial()})
}

// listenCollisions adds the collision events to the log
func (log *triggerLog) listenCollisions() *triggerLog {
	for _, eventType := range []EventType{EventCollisionEnter, EventCollisionStay, EventCollisionExit} {
		log.w.Events.Subscribe(eventType, log.record)
	}
	return log
}

// count of the events of the type between the trigger and the body
func (log *triggerLog) count(eventType EventType, trigger, body *actor.RigidBody) int {
	n := 0
	for _, record := range log.events {
		if record.eventType == eventType && pairOf(record, trigger, body) {
			n++
		}
	}
	return n
}

func pairOf(record triggerRecord, a, b *actor.RigidBody) bool {
	return (record.a == a.Serial() && record.b == b.Serial()) || (record.a == b.Serial() && record.b == a.Serial())
}

const (
	zoneHalf   = 2.0 // the zone is a box of 4 x 2 x 4 m, on the ground
	crateDrop  = 1.5 // a crate falls from this height, into the zone
	settleTime = 3.0 // s: a crate dropped into the zone has landed and fallen asleep
	stayTime   = 10.0
	pushSpeed  = 6.0 // m/s: with a friction of 0.5 a crate slides 3.7 m, out of the zone
)

// zoneScene: a trigger zone on the ground, of the body type, and a crate dropped in it
func zoneScene(workers int, zoneType actor.BodyType) (*World, *actor.RigidBody, *actor.RigidBody) {
	w := newScene(workers)
	addGround(w, 0.5)
	zone := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{zoneHalf, 1, zoneHalf}}, zoneType, 0, 0)
	zone.IsTrigger = true
	crate := addBody(w, mgl64.Vec3{0, crateDrop, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	return w, zone, crate
}

// A crate dropped in a zone lands, falls asleep, and stays in the zone: one enter, and no exit during 10 s
func TestASleepingCrateStaysInTheTrigger(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	if !crate.IsSleeping {
		t.Fatalf("the crate is awake after %v s: the scene doesn't test the sleep", settleTime)
	}
	simulate(w, stayTime, nil)
	if !crate.IsSleeping {
		t.Errorf("the crate woke up")
	}
	if enter := log.count(EventTriggerEnter, zone, crate); enter != 1 {
		t.Errorf("%d enters, want 1", enter)
	}
	if exit := log.count(EventTriggerExit, zone, crate); exit != 0 {
		t.Errorf("%d exits while the crate sleeps in the zone, want 0", exit)
	}
}

// While both bodies rest the overlap can't change: no stay event is sent (as for 2 sleeping bodies), the pair is kept
func TestASleepingCrateSendsNoStay(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	stays := log.count(EventTriggerStay, zone, crate)
	if stays == 0 {
		t.Fatalf("no stay while the crate fell in the zone")
	}
	simulate(w, stayTime, nil)
	if more := log.count(EventTriggerStay, zone, crate) - stays; more != 0 {
		t.Errorf("%d stays while the crate sleeps in the zone, want 0", more)
	}
}

// A sleeping crate pushed out of the zone leaves it once, when it is out
func TestACratePushedOutOfTheTriggerLeavesOnce(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	if !crate.IsSleeping {
		t.Fatalf("the crate is awake after %v s", settleTime)
	}
	crate.WakeUp()
	crate.Velocity = mgl64.Vec3{pushSpeed, 0, 0}
	var exitX float64
	simulate(w, settleTime, func() {
		if exitX == 0 && log.count(EventTriggerExit, zone, crate) > 0 {
			exitX = crate.Transform.Position.X()
		}
	})
	if crate.Transform.Position.X() < zoneHalf+cubeHalf {
		t.Fatalf("the crate stopped at x=%.2f, in the zone", crate.Transform.Position.X())
	}
	if exit := log.count(EventTriggerExit, zone, crate); exit != 1 {
		t.Errorf("%d exits, want 1", exit)
	}
	if enter := log.count(EventTriggerEnter, zone, crate); enter != 1 {
		t.Errorf("%d enters, want 1", enter)
	}
	// the exit is sent at the step the crate leaves the zone: its near face is past the far face of the zone
	if exitX-cubeHalf < zoneHalf-LinearSlop || exitX-cubeHalf > zoneHalf+pushSpeed*sceneDt {
		t.Errorf("the exit was sent with the crate at x=%.3f, want its face just past %.2f", exitX, zoneHalf)
	}
}

// A crate removed from the world while it sleeps in the zone leaves it: the exit is sent at the next step, once
func TestACrateRemovedFromTheTriggerLeaves(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	before := log.count(EventTriggerExit, zone, crate)
	w.RemoveBody(crate)
	w.Step(sceneDt)
	if exit := log.count(EventTriggerExit, zone, crate) - before; exit != 1 {
		t.Errorf("%d exits at the step after the removal of the crate, want 1", exit)
	}
	simulate(w, 1, nil)
	if exit := log.count(EventTriggerExit, zone, crate) - before; exit != 1 {
		t.Errorf("%d exits a second later, want 1", exit)
	}
}

// A trigger removed from the world ends its pairs: each body in it leaves, once
func TestARemovedTriggerEndsItsPairs(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	other := addBody(w, mgl64.Vec3{1, crateDrop, 1}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	before := map[*actor.RigidBody]int{}
	for _, body := range []*actor.RigidBody{crate, other} {
		before[body] = log.count(EventTriggerExit, zone, body)
	}
	w.RemoveBody(zone)
	simulate(w, 1, nil)
	for _, body := range []*actor.RigidBody{crate, other} {
		if exit := log.count(EventTriggerExit, zone, body) - before[body]; exit != 1 {
			t.Errorf("%d exits of a crate after the removal of the zone, want 1", exit)
		}
	}
}

// A static zone moved by the game away from a sleeping crate: the crate is out, it leaves (Box2D tests its sensors
// whatever the sleep, PhysX tests a trigger pair again when the pose of an actor is set)
func TestAZoneMovedAwayFromASleepingCrate(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	before := log.count(EventTriggerExit, zone, crate)
	zone.Transform.Position = mgl64.Vec3{3 * zoneHalf, 1, 0}
	zone.UpdateAABB()
	w.Step(sceneDt)
	if exit := log.count(EventTriggerExit, zone, crate) - before; exit != 1 {
		t.Errorf("%d exits once the zone moved away, want 1", exit)
	}
	if !crate.IsSleeping {
		t.Errorf("the zone woke the crate up")
	}
	// and back over the crate: it enters again
	zone.Transform.Position = mgl64.Vec3{0, 1, 0}
	zone.UpdateAABB()
	w.Step(sceneDt)
	if enter := log.count(EventTriggerEnter, zone, crate); enter != 2 {
		t.Errorf("%d enters once the zone is back, want 2", enter)
	}
}

// A sleeping crate moved by hand out of the zone (its Transform, then UpdateAABB, without waking it up) leaves it:
// the overlap of a pair at rest is kept only while the AABBs of its bodies stay the same
func TestASleepingCrateMovedByHandLeaves(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	// the crate lands across the side of the zone, 5 cm in it
	crate.Transform.Position = mgl64.Vec3{zoneHalf + cubeHalf - 0.05, crateDrop, 0}
	crate.UpdateAABB()
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	if !crate.IsSleeping || log.count(EventTriggerEnter, zone, crate) != 1 {
		t.Fatalf("the crate is not asleep in the zone (asleep %v)", crate.IsSleeping)
	}
	before := log.count(EventTriggerExit, zone, crate)
	// 7 cm further, out of the zone: the crate stays in its enlarged AABB of the broad phase (AABBMargin)
	crate.Transform.Position = crate.Transform.Position.Add(mgl64.Vec3{0.07, 0, 0})
	crate.UpdateAABB()
	w.Step(sceneDt)
	if exit := log.count(EventTriggerExit, zone, crate) - before; exit != 1 {
		t.Errorf("%d exits once the crate was moved out of the zone, want 1", exit)
	}
	if !crate.IsSleeping {
		t.Errorf("the move woke the crate up: the test doesn't test a body at rest")
	}
}

// A static box made a trigger by the game under a crate asleep on it: the crate enters it (its contact with the box
// is an overlap), its contact with the box ends, and nothing wakes up
func TestABoxMadeATriggerUnderASleepingCrate(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.5)
	pedestal := addBody(w, mgl64.Vec3{0, 0.5, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 0.5, 1}}, actor.BodyTypeStatic, 0.5, 0)
	crate := addBody(w, mgl64.Vec3{0, 1 + crateDrop, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	log := listenTriggers(w).listenCollisions()
	simulate(w, settleTime, nil)
	if !crate.IsSleeping {
		t.Fatalf("the crate is awake after %v s", settleTime)
	}
	exits := log.count(EventCollisionExit, pedestal, crate)
	pedestal.IsTrigger = true
	w.Step(sceneDt)
	if enter := log.count(EventTriggerEnter, pedestal, crate); enter != 1 {
		t.Errorf("%d enters once the box is a trigger, want 1", enter)
	}
	if exit := log.count(EventCollisionExit, pedestal, crate) - exits; exit != 1 {
		t.Errorf("%d collision exits once the box is a trigger, want 1: the contact ended", exit)
	}
	if !crate.IsSleeping {
		t.Errorf("the crate woke up")
	}
}

// A listener which removes the crate as it enters gets its exit in the same flush: an event of a removal during the
// flush is not lost
func TestARemovalDuringTheEventsIsSent(t *testing.T) {
	w, zone, crate := zoneScene(1, actor.BodyTypeStatic)
	log := listenTriggers(w)
	w.Events.Subscribe(EventTriggerEnter, func(Event) { w.RemoveBody(crate) })
	enterStep := uint32(0)
	simulate(w, settleTime, func() {
		if enterStep == 0 && log.count(EventTriggerEnter, zone, crate) > 0 {
			enterStep = w.step
		}
	})
	if enterStep == 0 {
		t.Fatalf("the crate never entered the zone")
	}
	exits := 0
	for _, record := range log.events {
		if record.eventType == EventTriggerExit && pairOf(record, zone, crate) {
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

// zoneFull: crates dropped in a zone, which fall asleep in it, then half of them pushed out, the other half staying
func zoneFull(workers int) (*World, *triggerLog) {
	w, _, _ := zoneScene(workers, actor.BodyTypeStatic)
	w.parallelFrom = 1
	for x := -2; x <= 2; x++ {
		for z := -2; z <= 2; z++ {
			addBody(w, mgl64.Vec3{float64(x) * 0.7, crateDrop + 1, float64(z) * 0.7}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
		}
	}
	log := listenTriggers(w)
	simulate(w, settleTime, nil)
	for _, body := range w.Bodies[2:] {
		if body.Transform.Position.Z() > 0 {
			body.WakeUp()
			body.Velocity = mgl64.Vec3{0, 0, pushSpeed}
		}
	}
	simulate(w, settleTime, nil)
	return w, log
}

// The events are the same, in the same order, with 1 and 8 workers
func TestTriggerEventsAreDeterministic(t *testing.T) {
	w, one := zoneFull(1)
	_, eight := zoneFull(8)
	sleeping, inside := 0, 0
	for _, body := range w.Bodies[2:] {
		if body.IsSleeping {
			sleeping++
		}
		if body.Transform.Position.Z() < zoneHalf {
			inside++
		}
	}
	if sleeping == 0 || inside == 0 || inside == len(w.Bodies)-2 {
		t.Fatalf("%d crates asleep, %d in the zone: the scene doesn't test the sleep and the exits", sleeping, inside)
	}
	exits := 0
	for _, record := range one.events {
		if record.eventType == EventTriggerExit {
			exits++
		}
	}
	if exits != len(w.Bodies)-2-inside {
		t.Errorf("%d exits, %d crates out of the zone", exits, len(w.Bodies)-2-inside)
	}
	// the serials differ between both worlds: the events are compared by the index of their bodies
	if len(one.events) != len(eight.events) {
		t.Fatalf("%d events with 1 worker, %d with 8", len(one.events), len(eight.events))
	}
	first, second := serialIndices(one), serialIndices(eight)
	for i := range one.events {
		a, b := one.events[i], eight.events[i]
		if a.step != b.step || a.eventType != b.eventType || first[a.a] != second[b.a] || first[a.b] != second[b.b] {
			t.Fatalf("event %d: %+v with 1 worker, %+v with 8", i, a, b)
		}
	}
}

// serialIndices: the index of each body of the log's world, by serial
func serialIndices(log *triggerLog) map[uint64]int {
	indices := map[uint64]int{}
	for i, body := range log.w.Bodies {
		indices[body.Serial()] = i
	}
	return indices
}

// A step with crates asleep in a zone, listened to, allocates nothing
func TestSleepingTriggerPairsDoNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 8} {
		w, log := zoneFull(workers)
		asleep := slices.ContainsFunc(w.Bodies[2:], func(body *actor.RigidBody) bool { return body.IsSleeping })
		if !asleep {
			t.Fatalf("workers=%d: no crate asleep", workers)
		}
		log.counting = true
		if allocs := testing.AllocsPerRun(10, func() { w.Step(sceneDt) }); allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
	}
}

// ========== KINEMATIC BODIES ==========
// A kinematic body enters a trigger and leaves it as a dynamic body does, the trigger static or kinematic, and a
// kinematic trigger detects the static and the kinematic bodies: Box2D v3 tests every sensor against its 3 trees
// (b2SensorTask: static, kinematic and dynamic), Jolt "These sensors will only detect collisions with active Dynamic or
// Kinematic bodies" for a static sensor (Body.h), and Unity "A dynamic or kinematic trigger collider collides with any
// collider type. A static trigger collider collides with any dynamic or Kinematic collider". A kinematic body still has
// no contact with them

// crossSpeed: a kinematic cube crosses the zone at this speed (m/s)
const crossSpeed = 2.0

// kinematicZoneScene: a static trigger zone on the ground plane, and a kinematic cube 4 m left of its center
func kinematicZoneScene(workers int) (*World, *actor.RigidBody, *actor.RigidBody) {
	w := newScene(workers)
	addGround(w, 0.5)
	zone := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{zoneHalf, 1, zoneHalf}}, actor.BodyTypeStatic, 0, 0)
	zone.IsTrigger = true
	mover := kinematicBox(w, mgl64.Vec3{-2 * zoneHalf, 0.5, 0}, cube().HalfExtents)
	return w, zone, mover
}

// A kinematic cube driven through a static zone enters it once when its face reaches the zone, stays in it while it
// moves, and leaves it once when its back face is past the zone; it has no contact with the zone nor the ground
func TestAKinematicBodyCrossesAStaticTrigger(t *testing.T) {
	w, zone, mover := kinematicZoneScene(1)
	log := listenTriggers(w).listenCollisions()
	var enterX, exitX float64
	entered, left := false, false
	drive(w, mover, mgl64.Vec3{crossSpeed, 0, 0}, 4*zoneHalf/crossSpeed, func() {
		if !entered && log.count(EventTriggerEnter, zone, mover) > 0 {
			entered, enterX = true, mover.Transform.Position.X()
		}
		if !left && log.count(EventTriggerExit, zone, mover) > 0 {
			left, exitX = true, mover.Transform.Position.X()
		}
	})
	if enter := log.count(EventTriggerEnter, zone, mover); enter != 1 {
		t.Fatalf("%d enters of the kinematic cube, want 1", enter)
	}
	if exit := log.count(EventTriggerExit, zone, mover); exit != 1 {
		t.Fatalf("%d exits of the kinematic cube, want 1", exit)
	}
	if stays := log.count(EventTriggerStay, zone, mover); stays == 0 {
		t.Errorf("no stay while the kinematic cube moved in the zone")
	}
	// the pair is tested at the start of a step, before the step moves the cube: within 2 steps of travel
	travel := 2 * crossSpeed * sceneDt
	if face := enterX + cubeHalf; face < -zoneHalf || face > -zoneHalf+travel {
		t.Errorf("the enter was sent with the front face at x=%.3f, want just past %.2f", face, -zoneHalf)
	}
	if face := exitX - cubeHalf; face < zoneHalf || face > zoneHalf+travel {
		t.Errorf("the exit was sent with the back face at x=%.3f, want just past %.2f", face, zoneHalf)
	}
	for _, record := range log.events {
		if record.eventType == EventCollisionEnter || record.eventType == EventCollisionStay || record.eventType == EventCollisionExit {
			t.Fatalf("collision event %v between bodies without mass", record.eventType)
		}
	}
}

// A kinematic cube stopped in a static zone falls asleep in it and stays in it: no exit and no stay during 10 s;
// removed from the world, it leaves once
func TestAKinematicBodyAsleepInATriggerStaysInIt(t *testing.T) {
	w, zone, mover := kinematicZoneScene(1)
	log := listenTriggers(w)
	drive(w, mover, mgl64.Vec3{crossSpeed, 0, 0}, 2*zoneHalf/crossSpeed, nil)
	simulate(w, settleTime, nil)
	if !mover.IsSleeping {
		t.Fatalf("the kinematic cube is awake %v s after its last target", settleTime)
	}
	stays := log.count(EventTriggerStay, zone, mover)
	simulate(w, stayTime, nil)
	if enter := log.count(EventTriggerEnter, zone, mover); enter != 1 {
		t.Errorf("%d enters, want 1", enter)
	}
	if exit := log.count(EventTriggerExit, zone, mover); exit != 0 {
		t.Errorf("%d exits while the kinematic cube sleeps in the zone, want 0", exit)
	}
	if more := log.count(EventTriggerStay, zone, mover) - stays; more != 0 {
		t.Errorf("%d stays while the kinematic cube sleeps in the zone, want 0", more)
	}
	w.RemoveBody(mover)
	w.Step(sceneDt)
	if exit := log.count(EventTriggerExit, zone, mover); exit != 1 {
		t.Errorf("%d exits at the step after the removal of the kinematic cube, want 1", exit)
	}
}

// A kinematic trigger driven over a static box and a kinematic cube enters each of them once and leaves each once,
// without any contact
func TestAKinematicTriggerDetectsStaticAndKinematicBodies(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.5)
	block := addBody(w, mgl64.Vec3{-1, 0.5, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.5, 0)
	still := kinematicBox(w, mgl64.Vec3{1, 0.5, 0}, cube().HalfExtents)
	zone := kinematicBox(w, mgl64.Vec3{-3, 0.5, 0}, mgl64.Vec3{0.5, 0.5, 0.5})
	zone.IsTrigger = true
	log := listenTriggers(w).listenCollisions()
	drive(w, zone, mgl64.Vec3{crossSpeed, 0, 0}, 6/crossSpeed, nil)
	for _, body := range []*actor.RigidBody{block, still} {
		if enter := log.count(EventTriggerEnter, zone, body); enter != 1 {
			t.Errorf("%d enters of the kinematic zone with a %v body, want 1", enter, body.BodyType)
		}
		if exit := log.count(EventTriggerExit, zone, body); exit != 1 {
			t.Errorf("%d exits of the kinematic zone with a %v body, want 1", exit, body.BodyType)
		}
	}
	if contacts := len(w.Contacts()); contacts != 0 {
		t.Errorf("%d contacts, want 0", contacts)
	}
	for _, record := range log.events {
		if record.eventType == EventCollisionEnter || record.eventType == EventCollisionStay || record.eventType == EventCollisionExit {
			t.Fatalf("collision event %v between bodies without mass", record.eventType)
		}
	}
}

// kinematicCrossing: kinematic cubes driven across a static zone at several depths, half of them stopping in it; the
// events of the types logged
func kinematicCrossing(workers int, eventTypes ...EventType) (*World, *triggerLog, func()) {
	w, _, _ := kinematicZoneScene(workers)
	w.parallelFrom = 1
	var movers []*actor.RigidBody
	for k := 0; k < 9; k++ {
		movers = append(movers, kinematicBox(w, mgl64.Vec3{-2*zoneHalf - 0.3*float64(k), 0.5, float64(k)*0.4 - 1.6}, cube().HalfExtents))
	}
	log := listenEvents(w, eventTypes...)
	step := func() {
		for k, mover := range movers {
			if k%2 == 1 && mover.Transform.Position.X() > 0 {
				continue
			}
			moveTo(mover, translated(mover.Transform, mgl64.Vec3{crossSpeed, 0, 0}))
		}
		w.Step(sceneDt)
	}
	return w, log, step
}

// The events of kinematic bodies are the same, in the same order, with 1 and 8 workers
func TestKinematicTriggerEventsAreDeterministic(t *testing.T) {
	_, one, step1 := kinematicCrossing(1, EventTriggerEnter, EventTriggerStay, EventTriggerExit)
	_, eight, step8 := kinematicCrossing(8, EventTriggerEnter, EventTriggerStay, EventTriggerExit)
	for i := 0; i < int(4*zoneHalf/crossSpeed/sceneDt)+60; i++ {
		step1()
		step8()
	}
	enters, exits := 0, 0
	for _, record := range one.events {
		switch record.eventType {
		case EventTriggerEnter:
			enters++
		case EventTriggerExit:
			exits++
		}
	}
	if enters != 9 || exits == 0 || exits == 9 {
		t.Fatalf("%d enters, %d exits: the scene doesn't test the kinematic bodies in and out of the zone", enters, exits)
	}
	if len(one.events) != len(eight.events) {
		t.Fatalf("%d events with 1 worker, %d with 8", len(one.events), len(eight.events))
	}
	first, second := serialIndices(one), serialIndices(eight)
	for i := range one.events {
		a, b := one.events[i], eight.events[i]
		if a.step != b.step || a.eventType != b.eventType || first[a.a] != second[b.a] || first[a.b] != second[b.b] {
			t.Fatalf("event %d: %+v with 1 worker, %+v with 8", i, a, b)
		}
	}
}

// A step with kinematic bodies moving in a zone, their enters and exits listened to, allocates nothing (an event
// allocates, as any value in an interface: the steps measured send none)
func TestKinematicTriggerPairsDoNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 8} {
		_, log, step := kinematicCrossing(workers, EventTriggerEnter, EventTriggerExit)
		// 2.5 s: the 9 cubes are in the zone, the first leaves it at 3.25 s
		for i := 0; i < int(2.5/sceneDt); i++ {
			step()
		}
		if enters := len(log.events); enters != 9 {
			t.Fatalf("workers=%d: %d enters, want the 9 cubes in the zone", workers, enters)
		}
		log.counting = true
		if allocs := testing.AllocsPerRun(10, step); allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
		if log.counted != 0 {
			t.Fatalf("workers=%d: %d events during the steps measured", workers, log.counted)
		}
	}
}

// A trigger mesh: a mesh has no core for GJK (its support point is null), so its overlap with a body is the one of its
// real contacts, as a plane or a heightfield. A sheet of 4 x 4 m at the height 1, its origin off the sheet: a crate
// falling through it and a kinematic cube driven down through it each enter it once and leave it once
func TestBodiesCrossATriggerMesh(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.5)
	sheet := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), gridMesh(4, 1, func(x, z float64) float64 { return 1 }), actor.BodyTypeStatic, 0, 0)
	sheet.IsTrigger = true
	crate := addBody(w, mgl64.Vec3{1.2, 2, 0.7}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	mover := kinematicBox(w, mgl64.Vec3{-1.2, 2, -0.7}, cube().HalfExtents)
	log := listenTriggers(w).listenCollisions()
	drive(w, mover, mgl64.Vec3{0, -crossSpeed, 0}, 1.5, nil)
	simulate(w, settleTime, nil)
	for _, body := range []*actor.RigidBody{crate, mover} {
		if enter := log.count(EventTriggerEnter, sheet, body); enter != 1 {
			t.Errorf("%d enters of body %d in the trigger mesh, want 1", enter, body.Serial())
		}
		if exit := log.count(EventTriggerExit, sheet, body); exit != 1 {
			t.Errorf("%d exits of body %d from the trigger mesh, want 1", exit, body.Serial())
		}
		if collisions := log.count(EventCollisionEnter, sheet, body); collisions != 0 {
			t.Errorf("%d collisions of body %d with the trigger mesh", collisions, body.Serial())
		}
	}
	if !crate.IsSleeping || crate.Transform.Position.Y() > 2*cubeHalf {
		t.Errorf("the crate is at y=%.3f, asleep %v: want asleep on the ground, under the sheet", crate.Transform.Position.Y(), crate.IsSleeping)
	}
}
