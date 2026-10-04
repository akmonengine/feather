package feather

import (
	"errors"
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// The shape and the density of a body changed in place (World.SetShape, World.SetDensity): the body keeps its index,
// its proxy, its joints, its filters and its island; its mass and its inertia follow; it wakes up with the bodies
// around it, and the pairs it had are tested again.

// restTolerance: how far (m) a resting body may sit from its resting height, the contact being soft; landingTolerance:
// how far it may sink under it when it lands from a fall (the spring of the contact absorbs the impact)
const (
	restTolerance    = 2 * LinearSlop
	landingTolerance = 4 * LinearSlop
)

// settleAt runs the scene for 2 s and fails if the body ever goes under the height it should rest at (or under its
// start, when it starts overlapping the ground) by more than a landing, then checks it rests at floor
func settleAt(t *testing.T, w *World, body *actor.RigidBody, floor float64) {
	t.Helper()
	lowest := min(body.Transform.Position.Y(), floor)
	simulate(w, 2, func() {
		if y := body.Transform.Position.Y(); y < lowest-landingTolerance {
			t.Fatalf("the body went down to %.4f, under %.4f: it goes through the ground", y, lowest)
		}
	})
	if y := body.Transform.Position.Y(); math.Abs(y-floor) > restTolerance {
		t.Fatalf("the body rests at %.4f, want %.4f", y, floor)
	}
}

// requireMassOfShape: the mass and the inertia of the body are those of its shape at its density, to the bit
func requireMassOfShape(t *testing.T, body *actor.RigidBody) {
	t.Helper()
	wantMass := body.Shape.ComputeMass(body.Material.Density)
	if body.Material.GetMass() != wantMass {
		t.Fatalf("mass %v, want the mass of the shape %v", body.Material.GetMass(), wantMass)
	}
	if body.InertiaLocal != body.Shape.ComputeInertia(wantMass) {
		t.Fatalf("inertia %v, want the inertia of the shape %v", body.InertiaLocal, body.Shape.ComputeInertia(wantMass))
	}
	if body.InverseInertiaLocal != body.InertiaLocal.Inv() {
		t.Fatalf("inverse inertia %v, want the inverse of %v", body.InverseInertiaLocal, body.InertiaLocal)
	}
}

// 1. A resting cube becomes a box twice as big, then a sphere, then a taller capsule: it stays stable, never goes
// through the ground, rests at the height of its new shape; its mass and its inertia are those of the new shape
func TestSetShapeOfARestingBodyKeepsItStable(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	body := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 1, nil)
	if !body.IsSleeping {
		t.Fatal("the cube should rest asleep")
	}
	index := func() int {
		for i, b := range w.Bodies {
			if b == body {
				return i
			}
		}
		return -1
	}
	wasAt := index()
	for _, next := range []struct {
		name  string
		shape actor.ShapeInterface
		floor float64
	}{
		{"a box twice as big", &actor.Box{HalfExtents: mgl64.Vec3{2 * cubeHalf, 2 * cubeHalf, 2 * cubeHalf}}, 2 * cubeHalf},
		{"a sphere", &actor.Sphere{Radius: cubeHalf}, cubeHalf},
		{"a taller capsule", &actor.Capsule{Radius: cubeHalf, HalfHeight: 2 * cubeHalf}, 3 * cubeHalf},
	} {
		if err := w.SetShape(body, next.shape); err != nil {
			t.Fatalf("%s: %v", next.name, err)
		}
		if body.Shape != next.shape {
			t.Fatalf("%s: the body has the shape %T", next.name, body.Shape)
		}
		if body.IsSleeping {
			t.Fatalf("%s: the body is still asleep", next.name)
		}
		if body.AABB() != next.shape.ComputeAABB(body.Transform) {
			t.Fatalf("%s: the AABB %v is not the one of the shape", next.name, body.AABB())
		}
		requireMassOfShape(t, body)
		if index() != wasAt {
			t.Fatalf("%s: the body moved from the index %d to %d", next.name, wasAt, index())
		}
		settleAt(t, w, body, next.floor)
		if !body.IsSleeping {
			t.Fatalf("%s: the body should fall asleep again", next.name)
		}
	}
}

// 1b. The contacts of the body are computed again at the step after the change, not taken from the pair cache: a
// cube resting awake made a sphere of the same height has one contact point with the ground at the next step, not the
// four corners of the cube moved with the bodies
func TestSetShapeComputesTheContactsAgain(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	body := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, actor.DefaultTimeToSleep/2, nil)
	if body.IsSleeping {
		t.Fatal("the cube is asleep already: the pair cache is empty, the test proves nothing")
	}
	points := func() int {
		n := 0
		for _, contact := range w.Contacts() {
			if contact.BodyA == body || contact.BodyB == body {
				n += contact.Count
			}
		}
		return n
	}
	if points() != 4 {
		t.Fatalf("the resting cube has %d contact points, want its 4 corners", points())
	}
	if err := w.SetShape(body, &actor.Sphere{Radius: cubeHalf}); err != nil {
		t.Fatal(err)
	}
	w.Step(sceneDt)
	if points() != 1 {
		t.Fatalf("the sphere has %d contact points at the step after the change, want 1", points())
	}
}

// 2. A ball hanging from a static anchor by a distance joint becomes a box: the joint holds, the rope keeps its length
func TestSetShapeKeepsTheJoints(t *testing.T) {
	w := newScene(1)
	anchor := addBody(w, mgl64.Vec3{0, 3, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.02}, actor.BodyTypeStatic, 0, 0)
	const rope = 1.0
	ball := addBody(w, anchor.Transform.Position.Sub(mgl64.Vec3{0, rope, 0}), mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0.5, 0)
	w.AddJoint(NewDistanceJoint(anchor, ball, anchor.Transform.Position, ball.Transform.Position))
	ball.Velocity = mgl64.Vec3{1.5, 0, 0} // it swings
	simulate(w, 1, nil)
	if err := w.SetShape(ball, &actor.Box{HalfExtents: mgl64.Vec3{0.15, 0.15, 0.15}}); err != nil {
		t.Fatal(err)
	}
	if len(w.Joints) != 1 {
		t.Fatalf("%d joints, want the rope", len(w.Joints))
	}
	const tolerance = 0.02
	simulate(w, 2, func() {
		if length := ball.Transform.Position.Sub(anchor.Transform.Position).Len(); math.Abs(length-rope) > tolerance {
			t.Fatalf("the rope is %.3f m long, want %.1f", length, rope)
		}
	})
}

// 3. A sleeping plate carrying a crate becomes thinner: the plate and the crate wake up (the crate is in the air), the
// crate lands on the thinner plate
func TestSetShapeWakesTheBodyAndItsNeighbors(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	const thick, thin = 0.1, 0.02
	plate := addBody(w, mgl64.Vec3{0, thick, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, thick, 1}}, actor.BodyTypeDynamic, 0.6, 0)
	crate := addBody(w, mgl64.Vec3{0.3, 2*thick + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 1.5, nil)
	if !plate.IsSleeping || !crate.IsSleeping {
		t.Fatalf("the plate and the crate should sleep: %v %v", plate.IsSleeping, crate.IsSleeping)
	}
	wakes := 0
	w.Events.Subscribe(EventWake, func(Event) { wakes++ })
	if err := w.SetShape(plate, &actor.Box{HalfExtents: mgl64.Vec3{1, thin, 1}}); err != nil {
		t.Fatal(err)
	}
	if plate.IsSleeping || crate.IsSleeping {
		t.Fatalf("after the change: plate asleep %v, crate asleep %v, want both awake", plate.IsSleeping, crate.IsSleeping)
	}
	// the plate falls on the ground by thick - thin, the crate lands on it
	settleAt(t, w, crate, 2*thin+cubeHalf)
	if math.Abs(plate.Transform.Position.Y()-thin) > restTolerance {
		t.Errorf("the plate rests at %.4f, want %.4f", plate.Transform.Position.Y(), thin)
	}
}

// 4. The shape of a static body changes under a sleeping crate: a wide box made a small one lets the crate fall to the
// ground; the ground plane made a box lifts the crate (the proxy leaves the planes for the static tree), made a plane
// again it lets it down (the proxy goes back)
func TestSetShapeOfAStaticBodyUpdatesItsPairs(t *testing.T) {
	w := newScene(1)
	ground := addGround(w, 0.6)
	const top = 0.5
	pedestal := addBody(w, mgl64.Vec3{0, top / 2, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, top / 2, 1}}, actor.BodyTypeStatic, 0.6, 0)
	crate := addBody(w, mgl64.Vec3{0.5, top + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 1, nil)
	if !crate.IsSleeping {
		t.Fatal("the crate should sleep on the pedestal")
	}
	// the pedestal shrinks to a post under its own center: the crate at x = 0.5 is in the air
	if err := w.SetShape(pedestal, &actor.Box{HalfExtents: mgl64.Vec3{0.1, top / 2, 0.1}}); err != nil {
		t.Fatal(err)
	}
	if crate.IsSleeping {
		t.Fatal("the crate is still asleep over the post")
	}
	settleAt(t, w, crate, cubeHalf)
	// the ground becomes a slab of 2 x top: its top face is at top, the crate is lifted
	if err := w.SetShape(ground, &actor.Box{HalfExtents: mgl64.Vec3{5, top, 5}}); err != nil {
		t.Fatal(err)
	}
	if crate.IsSleeping {
		t.Fatal("the crate is still asleep in the slab")
	}
	settleAt(t, w, crate, top+cubeHalf)
	if err := consistentOrder(w); err != nil {
		t.Fatal(err)
	}
	// and a plane again: the crate falls back
	if err := w.SetShape(ground, &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}); err != nil {
		t.Fatal(err)
	}
	settleAt(t, w, crate, cubeHalf)
	if err := consistentOrder(w); err != nil {
		t.Fatal(err)
	}
}

// 5. A trigger box holding a sleeping crate in its corner becomes its inscribed sphere, with the same AABB to the bit:
// the overlap is tested again (a pair at rest keeps it otherwise), the crate leaves the zone once
func TestSetShapeOfATriggerRetestsItsOverlap(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	const half, small = 0.5, 0.05
	crate := addBody(w, mgl64.Vec3{half - small, small, half - small}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{small, small, small}}, actor.BodyTypeDynamic, 0.6, 0)
	zone := addBody(w, mgl64.Vec3{0, small, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{half, half, half}}, actor.BodyTypeStatic, 0, 0)
	zone.IsTrigger = true
	log := listenTriggers(w)
	simulate(w, 1, nil)
	if !crate.IsSleeping {
		t.Fatal("the crate should sleep in the zone")
	}
	if enters, exits := log.count(EventTriggerEnter, zone, crate), log.count(EventTriggerExit, zone, crate); enters != 1 || exits != 0 {
		t.Fatalf("%d enters & %d exits before the change, want 1 & 0", enters, exits)
	}
	sphere := &actor.Sphere{Radius: half}
	if sphere.ComputeAABB(zone.Transform) != zone.AABB() {
		t.Fatalf("the sphere has the AABB %v, the box %v: the test needs the same", sphere.ComputeAABB(zone.Transform), zone.AABB())
	}
	if err := w.SetShape(zone, sphere); err != nil {
		t.Fatal(err)
	}
	simulate(w, 0.5, nil)
	if exits := log.count(EventTriggerExit, zone, crate); exits != 1 {
		t.Fatalf("%d exits after the zone became a sphere, want 1: the corner of the box is out of the sphere", exits)
	}
	// and the box again: the crate is in it again
	if err := w.SetShape(zone, &actor.Box{HalfExtents: mgl64.Vec3{half, half, half}}); err != nil {
		t.Fatal(err)
	}
	simulate(w, 0.5, nil)
	if enters := log.count(EventTriggerEnter, zone, crate); enters != 2 {
		t.Fatalf("%d enters after the zone became a box again, want 2", enters)
	}
}

// 6. A shape which cannot be simulated is refused with an error, the body left as it is: no shape, a plane, a
// heightfield or a mesh on a dynamic or a kinematic body; the same shape again does nothing; a refusal allocates nothing
func TestSetShapeRefusesWhatCannotBeSimulated(t *testing.T) {
	w := newScene(1)
	dynamic := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	kinematic := kinematicBox(w, mgl64.Vec3{2, 1, 0}, cube().HalfExtents)
	static := addBody(w, mgl64.Vec3{4, 1, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
	mesh, err := actor.NewTriangleMesh([]mgl64.Vec3{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}}, []int32{0, 2, 1, 0, 3, 2})
	if err != nil {
		t.Fatal(err)
	}
	expectError := func(what string, want error, fn func() error) {
		t.Helper()
		if err := fn(); !errors.Is(err, want) {
			t.Errorf("%s: error %v, want %v", what, err, want)
		}
	}
	plane := &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}
	expectError("no shape on a dynamic body", ErrNoShape, func() error { return w.SetShape(dynamic, nil) })
	expectError("no shape on a static body", ErrNoShape, func() error { return w.SetShape(static, nil) })
	expectError("a plane on a dynamic body", ErrShapeNotConvex, func() error { return w.SetShape(dynamic, plane) })
	expectError("a mesh on a dynamic body", ErrShapeNotConvex, func() error { return w.SetShape(dynamic, mesh) })
	expectError("a mesh on a kinematic body", ErrShapeNotConvex, func() error { return w.SetShape(kinematic, mesh) })
	for _, body := range []*actor.RigidBody{dynamic, kinematic, static} {
		if _, isBox := body.Shape.(*actor.Box); !isBox {
			t.Errorf("a refused body changed its shape to %T", body.Shape)
		}
	}
	// a static body takes a mesh and a plane
	if err := w.SetShape(static, mesh); err != nil || static.Shape != mesh {
		t.Errorf("a mesh on a static body: %v, shape %T", err, static.Shape)
	}
	if err := w.SetShape(static, plane); err != nil || static.Shape != plane {
		t.Errorf("a plane on a static body: %v, shape %T", err, static.Shape)
	}
	// the same shape again: nothing happens, the sleeping body stays asleep
	simulate(w, 1, nil)
	shape := dynamic.Shape
	if err := w.SetShape(dynamic, shape); err != nil {
		t.Errorf("the same shape again: %v", err)
	}
	if allocs := testing.AllocsPerRun(10, func() { _ = w.SetShape(dynamic, mesh) }); allocs > 0 {
		t.Errorf("a refusal allocates %.1f, want 0", allocs)
	}
}

// 7. The density of a body changed in place: the mass and the inertia of its shape at the new density, the same
// impulse gives half the velocity at twice the density; a sleeping body wakes up; a static body keeps the density
// written, without mass; a density which is not positive is refused on a dynamic body
func TestSetDensityRecomputesTheMassAndTheInertia(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	body := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	static := addBody(w, mgl64.Vec3{3, 1, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
	kinematic := kinematicBox(w, mgl64.Vec3{6, 1, 0}, cube().HalfExtents)
	before := body.Material.GetMass()
	if err := w.SetDensity(body, 2*body.Material.Density); err != nil {
		t.Fatal(err)
	}
	requireMassOfShape(t, body)
	if body.Material.GetMass() != 2*before {
		t.Fatalf("mass %v at twice the density, want %v", body.Material.GetMass(), 2*before)
	}
	body.AddImpulse(mgl64.Vec3{1, 0, 0})
	w.Step(sceneDt)
	if want := 1 / (2 * before); math.Abs(body.Velocity.X()-want) > 1e-12 {
		t.Errorf("velocity %v after an impulse of 1 N·s, want 1 / mass = %v", body.Velocity.X(), want)
	}
	body.Velocity = mgl64.Vec3{}
	simulate(w, actor.DefaultTimeToSleep+3*sceneDt, nil)
	if !body.IsSleeping {
		t.Fatal("the body at rest should sleep")
	}
	if err := w.SetDensity(body, 500); err != nil || body.IsSleeping {
		t.Fatalf("a sleeping body given a density: %v, asleep %v", err, body.IsSleeping)
	}
	// a static body keeps the density, for the day it becomes dynamic, and its infinite mass
	if err := w.SetDensity(static, 700); err != nil {
		t.Fatal(err)
	}
	if static.Material.Density != 700 || !math.IsInf(static.Material.GetMass(), 1) || static.InverseMass() != 0 {
		t.Errorf("a static body: density %v, mass %v, inverse mass %v, want 700, +Inf, 0", static.Material.Density, static.Material.GetMass(), static.InverseMass())
	}
	// a kinematic body gets the mass of its shape, for the day it becomes dynamic; the solver never reads it
	if err := w.SetDensity(kinematic, 700); err != nil {
		t.Fatal(err)
	}
	requireMassOfShape(t, kinematic)
	if kinematic.InverseMass() != 0 {
		t.Errorf("a kinematic body has an inverse mass %v, want 0", kinematic.InverseMass())
	}
	for _, density := range []float64{0, -1, math.NaN()} {
		if err := w.SetDensity(body, density); !errors.Is(err, ErrMasslessBody) {
			t.Errorf("density %v on a dynamic body: %v, want ErrMasslessBody", density, err)
		}
	}
	if body.Material.Density != 500 {
		t.Errorf("a refused density changed the body: %v", body.Material.Density)
	}
	if allocs := testing.AllocsPerRun(10, func() { _ = w.SetDensity(body, 600) }); allocs > 0 {
		t.Errorf("SetDensity allocates %.1f, want 0", allocs)
	}
}

// 8. The mixed scene with shapes and densities changed during the run gives the same bits with 1, 4 and 8 workers: a
// change every 8 steps, a sphere, a box, a capsule, a density in turn, on 30 bodies over 4 s (the static anchors of
// the chains, the chains, the linked boxes and the bodies falling on them). The changes keep the scene awake: a run of
// 1000 steps with a change every 25 steps had no body asleep either
func TestSetShapeIsDeterministic(t *testing.T) {
	const setShapeSteps, setShapeEvery = 240, 8
	steps := setShapeSteps
	if raceEnabled {
		steps = min(steps, raceSteps)
	}
	changed := 0
	compareRuns(t, steps, func(workers int) (*World, func(step int)) {
		w := mixedScene(workers)
		if workers == determinismWorkers[0] {
			changed = 0
		}
		return w, func(step int) {
			if step%setShapeEvery != setShapeEvery/2 {
				return
			}
			// a body of the scene, by turns; the chains are the first bodies, the pile the last ones
			body := w.Bodies[(step/setShapeEvery*7)%len(w.Bodies)]
			var err error
			switch (step / setShapeEvery) % 4 {
			case 0:
				err = w.SetShape(body, &actor.Sphere{Radius: 0.08})
			case 1:
				err = w.SetShape(body, &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.06, 0.08}})
			case 2:
				err = w.SetShape(body, &actor.Capsule{Radius: 0.05, HalfHeight: 0.1})
			default:
				if body.BodyType == actor.BodyTypeDynamic {
					err = w.SetDensity(body, body.Material.Density*1.5)
				}
			}
			if err != nil {
				t.Fatalf("step %d: %v", step, err)
			}
			if workers == determinismWorkers[0] {
				changed++
			}
		}
	})
	if want := steps/setShapeEvery - 1; changed < want {
		t.Fatalf("%d changes over the run, want %d at least: the scene doesn't cover them", changed, want)
	}
}

// 9. A step after a change of shape allocates nothing, with 1 and 8 workers: three crates standing on three cubes, the
// cubes made lower and back (a box, its top 10 % lower) at every step and given another density at every third, in a
// steady state — the buffers of the islands, of the pairs and of the contacts have grown for the bodies the changes keep
// awake (changedWarmup steps). Changes walking a pile would wake a new part of it at each step, and the buffers, sized to
// the contacts of a step, would grow with it: an allocation of the step, not of the change
func TestStepAfterSetShapeAllocatesNothing(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	const changedWarmup, stacks = 240, 3
	for _, workers := range []int{1, 8} {
		w := newScene(workers)
		addGround(w, 0.6)
		cube := &actor.Box{HalfExtents: mgl64.Vec3{cubeHalf, cubeHalf, cubeHalf}}
		var changed []*actor.RigidBody
		for i := range stacks {
			x := float64(i-1) * 2
			changed = append(changed, addBody(w, mgl64.Vec3{x, cubeHalf, 0}, mgl64.QuatIdent(), cube, actor.BodyTypeDynamic, 0.6, 0))
			addBody(w, mgl64.Vec3{x, 3 * cubeHalf, 0}, mgl64.QuatIdent(), cube, actor.BodyTypeDynamic, 0.6, 0)
		}
		simulate(w, 1, nil)
		shapes := []actor.ShapeInterface{&actor.Box{HalfExtents: mgl64.Vec3{cubeHalf, 0.9 * cubeHalf, cubeHalf}}, cube}
		turn := 0
		step := func() {
			body := changed[turn%stacks]
			if err := w.SetShape(body, shapes[(turn/stacks)%2]); err != nil {
				t.Fatal(err)
			}
			if turn%3 == 0 {
				if err := w.SetDensity(body, 400+float64(turn%7)); err != nil {
					t.Fatal(err)
				}
			}
			turn++
			w.Step(sceneDt)
		}
		for range changedWarmup {
			step()
		}
		if allocs := testing.AllocsPerRun(10, step); allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
	}
}
