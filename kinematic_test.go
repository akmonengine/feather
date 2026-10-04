package feather

import (
	"errors"
	"fmt"
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== KINEMATIC BODIES ==========
// A kinematic body goes where the game tells it (SetKinematicTarget: the pose it reaches at the end of the next step),
// over the sub-steps of the step, and pushes the dynamic bodies with the velocity of this motion. Nothing moves it.
// The bounds come from the engine: landingDepth (LinearSlop) for a depth, SpeculativeDistance for a contact kept, and
// the bits for what must not change at all.

const (
	// kinematicSpeed: the speed of the platforms and pushers of these scenes (m/s)
	kinematicSpeed = 1.0

	// legSpeed: the speed of the leg of a creature at a run, the criterion of #798 (m/s)
	legSpeed = 5.0
)

// kinematicBox: a kinematic box of half extents half, friction 0.6
func kinematicBox(w *World, position mgl64.Vec3, half mgl64.Vec3) *actor.RigidBody {
	return addBody(w, position, mgl64.QuatIdent(), &actor.Box{HalfExtents: half}, actor.BodyTypeKinematic, 0.6, 0)
}

// translated: the transform moved by the velocity during a step
func translated(transform actor.Transform, velocity mgl64.Vec3) actor.Transform {
	transform.Position = transform.Position.Add(velocity.Mul(sceneDt))
	return transform
}

// moveTo gives a kinematic body its target for the next step; the body is kinematic, the call can't fail
func moveTo(body *actor.RigidBody, target actor.Transform) {
	if err := body.SetKinematicTarget(target); err != nil {
		panic(err)
	}
}

// drive runs the scene for seconds, with the kinematic body moving at the velocity (a target before each step), and
// calls each after each step
func drive(w *World, body *actor.RigidBody, velocity mgl64.Vec3, seconds float64, each func()) {
	for i := 0; i < int(math.Round(seconds/sceneDt)); i++ {
		moveTo(body, translated(body.Transform, velocity))
		w.Step(sceneDt)
		if each != nil {
			each()
		}
	}
}

// faceGap: the gap along X between the front face of the pusher and the back face of the box (both cubes, not turned)
func faceGap(pusher, box *actor.RigidBody) float64 {
	return (box.Transform.Position.X() - cubeHalf) - (pusher.Transform.Position.X() + cubeHalf)
}

// ========== ACCEPTANCE ==========

// 1. A kinematic cube sliding on the ground pushes a dynamic cube resting in front of it: the cube takes its speed,
// rubbing on the ground, and stays against the pusher (within the speculative distance, never in it), flat on the ground
func TestKinematicPushesABox(t *testing.T) {
	w := newScene(1)
	ground := addGround(w, 0.6)
	pusher := kinematicBox(w, mgl64.Vec3{0, cubeHalf, 0}, cube().HalfExtents)
	box := addBody(w, mgl64.Vec3{2*cubeHalf + 0.01, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)

	worstGap, worstDepth, elapsed := 0.0, 0.0, 0.0
	drive(w, pusher, mgl64.Vec3{kinematicSpeed, 0, 0}, 2, func() {
		elapsed += sceneDt
		if gap := faceGap(pusher, box); elapsed > 0.1 {
			// in contact after the first centimeter: pushed, neither left behind nor entered
			worstGap = math.Max(worstGap, gap)
			worstDepth = math.Max(worstDepth, -gap)
		}
		worstDepth = math.Max(worstDepth, surfaceDepth(ground, box))
	})
	t.Logf("gap up to %.2f mm, depth up to %.2f mm, box at %.3f m/s", worstGap*1000, worstDepth*1000, box.Velocity.X())
	if worstGap > SpeculativeDistance {
		t.Errorf("the box got %.1f mm ahead of the pusher: not pushed", worstGap*1000)
	}
	if worstDepth > landingDepth {
		t.Errorf("the box went %.2f mm into the pusher or the ground", worstDepth*1000)
	}
	// the box has the speed of the pusher: it loses at most the friction of the ground during one step between two pushes
	if slip := kinematicSpeed - box.Velocity.X(); slip > 0.6*sceneGravity*sceneDt || slip < -0.6*sceneGravity*sceneDt {
		t.Errorf("the box moves at %.3f m/s, want the %.0f m/s of the pusher", box.Velocity.X(), kinematicSpeed)
	}
	if angle := rotationAngle(box.Transform.Rotation); angle > 1e-3 {
		t.Errorf("the box turned by %.4f rad while pushed flat on the ground", angle)
	}
}

// 2. A kinematic body is not moved by what falls on it: a cube 100 times heavier than the platform would be, and a
// stack of 10 cubes, land on a kinematic plate in the air. Its transform and its velocity are the same bit for bit at
// every step, with or without a target at its own pose
func TestDynamicBodiesDoNotMoveAKinematic(t *testing.T) {
	for _, targeted := range []bool{false, true} {
		w := newScene(1)
		plate := kinematicBox(w, mgl64.Vec3{0, 2, 0}, mgl64.Vec3{1, 0.1, 1})
		start := plate.Transform
		heavy := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0.5, 3, 0.5}, Rotation: mgl64.QuatIdent()}, cube(), actor.BodyTypeDynamic, 50000)
		heavy.Material.StaticFriction, heavy.Material.DynamicFriction = 0.6, 0.6
		w.AddBody(heavy)
		for i := 0; i < 10; i++ {
			addBody(w, mgl64.Vec3{-0.5, 2.1 + cubeHalf + float64(i)*2*cubeHalf, -0.5}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		}
		for step := 0; step < 120; step++ {
			if targeted {
				moveTo(plate, start)
			}
			w.Step(sceneDt)
			if plate.Transform != start {
				t.Fatalf("targeted=%v, step %d: the plate moved to %v, want %v bit for bit", targeted, step, plate.Transform, start)
			}
			if plate.Velocity != (mgl64.Vec3{}) || plate.AngularVelocity != (mgl64.Vec3{}) {
				t.Fatalf("targeted=%v, step %d: the plate has a velocity %v %v", targeted, step, plate.Velocity, plate.AngularVelocity)
			}
		}
		if heavy.Transform.Position.Y() < 2.1+cubeHalf-landingDepth {
			t.Errorf("targeted=%v: the heavy cube sank to y = %.4f through the plate", targeted, heavy.Transform.Position.Y())
		}
	}
}

// 3. The velocity of a kinematic body is the one of its motion: (target - pose) / dt, and the axis of the rotation
// to the target times its angle / dt. The body is at its target at the end of the step, bit for bit. Without a target,
// it doesn't move and its velocity is 0
func TestKinematicVelocityIsItsMotion(t *testing.T) {
	w := newScene(1)
	body := kinematicBox(w, mgl64.Vec3{0, 1, 0}, cube().HalfExtents)
	velocity := mgl64.Vec3{1, 0.5, -0.25}
	axis := mgl64.Vec3{0.3, 1, 0.2}.Normalize()
	const spin = 2.0 // rad/s
	for step := 0; step < 60; step++ {
		before := body.Transform
		target := actor.Transform{
			Position: before.Position.Add(velocity.Mul(sceneDt)),
			Rotation: mgl64.QuatRotate(spin*sceneDt, axis).Mul(before.Rotation).Normalize(),
		}
		moveTo(body, target)
		w.Step(sceneDt)
		if body.Transform != target {
			t.Fatalf("step %d: the body is at %v, want its target %v bit for bit", step, body.Transform, target)
		}
		wantVelocity := target.Position.Sub(before.Position).Mul(1 / sceneDt)
		if body.Velocity.Sub(wantVelocity).Len() > 1e-9 {
			t.Fatalf("step %d: velocity %v, want (target - pose) / dt = %v", step, body.Velocity, wantVelocity)
		}
		if body.AngularVelocity.Sub(axis.Mul(spin)).Len() > 1e-9 {
			t.Fatalf("step %d: angular velocity %v, want %v", step, body.AngularVelocity, axis.Mul(spin))
		}
	}
	// no target: nothing moves, no velocity
	last := body.Transform
	w.Step(sceneDt)
	if body.Transform != last || body.Velocity != (mgl64.Vec3{}) || body.AngularVelocity != (mgl64.Vec3{}) {
		t.Errorf("without a target: moved from %v to %v, velocity %v %v", last, body.Transform, body.Velocity, body.AngularVelocity)
	}
}

// 4. Teleport places a body without any velocity: a kinematic cube teleported against a resting cube (within the
// speculative distance) doesn't push it: the cube keeps the velocity of its own contact with the ground, the rounding
// (1e-19 m/s along X). The same cube brought there by a target pushes it
func TestTeleportGivesNoVelocity(t *testing.T) {
	for _, teleport := range []bool{true, false} {
		w := newScene(1)
		addGround(w, 0.6)
		box := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		pusher := kinematicBox(w, mgl64.Vec3{-3, cubeHalf, 0}, cube().HalfExtents)
		simulate(w, 1, nil)
		against := actor.Transform{Position: mgl64.Vec3{-2*cubeHalf - SpeculativeDistance/2, cubeHalf, 0}, Rotation: mgl64.QuatIdent()}
		if teleport {
			w.Teleport(pusher, against)
		} else {
			moveTo(pusher, against)
		}
		const rounding = 1e-9 // a nanometer (per second): the rounding of the contact with the ground, not a push
		pushed := false
		for step := 0; step < 30; step++ {
			w.Step(sceneDt)
			pushed = pushed || box.Velocity.X() > rounding || box.Transform.Position.X() > rounding
		}
		if pusher.Transform != against {
			t.Errorf("teleport=%v: the pusher is at %v, want %v", teleport, pusher.Transform, against)
		}
		if teleport && pushed {
			t.Errorf("a teleported kinematic pushed the box: velocity %v, position %v", box.Velocity, box.Transform.Position)
		}
		if teleport && pusher.Velocity != (mgl64.Vec3{}) {
			t.Errorf("a teleported kinematic has a velocity %v", pusher.Velocity)
		}
		if !teleport && !pushed {
			t.Errorf("a kinematic brought by a target didn't push the box")
		}
	}
}

// 5. No contact between a kinematic body and a static or a kinematic body: a kinematic cube overlapping the ground, a
// static box and another kinematic cube has no contact and sends no event with them, and only pushes the dynamic cube
func TestNoContactBetweenKinematicAndStaticOrKinematic(t *testing.T) {
	w := newScene(1)
	ground := addGround(w, 0.6)
	wall := addBody(w, mgl64.Vec3{0.3, 0.5, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.5, 1}}, actor.BodyTypeStatic, 0.6, 0)
	mover := kinematicBox(w, mgl64.Vec3{0, 0.1, 0}, cube().HalfExtents) // in the ground and the wall
	other := kinematicBox(w, mgl64.Vec3{0.1, 0.3, 0}, cube().HalfExtents)
	dynamic := addBody(w, mgl64.Vec3{0, 0.35 + 2*cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	events := 0
	w.Events.Subscribe(EventCollisionEnter, func(e Event) {
		pair := e.(CollisionEnterEvent)
		if pair.BodyA != dynamic && pair.BodyB != dynamic {
			t.Errorf("collision event between %v and %v, both without mass", pair.BodyA.BodyType, pair.BodyB.BodyType)
		}
		events++
	})
	w.Events.Subscribe(EventCollisionStay, func(Event) { events++ })
	drive(w, mover, mgl64.Vec3{0, 0, 0.1}, 1, func() {
		moveTo(other, translated(other.Transform, mgl64.Vec3{0, 0, 0.1}))
		for _, contact := range w.Contacts() {
			if contact.BodyA.BodyType != actor.BodyTypeDynamic && contact.BodyB.BodyType != actor.BodyTypeDynamic {
				t.Fatalf("contact between a %v and a %v", contact.BodyA.BodyType, contact.BodyB.BodyType)
			}
		}
	})
	_, _ = ground, wall
	if events == 0 {
		t.Errorf("no collision event with the dynamic cube carried by the kinematic one")
	}
	if dynamic.Transform.Position.Z() < 0.05 {
		t.Errorf("the dynamic cube was not carried along Z: z = %.3f", dynamic.Transform.Position.Z())
	}
}

// segmentsDistance: the distance between 2 segments (Ericson 5.1.9, written here to measure the engine from outside)
func segmentsDistance(a0, a1, b0, b1 mgl64.Vec3) float64 {
	d1, d2, r := a1.Sub(a0), b1.Sub(b0), a0.Sub(b0)
	a, e, f := d1.Dot(d1), d2.Dot(d2), d2.Dot(r)
	c, b := d1.Dot(r), d1.Dot(d2)
	denominator := a*e - b*b
	s := 0.0
	if denominator > 1e-12 {
		s = math.Max(0, math.Min(1, (b*f-c*e)/denominator))
	}
	t := (b*s + f) / e
	if t < 0 {
		t, s = 0, math.Max(0, math.Min(1, -c/a))
	} else if t > 1 {
		t, s = 1, math.Max(0, math.Min(1, (b-c)/a))
	}
	return a0.Add(d1.Mul(s)).Sub(b0.Add(d2.Mul(t))).Len()
}

// capsuleDepth: how deep 2 capsules overlap, 0 apart
func capsuleDepth(a, b *actor.RigidBody) float64 {
	segment := func(body *actor.RigidBody) (mgl64.Vec3, mgl64.Vec3, float64) {
		capsule := body.Shape.(*actor.Capsule)
		axis := body.Transform.Rotation.Rotate(mgl64.Vec3{0, capsule.HalfHeight, 0})
		return body.Transform.Position.Sub(axis), body.Transform.Position.Add(axis), capsule.Radius
	}
	a0, a1, radiusA := segment(a)
	b0, b1, radiusB := segment(b)
	return math.Max(0, radiusA+radiusB-segmentsDistance(a0, a1, b0, b1))
}

// 7. A kinematic leg at 5 m/s (8 cm per step at 60 Hz, 8 sub-steps) meets a dynamic body at rest on its path: the
// contact exists before they touch, from the speed of the kinematic body (speculative), and the dynamic body is pushed
// without being entered (landingDepth) nor gone through. A capsule against a capsule lying across (the leg of a Slyf
// against its tail), and a box against a box
func TestFastKinematicDoesNotGoThroughARestingBody(t *testing.T) {
	cases := []struct {
		name  string
		leg   actor.ShapeInterface
		tail  actor.ShapeInterface
		depth func(a, b *actor.RigidBody) float64
	}{
		{"capsules", &actor.Capsule{HalfHeight: 0.2, Radius: 0.05}, &actor.Capsule{HalfHeight: 0.3, Radius: 0.05}, capsuleDepth},
		{"boxes", &actor.Box{HalfExtents: mgl64.Vec3{0.05, 0.25, 0.05}}, &actor.Box{HalfExtents: mgl64.Vec3{0.05, 0.05, 0.35}}, boxOverlap},
	}
	for _, c := range cases {
		w := newScene(1)
		addGround(w, 0.6)
		legAABB := c.leg.ComputeAABB(actor.NewTransform())
		tailAABB := c.tail.ComputeAABB(actor.NewTransform())
		leg := addBody(w, mgl64.Vec3{0, -legAABB.Min.Y() + 0.001, 0}, mgl64.QuatIdent(), c.leg, actor.BodyTypeKinematic, 0.6, 0)
		tailRotation := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{1, 0, 0}) // the capsule lies along Z
		if _, isBox := c.tail.(*actor.Box); isBox {
			tailRotation = mgl64.QuatIdent()
		}
		tail := addBody(w, mgl64.Vec3{0.5, -tailAABB.Min.Y(), 0}, tailRotation, c.tail, actor.BodyTypeDynamic, 0.6, 0)
		tail.Transform.Position[1] = tail.Shape.ComputeAABB(actor.Transform{Rotation: tailRotation}).Max.Y()
		tail.UpdateAABB()
		simulate(w, 0.5, nil) // the tail settles

		worst, through := 0.0, false
		drive(w, leg, mgl64.Vec3{legSpeed, 0, 0}, 0.4, func() {
			worst = math.Max(worst, c.depth(leg, tail))
			through = through || tail.Transform.Position.X() < leg.Transform.Position.X()
		})
		t.Logf("%s: %.2f mm deep at worst, tail at %.2f m, leg at %.2f m", c.name, worst*1000, tail.Transform.Position.X(), leg.Transform.Position.X())
		if through {
			t.Errorf("%s: the leg went through the tail", c.name)
		}
		if worst > landingDepth {
			t.Errorf("%s: the leg entered the tail by %.2f mm", c.name, worst*1000)
		}
	}
}

// 8. SetBodyType changes a body between kinematic and dynamic in place: a kinematic plate carrying a cube, with a ball
// hanging from it by a distance joint, becomes dynamic and falls with them (the contact and the joint are kept, the
// cube doesn't jump), then becomes kinematic again and is lifted to its targets, the cube and the ball following
func TestSetBodyTypeKeepsTheContactsAndTheJoints(t *testing.T) {
	w := newScene(1)
	plate := kinematicBox(w, mgl64.Vec3{0, 3, 0}, mgl64.Vec3{0.5, 0.1, 0.5})
	cube := addBody(w, mgl64.Vec3{0.2, 3.1 + cubeHalf, 0.1}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	ball := addBody(w, mgl64.Vec3{0, 1.9, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0.6, 0)
	const rope = 1.0
	w.AddJoint(NewDistanceJoint(plate, ball, mgl64.Vec3{0, 2.9, 0}, mgl64.Vec3{0, 1.9, 0}))
	simulate(w, 1, nil)
	if !plate.IsSleeping || !cube.IsSleeping || !ball.IsSleeping {
		t.Fatalf("the plate, the cube and the ball should rest asleep together: %v %v %v", plate.IsSleeping, cube.IsSleeping, ball.IsSleeping)
	}
	contacts := func() int {
		n := 0
		for _, contact := range w.Contacts() {
			if (contact.BodyA == plate && contact.BodyB == cube) || (contact.BodyA == cube && contact.BodyB == plate) {
				n++
			}
		}
		return n
	}
	ropeLength := func() float64 {
		return ball.Transform.Position.Sub(plate.Transform.Position.Sub(mgl64.Vec3{0, 0.1, 0})).Len()
	}
	rides := func(what string) {
		t.Helper()
		if gap := cube.Transform.Position.Y() - cubeHalf - (plate.Transform.Position.Y() + 0.1); gap > SpeculativeDistance || gap < -landingDepth {
			t.Fatalf("%s: the cube is %.2f mm off the plate", what, gap*1000)
		}
		if stretch := math.Abs(ropeLength() - rope); stretch > landingDepth {
			t.Fatalf("%s: the rope is %.2f mm off its length", what, stretch*1000)
		}
	}

	// ========== kinematic -> dynamic: it falls, with the cube and the ball ==========
	if err := w.SetBodyType(plate, actor.BodyTypeDynamic); err != nil {
		t.Fatalf("the plate made dynamic: %v", err)
	}
	if plate.IsSleeping {
		t.Fatalf("the plate should wake up when it becomes dynamic")
	}
	top := plate.Transform.Position.Y()
	w.Step(sceneDt)
	if contacts() == 0 {
		t.Errorf("the contact between the plate and the cube was lost when the plate became dynamic")
	}
	if fall := top - plate.Transform.Position.Y(); fall > sceneGravity*sceneDt*sceneDt || fall <= 0 {
		t.Errorf("the plate moved by %.4f m in the step it became dynamic, want a fall under g dt²", fall)
	}
	rides("after the switch to dynamic")
	simulate(w, 0.5, func() { rides("falling") })
	// a free fall of 31 steps of 8 sub-steps, integrated as the solver does (v then x): g h² n (n + 1) / 2
	n, h := 31.0*sceneSubsteps, sceneDt/sceneSubsteps
	if fall, want := top-plate.Transform.Position.Y(), sceneGravity*h*h*n*(n+1)/2; math.Abs(fall-want) > landingDepth {
		t.Errorf("the plate fell by %.4f m, want the free fall of %.4f m: something held it", fall, want)
	}

	// ========== dynamic -> kinematic: it stops, then is lifted to its targets ==========
	if err := w.SetBodyType(plate, actor.BodyTypeKinematic); err != nil {
		t.Fatalf("the plate made kinematic again: %v", err)
	}
	if plate.Velocity != (mgl64.Vec3{}) || plate.AngularVelocity != (mgl64.Vec3{}) {
		t.Errorf("a body made kinematic keeps a velocity %v %v", plate.Velocity, plate.AngularVelocity)
	}
	held := plate.Transform
	w.Step(sceneDt)
	if plate.Transform != held {
		t.Errorf("a body made kinematic moved without a target")
	}
	simulate(w, 0.5, nil) // the cube and the ball settle on the stopped plate
	drive(w, plate, mgl64.Vec3{0, kinematicSpeed, 0}, 1, func() {
		rides("lifted")
	})
	if contacts() == 0 {
		t.Errorf("no contact between the plate and the cube it lifts")
	}
	if plate.Transform.Position.Y() < top-0.5 {
		t.Errorf("the plate is at y = %.3f after being lifted by 1 m", plate.Transform.Position.Y())
	}
}

// SetBodyType only changes a body between kinematic and dynamic: a static body is another kind of body, and a
// kinematic body created without a density has no mass to become dynamic. Both are refused with an error (never in
// silence) and left as they are; a body already of the type is nothing to do. The refusal allocates nothing
func TestSetBodyTypeRefusesTheStaticBodies(t *testing.T) {
	expectError := func(what string, want error, fn func() error) {
		t.Helper()
		if err := fn(); !errors.Is(err, want) {
			t.Errorf("%s: error %v, want %v", what, err, want)
		}
	}
	w := newScene(1)
	static := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
	dynamic := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	massless := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeKinematic, 0)
	w.AddBody(massless)
	expectError("a static body made kinematic", ErrStaticBody, func() error { return w.SetBodyType(static, actor.BodyTypeKinematic) })
	expectError("a static body made dynamic", ErrStaticBody, func() error { return w.SetBodyType(static, actor.BodyTypeDynamic) })
	expectError("a dynamic body made static", ErrStaticBody, func() error { return w.SetBodyType(dynamic, actor.BodyTypeStatic) })
	expectError("a kinematic body without mass made dynamic", ErrMasslessBody, func() error { return w.SetBodyType(massless, actor.BodyTypeDynamic) })
	if static.BodyType != actor.BodyTypeStatic || dynamic.BodyType != actor.BodyTypeDynamic || massless.BodyType != actor.BodyTypeKinematic {
		t.Errorf("a refused body changed its type: %v %v %v", static.BodyType, dynamic.BodyType, massless.BodyType)
	}
	if err := w.SetBodyType(dynamic, actor.BodyTypeDynamic); err != nil {
		t.Errorf("a body already of the type: %v, want nothing to do", err)
	}
	if allocs := testing.AllocsPerRun(10, func() { _ = w.SetBodyType(static, actor.BodyTypeKinematic) }); allocs > 0 {
		t.Errorf("a refusal allocates %.1f, want 0", allocs)
	}
}

// ========== SLEEP ==========

// A kinematic body which stops falls asleep with the bodies resting on it, in the same island; a target wakes them all
// up, and a target which moves it by a millimeter per second (under the sleep speed) keeps them awake: a kinematic body
// goes where it is told, there is no jitter to filter
func TestKinematicSleepsWithItsIsland(t *testing.T) {
	w := newScene(1)
	plate := kinematicBox(w, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{1, 0.1, 1})
	rider := addBody(w, mgl64.Vec3{0, 1.1 + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	sleeps, wakes := 0, 0
	w.Events.Subscribe(EventSleep, func(Event) { sleeps++ })
	w.Events.Subscribe(EventWake, func(Event) { wakes++ })
	drive(w, plate, mgl64.Vec3{kinematicSpeed, 0, 0}, 1, func() {
		if plate.IsSleeping || rider.IsSleeping {
			t.Fatalf("asleep while the plate moves: plate %v, rider %v", plate.IsSleeping, rider.IsSleeping)
		}
	})
	// no target: the plate stops at once, its rider slides on it (µ = 0.6: 1 m/s in 0.17 s) and rests, both sleep
	// DefaultTimeToSleep later
	simulate(w, kinematicSpeed/(0.6*sceneGravity)+actor.DefaultTimeToSleep+3*sceneDt, nil)
	if !plate.IsSleeping || !rider.IsSleeping {
		t.Fatalf("the plate stopped: plate asleep %v, rider asleep %v, want both asleep", plate.IsSleeping, rider.IsSleeping)
	}
	if sleeps != 2 {
		t.Errorf("%d sleep events, want 2 (the plate and its rider)", sleeps)
	}

	// a slow target wakes the island, and keeps it awake
	start := plate.Transform.Position.X()
	drive(w, plate, mgl64.Vec3{0.001, 0, 0}, 1, func() {
		if plate.IsSleeping || rider.IsSleeping {
			t.Fatalf("asleep while the plate crawls: plate %v, rider %v", plate.IsSleeping, rider.IsSleeping)
		}
	})
	if wakes != 2 {
		t.Errorf("%d wake events, want 2", wakes)
	}
	if moved, carried := plate.Transform.Position.X()-start, rider.Transform.Position.X()-start; math.Abs(carried-moved) > landingDepth {
		t.Errorf("the plate crawled by %.2f mm, its rider by %.2f mm: not carried", moved*1000, carried*1000)
	}
}

// A kinematic body arriving on a sleeping body wakes it up, and the sleeping bodies near a teleported body wake up
func TestKinematicWakesWhatItTouches(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	box := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	pusher := kinematicBox(w, mgl64.Vec3{-1, cubeHalf, 0}, cube().HalfExtents)
	simulate(w, 1, nil)
	if !box.IsSleeping {
		t.Fatalf("the box should be asleep")
	}
	drive(w, pusher, mgl64.Vec3{kinematicSpeed, 0, 0}, 1, nil)
	if box.IsSleeping || box.Transform.Position.X() < 0.3 {
		t.Errorf("the box was not woken up and pushed: asleep %v, x = %.3f", box.IsSleeping, box.Transform.Position.X())
	}

	other := addBody(w, mgl64.Vec3{5, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 1, nil)
	if !other.IsSleeping {
		t.Fatalf("the other box should be asleep")
	}
	w.Teleport(pusher, actor.Transform{Position: mgl64.Vec3{5 - 2*cubeHalf - SpeculativeDistance/2, cubeHalf, 0}, Rotation: mgl64.QuatIdent()})
	if other.IsSleeping {
		t.Errorf("the box next to the teleported body was not woken up")
	}
}

// ========== JOINTS & FORCES ==========

// A ball hanging from a moving kinematic body by a distance joint follows it: the joint pulls the ball with the
// velocity of the kinematic body
func TestKinematicJointPullsADynamicBody(t *testing.T) {
	w := newScene(1)
	hook := kinematicBox(w, mgl64.Vec3{0, 3, 0}, mgl64.Vec3{0.1, 0.1, 0.1})
	ball := addBody(w, mgl64.Vec3{0, 2, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0.6, 0)
	w.AddJoint(NewDistanceJoint(hook, ball, mgl64.Vec3{0, 2.9, 0}, mgl64.Vec3{0, 2, 0}))
	simulate(w, 1, nil)
	drive(w, hook, mgl64.Vec3{0, kinematicSpeed, 0}, 1, nil)
	if stretch := math.Abs(ball.Transform.Position.Sub(hook.Transform.Position).Len() - 1); stretch > landingDepth {
		t.Errorf("the rope is %.2f mm off its length while lifted", stretch*1000)
	}
	if math.Abs(ball.Velocity.Y()-kinematicSpeed) > 0.1*kinematicSpeed {
		t.Errorf("the ball rises at %.3f m/s, want about %.0f m/s", ball.Velocity.Y(), kinematicSpeed)
	}
}

// Forces, torques and impulses don't move a kinematic body, and neither does the gravity
func TestKinematicIgnoresTheForces(t *testing.T) {
	w := newScene(1)
	body := kinematicBox(w, mgl64.Vec3{0, 1, 0}, cube().HalfExtents)
	start := body.Transform
	for step := 0; step < 60; step++ {
		body.AddForce(mgl64.Vec3{100, 100, 100})
		body.AddTorque(mgl64.Vec3{10, 10, 10})
		body.AddImpulse(mgl64.Vec3{1, 1, 1})
		body.AddAngularImpulse(mgl64.Vec3{1, 1, 1})
		w.Step(sceneDt)
	}
	if body.Transform != start || body.Velocity != (mgl64.Vec3{}) || body.AngularVelocity != (mgl64.Vec3{}) {
		t.Errorf("the kinematic body moved: %v, velocity %v %v", body.Transform, body.Velocity, body.AngularVelocity)
	}
}

// ========== DETERMINISM & ALLOCATIONS ==========

// kinematicPile: a pile of boxes and spheres on the ground, 2 kinematic plates sweeping through it and a kinematic
// plate carrying part of it, on parallel paths whatever the count of bodies
func kinematicPile(count, workers int) (*World, []*actor.RigidBody) {
	w := benchScene(count, workers)
	w.parallelFrom = 1
	movers := []*actor.RigidBody{
		kinematicBox(w, mgl64.Vec3{-3, 0.3, 0}, mgl64.Vec3{0.2, 0.3, 2}),
		kinematicBox(w, mgl64.Vec3{0, 0.3, -3}, mgl64.Vec3{2, 0.3, 0.2}),
		kinematicBox(w, mgl64.Vec3{0, 0.05, 0}, mgl64.Vec3{1, 0.05, 1}),
	}
	return w, movers
}

// driveMovers runs the pile for seconds: the plates sweep along X and Z, the carrier rises and turns
func driveMovers(w *World, movers []*actor.RigidBody, seconds float64) {
	for i := 0; i < int(math.Round(seconds/sceneDt)); i++ {
		moveTo(movers[0], translated(movers[0].Transform, mgl64.Vec3{kinematicSpeed, 0, 0}))
		moveTo(movers[1], translated(movers[1].Transform, mgl64.Vec3{0, 0, kinematicSpeed}))
		carrier := translated(movers[2].Transform, mgl64.Vec3{0, 0.2, 0})
		carrier.Rotation = mgl64.QuatRotate(0.5*sceneDt, mgl64.Vec3{0, 1, 0}).Mul(carrier.Rotation).Normalize()
		moveTo(movers[2], carrier)
		w.Step(sceneDt)
	}
}

// The bits of a pile swept by kinematic bodies are the same with 1, 4 and 8 workers
func TestKinematicDeterminism(t *testing.T) {
	run := func(workers int) []uint64 {
		w, movers := kinematicPile(300, workers)
		driveMovers(w, movers, 2)
		var out []uint64
		for _, b := range w.Bodies {
			for _, v := range [3]mgl64.Vec3{b.Transform.Position, b.Transform.Rotation.V, b.Velocity} {
				for _, x := range v {
					out = append(out, math.Float64bits(x))
				}
			}
		}
		return out
	}
	reference := run(1)
	for _, workers := range []int{4, 8} {
		got := run(workers)
		for i := range reference {
			if got[i] != reference[i] {
				t.Fatalf("workers=%d: value %d differs: %x, want %x", workers, i, got[i], reference[i])
			}
		}
	}
}

// carriedPile: the pile of the bench scene dropped on a kinematic plate, which carries it round (a circle of 1 m at
// kinematicSpeed), on parallel paths whatever the count of bodies. Once the pile has landed, its pairs don't change:
// the step allocates nothing (as a pile on the ground; a body sweeping through a pile makes new pairs at every step,
// and the buffers of the contacts grow with them)
func carriedPile(count, workers int) (*World, func()) {
	w := newScene(workers)
	w.parallelFrom = 1
	addGround(w, 0.6)
	plate := kinematicBox(w, mgl64.Vec3{0, 0.1, 0}, mgl64.Vec3{4, 0.1, 4})
	pile := benchScene(count, 1)
	for _, body := range pile.Bodies[1:] {
		addBody(w, body.Transform.Position.Add(mgl64.Vec3{0, 0.2, 0}), mgl64.QuatIdent(), body.Shape, actor.BodyTypeDynamic, 0.6, 0)
	}
	angle := 0.0
	return w, func() {
		angle += kinematicSpeed * sceneDt // the plate goes round a circle of radius 1 m
		target := plate.Transform
		target.Position = mgl64.Vec3{math.Cos(angle) - 1, 0.1, math.Sin(angle)}
		moveTo(plate, target)
		w.Step(sceneDt)
	}
}

// A step with kinematic bodies allocates nothing, once the buffers have grown
func TestKinematicStepDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 8} {
		w, step := carriedPile(300, workers)
		for i := 0; i < 120; i++ {
			step()
		}
		carried := 0
		for _, contact := range w.Contacts() {
			if contact.BodyA.BodyType == actor.BodyTypeKinematic || contact.BodyB.BodyType == actor.BodyTypeKinematic {
				carried++
			}
		}
		if carried < 50 {
			t.Fatalf("%d contacts with the plate: the scene doesn't cover the solver", carried)
		}
		allocs := testing.AllocsPerRun(10, step)
		if allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
	}
}

// BenchmarkCarriedPile: the 300 bodies of the bench scene resting on a plate of 8 x 8 m, the plate kinematic and going
// round at 1 m/s (carriedPile) against the same plate static: the same contacts, the kinematic plate keeps them awake
// and rubbing. 2 s once the pile has landed
func BenchmarkCarriedPile(b *testing.B) {
	for _, kinematic := range []bool{false, true} {
		for _, workers := range []int{1, 8} {
			name := "static_plate"
			if kinematic {
				name = "kinematic_plate"
			}
			b.Run(fmt.Sprintf("%s_%d_workers", name, workers), func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					b.StopTimer()
					w, step := carriedPile(300, workers)
					w.parallelFrom = 0 // the paths of a game: parallel from minParallelBodies
					if !kinematic {
						for _, body := range w.Bodies {
							if body.BodyType == actor.BodyTypeKinematic {
								body.BodyType = actor.BodyTypeStatic
							}
						}
						step = func() { w.Step(sceneDt) }
					}
					for k := 0; k < 120; k++ {
						step()
					}
					b.StartTimer()
					for k := 0; k < 120; k++ {
						step()
					}
				}
			})
		}
	}
}
