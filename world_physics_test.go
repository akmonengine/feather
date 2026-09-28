package feather

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Physical scenarios with a known answer, at the rate AkmonEngine runs Feather:
// 50 Hz, 12 sub-steps.
const (
	sceneDt       = 1.0 / 50
	sceneSubsteps = 12
	sceneGravity  = 9.81
	cubeHalf      = 0.25
)

func newScene(workers int) *World {
	return &World{
		Gravity:     mgl64.Vec3{0, -sceneGravity, 0},
		Substeps:    sceneSubsteps,
		SpatialGrid: NewSpatialGrid(2.0, 4096),
		Workers:     workers,
		Events:      NewEvents(),
	}
}

func addBody(w *World, position mgl64.Vec3, rotation mgl64.Quat, shape actor.ShapeInterface, bodyType actor.BodyType, friction, restitution float64) *actor.RigidBody {
	b := actor.NewRigidBody(actor.Transform{Position: position, Rotation: rotation}, shape, bodyType, 500)
	b.Material.StaticFriction, b.Material.DynamicFriction, b.Material.Restitution = friction, friction, restitution
	w.AddBody(b)
	return b
}

func addGround(w *World, friction float64) *actor.RigidBody {
	return addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, actor.BodyTypeStatic, friction, 0)
}

func cube() *actor.Box { return &actor.Box{HalfExtents: mgl64.Vec3{cubeHalf, cubeHalf, cubeHalf}} }

func simulate(w *World, seconds float64, each func()) {
	for i := 0; i < int(math.Round(seconds/sceneDt)); i++ {
		w.Step(sceneDt)
		if each != nil {
			each()
		}
	}
}

func finite(v mgl64.Vec3) bool {
	for _, x := range v {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			return false
		}
	}
	return true
}

// A stack of boxes, 1 mm apart, stands for 10 s: the top box only settles (the gaps and the
// soft contacts under load) and does not move sideways. v0.2.0 (XPBD) sank 22 mm at 10 boxes.
func TestStackStands(t *testing.T) {
	for _, n := range []int{3, 5, 10} {
		w := newScene(1)
		addGround(w, 0.6)
		var boxes []*actor.RigidBody
		for i := 0; i < n; i++ {
			boxes = append(boxes, addBody(w, mgl64.Vec3{0, cubeHalf + float64(i)*0.501, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0))
		}
		simulate(w, 10, nil)

		top := boxes[n-1].Transform.Position
		restingTop := cubeHalf + float64(n-1)*2*cubeHalf
		// Soft contacts hold ~0.5 mm of overlap per contact under the weight above them.
		if sink := restingTop - top.Y(); !(sink > -0.001 && sink < 0.001*float64(n)) {
			t.Errorf("stack of %d: top box at y=%.4f, %.1f mm under its resting height", n, top.Y(), sink*1000)
		}
		if side := math.Hypot(top.X(), top.Z()); !(side < 0.004) {
			t.Errorf("stack of %d: top box moved %.2f mm sideways", n, side*1000)
		}
	}
}

// A pyramid of 55 boxes stands. v0.2.0 exploded (top box thrown 139 m up).
func TestPyramidStands(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	type placed struct {
		body  *actor.RigidBody
		start mgl64.Vec3
	}
	var boxes []placed
	const base = 10
	for row := 0; row < base; row++ {
		for i := 0; i < base-row; i++ {
			p := mgl64.Vec3{(float64(i) - float64(base-1-row)/2) * 0.52, cubeHalf + float64(row)*0.501, 0}
			boxes = append(boxes, placed{addBody(w, p, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0), p})
		}
	}
	simulate(w, 5, nil)
	for _, b := range boxes {
		p := b.body.Transform.Position
		if !finite(p) || math.Abs(p.X()-b.start.X()) > 0.01 || math.Abs(p.Z()) > 0.01 || b.start.Y()-p.Y() > 0.03 {
			t.Fatalf("box starting at %v moved to %v", b.start, p)
		}
	}
}

// A box on a slope: Coulomb friction decides. It sticks when tan θ < µ and otherwise
// slides with a = g (sin θ - µ cos θ). v0.2.0 slid 9.9 m at 20°, µ=0.6.
func TestInclineFollowsCoulomb(t *testing.T) {
	cases := []struct{ degrees, friction float64 }{{20, 0.6}, {20, 0.2}, {35, 0.3}, {10, 0.05}}
	for _, c := range cases {
		w := newScene(1)
		theta := c.degrees * math.Pi / 180
		tilt := mgl64.QuatRotate(theta, mgl64.Vec3{0, 0, 1})
		normal := tilt.Rotate(mgl64.Vec3{0, 1, 0})
		addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Plane{Normal: normal}, actor.BodyTypeStatic, c.friction, 0)
		box := addBody(w, normal.Mul(cubeHalf), tilt, cube(), actor.BodyTypeDynamic, c.friction, 0)

		simulate(w, 0.5, nil)
		start := box.Transform.Position
		const duration = 2.0
		simulate(w, duration, nil)
		slid := box.Transform.Position.Sub(start).Len()

		a := sceneGravity * (math.Sin(theta) - c.friction*math.Cos(theta))
		want := 0.0
		if a > 0 {
			want = a*0.5*duration + 0.5*a*duration*duration
		}
		if math.Abs(slid-want) > 0.01+0.005*want {
			t.Errorf("%.0f° µ=%.2f: slid %.4f m, want %.4f m", c.degrees, c.friction, slid, want)
		}
	}
}

// A sphere dropped from 1 m bounces back to e² m (energy e² kept), and not at all when e=0.
func TestBounceRestitution(t *testing.T) {
	for _, e := range []float64{0, 0.5, 0.8} {
		w := newScene(1)
		ground := addGround(w, 0)
		ground.Material.Restitution = e
		ball := addBody(w, mgl64.Vec3{0, 1 + cubeHalf, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: cubeHalf}, actor.BodyTypeDynamic, 0, e)
		hit, apex := false, 0.0
		simulate(w, 2.5, func() {
			h := ball.Transform.Position.Y() - cubeHalf
			if h < 0.01 {
				hit = true
			}
			if hit {
				apex = math.Max(apex, h)
			}
		})
		// The contact takes a few sub-steps: allow 10% of the drop.
		if math.Abs(apex-e*e) > 0.1 {
			t.Errorf("e=%.1f: rebound %.3f m, want %.3f m", e, apex, e*e)
		}
		if e == 0 && apex > 0.001 {
			t.Errorf("e=0: rebound %.4f m, want none", apex)
		}
	}
}

// AddForce and AddTorque are in newtons and newton-metres, applied during the next step.
func TestForcesAreSI(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	ball := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, actor.BodyTypeDynamic, 0, 0)
	m := ball.Material.GetMass()
	inertia := ball.InertiaLocal.At(0, 0)
	const force, torque = 10.0, 3.0
	for i := 0; i < 50; i++ {
		ball.AddForce(mgl64.Vec3{force, 0, 0})
		ball.AddTorque(mgl64.Vec3{0, torque, 0})
		w.Step(sceneDt)
	}
	if want := force / m; math.Abs(ball.Velocity.X()-want) > 1e-9 {
		t.Errorf("velocity after 1 s of %g N on %.2f kg = %.6f m/s, want %.6f", force, m, ball.Velocity.X(), want)
	}
	if want := torque / inertia; math.Abs(ball.AngularVelocity.Y()-want) > 1e-9 {
		t.Errorf("angular velocity after 1 s of %g N.m = %.6f rad/s, want %.6f", torque, ball.AngularVelocity.Y(), want)
	}
	if ball.Force() != (mgl64.Vec3{}) || ball.Torque() != (mgl64.Vec3{}) {
		t.Error("forces are not cleared after the step")
	}
}

// A free box spinning near its intermediate axis tumbles (Dzhanibekov effect) and keeps its
// angular momentum: the gyroscopic term is integrated, not dropped.
func TestTumblingKeepsAngularMomentum(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	box := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.3, 0.6}}, actor.BodyTypeDynamic, 0, 0)
	box.AngularVelocity = mgl64.Vec3{0.05, 4, 0.05}
	momentum := func() mgl64.Vec3 { return box.GetInertiaWorld().Mul3x1(box.AngularVelocity) }
	energy := func() float64 { return 0.5 * box.AngularVelocity.Dot(momentum()) }
	l0, e0 := momentum(), energy()
	flipped := false
	simulate(w, 10, func() {
		axis := box.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0})
		if axis.Dot(l0.Normalize()) < 0 {
			flipped = true
		}
	})
	if drift := momentum().Sub(l0).Len() / l0.Len(); drift > 0.01 {
		t.Errorf("angular momentum drifted by %.2f%% over 10 s", drift*100)
	}
	if e := energy(); e > e0*1.001 || e < e0*0.9 {
		t.Errorf("rotational energy went from %.4f to %.4f J", e0, e)
	}
	if !flipped {
		t.Error("the box never flipped around its intermediate axis (no gyroscopic effect)")
	}
}

// Damping slows bodies down by 1/(1+h·c) per sub-step.
func TestDamping(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	ball := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, actor.BodyTypeDynamic, 0, 0)
	ball.Material.LinearDamping, ball.Material.AngularDamping = 0.5, 2
	ball.Velocity, ball.AngularVelocity = mgl64.Vec3{1, 0, 0}, mgl64.Vec3{0, 0, 1}
	simulate(w, 1, nil)
	h := sceneDt / sceneSubsteps
	steps := float64(50 * sceneSubsteps)
	if want := math.Pow(1/(1+h*0.5), steps); math.Abs(ball.Velocity.X()-want) > 1e-9 {
		t.Errorf("linear speed %.6f, want %.6f", ball.Velocity.X(), want)
	}
	if want := math.Pow(1/(1+h*2), steps); math.Abs(ball.AngularVelocity.Z()-want) > 1e-9 {
		t.Errorf("angular speed %.6f, want %.6f", ball.AngularVelocity.Z(), want)
	}
}

// A rotated static box is a ramp: a ball rolls down its surface. With v0.2.0 a static body
// kept a zero inverse rotation (only integration filled it), so its collisions were wrong.
func TestRotatedStaticBoxIsARamp(t *testing.T) {
	w := newScene(1)
	tilt := mgl64.QuatRotate(30*math.Pi/180, mgl64.Vec3{0, 0, 1})
	addBody(w, mgl64.Vec3{}, tilt, &actor.Box{HalfExtents: mgl64.Vec3{3, 0.25, 1}}, actor.BodyTypeStatic, 0.5, 0)
	ball := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.25}, actor.BodyTypeDynamic, 0.5, 0)
	normal := tilt.Rotate(mgl64.Vec3{0, 1, 0})

	landed := false
	simulate(w, 1.2, func() {
		gap := ball.Transform.Position.Dot(normal) - 0.5
		if gap < 0.01 {
			landed = true
		}
		if landed && math.Abs(ball.Transform.Position.X()) < 2.2 && (gap < -0.006 || gap > 0.005) {
			t.Fatalf("ball %.4f m from the ramp surface at x=%.2f", gap, ball.Transform.Position.X())
		}
	})
	if !landed || ball.Transform.Position.X() > -0.5 {
		t.Errorf("ball at %v: it did not roll down the ramp", ball.Transform.Position)
	}
}

// The same scene gives the same result bit for bit, run after run, whatever the number of
// workers. v0.2.0 differed on 38 of 40 boxes between two runs.
func TestDeterminism(t *testing.T) {
	run := func(workers int) []mgl64.Vec3 {
		w := newScene(workers)
		addGround(w, 0.6)
		r := rand.New(rand.NewSource(1))
		for i := 0; i < 40; i++ {
			q := mgl64.QuatRotate(r.Float64()*math.Pi, mgl64.Vec3{r.Float64(), r.Float64(), r.Float64()}.Normalize())
			var shape actor.ShapeInterface = cube()
			switch i % 3 {
			case 1:
				shape = &actor.Sphere{Radius: cubeHalf}
			case 2:
				shape = &actor.Capsule{HalfHeight: 0.2, Radius: 0.15}
			}
			addBody(w, mgl64.Vec3{r.Float64()*3 - 1.5, 0.5 + float64(i)*0.6, r.Float64()*3 - 1.5}, q, shape, actor.BodyTypeDynamic, 0.6, 0.2)
		}
		simulate(w, 4, nil)
		var out []mgl64.Vec3
		for _, b := range w.Bodies {
			out = append(out, b.Transform.Position, b.Transform.Rotation.V)
		}
		return out
	}
	reference := run(1)
	for _, workers := range []int{1, 3, 8} {
		got := run(workers)
		for i := range reference {
			if got[i] != reference[i] {
				t.Fatalf("workers=%d: value %d is %v, want %v", workers, i, got[i], reference[i])
			}
		}
	}
}

// A resting box falls asleep; a moving box that hits it wakes it up.
func TestSleepAndWake(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	sleeper := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 2, nil)
	if !sleeper.IsSleeping {
		t.Fatal("a resting box did not fall asleep within 2 s")
	}
	before := sleeper.Transform.Position

	striker := addBody(w, mgl64.Vec3{-1, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	striker.Velocity = mgl64.Vec3{3, 0, 0}
	woke := false
	simulate(w, 1, func() {
		if !sleeper.IsSleeping {
			woke = true
		}
	})
	if !woke {
		t.Fatal("the struck box never woke up")
	}
	if sleeper.Transform.Position.X()-before.X() < 0.05 {
		t.Errorf("the struck box did not move: %v", sleeper.Transform.Position)
	}
	if striker.Transform.Position.X() > sleeper.Transform.Position.X()-2*cubeHalf+0.01 {
		t.Errorf("the striker went through: striker x=%.3f, box x=%.3f", striker.Transform.Position.X(), sleeper.Transform.Position.X())
	}
}

// A plane given as body B still pushes the body out of it (the old code reversed the
// normal in that case).
func TestPlaneAsBodyB(t *testing.T) {
	ball := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0, 0.4, 0}, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 0.5}, actor.BodyTypeDynamic, 1)
	plane := createPlane(mgl64.Vec3{0, 1, 0}, 0)
	m := NarrowPhase([]Pair{{BodyA: ball, BodyB: plane}}, 1)
	if len(m) != 1 || m[0].BodyA != ball || m[0].Normal != (mgl64.Vec3{0, -1, 0}) {
		t.Fatalf("manifold %+v, want the normal from the ball down to the plane", m)
	}
	if math.Abs(m[0].MinSeparation()+0.1) > 1e-12 {
		t.Errorf("separation %.6f, want -0.1", m[0].MinSeparation())
	}
}

// A fast ball does not tunnel through a thin static box: the speculative margin grows with
// the speed.
func TestNoTunnelling(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.02, 2}}, actor.BodyTypeStatic, 0, 0)
	ball := addBody(w, mgl64.Vec3{0, 3, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0, 0)
	ball.Velocity = mgl64.Vec3{0, -40, 0} // 80 cm per step, 40 times the wall thickness
	simulate(w, 0.5, nil)
	if y := ball.Transform.Position.Y(); y < 0.1 {
		t.Errorf("the ball went through the wall: y=%.3f", y)
	}
}

// Collision events fire when bodies touch, not while they are only speculative contacts.
func TestCollisionEventsOnTouch(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	ball := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: cubeHalf}, actor.BodyTypeDynamic, 0.6, 0)
	enteredAt := -1.0
	elapsed := 0.0
	w.Events.Subscribe(COLLISION_ENTER, func(Event) {
		if enteredAt < 0 {
			enteredAt = elapsed
		}
	})
	simulate(w, 1, func() { elapsed += sceneDt })
	// Free fall from 0.75 m: contact after sqrt(2*0.75/g) = 0.391 s.
	if enteredAt < 0 || math.Abs(enteredAt-0.391) > 2*sceneDt {
		t.Errorf("CollisionEnter at %.3f s, want ~0.391 s", enteredAt)
	}
	if ball.Transform.Position.Y() < cubeHalf-0.01 {
		t.Errorf("ball sank to y=%.3f", ball.Transform.Position.Y())
	}
}

// A heavy box (100x the mass) on a light one: the light box is not crushed through the
// ground and nothing jitters away.
func TestMassRatio(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	light := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	heavy := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0, 3*cubeHalf + 0.001, 0}, Rotation: mgl64.QuatIdent()}, cube(), actor.BodyTypeDynamic, 50000)
	heavy.Material.StaticFriction, heavy.Material.DynamicFriction = 0.6, 0.6
	w.AddBody(heavy)
	simulate(w, 5, nil)
	if y := light.Transform.Position.Y(); math.Abs(y-cubeHalf) > 0.005 {
		t.Errorf("light box at y=%.4f, want %.4f", y, cubeHalf)
	}
	if p := heavy.Transform.Position; math.Abs(p.Y()-3*cubeHalf) > 0.01 || math.Hypot(p.X(), p.Z()) > 0.005 {
		t.Errorf("heavy box at %v", p)
	}
}

// ContactHertz sets the stiffness: a stiffer world overlaps less under the same load.
func TestContactHertz(t *testing.T) {
	sink := func(hertz float64) float64 {
		w := newScene(1)
		w.ContactHertz = hertz
		addGround(w, 0.6)
		var top *actor.RigidBody
		for i := 0; i < 6; i++ {
			top = addBody(w, mgl64.Vec3{0, cubeHalf + float64(i)*2*cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		}
		simulate(w, 3, nil)
		return cubeHalf + 10*cubeHalf - top.Transform.Position.Y()
	}
	soft, stiff := sink(20), sink(0)
	if !(stiff < soft/2) {
		t.Errorf("sink at 20 Hz %.2f mm, at the default %.2f mm: want the default at least twice as stiff", soft*1000, stiff*1000)
	}
}
