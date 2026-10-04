package feather

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// The time of impact of a sphere moving towards a box: the exact fraction where the gap is toiTarget
func TestTimeOfImpact(t *testing.T) {
	box := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{3, 0, 0}, Rotation: mgl64.QuatIdent()}, &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 1}}, actor.BodyTypeStatic, 0)
	sphere := &actor.Sphere{Radius: 0.25}
	motion := sweep{
		start: actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()},
		end:   actor.Transform{Position: mgl64.Vec3{4, 0, 0}, Rotation: mgl64.QuatIdent()},
	}
	proxy := gjk.NewProxy(box)
	fraction := timeOfImpact(sphere, &motion, 0.25, &proxy, nil, 1)
	// the sphere touches the face x = 2.5 when its center is at 2.25, stopped toiTarget before
	want := (2.25 - toiTarget) / 4
	if math.Abs(fraction-want)*4 > toiTolerance {
		t.Errorf("fraction %.6f, want %.6f", fraction, want)
	}

	// against a plane, with a rotation: a box falling on a corner
	plane := &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}
	falling := sweep{
		start: actor.Transform{Position: mgl64.Vec3{0, 2, 0}, Rotation: mgl64.QuatIdent()},
		end:   actor.Transform{Position: mgl64.Vec3{0, -1, 0}, Rotation: mgl64.QuatRotate(0.5, mgl64.Vec3{0, 0, 1})},
	}
	falling.angle = 0.5
	cube := &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}
	fraction = timeOfImpact(cube, &falling, cube.HalfExtents.Len(), nil, plane, 1)
	at := falling.at(fraction)
	lowest := at.ToWorld(cube.Support(at.Rotation.Conjugate().Rotate(mgl64.Vec3{0, -1, 0}))).Y()
	if fraction <= 0 || fraction >= 1 || lowest < toiTarget-1e-9 || lowest > toiTarget+toiTolerance {
		t.Errorf("fraction %.4f, the lowest corner at %.5f m", fraction, lowest)
	}
}

// moveThrough gives the body the motion of a step, as if the solver had accelerated it: the body is at the end of the
// motion, its state in the solver remembers the start. Then the continuous collision runs on the world
func moveThrough(w *World, body *actor.RigidBody, motion sweep) {
	s := &w.solver
	i := stateOf(w, body)
	s.states[i].deltaPosition = motion.end.Position.Sub(motion.start.Position)
	s.states[i].deltaRotation = motion.end.Rotation.Mul(motion.start.Rotation.Conjugate()).Normalize()
	s.starts[i].rotation = motion.start.Rotation
	body.Transform = motion.end
	body.UpdateAABB()
	w.continuous(s, w.Contacts(), sceneDt)
}

// stateOf: the index of the state of the body in the solver, after a step
func stateOf(w *World, body *actor.RigidBody) int {
	for i, b := range w.Bodies {
		if b == body {
			if state := w.solver.stateIndex[i]; state >= 0 {
				return int(state)
			}
			panic("the body has no state: step the world first")
		}
	}
	panic("the body is not in the world")
}

// A fast body is stopped by every body it collides with: a static body, a kinematic body, a dynamic body awake or asleep,
// a dynamic body with all its axes locked. Nothing marks the body: every fast body is the bullet of Box2D
func TestFastBodyStopsOnEveryBody(t *testing.T) {
	plate := &actor.Box{HalfExtents: mgl64.Vec3{0.01, 1, 1}}
	cases := []struct {
		name string
		wall func(w *World) *actor.RigidBody
	}{
		{"static", func(w *World) *actor.RigidBody {
			return addBody(w, mgl64.Vec3{5, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeStatic, 0.5, 0)
		}},
		{"kinematic", func(w *World) *actor.RigidBody {
			return addBody(w, mgl64.Vec3{5, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeKinematic, 0.5, 0)
		}},
		{"dynamic", func(w *World) *actor.RigidBody {
			return addBody(w, mgl64.Vec3{5, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeDynamic, 0.5, 0)
		}},
		{"sleeping", func(w *World) *actor.RigidBody {
			wall := addBody(w, mgl64.Vec3{5, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeDynamic, 0.5, 0)
			wall.IsSleeping = true
			return wall
		}},
		{"locked", func(w *World) *actor.RigidBody {
			wall := addBody(w, mgl64.Vec3{5, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeDynamic, 0.5, 0)
			wall.LinearLock, wall.AngularLock = actor.AllAxes, actor.AllAxes
			return wall
		}},
	}
	for _, c := range cases {
		w := newScene(1)
		w.Gravity = mgl64.Vec3{}
		wall := c.wall(w)
		ball := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.02}, actor.BodyTypeDynamic, 0.5, 0)
		w.Step(sceneDt)
		moveThrough(w, ball, sweep{start: ball.Transform, end: actor.Transform{Position: mgl64.Vec3{10, 0, 0}, Rotation: mgl64.QuatIdent()}})
		x := ball.Transform.Position.X()
		if x > 5 || x < 4.9 {
			t.Errorf("%s: the ball is at x=%.3f, want it stopped before the wall at 4.98", c.name, x)
		}
		if wall.Transform.Position.X() != 5 {
			t.Errorf("%s: the wall moved to x=%.3f", c.name, wall.Transform.Position.X())
		}
	}
}

// 2 fast bodies which meet are stopped together, where they meet: a sweep of their relative motion against the start
// pose of the other, as the LinearCast of Jolt. Each stops at the same fraction of its own motion
func TestTwoFastBodiesAreStoppedTogether(t *testing.T) {
	const r = 0.05
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	a := addBody(w, mgl64.Vec3{-0.1, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: r}, actor.BodyTypeDynamic, 0.5, 0)
	b := addBody(w, mgl64.Vec3{0.1, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: r}, actor.BodyTypeDynamic, 0.5, 0)
	w.Step(sceneDt)
	// the motions cross: each sphere ends beyond the other
	s := &w.solver
	for _, body := range []*actor.RigidBody{a, b} {
		i := stateOf(w, body)
		start := body.Transform
		end := actor.Transform{Position: start.Position.Mul(-1).Add(start.Position.Mul(-1).Normalize().Mul(0.1333)), Rotation: mgl64.QuatIdent()}
		s.states[i].deltaPosition = end.Position.Sub(start.Position)
		s.states[i].deltaRotation = mgl64.QuatIdent()
		s.starts[i].rotation = start.Rotation
		body.Transform = end
		body.UpdateAABB()
	}
	w.continuous(s, w.Contacts(), sceneDt)
	gap := b.Transform.Position.X() - a.Transform.Position.X() - 2*r
	if gap < toiTarget-1e-9 || gap > toiTarget+toiTolerance {
		t.Errorf("the spheres are %.5f m apart after the continuous collision, want %.5f (a at %.4f, b at %.4f)", gap, toiTarget, a.Transform.Position.X(), b.Transform.Position.X())
	}
	if a.Transform.Position.X() != -b.Transform.Position.X() {
		t.Errorf("the spheres stopped at %.6f and %.6f, want the same fraction of symmetric motions", a.Transform.Position.X(), b.Transform.Position.X())
	}
}

// Criterion: 2 spheres of 10 cm thrown face to face at 20 m/s each, at 60 Hz with 8 sub-steps, touch and bounce, and
// never go through each other (today they go through from 7.5 m/s of relative speed)
func TestFastSpheresFaceToFaceBounce(t *testing.T) {
	const r, speed = 0.05, 20.0
	for _, workers := range []int{1, 8} {
		w := newScene(workers)
		w.parallelFrom = 2
		w.Gravity = mgl64.Vec3{}
		a := addBody(w, mgl64.Vec3{-2, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: r}, actor.BodyTypeDynamic, 0.5, 0.5)
		b := addBody(w, mgl64.Vec3{2, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: r}, actor.BodyTypeDynamic, 0.5, 0.5)
		a.Velocity, b.Velocity = mgl64.Vec3{speed, 0, 0}, mgl64.Vec3{-speed, 0, 0}
		minGap := math.Inf(1)
		simulate(w, 1, func() {
			gap := b.Transform.Position.X() - a.Transform.Position.X() - 2*r
			minGap = math.Min(minGap, gap)
			if a.Transform.Position.X() > b.Transform.Position.X() {
				t.Fatalf("workers=%d: the spheres went through each other: a at %.3f, b at %.3f", workers, a.Transform.Position.X(), b.Transform.Position.X())
			}
		})
		t.Logf("workers=%d: closest %.2f mm, a at %.2f m/s, b at %.2f m/s", workers, minGap*1000, a.Velocity.X(), b.Velocity.X())
		if minGap > SpeculativeDistance {
			t.Errorf("workers=%d: the spheres never came closer than %.1f mm: they didn't touch", workers, minGap*1000)
		}
		if minGap < -landingDepth {
			t.Errorf("workers=%d: the spheres overlapped by %.2f mm", workers, -minGap*1000)
		}
		// the bounce: each sphere comes back at half its speed (restitution 0.5), within 1 %
		if math.Abs(a.Velocity.X()+speed/2) > speed/200 || math.Abs(b.Velocity.X()-speed/2) > speed/200 {
			t.Errorf("workers=%d: after the bounce a goes at %.3f m/s and b at %.3f, want ∓%.1f", workers, a.Velocity.X(), b.Velocity.X(), speed/2)
		}
	}
}

// Criterion: a plate of 1 cm crossing at 30 m/s a plate of 1 cm resting (dynamic) is stopped or pushes it, never on
// the other side (today it goes through from 2.4 m/s)
func TestFastPlateNeverCrossesARestingPlate(t *testing.T) {
	plate := &actor.Box{HalfExtents: mgl64.Vec3{0.005, 0.5, 0.5}}
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	moving := addBody(w, mgl64.Vec3{-2, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeDynamic, 0.5, 0)
	resting := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeDynamic, 0.5, 0)
	moving.Velocity = mgl64.Vec3{30, 0, 0}
	worst := 0.0
	simulate(w, 1, func() {
		worst = math.Max(worst, boxOverlap(moving, resting))
		if moving.Transform.Position.X() > resting.Transform.Position.X() {
			t.Fatalf("the moving plate is on the other side: at %.3f, the resting plate at %.3f", moving.Transform.Position.X(), resting.Transform.Position.X())
		}
	})
	t.Logf("worst overlap %.2f mm, the plates at %.3f and %.3f m, going at %.2f and %.2f m/s", worst*1000, moving.Transform.Position.X(), resting.Transform.Position.X(), moving.Velocity.X(), resting.Velocity.X())
	if worst > landingDepth {
		t.Errorf("the plates overlapped by %.2f mm", worst*1000)
	}
	if resting.Velocity.X() <= 0 {
		t.Errorf("the resting plate was not pushed: %.3f m/s", resting.Velocity.X())
	}
}

// A body accelerated by the solver during the step is caught by the continuous collision: a heavy ball at 40 m/s hits
// a light plate at rest, which flies at the speed of the ball within the step (its pair with the plate behind it had
// the margin of 2 resting bodies); the plate is stopped on the plate behind it, never beyond
func TestKickedBodyIsStoppedByTheContinuousCollision(t *testing.T) {
	plate := &actor.Box{HalfExtents: mgl64.Vec3{0.005, 0.5, 0.5}}
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	ball := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{-3, 0, 0}, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 0.2}, actor.BodyTypeDynamic, 8000)
	ball.Material.Restitution = 0
	w.AddBody(ball)
	first := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()}, plate, actor.BodyTypeDynamic, 50)
	second := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0.3, 0, 0}, Rotation: mgl64.QuatIdent()}, plate, actor.BodyTypeDynamic, 50)
	w.AddBody(first)
	w.AddBody(second)
	ball.Velocity = mgl64.Vec3{40, 0, 0}
	kicked, worst := 0.0, 0.0
	simulate(w, 0.5, func() {
		kicked = math.Max(kicked, first.Velocity.X())
		worst = math.Max(worst, boxOverlap(first, second))
		if first.Transform.Position.X() > second.Transform.Position.X() {
			t.Fatalf("the kicked plate went through the plate behind it: at %.3f, the other at %.3f", first.Transform.Position.X(), second.Transform.Position.X())
		}
	})
	t.Logf("the first plate was kicked at %.1f m/s, worst overlap with the second %.2f mm", kicked, worst*1000)
	if kicked < 20 {
		t.Errorf("the first plate was kicked at %.1f m/s only: the scene doesn't accelerate it enough during a step", kicked)
	}
	if worst > landingDepth {
		t.Errorf("the plates overlapped by %.2f mm", worst*1000)
	}
}

// A dynamic body the fast body has a contact with is left to the solver: the sweep goes on to the next body. The
// contact holds them (the speculative row stops the fast body where they touch); swept too, the fast body would be
// lifted off the overlap the soft contact allows, each step, and its joints torn (a swinging chain of 20 links stretched
// by 214 mm instead of 9)
func TestContinuousLeavesTheContactsToTheSolver(t *testing.T) {
	plate := &actor.Box{HalfExtents: mgl64.Vec3{0.01, 1, 1}}
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	ball := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.05}, actor.BodyTypeDynamic, 0.5, 0)
	near := addBody(w, mgl64.Vec3{0.3, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeDynamic, 0.5, 0)
	far := addBody(w, mgl64.Vec3{3, 0, 0}, mgl64.QuatIdent(), plate, actor.BodyTypeDynamic, 0.5, 0)
	ball.Velocity = mgl64.Vec3{20, 0, 0}
	w.Step(sceneDt)
	// the fast ball has a contact with the near plate (25 cm away, within the margin of its speed), none with the far one
	contacts := map[*actor.RigidBody]bool{}
	for _, m := range w.Contacts() {
		contacts[m.BodyA], contacts[m.BodyB] = true, true
	}
	if !contacts[near] || contacts[far] {
		t.Fatalf("contacts with the near plate %v and the far plate %v, want true and false", contacts[near], contacts[far])
	}
	// a motion through both plates, as if the solver had accelerated the ball: stopped by the far plate only
	moveThrough(w, ball, sweep{start: actor.Transform{Position: mgl64.Vec3{0.1, 0, 0}, Rotation: mgl64.QuatIdent()}, end: actor.Transform{Position: mgl64.Vec3{5, 0, 0}, Rotation: mgl64.QuatIdent()}})
	x := ball.Transform.Position.X()
	if x < 2.9 || x > 3 {
		t.Errorf("the ball is at x=%.3f, want it stopped before the far plate at 2.94 (the near plate is left to its contact)", x)
	}
}

// The pairs of a fast body get the speculative margin of its speed: the contact with a resting body exists a step ahead,
// where 2 slow bodies at the same distance have none
func TestFastPairsGetTheSpeculativeMargin(t *testing.T) {
	const r = 0.05
	for _, speed := range []float64{1, 20} {
		w := newScene(1)
		w.Gravity = mgl64.Vec3{}
		a := addBody(w, mgl64.Vec3{-0.2, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: r}, actor.BodyTypeDynamic, 0.5, 0)
		addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: r}, actor.BodyTypeDynamic, 0.5, 0)
		a.Velocity = mgl64.Vec3{speed, 0, 0}
		w.Step(sceneDt)
		// the gap of 10 cm is beyond SpeculativeDistance: a body at 1 m/s (1.7 cm per step) has no contact, a body at
		// 20 m/s (33 cm per step) has one
		want := isFast(a, sceneDt)
		if got := len(w.Contacts()) > 0; got != want {
			t.Errorf("at %.0f m/s: contact %v, want %v (fast %v)", speed, got, want, want)
		}
	}
}

// isFast: a body which can move more than half of its smallest extent during the step, by its translation or its
// rotation; never a kinematic or a sleeping body
func TestIsFast(t *testing.T) {
	cube := func() *actor.RigidBody {
		return actor.NewRigidBody(actor.NewTransform(), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.5}}, actor.BodyTypeDynamic, 500)
	}
	slow := cube()
	slow.Velocity = mgl64.Vec3{7, 0, 0} // 11.7 cm per step, the half of the smallest extent is 12.5 cm
	fast := cube()
	fast.Velocity = mgl64.Vec3{8, 0, 0} // 13.3 cm
	spinning := cube()
	spinning.AngularVelocity = mgl64.Vec3{0, 0, 15} // 0.25 rad per step, the farthest corner at 0.612 m moves 15.3 cm
	kinematic := cube()
	kinematic.BodyType, kinematic.Velocity = actor.BodyTypeKinematic, mgl64.Vec3{8, 0, 0}
	sleeping := cube()
	sleeping.IsSleeping, sleeping.Velocity = true, mgl64.Vec3{8, 0, 0}
	for _, c := range []struct {
		name string
		body *actor.RigidBody
		want bool
	}{{"slow", slow, false}, {"fast", fast, true}, {"spinning", spinning, true}, {"kinematic", kinematic, false}, {"sleeping", sleeping, false}} {
		if got := isFast(c.body, sceneDt); got != c.want {
			t.Errorf("%s: fast %v, want %v", c.name, got, c.want)
		}
	}
}

// fastScene: projectiles thrown at 20 to 40 m/s into a pile of crates, in a closed static box, bouncing: fast bodies
// at every step
func fastScene(workers int) *World {
	w := newScene(workers)
	w.parallelFrom = 2
	addGround(w, 0.5)
	for _, x := range []float64{-4, 4} {
		addBody(w, mgl64.Vec3{x, 2, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 2, 4}}, actor.BodyTypeStatic, 0.5, 1)
	}
	for _, z := range []float64{-4, 4} {
		addBody(w, mgl64.Vec3{0, 2, z}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{4, 2, 0.1}}, actor.BodyTypeStatic, 0.5, 1)
	}
	for i := 0; i < 12; i++ {
		addBody(w, mgl64.Vec3{float64(i%3-1) * 0.6, 0.25 + float64(i/3)*0.5, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	}
	for i := 0; i < 16; i++ {
		angle := float64(i) * 2 * math.Pi / 16
		ball := addBody(w, mgl64.Vec3{3 * math.Cos(angle), 0.5 + 0.2*float64(i%4), 3 * math.Sin(angle)}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.03}, actor.BodyTypeDynamic, 0.5, 1)
		speed := 20 + 1.25*float64(i)
		ball.Velocity = mgl64.Vec3{-speed * math.Cos(angle), 2, -speed * math.Sin(angle)}
	}
	return w
}

// The continuous collision of the fast bodies gives the same bits with 1 and 8 workers, and allocates nothing
func TestContinuousIsDeterministicAndDoesNotAllocate(t *testing.T) {
	single, parallel := fastScene(1), fastScene(8)
	defer single.Close()
	defer parallel.Close()
	fast := 0
	for step := 0; step < 240; step++ {
		single.Step(sceneDt)
		parallel.Step(sceneDt)
		for _, body := range single.Bodies {
			if isFast(body, sceneDt) {
				fast++
			}
		}
	}
	for i := range single.Bodies {
		if single.Bodies[i].Transform != parallel.Bodies[i].Transform {
			t.Fatalf("body %d: %v with 1 worker, %v with 8", i, single.Bodies[i].Transform, parallel.Bodies[i].Transform)
		}
	}
	if fast < 240*8 {
		t.Errorf("%d fast bodies over 240 steps, want at least 8 per step: the scene is not fast enough", fast)
	}
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, w := range []*World{single, parallel} {
		if allocs := testing.AllocsPerRun(50, func() { w.Step(sceneDt) }); allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", w.Workers, allocs)
		}
	}
}
