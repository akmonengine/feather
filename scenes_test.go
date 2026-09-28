package feather

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== MINIMAL SCENES ==========
// Each scene isolates one mechanism, with one body (or two) and no chaos: the result doesn't depend on a draw.
// The bound of every scene comes from a quantity of the engine, never from a measure:
// landingDepth = LinearSlop, the length tolerance of the collision detection. A speculative contact stops a point before
// it touches: a point goes deeper only if the contact misses it (reduction, rotation during the step, no contact)
const landingDepth = LinearSlop

// cornerDown: the rotation putting the corner (1, 1, 1) of a box at the bottom
func cornerDown() mgl64.Quat {
	return mgl64.QuatBetweenVectors(mgl64.Vec3{1, 1, 1}.Normalize(), mgl64.Vec3{0, -1, 0})
}

// ridgeTerrain: a terrain folded along z at x = 0, 41x41 samples every 0.25 m, sloped by angle on both sides:
// a ridge (convex fold) or a V (concave fold)
func ridgeTerrain(w *World, angle float64, ridge bool) *actor.RigidBody {
	const samples, spacing = 41, 0.25
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			height := math.Abs(float64(x)-(samples-1)/2.0) * spacing * math.Tan(angle)
			if ridge {
				height = -height
			}
			heights[x*samples+z] = float32(height)
		}
	}
	field := actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{spacing, 1, spacing})
	return addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), field, actor.BodyTypeStatic, 0.6, 0)
}

// worstDepth runs the scene and returns the deepest point of the body under the static body, over all the steps
func worstDepth(w *World, surface, body *actor.RigidBody, seconds float64) float64 {
	worst := 0.0
	simulate(w, seconds, func() { worst = math.Max(worst, surfaceDepth(surface, body)) })
	return worst
}

// A box falls on a corner while spinning, on a plane, on a sloped terrain (across the diagonals of the cells) and on a
// static box. A thin plate falls fast and tilted: its 8 corners are in the speculative margin, only its bottom face
// touches. Turned by 45° over a box, the clipping of its face gives 8 points: the reduction to 4 must keep the deepest
func TestBoxLandsOnCorner(t *testing.T) {
	plate := &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.02, 0.3}}
	tilted := mgl64.QuatRotate(0.15, mgl64.Vec3{1, 0, 1}.Normalize())
	cases := []struct {
		name     string
		surface  string
		shape    actor.ShapeInterface
		rotation mgl64.Quat
		velocity mgl64.Vec3
		spin     mgl64.Vec3
	}{
		{"box on a plane", "plane", cube(), cornerDown(), mgl64.Vec3{}, mgl64.Vec3{3, 8, -2}},
		{"box on a terrain", "terrain", cube(), cornerDown(), mgl64.Vec3{}, mgl64.Vec3{3, 8, -2}},
		{"box on a box", "box", cube(), cornerDown(), mgl64.Vec3{}, mgl64.Vec3{3, 8, -2}},
		{"plate on a plane", "plane", plate, tilted, mgl64.Vec3{0, -12, 0}, mgl64.Vec3{0, 4, 0}},
		{"plate on a terrain", "terrain", plate, tilted, mgl64.Vec3{0, -12, 0}, mgl64.Vec3{0, 4, 0}},
		{"plate on a box", "box", plate, mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0}).Mul(tilted), mgl64.Vec3{0, -12, 0}, mgl64.Vec3{}},
	}
	for _, c := range cases {
		w := newScene(1)
		var surface *actor.RigidBody
		start := mgl64.Vec3{0.3, 1.2, 0.4}
		switch c.surface {
		case "terrain":
			surface = slopeTerrain(w, 15*math.Pi/180, 0.6)
			start[1] += 0.3 * math.Tan(15*math.Pi/180)
		case "box":
			surface = addBody(w, mgl64.Vec3{0.3, -0.25, 0.4}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.25, 0.3}}, actor.BodyTypeStatic, 0.6, 0)
		default:
			surface = addGround(w, 0.6)
		}
		body := addBody(w, start, c.rotation, c.shape, actor.BodyTypeDynamic, 0.6, 0)
		body.Velocity, body.AngularVelocity = c.velocity, c.spin
		depth := worstDepth(w, surface, body, 2)
		t.Logf("%s: %.2f mm", c.name, depth*1000)
		if depth > landingDepth {
			t.Errorf("%s: %.2f mm under the surface", c.name, depth*1000)
		}
	}
}

// A capsule tumbles at 30 rad/s while sliding on a plane, and spins like a top on its end at 20 rad/s: its contact
// points turn with it during the step, and its separation follows its rounded ends (cores)
func TestCapsuleTumbles(t *testing.T) {
	shape := &actor.Capsule{HalfHeight: 0.22, Radius: 0.15}
	tilted := mgl64.QuatRotate(0.6, mgl64.Vec3{0, 0, 1})
	axis := tilted.Rotate(mgl64.Vec3{0, 1, 0})
	cases := []struct {
		name     string
		position mgl64.Vec3
		rotation mgl64.Quat
		velocity mgl64.Vec3
		spin     mgl64.Vec3
	}{
		{"tumbling", mgl64.Vec3{0, 0.5, 0}, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1}), mgl64.Vec3{4, 0, 0}, mgl64.Vec3{0, 0, -30}},
		// resting on its lower end, spinning around its axis & precessing
		{"spinning top", axis.Mul(shape.HalfHeight).Add(mgl64.Vec3{0, shape.Radius, 0}), tilted, mgl64.Vec3{}, axis.Mul(20).Add(mgl64.Vec3{0, 5, 0})},
	}
	for _, c := range cases {
		w := newScene(1)
		ground := addGround(w, 0.15)
		capsule := addBody(w, c.position, c.rotation, shape, actor.BodyTypeDynamic, 0.15, 0)
		capsule.Velocity, capsule.AngularVelocity = c.velocity, c.spin
		depth := worstDepth(w, ground, capsule, 3)
		t.Logf("%s: %.2f mm", c.name, depth*1000)
		if depth > landingDepth {
			t.Errorf("%s: the capsule went %.2f mm under the ground", c.name, depth*1000)
		}
	}
}

// A sphere rolls fast into a static wall: it stops on it
func TestSphereRollsIntoWall(t *testing.T) {
	for _, speed := range []float64{3, 6, 10} {
		w := newScene(1)
		addGround(w, 0.6)
		wall := addBody(w, mgl64.Vec3{3.5, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 2}}, actor.BodyTypeStatic, 0.6, 0)
		sphere := addBody(w, mgl64.Vec3{0, 0.2, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.2}, actor.BodyTypeDynamic, 0.6, 0)
		sphere.Velocity = mgl64.Vec3{speed, 0, 0}
		sphere.AngularVelocity = mgl64.Vec3{0, 0, -speed / 0.2}
		depth := worstDepth(w, wall, sphere, 1)
		t.Logf("%.0f m/s: %.2f mm in the wall", speed, depth*1000)
		if depth > landingDepth {
			t.Errorf("%.0f m/s: the sphere went %.2f mm in the wall", speed, depth*1000)
		}
	}
}

// A box lands astride a ridge of the terrain: the active edge of the ridge holds it, it tips over on one side and rests
// on the slope (tan 20° < friction)
func TestBoxOnRidge(t *testing.T) {
	w := newScene(1)
	terrain := ridgeTerrain(w, 20*math.Pi/180, true)
	box := addBody(w, mgl64.Vec3{0.05, 0.6, 0.1}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.1, 0.3}}, actor.BodyTypeDynamic, 0.6, 0)
	depth := worstDepth(w, terrain, box, 3)
	// the slope under the box
	side := math.Copysign(1, box.Transform.Position.X())
	slope := mgl64.Vec3{side * math.Sin(20*math.Pi/180), math.Cos(20 * math.Pi / 180), 0}
	tilt := math.Acos(math.Min(1, math.Abs(box.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}).Dot(slope))))
	t.Logf("%.2f mm, at x=%.3f, %.2f° from the slope, asleep %v", depth*1000, box.Transform.Position.X(), degrees(tilt), box.IsSleeping)
	if depth > landingDepth {
		t.Errorf("the box went %.2f mm under the ridge", depth*1000)
	}
	// flat on the slope: its corners at the same depth, within the tolerance of the detection, across its width
	if !box.IsSleeping || tilt > math.Atan(LinearSlop/0.6) {
		t.Error("the box doesn't rest on the slope")
	}
}

// A sphere dropped in a V rests on both slopes: its center at r / cos(angle) above the fold
func TestSphereInV(t *testing.T) {
	const radius = 0.3
	angle := 30 * math.Pi / 180
	w := newScene(1)
	terrain := ridgeTerrain(w, angle, false)
	sphere := addBody(w, mgl64.Vec3{0.02, 1, 0.1}, mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, 0.6, 0)
	depth := worstDepth(w, terrain, sphere, 3)
	want := radius / math.Cos(angle)
	height := sphere.Transform.Position.Y()
	t.Logf("%.2f mm, height %.4f m (want %.4f), asleep %v", depth*1000, height, want, sphere.IsSleeping)
	if depth > landingDepth {
		t.Errorf("the sphere went %.2f mm under the terrain", depth*1000)
	}
	// at rest, no point is deeper than the tolerance of the detection
	if math.Abs(height-want) > LinearSlop || !sphere.IsSleeping {
		t.Errorf("the sphere rests at %.4f m, want %.4f", height, want)
	}
}

// A box falls on a corner on another box resting on the ground: the dynamic bodies don't overlap more than the
// tolerance of the detection
func TestBoxLandsOnBox(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	lower := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	upper := addBody(w, mgl64.Vec3{0.05, 1.5, 0.08}, cornerDown(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	upper.AngularVelocity = mgl64.Vec3{0, 5, 0}
	worst := 0.0
	simulate(w, 2, func() { worst = math.Max(worst, boxOverlap(lower, upper)) })
	t.Logf("%.2f mm", worst*1000)
	if worst > landingDepth {
		t.Errorf("the boxes overlap by %.2f mm", worst*1000)
	}
}

// A small ball at 200 m/s (4 m per step, 400 times the wall thickness) against a wall of 1 cm: the continuous collision
// stops it on the wall
func TestFastBodyAgainstThinWall(t *testing.T) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	wall := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.005, 2, 2}}, actor.BodyTypeStatic, 0, 0)
	ball := addBody(w, mgl64.Vec3{-6, 0.3, 0.2}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.05}, actor.BodyTypeDynamic, 0, 0)
	ball.Velocity = mgl64.Vec3{200, 0, 0}
	depth := worstDepth(w, wall, ball, 0.5)
	t.Logf("%.2f mm, at x=%.3f", depth*1000, ball.Transform.Position.X())
	if ball.Transform.Position.X() > 0 {
		t.Fatalf("the ball went through the wall: x=%.3f", ball.Transform.Position.X())
	}
	if depth > landingDepth {
		t.Errorf("the ball went %.2f mm in the wall", depth*1000)
	}
}
