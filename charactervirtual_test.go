package feather

import (
	"math"
	"runtime"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== CHARACTER ==========
// A character is a capsule moved by the queries of the world (CharacterVirtual): it walks on the ground, the terrains and
// the meshes, slides along the walls, climbs the slopes under its limit and the steps under its height, stays on the
// floor downhill, is carried by a moving platform, pushes the dynamic bodies with a bounded force, and never goes
// through another character. The game gives its velocity (walkingVelocity: the recipe of the samples of Jolt) and
// calls Update after each step of the world.

const (
	// the capsule of the characters of these tests: 1.7 m tall, 0.3 m wide, the standing shape of the Jolt samples
	characterRadius     = 0.3
	characterHalfHeight = 0.55

	// walkSpeed: the walking speed of the tests (m/s)
	walkSpeed = 2.0

	// characterDepthTolerance: the character never enters a body deeper than this (m): the collision tolerance of
	// Jolt (CharacterVirtualSettings::mCollisionTolerance)
	characterDepthTolerance = 1e-3
)

// addCharacter: a character standing at the position (the center of its capsule), with the defaults
func addCharacter(w *World, position mgl64.Vec3) *CharacterVirtual {
	c := NewCharacterVirtual(actor.Capsule{HalfHeight: characterHalfHeight, Radius: characterRadius}, position)
	w.AddCharacter(c)
	return c
}

// standing: the position of the center of a capsule whose feet are at the point
func standing(feet mgl64.Vec3) mgl64.Vec3 {
	return feet.Add(mgl64.Vec3{0, characterHalfHeight + characterRadius, 0})
}

// walkingVelocity: the velocity of a character which wants to walk at the horizontal velocity: the recipe of
// CharacterVirtualTest::HandleInput of Jolt (the inertia disabled): on the ground, the velocity of the ground, else the
// vertical velocity kept; plus the gravity of the step, plus the input
func walkingVelocity(w *World, c *CharacterVirtual, horizontal mgl64.Vec3, dt float64) mgl64.Vec3 {
	velocity := mgl64.Vec3{0, c.Velocity.Y(), 0}
	if c.GroundState() == CharacterOnGround && !c.IsSlopeTooSteep(c.GroundNormal()) {
		c.UpdateGroundVelocity()
		velocity = c.GroundVelocity()
	}
	return velocity.Add(w.Gravity.Mul(dt)).Add(horizontal)
}

// walk runs the world for seconds, the characters walking at their horizontal velocity (one per character), each
// updated after the step; each is called after every frame
func walk(w *World, characters []*CharacterVirtual, horizontal []mgl64.Vec3, seconds float64, each func()) {
	for i := 0; i < int(math.Round(seconds/sceneDt)); i++ {
		w.Step(sceneDt)
		for k, c := range characters {
			c.Velocity = walkingVelocity(w, c, horizontal[k], sceneDt)
			c.Update(sceneDt)
		}
		if each != nil {
			each()
		}
	}
}

// settle: the character stands for seconds, without input
func settle(w *World, c *CharacterVirtual, seconds float64) {
	walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{}}, seconds, nil)
}

// entersABody: the capsule of the character, shrunk by the tolerance, overlaps a body it collides with: the character
// is deeper in a body than the tolerance
func entersABody(w *World, c *CharacterVirtual) bool {
	shrunk := actor.Capsule{HalfHeight: c.Capsule.HalfHeight, Radius: c.Capsule.Radius - characterDepthTolerance}
	filter := DefaultQueryFilter()
	filter.Excluded = []*actor.RigidBody{c.Body()}
	var found []*actor.RigidBody
	found = w.Overlap(&shrunk, actor.Transform{Position: c.Position, Rotation: mgl64.QuatIdent()}, filter, found)
	return len(found) > 0
}

// hillyMesh: a mesh of 40 x 40 m of gentle hills (under 20° everywhere)
func hillyMesh() *actor.TriangleMesh {
	return gridMesh(40, 1, func(x, z float64) float64 {
		return 0.4*math.Sin(x*0.5) + 0.3*math.Cos(z*0.4+1)
	})
}

// Criterion 1: the character walks on a plane, a terrain and a mesh: never deeper than the tolerance in them, and it
// stays supported. Stopped, it doesn't tremble
func TestCharacterWalksOnTheGround(t *testing.T) {
	grounds := []struct {
		name string
		add  func(w *World)
		feet float64
	}{
		{"plane", func(w *World) { addGround(w, 0.6) }, 0},
		{"terrain", func(w *World) { bumpyTerrain(w, 7) }, 6},
		{"mesh", func(w *World) {
			addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), hillyMesh(), actor.BodyTypeStatic, 0.6, 0)
		}, 1},
	}
	for _, ground := range grounds {
		t.Run(ground.name, func(t *testing.T) {
			w := newScene(1)
			ground.add(w)
			c := addCharacter(w, standing(mgl64.Vec3{-8, ground.feet, -8}))
			settle(w, c, 2)
			if c.GroundState() != CharacterOnGround {
				t.Fatalf("%s: after 2 s the character is %v, want on the ground", ground.name, c.GroundState())
			}

			// a walk of 16 m along the diagonal: the character stays on the ground, never in it
			deepest, unsupported := false, 0
			walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed / math.Sqrt2, 0, walkSpeed / math.Sqrt2}}, 8, func() {
				if entersABody(w, c) {
					deepest = true
				}
				if !c.IsSupported() {
					unsupported++
				}
			})
			if deepest {
				t.Errorf("%s: the character entered the ground deeper than %.0f mm", ground.name, characterDepthTolerance*1000)
			}
			// over a crest the character can leave the ground for a frame (it was going up: it doesn't stick), and lands
			// at the next one
			if unsupported > 3 {
				t.Errorf("%s: the character was not supported during %d frames of its walk", ground.name, unsupported)
			}
			if traveled := c.Position.Sub(mgl64.Vec3{-8, 0, -8}); math.Hypot(traveled.X(), traveled.Z()) < 14 {
				t.Errorf("%s: the character walked %.2f m horizontally in 8 s at %g m/s", ground.name, math.Hypot(traveled.X(), traveled.Z()), walkSpeed)
			}

			// stopped, it doesn't tremble
			settle(w, c, 1)
			rest := c.Position
			worst := 0.0
			walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{}}, 2, func() {
				worst = math.Max(worst, c.Position.Sub(rest).Len())
			})
			if worst > 1e-5 {
				t.Errorf("%s: the character stopped moves by %.3f mm", ground.name, worst*1000)
			}
		})
	}
}

// Criterion 2: against a wall, the character slides along it: it keeps the component of its velocity along the wall,
// never enters it and doesn't stick to it
func TestCharacterSlidesAlongAWall(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	// a wall along z, 20 m long, at x = 1
	addBody(w, mgl64.Vec3{1.5, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 10}}, actor.BodyTypeStatic, 0.6, 0)
	c := addCharacter(w, standing(mgl64.Vec3{0, 0, -8}))
	settle(w, c, 0.5)

	// walking at 45° into the wall: the velocity along the wall is walkSpeed / √2
	start := c.Position
	entered := false
	walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed / math.Sqrt2, 0, walkSpeed / math.Sqrt2}}, 4, func() {
		if entersABody(w, c) {
			entered = true
		}
	})
	if entered {
		t.Error("the character entered the wall")
	}
	if x := c.Position.X(); x > 1-characterRadius+characterDepthTolerance {
		t.Errorf("the character is at x = %.4f, the wall at x = 1: it entered the wall", x)
	}
	if along := c.Position.Z() - start.Z(); along < 0.95*4*walkSpeed/math.Sqrt2 {
		t.Errorf("the character slid %.2f m along the wall in 4 s, want %.2f m", along, 4*walkSpeed/math.Sqrt2)
	}
	if c.GroundState() != CharacterOnGround {
		t.Errorf("against the wall the character is %v, want on the ground", c.GroundState())
	}
}

// ramp: a static box tilted by the angle around z, its top face through the origin, climbing towards +x: the surface
// is at the height x * tan(angle)
func ramp(w *World, angle float64) *actor.RigidBody {
	rotation := mgl64.QuatRotate(angle, mgl64.Vec3{0, 0, 1})
	half := mgl64.Vec3{20, 1, 10}
	center := rotation.Rotate(mgl64.Vec3{0, -half.Y(), 0})
	return addBody(w, center, rotation, &actor.Box{HalfExtents: half}, actor.BodyTypeStatic, 0.6, 0)
}

// Criterion 3: a slope under the limit is climbed, over the limit it blocks at its foot; a step under the step height
// is climbed, over it blocks
func TestCharacterSlopesAndSteps(t *testing.T) {
	t.Run("slopes", func(t *testing.T) {
		for _, angle := range []float64{30, 60} {
			w := newScene(1)
			addGround(w, 0.6)
			// the ramp rises from the ground at x = 0; the character walks to it from x = -1.5
			ramp(w, angle*math.Pi/180)
			c := addCharacter(w, standing(mgl64.Vec3{-1.5, 0, 0}))
			settle(w, c, 1)
			start := c.Position
			walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed, 0, 0}}, 3, nil)
			climbed := c.Position.Y() - start.Y()
			radians := angle * math.Pi / 180
			if angle < DefaultCharacterMaxSlopeAngle*180/math.Pi {
				// 0.75 s on the ground, then the horizontal velocity is projected on the slope (the slide of Jolt): the
				// character climbs at walkSpeed * cos(angle) along it, so sin * cos up (Godot keeps the speed with
				// floor_constant_speed only)
				if climbed < 0.9*2.25*walkSpeed*math.Sin(radians)*math.Cos(radians) {
					t.Errorf("slope of %g°: the character climbed %.2f m in 3 s, want %.2f", angle, climbed, 2.25*walkSpeed*math.Sin(radians)*math.Cos(radians))
				}
				if c.GroundState() != CharacterOnGround {
					t.Errorf("slope of %g°: the character is %v, want on the ground", angle, c.GroundState())
				}
			} else if climbed > 0.05 || c.Position.X() > 0 {
				// the lower sphere of the capsule stops against the slope before x = 0
				t.Errorf("slope of %g°: the character climbed %.3f m and is at x = %.3f, want blocked at the foot of the slope", angle, climbed, c.Position.X())
			}
		}
	})

	t.Run("steps", func(t *testing.T) {
		for _, rise := range []float64{0.2, 0.6} {
			w := newScene(1)
			addGround(w, 0.6)
			addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), stairMesh(6, 0.6, rise, 3), actor.BodyTypeStatic, 0.6, 0)
			c := addCharacter(w, standing(mgl64.Vec3{-2, 0, 0}))
			settle(w, c, 0.5)
			unsupported := 0
			walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed, 0, 0}}, 3, func() {
				if !c.IsSupported() {
					unsupported++
				}
			})
			if rise < DefaultCharacterStepHeight {
				// 6 steps of 0.6 m: the top tread starts at x = 3, 6 rises up
				if c.Position.X() < 3 || c.Position.Y() < standing(mgl64.Vec3{0, 6 * rise, 0}).Y()-characterDepthTolerance {
					t.Errorf("steps of %g m: the character is at (%.2f, %.2f), want on the top tread (x ≥ 3, y ≥ %.2f)", rise, c.Position.X(), c.Position.Y(), standing(mgl64.Vec3{0, 6 * rise, 0}).Y())
				}
				if unsupported > 0 {
					t.Errorf("steps of %g m: the character left the ground during %d frames", rise, unsupported)
				}
			} else if c.Position.X() > -characterRadius-CharacterPadding+characterDepthTolerance || c.Position.Y() > standing(mgl64.Vec3{}).Y()+CharacterPadding+characterDepthTolerance {
				t.Errorf("steps of %g m: the character is at (%.3f, %.3f), want blocked before the first riser", rise, c.Position.X(), c.Position.Y())
			}
		}
	})
}

// Criterion 4: downhill (a slope, then a step down) the character stays on the ground
func TestCharacterStaysOnTheFloorDownhill(t *testing.T) {
	t.Run("slope", func(t *testing.T) {
		w := newScene(1)
		ramp(w, 30*math.Pi/180)
		c := addCharacter(w, standing(mgl64.Vec3{6, 6 * math.Tan(30*math.Pi/180), 0}).Add(mgl64.Vec3{0, 0.3, 0}))
		settle(w, c, 1)
		unsupported, entered := 0, false
		walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{-1.5 * walkSpeed, 0, 0}}, 3, func() {
			if c.GroundState() != CharacterOnGround {
				unsupported++
			}
			if entersABody(w, c) {
				entered = true
			}
		})
		if unsupported > 0 {
			t.Errorf("walking down a 30° slope at %g m/s, the character left the ground during %d frames", 1.5*walkSpeed, unsupported)
		}
		if entered {
			t.Error("walking down the slope, the character entered it")
		}
	})

	t.Run("step", func(t *testing.T) {
		w := newScene(1)
		addGround(w, 0.6)
		// a platform 0.3 m high, the character walks off its edge at x = 2
		addBody(w, mgl64.Vec3{0, 0.15, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.15, 3}}, actor.BodyTypeStatic, 0.6, 0)
		c := addCharacter(w, standing(mgl64.Vec3{0, 0.3, 0}))
		settle(w, c, 0.5)
		unsupported := 0
		walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed, 0, 0}}, 2.5, func() {
			if !c.IsSupported() {
				unsupported++
			}
		})
		if unsupported > 0 {
			t.Errorf("walking off a step of 0.3 m, the character was in the air during %d frames", unsupported)
		}
		if feet := c.Position.Y() - characterHalfHeight - characterRadius; c.Position.X() < 3 || math.Abs(feet) > CharacterPadding+characterDepthTolerance {
			t.Errorf("after the step the character is at (%.2f, feet %.4f), want on the ground past x = 3", c.Position.X(), feet)
		}
	})
}

// The stick to the floor is a setting: off a platform of 1 m, a character which sticks to nothing flies, one which
// looks 0.5 m down (the default of Jolt) doesn't find the floor and flies too, one which looks 1.5 m down is put on it
func TestCharacterStickToFloorIsASetting(t *testing.T) {
	for _, stick := range []float64{0, DefaultCharacterStickToFloor, 1.5} {
		w := newScene(1)
		addGround(w, 0.6)
		// a platform 1 m high: the default of 0.5 m doesn't reach the floor off its edge, 1.5 m does
		addBody(w, mgl64.Vec3{0, 0.5, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 3}}, actor.BodyTypeStatic, 0.6, 0)
		c := addCharacter(w, standing(mgl64.Vec3{0, 1, 0}))
		if c.StickToFloor != DefaultCharacterStickToFloor {
			t.Fatalf("a new character sticks to the floor within %g m, want the default %g", c.StickToFloor, DefaultCharacterStickToFloor)
		}
		c.StickToFloor = stick
		settle(w, c, 0.5)
		unsupported := 0
		walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed, 0, 0}}, 2.5, func() {
			if !c.IsSupported() {
				unsupported++
			}
		})
		if stick >= 1 && unsupported > 0 {
			t.Errorf("sticking within %g m, the character was in the air during %d frames off the platform of 1 m", stick, unsupported)
		}
		// a fall of 1 m lasts about 27 frames, less what the edge of the platform takes
		if stick < 1 && (unsupported < 10 || unsupported > 40) {
			t.Errorf("sticking within %g m, the character was in the air during %d frames off the platform of 1 m, want a fall of 10 to 40 frames", stick, unsupported)
		}
		if feet := c.Position.Y() - characterHalfHeight - characterRadius; c.Position.X() < 3 || math.Abs(feet) > CharacterPadding+characterDepthTolerance {
			t.Errorf("sticking within %g m, the character is at (%.2f, feet %.4f), want on the ground past x = 3", stick, c.Position.X(), feet)
		}
	}
}

// Criterion 5: standing on a kinematic platform which moves, the character is carried, sideways and up
func TestCharacterIsCarriedByAPlatform(t *testing.T) {
	for _, direction := range []mgl64.Vec3{{1, 0, 0}, {0, 0.5, 0}} {
		w := newScene(1)
		addGround(w, 0.6)
		platform := kinematicBox(w, mgl64.Vec3{0, 0.5, 0}, mgl64.Vec3{2, 0.1, 2})
		c := addCharacter(w, standing(mgl64.Vec3{0, 0.6, 0}))
		settle(w, c, 0.5)
		start := c.Position
		for i := 0; i < 180; i++ {
			moveTo(platform, translated(platform.Transform, direction))
			w.Step(sceneDt)
			c.Velocity = walkingVelocity(w, c, mgl64.Vec3{}, sceneDt)
			c.Update(sceneDt)
		}
		carried := c.Position.Sub(start)
		want := direction.Mul(3)
		if carried.Sub(want).Len() > 0.05 {
			t.Errorf("platform moving at %v: the character was carried by %v in 3 s, want %v", direction, carried, want)
		}
		if c.GroundState() != CharacterOnGround || c.GroundBody() != platform {
			t.Errorf("platform moving at %v: the character is %v on %v, want on the ground of the platform", direction, c.GroundState(), c.GroundBody())
		}
	}
}

// Criterion 6: the character pushes a ball and a crate; a very heavy crate stops it. The pushed body never enters
// the inner body of the character further than the slop of the solver: after a step, the real capsule shrunk by the
// slop overlaps nothing (after an update the character can be in the padding of a body it pushes: its plane moves at
// the velocity of the body, and the step moves the body)
func TestCharacterPushesTheBodies(t *testing.T) {
	cases := []struct {
		name    string
		shape   actor.ShapeInterface
		density float64
		pushed  bool
	}{
		{"ball", &actor.Sphere{Radius: 0.3}, 100, true},
		{"crate", &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.3, 0.3}}, 100, true},
		{"heavy crate", &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}, 8000, false},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			w := newScene(1)
			addGround(w, 0.3)
			body := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{1, 0.5, 0}, Rotation: mgl64.QuatIdent()}, tc.shape, actor.BodyTypeDynamic, tc.density)
			body.Material.StaticFriction, body.Material.DynamicFriction = 0.3, 0.3
			w.AddBody(body)
			c := addCharacter(w, standing(mgl64.Vec3{0, 0, 0}))
			settle(w, c, 1)
			start := c.Position
			entered := false
			shrunk := actor.Capsule{HalfHeight: c.Capsule.HalfHeight, Radius: c.Capsule.Radius - LinearSlop}
			filter := DefaultQueryFilter()
			filter.Excluded = []*actor.RigidBody{c.Body()}
			var found []*actor.RigidBody
			for i := 0; i < 180; i++ {
				w.Step(sceneDt)
				if found = w.Overlap(&shrunk, actor.Transform{Position: c.Position, Rotation: mgl64.QuatIdent()}, filter, found[:0]); len(found) > 0 {
					entered = true
				}
				c.Velocity = walkingVelocity(w, c, mgl64.Vec3{walkSpeed, 0, 0}, sceneDt)
				c.Update(sceneDt)
			}
			advanced := c.Position.X() - start.X()
			if entered {
				t.Errorf("%s: the body entered the character further than the slop", tc.name)
			}
			if tc.pushed {
				if body.Transform.Position.X() < 2.5 || advanced < 2 {
					t.Errorf("%s: the body is at x = %.2f and the character advanced %.2f m in 3 s, want both pushed ahead", tc.name, body.Transform.Position.X(), advanced)
				}
			} else if advanced > 0.2 || body.Transform.Position.X() > 1.01 {
				t.Errorf("%s: the character advanced %.2f m and the crate moved to x = %.3f, want the character stopped", tc.name, advanced, body.Transform.Position.X())
			}
		})
	}
}

// A heavy dynamic body never goes through a standing character: it is stopped by the inner body of the character
func TestAHeavyBodyDoesNotGoThroughACharacter(t *testing.T) {
	w := newScene(1)
	addGround(w, 0)
	c := addCharacter(w, standing(mgl64.Vec3{0, 0, 0}))
	settle(w, c, 0.5)
	crate := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{-3, 0.5, 0}, Rotation: mgl64.QuatIdent()}, &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}, actor.BodyTypeDynamic, 8000)
	crate.Velocity = mgl64.Vec3{4, 0, 0}
	w.AddBody(crate)
	walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{}}, 2, nil)
	if crate.Transform.Position.X() > c.Position.X() {
		t.Errorf("the crate at x = %.2f went through the character at x = %.2f", crate.Transform.Position.X(), c.Position.X())
	}
	if crate.Transform.Position.X() < -1 {
		t.Errorf("the crate at x = %.2f never reached the character", crate.Transform.Position.X())
	}
}

// Criterion 7: 2 characters walking into each other stop against each other, 2 characters crossing slide along each
// other: they never go through each other
func TestCharactersDoNotGoThroughEachOther(t *testing.T) {
	for _, crossing := range []bool{false, true} {
		w := newScene(1)
		addGround(w, 0.6)
		a := addCharacter(w, standing(mgl64.Vec3{-2, 0, 0}))
		b := addCharacter(w, standing(mgl64.Vec3{2, 0, 0}))
		if crossing {
			b.Position = standing(mgl64.Vec3{0, 0, 2.2})
		}
		settle(w, a, 0.5)
		towardsB, towardsA := mgl64.Vec3{walkSpeed, 0, 0}, mgl64.Vec3{-walkSpeed, 0, 0}
		if crossing {
			towardsA = mgl64.Vec3{0, 0, -walkSpeed}
		}
		closest := math.Inf(1)
		walk(w, []*CharacterVirtual{a, b}, []mgl64.Vec3{towardsB, towardsA}, 3, func() {
			closest = math.Min(closest, a.Position.Sub(b.Position).Len())
		})
		if closest < 2*characterRadius-characterDepthTolerance {
			t.Errorf("crossing=%v: the characters came within %.3f m of each other, their radii sum to %.1f", crossing, closest, 2*characterRadius)
		}
		if !crossing && (math.Abs(a.Position.X()-b.Position.X()) > 2*characterRadius+2*CharacterPadding+0.05) {
			t.Errorf("head on, the characters are %.3f m apart along x, want stopped against each other", math.Abs(a.Position.X()-b.Position.X()))
		}
		if crossing && (a.Position.X() < 2 || b.Position.Z() > -1) {
			t.Errorf("crossing, a is at x = %.2f and b at z = %.2f, want both past the crossing", a.Position.X(), b.Position.Z())
		}
	}
}

// Both filters: a character doesn't see the bodies whose layer it doesn't collide with, and the pairs ignored
func TestCharacterFiltering(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	ghost := addBody(w, mgl64.Vec3{2, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 2}}, actor.BodyTypeStatic, 0.6, 0)
	ghost.Layer = 1 << 3
	ignored := addBody(w, mgl64.Vec3{5, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 2}}, actor.BodyTypeStatic, 0.6, 0)
	c := addCharacter(w, standing(mgl64.Vec3{0, 0, 0}))
	w.SetFilter(c.Body(), actor.LayerDefault, actor.AllLayers&^(1<<3))
	w.IgnoreCollision(c.Body(), ignored, true)
	settle(w, c, 0.5)
	walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed, 0, 0}}, 4, nil)
	if c.Position.X() < 7 {
		t.Errorf("the character is at x = %.2f: a wall it doesn't collide with stopped it", c.Position.X())
	}
}

// A character teleported takes its new place at once, and refreshes its contacts
func TestCharacterTeleport(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	c := addCharacter(w, standing(mgl64.Vec3{0, 0, 0}))
	settle(w, c, 0.5)
	c.Teleport(standing(mgl64.Vec3{10, 5, 0}))
	if c.GroundState() != CharacterInAir {
		t.Errorf("teleported in the air the character is %v", c.GroundState())
	}
	if c.Body().Transform.Position != c.Position {
		t.Errorf("the inner body is at %v, the character at %v", c.Body().Transform.Position, c.Position)
	}
	hit, ok := w.Raycast(mgl64.Vec3{10, 10, 0}, mgl64.Vec3{0, -10, 0}, DefaultQueryFilter())
	if !ok || hit.Body != c.Body() {
		t.Errorf("a ray over the new place hits %v, want the character", hit.Body)
	}
	settle(w, c, 2)
	if c.GroundState() != CharacterOnGround || math.Abs(c.Position.X()-10) > 1e-9 {
		t.Errorf("after falling the character is %v at %v", c.GroundState(), c.Position)
	}
}

// A character removed leaves the world: its inner body is gone, a ray doesn't see it
func TestRemoveCharacter(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	c := addCharacter(w, standing(mgl64.Vec3{0, 0, 0}))
	settle(w, c, 0.5)
	bodies := len(w.Bodies)
	w.RemoveCharacter(c)
	if len(w.Bodies) != bodies-1 {
		t.Fatalf("%d bodies after the removal, want %d", len(w.Bodies), bodies-1)
	}
	if _, ok := w.Raycast(mgl64.Vec3{0, 5, 0}, mgl64.Vec3{0, -4, 0}, DefaultQueryFilter()); ok {
		t.Error("a ray still hits the removed character")
	}
}

// charactersScene: count characters walking on a bumpy terrain towards its center, among walls and crates (24 is the
// capacity of Harmony), the walls to slide along and to push against, the crates to push. The same scene whatever the
// workers. Without the decor, the characters walk the open terrain in circles (openTerrainScene)
func charactersScene(count, workers int) (*World, []*CharacterVirtual, []mgl64.Vec3) {
	return charactersSceneOf(count, workers, nil)
}

// charactersSceneOf: charactersScene with the characters given (new ones if nil), out of any world
func charactersSceneOf(count, workers int, given []*CharacterVirtual) (*World, []*CharacterVirtual, []mgl64.Vec3) {
	w, characters, directions := openTerrainSceneOf(count, workers, given)
	for i, c := range characters {
		angle := float64(i) * 2 * math.Pi / float64(count)
		c.Teleport(standing(mgl64.Vec3{(9 + float64(i%3)) * math.Cos(angle), 4, (9 + float64(i%3)) * math.Sin(angle)}))
		directions[i] = mgl64.Vec3{-walkSpeed * math.Cos(angle), 0, -walkSpeed * math.Sin(angle)}
	}
	for i := 0; i < 6; i++ {
		angle := float64(i) * math.Pi / 3
		addBody(w, mgl64.Vec3{6 * math.Cos(angle), 3, 6 * math.Sin(angle)}, mgl64.QuatRotate(-angle, mgl64.Vec3{0, 1, 0}), &actor.Box{HalfExtents: mgl64.Vec3{0.3, 2, 2}}, actor.BodyTypeStatic, 0.6, 0)
	}
	for i := 0; i < 12; i++ {
		angle := float64(i) * math.Pi / 6
		addBody(w, mgl64.Vec3{3 * math.Cos(angle), 4, 3 * math.Sin(angle)}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0)
	}
	return w, characters, directions
}

// openTerrainScene: count characters on the bumpy terrain, each walking its own circle of 3 m, away from the others
func openTerrainScene(count, workers int) (*World, []*CharacterVirtual, []mgl64.Vec3) {
	return openTerrainSceneOf(count, workers, nil)
}

// openTerrainSceneOf: openTerrainScene with the characters given (new ones if nil), out of any world: each is put at
// its place without velocity, as a new one
func openTerrainSceneOf(count, workers int, given []*CharacterVirtual) (*World, []*CharacterVirtual, []mgl64.Vec3) {
	w := newScene(workers)
	w.parallelFrom = 1
	bumpyTerrain(w, 11)
	var characters []*CharacterVirtual
	var directions []mgl64.Vec3
	for i := 0; i < count; i++ {
		angle := float64(i) * 2 * math.Pi / float64(count)
		radius := 9 + float64(i%3)
		position := standing(mgl64.Vec3{radius * math.Cos(angle), 4, radius * math.Sin(angle)})
		if given == nil {
			characters = append(characters, addCharacter(w, position))
		} else {
			c := given[i]
			c.Position, c.Velocity, c.lastDt = position, mgl64.Vec3{}, 0
			w.AddCharacter(c)
			characters = append(characters, c)
		}
		directions = append(directions, mgl64.Vec3{-walkSpeed * math.Sin(angle), 0, walkSpeed * math.Cos(angle)})
	}
	return w, characters, directions
}

// circling: the horizontal velocities of the characters of openTerrainScene after the frame i: each turns on its own
// circle (a turn of 3 m of radius every 9.4 s)
func circling(directions []mgl64.Vec3, i int) []mgl64.Vec3 {
	turned := make([]mgl64.Vec3, len(directions))
	rotation := mgl64.QuatRotate(float64(i)*sceneDt*walkSpeed/3, mgl64.Vec3{0, 1, 0})
	for k, direction := range directions {
		turned[k] = rotation.Rotate(direction)
	}
	return turned
}

// Criterion 8: the same bits with 1 and 8 workers, the characters crossing the scene and each other
func TestCharactersAreDeterministic(t *testing.T) {
	var reference []mgl64.Vec3
	for _, workers := range []int{1, 8} {
		w, characters, directions := charactersScene(24, workers)
		walk(w, characters, directions, 6, nil)
		if workers == 1 {
			for _, c := range characters {
				reference = append(reference, c.Position)
			}
			continue
		}
		for i, c := range characters {
			for k := 0; k < 3; k++ {
				if math.Float64bits(c.Position[k]) != math.Float64bits(reference[i][k]) {
					t.Fatalf("workers=%d: the character %d is at %v, %v with 1 worker", workers, i, c.Position, reference[i])
				}
			}
		}
	}
}

// Criterion 8: the updates of 24 characters allocate nothing once their buffers have grown. The characters cross the
// crowded scene for 2 s, then the same scene built again and walked again from the same places: the same updates, bit
// for bit, in the buffers which grew for them the first time. The buffers of the contacts and of the sweeps of a
// character are its own: from a sync.Pool, which the garbage collector empties, they grew again at times, and an
// update allocated: 2 collections before each frame, which empty every sync.Pool, change nothing to the updates.
// The steps between the updates take theirs from the pools, they are not measured here (a step with
// kinematic bodies moving: TestKinematicStepDoesNotAllocate), nor the first frame of the second scene: its world grows
// its own buffers (the free nodes of its tree, when the inner bodies first move).
// The count is the one of testing.AllocsPerRun, per frame: the counter of the runtime counts its own allocations too,
// on its goroutines (the scavenger, the threads, the marking of a collection) and in its caches of the conversions
// between interfaces (filled at random, 1 call in 1024): in about 1 run in 25, one of them falls between 2 reads of the
// counter. An update which allocates allocates at every frame
func TestCharacterUpdateDoesNotAllocate(t *testing.T) {
	const seconds = 2
	first, characters, directions := charactersScene(24, 1)
	walk(first, characters, directions, seconds, nil)
	var reached []mgl64.Vec3
	for _, c := range characters {
		reached = append(reached, c.Position)
		first.RemoveCharacter(c)
	}
	first.Close()

	second, _, _ := charactersSceneOf(24, 1, characters)
	defer second.Close()
	var mallocs uint64
	var before, after runtime.MemStats
	frames := int(math.Round(seconds / sceneDt))
	for i := 0; i < frames; i++ {
		second.Step(sceneDt)
		runtime.GC()
		runtime.GC()
		runtime.ReadMemStats(&before)
		for k, c := range characters {
			c.Velocity = walkingVelocity(second, c, directions[k], sceneDt)
			c.Update(sceneDt)
		}
		runtime.ReadMemStats(&after)
		if i > 0 {
			mallocs += after.Mallocs - before.Mallocs
		}
	}
	for i, c := range characters {
		if c.Position != reached[i] {
			t.Fatalf("the character %d is at %v walking the scene again, %v the first time: not the same updates", i, c.Position, reached[i])
		}
	}
	if perFrame := mallocs / uint64(frames-1); perFrame > 0 {
		t.Errorf("%d allocations per frame of 24 updates (%d in %d frames), want 0", perFrame, mallocs, frames-1)
	}
}

// BenchmarkCharacters: the cost of the updates of 24 and 200 characters, walking the open terrain in circles (the
// walk of a game) and pushing against the walls and the crates of the crowded scene (every frame against a wall: a
// stair walk tried and cancelled), and the cost of the frame of the crowded scene, step included
func BenchmarkCharacters(b *testing.B) {
	for _, count := range []int{24, 200} {
		w, characters, directions := openTerrainScene(count, 1)
		walk(w, characters, directions, 1, nil)
		b.Run("open_terrain="+itoa(count), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				turned := circling(directions, i)
				w.Step(sceneDt)
				b.StartTimer()
				for k, c := range characters {
					c.Velocity = walkingVelocity(w, c, turned[k], sceneDt)
					c.Update(sceneDt)
				}
				b.StopTimer()
			}
		})
		w, characters, directions = charactersScene(count, 1)
		walk(w, characters, directions, 1, nil)
		b.Run("crowd="+itoa(count), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				w.Step(sceneDt)
				b.StartTimer()
				for k, c := range characters {
					c.Velocity = walkingVelocity(w, c, directions[k], sceneDt)
					c.Update(sceneDt)
				}
				b.StopTimer()
			}
		})
		b.Run("crowd_frame="+itoa(count), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				walk(w, characters, directions, sceneDt, nil)
			}
		})
	}
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	s := ""
	for n > 0 {
		s = string(rune('0'+n%10)) + s
		n /= 10
	}
	return s
}

// A character walks through a static trigger zone: its inner body enters the zone once and leaves it once, the zone
// never blocks it nor makes a contact with it; stopped in the zone, it stays in it (no exit while it stands)
func TestCharacterEntersAndLeavesATrigger(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.5)
	zone := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{zoneHalf, 1, zoneHalf}}, actor.BodyTypeStatic, 0, 0)
	zone.IsTrigger = true
	c := addCharacter(w, standing(mgl64.Vec3{-2 * zoneHalf, 0, 0}))
	settle(w, c, 0.5)
	log := listenTriggers(w).listenCollisions()
	body := c.Body()

	// to the center of the zone, then standing in it
	walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed, 0, 0}}, 2*zoneHalf/walkSpeed, nil)
	if x := c.Position.X(); math.Abs(x) > characterRadius {
		t.Fatalf("the character is at x=%.3f after walking to the center of the zone, want 0: the zone blocked it", x)
	}
	if enter, exit := log.count(EventTriggerEnter, zone, body), log.count(EventTriggerExit, zone, body); enter != 1 || exit != 0 {
		t.Fatalf("%d enters and %d exits walking into the zone, want 1 and 0", enter, exit)
	}
	settle(w, c, 2)
	if exit := log.count(EventTriggerExit, zone, body); exit != 0 {
		t.Fatalf("%d exits while the character stands in the zone, want 0", exit)
	}

	// out of the zone
	walk(w, []*CharacterVirtual{c}, []mgl64.Vec3{{walkSpeed, 0, 0}}, 2*zoneHalf/walkSpeed, nil)
	if x := c.Position.X(); x < 2*zoneHalf-characterRadius {
		t.Fatalf("the character is at x=%.3f after walking out of the zone, want %.1f", x, 2*zoneHalf)
	}
	if enter, exit := log.count(EventTriggerEnter, zone, body), log.count(EventTriggerExit, zone, body); enter != 1 || exit != 1 {
		t.Fatalf("%d enters and %d exits after crossing the zone, want 1 and 1", enter, exit)
	}
	for _, record := range log.events {
		if record.eventType == EventCollisionEnter || record.eventType == EventCollisionStay || record.eventType == EventCollisionExit {
			t.Fatalf("collision event %v of the character with the zone or the ground", record.eventType)
		}
	}
}
