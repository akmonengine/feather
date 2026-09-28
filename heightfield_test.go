package feather

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// slopeTerrain: a flat terrain tilted along X (height = tan(angle) * x), 64x64 samples every 0.5 m
func slopeTerrain(w *World, angle float64, friction float64) *actor.RigidBody {
	const samples, spacing = 64, 0.5
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32((float64(x) - (samples-1)/2.0) * spacing * math.Tan(angle))
		}
	}
	field := actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{spacing, 1, spacing})
	return addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), field, actor.BodyTypeStatic, friction, 0)
}

// On a flat slope, a sphere rolls exactly like on a plane: the inner edges of the terrain are invisible
func TestHeightfieldSphereRollsLikeOnPlane(t *testing.T) {
	angle := 15 * math.Pi / 180
	normal := mgl64.Vec3{-math.Sin(angle), math.Cos(angle), 0}
	start := mgl64.Vec3{3.1, 3.1*math.Tan(angle) + 0.3/math.Cos(angle), 0.37}

	onPlane := newScene(1)
	addBody(onPlane, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Plane{Normal: normal}, actor.BodyTypeStatic, 0.5, 0)
	planeSphere := addBody(onPlane, start, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 0.5, 0)

	onTerrain := newScene(1)
	slopeTerrain(onTerrain, angle, 0.5)
	terrainSphere := addBody(onTerrain, start, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 0.5, 0)

	worst := 0.0
	for step := 0; step < int(math.Round(2/sceneDt)); step++ {
		onPlane.Step(sceneDt)
		onTerrain.Step(sceneDt)
		worst = math.Max(worst, planeSphere.Transform.Position.Sub(terrainSphere.Transform.Position).Len())
	}
	travelled := terrainSphere.Transform.Position.Sub(start).Len()
	t.Logf("travelled %.2f m, worst gap with the plane %.4f mm", travelled, worst*1000)
	if travelled < 2 {
		t.Errorf("the sphere didn't roll: %.2f m", travelled)
	}
	if worst > 0.001 {
		t.Errorf("the sphere is %.3f mm from the sphere on the plane", worst*1000)
	}
}

// A box sliding on a flat terrain crosses the inner edges without being kicked
func TestHeightfieldBoxSlidesOverInnerEdges(t *testing.T) {
	for _, angle := range []float64{0, 10 * math.Pi / 180} {
		w := newScene(1)
		slopeTerrain(w, angle, 0.1)
		normal := mgl64.Vec3{-math.Sin(angle), math.Cos(angle), 0}
		rotation := mgl64.QuatBetweenVectors(mgl64.Vec3{0, 1, 0}, normal)
		position := mgl64.Vec3{-4, -4 * math.Tan(angle), 0.2}.Add(normal.Mul(cubeHalf))
		box := addBody(w, position, rotation, cube(), actor.BodyTypeDynamic, 0.1, 0)
		// sliding up the slope and sideways: across the diagonals and the sides of the cells
		box.Velocity = rotation.Rotate(mgl64.Vec3{5, 0, 2})

		worstNormalSpeed, worstSpin := 0.0, 0.0
		simulate(w, 1, func() {
			worstNormalSpeed = math.Max(worstNormalSpeed, math.Abs(box.Velocity.Dot(normal)))
			worstSpin = math.Max(worstSpin, box.AngularVelocity.Len())
		})
		travelled := box.Transform.Position.Sub(position).Len()
		t.Logf("slope %.0f°: travelled %.2f m, worst normal speed %.4f m/s, worst spin %.4f rad/s", degrees(angle), travelled, worstNormalSpeed, worstSpin)
		if travelled < 2 {
			t.Errorf("slope %.0f°: the box stopped after %.2f m", degrees(angle), travelled)
		}
		if worstNormalSpeed > 0.01 || worstSpin > 0.05 {
			t.Errorf("slope %.0f°: the box was kicked by an inner edge", degrees(angle))
		}
	}
}

// bumpyTerrain: random hills in a bowl (the bodies stay on the terrain), 48x48 samples every 0.5 m
func bumpyTerrain(w *World, seed int64) *actor.RigidBody {
	const samples = 48
	r := rand.New(rand.NewSource(seed))
	heights := make([]float32, samples*samples)
	phases := [4]float64{r.Float64() * 6, r.Float64() * 6, r.Float64() * 6, r.Float64() * 6}
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			bowlX, bowlZ := (float64(x)-(samples-1)/2.0)*0.5, (float64(z)-(samples-1)/2.0)*0.5
			heights[x*samples+z] = float32(0.03*(bowlX*bowlX+bowlZ*bowlZ) + 0.6*math.Sin(float64(x)*0.35+phases[0])*math.Cos(float64(z)*0.3+phases[1]) +
				0.3*math.Sin(float64(x+z)*0.8+phases[2]) + 0.05*math.Cos(float64(x-z)*1.7+phases[3]))
		}
	}
	field := actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{0.5, 1, 0.5})
	return addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), field, actor.BodyTypeStatic, 0.6, 0)
}

// A body falls through a hole of the terrain, and rests beside it
func TestHeightfieldHoles(t *testing.T) {
	w := newScene(1)
	heights := make([]float32, 9*9)
	field := actor.NewHeightfield(9, 9, heights, mgl64.Vec3{1, 1, 1})
	field.Holes = make([]bool, 8*8)
	// 2x2 cells around the center
	for _, cell := range [4][2]int{{3, 3}, {3, 4}, {4, 3}, {4, 4}} {
		field.Holes[cell[0]*8+cell[1]] = true
	}
	field.Update(0, 0, 8, 8)
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), field, actor.BodyTypeStatic, 0.5, 0)
	falling := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 0.5, 0)
	resting := addBody(w, mgl64.Vec3{2.5, 1, 2.5}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 0.5, 0)
	simulate(w, 1.5, nil)
	if falling.Transform.Position.Y() > -2 {
		t.Errorf("the sphere didn't fall through the hole: %v", falling.Transform.Position)
	}
	if math.Abs(resting.Transform.Position.Y()-0.3) > 0.001 {
		t.Errorf("the sphere beside the hole is at %.4f m, want 0.3", resting.Transform.Position.Y())
	}
}

// Digging the terrain under a sleeping body wakes it up: it falls in the pit
func TestHeightfieldUpdateWakesBodies(t *testing.T) {
	w := newScene(1)
	terrain := slopeTerrain(w, 0, 0.5)
	field := terrain.Shape.(*actor.Heightfield)
	box := addBody(w, mgl64.Vec3{0.1, cubeHalf, 0.2}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	far := addBody(w, mgl64.Vec3{8, cubeHalf, 8}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	simulate(w, 2, nil)
	if !box.IsSleeping || !far.IsSleeping {
		t.Fatal("the boxes are not asleep")
	}

	// a pit of 1 m under the box: the samples around the center
	for x := 29; x <= 34; x++ {
		for z := 29; z <= 34; z++ {
			field.Heights[x*64+z] = -1
		}
	}
	w.UpdateHeightfield(terrain, 29, 29, 34, 34)
	simulate(w, 2, nil)
	if box.Transform.Position.Y() > -0.7 {
		t.Errorf("the box didn't fall in the pit: %v", box.Transform.Position)
	}
	if !far.IsSleeping {
		t.Error("the box far from the pit woke up")
	}
}

// A box resting in a V valley touches both slopes: 2 patches, and it falls asleep
func TestHeightfieldValley(t *testing.T) {
	w := newScene(1)
	const samples = 21
	heights := make([]float32, samples*samples)
	angle := 30 * math.Pi / 180
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32(math.Abs(float64(x)-10) * 0.25 * math.Tan(angle))
		}
	}
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{0.25, 1, 0.25}), actor.BodyTypeStatic, 0.6, 0)
	box := addBody(w, mgl64.Vec3{0, 0.8, 0.1}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.2, 0.3}}, actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 0.5, nil)
	patches := 0
	for _, m := range w.Contacts() {
		if m.BodyA == box || m.BodyB == box {
			patches++
		}
	}
	simulate(w, 2.5, nil)
	// resting on both slopes: the bottom edges at 0.3 m from the middle, so the center at 0.3*tan + 0.2
	want := 0.3*math.Tan(angle) + 0.2
	t.Logf("%d patches, height %.4f m (want %.4f), asleep %v", patches, box.Transform.Position.Y(), want, box.IsSleeping)
	if patches != 2 {
		t.Errorf("%d patches, want 2", patches)
	}
	if !box.IsSleeping {
		t.Error("the box is not asleep")
	}
	if math.Abs(box.Transform.Position.Y()-want) > 0.002 {
		t.Errorf("the box rests at %.4f m, want %.4f", box.Transform.Position.Y(), want)
	}
}

// A moved and turned terrain: the bodies rest on its surface
func TestHeightfieldTransform(t *testing.T) {
	w := newScene(1)
	terrain := bumpyTerrain(w, 3)
	terrain.Transform = actor.Transform{Position: mgl64.Vec3{5, -2, 3}, Rotation: mgl64.QuatRotate(0.6, mgl64.Vec3{0, 1, 0})}
	terrain.UpdateAABB()
	field := terrain.Shape.(*actor.Heightfield)
	var spheres []*actor.RigidBody
	for i := 0; i < 5; i++ {
		local := mgl64.Vec3{float64(i)*2 - 4, 0, float64(i) - 2}
		height, _ := field.HeightAt(local.X(), local.Z())
		local[1] = height + 1
		sphere := addBody(w, terrain.Transform.ToWorld(local), mgl64.QuatIdent(), &actor.Sphere{Radius: 0.2}, actor.BodyTypeDynamic, 0.8, 0)
		sphere.Material.RollingResistance = 0.3
		spheres = append(spheres, sphere)
	}
	simulate(w, 4, nil)
	for i, sphere := range spheres {
		local := terrain.Transform.ToLocal(sphere.Transform.Position)
		height, ok := field.HeightAt(local.X(), local.Z())
		if !ok || local.Y() < height || local.Y() > height+0.3 {
			t.Errorf("sphere %d: %.3f m above the terrain", i, local.Y()-height)
		}
	}
}

// The pair keeps its order: the normal goes from A to B, the terrain can be either
func TestHeightfieldPairOrder(t *testing.T) {
	w := newScene(1)
	terrain := slopeTerrain(w, 0, 0.5)
	sphere := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0.1, 0.29, 0.2}, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 1)
	var terrainFirst, sphereFirst [MaxManifoldsPerPair]constraint.Manifold
	if CollideAll(terrain, sphere, 0.02, terrainFirst[:]) != 1 || CollideAll(sphere, terrain, 0.02, sphereFirst[:]) != 1 {
		t.Fatal("no contact")
	}
	if terrainFirst[0].Normal.Sub(mgl64.Vec3{0, 1, 0}).Len() > 1e-9 || sphereFirst[0].Normal.Sub(mgl64.Vec3{0, -1, 0}).Len() > 1e-9 {
		t.Errorf("normals %v and %v", terrainFirst[0].Normal, sphereFirst[0].Normal)
	}
	if sphereFirst[0].BodyA != sphere || math.Abs(sphereFirst[0].Points[0].Separation+0.01) > 1e-9 {
		t.Errorf("sphere first: %v", sphereFirst[0])
	}
	var single constraint.Manifold
	if !Collide(sphere, terrain, 0.02, &single) || single.Count != 1 {
		t.Error("Collide: no contact")
	}
}

// terrainScene: bodies on hills, for the determinism & the allocations. The parallel paths run under 256 bodies
func terrainScene(workers int) *World {
	w := newScene(workers)
	w.parallelFrom = 1
	bumpyTerrain(w, 4)
	r := rand.New(rand.NewSource(5))
	for i := 0; i < 200; i++ {
		var shape actor.ShapeInterface = &actor.Box{HalfExtents: mgl64.Vec3{0.2, 0.15, 0.25}}
		switch i % 3 {
		case 1:
			shape = &actor.Sphere{Radius: 0.2}
		case 2:
			shape = &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}
		}
		position := mgl64.Vec3{r.Float64()*16 - 8, 5 + r.Float64()*6, r.Float64()*16 - 8}
		addBody(w, position, mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{0, 1, 0}), shape, actor.BodyTypeDynamic, 0.6, 0)
	}
	return w
}

// The same steps with 1 and 8 workers
func TestHeightfieldDeterminism(t *testing.T) {
	single, parallel := terrainScene(1), terrainScene(8)
	defer single.Close()
	defer parallel.Close()
	for step := 0; step < 120; step++ {
		single.Step(sceneDt)
		parallel.Step(sceneDt)
	}
	for i := range single.Bodies {
		if single.Bodies[i].Transform != parallel.Bodies[i].Transform {
			t.Fatalf("body %d: %v with 1 worker, %v with 8", i, single.Bodies[i].Transform, parallel.Bodies[i].Transform)
		}
	}
}

func TestHeightfieldDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool allocates under the race detector")
	}
	w := terrainScene(4)
	defer w.Close()
	// the buffers grow while the bodies land
	for step := 0; step < 300; step++ {
		w.Step(sceneDt)
	}
	allocations := testing.AllocsPerRun(20, func() { w.Step(sceneDt) })
	if allocations > 0 {
		t.Errorf("%.1f allocations per step", allocations)
	}
}
