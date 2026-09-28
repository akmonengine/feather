package feather

import (
	"math"
	"math/rand"
	"slices"
	"sync"
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

// Bodies dropped on hills land without going through the terrain.
// A pile is chaotic: a tiny change moves every body, so 10 piles are measured, not one.
// Known limit: a capsule resting across a bump can stay a few mm in the terrain (logged, not checked): the contact of
// the face of a triangle comes from the feature of the body above the triangle, the middle of a capsule is missed
func TestHeightfieldPile(t *testing.T) {
	// the 10 piles run in parallel, each writes its own result
	const piles = 10
	landings, restings, fell := make([]float64, piles), make([]float64, piles), make([]bool, piles)
	var wg sync.WaitGroup
	for pile := 0; pile < piles; pile++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			seed := int64(pile + 1)
			w := newScene(1)
			terrain := bumpyTerrain(w, seed)
			field := terrain.Shape.(*actor.Heightfield)
			r := rand.New(rand.NewSource(seed + 100))
			var bodies []*actor.RigidBody
			for i := 0; i < 60; i++ {
				var shape actor.ShapeInterface = &actor.Box{HalfExtents: mgl64.Vec3{0.2 + 0.2*r.Float64(), 0.15, 0.25}}
				switch i % 3 {
				case 1:
					shape = &actor.Sphere{Radius: 0.2}
				case 2:
					shape = &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}
				}
				x, z := r.Float64()*16-8, r.Float64()*16-8
				ground, _ := field.HeightAt(x, z)
				position := mgl64.Vec3{x, ground + 1 + r.Float64()*3, z}
				rotation := mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{r.Float64(), r.Float64(), r.Float64()}.Normalize())
				body := addBody(w, position, rotation, shape, actor.BodyTypeDynamic, 0.6, 0)
				body.Material.RollingResistance = 0.1
				bodies = append(bodies, body)
			}

			// the deepest point of the bodies under the terrain
			depth := func() float64 {
				worst := 0.0
				for _, body := range bodies {
					worst = math.Max(worst, depthUnder(field, body))
				}
				return worst
			}
			simulate(w, 4, func() { landings[pile] = math.Max(landings[pile], depth()) })
			restings[pile] = depth()
			for _, body := range bodies {
				if !finite(body.Transform.Position) || body.Transform.Position.Y() < -3 {
					fell[pile] = true
				}
			}
		}()
	}
	wg.Wait()
	if slices.Contains(fell, true) {
		t.Fatal("a body fell through the terrain")
	}
	slices.Sort(landings)
	slices.Sort(restings)
	median, worst := (landings[4]+landings[5])/2, landings[9]
	t.Logf("landing depth under the terrain: median %.1f mm, worst %.1f mm; after 4 s: worst %.1f mm", median*1000, worst*1000, restings[9]*1000)
	// a body landing on a slope, or tumbling fast, sinks a few mm during a step (the contacts are found once per step)
	if median > 0.01 || worst > 0.02 {
		t.Errorf("landing depth: median %.1f mm, worst %.1f mm", median*1000, worst*1000)
	}
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

// terrainScene: bodies on hills, for the determinism & the allocations
func terrainScene(workers int) *World {
	w := newScene(workers)
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

// depthUnder: how deep the body is under the terrain (0 above it)
func depthUnder(field *actor.Heightfield, body *actor.RigidBody) float64 {
	depth := 0.0
	switch shape := body.Shape.(type) {
	case *actor.Box:
		for c := 0; c < 8; c++ {
			corner := shape.HalfExtents
			for k := 0; k < 3; k++ {
				if c&(1<<k) != 0 {
					corner[k] = -corner[k]
				}
			}
			depth = math.Max(depth, -terrainDistance(field, body.Transform.ToWorld(corner)))
		}
	case *actor.Sphere:
		depth = shape.Radius - terrainDistance(field, body.Transform.Position)
	case *actor.Capsule:
		bottom, top := shape.Segment(body.Transform)
		for k := 0; k <= 10; k++ {
			depth = math.Max(depth, shape.Radius-terrainDistance(field, bottom.Add(top.Sub(bottom).Mul(float64(k)/10))))
		}
	}
	return depth
}

// terrainDistance: the distance from the point to the triangles, negative under the terrain
func terrainDistance(field *actor.Heightfield, p mgl64.Vec3) float64 {
	best := math.Inf(1)
	around := mgl64.Vec3{1, 100, 1}
	for _, cell := range field.OverlapCells(actor.AABB{Min: p.Sub(around), Max: p.Add(around)}, nil) {
		x, z := int(cell)/(field.ZSamples-1), int(cell)%(field.ZSamples-1)
		for t := 0; t < 2; t++ {
			triangle, _ := field.Triangle(x, z, t)
			best = math.Min(best, p.Sub(closestOnTriangle(p, triangle)).Len())
		}
	}
	if height, ok := field.HeightAt(p.X(), p.Z()); ok && p.Y() < height {
		return -best
	}
	return best
}

// closestOnTriangle: the closest point of the triangle (Ericson 5.1.5)
func closestOnTriangle(p mgl64.Vec3, triangle [3]mgl64.Vec3) mgl64.Vec3 {
	a, b, c := triangle[0], triangle[1], triangle[2]
	ab, ac, ap := b.Sub(a), c.Sub(a), p.Sub(a)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return a
	}
	bp := p.Sub(b)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return b
	}
	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		return a.Add(ab.Mul(d1 / (d1 - d3)))
	}
	cp := p.Sub(c)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return c
	}
	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		return a.Add(ac.Mul(d2 / (d2 - d6)))
	}
	va := d3*d6 - d5*d4
	if va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		return b.Add(c.Sub(b).Mul((d4 - d3) / ((d4 - d3) + (d5 - d6))))
	}
	denominator := 1 / (va + vb + vc)
	return a.Add(ab.Mul(vb * denominator)).Add(ac.Mul(vc * denominator))
}
