package feather

import (
	"math"
	"math/rand"
	"slices"
	"sync"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== SCENES OF THE QUERIES ==========

func closeTo(a, b mgl64.Vec3, tolerance float64) bool {
	return a.Sub(b).Len() <= tolerance
}

// queryExtent: the bodies of queryScene are in a cube of this half size (m)
const queryExtent = 12.0

// queryScene: 300 bodies of the 4 shapes spread in a cube, on a terrain, over a plane: static, dynamic awake and
// asleep, "queries only", triggers, on 3 layers. 2 pairs of bodies are at the same place, a body out of 4 is not turned
func queryScene(workers int) *World {
	w := newScene(workers)
	w.Gravity = mgl64.Vec3{}
	r := rand.New(rand.NewSource(21))
	addBody(w, mgl64.Vec3{3, -1, 2}, randomRotation(r), &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: queryExtent + 2}, actor.BodyTypeStatic, 0.5, 0)
	terrain := addBody(w, mgl64.Vec3{1, -queryExtent, -2}, mgl64.QuatRotate(0.2, mgl64.Vec3{1, 0, 0.3}.Normalize()), queryTerrain(), actor.BodyTypeStatic, 0.5, 0)
	terrain.Layer = layerDecor
	for i := 0; i < 300; i++ {
		position := mgl64.Vec3{(r.Float64()*2 - 1) * queryExtent, (r.Float64()*2 - 1) * queryExtent, (r.Float64()*2 - 1) * queryExtent}
		rotation := randomRotation(r)
		if i%4 == 0 {
			rotation = mgl64.QuatIdent()
		}
		var shape actor.ShapeInterface
		switch i % 3 {
		case 0:
			shape = &actor.Sphere{Radius: 0.2 + r.Float64()}
		case 1:
			shape = &actor.Box{HalfExtents: mgl64.Vec3{0.2 + r.Float64(), 0.2 + r.Float64(), 0.2 + r.Float64()}}
		default:
			shape = &actor.Capsule{HalfHeight: r.Float64(), Radius: 0.2 + 0.6*r.Float64()}
		}
		bodyType := actor.BodyTypeDynamic
		if i%5 < 2 {
			bodyType = actor.BodyTypeStatic
		}
		body := addBody(w, position, rotation, shape, bodyType, 0.5, 0)
		switch i % 7 {
		case 1:
			body.Layer = layerDecor
		case 2:
			body.Layer = layerPawn
		case 3:
			// queries only
			body.Layer, body.Mask = layerDecor, actor.NoLayers
		case 4:
			body.IsTrigger = true
		}
		if bodyType == actor.BodyTypeDynamic && i%2 == 0 {
			body.Sleep()
		}
		if i == 100 || i == 200 {
			// a twin at the same place
			twin := addBody(w, position, rotation, shape, bodyType, 0.5, 0)
			twin.Layer, twin.Mask, twin.IsTrigger, twin.IsSleeping = body.Layer, body.Mask, body.IsTrigger, body.IsSleeping
		}
	}
	w.SyncQueries()
	return w
}

// queryTerrain: hills of 33x33 samples every meter, with a few holes
func queryTerrain() *actor.Heightfield {
	const samples = 33
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32(1.5*math.Sin(float64(x)*0.4)*math.Cos(float64(z)*0.3) + 0.3*math.Sin(float64(x+z)))
		}
	}
	field := actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{1, 1, 1})
	field.Holes = make([]bool, (samples-1)*(samples-1))
	for i := range field.Holes {
		field.Holes[i] = i%23 == 0
	}
	field.Update(0, 0, samples-1, samples-1)
	return field
}

// randomRay through the scene: any direction, along an axis, along an axis in the plane of a face of the AABB of a
// body, from the center of a body, or without length
func randomRay(r *rand.Rand, w *World, i int) (mgl64.Vec3, mgl64.Vec3) {
	point := func() mgl64.Vec3 {
		return mgl64.Vec3{(r.Float64()*2.4 - 1.2) * queryExtent, (r.Float64()*2.4 - 1.2) * queryExtent, (r.Float64()*2.4 - 1.2) * queryExtent}
	}
	origin, target := point(), point()
	body := w.Bodies[2+r.Intn(len(w.Bodies)-2)]
	switch i % 10 {
	case 1, 2:
		// along an axis
		axis := r.Intn(3)
		target = origin
		target[axis] = point()[axis]
	case 3, 4:
		// along an axis, in the plane of a face of the AABB of a body (the face of the body, for a box not turned)
		aabb := body.AABB()
		axis, face := r.Intn(3), r.Intn(3)
		origin = aabb.Min.Add(aabb.Max).Mul(0.5)
		if face != axis {
			origin[face] = aabb.Max[face]
			if r.Intn(2) == 0 {
				origin[face] = aabb.Min[face]
			}
		}
		target = origin
		origin[axis] -= 2 + 10*r.Float64()
		target[axis] += 2 + 10*r.Float64()
		if r.Intn(2) == 0 {
			origin, target = target, origin
		}
	case 5:
		origin = body.Transform.Position
	case 6:
		// a point
		if r.Intn(2) == 0 {
			origin = body.Transform.Position
		}
		target = origin
	}
	return origin, target.Sub(origin)
}

// randomFilter: all the layers most of the time, else a few layers, with the triggers or not, without a few bodies
func randomFilter(r *rand.Rand, w *World, i int) QueryFilter {
	filter := DefaultQueryFilter()
	switch i % 6 {
	case 1:
		filter.Triggers = true
	case 2:
		filter.Mask = layerDecor | layerPawn
	case 3:
		filter.Mask, filter.Triggers = actor.LayerDefault, true
	case 4:
		filter.Excluded = []*actor.RigidBody{w.Bodies[r.Intn(len(w.Bodies))], w.Bodies[r.Intn(len(w.Bodies))]}
	}
	return filter
}

// bruteForceRaycast: the first body on the ray, by a walk of World.Bodies in their order
func bruteForceRaycast(w *World, origin, translation mgl64.Vec3, filter QueryFilter) (*actor.RigidBody, actor.RayHit, bool) {
	var first *actor.RigidBody
	best := actor.RayHit{Fraction: math.Inf(1)}
	for _, body := range w.Bodies {
		if !filter.Accepts(body) {
			continue
		}
		if hit, ok := castRay(body, origin, translation, 1); ok && hit.Fraction < best.Fraction {
			first, best = body, hit
		}
	}
	return first, best, first != nil
}

// ========== RAY ==========

// Criterion 5: through the world, with any transform, the fraction, the point and the normal of a ray are the analytic
// ones
func TestRaycastIsAnalyticThroughTheWorld(t *testing.T) {
	tilted := mgl64.Vec3{1, 2, -0.5}.Normalize()
	capEntry := -math.Sqrt(0.3*0.3 - 0.2*0.2 - 0.1*0.1)
	// in the local space of the shape
	cases := []struct {
		name                string
		shape               actor.ShapeInterface
		origin, translation mgl64.Vec3
		fraction            float64
		normal              mgl64.Vec3
	}{
		{"sphere", &actor.Sphere{Radius: 0.5}, mgl64.Vec3{0, 4, 0}, mgl64.Vec3{0, -7, 0}, 3.5 / 7, mgl64.Vec3{0, 1, 0}},
		{"box", &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.25, 1}}, mgl64.Vec3{5, 0.1, -0.2}, mgl64.Vec3{-9, 0.2, 0.4}, 4.5 / 9, mgl64.Vec3{1, 0, 0}},
		{"capsule side", &actor.Capsule{HalfHeight: 0.6, Radius: 0.3}, mgl64.Vec3{2, 0.4, 0}, mgl64.Vec3{-4, 0.2, 0}, 1.7 / 4, mgl64.Vec3{1, 0, 0}},
		{"capsule cap", &actor.Capsule{HalfHeight: 0.6, Radius: 0.3}, mgl64.Vec3{-2, 0.8, 0.1}, mgl64.Vec3{5, 0, 0}, (2 + capEntry) / 5, mgl64.Vec3{capEntry, 0.2, 0.1}.Mul(1 / 0.3)},
	}
	r := rand.New(rand.NewSource(9))
	for _, c := range cases {
		for i := 0; i < 20; i++ {
			transform := actor.Transform{Position: mgl64.Vec3{r.Float64()*20 - 10, r.Float64()*20 - 10, r.Float64()*20 - 10}, Rotation: randomRotation(r)}
			w := newScene(1)
			body := addBody(w, transform.Position, transform.Rotation, c.shape, actor.BodyTypeStatic, 0.5, 0)
			w.SyncQueries()
			origin, translation := transform.ToWorld(c.origin), transform.Rotation.Rotate(c.translation)
			hit, ok := w.Raycast(origin, translation, DefaultQueryFilter())
			point, normal := origin.Add(translation.Mul(c.fraction)), transform.Rotation.Rotate(c.normal)
			if !ok || hit.Body != body || math.Abs(hit.Fraction-c.fraction) > 1e-9 || !closeTo(hit.Point, point, 1e-9) || !closeTo(hit.Normal, normal, 1e-9) || hit.Triangle != actor.NoTriangle {
				t.Errorf("%s at %v: hit %v fraction %.12f point %v normal %v, want %.12f %v %v", c.name, transform, ok, hit.Fraction, hit.Point, hit.Normal, c.fraction, point, normal)
			}
		}
	}

	// a plane is in world space: the transform of its body doesn't move it
	w := newScene(1)
	plane := addBody(w, mgl64.Vec3{5, 9, -3}, randomRotation(r), &actor.Plane{Normal: tilted, Distance: -1.5}, actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	hit, ok := w.Raycast(tilted.Mul(4), tilted.Mul(-5), DefaultQueryFilter())
	if !ok || hit.Body != plane || math.Abs(hit.Fraction-0.5) > 1e-9 || !closeTo(hit.Point, tilted.Mul(1.5), 1e-9) || !closeTo(hit.Normal, tilted, 1e-9) {
		t.Errorf("plane: hit %v fraction %.12f point %v normal %v", ok, hit.Fraction, hit.Point, hit.Normal)
	}
}

// Criterion 7: a ray which starts in a body hits it at its origin, the normal against its direction. A ray without
// length is a point query
func TestRaycastFromInsideABody(t *testing.T) {
	r := rand.New(rand.NewSource(10))
	shapes := []actor.ShapeInterface{&actor.Sphere{Radius: 0.5}, &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.25, 1}}, &actor.Capsule{HalfHeight: 0.6, Radius: 0.3}}
	for _, shape := range shapes {
		w := newScene(1)
		body := addBody(w, mgl64.Vec3{3, -2, 5}, randomRotation(r), shape, actor.BodyTypeDynamic, 0.5, 0)
		w.SyncQueries()
		inside, outside := body.Transform.ToWorld(mgl64.Vec3{0.1, 0.05, -0.1}), body.Transform.ToWorld(mgl64.Vec3{3, 3, 3})
		direction := mgl64.Vec3{2, -5, 1}
		hit, ok := w.Raycast(inside, direction, DefaultQueryFilter())
		if !ok || hit.Body != body || hit.Fraction != 0 || hit.Point != inside || !closeTo(hit.Normal, direction.Normalize().Mul(-1), 1e-12) {
			t.Errorf("%T, from inside: hit %v fraction %g point %v normal %v", shape, ok, hit.Fraction, hit.Point, hit.Normal)
		}
		hit, ok = w.Raycast(inside, mgl64.Vec3{}, DefaultQueryFilter())
		if !ok || hit.Body != body || hit.Fraction != 0 || hit.Point != inside || hit.Normal != (mgl64.Vec3{}) {
			t.Errorf("%T, a point inside: hit %v fraction %g point %v normal %v", shape, ok, hit.Fraction, hit.Point, hit.Normal)
		}
		if _, ok := w.Raycast(outside, mgl64.Vec3{}, DefaultQueryFilter()); ok {
			t.Errorf("%T: a point outside hits the body", shape)
		}
	}
	w := newScene(1)
	ground := addGround(w, 0.5)
	w.SyncQueries()
	under := mgl64.Vec3{4, -3, 1}
	hit, ok := w.Raycast(under, mgl64.Vec3{1, 1, 0}, DefaultQueryFilter())
	if !ok || hit.Body != ground || hit.Fraction != 0 || hit.Point != under || !closeTo(hit.Normal, mgl64.Vec3{-1, -1, 0}.Normalize(), 1e-12) {
		t.Errorf("under a plane: hit %v fraction %g point %v normal %v", ok, hit.Fraction, hit.Point, hit.Normal)
	}
	if hit, ok := w.Raycast(under, mgl64.Vec3{}, DefaultQueryFilter()); !ok || hit.Normal != (mgl64.Vec3{}) {
		t.Errorf("a point under a plane: hit %v normal %v", ok, hit.Normal)
	}
}

// Criterion 8: the terrain is moved and turned by the transform of its body
func TestRaycastOnAMovedTerrain(t *testing.T) {
	w := newScene(1)
	field := queryTerrain()
	terrain := addBody(w, mgl64.Vec3{30, -4, 12}, mgl64.QuatRotate(0.7, mgl64.Vec3{1, 0.4, -0.3}.Normalize()), field, actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	r := rand.New(rand.NewSource(12))
	up := terrain.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0})
	holes := 0
	for i := 0; i < 500; i++ {
		x, z := r.Float64()*32-16, r.Float64()*32-16
		height, solid := field.HeightAt(x, z)
		origin := terrain.Transform.ToWorld(mgl64.Vec3{x, 10, z})
		hit, ok := w.Raycast(origin, up.Mul(-20), DefaultQueryFilter())
		// a hole within a billionth of a cell is not told from its edge
		if !solid {
			holes++
			continue
		}
		point := terrain.Transform.ToWorld(mgl64.Vec3{x, height, z})
		if !ok || hit.Body != terrain || !closeTo(hit.Point, point, 1e-9) {
			t.Fatalf("ray %d at (%.3f, %.3f): hit %v at %v, the terrain is at %v", i, x, z, ok, hit.Point, point)
		}
		cell := int(hit.Triangle) / 2
		vertices, _ := field.Triangle(cell/(field.ZSamples-1), cell%(field.ZSamples-1), int(hit.Triangle)%2)
		normal := terrain.Transform.Rotation.Rotate(vertices[1].Sub(vertices[0]).Cross(vertices[2].Sub(vertices[0])).Normalize())
		if !closeTo(hit.Normal, normal, 1e-9) || math.Abs(terrain.Transform.ToLocal(hit.Point).Sub(vertices[0]).Dot(terrain.Transform.Rotation.Conjugate().Rotate(normal))) > 1e-9 {
			t.Fatalf("ray %d: normal %v, the triangle %d has %v", i, hit.Normal, hit.Triangle, normal)
		}
		// from below, the ray goes through
		if _, ok := w.Raycast(terrain.Transform.ToWorld(mgl64.Vec3{x, -10, z}), up.Mul(20), DefaultQueryFilter()); ok {
			t.Fatalf("ray %d from below hits the terrain", i)
		}
	}
	if holes == 0 || holes > 100 {
		t.Errorf("%d rays in a hole, the rays don't cover the holes and the triangles", holes)
	}
}

// Criterion 9: Raycast gives the body and the fraction of a walk of World.Bodies in their order, on 300 bodies of
// every kind and 5000 rays, some along the axes and in the planes of the faces. Of 2 bodies at the same place, the
// first one is hit
func TestRaycastIsTheBruteForce(t *testing.T) {
	for _, workers := range []int{1, 8} {
		w := queryScene(workers)
		w.parallelFrom = 1
		for phase := 0; phase < 2; phase++ {
			r := rand.New(rand.NewSource(int64(22 + phase)))
			hits, twins := 0, 0
			for i := 0; i < 5000; i++ {
				origin, translation := randomRay(r, w, i)
				filter := randomFilter(r, w, i)
				body, want, found := bruteForceRaycast(w, origin, translation, filter)
				hit, ok := w.Raycast(origin, translation, filter)
				if ok != found || hit.Body != body || (ok && math.Abs(hit.Fraction-want.Fraction) > 1e-12) {
					t.Fatalf("workers %d, phase %d, ray %d from %v along %v: hit %v on the body %d at %.15f, the brute force has %v on the body %d at %.15f",
						workers, phase, i, origin, translation, ok, slices.Index(w.Bodies, hit.Body), hit.Fraction, found, slices.Index(w.Bodies, body), want.Fraction)
				}
				if !ok {
					continue
				}
				hits++
				if !closeTo(hit.Normal, want.Normal, 1e-12) || hit.Triangle != want.Triangle || hit.Point != origin.Add(translation.Mul(hit.Fraction)) {
					t.Fatalf("workers %d, phase %d, ray %d: normal %v triangle %d point %v, the brute force has %v and %d", workers, phase, i, hit.Normal, hit.Triangle, hit.Point, want.Normal, want.Triangle)
				}
				if index := slices.Index(w.Bodies, hit.Body); index+1 < len(w.Bodies) && w.Bodies[index+1].Transform == hit.Body.Transform {
					twins++
				}
			}
			if hits < 2000 || twins == 0 {
				t.Errorf("workers %d, phase %d: %d hits, %d on a body with a twin: the rays don't cover the scene", workers, phase, hits, twins)
			}
			// the awake bodies drift and push each other: the steps keep the trees up to date
			for _, body := range w.Bodies {
				if isAwakeDynamic(body) {
					body.Velocity = mgl64.Vec3{1, -2, 0.5}
				}
			}
			simulate(w, 0.2, nil)
		}
	}
}

// Criterion 24: the queries give the same results with 1 and 8 workers
func TestQueriesDoNotDependOnTheWorkers(t *testing.T) {
	single, parallel := queryScene(1), queryScene(8)
	parallel.parallelFrom = 1
	for round := 0; round < 3; round++ {
		r := rand.New(rand.NewSource(31))
		for i := 0; i < 2000; i++ {
			origin, translation := randomRay(r, single, i)
			a, okA := single.Raycast(origin, translation, DefaultQueryFilter())
			b, okB := parallel.Raycast(origin, translation, DefaultQueryFilter())
			if okA != okB || slices.Index(single.Bodies, a.Body) != slices.Index(parallel.Bodies, b.Body) || a.Fraction != b.Fraction || a.Normal != b.Normal || a.Triangle != b.Triangle {
				t.Fatalf("round %d, ray %d: %v %+v with 1 worker, %v %+v with 8", round, i, okA, a, okB, b)
			}
			if i%4 > 0 {
				continue
			}
			shape, start, motion := randomMover(r, single, i)
			a, okA = single.Sweep(shape, start, motion, DefaultQueryFilter())
			b, okB = parallel.Sweep(shape, start, motion, DefaultQueryFilter())
			if okA != okB || slices.Index(single.Bodies, a.Body) != slices.Index(parallel.Bodies, b.Body) || a.Fraction != b.Fraction || a.Normal != b.Normal || a.Point != b.Point {
				t.Fatalf("round %d, sweep %d: %v %+v with 1 worker, %v %+v with 8", round, i, okA, a, okB, b)
			}
		}
		for _, w := range []*World{single, parallel} {
			for _, body := range w.Bodies {
				if isAwakeDynamic(body) {
					body.Velocity = mgl64.Vec3{1, -2, 0.5}
				}
			}
			simulate(w, 0.1, nil)
		}
	}
}

// Criterion 10: the filter of a query: its mask, its excluded bodies, the triggers, the layer 0, the bodies "queries
// only"
func TestRaycastFilter(t *testing.T) {
	w := newScene(1)
	// 5 walls in a row along x, the ray goes through them all
	wall := func(x float64) *actor.RigidBody {
		return addBody(w, mgl64.Vec3{x, 0, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 1, 1}}, actor.BodyTypeStatic, 0.5, 0)
	}
	trigger, nowhere, ghost, pawn, decor := wall(1), wall(2), wall(3), wall(4), wall(5)
	trigger.IsTrigger = true
	nowhere.Layer = 0
	ghost.Layer, ghost.Mask = layerDecor, actor.NoLayers
	pawn.Layer = layerPawn
	decor.Layer = layerDecor
	w.SyncQueries()

	cases := []struct {
		name   string
		filter QueryFilter
		first  *actor.RigidBody
		all    []*actor.RigidBody
	}{
		{"the default filter", DefaultQueryFilter(), ghost, []*actor.RigidBody{ghost, pawn, decor}},
		{"the zero filter", QueryFilter{}, nil, nil},
		{"with the triggers", QueryFilter{Mask: actor.AllLayers, Triggers: true}, trigger, []*actor.RigidBody{trigger, ghost, pawn, decor}},
		{"a layer", QueryFilter{Mask: layerPawn}, pawn, []*actor.RigidBody{pawn}},
		{"the layer of the triggers, without them", QueryFilter{Mask: actor.LayerDefault}, nil, nil},
		{"the layer of the triggers, with them", QueryFilter{Mask: actor.LayerDefault, Triggers: true}, trigger, []*actor.RigidBody{trigger}},
		{"the layer of a body queries only", QueryFilter{Mask: layerDecor}, ghost, []*actor.RigidBody{ghost, decor}},
		{"an excluded body", QueryFilter{Mask: actor.AllLayers, Excluded: []*actor.RigidBody{ghost}}, pawn, []*actor.RigidBody{pawn, decor}},
		{"excluded bodies", QueryFilter{Mask: actor.AllLayers, Triggers: true, Excluded: []*actor.RigidBody{trigger, ghost, decor}}, pawn, []*actor.RigidBody{pawn}},
	}
	origin, translation := mgl64.Vec3{0, 0.2, 0.3}, mgl64.Vec3{10, 0, 0}
	var hits []Hit
	for _, c := range cases {
		hit, ok := w.Raycast(origin, translation, c.filter)
		if ok != (c.first != nil) || hit.Body != c.first {
			t.Errorf("%s: the ray hits the wall %d (%v), want the wall %d", c.name, slices.Index(w.Bodies, hit.Body)+1, ok, slices.Index(w.Bodies, c.first)+1)
		}
		hits = w.RaycastAll(origin, translation, c.filter, hits[:0])
		var bodies []*actor.RigidBody
		for _, hit := range hits {
			bodies = append(bodies, hit.Body)
		}
		if !slices.Equal(bodies, c.all) {
			t.Errorf("%s: RaycastAll gives %d walls, want %d", c.name, len(bodies), len(c.all))
		}
	}
	if DefaultQueryFilter().Mask != actor.AllLayers || DefaultQueryFilter().Triggers || DefaultQueryFilter().Excluded != nil {
		t.Errorf("the default filter is %+v, want every layer and no trigger", DefaultQueryFilter())
	}
}

// sleepingPile: a pile of boxes asleep on the ground, and the count of the events it sent
func sleepingPile(t *testing.T) (*World, *int) {
	t.Helper()
	w := benchScene(40, 1)
	simulate(w, 4, nil)
	for i, body := range w.Bodies[1:] {
		if !body.IsSleeping {
			t.Fatalf("the body %d of the pile is still awake", i+1)
		}
	}
	events := new(int)
	for eventType := EventTriggerEnter; eventType <= EventWake; eventType++ {
		w.Events.Subscribe(eventType, func(Event) { *events++ })
	}
	return w, events
}

// mixedQuery i of a batch: a query among every kind, its result folded in a number
func mixedQuery(w *World, r *rand.Rand, i int, hits []Hit) (uint64, []Hit) {
	origin, translation := randomRay(r, w, i)
	if i%3 == 1 {
		shape, start, translation := randomMover(r, w, i)
		hit, ok := w.Sweep(shape, start, translation, DefaultQueryFilter())
		if !ok {
			return 0, hits
		}
		return math.Float64bits(hit.Fraction) + math.Float64bits(hit.Point.X()) + uint64(slices.Index(w.Bodies, hit.Body)), hits
	}
	if i%6 == 5 {
		shape, at, _ := randomMover(r, w, i)
		bodies := w.Overlap(shape, at, DefaultQueryFilter(), nil)
		sum := uint64(len(bodies))
		for _, body := range bodies {
			sum = sum*31 + uint64(slices.Index(w.Bodies, body))
		}
		return sum, hits
	}
	if i%3 == 0 {
		hits = w.RaycastAll(origin, translation, DefaultQueryFilter(), hits[:0])
		sum := uint64(len(hits))
		for _, hit := range hits {
			sum = sum*31 + math.Float64bits(hit.Fraction) + uint64(slices.Index(w.Bodies, hit.Body))
		}
		return sum, hits
	}
	hit, ok := w.Raycast(origin, translation, DefaultQueryFilter())
	if !ok {
		return 0, hits
	}
	return math.Float64bits(hit.Fraction) + uint64(slices.Index(w.Bodies, hit.Body)), hits
}

// Criterion 11: 10000 queries on a sleeping pile wake nobody up and send no event, and the next step is the step of a
// world without queries, to the bit
func TestQueriesWriteNothing(t *testing.T) {
	queried, events := sleepingPile(t)
	plain, _ := sleepingPile(t)
	r := rand.New(rand.NewSource(41))
	var hits []Hit
	found := uint64(0)
	for i := 0; i < 10000; i++ {
		var result uint64
		result, hits = mixedQuery(queried, r, i, hits)
		found += result
	}
	if found == 0 {
		t.Fatal("no query hits the pile")
	}
	for step := 0; step < 2; step++ {
		for i, body := range queried.Bodies[1:] {
			if !body.IsSleeping {
				t.Fatalf("step %d: the body %d woke up", step, i+1)
			}
		}
		if *events != 0 {
			t.Fatalf("step %d: %d events sent", step, *events)
		}
		for i := range queried.Bodies {
			a, b := queried.Bodies[i], plain.Bodies[i]
			if a.Transform != b.Transform || a.Velocity != b.Velocity || a.AngularVelocity != b.AngularVelocity || a.SleepTimer != b.SleepTimer || a.AABB() != b.AABB() {
				t.Fatalf("step %d: the body %d differs from the world without queries", step, i)
			}
		}
		if !snapshotTree(queried).equal(snapshotTree(plain)) {
			t.Fatalf("step %d: the trees differ from the world without queries", step)
		}
		queried.Step(sceneDt)
		plain.Step(sceneDt)
	}
}

// Criterion 12: RaycastAll gives one hit per body, the first one, sorted by fraction then by index, in the buffer of
// the caller
func TestRaycastAll(t *testing.T) {
	w := queryScene(1)
	r := rand.New(rand.NewSource(51))
	hits := make([]Hit, 0, 64)
	total := 0
	for i := 0; i < 2000; i++ {
		origin, translation := randomRay(r, w, i)
		filter := randomFilter(r, w, i)
		var want []Hit
		for index, body := range w.Bodies {
			if !filter.Accepts(body) {
				continue
			}
			if hit, ok := castRay(body, origin, translation, 1); ok {
				want = append(want, Hit{Body: body, Fraction: hit.Fraction, Normal: hit.Normal, Triangle: hit.Triangle, index: int32(index)})
			}
		}
		slices.SortStableFunc(want, func(a, b Hit) int {
			switch {
			case a.Fraction < b.Fraction:
				return -1
			case a.Fraction > b.Fraction:
				return 1
			}
			return 0
		})
		// the hits follow what the buffer holds, which is not sorted with them
		hits = append(hits[:0], Hit{Fraction: 2})
		hits = w.RaycastAll(origin, translation, filter, hits)
		if hits[0].Fraction != 2 || len(hits)-1 != len(want) {
			t.Fatalf("ray %d: %d hits after the one of the buffer, want %d", i, len(hits)-1, len(want))
		}
		for k, hit := range hits[1:] {
			if hit.Body != want[k].Body || hit.Fraction != want[k].Fraction || hit.Normal != want[k].Normal || hit.Triangle != want[k].Triangle || hit.Point != origin.Add(translation.Mul(hit.Fraction)) {
				t.Fatalf("ray %d, hit %d: the body %d at %.15f, want the body %d at %.15f", i, k, slices.Index(w.Bodies, hit.Body), hit.Fraction, want[k].index, want[k].Fraction)
			}
		}
		total += len(want)
		if first, ok := w.Raycast(origin, translation, filter); ok != (len(want) > 0) || (ok && (first.Body != want[0].Body || first.Fraction != want[0].Fraction)) {
			t.Fatalf("ray %d: Raycast doesn't give the first hit of RaycastAll", i)
		}
	}
	if total < 1000 {
		t.Errorf("%d hits for 2000 rays: the rays don't cross several bodies", total)
	}
}

// Criterion 13: a ray with a NaN or an infinite coordinate hits nothing, and doesn't panic
func TestRaycastNotFinite(t *testing.T) {
	w := queryScene(1)
	var hits []Hit
	for _, value := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for k := 0; k < 3; k++ {
			for _, inOrigin := range []bool{true, false} {
				origin, translation := mgl64.Vec3{0, 5, 0}, mgl64.Vec3{1, -30, 2}
				if inOrigin {
					origin[k] = value
				} else {
					translation[k] = value
				}
				if _, ok := w.Raycast(origin, translation, DefaultQueryFilter()); ok {
					t.Errorf("a hit from %v along %v", origin, translation)
				}
				if hits = w.RaycastAll(origin, translation, DefaultQueryFilter(), hits[:0]); len(hits) > 0 {
					t.Errorf("%d hits from %v along %v", len(hits), origin, translation)
				}
			}
		}
	}
	if _, ok := w.Raycast(mgl64.Vec3{0, 5, 0}, mgl64.Vec3{1, -30, 2}, DefaultQueryFilter()); !ok {
		t.Error("the finite ray hits nothing")
	}
}

// A corner of the pruning: a ray whose translation along an axis is too small to be inverted, from the plane of a
// face of the stored AABB of a body or of the body itself, gives the hit of the body alone (0 x infinity is a NaN)
func TestRaycastTinyTranslation(t *testing.T) {
	w := newScene(1)
	box := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	stored := w.tree.proxies[0].aabb
	hits := 0
	for _, tiny := range []float64{5e-324, -5e-324, 1e-310, -1e-310} {
		for _, face := range []float64{stored.Min[0], stored.Max[0], -cubeHalf, cubeHalf} {
			origin, translation := mgl64.Vec3{face, 3, 0}, mgl64.Vec3{tiny, -6, 0}
			// from a face of the stored AABB towards its inside: the ray is in the AABB all the way
			ray := newTreeRay(origin, translation, mgl64.Vec3{})
			if _, ok := ray.enters(&stored, 1); !ok && math.Abs(face) > cubeHalf && face*tiny < 0 {
				t.Errorf("the ray from x = %g, moving by %g along x, misses the stored AABB", face, tiny)
			}
			want, found := castRay(box, origin, translation, 1)
			hit, ok := w.Raycast(origin, translation, DefaultQueryFilter())
			if ok != found || (ok && hit.Fraction != want.Fraction) {
				t.Errorf("the ray from x = %g, moving by %g along x: hit %v, the body alone gives %v", face, tiny, ok, found)
			}
			if ok {
				hits++
			}
		}
	}
	if hits != 4 {
		t.Errorf("%d hits, want the 4 rays along a face of the box, moving towards it", hits)
	}
}

// A tree higher than the stack of its traversal loses no hit
func TestTreeCastHigherThanItsStack(t *testing.T) {
	w := newScene(1)
	const count = 4 * treeStackSize
	for i := 0; i < count; i++ {
		addBody(w, mgl64.Vec3{float64(i), 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeStatic, 0.5, 0)
	}
	w.SyncQueries()
	// the worst tree: a chain, each node a leaf and the rest of the chain
	tree := &w.tree.statics
	tree.clear()
	for i := 0; i < count; i++ {
		leaf := tree.allocate()
		tree.nodes[leaf].aabb, tree.nodes[leaf].body = w.tree.proxies[i].aabb, int32(i)
		w.tree.proxies[i].node = leaf
		if i == 0 {
			tree.root = leaf
			continue
		}
		pair := tree.allocate()
		tree.link(pair, leaf, tree.root)
		tree.root = pair
	}
	if tree.height() != count-1 {
		t.Fatalf("the chain is %d high, want %d", tree.height(), count-1)
	}
	for _, reversed := range []bool{false, true} {
		origin, translation := mgl64.Vec3{-5, 0.1, 0}, mgl64.Vec3{count + 10, 0, 0}
		first := 0
		if reversed {
			origin, translation = origin.Add(translation), translation.Mul(-1)
			first = count - 1
		}
		hits := w.RaycastAll(origin, translation, DefaultQueryFilter(), nil)
		if len(hits) != count {
			t.Fatalf("reversed %v: %d hits through the chain, want %d", reversed, len(hits), count)
		}
		for k := 1; k < len(hits); k++ {
			if hits[k].Fraction < hits[k-1].Fraction {
				t.Fatalf("reversed %v: the hits are not sorted", reversed)
			}
		}
		if hit, ok := w.Raycast(origin, translation, DefaultQueryFilter()); !ok || hit.Body != w.Bodies[first] {
			t.Errorf("reversed %v: the first body of the chain is not hit", reversed)
		}
	}
}

// ========== CONCURRENCY, ALLOCATIONS ==========

// Criterion 21: no allocation for a ray, nor for all its hits in a buffer large enough
func TestRaycastDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	w := queryScene(1)
	r := rand.New(rand.NewSource(61))
	const rays = 200
	origins, translations := make([]mgl64.Vec3, rays), make([]mgl64.Vec3, rays)
	for i := range origins {
		origins[i], translations[i] = randomRay(r, w, i)
	}
	filter := DefaultQueryFilter()
	filter.Excluded = []*actor.RigidBody{w.Bodies[5]}
	hits := make([]Hit, 0, 512)
	found := 0
	allocs := testing.AllocsPerRun(10, func() {
		for i := range origins {
			if _, ok := w.Raycast(origins[i], translations[i], filter); ok {
				found++
			}
			hits = w.RaycastAll(origins[i], translations[i], filter, hits[:0])
			found += len(hits)
		}
	})
	if allocs > 0 || found == 0 {
		t.Errorf("%.1f allocations for %d rays (%d hits), want 0", allocs, 2*rays, found)
	}
}

// Criterion 22: 8 goroutines run 10000 queries each at the same time: no race, and the results of a goroutine alone
func TestQueriesFromManyGoroutines(t *testing.T) {
	w := queryScene(1)
	const goroutines, queries = 8, 10000
	batch := func(seed int64) []uint64 {
		r := rand.New(rand.NewSource(seed))
		results := make([]uint64, queries)
		var hits []Hit
		for i := range results {
			results[i], hits = mixedQuery(w, r, i, hits)
		}
		return results
	}
	want := make([][]uint64, goroutines)
	for g := range want {
		want[g] = batch(int64(70 + g))
	}
	got := make([][]uint64, goroutines)
	var wait sync.WaitGroup
	for g := range got {
		wait.Add(1)
		go func() {
			defer wait.Done()
			got[g] = batch(int64(70 + g))
		}()
	}
	wait.Wait()
	for g := range got {
		if !slices.Equal(got[g], want[g]) {
			t.Errorf("the goroutine %d has other results than alone", g)
		}
	}
}

// Criterion 23: the guard is up while a step runs, and down after it
func TestGuardIsUpDuringAStep(t *testing.T) {
	w := benchScene(300, 1)
	done := make(chan struct{})
	go func() {
		defer close(done)
		simulate(w, 1, nil)
	}()
	up := false
	for running := true; running; {
		select {
		case <-done:
			running = false
		default:
			up = up || w.stepping.Load()
		}
	}
	if !up {
		t.Error("the guard was never seen up during 60 steps of 300 bodies")
	}
	if w.stepping.Load() {
		t.Error("the guard is still up after the steps")
	}
}

// Criterion 23: a query during a step panics; in a listener of an event, it sees the end of the step
func TestRaycastDuringStepPanics(t *testing.T) {
	w := brokenPile(1)
	w.Step(sceneDt)
	origin, translation := mgl64.Vec3{0, 5, 0}, mgl64.Vec3{0, -10, 0}
	queries := map[string]func(){
		"Raycast":    func() { w.Raycast(origin, translation, DefaultQueryFilter()) },
		"RaycastAll": func() { w.RaycastAll(origin, translation, DefaultQueryFilter(), nil) },
	}
	for name, query := range queries {
		if message := panicMessage(query); message != "" {
			t.Errorf("%s between 2 steps panics: %s", name, message)
		}
		w.stepping.Store(true)
		if message := panicMessage(query); message != queryDuringStep {
			t.Errorf("%s during a step: panic %q, want %q", name, message, queryDuringStep)
		}
		w.stepping.Store(false)
	}

	// in a listener, the rays find the bodies where the step left them
	calls := 0
	r := rand.New(rand.NewSource(81))
	w.Events.Subscribe(EventCollisionEnter, func(Event) {
		calls++
		for i := 0; i < 50; i++ {
			origin := mgl64.Vec3{r.Float64()*8 - 2, 0.1 + r.Float64(), r.Float64() - 0.5}
			translation := mgl64.Vec3{r.Float64()*8 - 2, 0.1, r.Float64() - 0.5}.Sub(origin)
			body, want, found := bruteForceRaycast(w, origin, translation, DefaultQueryFilter())
			hit, ok := w.Raycast(origin, translation, DefaultQueryFilter())
			if ok != found || hit.Body != body || (ok && hit.Fraction != want.Fraction) {
				t.Fatalf("in a listener, a ray hits %v at %g, the bodies of the end of the step give %v at %g", ok, hit.Fraction, found, want.Fraction)
			}
		}
	})
	simulate(w, 0.5, nil)
	if calls == 0 {
		t.Fatal("no collision event: the listener never ran")
	}
}
