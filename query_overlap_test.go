package feather

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== OVERLAP ==========

// exactGap between the shape at the transform and the body, false if the test has no exact measure of this pair (2
// boxes, a box and a terrain)
func exactGap(shape actor.ShapeInterface, at actor.Transform, body *actor.RigidBody) (float64, bool) {
	if _, isBox := shape.(*actor.Box); !isBox {
		return gapBetween(shape, at, body), true
	}
	switch body.Shape.(type) {
	case *actor.Plane:
		return gapBetween(shape, at, body), true
	case *actor.Sphere, *actor.Capsule:
		// the body against the box
		return gapBetween(body.Shape, body.Transform, actor.NewRigidBody(at, shape, actor.BodyTypeStatic, 0)), true
	}
	return 0, false
}

// Criterion 20: Overlap gives the bodies of a walk of World.Bodies, in their order, with the filters
func TestOverlapIsTheBruteForce(t *testing.T) {
	for _, workers := range []int{1, 8} {
		w := queryScene(workers)
		w.parallelFrom = 1
		for phase := 0; phase < 2; phase++ {
			r := rand.New(rand.NewSource(int64(91 + phase)))
			bodies := make([]*actor.RigidBody, 0, 64)
			total, onTerrain, onPlane := 0, 0, 0
			for i := 0; i < 500; i++ {
				var shape actor.ShapeInterface
				switch i % 3 {
				case 0:
					shape = &actor.Sphere{Radius: 0.2 + 2*r.Float64()}
				case 1:
					shape = &actor.Capsule{HalfHeight: 0.2 + r.Float64(), Radius: 0.2 + r.Float64()}
				default:
					shape = &actor.Box{HalfExtents: mgl64.Vec3{0.2 + r.Float64(), 0.2 + r.Float64(), 0.2 + r.Float64()}}
				}
				origin, _ := randomRay(r, w, i)
				if i%10 == 7 {
					// down to the plane
					origin[1] = -queryExtent - 2 + r.Float64()
				}
				at := actor.Transform{Position: origin, Rotation: randomRotation(r)}
				filter := randomFilter(r, w, i)

				bodies = append(bodies[:0], w.Bodies[3])
				bodies = w.Overlap(shape, at, filter, bodies)
				if bodies[0] != w.Bodies[3] {
					t.Fatalf("workers %d, overlap %d: the body of the buffer is lost", workers, i)
				}
				found := bodies[1:]
				for k := 1; k < len(found); k++ {
					if slices.Index(w.Bodies, found[k-1]) >= slices.Index(w.Bodies, found[k]) {
						t.Fatalf("workers %d, overlap %d: the bodies are not in the order of World.Bodies", workers, i)
					}
				}
				for index, body := range w.Bodies {
					gap, exact := exactGap(shape, at, body)
					got := slices.Contains(found, body)
					if !filter.Accepts(body) {
						if got {
							t.Fatalf("workers %d, overlap %d: the body %d is given, the filter refuses it", workers, i, index)
						}
						continue
					}
					// on the very limit, the rounding decides
					if !exact || math.Abs(gap) < 1e-9 {
						continue
					}
					if got != (gap < 0) {
						t.Fatalf("workers %d, phase %d, overlap %d of %T at %v: the body %d (%T) given %v, its gap is %.3g m", workers, phase, i, shape, at.Position, index, body.Shape, got, gap)
					}
					if got {
						total++
						switch body.Shape.(type) {
						case *actor.Heightfield:
							onTerrain++
						case *actor.Plane:
							onPlane++
						}
					}
				}
			}
			if total < 200 || onTerrain < 10 || onPlane < 10 {
				t.Errorf("workers %d, phase %d: %d bodies overlapped, %d times the terrain, %d times the plane: the overlaps don't cover the scene", workers, phase, total, onTerrain, onPlane)
			}
			for _, body := range w.Bodies {
				if isAwakeDynamic(body) {
					body.Velocity = mgl64.Vec3{1, -2, 0.5}
				}
			}
			simulate(w, 0.2, nil)
		}
	}
}

// Criterion 20: a body in contact with the shape overlaps it; a micrometer away, it doesn't
func TestOverlapCountsTheContact(t *testing.T) {
	ball := &actor.Sphere{Radius: 0.5}
	cases := []struct {
		name   string
		target actor.ShapeInterface
		// the center of the ball in contact with the target at the origin, and the direction away from it
		touching, away mgl64.Vec3
	}{
		{"sphere", &actor.Sphere{Radius: 0.5}, mgl64.Vec3{1, 0, 0}, mgl64.Vec3{1, 0, 0}},
		{"box", &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{0, 1, 0}},
		{"capsule", &actor.Capsule{HalfHeight: 0.5, Radius: 0.25}, mgl64.Vec3{0.75, 0.25, 0}, mgl64.Vec3{1, 0, 0}},
		{"plane", &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, mgl64.Vec3{3, 0.5, 2}, mgl64.Vec3{0, 1, 0}},
	}
	for _, c := range cases {
		w := newScene(1)
		body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), c.target, actor.BodyTypeStatic, 0.5, 0)
		w.SyncQueries()
		at := func(offset float64) actor.Transform {
			return actor.Transform{Position: c.touching.Add(c.away.Mul(offset)), Rotation: mgl64.QuatIdent()}
		}
		for _, offset := range []float64{0, -1e-6, -0.3} {
			if bodies := w.Overlap(ball, at(offset), DefaultQueryFilter(), nil); len(bodies) != 1 || bodies[0] != body {
				t.Errorf("%s: %d bodies overlap the ball %g m in it, want the body", c.name, len(bodies), -offset)
			}
		}
		if bodies := w.Overlap(ball, at(1e-6), DefaultQueryFilter(), nil); len(bodies) != 0 {
			t.Errorf("%s: the body overlaps the ball a micrometer away", c.name)
		}
	}
	// 2 boxes face to face
	w := newScene(1)
	body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	for _, c := range []struct {
		height float64
		want   int
	}{{2 * cubeHalf, 1}, {2*cubeHalf - 1e-6, 1}, {2*cubeHalf + 1e-6, 0}} {
		bodies := w.Overlap(cube(), actor.Transform{Position: mgl64.Vec3{0.1, c.height, -0.2}, Rotation: mgl64.QuatIdent()}, DefaultQueryFilter(), nil)
		if len(bodies) != c.want || (c.want == 1 && bodies[0] != body) {
			t.Errorf("a box at the height %.7f over a box: %d bodies, want %d", c.height, len(bodies), c.want)
		}
	}
}

// Criterion 20: a sphere closer to the terrain than its radius overlaps it, from any side, but not over a hole
func TestOverlapOnATerrain(t *testing.T) {
	w := newScene(1)
	terrain := flatTerrain(w)
	field := terrain.Shape.(*actor.Heightfield)
	field.Holes = make([]bool, 16*16)
	for _, cell := range [][2]int{{7, 7}, {7, 8}, {8, 7}, {8, 8}} {
		field.Holes[cell[0]*16+cell[1]] = true
	}
	w.UpdateHeightfield(terrain, 6, 6, 10, 10)
	w.SyncQueries()
	ball := &actor.Sphere{Radius: 0.3}
	cases := []struct {
		name     string
		position mgl64.Vec3
		want     bool
	}{
		{"over the terrain, closer than its radius", mgl64.Vec3{3.3, 0.29, -2.1}, true},
		{"over the terrain, further than its radius", mgl64.Vec3{3.3, 0.31, -2.1}, false},
		{"under the terrain, closer than its radius", mgl64.Vec3{3.3, -0.29, -2.1}, true},
		{"under the terrain, further than its radius", mgl64.Vec3{3.3, -0.31, -2.1}, false},
		{"in the hole", mgl64.Vec3{0.1, 0.05, -0.2}, false},
		{"in the hole, on its edge", mgl64.Vec3{0.8, 0.05, 0.1}, true},
		{"beside the terrain", mgl64.Vec3{8.4, 0, 0}, false},
		{"on the border of the terrain", mgl64.Vec3{8.2, 0.1, 0}, true},
	}
	for _, c := range cases {
		bodies := w.Overlap(ball, actor.Transform{Position: c.position, Rotation: mgl64.QuatIdent()}, DefaultQueryFilter(), nil)
		if (len(bodies) == 1) != c.want {
			t.Errorf("a sphere %s: %d bodies, want %v", c.name, len(bodies), c.want)
		}
	}
	// a small sphere over each triangle of a cell (the cell from (3, 2) to (4, 3): its triangle 0 where z - 2 > x - 3)
	pea := &actor.Sphere{Radius: 0.1}
	for _, position := range []mgl64.Vec3{{3.2, 0.05, 2.8}, {3.8, 0.05, 2.2}, {3.2, -0.05, 2.8}, {3.8, -0.05, 2.2}} {
		if bodies := w.Overlap(pea, actor.Transform{Position: position, Rotation: mgl64.QuatIdent()}, DefaultQueryFilter(), nil); len(bodies) != 1 {
			t.Errorf("a small sphere at %v, 5 cm from the terrain: %d bodies, want the terrain", position, len(bodies))
		}
	}
}

// Criterion 20: the filter of an overlap
func TestOverlapFilter(t *testing.T) {
	w := newScene(1)
	at := func(x float64) *actor.RigidBody {
		return addBody(w, mgl64.Vec3{x, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, actor.BodyTypeDynamic, 0.5, 0)
	}
	trigger, nowhere, ghost, pawn, far := at(0), at(0.2), at(0.4), at(0.6), at(50)
	trigger.IsTrigger = true
	nowhere.Layer = 0
	ghost.Layer, ghost.Mask = layerDecor, actor.NoLayers
	ghost.BodyType = actor.BodyTypeStatic
	pawn.Layer = layerPawn
	pawn.Sleep()
	w.SyncQueries()
	cases := []struct {
		name   string
		filter QueryFilter
		want   []*actor.RigidBody
	}{
		{"the default filter", DefaultQueryFilter(), []*actor.RigidBody{ghost, pawn}},
		{"the zero filter", QueryFilter{}, nil},
		{"with the triggers", QueryFilter{Mask: actor.AllLayers, Triggers: true}, []*actor.RigidBody{trigger, ghost, pawn}},
		{"a layer", QueryFilter{Mask: layerDecor}, []*actor.RigidBody{ghost}},
		{"an excluded body", QueryFilter{Mask: actor.AllLayers, Excluded: []*actor.RigidBody{ghost}}, []*actor.RigidBody{pawn}},
	}
	for _, c := range cases {
		bodies := w.Overlap(&actor.Sphere{Radius: 1}, actor.Transform{Position: mgl64.Vec3{0.3, 0, 0}, Rotation: mgl64.QuatIdent()}, c.filter, nil)
		if !slices.Equal(bodies, c.want) {
			t.Errorf("%s: %d bodies, want %d", c.name, len(bodies), len(c.want))
		}
	}
	if !pawn.IsSleeping || slices.Contains(w.Overlap(&actor.Sphere{Radius: 1}, actor.NewTransform(), DefaultQueryFilter(), nil), far) {
		t.Error("the overlap woke a body up, or gives a body 50 m away")
	}
}

// Criteria 13, 21 & 23, for an overlap: no allocation in a buffer large enough, nothing for a transform which is not
// finite, a panic during a step and for a shape which is not convex
func TestOverlapDoesNotAllocateAndIsGuarded(t *testing.T) {
	w := queryScene(1)
	r := rand.New(rand.NewSource(92))
	const overlaps = 100
	shapes, transforms := make([]actor.ShapeInterface, overlaps), make([]actor.Transform, overlaps)
	for i := range shapes {
		shapes[i], transforms[i], _ = randomMover(r, w, i)
		if i%5 == 0 {
			shapes[i] = &actor.Box{HalfExtents: mgl64.Vec3{2, 2, 2}}
		}
		if i%7 == 0 {
			transforms[i].Position[1] = -queryExtent
		}
	}
	filter := DefaultQueryFilter()
	bodies := make([]*actor.RigidBody, 0, 256)
	found := 0
	run := func() {
		for i := range shapes {
			bodies = w.Overlap(shapes[i], transforms[i], filter, bodies[:0])
			found += len(bodies)
		}
	}
	if !raceEnabled {
		run()
		if allocs := testing.AllocsPerRun(10, run); allocs > 0 || found == 0 {
			t.Errorf("%.1f allocations for %d overlaps (%d bodies), want 0", allocs, overlaps, found)
		}
	}
	w.stepping.Store(true)
	if message := panicMessage(run); message != queryDuringStep {
		t.Errorf("an overlap during a step: panic %q, want %q", message, queryDuringStep)
	}
	w.stepping.Store(false)

	for _, value := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for k := 0; k < 3; k++ {
			at := actor.NewTransform()
			at.Position[k] = value
			if bodies := w.Overlap(&actor.Sphere{Radius: 5}, at, filter, nil); len(bodies) > 0 {
				t.Errorf("%d bodies overlap a sphere at %v", len(bodies), at.Position)
			}
		}
	}
	if bodies := w.Overlap(&actor.Sphere{Radius: 5}, actor.NewTransform(), filter, nil); len(bodies) == 0 {
		t.Error("no body overlaps the sphere at a finite position")
	}
	field := actor.NewHeightfield(2, 2, make([]float32, 4), mgl64.Vec3{1, 1, 1})
	for _, shape := range []actor.ShapeInterface{&actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, field} {
		message := panicMessage(func() { w.Overlap(shape, actor.NewTransform(), filter, nil) })
		if message != notConvex {
			t.Errorf("the overlap of a %T: panic %q, want %q", shape, message, notConvex)
		}
	}
}
