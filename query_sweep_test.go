package feather

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== EXACT DISTANCES ==========
// The gap a sweep leaves is measured without the engine: the distance from a point to each shape is analytic, and the
// distance from a segment (the core of a capsule) to a convex shape is found along the segment by a ternary search.

// terrainTriangles: every triangle of the terrain, in world space, by its index
func terrainTriangles(terrain *actor.RigidBody) map[int32][3]mgl64.Vec3 {
	field := terrain.Shape.(*actor.Heightfield)
	triangles := map[int32][3]mgl64.Vec3{}
	cellsZ := field.ZSamples - 1
	for x := 0; x < field.XSamples-1; x++ {
		for z := 0; z < cellsZ; z++ {
			if field.Holes != nil && field.Holes[x*cellsZ+z] {
				continue
			}
			for t := 0; t < 2; t++ {
				vertices, _ := field.Triangle(x, z, t)
				for i := range vertices {
					vertices[i] = terrain.Transform.ToWorld(vertices[i])
				}
				triangles[int32(2*(x*cellsZ+z)+t)] = vertices
			}
		}
	}
	return triangles
}

// pointDistance from the point to the surface of the body, negative in the body (a terrain has no inside)
func pointDistance(body *actor.RigidBody, p mgl64.Vec3) float64 {
	local := body.Transform.ToLocal(p)
	switch shape := body.Shape.(type) {
	case *actor.Plane:
		return p.Dot(shape.Normal) + shape.Distance
	case *actor.Sphere:
		return local.Len() - shape.Radius
	case *actor.Capsule:
		return local.Sub(mgl64.Vec3{0, mgl64.Clamp(local.Y(), -shape.HalfHeight, shape.HalfHeight), 0}).Len() - shape.Radius
	case *actor.Box:
		outside, inside := mgl64.Vec3{}, math.Inf(-1)
		for k := 0; k < 3; k++ {
			d := math.Abs(local[k]) - shape.HalfExtents[k]
			outside[k], inside = math.Max(d, 0), math.Max(inside, d)
		}
		if distance := outside.Len(); distance > 0 {
			return distance
		}
		return inside
	case *actor.Heightfield:
		best := math.Inf(1)
		for _, triangle := range nearTriangles(body, p, 3) {
			best = math.Min(best, p.Sub(closestOnTriangle(p, triangle)).Len())
		}
		return best
	}
	panic("unknown shape")
}

// terrains: the triangles of the terrains of the tests, in world space (a terrain doesn't move)
var terrains = map[*actor.RigidBody][][3]mgl64.Vec3{}

// nearTriangles: the triangles of the terrain with a vertex closer to p than the distance
func nearTriangles(terrain *actor.RigidBody, p mgl64.Vec3, distance float64) [][3]mgl64.Vec3 {
	all, found := terrains[terrain]
	if !found {
		for _, triangle := range terrainTriangles(terrain) {
			all = append(all, triangle)
		}
		terrains[terrain] = all
	}
	var near [][3]mgl64.Vec3
	for _, triangle := range all {
		if triangle[0].Sub(p).Len() < distance || triangle[1].Sub(p).Len() < distance || triangle[2].Sub(p).Len() < distance {
			near = append(near, triangle)
		}
	}
	return near
}

// segmentDistance from the segment to the convex set whose distance to a point is given: this distance is convex
// along the segment, its minimum is found by a ternary search
func segmentDistance(bottom, top mgl64.Vec3, distance func(p mgl64.Vec3) float64) float64 {
	at := func(s float64) float64 { return distance(bottom.Add(top.Sub(bottom).Mul(s))) }
	low, high := 0.0, 1.0
	for i := 0; i < 100; i++ {
		a, b := low+(high-low)/3, high-(high-low)/3
		if at(a) < at(b) {
			high = b
		} else {
			low = a
		}
	}
	return at((low + high) / 2)
}

// gapBetween the shape (a sphere, a capsule or a box) at the transform and the body: the distance between their
// surfaces, negative if they overlap. A box is measured by its corners: exact against a plane or a face
func gapBetween(shape actor.ShapeInterface, transform actor.Transform, body *actor.RigidBody) float64 {
	switch shape := shape.(type) {
	case *actor.Sphere:
		return pointDistance(body, transform.Position) - shape.Radius
	case *actor.Capsule:
		bottom, top := shape.Segment(transform)
		// a terrain is not convex: each triangle around the capsule is
		if _, isTerrain := body.Shape.(*actor.Heightfield); isTerrain {
			best := math.Inf(1)
			for _, triangle := range nearTriangles(body, transform.Position, 3) {
				best = math.Min(best, segmentDistance(bottom, top, func(p mgl64.Vec3) float64 { return p.Sub(closestOnTriangle(p, triangle)).Len() }))
			}
			return best - shape.Radius
		}
		return segmentDistance(bottom, top, func(p mgl64.Vec3) float64 { return pointDistance(body, p) }) - shape.Radius
	case *actor.Box:
		best := math.Inf(1)
		for corner := 0; corner < 8; corner++ {
			local := shape.HalfExtents
			for k := 0; k < 3; k++ {
				if corner&(1<<k) != 0 {
					local[k] = -local[k]
				}
			}
			best = math.Min(best, pointDistance(body, transform.ToWorld(local)))
		}
		return best
	}
	panic("unknown shape")
}

// movedBy: the transform after the fraction of the translation
func movedBy(start actor.Transform, translation mgl64.Vec3, fraction float64) actor.Transform {
	return actor.Transform{Position: start.Position.Add(translation.Mul(fraction)), Rotation: start.Rotation}
}

const (
	// roundGap, flatGap: a moving shape stops at most this far from the body it hits, and never in it: with a rounded
	// shape in the pair (criterion 14), between 2 boxes (criterion 19)
	roundGap = 1e-6
	flatGap  = 1e-4
)

// gapBound of a pair of shapes
func gapBound(shape actor.ShapeInterface, body *actor.RigidBody) float64 {
	rounded := func(s actor.ShapeInterface) bool {
		switch s.(type) {
		case *actor.Sphere, *actor.Capsule:
			return true
		}
		return false
	}
	if rounded(shape) || rounded(body.Shape) {
		return roundGap
	}
	return flatGap
}

// ========== SWEEP ==========

// Criteria 14 & 15: a sphere and a capsule moved onto a plane, a sphere, a capsule and a box (its face, an edge, a
// corner) stop between 0 and 1 µm from them. At normal incidence, they travel the analytic distance. The point is on
// the body, the normal is the one of the face
func TestSweepStopsAtTheSurface(t *testing.T) {
	ball := &actor.Sphere{Radius: 0.3}
	pill := &actor.Capsule{HalfHeight: 0.4, Radius: 0.2}
	lying := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	tilted := mgl64.QuatRotate(0.6, mgl64.Vec3{1, 0, 1}.Normalize())
	edgeUp := mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 0, 1})
	cornerUp := mgl64.QuatBetweenVectors(mgl64.Vec3{1, 1, 1}.Normalize(), mgl64.Vec3{0, 1, 0})
	box := &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}
	diagonal, corner := 0.5*math.Sqrt2, 0.5*math.Sqrt(3)
	down := mgl64.Vec3{0, -6, 0}
	cases := []struct {
		name     string
		shape    actor.ShapeInterface
		rotation mgl64.Quat
		target   actor.ShapeInterface
		turned   mgl64.Quat
		// offset of the moving shape beside the target, and the distance it travels down from the height 4 (0 if the
		// case has no simple answer)
		offset mgl64.Vec3
		travel float64
		faceUp bool
	}{
		{"sphere on a plane", ball, mgl64.QuatIdent(), &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, mgl64.QuatIdent(), mgl64.Vec3{}, 4 - 0.3, true},
		{"sphere on a tilted plane", ball, mgl64.QuatIdent(), &actor.Plane{Normal: mgl64.Vec3{0.6, 0.8, 0}}, mgl64.QuatIdent(), mgl64.Vec3{}, (0.6*1 + 0.8*4 - 0.3) / 0.8, false},
		{"sphere on a sphere", ball, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, tilted, mgl64.Vec3{}, 4 - 0.8, false},
		{"sphere on the side of a sphere", ball, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, tilted, mgl64.Vec3{0.48, 0, 0.3}, 4 - math.Sqrt(0.8*0.8-0.48*0.48-0.3*0.3), false},
		{"sphere on a lying capsule", ball, mgl64.QuatIdent(), &actor.Capsule{HalfHeight: 0.6, Radius: 0.25}, lying, mgl64.Vec3{0.2, 0, 0}, 4 - 0.55, false},
		{"sphere on the cap of a capsule", ball, mgl64.QuatIdent(), &actor.Capsule{HalfHeight: 0.6, Radius: 0.25}, mgl64.QuatIdent(), mgl64.Vec3{}, 4 - 0.6 - 0.55, false},
		{"sphere on the face of a box", ball, mgl64.QuatIdent(), box, mgl64.QuatIdent(), mgl64.Vec3{0.2, 0, -0.1}, 4 - 0.8, true},
		{"sphere on the edge of a box", ball, mgl64.QuatIdent(), box, edgeUp, mgl64.Vec3{0, 0, 0.1}, 4 - diagonal - 0.3, false},
		{"sphere on the corner of a box", ball, mgl64.QuatIdent(), box, cornerUp, mgl64.Vec3{}, 4 - corner - 0.3, false},
		{"sphere beside the corner of a box", ball, mgl64.QuatIdent(), box, cornerUp, mgl64.Vec3{0.1, 0, 0.05}, 0, false},
		{"capsule on a plane", pill, mgl64.QuatIdent(), &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, mgl64.QuatIdent(), mgl64.Vec3{}, 4 - 0.6, true},
		{"lying capsule on a plane", pill, lying, &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, mgl64.QuatIdent(), mgl64.Vec3{}, 4 - 0.2, true},
		{"tilted capsule on a tilted plane", pill, tilted, &actor.Plane{Normal: mgl64.Vec3{0.6, 0.8, 0}}, mgl64.QuatIdent(), mgl64.Vec3{}, 0, false},
		{"capsule on a sphere", pill, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, mgl64.QuatIdent(), mgl64.Vec3{}, 4 - 0.5 - 0.6, false},
		{"lying capsule on a sphere", pill, lying, &actor.Sphere{Radius: 0.5}, mgl64.QuatIdent(), mgl64.Vec3{0.3, 0, 0}, 4 - 0.7, false},
		{"lying capsule across a lying capsule", pill, lying, &actor.Capsule{HalfHeight: 0.6, Radius: 0.25}, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{1, 0, 0}), mgl64.Vec3{0.1, 0, 0.2}, 4 - 0.45, false},
		{"tilted capsule on a capsule", pill, tilted, &actor.Capsule{HalfHeight: 0.6, Radius: 0.25}, lying, mgl64.Vec3{0.3, 0, 0.1}, 0, false},
		{"capsule on the face of a box", pill, mgl64.QuatIdent(), box, mgl64.QuatIdent(), mgl64.Vec3{0.1, 0, 0.2}, 4 - 0.5 - 0.6, true},
		{"lying capsule on the face of a box", pill, lying, box, mgl64.QuatIdent(), mgl64.Vec3{0.1, 0, 0.2}, 4 - 0.5 - 0.2, true},
		{"lying capsule across the edge of a box", pill, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{1, 0, 0}).Mul(mgl64.QuatRotate(0.5, mgl64.Vec3{0, 0, 1})), box, edgeUp, mgl64.Vec3{}, 4 - diagonal - 0.2, false},
		{"capsule on the corner of a box", pill, mgl64.QuatIdent(), box, cornerUp, mgl64.Vec3{}, 4 - corner - 0.6, false},
		{"tilted capsule on a tilted box", pill, tilted, box, mgl64.QuatRotate(0.4, mgl64.Vec3{0.2, 1, 0.5}.Normalize()), mgl64.Vec3{0.2, 0, 0.1}, 0, false},
	}
	for _, c := range cases {
		w := newScene(1)
		body := addBody(w, mgl64.Vec3{1, 0, -2}, c.turned, c.target, actor.BodyTypeStatic, 0.5, 0)
		w.SyncQueries()
		start := actor.Transform{Position: mgl64.Vec3{1, 4, -2}.Add(c.offset), Rotation: c.rotation}
		hit, ok := w.Sweep(c.shape, start, down, DefaultQueryFilter())
		if !ok || hit.Body != body {
			t.Errorf("%s: no hit", c.name)
			continue
		}
		gap := gapBetween(c.shape, movedBy(start, down, hit.Fraction), body)
		if gap < 0 || gap > roundGap {
			t.Errorf("%s: the shape stops %.3g m from the body, want between 0 and %g", c.name, gap, roundGap)
		}
		if c.travel > 0 && math.Abs(hit.Fraction*6-c.travel) > roundGap {
			t.Errorf("%s: the shape travels %.9f m, want %.9f", c.name, hit.Fraction*6, c.travel)
		}
		if distance := pointDistance(body, hit.Point); math.Abs(distance) > 1e-9 {
			t.Errorf("%s: the point of the hit is %.3g m from the surface of the body", c.name, distance)
		}
		if math.Abs(hit.Normal.Len()-1) > 1e-9 || hit.Normal.Dot(down) >= 0 || hit.Triangle != actor.NoTriangle {
			t.Errorf("%s: normal %v triangle %d, want a unit normal against the motion", c.name, hit.Normal, hit.Triangle)
		}
		// the normal goes from the point of the body to the shape: the point is the closest of the body to the shape
		if toShape := gapBetween(c.shape, movedBy(start, down, hit.Fraction), body) - gapBetween(c.shape, movedBy(start, down.Add(hit.Normal.Mul(-1e-4/hit.Fraction)), hit.Fraction), body); math.Abs(toShape-1e-4) > 2e-6 {
			t.Errorf("%s: moved by 0.1 mm against the normal, the shape gets %.3g mm closer", c.name, toShape*1000)
		}
		if c.faceUp && !closeTo(hit.Normal, mgl64.Vec3{0, 1, 0}, 1e-6) {
			t.Errorf("%s: normal %v, the face has (0, 1, 0)", c.name, hit.Normal)
		}
		// cut right before the hit: nothing
		if _, ok := w.Sweep(c.shape, start, down.Mul(hit.Fraction-1e-5), DefaultQueryFilter()); ok {
			t.Errorf("%s: a hit with the motion cut 60 µm before it", c.name)
		}
		// away from the body, and along it
		if _, ok := w.Sweep(c.shape, start, mgl64.Vec3{0, 6, 0}, DefaultQueryFilter()); ok {
			t.Errorf("%s: a hit with the motion going away", c.name)
		}
		// from the hit, the same motion hits at once, the motion back hits nothing
		resting := movedBy(start, down, hit.Fraction)
		if again, ok := w.Sweep(c.shape, resting, down, DefaultQueryFilter()); !ok || again.Fraction != 0 || !closeTo(again.Normal, hit.Normal, 1e-6) {
			t.Errorf("%s: from its hit, the shape hits %v at the fraction %g with the normal %v", c.name, ok, again.Fraction, again.Normal)
		}
		if _, ok := w.Sweep(c.shape, resting, mgl64.Vec3{0, 1, 0}, DefaultQueryFilter()); ok {
			t.Errorf("%s: from its hit, the shape moving back hits the body", c.name)
		}
	}
}

// Criterion 16: a shape which starts in a body hits it at once, the normal against the motion, at a point in both
func TestSweepFromAnOverlap(t *testing.T) {
	shapes := []actor.ShapeInterface{&actor.Sphere{Radius: 0.3}, &actor.Capsule{HalfHeight: 0.4, Radius: 0.2}, &actor.Box{HalfExtents: mgl64.Vec3{0.2, 0.3, 0.25}}}
	targets := []actor.ShapeInterface{&actor.Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: -0.5}, &actor.Sphere{Radius: 0.5}, &actor.Capsule{HalfHeight: 0.6, Radius: 0.25}, &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}}
	r := rand.New(rand.NewSource(13))
	for _, shape := range shapes {
		for _, target := range targets {
			for _, offset := range []mgl64.Vec3{{}, {0.2, 0.1, -0.15}, {-0.25, -0.2, 0.15}} {
				w := newScene(1)
				body := addBody(w, mgl64.Vec3{}, randomRotation(r), target, actor.BodyTypeStatic, 0.5, 0)
				w.SyncQueries()
				start := actor.Transform{Position: offset, Rotation: randomRotation(r)}
				// (a box is measured by its corners: they may all be out of the body it overlaps)
				if _, isBox := shape.(*actor.Box); !isBox && gapBetween(shape, start, body) > -0.01 {
					gap := gapBetween(shape, start, body)
					t.Fatalf("%T in %T at %v: the shapes don't overlap (%.3f m)", shape, target, offset, gap)
				}
				for _, translation := range []mgl64.Vec3{{0, -3, 0}, {2, 1, -1}, {}} {
					hit, ok := w.Sweep(shape, start, translation, DefaultQueryFilter())
					want := mgl64.Vec3{}
					if translation.Len() > 0 {
						want = translation.Normalize().Mul(-1)
					}
					if !ok || hit.Body != body || hit.Fraction != 0 || !closeTo(hit.Normal, want, 1e-12) {
						t.Errorf("%T in %T at %v along %v: hit %v fraction %g normal %v, want the fraction 0 and the normal %v", shape, target, offset, translation, ok, hit.Fraction, hit.Normal, want)
						continue
					}
					probe := actor.NewRigidBody(start, shape, actor.BodyTypeStatic, 0)
					if inBody, inShape := pointDistance(body, hit.Point), pointDistance(probe, hit.Point); inBody > 1e-9 || inShape > 1e-9 {
						t.Errorf("%T in %T at %v along %v: the point is %.3g m out of the body and %.3g m out of the shape", shape, target, offset, translation, inBody, inShape)
					}
				}
			}
		}
	}
	// a shape without motion beside a body hits nothing
	w := newScene(1)
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	if _, ok := w.Sweep(shapes[0], actor.Transform{Position: mgl64.Vec3{0, 1, 0}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{}, DefaultQueryFilter()); ok {
		t.Error("a sphere without motion, 45 cm over a box, hits it")
	}
}

// flatTerrain: a level terrain of 17x17 samples every meter at the height 0, its top side up
func flatTerrain(w *World) *actor.RigidBody {
	const samples = 17
	field := actor.NewHeightfield(samples, samples, make([]float32, samples*samples), mgl64.Vec3{1, 1, 1})
	return addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), field, actor.BodyTypeStatic, 0.5, 0)
}

// Criterion 17: a sphere moved down on a level terrain, then on a slope, stops between 0 and 1 µm from it, with the
// normal and the index of the triangle under it
func TestSweepOnATerrain(t *testing.T) {
	ball := &actor.Sphere{Radius: 0.3}
	pill := &actor.Capsule{HalfHeight: 0.4, Radius: 0.2}
	r := rand.New(rand.NewSource(14))
	for _, scene := range []string{"level", "slope", "hills"} {
		w := newScene(1)
		var terrain *actor.RigidBody
		switch scene {
		case "level":
			terrain = flatTerrain(w)
		case "slope":
			terrain = slopeTerrain(w, 0.4, 0.5)
		default:
			terrain = addBody(w, mgl64.Vec3{2, -1, 0.5}, mgl64.QuatRotate(0.3, mgl64.Vec3{0.5, 1, 0.2}.Normalize()), queryTerrain(), actor.BodyTypeStatic, 0.5, 0)
		}
		w.SyncQueries()
		triangles := terrainTriangles(terrain)
		up := terrain.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0})
		for i := 0; i < 60; i++ {
			var shape actor.ShapeInterface = ball
			if i%2 == 1 {
				shape = pill
			}
			start := actor.Transform{Position: terrain.Transform.ToWorld(mgl64.Vec3{r.Float64()*6 - 3, 6, r.Float64()*6 - 3}), Rotation: randomRotation(r)}
			translation := up.Mul(-12).Add(mgl64.Vec3{r.Float64() - 0.5, 0, r.Float64() - 0.5})
			hit, ok := w.Sweep(shape, start, translation, DefaultQueryFilter())
			if !ok || hit.Body != terrain {
				// through a hole
				if scene != "hills" {
					t.Fatalf("%s, sweep %d: no hit", scene, i)
				}
				continue
			}
			if gap := gapBetween(shape, movedBy(start, translation, hit.Fraction), terrain); gap < 0 || gap > roundGap {
				t.Fatalf("%s, sweep %d: the shape stops %.3g m from the terrain, want between 0 and %g", scene, i, gap, roundGap)
			}
			triangle, found := triangles[hit.Triangle]
			if !found {
				t.Fatalf("%s, sweep %d: the triangle %d is not in the terrain", scene, i, hit.Triangle)
			}
			if distance := hit.Point.Sub(closestOnTriangle(hit.Point, triangle)).Len(); distance > 1e-9 {
				t.Fatalf("%s, sweep %d: the point is %.3g m from the triangle %d", scene, i, distance, hit.Triangle)
			}
			normal := triangle[1].Sub(triangle[0]).Cross(triangle[2].Sub(triangle[0])).Normalize()
			if scene != "hills" && !closeTo(hit.Normal, normal, 1e-6) {
				t.Fatalf("%s, sweep %d: normal %v, the triangle has %v", scene, i, hit.Normal, normal)
			}
			if math.Abs(hit.Normal.Len()-1) > 1e-9 || hit.Normal.Dot(translation) >= 0 {
				t.Fatalf("%s, sweep %d: normal %v, want a unit normal against the motion", scene, i, hit.Normal)
			}
			// from below, the shape goes through
			under := actor.Transform{Position: start.Position.Sub(up.Mul(14)), Rotation: start.Rotation}
			if _, ok := w.Sweep(shape, under, translation.Mul(-1), DefaultQueryFilter()); ok {
				t.Fatalf("%s, sweep %d: a hit from below", scene, i)
			}
			if _, ok := w.Sweep(shape, under, mgl64.Vec3{3, 0, 1}, DefaultQueryFilter()); ok && scene != "hills" {
				t.Fatalf("%s, sweep %d: a hit under the terrain", scene, i)
			}
		}
	}
}

// Criterion 17: a sphere through a hole of the terrain hits nothing, on the edge of the hole it hits the edge
func TestSweepThroughAHole(t *testing.T) {
	w := newScene(1)
	terrain := flatTerrain(w)
	field := terrain.Shape.(*actor.Heightfield)
	field.Holes = make([]bool, 16*16)
	// 2x2 cells around the center
	for _, cell := range [][2]int{{7, 7}, {7, 8}, {8, 7}, {8, 8}} {
		field.Holes[cell[0]*16+cell[1]] = true
	}
	w.UpdateHeightfield(terrain, 6, 6, 10, 10)
	w.SyncQueries()
	ball := &actor.Sphere{Radius: 0.3}
	down := mgl64.Vec3{0, -6, 0}
	if _, ok := w.Sweep(ball, actor.Transform{Position: mgl64.Vec3{0.2, 3, -0.1}, Rotation: mgl64.QuatIdent()}, down, DefaultQueryFilter()); ok {
		t.Error("a sphere of 0.3 m through a hole of 2 m hits the terrain")
	}
	// 0.2 m from the edge of the hole, over the hole: the sphere lands on the edge
	start := actor.Transform{Position: mgl64.Vec3{0.8, 3, 0.1}, Rotation: mgl64.QuatIdent()}
	hit, ok := w.Sweep(ball, start, down, DefaultQueryFilter())
	if !ok {
		t.Fatal("a sphere over the edge of a hole hits nothing")
	}
	if gap := gapBetween(ball, movedBy(start, down, hit.Fraction), terrain); gap < 0 || gap > roundGap {
		t.Errorf("the sphere stops %.3g m from the edge of the hole", gap)
	}
	if want := 3 - math.Sqrt(0.3*0.3-0.2*0.2); math.Abs(hit.Fraction*6-want) > roundGap || math.Abs(hit.Point.X()-1) > 1e-9 || math.Abs(hit.Point.Y()) > 1e-9 {
		t.Errorf("the sphere travels %.9f m to the point %v, want %.9f m to the edge of the hole at x = 1", hit.Fraction*6, hit.Point, want)
	}
}

// A sphere sunk in the terrain hits it at once when it moves down or along it, and nothing when it leaves it. A sphere
// in a hole, lower than the terrain, moving level, hits the edge of the hole
func TestSweepLeavesATerrainAndHitsTheEdgeOfAHole(t *testing.T) {
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
	sunk := actor.Transform{Position: mgl64.Vec3{4.3, 0.2, -3.1}, Rotation: mgl64.QuatIdent()}
	for _, c := range []struct {
		translation mgl64.Vec3
		want        bool
	}{{mgl64.Vec3{0, -1, 0}, true}, {mgl64.Vec3{1, -0.5, 0}, true}, {mgl64.Vec3{1, 0, 0}, true}, {mgl64.Vec3{0, 1, 0}, false}, {mgl64.Vec3{1, 0.5, 0}, false}} {
		hit, ok := w.Sweep(ball, sunk, c.translation, DefaultQueryFilter())
		if ok != c.want || (ok && (hit.Fraction != 0 || !closeTo(hit.Normal, c.translation.Normalize().Mul(-1), 1e-12))) {
			t.Errorf("a sphere sunk in the terrain, moving along %v: hit %v at the fraction %g with the normal %v, want %v", c.translation, ok, hit.Fraction, hit.Normal, c.want)
		}
	}
	// in the hole (from x = -1 to 1), its center 10 cm over the terrain: 0.5 m to the edge at x = 1
	inHole := actor.Transform{Position: mgl64.Vec3{0.2, 0.1, 0.3}, Rotation: mgl64.QuatIdent()}
	level := mgl64.Vec3{2, 0, 0}
	hit, ok := w.Sweep(ball, inHole, level, DefaultQueryFilter())
	if !ok {
		t.Fatal("a sphere moving level in a hole, lower than the terrain, goes through the edge of the hole")
	}
	if want := 0.8 - math.Sqrt(0.3*0.3-0.1*0.1); math.Abs(hit.Fraction*2-want) > roundGap || math.Abs(hit.Point.X()-1) > 1e-9 || math.Abs(hit.Point.Y()) > 1e-9 {
		t.Errorf("the sphere travels %.9f m to %v, want %.9f m to the edge of the hole at x = 1", hit.Fraction*2, hit.Point, want)
	}
}

// On a terrain turned by a quarter of a turn, a long capsule lying along x reaches a peak 2.5 m beside its center:
// the cells under the capsule are those of its AABB in the space of the terrain
func TestSweepOfALongShapeOnATurnedTerrain(t *testing.T) {
	const samples = 17
	heights := make([]float32, samples*samples)
	// a peak of 1 m at the sample (8, 5): in the space of the terrain, 3 m from its center along -z
	heights[8*samples+5] = 1
	field := actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{1, 1, 1})
	w := newScene(1)
	turn := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 1, 0})
	terrain := addBody(w, mgl64.Vec3{}, turn, field, actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	peak := terrain.Transform.ToWorld(mgl64.Vec3{0, 1, -3})
	if math.Abs(math.Abs(peak.X())-3) > 1e-9 || math.Abs(peak.Z()) > 1e-9 {
		t.Fatalf("the peak is at %v, want 3 m from the center along x", peak)
	}
	pole := &actor.Capsule{HalfHeight: 3, Radius: 0.1}
	lying := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	// the center of the pole 0.5 m from the peak along x: its end is over the peak
	start := actor.Transform{Position: mgl64.Vec3{peak.X() / 6, 4, 0}, Rotation: lying}
	hit, ok := w.Sweep(pole, start, mgl64.Vec3{0, -6, 0}, DefaultQueryFilter())
	if !ok || math.Abs(hit.Fraction*6-(4-1-0.1)) > roundGap || !closeTo(hit.Point, peak, 1e-6) {
		t.Errorf("the pole lands %v after %.6f m at %v, want 2.9 m down to the peak at %v", ok, hit.Fraction*6, hit.Point, peak)
	}
}

// 2 bodies exactly in contact don't overlap: the moving one hits at once if it moves to the other, with the normal of
// the contact, and nothing if it moves away or along it
func TestSweepFromAnExactContact(t *testing.T) {
	w := newScene(1)
	body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	ball := &actor.Sphere{Radius: 0.5}
	touching := actor.Transform{Position: mgl64.Vec3{0, 1, 0}, Rotation: mgl64.QuatIdent()}
	hit, ok := w.Sweep(ball, touching, mgl64.Vec3{0.5, -1, 0}, DefaultQueryFilter())
	if !ok || hit.Body != body || hit.Fraction != 0 || !closeTo(hit.Normal, mgl64.Vec3{0, 1, 0}, 1e-9) || !closeTo(hit.Point, mgl64.Vec3{0, 0.5, 0}, 1e-9) {
		t.Errorf("a sphere in contact, moving to the body: hit %v at the fraction %g, the point %v, the normal %v", ok, hit.Fraction, hit.Point, hit.Normal)
	}
	for _, translation := range []mgl64.Vec3{{0, 1, 0}, {1, 0.5, 0}, {1, 0, 0}, {}} {
		if _, ok := w.Sweep(ball, touching, translation, DefaultQueryFilter()); ok {
			t.Errorf("a sphere in contact, moving along %v, hits the body", translation)
		}
	}
}

// The buffers of a sweep go back to their pool without the shape nor the bodies of the query: the pool keeps nothing
// of a world alive
func TestSweepReleasesItsBuffers(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	w := queryScene(1)
	for i := 0; i < 20; i++ {
		w.Sweep(&actor.Sphere{Radius: 0.3}, actor.Transform{Position: mgl64.Vec3{0, 5, 0}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{1, -30, 2}, DefaultQueryFilter())
		scratch := queryPool.Get().(*queryScratch)
		if scratch.mover.Shape != nil || scratch.sweep.world != nil || scratch.sweep.shape != nil || scratch.sweep.best.Body != nil || scratch.sweep.filter.Excluded != nil {
			t.Fatalf("sweep %d: the buffers in the pool still hold the query", i)
		}
		queryPool.Put(scratch)
	}
}

// Criterion 17: a sweep skimming a level terrain gives the hit of the same sweep on a plane: the edges between the
// triangles don't catch it
func TestSweepSkimsATerrainAsAPlane(t *testing.T) {
	onTerrain, onPlane := newScene(1), newScene(1)
	terrain := flatTerrain(onTerrain)
	plane := addGround(onPlane, 0.5)
	onTerrain.SyncQueries()
	onPlane.SyncQueries()
	r := rand.New(rand.NewSource(15))
	shapes := []actor.ShapeInterface{&actor.Sphere{Radius: 0.3}, &actor.Capsule{HalfHeight: 0.4, Radius: 0.2}, &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.2, 0.25}}}
	hits := 0
	for i := 0; i < 600; i++ {
		shape := shapes[i%3]
		start := actor.Transform{Position: mgl64.Vec3{r.Float64()*4 - 2, 0.65 + 0.3*r.Float64(), r.Float64()*4 - 2}, Rotation: randomRotation(r)}
		angle := r.Float64() * 2 * math.Pi
		// 5 m along the ground, down by 0 to 1 m: some sweeps stay over the ground
		translation := mgl64.Vec3{5 * math.Cos(angle), -r.Float64(), 5 * math.Sin(angle)}
		if i%10 == 0 {
			translation[1] = 0
		}
		a, okA := onTerrain.Sweep(shape, start, translation, DefaultQueryFilter())
		b, okB := onPlane.Sweep(shape, start, translation, DefaultQueryFilter())
		if okA != okB {
			t.Fatalf("sweep %d of %T: hit %v on the terrain, %v on the plane", i, shape, okA, okB)
		}
		if !okA {
			continue
		}
		hits++
		if a.Body != terrain || b.Body != plane {
			t.Fatalf("sweep %d: the hit is not on the ground", i)
		}
		bound := gapBound(shape, terrain)
		if math.Abs(a.Fraction-b.Fraction)*translation.Len() > bound || !closeTo(a.Normal, mgl64.Vec3{0, 1, 0}, 1e-6) || !closeTo(a.Point, b.Point, 2*bound) {
			t.Fatalf("sweep %d of %T: on the terrain, the fraction %.9f the point %v the normal %v; on the plane, %.9f %v %v", i, shape, a.Fraction, a.Point, a.Normal, b.Fraction, b.Point, b.Normal)
		}
	}
	if hits < 200 || hits > 550 {
		t.Errorf("%d hits for 600 sweeps: they don't cover the hits and the misses", hits)
	}
}

// bruteForceSweep: the first body touched, by a walk of World.Bodies in their order
func bruteForceSweep(w *World, shape actor.ShapeInterface, start actor.Transform, translation mgl64.Vec3, filter QueryFilter) (Hit, bool) {
	best, found := Hit{Fraction: math.Inf(1)}, false
	scratch := queryPool.Get().(*queryScratch)
	defer queryPool.Put(scratch)
	query := newSweepQuery(w, shape, start, translation, filter, scratch)
	for index, body := range w.Bodies {
		if !filter.Accepts(body) {
			continue
		}
		query.limit = 1
		if hit, ok := query.sweepBody(body); ok && hit.Fraction < best.Fraction {
			hit.index = int32(index)
			best, found = hit, true
		}
	}
	return best, found
}

// randomMover: a sphere or a capsule somewhere in the scene, and its motion
func randomMover(r *rand.Rand, w *World, i int) (actor.ShapeInterface, actor.Transform, mgl64.Vec3) {
	var shape actor.ShapeInterface = &actor.Sphere{Radius: 0.1 + 0.5*r.Float64()}
	if i%2 == 1 {
		shape = &actor.Capsule{HalfHeight: 0.2 + 0.6*r.Float64(), Radius: 0.1 + 0.3*r.Float64()}
	}
	origin, translation := randomRay(r, w, i)
	return shape, actor.Transform{Position: origin, Rotation: randomRotation(r)}, translation
}

// Criteria 17 & 18: Sweep gives the body and the fraction of a walk of World.Bodies in their order, for 2000 moving
// spheres and capsules on the 300 bodies of the scene and its terrain
func TestSweepIsTheBruteForce(t *testing.T) {
	w := queryScene(1)
	for phase := 0; phase < 2; phase++ {
		r := rand.New(rand.NewSource(int64(16 + phase)))
		hits, overlaps, onTerrain := 0, 0, 0
		for i := 0; i < 2000; i++ {
			shape, start, translation := randomMover(r, w, i)
			filter := randomFilter(r, w, i)
			want, found := bruteForceSweep(w, shape, start, translation, filter)
			hit, ok := w.Sweep(shape, start, translation, filter)
			if ok != found || hit.Body != want.Body || (ok && math.Abs(hit.Fraction-want.Fraction) > 1e-12) {
				t.Fatalf("phase %d, sweep %d of %T from %v along %v: hit %v on the body %d at %.15f, the brute force has %v on the body %d at %.15f",
					phase, i, shape, start.Position, translation, ok, slices.Index(w.Bodies, hit.Body), hit.Fraction, found, slices.Index(w.Bodies, want.Body), want.Fraction)
			}
			if !ok {
				continue
			}
			hits++
			if hit.Normal != want.Normal || hit.Point != want.Point || hit.Triangle != want.Triangle {
				t.Fatalf("phase %d, sweep %d: normal %v point %v triangle %d, the brute force has %v %v %d", phase, i, hit.Normal, hit.Point, hit.Triangle, want.Normal, want.Point, want.Triangle)
			}
			if hit.Fraction == 0 {
				overlaps++
				continue
			}
			if _, isTerrain := hit.Body.Shape.(*actor.Heightfield); isTerrain {
				// a shape which starts under the terrain goes through it: its gap is measured in the tests of the terrain
				onTerrain++
				if hit.Triangle == actor.NoTriangle {
					t.Fatalf("phase %d, sweep %d: a hit on the terrain without triangle", phase, i)
				}
				continue
			}
			if gap := gapBetween(shape, movedBy(start, translation, hit.Fraction), hit.Body); gap < 0 || gap > roundGap {
				t.Fatalf("phase %d, sweep %d of %T: the shape stops %.3g m from the body %d, want between 0 and %g", phase, i, shape, gap, slices.Index(w.Bodies, hit.Body), roundGap)
			}
		}
		if hits < 800 || overlaps < 50 || onTerrain < 20 {
			t.Errorf("phase %d: %d hits, %d from an overlap, %d on the terrain: the sweeps don't cover the scene", phase, hits, overlaps, onTerrain)
		}
		for _, body := range w.Bodies {
			if isAwakeDynamic(body) {
				body.Velocity = mgl64.Vec3{1, -2, 0.5}
			}
		}
		simulate(w, 0.2, nil)
	}
}

// Criterion 17: a long sweep across the hills gives the hit of a brute force on every triangle
func TestSweepAcrossATerrainIsTheBruteForce(t *testing.T) {
	w := newScene(1)
	terrain := addBody(w, mgl64.Vec3{2, -1, 0.5}, mgl64.QuatRotate(0.3, mgl64.Vec3{0.5, 1, 0.2}.Normalize()), queryTerrain(), actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	triangles := terrainTriangles(terrain)
	indices := make([]int32, 0, len(triangles))
	for index := range triangles {
		indices = append(indices, index)
	}
	slices.Sort(indices)
	r := rand.New(rand.NewSource(17))
	scratch := queryPool.Get().(*queryScratch)
	defer queryPool.Put(scratch)
	hits := 0
	for i := 0; i < 300; i++ {
		var shape actor.ShapeInterface = &actor.Sphere{Radius: 0.1 + 0.4*r.Float64()}
		switch i % 3 {
		case 1:
			shape = &actor.Capsule{HalfHeight: 0.2 + 0.5*r.Float64(), Radius: 0.1 + 0.2*r.Float64()}
		case 2:
			shape = &actor.Box{HalfExtents: mgl64.Vec3{0.1 + 0.3*r.Float64(), 0.1 + 0.3*r.Float64(), 0.1 + 0.3*r.Float64()}}
		}
		// across the terrain, from 0 to 3 m over it to 2 m under it at most
		from := mgl64.Vec3{r.Float64()*36 - 18, 1.5 + 3*r.Float64(), r.Float64()*36 - 18}
		to := mgl64.Vec3{r.Float64()*36 - 18, 3*r.Float64() - 2, r.Float64()*36 - 18}
		start := actor.Transform{Position: terrain.Transform.ToWorld(from), Rotation: randomRotation(r)}
		translation := terrain.Transform.ToWorld(to).Sub(start.Position)

		query := newSweepQuery(w, shape, start, translation, DefaultQueryFilter(), scratch)
		want, found := Hit{Fraction: 1}, false
		for _, index := range indices {
			if hit, ok := query.sweepTriangle(triangles[index], want.Fraction); ok && (!found || hit.Fraction < want.Fraction) {
				hit.Triangle = index
				want, found = hit, true
			}
		}
		hit, ok := w.Sweep(shape, start, translation, DefaultQueryFilter())
		if ok != found || (ok && (hit.Fraction != want.Fraction || hit.Triangle != want.Triangle || hit.Normal != want.Normal || hit.Point != want.Point)) {
			t.Fatalf("sweep %d of %T: hit %v at %.15f on the triangle %d, the brute force has %v at %.15f on %d", i, shape, ok, hit.Fraction, hit.Triangle, found, want.Fraction, want.Triangle)
		}
		if !ok {
			continue
		}
		hits++
		// a box is measured by its corners: it may touch the terrain by an edge
		_, isBox := shape.(*actor.Box)
		if gap := gapBetween(shape, movedBy(start, translation, hit.Fraction), terrain); hit.Fraction > 0 && (gap < 0 || (!isBox && gap > roundGap)) {
			t.Fatalf("sweep %d of %T: the shape stops %.3g m from the terrain", i, shape, gap)
		}
	}
	if hits < 150 {
		t.Errorf("%d hits for 300 sweeps across the terrain", hits)
	}
}

// Criterion 19: a moving box stops between 0 and 0.1 mm from a box and a plane, 1 µm from a rounded body. A plane or
// a heightfield cannot move: the sweep panics
func TestSweepOfABox(t *testing.T) {
	mover := &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.2, 0.25}}
	r := rand.New(rand.NewSource(18))
	targets := []actor.ShapeInterface{&actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}}, &actor.Sphere{Radius: 0.8}, &actor.Capsule{HalfHeight: 0.6, Radius: 0.5}}
	for _, target := range targets {
		for i := 0; i < 40; i++ {
			w := newScene(1)
			body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), target, actor.BodyTypeStatic, 0.5, 0)
			w.SyncQueries()
			rotation := randomRotation(r)
			if i%4 == 0 {
				// a face down
				rotation = mgl64.QuatIdent()
			}
			start := actor.Transform{Position: mgl64.Vec3{0.3*r.Float64() - 0.15, 4, 0.3*r.Float64() - 0.15}, Rotation: rotation}
			translation := mgl64.Vec3{0.2 * r.Float64(), -6, 0}
			hit, ok := w.Sweep(mover, start, translation, DefaultQueryFilter())
			if !ok || hit.Body != body {
				t.Fatalf("box on %T, sweep %d: no hit", target, i)
			}
			bound := gapBound(mover, body)
			if _, isBox := target.(*actor.Box); isBox || bound == flatGap {
				// the corners of the moving box are over the face of the target: the gap of the lowest corner is exact
				if gap := gapBetween(mover, movedBy(start, translation, hit.Fraction), body); gap < 0 || gap > bound {
					t.Fatalf("box on %T, sweep %d: the box stops %.3g m from the body, want between 0 and %g", target, i, gap, bound)
				}
			}
			// the other way: the box seen from the body
			probe := actor.NewRigidBody(movedBy(start, translation, hit.Fraction), mover, actor.BodyTypeStatic, 0)
			if _, isPlane := target.(*actor.Plane); !isPlane {
				if gap := gapBetween(target, body.Transform, probe); gap < 0 || (bound == roundGap && gap > bound) {
					t.Fatalf("box on %T, sweep %d: the body is %.3g m from the box, want between 0 and %g", target, i, gap, bound)
				}
			}
			if distance := pointDistance(body, hit.Point); math.Abs(distance) > 1e-9 {
				t.Fatalf("box on %T, sweep %d: the point is %.3g m from the surface of the body", target, i, distance)
			}
		}
	}

	w := newScene(1)
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	field := actor.NewHeightfield(2, 2, make([]float32, 4), mgl64.Vec3{1, 1, 1})
	for _, shape := range []actor.ShapeInterface{&actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, field} {
		message := panicMessage(func() { w.Sweep(shape, actor.NewTransform(), mgl64.Vec3{0, -1, 0}, DefaultQueryFilter()) })
		if message != notConvex {
			t.Errorf("the sweep of a %T: panic %q, want %q", shape, message, notConvex)
		}
	}
}

// Criterion 13, for a sweep: a motion which is not finite hits nothing
func TestSweepNotFinite(t *testing.T) {
	w := queryScene(1)
	ball := &actor.Sphere{Radius: 0.3}
	for _, value := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		for k := 0; k < 3; k++ {
			start, translation := actor.Transform{Position: mgl64.Vec3{0, 5, 0}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{1, -30, 2}
			translation[k] = value
			if _, ok := w.Sweep(ball, start, translation, DefaultQueryFilter()); ok {
				t.Errorf("a hit along %v", translation)
			}
			translation = mgl64.Vec3{1, -30, 2}
			start.Position[k] = value
			if _, ok := w.Sweep(ball, start, translation, DefaultQueryFilter()); ok {
				t.Errorf("a hit from %v", start.Position)
			}
		}
	}
	if _, ok := w.Sweep(ball, actor.Transform{Position: mgl64.Vec3{0, 5, 0}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{1, -30, 2}, DefaultQueryFilter()); !ok {
		t.Error("the finite sweep hits nothing")
	}
}

// Criteria 21 & 23: a sweep doesn't allocate, and panics during a step
func TestSweepDoesNotAllocateAndIsGuarded(t *testing.T) {
	w := queryScene(1)
	r := rand.New(rand.NewSource(19))
	const sweeps = 100
	shapes, starts, translations := make([]actor.ShapeInterface, sweeps), make([]actor.Transform, sweeps), make([]mgl64.Vec3, sweeps)
	for i := range shapes {
		shapes[i], starts[i], translations[i] = randomMover(r, w, i)
		if i%5 == 0 {
			shapes[i] = cube()
		}
	}
	filter := DefaultQueryFilter()
	found := 0
	run := func() {
		for i := range shapes {
			if _, ok := w.Sweep(shapes[i], starts[i], translations[i], filter); ok {
				found++
			}
		}
	}
	if !raceEnabled {
		run()
		if allocs := testing.AllocsPerRun(10, run); allocs > 0 || found == 0 {
			t.Errorf("%.1f allocations for %d sweeps (%d hits), want 0", allocs, sweeps, found)
		}
	}
	w.stepping.Store(true)
	if message := panicMessage(run); message != queryDuringStep {
		t.Errorf("a sweep during a step: panic %q, want %q", message, queryDuringStep)
	}
	w.stepping.Store(false)
}
