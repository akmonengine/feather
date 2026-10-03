package feather

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// foldedRidge: a ridge along Z at x = 0 and y = 0, folded by the angle: both slopes go down by half of it. 17x17
// samples every 0.5 m: the ridge is made of the edges of the triangles from a sample to the next
func foldedRidge(w *World, fold float64) *actor.RigidBody {
	const samples = 17
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32(-math.Abs(float64(x)-8) * 0.5 * math.Tan(fold/2))
		}
	}
	field := actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{0.5, 1, 0.5})
	return addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), field, actor.BodyTypeStatic, 0.6, 0)
}

// insideBody: how deep the point is in the body (m): its distance to the closest face of a box, to the surface of a
// sphere or of a capsule; 0 out of the body
func insideBody(b *actor.RigidBody, p mgl64.Vec3) float64 {
	local := b.Transform.ToLocal(p)
	switch shape := b.Shape.(type) {
	case *actor.Box:
		depth := math.Inf(1)
		for k := 0; k < 3; k++ {
			depth = math.Min(depth, shape.HalfExtents[k]-math.Abs(local[k]))
		}
		return math.Max(0, depth)
	case *actor.Sphere:
		return math.Max(0, shape.Radius-local.Len())
	case *actor.Capsule:
		onSegment := mgl64.Vec3{0, mgl64.Clamp(local.Y(), -shape.HalfHeight, shape.HalfHeight), 0}
		return math.Max(0, shape.Radius-local.Sub(onSegment).Len())
	}
	return 0
}

// terrainInBody: how deep the terrain (identity transform) is in the body (m), from the points of its surface every
// spacing under the body: never more than the real depth
func terrainInBody(field *actor.Heightfield, b *actor.RigidBody, spacing float64) float64 {
	bounds, depth := b.AABB(), 0.0
	for x := bounds.Min.X(); x <= bounds.Max.X(); x += spacing {
		for z := bounds.Min.Z(); z <= bounds.Max.Z(); z += spacing {
			if height, ok := field.HeightAt(x, z); ok {
				depth = math.Max(depth, insideBody(b, mgl64.Vec3{x, height, z}))
			}
		}
	}
	return depth
}

// terrainContacts: the contacts of the terrain (A) with the body, their deepest separation (+Inf without contact)
func terrainContacts(terrain, b *actor.RigidBody, margin float64) ([]constraint.Manifold, float64) {
	var manifolds [MaxManifoldsPerPair]constraint.Manifold
	count := CollideAll(terrain, b, margin, manifolds[:])
	deepest := math.Inf(1)
	for k := 0; k < count; k++ {
		deepest = math.Min(deepest, manifolds[k].MinSeparation())
	}
	return manifolds[:count], deepest
}

// A capsule and a box laid across a ridge rest on it: whatever its fold, the ridge is not in the body by more than
// LinearSlop, and the body stays where it is: balanced on the ridge, or tipped on a slope (its center then moves by
// its height above the ridge times the sine of the slope). Under 5° the ridge is an inactive edge, the body touches
// it by its side or by its face: none of its corners or of its ends
func TestHeightfieldRidge(t *testing.T) {
	for _, kind := range []string{"capsule", "box"} {
		for _, fold := range []float64{0, 1, 2, 3, 4, 4.9, 5.1, 6, 10} {
			w := newScene(1)
			field := foldedRidge(w, fold*math.Pi/180).Shape.(*actor.Heightfield)
			// across the ridge, 1 mm above it, over the middle of an edge of the ridge
			var b *actor.RigidBody
			height := 0.15
			if kind == "capsule" {
				height = 0.12
				b = addBody(w, mgl64.Vec3{0, 0.121, 0.1}, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1}), &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}, actor.BodyTypeDynamic, 0.6, 0)
			} else {
				b = addBody(w, mgl64.Vec3{0, 0.151, 0.1}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.15, 0.25}}, actor.BodyTypeDynamic, 0.6, 0)
			}
			start, worst := b.Transform.Position, 0.0
			simulate(w, 4, func() { worst = math.Max(worst, terrainInBody(field, b, 0.01)) })
			drift := math.Hypot(b.Transform.Position.X()-start.X(), b.Transform.Position.Z()-start.Z())
			t.Logf("%s, ridge of %.1f°: %.3f mm in the body at worst, %.3f mm at rest, drift %.2f mm, asleep %v", kind, fold, worst*1000,
				terrainInBody(field, b, 0.01)*1000, drift*1000, b.IsSleeping)
			if worst > LinearSlop {
				t.Errorf("%s, ridge of %.1f°: the ridge is %.2f mm in the body, want %.0f mm at most", kind, fold, worst*1000, LinearSlop*1000)
			}
			if drift > height*math.Sin(fold/2*math.Pi/180)+LinearSlop || !b.IsSleeping {
				t.Errorf("%s, ridge of %.1f°: the body moved by %.2f mm, asleep %v", kind, fold, drift*1000, b.IsSleeping)
			}
			w.Close()
		}
	}
}

// The points of a body across an inactive ridge: the ridge itself, where it crosses the side of the capsule or the
// face of the box, at its depth in the body
func TestHeightfieldRidgeContactPoints(t *testing.T) {
	fold := 3 * math.Pi / 180
	// along the normal of a slope, the ridge 0.1 mm in the body as at rest (the side of a capsule is round: its point
	// along this normal is 0.04 mm higher than its closest point)
	const inBody = 0.0001
	wantSeparation := -inBody * math.Cos(fold/2)
	for _, c := range []struct {
		name  string
		shape actor.ShapeInterface
		turn  mgl64.Quat
		half  float64
		// span: the length of the ridge under the body
		span float64
	}{
		{"capsule", &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}, mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1}), 0.12, 0},
		{"box", &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.15, 0.25}}, mgl64.QuatIdent(), 0.15, 0.5},
	} {
		w := newScene(1)
		terrain := foldedRidge(w, fold)
		b := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0, c.half - inBody, 0.1}, Rotation: c.turn}, c.shape, actor.BodyTypeDynamic, 500)
		manifolds, deepest := terrainContacts(terrain, b, SpeculativeDistance)
		minZ, maxZ, onRidge := math.Inf(1), math.Inf(-1), 0
		for _, m := range manifolds {
			for _, point := range m.Points[:m.Count] {
				if math.Abs(point.Position.X()) < LinearSlop && math.Abs(point.Separation-wantSeparation) < weldDistance {
					onRidge++
					minZ, maxZ = math.Min(minZ, point.Position.Z()), math.Max(maxZ, point.Position.Z())
				}
			}
		}
		if onRidge == 0 || math.Abs(deepest-wantSeparation) > weldDistance {
			t.Errorf("%s: %d points on the ridge, the deepest point at %.4f mm, want the ridge at %.4f mm: %+v", c.name, onRidge, deepest*1000, wantSeparation*1000, manifolds)
			continue
		}
		if maxZ-minZ < c.span-1e-6 {
			t.Errorf("%s: the points on the ridge span %.3f m, want both ends of the ridge under the body, %.1f m apart", c.name, maxZ-minZ, c.span)
		}
		// the vertices of the body keep their own manifold, as on a plane: the 4 corners of the box, both ends of the
		// capsule, above the slopes
		vertices := 4
		if c.name == "capsule" {
			vertices = 2
		}
		above := (0.25*math.Tan(fold/2) - inBody) * math.Cos(fold/2)
		found := false
		for _, m := range manifolds {
			if m.Count == vertices && math.Abs(m.MinSeparation()-above) < weldDistance {
				found = true
			}
		}
		if len(manifolds) != 2 || !found {
			t.Errorf("%s: %d manifolds, want the %d vertices of the body %.2f mm above the slopes in one, the ridge in another: %+v", c.name, len(manifolds), vertices, above*1000, manifolds)
		}
	}
}

// The deepest point of a body is a contact, on an inactive edge too: a capsule with its lower end 10 mm in a ridge
// folded by 3°, its other end above a slope within the margin. The end above the slope was the only contact
func TestHeightfieldKeepsDeepestPoint(t *testing.T) {
	fold := 3 * math.Pi / 180
	w := newScene(1)
	terrain := foldedRidge(w, fold)
	field := terrain.Shape.(*actor.Heightfield)
	capsule := &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}
	// the lower end over the ridge, the other end 0.3 m beside it
	low := mgl64.Vec3{0, capsule.Radius - 0.010, 0.1}
	high := mgl64.Vec3{0.3, capsule.Radius - 0.3*math.Tan(fold/2) + 0.015, 0.5}
	axis := high.Sub(low).Normalize()
	b := actor.NewRigidBody(actor.Transform{Position: low.Add(axis.Mul(capsule.HalfHeight)), Rotation: mgl64.QuatBetweenVectors(mgl64.Vec3{0, 1, 0}, axis)},
		capsule, actor.BodyTypeDynamic, 500)
	depth := terrainInBody(field, b, 0.002)
	_, deepest := terrainContacts(terrain, b, SpeculativeDistance+0.01)
	t.Logf("the terrain is %.2f mm in the capsule, its deepest contact at %.2f mm", depth*1000, deepest*1000)
	if depth < 0.009 {
		t.Fatalf("the ridge is %.2f mm in the capsule, want 10 mm", depth*1000)
	}
	if deepest > -depth*patchCos+LinearSlop/10 {
		t.Errorf("the deepest contact is at %.2f mm, the terrain %.2f mm in the capsule", deepest*1000, depth*1000)
	}
}

// The normal of a contact on an edge of a triangle (the rule of ActiveEdges::FixNormal of Jolt, without its part on
// the motion of the body): a sphere 10 mm in a ridge, right above it
func TestHeightfieldContactNormal(t *testing.T) {
	up := mgl64.Vec3{0, 1, 0}
	z := 0.25
	contact := func(fold float64, velocity mgl64.Vec3) (mgl64.Vec3, float64) {
		w := newScene(1)
		terrain := foldedRidge(w, fold)
		sphere := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0, 0.29, z}, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 500)
		sphere.Velocity = velocity
		manifolds, deepest := terrainContacts(terrain, sphere, SpeculativeDistance)
		// the normal of its deepest point
		for _, m := range manifolds {
			if m.MinSeparation() == deepest {
				return m.Normal, deepest
			}
		}
		t.Fatalf("ridge of %.0f°: no contact", degrees(fold))
		return mgl64.Vec3{}, 0
	}
	slopeNormal := func(fold float64, normal mgl64.Vec3) bool {
		left, right := mgl64.Vec3{-math.Sin(fold / 2), math.Cos(fold / 2), 0}, mgl64.Vec3{math.Sin(fold / 2), math.Cos(fold / 2), 0}
		return normal.Sub(left).Len() < 1e-6 || normal.Sub(right).Len() < 1e-6
	}

	// an active edge (10°): the normal of the contact, straight up
	if normal, deepest := contact(10*math.Pi/180, mgl64.Vec3{}); normal.Sub(up).Len() > 1e-9 || math.Abs(deepest+0.01) > 1e-9 {
		t.Errorf("active edge: normal %v at %.6f mm, want straight up at -10 mm", normal, deepest*1000)
	}
	// an inactive edge (3°): the normal of a triangle, the point and its depth are kept: the depth along this normal
	fold := 3 * math.Pi / 180
	if normal, deepest := contact(fold, mgl64.Vec3{}); !slopeNormal(fold, normal) || math.Abs(deepest+0.01*math.Cos(fold/2)) > 1e-7 {
		t.Errorf("inactive edge: normal %v at %.6f mm, want the normal of a slope at %.6f mm", normal, deepest*1000, -10*math.Cos(fold/2))
	}
	// a fold of 1°: no edge of the triangles is active, the normal of a slope
	fold = 1 * math.Pi / 180
	if normal, deepest := contact(fold, mgl64.Vec3{}); !slopeNormal(fold, normal) || math.Abs(deepest+0.01*math.Cos(fold/2)) > 1e-7 {
		t.Errorf("inactive edge of 1°: normal %v at %.6f mm, want the normal of a slope at %.6f mm", normal, deepest*1000, -10*math.Cos(fold/2))
	}
	// the same ridge in the last cell of the terrain: a triangle has an active edge, on the border. The normal of the
	// contact is closer than 1° to its normal: kept. Folded by 3°, it is not
	z = 3.75
	if normal, deepest := contact(fold, mgl64.Vec3{}); normal.Sub(up).Len() > 1e-9 || math.Abs(deepest+0.01) > 1e-9 {
		t.Errorf("inactive edge of 1°, a triangle with an active edge: normal %v at %.6f mm, want straight up at -10 mm", normal, deepest*1000)
	}
	fold = 3 * math.Pi / 180
	if normal, _ := contact(fold, mgl64.Vec3{}); !slopeNormal(fold, normal) {
		t.Errorf("inactive edge of 3°, a triangle with an active edge: normal %v, want the normal of a slope", normal)
	}
	z = 0.25
	// an inactive edge, whatever the motion of the body: the normal of a slope. Jolt keeps the normal of the contact
	// when it brakes the body less than the normal of the slope it leaves; the speculative margin of Feather grows with
	// the speed, and brings the edges of the triangles around under this rule: a box falling flat got 4 contacts
	// tilted by 10 to 60° on a flat terrain, and landed 2 mm deep
	for _, velocity := range []mgl64.Vec3{{1, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0.3, -5, 0}} {
		if normal, deepest := contact(fold, velocity); !slopeNormal(fold, normal) || math.Abs(deepest+0.01*math.Cos(fold/2)) > 1e-7 {
			t.Errorf("inactive edge, moving at %v: normal %v at %.6f mm, want the normal of a slope at %.6f mm", velocity, normal, deepest*1000, -10*math.Cos(fold/2))
		}
	}
}

// A sphere in a valley folded by 3°: the closest point of each slope is inside its triangle, 10 mm deep on one, 9.8 mm
// on the other, their normals in the same patch. Both are contacts of a face: both are kept (the shallower one was
// dropped as a closest point in a patch which had a deeper one, and a sphere rolling from a triangle to the next,
// folded by less than 5°, lost the contact of the triangle it reached: it sank by 3 mm). Beside the valley, the
// closest point of the other slope is on the edge: dropped, the sphere has its one point, as on a plane
func TestHeightfieldSphereInAShallowValleyTouchesBothSlopes(t *testing.T) {
	fold := 3 * math.Pi / 180
	w := newScene(1)
	terrain := foldedRidge(w, -fold)
	for _, x := range []float64{0.004, -0.004, 0.02} {
		// the deepest point 10 mm in the nearest slope
		nearest := mgl64.Vec3{-math.Copysign(math.Sin(fold/2), x), math.Cos(fold / 2), 0}
		y := (0.29 - x*nearest.X()) / nearest.Y()
		sphere := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{x, y, 0.25}, Rotation: mgl64.QuatIdent()}, &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 500)
		manifolds, deepest := terrainContacts(terrain, sphere, SpeculativeDistance)
		if len(manifolds) != 1 || manifolds[0].Normal.Sub(nearest).Len() > 1e-9 || math.Abs(deepest+0.01) > 1e-9 {
			t.Errorf("sphere at x = %.3f: %d manifolds, deepest %.4f mm, want 1 of normal %v at -10 mm: %+v", x, len(manifolds), deepest*1000, nearest, manifolds)
			continue
		}
		m := manifolds[0]
		// the other slope: its closest point is inside its triangle if the foot of the center is beyond the valley
		other := 0.3 * math.Sin(fold/2)
		if math.Abs(x) >= other {
			if m.Count != 1 {
				t.Errorf("sphere at x = %.3f, beside the valley: %d points, want its one point: %+v", x, m.Count, m.Points[:m.Count])
			}
			continue
		}
		if m.Count != 2 {
			t.Errorf("sphere at x = %.3f, in the valley: %d points, want one on each slope: %+v", x, m.Count, m.Points[:m.Count])
			continue
		}
		for _, point := range m.Points[:m.Count] {
			side := math.Copysign(1, point.Position.X())
			want := -0.01
			if side != math.Copysign(1, x) {
				want += 2 * math.Abs(x) * math.Sin(fold/2)
			}
			if math.Abs(point.Separation-want) > 1e-7 {
				t.Errorf("sphere at x = %.3f: the point at x = %.4f is at %.4f mm, want %.4f mm", x, point.Position.X(), point.Separation*1000, want*1000)
			}
		}
	}
}

// 2 bodies of the bench dropped alone which landed deep for a contact missed at their landing: the sphere 28 of the
// pile 51, which sank by 3.4 mm in the hills as it rolled from a triangle to the next, folded by 4.8° (the contact
// of the triangle it reached was dropped), and the box 36 of the pile 22 on the same grid made flat, which landed
// 2 mm deep, straight on a vertex of the grid, held by 4 contacts tilted by the motion rule of Jolt. Each lands
// within a tenth of their old depth
func TestHeightfieldDroppedBodiesLand(t *testing.T) {
	for _, c := range []struct {
		name          string
		seed          int64
		body          int
		flat          bool
		limit, before float64
	}{
		{"the sphere on the hills", 51, 28, false, 0.0003, 0.0034},
		{"the box on the flat terrain", 22, 36, true, 0.0002, 0.002},
	} {
		w := newScene(1)
		terrain := bumpyTerrain(w, c.seed)
		field := terrain.Shape.(*actor.Heightfield)
		if c.flat {
			for k := range field.Heights {
				field.Heights[k] = 0
			}
			w.UpdateHeightfield(terrain, 0, 0, field.XSamples-1, field.ZSamples-1)
		}
		r := rand.New(rand.NewSource(c.seed + 100))
		var b *actor.RigidBody
		for i := 0; i < 60; i++ {
			x, z := r.Float64()*16-8, r.Float64()*16-8
			ground, _ := field.HeightAt(x, z)
			position, rotation, shape := mgl64.Vec3{x, ground + 1 + r.Float64()*3, z}, droppedTurn(r), droppedShape(r, i)
			if i == c.body {
				b = addBody(w, position, rotation, shape, actor.BodyTypeDynamic, 0.6, 0)
				b.Material.RollingResistance = 0.1
			}
		}
		worst, worstStep := 0.0, 0
		for step := 0; step < 240; step++ {
			w.Step(sceneDt)
			if depth := terrainInBody(field, b, 0.002); depth > worst {
				worst, worstStep = depth, step
			}
		}
		if worst > c.limit {
			t.Errorf("%s landed %.2f mm deep at the step %d, want less than %.1f mm (%.1f mm before)", c.name, worst*1000, worstStep, c.limit*1000, c.before*1000)
		}
		w.Close()
	}
}

// A triangle seen from below is ignored: its plane is above the center of the body (as Jolt, Box3D and PhysX). A body
// whose center went through the terrain falls under it: the terrain is a surface, without thickness
func TestHeightfieldIgnoresTrianglesSeenFromBelow(t *testing.T) {
	w := newScene(1)
	terrain := slopeTerrain(w, 0, 0.5)
	for _, c := range []struct {
		name  string
		shape actor.ShapeInterface
	}{
		{"sphere", &actor.Sphere{Radius: 0.3}},
		{"box", &actor.Box{HalfExtents: mgl64.Vec3{0.3, 0.3, 0.3}}},
		{"capsule", &actor.Capsule{HalfHeight: 0.2, Radius: 0.3}},
	} {
		// the body still crosses the surface, 0.25 m above it or 0.25 m under it
		for _, height := range []float64{0.05, -0.05} {
			b := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0.1, height, 0.2}, Rotation: mgl64.QuatIdent()}, c.shape, actor.BodyTypeDynamic, 500)
			manifolds, deepest := terrainContacts(terrain, b, SpeculativeDistance)
			lowest := b.SupportWorld(mgl64.Vec3{0, -1, 0}).Y()
			if height > 0 && (len(manifolds) != 1 || math.Abs(deepest-lowest) > 1e-9) {
				t.Errorf("%s, center %.2f m above the terrain: %d manifolds, deepest %.4f m, want 1 at %.4f", c.name, height, len(manifolds), deepest, lowest)
			}
			if height < 0 && len(manifolds) != 0 {
				t.Errorf("%s, center %.2f m under the terrain: %d manifolds, want none", c.name, -height, len(manifolds))
			}
		}
	}

	sphere := addBody(w, mgl64.Vec3{0.1, -0.05, 0.2}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 0.5, 0)
	simulate(w, 1, nil)
	if sphere.Transform.Position.Y() > -2 {
		t.Errorf("the sphere whose center is under the terrain is at %.3f m, want it falling under the terrain", sphere.Transform.Position.Y())
	}
}

// On a flat terrain the contact of a body is its contact with a plane. A box lying over several cells keeps its 4
// corners, a lying capsule both its ends, at the same separation; a body on a corner, on an edge or on an end keeps
// its deepest point. The body moves, and the margin follows its speed as in the world: the edges of the triangles
// around are within it, and give no contact of their own (the motion rule of Jolt gave them one, tilted)
func TestHeightfieldFlatTerrainLikePlane(t *testing.T) {
	w := newScene(1)
	terrain := slopeTerrain(w, 0, 0.5)
	plane := actor.NewRigidBody(actor.Transform{Rotation: mgl64.QuatIdent()}, &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, actor.BodyTypeStatic, 0)
	r := rand.New(rand.NewSource(1))
	lying := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	for i := 0; i < 300; i++ {
		yaw := mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{0, 1, 0})
		if i%10 == 0 {
			// along the grid: the sides of the body on the edges of the triangles
			yaw = mgl64.QuatRotate(float64(i/10%4)*math.Pi/4, mgl64.Vec3{0, 1, 0})
		}
		tilt := mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{r.Float64(), r.Float64(), r.Float64()}.Normalize())
		var shape actor.ShapeInterface
		var rotation mgl64.Quat
		flat, name := false, ""
		switch i % 5 {
		case 0:
			shape, rotation, flat, name = &actor.Box{HalfExtents: mgl64.Vec3{0.2 + 0.4*r.Float64(), 0.15, 0.25 + 0.3*r.Float64()}}, yaw, true, "lying box"
		case 1:
			shape, rotation, flat, name = &actor.Capsule{HalfHeight: 0.25 + 0.4*r.Float64(), Radius: 0.12}, yaw.Mul(lying), true, "lying capsule"
		case 2:
			shape, rotation, name = &actor.Box{HalfExtents: mgl64.Vec3{0.2 + 0.4*r.Float64(), 0.15, 0.25}}, tilt, "tilted box"
		case 3:
			shape, rotation, name = &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}, tilt, "tilted capsule"
		case 4:
			shape, rotation, flat, name = &actor.Sphere{Radius: 0.2}, tilt, false, "sphere"
		}
		b := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{r.Float64()*8 - 4, 0, r.Float64()*8 - 4}, Rotation: rotation}, shape, actor.BodyTypeDynamic, 500)
		// its lowest point 0.1 mm in the terrain
		b.Transform.Position[1] = -b.SupportWorld(mgl64.Vec3{0, -1, 0}).Y() - 0.0001
		b.Velocity = mgl64.Vec3{r.Float64()*6 - 3, -r.Float64() * 6, r.Float64()*6 - 3}
		margin := SpeculativeDistance + b.Velocity.Len()*sceneDt

		var onPlane constraint.Manifold
		if !Collide(plane, b, margin, &onPlane) {
			t.Fatalf("%s %d: no contact with the plane", name, i)
		}
		manifolds, deepest := terrainContacts(terrain, b, margin)
		if len(manifolds) != 1 || manifolds[0].Normal.Sub(mgl64.Vec3{0, 1, 0}).Len() > 1e-9 {
			normals := make([]mgl64.Vec3, len(manifolds))
			for k, m := range manifolds {
				normals[k] = m.Normal
			}
			t.Errorf("%s %d: %d manifolds on the terrain, want 1 straight up: %.3f", name, i, len(manifolds), normals)
			continue
		}
		onTerrain := manifolds[0]
		has := func(m constraint.Manifold, point constraint.ContactPoint) bool {
			for _, other := range m.Points[:m.Count] {
				if other.Position.Sub(point.Position).Len() < 1e-9 && math.Abs(other.Separation-point.Separation) < 1e-9 {
					return true
				}
			}
			return false
		}
		if math.Abs(deepest-onPlane.MinSeparation()) > 1e-9 {
			t.Errorf("%s %d: the deepest point at %.6f mm on the terrain, %.6f mm on the plane", name, i, deepest*1000, onPlane.MinSeparation()*1000)
		}
		if !flat {
			continue
		}
		for _, point := range onPlane.Points[:onPlane.Count] {
			if !has(onTerrain, point) {
				t.Errorf("%s %d: the point %v of the plane is not a point of the terrain: %v", name, i, point.Position, onTerrain.Points[:onTerrain.Count])
			}
		}
		if _, box := shape.(*actor.Box); box && onTerrain.Count != onPlane.Count {
			t.Errorf("%s %d: %d points on the terrain, %d on the plane", name, i, onTerrain.Count, onPlane.Count)
		}
	}
}

// droppedShape & droppedTurn: the bodies of the piles of the bench (bench/regression.go: mixedShape, randomTurn)
func droppedShape(r *rand.Rand, i int) actor.ShapeInterface {
	switch i % 3 {
	case 1:
		return &actor.Sphere{Radius: 0.2}
	case 2:
		return &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}
	}
	return &actor.Box{HalfExtents: mgl64.Vec3{0.2 + 0.2*r.Float64(), 0.15, 0.25}}
}

func droppedTurn(r *rand.Rand) mgl64.Quat {
	return mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{r.Float64(), r.Float64(), r.Float64()}.Normalize())
}

// keepsItsContact: before each step, the deepest contact of the body with the terrain is as deep as the terrain is in
// the body, to the angle of a patch and a tenth of LinearSlop. A body deeper than its smallest half size is out of the
// rule: the triangles above its center are ignored
func keepsItsContact(t *testing.T, name string, terrain, b *actor.RigidBody, step int, worst *float64) {
	t.Helper()
	const thinnest = 0.12
	depth := terrainInBody(terrain.Shape.(*actor.Heightfield), b, 0.004)
	if depth < LinearSlop/10 || depth > thinnest {
		return
	}
	_, deepest := terrainContacts(terrain, b, SpeculativeDistance)
	if missing := depth*patchCos - LinearSlop/10 + deepest; missing > *worst {
		*worst = missing
		t.Errorf("%s, step %d: the terrain is %.2f mm in the body, its deepest contact at %.2f mm", name, step, depth*1000, deepest*1000)
	}
}

// 2 bodies of the bench which sank in the hills, their deepest point without a contact: a capsule dropped alone (the
// body 44 of the pile 7) which fell 76 mm in the terrain through an inactive edge, and a box in the rain (the body 130
// of the rain 33) which rested 19 cm in it
func TestHeightfieldDroppedBodiesKeepTheirContact(t *testing.T) {
	// ========== THE CAPSULE ==========
	w := newScene(1)
	terrain := bumpyTerrain(w, 7)
	field := terrain.Shape.(*actor.Heightfield)
	r := rand.New(rand.NewSource(107))
	var capsule *actor.RigidBody
	for i := 0; i < 60; i++ {
		x, z := r.Float64()*16-8, r.Float64()*16-8
		ground, _ := field.HeightAt(x, z)
		position, rotation, shape := mgl64.Vec3{x, ground + 1 + r.Float64()*3, z}, droppedTurn(r), droppedShape(r, i)
		if i == 44 {
			capsule = addBody(w, position, rotation, shape, actor.BodyTypeDynamic, 0.6, 0)
			capsule.Material.RollingResistance = 0.1
		}
	}
	worst := 0.0
	for step := 0; step < 240; step++ {
		keepsItsContact(t, "the capsule", terrain, capsule, step, &worst)
		w.Step(sceneDt)
	}
	w.Close()

	// ========== THE BOX ==========
	w = newScene(1)
	terrain = bumpyTerrain(w, 33)
	field = terrain.Shape.(*actor.Heightfield)
	r = rand.New(rand.NewSource(34))
	worst = 0
	for step := 0; step < 200; step++ {
		if step%5 == 0 && len(w.Bodies) < 200 {
			for i := 0; i < 10; i++ {
				x, z := r.Float64()*16-8, r.Float64()*16-8
				ground, _ := field.HeightAt(x, z)
				b := addBody(w, mgl64.Vec3{x, ground + 4, z}, droppedTurn(r), droppedShape(r, i), actor.BodyTypeDynamic, 0.6, 0)
				b.Velocity = mgl64.Vec3{0, -8, 0}
			}
		}
		if len(w.Bodies) > 130 {
			keepsItsContact(t, "the box", terrain, w.Bodies[130], step, &worst)
		}
		w.Step(sceneDt)
	}
	w.Close()
}
