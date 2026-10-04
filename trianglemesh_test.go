package feather

import (
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== MESHES ==========

// gridMesh: n x n cells of size cell, at the height of the function, the vertices shared, the triangles facing up
func gridMesh(n int, cell float64, height func(x, z float64) float64) *actor.TriangleMesh {
	vertices := make([]mgl64.Vec3, 0, (n+1)*(n+1))
	for x := 0; x <= n; x++ {
		for z := 0; z <= n; z++ {
			px, pz := (float64(x)-float64(n)/2)*cell, (float64(z)-float64(n)/2)*cell
			vertices = append(vertices, mgl64.Vec3{px, height(px, pz), pz})
		}
	}
	indices := make([]int32, 0, 6*n*n)
	for x := 0; x < n; x++ {
		for z := 0; z < n; z++ {
			a, b, c, d := int32(x*(n+1)+z), int32(x*(n+1)+z+1), int32((x+1)*(n+1)+z+1), int32((x+1)*(n+1)+z)
			indices = append(indices, a, b, c, a, c, d)
		}
	}
	m, err := actor.NewTriangleMesh(vertices, indices)
	if err != nil {
		panic(err)
	}
	return m
}

// stairMesh: steps going up along x, each of width and rise, 2*depth wide: the risers face -x, the treads face up. The
// treads start at x = 0: the tread s (from 0) is at the height (s+1)*rise, from x = s*width to (s+1)*width
func stairMesh(steps int, width, rise, depth float64) *actor.TriangleMesh {
	var vertices []mgl64.Vec3
	var indices []int32
	quad := func(a, b, c, d mgl64.Vec3) {
		i := int32(len(vertices))
		vertices = append(vertices, a, b, c, d)
		indices = append(indices, i, i+1, i+2, i, i+2, i+3)
	}
	for s := 0; s < steps; s++ {
		x0, x1 := float64(s)*width, float64(s+1)*width
		y := float64(s+1) * rise
		quad(mgl64.Vec3{x0, y - rise, -depth}, mgl64.Vec3{x0, y - rise, depth}, mgl64.Vec3{x0, y, depth}, mgl64.Vec3{x0, y, -depth})
		quad(mgl64.Vec3{x0, y, -depth}, mgl64.Vec3{x0, y, depth}, mgl64.Vec3{x1, y, depth}, mgl64.Vec3{x1, y, -depth})
	}
	m, err := actor.NewTriangleMesh(vertices, indices)
	if err != nil {
		panic(err)
	}
	return m
}

// meshDepth: the deepest point of the body under a flat grid mesh at the height 0 (its bottom along the normal)
func meshDepth(body *actor.RigidBody) float64 {
	return math.Max(0, -body.SupportWorld(mgl64.Vec3{0, -1, 0}).Y())
}

// Criterion 2: on a mesh plane (a tilted grid), a sphere rolls exactly as on a plane: the inner edges are invisible
func TestSphereRollsOnAMeshLikeOnAPlane(t *testing.T) {
	angle := 15 * math.Pi / 180
	normal := mgl64.Vec3{-math.Sin(angle), math.Cos(angle), 0}
	start := mgl64.Vec3{3.1, 3.1*math.Tan(angle) + 0.3/math.Cos(angle), 0.37}

	onPlane := newScene(1)
	addBody(onPlane, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Plane{Normal: normal}, actor.BodyTypeStatic, 0.5, 0)
	planeSphere := addBody(onPlane, start, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 0.5, 0)

	onMesh := newScene(1)
	mesh := gridMesh(64, 0.5, func(x, z float64) float64 { return x * math.Tan(angle) })
	addBody(onMesh, mgl64.Vec3{}, mgl64.QuatIdent(), mesh, actor.BodyTypeStatic, 0.5, 0)
	meshSphere := addBody(onMesh, start, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.3}, actor.BodyTypeDynamic, 0.5, 0)

	worst := 0.0
	for step := 0; step < int(math.Round(2/sceneDt)); step++ {
		onPlane.Step(sceneDt)
		onMesh.Step(sceneDt)
		worst = math.Max(worst, planeSphere.Transform.Position.Sub(meshSphere.Transform.Position).Len())
	}
	travelled := meshSphere.Transform.Position.Sub(start).Len()
	t.Logf("travelled %.2f m, worst gap with the plane %.4f mm", travelled, worst*1000)
	if travelled < 2 {
		t.Errorf("the sphere didn't roll: %.2f m", travelled)
	}
	if worst > 0.001 {
		t.Errorf("the sphere is %.3f mm from the sphere on the plane", worst*1000)
	}
}

// A box sliding on a flat mesh crosses the inner edges without being kicked, across the diagonals and the sides
func TestBoxSlidesOverTheInnerEdgesOfAMesh(t *testing.T) {
	w := newScene(1)
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), gridMesh(40, 0.5, func(x, z float64) float64 { return 0 }), actor.BodyTypeStatic, 0.1, 0)
	position := mgl64.Vec3{-6, cubeHalf, -3.2}
	box := addBody(w, position, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.1, 0)
	box.Velocity = mgl64.Vec3{5, 0, 2}
	worstNormalSpeed, worstSpin, worstDepth := 0.0, 0.0, 0.0
	simulate(w, 1, func() {
		worstNormalSpeed = math.Max(worstNormalSpeed, math.Abs(box.Velocity.Y()))
		worstSpin = math.Max(worstSpin, box.AngularVelocity.Len())
		worstDepth = math.Max(worstDepth, meshDepth(box))
	})
	travelled := box.Transform.Position.Sub(position).Len()
	t.Logf("travelled %.2f m, worst normal speed %.4f m/s, worst spin %.4f rad/s, worst depth %.3f mm", travelled, worstNormalSpeed, worstSpin, worstDepth*1000)
	if travelled < 2 {
		t.Errorf("the box stopped after %.2f m", travelled)
	}
	if worstNormalSpeed > 0.01 || worstSpin > 0.05 {
		t.Errorf("the box was kicked by an inner edge")
	}
	if worstDepth > landingDepth {
		t.Errorf("the box went %.2f mm in the mesh", worstDepth*1000)
	}
}

// bruteForceMeshRay: the first triangle of the mesh (at the identity transform) on the ray, from the side of its normal,
// the lowest index at the same fraction
func bruteForceMeshRay(m *actor.TriangleMesh, origin, translation mgl64.Vec3) (float64, int32, bool) {
	best, triangle, found := 1.0, actor.NoTriangle, false
	for i := 0; i < m.TriangleCount(); i++ {
		vertices, _ := m.Triangle(int32(i))
		edge1, edge2 := vertices[1].Sub(vertices[0]), vertices[2].Sub(vertices[0])
		p := translation.Cross(edge2)
		determinant := edge1.Dot(p)
		if determinant <= 0 {
			continue
		}
		s := origin.Sub(vertices[0])
		u := s.Dot(p) / determinant
		q := s.Cross(edge1)
		v := translation.Dot(q) / determinant
		fraction := edge2.Dot(q) / determinant
		if u < -1e-9 || v < -1e-9 || u+v > 1+1e-9 || fraction < 0 || fraction > best || (found && fraction == best && int32(i) > triangle) {
			continue
		}
		best, triangle, found = fraction, int32(i), true
	}
	return best, triangle, found
}

// Criterion 3: a ray on a mesh of 100 000 triangles gives the triangle of the brute force (250 rays, each tested
// against every triangle), and takes less than 5 µs on average (measured on 100 000 rays, without the race detector)
func TestRayOnAMeshOf100000Triangles(t *testing.T) {
	mesh := gridMesh(224, 0.25, func(x, z float64) float64 { return 2*math.Sin(x*0.3)*math.Cos(z*0.2) + 0.3*math.Sin(x*2+z) })
	if mesh.TriangleCount() < 100000 {
		t.Fatalf("%d triangles", mesh.TriangleCount())
	}
	w := newScene(1)
	body := addBody(w, mgl64.Vec3{1, 2, 3}, mgl64.QuatRotate(0.3, mgl64.Vec3{0, 1, 0}), mesh, actor.BodyTypeStatic, 0.5, 0)
	addBody(w, mgl64.Vec3{0, 50, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 1}, actor.BodyTypeDynamic, 0.5, 0)
	w.SyncQueries()
	r := rand.New(rand.NewSource(7))
	filter := DefaultQueryFilter()
	checked := 0
	for i := 0; i < 250; i++ {
		origin := mgl64.Vec3{(r.Float64() - 0.5) * 50, 10 + r.Float64()*5, (r.Float64() - 0.5) * 50}
		translation := mgl64.Vec3{r.NormFloat64() * 5, -20, r.NormFloat64() * 5}
		wantFraction, wantTriangle, wantOk := bruteForceMeshRay(mesh, body.Transform.ToLocal(origin), body.Transform.Rotation.Conjugate().Rotate(translation))
		hit, ok := w.Raycast(origin, translation, filter)
		if ok != wantOk || (ok && hit.Body != body) {
			t.Fatalf("ray %d: hit %v, the brute force says %v", i, ok, wantOk)
		}
		if !ok {
			continue
		}
		checked++
		if hit.Triangle != wantTriangle || math.Abs(hit.Fraction-wantFraction) > 1e-12 {
			t.Fatalf("ray %d: triangle %d at %.12f, the brute force says %d at %.12f", i, hit.Triangle, hit.Fraction, wantTriangle, wantFraction)
		}
		if gap := hit.Point.Sub(origin.Add(translation.Mul(hit.Fraction))).Len(); gap > 1e-9 {
			t.Fatalf("ray %d: the point is %.3g from the ray", i, gap)
		}
		vertices, _ := mesh.Triangle(hit.Triangle)
		if normal := body.Transform.Rotation.Rotate(vertices[1].Sub(vertices[0]).Cross(vertices[2].Sub(vertices[0])).Normalize()); hit.Normal.Sub(normal).Len() > 1e-9 {
			t.Fatalf("ray %d: normal %v, want %v", i, hit.Normal, normal)
		}
	}
	if checked < 125 {
		t.Fatalf("only %d rays checked", checked)
	}

	const rays = 100000
	origins := make([]mgl64.Vec3, rays)
	for i := range origins {
		origins[i] = mgl64.Vec3{(r.Float64() - 0.5) * 50, 12, (r.Float64() - 0.5) * 50}
	}
	hits := 0
	start := time.Now()
	for _, origin := range origins {
		if _, ok := w.Raycast(origin, mgl64.Vec3{1, -20, 0.5}, filter); ok {
			hits++
		}
	}
	perRay := time.Since(start) / rays
	t.Logf("%d rays on %d triangles: %d hits, %v per ray", rays, mesh.TriangleCount(), hits, perRay)
	if hits < rays/2 {
		t.Errorf("only %d hits", hits)
	}
	if !raceEnabled && perRay > 5*time.Microsecond {
		t.Errorf("%v per ray, want less than 5 µs", perRay)
	}
}

// Criterion 4: dynamic boxes rest on a staircase mesh for 10 s: a cube on a tread, a cube overhanging the edge of a
// tread, a plank lying along the slope on the edges of 3 treads (active edges only). None drifts, none sinks
func TestBoxesRestOnMeshStairs(t *testing.T) {
	const width, rise = 0.3, 0.17
	w := newScene(1)
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), stairMesh(8, width, rise, 1), actor.BodyTypeStatic, 0.6, 0)
	small := &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.1, 0.1}}
	onTread := addBody(w, mgl64.Vec3{2.5 * width, 3*rise + 0.1 + 0.002, 0}, mgl64.QuatIdent(), small, actor.BodyTypeDynamic, 0.6, 0)
	overhanging := addBody(w, mgl64.Vec3{4*width + 0.05, 5*rise + 0.1 + 0.002, 0.5}, mgl64.QuatIdent(), small, actor.BodyTypeDynamic, 0.6, 0)
	slope := math.Atan2(rise, width)
	plank := &actor.Box{HalfExtents: mgl64.Vec3{0.6, 0.03, 0.1}}
	onEdges := addBody(w, mgl64.Vec3{4 * width, 4*rise + 0.25, -0.6}, mgl64.QuatRotate(slope, mgl64.Vec3{0, 0, 1}), plank, actor.BodyTypeDynamic, 0.8, 0)

	bodies := map[string]*actor.RigidBody{"cube on a tread": onTread, "cube overhanging an edge": overhanging, "plank on 3 edges": onEdges}
	simulate(w, 2, nil)
	settled := map[string]actor.Transform{}
	for name, body := range bodies {
		settled[name] = body.Transform
	}
	simulate(w, 10, nil)
	for name, body := range bodies {
		drift := body.Transform.Position.Sub(settled[name].Position).Len()
		turned := 2 * math.Acos(math.Min(1, math.Abs(body.Transform.Rotation.Dot(settled[name].Rotation))))
		t.Logf("%s: drifted %.3f mm and turned %.4f° in 10 s, sleeping %v", name, drift*1000, turned*180/math.Pi, body.IsSleeping)
		if drift > 0.0005 || turned > 0.001 || !body.IsSleeping {
			t.Errorf("%s is not stable", name)
		}
	}
	// the cube on its tread, the plank on the edges: at their height
	if depth := 3*rise + 0.1 - onTread.Transform.Position.Y(); math.Abs(depth) > landingDepth {
		t.Errorf("the cube on the tread is %.2f mm deep", depth*1000)
	}
	// the plank's bottom line passes 3 cm (its half thickness) over the edges, in the direction of its normal
	normal := onEdges.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0})
	for _, edge := range []mgl64.Vec3{{3 * width, 4 * rise, -0.6}, {4 * width, 5 * rise, -0.6}, {5 * width, 6 * rise, -0.6}} {
		if gap := onEdges.Transform.Position.Sub(edge).Dot(normal) - 0.03; math.Abs(gap) > landingDepth {
			t.Errorf("the plank is %.2f mm from the edge %v", gap*1000, edge)
		}
	}
}

// A mesh is a surface with a side: a sphere coming from behind a wall goes through it, a ray from behind misses it, a
// sweep from behind misses it; the continuous collision stops a fast ball on its front
func TestMeshIsOneSided(t *testing.T) {
	// a wall at x = 0 facing -x
	wall, err := actor.NewTriangleMesh([]mgl64.Vec3{{0, -2, -2}, {0, 2, -2}, {0, 2, 2}, {0, -2, 2}}, []int32{0, 2, 1, 0, 3, 2})
	if err != nil {
		t.Fatal(err)
	}
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	wallBody := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), wall, actor.BodyTypeStatic, 0, 0)
	fast := addBody(w, mgl64.Vec3{-6, 0.3, 0.2}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.05}, actor.BodyTypeDynamic, 0, 0)
	fast.Velocity = mgl64.Vec3{200, 0, 0}
	slow := addBody(w, mgl64.Vec3{-1, -0.5, -0.5}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0, 0)
	slow.Velocity = mgl64.Vec3{3, 0, 0}
	behind := addBody(w, mgl64.Vec3{1, 0.5, 0.5}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeDynamic, 0, 0)
	behind.Velocity = mgl64.Vec3{-3, 0, 0}
	simulate(w, 0.5, nil)
	t.Logf("fast ball at x = %.4f, slow ball at x = %.4f, ball from behind at x = %.3f", fast.Transform.Position.X(), slow.Transform.Position.X(), behind.Transform.Position.X())
	if fast.Transform.Position.X() > 0 || fast.Transform.Position.X() < -0.05-landingDepth {
		t.Errorf("the fast ball is at x = %.4f, want on the wall", fast.Transform.Position.X())
	}
	if slow.Transform.Position.X() > 0 || slow.Transform.Position.X() < -0.1-landingDepth {
		t.Errorf("the slow ball is at x = %.4f, want on the wall", slow.Transform.Position.X())
	}
	if behind.Transform.Position.X() > -0.3 {
		t.Errorf("the ball from behind was stopped at x = %.3f", behind.Transform.Position.X())
	}

	w.SyncQueries()
	filter := DefaultQueryFilter()
	filter.Excluded = []*actor.RigidBody{fast, slow, behind}
	if hit, ok := w.Raycast(mgl64.Vec3{-1, 0.5, 0.5}, mgl64.Vec3{3, 0, 0}, filter); !ok || hit.Body != wallBody || math.Abs(hit.Fraction-1.0/3) > 1e-12 || hit.Normal != (mgl64.Vec3{-1, 0, 0}) || hit.Triangle != 0 {
		t.Errorf("ray on the front: %v %v", hit, ok)
	}
	if _, ok := w.Raycast(mgl64.Vec3{1, 0.5, 0.5}, mgl64.Vec3{-3, 0, 0}, filter); ok {
		t.Errorf("a ray from behind hits the wall")
	}
	ball := &actor.Sphere{Radius: 0.1}
	// (-0.5, 0.5) is in the triangle 1 of the wall, off its diagonal
	if hit, ok := w.Sweep(ball, actor.Transform{Position: mgl64.Vec3{-1, -0.5, 0.5}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{3, 0, 0}, filter); !ok || hit.Body != wallBody || math.Abs(hit.Fraction-0.3) > 1e-5 || hit.Triangle != 1 || hit.Point.Sub(mgl64.Vec3{0, -0.5, 0.5}).Len() > 1e-5 {
		t.Errorf("sweep on the front: %v %v", hit, ok)
	}
	if _, ok := w.Sweep(ball, actor.Transform{Position: mgl64.Vec3{1, 0.5, 0.5}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{-3, 0, 0}, filter); ok {
		t.Errorf("a sweep from behind hits the wall")
	}
	// an overlap sees both sides
	for _, x := range []float64{-0.05, 0.05} {
		if bodies := w.Overlap(ball, actor.Transform{Position: mgl64.Vec3{x, 0.5, 0.5}, Rotation: mgl64.QuatIdent()}, filter, nil); len(bodies) != 1 || bodies[0] != wallBody {
			t.Errorf("overlap at x = %v: %d bodies", x, len(bodies))
		}
	}
	if bodies := w.Overlap(ball, actor.Transform{Position: mgl64.Vec3{-0.2, 0.5, 0.5}, Rotation: mgl64.QuatIdent()}, filter, nil); len(bodies) != 0 {
		t.Errorf("overlap away from the wall: %d bodies", len(bodies))
	}
}

// A sweep on the stairs stops on the right tread, with its triangle; RaycastAll on a mesh gives one hit per body; a
// mesh as the moving shape of a sweep or of an overlap panics
func TestMeshQueries(t *testing.T) {
	const width, rise = 0.3, 0.17
	stairs := stairMesh(8, width, rise, 1)
	w := newScene(1)
	body := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), stairs, actor.BodyTypeStatic, 0.6, 0)
	w.SyncQueries()
	filter := DefaultQueryFilter()
	sole := &actor.Sphere{Radius: 0.08}
	for s := 0; s < 8; s++ {
		x := (float64(s) + 0.5) * width
		hit, ok := w.Sweep(sole, actor.Transform{Position: mgl64.Vec3{x, 3, 0.2}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{0, -4, 0}, filter)
		if !ok || hit.Body != body {
			t.Fatalf("step %d: no hit", s)
		}
		wantY := float64(s+1) * rise
		if math.Abs(3-4*hit.Fraction-0.08-wantY) > 1e-5 || math.Abs(hit.Point.Y()-wantY) > 1e-5 || hit.Normal.Sub(mgl64.Vec3{0, 1, 0}).Len() > 1e-6 {
			t.Errorf("step %d: stopped at %.5f, point %v, normal %v, want the tread at %.2f", s, 3-4*hit.Fraction, hit.Point, hit.Normal, wantY)
		}
		// the tread s is the quads 2s+1: triangles 4s+2 and 4s+3
		if hit.Triangle/4 != int32(s) || hit.Triangle%4 < 2 {
			t.Errorf("step %d: triangle %d, want one of the tread", s, hit.Triangle)
		}
		ray, ok := w.Raycast(mgl64.Vec3{x, 3, 0.2}, mgl64.Vec3{0, -4, 0}, filter)
		if !ok || math.Abs(ray.Point.Y()-wantY) > 1e-9 || ray.Triangle/4 != int32(s) {
			t.Errorf("step %d: ray %v %v", s, ray, ok)
		}
	}
	// a ray along the stairs, through 3 risers: the first one
	hit, ok := w.Raycast(mgl64.Vec3{-1, 2.5 * rise, 0}, mgl64.Vec3{4, 0, 0}, filter)
	if !ok || hit.Triangle/4 != 2 || hit.Triangle%4 > 1 || math.Abs(hit.Point.X()-2*width) > 1e-9 {
		t.Errorf("ray along the stairs: %v %v, want the riser of the step 2", hit, ok)
	}
	hits := w.RaycastAll(mgl64.Vec3{-1, 2.5 * rise, 0}, mgl64.Vec3{4, 0, 0}, filter, nil)
	if len(hits) != 1 {
		t.Errorf("RaycastAll gives %d hits on the mesh, want 1 (its first triangle)", len(hits))
	}

	for _, shape := range []actor.ShapeInterface{stairs, &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}} {
		func() {
			defer func() {
				if recover() == nil {
					t.Errorf("a sweep of a %T didn't panic", shape)
				}
			}()
			w.Sweep(shape, actor.NewTransform(), mgl64.Vec3{1, 0, 0}, filter)
		}()
		func() {
			defer func() {
				if recover() == nil {
					t.Errorf("an overlap of a %T didn't panic", shape)
				}
			}()
			w.Overlap(shape, actor.NewTransform(), filter, nil)
		}()
	}
}

// meshPile: bodies of every shape (boxes, spheres, capsules, hulls) dropped on a hilly mesh, one every 0.4 m of height
func meshPile(workers, count int) (*World, []*actor.RigidBody) {
	w := newScene(workers)
	w.parallelFrom = 1
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), gridMesh(40, 0.5, func(x, z float64) float64 { return 0.3 * math.Sin(x) * math.Cos(z*0.7) }), actor.BodyTypeStatic, 0.6, 0)
	r := rand.New(rand.NewSource(5))
	var bodies []*actor.RigidBody
	var points []mgl64.Vec3
	for i := 0; i < count; i++ {
		var shape actor.ShapeInterface = &actor.Box{HalfExtents: mgl64.Vec3{0.2, 0.15, 0.25}}
		switch i % 4 {
		case 1:
			shape = &actor.Sphere{Radius: 0.2}
		case 2:
			shape = &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}
		case 3:
			points = points[:0]
			for k := 0; k < 20; k++ {
				points = append(points, mgl64.Vec3{r.NormFloat64() * 0.2, r.NormFloat64() * 0.2, r.NormFloat64() * 0.2})
			}
			hull, err := actor.NewConvexHull(points, 32)
			if err != nil {
				panic(err)
			}
			shape = hull
		}
		position := mgl64.Vec3{(r.Float64() - 0.5) * 6, 1 + float64(i)*0.4, (r.Float64() - 0.5) * 6}
		rotation := mgl64.QuatRotate(r.Float64()*3, mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize())
		bodies = append(bodies, addBody(w, position, rotation, shape, actor.BodyTypeDynamic, 0.6, 0))
	}
	return w, bodies
}

// A pile of every shape on a mesh gives the same bits with 1 and 8 workers, and nothing falls through the mesh: 64
// bodies for 3 s (the last one, dropped from 26 m, lands within 2.3 s)
func TestMeshPileIsDeterministic(t *testing.T) {
	single, bodies := meshPile(1, 64)
	parallel, others := meshPile(8, 64)
	defer parallel.Close()
	for i := 0; i < 180; i++ {
		single.Step(sceneDt)
		parallel.Step(sceneDt)
	}
	for i := range bodies {
		if bodies[i].Transform != others[i].Transform {
			t.Fatalf("body %d: %v with 1 worker, %v with 8", i, bodies[i].Transform, others[i].Transform)
		}
		if !finite(bodies[i].Transform.Position) || bodies[i].Transform.Position.Y() < -1 {
			t.Errorf("body %d fell through the mesh: %v", i, bodies[i].Transform.Position)
		}
	}
}

// Criterion 5: a step with meshes and hulls allocates nothing once the buffers have grown, nor the queries on a mesh: a
// pile of 150 bodies after 4 s (a pile of 64 is still settling then, and its steps grow their buffers at times)
func TestMeshDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 4} {
		w, _ := meshPile(workers, 150)
		for i := 0; i < 240; i++ {
			w.Step(sceneDt)
		}
		if allocs := testing.AllocsPerRun(10, func() { w.Step(sceneDt) }); allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
		w.Close()
	}

	w := newScene(1)
	addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), stairMesh(8, 0.3, 0.17, 1), actor.BodyTypeStatic, 0.6, 0)
	w.SyncQueries()
	filter := DefaultQueryFilter()
	sole := &actor.Sphere{Radius: 0.08}
	at := actor.Transform{Position: mgl64.Vec3{1.35, 3, 0.2}, Rotation: mgl64.QuatIdent()}
	var bodies []*actor.RigidBody
	var hits []Hit
	found := 0
	allocs := testing.AllocsPerRun(10, func() {
		if _, ok := w.Raycast(at.Position, mgl64.Vec3{0, -4, 0}, filter); ok {
			found++
		}
		if _, ok := w.Sweep(sole, at, mgl64.Vec3{0, -4, 0}, filter); ok {
			found++
		}
		bodies = w.Overlap(sole, actor.Transform{Position: mgl64.Vec3{1.35, 0.9, 0.2}, Rotation: mgl64.QuatIdent()}, filter, bodies[:0])
		hits = w.RaycastAll(at.Position, mgl64.Vec3{0, -4, 0}, filter, hits[:0])
		found += len(bodies) + len(hits)
	})
	if allocs > 0 || found == 0 {
		t.Errorf("%.1f allocations for the queries on a mesh (%d hits), want 0", allocs, found)
	}
}

// ========== HULLS ==========

// cubeHull: the hull of the 8 corners of a cube
func cubeHull(half float64) *actor.ConvexHull {
	corners := make([]mgl64.Vec3, 0, 8)
	for i := 0; i < 8; i++ {
		corners = append(corners, mgl64.Vec3{float64(i&1)*2 - 1, float64(i>>1&1)*2 - 1, float64(i>>2&1)*2 - 1}.Mul(half))
	}
	hull, err := actor.NewConvexHull(corners, 8)
	if err != nil {
		panic(err)
	}
	return hull
}

// The hull of a cube rests on the ground and on a terrain as the Box does: the same depth within 0.1 mm, the same
// count of contact points, asleep
func TestHullCubeRestsLikeABox(t *testing.T) {
	for _, terrain := range []bool{false, true} {
		w := newScene(1)
		var surface *actor.RigidBody
		if terrain {
			surface = slopeTerrain(w, 0.1, 0.5)
		} else {
			surface = addGround(w, 0.5)
		}
		rotation := mgl64.QuatRotate(0.4, mgl64.Vec3{0, 1, 0})
		asHull := addBody(w, mgl64.Vec3{1, 1, 0}, rotation, cubeHull(cubeHalf), actor.BodyTypeDynamic, 0.5, 0)
		asBox := addBody(w, mgl64.Vec3{-1, 1, 0}, rotation, cube(), actor.BodyTypeDynamic, 0.5, 0)
		simulate(w, 3, nil)
		hullDepth, boxDepth := surfaceDepth(surface, asHull), surfaceDepth(surface, asBox)
		t.Logf("terrain %v: the hull rests %.4f mm deep, the box %.4f mm", terrain, hullDepth*1000, boxDepth*1000)
		if math.Abs(hullDepth-boxDepth) > 0.0001 || !asHull.IsSleeping {
			t.Errorf("terrain %v: the hull doesn't rest as the box", terrain)
		}
		if asHull.Material.GetMass() != asBox.Material.GetMass() {
			t.Errorf("mass %v, the box has %v", asHull.Material.GetMass(), asBox.Material.GetMass())
		}
		asHull.WakeUp()
		asBox.WakeUp()
		w.Step(sceneDt)
		hullPoints, boxPoints := 0, 0
		for _, m := range w.Contacts() {
			if m.BodyA == asHull || m.BodyB == asHull {
				hullPoints += m.Count
			}
			if m.BodyA == asBox || m.BodyB == asBox {
				boxPoints += m.Count
			}
		}
		if hullPoints != boxPoints || hullPoints < 4 {
			t.Errorf("terrain %v: the hull rests on %d points, the box on %d", terrain, hullPoints, boxPoints)
		}
	}
}

// Hulls dropped on hills and on stairs come to rest, none is lost; a hull of 32 vertices and one of 256 behave
func TestHullsRestOnSurfaces(t *testing.T) {
	r := rand.New(rand.NewSource(9))
	var points []mgl64.Vec3
	for _, stairs := range []bool{false, true} {
		w := newScene(1)
		if stairs {
			addGround(w, 0.6)
			addBody(w, mgl64.Vec3{-1.5, 0, 0}, mgl64.QuatIdent(), stairMesh(10, 0.3, 0.17, 3), actor.BodyTypeStatic, 0.6, 0)
		} else {
			bumpyTerrain(w, 4)
		}
		var hulls []*actor.RigidBody
		for i := 0; i < 20; i++ {
			points = points[:0]
			count, limit := 30, 32
			if i%5 == 0 {
				count, limit = 400, actor.MaxHullVertices
			}
			for k := 0; k < count; k++ {
				points = append(points, mgl64.Vec3{r.NormFloat64() * 0.2, r.NormFloat64() * 0.15, r.NormFloat64() * 0.2})
			}
			hull, err := actor.NewConvexHull(points, limit)
			if err != nil {
				t.Fatal(err)
			}
			position := mgl64.Vec3{(r.Float64() - 0.5) * 4, 3 + float64(i)*0.3, (r.Float64() - 0.5) * 4}
			rotation := mgl64.QuatRotate(r.Float64()*3, mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize())
			hulls = append(hulls, addBody(w, position, rotation, hull, actor.BodyTypeDynamic, 0.6, 0))
		}
		simulate(w, 8, nil)
		asleep := 0
		for i, h := range hulls {
			if h.IsSleeping {
				asleep++
			}
			if !finite(h.Transform.Position) || h.Transform.Position.Y() < -3 {
				t.Errorf("stairs %v: the hull %d is lost at %v", stairs, i, h.Transform.Position)
			}
		}
		t.Logf("stairs %v: %d/20 hulls asleep", stairs, asleep)
		if asleep < 18 {
			t.Errorf("stairs %v: only %d hulls asleep", stairs, asleep)
		}
	}
}

// A hull is swept and overlapped as any convex shape, and a ray hits it; a hull and a mesh collide through CollideAll
func TestHullQueries(t *testing.T) {
	w := newScene(1)
	hull := cubeHull(0.5)
	body := addBody(w, mgl64.Vec3{3, 0, 0}, mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 1, 0}), hull, actor.BodyTypeStatic, 0.5, 0)
	w.SyncQueries()
	filter := DefaultQueryFilter()
	// the corner of the turned cube is at x = 3 - sqrt(0.5)
	corner := 3 - math.Sqrt(0.5)
	if hit, ok := w.Raycast(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{4, 0, 0}, filter); !ok || hit.Body != body || math.Abs(hit.Point.X()-corner) > 1e-9 {
		t.Errorf("ray on the hull: %v %v", hit, ok)
	}
	if hit, ok := w.Sweep(&actor.Sphere{Radius: 0.1}, actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()}, mgl64.Vec3{4, 0, 0}, filter); !ok || math.Abs(hit.Point.X()-corner) > 1e-5 || math.Abs(4*hit.Fraction+0.1-corner) > 1e-5 {
		t.Errorf("sweep on the hull: %v %v", hit, ok)
	}
	if bodies := w.Overlap(hull, actor.Transform{Position: mgl64.Vec3{3.9, 0, 0}, Rotation: mgl64.QuatIdent()}, filter, nil); len(bodies) != 1 {
		t.Errorf("overlap of a hull: %d bodies", len(bodies))
	}
	if bodies := w.Overlap(hull, actor.Transform{Position: mgl64.Vec3{4.3, 0, 0}, Rotation: mgl64.QuatIdent()}, filter, nil); len(bodies) != 0 {
		t.Errorf("overlap of a hull away: %d bodies", len(bodies))
	}
}
