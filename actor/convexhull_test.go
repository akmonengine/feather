package actor

import (
	"errors"
	"math"
	"math/rand"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// cubeCorners of half size half, around the center
func cubeCorners(half float64, center mgl64.Vec3) []mgl64.Vec3 {
	corners := make([]mgl64.Vec3, 0, 8)
	for i := 0; i < 8; i++ {
		corners = append(corners, mgl64.Vec3{float64(i&1)*2 - 1, float64(i>>1&1)*2 - 1, float64(i>>2&1)*2 - 1}.Mul(half).Add(center))
	}
	return corners
}

// hullEdges: the count of edges of the hull, each shared by 2 faces
func hullEdges(h *ConvexHull) int {
	sides := 0
	for i := 0; i < h.FaceCount(); i++ {
		_, _, vertices := h.Face(i)
		sides += len(vertices)
	}
	return sides / 2
}

// checkHull: the hull is convex and closed: every point of the cloud is behind every face (within slack), every vertex
// is a point of the cloud, every face is a polygon turning counterclockwise around its normal, and V - E + F = 2
func checkHull(t *testing.T, h *ConvexHull, cloud []mgl64.Vec3, slack float64) {
	t.Helper()
	offset := h.CenterOfMass()
	for i := 0; i < h.FaceCount(); i++ {
		normal, distance, vertices := h.Face(i)
		if len(vertices) < 3 {
			t.Fatalf("face %d has %d vertices", i, len(vertices))
		}
		if math.Abs(normal.Len()-1) > 1e-12 {
			t.Fatalf("face %d: normal %v is not unit", i, normal)
		}
		for _, p := range cloud {
			if separation := p.Sub(offset).Dot(normal) + distance; separation > slack {
				t.Fatalf("face %d: the point %v is %.3g out of the hull", i, p, separation)
			}
		}
		for k, v := range vertices {
			if separation := h.Points[v].Dot(normal) + distance; math.Abs(separation) > slack {
				t.Fatalf("face %d: its vertex %d is %.3g off its plane", i, k, separation)
			}
			// the polygon turns counterclockwise seen from outside: each corner turns towards the inside
			previous, current, next := h.Points[vertices[(k+len(vertices)-1)%len(vertices)]], h.Points[v], h.Points[vertices[(k+1)%len(vertices)]]
			if current.Sub(previous).Cross(next.Sub(current)).Dot(normal) < 0 {
				t.Fatalf("face %d turns clockwise at its vertex %d", i, k)
			}
		}
	}
	for i, p := range h.Points {
		found := false
		for _, c := range cloud {
			if c.Sub(offset).Sub(p).Len() <= slack {
				found = true
				break
			}
		}
		if !found {
			t.Fatalf("the vertex %d (%v) is not a point of the cloud", i, p.Add(offset))
		}
	}
	if euler := len(h.Points) - hullEdges(h) + h.FaceCount(); euler != 2 {
		t.Fatalf("V - E + F = %d - %d + %d = %d, want 2", len(h.Points), hullEdges(h), h.FaceCount(), euler)
	}
	// the support point is the furthest point of the cloud
	r := rand.New(rand.NewSource(int64(len(cloud))))
	for i := 0; i < 200; i++ {
		direction := mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}
		support := h.Support(direction).Dot(direction)
		for _, p := range cloud {
			if dot := p.Sub(offset).Dot(direction); dot > support+slack*direction.Len() {
				t.Fatalf("direction %v: the support %.9f is under the point %v (%.9f)", direction, support, p, dot)
			}
		}
	}
}

// Criterion 1: the hull of the corners of a cube, with points inside it, on its faces and on its edges, is the cube:
// 8 vertices, 6 faces of 4 vertices, the center of mass at its center; its mass and its inertia are those of the Box,
// within 1e-6
func TestConvexHullOfANoisyCubeIsTheCube(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	center := mgl64.Vec3{1, 2, 3}
	half := mgl64.Vec3{0.3, 0.5, 0.2}
	var points []mgl64.Vec3
	for i := 0; i < 8; i++ {
		points = append(points, mgl64.Vec3{(float64(i&1)*2 - 1) * half[0], (float64(i>>1&1)*2 - 1) * half[1], (float64(i>>2&1)*2 - 1) * half[2]}.Add(center))
	}
	for i := 0; i < 1000; i++ {
		points = append(points, mgl64.Vec3{(r.Float64()*2 - 1) * half[0], (r.Float64()*2 - 1) * half[1], (r.Float64()*2 - 1) * half[2]}.Add(center))
	}
	// on the faces and on the edges, exactly
	points = append(points, center.Add(mgl64.Vec3{half[0], 0, 0}), center.Add(mgl64.Vec3{0, -half[1], 0.1}), center.Add(mgl64.Vec3{half[0], half[1], 0}), center.Add(mgl64.Vec3{-half[0], 0.2, half[2]}))
	r.Shuffle(len(points), func(i, j int) { points[i], points[j] = points[j], points[i] })

	h, err := NewConvexHull(points, MaxHullVertices)
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Points) != 8 || h.FaceCount() != 6 {
		t.Fatalf("%d vertices and %d faces, want 8 and 6", len(h.Points), h.FaceCount())
	}
	for i := 0; i < h.FaceCount(); i++ {
		if _, _, vertices := h.Face(i); len(vertices) != 4 {
			t.Errorf("face %d has %d vertices, want 4", i, len(vertices))
		}
	}
	if gap := h.CenterOfMass().Sub(center).Len(); gap > 1e-12 {
		t.Errorf("center of mass %v, want %v", h.CenterOfMass(), center)
	}
	checkHull(t, h, points, 1e-12)

	box := Box{HalfExtents: half}
	if mass, want := h.ComputeMass(700), box.ComputeMass(700); math.Abs(mass-want) > 1e-6*want {
		t.Errorf("mass %.9f, want %.9f", mass, want)
	}
	inertia, want := h.ComputeInertia(50), box.ComputeInertia(50)
	for i := range inertia {
		if math.Abs(inertia[i]-want[i]) > 1e-6*want[0] {
			t.Errorf("inertia %v, want %v", inertia, want)
			break
		}
	}
	// the bounds are the half extents, the support points the corners
	if aabb := h.ComputeAABB(NewTransform()); aabb.Min.Add(half).Len() > 1e-12 || aabb.Max.Sub(half).Len() > 1e-12 {
		t.Errorf("AABB %v, want ±%v", aabb, half)
	}
	if support := h.Support(mgl64.Vec3{1, -1, 1}); support.Sub(mgl64.Vec3{half[0], -half[1], half[2]}).Len() > 1e-12 {
		t.Errorf("support %v, want a corner", support)
	}
}

// The hull of a random cloud contains all its points, its vertices are points of the cloud, it is convex and closed; a
// cloud far from the origin and a cloud on a sphere too. The hull of 250 points on a sphere has them all as vertices,
// its volume is the one of the ball within 15 %, below it, its center of mass and its inertia are the ones of the ball
func TestConvexHullOfRandomClouds(t *testing.T) {
	r := rand.New(rand.NewSource(2))
	for i := 0; i < 20; i++ {
		count := 10 + r.Intn(300)
		center := mgl64.Vec3{r.NormFloat64() * 100, r.NormFloat64() * 100, r.NormFloat64() * 100}
		cloud := make([]mgl64.Vec3, count)
		for k := range cloud {
			cloud[k] = mgl64.Vec3{r.NormFloat64() * 0.5, r.NormFloat64() * 0.2, r.NormFloat64()}.Add(center)
		}
		h, err := NewConvexHull(cloud, MaxHullVertices)
		if err != nil {
			t.Fatalf("cloud %d: %v", i, err)
		}
		checkHull(t, h, cloud, 1e-9)
		if h.Volume() <= 0 {
			t.Errorf("cloud %d: volume %v", i, h.Volume())
		}
	}

	cloud := make([]mgl64.Vec3, 250)
	for k := range cloud {
		cloud[k] = mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize().Mul(2)
	}
	h, err := NewConvexHull(cloud, MaxHullVertices)
	if err != nil {
		t.Fatal(err)
	}
	checkHull(t, h, cloud, 1e-9)
	ball := 4.0 / 3 * math.Pi * 8
	t.Logf("sphere of 250 points: %d vertices, %d faces, volume %.4f (ball %.4f)", len(h.Points), h.FaceCount(), h.Volume(), ball)
	if len(h.Points) != 250 {
		t.Errorf("%d vertices, want the 250 points of the sphere", len(h.Points))
	}
	if h.Volume() > ball || h.Volume() < 0.85*ball {
		t.Errorf("volume %.4f, want the ball %.4f within 15 %%, below it", h.Volume(), ball)
	}
	if h.CenterOfMass().Len() > 0.05 {
		t.Errorf("center of mass %v, want the center of the sphere", h.CenterOfMass())
	}
	// the inertia of the ball: 2/5 m r², within 15 %
	inertia := h.ComputeInertia(1)
	if want := 0.4 * 4; math.Abs(inertia[0]-want) > 0.15*want || math.Abs(inertia[4]-want) > 0.15*want || math.Abs(inertia[8]-want) > 0.15*want {
		t.Errorf("inertia %v, want %.3f on the diagonal", inertia, want)
	}
}

// The vertex limit stops the hull: at most this many vertices, the hull inside the full one, and the points the
// furthest out added first: the hull of 32 vertices of a cloud on a sphere keeps 80 % of the volume of the full one.
// A limit out of [4, MaxHullVertices] is refused
func TestConvexHullVertexLimit(t *testing.T) {
	r := rand.New(rand.NewSource(3))
	cloud := make([]mgl64.Vec3, 500)
	for k := range cloud {
		cloud[k] = mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize()
	}
	full, err := NewConvexHull(cloud, MaxHullVertices)
	if err != nil {
		t.Fatal(err)
	}
	limited, err := NewConvexHull(cloud, 32)
	if err != nil {
		t.Fatal(err)
	}
	if len(limited.Points) > 32 || len(limited.Points) < 30 {
		t.Errorf("%d vertices, want 32", len(limited.Points))
	}
	for i := 0; i < full.FaceCount(); i++ {
		normal, distance, _ := full.Face(i)
		for _, p := range limited.Points {
			if separation := p.Add(limited.CenterOfMass()).Sub(full.CenterOfMass()).Dot(normal) + distance; separation > 1e-9 {
				t.Fatalf("a vertex of the limited hull is %.3g out of the full one", separation)
			}
		}
	}
	t.Logf("32 vertices: %d faces, volume %.4f of %.4f", limited.FaceCount(), limited.Volume(), full.Volume())
	if limited.Volume() < 0.8*full.Volume() {
		t.Errorf("the limited hull keeps %.0f %% of the volume, want 80 %%", 100*limited.Volume()/full.Volume())
	}
	for _, limit := range []int{0, 3, MaxHullVertices + 1} {
		if _, err := NewConvexHull(cloud, limit); !errors.Is(err, ErrHullVertexLimit) {
			t.Errorf("limit %d: %v, want ErrHullVertexLimit", limit, err)
		}
	}
}

// Fewer than 4 points, points on a line or on a plane: no hull
func TestConvexHullDegenerate(t *testing.T) {
	cases := map[string][]mgl64.Vec3{
		"3 points": {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
		"a line":   {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {-1, 0, 0}},
		"a plane":  {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0.3, 0.7, 0}, {2, -1, 0}},
		"a point":  {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}},
	}
	for name, points := range cases {
		if _, err := NewConvexHull(points, 64); !errors.Is(err, ErrHullDegenerate) {
			t.Errorf("%s: %v, want ErrHullDegenerate", name, err)
		}
	}
	// a tetrahedron is the smallest hull
	h, err := NewConvexHull([]mgl64.Vec3{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, 4)
	if err != nil || len(h.Points) != 4 || h.FaceCount() != 4 {
		t.Fatalf("tetrahedron: %v, %d vertices, %d faces", err, len(h.Points), h.FaceCount())
	}
	if math.Abs(h.Volume()-1.0/6) > 1e-12 {
		t.Errorf("volume %.9f, want 1/6", h.Volume())
	}
}

// A ray on the hull of a cube gives the hit of the Box: the planes of the faces. A ray from inside starts inside
func TestConvexHullCastRayIsTheBox(t *testing.T) {
	half := mgl64.Vec3{0.3, 0.5, 0.2}
	corners := cubeCorners(1, mgl64.Vec3{})
	for i := range corners {
		corners[i] = mgl64.Vec3{corners[i][0] * half[0], corners[i][1] * half[1], corners[i][2] * half[2]}
	}
	h, err := NewConvexHull(corners, 8)
	if err != nil {
		t.Fatal(err)
	}
	box := Box{HalfExtents: half}
	r := rand.New(rand.NewSource(4))
	hits := 0
	for i := 0; i < 2000; i++ {
		origin := mgl64.Vec3{r.NormFloat64(), r.NormFloat64(), r.NormFloat64()}.Normalize().Mul(1 + r.Float64())
		target := mgl64.Vec3{r.NormFloat64() * 0.4, r.NormFloat64() * 0.4, r.NormFloat64() * 0.4}
		translation := target.Sub(origin).Mul(0.5 + r.Float64())
		want, wantOk := box.CastRay(origin, translation, 1)
		hit, ok := h.CastRay(origin, translation, 1)
		if ok != wantOk {
			t.Fatalf("ray %d: hit %v, the box says %v", i, ok, wantOk)
		}
		if !ok {
			continue
		}
		hits++
		if math.Abs(hit.Fraction-want.Fraction) > 1e-12 || hit.Normal.Sub(want.Normal).Len() > 1e-12 || hit.Triangle != NoTriangle {
			t.Fatalf("ray %d: %v, the box says %v", i, hit, want)
		}
	}
	if hits < 500 {
		t.Fatalf("only %d hits", hits)
	}
	if hit, ok := h.CastRay(mgl64.Vec3{0.1, 0.1, 0.1}, mgl64.Vec3{1, 0, 0}, 1); !ok || hit.Fraction != 0 || hit.Normal != (mgl64.Vec3{-1, 0, 0}) {
		t.Errorf("from inside: %v %v, want the fraction 0 against the ray", hit, ok)
	}
	if _, ok := h.CastRay(mgl64.Vec3{1, 0, 0}, mgl64.Vec3{0, 1, 0}, 1); ok {
		t.Errorf("a ray beside the hull hits it")
	}
	if _, ok := h.CastRay(mgl64.Vec3{1, 0, 0}, mgl64.Vec3{-0.5, 0, 0}, 1); ok {
		t.Errorf("a ray stopping before the hull hits it")
	}
}

// The contact feature is the face the most aligned with the direction, and a face of more than 8 vertices gives 8 of
// them around the polygon; the points against a plane are the vertices of the supporting face, as for the Box
func TestConvexHullFeatures(t *testing.T) {
	var points []mgl64.Vec3
	for i := 0; i < 16; i++ {
		a := float64(i) / 16 * 2 * math.Pi
		points = append(points, mgl64.Vec3{0.3 * math.Cos(a), -0.2, 0.3 * math.Sin(a)}, mgl64.Vec3{0.3 * math.Cos(a), 0.2, 0.3 * math.Sin(a)})
	}
	cylinder, err := NewConvexHull(points, 64)
	if err != nil {
		t.Fatal(err)
	}
	if len(cylinder.Points) != 32 || cylinder.FaceCount() != 18 {
		t.Fatalf("%d vertices, %d faces, want 32 and 18", len(cylinder.Points), cylinder.FaceCount())
	}
	var feature [8]mgl64.Vec3
	var count int
	cylinder.GetContactFeature(mgl64.Vec3{0, -1, 0}, &feature, &count)
	if count != 8 {
		t.Fatalf("%d vertices of the bottom cap, want 8", count)
	}
	for k, p := range feature[:count] {
		if p[1] != -0.2 {
			t.Errorf("vertex %d of the bottom cap at %v", k, p)
		}
		// one vertex out of 2, around the cap: the neighbors are 45° apart
		if k > 0 {
			if cos := feature[k-1].Sub(mgl64.Vec3{0, -0.2, 0}).Normalize().Dot(p.Sub(mgl64.Vec3{0, -0.2, 0}).Normalize()); math.Abs(cos-math.Cos(math.Pi/4)) > 1e-9 {
				t.Errorf("vertices %d and %d of the cap are at %.1f°, want 45", k-1, k, math.Acos(cos)*180/math.Pi)
			}
		}
	}
	cylinder.GetContactFeature(mgl64.Vec3{1, 0.1, 0}, &feature, &count)
	if count != 4 {
		t.Errorf("%d vertices of a side, want 4", count)
	}

	// against a plane: the 4 corners of a face of a cube, the bottom ones only
	cube, err := NewConvexHull(cubeCorners(0.25, mgl64.Vec3{}), 8)
	if err != nil {
		t.Fatal(err)
	}
	contacts := cube.CollideWithPlane(mgl64.Vec3{0, 1, 0}, 0, Transform{Position: mgl64.Vec3{0, 0.26, 0}, Rotation: mgl64.QuatIdent()}, 1, nil)
	if len(contacts) != 4 {
		t.Fatalf("%d points against the plane, want 4", len(contacts))
	}
	for _, c := range contacts {
		if math.Abs(c.Separation-0.01) > 1e-12 || math.Abs(c.Position[1]-0.005) > 1e-12 {
			t.Errorf("point %v", c)
		}
	}
	// the AABB of a turned hull contains its turned vertices
	transform := Transform{Position: mgl64.Vec3{1, 2, 3}, Rotation: mgl64.QuatRotate(0.7, mgl64.Vec3{1, 2, 0.5}.Normalize())}
	aabb := cylinder.ComputeAABB(transform)
	for _, p := range cylinder.Points {
		if !aabb.ContainsPoint(transform.ToWorld(p)) {
			t.Fatalf("the vertex %v is out of the AABB %v", transform.ToWorld(p), aabb)
		}
	}
}

// A hull is built by NewRigidBody as any shape: the mass and the inertia of its volume
func TestConvexHullBody(t *testing.T) {
	h, err := NewConvexHull(cubeCorners(0.5, mgl64.Vec3{3, 3, 3}), 8)
	if err != nil {
		t.Fatal(err)
	}
	body := NewRigidBody(Transform{Position: h.CenterOfMass(), Rotation: mgl64.QuatIdent()}, h, BodyTypeDynamic, 1000)
	if math.Abs(body.Material.GetMass()-1000) > 1e-6 {
		t.Errorf("mass %v, want 1000", body.Material.GetMass())
	}
	if want := 1000.0 / 6; math.Abs(body.InertiaLocal[0]-want) > 1e-6 || math.Abs(body.InverseInertiaLocal[0]-1/want) > 1e-9 {
		t.Errorf("inertia %v, want %.3f", body.InertiaLocal, want)
	}
	if aabb := body.AABB(); aabb.Min != (mgl64.Vec3{2.5, 2.5, 2.5}) || aabb.Max != (mgl64.Vec3{3.5, 3.5, 3.5}) {
		t.Errorf("AABB %v", aabb)
	}
}
