package feather

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/go-gl/mathgl/mgl64"
)

// Test helper functions
func createBox(position mgl64.Vec3, halfExtents mgl64.Vec3, bodyType actor.BodyType) *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: position, Rotation: mgl64.QuatIdent()},
		&actor.Box{HalfExtents: halfExtents},
		bodyType,
		1.0,
	)
}

func createSphere(position mgl64.Vec3, radius float64, bodyType actor.BodyType) *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: position, Rotation: mgl64.QuatIdent()},
		&actor.Sphere{Radius: radius},
		bodyType,
		1.0,
	)
}

func createPlane(normal mgl64.Vec3, distance float64) *actor.RigidBody {
	return actor.NewRigidBody(
		actor.Transform{Position: mgl64.Vec3{}, Rotation: mgl64.QuatIdent()},
		&actor.Plane{Normal: normal, Distance: distance},
		actor.BodyTypeStatic,
		0.0,
	)
}

// TestBroadPhaseNoBodies tests broad phase with no bodies
func TestBroadPhaseNoBodies(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}
	pairs := BroadPhase(world.Bodies, world.Workers)

	if len(pairs) != 0 {
		t.Errorf("BroadPhase with no bodies returned %d pairs, want 0", len(pairs))
	}
}

func TestBroadPhaseSingleBody(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.Bodies, world.Workers)

	if len(pairs) != 0 {
		t.Errorf("BroadPhase with single body returned %d pairs, want 0", len(pairs))
	}
}

func TestBroadPhaseTwoBodiesOverlapping(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	world.AddBody(createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with overlapping bodies returned %d pairs, want 1", len(pairs))
	}
	if contactPairs[0].BodyA == contactPairs[0].BodyB {
		t.Error("Collision pair bodies don't match expected bodies")
	}
}

func TestBroadPhaseTwoBodiesNotOverlapping(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	world.AddBody(createBox(mgl64.Vec3{10.0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	if len(contactPairs) != 0 {
		t.Errorf("BroadPhase with non-overlapping bodies returned %d pairs, want 0", len(pairs))
	}
}

func TestBroadPhaseTwoStaticBodies(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic))
	world.AddBody(createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic))
	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	// Static-static collisions should be skipped
	if len(contactPairs) != 0 {
		t.Errorf("BroadPhase with two static bodies returned %d pairs, want 0 (should skip static-static)", len(pairs))
	}
}

func TestBroadPhaseStaticDynamicOverlapping(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic))
	world.AddBody(createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)
	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with static-dynamic overlapping returned %d pairs, want 1", len(contactPairs))
	}
}

func TestBroadPhaseMultipleBodies(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}

	// Create bodies
	body0 := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)   // 0
	body1 := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic) // 1 - overlaps with 0
	body2 := createBox(mgl64.Vec3{3, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)   // 2 - overlaps with 1
	body3 := createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)  // 3 - no overlaps

	world.AddBody(body0)
	world.AddBody(body1)
	world.AddBody(body2)
	world.AddBody(body3)

	pairs := BroadPhase(world.Bodies, world.Workers)

	// Expected pairs: (0,1), (1,2)
	expectedPairs := 2
	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	if len(contactPairs) != expectedPairs {
		t.Errorf("BroadPhase returned %d pairs, want %d", len(contactPairs), expectedPairs)
	}

	// Verify that we have the right pairs
	pairMap := make(map[string]bool)
	for _, pair := range contactPairs {
		// Create a key from body indices
		var key string
		bodies := []*actor.RigidBody{body0, body1, body2, body3}
		for i, body := range bodies {
			if body == pair.BodyA {
				for j, bodyB := range bodies {
					if bodyB == pair.BodyB {
						if i < j {
							key = string(rune('0'+i)) + string(rune('0'+j))
						}
					}
				}
			}
		}
		if key != "" {
			pairMap[key] = true
		}
	}

	if len(pairMap) != expectedPairs {
		t.Logf("Found pairs: %v", pairMap)
	}
}

//
// TestBroadPhaseSpheresOverlapping tests overlapping spheres

func TestBroadPhaseSpheresOverlapping(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}

	world.AddBody(createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic))
	world.AddBody(createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with overlapping spheres returned %d pairs, want 1", len(contactPairs))
	}
}

//
// TestBroadPhaseSpheresNotOverlapping tests non-overlapping spheres

func TestBroadPhaseSpheresNotOverlapping(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}

	world.AddBody(createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic))
	world.AddBody(createSphere(mgl64.Vec3{3, 0, 0}, 1.0, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	if len(contactPairs) != 0 {
		t.Errorf("BroadPhase with non-overlapping spheres returned %d pairs, want 0", len(contactPairs))
	}
}

//
// TestBroadPhaseMixedShapes tests boxes and spheres together

func TestBroadPhaseMixedShapes(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}

	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	world.AddBody(createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with box-sphere overlapping returned %d pairs, want 1", len(contactPairs))
	}
}

//
// TestBroadPhaseWithPlane tests bodies overlapping with a plane

func TestBroadPhaseWithPlane(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}

	world.AddBody(createPlane(mgl64.Vec3{0, 1, 0}, 0)) // Ground plane at y=0
	world.AddBody(createBox(mgl64.Vec3{0, 0.5, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	// The box should overlap with the plane's AABB
	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with plane-box returned %d pairs, want 1", len(contactPairs))
	}
}

// TestNarrowPhaseNoPairs tests narrow phase with no pairs
func TestNarrowPhaseNoPairs(t *testing.T) {
	pairs := []Pair{}

	contacts := NarrowPhase(pairs, 8)

	if len(contacts) != 0 {
		t.Errorf("NarrowPhase with no pairs returned %d contacts, want 0", len(contacts))
	}
}

// //
// TestNarrowPhaseOverlappingBoxes tests narrow phase with overlapping boxes
func TestNarrowPhaseOverlappingBoxes(t *testing.T) {
	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)

	pairs := []Pair{Pair{BodyA: bodyA, BodyB: bodyB}}

	contacts := NarrowPhase(pairs, 8)

	// Should detect collision
	if len(contacts) == 0 {
		t.Error("NarrowPhase with overlapping boxes returned no contacts, expected at least 1")
	}
}

// //
// TestNarrowPhaseNonOverlappingBoxes tests narrow phase with non-overlapping boxes
func TestNarrowPhaseNonOverlappingBoxes(t *testing.T) {
	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	bodyB := createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)

	pairs := []Pair{Pair{BodyA: bodyA, BodyB: bodyB}}

	contacts := NarrowPhase(pairs, 8)

	// Should not detect collision
	if len(contacts) != 0 {
		t.Errorf("NarrowPhase with non-overlapping boxes returned %d contacts, want 0", len(contacts))
	}
}

// //
// TestNarrowPhaseOverlappingSpheres tests narrow phase with overlapping spheres
func TestNarrowPhaseOverlappingSpheres(t *testing.T) {
	bodyA := createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic)
	bodyB := createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic)

	pairs := []Pair{Pair{BodyA: bodyA, BodyB: bodyB}}

	contacts := NarrowPhase(pairs, 8)

	// Should detect collision
	if len(contacts) == 0 {
		t.Error("NarrowPhase with overlapping spheres returned no contacts, expected at least 1")
	}
}

// //
// TestNarrowPhaseNonOverlappingSpheres tests narrow phase with non-overlapping spheres
func TestNarrowPhaseNonOverlappingSpheres(t *testing.T) {
	bodyA := createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic)
	bodyB := createSphere(mgl64.Vec3{5, 0, 0}, 1.0, actor.BodyTypeDynamic)

	pairs := []Pair{Pair{BodyA: bodyA, BodyB: bodyB}}

	contacts := NarrowPhase(pairs, 8)

	// Should not detect collision
	if len(contacts) != 0 {
		t.Errorf("NarrowPhase with non-overlapping spheres returned %d contacts, want 0", len(contacts))
	}
}

// //
// TestNarrowPhaseBoxSphere tests narrow phase with box and sphere
func TestNarrowPhaseBoxSphere(t *testing.T) {
	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	bodyB := createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic)

	pairs := []Pair{Pair{BodyA: bodyA, BodyB: bodyB}}

	contacts := NarrowPhase(pairs, 8)

	// Should detect collision
	if len(contacts) == 0 {
		t.Error("NarrowPhase with overlapping box-sphere returned no contacts, expected at least 1")
	}
}

// //
// TestNarrowPhaseSphereOnPlane tests narrow phase with sphere resting on plane
func TestNarrowPhaseSphereOnPlane(t *testing.T) {
	bodyA := createPlane(mgl64.Vec3{0, 1, 0}, 0)
	bodyB := createSphere(mgl64.Vec3{0, 0.5, 0}, 1.0, actor.BodyTypeDynamic)

	pairs := []Pair{Pair{BodyA: bodyA, BodyB: bodyB}}

	contacts := NarrowPhase(pairs, 8)

	// Should detect collision (sphere penetrating plane)
	if len(contacts) == 0 {
		t.Error("NarrowPhase with sphere on plane returned no contacts, expected at least 1")
	}
}

// //
// TestNarrowPhaseBoxOnPlane tests narrow phase with box resting on plane
func TestNarrowPhaseBoxOnPlane(t *testing.T) {
	bodyA := createPlane(mgl64.Vec3{0, 1, 0}, 0)
	bodyB := createBox(mgl64.Vec3{0, 0.5, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)

	pairs := []Pair{Pair{BodyA: bodyA, BodyB: bodyB}}

	contacts := NarrowPhase(pairs, 8)

	// Should detect collision (box penetrating plane)
	if len(contacts) == 0 {
		t.Error("NarrowPhase with box on plane returned no contacts, expected at least 1")
	}
}

// //
// TestNarrowPhaseMultiplePairs tests narrow phase with multiple collision pairs
func TestNarrowPhaseMultiplePairs(t *testing.T) {
	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	bodyC := createSphere(mgl64.Vec3{3, 0, 0}, 1.0, actor.BodyTypeDynamic)
	bodyD := createSphere(mgl64.Vec3{4, 0, 0}, 1.0, actor.BodyTypeDynamic)

	pairs := []Pair{
		{BodyA: bodyA, BodyB: bodyB}, // Should collide
		{BodyA: bodyC, BodyB: bodyD}, // Should collide
	}

	contacts := NarrowPhase(pairs, 8)

	// Should detect both collisions
	if len(contacts) < 2 {
		t.Errorf("NarrowPhase with 2 overlapping pairs returned %d contacts, want at least 2", len(contacts))
	}
}

//
// TestCollisionPairStruct tests the CollisionPair struct

func TestCollisionPairStruct(t *testing.T) {
	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)

	pair := Pair{
		BodyA: bodyA,
		BodyB: bodyB,
	}

	if pair.BodyA != bodyA {
		t.Error("Pair.BodyA doesn't match expected body")
	}
	if pair.BodyB != bodyB {
		t.Error("Pair.BodyB doesn't match expected body")
	}
}

//
// TestIntegrationBroadAndNarrowPhase tests the complete collision detection pipeline

func TestIntegrationBroadAndNarrowPhase(t *testing.T) {
	world := World{
		Bodies:  []*actor.RigidBody{},
		Workers: 8,
	}

	body0 := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	body1 := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	body2 := createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)

	world.AddBody(body0)
	world.AddBody(body1)
	world.AddBody(body2)

	// Broad phase
	pairs := BroadPhase(world.Bodies, world.Workers)

	var contactPairs []Pair
	contactPairs = append(contactPairs, pairs...)

	if len(contactPairs) == 0 {
		t.Fatal("BroadPhase returned no pairs, expected at least 1")
	}

	contacts := NarrowPhase(contactPairs, 8)

	if len(contacts) == 0 {
		t.Error("NarrowPhase returned no contacts, expected at least 1")
	}

	// Verify number of contacts matches number of actual collisions
	// bodies[0] and bodies[1] should collide
	// bodies[2] is far away and should not collide
	if len(contacts) != 1 {
		t.Errorf("Expected 1 contact from integration test, got %d", len(contacts))
	}
}

// BenchmarkLargeBroadPhase2-16    	    1315	   1110795 ns/op	    9035 B/op	     132 allocs/op
// BenchmarkLargeBroadPhase2-16    	     643	   1786301 ns/op	    3034 B/op	      24 allocs/op
// BenchmarkLargeBroadPhase2-16    	    4130	    330082 ns/op	   18882 B/op	      36 allocs/op
// BenchmarkLargeBroadPhase2-16    	    4173	    322531 ns/op	   11723 B/op	      29 allocs/op
// BenchmarkLargeBroadPhase2-16    	    4602	    278210 ns/op	   11714 B/op	      29 allocs/op
func BenchmarkLargeBroadPhase2(b *testing.B) {
	const cubesCount = 1000
	const rowSize = 100.0

	world := World{
		Gravity:  mgl64.Vec3{},
		Substeps: 20,
	}

	r := rand.New(rand.NewSource(0))
	for i := 0; i < cubesCount; i++ {
		x := 0.0
		y := r.Float64() * rowSize
		z := r.Float64() * rowSize

		world.AddBody(createBox(mgl64.Vec3{x, y, z}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	}

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		pair := BroadPhase(world.Bodies, world.Workers)

		for _, p := range pair {
			p.BodyA.IsSleeping = true
		}
	}
}

// largeOverlappingWorld is 1000 unit boxes on a 0.9 m grid: every neighbour overlaps.
func largeOverlappingWorld(workers int) *World {
	const cubesCount = 1000
	const rowSize = 100

	world := &World{
		Substeps: 20,
		Workers:  workers,
		Events:   NewEvents(),
	}
	for i := 0; i < cubesCount; i++ {
		row, col := i/rowSize, i%rowSize
		world.AddBody(createBox(mgl64.Vec3{0, float64(row) * 0.9, float64(col) * 0.9}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	}
	return world
}

// BenchmarkLargeNarrowPhase measures GJK/EPA and manifold generation on ~4000 overlapping
// box pairs. Profile with go test -bench LargeNarrowPhase -cpuprofile cpu.prof.
func BenchmarkLargeNarrowPhase(b *testing.B) {
	world := largeOverlappingWorld(8)
	pairs := BroadPhase(world.Bodies, world.Workers)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		NarrowPhase(pairs, world.Workers)
	}
}

// BenchmarkLargeWorldStep measures a whole step of 1000 overlapping boxes, 20 sub-steps.
func BenchmarkLargeWorldStep(b *testing.B) {
	world := largeOverlappingWorld(8)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		world.Step(1.0 / 60.0)
	}
}

// Against a plane, the contact keeps the deepest point of the shape, whatever its rotation
func TestPlaneContactKeepsDeepestPoint(t *testing.T) {
	r := rand.New(rand.NewSource(1))
	plane := createPlane(mgl64.Vec3{0, 1, 0}, 0)
	for i := 0; i < 500; i++ {
		rotation := mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{r.Float64() - 0.5, r.Float64() - 0.5, r.Float64() - 0.5}.Normalize())
		box := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{0, 0.25, 0}, Rotation: rotation}, &actor.Box{HalfExtents: mgl64.Vec3{0.2, 0.15, 0.25}}, actor.BodyTypeDynamic, 1)
		var m constraint.Manifold
		if !Collide(plane, box, 0.4, &m) {
			t.Fatal("no contact")
		}
		lowest := box.Transform.ToWorld(box.Shape.Support(rotation.Conjugate().Rotate(mgl64.Vec3{0, -1, 0}))).Y()
		if math.Abs(m.MinSeparation()-lowest) > 1e-9 {
			t.Fatalf("rotation %v: deepest point %.4f, the contact has %.4f", rotation, lowest, m.MinSeparation())
		}
	}
}
