package feather

import (
	"math/rand"
	"os"
	"runtime/pprof"
	"runtime/trace"
	"testing"

	"github.com/akmonengine/feather/actor"
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
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}
	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	if len(pairs) != 0 {
		t.Errorf("BroadPhase with no bodies returned %d pairs, want 0", len(pairs))
	}
}

func TestBroadPhaseSingleBody(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	if len(pairs) != 0 {
		t.Errorf("BroadPhase with single body returned %d pairs, want 0", len(pairs))
	}
}

func TestBroadPhaseTwoBodiesOverlapping(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	world.AddBody(createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with overlapping bodies returned %d pairs, want 1", len(pairs))
	}
	if contactPairs[0].BodyA == contactPairs[0].BodyB {
		t.Error("Collision pair bodies don't match expected bodies")
	}
}

func TestBroadPhaseTwoBodiesNotOverlapping(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	world.AddBody(createBox(mgl64.Vec3{10.0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	if len(contactPairs) != 0 {
		t.Errorf("BroadPhase with non-overlapping bodies returned %d pairs, want 0", len(pairs))
	}
}

func TestBroadPhaseTwoStaticBodies(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic))
	world.AddBody(createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic))
	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	// Static-static collisions should be skipped
	if len(pairs) != 0 {
		t.Errorf("BroadPhase with two static bodies returned %d pairs, want 0 (should skip static-static)", len(pairs))
	}
}

func TestBroadPhaseStaticDynamicOverlapping(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}
	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic))
	world.AddBody(createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}
	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with static-dynamic overlapping returned %d pairs, want 1", len(contactPairs))
	}
}

func TestBroadPhaseMultipleBodies(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
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

	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	// Expected pairs: (0,1), (1,2)
	expectedPairs := 2
	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

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
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}

	world.AddBody(createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic))
	world.AddBody(createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with overlapping spheres returned %d pairs, want 1", len(contactPairs))
	}
}

//
// TestBroadPhaseSpheresNotOverlapping tests non-overlapping spheres

func TestBroadPhaseSpheresNotOverlapping(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}

	world.AddBody(createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic))
	world.AddBody(createSphere(mgl64.Vec3{3, 0, 0}, 1.0, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	if len(contactPairs) != 0 {
		t.Errorf("BroadPhase with non-overlapping spheres returned %d pairs, want 0", len(contactPairs))
	}
}

//
// TestBroadPhaseMixedShapes tests boxes and spheres together

func TestBroadPhaseMixedShapes(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}

	world.AddBody(createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	world.AddBody(createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with box-sphere overlapping returned %d pairs, want 1", len(contactPairs))
	}
}

//
// TestBroadPhaseWithPlane tests bodies overlapping with a plane

func TestBroadPhaseWithPlane(t *testing.T) {
	world := World{
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}

	world.AddBody(createPlane(mgl64.Vec3{0, 1, 0}, 0)) // Ground plane at y=0
	world.AddBody(createBox(mgl64.Vec3{0, 0.5, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))

	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	// The box should overlap with the plane's AABB
	if len(contactPairs) != 1 {
		t.Errorf("BroadPhase with plane-box returned %d pairs, want 1", len(contactPairs))
	}
}

// TestNarrowPhaseNoPairs tests narrow phase with no pairs
func TestNarrowPhaseNoPairs(t *testing.T) {
	pairs := make(chan Pair)
	close(pairs) // Close immediately to signal no more pairs

	contacts := NarrowPhase(pairs)

	if len(contacts) != 0 {
		t.Errorf("NarrowPhase with no pairs returned %d contacts, want 0", len(contacts))
	}
}

// //
// TestNarrowPhaseOverlappingBoxes tests narrow phase with overlapping boxes
func TestNarrowPhaseOverlappingBoxes(t *testing.T) {
	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)

	pairs := make(chan Pair, 1)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB}
	close(pairs)

	contacts := NarrowPhase(pairs)

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

	pairs := make(chan Pair, 1)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB}
	close(pairs)

	contacts := NarrowPhase(pairs)

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

	pairs := make(chan Pair, 1)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB}
	close(pairs)

	contacts := NarrowPhase(pairs)

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

	pairs := make(chan Pair, 1)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB}
	close(pairs)

	contacts := NarrowPhase(pairs)

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

	pairs := make(chan Pair, 1)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB}
	close(pairs)

	contacts := NarrowPhase(pairs)

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

	pairs := make(chan Pair, 1)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB}
	close(pairs)

	contacts := NarrowPhase(pairs)

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

	pairs := make(chan Pair, 1)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB}
	close(pairs)

	contacts := NarrowPhase(pairs)

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

	pairs := make(chan Pair, 2)
	pairs <- Pair{BodyA: bodyA, BodyB: bodyB} // Should collide
	pairs <- Pair{BodyA: bodyC, BodyB: bodyD} // Should collide
	close(pairs)

	contacts := NarrowPhase(pairs)

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
		Bodies:      []*actor.RigidBody{},
		SpatialGrid: NewSpatialGrid(1.0, 1024),
	}

	body0 := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	body1 := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
	body2 := createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)

	world.AddBody(body0)
	world.AddBody(body1)
	world.AddBody(body2)

	// Broad phase
	pairs := BroadPhase(world.SpatialGrid, world.Bodies)

	var contactPairs []Pair
	for p := range pairs {
		contactPairs = append(contactPairs, p)
	}

	if len(contactPairs) == 0 {
		t.Fatal("BroadPhase returned no pairs, expected at least 1")
	}

	// Narrow phase - convert slice to channel
	pairChan := make(chan Pair, len(contactPairs))
	for _, pair := range contactPairs {
		pairChan <- pair
	}
	close(pairChan)

	contacts := NarrowPhase(pairChan)

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
		Gravity:     mgl64.Vec3{},
		Substeps:    20,
		SpatialGrid: NewSpatialGrid(6.0, 4096),
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
		pair := BroadPhase(world.SpatialGrid, world.Bodies)

		for p := range pair {
			p.BodyA.IsSleeping = true
		}
	}
}

// BenchmarkLargeGJK2-16    	    1010	   1174808 ns/op	  133546 B/op	    2362 allocs/op
// BenchmarkLargeGJK2-16    	     100	  10239257 ns/op	 1073609 B/op	   24118 allocs/op
// BenchmarkLargeGJK2-16    	     100	  19060863 ns/op	 1000860 B/op	   10399 allocs/op
// BenchmarkLargeGJK2-16    	     126	   9732729 ns/op	 1172060 B/op	   10395 allocs/op
func BenchmarkLargeGJK2(b *testing.B) {
	const cubesCount = 1000
	const rowSize = 100.0

	world := World{
		Substeps:    10,
		SpatialGrid: NewSpatialGrid(6.0, 4096),
	}
	for i := 0; i < cubesCount; i++ {
		row := i / rowSize
		col := i % rowSize
		x := 0.0
		y := float64(row) * 0.9
		z := float64(col) * 0.9

		world.AddBody(createBox(mgl64.Vec3{x, y, z}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	}

	f, _ := os.Create("trace.out")
	fcpu, _ := os.Create(`cpu.prof`)
	fheap, _ := os.Create(`heap.prof`)
	defer f.Close()
	defer fcpu.Close()
	defer fheap.Close()
	pprof.StartCPUProfile(fcpu)
	pprof.WriteHeapProfile(fheap)
	trace.Start(f)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		pair := BroadPhase(world.SpatialGrid, world.Bodies)
		b.StartTimer()

		collisionPair := GJK(pair)

		for cp := range collisionPair {
			s := cp.simplex.Count
			s++
		}
	}

	b.StopTimer()
	trace.Stop()
	pprof.StopCPUProfile()
}

// BenchmarkLargeEPA2-16    	      68	  16822520 ns/op	41372250 B/op	  385439 allocs/op
// BenchmarkLargeEPA2-16    	      60	  17793428 ns/op	16812076 B/op	  269212 allocs/op
// BenchmarkLargeEPA2-16    	      67	  16210373 ns/op	13802236 B/op	  186096 allocs/op
// BenchmarkLargeEPA2-16    	      74	  15506020 ns/op	 7415237 B/op	  113062 allocs/op
// BenchmarkLargeEPA2-16    	      68	  16088034 ns/op	 9779191 B/op	  144406 allocs/op
// BenchmarkLargeEPA2-16    	      88	  12838612 ns/op	 4697867 B/op	   81534 allocs/op
// BenchmarkLargeEPA2-16    	      94	  12424054 ns/op	 3703345 B/op	   71191 allocs/op
// BenchmarkLargeEPA2-16    	     100	  10844368 ns/op	 2250015 B/op	   20845 allocs/op
// BenchmarkLargeEPA2-16    	     100	  11876997 ns/op	 2033844 B/op	   20867 allocs/op
func BenchmarkLargeEPA2(b *testing.B) {
	const cubesCount = 1000
	const rowSize = 100.0

	world := World{
		SpatialGrid: NewSpatialGrid(6.0, 4096),
	}
	for i := 0; i < cubesCount; i++ {
		row := i / rowSize
		col := i % rowSize
		x := 0.0
		y := float64(row) * 0.9
		z := float64(col) * 0.9

		world.AddBody(createBox(mgl64.Vec3{x, y, z}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic))
	}

	f, _ := os.Create("trace.out")
	fcpu, _ := os.Create(`cpu.prof`)
	fheap, _ := os.Create(`heap.prof`)
	defer f.Close()
	defer fcpu.Close()
	defer fheap.Close()
	pprof.StartCPUProfile(fcpu)
	pprof.WriteHeapProfile(fheap)
	trace.Start(f)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		b.StopTimer()
		pair := BroadPhase(world.SpatialGrid, world.Bodies)
		collisionPair := GJK(pair)
		b.StartTimer()

		c := EPA(collisionPair)

		for cp := range c {
			cp.Normal.Add(mgl64.Vec3{1, 1, 1})
		}
	}

	b.StopTimer()
	trace.Stop()
	pprof.StopCPUProfile()
}

//// BenchmarkLargeFullProcess2-16    	     283	   4050792 ns/op	 6838094 B/op	   44201 allocs/op
//// BenchmarkLargeFullProcess2-16    	     292	   3824172 ns/op	 2055274 B/op	   33626 allocs/op
//// BenchmarkLargeFullProcess2-16    	     322	   3651005 ns/op	 1804978 B/op	   27433 allocs/op
//// BenchmarkLargeFullProcess2-16    	     387	   3478882 ns/op	 1243375 B/op	   20867 allocs/op
//// BenchmarkLargeFullProcess2-16    	     421	   2963511 ns/op	 1023337 B/op	   18366 allocs/op
//// BenchmarkLargeFullProcess2-16    	     447	   2738878 ns/op	  204897 B/op	    1599 allocs/op
//func BenchmarkLargeFullProcess2(b *testing.B) {
//	const cubesCount = 1000
//	const rowSize = 100.0
//
//	bodies := make([]*actor.RigidBody, cubesCount)
//	rand.Seed(0)
//	for i := 0; i < cubesCount; i++ {
//		x := 0.0
//		y := rand.Float64() * rowSize
//		z := rand.Float64() * rowSize
//
//		bodies[i] = createBox(mgl64.Vec3{x, y, z}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
//	}
//
//	//f, _ := os.Create("trace.out")
//	//fcpu, _ := os.Create(`cpu.prof`)
//	//fheap, _ := os.Create(`heap.prof`)
//	//defer f.Close()
//	//defer fcpu.Close()
//	//defer fheap.Close()
//	//pprof.StartCPUProfile(fcpu)
//	//pprof.WriteHeapProfile(fheap)
//	//trace.Start(f)
//
//	b.ReportAllocs()
//	b.ResetTimer()
//	for i := 0; i < b.N; i++ {
//		b.StopTimer()
//		pair := BroadPhase(bodies)
//		b.StartTimer()
//		c := NarrowPhase(pair)
//		//c := EPA(collisionPair)
//
//		for _, cp := range c {
//			cp.Normal.Add(mgl64.Vec3{1, 1, 1})
//		}
//	}
//	//
//	//b.StopTimer()
//	//trace.Stop()
//	//pprof.StopCPUProfile()
//}

// BenchmarkLargeWorldStep-16    	      31	  33618889 ns/op	12125978 B/op	  125992 allocs/op
// BenchmarkLargeWorldStep-16    	      18	  60079141 ns/op	 7362741 B/op	  111143 allocs/op
// BenchmarkLargeWorldStep-16    	      18	  59910275 ns/op	 5765873 B/op	   77687 allocs/op
// BenchmarkLargeWorldStep-16    	      20	  50151706 ns/op	 4242036 B/op	   60310 allocs/op
// BenchmarkLargeWorldStep-16    	      19	  56083404 ns/op	 3350388 B/op	   51131 allocs/op
// BenchmarkLargeWorldStep-16    	      18	  58991949 ns/op	 2254283 B/op	   41523 allocs/op
// BenchmarkLargeWorldStep-16    	      24	  42948575 ns/op	 1690602 B/op	   35338 allocs/op
// BenchmarkLargeWorldStep-16    	      72	  14198539 ns/op	  906586 B/op	   15005 allocs/op
// BenchmarkLargeWorldStep-16    	      88	  11727200 ns/op	  944759 B/op	   12895 allocs/op
// BenchmarkLargeWorldStep-16    	     110	   9633886 ns/op	  434964 B/op	    3836 allocs/op
// BenchmarkLargeWorldStep-16    	     116	  10779597 ns/op	  413915 B/op	    3692 allocs/op
// BenchmarkLargeWorldStep-16    	      97	  11073711 ns/op	  446703 B/op	    3952 allocs/op
func BenchmarkLargeWorldStep(b *testing.B) {
	const cubesCount = 1000
	const rowSize = 100.0

	world := World{
		Gravity:     mgl64.Vec3{},
		Substeps:    20,
		SpatialGrid: NewSpatialGrid(6.0, 4096),
	}
	bodies := make([]*actor.RigidBody, cubesCount)

	for i := 0; i < cubesCount; i++ {
		row := i / rowSize
		col := i % rowSize
		x := 0.0
		y := float64(row) * 0.9
		z := float64(col) * 0.9

		bodies[i] = createBox(mgl64.Vec3{x, y, z}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
		world.AddBody(bodies[i])
	}

	f, _ := os.Create("trace.out")
	fcpu, _ := os.Create(`cpu.prof`)
	fheap, _ := os.Create(`heap.prof`)
	defer f.Close()
	defer fcpu.Close()
	defer fheap.Close()
	pprof.StartCPUProfile(fcpu)
	pprof.WriteHeapProfile(fheap)
	trace.Start(f)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		world.Step(1.0 / 60.0)
	}

	b.StopTimer()
	trace.Stop()
	pprof.StopCPUProfile()
}
