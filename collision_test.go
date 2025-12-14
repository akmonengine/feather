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
		actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()},
		&actor.Plane{Normal: normal, Distance: distance},
		actor.BodyTypeStatic,
		0.0,
	)
}

// // TestComplianceConstants verifies that compliance constants are defined
//
//	func TestComplianceConstants(t *testing.T) {
//		constants := map[string]float64{
//			"CONCRETE_COMPLIANCE": CONCRETE_COMPLIANCE,
//			"WOOD_COMPLIANCE":     WOOD_COMPLIANCE,
//			"LEATHER_COMPLIANCE":  LEATHER_COMPLIANCE,
//			"TENDON_COMPLIANCE":   TENDON_COMPLIANCE,
//			"RUBBER_COMPLIANCE":   RUBBER_COMPLIANCE,
//			"MUSCLE_COMPLIANCE":   MUSCLE_COMPLIANCE,
//			"FAT_COMPLIANCE":      FAT_COMPLIANCE,
//			"STIFF_COMPLIANCE":    STIFF_COMPLIANCE,
//		}
//
//		for name, value := range constants {
//			if value <= 0 {
//				t.Errorf("%s should be positive, got %v", name, value)
//			}
//		}
//
//		// STIFF_COMPLIANCE should equal CONCRETE_COMPLIANCE
//		if STIFF_COMPLIANCE != CONCRETE_COMPLIANCE {
//			t.Errorf("STIFF_COMPLIANCE = %v, want %v (CONCRETE_COMPLIANCE)", STIFF_COMPLIANCE, CONCRETE_COMPLIANCE)
//		}
//
//		// Verify ordering: concrete < wood < tendon < leather < rubber < muscle < fat
//		if !(CONCRETE_COMPLIANCE < WOOD_COMPLIANCE) {
//			t.Error("CONCRETE_COMPLIANCE should be less than WOOD_COMPLIANCE")
//		}
//		if !(WOOD_COMPLIANCE < TENDON_COMPLIANCE) {
//			t.Error("WOOD_COMPLIANCE should be less than TENDON_COMPLIANCE")
//		}
//		if !(TENDON_COMPLIANCE < LEATHER_COMPLIANCE) {
//			t.Error("TENDON_COMPLIANCE should be less than LEATHER_COMPLIANCE")
//		}
//		if !(LEATHER_COMPLIANCE < RUBBER_COMPLIANCE) {
//			t.Error("LEATHER_COMPLIANCE should be less than RUBBER_COMPLIANCE")
//		}
//		if !(RUBBER_COMPLIANCE < MUSCLE_COMPLIANCE) {
//			t.Error("RUBBER_COMPLIANCE should be less than MUSCLE_COMPLIANCE")
//		}
//		if !(MUSCLE_COMPLIANCE < FAT_COMPLIANCE) {
//			t.Error("MUSCLE_COMPLIANCE should be less than FAT_COMPLIANCE")
//		}
//	}
//
// // TestBroadPhaseNoBodies tests broad phase with no bodies
//
//	func TestBroadPhaseNoBodies(t *testing.T) {
//		bodies := []*actor.RigidBody{}
//
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) != 0 {
//			t.Errorf("BroadPhase with no bodies returned %d pairs, want 0", len(pairs))
//		}
//	}
//
// // TestBroadPhaseSingleBody tests broad phase with a single body
//
//	func TestBroadPhaseSingleBody(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//		}
//
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) != 0 {
//			t.Errorf("BroadPhase with single body returned %d pairs, want 0", len(pairs))
//		}
//	}
//
// // TestBroadPhaseTwoBodiesOverlapping tests two overlapping bodies
// //func TestBroadPhaseTwoBodiesOverlapping(t *testing.T) {
// //	bodies := []*actor.RigidBody{
// //		createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
// //		createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
// //	}
// //
// //	pairs := BroadPhase(bodies)
// //
// //	if len(pairs) != 1 {
// //		t.Errorf("BroadPhase with overlapping bodies returned %d pairs, want 1", len(pairs))
// //	}
// //
// //	if len(pairs) > 0 {
// //		if pairs[0].BodyA != bodies[0] || pairs[0].BodyB != bodies[1] {
// //			t.Error("Collision pair bodies don't match expected bodies")
// //		}
// //	}
// //}
//
// // TestBroadPhaseTwoBodiesNotOverlapping tests two non-overlapping bodies
//
//	func TestBroadPhaseTwoBodiesNotOverlapping(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//			createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//		}
//
//		bodies[0].Shape.ComputeAABB(bodies[0].Transform)
//		bodies[1].Shape.ComputeAABB(bodies[1].Transform)
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) != 0 {
//			t.Errorf("BroadPhase with non-overlapping bodies returned %d pairs, want 0", len(pairs))
//		}
//	}
//
// // TestBroadPhaseTwoStaticBodies tests two static bodies (should be skipped)
//
//	func TestBroadPhaseTwoStaticBodies(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic),
//			createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic),
//		}
//
//		pairs := BroadPhase(bodies)
//
//		// Static-static collisions should be skipped
//		if len(pairs) != 0 {
//			t.Errorf("BroadPhase with two static bodies returned %d pairs, want 0 (should skip static-static)", len(pairs))
//		}
//	}
//
// // TestBroadPhaseStaticDynamicOverlapping tests static and dynamic bodies
//
//	func TestBroadPhaseStaticDynamicOverlapping(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeStatic),
//			createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//		}
//
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) != 1 {
//			t.Errorf("BroadPhase with static-dynamic overlapping returned %d pairs, want 1", len(pairs))
//		}
//	}
//
// // TestBroadPhaseMultipleBodies tests multiple bodies with various overlaps
//
//	func TestBroadPhaseMultipleBodies(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),   // 0
//			createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic), // 1 - overlaps with 0
//			createBox(mgl64.Vec3{3, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),   // 2 - overlaps with 1
//			createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),  // 3 - no overlaps
//		}
//
//		bodies[0].Shape.ComputeAABB(bodies[0].Transform)
//		bodies[1].Shape.ComputeAABB(bodies[1].Transform)
//		bodies[2].Shape.ComputeAABB(bodies[2].Transform)
//		bodies[3].Shape.ComputeAABB(bodies[3].Transform)
//		pairs := BroadPhase(bodies)
//
//		// Expected pairs: (0,1), (1,2)
//		expectedPairs := 2
//		if len(pairs) != expectedPairs {
//			t.Errorf("BroadPhase returned %d pairs, want %d", len(pairs), expectedPairs)
//		}
//
//		// Verify that we have the right pairs
//		pairMap := make(map[string]bool)
//		for pair := range pairs {
//			// Create a key from body indices
//			var key string
//			for i, body := range bodies {
//				if body == pair.BodyA {
//					for j, bodyB := range bodies {
//						if bodyB == pair.BodyB {
//							if i < j {
//								key = string(rune('0'+i)) + string(rune('0'+j))
//							}
//						}
//					}
//				}
//			}
//			if key != "" {
//				pairMap[key] = true
//			}
//		}
//
//		if len(pairMap) != expectedPairs {
//			t.Logf("Found pairs: %v", pairMap)
//		}
//	}
//
// // TestBroadPhaseSpheresOverlapping tests overlapping spheres
//
//	func TestBroadPhaseSpheresOverlapping(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic),
//			createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic),
//		}
//
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) != 1 {
//			t.Errorf("BroadPhase with overlapping spheres returned %d pairs, want 1", len(pairs))
//		}
//	}
//
// // TestBroadPhaseSpheresNotOverlapping tests non-overlapping spheres
//
//	func TestBroadPhaseSpheresNotOverlapping(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic),
//			createSphere(mgl64.Vec3{3, 0, 0}, 1.0, actor.BodyTypeDynamic),
//		}
//
//		bodies[0].Shape.ComputeAABB(bodies[0].Transform)
//		bodies[1].Shape.ComputeAABB(bodies[1].Transform)
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) != 0 {
//			t.Errorf("BroadPhase with non-overlapping spheres returned %d pairs, want 0", len(pairs))
//		}
//	}
//
// // TestBroadPhaseMixedShapes tests boxes and spheres together
//
//	func TestBroadPhaseMixedShapes(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//			createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic),
//		}
//
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) != 1 {
//			t.Errorf("BroadPhase with box-sphere overlapping returned %d pairs, want 1", len(pairs))
//		}
//	}
//
// // TestBroadPhaseWithPlane tests bodies overlapping with a plane
//
//	func TestBroadPhaseWithPlane(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createPlane(mgl64.Vec3{0, 1, 0}, 0), // Ground plane at y=0
//			createBox(mgl64.Vec3{0, 0.5, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//		}
//
//		pairs := BroadPhase(bodies)
//
//		// The box should overlap with the plane's AABB
//		if len(pairs) != 1 {
//			t.Errorf("BroadPhase with plane-box returned %d pairs, want 1", len(pairs))
//		}
//	}
//
// // TestNarrowPhaseNoPairs tests narrow phase with no pairs
// //func TestNarrowPhaseNoPairs(t *testing.T) {
// //	pairs := []CollisionPair{}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	if len(contacts) != 0 {
// //		t.Errorf("NarrowPhase with no pairs returned %d contacts, want 0", len(contacts))
// //	}
// //}
// //
// //// TestNarrowPhaseOverlappingBoxes tests narrow phase with overlapping boxes
// //func TestNarrowPhaseOverlappingBoxes(t *testing.T) {
// //	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //	bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB},
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should detect collision
// //	if len(contacts) == 0 {
// //		t.Error("NarrowPhase with overlapping boxes returned no contacts, expected at least 1")
// //	}
// //}
// //
// //// TestNarrowPhaseNonOverlappingBoxes tests narrow phase with non-overlapping boxes
// //func TestNarrowPhaseNonOverlappingBoxes(t *testing.T) {
// //	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //	bodyB := createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB},
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should not detect collision
// //	if len(contacts) != 0 {
// //		t.Errorf("NarrowPhase with non-overlapping boxes returned %d contacts, want 0", len(contacts))
// //	}
// //}
// //
// //// TestNarrowPhaseOverlappingSpheres tests narrow phase with overlapping spheres
// //func TestNarrowPhaseOverlappingSpheres(t *testing.T) {
// //	bodyA := createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic)
// //	bodyB := createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB},
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should detect collision
// //	if len(contacts) == 0 {
// //		t.Error("NarrowPhase with overlapping spheres returned no contacts, expected at least 1")
// //	}
// //}
// //
// //// TestNarrowPhaseNonOverlappingSpheres tests narrow phase with non-overlapping spheres
// //func TestNarrowPhaseNonOverlappingSpheres(t *testing.T) {
// //	bodyA := createSphere(mgl64.Vec3{0, 0, 0}, 1.0, actor.BodyTypeDynamic)
// //	bodyB := createSphere(mgl64.Vec3{5, 0, 0}, 1.0, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB},
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should not detect collision
// //	if len(contacts) != 0 {
// //		t.Errorf("NarrowPhase with non-overlapping spheres returned %d contacts, want 0", len(contacts))
// //	}
// //}
// //
// //// TestNarrowPhaseBoxSphere tests narrow phase with box and sphere
// //func TestNarrowPhaseBoxSphere(t *testing.T) {
// //	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //	bodyB := createSphere(mgl64.Vec3{1.5, 0, 0}, 1.0, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB},
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should detect collision
// //	if len(contacts) == 0 {
// //		t.Error("NarrowPhase with overlapping box-sphere returned no contacts, expected at least 1")
// //	}
// //}
// //
// //// TestNarrowPhaseSphereOnPlane tests narrow phase with sphere resting on plane
// //func TestNarrowPhaseSphereOnPlane(t *testing.T) {
// //	bodyA := createPlane(mgl64.Vec3{0, 1, 0}, 0)
// //	bodyB := createSphere(mgl64.Vec3{0, 0.5, 0}, 1.0, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB},
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should detect collision (sphere penetrating plane)
// //	if len(contacts) == 0 {
// //		t.Error("NarrowPhase with sphere on plane returned no contacts, expected at least 1")
// //	}
// //}
// //
// //// TestNarrowPhaseBoxOnPlane tests narrow phase with box resting on plane
// //func TestNarrowPhaseBoxOnPlane(t *testing.T) {
// //	bodyA := createPlane(mgl64.Vec3{0, 1, 0}, 0)
// //	bodyB := createBox(mgl64.Vec3{0, 0.5, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB},
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should detect collision (box penetrating plane)
// //	if len(contacts) == 0 {
// //		t.Error("NarrowPhase with box on plane returned no contacts, expected at least 1")
// //	}
// //}
// //
// //// TestNarrowPhaseMultiplePairs tests narrow phase with multiple collision pairs
// //func TestNarrowPhaseMultiplePairs(t *testing.T) {
// //	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //	bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //	bodyC := createSphere(mgl64.Vec3{3, 0, 0}, 1.0, actor.BodyTypeDynamic)
// //	bodyD := createSphere(mgl64.Vec3{4, 0, 0}, 1.0, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{
// //		{BodyA: bodyA, BodyB: bodyB}, // Should collide
// //		{BodyA: bodyC, BodyB: bodyD}, // Should collide
// //	}
// //
// //	contacts := NarrowPhase(pairs)
// //
// //	// Should detect both collisions
// //	if len(contacts) < 2 {
// //		t.Errorf("NarrowPhase with 2 overlapping pairs returned %d contacts, want at least 2", len(contacts))
// //	}
// //}
//
// // TestCollisionPairStruct tests the CollisionPair struct
//
//	func TestCollisionPairStruct(t *testing.T) {
//		bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
//		bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
//
//		pair := CollisionPair{
//			BodyA: bodyA,
//			BodyB: bodyB,
//		}
//
//		if pair.BodyA != bodyA {
//			t.Error("CollisionPair.BodyA doesn't match expected body")
//		}
//		if pair.BodyB != bodyB {
//			t.Error("CollisionPair.BodyB doesn't match expected body")
//		}
//	}
//
// // TestIntegrationBroadAndNarrowPhase tests the complete collision detection pipeline
//
//	func TestIntegrationBroadAndNarrowPhase(t *testing.T) {
//		bodies := []*actor.RigidBody{
//			createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//			createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//			createBox(mgl64.Vec3{10, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic),
//		}
//
//		// Broad phase
//		pairs := BroadPhase(bodies)
//
//		if len(pairs) == 0 {
//			t.Fatal("BroadPhase returned no pairs, expected at least 1")
//		}
//
//		// Narrow phase
//		contacts := NarrowPhase(pairs)
//
//		if len(contacts) == 0 {
//			t.Error("NarrowPhase returned no contacts, expected at least 1")
//		}
//
//		// Verify number of contacts matches number of actual collisions
//		// bodies[0] and bodies[1] should collide
//		// bodies[2] is far away and should not collide
//		if len(contacts) != 1 {
//			t.Errorf("Expected 1 contact from integration test, got %d", len(contacts))
//		}
//	}
//
// // Benchmark tests
//
//	func BenchmarkBroadPhase10Bodies(b *testing.B) {
//		bodies := make([]*actor.RigidBody, 10)
//		for i := 0; i < 10; i++ {
//			bodies[i] = createBox(
//				mgl64.Vec3{float64(i) * 2, 0, 0},
//				mgl64.Vec3{1, 1, 1},
//				actor.BodyTypeDynamic,
//			)
//		}
//
//		b.ResetTimer()
//		for i := 0; i < b.N; i++ {
//			BroadPhase(bodies)
//		}
//	}
//
//	func BenchmarkBroadPhase50Bodies(b *testing.B) {
//		bodies := make([]*actor.RigidBody, 50)
//		for i := 0; i < 50; i++ {
//			bodies[i] = createBox(
//				mgl64.Vec3{float64(i) * 2, 0, 0},
//				mgl64.Vec3{1, 1, 1},
//				actor.BodyTypeDynamic,
//			)
//		}
//
//		b.ResetTimer()
//		for i := 0; i < b.N; i++ {
//			BroadPhase(bodies)
//		}
//	}
//
// //func BenchmarkNarrowPhaseSinglePair(b *testing.B) {
// //	bodyA := createBox(mgl64.Vec3{0, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //	bodyB := createBox(mgl64.Vec3{1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //
// //	pairs := []CollisionPair{{BodyA: bodyA, BodyB: bodyB}}
// //
// //	b.ResetTimer()
// //	for i := 0; i < b.N; i++ {
// //		NarrowPhase(pairs)
// //	}
// //}
// //
// //func BenchmarkNarrowPhaseMultiplePairs(b *testing.B) {
// //	pairs := make([]CollisionPair, 10)
// //	for i := 0; i < 10; i++ {
// //		bodyA := createBox(mgl64.Vec3{float64(i) * 3, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //		bodyB := createBox(mgl64.Vec3{float64(i)*3 + 1.5, 0, 0}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
// //		pairs[i] = CollisionPair{BodyA: bodyA, BodyB: bodyB}
// //	}
// //
// //	b.ResetTimer()
// //	for i := 0; i < b.N; i++ {
// //		NarrowPhase(pairs)
// //	}
// //}
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

	rand.Seed(0)
	for i := 0; i < cubesCount; i++ {
		x := 0.0
		y := rand.Float64() * rowSize
		z := rand.Float64() * rowSize

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

// // BenchmarkLargeGJK2-16    	    1010	   1174808 ns/op	  133546 B/op	    2362 allocs/op
// // BenchmarkLargeGJK2-16    	     100	  10239257 ns/op	 1073609 B/op	   24118 allocs/op
// // BenchmarkLargeGJK2-16    	     100	  19060863 ns/op	 1000860 B/op	   10399 allocs/op
//
//	func BenchmarkLargeGJK2(b *testing.B) {
//		const cubesCount = 1000
//		const rowSize = 100.0
//
//		bodies := make([]*actor.RigidBody, cubesCount)
//		for i := 0; i < cubesCount; i++ {
//			row := i / rowSize
//			col := i % rowSize
//			x := 0.0
//			y := float64(row) * 0.9
//			z := float64(col) * 0.9
//
//			bodies[i] = createBox(mgl64.Vec3{x, y, z}, mgl64.Vec3{1, 1, 1}, actor.BodyTypeDynamic)
//		}
//
//		f, _ := os.Create("trace.out")
//		fcpu, _ := os.Create(`cpu.prof`)
//		fheap, _ := os.Create(`heap.prof`)
//		defer f.Close()
//		defer fcpu.Close()
//		defer fheap.Close()
//		pprof.StartCPUProfile(fcpu)
//		pprof.WriteHeapProfile(fheap)
//		trace.Start(f)
//
//		b.ReportAllocs()
//		b.ResetTimer()
//		for i := 0; i < b.N; i++ {
//			b.StopTimer()
//			pair := BroadPhase(bodies)
//			b.StartTimer()
//
//			collisionPair := GJK(pair)
//
//			for cp := range collisionPair {
//				s := cp.simplex.Count
//				s++
//			}
//		}
//
//		b.StopTimer()
//		trace.Stop()
//		pprof.StopCPUProfile()
//	}
//
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
	rand.Seed(0)
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
