// Package epa implements the Expanding Polytope Algorithm for computing penetration depth.
//
// EPA is run after GJK detects a collision to determine:
//   - Penetration depth (how far shapes overlap)
//   - Contact normal (direction to separate shapes)
//   - Contact points (where shapes touch)
//
// The algorithm expands a polytope (starting from GJK's final simplex) toward the origin
// in the Minkowski difference space, finding the closest face which gives us the
// Minimum Translation Vector (MTV) to separate the shapes.
//
// For detailed algorithm explanation with pseudocode and visual examples, see:
// ALGORITHMS.md - "EPA Algorithm" section
//
// References:
//   - Van den Bergen: "Proximity Queries and Penetration Depth Computation on 3D Game Objects" (2001)
package epa

import (
	"fmt"
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

const (
	// EPAMaxIterations limits polytope expansion to prevent infinite loops.
	// Typical convergence: 5-15 iterations for simple shapes.
	// If this limit is reached, EPA returns an error.
	EPAMaxIterations = 32

	// EPAConvergenceTolerance defines when EPA has converged.
	// If the distance to a new support point improves by less than this threshold,
	// we've found the closest face to the origin.
	// Lower values = more precision but slower convergence.
	EPAConvergenceTolerance = 0.001

	// EPAMinFaceDistance is the minimum face distance before we skip it.
	// Faces very close to or behind the origin are likely degenerate.
	EPAMinFaceDistance = 0.0001

	// NormalSnapThreshold is used to clamp nearly-zero normal components to exactly zero.
	// This helps with numerical stability and axis-aligned collisions.
	NormalSnapThreshold = 1e-8

	// DegeneratePenetrationEstimate is a fallback penetration depth for degenerate cases
	// where we have insufficient simplex points to compute accurate depth.
	DegeneratePenetrationEstimate = 0.01

	// Small initial capacity for PolytopeBuilder - grows dynamically as needed
	// Using very small initial capacity (4) for memory efficiency
	// No maximum limits - buffers grow to accommodate any reasonable polytope size
	polytopeInitialCapacity = 4
)

// EPA computes penetration depth and contact information for overlapping convex shapes.
//
// Algorithm overview:
//  1. Start with simplex from GJK (tetrahedron containing origin)
//  2. Build initial polytope faces from simplex
//  3. Find face closest to origin
//  4. Get support point in face normal direction
//  5. If converged (new point doesn't improve distance) → done
//  6. Otherwise, expand polytope by adding support point
//  7. Repeat from step 3
//
// Parameters:
//   - a, b: The two colliding rigid bodies
//   - simplex: Final simplex from GJK (typically 4 points forming tetrahedron)
//
// Returns:
//   - ContactConstraint: Contains contact normal, penetration depth, contact points
//   - error: Non-nil if EPA failed to converge or encountered degenerate case
//
// The contact normal points from body A toward body B (separation direction).
// Penetration depth is always positive (how far to move B away from A).
func EPA(a, b *actor.RigidBody, simplex *gjk.Simplex) (constraint.ContactConstraint, error) {
	// If simplex is too small (degenerate case), create a minimal contact
	if simplex.Count < 4 {
		return handleDegenerateSimplex(a, b, simplex), nil
	}

	// Get builder from pool - single allocation replacing multiple pools
	builder := polytopeBuilderPool.Get().(*PolytopeBuilder)
	defer polytopeBuilderPool.Put(builder)
	builder.Reset()

	// Step 1: Build initial polytope faces from the tetrahedron simplex
	if err := builder.BuildInitialFaces(simplex); err != nil {
		return constraint.ContactConstraint{}, err
	}

	var closestFaceIndex int
	var closestFace *Face
	var support mgl64.Vec3
	var distance float64

	// Step 2: Iteratively expand polytope toward origin
	for i := 0; i < EPAMaxIterations; i++ {
		if len(builder.faces) == 0 {
			// All faces removed (degenerate polytope) - should not happen
			break
		}

		// Step 3: Find the face closest to the origin
		// This face's normal and distance give us the current best MTV estimate
		closestFaceIndex = builder.FindClosestFaceIndex()
		closestFace = &builder.faces[closestFaceIndex]

		// Skip faces that are too close to or behind the origin (degenerate)
		if closestFace.Distance < EPAMinFaceDistance {
			// Remove this face and try the next one using swap-with-last
			builder.faces[closestFaceIndex] = builder.faces[len(builder.faces)-1]
			builder.faces = builder.faces[:len(builder.faces)-1]
			continue
		}

		// Step 4: Get support point in the direction of the closest face's normal
		support = gjk.MinkowskiSupport(a, b, closestFace.Normal)
		distance = support.Dot(closestFace.Normal)

		// Step 5: Check for convergence
		// If the new support point doesn't significantly improve the distance,
		// we've found the face of the Minkowski difference closest to the origin
		if distance-closestFace.Distance < EPAConvergenceTolerance {
			// Generate contact manifold (multiple contact points for stability)
			manifoldPoints := GenerateManifold(a, b, closestFace.Normal, closestFace.Distance)

			return constraint.ContactConstraint{
				BodyA:  a,
				BodyB:  b,
				Points: manifoldPoints,
				Normal: closestFace.Normal,
			}, nil
		}

		// Step 6: Expand polytope by adding the new support point
		// This removes faces that "see" the new point and adds new faces connecting to it
		// Zero allocations - all operations use fixed buffers
		if err := builder.AddPointAndRebuildFaces(support, closestFaceIndex); err != nil {
			// Buffer overflow - return current best estimate instead of failing
			manifoldPoints := GenerateManifold(a, b, closestFace.Normal, closestFace.Distance)
			return constraint.ContactConstraint{
				BodyA:  a,
				BodyB:  b,
				Points: manifoldPoints,
				Normal: closestFace.Normal,
			}, nil
		}
	}

	// EPA failed to converge within max iterations (rare, indicates numerical issues)
	return constraint.ContactConstraint{}, fmt.Errorf("EPA failed to converge after %d iterations", EPAMaxIterations)
}

// handleDegenerateSimplex creates a contact constraint when GJK returns an incomplete simplex.
//
// This happens in rare edge cases where shapes are touching but GJK couldn't build a full
// tetrahedron. We estimate the contact normal and penetration depth from available points.
//
// Cases:
//   - 2+ points: Use closest point to origin as penetration estimate
//   - 1 point: Estimate from body center separation (very approximate)
//
// Returns a valid ContactConstraint with estimated values.
func handleDegenerateSimplex(bodyA, bodyB *actor.RigidBody, simplex *gjk.Simplex) constraint.ContactConstraint {
	if simplex.Count >= 2 {
		// Use first two points to estimate
		a := simplex.Points[0]
		b := simplex.Points[1]

		// Find which point is closer to origin
		distA := math.Sqrt(a.Dot(a))
		distB := math.Sqrt(b.Dot(b))

		var penetration float64
		var normal mgl64.Vec3

		if distA < distB {
			penetration = distA
			normal = a.Normalize()
		} else {
			penetration = distB
			normal = b.Normalize()
		}

		manifoldPoints := GenerateManifold(bodyA, bodyB, normal, penetration)

		return constraint.ContactConstraint{
			BodyA:  bodyA,
			BodyB:  bodyB,
			Points: manifoldPoints,
			Normal: normal,
		}
	}

	// Single point simplex - most degenerate case
	// Estimate contact normal from body centers
	normal := bodyB.Transform.Position.Sub(bodyA.Transform.Position)
	normalLen := normal.Len()

	if normalLen < NormalSnapThreshold {
		// Centers are at same location, use default upward direction
		normal = mgl64.Vec3{0, 1, 0}
	} else {
		normal = normal.Mul(1.0 / normalLen)
	}

	// Estimate penetration depth (highly approximate for degenerate case)
	penetration := DegeneratePenetrationEstimate

	// Generate manifold with estimated normal
	manifoldPoints := GenerateManifold(bodyA, bodyB, normal, penetration)

	// Return fallback contact constraint
	return constraint.ContactConstraint{
		BodyA:  bodyA,
		BodyB:  bodyB,
		Points: manifoldPoints,
		Normal: normal,
	}
}

// snapNormalToAxis clamps nearly-zero components of a normal vector to exactly zero.
//
// This improves numerical stability for axis-aligned collisions (box on ground)
// by preventing tiny floating-point errors from causing jitter in tangent directions.
//
// Components with absolute value < NormalSnapThreshold are set to 0, then the
// vector is renormalized.
func snapNormalToAxis(normal mgl64.Vec3) mgl64.Vec3 {
	const threshold = NormalSnapThreshold

	x := normal[0]
	y := normal[1]
	z := normal[2]

	// Clamp tiny components to zero
	if math.Abs(x) < threshold {
		x = 0
	}
	if math.Abs(y) < threshold {
		y = 0
	}
	if math.Abs(z) < threshold {
		z = 0
	}

	// Reconstruct the normal
	clamped := mgl64.Vec3{x, y, z}

	// Renormalize (important!)
	length := math.Sqrt(clamped.Dot(clamped))
	if length > 1e-8 {
		clamped = clamped.Mul(1.0 / length)
	} else {
		// If all components were clamped to zero, return default
		return mgl64.Vec3{0, 1, 0}
	}

	return clamped
}
