// Package gjk implements the Gilbert-Johnson-Keerthi (GJK) algorithm for collision detection.
//
// GJK detects whether two convex shapes overlap by testing if their Minkowski difference
// contains the origin. The algorithm builds a simplex incrementally, converging toward
// the origin in typically 3-6 iterations.
//
// For detailed algorithm explanation with pseudocode and visual examples, see:
// ALGORITHMS.md - "GJK Algorithm" section
//
// References:
//   - Gilbert, Johnson, Keerthi: "A Fast Procedure for Computing the Distance Between
//     Complex Objects in Three-Dimensional Space" (1988)
//   - Van den Bergen: "Collision Detection in Interactive 3D Environments" (2003)
package gjk

import (
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Simplex represents a set of 1-4 points in the Minkowski difference space.
// The simplex evolves during GJK iterations, always containing the most recent support points.
// Size progression: 1 point → 2 points (line) → 3 points (triangle) → 4 points (tetrahedron)
type Simplex struct {
	Points [4]mgl64.Vec3
	Count  int
}

func (s *Simplex) Reset() {
	s.Count = 0
}

var SimplexPool = sync.Pool{
	New: func() interface{} {
		return &Simplex{}
	},
}

// MinkowskiSupport computes a support point in the Minkowski difference (A - B).
//
// The Minkowski difference A - B is the set of all vectors (a - b) where a ∈ A and b ∈ B.
// For collision detection, we only need the extreme points (support points) in any direction.
//
// Parameters:
//   - a, b: The two rigid bodies to test
//   - direction: The direction to find the furthest point
//
// Returns:
//
//	Support point: furthestPoint(A, direction) - furthestPoint(B, -direction)
//
// This is the fundamental query that makes GJK work for any convex shape - shapes only
// need to implement a Support() function, not expose their full geometry.
func MinkowskiSupport(a, b *actor.RigidBody, direction mgl64.Vec3) mgl64.Vec3 {
	supportA := a.SupportWorld(direction)
	supportB := b.SupportWorld(direction.Mul(-1))
	return supportA.Sub(supportB)
}

// GJK performs collision detection between two convex rigid bodies.
//
// Algorithm overview:
//  1. Start with initial search direction (toward B from A)
//  2. Get first support point in Minkowski difference
//  3. Iteratively refine simplex toward origin
//  4. If origin is contained → collision
//  5. If can't reach origin → no collision
//
// Typical convergence: 3-6 iterations for most shapes.
//
// Returns:
//   - bool: true if collision detected, false otherwise
//
// The simplex is modified in place and contains 1-4 points. For collisions, it's always
// a tetrahedron (4 points) containing the origin, which EPA uses as its initial polytope.
func GJK(a, b *actor.RigidBody, simplex *Simplex) bool {
	// Compute initial direction from A to B (optimization over random direction)
	// Starting toward the other shape typically reduces iterations
	direction := b.Transform.Position.Sub(a.Transform.Position)
	if direction.LenSqr() < 1e-8 {
		direction = mgl64.Vec3{1, 0, 0} // Fallback if positions are identical
	}

	// Get first point of the simplex in the Minkowski difference
	simplex.Points[0] = MinkowskiSupport(a, b, direction)
	simplex.Count = 1

	// New direction towards the origin from this first point
	direction = simplex.Points[0].Mul(-1)

	// If first support point is at/near origin, shapes are touching
	if direction.LenSqr() < 1e-16 {
		return true // Collision detected (rare: shapes exactly touching at point)
	}

	maxIterations := 32 // Safety limit to prevent infinite loops
	for i := 0; i < maxIterations; i++ {
		// Find a new support point in the direction towards the origin
		newPoint := MinkowskiSupport(a, b, direction)

		// Early exit test: If the new point doesn't pass the origin in the search direction,
		// the origin cannot be reached, therefore no collision.
		// This is the key optimization that makes GJK fast - we prove separation
		// without building the full Minkowski difference.
		if newPoint.Dot(direction) <= 0 {
			return false // No collision detected - shapes are separated
		}

		// Add the new point to the simplex
		simplex.Points[simplex.Count] = newPoint
		simplex.Count++

		// Check if the simplex contains the origin
		// This function also updates the simplex and direction for the next iteration
		// by reducing the simplex to its closest feature to the origin
		if containsOrigin(simplex, &direction) {
			return true // Collision detected - origin is inside simplex
		}
	}

	// Failed to converge after maxIterations (very rare, may indicate numerical issues)
	// In practice this almost never happens for valid convex shapes
	return false
}

// containsOrigin tests if the simplex contains the origin and refines the simplex.
//
// This is the heart of GJK - it determines which feature of the simplex (point, edge, face)
// is closest to the origin, keeps only the relevant points, and updates the search direction.
//
// Behavior by simplex dimension:
//   - 2 points (line): Test Voronoi regions, reduce to closest point or keep edge
//   - 3 points (triangle): Test Voronoi regions, reduce to closest edge or keep face
//   - 4 points (tetrahedron): Test if origin is inside; if not, reduce to closest face
//
// Returns:
//   - true: Origin is contained (only possible for tetrahedron) → collision!
//   - false: Origin is outside, simplex and direction updated for next iteration
func containsOrigin(simplex *Simplex, direction *mgl64.Vec3) bool {
	switch simplex.Count {
	case 2:
		return line(simplex, direction)
	case 3:
		return triangle(simplex, direction)
	case 4:
		return tetrahedron(simplex, direction)
	}
	return false
}

// line handles the line simplex case (2 points: A and B).
//
// Tests which Voronoi region contains the origin:
//   - Region A: Origin is closest to point A alone
//   - Region AB: Origin is closest to the line segment AB
//
// Returns false (a line cannot contain origin in 3D).
// Updates direction to point toward origin from the closest feature.
func line(simplex *Simplex, direction *mgl64.Vec3) bool {
	a := simplex.Points[1]
	b := simplex.Points[0]
	ab := b.Sub(a)
	ao := a.Mul(-1)

	// Handle degenerate case: identical points
	if ab.LenSqr() < 1e-8 {
		if ao.LenSqr() < 1e-8 {
			return true // origin is at the point
		}
		// Origin is not at the point, but simplex is degenerate
		simplex.Points[0] = a
		simplex.Count = 1
		*direction = ao
		return false
	}

	// Check if origin is in Voronoi region A (behind A, opposite direction from B)
	// If ab.Dot(ao) <= 0, the origin is closest to point A alone
	if ab.Dot(ao) <= 0 {
		// Reduce simplex to point A
		simplex.Points[0] = a
		simplex.Count = 1
		*direction = ao
		return false
	}

	// Origin is in Voronoi region AB (between A and B direction-wise)
	abPerp := ab.Cross(ao).Cross(ab)
	if abPerp.LenSqr() < 1e-8 {
		// Origin is on the line segment → touching
		return true
	}

	*direction = abPerp
	return false
}

// triangle handles the triangle simplex case (3 points: A, B, C).
//
// Tests which Voronoi region contains the origin:
//   - Region A: Origin closest to point A alone
//   - Region AB: Origin closest to edge AB
//   - Region AC: Origin closest to edge AC
//   - Region ABC (above): Origin above triangle plane
//   - Region ABC (below): Origin below triangle plane
//
// Degenerate case: If points are collinear (flat triangle), treats as line instead.
//
// Returns false (a triangle cannot contain origin in 3D, we need tetrahedron).
// Reduces simplex to closest feature and updates direction.
func triangle(simplex *Simplex, direction *mgl64.Vec3) bool {
	a := simplex.Points[2] // Most recent point
	b := simplex.Points[1]
	c := simplex.Points[0]

	ab := b.Sub(a)
	ac := c.Sub(a)
	ao := a.Mul(-1)

	abc := ab.Cross(ac) // Triangle normal

	// Check for degenerate triangle (colinear points)
	// If normal is nearly zero, points are on a line
	if abc.LenSqr() < 1e-10 {
		// Treat as line instead of triangle
		// Keep A and B (discard C which is furthest from recent history)
		simplex.Points[0] = b
		simplex.Points[1] = a
		simplex.Count = 2
		return line(simplex, direction)
	}

	// Test the 3 regions around the triangle

	// Region AB (edge)
	abPerp := ab.Cross(abc)
	if abPerp.Dot(ao) > 0 {
		simplex.Points[0] = b
		simplex.Points[1] = a
		simplex.Count = 2
		*direction = ab.Cross(ao).Cross(ab)
		return false
	}

	// Region AC (edge)
	acPerp := abc.Cross(ac)
	if acPerp.Dot(ao) > 0 {
		simplex.Points[0] = c
		simplex.Points[1] = a
		simplex.Count = 2
		*direction = ac.Cross(ao).Cross(ac)
		return false
	}

	// Origin is above or below the triangle
	if abc.Dot(ao) > 0 {
		// Above the triangle
		*direction = abc
	} else {
		// Below, reverse order to maintain correct orientation
		simplex.Points[0] = a
		simplex.Points[1] = c
		simplex.Points[2] = b
		simplex.Count = 3
		*direction = abc.Mul(-1)
	}

	return false // Triangle never contains origin in 3D (we need tetrahedron)
}

// tetrahedron handles the tetrahedron simplex case (4 points: A, B, C, D).
//
// This is the only case that can return true (collision detected).
//
// Tests if origin is inside the tetrahedron by checking which side of each face
// the origin lies on:
//   - If outside face ABC → reduce to triangle ABC
//   - If outside face ACD → reduce to triangle ACD
//   - If outside face ADB → reduce to triangle ADB
//   - If inside all faces → origin contained, collision!
//
// Face normals must point outward (away from the 4th vertex) to correctly test
// which side of each face the origin is on.
//
// Returns true if origin is inside tetrahedron, false otherwise.
func tetrahedron(simplex *Simplex, direction *mgl64.Vec3) bool {
	a := simplex.Points[3] // Most recent point
	b := simplex.Points[2]
	c := simplex.Points[1]
	d := simplex.Points[0]

	ab := b.Sub(a)
	ac := c.Sub(a)
	ad := d.Sub(a)
	ao := a.Mul(-1)

	// Compute face normals
	// IMPORTANT: Normal direction must point AWAY from the 4th vertex
	// to correctly represent the "outside" of each face

	// Face ABC (opposite to D)
	abc := ab.Cross(ac)
	// Check if normal points toward D or away from D
	if abc.Dot(ad) > 0 {
		// Normal points toward D, we want it pointing away
		abc = abc.Mul(-1)
	}

	// Face ACD (opposite to B)
	acd := ac.Cross(ad)
	if acd.Dot(ab) > 0 {
		acd = acd.Mul(-1)
	}

	// Face ADB (opposite to C)
	adb := ad.Cross(ab)
	if adb.Dot(ac) > 0 {
		adb = adb.Mul(-1)
	}

	// Check for degenerate tetrahedron
	if abc.LenSqr() < 1e-10 || acd.LenSqr() < 1e-10 || adb.LenSqr() < 1e-10 {
		simplex.Points[0] = c
		simplex.Points[1] = b
		simplex.Points[2] = a
		simplex.Count = 3
		return triangle(simplex, direction)
	}

	// Now test if origin is outside any face
	// If abc.Dot(ao) > 0, origin is on the outside of face ABC

	// Face ABC
	if abc.Dot(ao) > 0 {
		simplex.Points[0] = c
		simplex.Points[1] = b
		simplex.Points[2] = a
		simplex.Count = 3
		return triangle(simplex, direction)
	}

	// Face ACD
	if acd.Dot(ao) > 0 {
		simplex.Points[0] = d
		simplex.Points[1] = c
		simplex.Points[2] = a
		simplex.Count = 3
		return triangle(simplex, direction)
	}

	// Face ADB
	if adb.Dot(ao) > 0 {
		simplex.Points[0] = b
		simplex.Points[1] = d
		simplex.Points[2] = a
		simplex.Count = 3
		return triangle(simplex, direction)
	}

	// The origin is inside the tetrahedron
	return true
}
