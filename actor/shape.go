package actor

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

// ContactPoint is a contact against a plane: Position lies halfway between the shape's
// surface and the plane, Separation is their signed distance (negative when overlapping).
type ContactPoint struct {
	Position   mgl64.Vec3
	Separation float64
}

type PlaneContact []ContactPoint

// ShapeInterface is the interface that all collision shapes must implement
type ShapeInterface interface {
	// ComputeAABB returns the axis-aligned bounding box of the shape at the transform.
	// A shape has no state: it can be shared by several bodies, each body keeps its AABB
	ComputeAABB(transform Transform) AABB
	// ComputeMass calculates mass data for the shape given a density
	ComputeMass(density float64) float64
	ComputeInertia(mass float64) mgl64.Mat3
	Support(direction mgl64.Vec3) mgl64.Vec3
	GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int)
	// CollideWithPlane appends to contacts the points of the shape closer to the plane than the margin
	// (plane: planeNormal·p + planeDistance = 0). contacts is a buffer given by the caller, to avoid allocations
	CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact
	// CastRay: the first point of the segment origin + f*translation, f in [0, maxFraction], on the shape, in its
	// local space (see RayHit)
	CastRay(origin, translation mgl64.Vec3, maxFraction float64) (RayHit, bool)
}

// Box represents an oriented box collision shape
// The box is defined by its half-extents (half-width, half-height, half-depth)
type Box struct {
	HalfExtents mgl64.Vec3
}

func (b *Box) ComputeAABB(transform Transform) AABB {
	// the 8 corners of the box, in the local space
	corners := [8]mgl64.Vec3{
		{-b.HalfExtents.X(), -b.HalfExtents.Y(), -b.HalfExtents.Z()},
		{+b.HalfExtents.X(), -b.HalfExtents.Y(), -b.HalfExtents.Z()},
		{-b.HalfExtents.X(), +b.HalfExtents.Y(), -b.HalfExtents.Z()},
		{+b.HalfExtents.X(), +b.HalfExtents.Y(), -b.HalfExtents.Z()},
		{-b.HalfExtents.X(), -b.HalfExtents.Y(), +b.HalfExtents.Z()},
		{+b.HalfExtents.X(), -b.HalfExtents.Y(), +b.HalfExtents.Z()},
		{-b.HalfExtents.X(), +b.HalfExtents.Y(), +b.HalfExtents.Z()},
		{+b.HalfExtents.X(), +b.HalfExtents.Y(), +b.HalfExtents.Z()},
	}

	// the first corner initializes min & max, the other corners extend the AABB
	q, position := &transform.Rotation, transform.Position
	worldCorner := Rotate(q, corners[0]).Add(position)
	min := worldCorner
	max := worldCorner
	for i := 1; i < 8; i++ {
		worldCorner = Rotate(q, corners[i]).Add(position)
		for k := 0; k < 3; k++ {
			if worldCorner[k] < min[k] {
				min[k] = worldCorner[k]
			}
			if worldCorner[k] > max[k] {
				max[k] = worldCorner[k]
			}
		}
	}

	return AABB{Min: min, Max: max}
}

// ComputeMass calculates mass data for the box
func (b *Box) ComputeMass(density float64) float64 {
	// Volume = 8 * hx * hy * hz (full dimensions are 2*halfExtents)
	volume := 8.0 * b.HalfExtents.X() * b.HalfExtents.Y() * b.HalfExtents.Z()

	return density * volume
}

func (b *Box) ComputeInertia(mass float64) mgl64.Mat3 {
	// full dimensions
	x := b.HalfExtents.X() * 2
	y := b.HalfExtents.Y() * 2
	z := b.HalfExtents.Z() * 2

	// box: I = (m/12) * (dimension1² + dimension2²)
	factor := mass / 12.0
	ix := factor * (y*y + z*z)
	iy := factor * (x*x + z*z)
	iz := factor * (x*x + y*y)

	return mgl64.Mat3{
		ix, 0, 0,
		0, iy, 0,
		0, 0, iz,
	}
}

func (b *Box) Support(direction mgl64.Vec3) mgl64.Vec3 {
	hx, hy, hz := b.HalfExtents.X(), b.HalfExtents.Y(), b.HalfExtents.Z()

	if direction.X() < 0 {
		hx = -hx
	}
	if direction.Y() < 0 {
		hy = -hy
	}
	if direction.Z() < 0 {
		hz = -hz
	}

	return mgl64.Vec3{hx, hy, hz}
}

func (b *Box) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	// the face the most aligned with the direction
	axes := [3]mgl64.Vec3{
		{1, 0, 0}, {0, 1, 0}, {0, 0, 1},
	}

	maxAbsDot := 0.0
	bestAxisIdx := 0
	sign := 1.0

	for i, axis := range axes {
		dot := direction.Dot(axis)
		absDot := math.Abs(dot)

		if absDot > maxAbsDot {
			maxAbsDot = absDot
			bestAxisIdx = i
			if dot > 0 {
				sign = 1
			} else {
				sign = -1
			}
		}
	}

	halfSize := b.HalfExtents

	// the 4 corners of the face
	switch bestAxisIdx {
	case 0:
		x := sign * halfSize.X()
		output[0] = mgl64.Vec3{x, -halfSize.Y(), -halfSize.Z()}
		output[1] = mgl64.Vec3{x, -halfSize.Y(), halfSize.Z()}
		output[2] = mgl64.Vec3{x, halfSize.Y(), halfSize.Z()}
		output[3] = mgl64.Vec3{x, halfSize.Y(), -halfSize.Z()}
		*count = 4
	case 1:
		y := sign * halfSize.Y()
		output[0] = mgl64.Vec3{-halfSize.X(), y, -halfSize.Z()}
		output[1] = mgl64.Vec3{-halfSize.X(), y, halfSize.Z()}
		output[2] = mgl64.Vec3{halfSize.X(), y, halfSize.Z()}
		output[3] = mgl64.Vec3{halfSize.X(), y, -halfSize.Z()}
		*count = 4
	default:
		z := sign * halfSize.Z()
		output[0] = mgl64.Vec3{-halfSize.X(), -halfSize.Y(), z}
		output[1] = mgl64.Vec3{halfSize.X(), -halfSize.Y(), z}
		output[2] = mgl64.Vec3{halfSize.X(), halfSize.Y(), z}
		output[3] = mgl64.Vec3{-halfSize.X(), halfSize.Y(), z}
		*count = 4
	}
}

// CollideWithPlane returns the corners of the supporting face of the box (the face the most opposed to the normal of the
// plane, as the incident face of Jolt) closer to the plane than the margin. The other corners are behind this face:
// with a large margin, a thin box would give the corners of its top face instead of the deepest ones
func (b *Box) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact {
	var face [8]mgl64.Vec3
	var count int
	q := &myTransform.Rotation
	b.GetContactFeature(RotateInverse(q, planeNormal.Mul(-1)), &face, &count)

	for _, vertex := range face[:count] {
		worldVertex := myTransform.Position.Add(Rotate(q, vertex))
		separation := worldVertex.Dot(planeNormal) + planeDistance
		if separation > margin {
			continue
		}
		contacts = append(contacts, ContactPoint{
			Position:   worldVertex.Sub(planeNormal.Mul(separation / 2)),
			Separation: separation,
		})
	}

	return contacts
}

// CastRay by slabs, as RayAABox of Jolt: the ray is in the box while it is between its 3 pairs of faces at once. The
// normal is the one of the face it enters by
func (b *Box) CastRay(origin, translation mgl64.Vec3, maxFraction float64) (RayHit, bool) {
	if !finiteRay(origin, translation) {
		return RayHit{}, false
	}
	h := b.HalfExtents
	if math.Abs(origin[0]) <= h[0] && math.Abs(origin[1]) <= h[1] && math.Abs(origin[2]) <= h[2] {
		return startInside(translation), true
	}
	enter, exit := 0.0, maxFraction
	axis, side := -1, 0.0
	for k := 0; k < 3; k++ {
		if translation[k] == 0 {
			// along the faces: between them all the way, or never
			if math.Abs(origin[k]) > h[k] {
				return RayHit{}, false
			}
			continue
		}
		// the face the ray meets first is on the side it comes from
		near := 1.0
		if translation[k] > 0 {
			near = -1
		}
		if fraction := (near*h[k] - origin[k]) / translation[k]; fraction > enter {
			enter, axis, side = fraction, k, near
		}
		exit = math.Min(exit, (-near*h[k]-origin[k])/translation[k])
		if enter > exit {
			return RayHit{}, false
		}
	}
	if axis < 0 {
		return RayHit{}, false
	}
	hit := RayHit{Fraction: enter, Triangle: NoTriangle}
	hit.Normal[axis] = side
	return hit, true
}

// Sphere represents a spherical collision shape
type Sphere struct {
	Radius float64
}

// ComputeAABB calculates the axis-aligned bounding box for the sphere
func (s *Sphere) ComputeAABB(transform Transform) AABB {
	// Sphere AABB is not affected by rotation, only by position
	radiusVec := mgl64.Vec3{s.Radius, s.Radius, s.Radius}

	return AABB{
		Min: transform.Position.Sub(radiusVec),
		Max: transform.Position.Add(radiusVec),
	}
}

// ComputeMass calculates mass data for the sphere
func (s *Sphere) ComputeMass(density float64) float64 {
	// Volume of sphere = (4/3) * π * r³
	volume := (4.0 / 3.0) * math.Pi * s.Radius * s.Radius * s.Radius

	return density * volume
}

func (s *Sphere) ComputeInertia(mass float64) mgl64.Mat3 {
	// sphere: I = (2/5) * m * r²
	i := (2.0 / 5.0) * mass * s.Radius * s.Radius

	// the same inertia on all the axes
	return mgl64.Mat3{
		i, 0, 0,
		0, i, 0,
		0, 0, i,
	}
}

func (s *Sphere) Support(direction mgl64.Vec3) mgl64.Vec3 {
	length := direction.Len()
	if length == 0 {
		return mgl64.Vec3{0, s.Radius, 0}
	}
	return direction.Mul(s.Radius / length)
}

func (s *Sphere) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	output[0] = s.Support(direction)
	*count = 1
}

// CollideWithPlane returns the lowest point of the sphere, if closer to the plane than the margin
func (s *Sphere) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact {
	center := myTransform.Position
	separation := center.Dot(planeNormal) + planeDistance - s.Radius
	if separation > margin {
		return contacts
	}

	return append(contacts, ContactPoint{
		Position:   center.Sub(planeNormal.Mul(s.Radius + separation/2)),
		Separation: separation,
	})
}

// CastRay: where the ray enters the sphere
func (s *Sphere) CastRay(origin, translation mgl64.Vec3, maxFraction float64) (RayHit, bool) {
	if !finiteRay(origin, translation) {
		return RayHit{}, false
	}
	if origin.LenSqr() <= s.Radius*s.Radius {
		return startInside(translation), true
	}
	fraction, ok := raySphere(origin, translation, s.Radius, maxFraction)
	if !ok {
		return RayHit{}, false
	}
	return RayHit{Fraction: fraction, Normal: origin.Add(translation.Mul(fraction)).Normalize(), Triangle: NoTriangle}, true
}

// Plane represents an infinite plane collision shape
// The plane is defined by the equation: Normal · p + Distance = 0
// where Normal is the plane's normal vector (must be normalized)
// and Distance is the signed distance from the origin along the normal
type Plane struct {
	Normal   mgl64.Vec3 // Plane normal (must be normalized)
	Distance float64    // Plane constant (signed distance from origin)
}

// ComputeAABB: a plane is infinite, its AABB is the whole space. The planes are tested with every body
func (p *Plane) ComputeAABB(transform Transform) AABB {
	infinity := math.Inf(1)
	return AABB{Min: mgl64.Vec3{-infinity, -infinity, -infinity}, Max: mgl64.Vec3{infinity, infinity, infinity}}
}

// ComputeMass calculates mass data for the plane
// Planes are always static with infinite mass
func (p *Plane) ComputeMass(density float64) float64 {
	// Static planes have infinite mass
	// (they cannot be moved by collisions)
	return math.Inf(1)
}

func (p *Plane) ComputeInertia(mass float64) mgl64.Mat3 {
	return mgl64.Mat3{}
}

// Support: a plane has no support point, the narrow phase uses CollideWithPlane of the other shape
func (p *Plane) Support(direction mgl64.Vec3) mgl64.Vec3 {
	return mgl64.Vec3{}
}

// The narrow phase has specific code path for planes, this should not be called
func (p *Plane) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	output[0] = mgl64.Vec3{0, 0, 0}
	*count = 1
}

// CollideWithPlane - Plane/Plane collision (not supported)
func (p *Plane) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact {
	return contacts
}

// CastRay against the half space under the plane, as PlaneShape::CastRay of Jolt. The plane is in world space
// (Normal·p + Distance = 0): the ray too, whatever the transform of the body
func (p *Plane) CastRay(origin, translation mgl64.Vec3, maxFraction float64) (RayHit, bool) {
	if !finiteRay(origin, translation) {
		return RayHit{}, false
	}
	height := origin.Dot(p.Normal) + p.Distance
	if height <= 0 {
		return startInside(translation), true
	}
	descent := -translation.Dot(p.Normal)
	if descent <= 0 || height > maxFraction*descent {
		return RayHit{}, false
	}
	return RayHit{Fraction: height / descent, Normal: p.Normal, Triangle: NoTriangle}, true
}

// ========== RAYS ==========
// A ray is a segment: its origin and its translation, in the local space of the shape. Its hit is its first point in
// the shape, at the fraction of the translation where it is (the convention of Box3D: b3RayCastInput). The shapes are
// solid: a ray which starts in a shape hits it at its origin, the normal against its direction (the raycasts of PhysX,
// mTreatConvexAsSolid of Jolt). A ray without length is a point: a hit without normal if it is in the shape.

// NoTriangle: the triangle of a hit on a shape which has none
const NoTriangle int32 = -1

// RayHit of a ray against a shape, in the local space of the shape
type RayHit struct {
	Fraction float64    // of the translation, in [0, 1]
	Normal   mgl64.Vec3 // unit, out of the shape
	Triangle int32      // heightfield: 2*cell + t (cell = x*(ZSamples-1)+z), else NoTriangle
}

// finiteRay: a ray with a NaN or an infinite coordinate hits nothing (x - x is 0 for a finite x only)
func finiteRay(origin, translation mgl64.Vec3) bool {
	sum := 0.0
	for k := 0; k < 3; k++ {
		sum += (origin[k] - origin[k]) + (translation[k] - translation[k])
	}
	return sum == 0
}

// startInside: the hit of a ray which starts in the shape
func startInside(translation mgl64.Vec3) RayHit {
	hit := RayHit{Triangle: NoTriangle}
	if length := translation.Len(); length > 0 {
		hit.Normal = translation.Mul(-1 / length)
	}
	return hit
}

// raySphere: the fraction where a ray which starts out of the sphere of center 0 enters it. The closest point of the
// line to the center is found first, then its distance to the surface along the line: the difference of 2 squares of
// the quadratic never mixes the distance of the origin with the radius (b3RayCastSphere of Box3D, after "Precision
// Improvements for Ray/Sphere Intersection", Ray Tracing Gems, 2019)
func raySphere(origin, translation mgl64.Vec3, radius, maxFraction float64) (float64, bool) {
	lengthSqr := translation.LenSqr()
	closest := -origin.Dot(translation) / lengthSqr
	// the ray goes away from the center, or has no length
	if !(closest > 0) {
		return 0, false
	}
	inside := radius*radius - origin.Add(translation.Mul(closest)).LenSqr()
	if inside < 0 {
		return 0, false
	}
	fraction := math.Max(0, closest-math.Sqrt(inside/lengthSqr))
	return fraction, fraction <= maxFraction
}
