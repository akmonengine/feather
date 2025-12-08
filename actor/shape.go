package actor

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

// ShapeType represents the type of collision shape
type ShapeType int

const (
	ShapeTypeSphere ShapeType = iota
	ShapeTypeBox
	ShapeTypePlane
)

// ShapeInterface is the interface that all collision shapes must implement
type ShapeInterface interface {
	// ComputeAABB calculates the axis-aligned bounding box for the shape
	// at the given transform
	ComputeAABB(transform Transform)
	GetAABB() AABB
	// ComputeMass calculates mass data for the shape given a density
	ComputeMass(density float64) float64
	ComputeInertia(mass float64) mgl64.Mat3
	Support(direction mgl64.Vec3) mgl64.Vec3
	GetContactFeature(direction mgl64.Vec3) []mgl64.Vec3
}

// Box represents an oriented box collision shape
// The box is defined by its half-extents (half-width, half-height, half-depth)
type Box struct {
	HalfExtents mgl64.Vec3
	aabb        AABB
}

func (b *Box) ComputeAABB(transform Transform) {
	// Les 8 coins de la boîte en espace local
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

	// Transformer le premier coin pour initialiser min/max
	worldCorner := transform.Rotation.Rotate(corners[0]).Add(transform.Position)
	min := worldCorner
	max := worldCorner

	// Transformer tous les autres coins et étendre l'AABB
	for i := 1; i < 8; i++ {
		worldCorner = transform.Rotation.Rotate(corners[i]).Add(transform.Position)

		min[0] = math.Min(min[0], worldCorner[0])
		min[1] = math.Min(min[1], worldCorner[1])
		min[2] = math.Min(min[2], worldCorner[2])

		max[0] = math.Max(max[0], worldCorner[0])
		max[1] = math.Max(max[1], worldCorner[1])
		max[2] = math.Max(max[2], worldCorner[2])
	}

	b.aabb = AABB{Min: min, Max: max}
}

func (b *Box) GetAABB() AABB {
	return b.aabb
}

// ComputeMass calculates mass data for the box
func (b *Box) ComputeMass(density float64) float64 {
	// Volume = 8 * hx * hy * hz (full dimensions are 2*halfExtents)
	volume := 8.0 * b.HalfExtents.X() * b.HalfExtents.Y() * b.HalfExtents.Z()

	return density * volume
}

func (b *Box) ComputeInertia(mass float64) mgl64.Mat3 {
	// Dimensions complètes
	x := b.HalfExtents.X() * 2
	y := b.HalfExtents.Y() * 2
	z := b.HalfExtents.Z() * 2

	// Formule pour une boîte : I = (m/12) * (dimension1² + dimension2²)
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

func (b *Box) GetContactFeature(direction mgl64.Vec3) []mgl64.Vec3 {
	dir := direction.Normalize()

	// Trouver la face la plus parallèle à la direction
	// (celle dont la normale pointe le plus dans la direction)
	bestDot := -math.MaxFloat64
	var bestFace []mgl64.Vec3

	hx := b.HalfExtents.X()
	hy := b.HalfExtents.Y()
	hz := b.HalfExtents.Z()

	// Les 6 faces avec leurs vertices (ordre CCW vu de l'extérieur)
	faces := []struct {
		normal   mgl64.Vec3
		vertices []mgl64.Vec3
	}{
		// +X face
		{
			normal: mgl64.Vec3{1, 0, 0},
			vertices: []mgl64.Vec3{
				{hx, -hy, -hz},
				{hx, -hy, hz},
				{hx, hy, hz},
				{hx, hy, -hz},
			},
		},
		// -X face
		{
			normal: mgl64.Vec3{-1, 0, 0},
			vertices: []mgl64.Vec3{
				{-hx, -hy, hz},
				{-hx, -hy, -hz},
				{-hx, hy, -hz},
				{-hx, hy, hz},
			},
		},
		// +Y face
		{
			normal: mgl64.Vec3{0, 1, 0},
			vertices: []mgl64.Vec3{
				{-hx, hy, -hz},
				{-hx, hy, hz},
				{hx, hy, hz},
				{hx, hy, -hz},
			},
		},
		// -Y face
		{
			normal: mgl64.Vec3{0, -1, 0},
			vertices: []mgl64.Vec3{
				{-hx, -hy, hz},
				{hx, -hy, hz},
				{hx, -hy, -hz},
				{-hx, -hy, -hz},
			},
		},
		// +Z face
		{
			normal: mgl64.Vec3{0, 0, 1},
			vertices: []mgl64.Vec3{
				{-hx, -hy, hz},
				{-hx, hy, hz},
				{hx, hy, hz},
				{hx, -hy, hz},
			},
		},
		// -Z face
		{
			normal: mgl64.Vec3{0, 0, -1},
			vertices: []mgl64.Vec3{
				{hx, -hy, -hz},
				{hx, hy, -hz},
				{-hx, hy, -hz},
				{-hx, -hy, -hz},
			},
		},
	}

	// Trouver la meilleure face
	for _, face := range faces {
		dot := dir.Dot(face.normal)
		if dot > bestDot {
			bestDot = dot
			bestFace = face.vertices
		}
	}

	return bestFace
}

// Sphere represents a spherical collision shape
type Sphere struct {
	Radius float64
	aabb   AABB
}

// ComputeAABB calculates the axis-aligned bounding box for the sphere
func (s *Sphere) ComputeAABB(transform Transform) {
	// Sphere AABB is not affected by rotation, only by position
	radiusVec := mgl64.Vec3{s.Radius, s.Radius, s.Radius}

	s.aabb = AABB{
		Min: transform.Position.Sub(radiusVec),
		Max: transform.Position.Add(radiusVec),
	}
}

func (s *Sphere) GetAABB() AABB {
	return s.aabb
}

// ComputeMass calculates mass data for the sphere
func (s *Sphere) ComputeMass(density float64) float64 {
	// Volume of sphere = (4/3) * π * r³
	volume := (4.0 / 3.0) * math.Pi * math.Pow(s.Radius, 3)

	return density * volume
}

func (s *Sphere) ComputeInertia(mass float64) mgl64.Mat3 {
	// Pour une sphère : I = (2/5) * m * r²
	i := (2.0 / 5.0) * mass * s.Radius * s.Radius

	// Une sphère a la même inertie sur tous les axes
	return mgl64.Mat3{
		i, 0, 0,
		0, i, 0,
		0, 0, i,
	}
}

func (s *Sphere) Support(direction mgl64.Vec3) mgl64.Vec3 {
	return direction.Normalize().Mul(s.Radius)
}

func (s *Sphere) GetContactFeature(direction mgl64.Vec3) []mgl64.Vec3 {
	return []mgl64.Vec3{s.Support(direction)}
}

// Plane represents an infinite plane collision shape
// The plane is defined by the equation: Normal · p + Distance = 0
// where Normal is the plane's normal vector (must be normalized)
// and Distance is the signed distance from the origin along the normal
type Plane struct {
	Normal   mgl64.Vec3 // Plane normal (must be normalized)
	Distance float64    // Plane constant (signed distance from origin)
	aabb     AABB
}

func (p *Plane) ComputeAABB(transform Transform) {
	const thickness = 1.0 // épaisseur de détection du plan
	const infinity = 1e10 // grande valeur pour les dimensions infinies

	// Point on the plane closest to the origin
	// Assumes p.Normal is normalized
	planePoint := p.Normal.Mul(-p.Distance)

	// Create base bounds with thickness along the normal
	min := planePoint.Sub(p.Normal.Mul(thickness)).Add(transform.Position)
	max := planePoint.Add(transform.Position)

	// Extend the AABB to infinity in directions perpendicular to the normal
	absNormal := mgl64.Vec3{
		math.Abs(p.Normal.X()),
		math.Abs(p.Normal.Y()),
		math.Abs(p.Normal.Z()),
	}

	// Find the dominant axis (the one aligned with the normal)
	threshold := 1.0 // threshold to consider an axis as dominant

	// For NON-dominant axes, extend to infinity
	if absNormal.X() < threshold {
		min[0] = -infinity
		max[0] = infinity
	}
	if absNormal.Y() < threshold {
		min[1] = -infinity
		max[1] = infinity
	}
	if absNormal.Z() < threshold {
		min[2] = -infinity
		max[2] = infinity
	}

	p.aabb = AABB{Min: min, Max: max}
}

func (p *Plane) GetAABB() AABB {
	return p.aabb
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

// For simplicity, we use a 10000 width/height box. Can obviously break for bigger planes
func (p *Plane) Support(direction mgl64.Vec3) mgl64.Vec3 {
	boxHalfWidth := 1000.0
	boxHalfHeight := 0.5
	boxHalfDepth := 1000.0

	return mgl64.Vec3{
		func() float64 {
			if direction.X() < 0 {
				return -boxHalfWidth
			}
			return boxHalfWidth
		}(),
		func() float64 {
			if direction.Y() > 0 {
				return 0.0
			}
			return -boxHalfHeight
		}(),
		func() float64 {
			if direction.Z() < 0 {
				return -boxHalfDepth
			}
			return boxHalfDepth
		}(),
	}
}

func (p *Plane) GetContactFeature(direction mgl64.Vec3) []mgl64.Vec3 {
	// For a plane, return 4 points forming a large square
	// IN LOCAL SPACE (centered on the plane origin)

	// Find two tangent vectors to the plane
	tangent1, tangent2 := getTangentBasis(p.Normal)

	// Large size to cover contacts
	size := 1000.0

	// Points in local plane space (not transformed)
	return []mgl64.Vec3{
		tangent1.Mul(-size).Add(tangent2.Mul(-size)),
		tangent1.Mul(-size).Add(tangent2.Mul(size)),
		tangent1.Mul(size).Add(tangent2.Mul(size)),
		tangent1.Mul(size).Add(tangent2.Mul(-size)),
	}
}

// Helper to generate the tangent basis
func getTangentBasis(normal mgl64.Vec3) (mgl64.Vec3, mgl64.Vec3) {
	var tangent1 mgl64.Vec3
	if math.Abs(normal.X()) > 0.9 {
		tangent1 = mgl64.Vec3{0, 1, 0}
	} else {
		tangent1 = mgl64.Vec3{1, 0, 0}
	}

	tangent1 = tangent1.Sub(normal.Mul(tangent1.Dot(normal))).Normalize()
	tangent2 := normal.Cross(tangent1).Normalize()

	return tangent1, tangent2
}
