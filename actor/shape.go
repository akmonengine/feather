package actor

import (
	"math"
	"sync"

	"github.com/go-gl/mathgl/mgl64"
)

// ShapeType represents the type of collision shape
type ShapeType int

const (
	ShapeTypeSphere ShapeType = iota
	ShapeTypeBox
	ShapeTypePlane
)

var (
	vec3Pool = sync.Pool{
		New: func() interface{} {
			return &mgl64.Vec3{}
		},
	}
	vec3SlicePool4 = sync.Pool{
		New: func() interface{} {
			s := make([]*mgl64.Vec3, 4)
			return &s
		},
	}
	vec3SlicePool24 = sync.Pool{
		New: func() interface{} {
			s := make([]*mgl64.Vec3, 24)
			return &s
		},
	}
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
	hx, hy, hz := b.HalfExtents.X(), b.HalfExtents.Y(), b.HalfExtents.Z()

	allVerticesPtr := vec3SlicePool24.Get().(*[]*mgl64.Vec3)
	allVertices := *allVerticesPtr
	for i := 0; i < 24; i++ {
		allVertices[i] = vec3Pool.Get().(*mgl64.Vec3)
	}

	// +X face
	allVertices[0][0], allVertices[0][1], allVertices[0][2] = hx, -hy, -hz
	allVertices[1][0], allVertices[1][1], allVertices[1][2] = hx, -hy, hz
	allVertices[2][0], allVertices[2][1], allVertices[2][2] = hx, hy, hz
	allVertices[3][0], allVertices[3][1], allVertices[3][2] = hx, hy, -hz
	// -X face
	allVertices[4][0], allVertices[4][1], allVertices[4][2] = -hx, -hy, hz
	allVertices[5][0], allVertices[5][1], allVertices[5][2] = -hx, -hy, -hz
	allVertices[6][0], allVertices[6][1], allVertices[6][2] = -hx, hy, -hz
	allVertices[7][0], allVertices[7][1], allVertices[7][2] = -hx, hy, hz
	// +Y face
	allVertices[8][0], allVertices[8][1], allVertices[8][2] = -hx, hy, -hz
	allVertices[9][0], allVertices[9][1], allVertices[9][2] = -hx, hy, hz
	allVertices[10][0], allVertices[10][1], allVertices[10][2] = hx, hy, hz
	allVertices[11][0], allVertices[11][1], allVertices[11][2] = hx, hy, -hz
	// -Y face
	allVertices[12][0], allVertices[12][1], allVertices[12][2] = -hx, -hy, hz
	allVertices[13][0], allVertices[13][1], allVertices[13][2] = hx, -hy, hz
	allVertices[14][0], allVertices[14][1], allVertices[14][2] = hx, -hy, -hz
	allVertices[15][0], allVertices[15][1], allVertices[15][2] = -hx, -hy, -hz
	// +Z face
	allVertices[16][0], allVertices[16][1], allVertices[16][2] = -hx, -hy, hz
	allVertices[17][0], allVertices[17][1], allVertices[17][2] = -hx, hy, hz
	allVertices[18][0], allVertices[18][1], allVertices[18][2] = hx, hy, hz
	allVertices[19][0], allVertices[19][1], allVertices[19][2] = hx, -hy, hz
	// -Z face
	allVertices[20][0], allVertices[20][1], allVertices[20][2] = hx, -hy, -hz
	allVertices[21][0], allVertices[21][1], allVertices[21][2] = hx, hy, -hz
	allVertices[22][0], allVertices[22][1], allVertices[22][2] = -hx, hy, -hz
	allVertices[23][0], allVertices[23][1], allVertices[23][2] = -hx, -hy, -hz

	bestDot := -math.MaxFloat64
	bestFaceIdx := 0
	faceNormals := []mgl64.Vec3{
		{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
	}
	for i := 0; i < 6; i++ {
		dot := dir.Dot(faceNormals[i])
		if dot > bestDot {
			bestDot = dot
			bestFaceIdx = i
		}
	}

	resultSlicePtr := vec3SlicePool4.Get().(*[]*mgl64.Vec3)
	resultSlice := *resultSlicePtr
	startIdx := bestFaceIdx * 4
	for i := 0; i < 4; i++ {
		resultSlice[i] = allVertices[startIdx+i]
	}

	result := make([]mgl64.Vec3, 4)
	for i := 0; i < 4; i++ {
		result[i] = *resultSlice[i]
	}

	for i := 0; i < 24; i++ {
		vec3Pool.Put(allVertices[i])
	}
	vec3SlicePool24.Put(allVerticesPtr)
	vec3SlicePool4.Put(resultSlicePtr)

	return result
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
