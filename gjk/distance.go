package gjk

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

const (
	// distanceTolerance: GJK stops when the next support point gets the distance closer by less than this ratio
	distanceTolerance = 1e-10

	// overlapEpsilon: under this squared distance (relative to the size of the simplex), the shapes overlap
	overlapEpsilon = 1e-20
)

// DistanceResult: the distance between 2 convex shapes and their closest points (world space).
// Normal goes from A to B. If the shapes overlap, Distance is 0 and Overlap is true
type DistanceResult struct {
	Distance float64
	PointA   mgl64.Vec3
	PointB   mgl64.Vec3
	Normal   mgl64.Vec3
	Overlap  bool
}

// distanceSimplex: the vertices of the simplex with their barycentric weights of the closest point to the origin
type distanceSimplex struct {
	vertices [4]Vertex
	weights  [4]float64
	count    int
}

// Distance between 2 convex shapes: GJK (Gilbert, Johnson, Keerthi; Ericson 9.5).
// The simplex is reduced to the feature closest to the origin (Voronoi regions, Ericson 5.1)
func Distance(a, b *Proxy) DistanceResult {
	var s distanceSimplex
	direction := b.Position.Sub(a.Position)
	if direction.LenSqr() == 0 {
		direction = mgl64.Vec3{1, 0, 0}
	}
	s.vertices[0] = SupportProxies(a, b, direction.Mul(-1), 0)
	s.weights[0] = 1
	s.count = 1

	for i := 0; i < maxIterations; i++ {
		v := s.closest()
		size := s.size()
		if v.LenSqr() <= overlapEpsilon*size {
			return DistanceResult{Overlap: true}
		}

		w := SupportProxies(a, b, v.Mul(-1), 0)
		// no progress: v is the closest point of the Minkowski difference
		if v.LenSqr()-v.Dot(w.W) <= distanceTolerance*v.LenSqr() || s.has(w.W) {
			break
		}
		s.vertices[s.count] = w
		s.count++
		if !s.reduce() {
			return DistanceResult{Overlap: true}
		}
	}

	pointA, pointB := mgl64.Vec3{}, mgl64.Vec3{}
	for k := 0; k < s.count; k++ {
		pointA = pointA.Add(s.vertices[k].A.Mul(s.weights[k]))
		pointB = pointB.Add(s.vertices[k].B.Mul(s.weights[k]))
	}
	distance := pointB.Sub(pointA).Len()
	if distance == 0 {
		return DistanceResult{Overlap: true}
	}
	return DistanceResult{Distance: distance, PointA: pointA, PointB: pointB, Normal: pointB.Sub(pointA).Mul(1 / distance)}
}

// closest point of the simplex to the origin, from the weights
func (s *distanceSimplex) closest() mgl64.Vec3 {
	v := mgl64.Vec3{}
	for k := 0; k < s.count; k++ {
		v = v.Add(s.vertices[k].W.Mul(s.weights[k]))
	}
	return v
}

func (s *distanceSimplex) size() float64 {
	size := 0.0
	for k := 0; k < s.count; k++ {
		size = math.Max(size, s.vertices[k].W.LenSqr())
	}
	return size
}

func (s *distanceSimplex) has(w mgl64.Vec3) bool {
	for k := 0; k < s.count; k++ {
		if s.vertices[k].W == w {
			return true
		}
	}
	return false
}

// reduce the simplex to the smallest feature containing its closest point to the origin, with its weights.
// Returns false if the tetrahedron contains the origin
func (s *distanceSimplex) reduce() bool {
	switch s.count {
	case 2:
		s.segment(0, 1)
	case 3:
		s.triangle(0, 1, 2)
	case 4:
		return s.tetrahedron()
	}
	return true
}

func (s *distanceSimplex) keep(indices ...int) {
	var vertices [4]Vertex
	for k, i := range indices {
		vertices[k] = s.vertices[i]
	}
	s.vertices = vertices
	s.count = len(indices)
}

// segment [a, b]: closest point to the origin
func (s *distanceSimplex) segment(ia, ib int) {
	a, b := s.vertices[ia].W, s.vertices[ib].W
	ab := b.Sub(a)
	t := -a.Dot(ab)
	if t <= 0 {
		s.keep(ia)
		s.weights[0] = 1
		return
	}
	length := ab.LenSqr()
	if t >= length {
		s.keep(ib)
		s.weights[0] = 1
		return
	}
	t /= length
	s.keep(ia, ib)
	s.weights[0], s.weights[1] = 1-t, t
}

// triangle [a, b, c]: closest point to the origin (Ericson 5.1.5)
func (s *distanceSimplex) triangle(ia, ib, ic int) {
	a, b, c := s.vertices[ia].W, s.vertices[ib].W, s.vertices[ic].W
	ab, ac, ap := b.Sub(a), c.Sub(a), a.Mul(-1)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		s.keep(ia)
		s.weights[0] = 1
		return
	}
	bp := b.Mul(-1)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		s.keep(ib)
		s.weights[0] = 1
		return
	}
	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		t := d1 / (d1 - d3)
		s.keep(ia, ib)
		s.weights[0], s.weights[1] = 1-t, t
		return
	}
	cp := c.Mul(-1)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		s.keep(ic)
		s.weights[0] = 1
		return
	}
	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		t := d2 / (d2 - d6)
		s.keep(ia, ic)
		s.weights[0], s.weights[1] = 1-t, t
		return
	}
	va := d3*d6 - d5*d4
	if va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		t := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		s.keep(ib, ic)
		s.weights[0], s.weights[1] = 1-t, t
		return
	}
	denominator := 1 / (va + vb + vc)
	v, w := vb*denominator, vc*denominator
	s.keep(ia, ib, ic)
	s.weights[0], s.weights[1], s.weights[2] = 1-v-w, v, w
}

// tetrahedron: the closest point is on the faces the origin is in front of (Ericson 5.1.6).
// Returns false if the origin is inside
func (s *distanceSimplex) tetrahedron() bool {
	faces := [4][4]int{{0, 1, 2, 3}, {0, 2, 3, 1}, {0, 3, 1, 2}, {1, 3, 2, 0}}
	best, bestDistance := *s, math.Inf(1)
	inside := true
	for _, face := range faces {
		a, b, c, d := s.vertices[face[0]].W, s.vertices[face[1]].W, s.vertices[face[2]].W, s.vertices[face[3]].W
		n := b.Sub(a).Cross(c.Sub(a))
		// the origin and the opposite vertex on both sides of the face
		if a.Mul(-1).Dot(n)*d.Sub(a).Dot(n) >= 0 {
			continue
		}
		inside = false
		candidate := *s
		candidate.triangle(face[0], face[1], face[2])
		if distance := candidate.closest().LenSqr(); distance < bestDistance {
			best, bestDistance = candidate, distance
		}
	}
	if inside {
		return false
	}
	*s = best
	return true
}
