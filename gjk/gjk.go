// Package gjk implements the Gilbert-Johnson-Keerthi (GJK) algorithm for collision detection.
//
// GJK detects whether two convex shapes overlap by testing if their Minkowski difference
// contains the origin. The algorithm builds a simplex incrementally, converging toward
// the origin in typically 3-6 iterations.
//
// Each vertex of the simplex keeps its support points on A and B, for the witness points of EPA.
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

const (
	// maxIterations: safety limit to prevent infinite loops
	maxIterations = 64

	// degenerateEpsilon: the direction is null (the origin is on the simplex), relative to the size of the simplex
	degenerateEpsilon = 1e-24

	// hullEpsilon: the new point is on the same point/line/plane as the simplex, relative to its size
	hullEpsilon = 1e-18
)

// Vertex of the Minkowski difference, with its support points: W = A - B
type Vertex struct {
	W mgl64.Vec3
	A mgl64.Vec3
	B mgl64.Vec3
}

// Simplex represents a set of 1-4 points in the Minkowski difference space.
// The last point is always the most recent
type Simplex struct {
	Points [4]mgl64.Vec3
	A      [4]mgl64.Vec3
	B      [4]mgl64.Vec3
	Count  int
}

func (s *Simplex) Reset() {
	s.Count = 0
}

func (s *Simplex) Vertex(i int) Vertex {
	return Vertex{W: s.Points[i], A: s.A[i], B: s.B[i]}
}

func (s *Simplex) set(vertices ...Vertex) {
	for i, v := range vertices {
		s.Points[i], s.A[i], s.B[i] = v.W, v.A, v.B
	}
	s.Count = len(vertices)
}

var SimplexPool = sync.Pool{
	New: func() interface{} {
		return &Simplex{}
	},
}

// Supporter is a convex shape, by its support function: the farthest point in a direction, in local space
type Supporter interface {
	Support(direction mgl64.Vec3) mgl64.Vec3
}

// Proxy is a body prepared for the support queries: its rotation as matrices, computed once per pair
// instead of rotating each direction and each point with the quaternion
type Proxy struct {
	Position mgl64.Vec3
	Rotation mgl64.Mat3 // local to world
	Inverse  mgl64.Mat3 // world to local
	Shape    Supporter
}

func NewProxy(body *actor.RigidBody) Proxy {
	return NewProxyAt(body.Transform, body.Shape)
}

// ========== CORES ==========
// The core of a rounded shape is the shape without its radius (the convex radius of Bullet & Jolt): a point for a
// sphere, a segment for a capsule. The distance between the cores is the distance between the shapes minus the radii,
// and GJK finds it exactly against a polytope, where EPA on the rounded shape would tessellate it (13 iterations for a
// sphere against a box)

type sphereCore actor.Sphere

func (*sphereCore) Support(mgl64.Vec3) mgl64.Vec3 { return mgl64.Vec3{} }

type capsuleCore actor.Capsule

func (c *capsuleCore) Support(direction mgl64.Vec3) mgl64.Vec3 {
	if direction.Y() < 0 {
		return mgl64.Vec3{0, -c.HalfHeight, 0}
	}
	return mgl64.Vec3{0, c.HalfHeight, 0}
}

// NewCoreProxy: the core of the body and its radius. The other shapes are their own core, with a radius of 0
func NewCoreProxy(body *actor.RigidBody) (Proxy, float64) {
	switch shape := body.Shape.(type) {
	case *actor.Sphere:
		return NewProxyAt(body.Transform, (*sphereCore)(shape)), shape.Radius
	case *actor.Capsule:
		return NewProxyAt(body.Transform, (*capsuleCore)(shape)), shape.Radius
	}
	return NewProxy(body), 0
}

// NewProxyAt: the shape at the transform (a body during its motion)
func NewProxyAt(transform actor.Transform, shape Supporter) Proxy {
	q := transform.Rotation
	w, x, y, z := q.W, q.V[0], q.V[1], q.V[2]
	rotation := mgl64.Mat3{
		1 - 2*(y*y+z*z), 2 * (x*y + w*z), 2 * (x*z - w*y),
		2 * (x*y - w*z), 1 - 2*(x*x+z*z), 2 * (y*z + w*x),
		2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2*(x*x+y*y),
	}
	return Proxy{Position: transform.Position, Rotation: rotation, Inverse: rotation.Transpose(), Shape: shape}
}

// SupportWorld returns the farthest point of the shape in the direction, in world space
func (p *Proxy) SupportWorld(direction mgl64.Vec3) mgl64.Vec3 {
	return p.Position.Add(p.Rotation.Mul3x1(p.Shape.Support(p.Inverse.Mul3x1(direction))))
}

// MinkowskiSupport computes a support point in the Minkowski difference (A - B):
// furthestPoint(A, direction) - furthestPoint(B, -direction)
func MinkowskiSupport(a, b *actor.RigidBody, direction mgl64.Vec3) mgl64.Vec3 {
	return Support(a, b, direction, 0).W
}

// Support computes the support point of (A + margin) - B.
// With a margin, shapes closer than the margin overlap: EPA can compute their distance (margin - depth)
func Support(a, b *actor.RigidBody, direction mgl64.Vec3, margin float64) Vertex {
	proxyA, proxyB := NewProxy(a), NewProxy(b)
	return SupportProxies(&proxyA, &proxyB, direction, margin)
}

func SupportProxies(a, b *Proxy, direction mgl64.Vec3, margin float64) Vertex {
	supportA := a.SupportWorld(direction)
	if margin > 0 {
		if length := direction.Len(); length > 0 {
			supportA = supportA.Add(direction.Mul(margin / length))
		}
	}
	supportB := b.SupportWorld(direction.Mul(-1))
	return Vertex{W: supportA.Sub(supportB), A: supportA, B: supportB}
}

// GJK returns true if both bodies overlap. The simplex is then a tetrahedron containing the origin, for EPA
func GJK(a, b *actor.RigidBody, simplex *Simplex) bool {
	return GJKMargin(a, b, 0, simplex)
}

// GJKMargin returns true if A + margin overlaps B
func GJKMargin(a, b *actor.RigidBody, margin float64, simplex *Simplex) bool {
	proxyA, proxyB := NewProxy(a), NewProxy(b)
	return GJKProxies(&proxyA, &proxyB, margin, simplex)
}

// GJKProxies is GJKMargin for prepared bodies
func GJKProxies(a, b *Proxy, margin float64, simplex *Simplex) bool {
	direction := b.Position.Sub(a.Position)
	if direction.LenSqr() == 0 {
		direction = mgl64.Vec3{1, 0, 0}
	}

	simplex.set(SupportProxies(a, b, direction, margin))
	direction = simplex.Points[0].Mul(-1)

	for i := 0; i < maxIterations; i++ {
		if direction.LenSqr() <= degenerateEpsilon*simplexSize(simplex) {
			// The origin is on the simplex: the shapes are touching
			fillTetrahedron(a, b, margin, simplex)
			return true
		}

		v := SupportProxies(a, b, direction, margin)
		if v.W.Dot(direction) <= 0 {
			return false
		}

		simplex.Points[simplex.Count], simplex.A[simplex.Count], simplex.B[simplex.Count] = v.W, v.A, v.B
		simplex.Count++

		if containsOrigin(simplex, &direction) {
			return true
		}
	}

	return false
}

// containsOrigin reduces the simplex to its closest feature to the origin, and updates the direction.
// Returns true if the tetrahedron contains the origin
func containsOrigin(simplex *Simplex, direction *mgl64.Vec3) bool {
	switch simplex.Count {
	case 2:
		line(simplex, direction)
	case 3:
		triangle(simplex, direction)
	case 4:
		return tetrahedron(simplex, direction)
	}
	return false
}

// line: segment [b, a], a is the most recent point
func line(simplex *Simplex, direction *mgl64.Vec3) {
	a, b := simplex.Vertex(1), simplex.Vertex(0)
	ab := b.W.Sub(a.W)
	ao := a.W.Mul(-1)

	if ab.Dot(ao) > 0 {
		*direction = ab.Cross(ao).Cross(ab)
		return
	}
	simplex.set(a)
	*direction = ao
}

// triangle: [c, b, a], a is the most recent point
func triangle(simplex *Simplex, direction *mgl64.Vec3) {
	a, b, c := simplex.Vertex(2), simplex.Vertex(1), simplex.Vertex(0)
	ab := b.W.Sub(a.W)
	ac := c.W.Sub(a.W)
	ao := a.W.Mul(-1)
	abc := ab.Cross(ac)

	if abc.Cross(ac).Dot(ao) > 0 {
		if ac.Dot(ao) > 0 {
			simplex.set(c, a)
			*direction = ac.Cross(ao).Cross(ac)
			return
		}
		simplex.set(b, a)
		line(simplex, direction)
		return
	}

	if ab.Cross(abc).Dot(ao) > 0 {
		simplex.set(b, a)
		line(simplex, direction)
		return
	}

	if abc.Dot(ao) > 0 {
		simplex.set(c, b, a)
		*direction = abc
	} else {
		simplex.set(b, c, a)
		*direction = abc.Mul(-1)
	}
}

// tetrahedron: [d, c, b, a], a is the most recent point
func tetrahedron(simplex *Simplex, direction *mgl64.Vec3) bool {
	a, b, c, d := simplex.Vertex(3), simplex.Vertex(2), simplex.Vertex(1), simplex.Vertex(0)
	ab := b.W.Sub(a.W)
	ac := c.W.Sub(a.W)
	ad := d.W.Sub(a.W)
	ao := a.W.Mul(-1)

	abc := ab.Cross(ac)
	acd := ac.Cross(ad)
	adb := ad.Cross(ab)

	// The normals must point away from the opposite vertex
	if abc.Dot(ad) > 0 {
		abc = abc.Mul(-1)
	}
	if acd.Dot(ab) > 0 {
		acd = acd.Mul(-1)
	}
	if adb.Dot(ac) > 0 {
		adb = adb.Mul(-1)
	}

	if abc.Dot(ao) > 0 {
		simplex.set(c, b, a)
		triangle(simplex, direction)
		return false
	}
	if acd.Dot(ao) > 0 {
		simplex.set(d, c, a)
		triangle(simplex, direction)
		return false
	}
	if adb.Dot(ao) > 0 {
		simplex.set(b, d, a)
		triangle(simplex, direction)
		return false
	}

	return true
}

// simplexSize returns the largest squared distance of a vertex to the origin
func simplexSize(simplex *Simplex) float64 {
	size := 0.0
	for i := 0; i < simplex.Count; i++ {
		size = max(size, simplex.Points[i].LenSqr())
	}
	return size
}

// fillTetrahedron completes the simplex into a tetrahedron when the shapes are only touching,
// so that EPA can start. Returns false if the Minkowski difference is flat
func fillTetrahedron(a, b *Proxy, margin float64, simplex *Simplex) bool {
	axes := [6]mgl64.Vec3{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}

	var candidates [len(axes)]mgl64.Vec3
	for simplex.Count < 4 {
		added := false
		count := candidateDirections(simplex, &axes, &candidates)
		for _, axis := range candidates[:count] {
			v := SupportProxies(a, b, axis, margin)
			if isNewVertex(simplex, v.W) {
				simplex.Points[simplex.Count], simplex.A[simplex.Count], simplex.B[simplex.Count] = v.W, v.A, v.B
				simplex.Count++
				added = true
				break
			}
		}
		if !added {
			return false
		}
	}
	return true
}

// candidateDirections to add a dimension to the simplex, written in out (their count is returned, nothing is
// allocated): the axes for a point, perpendicular directions for a segment, both normals for a triangle
func candidateDirections(simplex *Simplex, axes, out *[6]mgl64.Vec3) int {
	switch simplex.Count {
	case 1:
		*out = *axes
		return len(axes)
	case 2:
		edge := simplex.Points[1].Sub(simplex.Points[0])
		count := 0
		for _, axis := range axes {
			if d := edge.Cross(axis); d.LenSqr() > 0 {
				out[count] = d
				count++
			}
		}
		return count
	default:
		n := simplex.Points[1].Sub(simplex.Points[0]).Cross(simplex.Points[2].Sub(simplex.Points[0]))
		out[0], out[1] = n, n.Mul(-1)
		return 2
	}
}

// isNewVertex returns true if w is not on the point/line/plane of the simplex
func isNewVertex(simplex *Simplex, w mgl64.Vec3) bool {
	p0 := simplex.Points[0]
	scale := max(simplexSize(simplex), w.LenSqr())
	if scale == 0 {
		return false
	}
	switch simplex.Count {
	case 1:
		return w.Sub(p0).LenSqr() > hullEpsilon*scale
	case 2:
		edge := simplex.Points[1].Sub(p0)
		return edge.Cross(w.Sub(p0)).LenSqr() > hullEpsilon*scale*edge.LenSqr()
	default:
		n := simplex.Points[1].Sub(p0).Cross(simplex.Points[2].Sub(p0))
		h := n.Dot(w.Sub(p0))
		return h*h > hullEpsilon*scale*n.LenSqr()
	}
}
