// Package epa implements the Expanding Polytope Algorithm, to compute the penetration depth.
//
// EPA runs after GJK when there is a collision. It returns the penetration depth, the contact normal,
// and the witness points of the contact on both shapes.
// The polytope starts from the tetrahedron of GJK and grows towards the Minkowski difference,
// until the closest face to the origin is on its surface.
//
// For detailed algorithm explanation, see:
// ALGORITHMS.md - "EPA Algorithm" section
//
// References:
//   - Van den Bergen: "Proximity Queries and Penetration Depth Computation on 3D Game Objects" (2001)
//   - Ericson: "Real-Time Collision Detection" (2004)
package epa

import (
	"errors"
	"math"
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

const (
	// EPAMaxIterations: flat faces converge in a few iterations, rounded shapes need more
	EPAMaxIterations = 128

	// EPAConvergenceTolerance (m): EPA stops when the new support point improves the distance by less than this value.
	// It is the error on the penetration depth
	EPAConvergenceTolerance = 1e-7
)

var ErrNoConvergence = errors.New("epa: no convergence")

// Result is the penetration of A (+ margin) into B
type Result struct {
	// Normal from A to B: moving B by Depth along Normal separates the shapes
	Normal mgl64.Vec3
	Depth  float64
	// Witness points: the deepest points of the contact, on A (+ margin) and on B.
	// WitnessA - WitnessB = Depth * Normal
	WitnessA mgl64.Vec3
	WitnessB mgl64.Vec3
}

type face struct {
	v        [3]int // counter-clockwise, seen from outside
	normal   mgl64.Vec3
	distance float64
}

type edge struct{ a, b int }

type polytope struct {
	vertices []gjk.Vertex
	faces    []face
	horizon  []edge
}

var polytopePool = sync.Pool{New: func() any { return &polytope{} }}

// EPA computes the penetration of A (+ margin) into B, from the tetrahedron of GJK
func EPA(a, b *actor.RigidBody, simplex *gjk.Simplex, margin float64) (Result, error) {
	proxyA, proxyB := gjk.NewProxy(a), gjk.NewProxy(b)
	return EPAProxies(&proxyA, &proxyB, simplex, margin)
}

// EPAProxies is EPA for prepared bodies
func EPAProxies(a, b *gjk.Proxy, simplex *gjk.Simplex, margin float64) (Result, error) {
	if simplex.Count != 4 {
		return Result{}, ErrNoConvergence
	}

	p := polytopePool.Get().(*polytope)
	defer polytopePool.Put(p)
	p.vertices = p.vertices[:0]
	p.faces = p.faces[:0]

	for i := 0; i < 4; i++ {
		p.vertices = append(p.vertices, simplex.Vertex(i))
	}
	for _, f := range [4][3]int{{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}} {
		if !p.addFace(f[0], f[1], f[2]) {
			return Result{}, ErrNoConvergence
		}
	}
	// The normals must point outwards: if the first face points to the 4th vertex, flip all faces
	if p.faces[0].normal.Dot(p.vertices[3].W.Sub(p.vertices[0].W)) > 0 {
		for i := range p.faces {
			f := &p.faces[i]
			f.v[1], f.v[2] = f.v[2], f.v[1]
			f.normal = f.normal.Mul(-1)
			f.distance = -f.distance
		}
	}

	for iteration := 0; iteration < EPAMaxIterations; iteration++ {
		closest := p.closestFace()
		f := p.faces[closest]

		v := gjk.SupportProxies(a, b, f.normal, margin)
		if v.W.Dot(f.normal)-f.distance < EPAConvergenceTolerance {
			return p.result(f), nil
		}

		if !p.expand(v) {
			return p.result(f), nil
		}
	}

	return p.result(p.faces[p.closestFace()]), nil
}

// addFace returns false if the triangle is degenerate
func (p *polytope) addFace(i, j, k int) bool {
	a, b, c := p.vertices[i].W, p.vertices[j].W, p.vertices[k].W
	n := b.Sub(a).Cross(c.Sub(a))
	length := n.Len()
	if length < 1e-14 {
		return false
	}
	n = n.Mul(1 / length)
	p.faces = append(p.faces, face{v: [3]int{i, j, k}, normal: n, distance: n.Dot(a)})
	return true
}

func (p *polytope) closestFace() int {
	best := 0
	for i := 1; i < len(p.faces); i++ {
		if p.faces[i].distance < p.faces[best].distance {
			best = i
		}
	}
	return best
}

// expand adds the vertex: the faces it can see are removed, and the hole is closed with new faces
// from the horizon to the vertex
func (p *polytope) expand(v gjk.Vertex) bool {
	p.vertices = append(p.vertices, v)
	index := len(p.vertices) - 1
	p.horizon = p.horizon[:0]

	kept := p.faces[:0]
	for _, f := range p.faces {
		if f.normal.Dot(v.W.Sub(p.vertices[f.v[0]].W)) > 0 {
			for e := 0; e < 3; e++ {
				p.toggleEdge(edge{f.v[e], f.v[(e+1)%3]})
			}
			continue
		}
		kept = append(kept, f)
	}
	p.faces = kept

	if len(p.horizon) == 0 {
		return false
	}
	for _, e := range p.horizon {
		if !p.addFace(e.a, e.b, index) {
			return false
		}
	}
	return true
}

// toggleEdge: an edge shared by 2 removed faces is inside and disappears, the others are the horizon
func (p *polytope) toggleEdge(e edge) {
	for i, h := range p.horizon {
		if h.a == e.b && h.b == e.a {
			p.horizon = append(p.horizon[:i], p.horizon[i+1:]...)
			return
		}
	}
	p.horizon = append(p.horizon, e)
}

// result projects the origin on the face, the barycentric coordinates give the witness points
func (p *polytope) result(f face) Result {
	a, b, c := p.vertices[f.v[0]], p.vertices[f.v[1]], p.vertices[f.v[2]]
	point := f.normal.Mul(f.distance)
	u, v, w := barycentric(point, a.W, b.W, c.W)

	return Result{
		Normal:   f.normal,
		Depth:    math.Max(f.distance, 0),
		WitnessA: a.A.Mul(u).Add(b.A.Mul(v)).Add(c.A.Mul(w)),
		WitnessB: a.B.Mul(u).Add(b.B.Mul(v)).Add(c.B.Mul(w)),
	}
}

// barycentric coordinates of p in the triangle, clamped to it (Ericson 3.4)
func barycentric(p, a, b, c mgl64.Vec3) (float64, float64, float64) {
	v0, v1, v2 := b.Sub(a), c.Sub(a), p.Sub(a)
	d00, d01, d11 := v0.Dot(v0), v0.Dot(v1), v1.Dot(v1)
	d20, d21 := v2.Dot(v0), v2.Dot(v1)
	denominator := d00*d11 - d01*d01
	if denominator == 0 {
		return 1, 0, 0
	}
	v := (d11*d20 - d01*d21) / denominator
	w := (d00*d21 - d01*d20) / denominator
	v, w = math.Max(v, 0), math.Max(w, 0)
	if sum := v + w; sum > 1 {
		v, w = v/sum, w/sum
	}
	return 1 - v - w, v, w
}
