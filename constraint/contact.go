package constraint

import (
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// MaxContactPoints: 4 points are enough to keep a box flat (Erin Catto, GDC 2007)
const MaxContactPoints = 4

type ContactPoint struct {
	// Position is halfway between both surfaces
	Position mgl64.Vec3
	// Separation < 0 when the bodies overlap, > 0 for a speculative contact (not touching yet)
	Separation float64

	// NormalImpulse applied by the solver during the last step (N·s), to warm start the next step
	NormalImpulse float64

	// LocalAnchorA is the point on the surface of A, in the local space of A, LocalAnchorB the point on the
	// surface of B, in the local space of B. They find the same point in the next step (warm starting),
	// and move the contact with the bodies (pair cache)
	LocalAnchorA mgl64.Vec3
	LocalAnchorB mgl64.Vec3
}

// Manifold is the contact between 2 bodies. Normal points from A to B
type Manifold struct {
	BodyA  *actor.RigidBody
	BodyB  *actor.RigidBody
	Normal mgl64.Vec3
	Points [MaxContactPoints]ContactPoint
	Count  int

	// Impulses of the friction applied by the solver during the last step, to warm start the next step: along the
	// tangents at the friction center of the points (N·s), and around the normal (N·m·s)
	FrictionImpulse mgl64.Vec3
	TwistImpulse    float64

	// RollingImpulse applied by the solver during the last step (N·m·s), to warm start the next step
	RollingImpulse mgl64.Vec3

	// When the contact points were computed: the normal in the local space of A,
	// and the position & rotation of B in the local space of A (pair cache)
	LocalNormal      mgl64.Vec3
	RelativePosition mgl64.Vec3
	RelativeRotation mgl64.Quat
}

func (m *Manifold) Reset(a, b *actor.RigidBody) {
	*m = Manifold{BodyA: a, BodyB: b}
}

// Add a point, ignored after MaxContactPoints
func (m *Manifold) Add(position mgl64.Vec3, separation float64) {
	if m.Count == MaxContactPoints {
		return
	}
	m.Points[m.Count] = ContactPoint{Position: position, Separation: separation}
	m.Count++
}

func (m *Manifold) Flip() {
	m.BodyA, m.BodyB = m.BodyB, m.BodyA
	m.Normal = m.Normal.Mul(-1)
}

func (m *Manifold) MinSeparation() float64 {
	min := posInf
	for i := 0; i < m.Count; i++ {
		if m.Points[i].Separation < min {
			min = m.Points[i].Separation
		}
	}
	return min
}
