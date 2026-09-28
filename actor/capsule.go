package actor

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

const (
	// capsuleSideFeatureTolerance: if the direction is perpendicular to the axis (~1.1°),
	// the contact feature is the whole side line, otherwise a single point on a cap
	capsuleSideFeatureTolerance = 0.02

	capsuleDirectionEpsilon = 1e-12
)

// Capsule is a cylinder with 2 hemispheres.
// The segment is along the local Y axis, from -HalfHeight to +HalfHeight: the total height is 2*(HalfHeight+Radius)
type Capsule struct {
	HalfHeight float64 // Half length of the inner segment (cylinder part)
	Radius     float64 // Radius of the cylinder and of both caps
}

// Segment returns both ends of the segment in world space (bottom, then top)
func (c *Capsule) Segment(transform Transform) (mgl64.Vec3, mgl64.Vec3) {
	axis := transform.Rotation.Rotate(mgl64.Vec3{0, c.HalfHeight, 0})
	return transform.Position.Sub(axis), transform.Position.Add(axis)
}

func (c *Capsule) ComputeAABB(transform Transform) AABB {
	axis := transform.Rotation.Rotate(mgl64.Vec3{0, c.HalfHeight, 0})
	extent := mgl64.Vec3{
		math.Abs(axis.X()) + c.Radius,
		math.Abs(axis.Y()) + c.Radius,
		math.Abs(axis.Z()) + c.Radius,
	}

	return AABB{
		Min: transform.Position.Sub(extent),
		Max: transform.Position.Add(extent),
	}
}

// ComputeMass: cylinder + 1 full sphere
func (c *Capsule) ComputeMass(density float64) float64 {
	return density * c.volume()
}

func (c *Capsule) volume() float64 {
	r := c.Radius
	cylinder := math.Pi * r * r * 2 * c.HalfHeight
	sphere := 4.0 / 3.0 * math.Pi * r * r * r
	return cylinder + sphere
}

// ComputeInertia: the mass is split between the cylinder (h = 2*HalfHeight) and both hemispheres, by volume
//
//	I_yy = mCylinder r²/2 + mCaps 2r²/5
//	I_xx = I_zz = mCylinder (r²/4 + h²/12) + mCaps (2r²/5 + h²/4 + 3hr/8)
//
// Parallel axis theorem for the caps: the centroid of a hemisphere is at 3r/8 from its flat face
func (c *Capsule) ComputeInertia(mass float64) mgl64.Mat3 {
	r := c.Radius
	h := 2 * c.HalfHeight

	volume := c.volume()
	if volume <= 0 {
		return mgl64.Mat3{}
	}
	cylinderMass := mass * (math.Pi * r * r * h) / volume
	capsMass := mass - cylinderMass

	axial := cylinderMass*r*r/2 + capsMass*2*r*r/5
	transverse := cylinderMass*(r*r/4+h*h/12) + capsMass*(2*r*r/5+h*h/4+3*h*r/8)

	return mgl64.Mat3{
		transverse, 0, 0,
		0, axial, 0,
		0, 0, transverse,
	}
}

func (c *Capsule) Support(direction mgl64.Vec3) mgl64.Vec3 {
	end := mgl64.Vec3{0, c.HalfHeight, 0}
	if direction.Y() < 0 {
		end[1] = -c.HalfHeight
	}

	length := direction.Len()
	if length < capsuleDirectionEpsilon {
		return end.Add(mgl64.Vec3{0, c.Radius, 0})
	}

	return end.Add(direction.Mul(c.Radius / length))
}

// GetContactFeature returns the side line (2 points) if the direction is perpendicular to the axis,
// otherwise the support point on a cap
func (c *Capsule) GetContactFeature(direction mgl64.Vec3, output *[8]mgl64.Vec3, count *int) {
	length := direction.Len()
	if c.HalfHeight > 0 && length >= capsuleDirectionEpsilon && math.Abs(direction.Y())/length < capsuleSideFeatureTolerance {
		radial := mgl64.Vec3{direction.X(), 0, direction.Z()}
		radial = radial.Mul(c.Radius / radial.Len())

		output[0] = radial.Add(mgl64.Vec3{0, c.HalfHeight, 0})
		output[1] = radial.Sub(mgl64.Vec3{0, c.HalfHeight, 0})
		*count = 2
		return
	}

	output[0] = c.Support(direction)
	*count = 1
}

// CollideWithPlane tests both caps: a lying capsule gets 2 contacts, so it does not roll
func (c *Capsule) CollideWithPlane(planeNormal mgl64.Vec3, planeDistance float64, myTransform Transform, margin float64, contacts PlaneContact) PlaneContact {
	bottom, top := c.Segment(myTransform)

	for _, end := range [2]mgl64.Vec3{bottom, top} {
		separation := end.Dot(planeNormal) + planeDistance - c.Radius
		if separation > margin {
			continue
		}

		contacts = append(contacts, ContactPoint{
			Position:   end.Sub(planeNormal.Mul(c.Radius + separation/2)),
			Separation: separation,
		})
	}

	return contacts
}
