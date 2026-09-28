package actor

import "github.com/go-gl/mathgl/mgl64"

// Transform represents a position in 3D space.
// The inverse of the rotation is its conjugate: no need to store it
type Transform struct {
	Position mgl64.Vec3
	Rotation mgl64.Quat
}

// NewTransform creates an identity transform
func NewTransform() Transform {
	return Transform{
		Position: mgl64.Vec3{0, 0, 0},
		Rotation: mgl64.QuatIdent(),
	}
}

// ToWorld maps a point from the local space of the transform to world space.
func (t Transform) ToWorld(local mgl64.Vec3) mgl64.Vec3 {
	return t.Position.Add(t.Rotation.Rotate(local))
}

// ToLocal maps a point from world space to the local space of the transform.
func (t Transform) ToLocal(world mgl64.Vec3) mgl64.Vec3 {
	return t.Rotation.Conjugate().Rotate(world.Sub(t.Position))
}
