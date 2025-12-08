package actor

import "github.com/go-gl/mathgl/mgl64"

// Transform represents a position in 3D space
type Transform struct {
	Position        mgl64.Vec3
	Rotation        mgl64.Quat
	InverseRotation mgl64.Quat
}

// NewTransform creates an identity transform
func NewTransform() Transform {
	return Transform{
		Position: mgl64.Vec3{0, 0, 0},
		Rotation: mgl64.QuatIdent(),
	}
}
