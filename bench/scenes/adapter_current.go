//go:build !v020

package scenes

import (
	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Version of Feather the scenes run on
const Version = "current"

// slop: the length tolerance of the collision detection (m)
const slop = feather.LinearSlop

// the features of this version
const (
	hasCapsules = true
	hasJoints   = true
)

func pose(position mgl64.Vec3, rotation mgl64.Quat) actor.Transform {
	return actor.Transform{Position: position, Rotation: rotation}
}

func capsule(halfHeight, radius float64) actor.ShapeInterface {
	return &actor.Capsule{HalfHeight: halfHeight, Radius: radius}
}

// hinge links 2 bodies around the axis through the anchor (world space)
func hinge(w *feather.World, a, b *actor.RigidBody, anchor, axis mgl64.Vec3) {
	w.AddJoint(feather.NewHingeJoint(a, b, anchor, axis))
}

// ball links 2 bodies at the anchor (world space)
func ball(w *feather.World, a, b *actor.RigidBody, anchor mgl64.Vec3) {
	w.AddJoint(feather.NewBallJoint(a, b, anchor, mgl64.Vec3{1, 0, 0}))
}

// moved: the transform of the body was set by hand
func moved(b *actor.RigidBody) {
	b.UpdateAABB()
}

// the setting of Feather for the games: 60 Hz (the default rate of Box3D, the reference, which runs its default 4
// substeps) with 8 substeps
const (
	Dt       = 1.0 / 60
	substeps = 8
)

func newWorld() *feather.World {
	return &feather.World{
		Gravity:  mgl64.Vec3{0, -gravity, 0},
		Substeps: substeps,
		Workers:  1,
		Events:   feather.NewEvents(),
	}
}
