//go:build v020

package scenes

import (
	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Version of Feather the scenes run on
const Version = "v0.2.0"

// slop: the length tolerance of the collision detection of the current version (m), for the same measures
const slop = 0.005

// the features of this version: no capsule, no joint
const (
	hasCapsules = false
	hasJoints   = false
)

func pose(position mgl64.Vec3, rotation mgl64.Quat) actor.Transform {
	return actor.Transform{Position: position, Rotation: rotation, InverseRotation: rotation.Inverse()}
}

func capsule(halfHeight, radius float64) actor.ShapeInterface {
	panic("scenes: no capsule in v0.2.0")
}

func hinge(w *feather.World, a, b *actor.RigidBody, anchor, axis mgl64.Vec3) {
	panic("scenes: no joint in v0.2.0")
}

func ball(w *feather.World, a, b *actor.RigidBody, anchor mgl64.Vec3) {
	panic("scenes: no joint in v0.2.0")
}

// moved: v0.2.0 computes the AABB at each step
func moved(b *actor.RigidBody) {}

func newWorld() *feather.World {
	return &feather.World{
		Gravity:     mgl64.Vec3{0, -gravity, 0},
		Substeps:    substeps,
		SpatialGrid: feather.NewSpatialGrid(2, 4096),
		Workers:     1,
		Events:      feather.NewEvents(),
	}
}
