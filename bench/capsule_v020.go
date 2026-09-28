//go:build v020

package main

import (
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

func exactCapsuleBox(a, b *actor.RigidBody) (float64, mgl64.Vec3, bool) {
	return 0, mgl64.Vec3{}, false
}
