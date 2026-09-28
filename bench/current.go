//go:build !v020

package main

import (
	"math/rand"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

const version = "current (soft step)"

func tr(p mgl64.Vec3, q mgl64.Quat) actor.Transform {
	return actor.Transform{Position: p, Rotation: q}
}

var capsuleMaker shapeMaker = func(r *rand.Rand) actor.ShapeInterface {
	return &actor.Capsule{HalfHeight: 0.05 + 0.5*r.Float64(), Radius: 0.05 + 0.3*r.Float64()}
}

// narrow measures EPA itself (depth and normal), not the clipped manifold.
func narrow(a, b *actor.RigidBody) (bool, mgl64.Vec3, float64, int) {
	var m constraint.Manifold
	if !feather.Collide(a, b, 0, &m) {
		return false, mgl64.Vec3{}, 0, 0
	}
	s := &gjk.Simplex{}
	if gjk.GJK(a, b, s) {
		if r, err := epa.EPA(a, b, s, 0); err == nil {
			return true, r.Normal, r.Depth, m.Count
		}
	}
	return true, m.Normal, -m.MinSeparation(), m.Count
}
