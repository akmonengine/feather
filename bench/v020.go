//go:build v020

package main

import (
	"fmt"
	"math"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

var capsuleMaker shapeMaker

const version = "v0.2.0 (XPBD)"

func tr(p mgl64.Vec3, q mgl64.Quat) actor.Transform {
	return actor.Transform{Position: p, Rotation: q, InverseRotation: q.Inverse()}
}

func narrow(a, b *actor.RigidBody) (bool, mgl64.Vec3, float64, int) {
	ch := make(chan feather.Pair, 1)
	ch <- feather.Pair{BodyA: a, BodyB: b}
	close(ch)
	cs := feather.NarrowPhase(ch, 1)
	if len(cs) == 0 {
		return false, mgl64.Vec3{}, 0, 0
	}
	c := cs[0]
	n := c.Normal
	if c.BodyA != a {
		n = n.Mul(-1)
	}
	depth := 0.0
	for _, p := range c.Points {
		depth = math.Max(depth, p.Penetration)
	}
	return true, n, depth, len(c.Points)
}

// regressions: the reference is measured on the working tree only
func regressions(update bool) bool {
	fmt.Println("the regressions run on the working tree, not on v0.2.0")
	return false
}
