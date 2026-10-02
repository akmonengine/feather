package scenes

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== FAR SCENES ==========
// A scene far from the origin (tens of km, as in Solver2D) keeps the criteria of the same scene at the origin. Both are
// run: "far deviation" is the largest difference between the positions of a body in both (relative to their origin).
// It is followed by the bench, not checked: a chaotic scene (a pile falling) amplifies the rounding far from the origin

// farScene builds its bodies around origin and returns them, with the result of the scene
type farScene func(origin mgl64.Vec3, play Player) ([]*actor.RigidBody, Result)

// far runs the scene at the origin and far from it: the result far, and the deviation between both
func far(origin mgl64.Vec3, scene farScene) func(size Size, play Player) Result {
	return func(size Size, play Player) Result {
		near, _ := scene(mgl64.Vec3{}, play)
		farBodies, result := scene(origin, play)
		deviation := 0.0
		for i := range near {
			deviation = math.Max(deviation, farBodies[i].Transform.Position.Sub(origin).Sub(near[i].Transform.Position).Len())
		}
		result["far deviation"] = mm(deviation)
		return result
	}
}

// farPyramid: a pyramid of 10 layers of cubes of 1 m, 25 cm apart: they fall on each other, then stand. The cubes land
// with their 4 points at the separation 0: which ones are solved as springs is a draw, at the origin and far from it.
// The far deviation is the distance between both draws: a millimetre, centimetres for a few. Its variants put the cubes
// 25 to 26 cm apart
var farPyramid = Scene{
	Name:  "far pyramid",
	Run:   farCubes(0.25),
	Draws: []string{"far deviation"},
	Vary:  across(0.25, 0.26, farCubes),
	Check: func(r Result) error {
		return atMost(r, "worst drift", layersSlop(10)*1000, "a contact per layer")
	},
}

// farCubes: the scene far pyramid, the cubes gap apart (m)
func farCubes(gap float64) func(size Size, play Player) Result {
	return far(mgl64.Vec3{100000, -80000, 60000}, func(origin mgl64.Vec3, play Player) ([]*actor.RigidBody, Result) {
		const count = 10
		w := newWorld()
		staticBox(w, origin.Add(mgl64.Vec3{0, -1, 0}), mgl64.QuatIdent(), mgl64.Vec3{100, 1, 100}, defaultMaterial)
		cubes := squarePyramid(w, origin.Add(mgl64.Vec3{0, 0.5, 0}), count, 0.5, gap, defaultMaterial)
		play(w, 2, nil)
		start := positions(cubes)
		play(w, 3, nil)
		return cubes, Result{"worst drift": mm(worstDrift(cubes, start))}
	})
}

// farStack: a plank on a small roller and a small box, 2 cubes on the plank. The roller is a capsule lying across the
// plank, the circle of Solver2D in 3D (a sphere would hold the plank on a point)
var farStack = Scene{
	Name:     "far stack",
	capsules: true,
	Run: far(mgl64.Vec3{40000, -25000, 30000}, func(origin mgl64.Vec3, play Player) ([]*actor.RigidBody, Result) {
		w := newWorld()
		staticBox(w, origin.Add(mgl64.Vec3{0, -1, 0}), mgl64.QuatIdent(), mgl64.Vec3{10, 1, 10}, defaultMaterial)
		bodies := []*actor.RigidBody{
			addBody(w, origin.Add(mgl64.Vec3{1.875, 0.1, 0}), mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{1, 0, 0}), capsule(0.4, 0.1), actor.BodyTypeDynamic, defaultMaterial),
			box(w, origin.Add(mgl64.Vec3{-1.875, 0.15, 0}), mgl64.Vec3{0.1, 0.125, 0.1}, defaultMaterial),
			box(w, origin.Add(mgl64.Vec3{0, 0.325, 0}), mgl64.Vec3{2, 0.05, 0.5}, defaultMaterial),
			box(w, origin.Add(mgl64.Vec3{-0.5, 0.9, 0}), mgl64.Vec3{0.25, 0.25, 0.25}, defaultMaterial),
			box(w, origin.Add(mgl64.Vec3{-0.55, 1.7, 0}), mgl64.Vec3{0.5, 0.5, 0.5}, defaultMaterial),
		}
		play(w, 2, nil)
		start := positions(bodies)
		play(w, 3, nil)
		return bodies, Result{"worst drift": mm(worstDrift(bodies, start))}
	}),
	Check: func(r Result) error {
		return atMost(r, "worst drift", layersSlop(3)*1000, "a contact per layer")
	},
}

// farRecovery: the overlap recovery, far from the origin
var farRecovery = Scene{
	Name: "far recovery",
	Run: far(mgl64.Vec3{80000, -70000, 50000}, func(origin mgl64.Vec3, play Player) ([]*actor.RigidBody, Result) {
		w := newWorld()
		ground(w, origin, defaultMaterial)
		cubes := squarePyramid(w, origin, 4, 0.5, -0.25, defaultMaterial)
		fastest := 0.0
		play(w, 3, func() { fastest = math.Max(fastest, maxSpeed(cubes)) })
		return cubes, Result{"max speed": {fastest, "m/s"}}
	}),
	Check: func(r Result) error {
		return atMost(r, "max speed", contactSpeed*4, "ContactSpeed per contact in series")
	},
}

// farChain: a chain of 40 capsules of 20 cm, starting horizontal, far from the origin
var farChain = Scene{
	Name:     "far chain",
	capsules: true,
	joints:   true,
	Run: far(mgl64.Vec3{40000, -35000, 30000}, func(origin mgl64.Vec3, play Player) ([]*actor.RigidBody, Result) {
		const count, hx, radius = 40, 0.1, 0.025
		w := newWorld()
		m := material{friction: 0.6, density: 20}
		height := float64(count) * hx
		previous := addBody(w, origin.Add(mgl64.Vec3{-0.05, height, 0}), mgl64.QuatIdent(), &actor.Sphere{Radius: 0.02}, actor.BodyTypeStatic, m)
		lying := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
		var bodies []*actor.RigidBody
		var links []link
		for i := 0; i < count; i++ {
			anchor := origin.Add(mgl64.Vec3{2 * float64(i) * hx, height, 0})
			body := damped(addBody(w, origin.Add(mgl64.Vec3{(1 + 2*float64(i)) * hx, height, 0}), lying, capsule(hx, radius), actor.BodyTypeDynamic, m))
			hinge(w, previous, body, anchor, mgl64.Vec3{0, 0, 1})
			links = append(links, newLink(previous, body, anchor))
			bodies = append(bodies, body)
			previous = body
		}
		gap := 0.0
		play(w, 5, func() { gap = math.Max(gap, worstGap(links)) })
		return bodies, Result{"worst gap": mm(gap), "finite": finiteMetric(bodies)}
	}),
	Check: func(r Result) error { return holds(r, 0.1) },
}
