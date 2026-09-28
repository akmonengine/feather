package scenes

import (
	"fmt"
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== JOINT SCENES ==========
// The tests check that a joint holds (its gap stays under half a link: the chain is not broken), the bench follows the
// exact gaps

// linkDamping of the bodies of the chains (as in Solver2D)
const linkDamping = 0.1

func damped(b *actor.RigidBody) *actor.RigidBody {
	b.Material.LinearDamping, b.Material.AngularDamping = linkDamping, linkDamping
	return b
}

// holds: the joints are not broken, and no body is thrown
func holds(r Result, halfLink float64) error {
	if r["finite"].Value != 1 {
		return fmt.Errorf("a body is not finite")
	}
	return atMost(r, "worst gap", halfLink*1000, "the chain is not broken")
}

func finiteMetric(bodies []*actor.RigidBody) Metric {
	if finiteBodies(bodies) {
		return Metric{1, ""}
	}
	return Metric{0, ""}
}

// bridge: planks of 1 m linked by hinges, both ends on static bodies
var bridge = Scene{
	Name:   "bridge",
	joints: true,
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 40, Full: 160}[size]
		w := newWorld()
		x0 := -0.5 * float64(count)
		m := material{friction: 0.6, density: 20}
		left := staticBox(w, mgl64.Vec3{x0 - 0.5, 20, 0}, mgl64.QuatIdent(), mgl64.Vec3{0.5, 0.125, 0.5}, m)
		right := staticBox(w, mgl64.Vec3{-x0 + 0.5, 20, 0}, mgl64.QuatIdent(), mgl64.Vec3{0.5, 0.125, 0.5}, m)
		previous := left
		var planks []*actor.RigidBody
		var links []link
		axis := mgl64.Vec3{0, 0, 1}
		for i := 0; i <= count; i++ {
			anchor := mgl64.Vec3{x0 + float64(i), 20, 0}
			next := right
			if i < count {
				next = damped(box(w, mgl64.Vec3{x0 + 0.5 + float64(i), 20, 0}, mgl64.Vec3{0.5, 0.125, 0.5}, m))
				planks = append(planks, next)
			}
			hinge(w, previous, next, anchor, axis)
			links = append(links, newLink(previous, next, anchor))
			previous = next
		}
		gap := 0.0
		play(w, 5, func() { gap = math.Max(gap, worstGap(links)) })
		sag := 20 - planks[count/2].Transform.Position.Y()
		return Result{"worst gap": mm(gap), "sag": {sag, "m"}, "finite": finiteMetric(planks)}
	},
	Check: func(r Result) error { return holds(r, 0.5) },
}

// ballAndChain: a chain of capsules of 1 m, free at its end, carrying a ball of 8 m. It starts horizontal and swings.
// In 3D the ball would be 37000 times heavier than a link: its density keeps the ratio of Solver2D, 672
var ballAndChain = Scene{
	Name:     "ball and chain",
	capsules: true,
	joints:   true,
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 20, Full: 40}[size]
		const hx, radius, ballRadius = 0.5, 0.125, 8.0
		w := newWorld()
		m := material{friction: 0.6, density: 20}
		height := float64(count) * hx
		anchorBody := addBody(w, mgl64.Vec3{-0.5, height, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeStatic, m)
		lying := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
		previous := anchorBody
		var bodies []*actor.RigidBody
		var links []link
		axis := mgl64.Vec3{0, 0, 1}
		for i := 0; i < count; i++ {
			anchor := mgl64.Vec3{2 * float64(i) * hx, height, 0}
			body := damped(addBody(w, mgl64.Vec3{(1 + 2*float64(i)) * hx, height, 0}, lying, capsule(hx, radius), actor.BodyTypeDynamic, m))
			hinge(w, previous, body, anchor, axis)
			links = append(links, newLink(previous, body, anchor))
			bodies = append(bodies, body)
			previous = body
		}
		anchor := mgl64.Vec3{2 * float64(count) * hx, height, 0}
		const ratio = 672
		linkVolume := math.Pi*radius*radius*2*hx + 4.0/3*math.Pi*radius*radius*radius
		ballVolume := 4.0 / 3 * math.Pi * ballRadius * ballRadius * ballRadius
		ball := damped(sphere(w, anchor.Add(mgl64.Vec3{ballRadius, 0, 0}), ballRadius, material{friction: 0.6, density: ratio * m.density * linkVolume / ballVolume}))
		hinge(w, previous, ball, anchor, axis)
		links = append(links, newLink(previous, ball, anchor))
		bodies = append(bodies, ball)
		gap := 0.0
		play(w, 5, func() { gap = math.Max(gap, worstGap(links)) })
		return Result{"worst gap": mm(gap), "finite": finiteMetric(bodies)}
	},
	Check: func(r Result) error { return holds(r, 0.5) },
}

// jointGrid: a net of spheres linked by ball joints to their 4 neighbours, held by 7 × 7 nodes in its middle, falling
// under twice the gravity (as in Solver2D)
var jointGrid = Scene{
	Name:   "joint grid",
	joints: true,
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 20, Full: 60}[size]
		w := newWorld()
		w.Gravity = w.Gravity.Mul(2)
		m := material{friction: 0.6, density: 1}
		nodes := make([]*actor.RigidBody, count*count)
		var links []link
		middle := count / 2
		for i := 0; i < count; i++ {
			for k := 0; k < count; k++ {
				bodyType := actor.BodyTypeDynamic
				if i >= middle-3 && i <= middle+3 && k >= middle-3 && k <= middle+3 {
					bodyType = actor.BodyTypeStatic
				}
				position := mgl64.Vec3{float64(k - middle), 0, float64(i - middle)}
				node := addBody(w, position, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.4}, bodyType, m)
				nodes[i*count+k] = node
				for _, neighbour := range []int{(i-1)*count + k, i*count + k - 1} {
					if (neighbour == (i-1)*count+k && i == 0) || (neighbour == i*count+k-1 && k == 0) {
						continue
					}
					other := nodes[neighbour]
					if other.BodyType == actor.BodyTypeStatic && bodyType == actor.BodyTypeStatic {
						continue
					}
					anchor := other.Transform.Position.Add(position).Mul(0.5)
					ball(w, other, node, anchor)
					links = append(links, newLink(other, node, anchor))
				}
			}
		}
		gap, fastest := 0.0, 0.0
		play(w, 3, func() {
			gap = math.Max(gap, worstGap(links))
			fastest = math.Max(fastest, maxSpeed(nodes))
		})
		return Result{"worst gap": mm(gap), "max speed": {fastest, "m/s"}, "finite": finiteMetric(nodes)}
	},
	Check: func(r Result) error { return holds(r, 0.5) },
}

// stretchedChain: a chain of 40 links of 1 m hanging from a static body, created stretched twice: it recovers the state
// of the same chain created at rest. A hanging chain sags at rest: its joints are springs of 60 Hz (as in Box2D v3)
var stretchedChain = Scene{
	Name:   "stretched chain",
	joints: true,
	Run: func(size Size, play Player) Result {
		chain := func(stretch float64) ([]*actor.RigidBody, []link, float64) {
			const count, length = 40, 1.0
			w := newWorld()
			top := float64(count) * length
			m := material{friction: 0.6, density: 1}
			previous := addBody(w, mgl64.Vec3{0, top, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.1}, actor.BodyTypeStatic, m)
			var bodies []*actor.RigidBody
			var links []link
			// the joints are made at the rest length, then the links are moved
			for i := 0; i < count; i++ {
				anchor := mgl64.Vec3{0, top - float64(i)*length, 0}
				body := sphere(w, anchor.Sub(mgl64.Vec3{0, 0.5 * length, 0}), 0.2, m)
				ball(w, previous, body, anchor)
				links = append(links, newLink(previous, body, anchor))
				bodies = append(bodies, body)
				previous = body
			}
			for i, body := range bodies {
				body.Transform.Position = mgl64.Vec3{0, top - (float64(i)+0.5)*length*stretch, 0}
				moved(body)
			}
			fastest := 0.0
			play(w, 5, func() { fastest = math.Max(fastest, maxSpeed(bodies)) })
			return bodies, links, fastest
		}
		_, restLinks, _ := chain(1)
		bodies, links, fastest := chain(2)
		return Result{"final gap": mm(worstGap(links)), "rest gap": mm(worstGap(restLinks)), "max speed": {fastest, "m/s"},
			"finite": finiteMetric(bodies)}
	},
	Check: func(r Result) error {
		if r["finite"].Value != 1 {
			return fmt.Errorf("a body is not finite")
		}
		// the joints pull the links back at 60 Hz (as in Box2D v3): the speed is followed by the bench, not bounded
		return atMost(r, "final gap", r["rest gap"].Value+slop*1000, "the chain recovered its state at rest")
	},
}
