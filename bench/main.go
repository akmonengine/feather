// Command bench measures Feather on physical scenarios with a known answer, on the accuracy
// of its narrow phase against exact references, and on its speed.
//
//	go run . [-only sim|epa|speed]                            # the working tree
//	go run -tags v020 -modfile=go.v020.mod . [-only ...]      # v0.2.0 (XPBD), for comparison
//	go run . -check                                           # the regressions against baseline.json (regression.go)
//	go run . -update                                          # write baseline.json, after a wanted change
//	go run . -scenes ; go run . -compare                      # the scenes of Solver2D (reference.go)
//
// Every scene runs at AkmonEngine's rate: 50 Hz, 12 sub-steps, one worker.
package main

import (
	"flag"
	"fmt"
	"math"
	"math/rand"
	"os"
	"time"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

const (
	dt       = 1.0 / 50
	substeps = 12
	g        = 9.81
)

func body(w *feather.World, t actor.Transform, s actor.ShapeInterface, typ actor.BodyType, mu, e float64) *actor.RigidBody {
	b := actor.NewRigidBody(t, s, typ, 500)
	b.Material.StaticFriction, b.Material.DynamicFriction, b.Material.Restitution = mu, mu, e
	w.AddBody(b)
	return b
}

func ground(w *feather.World, mu float64) {
	body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, actor.BodyTypeStatic, mu, 0)
}

func run(w *feather.World, seconds float64, each func()) {
	for i := 0; i < int(math.Round(seconds/dt)); i++ {
		w.Step(dt)
		if each != nil {
			each()
		}
	}
}

func bad(v float64) bool { return math.IsNaN(v) || math.IsInf(v, 0) }

func angle(q mgl64.Quat) float64 {
	w := math.Min(1, math.Abs(q.W))
	return 2 * math.Acos(w) * 180 / math.Pi
}

// rest: a body resting 10 s; drift of position and rotation after 1 s of settling.
func rest(name string, setup func(w *feather.World) *actor.RigidBody) {
	w := world(1)
	b := setup(w)
	run(w, 1, nil)
	p0, q0 := b.Transform.Position, b.Transform.Rotation
	maxD, maxA := 0.0, 0.0
	run(w, 10, func() {
		maxD = math.Max(maxD, b.Transform.Position.Sub(p0).Len())
		maxA = math.Max(maxA, angle(b.Transform.Rotation.Mul(q0.Inverse())))
	})
	fmt.Printf("%-34s drift %9.3f mm   rotation %7.3f°\n", name, maxD*1000, maxA)
}

// stack of n boxes (0.5 m cubes, 1 mm gaps) on the ground; top box displacement over 10 s.
func stack(n int, workers int) {
	w := world(workers)
	ground(w, 0.6)
	var bs []*actor.RigidBody
	for i := 0; i < n; i++ {
		bs = append(bs, body(w, tr(mgl64.Vec3{0, 0.25 + float64(i)*0.501, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0))
	}
	top := bs[n-1]
	start := top.Transform.Position
	run(w, 10, nil)
	d := top.Transform.Position.Sub(start)
	fell := false
	for i, b := range bs {
		if math.Abs(b.Transform.Position.Y()-(0.25+float64(i)*0.5)) > 0.1 || bad(b.Transform.Position.X()) {
			fell = true
		}
	}
	fmt.Printf("stack of %2d (workers %d)            top moved %8.3f mm (horizontal %8.3f mm) fell=%v\n", n, workers, d.Len()*1000, math.Hypot(d.X(), d.Z())*1000, fell)
}

func pyramid() {
	w := world(1)
	ground(w, 0.6)
	var bs []*actor.RigidBody
	for row := 0; row < 4; row++ {
		for i := 0; i < 4-row; i++ {
			x := (float64(i) - float64(3-row)/2) * 0.52
			y := 0.25 + float64(row)*0.501
			bs = append(bs, body(w, tr(mgl64.Vec3{x, y, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0))
		}
	}
	x0 := make([]mgl64.Vec3, len(bs))
	for i, b := range bs {
		x0[i] = b.Transform.Position
	}
	run(w, 10, nil)
	maxD := 0.0
	for i, b := range bs {
		maxD = math.Max(maxD, b.Transform.Position.Sub(x0[i]).Len())
	}
	fmt.Printf("pyramid of 10                      worst box moved %8.3f mm\n", maxD*1000)
}

// incline: box on a plane tilted by deg; analytic answer from Coulomb friction.
func incline(deg, mu float64) {
	w := world(1)
	th := deg * math.Pi / 180
	q := mgl64.QuatRotate(th, mgl64.Vec3{0, 0, 1})
	n := q.Rotate(mgl64.Vec3{0, 1, 0})
	body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), &actor.Plane{Normal: n}, actor.BodyTypeStatic, mu, 0)
	b := body(w, tr(n.Mul(0.25), q), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, mu, 0)
	run(w, 0.5, nil)
	p0 := b.Transform.Position
	T := 2.0
	run(w, T, nil)
	d := b.Transform.Position.Sub(p0).Len()
	a := g * (math.Sin(th) - mu*math.Cos(th))
	want := 0.0
	if a > 0 {
		v0 := a * 0.5
		want = v0*T + 0.5*a*T*T
	}
	fmt.Printf("incline %2.0f° μ=%.1f                   slid %8.3f m  (expected %6.3f m)\n", deg, mu, d, want)
}

// bounce: sphere dropped from 1 m (bottom) with restitution e; apex of first rebound.
func bounce(e float64) {
	w := world(1)
	ground(w, 0)
	w.Bodies[0].Material.Restitution = e
	b := body(w, tr(mgl64.Vec3{0, 1.25, 0}, mgl64.QuatIdent()), &actor.Sphere{Radius: 0.25}, actor.BodyTypeDynamic, 0, e)
	hit, apex := false, 0.0
	run(w, 3, func() {
		y := b.Transform.Position.Y() - 0.25
		if y < 0.01 {
			hit = true
		}
		if hit {
			apex = math.Max(apex, y)
		}
	})
	fmt.Printf("bounce e=%.1f from 1 m               rebound %6.3f m  (expected %6.3f m)\n", e, apex, e*e)
}

// ramp: sphere dropped on a static box rotated by 30°; it must roll down its surface.
func ramp() {
	w := world(1)
	q := mgl64.QuatRotate(30*math.Pi/180, mgl64.Vec3{0, 0, 1})
	body(w, tr(mgl64.Vec3{0, 0, 0}, q), &actor.Box{HalfExtents: mgl64.Vec3{3, 0.25, 1}}, actor.BodyTypeStatic, 0.5, 0)
	b := body(w, tr(mgl64.Vec3{0, 1.5, 0}, mgl64.QuatIdent()), &actor.Sphere{Radius: 0.25}, actor.BodyTypeDynamic, 0.5, 0)
	n := q.Rotate(mgl64.Vec3{0, 1, 0})
	minGap, maxGap, landed := math.Inf(1), math.Inf(-1), false
	run(w, 1.5, func() {
		gap := b.Transform.Position.Dot(n) - 0.25 - 0.25
		if gap < 0.01 {
			landed = true
		}
		if landed && math.Abs(b.Transform.Position.X()) < 2.2 {
			minGap, maxGap = math.Min(minGap, gap), math.Max(maxGap, gap)
		}
	})
	fmt.Printf("sphere rolling on a rotated static box: gap to its surface between %.4f and %.4f m (expected ~0), rolled to x=%.2f\n", minGap, maxGap, b.Transform.Position.X())
}

// force: 1 kg-equivalent body in zero gravity, constant force for 1 s.
func force() {
	w := world(1)
	w.Gravity = mgl64.Vec3{}
	b := body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), &actor.Sphere{Radius: 0.25}, actor.BodyTypeDynamic, 0, 0)
	m := b.Material.GetMass()
	F := 10.0
	for i := 0; i < 50; i++ {
		b.AddForce(mgl64.Vec3{F, 0, 0})
		w.Step(dt)
	}
	fmt.Printf("force %g N for 1 s on %.1f kg       velocity %10.3f m/s (expected %6.3f)\n", F, m, b.Velocity.X(), F/m)
}

func snapshot(w *feather.World) []mgl64.Vec3 {
	var s []mgl64.Vec3
	for _, b := range w.Bodies {
		s = append(s, b.Transform.Position)
	}
	return s
}

func determinism() {
	build := func(workers int) *feather.World {
		w := world(workers)
		ground(w, 0.6)
		r := rand.New(rand.NewSource(1))
		for i := 0; i < 40; i++ {
			q := mgl64.QuatRotate(r.Float64()*math.Pi, mgl64.Vec3{r.Float64(), r.Float64(), r.Float64()}.Normalize())
			body(w, tr(mgl64.Vec3{r.Float64()*3 - 1.5, 0.5 + float64(i)*0.6, r.Float64()*3 - 1.5}, q), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0)
		}
		run(w, 5, nil)
		return w
	}
	same := func(a, b []mgl64.Vec3) int {
		n := 0
		for i := range a {
			if a[i] != b[i] {
				n++
			}
		}
		return n
	}
	a, b := snapshot(build(1)), snapshot(build(1))
	c, d := snapshot(build(8)), snapshot(build(8))
	fmt.Printf("determinism 40 falling boxes, 5 s   workers1 run-vs-run: %d/40 differ; workers8 run-vs-run: %d/40 differ; w1 vs w8: %d/40 differ\n", same(a, b), same(c, d), same(a, c))
}

func main() {
	part := flag.String("only", "", "sim, epa or speed (default: all)")
	check := flag.Bool("check", false, "compare to the reference baseline.json, exit 1 on a regression")
	update := flag.Bool("update", false, "write the reference baseline.json")
	referenceScenes := flag.Bool("scenes", false, "run the scenes of Solver2D (bench/scenes) at their full size")
	compare := flag.Bool("compare", false, "print the scenes of the working tree and of v0.2.0 side by side")
	flag.Parse()
	if *referenceScenes || *compare {
		ok := *referenceScenes && runScenes() || *compare && compareScenes()
		if !ok {
			os.Exit(1)
		}
		return
	}
	if *check || *update {
		if !regressions(*update) {
			os.Exit(1)
		}
		return
	}
	if *part == "" || *part == "sim" {
		rest("box on ground", func(w *feather.World) *actor.RigidBody {
			ground(w, 0.6)
			return body(w, tr(mgl64.Vec3{0, 0.25, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0)
		})
		rest("box on static box", func(w *feather.World) *actor.RigidBody {
			body(w, tr(mgl64.Vec3{0, -0.5, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}}, actor.BodyTypeStatic, 0.6, 0)
			return body(w, tr(mgl64.Vec3{0, 0.25, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0)
		})
		rest("box off-centre on static box", func(w *feather.World) *actor.RigidBody {
			body(w, tr(mgl64.Vec3{0, -0.5, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{2, 0.5, 2}}, actor.BodyTypeStatic, 0.6, 0)
			return body(w, tr(mgl64.Vec3{1.3, 0.25, 0.7}, mgl64.QuatRotate(0.5, mgl64.Vec3{0, 1, 0})), &actor.Box{HalfExtents: mgl64.Vec3{0.4, 0.1, 0.2}}, actor.BodyTypeDynamic, 0.6, 0)
		})
		rest("sphere on ground", func(w *feather.World) *actor.RigidBody {
			ground(w, 0.6)
			return body(w, tr(mgl64.Vec3{0, 0.25, 0}, mgl64.QuatIdent()), &actor.Sphere{Radius: 0.25}, actor.BodyTypeDynamic, 0.6, 0)
		})
		stack(3, 1)
		stack(5, 1)
		stack(10, 1)
		stack(5, 8)
		pyramid()
		incline(20, 0.6)
		incline(20, 0.2)
		incline(35, 0.3)
		bounce(0.5)
		bounce(0.0)
		ramp()
		force()
		determinism()
	}
	if *part == "speed" {
		speed()
	}
	if *part == "" || *part == "epa" {
		epaAccuracy()
	}
}

// speed times scenes of growing size (wall clock of the whole simulation).
func speed() {
	scene := func(n int) *feather.World {
		w := world(1)
		ground(w, 0.6)
		r := rand.New(rand.NewSource(3))
		side := int(math.Ceil(math.Sqrt(float64(n))))
		for i := 0; i < n; i++ {
			x, z := float64(i%side)*0.6-float64(side)*0.3, float64((i/side)%side)*0.6-float64(side)*0.3
			y := 0.3 + float64(i/(side*side))*0.6 + r.Float64()*0.2
			var s actor.ShapeInterface = &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}
			if i%2 == 1 {
				s = &actor.Sphere{Radius: 0.25}
			}
			body(w, tr(mgl64.Vec3{x, y, z}, mgl64.QuatIdent()), s, actor.BodyTypeDynamic, 0.6, 0)
		}
		return w
	}
	for _, n := range []int{10, 100, 500} {
		w := scene(n)
		start := time.Now()
		run(w, 3, nil)
		el := time.Since(start)
		fmt.Printf("%-14s %4d bodies, 3 s simulated (150 steps x %d substeps): %8.1f ms  (%.3f ms/step)\n", version, n, substeps, float64(el.Microseconds())/1000, float64(el.Microseconds())/1000/150)
	}
	w := world(1)
	ground(w, 0.6)
	for row := 0; row < 10; row++ {
		for i := 0; i < 10-row; i++ {
			body(w, tr(mgl64.Vec3{(float64(i) - float64(9-row)/2) * 0.52, 0.25 + float64(row)*0.501, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0)
		}
	}
	start := time.Now()
	run(w, 3, nil)
	el := time.Since(start)
	maxY := 0.0
	for _, b := range w.Bodies[1:] {
		maxY = math.Max(maxY, b.Transform.Position.Y())
	}
	fmt.Printf("%-14s pyramid of 55, 3 s: %8.1f ms, top box at y=%.3f (expected %.3f)\n", version, float64(el.Microseconds())/1000, maxY, 0.25+9*0.5)
}
