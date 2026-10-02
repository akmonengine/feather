package scenes

import (
	"fmt"
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== CONTACT SCENES ==========

// contactSpeed: the fastest relative speed the solver pushes 2 overlapping bodies apart (feather.ContactSpeed, m/s). A
// body pushed through a chain of n overlapping contacts moves at n × contactSpeed at most: faster, it was thrown
const contactSpeed = 3.0

// layersSlop: a stack of n contacts may sink by the tolerance of the detection at each contact
func layersSlop(layers int) float64 { return float64(layers) * slop }

// A cube of 2 m falls on the ground and rests: it rests at its height, and doesn't move anymore
var singleBox = Scene{
	Name: "single box",
	Run: func(size Size, play Player) Result {
		w := newWorld()
		ground(w, mgl64.Vec3{}, material{friction: 0.5})
		body := box(w, mgl64.Vec3{0, 4, 0}, mgl64.Vec3{1, 1, 1}, material{friction: 0.5, density: 1})
		play(w, 1, nil)
		start := body.Transform.Position
		drift := 0.0
		play(w, 2, func() { drift = math.Max(drift, body.Transform.Position.Sub(start).Len()) })
		return Result{"height error": mm(math.Abs(body.Transform.Position.Y() - 1)), "rest drift": mm(drift)}
	},
	Check: func(r Result) error {
		return firstError(atMost(r, "height error", slop*1000, "1 contact"), atMost(r, "rest drift", slop*1000, "1 contact"))
	},
}

// 3 spheres stacked, the top one 100 times heavier, removed after 2.4 s: the impulses stored for the warm start must not
// throw the 2 others up
var warmStartEnergy = Scene{
	Name: "warm start energy",
	Run: func(size Size, play Player) Result {
		w := newWorld()
		ground(w, mgl64.Vec3{}, defaultMaterial)
		light := material{friction: 0.6, density: 1}
		sphere(w, mgl64.Vec3{0, 0.5, 0}, 0.5, light)
		middle := sphere(w, mgl64.Vec3{0, 1.5, 0}, 0.5, light)
		top := sphere(w, mgl64.Vec3{0, 2.5, 0}, 0.5, material{friction: 0.6, density: 100})
		play(w, 2.4, nil)
		w.RemoveBody(top)
		highest := middle.Transform.Position.Y()
		play(w, 2.6, func() { highest = math.Max(highest, middle.Transform.Position.Y()) })
		return Result{"overshoot": mm(highest - middle.Transform.Position.Y())}
	},
	Check: func(r Result) error {
		return atMost(r, "overshoot", slop*1000, "the spheres rise only by what the load pressed")
	},
}

// highMassRatio1: 3 pyramids of cubes of 2 m, a cube 100, 200 or 300 times heavier dropped on each from 2 m. Where the
// heavy cube rests is a draw: its variants drop it from 1.95 to 2.05 m
var highMassRatio1 = Scene{
	Name:  "high mass ratio 1",
	Run:   heavyCubes(2),
	Draws: []string{"heavy cube sag"},
	Vary:  across(1.95, 2.05, heavyCubes),
	// the pyramids hold: no cube leaves its place (moves by its half size), the heavy cube stays on the top
	Check: func(r Result) error {
		return firstError(atMost(r, "worst drift", 1000, "less than the half size of a cube"),
			atMost(r, "heavy cube sag", 1000, "less than the half size of a cube"))
	},
}

// heavyCubes: the scene high mass ratio 1, the heavy cubes dropped from drop (m) above the pyramids
func heavyCubes(drop float64) func(size Size, play Player) Result {
	return func(size Size, play Player) Result {
		count := map[Size]int{Small: 4, Full: 10}[size]
		w := newWorld()
		ground(w, mgl64.Vec3{}, material{friction: 0.5})
		var bodies, tops []*actor.RigidBody
		for j := 0; j < 3; j++ {
			origin := mgl64.Vec3{float64(j-1) * float64(2*count+2), 0, 0}
			bodies = append(bodies, squarePyramid(w, origin, count, 1, 0, material{friction: 0.5, density: 1})...)
			// the heavy cube, above the top
			top := origin.Add(mgl64.Vec3{0, 1 + 2*float64(count) + drop, 0})
			tops = append(tops, box(w, top, mgl64.Vec3{1, 1, 1}, material{friction: 0.5, density: 100 * float64(j+1)}))
		}
		start := positions(bodies)
		play(w, 5, nil)
		sag := 0.0
		for _, top := range tops {
			sag = math.Max(sag, 1+2*float64(count)-top.Transform.Position.Y())
		}
		return Result{"worst drift": mm(worstDrift(bodies, start)), "heavy cube sag": mm(sag), "layers": {float64(count), ""}}
	}
}

// highMass: a slab of 20 × 20 × 1 m dropped from 15 m on 2 cubes of 1 m, 400 times lighter, on a plane or on a thick
// static box: the scene of Solver2D extruded by 1 m (the same supports, the same ratio)
func highMass(thickGround bool) func(size Size, play Player) Result {
	return func(size Size, play Player) Result {
		w := newWorld()
		if thickGround {
			staticBox(w, mgl64.Vec3{0, -2, 0}, mgl64.QuatIdent(), mgl64.Vec3{40, 2, 40}, defaultMaterial)
		} else {
			ground(w, mgl64.Vec3{}, defaultMaterial)
		}
		var small []*actor.RigidBody
		for _, x := range []float64{-9, 9} {
			small = append(small, box(w, mgl64.Vec3{x, 0.5, 0}, mgl64.Vec3{0.5, 0.5, 0.5}, defaultMaterial))
		}
		big := box(w, mgl64.Vec3{0, 26, 0}, mgl64.Vec3{10, 10, 0.5}, defaultMaterial)
		start := positions(small)
		bounce, into, landed := 0.0, 0.0, false
		play(w, 5, func() {
			landed = landed || big.Transform.Position.Y() < 11.5
			if landed {
				bounce = math.Max(bounce, big.Velocity.Y())
			}
			for _, b := range small {
				into = math.Max(into, 0.5-b.Transform.Position.Y())
			}
		})
		return Result{"slab sag": mm(11 - big.Transform.Position.Y()), "small cubes drift": mm(worstDrift(small, start)),
			"bounce": {bounce, "m/s"}, "small cubes into ground": mm(into)}
	}
}

// checkHighMass: the slab stays on the small cubes (sinks less than their size), the small cubes stay in place (move
// less than their half size). A soft contact is a spring of a frequency, whatever the mass: under a load 400 times
// heavier, it sinks by about 400 g / ω²
func checkHighMass(r Result) error {
	return firstError(atMost(r, "slab sag", 1000, "less than the size of a small cube"),
		atMost(r, "small cubes drift", 500, "less than the half size of a small cube"))
}

var highMassRatio2 = Scene{Name: "high mass ratio 2", Run: highMass(false), Check: checkHighMass}
var highMassRatio3 = Scene{Name: "high mass ratio 3", Run: highMass(true), Check: checkHighMass}

// centeredImpact: a cube of 2 m, 10 times heavier, dropped from 2 m exactly on another, without friction: only the order
// of the points of a contact can turn them. Not in Solver2D: the probe of the gap of #821
var centeredImpact = Scene{
	Name: "centered impact",
	Run: func(size Size, play Player) Result {
		w := newWorld()
		ground(w, mgl64.Vec3{}, material{})
		low := box(w, mgl64.Vec3{0, 1, 0}, mgl64.Vec3{1, 1, 1}, material{density: 1})
		top := box(w, mgl64.Vec3{0, 5, 0}, mgl64.Vec3{1, 1, 1}, material{density: 10})
		lateral, bounce, landed := 0.0, 0.0, false
		play(w, 3, func() {
			landed = landed || top.Transform.Position.Y() < 3.2
			if landed {
				bounce = math.Max(bounce, top.Velocity.Y())
			}
			for _, b := range []*actor.RigidBody{low, top} {
				lateral = math.Max(lateral, math.Hypot(b.Transform.Position.X(), b.Transform.Position.Z()))
			}
		})
		return Result{"lateral drift": mm(lateral), "bounce": {bounce, "m/s"}}
	},
	// the top cube stays on the low one
	Check: func(r Result) error { return atMost(r, "lateral drift", 1000, "less than the half size of a cube") },
}

// frictionRamp: 5 cubes of friction 0.75, 0.5, 0.35, 0.1, 0 on a ramp of 0.25 rad and friction 0.2. The friction of 2
// bodies is their geometric mean: the 3 first stop (√(0.35 × 0.2) = 0.265 > tan 0.25 = 0.255), the 2 others slide with
// the acceleration of Coulomb, g (sin θ - μ cos θ)
var frictionRamp = Scene{
	Name: "friction ramp",
	Run: func(size Size, play Player) Result {
		const angle, rampFriction, duration = 0.25, 0.2, 2.0
		w := newWorld()
		rotation := mgl64.QuatRotate(-angle, mgl64.Vec3{0, 0, 1})
		down, normal := rotation.Rotate(mgl64.Vec3{1, 0, 0}), rotation.Rotate(mgl64.Vec3{0, 1, 0})
		center := mgl64.Vec3{0, 10, 0}
		staticBox(w, center, rotation, mgl64.Vec3{13, 0.25, 6}, material{friction: rampFriction})
		frictions := []float64{0.75, 0.5, 0.35, 0.1, 0}
		var cubes []*actor.RigidBody
		for i, friction := range frictions {
			position := center.Add(normal.Mul(0.75)).Add(down.Mul(-10)).Add(mgl64.Vec3{0, 0, float64(i-2) * 2})
			cubes = append(cubes, addBody(w, position, rotation, &actor.Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}, actor.BodyTypeDynamic, material{friction: friction, density: 25}))
		}
		play(w, 0.5, nil)
		start, speeds := positions(cubes), make([]float64, len(cubes))
		for i, c := range cubes {
			speeds[i] = c.Velocity.Dot(down)
		}
		play(w, duration, nil)
		stopped, sliding := 0.0, 0.0
		for i, c := range cubes {
			travelled := c.Transform.Position.Sub(start[i]).Dot(down)
			mixed := math.Sqrt(frictions[i] * rampFriction)
			if mixed >= math.Tan(angle) {
				stopped = math.Max(stopped, math.Abs(travelled))
				continue
			}
			acceleration := gravity * (math.Sin(angle) - mixed*math.Cos(angle))
			want := speeds[i]*duration + 0.5*acceleration*duration*duration
			sliding = math.Max(sliding, math.Abs(travelled-want))
		}
		return Result{"stopped slide": mm(stopped), "sliding error": mm(sliding)}
	},
	Check: func(r Result) error {
		// the constant acceleration a integrated by sub-steps of h during T: the position is late by a h T / 2 at most
		const angle, duration = 0.25, 2.0
		bound := gravity*math.Sin(angle)*(Dt/substeps)*duration/2 + slop
		return firstError(atMost(r, "stopped slide", slop*1000, "static friction"), atMost(r, "sliding error", bound*1000, "Coulomb"))
	},
}

// overlapRecovery: a pyramid of cubes created overlapping by 25 %: the solver pushes them apart without throwing them
var overlapRecovery = Scene{
	Name: "overlap recovery",
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 4, Full: 6}[size]
		w := newWorld()
		ground(w, mgl64.Vec3{}, defaultMaterial)
		cubes := squarePyramid(w, mgl64.Vec3{}, count, 0.5, -0.25, defaultMaterial)
		fastest := 0.0
		// the pyramid falls apart as the cubes are pushed out: 6 layers need 4 s to part
		play(w, 5, func() { fastest = math.Max(fastest, maxSpeed(cubes)) })
		overlap := 0.0
		for i := range cubes {
			for j := i + 1; j < len(cubes); j++ {
				overlap = math.Max(overlap, boxOverlap(cubes[i], cubes[j]))
			}
		}
		return Result{"max speed": {fastest, "m/s"}, "final overlap": mm(overlap), "layers": {float64(count), ""}}
	},
	Check: func(r Result) error {
		// the top cube is pushed through the layers & the ground
		return firstError(atMost(r, "max speed", contactSpeed*r["layers"].Value, "ContactSpeed per contact in series"),
			atMost(r, "final overlap", slop*1000, "resting contacts"))
	},
}

// verticalStack: cubes of 1 m dropped by 10 cm on each other, shifted by 1 cm to alternate sides
var verticalStack = Scene{
	Name: "vertical stack",
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 10, Full: 15}[size]
		w := newWorld()
		staticBox(w, mgl64.Vec3{0, -1, 0}, mgl64.QuatIdent(), mgl64.Vec3{100, 1, 100}, material{friction: 0.3})
		var cubes []*actor.RigidBody
		for i := 0; i < count; i++ {
			shift := 0.01
			if i%2 == 0 {
				shift = -shift
			}
			cubes = append(cubes, box(w, mgl64.Vec3{shift, 0.55 + 1.1*float64(i), shift}, mgl64.Vec3{0.5, 0.5, 0.5}, material{friction: 0.3, density: 1}))
		}
		start := positions(cubes)
		play(w, 5, nil)
		drift := 0.0
		for i, c := range cubes {
			d := c.Transform.Position.Sub(start[i])
			drift = math.Max(drift, math.Hypot(d.X(), d.Z()))
		}
		return Result{"horizontal drift": mm(drift), "layers": {float64(count), ""}}
	},
	Check: func(r Result) error {
		return atMost(r, "horizontal drift", layersSlop(int(r["layers"].Value))*1000, "a contact per layer")
	},
}

// pyramid: a square pyramid of cubes of 1 m, built touching, stands
var pyramid = Scene{
	Name: "pyramid",
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 8, Full: 20}[size]
		w := newWorld()
		staticBox(w, mgl64.Vec3{0, -1, 0}, mgl64.QuatIdent(), mgl64.Vec3{100, 1, 100}, defaultMaterial)
		cubes := squarePyramid(w, mgl64.Vec3{}, count, 0.5, 0, defaultMaterial)
		start := positions(cubes)
		play(w, 5, nil)
		return Result{"worst drift": mm(worstDrift(cubes, start)), "layers": {float64(count), ""}}
	},
	Check: func(r Result) error {
		return atMost(r, "worst drift", layersSlop(int(r["layers"].Value))*1000, "a contact per layer")
	},
}

// rush: spheres pulled towards a static sphere without gravity (12.7 m/s², as in Solver2D), from a spiral of 5 m to
// 25 m: they gather into a ball. The ball is a block held by a few contacts, rolling on the static sphere: how fast it
// still moves after 5 s and how much it is pressed are draws, their variants start the spiral from 5 to 7.4 m
var rush = Scene{
	Name:  "rush",
	Run:   rushFrom(5),
	Draws: []string{"final speed", "final overlap"},
	Vary:  across(5, 7.4, rushFrom),
	Check: func(r Result) error {
		// free fall from 25 m under 12.7 m/s² reaches 25 m/s
		return atMost(r, "max speed", math.Sqrt(2*1000/(100*math.Pi*0.25)*25), "no body faster than its fall")
	},
}

// rushFrom: the scene rush, the spiral starting at radius (m) from the static sphere
func rushFrom(radius float64) func(size Size, play Player) Result {
	return func(size Size, play Player) Result {
		const pull = 1000 / (100 * math.Pi * 0.25)
		count := map[Size]int{Small: 100, Full: 400}[size]
		w := newWorld()
		w.Gravity = mgl64.Vec3{}
		m := material{friction: 0.2, density: 100}
		addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.5}, actor.BodyTypeStatic, m)
		var spheres []*actor.RigidBody
		golden := math.Pi * (3 - math.Sqrt(5))
		for i := 0; i < count; i++ {
			// the directions of a Fibonacci sphere, further and further
			y := 1 - 2*(float64(i)+0.5)/float64(count)
			ring := math.Sqrt(1 - y*y)
			direction := mgl64.Vec3{ring * math.Cos(golden*float64(i)), y, ring * math.Sin(golden*float64(i))}
			spheres = append(spheres, sphere(w, direction.Mul(radius+0.05*float64(i)), 0.5, m))
		}
		pullAll := func() {
			for _, s := range spheres {
				if distance := s.Transform.Position.Len(); distance > 0.1 {
					s.AddForce(s.Transform.Position.Mul(-pull * s.Material.GetMass() / distance))
				}
			}
		}
		pullAll()
		fastest := 0.0
		play(w, 5, func() {
			fastest = math.Max(fastest, maxSpeed(spheres))
			pullAll()
		})
		overlap := 0.0
		for i := range spheres {
			for j := i + 1; j < len(spheres); j++ {
				overlap = math.Max(overlap, sphereOverlap(spheres[i], spheres[j]))
			}
		}
		return Result{"max speed": {fastest, "m/s"}, "final speed": {maxSpeed(spheres), "m/s"}, "final overlap": mm(overlap)}
	}
}

// doubleDomino: 15 dominos 1 m apart, the first pushed at its top by 0.2 N·s: each one topples the next, all fall. The
// step where the last one falls is a draw: its variants push the first from 0.2 to 0.22 N·s
var doubleDomino = Scene{
	Name:  "double domino",
	Run:   dominoRow(0.2),
	Draws: []string{"time of the last"},
	Vary:  across(0.2, 0.22, dominoRow),
	Check: func(r Result) error {
		if r["fallen"].Value != 15 {
			return fmt.Errorf("%v dominos fell, want 15", r["fallen"].Value)
		}
		return nil
	},
}

// dominoRow: the scene double domino, the first domino pushed by push (N·s)
func dominoRow(push float64) func(size Size, play Player) Result {
	return func(size Size, play Player) Result {
		const count = 15
		w := newWorld()
		staticBox(w, mgl64.Vec3{0, -1, 0}, mgl64.QuatIdent(), mgl64.Vec3{100, 1, 100}, defaultMaterial)
		var dominos []*actor.RigidBody
		for i := 0; i < count; i++ {
			dominos = append(dominos, box(w, mgl64.Vec3{-0.5*count + float64(i), 0.5, 0}, mgl64.Vec3{0.125, 0.5, 0.5}, defaultMaterial))
		}
		// an impulse along X at the top of the first domino
		first := dominos[0]
		impulse, arm := mgl64.Vec3{push, 0, 0}, mgl64.Vec3{0, 0.5, 0}
		first.Velocity = impulse.Mul(1 / first.Material.GetMass())
		first.AngularVelocity = first.GetInverseInertiaWorld().Mul3x1(arm.Cross(impulse))
		tipping := math.Atan(0.125 / 0.5)
		fallTime, elapsed := 0.0, 0.0
		// 16 s: the last domino falls after 7 s, then the leaning dominos settle flat on the ground
		play(w, 16, func() {
			elapsed += Dt
			if tilt(dominos[count-1]) > tipping && fallTime == 0 {
				fallTime = elapsed
			}
		})
		fallen := 0
		for _, d := range dominos {
			if tilt(d) > tipping {
				fallen++
			}
		}
		return Result{"fallen": {float64(fallen), ""}, "time of the last": {fallTime, "s"}}
	}
}

// confined: spheres created overlapping in a box too small for them, without gravity: they stay inside, and are not
// thrown
var confined = Scene{
	Name: "confined",
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 6, Full: 9}[size]
		w := newWorld()
		w.Gravity = mgl64.Vec3{}
		// the inside of the box: 0.8 m per sphere of 1 m
		half := 0.4 * float64(count)
		for axis := 0; axis < 3; axis++ {
			for _, side := range []float64{-1, 1} {
				var center, extents mgl64.Vec3
				center[axis] = side * (half + 0.5)
				extents = mgl64.Vec3{half + 1, half + 1, half + 1}
				extents[axis] = 0.5
				staticBox(w, center, mgl64.QuatIdent(), extents, defaultMaterial)
			}
		}
		var spheres []*actor.RigidBody
		pitch := 0.72
		for i := 0; i < count; i++ {
			for j := 0; j < count; j++ {
				for k := 0; k < count; k++ {
					p := mgl64.Vec3{float64(i), float64(j), float64(k)}.Sub(mgl64.Vec3{1, 1, 1}.Mul(float64(count-1) / 2)).Mul(pitch)
					spheres = append(spheres, sphere(w, p, 0.5, defaultMaterial))
				}
			}
		}
		fastest := 0.0
		play(w, 5, func() { fastest = math.Max(fastest, maxSpeed(spheres)) })
		escaped := 0
		for _, s := range spheres {
			p := s.Transform.Position
			if math.Abs(p.X()) > half || math.Abs(p.Y()) > half || math.Abs(p.Z()) > half {
				escaped++
			}
		}
		return Result{"max speed": {fastest, "m/s"}, "escaped": {float64(escaped), ""}, "row": {float64(count), ""}}
	},
	Check: func(r Result) error {
		if r["escaped"].Value > 0 {
			return fmt.Errorf("%v spheres escaped", r["escaped"].Value)
		}
		// a row of spheres between 2 walls: row + 1 contacts in series
		return atMost(r, "max speed", contactSpeed*(r["row"].Value+1), "ContactSpeed per contact in series")
	},
}

// cardHouse: a house of cards of 40 × 30 cm and 2 mm thick, leaning by 25° (from PEEL, as in Solver2D): it stands
var cardHouse = Scene{
	Name: "card house",
	Run: func(size Size, play Player) Result {
		levels := map[Size]int{Small: 3, Full: 5}[size]
		const height, thickness, depth = 0.2, 0.001, 0.15
		w := newWorld()
		m := material{friction: 0.7, density: 1}
		staticBox(w, mgl64.Vec3{0, -2, 0}, mgl64.QuatIdent(), mgl64.Vec3{40, 2, 40}, m)
		half := mgl64.Vec3{thickness, height, depth}
		card := func(x, y, angle float64) *actor.RigidBody {
			return addBody(w, mgl64.Vec3{x, y, 0}, mgl64.QuatRotate(angle, mgl64.Vec3{0, 0, 1}), &actor.Box{HalfExtents: half}, actor.BodyTypeDynamic, m)
		}
		var cards []*actor.RigidBody
		lean := 25 * math.Pi / 180
		x0, y := 0.0, height-0.02
		for n := levels; n > 0; n-- {
			x := x0
			for i := 0; i < n; i++ {
				if i != n-1 {
					cards = append(cards, card(x+0.25, y+height-0.015, math.Pi/2))
				}
				cards = append(cards, card(x, y, -lean))
				x += 0.175
				cards = append(cards, card(x, y, lean))
				x += 0.175
			}
			y += 2*height - 0.03
			x0 += 0.175
		}
		play(w, 0.5, nil)
		start := positions(cards)
		play(w, 3, nil)
		return Result{"worst drift": mm(worstDrift(cards, start)), "levels": {float64(levels), ""}}
	},
	Check: func(r Result) error {
		return atMost(r, "worst drift", layersSlop(int(r["levels"].Value))*1000, "a contact per level")
	},
}

// circleStack: spheres of 1 m dropped exactly above each other: an aligned stack has no horizontal force, only the
// rounding moves it sideways
var circleStack = Scene{
	Name: "circle stack",
	Run: func(size Size, play Player) Result {
		count := map[Size]int{Small: 5, Full: 10}[size]
		w := newWorld()
		ground(w, mgl64.Vec3{}, defaultMaterial)
		var spheres []*actor.RigidBody
		for i := 0; i < count; i++ {
			spheres = append(spheres, sphere(w, mgl64.Vec3{0, 4 + 3*float64(i), 0}, 1, defaultMaterial))
		}
		drift := 0.0
		play(w, 8, func() {
			for _, s := range spheres {
				drift = math.Max(drift, math.Hypot(s.Transform.Position.X(), s.Transform.Position.Z()))
			}
		})
		top := spheres[count-1].Transform.Position.Y()
		return Result{"horizontal drift": mm(drift), "top height error": mm(math.Abs(top - (2*float64(count) - 1)))}
	},
	Check: func(r Result) error {
		return atMost(r, "horizontal drift", 1e-6*1000, "no horizontal force")
	},
}
