//go:build !v020

package main

import (
	"bufio"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"hash/fnv"
	"math"
	"math/rand"
	"os"
	"runtime"
	"slices"
	"strings"
	"sync"
	"time"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/bench/scenes"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== REGRESSIONS ==========
// Scenes run against a reference committed in baseline.json:
//
//	go run . -check     # compare the working tree to the reference, exit 1 on a regression
//	go run . -update    # write the reference, after a wanted change
//
// Each scene gives:
//   - its fingerprint, a hash of the bits of the positions & rotations at the end: it must be identical on the same
//     architecture (GOARCH). A change of behaviour, even tiny, changes it: -update after a wanted change
//   - its quality metrics, each with its tolerance (qualityTolerances): only a worse value is a regression
//   - its draws: the measures decided by the rounding, followed by their median over the variants of the scene, with a
//     tolerance coming from their spread (DRAWS, below)
//   - its speed: the time of a step and of its phases (feather.Profile), each the best of speedRuns runs, every run at
//     another depth of the stack (stack.go: the time of a phase depends on it). Only compared on the machine of the
//     reference: a step slower by more than stepTolerance, or a phase taking more than 5 % of the step and slower by
//     more than phaseTolerance, is a regression

const (
	baselineFile = "baseline.json"

	// speedRuns: the speed of a step and of each phase is its best run. Each run starts at another depth of the stack,
	// pageSize/speedRuns apart (stack.go)
	speedRuns = 3

	// stepTolerance: a step may be 20 % slower (the noise of a shared machine)
	stepTolerance = 0.20

	// phaseTolerance: a phase may be 30 % slower (shorter, noisier)
	phaseTolerance = 0.30

	// phaseShare: the phases under 5 % of the step are too short to be compared
	phaseShare = 0.05

	// minComparedStepMs: under 0.1 ms per step, the noise of the timer dominates: the speed is not compared
	minComparedStepMs = 0.1
)

// qualityTolerances: how much worse a quality metric may get, by unit
var qualityTolerances = map[string]float64{
	"mm":  0.5,    // a depth, a drift: 0.5 mm, a tenth of LinearSlop
	"m":   0.0005, // the same, in m
	"%":   0.1,    // an energy gain, in % of the energy
	"m/s": 0.1,    // a speed
	"s":   0.02,   // a time: one step
	"":    1e-9,   // a count
}

// metric: a quality measure of a scene
type metric struct {
	Value float64 `json:"value"`
	Unit  string  `json:"unit"`
}

// draw: a measure of a scene decided by the rounding, over the variants of the scene (DRAWS, below)
type draw struct {
	// Median of the variants
	Median float64 `json:"median"`
	Unit   string  `json:"unit"`
	// Variants measured
	Variants int `json:"variants"`
	// Tolerance: how far over Median the median of another draw of the same variants may lie
	Tolerance float64 `json:"tolerance"`
	// Worst variant: written to be read, not compared (the tail of a draw is a draw of a few variants)
	Worst float64 `json:"worst"`
}

// sceneResult: what a scene gives
type sceneResult struct {
	Fingerprint string            `json:"fingerprint"`
	Quality     map[string]metric `json:"quality"`
	Draws       map[string]draw   `json:"draws,omitempty"`
	// StepMs: the mean time of a step (ms), PhasesMs the mean time of each phase
	StepMs   float64            `json:"stepMs"`
	PhasesMs map[string]float64 `json:"phasesMs"`
}

// baseline: the reference, and the machine where its speed was measured
type baseline struct {
	Arch    string                 `json:"arch"`
	Machine string                 `json:"machine"`
	Scenes  map[string]sceneResult `json:"scenes"`
}

// regressionScene builds its worlds, runs them with play, and measures its quality. The same scene gives the same bits
type regressionScene struct {
	name string
	run  func() map[string]metric
	// draws: the measures of the scene which are a draw, by name. They are measured over its variants, not by run:
	// variant runs one of them, out of drawVariants, with the player it is given (the variants run along each other)
	draws   []string
	variant func(variant int, play scenes.Player) map[string]metric
}

// recorder: the worlds of the scene being measured, and the time of their steps
type recorder struct {
	// measuring: the quality is measured (the first run), else the scene is only timed
	measuring bool
	worlds    []*feather.World
	step      time.Duration
	phases    map[string]time.Duration
	steps     int
	// results of the queries of the scene (queries.go): they enter its fingerprint
	results []uint64
}

var recording *recorder

// play steps the world for seconds, recording the time of each step
func play(w *feather.World, seconds float64, each func()) {
	if !slices.Contains(recording.worlds, w) {
		recording.worlds = append(recording.worlds, w)
	}
	for i := 0; i < int(math.Round(seconds/dt)); i++ {
		w.Step(dt)
		profile := w.Profile()
		recording.step += profile.Step
		for name, phase := range profilePhases(profile) {
			recording.phases[name] += phase
		}
		recording.steps++
		if each != nil && recording.measuring {
			each()
		}
	}
}

// profilePhases by name
func profilePhases(p feather.Profile) map[string]time.Duration {
	return map[string]time.Duration{
		"broad phase": p.BroadPhase, "narrow phase": p.NarrowPhase, "prepare": p.Prepare, "substeps": p.Substeps,
		"restitution": p.Restitution, "continuous": p.Continuous, "islands": p.Islands,
	}
}

// ========== SCENES ==========

// The depths of a pile are draws: the variants of the 3 piles are other piles, by their seed
var regressionScenes = append([]regressionScene{
	{name: "slope pile", run: func() map[string]metric { return slopePile(2, play) },
		draws: []string{"landing depth", "resting depth"}, variant: func(variant int, play scenes.Player) map[string]metric {
			return slopePile(int64(variant+1), play)
		}},
	{name: "terrain piles", run: terrainPiles,
		draws: []string{"landing depth", "resting depth"}, variant: func(variant int, play scenes.Player) map[string]metric {
			return terrainPile(int64(variant+1), play)
		}},
	{name: "pyramid", run: pyramidDrift},
	{name: "pile of 500", run: pile500},
	{name: "joint chain", run: jointChain},
	{name: "rain on terrain", run: func() map[string]metric { return rainOnTerrain(7, unmeasured) },
		draws: []string{"landing depth"}, variant: func(variant int, play scenes.Player) map[string]metric {
			return rainOnTerrain(int64(variant+1), play)
		}},
	{name: "spinning tops", run: spinningTops},
	{name: "locked bodies", run: lockedBodies},
}, append(queryScenes, solverScenes()...)...)

// solverScenes: the scenes of Solver2D (bench/scenes), at their small size (the full size is for -scenes)
func solverScenes() []regressionScene {
	metrics := func(result scenes.Result) map[string]metric {
		quality := map[string]metric{}
		for name, m := range result {
			quality[name] = metric{m.Value, m.Unit}
		}
		return quality
	}
	var result []regressionScene
	for _, scene := range scenes.All {
		entry := regressionScene{name: "solver2d " + scene.Name, run: func() map[string]metric {
			return metrics(scene.Run(scenes.Small, play))
		}}
		if scene.Vary != nil {
			entry.draws = scene.Draws
			entry.variant = func(variant int, play scenes.Player) map[string]metric {
				return metrics(scene.Vary(scenes.Small, float64(variant)/drawVariants, play))
			}
		}
		result = append(result, entry)
	}
	return result
}

// unmeasured plays without the measures of each step: the depth of a pile in a terrain costs more than its step, and is
// a draw, measured over the variants
func unmeasured(w *feather.World, seconds float64, _ func()) {
	play(w, seconds, nil)
}

// mixedShape: a box, a sphere or a capsule, of 20 to 40 cm
func mixedShape(r *rand.Rand, i int) actor.ShapeInterface {
	switch i % 3 {
	case 1:
		return &actor.Sphere{Radius: 0.2}
	case 2:
		return &actor.Capsule{HalfHeight: 0.25, Radius: 0.12}
	}
	return &actor.Box{HalfExtents: mgl64.Vec3{0.2 + 0.2*r.Float64(), 0.15, 0.25}}
}

func randomTurn(r *rand.Rand) mgl64.Quat {
	return mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{r.Float64(), r.Float64(), r.Float64()}.Normalize())
}

// slopePile: 60 bodies dropped on a slope of 30°, hitting each other and tumbling
func slopePile(seed int64, play scenes.Player) map[string]metric {
	angle := 30 * math.Pi / 180
	normal := mgl64.Vec3{-math.Sin(angle), math.Cos(angle), 0}
	w := world(1)
	slope := body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), &actor.Plane{Normal: normal}, actor.BodyTypeStatic, 0.6, 0)
	r := rand.New(rand.NewSource(seed))
	for i := 0; i < 60; i++ {
		x, z := r.Float64()*8-4, r.Float64()*16-8
		position := mgl64.Vec3{x, x*math.Tan(angle) + 1 + r.Float64()*3, z}
		body(w, tr(position, randomTurn(r)), mixedShape(r, i), actor.BodyTypeDynamic, 0.6, 0)
	}
	landing := 0.0
	play(w, 4, func() { landing = math.Max(landing, deepest(w, slope)) })
	return map[string]metric{
		"landing depth": {landing * 1000, "mm"},
		"resting depth": {deepest(w, slope) * 1000, "mm"},
	}
}

// terrainPiles: 10 piles on hills, for the fingerprint and the speed
func terrainPiles() map[string]metric {
	for seed := int64(1); seed <= 10; seed++ {
		terrainPile(seed, unmeasured)
	}
	return map[string]metric{}
}

// terrainPile: 60 bodies dropped on hills
func terrainPile(seed int64, play scenes.Player) map[string]metric {
	w := world(1)
	terrain := hills(w, seed)
	field := terrain.Shape.(*actor.Heightfield)
	r := rand.New(rand.NewSource(seed + 100))
	for i := 0; i < 60; i++ {
		x, z := r.Float64()*16-8, r.Float64()*16-8
		ground, _ := field.HeightAt(x, z)
		b := body(w, tr(mgl64.Vec3{x, ground + 1 + r.Float64()*3, z}, randomTurn(r)), mixedShape(r, i), actor.BodyTypeDynamic, 0.6, 0)
		b.Material.RollingResistance = 0.1
	}
	landing := 0.0
	play(w, 4, func() { landing = math.Max(landing, deepest(w, terrain)) })
	return map[string]metric{
		"landing depth": {landing * 1000, "mm"},
		"resting depth": {deepest(w, terrain) * 1000, "mm"},
	}
}

// pyramidDrift: a pyramid of 55 boxes stands 10 s
func pyramidDrift() map[string]metric {
	w := world(1)
	ground(w, 0.6)
	for row := 0; row < 10; row++ {
		for i := 0; i < 10-row; i++ {
			position := mgl64.Vec3{(float64(i) - float64(9-row)/2) * 0.52, 0.25 + float64(row)*0.501, 0}
			body(w, tr(position, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}, actor.BodyTypeDynamic, 0.6, 0)
		}
	}
	start := positions(w)
	play(w, 10, nil)
	drift := 0.0
	for i, b := range w.Bodies {
		drift = math.Max(drift, b.Transform.Position.Sub(start[i]).Len())
	}
	return map[string]metric{"worst drift": {drift * 1000, "mm"}}
}

// pile500: 500 boxes & spheres falling on the ground in a column, landing on each other
func pile500() map[string]metric {
	w := world(1)
	floor := body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}, actor.BodyTypeStatic, 0.6, 0)
	const count, side = 500, 12
	for i := 0; i < count; i++ {
		x, z := float64(i%side)*0.55-side*0.275, float64((i/side)%side)*0.55-side*0.275
		var shape actor.ShapeInterface = &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}
		if i%2 == 1 {
			shape = &actor.Sphere{Radius: 0.25}
		}
		body(w, tr(mgl64.Vec3{x, 0.3 + float64(i/(side*side))*0.55, z}, mgl64.QuatIdent()), shape, actor.BodyTypeDynamic, 0.6, 0)
	}
	landing := 0.0
	play(w, 3, func() { landing = math.Max(landing, deepest(w, floor)) })
	return map[string]metric{"landing depth": {landing * 1000, "mm"}}
}

// jointChain: 20 capsules linked by ball joints, hanging from a static body, released horizontal: the chain swings
// without stretching and without gaining energy
func jointChain() map[string]metric {
	w := world(1)
	const links, length = 20, 0.3
	anchor := body(w, tr(mgl64.Vec3{0, 10, 0}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.1, 0.1}}, actor.BodyTypeStatic, 0.5, 0)
	turned := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	previous := anchor
	var chain []*actor.RigidBody
	for i := 0; i < links; i++ {
		center := mgl64.Vec3{-(float64(i) + 0.5) * length, 10, 0}
		link := body(w, tr(center, turned), &actor.Capsule{HalfHeight: length/2 - 0.05, Radius: 0.05}, actor.BodyTypeDynamic, 0.5, 0)
		w.AddJoint(feather.NewBallJoint(previous, link, center.Add(mgl64.Vec3{length / 2, 0, 0}), mgl64.Vec3{1, 0, 0}))
		chain = append(chain, link)
		previous = link
	}
	energy := func() float64 {
		total := 0.0
		for _, b := range chain {
			mass := b.Material.GetMass()
			total += 0.5*mass*b.Velocity.LenSqr() + 0.5*b.AngularVelocity.Dot(b.GetInertiaWorld().Mul3x1(b.AngularVelocity)) + mass*g*b.Transform.Position.Y()
		}
		return total
	}
	start := energy()
	stretch, gain := 0.0, 0.0
	play(w, 5, func() {
		// the end of a link against the start of the next (the axis Y of a link points away from the anchor)
		for i := 0; i+1 < len(chain); i++ {
			end := chain[i].Transform.ToWorld(mgl64.Vec3{0, length / 2, 0})
			next := chain[i+1].Transform.ToWorld(mgl64.Vec3{0, -length / 2, 0})
			stretch = math.Max(stretch, end.Sub(next).Len())
		}
		gain = math.Max(gain, energy()-start)
	})
	return map[string]metric{
		"worst stretch": {stretch * 1000, "mm"},
		"energy gain":   {gain / math.Abs(start) * 100, "%"},
	}
}

// rainOnTerrain: 200 bodies falling on hills, 10 more every 0.1 s
func rainOnTerrain(seed int64, play scenes.Player) map[string]metric {
	w := world(1)
	terrain := hills(w, seed)
	field := terrain.Shape.(*actor.Heightfield)
	r := rand.New(rand.NewSource(seed + 1))
	landing, fell := 0.0, 0
	for step := 0; step < int(math.Round(5/dt)); step++ {
		if step%5 == 0 && len(w.Bodies) < 200 {
			for i := 0; i < 10; i++ {
				x, z := r.Float64()*16-8, r.Float64()*16-8
				ground, _ := field.HeightAt(x, z)
				b := body(w, tr(mgl64.Vec3{x, ground + 4, z}, randomTurn(r)), mixedShape(r, i), actor.BodyTypeDynamic, 0.6, 0)
				b.Velocity = mgl64.Vec3{0, -8, 0}
			}
		}
		play(w, dt, func() { landing = math.Max(landing, deepest(w, terrain)) })
	}
	for _, b := range w.Bodies[1:] {
		if b.Transform.Position.Y() < -5 {
			fell++
		}
	}
	return map[string]metric{
		"landing depth": {landing * 1000, "mm"},
		"fell through":  {float64(fell), ""},
	}
}

// spinningTops: 144 balls & capsules thrown spinning on the ground, with a spinning & a rolling resistance: the balls
// and the capsules standing on a cap are tops braking in place, the capsules lying turn on their side, the tilted ones
// fall and roll (the rolling & the spinning rows of a tilted capsule see each other). All stop and fall asleep
func spinningTops() map[string]metric {
	const (
		side, spacing = 12, 1.5
		radius        = 0.12
		halfHeight    = 0.25
		spin          = 12.0
	)
	w := world(1)
	ground(w, 0.6)
	for i := 0; i < side*side; i++ {
		position := mgl64.Vec3{float64(i%side) * spacing, radius, float64(i/side) * spacing}
		rotation := mgl64.QuatIdent()
		var shape actor.ShapeInterface = &actor.Capsule{HalfHeight: halfHeight, Radius: radius}
		angular := mgl64.Vec3{0, spin, 0}
		switch i % 4 {
		case 0:
			shape = &actor.Sphere{Radius: radius}
		case 1: // standing on its cap
			position[1] = halfHeight + radius
		case 2: // lying
			rotation = mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
		case 3: // tilted by 30°, on its cap: it falls
			rotation = mgl64.QuatRotate(math.Pi/6, mgl64.Vec3{0, 0, 1})
			position[1] = halfHeight*math.Cos(math.Pi/6) + radius
			angular = mgl64.Vec3{1, spin, -2}
		}
		b := body(w, tr(position, rotation), shape, actor.BodyTypeDynamic, 0.6, 0)
		b.Material.SpinningResistance, b.Material.RollingResistance = 0.05, 0.1
		// one top out of two the other way
		if (i/4)%2 == 1 {
			angular = angular.Mul(-1)
		}
		b.AngularVelocity = angular
	}
	start := positions(w)
	elapsed, lastSpin := 0.0, 0.0
	play(w, 3, func() {
		elapsed += dt
		for _, b := range w.Bodies[1:] {
			if b.AngularVelocity.Len() >= actor.DefaultSleepSpeed {
				lastSpin = elapsed
			}
		}
	})
	awake, drift := 0, 0.0
	for i, b := range w.Bodies[1:] {
		if !b.IsSleeping {
			awake++
		}
		// the tops: the balls and the standing capsules
		if i%4 < 2 {
			drift = math.Max(drift, b.Transform.Position.Sub(start[i+1]).Len())
		}
	}
	return map[string]metric{
		"last spin":       {lastSpin, "s"},
		"awake":           {float64(awake), ""},
		"worst top drift": {drift * 1000, "mm"},
	}
}

// lockedBodies: a pile in 2D (90 bodies which can't move along Z, nor turn around X and Y), 10 characters (upright
// capsules which only turn around Y) hit by balls, and a chain locked in its plane, hanging from a body which has all its
// axes locked
func lockedBodies() map[string]metric {
	const links, length = 12, 0.3
	w := world(1)
	ground(w, 0.6)
	r := rand.New(rand.NewSource(5))
	var flat, upright, chain []*actor.RigidBody
	for i := 0; i < 90; i++ {
		position := mgl64.Vec3{float64(i%9)*0.7 + 0.2*r.Float64(), 0.5 + float64(i/9)*0.7, 0}
		b := body(w, tr(position, mgl64.QuatRotate(r.Float64()*6, mgl64.Vec3{0, 0, 1})), mixedShape(r, i), actor.BodyTypeDynamic, 0.6, 0)
		b.LinearLock, b.AngularLock = actor.AxisZ, actor.AxisX|actor.AxisY
		flat = append(flat, b)
	}
	for i := 0; i < 10; i++ {
		x := float64(i) * 1.2
		b := body(w, tr(mgl64.Vec3{x, 0.9, 4}, mgl64.QuatIdent()), &actor.Capsule{HalfHeight: 0.6, Radius: 0.3}, actor.BodyTypeDynamic, 0.6, 0)
		b.AngularLock = actor.AxisX | actor.AxisZ
		upright = append(upright, b)
		ball := body(w, tr(mgl64.Vec3{x + 0.1, 1.5, 7 + 0.3*float64(i)}, mgl64.QuatIdent()), &actor.Sphere{Radius: 0.2}, actor.BodyTypeDynamic, 0.6, 0)
		ball.Velocity = mgl64.Vec3{0, 2, -12}
	}
	previous := body(w, tr(mgl64.Vec3{0, 6, 9}, mgl64.QuatIdent()), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 0.1, 0.1}}, actor.BodyTypeDynamic, 0.5, 0)
	previous.LinearLock, previous.AngularLock = actor.AllAxes, actor.AllAxes
	turned := mgl64.QuatRotate(math.Pi/2, mgl64.Vec3{0, 0, 1})
	for i := 0; i < links; i++ {
		center := mgl64.Vec3{-(float64(i) + 0.5) * length, 6, 9}
		link := body(w, tr(center, turned), &actor.Capsule{HalfHeight: length/2 - 0.05, Radius: 0.05}, actor.BodyTypeDynamic, 0.5, 0)
		link.LinearLock, link.AngularLock = actor.AxisZ, actor.AxisX|actor.AxisY
		w.AddJoint(feather.NewBallJoint(previous, link, center.Add(mgl64.Vec3{length / 2, 0, 0}), mgl64.Vec3{1, 0, 0}))
		chain = append(chain, link)
		previous = link
	}

	start := positions(w)
	drift, lean, stretch := 0.0, 0.0, 0.0
	play(w, 4, func() {
		for i, b := range w.Bodies {
			for k := 0; k < 3; k++ {
				if b.LinearLock.Has(k) {
					drift = math.Max(drift, math.Abs(b.Transform.Position[k]-start[i][k]))
				}
			}
		}
		for _, b := range upright {
			lean = math.Max(lean, angle(mgl64.QuatBetweenVectors(mgl64.Vec3{0, 1, 0}, b.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}))))
		}
		for i := 0; i+1 < len(chain); i++ {
			end := chain[i].Transform.ToWorld(mgl64.Vec3{0, length / 2, 0})
			next := chain[i+1].Transform.ToWorld(mgl64.Vec3{0, -length / 2, 0})
			stretch = math.Max(stretch, end.Sub(next).Len())
		}
	})
	pushed := 0.0
	for i, b := range upright {
		pushed = math.Max(pushed, b.Transform.Position.Sub(mgl64.Vec3{float64(i) * 1.2, 0.9, 4}).Len())
	}
	turn := 0.0
	for _, b := range flat {
		turn = math.Max(turn, math.Hypot(b.Transform.Rotation.V[0], b.Transform.Rotation.V[1]))
	}
	return map[string]metric{
		"worst locked drift":   {drift * 1000, "mm"},
		"worst lean":           {lean, ""},
		"worst turn off plane": {turn, ""},
		"worst stretch":        {stretch * 1000, "mm"},
		"least push":           {-pushed, "m"},
	}
}

// hills: random hills in a bowl, 48x48 samples every 0.5 m (the terrain of the tests)
func hills(w *feather.World, seed int64) *actor.RigidBody {
	const samples = 48
	r := rand.New(rand.NewSource(seed))
	heights := make([]float32, samples*samples)
	phases := [4]float64{r.Float64() * 6, r.Float64() * 6, r.Float64() * 6, r.Float64() * 6}
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			bowlX, bowlZ := (float64(x)-(samples-1)/2.0)*0.5, (float64(z)-(samples-1)/2.0)*0.5
			heights[x*samples+z] = float32(0.03*(bowlX*bowlX+bowlZ*bowlZ) + 0.6*math.Sin(float64(x)*0.35+phases[0])*math.Cos(float64(z)*0.3+phases[1]) +
				0.3*math.Sin(float64(x+z)*0.8+phases[2]) + 0.05*math.Cos(float64(x-z)*1.7+phases[3]))
		}
	}
	field := actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{0.5, 1, 0.5})
	return body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), field, actor.BodyTypeStatic, 0.6, 0)
}

func positions(w *feather.World) []mgl64.Vec3 {
	result := make([]mgl64.Vec3, len(w.Bodies))
	for i, b := range w.Bodies {
		result[i] = b.Transform.Position
	}
	return result
}

// ========== MEASURES ==========

// deepest: the deepest dynamic body in the plane or the terrain (m)
func deepest(w *feather.World, surface *actor.RigidBody) float64 {
	worst := 0.0
	for _, b := range w.Bodies {
		if b.BodyType != actor.BodyTypeDynamic {
			continue
		}
		switch shape := surface.Shape.(type) {
		case *actor.Plane:
			lowest := b.SupportWorld(shape.Normal.Mul(-1))
			worst = math.Max(worst, -(lowest.Dot(shape.Normal) + shape.Distance))
		case *actor.Heightfield:
			worst = math.Max(worst, underTerrain(shape, b))
		}
	}
	return worst
}

// underTerrain: the deepest point of the body under the terrain (identity transform), 0 above it
func underTerrain(field *actor.Heightfield, b *actor.RigidBody) float64 {
	depth := 0.0
	switch shape := b.Shape.(type) {
	case *actor.Box:
		for c := 0; c < 8; c++ {
			corner := shape.HalfExtents
			for k := 0; k < 3; k++ {
				if c&(1<<k) != 0 {
					corner[k] = -corner[k]
				}
			}
			depth = math.Max(depth, sunk(field, b.Transform.ToWorld(corner), 0))
		}
		depth = math.Max(depth, terrainInBox(field, b, shape))
	case *actor.Sphere:
		depth = sunk(field, b.Transform.Position, shape.Radius)
	case *actor.Capsule:
		bottom, top := shape.Segment(b.Transform)
		for k := 0; k <= 50; k++ {
			depth = math.Max(depth, sunk(field, bottom.Add(top.Sub(bottom).Mul(float64(k)/50)), shape.Radius))
		}
	}
	return depth
}

// terrainInBox: how deep the terrain is in the box (identity transform of the terrain), 0 if it stays out. The corners
// of a box can all be above the terrain while a ridge or a peak is in one of its faces. A point in the box is as deep
// as its distance to the closest face; the terrain is made of planes, its deepest point in a face of the box is on an
// edge of a triangle: the edges of the triangles under the box are followed, each from end to end
func terrainInBox(field *actor.Heightfield, b *actor.RigidBody, box *actor.Box) float64 {
	depth := 0.0
	cellsZ := field.ZSamples - 1
	for _, cell := range field.OverlapCells(b.AABB(), nil) {
		for t := 0; t < 2; t++ {
			triangle, _ := field.Triangle(int(cell)/cellsZ, int(cell)%cellsZ, t)
			for e := 0; e < 3; e++ {
				start, end := b.Transform.ToLocal(triangle[e]), b.Transform.ToLocal(triangle[(e+1)%3])
				depth = math.Max(depth, segmentInBox(start, end, box.HalfExtents))
			}
		}
	}
	return depth
}

// segmentInBox: the deepest point of the segment in the box (in its local space): the highest value, over the segment,
// of the distance to the closest face, 0 if the segment stays out. Along the segment the distance to each of the 6
// faces is linear; the closest face changes where 2 of these distances are equal: the deepest point is at an end, or
// at one of these crossings
func segmentInBox(start, end, halfExtents mgl64.Vec3) float64 {
	// the distance to the face i at the fraction f of the segment: at[i] + f*along[i]
	var at, along [6]float64
	for k := 0; k < 3; k++ {
		at[2*k], along[2*k] = halfExtents[k]-start[k], start[k]-end[k]
		at[2*k+1], along[2*k+1] = halfExtents[k]+start[k], end[k]-start[k]
	}
	closest := func(fraction float64) float64 {
		distance := math.Inf(1)
		for i := range at {
			distance = math.Min(distance, at[i]+fraction*along[i])
		}
		return distance
	}
	depth := math.Max(0, math.Max(closest(0), closest(1)))
	for i := range at {
		for j := i + 1; j < len(at); j++ {
			if along[i] == along[j] {
				continue
			}
			if fraction := (at[j] - at[i]) / (along[i] - along[j]); fraction > 0 && fraction < 1 {
				depth = math.Max(depth, closest(fraction))
			}
		}
	}
	return depth
}

// sunkReach: a point under the terrain is measured against the triangles 1 m around it at most
const sunkReach = 1.0

// sunk: how deep the ball around the point is under the terrain: its radius minus the distance from the point to the
// triangles, counted negative under the terrain; 0 if the ball is above. Only the triangles which can be the closest
// are measured: the ones within the radius of a point above the terrain (further, the ball doesn't touch them), the
// ones as close as the terrain straight above a point under it
func sunk(field *actor.Heightfield, p mgl64.Vec3, radius float64) float64 {
	reach := radius
	height, found := field.HeightAt(p.X(), p.Z())
	under := found && p.Y() < height
	if under {
		reach = math.Min(height-p.Y(), sunkReach)
	}
	if reach == 0 {
		return 0
	}
	closest := math.Inf(1)
	around := mgl64.Vec3{reach, 100, reach}
	for _, cell := range field.OverlapCells(actor.AABB{Min: p.Sub(around), Max: p.Add(around)}, nil) {
		x, z := int(cell)/(field.ZSamples-1), int(cell)%(field.ZSamples-1)
		for t := 0; t < 2; t++ {
			triangle, _ := field.Triangle(x, z, t)
			closest = math.Min(closest, p.Sub(closestOnTriangle(p, triangle)).Len())
		}
	}
	if under {
		return radius + closest
	}
	return math.Max(0, radius-closest)
}

// closestOnTriangle: the closest point of the triangle (Ericson 5.1.5)
func closestOnTriangle(p mgl64.Vec3, triangle [3]mgl64.Vec3) mgl64.Vec3 {
	a, b, c := triangle[0], triangle[1], triangle[2]
	ab, ac, ap := b.Sub(a), c.Sub(a), p.Sub(a)
	d1, d2 := ab.Dot(ap), ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return a
	}
	bp := p.Sub(b)
	d3, d4 := ab.Dot(bp), ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return b
	}
	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		return a.Add(ab.Mul(d1 / (d1 - d3)))
	}
	cp := p.Sub(c)
	d5, d6 := ab.Dot(cp), ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return c
	}
	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		return a.Add(ac.Mul(d2 / (d2 - d6)))
	}
	va := d3*d6 - d5*d4
	if va <= 0 && d4-d3 >= 0 && d5-d6 >= 0 {
		return b.Add(c.Sub(b).Mul((d4 - d3) / ((d4 - d3) + (d5 - d6))))
	}
	denominator := 1 / (va + vb + vc)
	return a.Add(ab.Mul(vb * denominator)).Add(ac.Mul(vc * denominator))
}

// ========== DRAWS ==========
// Some measures are decided by the rounding: a contact at the separation 0 is solved as a spring or as a speculative
// row, 2 impacts come in one order or the other, and the pile, the ball of spheres or the net ends elsewhere. Pushing
// every body of the scene by 1 µm/s moves such a measure by more than its tolerance: any change of the engine draws it
// again, and one value of it says nothing (rush: 0.12 to 0.91 m/s). The measures which are a draw are named by their
// scene (regressionScene.draws, scenes.Scene.Draws); a scene which only keeps a symmetry (circle stack, warm start
// energy, stretched chain, high mass ratio 2) is not one, the push breaks what it measures: its measures stay smooth
// across a finite length of the scene.
//
// A draw is followed over an ensemble: drawVariants variants of its scene, each differing by a finite length of the
// scene (the seed of a pile, a drop height, a radius: a variant is not the scene pushed by a rounding). The reference
// holds:
//   - the median of the variants;
//   - its tolerance: how far over it the median of another draw of the same ensemble may lie. A change of the engine
//     which keeps the quality draws every variant again: its median falls among the sorted variants of the reference
//     at the rank n/2 ± √(n/2) (a variant is under the true median with a chance of 1/2, in both ensembles). The
//     distance from the median to the variant 2 √(n/2) ranks over it (8 ranks out of 32: the upper quartile) is 2
//     standard deviations of the median: the tolerance is drawSigmas of them. It is measured in the middle of the
//     ensemble and assumes no law: the tail of a draw (a pile out of 8 with a body sunk by centimetres) doesn't widen
//     it, 2 modes do. It is never under the tolerance of the unit;
//   - the worst variant: to be read, it is not compared (the tail of an ensemble is a draw of a few variants).
//
// 2 draws of the same law are told apart less than 1 time out of 100, a law moved by twice its spread (the distance
// between its quartiles) is seen 8 times out of 10 at least (TestDrawToleranceTellsADrawFromARegression). The variants
// are not timed: they run along each other, after the timed scenes.

const (
	// drawVariants: the variants of an ensemble
	drawVariants = 32

	// drawSigmas: the median of a draw further than 3 of its standard deviations is a regression
	drawSigmas = 3.0

	// drawRankSigmas: the standard deviation of the median is measured over 2 standard deviations of its rank
	drawRankSigmas = 2.0
)

// drawOf: the median of the variants, its tolerance and the worst variant
func drawOf(variants []float64, unit string) draw {
	sorted := slices.Clone(variants)
	slices.Sort(sorted)
	n := len(sorted)
	median := (sorted[(n-1)/2] + sorted[n/2]) / 2
	over := min(n-1, n/2+int(math.Ceil(drawRankSigmas*math.Sqrt(float64(n)/2))))
	return draw{Median: median, Unit: unit, Variants: n, Tolerance: drawSigmas / drawRankSigmas * (sorted[over] - median), Worst: sorted[n-1]}
}

// tolerance of the draw: the one of its ensemble, the one of its unit at least
func (d draw) tolerance() float64 {
	return math.Max(d.Tolerance, qualityTolerances[d.Unit])
}

// measureDraws runs the variants of the scene along each other and gives its draws. The same scene gives the same bits:
// a variant doesn't see the others
func measureDraws(scene regressionScene) map[string]draw {
	results := make([]map[string]metric, drawVariants)
	var running sync.WaitGroup
	slots := make(chan struct{}, runtime.GOMAXPROCS(0))
	for variant := range results {
		running.Add(1)
		go func() {
			defer running.Done()
			slots <- struct{}{}
			defer func() { <-slots }()
			var worlds []*feather.World
			results[variant] = scene.variant(variant, func(w *feather.World, seconds float64, each func()) {
				if !slices.Contains(worlds, w) {
					worlds = append(worlds, w)
				}
				scenes.Step(w, seconds, each)
			})
			for _, w := range worlds {
				w.Close()
			}
		}()
	}
	running.Wait()
	draws := map[string]draw{}
	for _, name := range scene.draws {
		values := make([]float64, drawVariants)
		for variant, result := range results {
			values[variant] = result[name].Value
		}
		draws[name] = drawOf(values, results[0][name].Unit)
	}
	return draws
}

// ========== RUN & COMPARE ==========

// measureScene runs the scene speedRuns times: the quality & the fingerprint of the first run (all runs give the same
// bits), without its draws (measureDraws), the speed of the step and of each phase from its best run. The runs start
// at stackOffset in the page of the stack, then a third of a page further each: a phase slow at a depth of the stack
// is not at the next one (stack.go)
func measureScene(scene regressionScene, stackOffset int) sceneResult {
	var result sceneResult
	for attempt := 0; attempt < speedRuns; attempt++ {
		recording = &recorder{measuring: attempt == 0, phases: map[string]time.Duration{}}
		var quality map[string]metric
		atStackOffset(stackOffset+attempt*pageSize/speedRuns, func() { quality = scene.run() })
		if attempt == 0 {
			hash := fnv.New64a()
			for _, w := range recording.worlds {
				for _, b := range w.Bodies {
					p, q := b.Transform.Position, b.Transform.Rotation
					for _, x := range []float64{p[0], p[1], p[2], q.W, q.V[0], q.V[1], q.V[2]} {
						_ = binary.Write(hash, binary.LittleEndian, math.Float64bits(x))
					}
				}
			}
			for _, x := range recording.results {
				_ = binary.Write(hash, binary.LittleEndian, x)
			}
			result.Fingerprint = fmt.Sprintf("%016x", hash.Sum64())
			for _, name := range scene.draws {
				delete(quality, name)
			}
			result.Quality = quality
		}
		stepMs := milliseconds(recording.step) / float64(recording.steps)
		if attempt == 0 {
			result.StepMs = stepMs
			result.PhasesMs = map[string]float64{}
		}
		result.StepMs = min(result.StepMs, stepMs)
		for name, phase := range recording.phases {
			phaseMs := milliseconds(phase) / float64(recording.steps)
			if best, found := result.PhasesMs[name]; !found || phaseMs < best {
				result.PhasesMs[name] = phaseMs
			}
		}
		for _, w := range recording.worlds {
			w.Close()
		}
	}
	return result
}

// machine: the CPU, for the speed
func machine() string {
	model := "unknown CPU"
	if file, err := os.Open("/proc/cpuinfo"); err == nil {
		defer func() { _ = file.Close() }()
		scanner := bufio.NewScanner(file)
		for scanner.Scan() {
			if name, found := strings.CutPrefix(scanner.Text(), "model name"); found {
				model = strings.TrimSpace(strings.TrimPrefix(strings.TrimSpace(name), ":"))
				break
			}
		}
	}
	return fmt.Sprintf("%s, %d CPUs", model, runtime.NumCPU())
}

// regressions runs the scenes, then writes the reference (update) or compares to it. Returns false on a regression.
// stackOffset moves the stack of the scenes in its page (-stack): the speed must not depend on it
func regressions(update bool, stackOffset int) bool {
	current := baseline{Arch: runtime.GOOS + "/" + runtime.GOARCH, Machine: machine(), Scenes: map[string]sceneResult{}}
	for _, scene := range regressionScenes {
		current.Scenes[scene.name] = measureScene(scene, stackOffset)
		fmt.Printf("%-16s %s  %.3f ms/step\n", scene.name, current.Scenes[scene.name].Fingerprint, current.Scenes[scene.name].StepMs)
	}
	for _, scene := range regressionScenes {
		if scene.variant == nil {
			continue
		}
		result := current.Scenes[scene.name]
		result.Draws = measureDraws(scene)
		current.Scenes[scene.name] = result
		for _, name := range sortedKeys(result.Draws) {
			d := result.Draws[name]
			fmt.Printf("%-16s %s: median %.3f %s of %d variants (tolerance %.3f, worst %.3f)\n", scene.name, name, d.Median, d.Unit, d.Variants, d.tolerance(), d.Worst)
		}
	}
	if update {
		data, err := json.MarshalIndent(current, "", "  ")
		if err == nil {
			err = os.WriteFile(baselineFile, append(data, '\n'), 0o644)
		}
		if err != nil {
			fmt.Println("cannot write the reference:", err)
			return false
		}
		fmt.Println("reference written in", baselineFile)
		return true
	}

	var reference baseline
	data, err := os.ReadFile(baselineFile)
	if err == nil {
		err = json.Unmarshal(data, &reference)
	}
	if err != nil {
		fmt.Println("cannot read the reference:", err)
		return false
	}
	return compare(reference, current)
}

// compare the scenes to the reference, prints every difference. Returns false on a regression
func compare(reference, current baseline) bool {
	ok := true
	fail := func(format string, args ...any) {
		fmt.Printf("REGRESSION "+format+"\n", args...)
		ok = false
	}
	sameArch, sameMachine := reference.Arch == current.Arch, reference.Arch == current.Arch && reference.Machine == current.Machine
	if !sameMachine {
		fmt.Printf("the reference comes from %s (%s): the speed is not compared\n", reference.Machine, reference.Arch)
	}
	for _, scene := range regressionScenes {
		want, found := reference.Scenes[scene.name]
		if !found {
			fail("%s: not in the reference (-update)", scene.name)
			continue
		}
		got := current.Scenes[scene.name]
		if sameArch && got.Fingerprint != want.Fingerprint {
			fail("%s: fingerprint %s, the reference is %s (the behaviour changed: -update if it is wanted)", scene.name, got.Fingerprint, want.Fingerprint)
		}
		for _, name := range sortedKeys(want.Quality) {
			before, after := want.Quality[name], got.Quality[name]
			tolerance := qualityTolerances[before.Unit]
			switch _, measured := got.Quality[name]; {
			case !measured:
				fail("%s: %s is in the reference, and not measured anymore (-update)", scene.name, name)
			case after.Value > before.Value+tolerance:
				fail("%s: %s %.3f %s, the reference is %.3f (tolerance %g)", scene.name, name, after.Value, before.Unit, before.Value, tolerance)
			case after.Value < before.Value-tolerance:
				fmt.Printf("better     %s: %s %.3f %s, the reference is %.3f\n", scene.name, name, after.Value, before.Unit, before.Value)
			}
		}
		for _, name := range sortedKeys(got.Draws) {
			before, after := want.Draws[name], got.Draws[name]
			tolerance := before.tolerance()
			switch _, drawn := want.Draws[name]; {
			case !drawn || before.Variants != after.Variants:
				fail("%s: %s over %d variants is not in the reference (-update)", scene.name, name, after.Variants)
			case after.Median > before.Median+tolerance:
				fail("%s: %s median %.3f %s of %d variants, the reference is %.3f (tolerance %.3f)", scene.name, name, after.Median, before.Unit, after.Variants, before.Median, tolerance)
			case after.Median < before.Median-tolerance:
				fmt.Printf("better     %s: %s median %.3f %s of %d variants, the reference is %.3f\n", scene.name, name, after.Median, before.Unit, after.Variants, before.Median)
			}
		}
		if !sameMachine || want.StepMs < minComparedStepMs {
			continue
		}
		if got.StepMs > want.StepMs*(1+stepTolerance) {
			fail("%s: %.3f ms per step, the reference is %.3f (tolerance %.0f %%)", scene.name, got.StepMs, want.StepMs, stepTolerance*100)
		}
		for _, name := range sortedKeys(want.PhasesMs) {
			before, after := want.PhasesMs[name], got.PhasesMs[name]
			if before >= phaseShare*want.StepMs && after > before*(1+phaseTolerance) {
				fail("%s: %s %.3f ms per step, the reference is %.3f (tolerance %.0f %%)", scene.name, name, after, before, phaseTolerance*100)
			}
		}
	}
	if ok {
		fmt.Println("no regression")
	}
	return ok
}
