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
	"time"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
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
//   - its speed: the time of a step and of its phases (feather.Profile), the best of speedRuns runs. Only compared on
//     the machine of the reference: a step slower by more than stepTolerance, or a phase taking more than 5 % of the
//     step and slower by more than phaseTolerance, is a regression

const (
	baselineFile = "baseline.json"

	// speedRuns: the speed of a scene is its best run
	speedRuns = 3

	// stepTolerance: a step may be 20 % slower (the noise of a shared machine)
	stepTolerance = 0.20

	// phaseTolerance: a phase may be 30 % slower (shorter, noisier)
	phaseTolerance = 0.30

	// phaseShare: the phases under 5 % of the step are too short to be compared
	phaseShare = 0.05
)

// qualityTolerances: how much worse a quality metric may get, by unit
var qualityTolerances = map[string]float64{
	"mm": 0.5,  // a depth, a drift: 0.5 mm, a tenth of LinearSlop
	"%":  0.1,  // an energy gain, in % of the energy
	"":   1e-9, // a count
}

// metric: a quality measure of a scene
type metric struct {
	Value float64 `json:"value"`
	Unit  string  `json:"unit"`
}

// sceneResult: what a scene gives
type sceneResult struct {
	Fingerprint string            `json:"fingerprint"`
	Quality     map[string]metric `json:"quality"`
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
}

// recorder: the worlds of the scene being measured, and the time of their steps
type recorder struct {
	// measuring: the quality is measured (the first run), else the scene is only timed
	measuring bool
	worlds    []*feather.World
	step      time.Duration
	phases    map[string]time.Duration
	steps     int
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

var regressionScenes = []regressionScene{
	{"slope pile", slopePile},
	{"terrain piles", terrainPiles},
	{"pyramid", pyramidDrift},
	{"pile of 500", pile500},
	{"joint chain", jointChain},
	{"rain on terrain", rainOnTerrain},
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
func slopePile() map[string]metric {
	angle := 30 * math.Pi / 180
	normal := mgl64.Vec3{-math.Sin(angle), math.Cos(angle), 0}
	w := world(1)
	slope := body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), &actor.Plane{Normal: normal}, actor.BodyTypeStatic, 0.6, 0)
	r := rand.New(rand.NewSource(2))
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

// terrainPiles: 60 bodies dropped on hills, 10 times (a pile is chaotic: its median & its worst are measured)
func terrainPiles() map[string]metric {
	const piles = 10
	landings, restings := make([]float64, piles), make([]float64, piles)
	for pile := 0; pile < piles; pile++ {
		seed := int64(pile + 1)
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
		play(w, 4, func() { landings[pile] = math.Max(landings[pile], deepest(w, terrain)) })
		restings[pile] = deepest(w, terrain)
	}
	slices.Sort(landings)
	slices.Sort(restings)
	return map[string]metric{
		"median landing depth": {(landings[piles/2-1] + landings[piles/2]) / 2 * 1000, "mm"},
		"worst landing depth":  {landings[piles-1] * 1000, "mm"},
		"worst resting depth":  {restings[piles-1] * 1000, "mm"},
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
func rainOnTerrain() map[string]metric {
	w := world(1)
	terrain := hills(w, 7)
	field := terrain.Shape.(*actor.Heightfield)
	r := rand.New(rand.NewSource(8))
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
		if b.Transform.Position.Y() < -5 && recording.measuring {
			fell++
		}
	}
	return map[string]metric{
		"landing depth": {landing * 1000, "mm"},
		"fell through":  {float64(fell), ""},
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
			depth = math.Max(depth, -surfaceDistance(field, b.Transform.ToWorld(corner)))
		}
	case *actor.Sphere:
		depth = shape.Radius - surfaceDistance(field, b.Transform.Position)
	case *actor.Capsule:
		bottom, top := shape.Segment(b.Transform)
		for k := 0; k <= 50; k++ {
			depth = math.Max(depth, shape.Radius-surfaceDistance(field, bottom.Add(top.Sub(bottom).Mul(float64(k)/50))))
		}
	}
	return math.Max(0, depth)
}

// surfaceDistance: the distance from the point to the triangles, negative under the terrain
func surfaceDistance(field *actor.Heightfield, p mgl64.Vec3) float64 {
	best := math.Inf(1)
	around := mgl64.Vec3{1, 100, 1}
	for _, cell := range field.OverlapCells(actor.AABB{Min: p.Sub(around), Max: p.Add(around)}, nil) {
		x, z := int(cell)/(field.ZSamples-1), int(cell)%(field.ZSamples-1)
		for t := 0; t < 2; t++ {
			triangle, _ := field.Triangle(x, z, t)
			best = math.Min(best, p.Sub(closestOnTriangle(p, triangle)).Len())
		}
	}
	if height, ok := field.HeightAt(p.X(), p.Z()); ok && p.Y() < height {
		return -best
	}
	return best
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

// ========== RUN & COMPARE ==========

// measureScene runs the scene speedRuns times: the quality & the fingerprint of the first run (all runs give the same
// bits), the speed of the best run
func measureScene(scene regressionScene) sceneResult {
	var result sceneResult
	for attempt := 0; attempt < speedRuns; attempt++ {
		recording = &recorder{measuring: attempt == 0, phases: map[string]time.Duration{}}
		quality := scene.run()
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
			result.Fingerprint = fmt.Sprintf("%016x", hash.Sum64())
			result.Quality = quality
		}
		stepMs := milliseconds(recording.step) / float64(recording.steps)
		if attempt == 0 || stepMs < result.StepMs {
			result.StepMs = stepMs
			result.PhasesMs = map[string]float64{}
			for name, phase := range recording.phases {
				result.PhasesMs[name] = milliseconds(phase) / float64(recording.steps)
			}
		}
		for _, w := range recording.worlds {
			w.Close()
		}
	}
	return result
}

func milliseconds(d time.Duration) float64 {
	return float64(d.Nanoseconds()) / 1e6
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

// regressions runs the scenes, then writes the reference (update) or compares to it. Returns false on a regression
func regressions(update bool) bool {
	current := baseline{Arch: runtime.GOOS + "/" + runtime.GOARCH, Machine: machine(), Scenes: map[string]sceneResult{}}
	for _, scene := range regressionScenes {
		current.Scenes[scene.name] = measureScene(scene)
		fmt.Printf("%-16s %s  %.3f ms/step\n", scene.name, current.Scenes[scene.name].Fingerprint, current.Scenes[scene.name].StepMs)
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
			switch {
			case after.Value > before.Value+tolerance:
				fail("%s: %s %.3f %s, the reference is %.3f (tolerance %g)", scene.name, name, after.Value, before.Unit, before.Value, tolerance)
			case after.Value < before.Value-tolerance:
				fmt.Printf("better     %s: %s %.3f %s, the reference is %.3f\n", scene.name, name, after.Value, before.Unit, before.Value)
			}
		}
		if !sameMachine {
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

func sortedKeys[V any](m map[string]V) []string {
	keys := make([]string, 0, len(m))
	for key := range m {
		keys = append(keys, key)
	}
	slices.Sort(keys)
	return keys
}
