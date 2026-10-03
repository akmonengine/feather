//go:build !v020

package main

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// The draw of an ensemble: its median, its worst variant, and a tolerance of 3 standard deviations of the median,
// measured from the median to the variant 2 standard deviations of rank over it (8 ranks out of 32)
func TestDrawOf(t *testing.T) {
	variants := make([]float64, 32)
	for i := range variants {
		// 0, 10, 20... in another order
		variants[i] = float64((i*7)%32) * 10
	}
	got := drawOf(variants, "mm")
	if got.Median != 155 || got.Worst != 310 || got.Variants != 32 || got.Unit != "mm" {
		t.Errorf("median %g, worst %g, %d variants in %q, want 155, 310, 32 in mm", got.Median, got.Worst, got.Variants, got.Unit)
	}
	// the variant of rank 16 + 8 is 240: 85 over the median, for 2 standard deviations
	if want := 1.5 * 85; got.Tolerance != want {
		t.Errorf("tolerance %g, want %g", got.Tolerance, want)
	}

	// the tail doesn't widen the tolerance: the 7 worst variants thrown far
	for i, v := range variants {
		if v >= 250 {
			variants[i] = v * 1000
		}
	}
	if tailed := drawOf(variants, "mm"); tailed.Tolerance != got.Tolerance || tailed.Median != got.Median || tailed.Worst != 310000 {
		t.Errorf("with a tail: median %g, tolerance %g, worst %g, want %g, %g, 310000", tailed.Median, tailed.Tolerance, tailed.Worst, got.Median, got.Tolerance)
	}

	// an odd ensemble: its median is its middle variant
	if odd := drawOf([]float64{5, 1, 3}, "s"); odd.Median != 3 || odd.Worst != 5 {
		t.Errorf("3 variants: median %g, worst %g, want 3 and 5", odd.Median, odd.Worst)
	}

	// the tolerance of a draw is never under the one of its unit
	tight := drawOf(slices.Repeat([]float64{7}, 32), "mm")
	if tight.Tolerance != 0 || tight.tolerance() != qualityTolerances["mm"] {
		t.Errorf("32 equal variants: tolerance %g of the ensemble, %g compared, want 0 and %g", tight.Tolerance, tight.tolerance(), qualityTolerances["mm"])
	}
}

// 2 draws of the same law are within the tolerance, whatever the law (a tail, 2 modes, steps): told apart less than 1
// time out of 100. A law moved by twice its spread is out of it, 8 times out of 10 at least
func TestDrawToleranceTellsADrawFromARegression(t *testing.T) {
	const trials = 4000
	r := rand.New(rand.NewSource(1))
	laws := []struct {
		name string
		draw func() float64
		// spread: the distance between the quartiles of the law
		spread float64
	}{
		{"normal", r.NormFloat64, 1.349},
		{"long tail", func() float64 { return math.Exp(r.NormFloat64()) }, 1.453},
		{"a variant out of 7 sunk", func() float64 {
			if r.Float64() < 0.15 {
				return 7 + 17*r.Float64()
			}
			return 0.4 + 0.05*r.NormFloat64()
		}, 0.072},
		{"2 modes", func() float64 {
			if r.Float64() < 0.5 {
				return 0.01 * r.NormFloat64()
			}
			return 2 + 3*r.Float64()
		}, 3.5},
		{"steps", func() float64 { return math.Round(2*r.NormFloat64()) / 60 }, 3.0 / 60},
	}
	ensemble := func(draw func() float64, moved float64) []float64 {
		variants := make([]float64, drawVariants)
		for i := range variants {
			variants[i] = draw() + moved
		}
		return variants
	}
	for _, law := range laws {
		alarms, seen := 0, 0
		for trial := 0; trial < trials; trial++ {
			reference := drawOf(ensemble(law.draw, 0), "")
			if drawOf(ensemble(law.draw, 0), "").Median > reference.Median+reference.Tolerance {
				alarms++
			}
			if drawOf(ensemble(law.draw, 2*law.spread), "").Median > reference.Median+reference.Tolerance {
				seen++
			}
		}
		t.Logf("%s: %d false alarms, %d regressions seen, out of %d", law.name, alarms, seen, trials)
		if alarms > trials/100 {
			t.Errorf("%s: 2 draws of the same law told apart %d times out of %d, want 1 %% at most", law.name, alarms, trials)
		}
		if seen < trials*8/10 {
			t.Errorf("%s: the law moved by twice its spread seen %d times out of %d, want 8 out of 10", law.name, seen, trials)
		}
	}
}

// The draws of a scene are measured over its variants, not by its run; the variants run along each other and give the
// same bits at each run
func TestDrawsAreMeasuredOverTheVariants(t *testing.T) {
	drawn := 0
	for _, scene := range regressionScenes {
		if (scene.variant == nil) != (len(scene.draws) == 0) {
			t.Errorf("%s: %d draws, variants %v", scene.name, len(scene.draws), scene.variant != nil)
		}
		drawn += len(scene.draws)
	}
	if drawn == 0 {
		t.Fatal("no scene has a draw")
	}
	for _, scene := range regressionScenes {
		if scene.name != "slope pile" && scene.name != "solver2d double domino" {
			continue
		}
		for _, name := range scene.draws {
			if _, found := measureScene(scene, 0).Quality[name]; found {
				t.Errorf("%s: the draw %q is also measured by the run of the scene", scene.name, name)
			}
		}
		first, second := measureDraws(scene), measureDraws(scene)
		if len(first) != len(scene.draws) {
			t.Errorf("%s: %d draws measured, want %d", scene.name, len(first), len(scene.draws))
		}
		for _, name := range scene.draws {
			if first[name] != second[name] {
				t.Errorf("%s: %s is %+v, then %+v", scene.name, name, first[name], second[name])
			}
			if first[name].Variants != drawVariants || first[name].Worst < first[name].Median || first[name].Unit == "" {
				t.Errorf("%s: %s is %+v", scene.name, name, first[name])
			}
		}
	}
}

// A draw is a regression over the tolerance of its reference only, and must be in the reference with its variants
func TestCompareDraws(t *testing.T) {
	scenesWith := func(d draw) baseline {
		b := baseline{Arch: "test", Machine: "test", Scenes: map[string]sceneResult{}}
		for _, scene := range regressionScenes {
			b.Scenes[scene.name] = sceneResult{}
		}
		b.Scenes["slope pile"] = sceneResult{Draws: map[string]draw{"resting depth": d}}
		return b
	}
	reference := scenesWith(draw{Median: 2, Unit: "mm", Variants: 32, Tolerance: 1.2, Worst: 9})
	for _, c := range []struct {
		name string
		got  draw
		ok   bool
	}{
		{"the same draw", draw{Median: 2, Unit: "mm", Variants: 32, Tolerance: 1.2, Worst: 9}, true},
		{"another draw, within the tolerance", draw{Median: 3.1, Unit: "mm", Variants: 32, Tolerance: 0.6, Worst: 40}, true},
		{"over the tolerance", draw{Median: 3.3, Unit: "mm", Variants: 32, Tolerance: 5, Worst: 9}, false},
		{"better", draw{Median: 0.1, Unit: "mm", Variants: 32, Tolerance: 0.1, Worst: 1}, true},
		{"other variants", draw{Median: 2, Unit: "mm", Variants: 16, Tolerance: 1.2, Worst: 9}, false},
	} {
		if ok := compare(reference, scenesWith(c.got)); ok != c.ok {
			t.Errorf("%s: compared %v, want %v", c.name, ok, c.ok)
		}
	}
	// a tolerance of the ensemble under the one of the unit: the unit counts
	tight := scenesWith(draw{Median: 2, Unit: "mm", Variants: 32, Tolerance: 0.01, Worst: 2})
	if !compare(tight, scenesWith(draw{Median: 2.4, Unit: "mm", Variants: 32})) || compare(tight, scenesWith(draw{Median: 2.6, Unit: "mm", Variants: 32})) {
		t.Error("a draw 0.4 mm over its reference must pass, 0.6 mm over it must not (0.5 mm for a depth)")
	}
	// a draw missing in the reference, a measure of the reference which is not measured anymore
	old := scenesWith(draw{})
	old.Scenes["slope pile"] = sceneResult{Quality: map[string]metric{"resting depth": {2, "mm"}}}
	if compare(old, reference) {
		t.Error("the reference holds one value of a measure which is now a draw: want a regression")
	}
}

// ridgeField: a ridge along Z at x = 0 and y = 0, both slopes going down by the angle, 9x9 samples every 0.5 m
func ridgeField(angle float64) *actor.Heightfield {
	const samples = 9
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32(-math.Abs(float64(x)-4) * 0.5 * math.Tan(angle))
		}
	}
	return actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{0.5, 1, 0.5})
}

// The depth of a box in a terrain counts the terrain in its faces: a ridge 10 mm in the bottom face of a box, its 8
// corners above the slopes. Between 2 rows of samples no sample is in the box: the ridge is an edge of the triangles
func TestUnderTerrainSeesTheTerrainInTheFacesOfABox(t *testing.T) {
	field := ridgeField(10 * math.Pi / 180)
	box := &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.15, 0.15}}
	for _, c := range []struct {
		name     string
		position mgl64.Vec3
		rotation mgl64.Quat
		want     float64
	}{
		{"across the ridge, between 2 rows of samples", mgl64.Vec3{0, 0.14, 0.25}, mgl64.QuatIdent(), 0.010},
		{"turned around the vertical", mgl64.Vec3{0, 0.14, 0.25}, mgl64.QuatRotate(0.5, mgl64.Vec3{0, 1, 0}), 0.010},
		{"over a sample of the ridge", mgl64.Vec3{0, 0.147, 0.5}, mgl64.QuatIdent(), 0.003},
		{"above the ridge", mgl64.Vec3{0, 0.151, 0.25}, mgl64.QuatIdent(), 0},
		{"a corner under a slope, deeper than the ridge in the face", mgl64.Vec3{1, 0.15 - math.Tan(10*math.Pi/180) - 0.02, 0.25}, mgl64.QuatIdent(), 0},
	} {
		b := actor.NewRigidBody(actor.Transform{Position: c.position, Rotation: c.rotation}, box, actor.BodyTypeDynamic, 500)
		got := underTerrain(field, b)
		if c.want == 0 && c.name != "above the ridge" {
			// the corners: their distance to the slope, as before
			corners := 0.0
			for k := 0; k < 8; k++ {
				corner := box.HalfExtents
				for axis := 0; axis < 3; axis++ {
					if k&(1<<axis) != 0 {
						corner[axis] = -corner[axis]
					}
				}
				corners = math.Max(corners, sunk(field, b.Transform.ToWorld(corner), 0))
			}
			if corners < 0.02 || got < corners {
				t.Errorf("%s: %.3f mm, the corners alone %.3f mm", c.name, got*1000, corners*1000)
			}
			continue
		}
		if math.Abs(got-c.want) > 1e-6 {
			t.Errorf("%s: the terrain is %.4f mm in the box, want %.4f", c.name, got*1000, c.want*1000)
		}
	}
}

// The deepest point of a segment in a box: at an end, or where the closest face changes
func TestSegmentInBox(t *testing.T) {
	half := mgl64.Vec3{1, 1, 1}
	for _, c := range []struct {
		name       string
		start, end mgl64.Vec3
		want       float64
	}{
		{"through 2 opposite faces, along a face at 0.5", mgl64.Vec3{-2, 0.5, 0}, mgl64.Vec3{2, 0.5, 0}, 0.5},
		{"from a corner edge to the opposite one, through the center", mgl64.Vec3{-1, 1, 0}, mgl64.Vec3{1, -1, 0}, 1},
		{"out of the box", mgl64.Vec3{-2, 2, 0}, mgl64.Vec3{2, 2, 0}, 0},
		{"an end in the box", mgl64.Vec3{0.2, 0.3, 0.6}, mgl64.Vec3{3, 0.3, 0.6}, 0.4},
		{"a point", mgl64.Vec3{0.2, 0.3, 0.6}, mgl64.Vec3{0.2, 0.3, 0.6}, 0.4},
		{"across a corner", mgl64.Vec3{0.8, 2, 0}, mgl64.Vec3{2, 0.8, 0}, 0},
		{"cutting a corner", mgl64.Vec3{0.4, 1.2, 0}, mgl64.Vec3{1.2, 0.4, 0}, 0.2},
	} {
		if got := segmentInBox(c.start, c.end, half); math.Abs(got-c.want) > 1e-12 {
			t.Errorf("%s: %.6f, want %.6f", c.name, got, c.want)
		}
	}
}
