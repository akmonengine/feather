//go:build !v020

package main

import (
	"fmt"
	"math"
	"math/rand"
	"sync"
	"time"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== QUERIES ==========
// The scenes of the queries (rays, sweeps, overlaps) are regression scenes as the others: their fingerprint is the
// hash of their results, a "step" is a round of all their batches of queries, a "phase" the time of a batch.
//
//	go run . -queries     # the cost of a query of each kind (ns), and the queries per second of 8 goroutines

const (
	// queryBatch: queries per batch
	queryBatch = 10000

	// queryRounds: rounds of all the batches per run of a scene
	queryRounds = 3

	// longRays: the rays of 1000 m across the terrain, per batch
	longRays = 1000

	// syncRounds: calls of SyncQueries per batch
	syncRounds = 100

	// queryGoroutines of the throughput measure
	queryGoroutines = 8
)

// queryCounts: the queries of the batch of each phase, for the cost of a query
var queryCounts = map[string]int{}

// batch runs count queries, records their time as the phase name, and their results in the fingerprint of the scene
func batch(name string, count int, query func(i int) uint64) {
	queryCounts[name] = count
	sum := uint64(0)
	start := time.Now()
	for i := 0; i < count; i++ {
		sum = sum*31 + query(i)
	}
	elapsed := time.Since(start)
	recording.phases[name] += elapsed
	recording.step += elapsed
	recording.results = append(recording.results, sum)
}

// folded: a hit as a number, for the fingerprint: its body, its fraction, its point, its normal and its triangle
func folded(w *feather.World, hit feather.Hit, ok bool) uint64 {
	if !ok {
		return 1
	}
	sum := uint64(hit.Body.Serial()-w.Bodies[0].Serial()) + math.Float64bits(hit.Fraction) + uint64(hit.Triangle+1)
	for k := 0; k < 3; k++ {
		sum = sum*31 + math.Float64bits(hit.Point[k]) + math.Float64bits(hit.Normal[k])
	}
	return sum
}

// scattered: 1000 bodies (500 boxes, 500 spheres of 0.5 m) at rest on a plane of 100 x 100 m, static or dynamic by
// halves, and 10000 points 1 m over the ground with a level direction each
func scattered() (*feather.World, []mgl64.Vec3, []mgl64.Vec3) {
	w := world(1)
	ground(w, 0.6)
	r := rand.New(rand.NewSource(1))
	for i := 0; i < 1000; i++ {
		var shape actor.ShapeInterface = &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}
		if i%2 == 1 {
			shape = &actor.Sphere{Radius: 0.25}
		}
		bodyType := actor.BodyTypeStatic
		if i%4 < 2 {
			bodyType = actor.BodyTypeDynamic
		}
		body(w, tr(mgl64.Vec3{r.Float64()*100 - 50, 0.25, r.Float64()*100 - 50}, mgl64.QuatIdent()), shape, bodyType, 0.6, 0)
	}
	origins, directions := make([]mgl64.Vec3, queryBatch), make([]mgl64.Vec3, queryBatch)
	for i := range origins {
		origins[i] = mgl64.Vec3{r.Float64()*100 - 50, 1, r.Float64()*100 - 50}
		angle := r.Float64() * 2 * math.Pi
		directions[i] = mgl64.Vec3{math.Cos(angle), 0, math.Sin(angle)}
	}
	return w, origins, directions
}

// queriesScattered: the bodies fall asleep in 1 s, then each round throws 10000 rays of 2 m down (a foot), 10000 level
// rays of 50 m (a line of sight), all the hits of 10000 level rays, 10000 spheres of 10 cm moved 2 m down and 10000
// capsules moved 2 m level, 10000 overlaps of a sphere of 1 m, and 100 SyncQueries of the world which didn't change
func queriesScattered() map[string]metric {
	w, origins, directions := scattered()
	play(w, 1, nil)
	// the time of the scene is the time of its queries
	recording.step, recording.steps = 0, 0
	clear(recording.phases)

	asleep := sleeping(w)
	filter := feather.DefaultQueryFilter()
	down := mgl64.Vec3{0, -2, 0}
	ball, pill, around := &actor.Sphere{Radius: 0.1}, &actor.Capsule{HalfHeight: 0.6, Radius: 0.3}, &actor.Sphere{Radius: 1}
	hits := make([]feather.Hit, 0, 64)
	bodies := make([]*actor.RigidBody, 0, 64)
	missed := 0.0
	for round := 0; round < queryRounds; round++ {
		batch("ray down 2 m", queryBatch, func(i int) uint64 {
			hit, ok := w.Raycast(origins[i], down, filter)
			if !ok {
				missed++
			}
			return folded(w, hit, ok)
		})
		batch("ray level 50 m", queryBatch, func(i int) uint64 {
			origin := origins[i]
			origin[1] = 0.25
			hit, ok := w.Raycast(origin, directions[i].Mul(50), filter)
			return folded(w, hit, ok)
		})
		batch("all hits level 50 m", queryBatch, func(i int) uint64 {
			origin := origins[i]
			origin[1] = 0.25
			hits = w.RaycastAll(origin, directions[i].Mul(50), filter, hits[:0])
			sum := uint64(len(hits))
			for _, hit := range hits {
				sum = sum*31 + folded(w, hit, true)
			}
			return sum
		})
		batch("sphere down 2 m", queryBatch, func(i int) uint64 {
			hit, ok := w.Sweep(ball, tr(origins[i], mgl64.QuatIdent()), down, filter)
			if !ok {
				missed++
			}
			return folded(w, hit, ok)
		})
		batch("capsule level 2 m", queryBatch, func(i int) uint64 {
			hit, ok := w.Sweep(pill, tr(origins[i], mgl64.QuatIdent()), directions[i].Mul(2), filter)
			return folded(w, hit, ok)
		})
		batch("overlap sphere 1 m", queryBatch, func(i int) uint64 {
			bodies = w.Overlap(around, tr(origins[i], mgl64.QuatIdent()), filter, bodies[:0])
			sum := uint64(len(bodies))
			for _, b := range bodies {
				sum = sum*31 + uint64(b.Serial()-w.Bodies[0].Serial())
			}
			return sum
		})
		batch("sync queries", syncRounds, func(int) uint64 {
			w.SyncQueries()
			return 0
		})
		recording.steps++
	}
	return map[string]metric{
		// a ray or a sphere down always finds the ground; the queries wake nobody up
		"missed the ground":    {missed, ""},
		"woken by the queries": {float64(asleep - sleeping(w)), ""},
	}
}

// sleeping: the bodies asleep
func sleeping(w *feather.World) int {
	count := 0
	for _, b := range w.Bodies {
		if b.IsSleeping {
			count++
		}
	}
	return count
}

// wideTerrain: hills of 1025 x 1025 samples every meter, between -12 and 12 m. Built once: a shape has no state
var wideTerrain = sync.OnceValue(func() *actor.Heightfield {
	const samples = 1025
	heights := make([]float32, samples*samples)
	for x := 0; x < samples; x++ {
		for z := 0; z < samples; z++ {
			heights[x*samples+z] = float32(8*math.Sin(float64(x)*0.013)*math.Cos(float64(z)*0.011) + 3*math.Sin(float64(x+z)*0.05) + math.Cos(float64(x-z)*0.21))
		}
	}
	return actor.NewHeightfield(samples, samples, heights, mgl64.Vec3{1, 1, 1})
})

// queriesTerrain: on a terrain of 1025 x 1025 samples, each round throws 10000 rays straight down, 1000 rays of 1000 m
// across the terrain between its hills (they skip the blocks they fly over), and moves 10000 spheres 2 m down
func queriesTerrain() map[string]metric {
	w := world(1)
	field := wideTerrain()
	terrain := body(w, tr(mgl64.Vec3{}, mgl64.QuatIdent()), field, actor.BodyTypeStatic, 0.6, 0)
	w.SyncQueries()
	recording.worlds = append(recording.worlds, w)
	r := rand.New(rand.NewSource(3))
	points := make([]mgl64.Vec3, queryBatch)
	for i := range points {
		points[i] = mgl64.Vec3{r.Float64()*1000 - 500, 0, r.Float64()*1000 - 500}
		height, _ := field.HeightAt(points[i].X(), points[i].Z())
		points[i][1] = height
	}
	filter := feather.DefaultQueryFilter()
	ball := &actor.Sphere{Radius: 0.1}
	worst, missed := 0.0, 0.0
	for round := 0; round < queryRounds; round++ {
		batch("ray down on the terrain", queryBatch, func(i int) uint64 {
			hit, ok := w.Raycast(points[i].Add(mgl64.Vec3{0, 30, 0}), mgl64.Vec3{0, -60, 0}, filter)
			if !ok || hit.Body != terrain {
				missed++
			} else if recording.measuring {
				worst = math.Max(worst, math.Abs(hit.Point.Y()-points[i].Y()))
			}
			return folded(w, hit, ok)
		})
		batch("ray of 1000 m across the terrain", longRays, func(i int) uint64 {
			// from a side of the terrain to the other, at the height of the hills
			from, to := points[2*i], points[2*i+1]
			from[0], to[0] = -500, 500
			from[1], to[1] = 4+from.Y()/2, 4+to.Y()/2
			hit, ok := w.Raycast(from, to.Sub(from), filter)
			return folded(w, hit, ok)
		})
		batch("sphere down on the terrain", queryBatch, func(i int) uint64 {
			hit, ok := w.Sweep(ball, tr(points[i].Add(mgl64.Vec3{0, 1, 0}), mgl64.QuatIdent()), mgl64.Vec3{0, -2, 0}, filter)
			if !ok {
				missed++
			}
			return folded(w, hit, ok)
		})
		recording.steps++
	}
	return map[string]metric{
		"missed the terrain":    {missed, ""},
		"worst height of a ray": {worst * 1000, "mm"},
	}
}

// queryScenes: the scenes of the queries
var queryScenes = []regressionScene{
	{name: "queries scattered", run: queriesScattered},
	{name: "queries terrain", run: queriesTerrain},
}

// queryCosts prints the cost of a query of each kind, from the best run of its batch, and the queries per second of
// queryGoroutines goroutines throwing rays at the same world
func queryCosts() {
	for _, scene := range queryScenes {
		result := measureScene(scene, 0)
		fmt.Printf("%s (%s)\n", scene.name, result.Fingerprint)
		for _, name := range sortedKeys(result.PhasesMs) {
			count := queryCounts[name]
			if count == 0 {
				continue
			}
			fmt.Printf("  %-34s %9.0f ns per query (%d queries in %.3f ms)\n", name, result.PhasesMs[name]*1e6/float64(count), count, result.PhasesMs[name])
		}
	}

	recording = &recorder{phases: map[string]time.Duration{}}
	w, origins, _ := scattered()
	play(w, 1, nil)
	filter := feather.DefaultQueryFilter()
	down := mgl64.Vec3{0, -2, 0}
	rays := func() {
		for round := 0; round < 20; round++ {
			for i := range origins {
				w.Raycast(origins[i], down, filter)
			}
		}
	}
	const perGoroutine = 20 * queryBatch
	start := time.Now()
	rays()
	alone := time.Since(start)
	start = time.Now()
	var wait sync.WaitGroup
	for g := 0; g < queryGoroutines; g++ {
		wait.Add(1)
		go func() {
			defer wait.Done()
			rays()
		}()
	}
	wait.Wait()
	together := time.Since(start)
	fmt.Printf("rays of 2 m down: %.2f million per second on 1 goroutine, %.2f million per second on %d goroutines (x %.1f)\n",
		perGoroutine/alone.Seconds()/1e6, queryGoroutines*perGoroutine/together.Seconds()/1e6, queryGoroutines,
		queryGoroutines*alone.Seconds()/together.Seconds())
	w.Close()
}
