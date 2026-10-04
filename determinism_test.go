package feather

import (
	"fmt"
	"math"
	"math/rand"
	"slices"
	"sync/atomic"
	"testing"
	"time"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Determinism: the same inputs give the same state bit for bit, run after run and whatever the number of Workers.
// The states are compared by their bits (math.Float64bits), not by ==: -0 and 0 differ, a NaN equals itself.
//
// A step runs on several goroutines from minParallelBodies (256) bodies only, and each stage only when it has more
// items than a chunk. Both scenes below are checked to reach the parallel path of every stage (parallelStages): a scene
// which stops reaching one fails, it would prove nothing about it.
//   - "mixed": 200 bodies, under the threshold, so World.parallelFrom forces the parallel paths;
//   - "pile": 600 bodies, over the threshold, nothing forced: the paths of a game.

const (
	// determinismSteps: the steps of the mixed scene (16.7 s: the bodies fall, bounce and pile up, some fall asleep)
	determinismSteps = 1000

	// raceSteps: the steps of the mixed scene under the race detector (10 to 20 times slower), which also plays each
	// count of workers once: it looks for the data races, the bits are compared without it
	raceSteps = 200

	// determinismBodies: the bodies of the mixed scene, the ground included
	determinismBodies = 200

	// pileBodies & pileSteps: the pile over minParallelBodies, long enough to land and rest
	pileBodies = 600
	pileSteps  = 180
)

// determinismWorkers: the reference is the first run with 1 worker
var determinismWorkers = []int{1, 4, 8}

// mixedScene: 200 bodies on a plane. 3 chains of 10 capsules (ball joints) hang from static anchors, 12 pairs of boxes
// are linked (hinge, distance & fixed joints), and boxes, spheres & capsules fall on them and bounce (restitution 0.5).
// The same scene whatever the workers
func mixedScene(workers int) *World {
	w := newScene(workers)
	w.parallelFrom = 1
	addGround(w, 0.6)
	r := rand.New(rand.NewSource(804))

	const links, halfHeight, radius = 10, 0.2, 0.05
	for chain := 0; chain < 3; chain++ {
		top := mgl64.Vec3{float64(chain) - 1, 4.5, 0}
		previous := addBody(w, top, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.02}, actor.BodyTypeStatic, 0, 0)
		for i := 0; i < links; i++ {
			anchor := top.Sub(mgl64.Vec3{0, float64(i) * 2 * halfHeight, 0})
			capsule := addBody(w, anchor.Sub(mgl64.Vec3{0, halfHeight, 0}), mgl64.QuatIdent(), &actor.Capsule{HalfHeight: halfHeight, Radius: radius}, actor.BodyTypeDynamic, 0.5, 0)
			w.AddJoint(NewBallJoint(previous, capsule, anchor, mgl64.Vec3{0, -1, 0}))
			previous = capsule
		}
	}

	for pair := 0; pair < 12; pair++ {
		center := mgl64.Vec3{r.Float64()*3 - 1.5, 5 + float64(pair)*0.6, r.Float64()*3 - 1.5}
		a := addBody(w, center.Sub(mgl64.Vec3{0.3, 0, 0}), mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0.5)
		b := addBody(w, center.Add(mgl64.Vec3{0.3, 0, 0}), mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0.5)
		switch pair % 3 {
		case 0:
			w.AddJoint(NewHingeJoint(a, b, center, mgl64.Vec3{0, 0, 1}))
		case 1:
			w.AddJoint(NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position))
		case 2:
			w.AddJoint(NewFixedJoint(a, b, center))
		}
	}

	for i := 0; len(w.Bodies) < determinismBodies; i++ {
		rotation := mgl64.QuatRotate(r.Float64()*math.Pi, mgl64.Vec3{r.Float64(), r.Float64(), r.Float64()}.Normalize())
		var shape actor.ShapeInterface = cube()
		switch i % 3 {
		case 1:
			shape = &actor.Sphere{Radius: cubeHalf}
		case 2:
			shape = &actor.Capsule{HalfHeight: 0.2, Radius: 0.15}
		}
		position := mgl64.Vec3{r.Float64()*3 - 1.5, 1 + float64(i)*0.15, r.Float64()*3 - 1.5}
		addBody(w, position, rotation, shape, actor.BodyTypeDynamic, 0.6, 0.5)
	}
	return w
}

// stateBits appends the bits of the whole state the next step starts from. The bodies in the order of World.Bodies
// (position, rotation, velocities, sleep), then the contacts in their order (indices of their bodies, normal, points,
// impulses of the warm start)
func stateBits(w *World, bits []uint64) []uint64 {
	vector := func(v mgl64.Vec3) {
		bits = append(bits, math.Float64bits(v[0]), math.Float64bits(v[1]), math.Float64bits(v[2]))
	}
	for _, body := range w.Bodies {
		vector(body.Transform.Position)
		bits = append(bits, math.Float64bits(body.Transform.Rotation.W))
		vector(body.Transform.Rotation.V)
		vector(body.Velocity)
		vector(body.AngularVelocity)
		sleeping := uint64(0)
		if body.IsSleeping {
			sleeping = 1
		}
		bits = append(bits, sleeping)
	}
	for i := range w.Contacts() {
		contact := &w.Contacts()[i]
		bits = append(bits, uint64(contact.IndexA), uint64(contact.IndexB), uint64(contact.Count))
		vector(contact.Normal)
		for _, point := range contact.Points[:contact.Count] {
			vector(point.Position)
			bits = append(bits, math.Float64bits(point.Separation), math.Float64bits(point.NormalImpulse))
		}
		vector(contact.FrictionImpulse)
		bits = append(bits, math.Float64bits(contact.TwistImpulse))
	}
	return bits
}

// hashBits: FNV-1a over the values, to keep the state of every step
func hashBits(bits []uint64) uint64 {
	const offset, prime = 14695981039346656037, 1099511628211
	hash := uint64(offset)
	for _, value := range bits {
		hash = (hash ^ value) * prime
	}
	return hash
}

// parallelStages follows which stages of the steps of a World ran on several goroutines (workerPool.run splits a stage
// with more items than its chunk)
type parallelStages struct {
	world *World
	// the chunks of the pair search are only known during the step: its jobs are wrapped
	queries, scans atomic.Bool
	// the largest count of items of each other stage, read after each step
	bodies, pairs, contacts, color int
}

func watchParallelStages(w *World) *parallelStages {
	stages := &parallelStages{world: w}
	w.tree.queryJob = func(chunk int) {
		if chunk > 0 {
			stages.queries.Store(true)
		}
		w.tree.query(chunk)
	}
	w.tree.scanJob = func(chunk int) {
		if chunk > 0 {
			stages.scans.Store(true)
		}
		w.tree.scan(chunk)
	}
	return stages
}

// afterStep reads the sizes of the stages of the last step
func (stages *parallelStages) afterStep() {
	w := stages.world
	stages.bodies = max(stages.bodies, len(w.Bodies))
	stages.pairs = max(stages.pairs, len(w.pairs))
	stages.contacts = max(stages.contacts, len(w.Contacts()))
	for k := range w.solver.graph.colors {
		stages.color = max(stages.color, len(w.solver.graph.colors[k].items))
	}
}

// missing: the stages which never had enough items to be split between the workers
func (stages *parallelStages) missing() []string {
	var missing []string
	check := func(reached bool, stage string) {
		if !reached {
			missing = append(missing, stage)
		}
	}
	pool := stages.world.workerPool()
	check(pool.helpers == stages.world.Workers-1 && pool.generation.Load() > 0, "the workers (none was given a stage)")
	check(stages.queries.Load(), "the pairs of the moved bodies (Tree.query)")
	check(stages.scans.Load(), "the pairs of the step (Tree.scan)")
	check(stages.bodies > bodiesChunk, "the AABBs & the integration of the bodies")
	check(stages.pairs > pairsPerChunk, "the narrow phase")
	check(stages.contacts > constraintsChunk, "the preparation of the contacts")
	check(stages.color > constraintsChunk, "the solver (no color larger than a chunk)")
	return missing
}

// runDeterministic plays the steps of a scene and returns the hash of its state after each step, and its final state
func runDeterministic(t *testing.T, w *World, steps int, each func(step int)) ([]uint64, []uint64) {
	t.Helper()
	defer w.Close()
	stages := watchParallelStages(w)
	hashes := make([]uint64, 0, steps)
	var bits []uint64
	for step := 0; step < steps; step++ {
		if each != nil {
			each(step)
		}
		w.Step(sceneDt)
		stages.afterStep()
		bits = stateBits(w, bits[:0])
		hashes = append(hashes, hashBits(bits))
	}
	if w.Workers > 1 {
		if missing := stages.missing(); len(missing) > 0 {
			t.Fatalf("workers=%d: stages never run in parallel, the scene proves nothing about them: %v", w.Workers, missing)
		}
	}
	for _, body := range w.Bodies {
		if !finite(body.Transform.Position) || !finite(body.Velocity) {
			t.Fatalf("workers=%d: the scene exploded: %v", w.Workers, body.Transform.Position)
		}
	}
	return hashes, bits
}

// sameRun fails at the first step whose state differs from the reference, then names the first value of the final
// state which differs
func sameRun(t *testing.T, what string, hashes, final, referenceHashes, referenceFinal []uint64) {
	t.Helper()
	for step := range referenceHashes {
		if hashes[step] != referenceHashes[step] {
			t.Fatalf("%s: the state differs from the reference from the step %d of %d", what, step, len(referenceHashes))
		}
	}
	if len(final) != len(referenceFinal) {
		t.Fatalf("%s: %d values in the final state, the reference has %d", what, len(final), len(referenceFinal))
	}
	for i := range referenceFinal {
		if final[i] != referenceFinal[i] {
			t.Fatalf("%s: value %d of the final state is %#x, the reference has %#x", what, i, final[i], referenceFinal[i])
		}
	}
}

// compareRuns plays the scene twice with each count of workers (once under the race detector): every run gives the
// states of the first one
func compareRuns(t *testing.T, steps int, scene func(workers int) (*World, func(step int))) {
	t.Helper()
	runs := 2
	if raceEnabled {
		runs = 1
	}
	var referenceHashes, referenceFinal []uint64
	for _, workers := range determinismWorkers {
		for run := 1; run <= runs; run++ {
			w, each := scene(workers)
			hashes, final := runDeterministic(t, w, steps, each)
			if referenceHashes == nil {
				referenceHashes, referenceFinal = hashes, final
				continue
			}
			sameRun(t, fmt.Sprintf("workers=%d, run %d", workers, run), hashes, final, referenceHashes, referenceFinal)
		}
	}
}

// 200 bodies (a pile bouncing on chains and on linked boxes), 1000 steps: the state after every step is the same bit for
// bit between two runs, and with 1, 4 and 8 workers. The scene covers the sleep: some bodies fall asleep during the run
// (and some wake up again: the fast bodies of the scene hit them)
func TestMixedSceneIsDeterministic(t *testing.T) {
	steps := determinismSteps
	if raceEnabled {
		steps = raceSteps
	}
	asleep, moving := 0, 0
	compareRuns(t, steps, func(workers int) (*World, func(step int)) {
		w := mixedScene(workers)
		if len(w.Bodies) != determinismBodies || len(w.Joints) == 0 {
			t.Fatalf("%d bodies & %d joints, want %d bodies and joints", len(w.Bodies), len(w.Joints), determinismBodies)
		}
		if workers == determinismWorkers[0] {
			asleep, moving = 0, 0
		}
		return w, func(step int) {
			if workers != determinismWorkers[0] {
				return
			}
			// the most bodies asleep at the start of a step, over the run
			sleeping, awake := 0, 0
			for _, body := range w.Bodies {
				if body.IsSleeping {
					sleeping++
				} else if body.BodyType == actor.BodyTypeDynamic {
					awake++
				}
			}
			if sleeping > asleep {
				asleep, moving = sleeping, awake
			}
		}
	})
	t.Logf("at most %d bodies asleep at a step, %d dynamic bodies awake then", asleep, moving)
	if asleep == 0 {
		t.Error("no body fell asleep: the scene doesn't cover the sleep")
	}
}

// 600 bodies, over minParallelBodies: nothing forces the parallel paths, they run as in a game
func TestLargePileIsDeterministic(t *testing.T) {
	compareRuns(t, pileSteps, func(workers int) (*World, func(step int)) {
		w := benchScene(pileBodies, workers)
		if w.parallelFrom != 0 || len(w.Bodies) < minParallelBodies {
			t.Fatalf("%d bodies, parallelFrom %d: the scene must be parallel by itself", len(w.Bodies), w.parallelFrom)
		}
		return w, nil
	})
}

// consistentOrder: the order of World.Bodies is the one of the whole step. The proxies of the broad phase are those of
// the bodies at the same index, the pairs are sorted by the indices of their bodies, and the contacts follow the pairs
// (that the pairs are those of a search from scratch is TestTreePairsAsBruteForce). Returns the first gap
func consistentOrder(w *World) error {
	tree := &w.tree
	if !slices.Equal(tree.bodies, w.Bodies) || len(tree.proxies) != len(w.Bodies) {
		return fmt.Errorf("the tree has %d bodies & %d proxies in another order than the %d bodies of the World", len(tree.bodies), len(tree.proxies), len(w.Bodies))
	}
	planes := 0
	for i, body := range w.Bodies {
		p := tree.proxies[i]
		if p.kind != kindOf(body) {
			return fmt.Errorf("body %d: proxy of kind %d, want %d", i, p.kind, kindOf(body))
		}
		switch p.kind {
		case proxyLarge:
			if planes >= len(tree.planes) || tree.planes[planes] != int32(i) {
				return fmt.Errorf("body %d: not the large body %d of the tree (%v)", i, planes, tree.planes)
			}
			planes++
		case proxyDynamic:
			if owner := tree.dynamics.nodes[p.node].body; owner != int32(i) {
				return fmt.Errorf("body %d: its leaf belongs to the body %d", i, owner)
			}
		case proxyStatic:
			if owner := tree.statics.nodes[p.node].body; owner != int32(i) {
				return fmt.Errorf("body %d: its leaf belongs to the body %d", i, owner)
			}
		}
	}
	if planes != len(tree.planes) {
		return fmt.Errorf("%d large bodies in the tree, %d in the World", len(tree.planes), planes)
	}

	// the pairs: by the index of their first body (the one which is not a plane, of the lowest index), its planes first
	// in their order, then its other bodies in their order. Tree.sortPairs is not asked: the indices are read in
	// World.Bodies
	previous := [3]int{-1, 0, 0}
	for i, pair := range w.pairs {
		a, b := slices.Index(w.Bodies, pair.BodyA), slices.Index(w.Bodies, pair.BodyB)
		if a < 0 || b < 0 || int(pair.IndexA) != a || int(pair.IndexB) != b {
			return fmt.Errorf("pair %d: its indices %d & %d are not those of its bodies (%d & %d)", i, pair.IndexA, pair.IndexB, a, b)
		}
		key := [3]int{min(a, b), 1, max(a, b)}
		if isLarge(pair.BodyA) {
			key = [3]int{b, 0, a}
		} else if isLarge(pair.BodyB) {
			key = [3]int{a, 0, b}
		}
		if slices.Compare(key[:], previous[:]) <= 0 {
			return fmt.Errorf("pair %d (bodies %d & %d): not after the pair %d in the order of the bodies", i, a, b, i-1)
		}
		previous = key
	}
	next := 0
	for i := range w.Contacts() {
		contact := &w.Contacts()[i]
		for next < len(w.pairs) && (w.pairs[next].BodyA != contact.BodyA || w.pairs[next].BodyB != contact.BodyB) {
			next++
		}
		if next == len(w.pairs) {
			return fmt.Errorf("contact %d: out of the order of the pairs", i)
		}
	}
	return nil
}

// Bodies removed then added again keep a defined order: the bodies left keep theirs (they move up), a body added goes
// last, whatever its former index. The broad phase, the pairs & the contacts follow this order at every step, and the
// whole run is the same bit for bit between two runs and with 1, 4 and 8 workers.
//
// The bodies leave from the middle of the pile, from a chain (its joints leave with it) and from the end, then while
// they sleep; they come back 1 s later with their joints, in another order than they had
func TestRemovedAndAddedBodiesKeepADefinedOrder(t *testing.T) {
	const removeAt, addAt, removeAsleepAt, addAsleepAt, steps = 90, 150, 300, 360, 450
	// the indices of the bodies removed first: a link in the middle of the first chain, the first box of a hinge, bodies
	// of the pile, the last body
	leaving := []int{5, 34, 80, 81, 150, determinismBodies - 1}
	// sleepersLeaving: the count of sleeping bodies removed the second time, with the body in the middle of the World
	const sleepersLeaving = 4

	compareRuns(t, steps, func(workers int) (*World, func(step int)) {
		w := mixedScene(workers)
		var removed []*actor.RigidBody
		var joints []Joint

		remove := func(step int, indices []int) {
			before := slices.Clone(w.Bodies)
			removed, joints = removed[:0], joints[:0]
			// from the last one: the indices before it don't move
			for i := len(indices) - 1; i >= 0; i-- {
				body := w.Bodies[indices[i]]
				for _, joint := range w.Joints {
					if base := joint.base(); base.BodyA == body || base.BodyB == body {
						joints = append(joints, joint)
					}
				}
				removed = append(removed, body)
				w.RemoveBody(body)
			}
			want := slices.DeleteFunc(before, func(body *actor.RigidBody) bool { return slices.Contains(removed, body) })
			if !slices.Equal(w.Bodies, want) {
				t.Fatalf("workers=%d, step %d: the bodies left changed of order", workers, step)
			}
			for _, joint := range joints {
				if slices.Contains(w.Joints, joint) {
					t.Fatalf("workers=%d, step %d: a joint of a removed body is still in the World", workers, step)
				}
			}
		}
		add := func(step int) {
			before := slices.Clone(w.Bodies)
			// in the order they left: the last body of the World first
			for _, body := range removed {
				w.AddBody(body)
			}
			for _, joint := range joints {
				w.AddJoint(joint)
			}
			if !slices.Equal(w.Bodies, append(before, removed...)) {
				t.Fatalf("workers=%d, step %d: the bodies added are not the last ones, in the order of AddBody", workers, step)
			}
		}

		return w, func(step int) {
			if step > 0 {
				// the step before: its pairs & contacts follow the order of the bodies
				if err := consistentOrder(w); err != nil {
					t.Fatalf("workers=%d, step %d: %v", workers, step-1, err)
				}
			}
			switch step {
			case removeAt:
				remove(step, leaving)
			case removeAsleepAt:
				indices := []int{len(w.Bodies) / 2}
				for i, body := range w.Bodies {
					if body.IsSleeping && i != indices[0] && len(indices) <= sleepersLeaving {
						indices = append(indices, i)
					}
				}
				if len(indices) <= sleepersLeaving {
					t.Fatalf("workers=%d: %d bodies asleep at the step %d, want %d to remove", workers, len(indices)-1, step, sleepersLeaving)
				}
				slices.Sort(indices)
				remove(step, indices)
			case addAt, addAsleepAt:
				add(step)
			}
		}
	})
}

// The cost of Tree.sortPairs on the pairs of a step, as the pair search hands them over (in the order of its chunks),
// while a pile lands (0.5 s after its start: asleep, it has no pair left). "%step" is its share of the steps of the
// scene around this one, with these workers (the sort runs on one goroutine). The figures are in ARCHITECTURE.md
func BenchmarkSortPairs(b *testing.B) {
	for _, count := range []int{500, 2000} {
		for _, workers := range []int{1, 8} {
			b.Run(fmt.Sprintf("%d_bodies_%d_workers", count, workers), func(b *testing.B) {
				w := benchScene(count, workers)
				defer w.Close()
				simulate(w, 0.25, nil)
				const measuredSteps = 15
				var step time.Duration
				for range measuredSteps {
					w.Step(sceneDt)
					step += w.Profile().Step
				}
				var unsorted []Pair
				for c := range w.tree.chunks {
					unsorted = append(unsorted, w.tree.chunks[c].pairs...)
				}
				if len(unsorted) == 0 || len(unsorted) != len(w.pairs) {
					b.Fatalf("%d pairs in the chunks of the search, %d in the step: nothing to measure", len(unsorted), len(w.pairs))
				}
				// sortPairs reads its input and writes Tree.sorted, then swaps them: given back at each run
				output := make([]Pair, len(unsorted))
				b.ResetTimer()
				for range b.N {
					w.tree.sorted = output
					w.tree.sortPairs(unsorted, len(w.Bodies))
				}
				b.StopTimer()
				sort := float64(b.Elapsed().Nanoseconds()) / float64(b.N)
				b.ReportMetric(float64(len(unsorted)), "pairs")
				b.ReportMetric(100*sort*measuredSteps/float64(step.Nanoseconds()), "%step")
			})
		}
	}
}
