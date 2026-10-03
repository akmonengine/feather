package feather

import (
	"math"
	"math/rand"
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

func randomAABB(r *rand.Rand, spread float64) actor.AABB {
	c := mgl64.Vec3{r.Float64() * spread, r.Float64() * spread, r.Float64() * spread}
	h := mgl64.Vec3{0.1 + r.Float64(), 0.1 + r.Float64(), 0.1 + r.Float64()}
	return actor.AABB{Min: c.Sub(h), Max: c.Add(h)}
}

// The tree finds exactly the leaves overlapping a query, as a brute force does, through insertions and removals
func TestTreeQueryIsExact(t *testing.T) {
	r := rand.New(rand.NewSource(7))
	var tree aabbTree
	tree.clear()
	boxes := map[int32]actor.AABB{}
	nodes := map[int32]int32{}
	var stack, out []int32
	for round := 0; round < 3000; round++ {
		if len(boxes) > 0 && r.Intn(3) == 0 {
			// remove one
			keys := make([]int32, 0, len(boxes))
			for k := range boxes {
				keys = append(keys, k)
			}
			k := keys[r.Intn(len(keys))]
			tree.remove(nodes[k])
			delete(boxes, k)
			delete(nodes, k)
		} else {
			k := int32(round)
			boxes[k] = randomAABB(r, 30)
			nodes[k] = tree.insert(boxes[k], k)
		}
		if round%100 == 99 {
			query := randomAABB(r, 30)
			stack, out = tree.query(query, stack, out[:0])
			slices.Sort(out)
			var want []int32
			for k, b := range boxes {
				if b.Overlaps(query) {
					want = append(want, k)
				}
			}
			slices.Sort(want)
			if !slices.Equal(out, want) {
				t.Fatalf("round %d: query found %v, want %v", round, out, want)
			}
		}
	}
}

// The tree stays balanced: its height is logarithmic
func TestTreeIsBalanced(t *testing.T) {
	r := rand.New(rand.NewSource(3))
	var tree aabbTree
	tree.clear()
	const count = 4096
	for i := 0; i < count; i++ {
		tree.insert(randomAABB(r, 100), int32(i))
	}
	if h := tree.height(); float64(h) > 2*math.Log2(count) {
		t.Errorf("height %d for %d leaves, want at most %.0f", h, count, 2*math.Log2(count))
	}
	// the invariants of every node: height, AABB containing its children
	var check func(n int32) int32
	check = func(n int32) int32 {
		node := tree.nodes[n]
		if node.height == 0 {
			return 0
		}
		h1, h2 := check(node.child1), check(node.child2)
		if node.height != 1+max(h1, h2) {
			t.Errorf("node %d: height %d, children %d and %d", n, node.height, h1, h2)
		}
		if tree.nodes[node.child1].parent != n || tree.nodes[node.child2].parent != n {
			t.Errorf("node %d: a child doesn't point back to it", n)
		}
		if !contains(node.aabb, tree.nodes[node.child1].aabb) || !contains(node.aabb, tree.nodes[node.child2].aabb) {
			t.Errorf("node %d: its AABB doesn't contain its children", n)
		}
		return node.height
	}
	check(tree.root)
}

func bruteForcePairs(bodies []*actor.RigidBody, boxes []actor.AABB) []Pair {
	var pairs []Pair
	for i, a := range bodies {
		if isLarge(a) {
			continue
		}
		for j, b := range bodies {
			if isLarge(b) && emitted(a, b) && boxes[i].Overlaps(boxes[j]) {
				pairs = append(pairs, Pair{BodyA: b, BodyB: a})
			}
		}
		for j := i + 1; j < len(bodies); j++ {
			b := bodies[j]
			if !isLarge(b) && emitted(a, b) && boxes[i].Overlaps(boxes[j]) {
				pairs = append(pairs, Pair{BodyA: a, BodyB: b})
			}
		}
	}
	return pairs
}

// emitted: the pair of the bodies is one of the step, to solve or a trigger
func emitted(a, b *actor.RigidBody) bool {
	return needsSolving(a, b) || detectsTrigger(a, b)
}

func samePairs(t *testing.T, got, want []Pair, what string) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("%s: %d pairs, want %d", what, len(got), len(want))
	}
	for i := range got {
		if got[i].BodyA != want[i].BodyA || got[i].BodyB != want[i].BodyB {
			t.Fatalf("%s: pair %d differs", what, i)
		}
	}
}

// The pairs of the tree are those of the brute force, in the same order (planes of a body first, then by index), with
// static and sleeping bodies, through the steps of a scene with 1 and 8 workers, and after the removal of bodies
func TestTreePairsAsBruteForce(t *testing.T) {
	for _, workers := range []int{1, 8} {
		w := newScene(workers)
		addGround(w, 0.6)
		r := rand.New(rand.NewSource(11))
		for i := 0; i < 300; i++ {
			shape := actor.ShapeInterface(cube())
			if i%3 == 1 {
				shape = &actor.Sphere{Radius: cubeHalf}
			}
			bodyType := actor.BodyTypeDynamic
			if i%7 == 6 {
				bodyType = actor.BodyTypeStatic
			}
			addBody(w, mgl64.Vec3{r.Float64()*8 - 4, 0.5 + r.Float64()*6, r.Float64()*8 - 4}, mgl64.QuatIdent(), shape, bodyType, 0.6, 0)
		}
		for step := 0; step < 120; step++ {
			w.Step(1.0 / 60)
			if step%20 == 19 {
				// the pairs of the step, on the same AABBs
				pool := w.workerPool()
				pool.begin(workers)
				got := w.tree.findPairs(w.Bodies, w.aabbs, pool)
				pool.end()
				samePairs(t, got, bruteForcePairs(w.Bodies, w.aabbs), "step")
			}
			if step == 60 {
				// remove a body in the middle and one at the end
				w.RemoveBody(w.Bodies[len(w.Bodies)/2])
				w.RemoveBody(w.Bodies[len(w.Bodies)-1])
			}
		}
		asleep := 0
		for _, b := range w.Bodies {
			if b.IsSleeping {
				asleep++
			}
		}
		if asleep == 0 {
			t.Errorf("workers %d: no sleeping body after 2 s, the test doesn't cover the sleeping pairs", workers)
		}
	}
}

// The pairs of a body with hundreds of pairs (a static zone with 400 crates in it, asleep or awake, kinematic cubes
// moving in it, and a kinematic trigger) are those of the brute force, in the same order, with 1 and 8 workers
func TestTreePairsOfAZoneAsBruteForce(t *testing.T) {
	for _, workers := range []int{1, 8} {
		w := newScene(workers)
		addGround(w, 0.6)
		zone := addBody(w, mgl64.Vec3{0, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{10, 1, 10}}, actor.BodyTypeStatic, 0, 0)
		zone.IsTrigger = true
		for k := 0; k < 400; k++ {
			addBody(w, mgl64.Vec3{float64(k%20)*0.8 - 8, 0.3 + float64(k%3), float64(k/20)*0.8 - 8}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		}
		var movers []*actor.RigidBody
		for k := 0; k < 8; k++ {
			movers = append(movers, kinematicBox(w, mgl64.Vec3{float64(k)*2 - 8, 2.5, 0}, cube().HalfExtents))
		}
		movers[0].IsTrigger = true
		for step := 0; step < 240; step++ {
			for _, mover := range movers {
				moveTo(mover, translated(mover.Transform, mgl64.Vec3{0, 0, kinematicSpeed}))
			}
			w.Step(sceneDt)
			if step%40 == 39 {
				pool := w.workerPool()
				pool.begin(workers)
				got := w.tree.findPairs(w.Bodies, w.aabbs, pool)
				pool.end()
				samePairs(t, got, bruteForcePairs(w.Bodies, w.aabbs), "zone")
				zonePairs := 0
				for _, pair := range got {
					if pair.BodyA == zone || pair.BodyB == zone {
						zonePairs++
					}
				}
				if zonePairs < 400 {
					t.Fatalf("workers %d: %d pairs of the zone, the test doesn't cover a body with hundreds of pairs", workers, zonePairs)
				}
			}
		}
		if !slices.ContainsFunc(w.Bodies, func(b *actor.RigidBody) bool { return b.IsSleeping && b.BodyType == actor.BodyTypeDynamic }) {
			t.Errorf("workers %d: no crate asleep, the test doesn't cover the pairs at rest", workers)
		}
	}
}

// A body removed right after a step it left its enlarged AABB in (the tree keeps it to find its pairs at the next step)
// is forgotten: the next step finds the pairs of the bodies left, the removed body being the last one of the World or
// one in the middle
func TestTreeForgetsARemovedBodyWhichMoved(t *testing.T) {
	for _, last := range []bool{true, false} {
		w := newScene(1)
		addGround(w, 0.6)
		addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		fast := addBody(w, mgl64.Vec3{3, 5, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		fast.Velocity = mgl64.Vec3{30, 0, 0}
		if !last {
			addBody(w, mgl64.Vec3{-3, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		}
		w.Step(sceneDt)
		if !slices.Contains(w.tree.moved, 2) {
			t.Fatalf("last %v: the fast body didn't leave its enlarged AABB (moved: %v): the scene tests nothing", last, w.tree.moved)
		}
		w.RemoveBody(fast)
		if slices.Contains(w.tree.moved, 2) {
			t.Errorf("last %v: the tree still has to find the pairs of the removed body (moved: %v)", last, w.tree.moved)
		}
		w.Step(sceneDt)
		pool := w.workerPool()
		pool.begin(1)
		got := w.tree.findPairs(w.Bodies, w.aabbs, pool)
		pool.end()
		samePairs(t, got, bruteForcePairs(w.Bodies, w.aabbs), "after the removal")
	}
}

// A body which changes of type (static to dynamic) moves between the trees
func TestTreeBodyChangesType(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	// the static cube floats at 0.5, the dynamic one lands on it, then the static one becomes dynamic: both fall
	a := addBody(w, mgl64.Vec3{0, 0.5, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
	b := addBody(w, mgl64.Vec3{0, 3, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 1, nil)
	if b.Transform.Position.Y() < 0.5+2*cubeHalf-0.01 {
		t.Fatalf("the cube fell through the static one: y = %.3f", b.Transform.Position.Y())
	}
	a.BodyType = actor.BodyTypeDynamic
	a.Material = actor.NewRigidBody(a.Transform, a.Shape, actor.BodyTypeDynamic, 1).Material
	simulate(w, 1, nil)
	if a.Transform.Position.Y() > cubeHalf+0.01 || b.Transform.Position.Y() > 3*cubeHalf+0.01 {
		t.Errorf("after the change of type: a at y = %.3f, b at y = %.3f", a.Transform.Position.Y(), b.Transform.Position.Y())
	}
}
