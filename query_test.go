package feather

import (
	"math"
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== THE TREES OF THE QUERIES ==========

// staleProxies: the bodies whose exact AABB leaves the AABB stored in the broad phase, and the worst overshoot (m)
func staleProxies(t *testing.T, w *World) (int, float64) {
	t.Helper()
	if len(w.tree.proxies) != len(w.Bodies) {
		t.Fatalf("%d proxies for %d bodies", len(w.tree.proxies), len(w.Bodies))
	}
	stale, worst := 0, 0.0
	for i, body := range w.Bodies {
		exact := body.Shape.ComputeAABB(body.Transform)
		if exact != body.AABB() {
			t.Fatalf("body %d: its AABB is not the AABB of its shape at its transform", i)
		}
		stored := w.tree.proxies[i].aabb
		if contains(stored, exact) {
			continue
		}
		stale++
		for k := 0; k < 3; k++ {
			worst = math.Max(worst, math.Max(stored.Min[k]-exact.Min[k], exact.Max[k]-stored.Max[k]))
		}
	}
	return stale, worst
}

// brokenPile: 5 boxes at rest in a row, hit by a ball at 60 m/s
func brokenPile(workers int) *World {
	w := newScene(workers)
	addGround(w, 0.6)
	for i := 0; i < 5; i++ {
		addBody(w, mgl64.Vec3{float64(i) * 0.6, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	}
	simulate(w, 1, nil)
	ball := addBody(w, mgl64.Vec3{-3, cubeHalf, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: cubeHalf}, actor.BodyTypeDynamic, 0.6, 0.5)
	ball.Velocity = mgl64.Vec3{60, 0, 0}
	return w
}

// Criterion 1: after each step, the AABB stored for each body contains its exact AABB. The solver accelerates the
// bodies hit during the step: before, the trees were only right at the start of the next step
func TestStepLeavesTheTreesUpToDate(t *testing.T) {
	w := brokenPile(1)
	stale, worst := 0, 0.0
	for step := 0; step < 120; step++ {
		w.Step(sceneDt)
		count, overshoot := staleProxies(t, w)
		stale, worst = stale+count, math.Max(worst, overshoot)
	}
	if stale > 0 {
		t.Errorf("%d times, the exact AABB of a body left its stored AABB after a step (by %.3f m at worst)", stale, worst)
	}
}

// The update of the trees at the end of a step only takes the bodies the step moved (the bodies of the solver). What
// the game wrote to a static body or to a sleeping body is not looked at: it is taken by SyncQueries, or by the next step
func TestEndOfStepOnlyTakesTheMovedBodies(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	wall := addBody(w, mgl64.Vec3{5, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 1, 2}}, actor.BodyTypeStatic, 0.6, 0)
	sleeper := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	for step := 0; step < 600 && !sleeper.IsSleeping; step++ {
		w.Step(sceneDt)
	}
	if !sleeper.IsSleeping {
		t.Fatal("the cube at rest on the ground never fell asleep")
	}
	ball := addBody(w, mgl64.Vec3{-3, 4, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: cubeHalf}, actor.BodyTypeDynamic, 0.6, 0.5)
	w.Step(sceneDt)
	if sleeper.IsSleeping == ball.IsSleeping {
		t.Fatalf("the cube asleep: %v, the ball asleep: %v, want the cube only", sleeper.IsSleeping, ball.IsSleeping)
	}

	// the end of the step is run again, on bodies written since
	for _, body := range []*actor.RigidBody{wall, sleeper, ball} {
		body.Transform.Position = body.Transform.Position.Add(mgl64.Vec3{0, 10, 0})
		body.UpdateAABB()
	}
	before := slices.Clone(w.tree.proxies)
	w.syncMoved()
	for name, body := range map[string]*actor.RigidBody{"static": wall, "sleeping": sleeper} {
		if i := slices.Index(w.Bodies, body); w.tree.proxies[i] != before[i] {
			t.Errorf("the %s body was put back in its tree by the end of the step", name)
		}
	}
	if i := slices.Index(w.Bodies, ball); !contains(w.tree.proxies[i].aabb, ball.AABB()) {
		t.Errorf("the body moved by the step is not in the trees at its place after the end of the step")
	}

	// a world asleep: the end of its step touches no tree
	w = newScene(1)
	addGround(w, 0.6)
	for i := 0; i < 5; i++ {
		addBody(w, mgl64.Vec3{float64(i) * 0.6, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	}
	for step := 0; step < 600 && !allAsleep(w); step++ {
		w.Step(sceneDt)
	}
	if !allAsleep(w) {
		t.Fatal("the row of cubes never fell asleep")
	}
	w.Step(sceneDt)
	if count := len(w.solver.stateBody); count != 0 {
		t.Fatalf("%d bodies in the solver of a world asleep", count)
	}
	asleep := snapshotTree(w)
	w.syncMoved()
	if !snapshotTree(w).equal(asleep) {
		t.Errorf("the end of the step of a world asleep changed its trees")
	}
	if stale, worst := staleProxies(t, w); stale > 0 {
		t.Errorf("%d bodies of a world asleep out of their stored AABB (by %.3f m at worst)", stale, worst)
	}
}

// allAsleep: no dynamic body of the world is awake
func allAsleep(w *World) bool {
	return !slices.ContainsFunc(w.Bodies, isAwakeDynamic)
}

// treeState: what the trees of the broad phase hold
type treeState struct {
	dynamics, statics []treeNode
	planes, moved     []int32
	proxies           []proxy
}

func snapshotTree(w *World) treeState {
	return treeState{
		dynamics: slices.Clone(w.tree.dynamics.nodes), statics: slices.Clone(w.tree.statics.nodes),
		planes: slices.Clone(w.tree.planes), moved: slices.Clone(w.tree.moved), proxies: slices.Clone(w.tree.proxies),
	}
}

func (s treeState) equal(other treeState) bool {
	return slices.Equal(s.dynamics, other.dynamics) && slices.Equal(s.statics, other.statics) &&
		slices.Equal(s.planes, other.planes) && slices.Equal(s.moved, other.moved) && slices.Equal(s.proxies, other.proxies)
}

// Criterion 2: without any step, a body added, a static body moved and a body whose shape changed are in the trees at
// their place after SyncQueries
func TestSyncQueriesTakesTheChangesOfTheGame(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	wall := addBody(w, mgl64.Vec3{5, 1, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.1, 1, 2}}, actor.BodyTypeStatic, 0.6, 0)
	crate := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 0.5, nil)

	// a body added
	late := addBody(w, mgl64.Vec3{0, 5, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
	if len(w.tree.proxies) != len(w.Bodies)-1 {
		t.Fatalf("%d proxies for %d bodies before SyncQueries: the body added is already in the trees", len(w.tree.proxies), len(w.Bodies))
	}
	// a static body moved
	wall.Transform.Position = mgl64.Vec3{-40, 1, 3}
	wall.UpdateAABB()
	// a shape changed
	crate.Shape = &actor.Sphere{Radius: 2}
	crate.UpdateAABB()

	// without SyncQueries, the rays see the trees of the last step
	down := mgl64.Vec3{0, -20, 0}
	over := func(body *actor.RigidBody) (Hit, bool) {
		filter := DefaultQueryFilter()
		for _, other := range w.Bodies {
			if other != body {
				filter.Excluded = append(filter.Excluded, other)
			}
		}
		return w.Raycast(body.Transform.Position.Add(mgl64.Vec3{0, 10, 0}), down, filter)
	}
	if _, ok := over(late); ok {
		t.Error("the body added is hit before SyncQueries")
	}
	if _, ok := over(wall); ok {
		t.Error("the wall is hit at its new place before SyncQueries")
	}

	w.SyncQueries()
	for name, body := range map[string]*actor.RigidBody{"added": late, "moved": wall, "reshaped": crate} {
		hit, ok := over(body)
		if top := body.AABB().Max.Y(); !ok || hit.Body != body || math.Abs(hit.Point.Y()-top) > 1e-9 {
			t.Errorf("after SyncQueries, the ray over the body %s hits %v at the height %g, want its top at %g", name, ok, hit.Point.Y(), top)
		}
	}
	if _, ok := w.Raycast(mgl64.Vec3{5, 10, 0}, down, QueryFilter{Mask: actor.AllLayers, Excluded: []*actor.RigidBody{w.Bodies[0]}}); ok {
		t.Error("the wall is still hit at its old place after SyncQueries")
	}
	if stale, worst := staleProxies(t, w); stale > 0 {
		t.Errorf("%d bodies out of their stored AABB after SyncQueries (by %.3f m at worst)", stale, worst)
	}
	for _, body := range []*actor.RigidBody{wall, crate, late} {
		i := slices.Index(w.Bodies, body)
		p := w.tree.proxies[i]
		nodes := w.tree.statics.nodes
		if body.BodyType == actor.BodyTypeDynamic {
			nodes = w.tree.dynamics.nodes
		}
		if p.node == nullNode || nodes[p.node].body != int32(i) || nodes[p.node].aabb != p.aabb {
			t.Errorf("body %d: its leaf is not in its tree with its stored AABB", i)
		}
	}

	// a body which becomes static changes of tree
	crate.BodyType = actor.BodyTypeStatic
	w.SyncQueries()
	i := slices.Index(w.Bodies, crate)
	if p := w.tree.proxies[i]; p.kind != proxyStatic || w.tree.statics.nodes[p.node].body != int32(i) {
		t.Errorf("the body which became static is not in the tree of the static bodies")
	}
}

// Criterion 3: SyncQueries on a world which didn't change touches no tree, and allocates nothing
func TestSyncQueriesOnAnUnchangedWorld(t *testing.T) {
	for _, workers := range []int{1, 8} {
		w := benchScene(300, workers)
		addBody(w, mgl64.Vec3{3, 1, 3}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
		slopeTerrain(w, 0.1, 0.6)
		simulate(w, 0.5, nil)
		before := snapshotTree(w)
		w.SyncQueries()
		if !snapshotTree(w).equal(before) {
			t.Errorf("workers=%d: SyncQueries changed the trees of a world which didn't change", workers)
		}
		if raceEnabled {
			continue
		}
		if allocs := testing.AllocsPerRun(20, w.SyncQueries); allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per SyncQueries, want 0", workers, allocs)
		}
	}
}

// Criterion 3: a scene stepped with a SyncQueries between the steps gives the same bits as the scene without
func TestSyncQueriesDoesNotChangeTheSimulation(t *testing.T) {
	for _, workers := range []int{1, 8} {
		plain, synced := brokenPile(workers), brokenPile(workers)
		for step := 0; step < 240; step++ {
			plain.Step(sceneDt)
			synced.SyncQueries()
			synced.Step(sceneDt)
			synced.SyncQueries()
		}
		for i := range plain.Bodies {
			a, b := plain.Bodies[i], synced.Bodies[i]
			if a.Transform != b.Transform || a.Velocity != b.Velocity || a.AngularVelocity != b.AngularVelocity || a.IsSleeping != b.IsSleeping {
				t.Fatalf("workers=%d: body %d differs with SyncQueries between the steps", workers, i)
			}
		}
	}
}

// ========== THE GUARD ==========

// panicMessage of the call, "" if it doesn't panic
func panicMessage(call func()) (message string) {
	defer func() {
		if r := recover(); r != nil {
			message, _ = r.(string)
			if message == "" {
				message = "panic"
			}
		}
	}()
	call()
	return ""
}

// Criterion 23: the queries panic during a step
func TestQueryDuringStepPanics(t *testing.T) {
	w := brokenPile(1)
	w.Step(sceneDt)
	if message := panicMessage(w.SyncQueries); message != "" {
		t.Fatalf("SyncQueries between 2 steps panics: %s", message)
	}
	w.stepping.Store(true)
	if message := panicMessage(w.SyncQueries); message != "feather: query during Step" {
		t.Errorf("SyncQueries during a step: panic %q, want %q", message, "feather: query during Step")
	}
	w.stepping.Store(false)
}

// Criterion 23: a listener of an event runs after the trees are updated and the guard is lifted: it sees the end of
// the step
func TestListenerSeesTheEndOfTheStep(t *testing.T) {
	w := brokenPile(1)
	calls, stale := 0, 0
	w.Events.Subscribe(EventCollisionEnter, func(Event) {
		calls++
		if w.stepping.Load() {
			t.Fatal("the guard is still up in a listener")
		}
		count, _ := staleProxies(t, w)
		stale += count
		before := snapshotTree(w)
		if message := panicMessage(w.SyncQueries); message != "" {
			t.Fatalf("SyncQueries in a listener panics: %s", message)
		}
		if !snapshotTree(w).equal(before) {
			stale++
		}
	})
	simulate(w, 1, nil)
	if calls == 0 {
		t.Fatal("no collision event: the listener never ran")
	}
	if stale > 0 {
		t.Errorf("%d times, a listener saw a tree which was not the tree of the end of the step", stale)
	}
}

// A query on a world which never ran Step nor SyncQueries reads trees nobody built yet: it finds nothing, as on an
// empty world, and the bodies just added are seen after SyncQueries, as any body added between 2 steps
func TestQueriesBeforeTheFirstSync(t *testing.T) {
	origin, down := mgl64.Vec3{0, 10, 0}, mgl64.Vec3{0, -20, 0}
	ball := &actor.Sphere{Radius: 0.5}
	above := actor.Transform{Position: origin, Rotation: mgl64.QuatIdent()}
	filter := DefaultQueryFilter()
	// seen: the bodies hit by a ray, by all the hits of a ray, by a sweep, and overlapped at the height of the crates
	seen := func(w *World) [4]int {
		var counts [4]int
		if _, ok := w.Raycast(origin, down, filter); ok {
			counts[0] = 1
		}
		counts[1] = len(w.RaycastAll(origin, down, filter, nil))
		if _, ok := w.Sweep(ball, above, down, filter); ok {
			counts[2] = 1
		}
		at := actor.Transform{Position: mgl64.Vec3{0, cubeHalf, 0}, Rotation: mgl64.QuatIdent()}
		counts[3] = len(w.Overlap(ball, at, filter, nil))
		return counts
	}

	empty := newScene(1)
	if counts := seen(empty); counts != [4]int{} {
		t.Errorf("an empty world: %v bodies seen by Raycast, RaycastAll, Sweep and Overlap, 0 expected", counts)
	}
	empty.SyncQueries()
	if counts := seen(empty); counts != [4]int{} {
		t.Errorf("an empty world after SyncQueries: %v bodies seen, 0 expected", counts)
	}

	w := newScene(1)
	addGround(w, 0.6)
	addBody(w, mgl64.Vec3{0, 3, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
	addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	if counts := seen(w); counts != [4]int{} {
		t.Errorf("bodies added, before SyncQueries: %v bodies seen by Raycast, RaycastAll, Sweep and Overlap, 0 expected", counts)
	}
	w.SyncQueries()
	// the ground, the static crate and the dynamic one are on the ray; the ball overlaps the ground and the low crate
	if counts := seen(w); counts != [4]int{1, 3, 1, 2} {
		t.Errorf("after SyncQueries: %v bodies seen by Raycast, RaycastAll, Sweep and Overlap, [1 3 1 2] expected", counts)
	}

	// a body removed before the first sync was never in the trees
	late := newScene(1)
	addBody(late, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	late.RemoveBody(late.Bodies[0])
	if counts := seen(late); counts != [4]int{} {
		t.Errorf("a body added and removed: %v bodies seen, 0 expected", counts)
	}
}
