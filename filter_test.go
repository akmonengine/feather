package feather

import (
	"math/rand"
	"slices"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== COLLISION FILTERING ==========

const (
	layerDecor  actor.Layers = 1 << 1
	layerPawn   actor.Layers = 1 << 2
	layerDebris actor.Layers = 1 << 3

	// the half thickness of the floors, and the radius of the balls
	floorHalf  = 0.25
	ballRadius = 0.25
	// restingHeight of a ball on a floor at the origin
	restingHeight = floorHalf + ballRadius
)

// addFloor adds a static box, its center at the height
func addFloor(w *World, height float64) *actor.RigidBody {
	return addBody(w, mgl64.Vec3{0, height, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{2, floorHalf, 2}}, actor.BodyTypeStatic, 0.5, 0)
}

func addBall(w *World, position mgl64.Vec3) *actor.RigidBody {
	return addBody(w, position, mgl64.QuatIdent(), &actor.Sphere{Radius: ballRadius}, actor.BodyTypeDynamic, 0.5, 0)
}

// rests: the ball lies on a floor whose center is at the height
func rests(ball *actor.RigidBody, height float64) bool {
	y := ball.Transform.Position.Y() - height
	return y > restingHeight-2*LinearSlop && y < restingHeight+2*LinearSlop
}

// A body created by NewRigidBody collides with everything
func TestDefaultFilterCollidesWithEverything(t *testing.T) {
	a := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	b := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeStatic, 0)
	for _, body := range []*actor.RigidBody{a, b} {
		if body.Layer != actor.LayerDefault || body.Mask != actor.AllLayers {
			t.Errorf("layer %#x mask %#x, want %#x and %#x", body.Layer, body.Mask, actor.LayerDefault, actor.AllLayers)
		}
	}
	if !ShouldCollide(a, b) {
		t.Error("2 default bodies must collide")
	}
	// the 32 layers are in the default mask
	for bit := 0; bit < 32; bit++ {
		a.Layer = 1 << bit
		if !ShouldCollide(a, b) {
			t.Errorf("the layer %d is not in the default mask", bit)
		}
	}
}

// 2 bodies collide if each one has its layer in the mask of the other
func TestShouldCollide(t *testing.T) {
	cases := []struct {
		name                         string
		layerA, maskA, layerB, maskB actor.Layers
		want                         bool
	}{
		{"both masks accept", layerPawn, layerDecor, layerDecor, layerPawn, true},
		{"several layers in the masks", layerPawn, layerDecor | layerDebris, layerDebris, layerPawn | layerDecor, true},
		{"only A accepts", layerPawn, layerDecor, layerDecor, layerDebris, false},
		{"only B accepts", layerPawn, layerDebris, layerDecor, layerPawn, false},
		{"none accepts", layerPawn, layerPawn, layerDecor, layerDecor, false},
		{"empty mask of A", layerPawn, actor.NoLayers, layerDecor, actor.AllLayers, false},
		{"empty mask of B", layerPawn, actor.AllLayers, layerDecor, actor.NoLayers, false},
	}
	for _, c := range cases {
		a := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
		b := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
		a.Layer, a.Mask, b.Layer, b.Mask = c.layerA, c.maskA, c.layerB, c.maskB
		if got := ShouldCollide(a, b); got != c.want {
			t.Errorf("%s: %v, want %v", c.name, got, c.want)
		}
		if got := ShouldCollide(b, a); got != c.want {
			t.Errorf("%s, reversed: %v, want %v", c.name, got, c.want)
		}
	}
}

// A body of layer 0 is on no layer: it collides with nothing, whatever the masks, and no query sees it. A ball of layer
// 0 goes through the floor without any pair
func TestBodyWithoutLayer(t *testing.T) {
	w := newScene(1)
	floor := addFloor(w, 0)
	ball := addBall(w, mgl64.Vec3{0, 1, 0})
	ball.Layer = actor.NoLayers
	if ShouldCollide(ball, floor) || ShouldCollide(floor, ball) || w.ShouldCollide(ball, floor) {
		t.Error("a body of layer 0 collides with a body of full mask")
	}
	if (QueryFilter{Mask: actor.AllLayers}).Accepts(ball) {
		t.Error("a query of full mask sees a body of layer 0")
	}
	pairs := 0
	simulate(w, 1, func() { pairs += len(w.pairs) })
	if ball.Transform.Position.Y() > -1 || pairs > 0 {
		t.Errorf("the ball of layer 0 is at y=%.3f with %d pairs, want it through the floor without any", ball.Transform.Position.Y(), pairs)
	}
}

// A layer of several bits: the body is on each of these layers. One of them in the mask of the other body is enough to
// collide, and one of them in the mask of a query is enough to be seen (as the categoryBits of Box2D)
func TestLayerOfSeveralBits(t *testing.T) {
	both := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	both.Layer = layerPawn | layerDebris
	other := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	cases := []struct {
		name string
		mask actor.Layers
		want bool
	}{
		{"the first layer", layerPawn, true},
		{"the second layer", layerDebris, true},
		{"both layers", layerPawn | layerDebris, true},
		{"another layer", layerDecor, false},
	}
	for _, c := range cases {
		other.Mask = c.mask
		if got := ShouldCollide(both, other); got != c.want {
			t.Errorf("%s in the mask of the other body: collides %v, want %v", c.name, got, c.want)
		}
		if got := ShouldCollide(other, both); got != c.want {
			t.Errorf("%s in the mask of the other body, reversed: collides %v, want %v", c.name, got, c.want)
		}
		if got := (QueryFilter{Mask: c.mask}).Accepts(both); got != c.want {
			t.Errorf("%s in the mask of the query: accepted %v, want %v", c.name, got, c.want)
		}
	}

	// in a world: the ball of 2 layers rests on a floor which accepts only one of them
	w := newScene(1)
	floor := addFloor(w, 0)
	ball := addBall(w, mgl64.Vec3{0, 1, 0})
	ball.Layer, floor.Mask = layerPawn|layerDebris, layerDebris
	simulate(w, 1, nil)
	if !rests(ball, 0) {
		t.Errorf("the ball of 2 layers is at y=%.3f, want it resting at %.3f", ball.Transform.Position.Y(), restingHeight)
	}
}

// Criterion 1: a ball falls on a floor. With compatible layers it rests on it, else it goes through
func TestLayersFilterCollisions(t *testing.T) {
	cases := []struct {
		name                  string
		floorLayer, floorMask actor.Layers
		ballLayer, ballMask   actor.Layers
		collide               bool
	}{
		{"default", actor.LayerDefault, actor.AllLayers, actor.LayerDefault, actor.AllLayers, true},
		{"compatible layers", layerDecor, layerPawn, layerPawn, layerDecor, true},
		{"the ball doesn't accept the floor", layerDecor, actor.AllLayers, layerPawn, actor.AllLayers &^ layerDecor, false},
		{"the floor doesn't accept the ball", layerDecor, layerDebris, layerPawn, actor.AllLayers, false},
	}
	for _, c := range cases {
		w := newScene(1)
		floor := addFloor(w, 0)
		ball := addBall(w, mgl64.Vec3{0, 1, 0})
		floor.Layer, floor.Mask = c.floorLayer, c.floorMask
		ball.Layer, ball.Mask = c.ballLayer, c.ballMask
		contacts := 0
		simulate(w, 1, func() { contacts += len(w.Contacts()) })
		if c.collide && !rests(ball, 0) {
			t.Errorf("%s: the ball is at y=%.3f, want it resting at %.3f", c.name, ball.Transform.Position.Y(), restingHeight)
		}
		if !c.collide && (ball.Transform.Position.Y() > -1 || contacts > 0) {
			t.Errorf("%s: the ball is at y=%.3f with %d contacts, want it through the floor without any", c.name, ball.Transform.Position.Y(), contacts)
		}
	}
}

// Criterion 2: an ignored pair goes through, the other pairs of both bodies collide. The pair collides again once it is
// not ignored anymore
func TestIgnoredPairGoesThrough(t *testing.T) {
	w := newScene(1)
	const lower = -3.0
	floor := addFloor(w, 0)
	addFloor(w, lower)
	ignored := addBall(w, mgl64.Vec3{-0.8, 1, 0})
	other := addBall(w, mgl64.Vec3{0.8, 1, 0})
	w.IgnoreCollision(ignored, floor, true)
	if w.ShouldCollide(ignored, floor) || w.ShouldCollide(floor, ignored) {
		t.Error("the ignored pair should not collide")
	}
	if !w.ShouldCollide(other, floor) || !w.ShouldCollide(ignored, other) {
		t.Error("the other pairs should collide")
	}

	simulate(w, 2, nil)
	if !rests(ignored, lower) {
		t.Errorf("the ignored ball is at y=%.3f, want it resting on the lower floor at %.3f", ignored.Transform.Position.Y(), lower+restingHeight)
	}
	if !rests(other, 0) {
		t.Errorf("the other ball is at y=%.3f, want it resting on the floor at %.3f", other.Transform.Position.Y(), restingHeight)
	}

	// ignoring twice then restoring once: the pair is a set, not a count
	w.IgnoreCollision(floor, ignored, true)
	w.IgnoreCollision(floor, ignored, false)
	if !w.ShouldCollide(ignored, floor) {
		t.Error("the restored pair should collide")
	}
	ignored.Transform.Position = mgl64.Vec3{-0.8, 1, 0}
	ignored.Velocity = mgl64.Vec3{}
	simulate(w, 2, nil)
	if !rests(ignored, 0) {
		t.Errorf("the restored ball is at y=%.3f, want it resting on the floor at %.3f", ignored.Transform.Position.Y(), restingHeight)
	}
}

// filteredScene: overlapping bodies of 3 layers over a plane (the debris don't collide with each other, the decor
// collides with nothing), with ignored pairs and a joint. Returns the pairs which must not be emitted
func filteredScene(workers int) (*World, map[pairKey]bool) {
	w := newScene(workers)
	w.parallelFrom = 1
	w.Gravity = mgl64.Vec3{}
	addGround(w, 0.6)
	r := rand.New(rand.NewSource(7))
	for i := 0; i < 120; i++ {
		bodyType := actor.BodyTypeDynamic
		if i%7 == 6 {
			bodyType = actor.BodyTypeStatic
		}
		body := addBody(w, mgl64.Vec3{r.Float64()*3 - 1.5, 0.3 + r.Float64()*2, r.Float64()*3 - 1.5}, mgl64.QuatIdent(), cube(), bodyType, 0.6, 0)
		switch i % 3 {
		case 0:
			body.Layer, body.Mask = layerPawn, actor.AllLayers
		case 1:
			body.Layer, body.Mask = layerDebris, actor.AllLayers&^layerDebris
		default:
			body.Layer, body.Mask = layerDecor, actor.NoLayers
		}
	}
	// the pawns ignore the plane and their next pawn, the first 2 pawns are linked
	excluded := map[pairKey]bool{}
	pawns := []*actor.RigidBody{}
	for i, body := range w.Bodies {
		if body.Layer == layerPawn && i%2 == 0 {
			pawns = append(pawns, body)
		}
	}
	for i := 0; i+1 < len(pawns); i++ {
		w.IgnoreCollision(pawns[i], pawns[i+1], true)
		excluded[makePairKey(pawns[i], pawns[i+1])] = true
		w.IgnoreCollision(pawns[i], w.Bodies[0], true)
		excluded[makePairKey(pawns[i], w.Bodies[0])] = true
	}
	linked := []*actor.RigidBody{}
	for _, body := range w.Bodies {
		if body.Layer == layerPawn && body.BodyType == actor.BodyTypeDynamic && !slices.Contains(pawns, body) {
			linked = append(linked, body)
		}
	}
	w.AddJoint(NewDistanceJoint(linked[0], linked[1], linked[0].Transform.Position, linked[1].Transform.Position))
	excluded[makePairKey(linked[0], linked[1])] = true
	return w, excluded
}

// Criterion 3: the broad phase emits no filtered pair. Its pairs are exactly those of the brute force which pass the
// filter (layers, ignored pairs, linked bodies), in the same order, with 1 and 8 workers
func TestBroadPhaseEmitsNoFilteredPair(t *testing.T) {
	for _, workers := range []int{1, 8} {
		w, excluded := filteredScene(workers)
		layersAllow := func(a, b *actor.RigidBody) bool {
			return a.Mask&b.Layer != 0 && b.Mask&a.Layer != 0
		}
		for step := 0; step < 30; step++ {
			w.Step(sceneDt)
			pool := w.workerPool()
			pool.begin(workers)
			got := w.tree.findPairs(w.Bodies, w.aabbs, pool)
			pool.end()

			all := bruteForcePairs(w.Bodies, w.aabbs)
			var want []Pair
			for _, pair := range all {
				if layersAllow(pair.BodyA, pair.BodyB) && !excluded[makePairKey(pair.BodyA, pair.BodyB)] {
					want = append(want, pair)
				}
			}
			if len(want) == 0 || len(want) > len(all)/2 {
				t.Fatalf("workers %d: %d pairs kept of %d, the scene doesn't filter enough", workers, len(want), len(all))
			}
			samePairs(t, got, want, "filtered pairs")
			for _, pair := range got {
				if pair.BodyA.Mask == actor.NoLayers || pair.BodyB.Mask == actor.NoLayers {
					t.Fatalf("workers %d: a pair with a body of empty mask", workers)
				}
			}
		}
	}
}

// The exported BroadPhase has no world: it filters by the layers of the bodies
func TestBroadPhaseFunctionFiltersLayers(t *testing.T) {
	a := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	b := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	c := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	bodies := []*actor.RigidBody{a, b, c}
	if pairs := BroadPhase(bodies, 1); len(pairs) != 3 {
		t.Fatalf("%d pairs with the default filter, want 3", len(pairs))
	}
	c.Mask = actor.NoLayers
	pairs := BroadPhase(bodies, 1)
	if len(pairs) != 1 || pairs[0].BodyA != a || pairs[0].BodyB != b {
		t.Errorf("%d pairs with a body of empty mask, want the pair of the 2 others", len(pairs))
	}
}

// Criterion 4: a step with layers, ignored pairs and linked bodies doesn't allocate
func TestFilteredStepDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 8} {
		w := benchScene(500, workers)
		for i, body := range w.Bodies[1:] {
			switch i % 3 {
			case 0:
				body.Layer, body.Mask = layerPawn, actor.AllLayers
			case 1:
				body.Layer, body.Mask = layerDebris, actor.AllLayers&^layerDebris
			}
			if i%5 == 0 && i > 0 {
				w.IgnoreCollision(body, w.Bodies[i], true)
			}
		}
		hangingChain(w, 10)
		simulate(w, 0.4, nil)
		if len(w.Contacts()) == 0 {
			t.Fatal("no contact, the scene doesn't cover the narrow phase")
		}
		allocs := testing.AllocsPerRun(10, func() {
			w.Step(sceneDt)
		})
		if allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
	}
}

// "Queries only": a body of empty mask makes no pair, no contact and no event, but stays in the tree of the broad phase
// with its layer, where a query filter finds it
func TestQueryOnlyBodyMakesNoPair(t *testing.T) {
	w := newScene(1)
	decor := addFloor(w, 0)
	decor.Layer, decor.Mask = layerDecor, actor.NoLayers
	trigger := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 1}, actor.BodyTypeStatic, 0, 0)
	trigger.IsTrigger, trigger.Mask = true, actor.NoLayers
	ball := addBall(w, mgl64.Vec3{0, 1, 0})
	events := 0
	for _, eventType := range []EventType{EventCollisionEnter, EventTriggerEnter, EventCollisionStay, EventTriggerStay} {
		w.Events.Subscribe(eventType, func(Event) { events++ })
	}

	pairs, contacts := 0, 0
	simulate(w, 1, func() {
		pairs += len(w.pairs)
		contacts += len(w.Contacts())
	})
	if pairs != 0 || contacts != 0 || events != 0 {
		t.Errorf("%d pairs, %d contacts, %d events, want none", pairs, contacts, events)
	}
	if ball.Transform.Position.Y() > -1 {
		t.Errorf("the ball is at y=%.3f, want it through the decor", ball.Transform.Position.Y())
	}

	// the decor is still a leaf of the static tree, at its index
	_, candidates := w.tree.queryCandidates(decor.AABB(), nil, nil)
	if !slices.Contains(candidates, int32(slices.Index(w.Bodies, decor))) {
		t.Error("the decor is not in the tree of the broad phase")
	}
	if filter := (QueryFilter{Mask: layerDecor}); !filter.Accepts(decor) {
		t.Error("a query on the layer of the decor must see it")
	}
}

// "Queries only": a body of empty mask wakes nobody up, neither by moving through a sleeping body nor when it is removed
func TestQueryOnlyBodyWakesNobody(t *testing.T) {
	w := newScene(1)
	addGround(w, 0.6)
	sleeper := addBody(w, mgl64.Vec3{0, cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 2, nil)
	if !sleeper.IsSleeping {
		t.Fatal("the box doesn't sleep after 2 s")
	}
	position := sleeper.Transform.Position

	// a ghost falls through the sleeping box
	ghost := addBall(w, mgl64.Vec3{0, 2, 0})
	ghost.Mask = actor.NoLayers
	simulate(w, 1.5, func() {
		if !sleeper.IsSleeping {
			t.Fatal("the ghost woke the sleeping box up")
		}
	})
	if ghost.Transform.Position.Y() > -1 {
		t.Fatalf("the ghost is at y=%.3f, want it through the box and the ground", ghost.Transform.Position.Y())
	}

	// a decor around the sleeping box is added then removed
	decor := addBody(w, position, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{1, 1, 1}}, actor.BodyTypeStatic, 0.6, 0)
	decor.Layer, decor.Mask = layerDecor, actor.NoLayers
	simulate(w, 0.5, nil)
	w.RemoveBody(decor)
	simulate(w, 0.5, nil)
	if !sleeper.IsSleeping || sleeper.Transform.Position != position {
		t.Errorf("the decor woke the sleeping box up (sleeping %v, moved by %v)", sleeper.IsSleeping, sleeper.Transform.Position.Sub(position))
	}
}

// contactsBetween counts the contacts of the pair in the last step
func contactsBetween(w *World, a, b *actor.RigidBody) int {
	count := 0
	for _, contact := range w.Contacts() {
		if (contact.BodyA == a && contact.BodyB == b) || (contact.BodyA == b && contact.BodyB == a) {
			count++
		}
	}
	return count
}

// overlappingCubes: 2 dynamic cubes overlapping, without gravity
func overlappingCubes() (*World, *actor.RigidBody, *actor.RigidBody) {
	w := newScene(1)
	w.Gravity = mgl64.Vec3{}
	a := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	b := addBody(w, mgl64.Vec3{0.4, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	return w, a, b
}

// The bodies linked by a joint are filtered in the broad phase like an ignored pair, in the same table: the pair comes
// back when its last joint leaves, unless it is ignored too
func TestLinkedBodiesAreFilteredInTheBroadPhase(t *testing.T) {
	w, a, b := overlappingCubes()
	first := NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position)
	second := NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position)
	w.AddJoint(first)
	w.AddJoint(second)
	w.Step(sceneDt)
	if len(w.pairs) != 0 || w.ShouldCollide(a, b) {
		t.Errorf("%d pairs between linked bodies, want none", len(w.pairs))
	}

	w.RemoveJoint(first)
	w.Step(sceneDt)
	if len(w.pairs) != 0 {
		t.Errorf("%d pairs with one joint left, want none", len(w.pairs))
	}

	// the pair is ignored too: the last joint leaves, the pair stays filtered
	w.IgnoreCollision(a, b, true)
	w.RemoveJoint(second)
	w.Step(sceneDt)
	if len(w.pairs) != 0 {
		t.Errorf("%d pairs for an ignored pair without joint, want none", len(w.pairs))
	}

	w.IgnoreCollision(a, b, false)
	w.Step(sceneDt)
	if len(w.pairs) != 1 || contactsBetween(w, a, b) == 0 || !w.ShouldCollide(a, b) {
		t.Errorf("%d pairs once the pair is free, want 1 with a contact", len(w.pairs))
	}
}

// A pair linked by a joint, ignored then restored, stays filtered by its joint: the game restores what it ignored, not
// what the joint filters. The pair comes back when the joint leaves
func TestLinkedPairIgnoredThenRestoredStaysFiltered(t *testing.T) {
	w, a, b := overlappingCubes()
	joint := NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position)
	w.AddJoint(joint)
	w.IgnoreCollision(a, b, true)
	w.IgnoreCollision(a, b, false)
	w.Step(sceneDt)
	if len(w.pairs) != 0 || w.ShouldCollide(a, b) {
		t.Fatalf("%d pairs between linked bodies once the pair is restored, want none", len(w.pairs))
	}

	w.RemoveJoint(joint)
	w.Step(sceneDt)
	if len(w.pairs) != 1 || contactsBetween(w, a, b) == 0 || !w.ShouldCollide(a, b) {
		t.Errorf("%d pairs once the joint is removed, want 1 with a contact", len(w.pairs))
	}
}

// A joint with CollideConnected doesn't filter its bodies, and doesn't free a pair ignored by another joint. A joint
// whose CollideConnected changed after AddJoint leaves the table as it entered it
func TestCollideConnectedIsReadWhenTheJointIsAdded(t *testing.T) {
	w, a, b := overlappingCubes()
	filtering := NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position)
	colliding := NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position)
	colliding.CollideConnected = true
	w.AddJoint(filtering)
	w.AddJoint(colliding)
	w.RemoveJoint(colliding)
	if w.ShouldCollide(a, b) {
		t.Error("the removal of a colliding joint freed the pair of a filtering joint")
	}

	filtering.CollideConnected = true
	w.RemoveJoint(filtering)
	if !w.ShouldCollide(a, b) {
		t.Error("the pair is still filtered after the removal of its joint")
	}
	w.Step(sceneDt)
	if contactsBetween(w, a, b) == 0 {
		t.Error("no contact once the joint is removed")
	}
}

// A removed body leaves the table of the ignored pairs
func TestRemovedBodyForgetsItsIgnoredPairs(t *testing.T) {
	w, a, b := overlappingCubes()
	w.IgnoreCollision(a, b, true)
	w.AddJoint(NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position))
	w.Step(sceneDt)
	w.RemoveBody(a)
	w.AddBody(a)
	if !w.ShouldCollide(a, b) {
		t.Fatal("the pair of a removed body is still ignored")
	}
	w.Step(sceneDt)
	if contactsBetween(w, a, b) == 0 {
		t.Error("no contact with the body added again")
	}

	// the other body of the pair, and only its pairs
	c := addBody(w, mgl64.Vec3{0, 5, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.5, 0)
	w.IgnoreCollision(a, b, true)
	w.IgnoreCollision(a, c, true)
	w.RemoveBody(b)
	w.AddBody(b)
	if !w.ShouldCollide(a, b) {
		t.Error("the pair of the second removed body is still ignored")
	}
	if w.ShouldCollide(a, c) {
		t.Error("the pair of 2 bodies still in the world was forgotten")
	}
}

// A sleeping body which ignores a body (an ignored pair, not the layers) stays asleep when this body is removed: the
// table is read before the pairs of the removed body are forgotten. A body which collides with it wakes up
func TestRemovedBodyWakesOnlyTheBodiesItCollidesWith(t *testing.T) {
	for _, ignored := range []bool{true, false} {
		w := newScene(1)
		addFloor(w, 0)
		box := addBody(w, mgl64.Vec3{0, floorHalf + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		// a static body around the box, which doesn't collide with it while it falls asleep
		around := addBody(w, box.Transform.Position, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
		if ignored {
			w.IgnoreCollision(box, around, true)
		} else {
			around.Mask = actor.NoLayers
		}
		simulate(w, 2, nil)
		if !box.IsSleeping {
			t.Fatalf("ignored %v: the box doesn't sleep after 2 s", ignored)
		}
		around.Mask = actor.AllLayers
		w.RemoveBody(around)
		if box.IsSleeping != ignored {
			t.Errorf("ignored %v: the box is sleeping %v after the removal of the body around it", ignored, box.IsSleeping)
		}
		if w.AddBody(around); !w.ShouldCollide(box, around) {
			t.Errorf("ignored %v: the pair of the removed body is still ignored", ignored)
		}
	}
}

// The triggers go through the same filter: a filtered pair sends no trigger event
func TestTriggerRespectsFilter(t *testing.T) {
	cases := []struct {
		name   string
		filter func(w *World, trigger, body *actor.RigidBody)
		events int
	}{
		{"default", func(*World, *actor.RigidBody, *actor.RigidBody) {}, 1},
		{"layers", func(_ *World, trigger, body *actor.RigidBody) {
			body.Layer, trigger.Mask = layerPawn, actor.AllLayers&^layerPawn
		}, 0},
		{"ignored pair", func(w *World, trigger, body *actor.RigidBody) { w.IgnoreCollision(trigger, body, true) }, 0},
	}
	for _, c := range cases {
		w, trigger, body := overlappingCubes()
		trigger.IsTrigger = true
		c.filter(w, trigger, body)
		events := 0
		w.Events.Subscribe(EventTriggerEnter, func(Event) { events++ })
		w.Step(sceneDt)
		if events != c.events {
			t.Errorf("%s: %d trigger events, want %d", c.name, events, c.events)
		}
	}
}

// The continuous collision goes through the same filter: a fast body is not stopped by a body it doesn't collide with
func TestContinuousRespectsFilter(t *testing.T) {
	cases := []struct {
		name    string
		filter  func(w *World, ball, wall *actor.RigidBody)
		stopped bool
	}{
		{"default", func(*World, *actor.RigidBody, *actor.RigidBody) {}, true},
		{"layers", func(_ *World, ball, wall *actor.RigidBody) {
			wall.Layer, ball.Mask = layerDecor, actor.AllLayers&^layerDecor
		}, false},
		{"empty mask of the wall", func(_ *World, _, wall *actor.RigidBody) { wall.Mask = actor.NoLayers }, false},
		{"ignored pair", func(w *World, ball, wall *actor.RigidBody) { w.IgnoreCollision(ball, wall, true) }, false},
		{"linked bodies", func(w *World, ball, wall *actor.RigidBody) {
			w.AddJoint(NewDistanceJoint(ball, wall, ball.Transform.Position, wall.Transform.Position))
		}, false},
	}
	for _, c := range cases {
		w := newScene(1)
		w.Gravity = mgl64.Vec3{}
		wall := addBody(w, mgl64.Vec3{5, 0, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.01, 1, 1}}, actor.BodyTypeStatic, 0.5, 0)
		ball := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.02}, actor.BodyTypeDynamic, 0.5, 0)
		c.filter(w, ball, wall)
		w.Step(sceneDt)

		// a motion through the wall, as if the solver had accelerated the ball
		moveThrough(w, ball, sweep{start: actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()}, end: actor.Transform{Position: mgl64.Vec3{10, 0, 0}, Rotation: mgl64.QuatIdent()}})
		if stopped := ball.Transform.Position.X() < 5; stopped != c.stopped {
			t.Errorf("%s: the ball is at x=%.3f, stopped %v, want %v", c.name, ball.Transform.Position.X(), stopped, c.stopped)
		}
	}
}

// A sleeping body resting on a body wakes up and falls when a change of filter removes its support
func TestFilterChangeWakesTheRestingBodies(t *testing.T) {
	cases := []struct {
		name   string
		change func(w *World, platform, box *actor.RigidBody)
	}{
		{"SetFilter of the platform", func(w *World, platform, _ *actor.RigidBody) { w.SetFilter(platform, layerDecor, actor.NoLayers) }},
		{"SetFilter of the box", func(w *World, _, box *actor.RigidBody) { w.SetFilter(box, layerPawn, actor.NoLayers) }},
		// the box doesn't collide with its own layer: it is not one of its neighbors
		{"SetFilter of a box out of its own mask", func(w *World, _, box *actor.RigidBody) {
			w.SetFilter(box, layerPawn, actor.AllLayers&^layerPawn)
			simulate(w, 2, nil)
			w.SetFilter(box, layerPawn, actor.NoLayers)
		}},
		{"IgnoreCollision", func(w *World, platform, box *actor.RigidBody) { w.IgnoreCollision(platform, box, true) }},
		{"IgnoreCollision, reversed", func(w *World, platform, box *actor.RigidBody) { w.IgnoreCollision(box, platform, true) }},
	}
	for _, c := range cases {
		w := newScene(1)
		platform := addFloor(w, 0)
		box := addBody(w, mgl64.Vec3{0, floorHalf + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		simulate(w, 2, nil)
		if !box.IsSleeping {
			t.Fatalf("%s: the box doesn't sleep after 2 s", c.name)
		}
		c.change(w, platform, box)
		simulate(w, 1, nil)
		if box.Transform.Position.Y() > -1 {
			t.Errorf("%s: the box is at y=%.3f (sleeping %v), want it fallen through the platform", c.name, box.Transform.Position.Y(), box.IsSleeping)
		}
	}
}

// SetFilter writes the layer and the mask of the body
func TestSetFilter(t *testing.T) {
	w, a, b := overlappingCubes()
	w.SetFilter(a, layerPawn, layerDecor)
	if a.Layer != layerPawn || a.Mask != layerDecor {
		t.Errorf("layer %#x mask %#x, want %#x and %#x", a.Layer, a.Mask, layerPawn, layerDecor)
	}
	if w.ShouldCollide(a, b) {
		t.Error("the bodies should not collide anymore")
	}
}

// The filter of the queries: the layers of its mask, without the excluded bodies
func TestQueryFilter(t *testing.T) {
	pawn := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	decor := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeStatic, 0)
	ghost := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeStatic, 0)
	pawn.Layer, decor.Layer = layerPawn, layerDecor
	ghost.Layer, ghost.Mask = layerDecor, actor.NoLayers

	cases := []struct {
		name   string
		filter QueryFilter
		want   [3]bool // pawn, decor, ghost
	}{
		{"all layers", QueryFilter{Mask: actor.AllLayers}, [3]bool{true, true, true}},
		{"one layer", QueryFilter{Mask: layerDecor}, [3]bool{false, true, true}},
		{"two layers", QueryFilter{Mask: layerDecor | layerPawn}, [3]bool{true, true, true}},
		{"another layer", QueryFilter{Mask: layerDebris}, [3]bool{false, false, false}},
		{"excluded body", QueryFilter{Mask: actor.AllLayers, Excluded: []*actor.RigidBody{pawn}}, [3]bool{false, true, true}},
		{"excluded bodies", QueryFilter{Mask: actor.AllLayers, Excluded: []*actor.RigidBody{decor, ghost}}, [3]bool{true, false, false}},
		{"empty mask", QueryFilter{}, [3]bool{false, false, false}},
	}
	for _, c := range cases {
		for i, body := range []*actor.RigidBody{pawn, decor, ghost} {
			if got := c.filter.Accepts(body); got != c.want[i] {
				t.Errorf("%s: body %d accepted %v, want %v", c.name, i, got, c.want[i])
			}
		}
	}
}

// The test of a query filter doesn't allocate
func TestQueryFilterDoesNotAllocate(t *testing.T) {
	a := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	b := actor.NewRigidBody(actor.NewTransform(), cube(), actor.BodyTypeDynamic, 500)
	filter := QueryFilter{Mask: actor.AllLayers, Excluded: []*actor.RigidBody{a}}
	accepted := 0
	allocs := testing.AllocsPerRun(100, func() {
		if filter.Accepts(a) {
			accepted++
		}
		if filter.Accepts(b) {
			accepted++
		}
	})
	if allocs > 0 || accepted != 101 {
		t.Errorf("%.1f allocations, %d bodies accepted, want 0 and 101", allocs, accepted)
	}
}

// Ignoring a pair already ignored (or restoring a pair which is not) changes nothing: nobody wakes up
func TestIgnoreCollisionWithoutChangeWakesNobody(t *testing.T) {
	w := newScene(1)
	platform := addFloor(w, 0)
	box := addBody(w, mgl64.Vec3{0, floorHalf + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	far := addBody(w, mgl64.Vec3{50, 0, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeStatic, 0.6, 0)
	w.IgnoreCollision(box, far, true)
	simulate(w, 2, nil)
	if !box.IsSleeping {
		t.Fatal("the box doesn't sleep after 2 s")
	}
	w.IgnoreCollision(box, far, true)
	w.IgnoreCollision(box, platform, false)
	if !box.IsSleeping {
		t.Error("a call which changes nothing woke the box up")
	}
}

// SetFilter with the layer and the mask the body already has changes nothing: nobody wakes up, neither the body nor the
// sleeping bodies it collides with. A game which sets the filter at each frame lets its bodies sleep
func TestSetFilterWithoutChangeWakesNobody(t *testing.T) {
	w := newScene(1)
	platform := addFloor(w, 0)
	box := addBody(w, mgl64.Vec3{0, floorHalf + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	top := addBody(w, mgl64.Vec3{0, floorHalf + 3*cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
	simulate(w, 2, nil)
	if !box.IsSleeping || !top.IsSleeping {
		t.Fatal("the boxes don't sleep after 2 s")
	}
	for _, body := range []*actor.RigidBody{platform, box, top} {
		w.SetFilter(body, body.Layer, body.Mask)
		if !box.IsSleeping || !top.IsSleeping {
			t.Fatalf("a call which changes nothing woke the boxes up (sleeping %v and %v)", box.IsSleeping, top.IsSleeping)
		}
	}
	// a change of the layer alone, or of the mask alone, is a change
	w.SetFilter(box, layerPawn, box.Mask)
	if box.IsSleeping || top.IsSleeping {
		t.Errorf("a change of the layer left the boxes asleep (sleeping %v and %v)", box.IsSleeping, top.IsSleeping)
	}
	simulate(w, 2, nil)
	if !box.IsSleeping || !top.IsSleeping {
		t.Fatal("the boxes don't sleep again after 2 s")
	}
	w.SetFilter(box, box.Layer, actor.AllLayers&^layerDecor)
	if box.IsSleeping || top.IsSleeping {
		t.Errorf("a change of the mask left the boxes asleep (sleeping %v and %v)", box.IsSleeping, top.IsSleeping)
	}
}

// A sleeping body over a terrain it doesn't collide with (by its layers or by an ignored pair) stays asleep when the
// terrain changes under it
func TestHeightfieldUpdateWakesOnlyTheBodiesItCollidesWith(t *testing.T) {
	cases := []struct {
		name   string
		filter func(w *World, terrain, box *actor.RigidBody)
		wakes  bool
	}{
		{"colliding", func(*World, *actor.RigidBody, *actor.RigidBody) {}, true},
		{"filtered by the layers", func(_ *World, _, box *actor.RigidBody) { box.Mask = actor.AllLayers &^ layerDecor }, false},
		{"ignored pair", func(w *World, terrain, box *actor.RigidBody) { w.IgnoreCollision(terrain, box, true) }, false},
	}
	for _, c := range cases {
		w := newScene(1)
		const samples = 9
		field := actor.NewHeightfield(samples, samples, make([]float32, samples*samples), mgl64.Vec3{1, 1, 1})
		terrain := addBody(w, mgl64.Vec3{}, mgl64.QuatIdent(), field, actor.BodyTypeStatic, 0.6, 0)
		terrain.Layer = layerDecor
		addFloor(w, 1)
		box := addBody(w, mgl64.Vec3{0, 1 + floorHalf + cubeHalf, 0}, mgl64.QuatIdent(), cube(), actor.BodyTypeDynamic, 0.6, 0)
		c.filter(w, terrain, box)
		simulate(w, 2, nil)
		if !box.IsSleeping {
			t.Fatalf("%s: the box doesn't sleep after 2 s", c.name)
		}
		w.UpdateHeightfield(terrain, 3, 3, 5, 5)
		if box.IsSleeping == c.wakes {
			t.Errorf("%s: the box is sleeping %v after the update of the terrain", c.name, box.IsSleeping)
		}
	}
}
