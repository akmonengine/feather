package feather

import (
	"cmp"
	"slices"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== BROAD PHASE ==========
// The broad phase is a pair of dynamic AABB trees (Catto, "Dynamic Bounding Volume Hierarchies", GDC 2019; the
// b2DynamicTree of Box2D, the btDbvt of Bullet), one per kind of body as the trees of Box2D v3: one for the static
// bodies, updated when a body is added, removed or moved by the game, one for the dynamic and the kinematic bodies,
// awake or asleep (Box2D v3 gives the kinematic bodies a third tree, so that a kinematic proxy never queries the static
// tree for its contacts: here a kinematic proxy queries it for the triggers, and its pairs with the static bodies are
// emitted only with a trigger, Tree.scan). A dynamic or kinematic body is stored with its AABB enlarged by
// AABBMargin: a body which moves inside its enlarged AABB doesn't touch the tree, a sleeping body never does. The
// planes and the heightfields are not in the trees: they are tested against every awake dynamic body.
//
// The pairs of overlapping stored AABBs are kept from a step to the next (see findPairs): only a body put back in a
// tree queries it. The pairs of the step are those whose exact AABBs overlap, sorted by the index of the first body,
// the planes of a body before its other pairs.

const (
	// AABBMargin: the AABB of a dynamic body is enlarged by this margin in the tree (m). Larger: fewer updates of the
	// tree, more candidates per query. 0.1 m as Box2D v2.4 (v3 uses 0.05 m)
	AABBMargin = 0.1

	nullNode = -1
)

// Pair of bodies potentially in collision, BodyA of the lower index (a plane first)
type Pair struct {
	BodyA *actor.RigidBody
	BodyB *actor.RigidBody
	// the indices of the bodies in World.Bodies
	IndexA, IndexB int32
	// the sort keys: the index of the body owning the pair, the index of the other body or of the plane
	first, second int32
	plane         bool
	// slot of the pair in the records of the broad phase, for the contacts of the previous step
	slot int32
}

type treeNode struct {
	aabb   actor.AABB
	parent int32
	child1 int32
	child2 int32
	height int32 // 0 for a leaf
	body   int32 // index of the body (leaves)
}

// aabbTree: a binary tree of AABBs, the bodies at its leaves, each node the union of its children. A leaf is inserted
// next to the sibling which enlarges the tree the least (the surface area heuristic: the area of a node is the chance a
// query visits it), found down the tree, then the ancestors are refitted and each of them tries the rotation which
// shrinks it the most (Catto, "Dynamic Bounding Volume Hierarchies", GDC 2019). The heights are kept for the leaves
// (0) and the tests; the balance comes from the areas, not from the heights
type aabbTree struct {
	nodes []treeNode
	root  int32
	free  []int32 // released nodes, reused first
}

func (t *aabbTree) allocate() int32 {
	if n := len(t.free); n > 0 {
		index := t.free[n-1]
		t.free = t.free[:n-1]
		t.nodes[index] = treeNode{parent: nullNode, child1: nullNode, child2: nullNode, body: nullNode}
		return index
	}
	t.nodes = append(t.nodes, treeNode{parent: nullNode, child1: nullNode, child2: nullNode, body: nullNode})
	return int32(len(t.nodes) - 1)
}

func (t *aabbTree) release(n int32) {
	t.free = append(t.free, n)
}

// empty: the tree has no leaf. The zero value of a tree is empty: no node, and a root at 0 until the first sync clears
// it (a World is a literal: a query can come before any Step or SyncQueries)
func (t *aabbTree) empty() bool {
	return t.root == nullNode || len(t.nodes) == 0
}

func (t *aabbTree) clear() {
	t.nodes = t.nodes[:0]
	t.free = t.free[:0]
	t.root = nullNode
}

// insert a leaf for the body with the AABB, returns the node
func (t *aabbTree) insert(aabb actor.AABB, body int32) int32 {
	leaf := t.allocate()
	t.nodes[leaf].aabb = aabb
	t.nodes[leaf].body = body
	t.insertLeaf(leaf)
	return leaf
}

// remove the leaf
func (t *aabbTree) remove(leaf int32) {
	t.removeLeaf(leaf)
	t.release(leaf)
}

// surfaceArea of an AABB: the cost of a node (the probability a query visits it)
func surfaceArea(a actor.AABB) float64 {
	d := a.Max.Sub(a.Min)
	return 2 * (d.X()*d.Y() + d.Y()*d.Z() + d.Z()*d.X())
}

func union(a, b actor.AABB) actor.AABB {
	return actor.AABB{
		Min: mgl64.Vec3{min(a.Min.X(), b.Min.X()), min(a.Min.Y(), b.Min.Y()), min(a.Min.Z(), b.Min.Z())},
		Max: mgl64.Vec3{max(a.Max.X(), b.Max.X()), max(a.Max.Y(), b.Max.Y()), max(a.Max.Z(), b.Max.Z())},
	}
}

func contains(outer, inner actor.AABB) bool {
	return outer.Min.X() <= inner.Min.X() && outer.Min.Y() <= inner.Min.Y() && outer.Min.Z() <= inner.Min.Z() &&
		inner.Max.X() <= outer.Max.X() && inner.Max.Y() <= outer.Max.Y() && inner.Max.Z() <= outer.Max.Z()
}

// insertLeaf under the best sibling, then refits and rotates the ancestors
func (t *aabbTree) insertLeaf(leaf int32) {
	if t.root == nullNode {
		t.root = leaf
		return
	}
	sibling := t.bestSibling(t.nodes[leaf].aabb)

	// a new node takes the place of the sibling, with the sibling and the leaf under it
	above := t.nodes[sibling].parent
	pair := t.allocate()
	t.nodes[pair].parent = above
	t.link(pair, sibling, leaf)
	if above == nullNode {
		t.root = pair
	} else if t.nodes[above].child1 == sibling {
		t.nodes[above].child1 = pair
	} else {
		t.nodes[above].child2 = pair
	}
	// the new node is fitted by link: its ancestors grow
	t.refitUp(above)
}

// bestSibling for a leaf: the node whose pairing with the leaf costs the least, the cost being the area of their union
// plus the growth of every ancestor. The tree is descended greedily: at each node, the leaf stops there if pairing with
// the node beats what any subtree of its children can reach (under a child, the leaf pairs with the child or with a
// node below it, which costs at least the growth of the child plus the area of the leaf), else it goes under the child
// with the better reach. O(log n), the tree quality of the surface area heuristic (Catto, GDC 2019)
func (t *aabbTree) bestSibling(aabb actor.AABB) int32 {
	leafArea := surfaceArea(aabb)
	node, inherited := t.root, 0.0
	for {
		n := &t.nodes[node]
		joined := surfaceArea(union(n.aabb, aabb))
		if n.height == 0 {
			return node
		}
		here := joined + inherited
		growth := inherited + (joined - surfaceArea(n.aabb))
		reach1, reach2 := t.reach(n.child1, aabb, growth, leafArea), t.reach(n.child2, aabb, growth, leafArea)
		if here <= min(reach1, reach2) {
			return node
		}
		if reach1 < reach2 {
			node = n.child1
		} else {
			node = n.child2
		}
		inherited = growth
	}
}

// reach: the least a pairing under the child can cost, the ancestors having grown by growth
func (t *aabbTree) reach(child int32, aabb actor.AABB, growth, leafArea float64) float64 {
	c := &t.nodes[child]
	joined := surfaceArea(union(c.aabb, aabb))
	pairing := joined + growth
	if c.height == 0 {
		return pairing
	}
	return min(pairing, growth+(joined-surfaceArea(c.aabb))+leafArea)
}

// removeLeaf: its sibling takes the place of their parent, the ancestors shrink
func (t *aabbTree) removeLeaf(leaf int32) {
	if leaf == t.root {
		t.root = nullNode
		return
	}
	pair := t.nodes[leaf].parent
	sibling := t.nodes[pair].child1
	if sibling == leaf {
		sibling = t.nodes[pair].child2
	}
	above := t.nodes[pair].parent
	t.nodes[sibling].parent = above
	if above == nullNode {
		t.root = sibling
	} else {
		if t.nodes[above].child1 == pair {
			t.nodes[above].child1 = sibling
		} else {
			t.nodes[above].child2 = sibling
		}
		t.refitUp(above)
	}
	t.release(pair)
}

// link the children to the node, and refit it
func (t *aabbTree) link(node, child1, child2 int32) {
	t.nodes[node].child1, t.nodes[node].child2 = child1, child2
	t.nodes[child1].parent, t.nodes[child2].parent = node, node
	t.refit(node)
}

// refit the node on its children: its AABB and its height
func (t *aabbTree) refit(node int32) {
	n := &t.nodes[node]
	c1, c2 := &t.nodes[n.child1], &t.nodes[n.child2]
	n.aabb = union(c1.aabb, c2.aabb)
	n.height = 1 + max(c1.height, c2.height)
}

// refitUp: the node and its ancestors are refitted, and each tries a rotation, up to the first ancestor which doesn't
// change: the ones above it don't change either, and their rotations were tried when they last changed
func (t *aabbTree) refitUp(node int32) {
	for ; node != nullNode; node = t.nodes[node].parent {
		before := t.nodes[node]
		t.refit(node)
		if n := &t.nodes[node]; n.aabb == before.aabb && n.height == before.height {
			return
		}
		t.rotate(node)
	}
}

// rotate: among the 4 exchanges of a child of the node with a grandchild, the one which shrinks the other child the
// most (the child losing a grandchild takes the exchanged child instead). The AABB of the node itself doesn't change
func (t *aabbTree) rotate(node int32) {
	n := &t.nodes[node]
	if n.height < 2 {
		return
	}
	children := [2]int32{n.child1, n.child2}
	bestGain, bestChild, bestGrandchild := 0.0, int32(nullNode), int32(nullNode)
	for k, child := range children {
		other := &t.nodes[children[1-k]]
		if other.height == 0 {
			continue
		}
		// the child goes under the other child, in the place of one of its grandchildren
		grandchildren := [2]int32{other.child1, other.child2}
		for g, grandchild := range grandchildren {
			kept := t.nodes[grandchildren[1-g]].aabb
			shrunk := surfaceArea(union(kept, t.nodes[child].aabb))
			if gain := surfaceArea(other.aabb) - shrunk; gain > bestGain {
				bestGain, bestChild, bestGrandchild = gain, child, grandchild
			}
		}
	}
	if bestChild == nullNode {
		return
	}
	// exchange: the grandchild becomes a child of the node, the child a child of the other child
	other := t.nodes[bestGrandchild].parent
	if n.child1 == bestChild {
		n.child1 = bestGrandchild
	} else {
		n.child2 = bestGrandchild
	}
	t.nodes[bestGrandchild].parent = node
	o := &t.nodes[other]
	if o.child1 == bestGrandchild {
		o.child1 = bestChild
	} else {
		o.child2 = bestChild
	}
	t.nodes[bestChild].parent = other
	t.refit(other)
	t.refit(node)
}

// query appends the bodies of the leaves overlapping the AABB to out, in the order of the traversal. stack is reused
func (t *aabbTree) query(aabb actor.AABB, stack []int32, out []int32) ([]int32, []int32) {
	if t.empty() {
		return stack, out
	}
	stack = append(stack[:0], t.root)
	for len(stack) > 0 {
		n := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		node := &t.nodes[n]
		if !node.aabb.Overlaps(aabb) {
			continue
		}
		if node.height == 0 {
			out = append(out, node.body)
		} else {
			stack = append(stack, node.child1, node.child2)
		}
	}
	return stack, out
}

// height of the tree (0 for one leaf), for the tests
func (t *aabbTree) height() int32 {
	if t.empty() {
		return -1
	}
	return t.nodes[t.root].height
}

// ========== THE BROAD PHASE OF A WORLD ==========

// proxyKind: where a body is
type proxyKind uint8

const (
	proxyDynamic proxyKind = iota
	proxyStatic
	proxyLarge     // planes & heightfields: tested against every awake dynamic body
	proxyKinematic // in the tree of the dynamic bodies: its pairs with the static bodies and the planes are only for the triggers
)

type proxy struct {
	node int32
	kind proxyKind
	aabb actor.AABB // as stored in the tree (enlarged for a dynamic body)
	// box: the AABB of the body at its last sync, and changed: the step it last changed at (the body moved by a step, or
	// by the game). A trigger pair at rest keeps its overlap while the boxes of both bodies stay the same
	// (World.overlapKept)
	box     actor.AABB
	changed uint32
}

// Tree is the broad phase of a World: both AABB trees and the proxy of each body, in the order of World.Bodies
type Tree struct {
	dynamics aabbTree
	statics  aabbTree
	planes   []int32 // the large bodies
	proxies  []proxy
	bodies   []*actor.RigidBody // the bodies the proxies were made for: a mismatch rebuilds everything
	// step: the step of the World, the stamp of a change of the box of a proxy
	step uint32

	// filter: the pairs of bodies which never collide (nil without a World)
	filter *pairFilter

	// the pairs of proxies whose stored AABBs overlap, kept from a step to the next, and the proxies put in a tree
	// since the last search
	fat      []pairRecord
	fatIndex map[fatPair]struct{}
	dead     int // records of pairs which no longer overlap, compacted at the next search
	moved    []int32

	// buffers of the pair search: a chunk of work per worker
	chunks   []treeChunk
	queryJob func(i int)
	scanJob  func(i int)
	bodyList []*actor.RigidBody
	boxes    []actor.AABB
	pairs    []Pair
	sorted   []Pair
	counts   []int32
}

// treeChunk: the buffers of a unit of work of the workers
type treeChunk struct {
	stack      []int32
	candidates []int32
	found      []fatPair // the pairs found by the queries of the moved proxies
	pairs      []Pair    // the pairs of the step
	dead       []fatPair // the pairs whose stored AABBs no longer overlap
}

// fatPair: a pair of proxies, a < b
type fatPair struct {
	a, b int32
}

// pairRecord: a pair kept from a step to the next, with the contacts the World computed for it during the step stamp
// (the pair cache and the warm start of the next step). A record whose pair no longer overlaps is a tombstone (a < 0)
type pairRecord struct {
	key          fatPair
	first, count int32
	stamp        uint32
	// trigger: the pair had a trigger at the step stamp, and overlap: its shapes overlapped (it has no contact)
	trigger, overlap bool
}

func kindOf(body *actor.RigidBody) proxyKind {
	if isLarge(body) {
		return proxyLarge
	}
	switch body.BodyType {
	case actor.BodyTypeDynamic:
		return proxyDynamic
	case actor.BodyTypeKinematic:
		return proxyKinematic
	}
	return proxyStatic
}

// isLarge: planes & heightfields are not in the trees
func isLarge(body *actor.RigidBody) bool {
	switch body.Shape.(type) {
	case *actor.Plane, *actor.Heightfield:
		return true
	}
	return false
}

func enlarged(aabb actor.AABB) actor.AABB {
	margin := mgl64.Vec3{AABBMargin, AABBMargin, AABBMargin}
	return actor.AABB{Min: aabb.Min.Sub(margin), Max: aabb.Max.Add(margin)}
}

// sync the trees with the bodies and their AABBs of this step: a dynamic body out of its enlarged AABB is moved, a
// static body whose AABB changed too. The bodies must be the same slice as the last time, else everything is rebuilt
func (t *Tree) sync(bodies []*actor.RigidBody, boxes []actor.AABB) {
	if len(t.bodies) == 0 || !t.matches(bodies) {
		t.rebuild(bodies, boxes)
		return
	}
	known := len(t.bodies)
	for i := range bodies[:known] {
		t.update(int32(i), bodies[i], boxes[i])
	}
	// the bodies added since the last step
	for i := known; i < len(bodies); i++ {
		t.proxies = append(t.proxies, proxy{node: nullNode, kind: proxyLarge})
		t.bodies = append(t.bodies, bodies[i])
		t.place(int32(i), bodies[i], boxes[i])
	}
}

// matches: the bodies known start the slice (bodies were added at its end at most)
func (t *Tree) matches(bodies []*actor.RigidBody) bool {
	if len(t.bodies) > len(bodies) {
		return false
	}
	for i, body := range t.bodies {
		if bodies[i] != body {
			return false
		}
	}
	return true
}

func (t *Tree) rebuild(bodies []*actor.RigidBody, boxes []actor.AABB) {
	t.dynamics.clear()
	t.statics.clear()
	t.planes = t.planes[:0]
	t.proxies = t.proxies[:0]
	t.fat = t.fat[:0]
	clear(t.fatIndex)
	t.dead = 0
	t.moved = t.moved[:0]
	t.bodies = append(t.bodies[:0], bodies...)
	for i, body := range bodies {
		t.proxies = append(t.proxies, proxy{node: nullNode, kind: proxyLarge})
		t.place(int32(i), body, boxes[i])
	}
}

// place the body in its tree (or in the planes)
func (t *Tree) place(i int32, body *actor.RigidBody, aabb actor.AABB) {
	p := &t.proxies[i]
	p.box, p.changed = aabb, t.step
	p.kind = kindOf(body)
	switch p.kind {
	case proxyDynamic, proxyKinematic:
		p.aabb = enlarged(aabb)
		p.node = t.dynamics.insert(p.aabb, i)
		t.moved = append(t.moved, i)
	case proxyStatic:
		p.aabb = aabb
		p.node = t.statics.insert(aabb, i)
		t.moved = append(t.moved, i)
	default:
		p.aabb = aabb
		p.node = nullNode
		t.planes = append(t.planes, i)
		t.moved = append(t.moved, i)
	}
}

func (t *Tree) unplace(i int32) {
	p := &t.proxies[i]
	switch p.kind {
	case proxyDynamic, proxyKinematic:
		t.dynamics.remove(p.node)
	case proxyStatic:
		t.statics.remove(p.node)
	default:
		if k := slices.Index(t.planes, i); k >= 0 {
			t.planes = slices.Delete(t.planes, k, k+1)
		}
	}
	p.node = nullNode
}

func (t *Tree) update(i int32, body *actor.RigidBody, aabb actor.AABB) {
	p := &t.proxies[i]
	if p.box != aabb {
		p.box, p.changed = aabb, t.step
	}
	kind := kindOf(body)
	if kind != p.kind {
		t.unplace(i)
		t.place(i, body, aabb)
		return
	}
	switch kind {
	case proxyDynamic, proxyKinematic:
		if !contains(p.aabb, aabb) {
			t.dynamics.remove(p.node)
			p.aabb = enlarged(aabb)
			p.node = t.dynamics.insert(p.aabb, i)
			t.moved = append(t.moved, i)
		}
	case proxyStatic:
		if p.aabb != aabb {
			t.statics.remove(p.node)
			p.aabb = aabb
			p.node = t.statics.insert(aabb, i)
			t.moved = append(t.moved, i)
		}
	default:
		if p.aabb != aabb {
			p.aabb = aabb
			t.moved = append(t.moved, i)
		}
	}
}

// removed: the body at index k left World.Bodies (the following bodies moved up by one)
func (t *Tree) removed(k int) {
	if k >= len(t.proxies) {
		return
	}
	t.unplace(int32(k))
	t.proxies = slices.Delete(t.proxies, k, k+1)
	t.bodies = slices.Delete(t.bodies, k, k+1)
	t.forget(int32(k))
	for i := k; i < len(t.proxies); i++ {
		p := &t.proxies[i]
		if p.node != nullNode {
			if p.kind == proxyStatic {
				t.statics.nodes[p.node].body = int32(i)
			} else {
				t.dynamics.nodes[p.node].body = int32(i)
			}
		}
	}
	for j, index := range t.planes {
		if index > int32(k) {
			t.planes[j] = index - 1
		}
	}
}

// forget the pairs of the proxy k, removed: the proxies after it move up by one
func (t *Tree) forget(k int32) {
	kept := t.fat[:0]
	for _, record := range t.fat {
		pair := record.key
		if pair.a < 0 || pair.a == k || pair.b == k {
			continue
		}
		if pair.a > k {
			pair.a--
		}
		if pair.b > k {
			pair.b--
		}
		record.key = pair
		kept = append(kept, record)
	}
	t.fat = kept
	t.dead = 0
	clear(t.fatIndex)
	for _, record := range t.fat {
		t.fatIndex[record.key] = struct{}{}
	}
	// the removed proxy has no pair to find anymore: kept, its index would be the one of the next body, or past the
	// last one
	moved := t.moved[:0]
	for _, index := range t.moved {
		if index == k {
			continue
		}
		if index > k {
			index--
		}
		moved = append(moved, index)
	}
	t.moved = moved
}

// shiftContacts: contacts were removed from the contacts of the step stamp: shift[i] of them before the contact i.
// The records move their contacts down
func (t *Tree) shiftContacts(shift []int32, stamp uint32) {
	for i := range t.fat {
		record := &t.fat[i]
		if record.stamp == stamp && record.count > 0 {
			record.first -= shift[record.first]
		}
	}
}

// queryCandidates appends the indices of the bodies whose stored AABB overlaps the AABB, the planes first
func (t *Tree) queryCandidates(aabb actor.AABB, stack []int32, out []int32) ([]int32, []int32) {
	out = append(out, t.planes...)
	stack, out = t.statics.query(aabb, stack, out)
	stack, out = t.dynamics.query(aabb, stack, out)
	return stack, out
}

// ========== PAIRS ==========
// The pairs of proxies whose stored AABBs overlap are kept from a step to the next, as the pairs of Box2D v3: only a
// proxy put in a tree since the last search (a dynamic body out of its enlarged AABB, a static body moved by the
// game, a body added) queries the trees, and a resting body costs nothing. The planes and the heightfields pair with
// every body which moved. The pairs are dropped when their stored AABBs no longer overlap. The pairs of the step are
// the ones whose exact AABBs overlap and which pass the collision filters (filter.go), with an awake dynamic body, or a
// trigger and a dynamic or kinematic body awake or asleep (detectsTrigger): the same pairs as a search from scratch, in the same
// order. A filtered pair stays in the records (the filters can change at any time) but is never emitted. Each pair
// keeps the contacts of its last step (pairRecord): the World finds them without any lookup

// findPairs: the pairs of bodies whose AABBs overlap and which pass the collision filters, with at least an awake
// dynamic body, or a trigger (detectsTrigger), sorted by the index of the first body (the planes of a body before its
// other pairs, in the order of the planes). The slice is reused
func (t *Tree) findPairs(bodies []*actor.RigidBody, boxes []actor.AABB, pool *workerPool) []Pair {
	t.pairs = t.pairs[:0]
	t.bodyList, t.boxes = bodies, boxes
	if t.queryJob == nil {
		t.queryJob, t.scanJob = t.query, t.scan
	}
	if t.fatIndex == nil {
		t.fatIndex = map[fatPair]struct{}{}
	}
	if t.dead > 0 {
		t.compact()
	}

	// the proxies put in a tree since the last search find their pairs, by chunks; the pairs are then recorded once
	t.chunks = t.chunks[:0]
	queries := (len(t.moved) + movedPerChunk - 1) / movedPerChunk
	t.chunks = slices.Grow(t.chunks, queries)[:queries]
	pool.run(queries, 1, t.queryJob)
	for c := range t.chunks {
		for _, pair := range t.chunks[c].found {
			if _, known := t.fatIndex[pair]; !known {
				t.fatIndex[pair] = struct{}{}
				t.fat = append(t.fat, pairRecord{key: pair})
			}
		}
	}
	t.moved = t.moved[:0]

	// the pairs of the step, by chunks; the pairs which no longer overlap are then forgotten
	scans := (len(t.fat) + fatPerChunk - 1) / fatPerChunk
	t.chunks = slices.Grow(t.chunks[:0], scans)[:scans]
	pool.run(scans, 1, t.scanJob)
	for c := range t.chunks {
		t.pairs = append(t.pairs, t.chunks[c].pairs...)
		for _, pair := range t.chunks[c].dead {
			delete(t.fatIndex, pair)
		}
		t.dead += len(t.chunks[c].dead)
	}
	t.bodyList, t.boxes = nil, nil

	t.pairs = t.sortPairs(t.pairs, len(bodies))
	return t.pairs
}

// compact the records: the tombstones leave
func (t *Tree) compact() {
	kept := t.fat[:0]
	for _, record := range t.fat {
		if record.key.a >= 0 {
			kept = append(kept, record)
		}
	}
	t.fat = kept
	t.dead = 0
}

const (
	// movedPerChunk: queries of moved proxies per unit of work of the workers (a query costs ~1 µs, a unit of work
	// about the same as one of fatPerChunk records)
	movedPerChunk = 128
	// fatPerChunk: stored pairs per unit of work of the workers
	fatPerChunk = 1024
)

// query: the chunk c of the moved proxies finds its pairs (a pair of static bodies never needs solving, nor a static
// body against a plane). A kinematic body has no contact with the static bodies, the planes and the heightfields, as
// the kinematic proxies of Box2D v3 only query its dynamic tree for their contacts, but it enters and leaves their
// triggers, as the sensors of Box2D v3 query its 3 trees (b2SensorTask): it queries them too, its pairs without a
// trigger are kept but never emitted (Tree.scan), as a pair filtered
func (t *Tree) query(c int) {
	chunk := &t.chunks[c]
	chunk.found = chunk.found[:0]
	start := c * movedPerChunk
	for _, i := range t.moved[start:min(start+movedPerChunk, len(t.moved))] {
		p := &t.proxies[i]
		chunk.candidates = chunk.candidates[:0]
		chunk.stack, chunk.candidates = t.dynamics.query(p.aabb, chunk.stack, chunk.candidates)
		if p.kind == proxyDynamic || p.kind == proxyKinematic {
			chunk.stack, chunk.candidates = t.statics.query(p.aabb, chunk.stack, chunk.candidates)
			chunk.candidates = append(chunk.candidates, t.planes...)
		}
		for _, j := range chunk.candidates {
			if j == i {
				continue
			}
			if j < i {
				chunk.found = append(chunk.found, fatPair{j, i})
			} else {
				chunk.found = append(chunk.found, fatPair{i, j})
			}
		}
	}
}

// scan: the chunk c of the records emits the pairs of the step (the filtered pairs are skipped), and the pairs which
// no longer overlap become tombstones
func (t *Tree) scan(c int) {
	chunk := &t.chunks[c]
	chunk.pairs, chunk.dead = chunk.pairs[:0], chunk.dead[:0]
	bodies, boxes := t.bodyList, t.boxes
	start := c * fatPerChunk
	for k := start; k < min(start+fatPerChunk, len(t.fat)); k++ {
		record := &t.fat[k]
		i, j := record.key.a, record.key.b
		if !t.proxies[i].aabb.Overlaps(t.proxies[j].aabb) {
			chunk.dead = append(chunk.dead, record.key)
			record.key.a = -1
			continue
		}
		if !boxes[i].Overlaps(boxes[j]) {
			continue
		}
		if plane := t.proxies[i].kind == proxyLarge; plane || t.proxies[j].kind == proxyLarge {
			// a plane (or a heightfield) against a body: the pair of the body, the plane first
			if !plane {
				i, j = j, i
			}
			tested := isAwakeDynamic(bodies[j]) || detectsTrigger(bodies[i], bodies[j])
			if !tested || !t.filter.collides(bodies[i], bodies[j]) {
				continue
			}
			chunk.pairs = append(chunk.pairs, Pair{BodyA: bodies[i], BodyB: bodies[j], IndexA: i, IndexB: j, first: j, second: int32(slices.Index(t.planes, i)), plane: true, slot: int32(k)})
			continue
		}
		// the filters last: the pairs of resting bodies, the most of them, are skipped before
		if (needsSolving(bodies[i], bodies[j]) || detectsTrigger(bodies[i], bodies[j])) && t.filter.collides(bodies[i], bodies[j]) {
			chunk.pairs = append(chunk.pairs, Pair{BodyA: bodies[i], BodyB: bodies[j], IndexA: i, IndexB: j, first: i, second: j, slot: int32(k)})
		}
	}
}

// sortPairs by the index of the first body: a counting sort (O(pairs + bodies), the pairs are many and the keys are
// small), then the pairs of each body by their second key with the sort of the standard library (pdqsort, an insertion
// sort under 12 elements: a body with a few pairs costs as before, a zone or a ground with hundreds of pairs no longer
// costs their square). The keys of a body are unique, the order doesn't depend on the sort. Returns the sorted slice
// (the buffers are swapped)
func (t *Tree) sortPairs(pairs []Pair, bodiesCount int) []Pair {
	if cap(t.counts) < bodiesCount+1 {
		t.counts = make([]int32, bodiesCount+1)
	}
	counts := t.counts[:bodiesCount+1]
	for i := range counts {
		counts[i] = 0
	}
	for i := range pairs {
		counts[pairs[i].first+1]++
	}
	for i := 1; i < len(counts); i++ {
		counts[i] += counts[i-1]
	}
	if cap(t.sorted) < len(pairs) {
		t.sorted = make([]Pair, len(pairs))
	}
	sorted := t.sorted[:len(pairs)]
	for i := range pairs {
		k := pairs[i].first
		sorted[counts[k]] = pairs[i]
		counts[k]++
	}
	// within a body: its planes first (in their order), then its other pairs by index
	start := 0
	for end := 1; end <= len(sorted); end++ {
		if end < len(sorted) && sorted[end].first == sorted[start].first {
			continue
		}
		if end-start > 1 {
			slices.SortFunc(sorted[start:end], comparePairs)
		}
		start = end
	}
	t.sorted, t.pairs = pairs, sorted
	return sorted
}

// comparePairs: the order of the pairs of a body, its planes first
func comparePairs(a, b Pair) int {
	if a.plane != b.plane {
		if a.plane {
			return -1
		}
		return 1
	}
	return cmp.Compare(a.second, b.second)
}

// needsSolving: a dynamic body, and an awake body which moves (dynamic or kinematic): a pair of a kinematic body with a
// static or a kinematic body has no contact, a sleeping dynamic body is woken up by the kinematic body which reaches it
func needsSolving(a, b *actor.RigidBody) bool {
	return (isAwakeMover(a) || isAwakeMover(b)) && (a.BodyType == actor.BodyTypeDynamic || b.BodyType == actor.BodyTypeDynamic)
}

// detectsTrigger: a pair of a trigger and a body which moves (dynamic or kinematic), the trigger or the other one, is
// tested whatever the sleep, as the sensors of Box2D v3 ("Sensors do not consider sleep", docs/simulation.md;
// b2SensorTask queries every sensor against its 3 trees at each step): the pair ends when the shapes no longer
// overlap, never because a body falls asleep, and a static trigger the game moves away from a sleeping body, or over
// it, ends or starts the pair. While both bodies rest, the pair keeps its overlap without any test (World.overlapKept):
// it costs its place in the pairs of the step. A kinematic body enters a static trigger, and a kinematic trigger
// detects the static and the kinematic bodies, as in Box2D v3 and in Unity ("A dynamic or kinematic trigger collider
// collides with any collider type. A static trigger collider collides with any dynamic or Kinematic collider"); Jolt
// does it for a static sensor ("These sensors will only detect collisions with active Dynamic or Kinematic bodies",
// Body.h), for a kinematic sensor against the static bodies only on demand (SetCollideKinematicVsNonDynamic). 2 static
// bodies never pair
func detectsTrigger(a, b *actor.RigidBody) bool {
	return (a.IsTrigger || b.IsTrigger) && (a.BodyType != actor.BodyTypeStatic || b.BodyType != actor.BodyTypeStatic)
}

func isAwakeDynamic(body *actor.RigidBody) bool {
	return body.BodyType == actor.BodyTypeDynamic && !body.IsSleeping
}
