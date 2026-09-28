package feather

import (
	"slices"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== BROAD PHASE ==========
// The broad phase is a pair of dynamic AABB trees (Catto, "Dynamic Bounding Volume Hierarchies", GDC 2019; the
// b2DynamicTree of Box2D, the btDbvt of Bullet, the QuadTree of Jolt): one for the static bodies, updated when a body is
// added, removed or moved by the game, one for the dynamic bodies, awake or asleep. A dynamic body is stored with its
// AABB enlarged by AABBMargin: a body which moves inside its enlarged AABB doesn't touch the tree, a sleeping body
// never does. The planes and the heightfields are not in the trees: they are tested against every awake body.
//
// The pairs of overlapping stored AABBs are kept from a step to the next (see findPairs): only a body put back in a
// tree queries it. The pairs of the step are those whose exact AABBs overlap, sorted by the index of the first body,
// the planes of a body before its other pairs.

const (
	// AABBMargin: the AABB of a dynamic body is enlarged by this margin in the tree (m). Larger: fewer updates of the
	// tree, more candidates per query. 0.1 m as Box2D
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

// aabbTree: a binary tree of AABBs, the bodies at its leaves
type aabbTree struct {
	nodes []treeNode
	root  int32
	free  int32 // first free node, linked by parent
}

func (t *aabbTree) allocate() int32 {
	if t.free == nullNode {
		t.nodes = append(t.nodes, treeNode{})
		t.free = int32(len(t.nodes) - 1)
		t.nodes[t.free].parent = nullNode
	}
	n := t.free
	t.free = t.nodes[n].parent
	t.nodes[n] = treeNode{parent: nullNode, child1: nullNode, child2: nullNode, body: nullNode}
	return n
}

func (t *aabbTree) release(n int32) {
	t.nodes[n].parent = t.free
	t.nodes[n].height = -1
	t.free = n
}

func (t *aabbTree) clear() {
	t.nodes = t.nodes[:0]
	t.root, t.free = nullNode, nullNode
}

// insert a leaf for the body with the AABB, returns the node
func (t *aabbTree) insert(aabb actor.AABB, body int32) int32 {
	leaf := t.allocate()
	t.nodes[leaf].aabb = aabb
	t.nodes[leaf].body = body
	t.insertLeaf(leaf)
	return leaf
}

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

// insertLeaf: the sibling is chosen down the tree by the surface area heuristic (the cost of the new parent plus the
// cost inherited by the ancestors, as Box2D v2.4), then the ancestors are enlarged and balanced by rotations
func (t *aabbTree) insertLeaf(leaf int32) {
	if t.root == nullNode {
		t.root = leaf
		t.nodes[leaf].parent = nullNode
		return
	}
	leafAABB := t.nodes[leaf].aabb
	index := t.root
	for t.nodes[index].height > 0 {
		child1, child2 := t.nodes[index].child1, t.nodes[index].child2
		area := surfaceArea(t.nodes[index].aabb)
		combinedArea := surfaceArea(union(t.nodes[index].aabb, leafAABB))
		// the cost of making a new parent for this node and the leaf
		cost := 2 * combinedArea
		// the cost of pushing the leaf further down the tree
		inheritance := 2 * (combinedArea - area)
		costOf := func(child int32) float64 {
			childArea := surfaceArea(union(leafAABB, t.nodes[child].aabb))
			if t.nodes[child].height == 0 {
				return childArea + inheritance
			}
			return childArea - surfaceArea(t.nodes[child].aabb) + inheritance
		}
		cost1, cost2 := costOf(child1), costOf(child2)
		if cost < cost1 && cost < cost2 {
			break
		}
		if cost1 < cost2 {
			index = child1
		} else {
			index = child2
		}
	}
	sibling := index

	// a new parent above the sibling
	oldParent := t.nodes[sibling].parent
	newParent := t.allocate()
	t.nodes[newParent].parent = oldParent
	t.nodes[newParent].aabb = union(leafAABB, t.nodes[sibling].aabb)
	t.nodes[newParent].height = t.nodes[sibling].height + 1
	t.nodes[newParent].child1, t.nodes[newParent].child2 = sibling, leaf
	t.nodes[sibling].parent, t.nodes[leaf].parent = newParent, newParent
	if oldParent == nullNode {
		t.root = newParent
	} else if t.nodes[oldParent].child1 == sibling {
		t.nodes[oldParent].child1 = newParent
	} else {
		t.nodes[oldParent].child2 = newParent
	}

	// the ancestors grow, and are balanced
	for index = t.nodes[leaf].parent; index != nullNode; index = t.nodes[index].parent {
		index = t.balance(index)
		child1, child2 := t.nodes[index].child1, t.nodes[index].child2
		t.nodes[index].height = 1 + max(t.nodes[child1].height, t.nodes[child2].height)
		t.nodes[index].aabb = union(t.nodes[child1].aabb, t.nodes[child2].aabb)
	}
}

func (t *aabbTree) removeLeaf(leaf int32) {
	if leaf == t.root {
		t.root = nullNode
		return
	}
	parent := t.nodes[leaf].parent
	grandParent := t.nodes[parent].parent
	sibling := t.nodes[parent].child1
	if sibling == leaf {
		sibling = t.nodes[parent].child2
	}
	if grandParent == nullNode {
		t.root = sibling
		t.nodes[sibling].parent = nullNode
		t.release(parent)
		return
	}
	// the sibling takes the place of the parent
	if t.nodes[grandParent].child1 == parent {
		t.nodes[grandParent].child1 = sibling
	} else {
		t.nodes[grandParent].child2 = sibling
	}
	t.nodes[sibling].parent = grandParent
	t.release(parent)
	for index := grandParent; index != nullNode; index = t.nodes[index].parent {
		index = t.balance(index)
		child1, child2 := t.nodes[index].child1, t.nodes[index].child2
		t.nodes[index].aabb = union(t.nodes[child1].aabb, t.nodes[child2].aabb)
		t.nodes[index].height = 1 + max(t.nodes[child1].height, t.nodes[child2].height)
	}
}

// balance the subtree at a by a rotation if its children differ in height by more than 1 (an AVL rotation), returns
// the new root of the subtree
func (t *aabbTree) balance(a int32) int32 {
	na := &t.nodes[a]
	if na.height < 2 {
		return a
	}
	b, c := na.child1, na.child2
	balance := t.nodes[c].height - t.nodes[b].height
	if balance > 1 {
		return t.rotate(a, c, b)
	}
	if balance < -1 {
		return t.rotate(a, b, c)
	}
	return a
}

// rotate the child up above a: up takes the place of a, a takes the place of the shallower child of up, the other
// child (other) stays under a
func (t *aabbTree) rotate(a, up, other int32) int32 {
	nodes := t.nodes
	f, g := nodes[up].child1, nodes[up].child2
	// up replaces a
	nodes[up].child1, nodes[up].child2 = a, f
	nodes[up].parent = nodes[a].parent
	nodes[a].parent = up
	if nodes[up].parent == nullNode {
		t.root = up
	} else if nodes[nodes[up].parent].child1 == a {
		nodes[nodes[up].parent].child1 = up
	} else {
		nodes[nodes[up].parent].child2 = up
	}
	// the taller of f, g stays with up; the other goes under a, next to other
	if nodes[f].height > nodes[g].height {
		nodes[up].child2 = f
		t.setChildren(a, other, g)
	} else {
		nodes[up].child2 = g
		t.setChildren(a, other, f)
	}
	nodes[a].aabb = union(nodes[nodes[a].child1].aabb, nodes[nodes[a].child2].aabb)
	nodes[a].height = 1 + max(nodes[nodes[a].child1].height, nodes[nodes[a].child2].height)
	nodes[up].aabb = union(nodes[nodes[up].child1].aabb, nodes[nodes[up].child2].aabb)
	nodes[up].height = 1 + max(nodes[nodes[up].child1].height, nodes[nodes[up].child2].height)
	return up
}

func (t *aabbTree) setChildren(parent, child1, child2 int32) {
	t.nodes[parent].child1, t.nodes[parent].child2 = child1, child2
	t.nodes[child1].parent, t.nodes[child2].parent = parent, parent
}

// query appends the bodies of the leaves overlapping the AABB to out, in the order of the traversal. stack is reused
func (t *aabbTree) query(aabb actor.AABB, stack []int32, out []int32) ([]int32, []int32) {
	if t.root == nullNode {
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
	if t.root == nullNode {
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
	proxyLarge // planes & heightfields: tested against every awake body
)

type proxy struct {
	node int32
	kind proxyKind
	aabb actor.AABB // as stored in the tree (enlarged for a dynamic body)
}

// Tree is the broad phase of a World: both AABB trees and the proxy of each body, in the order of World.Bodies
type Tree struct {
	dynamics aabbTree
	statics  aabbTree
	planes   []int32 // the large bodies
	proxies  []proxy
	bodies   []*actor.RigidBody // the bodies the proxies were made for: a mismatch rebuilds everything

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
}

func kindOf(body *actor.RigidBody) proxyKind {
	if isLarge(body) {
		return proxyLarge
	}
	if body.BodyType == actor.BodyTypeDynamic {
		return proxyDynamic
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
	p.kind = kindOf(body)
	switch p.kind {
	case proxyDynamic:
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
	case proxyDynamic:
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
	kind := kindOf(body)
	if kind != p.kind {
		t.unplace(i)
		t.place(i, body, aabb)
		return
	}
	switch kind {
	case proxyDynamic:
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
			if p.kind == proxyDynamic {
				t.dynamics.nodes[p.node].body = int32(i)
			} else {
				t.statics.nodes[p.node].body = int32(i)
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
	for i, index := range t.moved {
		if index > k {
			t.moved[i] = index - 1
		}
	}
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
// the ones whose exact AABBs overlap, with an awake dynamic body: the same pairs as a search from scratch, in the same
// order. Each pair keeps the contacts of its last step (pairRecord): the World finds them without any lookup

// findPairs: the pairs of bodies whose AABBs overlap, with at least an awake dynamic body, sorted by the index of the
// first body (the planes of a body before its other pairs, in the order of the planes). The slice is reused
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
	// movedPerChunk: queries of moved proxies per unit of work of the workers
	movedPerChunk = 64
	// fatPerChunk: stored pairs per unit of work of the workers
	fatPerChunk = 1024
)

// query: the chunk c of the moved proxies finds its pairs (a pair of static bodies never needs solving, nor a static
// body against a plane)
func (t *Tree) query(c int) {
	chunk := &t.chunks[c]
	chunk.found = chunk.found[:0]
	start := c * movedPerChunk
	for _, i := range t.moved[start:min(start+movedPerChunk, len(t.moved))] {
		p := &t.proxies[i]
		chunk.candidates = chunk.candidates[:0]
		chunk.stack, chunk.candidates = t.dynamics.query(p.aabb, chunk.stack, chunk.candidates)
		if p.kind == proxyDynamic {
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

// scan: the chunk c of the records emits the pairs of the step, and the pairs which no longer overlap become
// tombstones
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
			if !isAwakeDynamic(bodies[j]) {
				continue
			}
			chunk.pairs = append(chunk.pairs, Pair{BodyA: bodies[i], BodyB: bodies[j], IndexA: i, IndexB: j, first: j, second: int32(slices.Index(t.planes, i)), plane: true, slot: int32(k)})
			continue
		}
		if needsSolving(bodies[i], bodies[j]) {
			chunk.pairs = append(chunk.pairs, Pair{BodyA: bodies[i], BodyB: bodies[j], IndexA: i, IndexB: j, first: i, second: j, slot: int32(k)})
		}
	}
}

// sortPairs by the index of the first body: a counting sort (O(pairs + bodies), the pairs are many and the keys are
// small), then the few pairs of a body by their second key. Returns the sorted slice (the buffers are swapped)
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
	// within a body: its planes first (in their order), then its other pairs by index: an insertion sort of a few pairs
	start := 0
	for end := 1; end <= len(sorted); end++ {
		if end < len(sorted) && sorted[end].first == sorted[start].first {
			continue
		}
		for i := start + 1; i < end; i++ {
			for j := i; j > start && pairBefore(sorted[j], sorted[j-1]); j-- {
				sorted[j], sorted[j-1] = sorted[j-1], sorted[j]
			}
		}
		start = end
	}
	t.sorted, t.pairs = pairs, sorted
	return sorted
}

// pairBefore: the order of the pairs of a body
func pairBefore(a, b Pair) bool {
	if a.plane != b.plane {
		return a.plane
	}
	return a.second < b.second
}

// needsSolving - At least one body must be dynamic and awake
func needsSolving(a, b *actor.RigidBody) bool {
	return isAwakeDynamic(a) || isAwakeDynamic(b)
}

func isAwakeDynamic(body *actor.RigidBody) bool {
	return body.BodyType == actor.BodyTypeDynamic && !body.IsSleeping
}
