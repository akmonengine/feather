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
// The pairs come from a traversal of the trees against themselves (see findPairs), sorted by the index of the first
// body, the planes of a body before its other pairs. The result doesn't depend on the number of workers.

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
}

type treeNode struct {
	aabb   actor.AABB
	parent int32
	child1 int32
	child2 int32
	height int32 // 0 for a leaf
	body   int32 // index of the body (leaves)
	awake  bool  // an awake dynamic body under this node (this step)
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

	// buffers of the pair search
	batches []nodePair
	next    []nodePair
	chunks  []treeChunk
	pairs   []Pair
	sorted  []Pair
	counts  []int32
	boxes   []actor.AABB
	job     func(i int)
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
	if !t.matches(bodies) {
		t.rebuild(bodies, boxes)
		return
	}
	for i := range bodies {
		t.update(int32(i), bodies[i], boxes[i])
	}
}

func (t *Tree) matches(bodies []*actor.RigidBody) bool {
	if len(t.bodies) != len(bodies) {
		return false
	}
	for i, body := range bodies {
		if t.bodies[i] != body {
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
	case proxyStatic:
		p.aabb = aabb
		p.node = t.statics.insert(aabb, i)
	default:
		p.aabb = aabb
		p.node = nullNode
		t.planes = append(t.planes, i)
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
		}
	case proxyStatic:
		if p.aabb != aabb {
			t.statics.remove(p.node)
			p.aabb = aabb
			p.node = t.statics.insert(aabb, i)
		}
	default:
		p.aabb = aabb
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

// queryCandidates appends the indices of the bodies whose stored AABB overlaps the AABB, the planes first
func (t *Tree) queryCandidates(aabb actor.AABB, stack []int32, out []int32) ([]int32, []int32) {
	out = append(out, t.planes...)
	stack, out = t.statics.query(aabb, stack, out)
	stack, out = t.dynamics.query(aabb, stack, out)
	return stack, out
}

// ========== PAIRS ==========
// The pairs are found by a traversal of the trees against themselves (the tree-versus-tree collision of the btDbvt of
// Bullet, the same in PhysX): each pair of overlapping nodes is visited once, a subtree without any awake body is
// pruned. The pairs of nodes at the top of the trees are split into batches for the workers; the pairs found are
// sorted at the end, so the result doesn't depend on the workers.

const (
	// pairBatches: the traversal starts with about this many pairs of nodes, spread on the workers
	pairBatches = 256
	// batchesPerChunk: pairs of nodes per unit of work
	batchesPerChunk = 4
)

type nodePair struct {
	a, b int32
}

type treeChunk struct {
	pairs []Pair
	stack []nodePair
}

// flagAwake marks the nodes with an awake dynamic body under them
func (t *Tree) flagAwake(bodies []*actor.RigidBody) {
	nodes := t.dynamics.nodes
	for i := range nodes {
		nodes[i].awake = false
	}
	for i, body := range bodies {
		p := &t.proxies[i]
		if p.kind != proxyDynamic || !isAwakeDynamic(body) {
			continue
		}
		for n := p.node; n != nullNode && !nodes[n].awake; n = nodes[n].parent {
			nodes[n].awake = true
		}
	}
}

// findPairs: the pairs of bodies whose AABBs overlap, with at least an awake dynamic body, sorted by the index of the
// first body (the planes of a body before its other pairs, in the order of the planes). The slice is reused
func (t *Tree) findPairs(bodies []*actor.RigidBody, boxes []actor.AABB, pool *workerPool) []Pair {
	t.flagAwake(bodies)
	t.boxes = boxes
	t.pairs = t.pairs[:0]

	// the planes, against every awake body
	for i, body := range bodies {
		if !isAwakeDynamic(body) || t.proxies[i].kind == proxyLarge {
			continue
		}
		for k, planeIdx := range t.planes {
			if boxes[planeIdx].Overlaps(boxes[i]) {
				t.pairs = append(t.pairs, Pair{BodyA: bodies[planeIdx], BodyB: body, IndexA: planeIdx, IndexB: int32(i), first: int32(i), second: int32(k), plane: true})
			}
		}
	}

	// the pairs of nodes at the top of the trees, then their traversal by batches
	t.batches = t.batches[:0]
	if root := t.dynamics.root; root != nullNode && t.dynamics.nodes[root].awake {
		t.batches = append(t.batches, nodePair{root, root})
		if t.statics.root != nullNode {
			t.batches = append(t.batches, nodePair{root, ^t.statics.root})
		}
	}
	t.batches = t.expand(t.batches)
	chunksCount := (len(t.batches) + batchesPerChunk - 1) / batchesPerChunk
	if len(t.chunks) < chunksCount {
		t.chunks = append(t.chunks, make([]treeChunk, chunksCount-len(t.chunks))...)
	}
	if t.job == nil {
		t.job = func(i int) {
			start := i * batchesPerChunk
			t.traverse(t.batches[start:min(start+batchesPerChunk, len(t.batches))], &t.chunks[i])
		}
	}
	pool.run(chunksCount, 1, t.job)
	for i := 0; i < chunksCount; i++ {
		t.pairs = append(t.pairs, t.chunks[i].pairs...)
	}
	t.boxes = nil

	t.pairs = t.sortPairs(t.pairs, len(bodies))
	return t.pairs
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

// A pair of nodes (a, b): b >= 0 is a node of the dynamic tree, b < 0 is the node ^b of the static tree.
// (a, a) is the pair of a node with itself: its own leaves against each other

// node of a pair: the node, its tree, and whether it is on the static side
func (t *Tree) node(n int32) (*treeNode, *aabbTree) {
	if n < 0 {
		return &t.statics.nodes[^n], &t.statics
	}
	return &t.dynamics.nodes[n], &t.dynamics
}

// children of a pair of nodes: the pairs to visit next, appended to out (none for a pair of leaves)
func (t *Tree) children(pair nodePair, out []nodePair) []nodePair {
	na, _ := t.node(pair.a)
	if pair.a == pair.b {
		if na.height == 0 {
			return out
		}
		c1, c2 := na.child1, na.child2
		return append(out, nodePair{c1, c1}, nodePair{c2, c2}, nodePair{c1, c2})
	}
	nb, _ := t.node(pair.b)
	if !na.aabb.Overlaps(nb.aabb) {
		return out
	}
	// the static side is never awake: prune on the dynamic side
	if !na.awake && (pair.b < 0 || !nb.awake) {
		return out
	}
	if na.height == 0 && nb.height == 0 {
		return append(out, pair) // a pair of leaves: kept as is
	}
	// split the taller node
	if nb.height == 0 || (na.height != 0 && na.height >= nb.height) {
		return append(out, nodePair{na.child1, pair.b}, nodePair{na.child2, pair.b})
	}
	if pair.b < 0 {
		return append(out, nodePair{pair.a, ^nb.child1}, nodePair{pair.a, ^nb.child2})
	}
	return append(out, nodePair{pair.a, nb.child1}, nodePair{pair.a, nb.child2})
}

func isLeafPair(t *Tree, pair nodePair) bool {
	if pair.a == pair.b {
		return false
	}
	na, _ := t.node(pair.a)
	nb, _ := t.node(pair.b)
	return na.height == 0 && nb.height == 0
}

// expand the pairs of nodes breadth first until there are enough batches for the workers
func (t *Tree) expand(batches []nodePair) []nodePair {
	for len(batches) > 0 && len(batches) < pairBatches {
		t.next = t.next[:0]
		expanded := false
		for _, pair := range batches {
			if isLeafPair(t, pair) {
				t.next = append(t.next, pair)
			} else {
				t.next = t.children(pair, t.next)
				expanded = true
			}
		}
		batches, t.next = t.next, batches
		if !expanded {
			break
		}
	}
	return batches
}

// traverse the batches depth first, emitting the pairs of bodies whose exact AABBs overlap
func (t *Tree) traverse(batches []nodePair, chunk *treeChunk) {
	chunk.pairs = chunk.pairs[:0]
	bodies, boxes := t.bodies, t.boxes
	stack := append(chunk.stack[:0], batches...)
	for len(stack) > 0 {
		pair := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if isLeafPair(t, pair) {
			na, _ := t.node(pair.a)
			nb, _ := t.node(pair.b)
			i, j := na.body, nb.body
			if !boxes[i].Overlaps(boxes[j]) || !needsSolving(bodies[i], bodies[j]) {
				continue
			}
			if j < i {
				i, j = j, i
			}
			chunk.pairs = append(chunk.pairs, Pair{BodyA: bodies[i], BodyB: bodies[j], IndexA: i, IndexB: j, first: i, second: j})
			continue
		}
		stack = t.children(pair, stack)
	}
	chunk.stack = stack
}

// needsSolving - At least one body must be dynamic and awake
func needsSolving(a, b *actor.RigidBody) bool {
	return isAwakeDynamic(a) || isAwakeDynamic(b)
}

func isAwakeDynamic(body *actor.RigidBody) bool {
	return body.BodyType == actor.BodyTypeDynamic && !body.IsSleeping
}
