package feather

// Graph coloring, as in Box2D v3 (constraint_graph.c): the constraints of a color don't share any dynamic body,
// so a color can be solved in parallel. The static bodies don't count, they never move.
// The contacts (in the order of the pairs) then the joints are colored, and solved color by color: the result is the
// same whatever the number of workers. The joints of the articulations (solved together, before the colors) keep their
// other rows in the colors.
const (
	// graphColorsCount: the constraints without a free color go to the overflow, solved sequentially
	graphColorsCount = 16

	// constraintsChunk: the workers take the constraints of a color by chunks of this size.
	// A color smaller than a chunk is solved by a single worker.
	constraintsChunk = 32

	// bodiesChunk: same for the bodies integration
	bodiesChunk = 64

	// minParallelBodies: under this count of bodies, the step runs on a single goroutine
	minParallelBodies = 256

	// pairsPerChunk: the workers take the pairs of the narrow phase by chunks of this size
	pairsPerChunk = 16
)

type graphColor struct {
	items  []int    // the items of the color: a contact constraint, or a joint after the constraints
	bodies []uint64 // bitset of the dynamic bodies used by the color
}

// graphItem: the dynamic bodies of a constraint to color (-1 for a static body)
type graphItem struct {
	indexA, indexB int
}

type constraintGraph struct {
	colors   [graphColorsCount]graphColor
	overflow []int
}

// color assigns each item to the first color where both of its dynamic bodies are free. An item with a static body
// never takes the color 0 (as in Box2D v3): it is solved after the contacts between dynamic bodies, the ground has the
// last word. Solved first, a body pressed by a heavier one would leave the step moving into the ground
func (g *constraintGraph) color(items []graphItem, bodiesCount int) {
	words := (bodiesCount + 63) / 64
	for i := range g.colors {
		color := &g.colors[i]
		color.items = color.items[:0]
		if cap(color.bodies) < words {
			color.bodies = make([]uint64, words)
		}
		color.bodies = color.bodies[:words]
		clear(color.bodies)
	}
	g.overflow = g.overflow[:0]

	for i := range items {
		indexA, indexB := items[i].indexA, items[i].indexB
		colored := false
		first := 0
		if indexA < 0 || indexB < 0 {
			first = 1
		}
		for k := first; k < len(g.colors); k++ {
			color := &g.colors[k]
			if isUsed(color.bodies, indexA) || isUsed(color.bodies, indexB) {
				continue
			}
			use(color.bodies, indexA)
			use(color.bodies, indexB)
			color.items = append(color.items, i)
			colored = true
			break
		}
		if !colored {
			g.overflow = append(g.overflow, i)
		}
	}
}

func isUsed(bits []uint64, index int) bool {
	return index >= 0 && bits[index/64]&(1<<(index%64)) != 0
}

func use(bits []uint64, index int) {
	if index >= 0 {
		bits[index/64] |= 1 << (index % 64)
	}
}

// solveConstraints: the overflow first (sequential), then each color (parallel). The joints go through the joint
// stage (none for the restitution)
func (s *solver) solveConstraints(solve func(c *contactConstraint), solveJoint func(j Joint)) {
	s.stage, s.jointStage = solve, solveJoint
	for _, i := range s.graph.overflow {
		s.solveItem(i)
	}
	for k := range s.graph.colors {
		s.color = s.graph.colors[k].items
		s.pool.run(len(s.color), constraintsChunk, s.jobs.color)
	}
}

// solveItem: a contact constraint, or a joint after the constraints
func (s *solver) solveItem(item int) {
	if item < len(s.constraints) {
		s.stage(&s.constraints[item])
		return
	}
	if s.jointStage != nil {
		s.jointStage(s.joints[item-len(s.constraints)])
	}
}

// forEachBody runs fn for each body state, in parallel for large scenes
func (s *solver) forEachBody(fn func(i int)) {
	s.pool.run(len(s.states), bodiesChunk, fn)
}
