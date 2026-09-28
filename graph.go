package feather

// Graph coloring, as in Box2D v3 (constraint_graph.c): the constraints of a color don't share any dynamic body,
// so a color can be solved in parallel. The static bodies don't count, they never move.
// The constraints are colored in the order of the pairs, then solved color by color: the result is the same
// whatever the number of workers.
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
	constraints []int
	bodies      []uint64 // bitset of the dynamic bodies used by the color
}

type constraintGraph struct {
	colors   [graphColorsCount]graphColor
	overflow []int
}

// color assigns each constraint to the first color where both of its dynamic bodies are free
func (g *constraintGraph) color(constraints []contactConstraint, bodiesCount int) {
	words := (bodiesCount + 63) / 64
	for i := range g.colors {
		color := &g.colors[i]
		color.constraints = color.constraints[:0]
		if cap(color.bodies) < words {
			color.bodies = make([]uint64, words)
		}
		color.bodies = color.bodies[:words]
		clear(color.bodies)
	}
	g.overflow = g.overflow[:0]

	for i := range constraints {
		indexA, indexB := constraints[i].indexA, constraints[i].indexB
		colored := false
		for k := range g.colors {
			color := &g.colors[k]
			if isUsed(color.bodies, indexA) || isUsed(color.bodies, indexB) {
				continue
			}
			use(color.bodies, indexA)
			use(color.bodies, indexB)
			color.constraints = append(color.constraints, i)
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

// solveConstraints: the overflow first (sequential), then each color (parallel)
func (s *solver) solveConstraints(solve func(c *contactConstraint)) {
	for _, i := range s.graph.overflow {
		solve(&s.constraints[i])
	}
	s.stage = solve
	for k := range s.graph.colors {
		s.color = s.graph.colors[k].constraints
		s.pool.run(len(s.color), constraintsChunk, s.jobs.color)
	}
}

// forEachBody runs fn for each body state, in parallel for large scenes
func (s *solver) forEachBody(fn func(i int)) {
	s.pool.run(len(s.states), bodiesChunk, fn)
}
