package feather

import (
	"github.com/go-gl/mathgl/mgl64"
)

// ========== ARTICULATIONS ==========
// The point constraints of the joints linking dynamic bodies are solved together, exactly, by tree of joints: the linear
// time dynamics of Baraff ("Linear-Time Dynamics using Lagrange Multipliers", SIGGRAPH 1996), the joints eliminated from
// the leaves to the root. Solved one by one, the soft spring of a joint acts on the mass of its own bodies: a heavy body
// hanging on light links stretches them (a ball 670 times heavier than a link stretched each joint by 7 cm). Solved
// together, the spring acts on the mass of the whole system (the frequency whatever the mass, of Catto's soft
// constraints, for the system): the chain holds the ball.
//
// The joints of a net (a loop between dynamic bodies) are solved one by one, as the other rows of the joints (axes,
// limits, motors, springs). A chain taut between 2 fixed points has a redundant row: the proximal term of the contacts
// (blockRegularization) keeps the system invertible, the impulses closest to the previous ones.

// articulations: the joints of the trees, in the order of elimination, and the factorization of their mass matrix
type articulations struct {
	// joints in the order of elimination, the trees one after the other; tree[i] is the first joint of the tree of i
	joints []*JointBase
	// later: the neighbors of each joint eliminated after it (sharing a body, or filled by the elimination), from
	// start[i] to start[i+1]
	start []int
	later []int
	// the lower blocks of the matrix below each joint, then its factor L; diag: the diagonal block D, inverse: D⁻¹
	lower   []mgl64.Mat3
	diag    []mgl64.Mat3
	inverse []mgl64.Mat3
	// anchors of the joints during the pass, and the right-hand side, then the solution
	anchorA, anchorB []mgl64.Vec3
	vector           []mgl64.Vec3

	// buffers of build
	bodyJoints [][]int // the joints of each state
	position   []int   // the position of each joint of s.joints in the order, -1 if not articulated
	visited    []bool
	stack      [][3]int // depth first search: body, joint to its parent, next joint to visit
	adjacency  [][]int
}

// buildArticulations orders the joints of the trees of dynamic bodies from the leaves to the root, and finds the fill
// of the elimination (none for a chain, the siblings of a body with several children)
func (s *solver) buildArticulations() {
	a := &s.articulations
	a.joints = a.joints[:0]
	bodies := len(s.states)
	a.bodyJoints = resizeSlices(a.bodyJoints, bodies)
	a.visited = resizeBools(a.visited, bodies)
	a.position = resizeInts(a.position, len(s.joints))
	for i, joint := range s.joints {
		j := joint.base()
		j.inArticulation = false
		a.position[i] = -1
		if j.indexA >= 0 {
			a.bodyJoints[j.indexA] = append(a.bodyJoints[j.indexA], i)
		}
		if j.indexB >= 0 {
			a.bodyJoints[j.indexB] = append(a.bodyJoints[j.indexB], i)
		}
	}

	// ========== order: post-order of the trees of bodies ==========
	// A tree starts at a body attached to a static body if any. A joint is placed once all the joints below it are: the
	// joints to the static bodies of a body, then the joint to its parent
	for pass := 0; pass < 2; pass++ {
		for root := 0; root < bodies; root++ {
			if a.visited[root] || len(a.bodyJoints[root]) == 0 || (pass == 0 && !s.attachedToStatic(root)) {
				continue
			}
			first := len(a.joints)
			loop := s.orderTree(root)
			if len(a.joints)-first < 2 || loop {
				// a single joint: solved alone. A net (a loop between dynamic bodies): its joints are solved one by one,
				// the tree exact and the joints closing the loops alone converge slowly (a stiff subsystem against the rows
				// coupled to it: a net of 60 x 60 opened by 357 mm, 320 by joint)
				for i, position := range a.position {
					if position >= first {
						a.position[i] = -1
						s.joints[i].base().inArticulation = false
					}
				}
				a.joints = a.joints[:first]
			}
		}
	}

	// ========== fill ==========
	n := len(a.joints)
	a.adjacency = resizeSlices(a.adjacency, n)
	local := a.position
	for _, joints := range a.bodyJoints[:bodies] {
		for _, x := range joints {
			for _, y := range joints {
				px, py := local[x], local[y]
				if px >= 0 && py > px {
					a.adjacency[px] = appendUnique(a.adjacency[px], py)
				}
			}
		}
	}
	a.start = resizeInts(a.start, n+1)
	a.later = a.later[:0]
	for i := 0; i < n; i++ {
		a.start[i] = len(a.later)
		neighbors := a.adjacency[i]
		a.later = append(a.later, neighbors...)
		// eliminating i links all its later neighbors together
		for u, x := range neighbors {
			for _, y := range neighbors[u+1:] {
				lo, hi := min(x, y), max(x, y)
				a.adjacency[lo] = appendUnique(a.adjacency[lo], hi)
			}
		}
	}
	a.start[n] = len(a.later)
	a.lower = resizeMats(a.lower, len(a.later))
	a.diag = resizeMats(a.diag, n)
	a.inverse = resizeMats(a.inverse, n)
	a.anchorA = resizeVecs(a.anchorA, n)
	a.anchorB = resizeVecs(a.anchorB, n)
	a.vector = resizeVecs(a.vector, n)
	for i := range a.bodyJoints[:bodies] {
		a.bodyJoints[i] = a.bodyJoints[i][:0]
		a.visited[i] = false
	}
}

func (s *solver) attachedToStatic(body int) bool {
	for _, i := range s.articulations.bodyJoints[body] {
		if j := s.joints[i].base(); j.indexA < 0 || j.indexB < 0 {
			return true
		}
	}
	return false
}

// orderTree: depth first from the root, each joint placed after the subtree of its child body. Returns true if a joint
// reaching a body already in the tree closes a loop
func (s *solver) orderTree(root int) bool {
	loop := false
	a := &s.articulations
	a.stack = append(a.stack[:0], [3]int{root, -1, 0})
	a.visited[root] = true
	for len(a.stack) > 0 {
		top := &a.stack[len(a.stack)-1]
		body, parentJoint := top[0], top[1]
		joints := a.bodyJoints[body]
		if top[2] < len(joints) {
			i := joints[top[2]]
			top[2]++
			if i == parentJoint || a.position[i] != -1 {
				continue
			}
			j := s.joints[i].base()
			other := j.indexA
			if other == body {
				other = j.indexB
			}
			switch {
			case other < 0:
				// attached to a static body: a leaf
				s.place(i)
			case !a.visited[other]:
				a.visited[other] = true
				a.stack = append(a.stack, [3]int{other, i, 0})
			default:
				loop = true
				a.position[i] = -2
			}
			continue
		}
		a.stack = a.stack[:len(a.stack)-1]
		if parentJoint >= 0 {
			s.place(parentJoint)
		}
	}
	for i := range a.position {
		if a.position[i] == -2 {
			a.position[i] = -1
		}
	}
	return loop
}

func (s *solver) place(i int) {
	a := &s.articulations
	j := s.joints[i].base()
	a.position[i] = len(a.joints)
	a.joints = append(a.joints, j)
	j.inArticulation = true
}

// solveArticulations: for each tree, K Δλ = -(ċ + bias), K = J M⁻¹ Jᵀ the mass matrix of the point constraints of all
// its joints, factored from the leaves (block LDLᵀ). The soft spring acts on the whole system:
// Δλ = -(K⁻¹ (ċ + bias) + γ λ) / (1 + γ)
func (s *solver) solveArticulations(useBias bool) {
	a := &s.articulations
	n := len(a.joints)
	if n == 0 {
		return
	}
	for i, j := range a.joints {
		stateA, stateB := s.state(j.indexA), s.state(j.indexB)
		rA, rB := j.currentAnchors(stateA, stateB)
		a.anchorA[i], a.anchorB[i] = rA, rB
		cdot := stateB.velocity.Add(stateB.angularVelocity.Cross(rB)).Sub(stateA.velocity.Add(stateA.angularVelocity.Cross(rA)))
		if useBias {
			separation := stateB.deltaPosition.Sub(stateA.deltaPosition).Add(rB.Sub(rA)).Add(j.deltaCenter)
			cdot = cdot.Add(separation.Mul(j.spring.biasRate))
		}
		a.vector[i] = cdot
		a.diag[i] = s.coupling(j, j, rA, rB, rA, rB)
	}
	for i, j := range a.joints {
		for t := a.start[i]; t < a.start[i+1]; t++ {
			k := a.later[t]
			a.lower[t] = s.coupling(a.joints[k], j, a.anchorA[k], a.anchorB[k], a.anchorA[i], a.anchorB[i])
		}
		// the proximal term
		for c := 0; c < 3; c++ {
			a.diag[i].Set(c, c, a.diag[i].At(c, c)*(1+blockRegularization))
		}
	}

	// ========== factor: A = L D Lᵀ, from the leaves ==========
	for i := 0; i < n; i++ {
		inverse := a.diag[i].Inv()
		a.inverse[i] = inverse
		for t := a.start[i]; t < a.start[i+1]; t++ {
			// update the later blocks with -A_ki D⁻¹ A_li
			k := a.later[t]
			ak := a.lower[t]
			for u := a.start[i]; u < a.start[i+1]; u++ {
				l := a.later[u]
				if l < k {
					continue
				}
				update := a.lower[u].Mul3(inverse).Mul3(ak.Transpose())
				if l == k {
					a.diag[k] = a.diag[k].Sub(update)
				} else {
					a.lower[a.find(k, l)] = a.lower[a.find(k, l)].Sub(update)
				}
			}
		}
		for t := a.start[i]; t < a.start[i+1]; t++ {
			a.lower[t] = a.lower[t].Mul3(inverse)
		}
	}

	// ========== solve ==========
	for i := 0; i < n; i++ {
		for t := a.start[i]; t < a.start[i+1]; t++ {
			k := a.later[t]
			a.vector[k] = a.vector[k].Sub(a.lower[t].Mul3x1(a.vector[i]))
		}
	}
	for i := 0; i < n; i++ {
		a.vector[i] = a.inverse[i].Mul3x1(a.vector[i])
	}
	for i := n - 1; i >= 0; i-- {
		for t := a.start[i]; t < a.start[i+1]; t++ {
			a.vector[i] = a.vector[i].Sub(a.lower[t].Transpose().Mul3x1(a.vector[a.later[t]]))
		}
	}

	// ========== impulses ==========
	for i, j := range a.joints {
		row := rigid
		if useBias {
			row = j.spring
		}
		impulse := a.vector[i].Add(j.linearImpulse.Mul(row.gamma)).Mul(-1 / (1 + row.gamma))
		j.linearImpulse = j.linearImpulse.Add(impulse)
		applyLinear(s.state(j.indexA), s.state(j.indexB), a.anchorA[i], a.anchorB[i], impulse)
	}
}

// find the block (row l, column k) below k
func (a *articulations) find(k, l int) int {
	for t := a.start[k]; t < a.start[k+1]; t++ {
		if a.later[t] == l {
			return t
		}
	}
	panic("feather: articulation fill missing")
}

// coupling: the block of K between the point constraints of the joints x (row) and y (column), through their shared
// dynamic bodies: s_x s_y (m⁻¹ I - [r_x]× I⁻¹ [r_y]×), s = -1 on A, +1 on B
func (s *solver) coupling(x, y *JointBase, rAx, rBx, rAy, rBy mgl64.Vec3) mgl64.Mat3 {
	var block mgl64.Mat3
	add := func(body int, signX float64, rX mgl64.Vec3, signY float64, rY mgl64.Vec3) {
		state := s.state(body)
		term := mgl64.Ident3().Mul(state.invMass).Sub(skew(rX).Mul3(state.inverseInertia).Mul3(skew(rY)))
		block = block.Add(term.Mul(signX * signY))
	}
	if x.indexA >= 0 && x.indexA == y.indexA {
		add(x.indexA, -1, rAx, -1, rAy)
	}
	if x.indexA >= 0 && x.indexA == y.indexB {
		add(x.indexA, -1, rAx, 1, rBy)
	}
	if x.indexB >= 0 && x.indexB == y.indexA {
		add(x.indexB, 1, rBx, -1, rAy)
	}
	if x.indexB >= 0 && x.indexB == y.indexB {
		add(x.indexB, 1, rBx, 1, rBy)
	}
	return block
}

// ========== buffers, reused from a step to the next ==========

func appendUnique(list []int, value int) []int {
	for _, v := range list {
		if v == value {
			return list
		}
	}
	return append(list, value)
}

func resizeInts(s []int, n int) []int {
	if cap(s) < n {
		return make([]int, n, 2*n)
	}
	return s[:n]
}

func resizeBools(s []bool, n int) []bool {
	if cap(s) < n {
		return make([]bool, n, 2*n)
	}
	s = s[:n]
	clear(s)
	return s
}

func resizeMats(s []mgl64.Mat3, n int) []mgl64.Mat3 {
	if cap(s) < n {
		return make([]mgl64.Mat3, n, 2*n)
	}
	return s[:n]
}

func resizeVecs(s []mgl64.Vec3, n int) []mgl64.Vec3 {
	if cap(s) < n {
		return make([]mgl64.Vec3, n, 2*n)
	}
	return s[:n]
}

// resizeSlices: n empty slices, keeping the capacity of the ones already there
func resizeSlices(s [][]int, n int) [][]int {
	for len(s) < n {
		s = append(s, nil)
	}
	s = s[:n]
	for i := range s {
		s[i] = s[i][:0]
	}
	return s
}
