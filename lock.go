package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== LOCKS ==========
// A body has no inverse mass along its locked world axes, and no inverse inertia around them: the solver and the
// integration don't move it along them (the allowed degrees of freedom of Jolt). Without lock, the arithmetic is the one
// of a body which has no lock at all, bit for bit

func (state *bodyState) isLocked() bool {
	return state.linearLock|state.angularLock != actor.NoAxes
}

// lockVelocities: what doesn't go through the inverse mass (the gravity, the gyroscopic torque, a velocity written by
// the game) is cleared along the locked axes
func (state *bodyState) lockVelocities() {
	state.velocity = state.linearLock.LockVector(state.velocity)
	state.angularVelocity = state.angularLock.LockVector(state.angularVelocity)
}

// linearMass: the inverse mass of both bodies along the unit direction d, d·(MA⁻¹ + MB⁻¹)·d. Without lock, the sum of
// both inverse masses
func linearMass(stateA, stateB *bodyState, d mgl64.Vec3) float64 {
	if stateA.linearLock|stateB.linearLock == actor.NoAxes {
		return stateA.invMass + stateB.invMass
	}
	return crossMass(stateA, stateB, d, d)
}

// crossMass: x·(MA⁻¹ + MB⁻¹)·y, the inverse masses by axis
func crossMass(stateA, stateB *bodyState, x, y mgl64.Vec3) float64 {
	a, b := &stateA.invMassAxes, &stateB.invMassAxes
	return x[0]*y[0]*(a[0]+b[0]) + x[1]*y[1]*(a[1]+b[1]) + x[2]*y[2]*(a[2]+b[2])
}

// linearMassMatrix: MA⁻¹ + MB⁻¹, the linear part of the mass matrix of a point constraint
func linearMassMatrix(stateA, stateB *bodyState) mgl64.Mat3 {
	a, b := &stateA.invMassAxes, &stateB.invMassAxes
	return mgl64.Mat3{a[0] + b[0], 0, 0, 0, a[1] + b[1], 0, 0, 0, a[2] + b[2]}
}

// ========== JOINTS ==========
// lockedPivotTolerance: a row which answers this many times less than the stiffest row of its block doesn't answer
const lockedPivotTolerance = 1e-6

// massInverse: the inverse of the mass matrix K of 3 rows solved together (a point, 3 rotations). false: no row can be
// solved
func massInverse(stateA, stateB *bodyState, k *mgl64.Mat3) (mgl64.Mat3, bool) {
	if stateA.isLocked() || stateB.isLocked() {
		return lockedInverse(k)
	}
	if math.Abs(actor.Det3(k)) < 1e-30 {
		return mgl64.Mat3{}, false
	}
	return actor.Inv3(k), true
}

// lockedInverse: the inverse of K when a body has locks. Both bodies may not answer along a direction (a body which can't
// turn held by a fixed joint, a body which can't move pinned to the world): K is singular, where Jolt gives up the whole
// block (PointConstraintPart) and PhysX keeps the inertia for this reason (DyRigidBodyToSolverBody.cpp). The rows which
// don't answer once the previous ones are solved are left out, their impulse stays null: the inverse of the others, at
// their place (a generalized inverse G of the positive semidefinite K: K G K = K). With its 3 rows, the inverse of K
func lockedInverse(k *mgl64.Mat3) (mgl64.Mat3, bool) {
	tolerance := lockedPivotTolerance * math.Max(k[0], math.Max(k[4], k[8]))
	if tolerance <= 0 {
		return mgl64.Mat3{}, false
	}
	// elimination without pivoting (K is symmetric, positive semidefinite): a row is kept if its pivot answers
	a := [3][3]float64{{k[0], k[3], k[6]}, {k[1], k[4], k[7]}, {k[2], k[5], k[8]}}
	var kept [3]int
	n := 0
	for i := 0; i < 3; i++ {
		if a[i][i] <= tolerance {
			continue
		}
		kept[n] = i
		n++
		for row := i + 1; row < 3; row++ {
			f := a[row][i] / a[i][i]
			for column := i; column < 3; column++ {
				a[row][column] -= f * a[i][column]
			}
		}
	}

	var inverse mgl64.Mat3
	switch n {
	case 3:
		return actor.Inv3(k), true
	case 2:
		i, j := kept[0], kept[1]
		kii, kij, kjj := k[4*i], k[3*j+i], k[4*j]
		det := kii*kjj - kij*kij
		inverse[4*i], inverse[4*j] = kjj/det, kii/det
		inverse[3*j+i], inverse[3*i+j] = -kij/det, -kij/det
	case 1:
		inverse[4*kept[0]] = 1 / k[4*kept[0]]
	}
	return inverse, true
}

// lockedInverse2: the same for 2 rows (both tangents of a contact, both axes of a hinge), as xx, xy, yy. With a single
// row which answers (K = k u uᵀ, of rank 1, u any direction between both rows), the inverse of least norm K / tr(K)²:
// its impulse is along u, the one the bodies answer to. Any other generalized inverse adds an impulse the locks absorb,
// which counts in the friction cone without rubbing
func lockedInverse2(k11, k12, k22 float64) ([3]float64, bool) {
	stiffest := math.Max(k11, k22)
	tolerance := lockedPivotTolerance * stiffest
	if tolerance <= 0 {
		return [3]float64{}, false
	}
	det := k11*k22 - k12*k12
	// det / stiffest: what the other row answers once the stiffest one is solved
	if det <= tolerance*stiffest {
		trace := k11 + k22
		scale := 1 / (trace * trace)
		return [3]float64{k11 * scale, k12 * scale, k22 * scale}, true
	}
	return [3]float64{k22 / det, -k12 / det, k11 / det}, true
}
