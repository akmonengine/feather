package actor

import (
	"math/bits"

	"github.com/go-gl/mathgl/mgl64"
)

// Axes is a set of world axes, one bit per axis
type Axes uint8

const (
	AxisX Axes = 1 << iota
	AxisY
	AxisZ

	// NoAxes: the locks of a body created by NewRigidBody, it moves and turns freely
	NoAxes Axes = 0

	// AllAxes: the 3 axes
	AllAxes = AxisX | AxisY | AxisZ
)

// Has: the axis k (0: X, 1: Y, 2: Z) is in the set
func (axes Axes) Has(k int) bool {
	return axes&(1<<k) != 0
}

// Count of axes in the set
func (axes Axes) Count() int {
	return bits.OnesCount8(uint8(axes & AllAxes))
}

// LockVector: the vector without its components along the axes of the set
func (axes Axes) LockVector(v mgl64.Vec3) mgl64.Vec3 {
	for k := range v {
		if axes.Has(k) {
			v[k] = 0
		}
	}
	return v
}

// LockInverseInertia: the inverse inertia in world space K = I⁻¹ of a body which can't turn around the axes of the set.
// It is the inverse of the free block of the inertia, (I_FF)⁻¹ = K_FF - K_FL K_LL⁻¹ K_LF (F the free axes, L the locked
// ones), with null rows and columns around the locked axes: the moment of inertia of a body held by bearings. For a
// body whose axes of inertia are the ones of the world, K_FL is null: the free block of K, bit for bit
func (axes Axes) LockInverseInertia(k *mgl64.Mat3) {
	var locked, free [3]int
	lockedCount, freeCount := 0, 0
	for axis := 0; axis < 3; axis++ {
		if axes.Has(axis) {
			locked[lockedCount] = axis
			lockedCount++
		} else {
			free[freeCount] = axis
			freeCount++
		}
	}

	var held mgl64.Mat3
	switch lockedCount {
	case 0:
		return
	case 1:
		// a body turning around 2 axes: K_LL is the scalar kll
		a, b, l := free[0], free[1], locked[0]
		kal, kbl := k[3*l+a], k[3*l+b]
		inverse := 0.0
		if kll := k[4*l]; kll > 0 {
			inverse = 1 / kll
		}
		held[4*a] = k[4*a] - kal*kal*inverse
		held[4*b] = k[4*b] - kbl*kbl*inverse
		held[3*b+a] = k[3*b+a] - kal*kbl*inverse
		held[3*a+b] = held[3*b+a]
	case 2:
		// a body turning around a single axis f: 1 / (f·I·f)
		f, a, b := free[0], locked[0], locked[1]
		kaa, kab, kbb := k[4*a], k[3*b+a], k[4*b]
		kfa, kfb := k[3*a+f], k[3*b+f]
		inverse := 0.0
		if det := kaa*kbb - kab*kab; det > 0 {
			inverse = 1 / det
		}
		held[4*f] = k[4*f] - (kfa*(kbb*kfa-kab*kfb)+kfb*(kaa*kfb-kab*kfa))*inverse
	}
	*k = held
}
