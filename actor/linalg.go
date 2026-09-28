package actor

import "github.com/go-gl/mathgl/mgl64"

// ========== PRODUCTS ==========
// The products of mgl64 on pointers: Mat3.Mul3x1 and Mat3.Mul3 copy their 72 bytes at each call, and the methods
// are not always inlined. The arithmetic is the same, in the same order: the results are bit-exact

// MulMat3 is m × v
func MulMat3(m *mgl64.Mat3, v mgl64.Vec3) mgl64.Vec3 {
	return mgl64.Vec3{
		m[0]*v[0] + m[3]*v[1] + m[6]*v[2],
		m[1]*v[0] + m[4]*v[1] + m[7]*v[2],
		m[2]*v[0] + m[5]*v[1] + m[8]*v[2],
	}
}

// Mul3 is a × b
func Mul3(a, b *mgl64.Mat3) mgl64.Mat3 {
	return mgl64.Mat3{
		a[0]*b[0] + a[3]*b[1] + a[6]*b[2],
		a[1]*b[0] + a[4]*b[1] + a[7]*b[2],
		a[2]*b[0] + a[5]*b[1] + a[8]*b[2],
		a[0]*b[3] + a[3]*b[4] + a[6]*b[5],
		a[1]*b[3] + a[4]*b[4] + a[7]*b[5],
		a[2]*b[3] + a[5]*b[4] + a[8]*b[5],
		a[0]*b[6] + a[3]*b[7] + a[6]*b[8],
		a[1]*b[6] + a[4]*b[7] + a[7]*b[8],
		a[2]*b[6] + a[5]*b[7] + a[8]*b[8],
	}
}

// Rotate is q v q⁻¹, the arithmetic of Quat.Rotate: v + 2 w (q × v) + 2 q × (q × v)
func Rotate(q *mgl64.Quat, v mgl64.Vec3) mgl64.Vec3 {
	c := mgl64.Vec3{q.V[1]*v[2] - q.V[2]*v[1], q.V[2]*v[0] - q.V[0]*v[2], q.V[0]*v[1] - q.V[1]*v[0]}
	w2 := 2 * q.W
	q2 := mgl64.Vec3{q.V[0] * 2, q.V[1] * 2, q.V[2] * 2}
	return mgl64.Vec3{
		v[0] + c[0]*w2 + (q2[1]*c[2] - q2[2]*c[1]),
		v[1] + c[1]*w2 + (q2[2]*c[0] - q2[0]*c[2]),
		v[2] + c[2]*w2 + (q2[0]*c[1] - q2[1]*c[0]),
	}
}

// RotateInverse is q⁻¹ v q: the rotation by the conjugate
func RotateInverse(q *mgl64.Quat, v mgl64.Vec3) mgl64.Vec3 {
	conjugate := mgl64.Quat{W: q.W, V: mgl64.Vec3{q.V[0] * -1, q.V[1] * -1, q.V[2] * -1}}
	return Rotate(&conjugate, v)
}

// MulQuat is a × b, the arithmetic of Quat.Mul
func MulQuat(a, b *mgl64.Quat) mgl64.Quat {
	c := mgl64.Vec3{a.V[1]*b.V[2] - a.V[2]*b.V[1], a.V[2]*b.V[0] - a.V[0]*b.V[2], a.V[0]*b.V[1] - a.V[1]*b.V[0]}
	return mgl64.Quat{
		W: a.W*b.W - (a.V[0]*b.V[0] + a.V[1]*b.V[1] + a.V[2]*b.V[2]),
		V: mgl64.Vec3{c[0] + b.V[0]*a.W + a.V[0]*b.W, c[1] + b.V[1]*a.W + a.V[1]*b.W, c[2] + b.V[2]*a.W + a.V[2]*b.W},
	}
}

// Add3 is a + b, Sub3 is a - b
func Add3(a, b *mgl64.Mat3) mgl64.Mat3 {
	return mgl64.Mat3{a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3], a[4] + b[4], a[5] + b[5], a[6] + b[6], a[7] + b[7], a[8] + b[8]}
}

func Sub3(a, b *mgl64.Mat3) mgl64.Mat3 {
	return mgl64.Mat3{a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3], a[4] - b[4], a[5] - b[5], a[6] - b[6], a[7] - b[7], a[8] - b[8]}
}

// Det3 is the determinant, the arithmetic of Mat3.Det
func Det3(m *mgl64.Mat3) float64 {
	return m[0]*m[4]*m[8] + m[3]*m[7]*m[2] + m[6]*m[1]*m[5] - m[6]*m[4]*m[2] - m[3]*m[1]*m[8] - m[0]*m[7]*m[5]
}

// Inv3 is m⁻¹, the arithmetic of Mat3.Inv: the adjugate over the determinant, the zero matrix if the determinant is 0
func Inv3(m *mgl64.Mat3) mgl64.Mat3 {
	det := Det3(m)
	if mgl64.FloatEqual(det, 0) {
		return mgl64.Mat3{}
	}
	c := 1 / det
	return mgl64.Mat3{
		(m[4]*m[8] - m[5]*m[7]) * c,
		(m[2]*m[7] - m[1]*m[8]) * c,
		(m[1]*m[5] - m[2]*m[4]) * c,
		(m[5]*m[6] - m[3]*m[8]) * c,
		(m[0]*m[8] - m[2]*m[6]) * c,
		(m[2]*m[3] - m[0]*m[5]) * c,
		(m[3]*m[7] - m[4]*m[6]) * c,
		(m[1]*m[6] - m[0]*m[7]) * c,
		(m[0]*m[4] - m[1]*m[3]) * c,
	}
}

// Transpose3 is mᵀ
func Transpose3(m *mgl64.Mat3) mgl64.Mat3 {
	return mgl64.Mat3{m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]}
}
