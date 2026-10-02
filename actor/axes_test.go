package actor

import (
	"math"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

func TestAxes(t *testing.T) {
	if AxisX != 1 || AxisY != 2 || AxisZ != 4 || NoAxes != 0 || AllAxes != 7 {
		t.Fatalf("axes %d %d %d, none %d, all %d", AxisX, AxisY, AxisZ, NoAxes, AllAxes)
	}
	cases := []struct {
		axes   Axes
		has    [3]bool
		count  int
		vector mgl64.Vec3
	}{
		{NoAxes, [3]bool{}, 0, mgl64.Vec3{1, 2, 3}},
		{AxisX, [3]bool{true, false, false}, 1, mgl64.Vec3{0, 2, 3}},
		{AxisY, [3]bool{false, true, false}, 1, mgl64.Vec3{1, 0, 3}},
		{AxisZ, [3]bool{false, false, true}, 1, mgl64.Vec3{1, 2, 0}},
		{AxisX | AxisZ, [3]bool{true, false, true}, 2, mgl64.Vec3{0, 2, 0}},
		{AllAxes, [3]bool{true, true, true}, 3, mgl64.Vec3{}},
	}
	if count := (AxisX | 8 | 64).Count(); count != 1 {
		t.Errorf("Count() = %d with bits which are not axes, want 1", count)
	}
	for _, c := range cases {
		for k := 0; k < 3; k++ {
			if c.axes.Has(k) != c.has[k] {
				t.Errorf("axes %d: Has(%d) = %v", c.axes, k, c.axes.Has(k))
			}
		}
		if c.axes.Count() != c.count {
			t.Errorf("axes %d: Count() = %d, want %d", c.axes, c.axes.Count(), c.count)
		}
		if v := c.axes.LockVector(mgl64.Vec3{1, 2, 3}); v != c.vector {
			t.Errorf("axes %d: LockVector = %v, want %v", c.axes, v, c.vector)
		}
	}
}

// allAxesSets: the 8 sets of axes
var allAxesSets = []Axes{NoAxes, AxisX, AxisY, AxisZ, AxisX | AxisY, AxisX | AxisZ, AxisY | AxisZ, AllAxes}

// The locked inverse inertia is the inverse of the free block of the inertia (the body held by bearings around its
// locked axes), with null rows and columns around the locked axes
func TestLockInverseInertia(t *testing.T) {
	body := lockedBody()
	inertia, free := body.GetInertiaWorld(), body.GetFreeInverseInertiaWorld()
	if free[1] == 0 || free[2] == 0 || free[5] == 0 {
		t.Fatal("the inertia of the leaning box is diagonal, the test needs its rows coupled")
	}
	for _, axes := range allAxesSets {
		locked := free
		axes.LockInverseInertia(&locked)
		if axes == NoAxes && locked != free {
			t.Errorf("without lock: %v, want the matrix untouched %v", locked, free)
		}
		// locked * inertia is the identity on the free axes, null elsewhere
		product := locked.Mul3(inertia)
		for row := 0; row < 3; row++ {
			for column := 0; column < 3; column++ {
				if axes.Has(row) || axes.Has(column) {
					if locked[3*column+row] != 0 {
						t.Errorf("axes %d: locked inverse inertia %v, want null rows & columns around the locked axes", axes, locked)
					}
					continue
				}
				want := 0.0
				if row == column {
					want = 1
				}
				if math.Abs(product[3*column+row]-want) > 1e-12 {
					t.Errorf("axes %d: (locked I⁻¹ · I)[%d %d] = %v, want %v: not the inverse of the free block of I", axes, row, column, product[3*column+row], want)
				}
			}
		}
	}
}

// A body whose axes of inertia are the ones of the world: the free block of its inverse inertia, bit for bit
func TestLockInverseInertiaOfAnAlignedBody(t *testing.T) {
	aligned := mgl64.Mat3{0.3, 0, 0, 0, 0.7, 0, 0, 0, 1.1}
	// coupled around its free axes only
	coupled := mgl64.Mat3{0.3, 0, 0, 0, 0.7, 0.2, 0, 0.2, 1.1}
	cases := []struct {
		axes         Axes
		matrix, want mgl64.Mat3
	}{
		{AxisX, aligned, mgl64.Mat3{0, 0, 0, 0, 0.7, 0, 0, 0, 1.1}},
		{AxisY, aligned, mgl64.Mat3{0.3, 0, 0, 0, 0, 0, 0, 0, 1.1}},
		{AxisZ, aligned, mgl64.Mat3{0.3, 0, 0, 0, 0.7, 0, 0, 0, 0}},
		{AxisX | AxisZ, aligned, mgl64.Mat3{0, 0, 0, 0, 0.7, 0, 0, 0, 0}},
		{AxisY | AxisZ, aligned, mgl64.Mat3{0.3, 0, 0, 0, 0, 0, 0, 0, 0}},
		{AllAxes, aligned, mgl64.Mat3{}},
		{AxisX, coupled, mgl64.Mat3{0, 0, 0, 0, 0.7, 0.2, 0, 0.2, 1.1}},
		{AxisY | AxisZ, coupled, mgl64.Mat3{0.3, 0, 0, 0, 0, 0, 0, 0, 0}},
	}
	for _, c := range cases {
		locked := c.matrix
		c.axes.LockInverseInertia(&locked)
		if locked != c.want {
			t.Errorf("axes %d: %v, want %v bit for bit", c.axes, locked, c.want)
		}
	}
}

func lockedBody() *RigidBody {
	transform := Transform{Position: mgl64.Vec3{1, 2, 3}, Rotation: mgl64.QuatRotate(0.6, mgl64.Vec3{1, 2, 3}.Normalize())}
	return NewRigidBody(transform, &Box{HalfExtents: mgl64.Vec3{0.1, 0.3, 0.2}}, BodyTypeDynamic, 500)
}

// A body has no lock by default; SetLocks clears the velocities along the locked axes and wakes the body up
func TestSetLocks(t *testing.T) {
	body := lockedBody()
	if body.LinearLock != NoAxes || body.AngularLock != NoAxes {
		t.Fatalf("a new body has the locks %d %d", body.LinearLock, body.AngularLock)
	}
	body.Velocity, body.AngularVelocity = mgl64.Vec3{1, 2, 3}, mgl64.Vec3{4, 5, 6}
	body.IsSleeping, body.SleepTimer = true, 0.3
	body.SetLocks(AxisX|AxisZ, AxisY)
	if body.LinearLock != AxisX|AxisZ || body.AngularLock != AxisY {
		t.Errorf("locks %d %d, want %d %d", body.LinearLock, body.AngularLock, AxisX|AxisZ, AxisY)
	}
	if body.Velocity != (mgl64.Vec3{0, 2, 0}) || body.AngularVelocity != (mgl64.Vec3{4, 0, 6}) {
		t.Errorf("velocities %v %v, want the locked axes cleared", body.Velocity, body.AngularVelocity)
	}
	if body.IsSleeping || body.SleepTimer != 0 {
		t.Error("SetLocks didn't wake the body up")
	}

	// no change: the body stays asleep
	body.IsSleeping = true
	body.SetLocks(AxisX|AxisZ, AxisY)
	if !body.IsSleeping {
		t.Error("SetLocks without change woke the body up")
	}
	// the bits which are not axes are ignored
	body.SetLocks(AxisX|AxisZ|64, AxisY|8)
	if !body.IsSleeping || body.LinearLock != AxisX|AxisZ || body.AngularLock != AxisY {
		t.Errorf("locks %d %d (sleeping %v) with bits which are not axes", body.LinearLock, body.AngularLock, body.IsSleeping)
	}
	body.SetLocks(AxisX|AxisZ, NoAxes)
	if body.IsSleeping || body.AngularLock != NoAxes {
		t.Errorf("freeing an axis: sleeping %v, angular lock %d", body.IsSleeping, body.AngularLock)
	}
}

// A static body never moves: it has no lock
func TestSetLocksOnStaticBody(t *testing.T) {
	body := NewRigidBody(NewTransform(), &Box{HalfExtents: mgl64.Vec3{1, 1, 1}}, BodyTypeStatic, 0)
	body.IsSleeping = true
	body.SetLocks(AllAxes, AllAxes)
	if body.LinearLock != NoAxes || body.AngularLock != NoAxes || !body.IsSleeping {
		t.Errorf("a static body took the locks %d %d (sleeping %v)", body.LinearLock, body.AngularLock, body.IsSleeping)
	}
	if body.InverseMassAxes() != (mgl64.Vec3{}) {
		t.Errorf("inverse mass of a static body: %v", body.InverseMassAxes())
	}
}

// The inverse mass is null along the locked axes; the inverse inertia in world space is the one of the body held around
// its locked axes: for a leaning box locked around X, not the free block of its inverse inertia (what Jolt keeps,
// MotionProperties::GetInverseInertiaForRotation)
func TestLockedInverseMassAndInertia(t *testing.T) {
	body := lockedBody()
	inverseMass := body.InverseMass()
	if body.InverseMassAxes() != (mgl64.Vec3{inverseMass, inverseMass, inverseMass}) {
		t.Errorf("inverse mass of a free body by axis: %v, want %v", body.InverseMassAxes(), inverseMass)
	}
	free := body.GetInverseInertiaWorld()
	if free != body.GetFreeInverseInertiaWorld() {
		t.Errorf("without lock, the inverse inertia %v is not the free one %v", free, body.GetFreeInverseInertiaWorld())
	}

	body.LinearLock, body.AngularLock = AxisY, AxisX
	if body.InverseMass() != inverseMass || body.InverseMassAxes() != (mgl64.Vec3{inverseMass, 0, inverseMass}) {
		t.Errorf("inverse mass %v, by axis %v", body.InverseMass(), body.InverseMassAxes())
	}
	if body.GetFreeInverseInertiaWorld() != free || body.GetInertiaWorld() == (mgl64.Mat3{}) {
		t.Error("the locks changed the free inverse inertia, or the inertia")
	}
	locked := body.GetInverseInertiaWorld()
	want := free
	AxisX.LockInverseInertia(&want)
	if locked != want {
		t.Errorf("locked inverse inertia %v, want %v", locked, want)
	}
	if masked := (mgl64.Mat3{0, 0, 0, 0, free[4], free[5], 0, free[7], free[8]}); locked == masked {
		t.Errorf("locked inverse inertia %v: the free block of the inverse, want the inverse of the free block", locked)
	}
	if free[1] == 0 || free[2] == 0 {
		t.Fatal("the inertia of the leaning box is diagonal, the test needs its rows coupled")
	}
}

// The impulses don't move a locked axis; the forces are held by the solver
func TestImpulsesRespectLocks(t *testing.T) {
	body := lockedBody()
	free := lockedBody()
	body.LinearLock, body.AngularLock = AxisX, AxisZ
	impulse, point := mgl64.Vec3{3, -2, 5}, mgl64.Vec3{1.2, 2.5, 2.9}
	body.AddImpulseAtPoint(impulse, point)
	free.AddImpulseAtPoint(impulse, point)
	if body.Velocity != (mgl64.Vec3{0, free.Velocity.Y(), free.Velocity.Z()}) {
		t.Errorf("velocity %v, want the one of the free body %v without X", body.Velocity, free.Velocity)
	}
	inverseInertia := body.GetInverseInertiaWorld()
	want := inverseInertia.Mul3x1(point.Sub(body.Transform.Position).Cross(impulse))
	if body.AngularVelocity[2] != 0 || body.AngularVelocity.Sub(want).Len() > 1e-12 || math.Abs(want[0]) < 1e-3 {
		t.Errorf("angular velocity %v, want %v", body.AngularVelocity, want)
	}

	body.AngularVelocity = mgl64.Vec3{}
	body.AddAngularImpulse(mgl64.Vec3{0, 0, 2})
	if body.AngularVelocity[2] != 0 {
		t.Errorf("an angular impulse around the locked axis gives %v", body.AngularVelocity)
	}
}
