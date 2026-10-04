package actor

import (
	"errors"
	"math"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// A kinematic body has no inverse mass nor inverse inertia for the solver, but keeps the mass of its shape and
// density, for the day it becomes dynamic. Its forces are ignored
func TestNewRigidBody_Kinematic(t *testing.T) {
	box := &Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}
	body := NewRigidBody(NewTransform(), box, BodyTypeKinematic, 1000)
	if body.BodyType != BodyTypeKinematic {
		t.Fatalf("type %v, want kinematic", body.BodyType)
	}
	if mass := body.Material.GetMass(); mass != box.ComputeMass(1000) {
		t.Errorf("mass %v, want the mass of the shape %v", mass, box.ComputeMass(1000))
	}
	if body.InverseMass() != 0 {
		t.Errorf("inverse mass %v, want 0", body.InverseMass())
	}
	if body.GetInverseInertiaWorld() != (mgl64.Mat3{}) || body.GetFreeInverseInertiaWorld() != (mgl64.Mat3{}) {
		t.Errorf("inverse inertia %v, want 0", body.GetInverseInertiaWorld())
	}
	if body.InertiaLocal == (mgl64.Mat3{}) {
		t.Errorf("the inertia of the shape is kept for a switch to dynamic")
	}
	body.AddForce(mgl64.Vec3{1, 1, 1})
	body.AddTorque(mgl64.Vec3{1, 1, 1})
	body.AddImpulse(mgl64.Vec3{1, 1, 1})
	body.AddAngularImpulse(mgl64.Vec3{1, 1, 1})
	if body.Force() != (mgl64.Vec3{}) || body.Torque() != (mgl64.Vec3{}) || body.Velocity != (mgl64.Vec3{}) || body.AngularVelocity != (mgl64.Vec3{}) {
		t.Errorf("a kinematic body took a force %v, a torque %v or a velocity %v %v", body.Force(), body.Torque(), body.Velocity, body.AngularVelocity)
	}

	// without density: no mass
	massless := NewRigidBody(NewTransform(), box, BodyTypeKinematic, 0)
	if mass := massless.Material.GetMass(); mass != 0 || math.IsNaN(mass) {
		t.Errorf("mass %v without density, want 0", mass)
	}
}

// The target of a kinematic body is kept until the step takes it; it wakes the body up if it moves it. A body which
// is not kinematic refuses it with ErrNotKinematic, never in silence; neither call allocates
func TestKinematicTarget(t *testing.T) {
	body := NewRigidBody(NewTransform(), &Sphere{Radius: 0.1}, BodyTypeKinematic, 1000)
	if _, ok := body.KinematicTarget(); ok {
		t.Fatalf("a new body has a target")
	}
	body.Sleep()
	if err := body.SetKinematicTarget(body.Transform); err != nil {
		t.Fatalf("a target on a kinematic body: %v", err)
	}
	if !body.IsSleeping {
		t.Errorf("a target at the pose of the body woke it up")
	}
	target := Transform{Position: mgl64.Vec3{1, 2, 3}, Rotation: mgl64.QuatRotate(0.5, mgl64.Vec3{0, 1, 0})}
	if err := body.SetKinematicTarget(target); err != nil {
		t.Fatalf("a target on a kinematic body: %v", err)
	}
	if got, ok := body.KinematicTarget(); !ok || got != target {
		t.Errorf("target %v %v, want %v", got, ok, target)
	}
	if body.IsSleeping {
		t.Errorf("a target away from the body didn't wake it up")
	}
	body.ClearKinematicTarget()
	if _, ok := body.KinematicTarget(); ok {
		t.Errorf("the target was not cleared")
	}

	for _, bodyType := range []BodyType{BodyTypeDynamic, BodyTypeStatic} {
		other := NewRigidBody(NewTransform(), &Sphere{Radius: 0.1}, bodyType, 1000)
		if err := other.SetKinematicTarget(target); !errors.Is(err, ErrNotKinematic) {
			t.Errorf("a target on a body of type %v: error %v, want ErrNotKinematic", bodyType, err)
		}
		if _, ok := other.KinematicTarget(); ok {
			t.Errorf("a body of type %v took a kinematic target", bodyType)
		}
		if allocs := testing.AllocsPerRun(10, func() { _ = other.SetKinematicTarget(target) }); allocs > 0 {
			t.Errorf("the refusal of a target allocates %.1f, want 0", allocs)
		}
	}
	if allocs := testing.AllocsPerRun(10, func() { _ = body.SetKinematicTarget(target) }); allocs > 0 {
		t.Errorf("a target allocates %.1f, want 0", allocs)
	}
}
