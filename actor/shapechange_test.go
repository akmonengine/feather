package actor

import (
	"math"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// RigidBody.SetShape and RigidBody.SetDensity, without a world: the shape, the mass, the inertia and the AABB of the
// body follow; a static body keeps its infinite mass and the density written; the velocities and the locks stay

func TestSetShapeOfABody(t *testing.T) {
	box := &Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}
	sphere := &Sphere{Radius: 0.5}
	transform := Transform{Position: mgl64.Vec3{1, 2, 3}, Rotation: mgl64.QuatIdent()}
	for _, bodyType := range []BodyType{BodyTypeDynamic, BodyTypeKinematic, BodyTypeStatic} {
		body := NewRigidBody(transform, box, bodyType, 1000)
		body.Velocity, body.AngularVelocity = mgl64.Vec3{1, 0, 0}, mgl64.Vec3{0, 1, 0}
		body.SetLocks(AxisY, AxisX)
		body.SetShape(sphere)
		if body.Shape != sphere {
			t.Fatalf("%v: shape %T, want the sphere", bodyType, body.Shape)
		}
		if body.AABB() != sphere.ComputeAABB(transform) {
			t.Errorf("%v: AABB %v, want the one of the sphere", bodyType, body.AABB())
		}
		if bodyType == BodyTypeStatic {
			if !math.IsInf(body.Material.GetMass(), 1) || body.InertiaLocal != (mgl64.Mat3{}) {
				t.Errorf("a static body: mass %v, inertia %v, want +Inf and none", body.Material.GetMass(), body.InertiaLocal)
			}
		} else {
			if want := sphere.ComputeMass(1000); body.Material.GetMass() != want {
				t.Errorf("%v: mass %v, want %v", bodyType, body.Material.GetMass(), want)
			}
			if want := sphere.ComputeInertia(body.Material.GetMass()); body.InertiaLocal != want || body.InverseInertiaLocal != want.Inv() {
				t.Errorf("%v: inertia %v, want %v", bodyType, body.InertiaLocal, want)
			}
		}
		if body.Velocity != (mgl64.Vec3{1, 0, 0}) || body.AngularVelocity != (mgl64.Vec3{0, 1, 0}) {
			if bodyType != BodyTypeStatic {
				t.Errorf("%v: the velocities changed: %v %v", bodyType, body.Velocity, body.AngularVelocity)
			}
		}
		if bodyType != BodyTypeStatic && (body.LinearLock != AxisY || body.AngularLock != AxisX) {
			t.Errorf("%v: the locks changed: %v %v", bodyType, body.LinearLock, body.AngularLock)
		}
	}
}

func TestSetDensityOfABody(t *testing.T) {
	box := &Box{HalfExtents: mgl64.Vec3{0.5, 0.5, 0.5}}
	dynamic := NewRigidBody(NewTransform(), box, BodyTypeDynamic, 1000)
	dynamic.SetDensity(250)
	if dynamic.Material.Density != 250 || dynamic.Material.GetMass() != box.ComputeMass(250) {
		t.Errorf("density %v, mass %v, want 250 and %v", dynamic.Material.Density, dynamic.Material.GetMass(), box.ComputeMass(250))
	}
	if want := box.ComputeInertia(box.ComputeMass(250)); dynamic.InertiaLocal != want || dynamic.InverseInertiaLocal != want.Inv() {
		t.Errorf("inertia %v, want %v", dynamic.InertiaLocal, want)
	}
	static := NewRigidBody(NewTransform(), box, BodyTypeStatic, 0)
	static.SetDensity(800)
	if static.Material.Density != 800 || !math.IsInf(static.Material.GetMass(), 1) || static.InverseMass() != 0 {
		t.Errorf("a static body: density %v, mass %v, want 800 kept and +Inf", static.Material.Density, static.Material.GetMass())
	}
	kinematic := NewRigidBody(NewTransform(), box, BodyTypeKinematic, 0)
	kinematic.SetDensity(800)
	if kinematic.Material.GetMass() != box.ComputeMass(800) || kinematic.InverseMass() != 0 {
		t.Errorf("a kinematic body: mass %v, inverse mass %v, want %v and 0", kinematic.Material.GetMass(), kinematic.InverseMass(), box.ComputeMass(800))
	}
}
