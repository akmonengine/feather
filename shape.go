package feather

import (
	"errors"

	"github.com/akmonengine/feather/actor"
)

// The shape and the density of a body changed while it is in the world (World.SetShape, World.SetDensity): the body
// keeps its index in World.Bodies, its proxy in the broad phase, its joints, its layers, the pairs it ignores and its
// island; what the change makes false is computed again at the next step.

// ErrNoShape: SetShape was given no shape. The body is left as it is
var ErrNoShape = errors.New("feather: SetShape needs a shape")

// ErrShapeNotConvex: SetShape was asked to give a plane, a heightfield or a triangle mesh to a dynamic or a kinematic
// body. These surfaces have no support point for GJK and no volume for a mass: they are the shapes of the static
// bodies (the complex collision of Unreal cannot be simulated, the concave shape of Godot is for the static bodies, a
// non-convex mesh collider of Unity is not simulated). The body is left as it is
var ErrShapeNotConvex = errors.New("feather: a plane, a heightfield and a triangle mesh are the shapes of the static bodies: a dynamic or a kinematic body needs a convex shape")

// SetShape gives a body of the world another shape, in place, as BodyInterface::SetShape of Jolt (BodyInterface.cpp:
// 298-326, v5.3.0, with the mass properties updated and the body activated):
//   - the body keeps its index, its proxy (the broad phase moves it to the tree of its new kind at the next step: a
//     plane made a box leaves the planes for the static tree), its joints, its layers, the pairs it ignores, its island,
//     its velocities, its target and its locks;
//   - a dynamic or a kinematic body takes the mass and the inertia of the shape at its density (Body::SetShapeInternal,
//     MotionProperties::SetMassProperties); a static body keeps its infinite mass;
//   - the contacts of its pairs are computed again at the next step instead of being taken from the pair cache
//     (BodyManager::InvalidateContactCacheForBody: the cache of the body is ignored for one step; the pair cache of
//     Feather would move the corners of a cube made a sphere with the bodies);
//   - the body wakes up with its island, and the sleeping bodies around its old and its new shape wake up: a static
//     pedestal shrunk under a pile lets it fall, a static slab grown under a crate lifts it, a trigger made its
//     inscribed sphere tests again the overlap of the crate asleep in its corner (the rule of Teleport and
//     UpdateHeightfield; Jolt leaves the neighbours asleep).
//
// The shapes of Feather are centered on their origin: the body stays where it is (Jolt moves it by the difference of
// the centers of mass). The caller places the body where the new shape belongs, by its Transform or Teleport. A body
// which overlaps something with its new shape is pushed out by the spring of the contact over the next steps, at
// ContactSpeed at most. PhysX (PxShape::setGeometry, PxShape.h:122-133) keeps the type of the geometry and "does not
// guarantee correct/continuous behavior when objects are resting on top of old or new geometry": Feather changes the
// kind and keeps the resting bodies stable (TestSetShapeOfARestingBodyKeepsItStable).
//
// No shape is refused (ErrNoShape), a plane, a heightfield or a mesh on a dynamic or a kinematic body too
// (ErrShapeNotConvex): a refused body is left as it is. The same shape again does nothing. The call allocates nothing
// once the world has stepped
func (w *World) SetShape(body *actor.RigidBody, shape actor.ShapeInterface) error {
	if shape == nil {
		return ErrNoShape
	}
	if body.BodyType != actor.BodyTypeStatic && !isConvex(shape) {
		return ErrShapeNotConvex
	}
	if body.Shape == shape {
		return nil
	}
	before := body.AABB()
	body.SetShape(shape)
	w.bodyChanged(body, union(before, body.AABB()))
	return nil
}

// SetDensity gives a body of the world another density, in place: a dynamic or a kinematic body takes the mass and the
// inertia of its shape at this density (PxRigidBodyExt::updateMassAndInertia of PhysX, ExtRigidBodyExt.cpp:282-290,
// v5.6.0), and wakes up with its island; a static body keeps the density written, for the day it becomes dynamic, and
// its infinite mass. A density which is not positive on a dynamic body is refused (ErrMasslessBody), the body left as
// it is. The call allocates nothing
func (w *World) SetDensity(body *actor.RigidBody, density float64) error {
	if body.BodyType == actor.BodyTypeDynamic && !(density > 0) {
		return ErrMasslessBody
	}
	body.SetDensity(density)
	if body.BodyType != actor.BodyTypeStatic {
		w.islands.wake(body)
	}
	return nil
}

// bodyChanged: the shape of the body changed, reach is the union of its old and its new AABB. Its pairs are computed
// again at the next step (isChanged, as a changed heightfield), the body wakes up with its island and the sleeping
// bodies within reach wake up (a trigger pair at rest keeps its overlap, overlapKept: its body awake, it is tested
// again)
func (w *World) bodyChanged(body *actor.RigidBody, reach actor.AABB) {
	w.changed = append(w.changed, body)
	w.islands.wake(body)
	w.wakeNeighborsIn(body, reach)
}
