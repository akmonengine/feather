package feather

import (
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// The time of impact of a sphere moving towards a box: the exact fraction where the gap is toiTarget
func TestTimeOfImpact(t *testing.T) {
	box := actor.NewRigidBody(actor.Transform{Position: mgl64.Vec3{3, 0, 0}, Rotation: mgl64.QuatIdent()}, &actor.Box{HalfExtents: mgl64.Vec3{0.5, 1, 1}}, actor.BodyTypeStatic, 0)
	sphere := &actor.Sphere{Radius: 0.25}
	motion := sweep{
		start: actor.Transform{Position: mgl64.Vec3{0, 0, 0}, Rotation: mgl64.QuatIdent()},
		end:   actor.Transform{Position: mgl64.Vec3{4, 0, 0}, Rotation: mgl64.QuatIdent()},
	}
	proxy := gjk.NewProxy(box)
	fraction := timeOfImpact(sphere, &motion, 0.25, &proxy, nil, 1)
	// the sphere touches the face x = 2.5 when its center is at 2.25, stopped toiTarget before
	want := (2.25 - toiTarget) / 4
	if math.Abs(fraction-want)*4 > toiTolerance {
		t.Errorf("fraction %.6f, want %.6f", fraction, want)
	}

	// against a plane, with a rotation: a box falling on a corner
	plane := &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}}
	falling := sweep{
		start: actor.Transform{Position: mgl64.Vec3{0, 2, 0}, Rotation: mgl64.QuatIdent()},
		end:   actor.Transform{Position: mgl64.Vec3{0, -1, 0}, Rotation: mgl64.QuatRotate(0.5, mgl64.Vec3{0, 0, 1})},
	}
	falling.angle = 0.5
	cube := &actor.Box{HalfExtents: mgl64.Vec3{0.25, 0.25, 0.25}}
	fraction = timeOfImpact(cube, &falling, cube.HalfExtents.Len(), nil, plane, 1)
	at := falling.at(fraction)
	lowest := at.ToWorld(cube.Support(at.Rotation.Conjugate().Rotate(mgl64.Vec3{0, -1, 0}))).Y()
	if fraction <= 0 || fraction >= 1 || lowest < toiTarget-1e-9 || lowest > toiTarget+toiTolerance {
		t.Errorf("fraction %.4f, the lowest corner at %.5f m", fraction, lowest)
	}
}

// A normal body goes through the dynamic bodies during the continuous collision, a bullet stops on them
func TestBulletStopsOnDynamicBodies(t *testing.T) {
	for _, bullet := range []bool{false, true} {
		w := newScene(1)
		w.Gravity = mgl64.Vec3{}
		addBody(w, mgl64.Vec3{5, 0, 0}, mgl64.QuatIdent(), &actor.Box{HalfExtents: mgl64.Vec3{0.01, 1, 1}}, actor.BodyTypeDynamic, 0.5, 0)
		ball := addBody(w, mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent(), &actor.Sphere{Radius: 0.02}, actor.BodyTypeDynamic, 0.5, 0)
		ball.IsBullet = bullet
		w.Step(sceneDt)

		// a motion through the plate, as if the solver had accelerated the ball
		motion := sweep{start: ball.Transform, end: actor.Transform{Position: mgl64.Vec3{10, 0, 0}, Rotation: mgl64.QuatIdent()}}
		ball.Transform = motion.end
		scratch := ccdPool.Get().(*ccdScratch)
		scratch.seen = make([]bool, len(w.Bodies))
		scratch.core.Radius = coreFraction * 0.02
		w.stopAtImpact(ball, &motion, 0.02, scratch)
		stopped := ball.Transform.Position.X() < 5
		if stopped != bullet {
			t.Errorf("bullet %v: the ball is at x=%.3f", bullet, ball.Transform.Position.X())
		}
	}
}
