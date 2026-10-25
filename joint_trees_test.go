package feather

import (
	"fmt"
	"math"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// swayingJoints: the joints of the swaying chains, as the bones of a character's Physics Asset: limited (a cone and a
// twist) and driven towards the rest, or free
type swayingJoints struct {
	name            string
	limited, driven bool
}

var swayingKinds = []swayingJoints{{"limited and driven", true, true}, {"limited", true, false}, {"free and driven", false, true}, {"free", false, false}}

// swayingChains: count chains of 5 capsules 0.3 m apart below a kinematic root each, 3 m apart from each other, on
// configurable joints with their point locked: a tree of joints per chain, no contact
func swayingChains(count, workers int, kind swayingJoints) (*World, []*actor.RigidBody) {
	const links, spacing = 5, 0.3
	w := newScene(workers)
	var roots []*actor.RigidBody
	link := func() actor.ShapeInterface { return &actor.Capsule{HalfHeight: 0.1, Radius: 0.05} }
	for c := range count {
		x, z := 3*float64(c%20), -3*float64(c/20)
		previous := addBody(w, mgl64.Vec3{x, 2.5, z}, mgl64.QuatIdent(), link(), actor.BodyTypeKinematic, 0.6, 0)
		roots = append(roots, previous)
		for i := 1; i <= links; i++ {
			body := addBody(w, mgl64.Vec3{x, 2.5 - spacing*float64(i), z}, mgl64.QuatIdent(), link(), actor.BodyTypeDynamic, 0.6, 0)
			joint := NewConfigurableJoint(previous, body, mgl64.Vec3{x, 2.5 - spacing*(float64(i)-0.5), z}, mgl64.Vec3{0, -1, 0})
			joint.TwistMotion, joint.SwingYMotion, joint.SwingZMotion = MotionFree, MotionFree, MotionFree
			if kind.limited {
				joint.TwistMotion, joint.TwistMin, joint.TwistMax = MotionLimited, -0.3, 0.3
				joint.SwingYMotion, joint.SwingLimitY = MotionLimited, 0.6
				joint.SwingZMotion, joint.SwingLimitZ = MotionLimited, 0.6
			}
			if kind.driven {
				joint.EnableAngularDrive, joint.AngularDriveHertz, joint.AngularDriveDampingRatio = true, 5, 0.7
			}
			w.AddJoint(joint)
			previous = body
		}
	}
	return w, roots
}

// sway turns each root around Z, ±0.5 rad at 2 rad/s at its own phase, then steps
func sway(w *World, roots []*actor.RigidBody, step int) {
	t := float64(step) * sceneDt
	for i, root := range roots {
		target := root.Transform
		target.Rotation = mgl64.QuatRotate(0.5*math.Sin(2*t+float64(i)), mgl64.Vec3{0, 0, 1})
		if err := root.SetKinematicTarget(target); err != nil {
			panic(err)
		}
	}
	w.Step(sceneDt)
}

// The trees of joints are solved in parallel: 60 swaying chains, limited and driven, give the same bits with 1, 4 and 8
// workers for 2 s
func TestSwayingChainsAreDeterministic(t *testing.T) {
	var reference []actor.Transform
	for _, workers := range []int{1, 4, 8} {
		w, _ := swayingChains(60, workers, swayingKinds[0])
		roots := w.Bodies[:0:0]
		for _, body := range w.Bodies {
			if body.BodyType == actor.BodyTypeKinematic {
				roots = append(roots, body)
			}
		}
		for step := range 120 {
			sway(w, roots, step)
		}
		w.Close()
		for i, body := range w.Bodies {
			if workers == 1 {
				reference = append(reference, body.Transform)
			} else if body.Transform != reference[i] {
				t.Fatalf("body %d: %v with %d workers, %v with 1", i, body.Transform, workers, reference[i])
			}
		}
	}
}

// A step of the swaying chains, solved in parallel or not, allocates nothing
func TestSwayingChainsDoNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 4} {
		w, roots := swayingChains(60, workers, swayingKinds[0])
		step := 0
		for ; step < 60; step++ {
			sway(w, roots, step)
		}
		if allocs := testing.AllocsPerRun(10, func() { sway(w, roots, step); step++ }); allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
		w.Close()
	}
}

// BenchmarkSwayingChains: a step of 200 swaying chains (1 200 bodies, 1 000 joints), each kind of joint, on 1 and 6
// workers
func BenchmarkSwayingChains(b *testing.B) {
	for _, workers := range []int{1, 6} {
		for _, kind := range swayingKinds {
			b.Run(fmt.Sprintf("%s, %d workers", kind.name, workers), func(b *testing.B) {
				w, roots := swayingChains(200, workers, kind)
				defer w.Close()
				step := 0
				for ; step < 60; step++ {
					sway(w, roots, step)
				}
				b.ReportAllocs()
				b.ResetTimer()
				for b.Loop() {
					sway(w, roots, step)
					step++
				}
			})
		}
	}
}
