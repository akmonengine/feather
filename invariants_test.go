package feather

import (
	"fmt"
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// ========== INVARIANTS ==========
// Random scenes, each from a fixed seed, checked at every step against the laws every step must keep:
//   - finite positions, velocities & rotations, unit quaternions
//   - 1 and 8 workers give the same bits
//   - no body stays deeper than LinearSlop in a plane: a hit can push it deeper (the contacts are springs, as in Box2D
//     & Box3D: ~3 % of the random piles), then it gets out, less deep at each step. In a terrain, the depth is only
//     logged: the face contact of a triangle comes from the corners above it, a corner just beside it can stay a few
//     mm deep (#819, see ARCHITECTURE.md)
//   - a closed system never gains energy, with a restitution up to 0.5 (over it, a body spinning fast can bounce
//     higher than it fell, see ARCHITECTURE.md)
//   - in free flight, the momentum & the angular momentum are kept (the angular momentum during the steps without
//     contact)
// A scene is a pile on a plane, a pile on a terrain, or bodies & joints colliding in free flight.

const (
	// invariantScenes: the count of random scenes
	invariantScenes = 60

	// invariantSeconds: the simulated time of a scene (s)
	invariantSeconds = 3

	// unitTolerance of a quaternion: it is normalized at every sub-step, only the rounding remains
	unitTolerance = 1e-9

	// momentumTolerance: the impulses are applied to both bodies, opposite: only the rounding changes the momentum
	momentumTolerance = 1e-9

	// maxRestitution of the random scenes: the energy never grows up to it
	maxRestitution = 0.5
)

// sceneKind of a random scene
type sceneKind int

const (
	pileOnPlane sceneKind = iota
	pileOnTerrain
	freeFlight
	sceneKinds
)

func (k sceneKind) String() string {
	return [...]string{"pile on a plane", "pile on a terrain", "free flight"}[k]
}

// randomShape: a box, a sphere or a capsule, of 10 to 40 cm
func randomShape(r *rand.Rand) actor.ShapeInterface {
	switch r.Intn(3) {
	case 0:
		return &actor.Box{HalfExtents: mgl64.Vec3{0.05 + 0.25*r.Float64(), 0.02 + 0.2*r.Float64(), 0.05 + 0.25*r.Float64()}}
	case 1:
		return &actor.Sphere{Radius: 0.05 + 0.2*r.Float64()}
	}
	return &actor.Capsule{HalfHeight: 0.05 + 0.25*r.Float64(), Radius: 0.05 + 0.15*r.Float64()}
}

func randomRotation(r *rand.Rand) mgl64.Quat {
	return mgl64.QuatRotate(r.Float64()*2*math.Pi, mgl64.Vec3{r.Float64() - 0.5, r.Float64() - 0.5, r.Float64() - 0.5}.Normalize())
}

// randomVector of length up to size
func randomVector(r *rand.Rand, size float64) mgl64.Vec3 {
	return mgl64.Vec3{r.Float64() - 0.5, r.Float64() - 0.5, r.Float64() - 0.5}.Mul(2 * size)
}

// randomScene: the same seed & kind give the same scene, whatever the workers
func randomScene(seed int64, workers int) (*World, sceneKind) {
	r := rand.New(rand.NewSource(seed))
	kind := sceneKind(seed % int64(sceneKinds))
	w := newScene(workers)
	w.parallelFrom = 1
	count := 4 + r.Intn(9)
	// the same restitution for all the bodies & the ground (it is averaged between 2 bodies): the highest in a third
	// of the scenes
	restitution := r.Float64() * maxRestitution
	if r.Intn(3) == 0 {
		restitution = maxRestitution
	}

	if kind == freeFlight {
		// bodies thrown at each other around the origin, spinning up to 100 rad/s, some linked by joints
		w.Gravity = mgl64.Vec3{}
		var bodies []*actor.RigidBody
		for i := 0; i < count; i++ {
			position := mgl64.Vec3{float64(i%3) - 1, float64(i/3%3) - 1, float64(i/9) - 0.5}.Mul(1.2).Add(randomVector(r, 0.1))
			body := addBody(w, position, randomRotation(r), randomShape(r), actor.BodyTypeDynamic, r.Float64(), restitution)
			body.Velocity = position.Mul(-2).Add(randomVector(r, 2))
			body.AngularVelocity = randomVector(r, 100/math.Sqrt(3))
			bodies = append(bodies, body)
		}
		for i := 0; i+1 < len(bodies); i += 3 {
			a, b := bodies[i], bodies[i+1]
			middle := a.Transform.Position.Add(b.Transform.Position).Mul(0.5)
			if r.Intn(2) == 0 {
				w.AddJoint(NewBallJoint(a, b, middle, mgl64.Vec3{0, 1, 0}))
			} else {
				w.AddJoint(NewDistanceJoint(a, b, a.Transform.Position, b.Transform.Position))
			}
		}
		return w, kind
	}

	// a pile falling from 0.5 to 3 m, turned, some thrown down & spinning
	var field *actor.Heightfield
	if kind == pileOnTerrain {
		terrain := bumpyTerrain(w, seed)
		terrain.Material.Restitution = restitution
		field = terrain.Shape.(*actor.Heightfield)
	} else {
		addGround(w, r.Float64()).Material.Restitution = restitution
	}
	for i := 0; i < count; i++ {
		x, z := r.Float64()*3-1.5, r.Float64()*3-1.5
		ground := 0.0
		if field != nil {
			ground, _ = field.HeightAt(x, z)
		}
		position := mgl64.Vec3{x, ground + 0.5 + 0.4*float64(i) + r.Float64()*0.2, z}
		body := addBody(w, position, randomRotation(r), randomShape(r), actor.BodyTypeDynamic, r.Float64(), restitution)
		body.Velocity = randomVector(r, 3)
		body.AngularVelocity = randomVector(r, 20)
	}
	return w, kind
}

// mechanics: the quantities a closed system keeps
type mechanics struct {
	energy          float64
	momentum        mgl64.Vec3
	angularMomentum mgl64.Vec3
	// scale of the momentum & of the angular momentum, for the tolerances: the sums of their norms
	momentumScale        float64
	angularMomentumScale float64
	// potentialSlop: the energy gained if every body rose by LinearSlop
	potentialSlop float64
	// gyroscopicError: the change of the angular momentum allowed by the implicit gyroscopic torque during a step.
	// It is a first order method (Catto, GDC 2015): each sub-step changes the angular momentum I ω of a body by
	// less than |I ω| (|ω| h)²
	gyroscopicError float64
	// jointCouple: the change of the angular momentum allowed by the joints during a step. A soft joint pulls its 2
	// anchors, apart by its gap, with opposite impulses: a couple of gap × impulse at each sub-step
	jointCouple float64
	// depths of the bodies in the planes
	depths []float64
}

func measure(w *World) mechanics {
	var m mechanics
	gravity := w.Gravity.Len()
	h := sceneDt / float64(w.Substeps)
	for _, body := range w.Bodies {
		if body.BodyType != actor.BodyTypeDynamic {
			continue
		}
		mass := body.Material.GetMass()
		spin := body.GetInertiaWorld().Mul3x1(body.AngularVelocity)
		m.energy += 0.5*mass*body.Velocity.LenSqr() + 0.5*body.AngularVelocity.Dot(spin) - mass*w.Gravity.Dot(body.Transform.Position)
		m.momentum = m.momentum.Add(body.Velocity.Mul(mass))
		m.momentumScale += mass * body.Velocity.Len()
		orbital := body.Transform.Position.Cross(body.Velocity.Mul(mass))
		m.angularMomentum = m.angularMomentum.Add(orbital).Add(spin)
		m.angularMomentumScale += orbital.Len() + spin.Len()
		m.potentialSlop += mass * gravity * LinearSlop
		turn := body.AngularVelocity.Len() * h
		m.gyroscopicError += spin.Len() * float64(w.Substeps) * turn * turn
	}
	m.depths = planeDepths(w)
	for _, joint := range w.Joints {
		j := joint.base()
		gap := j.BodyA.Transform.ToWorld(j.LocalFrameA.Position).Sub(j.BodyB.Transform.ToWorld(j.LocalFrameB.Position)).Len()
		m.jointCouple += float64(w.Substeps) * gap * j.linearImpulse.Len()
	}
	return m
}

// planeDepths: the depth of each dynamic body in the static bodies which are not terrains (0 for the others)
func planeDepths(w *World) []float64 {
	depths := make([]float64, len(w.Bodies))
	for _, surface := range w.Bodies {
		if surface.BodyType != actor.BodyTypeStatic {
			continue
		}
		if _, terrain := surface.Shape.(*actor.Heightfield); terrain {
			continue
		}
		for i, body := range w.Bodies {
			if body.BodyType == actor.BodyTypeDynamic {
				depths[i] = math.Max(depths[i], surfaceDepth(surface, body))
			}
		}
	}
	return depths
}

// checkInvariants of the world after a step, against its state at the start of the scene (start) and of the step
// (before), and the same scene run with other workers (twin). Returns the first broken invariant
func checkInvariants(w, twin *World, kind sceneKind, start, before mechanics) error {
	for i, body := range w.Bodies {
		transform := body.Transform
		if !finite(transform.Position) || !finite(body.Velocity) || !finite(body.AngularVelocity) {
			return fmt.Errorf("body %d is not finite: %v %v %v", i, transform.Position, body.Velocity, body.AngularVelocity)
		}
		if length := transform.Rotation.Len(); math.Abs(length-1) > unitTolerance {
			return fmt.Errorf("body %d: the rotation is not a unit quaternion (%v)", i, length)
		}
		other := twin.Bodies[i]
		if transform != other.Transform || body.Velocity != other.Velocity || body.AngularVelocity != other.AngularVelocity {
			return fmt.Errorf("body %d: %v with %d workers, %v with %d", i, transform, w.Workers, other.Transform, twin.Workers)
		}
	}

	for i, depth := range planeDepths(w) {
		if depth > LinearSlop && before.depths[i] > LinearSlop && depth >= before.depths[i] {
			return fmt.Errorf("body %d (%T) stays %.2f mm in the ground (%.2f mm the step before)", i, w.Bodies[i].Shape,
				depth*1000, before.depths[i]*1000)
		}
	}

	now := measure(w)
	// the free fall integrated by the symplectic Euler loses m g² h² / 2 per sub-step, the restitution is at most 1:
	// the energy grows only when a contact pushes a body out of the ground, by LinearSlop at most
	if gain := now.energy - start.energy; gain > start.potentialSlop+1e-9*math.Abs(start.energy) {
		return fmt.Errorf("the energy grew by %.4f J (from %.4f J)", gain, start.energy)
	}
	if kind == freeFlight {
		if drift := now.momentum.Sub(start.momentum).Len(); drift > momentumTolerance*start.momentumScale {
			return fmt.Errorf("the momentum changed by %.3g kg·m/s (of %.3f)", drift, start.momentumScale)
		}
		// the gyroscopic error and the couple of the joints, before or after the step, whichever is larger
		allowed := math.Max(before.gyroscopicError, now.gyroscopicError) + math.Max(before.jointCouple, now.jointCouple)
		if drift := now.angularMomentum.Sub(before.angularMomentum).Len(); len(w.Contacts()) == 0 && drift > allowed {
			return fmt.Errorf("the angular momentum changed by %.4f kg·m²/s during the step (%.4f allowed)", drift, allowed)
		}
	}
	return nil
}

// worldTerrainDepth: the deepest body in a terrain of the world, 0 without terrain
func worldTerrainDepth(w *World) float64 {
	worst := 0.0
	for _, surface := range w.Bodies {
		if _, terrain := surface.Shape.(*actor.Heightfield); terrain {
			for _, body := range w.Bodies {
				if body.BodyType == actor.BodyTypeDynamic {
					worst = math.Max(worst, surfaceDepth(surface, body))
				}
			}
		}
	}
	return worst
}

// Every random scene keeps the invariants at every step. The scenes run in parallel
func TestInvariants(t *testing.T) {
	for seed := int64(0); seed < invariantScenes; seed++ {
		t.Run(fmt.Sprint(seed), func(t *testing.T) {
			t.Parallel()
			single, kind := randomScene(seed, 1)
			parallel, _ := randomScene(seed, 8)
			defer single.Close()
			defer parallel.Close()
			start := measure(single)
			before := start
			terrainDepth := 0.0
			for step := 0; step < int(math.Round(invariantSeconds/sceneDt)); step++ {
				single.Step(sceneDt)
				parallel.Step(sceneDt)
				err := checkInvariants(single, parallel, kind, start, before)
				before = measure(single)
				if err != nil {
					t.Fatalf("seed %d, %s, step %d: %v", seed, kind, step, err)
				}
				terrainDepth = math.Max(terrainDepth, worldTerrainDepth(single))
			}
			if kind == pileOnTerrain {
				t.Logf("seed %d: deepest in the terrain %.2f mm", seed, terrainDepth*1000)
			}
		})
	}
}
