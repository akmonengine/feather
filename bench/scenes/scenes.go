// Package scenes holds the reference scenes of the solver, after the samples of Solver2D (Erin Catto, 2024,
// https://box2d.org/posts/2024/02/solver2d/), rewritten in 3D and compared to Box3D (references.go). Each scene isolates
// a known difficulty of a solver (a stack, a high mass ratio, a chain, bodies created overlapping...) and measures it
// with numbers, not by eye.
//
// The same code runs on the working tree and on v0.2.0 (-tags v020): the scenes needing a feature missing in v0.2.0
// (capsules, joints) are skipped there. Every scene has 2 sizes: Small for the tests, Full for the bench.
package scenes

import (
	"fmt"
	"math"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// Each version runs at its own setting (adapter_current.go, adapter_v020.go): Dt of a step and the substeps.
// The contacts are springs of 30 Hz whatever the substeps: a light body under a load sinks by (mass ratio) g / ω² at
// rest, and more substeps make the stacks, the impacts and the far scenes better
const gravity = 9.81

// Size of a scene
type Size int

const (
	// Small for the tests
	Small Size = iota
	// Full for the bench
	Full
)

// Metric: a measure of a scene
type Metric struct {
	Value float64 `json:"value"`
	Unit  string  `json:"unit"`
}

// Result: the measures of a scene, by name
type Result map[string]Metric

// Player steps the world for seconds, calling each (if not nil) after every step. The bench times the steps
type Player func(w *feather.World, seconds float64, each func())

// Scene of reference
type Scene struct {
	Name string
	// capsules & joints: the features the scene needs
	capsules, joints bool
	// Run builds the scene at its size, plays it and measures it
	Run func(size Size, play Player) Result
	// Check the result against the criteria of the scene: nil if it passes
	Check func(Result) error
	// Reference: at least as good as Box3D on the same scene, at the size (nil if no measure is compared, see
	// references.go). Gap: the ticket following a known gap to Box3D, if any
	Reference func(Result, Size) error
	Gap       string
	// Draws: the measures of the scene which are a draw. They are decided by the rounding (a contact at the separation 0
	// solved as a spring or as a speculative row, 2 impacts in one order or the other): any change of the engine draws
	// them again, further than the tolerance of a measure. The bench follows their median over the variants of the scene.
	// Vary runs the variant at spread, from 0 to 1: a length of the scene (a drop height, a radius, a gap...) across a
	// finite range, where every variant is the same difficulty. Nil: no measure of the scene is a draw
	Draws []string
	Vary  func(size Size, spread float64, play Player) Result
}

// across: the variants of a scene built from a parameter, going from..to with the spread
func across(from, to float64, at func(parameter float64) func(Size, Player) Result) func(Size, float64, Player) Result {
	return func(size Size, spread float64, play Player) Result {
		return at(from+(to-from)*spread)(size, play)
	}
}

// Supported: the scene runs on this version
func (s Scene) Supported() bool {
	return (!s.capsules || hasCapsules) && (!s.joints || hasJoints)
}

// All the scenes, in the order of Solver2D
var All = []Scene{
	singleBox, warmStartEnergy, highMassRatio1, highMassRatio2, highMassRatio3, frictionRamp, overlapRecovery,
	verticalStack, pyramid, rush, doubleDomino, confined, cardHouse, circleStack, centeredImpact,
	bridge, ballAndChain, jointGrid, stretchedChain,
	farPyramid, farStack, farRecovery, farChain,
}

// Step: a Player without timing
func Step(w *feather.World, seconds float64, each func()) {
	for i := 0; i < int(math.Round(seconds/Dt)); i++ {
		w.Step(Dt)
		if each != nil {
			each()
		}
	}
}

// ========== BUILDING ==========

// material of a body
type material struct {
	friction, restitution, density float64
}

var defaultMaterial = material{friction: 0.6, density: 1}

func addBody(w *feather.World, position mgl64.Vec3, rotation mgl64.Quat, shape actor.ShapeInterface, bodyType actor.BodyType, m material) *actor.RigidBody {
	body := actor.NewRigidBody(pose(position, rotation), shape, bodyType, m.density)
	body.Material.StaticFriction, body.Material.DynamicFriction, body.Material.Restitution = m.friction, m.friction, m.restitution
	w.AddBody(body)
	return body
}

func box(w *feather.World, position mgl64.Vec3, half mgl64.Vec3, m material) *actor.RigidBody {
	return addBody(w, position, mgl64.QuatIdent(), &actor.Box{HalfExtents: half}, actor.BodyTypeDynamic, m)
}

func staticBox(w *feather.World, position mgl64.Vec3, rotation mgl64.Quat, half mgl64.Vec3, m material) *actor.RigidBody {
	return addBody(w, position, rotation, &actor.Box{HalfExtents: half}, actor.BodyTypeStatic, m)
}

func sphere(w *feather.World, position mgl64.Vec3, radius float64, m material) *actor.RigidBody {
	return addBody(w, position, mgl64.QuatIdent(), &actor.Sphere{Radius: radius}, actor.BodyTypeDynamic, m)
}

// ground: a plane through origin, normal Y
func ground(w *feather.World, origin mgl64.Vec3, m material) *actor.RigidBody {
	return addBody(w, origin, mgl64.QuatIdent(), &actor.Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: -origin.Y()}, actor.BodyTypeStatic, m)
}

// squarePyramid: layers of cubes of half size h, the base of count × count, the cubes spaced by gap (0 = touching)
func squarePyramid(w *feather.World, origin mgl64.Vec3, count int, h, gap float64, m material) []*actor.RigidBody {
	var bodies []*actor.RigidBody
	pitch := 2*h + gap
	for layer := 0; layer < count; layer++ {
		side := count - layer
		for i := 0; i < side; i++ {
			for k := 0; k < side; k++ {
				x := (float64(i) - float64(side-1)/2) * pitch
				z := (float64(k) - float64(side-1)/2) * pitch
				y := h + float64(layer)*pitch
				bodies = append(bodies, box(w, origin.Add(mgl64.Vec3{x, y, z}), mgl64.Vec3{h, h, h}, m))
			}
		}
	}
	return bodies
}

// ========== MEASURES ==========

func positions(bodies []*actor.RigidBody) []mgl64.Vec3 {
	result := make([]mgl64.Vec3, len(bodies))
	for i, b := range bodies {
		result[i] = b.Transform.Position
	}
	return result
}

// worstDrift: the largest move of a body since start (m)
func worstDrift(bodies []*actor.RigidBody, start []mgl64.Vec3) float64 {
	worst := 0.0
	for i, b := range bodies {
		worst = math.Max(worst, b.Transform.Position.Sub(start[i]).Len())
	}
	return worst
}

// maxSpeed of the bodies (m/s)
func maxSpeed(bodies []*actor.RigidBody) float64 {
	worst := 0.0
	for _, b := range bodies {
		worst = math.Max(worst, b.Velocity.Len())
	}
	return worst
}

// tilt of a body: the angle of its Y axis from the world Y (rad)
func tilt(b *actor.RigidBody) float64 {
	return math.Acos(math.Max(-1, math.Min(1, b.Transform.Rotation.Rotate(mgl64.Vec3{0, 1, 0}).Y())))
}

// boxOverlap: how much 2 boxes overlap along their best separating axis (the 15 axes of the separating axis theorem,
// exact for 2 boxes), 0 if they are apart
func boxOverlap(a, b *actor.RigidBody) float64 {
	axesOf := func(body *actor.RigidBody) [3]mgl64.Vec3 {
		r := body.Transform.Rotation
		return [3]mgl64.Vec3{r.Rotate(mgl64.Vec3{1, 0, 0}), r.Rotate(mgl64.Vec3{0, 1, 0}), r.Rotate(mgl64.Vec3{0, 0, 1})}
	}
	axesA, axesB := axesOf(a), axesOf(b)
	axes := append(axesA[:], axesB[:]...)
	for _, x := range axesA {
		for _, y := range axesB {
			if cross := x.Cross(y); cross.Len() > 1e-9 {
				axes = append(axes, cross.Normalize())
			}
		}
	}
	overlap := math.Inf(1)
	for _, n := range axes {
		maxA, minA := a.SupportWorld(n).Dot(n), a.SupportWorld(n.Mul(-1)).Dot(n)
		maxB, minB := b.SupportWorld(n).Dot(n), b.SupportWorld(n.Mul(-1)).Dot(n)
		overlap = math.Min(overlap, math.Min(maxA-minB, maxB-minA))
	}
	return math.Max(0, overlap)
}

// sphereOverlap: how much 2 spheres overlap, 0 if they are apart
func sphereOverlap(a, b *actor.RigidBody) float64 {
	ra, rb := a.Shape.(*actor.Sphere).Radius, b.Shape.(*actor.Sphere).Radius
	return math.Max(0, ra+rb-a.Transform.Position.Sub(b.Transform.Position).Len())
}

// link: a joint seen by the measures, its anchor in the local space of each body
type link struct {
	a, b           *actor.RigidBody
	localA, localB mgl64.Vec3
}

// newLink from the anchor in world space
func newLink(a, b *actor.RigidBody, anchor mgl64.Vec3) link {
	local := func(body *actor.RigidBody) mgl64.Vec3 {
		return body.Transform.Rotation.Conjugate().Rotate(anchor.Sub(body.Transform.Position))
	}
	return link{a: a, b: b, localA: local(a), localB: local(b)}
}

// gap: the distance between the anchor seen by both bodies (m), 0 for a joint kept exactly
func (l link) gap() float64 {
	return toWorld(l.a, l.localA).Sub(toWorld(l.b, l.localB)).Len()
}

// toWorld: a point of the body in world space
func toWorld(b *actor.RigidBody, local mgl64.Vec3) mgl64.Vec3 {
	return b.Transform.Position.Add(b.Transform.Rotation.Rotate(local))
}

func worstGap(links []link) float64 {
	worst := 0.0
	for _, l := range links {
		worst = math.Max(worst, l.gap())
	}
	return worst
}

// finiteBodies: no NaN or infinity
func finiteBodies(bodies []*actor.RigidBody) bool {
	for _, b := range bodies {
		for _, x := range []float64{b.Transform.Position.X(), b.Transform.Position.Y(), b.Transform.Position.Z(), b.Velocity.Len()} {
			if math.IsNaN(x) || math.IsInf(x, 0) {
				return false
			}
		}
	}
	return true
}

func mm(meters float64) Metric { return Metric{meters * 1000, "mm"} }

// ========== CRITERIA ==========

// atMost: the metric must not exceed the bound (in the unit of the metric)
func atMost(r Result, name string, bound float64, why string) error {
	if r[name].Value > bound {
		return fmt.Errorf("%s %.3f %s, at most %.3f (%s)", name, r[name].Value, r[name].Unit, bound, why)
	}
	return nil
}

// firstError of the checks
func firstError(errs ...error) error {
	for _, err := range errs {
		if err != nil {
			return err
		}
	}
	return nil
}
