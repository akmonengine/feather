package main

import (
	"fmt"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

// CollisionDebugger interface pour instrumenter les collisions
type CollisionDebugger interface {
	DebugGJK(bodyA, bodyB *actor.RigidBody, direction mgl64.Vec3)
	DebugEPA(bodyA, bodyB *actor.RigidBody, simplex gjk.Simplex)
	DebugManifold(bodyA, bodyB *actor.RigidBody, contacts []constraint.ContactPoint)
	DebugContactConstraint(bodyA, bodyB *actor.RigidBody, contact *constraint.ContactConstraint)
}

// SimpleDebugger implémente l'interface pour afficher les infos
type SimpleDebugger struct{}

func (d *SimpleDebugger) DebugGJK(bodyA, bodyB *actor.RigidBody, direction mgl64.Vec3) {
	fmt.Printf("🔍 GJK Debug:\n")
	fmt.Printf("   Body A pos: %v\n", bodyA.Transform.Position)
	fmt.Printf("   Body B pos: %v\n", bodyB.Transform.Position)
	fmt.Printf("   Direction: %v\n", direction)
	fmt.Printf("   Distance: %v\n", direction.Len())
}

func (d *SimpleDebugger) DebugEPA(bodyA, bodyB *actor.RigidBody, simplex gjk.Simplex) {
	fmt.Printf("🔧 EPA Debug:\n")
	fmt.Printf("   Simplex points: %d\n", len(simplex))
	for i, point := range simplex {
		fmt.Printf("   Point %d: %v (distance: %v)\n", i, point, point.Len())
	}
}

func (d *SimpleDebugger) DebugManifold(bodyA, bodyB *actor.RigidBody, contacts []constraint.ContactPoint) {
	fmt.Printf("🎯 Manifold Debug:\n")
	fmt.Printf("   Contact points: %d\n", len(contacts))
	fmt.Printf("   Body A (plane) pos: %v\n", bodyA.Transform.Position)
	fmt.Printf("   Body B (cube) pos: %v\n", bodyB.Transform.Position)
	for i, point := range contacts {
		fmt.Printf("   Point %d: position=%v penetration=%.6f\n", i, point.Position, point.Penetration)

		// Calculer le bras de levier (r) pour chaque body
		rA := point.Position.Sub(bodyA.Transform.Position)
		rB := point.Position.Sub(bodyB.Transform.Position)

		fmt.Printf("      rA (plane): %v (len=%.3f)\n", rA, rA.Len())
		fmt.Printf("      rB (cube):  %v (len=%.3f)\n", rB, rB.Len())
	}
}

func (d *SimpleDebugger) DebugContactConstraint(bodyA, bodyB *actor.RigidBody, contact *constraint.ContactConstraint) {
	fmt.Printf("⚙️  Contact Constraint Debug:\n")
	fmt.Printf("   Body A velocity: %v\n", bodyA.Velocity)
	fmt.Printf("   Body A angular velocity: %v\n", bodyA.AngularVelocity)
	fmt.Printf("   Body B velocity: %v\n", bodyB.Velocity)
	fmt.Printf("   Body B angular velocity: %v\n", bodyB.AngularVelocity)
	fmt.Printf("   Normal: %v\n", contact.Normal)
	fmt.Printf("   Points: %d\n", len(contact.Points))
}

// DebugGJKWrapper wraps GJK with debugging
func DebugGJK(bodyA, bodyB *actor.RigidBody, direction mgl64.Vec3, debugger CollisionDebugger) (bool, gjk.Simplex) {
	debugger.DebugGJK(bodyA, bodyB, direction)

	// Call actual GJK
	collides, simplex := gjk.GJK(bodyA, bodyB)

	return collides, simplex
}

// SetupScene creates the test scene with a plane and cube
func SetupScene() (*feather.World, *actor.RigidBody, *actor.RigidBody, CollisionDebugger) {
	debugger := &SimpleDebugger{}
	world := &feather.World{
		Gravity:  mgl64.Vec3{0, -9.81, 0},
		Substeps: 1,
	}

	// Create ground plane (y=0)
	planeShape := &actor.Plane{
		Normal:   mgl64.Vec3{0, 1, 0},
		Distance: 0.0,
	}

	// The plane's transform position should be at the plane's surface
	// For a plane at y=0 (distance=0), the position is at the origin
	planeTransform := actor.Transform{
		Position: mgl64.Vec3{0, 0, 0},
	}

	planeBody := actor.NewRigidBody(planeTransform, planeShape, actor.BodyTypeStatic, 0.0)
	world.AddBody(planeBody)

	// Create cube with scale incorporated in half extents
	boxShape := &actor.Box{
		HalfExtents: mgl64.Vec3{1.5, 1.5, 1.5}, // scale [3.0, 3.0, 3.0]
	}

	// Test avec cube qui tombe à plat avec restitution
	cubeTransform := actor.Transform{
		Position: mgl64.Vec3{-5.0, 5.0, -5.0}, // En hauteur
		Rotation: mgl64.QuatRotate(70.0, mgl64.Vec3{0, 0, 1}),
	}

	cubeBody := actor.NewRigidBody(cubeTransform, boxShape, actor.BodyTypeDynamic, 1.0)
	cubeBody.Material.Restitution = 0.8 // Haute restitution pour tester les rebonds

	world.AddBody(cubeBody)

	return world, planeBody, cubeBody, debugger
}

// TestCubeRotation teste le comportement du cube en rotation
func TestCubeRotation() {
	fmt.Println("🧪 Test d'intégration: Cube en rotation constante")
	fmt.Println("==================================================")

	world, planeBody, cubeBody, debugger := SetupScene()

	fmt.Printf("Configuration initiale:\n")
	fmt.Printf("  Plan: position %v\n", planeBody.Transform.Position)
	fmt.Printf("  Cube: position %v, rotation %v\n",
		cubeBody.Transform.Position,
		cubeBody.Transform.Rotation)
	fmt.Printf("  Gravité: %v\n", world.Gravity)
	fmt.Println()

	// Simulation pas à pas pour déboguer
	const dt float64 = 1.0 / 60.0
	const maxSteps int = 200

	for step := 0; step < maxSteps; step++ {
		fmt.Printf("--- ÉTAPE %d ---\n", step+1)
		fmt.Printf("Cube état AVANT:\n")
		fmt.Printf("  Position: %v\n", cubeBody.Transform.Position)
		fmt.Printf("  Velocity: %v\n", cubeBody.Velocity)
		fmt.Printf("  Angular Velocity: %v (len=%.3f)\n", cubeBody.AngularVelocity, cubeBody.AngularVelocity.Len())
		fmt.Printf("  Rotation: %v\n", cubeBody.Transform.Rotation)

		// Test collision GJK/EPA
		direction := mgl64.Vec3{0, -1, 0} // Gravity direction
		collides, simplex := DebugGJK(planeBody, cubeBody, direction, debugger)

		if collides {
			fmt.Printf("  Collision détectée!\n")
			debugger.DebugEPA(planeBody, cubeBody, simplex)

			// Manually call EPA to get manifold debug
			contact, err := epa.EPA(planeBody, cubeBody, simplex)
			if err == nil {
				debugger.DebugManifold(planeBody, cubeBody, contact.Points)
				fmt.Printf("  Contact Normal: %v\n", contact.Normal)
			}
		} else {
			fmt.Printf("  Pas de collision\n")
		}

		// Step physics
		world.Step(dt)

		fmt.Printf("Cube état APRÈS:\n")
		fmt.Printf("  Position: %v\n", cubeBody.Transform.Position)
		fmt.Printf("  Velocity: %v\n", cubeBody.Velocity)
		fmt.Printf("  Angular Velocity: %v (len=%.3f)\n", cubeBody.AngularVelocity, cubeBody.AngularVelocity.Len())
		fmt.Printf("  Rotation: %v\n", cubeBody.Transform.Rotation)

		// Calculer le changement de rotation
		qDelta := cubeBody.Transform.Rotation.Mul(cubeBody.PreviousTransform.Rotation.Conjugate()).Normalize()
		fmt.Printf("  Rotation delta: qDelta=%v (|V|=%.6f)\n", qDelta, qDelta.V.Len())
		fmt.Println()
	}

	fmt.Println("Test terminé!")
}

func main() {
	TestCubeRotation()
}
