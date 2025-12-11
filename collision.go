package feather

import (
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
	"github.com/go-gl/mathgl/mgl64"
)

const STIFF_COMPLIANCE = CONCRETE_COMPLIANCE

const (
	CONCRETE_COMPLIANCE = 0.04e-9
	WOOD_COMPLIANCE     = 0.16e-9
	LEATHER_COMPLIANCE  = 14e-8
	TENDON_COMPLIANCE   = 0.2e-7
	RUBBER_COMPLIANCE   = 1e-6
	MUSCLE_COMPLIANCE   = 0.2e-3
	FAT_COMPLIANCE      = 1e-3
)

// CollisionPair represents a pair of rigid bodies that potentially collide
type CollisionPair struct {
	BodyA   *actor.RigidBody
	BodyB   *actor.RigidBody
	simplex gjk.Simplex
}

// BroadPhase performs broad-phase collision detection using AABB overlap tests
// It returns pairs of bodies whose AABBs overlap and might be colliding
// This is an O(n²) brute-force approach suitable for small numbers of bodies

func BroadPhase(bodies []*actor.RigidBody) <-chan CollisionPair {
	// Brute force: test all pairs
	checkingPairs := make(chan CollisionPair, WORKERS*4)

	var wg sync.WaitGroup
	n := len(bodies)
	chunkSize := (n + WORKERS - 1) / WORKERS

	for workerID := 0; workerID < WORKERS; workerID++ {
		wg.Add(1)
		go func(start, end int) {
			defer wg.Done()
			for i := start; i < end; i++ {
				bodyA := bodies[i]

				for j := i + 1; j < n; j++ {
					bodyB := bodies[j]

					// Skip if both bodies are static (static-static collisions don't matter)
					if bodyA.BodyType == actor.BodyTypeStatic && bodyB.BodyType == actor.BodyTypeStatic {
						continue
					}
					if bodyA.IsSleeping && bodyB.IsSleeping {
						continue
					}

					_, aIsPlane := bodyA.Shape.(*actor.Plane)
					_, bIsPlane := bodyB.Shape.(*actor.Plane)
					if aIsPlane || bIsPlane {
						checkingPairs <- CollisionPair{BodyA: bodyA, BodyB: bodyB}
						continue
					}

					aabbA := bodyA.Shape.GetAABB()
					aabbB := bodyB.Shape.GetAABB()
					if aabbA.Overlaps(aabbB) {
						checkingPairs <- CollisionPair{BodyA: bodyA, BodyB: bodyB}
					}
				}
			}
		}(workerID*chunkSize, min((workerID+1)*chunkSize, n))
	}

	go func() {
		wg.Wait()
		close(checkingPairs)
	}()

	return checkingPairs
}

func NarrowPhase(pairs <-chan CollisionPair) []*constraint.ContactConstraint {
	// Dispatcher: separate pairs with planes, and normal convex objects
	planePairs := make(chan CollisionPair, WORKERS)
	gjkPairs := make(chan CollisionPair, WORKERS)

	go func() {
		defer close(planePairs)
		defer close(gjkPairs)

		for pair := range pairs {
			_, aIsPlane := pair.BodyA.Shape.(*actor.Plane)
			_, bIsPlane := pair.BodyB.Shape.(*actor.Plane)

			if aIsPlane || bIsPlane {
				planePairs <- pair
			} else {
				gjkPairs <- pair
			}
		}
	}()

	// Canal pour collecter tous les contacts
	allContacts := make(chan *constraint.ContactConstraint, WORKERS*2)
	var wg sync.WaitGroup
	// Path 1: GJK/EPA for convex objects
	wg.Add(1)
	go func() {
		defer wg.Done()
		collisionPairs := GJK(gjkPairs)
		contactsChan := EPA(collisionPairs)
		for contact := range contactsChan {
			allContacts <- contact
		}
	}()

	// Path 2: analytic collisions with planes
	wg.Add(1)
	go func() {
		defer wg.Done()
		contactsChan := collidePlane(planePairs)
		for contact := range contactsChan {
			allContacts <- contact
		}
	}()

	// Fermer le canal de sortie quand tout est fini
	go func() {
		wg.Wait()
		close(allContacts)
	}()

	// Collecter tous les contacts
	contacts := make([]*constraint.ContactConstraint, 0)
	for c := range allContacts {
		contacts = append(contacts, c)
	}
	return contacts
}

func GJK(pairChan <-chan CollisionPair) <-chan CollisionPair {
	collisionChan := make(chan CollisionPair, WORKERS)

	go func() {
		var wg sync.WaitGroup
		defer close(collisionChan)

		for range WORKERS {
			wg.Add(1)
			go func() {
				defer wg.Done()

				for p := range pairChan {
					if collision, simplex := gjk.GJK(p.BodyA, p.BodyB); collision {
						p.simplex = simplex
						collisionChan <- p
					}
				}
			}()

		}
		wg.Wait()
	}()

	return collisionChan
}

func EPA(p <-chan CollisionPair) <-chan *constraint.ContactConstraint {
	ch := make(chan *constraint.ContactConstraint, WORKERS)

	go func() {
		var wg sync.WaitGroup
		defer close(ch)

		for range WORKERS {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for pair := range p {
					contact, err := epa.EPA(pair.BodyA, pair.BodyB, pair.simplex)
					if err != nil {
						continue
					}
					ch <- &contact
				}
			}()
		}

		wg.Wait()
	}()

	return ch
}

func collidePlane(pairs <-chan CollisionPair) <-chan *constraint.ContactConstraint {
	ch := make(chan *constraint.ContactConstraint, WORKERS)

	go func() {
		var wg sync.WaitGroup
		defer close(ch)

		for range WORKERS {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for pair := range pairs {
					// Identifier quel body est le plan
					var plane *actor.Plane
					var object *actor.RigidBody
					var planeBody *actor.RigidBody
					var contactNormal mgl64.Vec3

					if p, ok := pair.BodyA.Shape.(*actor.Plane); ok {
						plane = p
						planeBody = pair.BodyA
						object = pair.BodyB
						contactNormal = plane.Normal
					} else if p, ok := pair.BodyB.Shape.(*actor.Plane); ok {
						plane = p
						planeBody = pair.BodyB
						object = pair.BodyA
						contactNormal = plane.Normal.Mul(-1)
					} else {
						continue // No plane (should not happen, the data is prefiltered in NarrowPhase)
					}

					collision, result := object.Shape.CollideWithPlane(plane.Normal, plane.Distance, object.Transform)

					if !collision {
						continue
					}

					var points []constraint.ContactPoint
					for _, point := range result {
						points = append(points, constraint.ContactPoint{Position: point.Position, Penetration: point.Penetration})
					}

					// Créer la contrainte
					contact := &constraint.ContactConstraint{
						BodyA:       planeBody,
						BodyB:       object,
						Normal:      contactNormal,
						Points:      points,
						Compliance:  CONCRETE_COMPLIANCE,
						Restitution: constraint.ComputeRestitution(planeBody.Material, object.Material),
					}

					ch <- contact
				}
			}()
		}

		wg.Wait()
	}()

	return ch
}
