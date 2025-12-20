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
	simplex *gjk.Simplex
}

// BroadPhase performs broad-phase collision detection using AABB overlap tests
// It returns pairs of bodies whose AABBs overlap and might be colliding
// This is an O(n²) brute-force approach suitable for small numbers of bodies
func BroadPhase(spatialGrid *SpatialGrid, bodies []*actor.RigidBody, workersCount int) <-chan Pair {
	spatialGrid.Clear()
	for i, body := range bodies {
		spatialGrid.Insert(i, body)
	}
	spatialGrid.SortCells()

	checkingPairs := spatialGrid.FindPairsParallel(bodies, workersCount)

	return checkingPairs
}

func NarrowPhase(pairs <-chan Pair, workersCount int) []*constraint.ContactConstraint {
	// Dispatcher: separate pairs with planes, and normal convex objects
	planePairs := make(chan Pair, workersCount)
	gjkPairs := make(chan Pair, workersCount)

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
	allContacts := make(chan *constraint.ContactConstraint, workersCount*2)
	var wg sync.WaitGroup
	// Path 1: GJK/EPA for convex objects
	wg.Add(1)
	go func() {
		defer wg.Done()
		collisionPairs := GJK(gjkPairs, workersCount)
		contactsChan := EPA(collisionPairs, workersCount)
		for contact := range contactsChan {
			allContacts <- contact
		}
	}()

	// Path 2: analytic collisions with planes
	wg.Add(1)
	go func() {
		defer wg.Done()
		contactsChan := collidePlane(planePairs, workersCount)
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
	//fmt.Println("COUNT PAIRS", len(contacts))
	return contacts
}

func GJK(pairChan <-chan Pair, workersCount int) <-chan CollisionPair {
	collisionChan := make(chan CollisionPair, workersCount)

	go func() {
		var wg sync.WaitGroup
		defer close(collisionChan)

		for range workersCount {
			wg.Add(1)
			go func() {
				defer wg.Done()

				for p := range pairChan {
					simplex := gjk.SimplexPool.Get().(*gjk.Simplex)
					simplex.Reset()

					if collision := gjk.GJK(p.BodyA, p.BodyB, simplex); collision {
						collisionChan <- CollisionPair{
							BodyA:   p.BodyA,
							BodyB:   p.BodyB,
							simplex: simplex,
						}
					} else {
						gjk.SimplexPool.Put(simplex)
					}
				}
			}()

		}
		wg.Wait()
	}()

	return collisionChan
}

func EPA(p <-chan CollisionPair, workersCount int) <-chan *constraint.ContactConstraint {
	ch := make(chan *constraint.ContactConstraint, workersCount)

	go func() {
		var wg sync.WaitGroup
		defer close(ch)

		for range workersCount {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for pair := range p {
					contact, err := epa.EPA(pair.BodyA, pair.BodyB, pair.simplex)
					gjk.SimplexPool.Put(pair.simplex)
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

func collidePlane(pairs <-chan Pair, workersCount int) <-chan *constraint.ContactConstraint {
	ch := make(chan *constraint.ContactConstraint, workersCount)

	go func() {
		var wg sync.WaitGroup
		defer close(ch)

		for range workersCount {
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
						BodyA:  planeBody,
						BodyB:  object,
						Normal: contactNormal,
						Points: points,
					}

					ch <- contact
				}
			}()
		}

		wg.Wait()
	}()

	return ch
}
