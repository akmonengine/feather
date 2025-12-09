package feather

import (
	"sync"

	"github.com/akmonengine/feather/actor"
	"github.com/akmonengine/feather/constraint"
	"github.com/akmonengine/feather/epa"
	"github.com/akmonengine/feather/gjk"
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
	collisionPair := GJK(pairs)
	contactsChan := EPA(collisionPair)

	contacts := make([]*constraint.ContactConstraint, 0)
	for c := range contactsChan {
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
