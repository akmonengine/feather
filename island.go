package feather

import (
	"math"

	"github.com/akmonengine/feather/actor"
)

// sleepIslands: the bodies touching each other form an island (as in Box2D). An island falls asleep when all
// its bodies are resting, and wakes up entirely when one of its bodies wakes up.
// A body can't sleep under a moving body anymore, and the bodies above a removed body wake up.
type sleepIslands struct {
	// union-find over the dynamic bodies of the solver
	parent   []int
	minTimer []float64
	island   []int

	// the sleeping islands, and the island of each sleeping body
	islands  [][]*actor.RigidBody
	free     []int
	islandOf map[*actor.RigidBody]int
}

func (si *sleepIslands) find(i int) int {
	for si.parent[i] != i {
		si.parent[i] = si.parent[si.parent[i]]
		i = si.parent[i]
	}
	return i
}

func (si *sleepIslands) union(a, b int) {
	rootA, rootB := si.find(a), si.find(b)
	// the smallest index is the root: the islands never depend on the order of the contacts
	if rootA < rootB {
		si.parent[rootB] = rootA
	} else if rootB < rootA {
		si.parent[rootA] = rootB
	}
}

// update the sleep timers of the bodies, and puts to sleep the islands resting long enough
func (si *sleepIslands) update(s *solver, dt float64) {
	count := len(s.states)
	if cap(si.parent) < count {
		si.parent = make([]int, count)
		si.minTimer = make([]float64, count)
		si.island = make([]int, count)
	}
	si.parent, si.minTimer, si.island = si.parent[:count], si.minTimer[:count], si.island[:count]

	// ========== 1. Timers ==========
	for i := range s.states {
		body := s.states[i].body
		if body.Velocity.Len() < actor.DefaultSleepSpeed && body.AngularVelocity.Len() < actor.DefaultSleepSpeed {
			body.SleepTimer += dt
		} else {
			body.SleepTimer = 0
		}
		si.parent[i] = i
		si.minTimer[i] = math.Inf(1)
		si.island[i] = -1
	}

	// ========== 2. Islands: the dynamic bodies linked by a contact ==========
	for i := range s.constraints {
		c := &s.constraints[i]
		if c.indexA >= 0 && c.indexB >= 0 && c.pointsCount > 0 {
			si.union(c.indexA, c.indexB)
		}
	}
	for i := range s.states {
		root := si.find(i)
		si.minTimer[root] = math.Min(si.minTimer[root], s.states[i].body.SleepTimer)
	}

	// ========== 3. Sleep ==========
	for i := range s.states {
		root := si.find(i)
		if si.minTimer[root] < actor.DefaultTimeToSleep {
			continue
		}
		if si.island[root] < 0 {
			si.island[root] = si.newIsland()
		}
		body := s.states[i].body
		body.Sleep()
		k := si.island[root]
		si.islands[k] = append(si.islands[k], body)
		si.islandOf[body] = k
	}
}

func (si *sleepIslands) newIsland() int {
	if si.islandOf == nil {
		si.islandOf = make(map[*actor.RigidBody]int)
	}
	if n := len(si.free); n > 0 {
		k := si.free[n-1]
		si.free = si.free[:n-1]
		return k
	}
	si.islands = append(si.islands, nil)
	return len(si.islands) - 1
}

// wake the island of the body (if it is sleeping)
func (si *sleepIslands) wake(body *actor.RigidBody) {
	k, ok := si.islandOf[body]
	if !ok {
		body.WakeUp()
		return
	}
	for _, member := range si.islands[k] {
		member.WakeUp()
		delete(si.islandOf, member)
	}
	si.islands[k] = si.islands[k][:0]
	si.free = append(si.free, k)
}

// wakeWoken: a body woken up from outside (AddForce, WakeUp) wakes up its whole island
func (si *sleepIslands) wakeWoken() {
	for k := range si.islands {
		for _, member := range si.islands[k] {
			if !member.IsSleeping {
				si.wake(member)
				break
			}
		}
	}
}

// remove a body from its island: the island wakes up, the bodies it was holding must fall
func (si *sleepIslands) remove(body *actor.RigidBody) {
	si.wake(body)
}
