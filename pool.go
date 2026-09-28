package feather

import (
	"runtime"
	"sync/atomic"
)

// spinsBeforeYield: an idle worker checks for new work this many times before letting other goroutines run
const spinsBeforeYield = 64

// workerPool runs the stages of a step in parallel.
// The workers are created once, and sleep between the steps (waiting on a channel, no CPU used).
// During a step they wait for the stages by spinning: a stage is often too short to wake up a goroutine.
// Each index is processed exactly once and writes only its own result: the order doesn't matter.
type workerPool struct {
	helpers    int // workers - 1, the caller is the last worker
	wake       chan struct{}
	generation atomic.Uint64
	parking    atomic.Bool
	awake      atomic.Int64 // helpers not sleeping
	pending    atomic.Int64 // helpers still working on the current job
	next       atomic.Int64 // next index to process
	job        func(i int)
	count      int64
	chunkSize  int64
	start      uint64 // generation when the helpers are woken up
}

// begin wakes the helpers up for a step. They are created the first time (or if the count of workers changes)
func (p *workerPool) begin(workers int) {
	if p.helpers != workers-1 {
		p.close()
		p.helpers = workers - 1
		p.wake = make(chan struct{}, p.helpers)
		for range p.helpers {
			go p.loop()
		}
	}

	p.parking.Store(false)
	p.start = p.generation.Load()
	p.awake.Store(int64(p.helpers))
	for range p.helpers {
		p.wake <- struct{}{}
	}
}

// end puts the helpers to sleep until the next step
func (p *workerPool) end() {
	if p.helpers == 0 {
		return
	}
	p.parking.Store(true)
	for spins := 0; p.awake.Load() != 0; spins++ {
		if spins > spinsBeforeYield {
			runtime.Gosched()
		}
	}
}

// close stops the helpers
func (p *workerPool) close() {
	if p.wake != nil {
		close(p.wake)
		p.wake = nil
	}
	p.helpers = 0
}

func (p *workerPool) loop() {
	wake := p.wake
	for {
		if _, ok := <-wake; !ok {
			return
		}

		seen, spins := p.start, 0
		for !p.parking.Load() {
			generation := p.generation.Load()
			if generation == seen {
				spins++
				if spins > spinsBeforeYield {
					runtime.Gosched()
				}
				continue
			}

			seen, spins = generation, 0
			p.work()
			p.pending.Add(-1)
		}
		p.awake.Add(-1)
	}
}

// run calls job(i) for each i in [0, count), on all the workers (if they are awake)
func (p *workerPool) run(count int, chunkSize int, job func(i int)) {
	if p.helpers == 0 || p.parking.Load() || count <= chunkSize {
		for i := 0; i < count; i++ {
			job(i)
		}
		return
	}

	p.job, p.count, p.chunkSize = job, int64(count), int64(chunkSize)
	p.next.Store(0)
	p.pending.Store(int64(p.helpers))
	p.generation.Add(1)

	p.work()
	for spins := 0; p.pending.Load() != 0; spins++ {
		if spins > spinsBeforeYield {
			runtime.Gosched()
		}
	}
	// the job references the World: the workers must not keep it alive
	p.job = nil
}

func (p *workerPool) work() {
	for {
		start := p.next.Add(p.chunkSize) - p.chunkSize
		if start >= p.count {
			return
		}
		end := min(start+p.chunkSize, p.count)
		for i := start; i < end; i++ {
			p.job(int(i))
		}
	}
}
