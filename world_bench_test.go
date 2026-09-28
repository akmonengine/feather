package feather

import (
	"fmt"
	"runtime"
	"testing"
	"time"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// benchScene is a pile of boxes and spheres falling on the ground, then resting on each other
func benchScene(count, workers int) *World {
	w := newScene(workers)
	addGround(w, 0.6)
	side := 1
	for side*side*4 < count {
		side++
	}
	for i := 0; i < count; i++ {
		x := float64(i%side)*0.55 - float64(side)*0.275
		z := float64((i/side)%side)*0.55 - float64(side)*0.275
		y := 0.3 + float64(i/(side*side))*0.55
		var shape actor.ShapeInterface = cube()
		if i%2 == 1 {
			shape = &actor.Sphere{Radius: cubeHalf}
		}
		addBody(w, mgl64.Vec3{x, y, z}, mgl64.QuatIdent(), shape, actor.BodyTypeDynamic, 0.6, 0)
	}
	return w
}

// BenchmarkWorldStep simulates the first second of the pile: the bodies fall and land on each other
func BenchmarkWorldStep(b *testing.B) {
	for _, count := range []int{100, 500, 2000} {
		for _, workers := range []int{1, 8} {
			b.Run(fmt.Sprintf("%d_bodies_%d_workers", count, workers), func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					b.StopTimer()
					w := benchScene(count, workers)
					b.StartTimer()
					simulate(w, 1, nil)
				}
			})
		}
	}
}

// After the first steps (buffers growing), a step doesn't allocate
func TestStepDoesNotAllocate(t *testing.T) {
	if raceEnabled {
		t.Skip("sync.Pool drops its items with the race detector")
	}
	for _, workers := range []int{1, 8} {
		w := benchScene(500, workers)
		// the bodies are still falling and colliding
		simulate(w, 0.2, nil)
		allocs := testing.AllocsPerRun(10, func() {
			w.Step(sceneDt)
		})
		if allocs > 0 {
			t.Errorf("workers=%d: %.1f allocations per step, want 0", workers, allocs)
		}
	}
}

// The workers sleep between the steps, and stop with Close, or when the World is not used anymore
func TestWorkersDoNotLeak(t *testing.T) {
	waitGoroutines := func(want int) int {
		n := runtime.NumGoroutine()
		for i := 0; i < 200 && n > want; i++ {
			runtime.GC()
			time.Sleep(5 * time.Millisecond)
			n = runtime.NumGoroutine()
		}
		return n
	}
	// the worlds of the previous tests are collected first
	before := waitGoroutines(0)

	w := benchScene(300, 8)
	simulate(w, 0.1, nil)
	if n := runtime.NumGoroutine(); n != before+7 {
		t.Errorf("%d goroutines during the simulation, want %d (7 workers)", n, before+7)
	}
	w.Close()
	if n := waitGoroutines(before); n != before {
		t.Errorf("%d goroutines after Close, want %d", n, before)
	}

	func() {
		abandoned := benchScene(300, 8)
		simulate(abandoned, 0.1, nil)
	}()
	if n := waitGoroutines(before); n != before {
		t.Errorf("%d goroutines after the World was abandoned, want %d", n, before)
	}
}
