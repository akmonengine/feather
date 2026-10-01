package feather

import (
	"math"
	"math/rand"
	"testing"

	"github.com/akmonengine/feather/actor"
	"github.com/go-gl/mathgl/mgl64"
)

// scatteredScene: count boxes and spheres of 0.5 m scattered on a ground of 100 x 100 m, static or dynamic and asleep
func scatteredScene(count int, static bool) *World {
	w := newScene(1)
	addGround(w, 0.6)
	r := rand.New(rand.NewSource(1))
	bodyType := actor.BodyTypeDynamic
	if static {
		bodyType = actor.BodyTypeStatic
	}
	for i := 0; i < count; i++ {
		var shape actor.ShapeInterface = cube()
		if i%2 == 1 {
			shape = &actor.Sphere{Radius: cubeHalf}
		}
		addBody(w, mgl64.Vec3{r.Float64()*100 - 50, cubeHalf, r.Float64()*100 - 50}, mgl64.QuatIdent(), shape, bodyType, 0.6, 0)
	}
	simulate(w, 1, nil)
	return w
}

// scatteredRays: origins 1 m over the ground of scatteredScene, and level directions
func scatteredRays(count int) ([]mgl64.Vec3, []mgl64.Vec3) {
	r := rand.New(rand.NewSource(2))
	origins, directions := make([]mgl64.Vec3, count), make([]mgl64.Vec3, count)
	for i := range origins {
		origins[i] = mgl64.Vec3{r.Float64()*100 - 50, 1, r.Float64()*100 - 50}
		angle := r.Float64() * 2 * math.Pi
		directions[i] = mgl64.Vec3{math.Cos(angle), 0, math.Sin(angle)}
	}
	return origins, directions
}

// BenchmarkRaycast: a ray of 2 m down (a foot), and a level ray of 50 m (a line of sight), among 1000 bodies
func BenchmarkRaycast(b *testing.B) {
	const rays = 10000
	origins, directions := scatteredRays(rays)
	filter := DefaultQueryFilter()
	for _, scene := range []struct {
		name   string
		static bool
	}{{"static", true}, {"asleep", false}} {
		w := scatteredScene(1000, scene.static)
		b.Run(scene.name+"/down 2 m", func(b *testing.B) {
			b.ReportAllocs()
			hits := 0
			for i := 0; i < b.N; i++ {
				if _, ok := w.Raycast(origins[i%rays], mgl64.Vec3{0, -2, 0}, filter); ok {
					hits++
				}
			}
			b.ReportMetric(float64(hits)/float64(b.N), "hits/ray")
		})
		b.Run(scene.name+"/level 50 m", func(b *testing.B) {
			b.ReportAllocs()
			hits := 0
			for i := 0; i < b.N; i++ {
				origin := origins[i%rays]
				origin[1] = cubeHalf
				if _, ok := w.Raycast(origin, directions[i%rays].Mul(50), filter); ok {
					hits++
				}
			}
			b.ReportMetric(float64(hits)/float64(b.N), "hits/ray")
		})
	}
}

// BenchmarkSweep: a sphere of 10 cm and a capsule moved 2 m down, among 1000 bodies
func BenchmarkSweep(b *testing.B) {
	const sweeps = 10000
	origins, _ := scatteredRays(sweeps)
	filter := DefaultQueryFilter()
	w := scatteredScene(1000, true)
	for _, mover := range []struct {
		name  string
		shape actor.ShapeInterface
	}{{"sphere", &actor.Sphere{Radius: 0.1}}, {"capsule", &actor.Capsule{HalfHeight: 0.3, Radius: 0.1}}, {"box", cube()}} {
		b.Run(mover.name+" down 2 m", func(b *testing.B) {
			b.ReportAllocs()
			hits := 0
			for i := 0; i < b.N; i++ {
				start := actor.Transform{Position: origins[i%sweeps].Add(mgl64.Vec3{0, 0.5, 0}), Rotation: mgl64.QuatIdent()}
				if _, ok := w.Sweep(mover.shape, start, mgl64.Vec3{0, -2, 0}, filter); ok {
					hits++
				}
			}
			b.ReportMetric(float64(hits)/float64(b.N), "hits/sweep")
		})
	}
}

// BenchmarkOverlap: the bodies in a sphere of 1 m, among 1000 bodies; BenchmarkSyncQueries: 1000 bodies which
// didn't change
func BenchmarkOverlap(b *testing.B) {
	const overlaps = 10000
	origins, _ := scatteredRays(overlaps)
	filter := DefaultQueryFilter()
	w := scatteredScene(1000, true)
	ball := &actor.Sphere{Radius: 1}
	bodies := make([]*actor.RigidBody, 0, 64)
	b.ReportAllocs()
	found := 0
	for i := 0; i < b.N; i++ {
		bodies = w.Overlap(ball, actor.Transform{Position: origins[i%overlaps], Rotation: mgl64.QuatIdent()}, filter, bodies[:0])
		found += len(bodies)
	}
	b.ReportMetric(float64(found)/float64(b.N), "bodies/overlap")
}

func BenchmarkSyncQueries(b *testing.B) {
	for _, scene := range []struct {
		name   string
		static bool
	}{{"static", true}, {"asleep", false}} {
		w := scatteredScene(1000, scene.static)
		b.Run(scene.name, func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				w.SyncQueries()
			}
		})
	}
}
