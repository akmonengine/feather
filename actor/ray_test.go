package actor

import (
	"math"
	"math/rand"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// rayTolerance: a ray against a primitive is exact, to the rounding (criterion 5)
const rayTolerance = 1e-9

func near(a, b mgl64.Vec3, tolerance float64) bool {
	return a.Sub(b).Len() <= tolerance
}

// signedDistance from the point to the surface of the shape, negative inside, and the unit normal of the closest point
// of the surface (the gradient of the distance). The plane is the half space under it
func signedDistance(shape ShapeInterface, p mgl64.Vec3) (float64, mgl64.Vec3) {
	switch s := shape.(type) {
	case *Sphere:
		return p.Len() - s.Radius, p.Normalize()
	case *Capsule:
		onAxis := mgl64.Vec3{0, mgl64.Clamp(p.Y(), -s.HalfHeight, s.HalfHeight), 0}
		return p.Sub(onAxis).Len() - s.Radius, p.Sub(onAxis).Normalize()
	case *Plane:
		return p.Dot(s.Normal) + s.Distance, s.Normal
	case *Box:
		outside, inside, axis := mgl64.Vec3{}, math.Inf(-1), 0
		for k := 0; k < 3; k++ {
			d := math.Abs(p[k]) - s.HalfExtents[k]
			outside[k] = math.Copysign(math.Max(d, 0), p[k])
			if d > inside {
				inside, axis = d, k
			}
		}
		if distance := outside.Len(); distance > 0 {
			return distance, outside.Mul(1 / distance)
		}
		normal := mgl64.Vec3{}
		normal[axis] = math.Copysign(1, p[axis])
		return inside, normal
	}
	panic("unknown shape")
}

// Criterion 5: fraction and normal of a ray against a sphere, a box, a capsule (side and caps) and a plane, against
// the analytic answer
func TestCastRayIsAnalytic(t *testing.T) {
	sphere := &Sphere{Radius: 0.5}
	box := &Box{HalfExtents: mgl64.Vec3{0.5, 0.25, 1}}
	capsule := &Capsule{HalfHeight: 0.6, Radius: 0.3}
	tilted := mgl64.Vec3{1, 2, -0.5}.Normalize()
	plane := &Plane{Normal: tilted, Distance: -1.5}
	// where the ray from (-3, 0.3, 0.1) along x enters the sphere, and the cap of the capsule from (-2, 0.8, 0.1)
	sphereEntry := -math.Sqrt(0.5*0.5 - 0.3*0.3 - 0.1*0.1)
	capEntry := -math.Sqrt(0.3*0.3 - 0.2*0.2 - 0.1*0.1)
	cases := []struct {
		name                string
		shape               ShapeInterface
		origin, translation mgl64.Vec3
		fraction            float64
		normal              mgl64.Vec3
	}{
		{"sphere, through its center", sphere, mgl64.Vec3{0, 4, 0}, mgl64.Vec3{0, -7, 0}, 3.5 / 7, mgl64.Vec3{0, 1, 0}},
		{"sphere, off its center", sphere, mgl64.Vec3{-3, 0.3, 0.1}, mgl64.Vec3{10, 0, 0}, (3 + sphereEntry) / 10, mgl64.Vec3{sphereEntry, 0.3, 0.1}.Mul(2)},
		{"box, face +x", box, mgl64.Vec3{5, 0.1, -0.2}, mgl64.Vec3{-9, 0.2, 0.4}, 4.5 / 9, mgl64.Vec3{1, 0, 0}},
		{"box, face -x", box, mgl64.Vec3{-5, 0.1, -0.2}, mgl64.Vec3{9, 0.2, 0.4}, 4.5 / 9, mgl64.Vec3{-1, 0, 0}},
		{"box, face +y", box, mgl64.Vec3{0.2, 2.25, 0.3}, mgl64.Vec3{0.1, -4, -0.3}, 2.0 / 4, mgl64.Vec3{0, 1, 0}},
		{"box, face -y", box, mgl64.Vec3{0.2, -2.25, 0.3}, mgl64.Vec3{0.1, 4, -0.3}, 2.0 / 4, mgl64.Vec3{0, -1, 0}},
		{"box, face +z", box, mgl64.Vec3{0.2, 0.1, 3}, mgl64.Vec3{-0.3, -0.1, -8}, 2.0 / 8, mgl64.Vec3{0, 0, 1}},
		{"box, face -z", box, mgl64.Vec3{0.2, 0.1, -3}, mgl64.Vec3{-0.3, -0.1, 8}, 2.0 / 8, mgl64.Vec3{0, 0, -1}},
		{"box, along an axis", box, mgl64.Vec3{0.5, 3, 1}, mgl64.Vec3{0, -5, 0}, 2.75 / 5, mgl64.Vec3{0, 1, 0}},
		{"capsule, its side", capsule, mgl64.Vec3{2, 0.4, 0}, mgl64.Vec3{-4, 0.2, 0}, 1.7 / 4, mgl64.Vec3{1, 0, 0}},
		{"capsule, its top", capsule, mgl64.Vec3{0, 5, 0}, mgl64.Vec3{0, -8, 0}, 4.1 / 8, mgl64.Vec3{0, 1, 0}},
		{"capsule, its bottom", capsule, mgl64.Vec3{0, -5, 0}, mgl64.Vec3{0, 8, 0}, 4.1 / 8, mgl64.Vec3{0, -1, 0}},
		{"capsule, the side of its top cap", capsule, mgl64.Vec3{-2, 0.8, 0.1}, mgl64.Vec3{5, 0, 0}, (2 + capEntry) / 5, mgl64.Vec3{capEntry, 0.2, 0.1}.Mul(1 / 0.3)},
		{"capsule, across its axis above its side", capsule, mgl64.Vec3{0.1, 3, 0}, mgl64.Vec3{0, -4, 0}, (3 - 0.6 - math.Sqrt(0.3*0.3-0.1*0.1)) / 4, mgl64.Vec3{0.1, math.Sqrt(0.3*0.3 - 0.1*0.1), 0}.Mul(1 / 0.3)},
		{"plane", plane, tilted.Mul(4), tilted.Mul(-5), 2.5 / 5, tilted},
		{"plane, slanted ray", plane, mgl64.Vec3{0, 3, 0}, mgl64.Vec3{0, -4, 0}, (3*tilted.Y() - 1.5) / (4 * tilted.Y()), tilted},
	}
	for _, c := range cases {
		hit, ok := c.shape.CastRay(c.origin, c.translation, 1)
		if !ok {
			t.Errorf("%s: no hit", c.name)
			continue
		}
		if math.Abs(hit.Fraction-c.fraction) > rayTolerance || !near(hit.Normal, c.normal, rayTolerance) || hit.Triangle != NoTriangle {
			t.Errorf("%s: fraction %.12f normal %v triangle %d, want %.12f %v %d", c.name, hit.Fraction, hit.Normal, hit.Triangle, c.fraction, c.normal, NoTriangle)
		}
		// the same ray, cut right before its hit, then right at it
		if _, ok := c.shape.CastRay(c.origin, c.translation, c.fraction-1e-6); ok {
			t.Errorf("%s: a hit before the fraction %.6f", c.name, c.fraction-1e-6)
		}
		if _, ok := c.shape.CastRay(c.origin, c.translation, c.fraction+1e-6); !ok {
			t.Errorf("%s: no hit with a maximum fraction right after the hit", c.name)
		}
		// away from the shape
		if _, ok := c.shape.CastRay(c.origin, c.translation.Mul(-1), 1); ok {
			t.Errorf("%s: a hit with the ray going away", c.name)
		}
	}
}

// A hit exactly at the maximum fraction is a hit: the segment holds its end
func TestCastRayAtItsMaximumFraction(t *testing.T) {
	if NoTriangle != -1 {
		t.Errorf("NoTriangle is %d, want -1", NoTriangle)
	}
	field := NewHeightfield(3, 3, make([]float32, 9), mgl64.Vec3{1, 1, 1})
	shapes := map[string]ShapeInterface{
		"sphere":      &Sphere{Radius: 1},
		"box":         &Box{HalfExtents: mgl64.Vec3{1, 1, 1}},
		"capsule":     &Capsule{HalfHeight: 0.5, Radius: 0.5},
		"plane":       &Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: -1},
		"heightfield": field,
	}
	for name, shape := range shapes {
		// from the height 3 down to the top of the shape at the height 1 (0 for the heightfield): the half of the ray
		origin, translation := mgl64.Vec3{0.25, 3, 0.125}, mgl64.Vec3{0, -4, 0}
		if shape == field {
			translation = mgl64.Vec3{0, -6, 0}
		} else if name == "sphere" || name == "capsule" {
			origin = mgl64.Vec3{0, 3, 0}
		}
		hit, ok := shape.CastRay(origin, translation, 0.5)
		if !ok || hit.Fraction != 0.5 {
			t.Errorf("%s: hit %v at the fraction %g with the maximum fraction 0.5, want a hit at 0.5", name, ok, hit.Fraction)
		}
		if _, ok := shape.CastRay(origin, translation, 0.4999999); ok {
			t.Errorf("%s: a hit with the maximum fraction 0.4999999", name)
		}
	}
}

// Criterion 5, on random rays: the hit is the first point of the ray on the surface, with the normal of the surface.
// A miss never crosses the shape
func TestCastRayFindsTheFirstPointOfTheSurface(t *testing.T) {
	shapes := map[string]ShapeInterface{
		"sphere":       &Sphere{Radius: 0.7},
		"box":          &Box{HalfExtents: mgl64.Vec3{0.4, 0.9, 0.2}},
		"capsule":      &Capsule{HalfHeight: 0.8, Radius: 0.35},
		"ball capsule": &Capsule{HalfHeight: 0, Radius: 0.5},
		"plane":        &Plane{Normal: mgl64.Vec3{0.3, 1, -0.2}.Normalize(), Distance: 0.4},
	}
	const samples = 400
	for name, shape := range shapes {
		r := rand.New(rand.NewSource(11))
		hits, misses := 0, 0
		for i := 0; i < 20000; i++ {
			origin := mgl64.Vec3{r.Float64()*6 - 3, r.Float64()*6 - 3, r.Float64()*6 - 3}
			target := mgl64.Vec3{r.Float64()*2.4 - 1.2, r.Float64()*2.4 - 1.2, r.Float64()*2.4 - 1.2}
			translation := target.Sub(origin).Mul(0.5 + 1.5*r.Float64())
			if i%7 == 0 {
				// along an axis
				axis := r.Intn(3)
				translation = mgl64.Vec3{}
				translation[axis] = (target[axis] - origin[axis]) * 2
				origin[(axis+1)%3], origin[(axis+2)%3] = target[(axis+1)%3], target[(axis+2)%3]
			}
			if start, _ := signedDistance(shape, origin); start <= 0 {
				continue
			}
			maxFraction := 0.3 + 0.7*r.Float64()
			hit, ok := shape.CastRay(origin, translation, maxFraction)
			end := maxFraction
			if ok {
				hits++
				end = hit.Fraction
				distance, normal := signedDistance(shape, origin.Add(translation.Mul(hit.Fraction)))
				if hit.Fraction < 0 || hit.Fraction > maxFraction || math.Abs(distance) > rayTolerance {
					t.Fatalf("%s, ray %d: the hit at the fraction %.12f is %.3g m from the surface", name, i, hit.Fraction, distance)
				}
				if math.Abs(hit.Normal.Len()-1) > rayTolerance || hit.Normal.Dot(translation) > 0 {
					t.Fatalf("%s, ray %d: the normal %v is not a unit vector against the ray", name, i, hit.Normal)
				}
				// on an edge of the box, the normal is the one of the face the ray enters by
				if _, isBox := shape.(*Box); !isBox && !near(hit.Normal, normal, 1e-6) {
					t.Fatalf("%s, ray %d: normal %v, the surface has %v", name, i, hit.Normal, normal)
				}
			} else {
				misses++
			}
			for k := 0; k < samples; k++ {
				f := end * float64(k) / samples
				if distance, _ := signedDistance(shape, origin.Add(translation.Mul(f))); distance < -rayTolerance {
					t.Fatalf("%s, ray %d (hit %v at %.9f): the ray is %.3g m inside the shape at the fraction %.9f", name, i, ok, hit.Fraction, -distance, f)
				}
			}
		}
		if hits < 1000 || misses < 1000 {
			t.Errorf("%s: %d hits and %d misses, the rays don't cover both", name, hits, misses)
		}
	}
}

// Criterion 5: the normal of a box is the normal of the face the ray enters by
func TestCastRayBoxNormalIsTheEnteredFace(t *testing.T) {
	box := &Box{HalfExtents: mgl64.Vec3{0.4, 0.9, 0.2}}
	r := rand.New(rand.NewSource(5))
	for i := 0; i < 5000; i++ {
		origin := mgl64.Vec3{r.Float64()*6 - 3, r.Float64()*6 - 3, r.Float64()*6 - 3}
		target := mgl64.Vec3{r.Float64()*0.8 - 0.4, r.Float64()*1.8 - 0.9, r.Float64()*0.4 - 0.2}
		translation := target.Sub(origin).Mul(1.5)
		hit, ok := box.CastRay(origin, translation, 1)
		if distance, _ := signedDistance(box, origin); distance <= 0 {
			continue
		}
		if !ok {
			t.Fatalf("ray %d: no hit towards a point of the box", i)
		}
		point := origin.Add(translation.Mul(hit.Fraction))
		axes := 0
		for k := 0; k < 3; k++ {
			if hit.Normal[k] == 0 {
				continue
			}
			axes++
			if math.Abs(hit.Normal[k]) != 1 || math.Abs(point[k]-hit.Normal[k]*box.HalfExtents[k]) > rayTolerance {
				t.Fatalf("ray %d: normal %v at %v, not on this face", i, hit.Normal, point)
			}
		}
		if axes != 1 {
			t.Fatalf("ray %d: normal %v is not the normal of a face", i, hit.Normal)
		}
	}
}

// Criterion 6: a ray passing at r(1 + 1e-9) of the center of a sphere misses it, at r(1 - 1e-9) it hits. The same
// along the side and around a cap of a capsule, and past an edge of a box
func TestCastRayGrazing(t *testing.T) {
	const radius = 0.5
	sphere := &Sphere{Radius: radius}
	capsule := &Capsule{HalfHeight: 0.6, Radius: radius}
	box := &Box{HalfExtents: mgl64.Vec3{radius, 1, 1}}
	for _, side := range []float64{1, -1} {
		out, in := radius*(1+1e-9)*side, radius*(1-1e-9)*side
		cases := []struct {
			name      string
			shape     ShapeInterface
			at, along func(offset float64) mgl64.Vec3
		}{
			{"sphere", sphere, func(o float64) mgl64.Vec3 { return mgl64.Vec3{o, 5, 0} }, func(float64) mgl64.Vec3 { return mgl64.Vec3{0, -10, 0} }},
			{"sphere, slanted", sphere, func(o float64) mgl64.Vec3 { return mgl64.Vec3{3, 4, 0}.Add(mgl64.Vec3{0.8, -0.6, 0}.Mul(o)) }, func(float64) mgl64.Vec3 { return mgl64.Vec3{-6, -8, 0} }},
			{"capsule side", capsule, func(o float64) mgl64.Vec3 { return mgl64.Vec3{o, 0.2, 5} }, func(float64) mgl64.Vec3 { return mgl64.Vec3{0, 0, -10} }},
			{"capsule cap", capsule, func(o float64) mgl64.Vec3 { return mgl64.Vec3{0, 0.6 + math.Abs(o), 5} }, func(float64) mgl64.Vec3 { return mgl64.Vec3{0, 0, -10} }},
			{"capsule along its axis", capsule, func(o float64) mgl64.Vec3 { return mgl64.Vec3{o, 5, 0} }, func(float64) mgl64.Vec3 { return mgl64.Vec3{0, -10, 0} }},
			{"box", box, func(o float64) mgl64.Vec3 { return mgl64.Vec3{o, 5, 0} }, func(float64) mgl64.Vec3 { return mgl64.Vec3{0, -10, 0} }},
		}
		for _, c := range cases {
			if _, ok := c.shape.CastRay(c.at(out), c.along(out), 1); ok {
				t.Errorf("%s: a ray passing a billionth out of the shape hits it (side %+.0f)", c.name, side)
			}
			if _, ok := c.shape.CastRay(c.at(in), c.along(in), 1); !ok {
				t.Errorf("%s: a ray passing a billionth in the shape misses it (side %+.0f)", c.name, side)
			}
		}
	}
}

// Criterion 7: a ray which starts in a shape hits it at its origin, the normal against its direction. A ray without
// length is a point: a hit without normal if it is in the shape
func TestCastRayFromInside(t *testing.T) {
	plane := &Plane{Normal: mgl64.Vec3{0, 1, 0}, Distance: -1}
	cases := []struct {
		name            string
		shape           ShapeInterface
		inside, outside mgl64.Vec3
	}{
		{"sphere", &Sphere{Radius: 0.5}, mgl64.Vec3{0.1, -0.2, 0.3}, mgl64.Vec3{0.4, 0.4, 0.4}},
		{"box", &Box{HalfExtents: mgl64.Vec3{0.5, 0.25, 1}}, mgl64.Vec3{0.4, -0.2, 0.9}, mgl64.Vec3{0.4, 0.3, 0.9}},
		{"capsule, in its cylinder", &Capsule{HalfHeight: 0.6, Radius: 0.3}, mgl64.Vec3{0.2, 0.5, 0.1}, mgl64.Vec3{0.25, 0.5, 0.25}},
		{"capsule, in its cap", &Capsule{HalfHeight: 0.6, Radius: 0.3}, mgl64.Vec3{0.1, 0.8, 0.1}, mgl64.Vec3{0.2, 0.85, 0.1}},
		{"plane", plane, mgl64.Vec3{3, 0.5, -2}, mgl64.Vec3{3, 1.5, -2}},
	}
	directions := []mgl64.Vec3{{3, 0, 0}, {0, -0.01, 0}, {1, 2, -3}, {0, 7, 0}}
	for _, c := range cases {
		for _, direction := range directions {
			hit, ok := c.shape.CastRay(c.inside, direction, 1)
			want := direction.Normalize().Mul(-1)
			if !ok || hit.Fraction != 0 || !near(hit.Normal, want, 1e-12) || hit.Triangle != NoTriangle {
				t.Errorf("%s, from inside along %v: hit %v fraction %g normal %v, want the fraction 0 and the normal %v", c.name, direction, ok, hit.Fraction, hit.Normal, want)
			}
		}
		hit, ok := c.shape.CastRay(c.inside, mgl64.Vec3{}, 1)
		if !ok || hit.Fraction != 0 || hit.Normal != (mgl64.Vec3{}) {
			t.Errorf("%s: a point in the shape gives hit %v fraction %g normal %v, want a hit without normal", c.name, ok, hit.Fraction, hit.Normal)
		}
		if _, ok := c.shape.CastRay(c.outside, mgl64.Vec3{}, 1); ok {
			t.Errorf("%s: a point out of the shape hits it", c.name)
		}
	}
}

// Criterion 13: a ray with a NaN or an infinite coordinate hits nothing, and doesn't panic
func TestCastRayNotFinite(t *testing.T) {
	shapes := []ShapeInterface{&Sphere{Radius: 0.5}, &Box{HalfExtents: mgl64.Vec3{0.5, 0.25, 1}}, &Capsule{HalfHeight: 0.6, Radius: 0.3}, &Plane{Normal: mgl64.Vec3{0, 1, 0}}}
	bad := []float64{math.NaN(), math.Inf(1), math.Inf(-1)}
	for _, shape := range shapes {
		for _, value := range bad {
			for k := 0; k < 3; k++ {
				origin, translation := mgl64.Vec3{0, 0.1, 0}, mgl64.Vec3{0, -1, 0}
				origin[k] = value
				if _, ok := shape.CastRay(origin, translation, 1); ok {
					t.Errorf("%T: a hit with the origin %v", shape, origin)
				}
				origin, translation = mgl64.Vec3{0, 3, 0}, mgl64.Vec3{0, -6, 0}
				translation[k] = value
				if _, ok := shape.CastRay(origin, translation, 1); ok {
					t.Errorf("%T: a hit with the translation %v", shape, translation)
				}
			}
		}
	}
}

// A ray doesn't allocate
func TestCastRayDoesNotAllocate(t *testing.T) {
	shapes := []ShapeInterface{&Sphere{Radius: 0.5}, &Box{HalfExtents: mgl64.Vec3{0.5, 0.25, 1}}, &Capsule{HalfHeight: 0.6, Radius: 0.3}, &Plane{Normal: mgl64.Vec3{0, 1, 0}}}
	hits := 0
	allocs := testing.AllocsPerRun(100, func() {
		for _, shape := range shapes {
			if _, ok := shape.CastRay(mgl64.Vec3{0.1, 3, 0.1}, mgl64.Vec3{0, -6, 0}, 1); ok {
				hits++
			}
		}
	})
	if allocs > 0 || hits == 0 {
		t.Errorf("%.1f allocations for 4 rays (%d hits), want 0", allocs, hits)
	}
}
