package actor

import (
	"math"
	"testing"

	"github.com/go-gl/mathgl/mgl64"
)

// capsuleTransform builds a transform with a consistent inverse rotation.
func capsuleTransform(position mgl64.Vec3, rotation mgl64.Quat) Transform {
	return Transform{Position: position, Rotation: rotation}
}

// lyingAlongX rotates the local Y axis onto the world X axis.
var lyingAlongX = mgl64.QuatRotate(-math.Pi/2, mgl64.Vec3{0, 0, 1})

func TestCapsuleComputeMass(t *testing.T) {
	c := &Capsule{HalfHeight: 1.5, Radius: 0.4}
	density := 2.5

	cylinder := math.Pi * 0.4 * 0.4 * 3.0
	sphere := 4.0 / 3.0 * math.Pi * 0.4 * 0.4 * 0.4
	want := density * (cylinder + sphere)

	if got := c.ComputeMass(density); math.Abs(got-want) > 1e-12 {
		t.Errorf("ComputeMass = %.15f, want %.15f", got, want)
	}
}

// expectedCapsuleInertia derives the inertia with the parallel-axis theorem, independently
// of the implementation: a solid cylinder plus two hemispheres whose centroids sit
// 3r/8 away from their flat faces.
func expectedCapsuleInertia(mass, halfHeight, radius float64) (axial, transverse float64) {
	height := 2 * halfHeight
	cylinderVolume := math.Pi * radius * radius * height
	hemisphereVolume := 2.0 / 3.0 * math.Pi * radius * radius * radius
	totalVolume := cylinderVolume + 2*hemisphereVolume

	cylinderMass := mass * cylinderVolume / totalVolume
	hemisphereMass := mass * hemisphereVolume / totalVolume

	centroidOffset := 3.0 * radius / 8.0
	hemisphereAboutFlatFace := 2.0 / 5.0 * hemisphereMass * radius * radius
	hemisphereAboutCentroid := hemisphereAboutFlatFace - hemisphereMass*centroidOffset*centroidOffset
	hemisphereDistance := halfHeight + centroidOffset

	axial = cylinderMass*radius*radius/2 + 2*hemisphereAboutFlatFace
	transverse = cylinderMass*(radius*radius/4+height*height/12) +
		2*(hemisphereAboutCentroid+hemisphereMass*hemisphereDistance*hemisphereDistance)
	return axial, transverse
}

// integrateCapsuleInertia integrates the inertia slice by slice along the axis with a
// composite Simpson rule. Each slice is a disk of radius rho(y); the integrands are
// piecewise polynomials, so the quadrature converges far below 1e-9.
func integrateCapsuleInertia(mass, halfHeight, radius float64) (axial, transverse float64) {
	rhoSquared := func(y float64) float64 {
		overshoot := math.Abs(y) - halfHeight
		if overshoot <= 0 {
			return radius * radius
		}
		return math.Max(0, radius*radius-overshoot*overshoot)
	}
	simpson := func(f func(float64) float64, from, to float64) float64 {
		const intervals = 2000
		step := (to - from) / intervals
		sum := f(from) + f(to)
		for i := 1; i < intervals; i++ {
			weight := 2.0
			if i%2 == 1 {
				weight = 4.0
			}
			sum += weight * f(from+float64(i)*step)
		}
		return sum * step / 3
	}
	integrate := func(f func(float64) float64) float64 {
		return simpson(f, -halfHeight-radius, -halfHeight) +
			simpson(f, -halfHeight, halfHeight) +
			simpson(f, halfHeight, halfHeight+radius)
	}

	volume := integrate(func(y float64) float64 { return math.Pi * rhoSquared(y) })
	density := mass / volume
	axial = density * integrate(func(y float64) float64 {
		r2 := rhoSquared(y)
		return math.Pi * r2 * r2 / 2
	})
	transverse = density * integrate(func(y float64) float64 {
		r2 := rhoSquared(y)
		return math.Pi*r2*r2/4 + math.Pi*r2*y*y
	})
	return axial, transverse
}

func TestCapsuleComputeInertia(t *testing.T) {
	const tolerance = 1e-9

	cases := []struct {
		name               string
		mass, half, radius float64
	}{
		{"unit", 1, 1, 0.5},
		{"character", 80, 0.6, 0.3},
		{"thin bone", 0.25, 2, 0.05},
		{"stubby", 12, 0.1, 1.2},
		{"sphere limit", 3, 0, 0.7},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			c := &Capsule{HalfHeight: tc.half, Radius: tc.radius}
			got := c.ComputeInertia(tc.mass)

			axial, transverse := expectedCapsuleInertia(tc.mass, tc.half, tc.radius)
			want := mgl64.Mat3{transverse, 0, 0, 0, axial, 0, 0, 0, transverse}
			if !mat3Equal(got, want, tolerance) {
				t.Errorf("ComputeInertia = %v, closed form = %v", got, want)
			}

			axialNum, transverseNum := integrateCapsuleInertia(tc.mass, tc.half, tc.radius)
			if math.Abs(got.At(1, 1)-axialNum) > tolerance || math.Abs(got.At(0, 0)-transverseNum) > tolerance ||
				math.Abs(got.At(2, 2)-transverseNum) > tolerance {
				t.Errorf("ComputeInertia = %v, numerical integration axial=%.12f transverse=%.12f",
					got, axialNum, transverseNum)
			}
		})
	}
}

func TestCapsuleInertiaSphereLimit(t *testing.T) {
	c := &Capsule{HalfHeight: 0, Radius: 0.8}
	s := &Sphere{Radius: 0.8}
	if !mat3Equal(c.ComputeInertia(5), s.ComputeInertia(5), 1e-12) {
		t.Errorf("zero-height capsule inertia %v differs from sphere %v", c.ComputeInertia(5), s.ComputeInertia(5))
	}
}

func TestCapsuleComputeAABB(t *testing.T) {
	c := &Capsule{HalfHeight: 1, Radius: 0.5}

	got := c.ComputeAABB(capsuleTransform(mgl64.Vec3{1, 2, 3}, mgl64.QuatIdent()))
	want := AABB{Min: mgl64.Vec3{0.5, 0.5, 2.5}, Max: mgl64.Vec3{1.5, 3.5, 3.5}}
	if !vec3Equal(got.Min, want.Min, 1e-12) || !vec3Equal(got.Max, want.Max, 1e-12) {
		t.Errorf("upright AABB = %v, want %v", got, want)
	}

	got = c.ComputeAABB(capsuleTransform(mgl64.Vec3{0, 0, 0}, lyingAlongX))
	want = AABB{Min: mgl64.Vec3{-1.5, -0.5, -0.5}, Max: mgl64.Vec3{1.5, 0.5, 0.5}}
	if !vec3Equal(got.Min, want.Min, 1e-12) || !vec3Equal(got.Max, want.Max, 1e-12) {
		t.Errorf("lying AABB = %v, want %v", got, want)
	}

	// 45° around Z: the segment end sits at (±sqrt(2)/2, ±sqrt(2)/2, 0).
	got = c.ComputeAABB(capsuleTransform(mgl64.Vec3{0, 0, 0}, mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 0, 1})))
	e := math.Sqrt2/2 + 0.5
	want = AABB{Min: mgl64.Vec3{-e, -e, -0.5}, Max: mgl64.Vec3{e, e, 0.5}}
	if !vec3Equal(got.Min, want.Min, 1e-12) || !vec3Equal(got.Max, want.Max, 1e-12) {
		t.Errorf("tilted AABB = %v, want %v", got, want)
	}
}

func TestCapsuleSupport(t *testing.T) {
	c := &Capsule{HalfHeight: 1, Radius: 0.5}

	cases := []struct {
		direction, want mgl64.Vec3
	}{
		{mgl64.Vec3{0, 1, 0}, mgl64.Vec3{0, 1.5, 0}},
		{mgl64.Vec3{0, -3, 0}, mgl64.Vec3{0, -1.5, 0}},
		{mgl64.Vec3{1, 1, 0}, mgl64.Vec3{0.5 * math.Sqrt2 / 2, 1 + 0.5*math.Sqrt2/2, 0}},
		{mgl64.Vec3{0, -1, -1}, mgl64.Vec3{0, -1 - 0.5*math.Sqrt2/2, -0.5 * math.Sqrt2 / 2}},
	}
	for _, tc := range cases {
		if got := c.Support(tc.direction); !vec3Equal(got, tc.want, 1e-12) {
			t.Errorf("Support(%v) = %v, want %v", tc.direction, got, tc.want)
		}
	}

	// Perpendicular direction: any point of the side line is a valid support, but it
	// must lie on that line.
	got := c.Support(mgl64.Vec3{2, 0, 0})
	if math.Abs(got.X()-0.5) > 1e-12 || math.Abs(got.Z()) > 1e-12 || math.Abs(got.Y()) > 1+1e-12 {
		t.Errorf("Support(+X) = %v, want a point of the line x=0.5, |y|<=1", got)
	}

	// A zero direction must not produce NaN.
	for _, v := range c.Support(mgl64.Vec3{}) {
		if math.IsNaN(v) {
			t.Fatalf("Support(0) = %v, must not be NaN", c.Support(mgl64.Vec3{}))
		}
	}
}

// TestCapsuleSupportIsExtreme checks the support property against sampled surface points.
func TestCapsuleSupportIsExtreme(t *testing.T) {
	c := &Capsule{HalfHeight: 0.7, Radius: 0.3}
	directions := []mgl64.Vec3{{1, 0.2, 0}, {-0.3, 1, 0.5}, {0.1, -1, -0.2}, {0, 0.05, -1}}

	for _, d := range directions {
		support := c.Support(d).Dot(d)
		for i := 0; i <= 40; i++ {
			y := -0.7 + 1.4*float64(i)/40
			for j := 0; j < 36; j++ {
				angle := float64(j) * math.Pi / 18
				for _, sphereY := range []float64{-1, 0, 1} {
					// Points of the cylinder side and of the caps' equators.
					p := mgl64.Vec3{0.3 * math.Cos(angle), y, 0.3 * math.Sin(angle)}
					if sphereY != 0 {
						p = mgl64.Vec3{0, sphereY * 0.7, 0}.Add(mgl64.Vec3{math.Cos(angle), sphereY, math.Sin(angle)}.Normalize().Mul(0.3))
					}
					if p.Dot(d) > support+1e-12 {
						t.Fatalf("Support(%v)·d = %f but surface point %v gives %f", d, support, p, p.Dot(d))
					}
				}
			}
		}
	}
}

func TestCapsuleGetContactFeature(t *testing.T) {
	c := &Capsule{HalfHeight: 1, Radius: 0.5}
	var output [8]mgl64.Vec3
	var count int

	// Side contact: the feature is the side line of the cylinder.
	c.GetContactFeature(mgl64.Vec3{0, 0, -2}, &output, &count)
	if count != 2 {
		t.Fatalf("side feature count = %d, want 2", count)
	}
	top, bottom := output[0], output[1]
	if top.Y() < bottom.Y() {
		top, bottom = bottom, top
	}
	if !vec3Equal(top, mgl64.Vec3{0, 1, -0.5}, 1e-12) || !vec3Equal(bottom, mgl64.Vec3{0, -1, -0.5}, 1e-12) {
		t.Errorf("side feature = %v %v, want (0,±1,-0.5)", output[0], output[1])
	}

	// Cap contact: a single point.
	c.GetContactFeature(mgl64.Vec3{0, -1, 0}, &output, &count)
	if count != 1 || !vec3Equal(output[0], mgl64.Vec3{0, -1.5, 0}, 1e-12) {
		t.Errorf("cap feature = %v (count %d), want (0,-1.5,0)", output[0], count)
	}

	// Oblique contact: a single support point.
	d := mgl64.Vec3{1, 1, 0}
	c.GetContactFeature(d, &output, &count)
	if count != 1 || !vec3Equal(output[0], c.Support(d), 1e-12) {
		t.Errorf("oblique feature = %v (count %d), want %v", output[0], count, c.Support(d))
	}
}

func TestCapsuleSegment(t *testing.T) {
	c := &Capsule{HalfHeight: 2, Radius: 0.1}
	a, b := c.Segment(capsuleTransform(mgl64.Vec3{1, 1, 1}, lyingAlongX))
	if !vec3Equal(a, mgl64.Vec3{-1, 1, 1}, 1e-12) || !vec3Equal(b, mgl64.Vec3{3, 1, 1}, 1e-12) {
		t.Errorf("Segment = %v %v, want (-1,1,1) (3,1,1)", a, b)
	}
}

func TestCapsuleCollideWithPlane(t *testing.T) {
	c := &Capsule{HalfHeight: 1, Radius: 0.5}
	up := mgl64.Vec3{0, 1, 0}

	// Contacts lie halfway between the capsule surface and the plane; the separation is
	// negative when they overlap.
	t.Run("upright on its cap", func(t *testing.T) {
		contacts := c.CollideWithPlane(up, 0, capsuleTransform(mgl64.Vec3{0, 1.4, 0}, mgl64.QuatIdent()), 0, nil)
		ok := len(contacts) > 0
		if !ok || len(contacts) != 1 {
			t.Fatalf("collision = %v, contacts = %v, want 1 contact", ok, contacts)
		}
		if !vec3Equal(contacts[0].Position, mgl64.Vec3{0, -0.05, 0}, 1e-12) || !floatEqual(contacts[0].Separation, -0.1, 1e-12) {
			t.Errorf("contact = %+v, want (0,-0.05,0) separation -0.1", contacts[0])
		}
	})

	t.Run("lying on its side", func(t *testing.T) {
		contacts := c.CollideWithPlane(up, 0, capsuleTransform(mgl64.Vec3{2, 0.45, 0}, lyingAlongX), 0, nil)
		ok := len(contacts) > 0
		if !ok || len(contacts) != 2 {
			t.Fatalf("collision = %v, contacts = %v, want 2 contacts", ok, contacts)
		}
		xs := []float64{contacts[0].Position.X(), contacts[1].Position.X()}
		if xs[0] > xs[1] {
			xs[0], xs[1] = xs[1], xs[0]
		}
		if !floatEqual(xs[0], 1, 1e-12) || !floatEqual(xs[1], 3, 1e-12) {
			t.Errorf("contact x = %v, want [1 3]", xs)
		}
		for _, p := range contacts {
			if !floatEqual(p.Position.Y(), -0.025, 1e-12) || !floatEqual(p.Separation, -0.05, 1e-12) {
				t.Errorf("contact = %+v, want y=-0.025 separation -0.05", p)
			}
		}
	})

	t.Run("tilted: only the lower cap touches", func(t *testing.T) {
		rotation := mgl64.QuatRotate(math.Pi/4, mgl64.Vec3{0, 0, 1})
		// Lower segment end at (sqrt2/2, -sqrt2/2 + y) ; put it 0.4 above the plane.
		y := math.Sqrt2/2 + 0.4
		contacts := c.CollideWithPlane(up, 0, capsuleTransform(mgl64.Vec3{0, y, 0}, rotation), 0, nil)
		ok := len(contacts) > 0
		if !ok || len(contacts) != 1 {
			t.Fatalf("collision = %v, contacts = %v, want 1 contact", ok, contacts)
		}
		if !vec3Equal(contacts[0].Position, mgl64.Vec3{math.Sqrt2 / 2, -0.05, 0}, 1e-12) || !floatEqual(contacts[0].Separation, -0.1, 1e-12) {
			t.Errorf("contact = %+v, want (0.707,-0.05,0) separation -0.1", contacts[0])
		}
	})

	t.Run("offset oblique plane", func(t *testing.T) {
		// Plane (x + y)/sqrt2 = -1, i.e. Normal·p + Distance = 0 with Distance = 1.
		n := mgl64.Vec3{1, 1, 0}.Normalize()
		contacts := c.CollideWithPlane(n, 1, capsuleTransform(mgl64.Vec3{0, 0, 0}, mgl64.QuatIdent()), 0, nil)
		ok := len(contacts) > 0
		if !ok || len(contacts) != 1 {
			t.Fatalf("collision = %v, contacts = %v, want 1 contact", ok, contacts)
		}
		// Only the lower end (0,-1,0) is closer than the radius: signed distance 1 - sqrt2/2.
		distance := 1 - math.Sqrt2/2
		separation := distance - 0.5
		want := mgl64.Vec3{0, -1, 0}.Sub(n.Mul(0.5 + separation/2))
		if !vec3Equal(contacts[0].Position, want, 1e-12) || !floatEqual(contacts[0].Separation, separation, 1e-12) {
			t.Errorf("contact = %+v, want %v separation %f", contacts[0], want, separation)
		}
	})

	t.Run("above the plane", func(t *testing.T) {
		if contacts := c.CollideWithPlane(up, 0, capsuleTransform(mgl64.Vec3{0, 1.6, 0}, mgl64.QuatIdent()), 0, nil); len(contacts) != 0 {
			t.Errorf("capsule above the plane reported a contact %v", contacts)
		}
	})

	t.Run("speculative: within the margin", func(t *testing.T) {
		contacts := c.CollideWithPlane(up, 0, capsuleTransform(mgl64.Vec3{0, 1.51, 0}, mgl64.QuatIdent()), 0.02, nil)
		ok := len(contacts) > 0
		if !ok || len(contacts) != 1 || !floatEqual(contacts[0].Separation, 0.01, 1e-12) {
			t.Fatalf("collision = %v, contacts = %v, want 1 speculative contact at separation 0.01", ok, contacts)
		}
		if contacts := c.CollideWithPlane(up, 0, capsuleTransform(mgl64.Vec3{0, 1.53, 0}, mgl64.QuatIdent()), 0.02, nil); len(contacts) > 0 {
			t.Error("capsule beyond the margin reported a contact")
		}
	})
}
