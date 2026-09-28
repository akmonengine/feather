//go:build !v020

package scenes

import "fmt"

// ========== REFERENCES ==========
// The scenes run on Box3D (Erin Catto, https://github.com/erincatto/box3d, commit 5643cd8 of 25/09/2026) with the same
// bodies and the same measures: bench/box3d/scenes.c, in its large world mode (double precision positions), at the rate
// of the bench (60 Hz, 4 sub-steps). Feather must do at least as well.
//
// Only the measures where lower is better are compared. Not compared:
//   - the fastest body of "overlap recovery" & "far recovery": the push of the overlapping layers adds up along the pile,
//     a solver converging less throws the top slower
//   - the final speed of "rush": the packed ball never rests (both engines jitter between 0.7 & 3.6 m/s from 5 to 10 s)
//   - the deviation of the far scenes: Box3D solves in float32 relative to the bodies, the rounding of the positions far
//     from the origin vanishes in it; Feather keeps it (1e-11 m) and TestPlaceIndependence bounds its effect
//   - the speeds of "joint grid" & "stretched chain": the swing of the net, the pull of the joints, physics not errors

const (
	// box3dTolerance & box3dMargin: at least as good, to the measure: 2 % and 0.01 (in the unit of the measure)
	box3dTolerance = 1.02
	box3dMargin    = 0.01
)

// box3dValue: a measure of a scene, and its value in Box3D at both sizes
type box3dValue struct {
	measure     string
	small, full float64
}

var box3dValues = map[string][]box3dValue{
	"single box":        {{"height error", 0.0689445, 0.0689445}, {"rest drift", 0.0262277, 0.0262277}},
	"warm start energy": {{"overshoot", 12.4302, 12.4302}},
	"high mass ratio 1": {{"worst drift", 729.466, 499.656}, {"heavy cube sag", 126.722, 132.724}},
	"high mass ratio 2": {{"slab sag", 96.7198, 96.7198}, {"small cubes drift", 140.703, 140.703}, {"bounce", 4.33959, 4.33959},
		{"small cubes into ground", 594.57, 594.57}},
	"high mass ratio 3": {{"slab sag", 96.9108, 96.9108}, {"small cubes drift", 138.45, 138.45}, {"bounce", 4.43974, 4.43974},
		{"small cubes into ground", 593.361, 593.361}},
	"friction ramp":    {{"stopped slide", 0, 0}, {"sliding error", 64.9105, 64.9105}},
	"overlap recovery": {{"final overlap", 4.9123, 3.7884}},
	"vertical stack":   {{"horizontal drift", 6.45517, 14.1358}},
	"pyramid":          {{"worst drift", 1.62748, 26.9122}},
	"rush":             {{"final overlap", 4.01148, 32.734}},
	"confined":         {{"max speed", 5.74906, 9.92108}},
	"card house":       {{"worst drift", 0.0594942, 815.902}},
	"circle stack":     {{"horizontal drift", 0, 0}, {"top height error", 6.86616, 26.7069}},
	"centered impact":  {{"lateral drift", 308.96, 308.96}, {"bounce", 2.29705, 2.29705}},
	"bridge":           {{"worst gap", 26.0555, 69.2763}},
	"ball and chain":   {{"worst gap", 235.321, 232.72}},
	"joint grid":       {{"worst gap", 64.768, 319.21}},
	"stretched chain":  {{"final gap", 12.2763, 12.2763}, {"rest gap", 5.46556, 5.46556}},
	"far pyramid":      {{"worst drift", 0, 0}},
	"far stack":        {{"worst drift", 0.105557, 0.105557}},
	"far chain":        {{"worst gap", 49.4286, 49.4286}},
}

// box3dGaps: the known gaps to Box3D, and the ticket following them. Feather runs 8 substeps (its setting for the games),
// Box3D its default 4: the gaps of the sinking under a load (high mass ratio 2 & 3), of the far stack, of the confined
// spheres, of the pyramid and of high mass ratio 1 closed at 8 substeps
var box3dGaps = map[string]string{
	// at the small size: the cards of 2 mm move by 0.14 mm, 0.06 in Box3D (at the full size, the house of Box3D falls)
	"card house": "#821",
	// at the small size: the net of 20 x 20 opens by 85 mm, 65 in Box3D (at the full size, 60 x 60, it is as good)
	"joint grid": "#821",
	// the stack of 5 spheres sinks by 7.10 mm on its springs, 6.87 in Box3D (within 3 %)
	"circle stack": "#821",
}

func init() {
	for i := range All {
		values, found := box3dValues[All[i].Name]
		if !found {
			continue
		}
		All[i].Reference = func(r Result, size Size) error {
			for _, value := range values {
				reference := map[Size]float64{Small: value.small, Full: value.full}[size]
				if err := atMost(r, value.measure, reference*box3dTolerance+box3dMargin, fmt.Sprintf("Box3D %.3g", reference)); err != nil {
					return err
				}
			}
			return nil
		}
		All[i].Gap = box3dGaps[All[i].Name]
	}
}
