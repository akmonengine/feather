//go:build !v020

package scenes

import "testing"

// The scenes which have a draw: every draw is a measure of the scene, and the variants at both ends of their range and
// in its middle are the same difficulty (they meet the criteria of the scene)
func TestVariants(t *testing.T) {
	varied := 0
	for _, scene := range All {
		if (scene.Vary == nil) != (len(scene.Draws) == 0) {
			t.Errorf("%s: %d draws, variants %v", scene.Name, len(scene.Draws), scene.Vary != nil)
		}
		if scene.Vary == nil {
			continue
		}
		varied++
		t.Run(scene.Name, func(t *testing.T) {
			t.Parallel()
			for _, spread := range []float64{0, 0.5, 31.0 / 32} {
				result := scene.Vary(Small, spread, Step)
				for _, name := range scene.Draws {
					if _, found := result[name]; !found {
						t.Errorf("variant %g: no measure %q", spread, name)
					}
				}
				if err := scene.Check(result); err != nil {
					t.Errorf("variant %g: %v", spread, err)
				}
			}
		})
	}
	if varied == 0 {
		t.Error("no scene has a draw")
	}
}
