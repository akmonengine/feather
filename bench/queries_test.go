//go:build !v020

package main

import "testing"

// The scenes of the queries give the same results at each run, find the ground under every ray and wake nobody up
func TestQueryScenes(t *testing.T) {
	for _, scene := range queryScenes {
		first, second := measureScene(scene, 0), measureScene(scene, 0)
		if first.Fingerprint != second.Fingerprint {
			t.Errorf("%s: fingerprint %s, then %s", scene.name, first.Fingerprint, second.Fingerprint)
		}
		for name, m := range first.Quality {
			if m.Unit == "" && m.Value != 0 {
				t.Errorf("%s: %s is %g, want 0", scene.name, name, m.Value)
			}
		}
		if len(first.PhasesMs) < 3 || first.StepMs <= 0 {
			t.Errorf("%s: %d batches timed, %.3f ms per round", scene.name, len(first.PhasesMs), first.StepMs)
		}
	}
}
