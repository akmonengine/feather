package feather

import (
	"testing"
	"time"
)

// Every phase of a step with contacts takes some time, and the phases fit in the step
func TestProfile(t *testing.T) {
	w := terrainScene(4)
	defer w.Close()
	simulate(w, 1, nil)
	profile := w.Profile()
	phases := []time.Duration{profile.BroadPhase, profile.NarrowPhase, profile.Prepare, profile.Substeps, profile.Restitution, profile.Continuous, profile.Islands}
	var sum time.Duration
	for i, phase := range phases {
		if phase <= 0 {
			t.Errorf("phase %d: %v", i, phase)
		}
		sum += phase
	}
	t.Logf("%+v", profile)
	if sum > profile.Step {
		t.Errorf("the phases take %v, the step %v", sum, profile.Step)
	}
}
