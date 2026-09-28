//go:build !v020

package scenes

import "testing"

// Every scene, at its small size, meets its criteria. The scenes run in parallel
func TestScenes(t *testing.T) {
	for _, scene := range All {
		t.Run(scene.Name, func(t *testing.T) {
			t.Parallel()
			result := scene.Run(Small, Step)
			t.Logf("%v", result)
			if err := scene.Check(result); err != nil {
				t.Error(err)
			}
			if scene.Reference == nil {
				return
			}
			switch err := scene.Reference(result, Small); {
			case err != nil && scene.Gap != "":
				t.Logf("known gap (%s): %v", scene.Gap, err)
			case err != nil:
				t.Error(err)
			case scene.Gap != "":
				t.Errorf("the gap %s is closed: remove it", scene.Gap)
			}
		})
	}
}
