package main

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"slices"
	"strings"
	"time"

	"github.com/akmonengine/feather"
	"github.com/akmonengine/feather/bench/scenes"
)

// ========== REFERENCE SCENES ==========
// The scenes of Solver2D (bench/scenes) at their full size, on this version:
//
//	go run . -scenes                                          # the working tree
//	go run -tags v020 -modfile=go.v020.mod . -scenes          # v0.2.0
//	go run . -compare                                         # both side by side, in Markdown

// sceneRun: the result of a scene on a version, and its time per step
type sceneRun struct {
	Result scenes.Result `json:"result"`
	Passed bool          `json:"passed"`
	// Box3D: "at least as good as Box3D", "known gap" or "" (no reference)
	Box3D  string  `json:"box3d"`
	StepMs float64 `json:"stepMs"`
}

func sceneFile(version string) string {
	return filepath.Join(os.TempDir(), "feather-scenes-"+version+".json")
}

// runScenes at their full size, prints and saves them
func runScenes() bool {
	runs := map[string]sceneRun{}
	for _, scene := range scenes.All {
		if !scene.Supported() {
			fmt.Printf("%-18s not supported by %s\n", scene.Name, scenes.Version)
			continue
		}
		steps, elapsed := 0, time.Duration(0)
		player := func(w *feather.World, seconds float64, each func()) {
			scenes.Step(w, seconds, func() {
				steps++
				if each != nil {
					each()
				}
			})
		}
		start := time.Now()
		result := scene.Run(scenes.Full, player)
		elapsed = time.Since(start)
		run := sceneRun{Result: result, StepMs: milliseconds(elapsed) / float64(steps)}
		run.Passed = scene.Check == nil || scene.Check(result) == nil
		if scene.Reference != nil {
			run.Box3D = "at least as good as Box3D"
			if scene.Reference(result, scenes.Full) != nil {
				run.Box3D = "known gap " + scene.Gap
			}
		}
		runs[scene.Name] = run
		fmt.Printf("%-18s %-6v %s\n", scene.Name, run.Passed, formatResult(result))
	}
	data, err := json.MarshalIndent(runs, "", "  ")
	if err == nil {
		err = os.WriteFile(sceneFile(scenes.Version), data, 0o644)
	}
	if err != nil {
		fmt.Println("cannot save the scenes:", err)
		return false
	}
	return true
}

func formatResult(result scenes.Result) string {
	var parts []string
	for _, name := range sortedKeys(result) {
		parts = append(parts, fmt.Sprintf("%s %.3g %s", name, result[name].Value, result[name].Unit))
	}
	return strings.Join(parts, ", ")
}

// compareScenes prints the scenes of the working tree and of v0.2.0, side by side
func compareScenes() bool {
	load := func(version string) map[string]sceneRun {
		runs := map[string]sceneRun{}
		if data, err := os.ReadFile(sceneFile(version)); err == nil {
			_ = json.Unmarshal(data, &runs)
		}
		return runs
	}
	current, old := load("current"), load("v0.2.0")
	fmt.Println("| Scene | Measure | current | v0.2.0 |")
	fmt.Println("|---|---|---|---|")
	for _, scene := range scenes.All {
		now, found := current[scene.Name]
		if !found {
			continue
		}
		before, oldFound := old[scene.Name]
		for _, name := range sortedKeys(now.Result) {
			value := func(run sceneRun, found bool) string {
				if !found {
					return "—"
				}
				m, ok := run.Result[name]
				if !ok {
					return "—"
				}
				return fmt.Sprintf("%.3g %s", m.Value, m.Unit)
			}
			fmt.Printf("| %s | %s | %s | %s |\n", scene.Name, name, value(now, true), value(before, oldFound))
		}
		verdict := func(run sceneRun, found bool) string {
			switch {
			case !found:
				return "not supported"
			case run.Passed:
				return "passes"
			}
			return "**fails**"
		}
		fmt.Printf("| %s | criteria | %s | %s |\n", scene.Name, verdict(now, true), verdict(before, oldFound))
		if now.Box3D != "" {
			fmt.Printf("| %s | Box3D | %s | %s |\n", scene.Name, now.Box3D, map[bool]string{true: before.Box3D, false: "—"}[oldFound])
		}
		fmt.Printf("| %s | ms per step | %.3g | %s |\n", scene.Name, now.StepMs, map[bool]string{true: fmt.Sprintf("%.3g", before.StepMs), false: "—"}[oldFound])
	}
	return true
}

func milliseconds(d time.Duration) float64 {
	return float64(d.Nanoseconds()) / 1e6
}

func sortedKeys[V any](m map[string]V) []string {
	keys := make([]string, 0, len(m))
	for key := range m {
		keys = append(keys, key)
	}
	slices.Sort(keys)
	return keys
}
