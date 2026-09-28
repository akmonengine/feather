module github.com/akmonengine/feather/bench

go 1.24

require (
	github.com/akmonengine/feather v0.2.0
	github.com/go-gl/mathgl v1.2.0
)

// The bench runs against the working tree; go.v020.mod runs it against the published
// v0.2.0 (the XPBD solver) for comparison.
replace github.com/akmonengine/feather => ../
