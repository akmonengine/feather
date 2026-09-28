//go:build race

package feather

// the race detector drops the items of sync.Pool on purpose: the allocations cannot be measured
const raceEnabled = true
