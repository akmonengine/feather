//go:build !v020

package main

import (
	"runtime"
	"testing"
	"unsafe"
)

// The frame which runs the scene lies at the wanted offset of its page: the callee is at the same distance from every
// offset, so at the offset plus a constant
func TestAtStackOffset(t *testing.T) {
	// the stack of arm64 is aligned on 16 bytes
	step := wordSize
	if runtime.GOARCH != "amd64" {
		step = 2 * wordSize
	}
	var distances []uintptr
	calls := 0
	callee := func(offset int) func() {
		return func() {
			var mark uint64
			calls++
			at := uintptr(unsafe.Pointer(&mark)) % pageSize
			distances = append(distances, (uintptr(offset)+pageSize-at)%pageSize)
		}
	}
	for offset := 0; offset < pageSize; offset += step {
		atStackOffset(offset, callee(offset))
	}
	if want := pageSize / step; calls != want {
		t.Fatalf("%d calls, want %d", calls, want)
	}
	for i, distance := range distances {
		if distance != distances[0] {
			t.Fatalf("offset %d: the scene starts %d bytes under the offset, %d bytes under the offset 0", i*step, distance, distances[0])
		}
	}
	// an offset out of the page, or between two words, is brought back to a word of the page
	atStackOffset(pageSize+3*wordSize+5, callee(3*wordSize))
	if last := distances[len(distances)-1]; last != distances[0] {
		t.Errorf("offset %d: the scene starts %d bytes under the offset of its word, want %d", pageSize+3*wordSize+5, last, distances[0])
	}
}
