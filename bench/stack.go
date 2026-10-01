//go:build !v020

package main

import "unsafe"

// ========== STACK DEPTH ==========
// The time of a phase depends on where the stack lies in its pages of 4 KiB. The broad phase copies AABBs of 48 bytes
// on the stack (Tree.scan, for AABB.Overlaps): at a few depths, a 16 bytes write of such a copy straddles two pages,
// and is read back by halves right after. The store cannot be forwarded to the loads: the broad phase of
// "solver2d confined" takes 0.084 ms instead of 0.046, on every run of the same binary. The depth of the stack comes
// from the size of every frame above the phase: 8 more bytes in a frame of the engine, or of this bench, were enough to
// report a phase twice slower (or twice faster) with the same code.
//
// So the speed is measured at speedRuns depths, a third of a page apart, and each phase keeps its best run: the slow
// depths are a few words wide, a phase cannot hit them at every depth. The depths are offsets in the page, not
// distances from main: the frames of the bench don't move them.

const (
	// pageSize of the memory: 4 KiB (the page of x86-64, and the smallest one of arm64)
	pageSize = 4096

	// wordSize: the stack moves by words of 8 bytes
	wordSize = 8
)

// atStackOffset calls run from a frame lying at this offset in its page: the frames under it, the ones of the scene
// and of the engine, lie at the same offsets whatever is above. Where no frame can lie at this offset (a stack aligned
// on 16 bytes), run is called from the last frame tried
func atStackOffset(offset int, run func()) {
	descend(uintptr(offset%pageSize)&^(wordSize-1), 0, run)
}

// descend calls itself until its frame lies at the offset: each call goes one frame down. After a page, a wider frame
// (descendWide) reaches the offsets that the frames of descend step over; after two pages, the offset cannot be reached
//
//go:noinline
func descend(offset uintptr, words int, run func()) {
	var mark uint64
	switch at := uintptr(unsafe.Pointer(&mark)) % pageSize; {
	case at == offset || words >= 2*pageSize/wordSize:
		run()
	case words == pageSize/wordSize:
		descendWide(offset, words+1, run)
	default:
		descend(offset, words+1, run)
	}
}

// descendWide: a frame of descend, one word wider
//
//go:noinline
func descendWide(offset uintptr, words int, run func()) {
	var marks [2]uint64
	if uintptr(unsafe.Pointer(&marks[1]))%pageSize == offset {
		run()
		return
	}
	descend(offset, words, run)
}
