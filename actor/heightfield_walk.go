package actor

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
)

// ========== WALK ==========
// The cells of a heightfield under a moving box (a ray is a box without size), in the order of the motion: the
// traversal of a grid by a segment (Amanatides & Woo, "A Fast Voxel Traversal Algorithm for Ray Tracing", 1987; the
// rays and the shape casts of the height fields of Box3D and PhysX walk their grid the same way), widened to a box as
// b3ShapeCastHeightField of Box3D. The walk follows the axis the box moves the most along, column by column: in a
// column, the box covers a range of rows, known from where its center is when it enters and leaves the column.
// The grid has 2 levels: the columns are taken by blocks of HeightfieldBlockSize, and a block the box flies over or
// under, by its lowest and highest heights, is skipped with all its cells.

const (
	// walkSlack: the box of a walk is widened by this many cells: a ray on a line of the grid visits the cells on both
	// sides of the line
	walkSlack = 2e-9

	// footprintSlack: a ray hits a triangle up to this many cells out of it (half of walkSlack: the cell of such a hit
	// is always visited)
	footprintSlack = 1e-9
)

// CellWalk: the cells under a moving box, from Heightfield.WalkCells
type CellWalk struct {
	field *Heightfield
	// the motion of the center in the grid (a cell is a unit square), along the major axis (the columns) and the
	// minor axis (the rows), and the half sizes of the box
	from, along, extent [2]float64
	// the heights of the center, and the half height of the box
	startY, alongY, extentY float64
	// the fractions of the motion over the terrain
	first, last float64
	// alongZ: the columns are along z (the box moves the most along z), else along x
	alongZ bool
	// the columns: the one being walked, the next one, the last one, and the step from a column to the next
	column, next, lastColumn, step int
	// blockEnd: the last column of the block of columns being walked
	blockEnd int
	// done: no column is left after the one being walked
	done bool
	// the rows left in the column being walked, and the heights of the box over this column
	row, lastRow int
	low, high    float64
}

// WalkCells: the cells a box of half sizes extents can touch while its center moves from origin by maxFraction *
// translation, in the local space of the terrain. The holes are skipped
func (h *Heightfield) WalkCells(origin, translation, extents mgl64.Vec3, maxFraction float64) CellWalk {
	var w CellWalk
	w.start(h, origin, translation, extents, maxFraction)
	return w
}

// start the walk (in place: CastRay keeps its walk on its stack, without a copy)
func (w *CellWalk) start(h *Heightfield, origin, translation, extents mgl64.Vec3, maxFraction float64) {
	*w = CellWalk{field: h, startY: origin.Y(), alongY: translation.Y(), extentY: extents.Y(), last: maxFraction, row: 1}
	cells := [2]int{h.XSamples - 1, h.ZSamples - 1}
	w.from = [2]float64{origin.X()/h.Scale.X() + float64(cells[0])/2, origin.Z()/h.Scale.Z() + float64(cells[1])/2}
	w.along = [2]float64{translation.X() / h.Scale.X(), translation.Z() / h.Scale.Z()}
	w.extent = [2]float64{extents.X()/h.Scale.X() + walkSlack, extents.Z()/h.Scale.Z() + walkSlack}
	if math.Abs(w.along[1]) > math.Abs(w.along[0]) {
		w.alongZ = true
		cells[0], cells[1] = cells[1], cells[0]
		w.from[0], w.from[1] = w.from[1], w.from[0]
		w.along[0], w.along[1] = w.along[1], w.along[0]
		w.extent[0], w.extent[1] = w.extent[1], w.extent[0]
	}

	// the part of the motion where the box is over the terrain, between its lowest and highest heights
	over := w.clip(w.from[0], w.along[0], -w.extent[0], float64(cells[0])+w.extent[0]) &&
		w.clip(w.from[1], w.along[1], -w.extent[1], float64(cells[1])+w.extent[1]) &&
		w.clip(w.startY, w.alongY, h.minHeight-w.extentY, h.maxHeight+w.extentY)
	if !over {
		w.done = true
		return
	}

	// the columns the box covers, from the first one it meets
	from, to := w.from[0]+w.first*w.along[0], w.from[0]+w.last*w.along[0]
	low, high := cellOf(math.Min(from, to)-w.extent[0], cells[0]), cellOf(math.Max(from, to)+w.extent[0], cells[0])
	w.next, w.lastColumn, w.step = low, high, 1
	if w.along[0] < 0 {
		w.next, w.lastColumn, w.step = high, low, -1
	}
	// the first column starts a block
	w.blockEnd = w.next - w.step
}

// clip the fractions of the motion to those where the coordinate start + fraction * along is in [low, high].
// Returns false if there is none
func (w *CellWalk) clip(start, along, low, high float64) bool {
	if along == 0 {
		return start >= low && start <= high
	}
	enter, exit := (low-start)/along, (high-start)/along
	if enter > exit {
		enter, exit = exit, enter
	}
	w.first, w.last = math.Max(w.first, enter), math.Min(w.last, exit)
	return w.first <= w.last
}

// cellOf the coordinate, in the grid of count cells
func cellOf(coordinate float64, count int) int {
	return int(math.Min(math.Max(math.Floor(coordinate), 0), float64(count-1)))
}

// Next cell (x, z) of the walk, false after the last one. The walk ends when the box enters the next column after the
// fraction limit of its motion: the caller lowers the limit to its best hit
func (w *CellWalk) Next(limit float64) (int, int, bool) {
	h := w.field
	for {
		for w.row <= w.lastRow {
			row := w.row
			w.row++
			x, z := w.column, row
			if w.alongZ {
				x, z = z, x
			}
			if block := h.blocks[x/HeightfieldBlockSize*h.blocksZ+z/HeightfieldBlockSize]; w.low > block.max || w.high < block.min {
				// the rows left in this block are over or under the box too
				w.row = (row/HeightfieldBlockSize + 1) * HeightfieldBlockSize
				continue
			}
			if h.hasCell(x, z) {
				return x, z, true
			}
		}
		if !w.nextColumn(limit) {
			return 0, 0, false
		}
	}
}

// nextColumn: the walk moves to its next column, false after the last one
func (w *CellWalk) nextColumn(limit float64) bool {
	for !w.done {
		column := w.next
		if (column-w.blockEnd)*w.step > 0 {
			// a new block of columns, up to end: skipped if the box flies over or under all its blocks
			end := column / HeightfieldBlockSize * HeightfieldBlockSize
			if w.step > 0 {
				end = min(end+HeightfieldBlockSize-1, w.lastColumn)
			} else {
				end = max(end, w.lastColumn)
			}
			// (a block of one column is tested as a column, below)
			if end != column {
				enter, touched := w.cover(column, end)
				if enter > limit {
					break
				}
				if !touched {
					if end == w.lastColumn {
						break
					}
					w.next = end + w.step
					continue
				}
			}
			w.blockEnd = end
		}
		if enter, _ := w.cover(column, column); enter > limit {
			break
		}
		w.column, w.next, w.done = column, column+w.step, column == w.lastColumn
		return true
	}
	w.done = true
	return false
}

// cover: the box over the columns [low, high]: the fraction where it enters them, its rows and its heights there (in
// row, lastRow, low & high), and whether a block under these rows reaches its heights
func (w *CellWalk) cover(low, high int) (float64, bool) {
	h := w.field
	if low > high {
		low, high = high, low
	}
	enter, exit := w.first, w.last
	if w.along[0] != 0 {
		// the center of the box is over the columns, or closer to them than its half size
		a, b := (float64(low)-w.extent[0]-w.from[0])/w.along[0], (float64(high+1)+w.extent[0]-w.from[0])/w.along[0]
		if a > b {
			a, b = b, a
		}
		enter, exit = math.Max(enter, a), math.Min(exit, b)
	}
	rows := h.ZSamples - 1
	if w.alongZ {
		rows = h.XSamples - 1
	}
	from, to := w.from[1]+enter*w.along[1], w.from[1]+exit*w.along[1]
	w.row, w.lastRow = cellOf(math.Min(from, to)-w.extent[1], rows), cellOf(math.Max(from, to)+w.extent[1], rows)
	fromY, toY := w.startY+enter*w.alongY, w.startY+exit*w.alongY
	w.low, w.high = math.Min(fromY, toY)-w.extentY, math.Max(fromY, toY)+w.extentY

	for column := low / HeightfieldBlockSize; column <= high/HeightfieldBlockSize; column++ {
		for row := w.row / HeightfieldBlockSize; row <= w.lastRow/HeightfieldBlockSize; row++ {
			block := h.blocks[column*h.blocksZ+row]
			if w.alongZ {
				block = h.blocks[row*h.blocksZ+column]
			}
			if w.low <= block.max && w.high >= block.min {
				return enter, true
			}
		}
	}
	return enter, false
}
