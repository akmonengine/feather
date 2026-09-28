package actor

import "github.com/go-gl/mathgl/mgl64"

// AABB represents an axis-aligned bounding box
type AABB struct {
	Min mgl64.Vec3
	Max mgl64.Vec3
}

// ContainsPoint checks if a point is inside the AABB
func (a AABB) ContainsPoint(point mgl64.Vec3) bool {
	return point.X() >= a.Min.X() && point.X() <= a.Max.X() &&
		point.Y() >= a.Min.Y() && point.Y() <= a.Max.Y() &&
		point.Z() >= a.Min.Z() && point.Z() <= a.Max.Z()
}

// Overlaps checks if two AABBs overlap
func (a AABB) Overlaps(other AABB) bool {
	// AABBs overlap if they overlap on all three axes
	return a.Max[0] >= other.Min[0] && a.Min[0] <= other.Max[0] &&
		a.Max[1] >= other.Min[1] && a.Min[1] <= other.Max[1] &&
		a.Max[2] >= other.Min[2] && a.Min[2] <= other.Max[2]
}
