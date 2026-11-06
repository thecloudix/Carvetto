#pragma once

// Internal conversion between Carvetto contours and Clipper2 fixed-point paths. Not installed;
// only offset.cpp and boolean.cpp include it.

#include <clipper2/clipper.h>

#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::geometry::detail {

/// Millimetres -> Clipper2 integer grid (rounded to nearest).
Clipper2Lib::Point64 toClipper(Vec2 p);
/// Clipper2 integer grid -> millimetres.
Vec2 fromClipper(const Clipper2Lib::Point64& p);
/// Millimetre distance -> grid distance.
double toClipperDistance(double mm);

/// Polygonises the contour at `tol.chordal` and converts it. Consecutive duplicate vertices are
/// dropped. Returns an empty path when the contour is degenerate (fewer than 2 distinct points
/// open, fewer than 3 closed).
Clipper2Lib::Path64 toClipperPath(const Contour& c, const Tolerance& tol);

/// Converts a closed Clipper2 path to a contour of line spans (empty for < 3 points).
Contour closedContourFromClipper(const Clipper2Lib::Path64& path);
/// Converts an open Clipper2 path to a contour of line spans (empty for < 2 points).
Contour openContourFromClipper(const Clipper2Lib::Path64& path);
/// Converts every path of `paths` with `closed`, skipping degenerate ones.
std::vector<Contour> contoursFromClipper(const Clipper2Lib::Paths64& paths, bool closed);

/// One Clipper2 boolean of closed paths (defined in boolean.cpp), batched like `booleanOp` when
/// many far-apart groups sit side by side (T-0142, ADR-0006 §10); `reverse` sets Clipper2's
/// ReverseSolution (outers wound clockwise).
Clipper2Lib::Paths64 clipClosedPaths(Clipper2Lib::ClipType op, Clipper2Lib::FillRule fill, bool reverse,
                                     Clipper2Lib::Paths64 subjects, Clipper2Lib::Paths64 clips);

}  // namespace carvetto::geometry::detail
