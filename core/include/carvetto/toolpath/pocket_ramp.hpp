#pragma once

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::toolpath {

/// Appends a zig-zag ramp that takes the tool from `zFrom` down to `zTo` along the start of
/// `path` and back, so the tool enters the material on the path it is about to cut instead of
/// plunging vertically (feature-inventory §5 Pocketing "Ramp Plunge Moves").
///
/// The tool is assumed to sit at `path`'s first point at `zFrom`. The ramp runs forward along the
/// path for ramp length L = min(`maxLength`, path length) descending to the midway height, then
/// back along the same points to the first point, arriving exactly at `zTo`. Z is linear in the
/// distance travelled, so the slope is (zFrom − zTo) / 2L on both legs. Every ramp move is a
/// `Feed` lying on the path itself, so the ramp never leaves the area the path cuts. For a closed
/// path the closing segment counts as part of the path. When L is not longer than
/// `minLength` (or the path has fewer than two points) a single straight `Plunge` to `zTo` is
/// emitted instead. Returns true when a ramp was emitted, false for the plunge fallback.
///
/// Pocket-specific: the Profile toolpath (T-0120) has its own ramp family; a later task may unify
/// the two.
bool appendPathRamp(const geometry::Polyline& path, double zFrom, double zTo, double maxLength,
                    double minLength, Toolpath& out);

}  // namespace carvetto::toolpath
