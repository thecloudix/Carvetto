#pragma once

// Internal to core (offset.cpp) and its tests: the start-angle turn that closes the last junction
// of a closed offset chain of arcs only (ADR-0012 A1), callable on a given junction so tests can
// check it on its own; inside offsetOpenContour the walk-from-another-arc retry can hide it.

#include "carvetto/geometry/span.hpp"

namespace carvetto::geometry::detail {

/// Closes the junction a -> b by turning b's start angle only (b's centre, radius and end point
/// stay bit-identical): b's start becomes a's end exactly where some angle gives it, else the
/// nearest start level with or ahead of a's end along a's travel within
/// 1e-12 x max(1, |x|, |y|, r). Returns false, leaving b unchanged, where no angle does.
bool turnArcOntoJunction(const Span& a, ArcSpan& b);

}  // namespace carvetto::geometry::detail
