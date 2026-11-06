#pragma once

// Internal to core (offset.cpp) and its tests: the start-angle turn that closes the last junction
// of a closed offset chain of arcs only (ADR-0012 A1), callable on a given junction so tests can
// check it on its own; inside offsetOpenContour the walk-from-another-arc retry can hide it. Also
// the junction repair between two spans and the self-crossing cuts of a strand (T-0172), whose
// rare paths (residuals that no centre or radius ulp closes exactly, strands that cross
// themselves) the fuzzed offsets reach only with other platforms' sin/cos or not at all.

#include <cstddef>
#include <vector>

#include "carvetto/geometry/span.hpp"

namespace carvetto::geometry::detail {

/// Closes the junction a -> b by turning b's start angle only (b's centre, radius and end point
/// stay bit-identical): b's start becomes a's end exactly where some angle gives it, else the
/// nearest start level with or ahead of a's end along a's travel within
/// 1e-12 x max(1, |x|, |y|, r). Returns false, leaving b unchanged, where no angle does.
bool turnArcOntoJunction(const Span& a, ArcSpan& b);

/// Makes the junction a -> b exact (offset.cpp exactJunction): a line end takes the other span's
/// end point; between two arcs, b's centre (and radius) is moved by the residual where that is at
/// most 1e-10 x max(1, |x|, |y|, r) and nudged by ulps until b starts exactly at a's end, and where
/// no such nudge hits it, b is left starting level with or ahead of a's end along a's travel
/// within 1e-12 x max(1, |x|, |y|, r). Larger residuals are left as they are.
void closeJunction(Span& a, Span& b);

/// Where a strand must be cut so that no part of it crosses or touches itself (two spans of a
/// part that are not neighbours meet): the index of the first span of every part after the first,
/// chosen greedily from the start; empty when the strand does not cross itself.
std::vector<std::size_t> selfCrossingCuts(const std::vector<Span>& spans);

}  // namespace carvetto::geometry::detail
