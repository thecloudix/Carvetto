#pragma once

// Internal to core (offset.cpp) and its tests: the start-angle turn that closes the last junction
// of a closed offset chain of arcs only (ADR-0012 A1), callable on a given junction so tests can
// check it on its own; inside offsetOpenContour the walk-from-another-arc retry can hide it. Also
// the junction repair between two spans and the self-crossing cuts of a strand (T-0172), whose
// rare paths (residuals that no centre or radius ulp closes exactly, strands that cross
// themselves, near-tangent pairs) the fuzzed offsets reach only with other platforms' sin/cos or
// not at all.

#include <cstddef>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::geometry::detail {

/// Closes the junction a -> b by turning b's start angle only (b's centre, radius and end point
/// stay bit-identical): b's start becomes a's end exactly where some angle gives it, else the
/// nearest start level with or ahead of a's end along a's travel within
/// 1e-12 x max(1, |x|, |y|, r). Returns false, leaving b unchanged, where no angle does.
bool turnArcOntoJunction(const Span& a, ArcSpan& b);

/// Makes the junction a -> b exact (offset.cpp exactJunction): where a is a line its end takes b's
/// start, else where b is a line its start takes a's end. Between two arcs, b's centre (and radius)
/// is moved by the residual where that is at most 1e-10 x max(1, |x|, |y|, r) and nudged by ulps
/// until b starts exactly at a's end. Where no nudge hits it, b is kept if it starts level with or
/// ahead of a's end along a's travel within 1e-12 x max(1, |x|, |y|, r), else it becomes the nearest
/// of the nudged candidates that does; where none does either, b is left unchanged (it may then
/// start behind a's end, or farther from it). Larger residuals are left as they are.
void closeJunction(Span& a, Span& b);

/// The pairwise test of the self-crossing cuts (offset.cpp spansMeet): the carriers of two
/// line-or-arc spans meet at a point that lies on both spans to within 1e-9 of their lengths
/// beyond their ends (carrierIntersections' tangency tolerances included).
bool spansMeet(const Span& a, const Span& b);

/// How far (mm) beyond the box of `s` a point can lie at which spansMeet finds `s` meeting another
/// span: two spans whose boxes, each grown by its meetReach, are apart never meet.
double meetReach(const Span& s);

/// Where a strand must be cut so that no part of it crosses or touches itself (two spans of a
/// part that are not neighbours meet, spansMeet): the index of the first span of every part after
/// the first, chosen greedily from the start; empty when the strand does not cross itself.
std::vector<std::size_t> selfCrossingCuts(const std::vector<Span>& spans);

/// A strand put back by offsetOpenContour step 7 as chains that do not cross themselves: the
/// strand itself, or its consecutive parts between selfCrossingCuts, each without spans no longer
/// than tol.distance at its ends; parts left empty go.
std::vector<std::vector<Span>> nonCrossingRuns(const std::vector<Span>& strand, const Tolerance& tol);

}  // namespace carvetto::geometry::detail
