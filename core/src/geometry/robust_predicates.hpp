#pragma once

// Exact orientation and in-circle predicates for double coordinates (internal, T-0152).
//
// Both predicates first evaluate the determinant in double precision together with a forward
// error bound; only when the result is within that bound (or near the underflow range) is it
// recomputed exactly with floating-point expansion arithmetic (sums of non-overlapping doubles,
// error-free transforms TwoSum and TwoProduct), following the method published by
// J. R. Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric
// Predicates" (1997). TwoProduct uses std::fma, so exactness does not depend on floating-point
// contraction (GCC contracts by default on aarch64; MSVC's default /fp:precise does not).
// Before the exact evaluation the points are scaled by a power of two (exact) so the largest
// coordinate is about 1, which keeps tiny regions out of underflow.
//
// The returned sign is exact for finite input whose largest coordinate magnitude M is below
// about 1e70 and in which every non-zero coordinate, and every non-zero coordinate difference,
// is at least about 1e-70 M in magnitude (a single tiny non-zero coordinate, ~1e-150 M or less,
// breaks it even when the differences are large); geometry in millimetres is far inside this.
// IEEE double arithmetic with 53-bit rounding is assumed (SSE2, or x87 in double-precision
// mode): x87 extended precision can double-round inside the error-free sums.

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry::detail {

/// Error-free product: x = fl(a * b) and x + y == a * b exactly (barring overflow/underflow).
void twoProduct(double a, double b, double& x, double& y);

/// Sign of the orientation determinant: +1 when a, b, c turn counter-clockwise, -1 clockwise,
/// 0 when exactly collinear.
int orient2dSign(Vec2 a, Vec2 b, Vec2 c);

/// Sign of the in-circle determinant: +1 when d lies strictly inside the circle through the
/// counter-clockwise triangle a, b, c, -1 strictly outside, 0 exactly on it (the sign flips for
/// a clockwise triangle).
int incircleSign(Vec2 a, Vec2 b, Vec2 c, Vec2 d);

}  // namespace carvetto::geometry::detail
