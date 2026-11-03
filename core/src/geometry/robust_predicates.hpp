#pragma once

// Exact orientation and in-circle predicates for double coordinates (internal, T-0152).
//
// Both predicates first evaluate the determinant in double precision together with a forward
// error bound; only when the result is within that bound is it recomputed exactly with
// floating-point expansion arithmetic (sums of non-overlapping doubles, error-free transforms
// TwoSum / TwoProduct with Dekker splitting), following the method published by J. R. Shewchuk,
// "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates" (1997).
// The returned sign is exact for any finite input whose products neither overflow nor underflow
// (coordinates between about 1e-140 and 1e+70 in magnitude, or zero).

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
