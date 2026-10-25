#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

// ------------------------------------------------------------- area / orientation

/// Signed area enclosed by a closed contour (Green's theorem), positive when the contour runs
/// counter-clockwise. Exact for lines and arcs; Bezier contributions are integrated with a
/// 3-point Gauss–Legendre rule, which is exact for the degree-5 integrand. Open contours
/// return 0, matching `Polyline::signedArea`.
double signedArea(const Contour& c);

/// Absolute enclosed area of a closed contour (0 when open).
double area(const Contour& c);

/// Direction of travel around a closed contour.
enum class Orientation { CCW, CW, Degenerate };

/// Orientation from the sign of the enclosed area; `Degenerate` when the contour is open or
/// its area is below `tol.distance` squared.
Orientation orientation(const Contour& c, const Tolerance& tol);

/// Reverses `c` in place when needed so that it runs in `want`. No-op for open or degenerate
/// contours, or when `want` is `Degenerate`.
void setOrientation(Contour& c, Orientation want, const Tolerance& tol);

// ----------------------------------------------------------------- point in contour

/// Where a point lies relative to a (set of) closed contour(s).
enum class PointLocation { Inside, Outside, OnBoundary };

/// Winding number of `c` around `p`: the net count of counter-clockwise turns the closed contour
/// makes about the point (positive for CCW). Computed by casting a ray towards +X and summing
/// signed crossings of y-monotone pieces, which makes the half-open vertex rule fall out
/// naturally. Lines and arcs are exact; Beziers are polygonised at `tol.chordal`. Open contours
/// are treated as implicitly closed. Undefined (but finite) for points on the boundary.
int windingNumber(const Contour& c, Vec2 p, const Tolerance& tol);

/// Classifies `p` against one closed contour. Points within `tol.distance` of the boundary are
/// `OnBoundary`; `rule` decides the interior of a self-intersecting contour.
PointLocation locatePoint(const Contour& c, Vec2 p, FillRule rule, const Tolerance& tol);
/// Classifies `p` against a set of closed contours treated as one filled region (outer
/// boundaries and holes together) under `rule`.
PointLocation locatePoint(const std::vector<Contour>& contours, Vec2 p, FillRule rule, const Tolerance& tol);

/// True when `p` is inside or on the boundary of `c` under `rule`. Independent of orientation.
bool isInside(const Contour& c, Vec2 p, FillRule rule, const Tolerance& tol);
/// True when `p` is inside or on the boundary of the region described by `contours` under `rule`.
bool isInside(const std::vector<Contour>& contours, Vec2 p, FillRule rule, const Tolerance& tol);

// -------------------------------------------------------------------- closest point

/// Nearest point of a span to a query point.
struct ClosestPoint {
    /// The point on the span.
    Vec2 point;
    /// Span parameter in [0,1] of that point.
    double t{0.0};
    /// Distance from the query point.
    double distance{0.0};
};

/// Closest point on a line segment (orthogonal projection clamped to the segment).
ClosestPoint closestPoint(const LineSpan& s, Vec2 p);
/// Closest point on an arc: radial projection when the query direction falls inside the sweep,
/// otherwise the nearer end point. A query at the centre returns the start point.
ClosestPoint closestPoint(const ArcSpan& s, Vec2 p);
/// Global closest point on a cubic Bezier: every root in [0,1] of the quintic
/// (B(t) - p) . B'(t) is isolated by Bernstein subdivision and bisected, and the best of those
/// and both end points is returned (loops, cusps and degenerate controls included).
ClosestPoint closestPoint(const BezierSpan& s, Vec2 p);
/// Closest point on any span.
ClosestPoint closestPoint(const Span& s, Vec2 p);

/// Nearest point of a contour to a query point, with the span it lies on.
struct ContourClosestPoint {
    Vec2 point;
    std::size_t spanIndex{0};
    double t{0.0};
    double distance{0.0};
};

/// Closest point over all spans of `c` (ties go to the lower span index). An empty contour
/// returns `p` itself at infinite distance.
ContourClosestPoint closestPoint(const Contour& c, Vec2 p);

/// Shortest distance from `p` to the contour's spans (infinite for an empty contour).
double distanceTo(const Contour& c, Vec2 p);

}  // namespace carvetto::geometry
