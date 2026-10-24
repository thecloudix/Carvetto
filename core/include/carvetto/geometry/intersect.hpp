#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// A single point where two spans meet. `tA`/`tB` are the spans' own parameters (see ADR-0005)
/// clamped to [0,1]; `point` is within `tol.distance` of both A(tA) and B(tB). `tangent` is set when the
/// curves touch without crossing (parallel tangents at the contact point), which includes collinear
/// end-to-end touches.
struct Intersection {
    Vec2 point;
    double tA{0.0};
    double tB{0.0};
    bool tangent{false};
};

/// A stretch along which two spans coincide (collinear line segments or arcs on the same
/// circle). Parameters are clamped to [0,1]; `tA0 < tA1` always, while `tB0`/`tB1` follow B's
/// direction of travel over the same stretch (so `tB0 > tB1` when B runs the other way).
/// Arcs on the same circle may yield two intervals when the overlap wraps across A's start.
struct Overlap {
    double tA0{0.0};
    double tA1{0.0};
    double tB0{0.0};
    double tB1{0.0};
};

/// Everything two spans have in common: isolated points and coincident stretches. Points that
/// lie inside a reported overlap are not repeated in `points`.
struct IntersectionResult {
    std::vector<Intersection> points;
    std::vector<Overlap> overlaps;

    bool isEmpty() const { return points.empty() && overlaps.empty(); }
};

/// Exact line–line intersection, symmetric in its arguments (swapping A and B swaps the
/// parameters and nothing else). Segments are collinear when both end points of the shorter lie
/// within `tol.distance` of the longer one's line; they then give an overlap, or a single tangent
/// point when they only touch end to end. Otherwise the lines are intersected however small the
/// angle between them, and an end point within `tol.distance` of the other segment also counts.
IntersectionResult intersect(const LineSpan& a, const LineSpan& b, const Tolerance& tol);
/// Exact line–arc intersection from the foot of the perpendicular dropped from the centre
/// (hits at ±sqrt(r² − h²) along the line), which stays precise for lines much longer than the
/// circle. A line whose distance from the centre is within `tol.distance` of the radius gives
/// one tangent hit.
IntersectionResult intersect(const LineSpan& a, const ArcSpan& b, const Tolerance& tol);
/// Exact arc–arc intersection from the circle–circle solution. Externally or internally tangent
/// circles (within `tol.distance`) give one tangent hit at the midpoint of the two circles'
/// facing points on the centre line, so it lies within tolerance of both however close the
/// centres are. Arcs on the same circle give overlaps; concentric circles of different radii
/// give nothing.
IntersectionResult intersect(const ArcSpan& a, const ArcSpan& b, const Tolerance& tol);

/// Intersection of any two spans. Line/arc pairs are solved analytically; pairs involving a
/// Bezier are polygonised at `tol.chordal`, candidate crossings are refined by Newton iteration
/// on A(t) - B(s) = 0 (falling back to alternating closest-point projection near tangencies) and
/// accepted when the residual is within `tol.distance`. Degenerate (zero-length) spans are
/// treated as points. Results are ordered by `tA` and de-duplicated within `tol.distance`.
///
/// Limitation: overlap detection is implemented for line/line and arc/arc pairs only. Coincident
/// stretches involving a Bezier are reported as a set of sampled tangent points along the shared
/// part, not as `Overlap`s (Bezier overlap detection is future work; see ADR-0009).
IntersectionResult intersect(const Span& a, const Span& b, const Tolerance& tol);

/// An intersection between two contours (or two spans of one contour), tagged with the span
/// indices it belongs to. For self-intersections `spanA <= spanB`.
struct ContourIntersection {
    std::size_t spanA{0};
    std::size_t spanB{0};
    Intersection hit;
};

/// A coincident stretch between span `spanA` of one contour and span `spanB` of another.
struct ContourOverlap {
    std::size_t spanA{0};
    std::size_t spanB{0};
    Overlap overlap;
};

/// Points and overlaps shared by two contours (or by one contour with itself).
struct ContourIntersections {
    std::vector<ContourIntersection> points;
    std::vector<ContourOverlap> overlaps;

    bool isEmpty() const { return points.empty() && overlaps.empty(); }
};

/// All intersections between spans of `a` and spans of `b`. A hit at a junction shared by two
/// consecutive spans is reported once (first span wins). Ordered by (spanA, tA).
ContourIntersections intersect(const Contour& a, const Contour& b, const Tolerance& tol);

/// Points where a contour crosses, touches or overlaps itself. The shared junction of two
/// consecutive spans (including the closing junction) does not count; everything else does,
/// including a cubic Bezier that loops back through itself (reported with `spanA == spanB`).
///
/// Known limitation (tracked as T-0005): a closed contour of exactly two spans (e.g. a
/// half-disc: line + arc, or a two-Bezier lens) wrongly reports its closing junction as a
/// self-intersection.
ContourIntersections selfIntersections(const Contour& c, const Tolerance& tol);

/// True when `selfIntersections(c, tol)` is non-empty (stops at the first hit).
bool hasSelfIntersections(const Contour& c, const Tolerance& tol);

}  // namespace carvetto::geometry
