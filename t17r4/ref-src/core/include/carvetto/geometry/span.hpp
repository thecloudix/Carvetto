#pragma once

#include <cstddef>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

struct BezierSpan;

/// Direction of travel around an arc.
enum class ArcDirection { CCW, CW };

/// Straight segment from `start` to `end`.
struct LineSpan {
    Vec2 start;
    Vec2 end;

    constexpr bool operator==(const LineSpan&) const = default;

    double length() const { return distance(start, end); }
    /// Point at parameter t in [0,1] (linear in arc length).
    constexpr Vec2 pointAt(double t) const { return lerp(start, end, t); }
    /// Unit tangent (direction of travel); {0,0} for a degenerate span.
    Vec2 tangentAt(double t) const;
    Box2 bounds() const { return Box2{start, end}; }
    LineSpan reversed() const { return {end, start}; }
    LineSpan transformed(const Transform2& m) const { return {m.apply(start), m.apply(end)}; }
};

/// Circular arc given by centre, radius, start/end angles (radians from +X) and direction.
///
/// The arc travels from `startAngle` to `endAngle` in `direction`. The swept angle is
/// taken in (0, 2*pi]: when the two angles coincide (mod 2*pi) the arc is a full circle,
/// so a zero-length arc cannot be represented (use a degenerate LineSpan instead).
struct ArcSpan {
    Vec2 centre;
    double radius{0.0};
    double startAngle{0.0};
    double endAngle{0.0};
    ArcDirection direction{ArcDirection::CCW};

    constexpr bool operator==(const ArcSpan&) const = default;

    /// Full circle starting (and ending) at angle 0, i.e. at (centre.x + radius, centre.y).
    static ArcSpan fullCircle(Vec2 centre, double radius, ArcDirection dir = ArcDirection::CCW);
    /// Arc through `start` and `end` about `centre`; the radius is taken from `start`.
    static ArcSpan fromCentre(Vec2 centre, Vec2 start, Vec2 end, ArcDirection dir);

    /// Signed swept angle: positive in (0, 2*pi] for CCW, negative in [-2*pi, 0) for CW.
    double sweepAngle() const;
    /// True when the arc covers the whole circle.
    bool isFullCircle(double angleTol = 1e-9) const;
    double length() const { return radius * std::fabs(sweepAngle()); }
    /// Angle at parameter t in [0,1].
    double angleAt(double t) const { return startAngle + t * sweepAngle(); }
    /// Point at parameter t in [0,1] (linear in arc length).
    Vec2 pointAt(double t) const { return centre + Vec2::fromAngle(angleAt(t), radius); }
    /// Unit tangent in the direction of travel at parameter t.
    Vec2 tangentAt(double t) const;
    Vec2 startPoint() const { return pointAt(0.0); }
    Vec2 endPoint() const { return pointAt(1.0); }
    /// True when the absolute `angle` lies on the arc (inclusive of the ends, within `angleTol`).
    bool containsAngle(double angle, double angleTol = 1e-9) const;
    /// Exact bounds, accounting for axis extremes inside the sweep.
    Box2 bounds() const;
    ArcSpan reversed() const;
    /// Approximates the arc by cubic Beziers of at most 90 degrees each.
    std::vector<BezierSpan> toBeziers() const;
    /// Number of equal chords needed so that chordal deviation <= `chordalTol` (at least 1).
    std::size_t segmentCountFor(double chordalTol) const;
};

/// Cubic Bezier from p0 to p3 with control points p1, p2.
struct BezierSpan {
    Vec2 p0;
    Vec2 p1;
    Vec2 p2;
    Vec2 p3;

    constexpr bool operator==(const BezierSpan&) const = default;

    /// Arc length by adaptive subdivision (see span.cpp for the scheme); `relTol` is relative.
    double length(double relTol = 1e-10) const;
    /// Point at curve parameter t in [0,1] (de Casteljau; NOT linear in arc length).
    Vec2 pointAt(double t) const;
    /// First derivative at t.
    Vec2 derivativeAt(double t) const;
    /// Unit tangent at t; handles degenerate end tangents by falling back to neighbouring controls.
    Vec2 tangentAt(double t) const;
    /// Exact bounds from the extrema of the x(t) and y(t) polynomials.
    Box2 bounds() const;
    BezierSpan reversed() const { return {p3, p2, p1, p0}; }
    BezierSpan transformed(const Transform2& m) const {
        return {m.apply(p0), m.apply(p1), m.apply(p2), m.apply(p3)};
    }
    /// Splits at parameter t into two Beziers.
    std::pair<BezierSpan, BezierSpan> split(double t) const;
    /// Distance of the control points from the chord p0-p3 (flatness measure).
    double flatness() const;
};

/// One piece of a contour: a line, a circular arc or a cubic Bezier.
using Span = std::variant<LineSpan, ArcSpan, BezierSpan>;

/// Arc length of the span.
double length(const Span& s);
/// Start point of the span.
Vec2 startPoint(const Span& s);
/// End point of the span.
Vec2 endPoint(const Span& s);
/// Point at the span's own parameter t in [0,1].
Vec2 pointAt(const Span& s, double t);
/// Unit tangent in the direction of travel at parameter t.
Vec2 tangentAt(const Span& s, double t);
/// Exact bounding box of the span.
Box2 bounds(const Span& s);
/// Same geometry, opposite direction of travel.
Span reversed(const Span& s);
/// Transforms a span. Lines and Beziers map to one span. Arcs stay arcs under conformal
/// transforms (rotation, uniform scale, mirror, translation); under other affine maps the
/// arc is converted to cubic Beziers first, which is why the result is a list.
std::vector<Span> transformed(const Span& s, const Transform2& m);
/// Appends the polygonisation of `s` to `out`, excluding the start point and including the
/// end point, so consecutive spans can be chained. Chordal deviation stays within `tol.chordal`.
void polygoniseInto(const Span& s, const Tolerance& tol, std::vector<Vec2>& out);
/// True when the span has (near) zero length.
bool isDegenerate(const Span& s, const Tolerance& tol);

}  // namespace carvetto::geometry
