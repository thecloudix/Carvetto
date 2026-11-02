#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"

/// Internal: curve conversions used by the DXF importer (ADR-0015).
namespace carvetto::io::detail {

/// Outcome of a curve conversion.
enum class CurveStatus { Ok, Invalid, TooManySpans };

/// Upper bound of the radial error of the standard cubic Bezier approximation
/// (control distance 4/3*tan(theta/4)) of a unit-circle arc of `theta` radians (|theta| <= pi).
double unitArcBezierError(double theta);

/// Largest singular value of the 2x2 matrix with columns `u` and `v`.
double maxSingularValue(geometry::Vec2 u, geometry::Vec2 v);

/// Appends spans for the elliptical arc P(t) = c + cos(t)*u + sin(t)*v, t from `t0` over the
/// signed `sweep` (|sweep| >= 2*pi - 1e-12 means the full curve). When the curve is a circle
/// (|u| = |v|, u perpendicular to v) one exact `ArcSpan` is produced; otherwise cubic Beziers
/// whose deviation is at most `chordal`. Fails with TooManySpans when more than `budget` spans
/// would be needed.
CurveStatus appendEllipticalArc(geometry::Vec2 c, geometry::Vec2 u, geometry::Vec2 v, double t0, double sweep,
                                double chordal, std::size_t budget, std::vector<geometry::Span>& out);

/// A NURBS curve as stored in a DXF SPLINE entity.
struct NurbsCurve {
    int degree{3};
    std::vector<double> knots;
    std::vector<geometry::Vec2> controls;
    /// Empty, or one positive weight per control point.
    std::vector<double> weights;
};

/// Maximum NURBS degree accepted (higher degrees are rejected as invalid).
inline constexpr int kMaxNurbsDegree = 11;

/// Converts a NURBS curve to spans. Non-rational curves of degree <= 3 convert exactly (degree 1
/// gives lines, degree 2 is elevated). Rational curves and degrees > 3 are split into rational
/// Bezier pieces (blossoming) and each piece is approximated by cubic Hermite Beziers,
/// subdivided until 23 interior samples lie within `chordal` (at most
/// 24 halvings per knot span). `why` describes an Invalid result.
CurveStatus nurbsToSpans(const NurbsCurve& curve, double chordal, std::size_t budget,
                         std::vector<geometry::Span>& out, std::string& why);

/// C2 cubic interpolation through `points` with chord-length parameters, as cubic Beziers.
/// End tangents are used (as unit directions) when given, otherwise natural end conditions.
/// Consecutive duplicate points are dropped. Fails with Invalid for fewer than two points.
CurveStatus interpolateFitPoints(std::span<const geometry::Vec2> points,
                                 std::optional<geometry::Vec2> startTangent,
                                 std::optional<geometry::Vec2> endTangent, std::size_t budget,
                                 std::vector<geometry::Span>& out);

}  // namespace carvetto::io::detail
