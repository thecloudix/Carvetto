#pragma once

#include <string>
#include <utility>
#include <variant>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

/// Shape builders (T-0200): pure functions from drawing parameters to exact span contours.
///
/// Conventions shared by every closed builder (stable; tests pin them):
///  * Winding is counter-clockwise (positive `signedArea`).
///  * The start point is documented per builder; it is a function of the parameters only.
///  * Output depends on nothing but the arguments, so rebuilding from stored parameters gives
///    bit-identical contours.
///  * Invalid parameters (non-finite, non-positive sizes, impossible radii) return an error
///    message and never a contour.
///  * Limits: every coordinate and size must be finite with magnitude at most 1e9 mm (larger
///    values return an error), polygon sides and star points are capped at 100000, and an
///    ellipse never uses more than 1024 Bezier spans.
///  * Zero-length spans are never emitted: where a corner radius or chamfer consumes a whole
///    side, that side's line span is left out.
namespace carvetto::geometry {

/// Value-or-error result of a shape builder.
template <typename T>
class ShapeResult {
public:
    ShapeResult(T value) : v_(std::move(value)) {}                 // NOLINT(google-explicit-constructor)
    ShapeResult(std::string error) : v_(std::move(error)) {}       // NOLINT(google-explicit-constructor)
    bool ok() const { return std::holds_alternative<T>(v_); }
    explicit operator bool() const { return ok(); }
    /// Precondition: `ok()`.
    const T& value() const { return std::get<T>(v_); }
    T& value() { return std::get<T>(v_); }
    /// Precondition: `!ok()`.
    const std::string& error() const { return std::get<std::string>(v_); }

private:
    std::variant<T, std::string> v_;
};

/// How a rectangle corner is treated (mirrors `document::RectangleParams::CornerStyle`).
enum class RectCorner { Square, Rounded, Chamfered };

/// Axis-aligned rectangle of `width` x `height` about `centre`.
///
/// `Rounded`: quarter-circle arcs of radius `cornerSize` (exact). `Chamfered`: straight cuts
/// whose two legs each measure `cornerSize`. `cornerSize` must be in [0, min(width,height)/2];
/// 0 gives a plain rectangle. `Square` ignores `cornerSize`.
/// Closed, CCW. Start point: on the bottom side at the left end of its straight part, i.e.
/// (xmin + cornerSize, ymin) (the bottom-left corner for square corners); spans run bottom,
/// bottom-right corner, right, top-right corner, top, top-left corner, left, bottom-left corner.
ShapeResult<Contour> buildRectangle(Vec2 centre, double width, double height, RectCorner corner,
                                    double cornerSize);

/// Circle of one full-circle CCW arc span (exact). Start point (centre.x + radius, centre.y).
ShapeResult<Contour> buildCircle(Vec2 centre, double radius);

/// Ellipse of full axes `width` (local x) and `height` (local y), rotated CCW by `angle`.
///
/// Built from cubic Beziers (4, 8, 16 ... equal parameter steps), the count chosen so the
/// radial deviation from the true ellipse is below `tol.distance` (never more than 1024 spans);
/// the Bezier ends lie exactly on the ellipse. When width == height the result is the exact
/// circle arc (one span, see `buildCircle`) and `angle` has no effect.
ShapeResult<Contour> buildEllipse(Vec2 centre, double width, double height, double angle,
                                  const Tolerance& tol = Tolerance::defaults());

/// Regular polygon with `sides` >= 3 vertices on the circle of `radius` (circumscribed).
/// Closed, CCW; vertex k is at angle `angle + 2*pi*k/sides`; start at vertex 0.
ShapeResult<Contour> buildPolygon(Vec2 centre, int sides, double radius, double angle);

/// Star with `points` >= 3 tips on `outerRadius` and valleys on `innerRadius`, with
/// 0 < innerRadius < outerRadius. Closed, CCW, 2*points line spans; tip k is at angle
/// `angle + 2*pi*k/points`, the valley after it at half a step further. Start at tip 0.
ShapeResult<Contour> buildStar(Vec2 centre, int points, double outerRadius, double innerRadius,
                               double angle);

/// Arc from `a` through `b` to `c` (open, one ArcSpan). Direction follows the turn a->b->c.
/// Error when the points are collinear or two coincide (within `tol.distance`).
ShapeResult<Contour> buildArcThreePoint(Vec2 a, Vec2 b, Vec2 c, const Tolerance& tol = Tolerance::defaults());

/// Arc about `centre` from `start` to the ray through `end` (radius = |start - centre|), travelling
/// in `dir`. The end point is the projection of `end` on the circle. Error when start or end
/// coincides with the centre, or start and end lie on the same ray (a full circle is not an arc).
ShapeResult<Contour> buildArcCentreStartEnd(Vec2 centre, Vec2 start, Vec2 end, ArcDirection dir,
                                            const Tolerance& tol = Tolerance::defaults());

/// Arc of `radius` from `start` to `end`, travelling in `dir`; `large` selects the arc of more
/// than half a circle (the other of the two arcs is the shorter one). Error when the points
/// coincide or the chord exceeds the diameter (beyond `tol.distance`; a chord within it gives
/// the semicircle).
ShapeResult<Contour> buildArcStartEndRadius(Vec2 start, Vec2 end, double radius, ArcDirection dir,
                                            bool large, const Tolerance& tol = Tolerance::defaults());

}  // namespace carvetto::geometry
