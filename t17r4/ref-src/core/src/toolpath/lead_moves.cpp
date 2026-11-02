#include "carvetto/toolpath/lead_moves.hpp"

#include <cmath>

#include "carvetto/geometry/span.hpp"

namespace carvetto::toolpath {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

Vec2 rotate(Vec2 v, double a) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

}  // namespace

Contour leadIn(Vec2 at, Vec2 tangent, bool wasteOnLeft, const LeadOptions& options, double scale) {
    Contour c;
    const Vec2 t = tangent.normalized();
    if (t.lengthSquared() == 0.0) {
        return c;
    }
    if (options.type == LeadType::Straight && options.length > 0.0) {
        // Direction of travel along the lead: the path direction turned away from the waste
        // side, so the lead's start lies on the waste side.
        const Vec2 d = rotate(t, wasteOnLeft ? -options.angle : options.angle);
        c.append(LineSpan{at - d * (options.length * scale), at});
    } else if (options.type == LeadType::Circular && options.radius > 0.0 && options.sweep > 0.0) {
        const double r = options.radius * scale;
        const Vec2 n = wasteOnLeft ? t.perp() : -t.perp();
        const Vec2 centre = at + n * r;
        const double end = (at - centre).angle();
        // Centre on the left: travelling CCW; on the right: CW.
        const ArcDirection dir = wasteOnLeft ? ArcDirection::CCW : ArcDirection::CW;
        const double start = wasteOnLeft ? end - options.sweep : end + options.sweep;
        c.append(ArcSpan{centre, r, start, end, dir});
    }
    return c;
}

Contour leadOut(Vec2 at, Vec2 tangent, bool wasteOnLeft, const LeadOptions& options, double scale) {
    // The lead-out is the lead-in of the reversed travel, reversed: reversing the travel swaps
    // the waste side.
    Contour c = leadIn(at, -tangent, !wasteOnLeft, options, scale);
    c.reverse();
    return c;
}

}  // namespace carvetto::toolpath
