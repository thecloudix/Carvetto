#include "carvetto/geometry/vector_unwrapper.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace carvetto::geometry {

namespace {

constexpr double kCentreEps = 1e-12;

/// `a` brought within pi of `ref` by whole turns.
double nearAngle(double a, double ref) {
    return a - kTwoPi * std::round((a - ref) / kTwoPi);
}

struct Polar {
    double angle;  ///< Continuous (unwrapped) angle.
    double dist;
};

Polar polarOf(Vec2 p, Vec2 centre, double prevAngle) {
    const Vec2 d = p - centre;
    const double dist = d.length();
    if (dist <= kCentreEps) return {prevAngle, 0.0};
    return {nearAngle(std::atan2(d.y, d.x), prevAngle), dist};
}

struct Refiner {
    Vec2 centre;
    double radius;
    double sign;
    double tolerance;
    std::vector<Vec2>* out;
    bool tooMany{false};

    Vec2 unrolled(const Polar& p) const { return {sign * p.angle * radius, p.dist}; }

    /// Appends the points strictly between the segment ends (a->b in cross-section space), then the caller
    /// appends b. `pa` and `pb` are the polar coordinates of the ends (continuous angles).
    void split(Vec2 a, Vec2 b, const Polar& pa, const Polar& pb, int depth) {
        if (depth <= 0 || tooMany) return;
        const Vec2 ua = unrolled(pa), ub = unrolled(pb);
        bool needed = false;
        for (const double t : {0.25, 0.5, 0.75}) {
            const Polar pm = polarOf(a + (b - a) * t, centre, pa.angle);
            const Vec2 truth = unrolled(pm);
            if (distance(truth, ua + (ub - ua) * t) > tolerance) {
                needed = true;
                break;
            }
        }
        if (!needed) return;
        const Vec2 m = (a + b) * 0.5;
        const Polar pm = polarOf(m, centre, pa.angle);
        split(a, m, pa, pm, depth - 1);
        if (out->size() >= kMaxUnwrapPoints) {
            tooMany = true;
            return;
        }
        out->push_back(unrolled(pm));
        split(m, b, pm, pb, depth - 1);
    }
};

}  // namespace

Vec2 unwrapCentre(const UnwrapOptions& options, const Box2& bounds) {
    switch (options.centre) {
        case UnwrapCentre::Workspace: return options.workspaceCentre;
        case UnwrapCentre::Custom: return options.customCentre;
        case UnwrapCentre::Contour: break;
    }
    return (bounds.min + bounds.max) * 0.5;
}

ShapeResult<UnwrapResult> unwrapCrossSection(const Polyline& cs, const UnwrapOptions& o) {
    if (cs.points.size() < 2) return std::string("A cross-section needs at least two points.");
    if (!(std::isfinite(o.radius) && o.radius > 0.0)) return std::string("The cylinder radius must be positive.");
    if (o.refine && !(std::isfinite(o.tolerance) && o.tolerance > 0.0)) {
        return std::string("The tolerance must be positive.");
    }
    if (cs.points.size() > kMaxUnwrapPoints) return std::string("The cross-section has too many points.");
    for (const Vec2& p : cs.points) {
        if (!(std::isfinite(p.x) && std::isfinite(p.y))) return std::string("The cross-section has a non-finite point.");
    }
    if (o.centre == UnwrapCentre::Workspace &&
        !(std::isfinite(o.workspaceCentre.x) && std::isfinite(o.workspaceCentre.y))) {
        return std::string("The workspace centre is not a number.");
    }
    if (o.centre == UnwrapCentre::Custom && !(std::isfinite(o.customCentre.x) && std::isfinite(o.customCentre.y))) {
        return std::string("The centre point is not a number.");
    }
    UnwrapResult result;
    result.centre = unwrapCentre(o, cs.bounds());
    const Vec2 centre = result.centre;
    const double sign = o.flip ? -1.0 : 1.0;

    // Continuous polar coordinates; a closed contour is walked once more back to its start.
    std::vector<Polar> polar;
    polar.reserve(cs.points.size() + 1);
    {
        const Vec2 d = cs.points[0] - centre;
        const double dist = d.length();
        polar.push_back({dist <= kCentreEps ? 0.0 : std::atan2(d.y, d.x), dist});
    }
    for (std::size_t i = 1; i < cs.points.size(); ++i) polar.push_back(polarOf(cs.points[i], centre, polar.back().angle));
    std::size_t segments = cs.points.size() - 1;
    if (cs.closed) {
        const Polar back = polarOf(cs.points[0], centre, polar.back().angle);
        result.turns = static_cast<int>(std::lround((back.angle - polar[0].angle) / kTwoPi));
        if (result.turns != 0) {
            polar.push_back(back);
            segments = cs.points.size();
        } else {
            segments = cs.points.size();  // the closing segment is refined too; the closing vertex is dropped
        }
    }
    // The point a segment ends at, in cross-section space.
    auto pointAt = [&](std::size_t i) { return cs.points[i % cs.points.size()]; };
    const bool closedOut = cs.closed && result.turns == 0;
    std::vector<Vec2> out;
    Refiner refiner{centre, o.radius, sign, o.tolerance, &out, false};
    out.push_back(refiner.unrolled(polar[0]));
    for (std::size_t s = 0; s < segments; ++s) {
        const bool closing = closedOut && s + 1 == segments;
        Polar pb = closing ? polarOf(cs.points[0], centre, polar[s].angle) : polar[s + 1];
        if (o.refine) {
            refiner.split(pointAt(s), pointAt(s + 1), polar[s], pb, 24);
            if (refiner.tooMany) return std::string("Unwrapping needs too many points; raise the tolerance.");
        }
        if (!closing) out.push_back(refiner.unrolled(pb));
        if (out.size() > kMaxUnwrapPoints) return std::string("Unwrapping needs too many points; raise the tolerance.");
    }
    result.contour.points = std::move(out);
    result.contour.closed = closedOut;
    return result;
}

Polyline wrapCrossSection(const Polyline& unrolled, Vec2 centre, double radius, bool flip) {
    Polyline out;
    out.closed = unrolled.closed;
    out.points.reserve(unrolled.points.size());
    const double sign = flip ? -1.0 : 1.0;
    for (const Vec2& u : unrolled.points) {
        const double angle = sign * u.x / radius;
        out.points.push_back(centre + Vec2{std::cos(angle), std::sin(angle)} * u.y);
    }
    return out;
}

}  // namespace carvetto::geometry
