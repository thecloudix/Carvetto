#include "v_carve_corners.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/toolpath/v_carve.hpp"

namespace carvetto::toolpath::detail {

namespace {

using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;

/// Most passes either side of a corner's bisector (a stepover tiny against the end mill would
/// otherwise make the work explode; Pocket refuses such stepovers before this is reached).
constexpr long kMaxPassesPerSide = 100000;

double distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
    return geometry::distance(p, a + ab * t);
}

Box2 segmentBox(Vec2 a, Vec2 b) {
    Box2 box;
    box.expand(a);
    box.expand(b);
    return box;
}

/// Length used to skip degenerate spans: lines and arcs exactly, Beziers by their control
/// polygon (an upper bound of their length; the exact length subdivides at cusps).
double spanSize(const geometry::Span& s) {
    if (const auto* bz = std::get_if<geometry::BezierSpan>(&s)) {
        return geometry::distance(bz->p0, bz->p1) + geometry::distance(bz->p1, bz->p2) +
               geometry::distance(bz->p2, bz->p3);
    }
    return geometry::length(s);
}

/// Exact distance test against the boundary spans of a region, indexed by box.
class SpanIndex {
   public:
    explicit SpanIndex(const std::vector<Contour>& boundaries) {
        std::vector<Box2> boxes;
        for (const Contour& c : boundaries) {
            for (const geometry::Span& s : c.spans) {
                spans_.push_back(&s);
                boxes.push_back(geometry::bounds(s));
            }
        }
        index_ = BoxIndex(std::move(boxes));
    }
    /// True when some span is strictly closer to p than `limit`.
    bool closer(Vec2 p, double limit) const {
        const Vec2 h{limit, limit};
        for (const std::size_t i : index_.query(Box2{p - h, p + h})) {
            if (geometry::closestPoint(*spans_[i], p).distance < limit) {
                return true;
            }
        }
        return false;
    }

   private:
    std::vector<const geometry::Span*> spans_;
    BoxIndex index_;
};

}  // namespace

// ----------------------------------------------------------------------------- RingRegion

RingRegion::RingRegion(std::vector<std::vector<Vec2>> rings, double eps) : eps_(eps) {
    std::vector<Box2> boxes;
    for (const std::vector<Vec2>& r : rings) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2 a = r[i];
            const Vec2 b = r[(i + 1) % n];
            if (a == b) {
                continue;
            }
            edges_.push_back(Edge{a, b});
            boxes.push_back(segmentBox(a, b));
            bounds_.expand(a);
        }
    }
    index_ = BoxIndex(std::move(boxes));
}

bool RingRegion::inside(Vec2 p) const {
    if (edges_.empty()) {
        return false;
    }
    const Vec2 h{eps_, eps_};
    for (const std::size_t i : index_.query(Box2{p - h, p + h})) {
        if (distToSegment(p, edges_[i].a, edges_[i].b) <= eps_) {
            return true;
        }
    }
    if (!(bounds_.max.x > p.x)) {
        return false;
    }
    bool in = false;
    for (const std::size_t i : index_.query(Box2{p, Vec2{bounds_.max.x + 1.0, p.y}})) {
        const Vec2 a = edges_[i].a;
        const Vec2 b = edges_[i].b;
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x > p.x) {
                in = !in;
            }
        }
    }
    return in;
}

std::vector<std::pair<double, double>> RingRegion::clip(Vec2 a, Vec2 b) const {
    std::vector<std::pair<double, double>> out;
    const Vec2 d = b - a;
    const double len2 = d.lengthSquared();
    if (!(len2 > 0.0)) {
        if (inside(a)) {
            out.emplace_back(0.0, 1.0);
        }
        return out;
    }
    // Every parameter where the segment may change sides: proper crossings with an edge, and the
    // feet of edge end points within eps of the segment (touches, collinear overlaps).
    std::vector<double> ts{0.0, 1.0};
    for (const std::size_t i : index_.query(segmentBox(a, b).inflated(eps_))) {
        const Vec2 c = edges_[i].a;
        const Vec2 e = edges_[i].b - c;
        const double denom = d.cross(e);
        if (denom != 0.0) {
            const double t = (c - a).cross(e) / denom;
            const double u = (c - a).cross(d) / denom;
            if (t > 0.0 && t < 1.0 && u >= 0.0 && u <= 1.0) {
                ts.push_back(t);
            }
        }
        for (const Vec2 q : {c, edges_[i].b}) {
            if (distToSegment(q, a, b) <= eps_) {
                ts.push_back(std::clamp((q - a).dot(d) / len2, 0.0, 1.0));
            }
        }
    }
    std::sort(ts.begin(), ts.end());
    ts.erase(std::unique(ts.begin(), ts.end()), ts.end());
    for (std::size_t k = 0; k + 1 < ts.size(); ++k) {
        const double t0 = ts[k];
        const double t1 = ts[k + 1];
        if (!inside(a + d * (0.5 * (t0 + t1)))) {
            continue;
        }
        if (!out.empty() && out.back().second == t0) {
            out.back().second = t1;
        } else {
            out.emplace_back(t0, t1);
        }
    }
    return out;
}

bool RingRegion::segmentInside(Vec2 a, Vec2 b) const {
    const std::vector<std::pair<double, double>> in = clip(a, b);
    return in.size() == 1 && in.front().first == 0.0 && in.front().second == 1.0;
}

// ----------------------------------------------------------------------------- corners

double predictedCornerLeftover(const CornerSpec& spec, double angle, double millRadius) {
    const double reach = millRadius * (1.0 - std::sin(0.5 * angle));
    return std::min(spec.limit, vCarvingDepthForRadius(*spec.tool, reach));
}

std::vector<SharpCorner> sharpCorners(const std::vector<Contour>& boundaries, const RingRegion& walls,
                                      const CornerSpec& spec) {
    std::vector<SharpCorner> out;
    if (walls.isEmpty() || spec.millRadii.empty()) {
        return out;
    }
    std::vector<double> radii = spec.millRadii;
    std::sort(radii.begin(), radii.end());
    std::optional<SpanIndex> spans;  // built when a corner needs the reach test
    for (std::size_t k = 0; k < boundaries.size(); ++k) {
        const Contour& c = boundaries[k];
        if (!c.closed) {
            continue;
        }
        // The region lies left of the outer when it runs counter-clockwise and left of a hole when
        // the hole runs clockwise.
        const double area =
            geometry::signedArea(c.transformed(geometry::Transform2::translation(-c.bounds().centre())));
        if (!(area != 0.0) || !std::isfinite(area)) {
            continue;
        }
        const bool insideLeft = (area > 0.0) == (k == 0);
        std::vector<std::size_t> live;
        for (std::size_t i = 0; i < c.spans.size(); ++i) {
            if (spanSize(c.spans[i]) > spec.distance) {
                live.push_back(i);
            }
        }
        for (std::size_t j = 0; j < live.size() && live.size() >= 2; ++j) {
            const geometry::Span& in = c.spans[live[j]];
            const geometry::Span& next = c.spans[live[(j + 1) % live.size()]];
            const Vec2 a = geometry::tangentAt(in, 1.0);
            const Vec2 b = geometry::tangentAt(next, 0.0);
            const double turn = std::atan2(a.cross(b), a.dot(b));
            const double towardsInside = insideLeft ? turn : -turn;
            if (!(towardsInside > 0.0)) {
                continue;  // straight on, or a reflex corner
            }
            const double angle = geometry::kPi - towardsInside;
            const double half = 0.5 * angle;
            const double sh = std::sin(half);
            const Vec2 bis = b - a;
            const double bl = bis.length();
            if (!(angle > 1e-9) || !(sh > 0.0) || !(bl > 0.0)) {
                continue;  // a spike back on itself: no corner to speak of
            }
            // The leftover only grows with the end mill's radius: below the threshold even for
            // the largest one, the corner is left alone without further work.
            if (!(predictedCornerLeftover(spec, angle, radii.back()) > spec.chordal)) {
                continue;
            }
            const Vec2 u = bis / bl;
            const Vec2 v = geometry::startPoint(next);
            // A flat-area corner: just inside the wall ring's corner lies inside the wall rings.
            if (!walls.inside(v + u * (spec.ringInset / sh + spec.chordal))) {
                continue;
            }
            // The smallest end mill that reaches into the corner: its centre at the corner of its
            // centre region (inset by R + ρ + Pocket's chordal) is clear of every wall.
            if (!spans) {
                spans.emplace(boundaries);
            }
            double mill = 0.0;
            for (const double r : radii) {
                const double inset = r + spec.limitRadius + spec.pocketChordal;
                if (!spans->closer(v + u * (inset / sh), inset - 0.5 * spec.pocketChordal)) {
                    mill = r;
                    break;
                }
            }
            if (!(mill > 0.0)) {
                continue;  // no end mill reaches: not a clean flat floor there
            }
            const double predicted = predictedCornerLeftover(spec, angle, mill);
            if (!(predicted > spec.chordal)) {
                continue;
            }
            out.push_back(SharpCorner{v, u, angle, mill, predicted});
        }
    }
    return out;
}

std::vector<std::vector<Vec2>> cornerPasses(const SharpCorner& corner, const RingRegion& walls, const CornerSpec& spec) {
    std::vector<std::vector<Vec2>> cuts;
    const double half = 0.5 * corner.angle;
    const double sh = std::sin(half);
    const double ch = std::cos(half);
    const double th = sh / ch;
    const Vec2 u = corner.bisector;
    const Vec2 n{-u.y, u.x};
    // Frame: x along the bisector from the wall ring's corner W, y across it. The ring's edges
    // leave W at ±α/2; the end mill's centre stops at C = (R / sin(α/2), 0) and moves away along
    // the rays from C parallel to the edges, so its swept area is the band within R of them.
    const Vec2 w = corner.vertex + u * (spec.ringInset / sh);
    const double r = corner.millRadius;
    const double l = r / sh;
    const double ov = std::min(spec.chordal, 0.5 * r);
    const double re = r - ov;  // overlap: the passes end this far inside the swept area
    const double yMax = r * ch;  // the fillet touches the ring's edges at |y| = R cos(α/2)
    const double s = spec.stepover;
    if (!(s > 0.0)) {
        return cuts;
    }
    const auto sides = static_cast<long>(std::min(std::floor(yMax / s), static_cast<double>(kMaxPassesPerSide)));
    // Each pass starts a little outside the wall ring's edge (clipped back to the ring itself).
    const double ext = (3.0 * spec.pocketChordal + spec.distance) / sh;
    struct Piece {
        Vec2 from;
        Vec2 to;
    };
    std::vector<std::vector<Piece>> passes;
    for (long k = -sides; k <= sides; ++k) {
        const double y = static_cast<double>(k) * s;
        const double ay = std::fabs(y);
        const double x0 = ay / th;
        // First x at which the line is within re of the end mill's centre path.
        double entry = std::numeric_limits<double>::infinity();
        if (ay < re) {
            entry = l - std::sqrt((re - ay) * (re + ay));
        }
        const double xr = l + (ay * ch - re) / sh;
        if ((xr - l) * ch + ay * sh >= 0.0) {
            entry = std::min(entry, xr);
        }
        if (!(entry > x0 + spec.distance) || !std::isfinite(entry)) {
            continue;  // already inside the swept area at the ring
        }
        const Vec2 p0 = w + u * (x0 - ext) + n * y;
        const Vec2 p1 = w + u * entry + n * y;
        std::vector<Piece> pieces;
        for (const auto& [t0, t1] : walls.clip(p0, p1)) {
            const Vec2 a = p0 + (p1 - p0) * t0;
            const Vec2 b = p0 + (p1 - p0) * t1;
            if (geometry::distance(a, b) > spec.distance) {
                pieces.push_back(Piece{a, b});
            }
        }
        if (!pieces.empty()) {
            passes.push_back(std::move(pieces));
        }
    }
    // Zig-zag: every other pass runs backwards, so consecutive passes end near each other (both in
    // the swept area, or both on the ring's edge) and join at depth when the link stays inside.
    std::vector<Vec2> cur;
    for (std::size_t i = 0; i < passes.size(); ++i) {
        std::vector<Piece> pieces = passes[i];
        if (i % 2 == 1) {
            std::reverse(pieces.begin(), pieces.end());
            for (Piece& p : pieces) {
                std::swap(p.from, p.to);
            }
        }
        for (const Piece& p : pieces) {
            if (!cur.empty() && !walls.segmentInside(cur.back(), p.from)) {
                cuts.push_back(std::move(cur));
                cur.clear();
            }
            if (cur.empty() || cur.back() != p.from) {
                cur.push_back(p.from);
            }
            cur.push_back(p.to);
        }
    }
    if (!cur.empty()) {
        cuts.push_back(std::move(cur));
    }
    return cuts;
}

}  // namespace carvetto::toolpath::detail
