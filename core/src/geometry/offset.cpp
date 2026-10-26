#include "carvetto/geometry/offset.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "clipper_interop.hpp"

namespace carvetto::geometry {

// ---------------------------------------------------------------------------------------------
// Closed contours: Clipper2.
// ---------------------------------------------------------------------------------------------

namespace {

Clipper2Lib::JoinType toClipperJoin(JoinType join) {
    switch (join) {
        case JoinType::Round:
            return Clipper2Lib::JoinType::Round;
        case JoinType::Sharp:
            return Clipper2Lib::JoinType::Miter;
        case JoinType::Square:
            return Clipper2Lib::JoinType::Square;
    }
    return Clipper2Lib::JoinType::Round;
}

}  // namespace

std::vector<Contour> offsetContours(const std::vector<Contour>& contours, double distance, JoinType join,
                                    const Tolerance& tol, double mitreLimit) {
    // Clipper2 approximates round joins with chords; its arc tolerance is in grid units.
    Clipper2Lib::ClipperOffset offsetter(std::max(mitreLimit, 1.0), detail::toClipperDistance(tol.chordal));
    offsetter.PreserveCollinear(false);
    if (!std::isfinite(distance)) {
        return {};
    }
    const bool zeroDistance = tol.isZero(distance);
    // All closed contours go into ONE Clipper2 group so that the outermost one decides the
    // orientation convention and opposite-oriented contours are treated as holes.
    Clipper2Lib::Paths64 closedPaths;
    Clipper2Lib::Paths64 openPaths;
    for (const Contour& c : contours) {
        if (!c.closed && zeroDistance) {
            continue;  // outlining an open contour with zero width yields nothing
        }
        Clipper2Lib::Path64 path = detail::toClipperPath(c, tol);
        if (path.empty()) {
            continue;
        }
        (c.closed ? closedPaths : openPaths).push_back(std::move(path));
    }
    if (closedPaths.empty() && openPaths.empty()) {
        return {};
    }
    if (!closedPaths.empty()) {
        offsetter.AddPaths(closedPaths, toClipperJoin(join), Clipper2Lib::EndType::Polygon);
    }
    if (!openPaths.empty()) {
        const auto end = join == JoinType::Round ? Clipper2Lib::EndType::Round : Clipper2Lib::EndType::Square;
        offsetter.AddPaths(openPaths, toClipperJoin(join), end);
    }
    Clipper2Lib::Paths64 out;
    offsetter.Execute(detail::toClipperDistance(distance), out);
    return detail::contoursFromClipper(out, true);
}

// ---------------------------------------------------------------------------------------------
// Open contours: exact single-sided offset of lines and arcs.
//
// Raw offset + pruning (the approach used by robust polyline offsetters such as
// CavalierContours, described here from first principles):
//  1. Offset every input line/arc on its own (lines shift along their normal, arcs change
//     radius; an arc whose radius would go negative is inverted). Chain the pieces: tangent-
//     continuous junctions need nothing; outside corners get a join (Round: arc of radius d about
//     the vertex; Sharp: both neighbours extended along their own carriers to where they meet,
//     falling back to Square beyond the mitre limit; Square: both neighbours extended to a
//     chamfer line at distance d from the vertex, falling back to Round when an arc cannot
//     reach it); inside corners get a straight connector.
//  2. Discard raw elements lying wholly within d of one input line. Split the rest at all
//     their self-intersections (and where elements overlap on one carrier) and where they
//     cross the rest of the d-neighbourhood boundary (the outer arcs of the circles about the
//     input vertices, offsets on both sides, inverted arcs). A uniform grid supplies the
//     candidate pairs, so this is close to linear in the number of spans.
//  3. Discard every piece that comes closer than d to the input, or that lies strictly inside
//     the region a Sharp/Square corner adds beyond the round neighbourhood (so a neighbour
//     running past a mitre loses its stub), and one of any two pieces that coincide. What
//     remains bounds the d-neighbourhood (plus those corner regions) on the requested side, so
//     spans consumed between inside corners vanish and nothing backtracks.
//  4. Stitch the surviving pieces into components by following their end points.
//  5. Cut every component where it returns to a point it already passed, into open chains and
//     closed loops that never meet themselves, and order them along the input (ADR-0012).
//  6. Snap each chain's junctions so consecutive spans share their end points exactly
//     (T-0006; see snapChain).
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double kParamEps = 1e-9;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Flattened input span list: lines and arcs only, Beziers polygonised, degenerate pieces gone.
std::vector<Span> flattenForOffset(const Contour& c, const Tolerance& tol) {
    std::vector<Span> out;
    for (const Span& s : c.spans) {
        if (isDegenerate(s, tol)) {
            continue;
        }
        if (const auto* bez = std::get_if<BezierSpan>(&s)) {
            std::vector<Vec2> pts{bez->p0};
            polygoniseInto(s, tol, pts);
            for (std::size_t i = 1; i < pts.size(); ++i) {
                if (!tol.equalPoints(pts[i - 1], pts[i])) {
                    out.emplace_back(LineSpan{pts[i - 1], pts[i]});
                }
            }
        } else {
            out.push_back(s);
        }
    }
    return out;
}

// Radius below which an offset arc has collapsed to its centre point (rounding level only: a
// tiny but real offset arc is kept so that the raw chain still passes through the centre).
double collapseRadius(const ArcSpan& arc, double d) {
    return 1e-12 * std::max({1.0, arc.radius, d});
}

// Offsets one line or arc span by `d` towards `sign` (+1 left, -1 right). Returns nullopt when
// an arc collapses (its offset radius is not positive).
std::optional<Span> offsetSpan(const Span& s, double d, double sign) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 n = line->tangentAt(0.0).perp() * (d * sign);
        return Span{LineSpan{line->start + n, line->end + n}};
    }
    const auto& arc = std::get<ArcSpan>(s);
    // On a CCW arc the left normal points at the centre; on a CW arc it points away.
    const double towardsCentre = arc.direction == ArcDirection::CCW ? sign : -sign;
    const double r = arc.radius - towardsCentre * d;
    if (r <= collapseRadius(arc, d)) {
        return std::nullopt;
    }
    ArcSpan out = arc;
    out.radius = r;
    return Span{out};
}

// Intersections of the carriers (infinite line / full circle) of two line-or-arc spans.
std::vector<Vec2> carrierIntersections(const Span& a, const Span& b) {
    std::vector<Vec2> out;
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    const auto* ca = std::get_if<ArcSpan>(&a);
    const auto* cb = std::get_if<ArcSpan>(&b);
    if (la && lb) {
        const Vec2 u = la->end - la->start;
        const Vec2 v = lb->end - lb->start;
        const double den = u.cross(v);
        if (std::fabs(den) <= 1e-12 * u.length() * v.length()) {
            return out;  // parallel
        }
        out.push_back(la->start + u * ((lb->start - la->start).cross(v) / den));
        return out;
    }
    if (la || lb) {
        const LineSpan& line = la ? *la : *lb;
        const ArcSpan& arc = ca ? *ca : *cb;
        const Vec2 u = line.end - line.start;
        const Vec2 w = line.start - arc.centre;
        const double qa = u.lengthSquared();
        const double qb = 2.0 * w.dot(u);
        const double qc = w.lengthSquared() - arc.radius * arc.radius;
        double disc = qb * qb - 4.0 * qa * qc;
        if (disc < 0.0) {
            if (disc < -1e-9 * qa * arc.radius * arc.radius) {
                return out;
            }
            disc = 0.0;  // tangent contact
        }
        const double root = std::sqrt(disc);
        out.push_back(line.start + u * ((-qb - root) / (2.0 * qa)));
        if (root > 0.0) {
            out.push_back(line.start + u * ((-qb + root) / (2.0 * qa)));
        }
        return out;
    }
    const Vec2 delta = cb->centre - ca->centre;
    const double dist = delta.length();
    const double r1 = ca->radius;
    const double r2 = cb->radius;
    if (dist <= 1e-12 || dist > r1 + r2 + 1e-9 || dist < std::fabs(r1 - r2) - 1e-9) {
        return out;
    }
    const Vec2 u = delta / dist;
    const double along = (r1 * r1 - r2 * r2 + dist * dist) / (2.0 * dist);
    const double h = std::sqrt(std::max(0.0, r1 * r1 - along * along));
    const Vec2 base = ca->centre + u * along;
    out.push_back(base + u.perp() * h);
    if (h > 0.0) {
        out.push_back(base - u.perp() * h);
    }
    return out;
}

// Parameter of carrier point `p` relative to the span: 0 at the start, 1 at the end, linear in
// arc length, negative before the start and > 1 past the end. On an arc the gap outside the
// sweep is split between "past the end" and "before the start" by whichever is nearer.
double carrierParam(const Span& s, Vec2 p) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 u = line->end - line->start;
        return (p - line->start).dot(u) / u.lengthSquared();
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double sweep = std::fabs(arc.sweepAngle());
    const double angle = (p - arc.centre).angle();
    const double phi = arc.direction == ArcDirection::CCW ? normalizeAngle(angle - arc.startAngle)
                                                          : normalizeAngle(arc.startAngle - angle);
    if (phi <= sweep) {
        return phi / sweep;
    }
    const double after = phi - sweep;
    const double before = kTwoPi - phi;
    return after < before ? phi / sweep : -before / sweep;
}

// Parameter of carrier point `p` for extending `s` past its end (`pastEnd`) or before its start:
// like carrierParam, but on an arc the WHOLE gap outside the sweep counts as extension in the
// requested direction (carrierParam splits it in half, which made an arc whose mitre point lay
// more than half the gap away fall back to Square/Round, T-0006). Nothing when reaching `p` would
// take the arc (within 1e-9 rad) all the way round to its own other end.
std::optional<double> extensionParam(const Span& s, Vec2 p, bool pastEnd) {
    if (std::holds_alternative<LineSpan>(s)) {
        return carrierParam(s, p);
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double sweep = std::fabs(arc.sweepAngle());
    const double angle = (p - arc.centre).angle();
    const double phi = arc.direction == ArcDirection::CCW ? normalizeAngle(angle - arc.startAngle)
                                                          : normalizeAngle(arc.startAngle - angle);
    if (phi <= sweep) {
        return phi / sweep;
    }
    const double after = phi - sweep;    // travelled past the end
    const double before = kTwoPi - phi;  // travelled back from the start
    const double gap = kTwoPi - sweep;
    if ((pastEnd ? after : before) > gap - 1e-9) {
        return std::nullopt;
    }
    return pastEnd ? 1.0 + after / sweep : -before / sweep;
}

// Same carrier, end moved to `p` (which must lie on the carrier).
Span withEnd(Span s, Vec2 p) {
    if (auto* line = std::get_if<LineSpan>(&s)) {
        line->end = p;
    } else {
        auto& arc = std::get<ArcSpan>(s);
        arc.endAngle = (p - arc.centre).angle();
    }
    return s;
}

// Same carrier, start moved to `p` (which must lie on the carrier).
Span withStart(Span s, Vec2 p) {
    if (auto* line = std::get_if<LineSpan>(&s)) {
        line->start = p;
    } else {
        auto& arc = std::get<ArcSpan>(s);
        arc.startAngle = (p - arc.centre).angle();
    }
    return s;
}

bool inUnit(double t) {
    return t >= -kParamEps && t <= 1.0 + kParamEps;
}

// Point of carrier contact between `a` (ending) and `b` (starting) beyond a's end and before
// b's start, within `maxDist` of `vertex`: where the two extensions first meet (smallest total
// extension length). An arc may be extended over the whole gap outside its sweep.
// Cap on extending an arc to a mitre or chamfer (T-0006): the extension may sweep at most a
// quarter turn and its new end point must stay within `arcReach` (mitreLimit x d) of the corner
// vertex. Beyond that a tiny offset arc would be dragged round much of its circle to meet the
// other side, which is no longer a corner. Lines are bounded by the callers' own reach.
constexpr double kMaxArcExtension = kPi / 2.0;

bool arcExtensionOk(const Span& s, double t, bool pastEnd, Vec2 p, Vec2 vertex, double arcReach) {
    const auto* arc = std::get_if<ArcSpan>(&s);
    if (!arc) {
        return true;
    }
    const double sweep = std::fabs(arc->sweepAngle()) * (pastEnd ? t - 1.0 : -t);
    return sweep <= kMaxArcExtension + 1e-12 && geometry::distance(p, vertex) <= arcReach;
}

std::optional<Vec2> extensionMeet(const Span& a, const Span& b, Vec2 vertex, double maxDist,
                                  double arcReach) {
    std::optional<Vec2> best;
    double bestCost = std::numeric_limits<double>::infinity();
    const double la = length(a);
    const double lb = length(b);
    for (const Vec2 p : carrierIntersections(a, b)) {
        const std::optional<double> ta = extensionParam(a, p, true);
        const std::optional<double> tb = extensionParam(b, p, false);
        if (!ta || !tb || *ta < 1.0 - kParamEps || *tb > kParamEps) {
            continue;
        }
        if (geometry::distance(p, vertex) > maxDist || !arcExtensionOk(a, *ta, true, p, vertex, arcReach) ||
            !arcExtensionOk(b, *tb, false, p, vertex, arcReach)) {
            continue;
        }
        const double cost = std::max(0.0, *ta - 1.0) * la + std::max(0.0, -*tb) * lb;
        if (cost < bestCost) {
            bestCost = cost;
            best = p;
        }
    }
    return best;
}

// Square join: extends `a` and `b` to the chamfer line at distance d from the vertex and
// returns the chamfer; nullopt when an element cannot reach it (caller falls back to Round).
std::optional<LineSpan> squareJoin(Span& a, Span& b, Vec2 vertex, Vec2 tangentIn, double d, double arcReach) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    Vec2 bisector = ((e - vertex) + (s - vertex)).normalized();
    if (bisector.lengthSquared() == 0.0) {
        bisector = tangentIn;  // full reversal: cap perpendicular to the direction of travel
    }
    const Vec2 q = vertex + bisector * d;
    const Span chamferCarrier = LineSpan{q, q + bisector.perp()};
    const double reach = 1.5 * d;  // chamfer ends lie within d / cos(45 deg) of the vertex
    // First hit of each extension (an arc's may use the whole gap outside its sweep).
    std::optional<Vec2> h1;
    double t1 = 0.0;
    for (const Vec2 p : carrierIntersections(a, chamferCarrier)) {
        const std::optional<double> t = extensionParam(a, p, true);
        if (t && *t >= 1.0 - kParamEps && geometry::distance(p, vertex) <= reach &&
            arcExtensionOk(a, *t, true, p, vertex, arcReach) && (!h1 || *t < t1)) {
            h1 = p;
            t1 = *t;
        }
    }
    std::optional<Vec2> h2;
    double t2 = 0.0;
    for (const Vec2 p : carrierIntersections(b, chamferCarrier)) {
        const std::optional<double> t = extensionParam(b, p, false);
        if (t && *t <= kParamEps && geometry::distance(p, vertex) <= reach &&
            arcExtensionOk(b, *t, false, p, vertex, arcReach) && (!h2 || *t > t2)) {
            h2 = p;
            t2 = *t;
        }
    }
    if (!h1 || !h2 || (*h2 - *h1).dot(s - e) < 0.0) {
        return std::nullopt;
    }
    a = withEnd(a, *h1);
    b = withStart(b, *h2);
    return LineSpan{*h1, *h2};
}

// The closed region a Sharp or Square join adds beyond the round d-neighbourhood at one
// corner: vertex -> offset end of the incoming element -> its extension -> (chamfer) -> the
// outgoing element's extension -> its offset start -> vertex. Raw pieces strictly inside it are
// interior to the offset shape (e.g. the stub of a neighbour running past a mitre) and pruned.
using Wedge = std::vector<Span>;

// Piece of `carrier`'s line or circle from `from` to `to` (both on it), or nothing when the two
// points coincide (an arc with equal angles would read as a full circle).
void appendCarrierPiece(Wedge& w, const Span& carrier, Vec2 from, Vec2 to) {
    if (geometry::distance(from, to) <= 1e-12 * std::max(1.0, geometry::distance(from, Vec2{}))) {
        return;
    }
    w.push_back(withStart(withEnd(carrier, to), from));
}

// Joins `prev` (the last raw element) to `next` around an outside corner at `vertex`. Sharp and
// Square modify the two elements in place (extension) and record the region they add in
// `wedges`; any join element is appended to `raw`.
void outsideCorner(std::vector<Span>& raw, Span& next, Vec2 vertex, Vec2 tangentIn, double d, double sign,
                   JoinType join, double mitreLimit, const Tolerance& tol, std::vector<Wedge>& wedges) {
    Span& prev = raw.back();
    const Vec2 e = endPoint(prev);
    const Vec2 s = startPoint(next);
    const double arcReach = std::max(mitreLimit, 1.0) * d;
    auto mitre = [&](double reach) {
        const auto m = extensionMeet(prev, next, vertex, reach, arcReach);
        if (!m) {
            return false;
        }
        Wedge w{LineSpan{vertex, e}};
        appendCarrierPiece(w, prev, e, *m);
        appendCarrierPiece(w, next, *m, s);
        w.emplace_back(LineSpan{s, vertex});
        wedges.push_back(std::move(w));
        prev = withEnd(prev, *m);
        next = withStart(next, *m);
        return true;
    };
    if (join == JoinType::Sharp && mitre(std::max(mitreLimit, 1.0) * d)) {
        return;
    }
    // Sharp beyond the mitre limit (or where the carriers never meet) squares off the corner.
    if (join != JoinType::Round) {
        Span a = prev;
        Span b = next;
        if (const auto chamfer = squareJoin(a, b, vertex, tangentIn, d, arcReach)) {
            // A chamfer shorter than the tolerance (a nearly tangent corner) would be dropped
            // and leave a gap; meeting the extended neighbours there is the same corner.
            if (tol.equalPoints(chamfer->start, chamfer->end) && mitre(1.5 * d)) {
                return;
            }
            Wedge w{LineSpan{vertex, e}};
            appendCarrierPiece(w, prev, e, chamfer->start);
            w.emplace_back(*chamfer);
            appendCarrierPiece(w, next, chamfer->end, s);
            w.emplace_back(LineSpan{s, vertex});
            wedges.push_back(std::move(w));
            prev = a;
            next = b;
            if (!tol.equalPoints(chamfer->start, chamfer->end)) {
                raw.emplace_back(*chamfer);
            }
            return;
        }
        // An arc that cannot reach the chamfer line: fall back to a round corner.
    }
    const ArcDirection dir = sign > 0.0 ? ArcDirection::CW : ArcDirection::CCW;
    raw.emplace_back(ArcSpan::fromCentre(vertex, e, s, dir));
}

enum class CornerKind { Continuous, Outside, Inside };

CornerKind classifyCorner(Vec2 tangentIn, Vec2 tangentOut, double sign) {
    const double cross = tangentIn.cross(tangentOut);
    const double dot = tangentIn.dot(tangentOut);
    if (std::fabs(cross) <= 1e-9) {
        return dot > 0.0 ? CornerKind::Continuous : CornerKind::Outside;  // reversal: cap it
    }
    return sign * cross < 0.0 ? CornerKind::Outside : CornerKind::Inside;
}

// Connects the raw chain to `next` across the corner at `vertex`.
void connect(std::vector<Span>& raw, Span& next, Vec2 vertex, Vec2 tangentIn, Vec2 tangentOut, double d,
             double sign, JoinType join, double mitreLimit, const Tolerance& tol,
             std::vector<Wedge>& wedges) {
    const Vec2 e = endPoint(raw.back());
    if (tol.equalPoints(e, startPoint(next))) {
        return;
    }
    if (classifyCorner(tangentIn, tangentOut, sign) == CornerKind::Outside) {
        outsideCorner(raw, next, vertex, tangentIn, d, sign, join, mitreLimit, tol, wedges);
    } else {
        // Inside corner (or a gap left by an inverted arc): a straight connector. It lies within
        // d of the vertex, so it is always discarded by the pruning step.
        raw.emplace_back(LineSpan{e, startPoint(next)});
    }
}

// Exact distance from `p` to a line or arc span.
double distanceToSpan(Vec2 p, const Span& s) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 ab = line->end - line->start;
        const double len2 = ab.lengthSquared();
        const double t = len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0;
        return geometry::distance(p, line->start + ab * t);
    }
    const auto& arc = std::get<ArcSpan>(s);
    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
        return std::fabs(geometry::distance(p, arc.centre) - arc.radius);
    }
    return std::min(geometry::distance(p, arc.startPoint()), geometry::distance(p, arc.endPoint()));
}

// Sub-span of a line or arc between parameters t0 < t1 (arc-length linear).
Span subSpan(const Span& s, double t0, double t1) {
    if (t0 <= 0.0 && t1 >= 1.0) {
        return s;
    }
    const Vec2 a = pointAt(s, t0);
    const Vec2 b = pointAt(s, t1);
    return withStart(withEnd(s, b), a);
}

// True when two line or arc elements lie on the same infinite line or the same circle.
bool sameCarrier(const Span& a, const Span& b) {
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    if (la && lb) {
        const Vec2 u = la->end - la->start;
        const double len = u.length();
        const double scale = 1e-9 * std::max(1.0, len);
        return std::fabs(u.cross(lb->start - la->start)) <= scale * len &&
               std::fabs(u.cross(lb->end - la->start)) <= scale * len;
    }
    const auto* ca = std::get_if<ArcSpan>(&a);
    const auto* cb = std::get_if<ArcSpan>(&b);
    if (ca && cb) {
        const double scale = 1e-9 * std::max(1.0, ca->radius);
        return geometry::distance(ca->centre, cb->centre) <= scale &&
               std::fabs(ca->radius - cb->radius) <= scale;
    }
    return false;
}

// Uniform grid over a fixed set of boxes, used to find candidate pairs for the intersection,
// split and pruning steps in roughly linear time instead of testing every pair. The cell size
// is chosen so that the grid holds about as many cells as boxes (capped at 1024 per axis); a
// query visits the cells its box overlaps and reports each item once, in no particular order
// (callers only accumulate results, so the order never affects the output).
class BoxGrid {
   public:
    explicit BoxGrid(const std::vector<Box2>& boxes) : boxes_(boxes), stamp_(boxes.size(), 0) {
        for (const Box2& b : boxes_) {
            extent_.expand(b);
        }
        if (boxes_.empty() || extent_.isEmpty()) {
            return;
        }
        const double w = extent_.width();
        const double h = extent_.height();
        const double n = static_cast<double>(boxes_.size());
        double cell = std::sqrt(std::max(w * h, 1e-300) / n);
        cell = std::max({cell, w / kMaxCells, h / kMaxCells, 1e-9});
        nx_ = static_cast<std::size_t>(std::clamp(std::ceil(w / cell), 1.0, kMaxCells));
        ny_ = static_cast<std::size_t>(std::clamp(std::ceil(h / cell), 1.0, kMaxCells));
        cw_ = w > 0.0 ? w / static_cast<double>(nx_) : 1.0;
        ch_ = h > 0.0 ? h / static_cast<double>(ny_) : 1.0;
        cells_.resize(nx_ * ny_);
        for (std::size_t i = 0; i < boxes_.size(); ++i) {
            if (boxes_[i].isEmpty()) {
                continue;  // never reported
            }
            const auto [x0, x1, y0, y1] = cellRange(boxes_[i]);
            for (std::size_t y = y0; y <= y1; ++y) {
                for (std::size_t x = x0; x <= x1; ++x) {
                    cells_[y * nx_ + x].push_back(i);
                }
            }
        }
    }

    // Calls f(i) once for every item whose box intersects `query`.
    template <class F>
    void query(const Box2& query, F&& f) {
        queryUntil(query, [&](std::size_t i) {
            f(i);
            return false;
        });
    }

    // Like query(), but stops as soon as f(i) returns true; returns whether it did.
    template <class F>
    bool queryUntil(const Box2& query, F&& f) {
        if (cells_.empty() || !query.intersects(extent_)) {
            return false;
        }
        ++epoch_;
        const auto [x0, x1, y0, y1] = cellRange(query);
        for (std::size_t y = y0; y <= y1; ++y) {
            for (std::size_t x = x0; x <= x1; ++x) {
                for (const std::size_t i : cells_[y * nx_ + x]) {
                    if (stamp_[i] != epoch_ && boxes_[i].intersects(query)) {
                        stamp_[i] = epoch_;
                        if (f(i)) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

   private:
    static constexpr double kMaxCells = 1024.0;

    struct Range {
        std::size_t x0, x1, y0, y1;
    };

    std::size_t index(double v, double origin, double size, std::size_t count) const {
        const double k = std::floor((v - origin) / size);
        return static_cast<std::size_t>(std::clamp(k, 0.0, static_cast<double>(count - 1)));
    }

    Range cellRange(const Box2& b) const {
        return {index(b.min.x, extent_.min.x, cw_, nx_), index(b.max.x, extent_.min.x, cw_, nx_),
                index(b.min.y, extent_.min.y, ch_, ny_), index(b.max.y, extent_.min.y, ch_, ny_)};
    }

    std::vector<Box2> boxes_;
    Box2 extent_;
    std::size_t nx_{0};
    std::size_t ny_{0};
    double cw_{1.0};
    double ch_{1.0};
    std::vector<std::vector<std::size_t>> cells_;
    std::vector<std::uint64_t> stamp_;
    std::uint64_t epoch_{0};
};

// True when some input span is closer than `limit` to `p`. `witness` is the span that last
// answered yes; it is tried first and updated, because neighbouring samples along the raw chain
// are almost always rejected by the same span (most raw pieces of a dense path at a large
// distance lie deep inside the neighbourhood). Without it each rejected sample visited every
// span within `limit`, which dominated the cost at large d (T-0006). Only spans whose boxes meet
// the query square can be closer than `limit`, so the answer is exactly that of a full scan.
bool closerThan(Vec2 p, double limit, const std::vector<Span>& input, BoxGrid& grid, std::size_t& witness) {
    if (witness < input.size() && distanceToSpan(p, input[witness]) < limit) {
        return true;
    }
    return grid.queryUntil(Box2{p - Vec2{limit, limit}, p + Vec2{limit, limit}}, [&](std::size_t i) {
        if (i != witness && distanceToSpan(p, input[i]) < limit) {
            witness = i;
            return true;
        }
        return false;
    });
}

// Points whose convex hull contains the line or arc `s`: a line's end points; for an arc, the
// end points of sub-arcs of at most 90 degrees plus, for each, the meeting point of its end
// tangents (at radius r / cos(half sweep) on the bisector).
void hullPoints(const Span& s, std::vector<Vec2>& out) {
    out.clear();
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        out.push_back(line->start);
        out.push_back(line->end);
        return;
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double sweep = arc.sweepAngle();
    const int parts = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / (kPi / 2.0))));
    const double step = sweep / parts;
    const double apex = arc.radius / std::cos(0.5 * step);
    out.push_back(arc.startPoint());
    for (int k = 0; k < parts; ++k) {
        const double a0 = arc.startAngle + step * k;
        out.push_back(arc.centre + Vec2::fromAngle(a0 + 0.5 * step, apex));
        out.push_back(arc.centre + Vec2::fromAngle(a0 + step, arc.radius));
    }
}

// A sub-range [t0, t1] of one raw element.
struct Piece {
    std::size_t element;
    double t0;
    double t1;
};

// Arc whose offset towards its centre would have a negative radius, reflected through the
// centre (radius d - r): the part of the d-neighbourhood boundary that lies beyond the centre.
std::optional<Span> invertedOffset(const Span& s, double d, double sign) {
    const auto* arc = std::get_if<ArcSpan>(&s);
    if (!arc) {
        return std::nullopt;
    }
    const double towardsCentre = arc->direction == ArcDirection::CCW ? sign : -sign;
    ArcSpan inverted = *arc;
    inverted.radius = towardsCentre * d - arc->radius;
    if (inverted.radius <= collapseRadius(*arc, d)) {
        return std::nullopt;
    }
    inverted.startAngle += kPi;
    inverted.endAngle += kPi;
    return Span{inverted};
}

// Points bucketed on a grid of a few tolerances, to find the ones within tol.distance of a
// query point without scanning them all.
class PointIndex {
   public:
    explicit PointIndex(double tolDistance) : tol_(tolDistance), cell_(std::max(4.0 * tolDistance, 1e-9)) {}

    void add(Vec2 p, std::size_t id) { map_[key(p)].push_back({p, id}); }
    void clear() { map_.clear(); }

    // Calls f(id) for every stored point within tolerance of `p`.
    template <class F>
    void forEachNear(Vec2 p, F&& f) const {
        const auto [cx, cy] = key(p);
        for (long long dy = -1; dy <= 1; ++dy) {
            for (long long dx = -1; dx <= 1; ++dx) {
                const auto it = map_.find({cx + dx, cy + dy});
                if (it == map_.end()) {
                    continue;
                }
                for (const auto& [q, id] : it->second) {
                    if (geometry::distance(q, p) <= tol_) {
                        f(id);
                    }
                }
            }
        }
    }

    // Smallest id stored within tolerance of `p`, if any.
    std::optional<std::size_t> findFirst(Vec2 p) const {
        std::optional<std::size_t> best;
        forEachNear(p, [&](std::size_t id) {
            if (!best || id < *best) {
                best = id;
            }
        });
        return best;
    }

   private:
    std::pair<long long, long long> key(Vec2 p) const {
        return {static_cast<long long>(std::floor(p.x / cell_)),
                static_cast<long long>(std::floor(p.y / cell_))};
    }

    double tol_;
    double cell_;
    std::map<std::pair<long long, long long>, std::vector<std::pair<Vec2, std::size_t>>> map_;
};

// True when `p` lies inside the closed curve `edges` (lines and arcs, in order), by counting
// crossings of the ray from `p` towards +x. Arcs are cut at their top and bottom points so that
// every piece is monotone in y and crosses the ray at most once.
bool insideRegion(Vec2 p, const Wedge& edges) {
    bool inside = false;
    auto crossLine = [&](Vec2 a, Vec2 b) {
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x > p.x) {
                inside = !inside;
            }
        }
    };
    for (const Span& s : edges) {
        if (const auto* line = std::get_if<LineSpan>(&s)) {
            crossLine(line->start, line->end);
            continue;
        }
        const auto& arc = std::get<ArcSpan>(s);
        const double a0 = arc.startAngle;
        const double a1 = arc.startAngle + arc.sweepAngle();
        const double lo = std::min(a0, a1);
        const double hi = std::max(a0, a1);
        std::vector<double> angles{lo};
        for (double k = std::ceil((lo - kPi / 2.0) / kPi); kPi / 2.0 + k * kPi < hi; k += 1.0) {
            const double theta = kPi / 2.0 + k * kPi;
            if (theta > lo) {
                angles.push_back(theta);
            }
        }
        angles.push_back(hi);
        for (std::size_t k = 0; k + 1 < angles.size(); ++k) {
            const double y0 = arc.centre.y + arc.radius * std::sin(angles[k]);
            const double y1 = arc.centre.y + arc.radius * std::sin(angles[k + 1]);
            if ((y0 > p.y) == (y1 > p.y)) {
                continue;
            }
            const double dy = p.y - arc.centre.y;
            const double half = std::sqrt(std::max(0.0, arc.radius * arc.radius - dy * dy));
            const double x =
                arc.centre.x + (std::cos(0.5 * (angles[k] + angles[k + 1])) >= 0.0 ? half : -half);
            if (x > p.x) {
                inside = !inside;
            }
        }
    }
    return inside;
}

// Moves the end of `a` and the start of `b` to the common point `j` (on both carriers), unless
// that would reverse either span or wrap an arc round to its other end; returns whether it did.
bool moveJunction(Span& a, Span& b, Vec2 j) {
    const std::optional<double> ta = extensionParam(a, j, true);
    const std::optional<double> tb = extensionParam(b, j, false);
    if (!ta || !tb || *ta <= kParamEps || *tb >= 1.0 - kParamEps) {
        return false;
    }
    a = withEnd(a, j);
    b = withStart(b, j);
    return true;
}

// What the junction snapping needs from the offset: the side, the tolerance, and a test that a
// point is at least d (to the pruning margin) from the input.
struct SnapContext {
    double sign;
    const Tolerance& tol;
    std::function<bool(Vec2)> clear;   // at least d from the input (to the snapping margin)
    std::function<bool(Vec2)> notFar;  // Round only: at most d (to the same margin) from it
    std::function<double(Vec2)> distance;  // to the input (infinity if much further than d)
    std::function<Vec2(Vec2)> nearest;     // closest input point (the point itself if none near)
    double d;
    double allowance;  // how far snapped extensions and bends may come inside d
    bool round;
};

// How far beyond d a Round offset point lies (0 for the other joins, or inside d).
double beyond(const SnapContext& ctx, Vec2 p) {
    return ctx.round ? std::max(0.0, ctx.distance(p) - ctx.d) : 0.0;
}

// True when the stretch of `carrier`'s line or circle from `from` to `to` (both on it, the
// short way round) passes ctx.clear and ctx.notFar at points no more than tol / 8 apart, ends
// included except `from` (already on the offset); with a non-negative `farSlack` a Round point
// may also lie up to that far beyond d. Checking only the far end and the middle let a
// Round extension next to a wiggly input reach 1.3e-6 mm beyond d between them: there the
// distance to the input is not monotone along the stretch (T-0017, gen3 0x9300156).
bool stretchClear(const Span* carrier, Vec2 from, Vec2 to, const SnapContext& ctx, double farSlack = -1.0) {
    const double len = geometry::distance(from, to);
    const int n = static_cast<int>(std::clamp(std::ceil(len / (0.125 * ctx.tol.distance)), 2.0, 512.0));
    const auto* arc = carrier ? std::get_if<ArcSpan>(carrier) : nullptr;
    for (int k = 1; k <= n; ++k) {
        Vec2 q = lerp(from, to, static_cast<double>(k) / n);
        if (arc && q != arc->centre) {
            q = arc->centre + (q - arc->centre).normalized() * arc->radius;
        }
        if (!ctx.clear(q) || (!ctx.notFar(q) && !(farSlack >= 0.0 && beyond(ctx, q) <= farSlack))) {
            return false;
        }
    }
    return true;
}

// Geometric snap of the junction between `a` and `b`, consecutive spans of an output chain whose
// end points agree to the tolerance. Both are trimmed or extended along their own carriers to
// where those meet, so the junction lies on both. That is only done where it keeps both spans on
// the offset: a trimmed span stays on itself, and an extension (e.g. the mitre of a corner whose
// sub-tolerance join piece was dropped, at most about tol^2 / (8d) beyond d) must be at least d
// from the input (and, for Round, not beyond d), within a few gaps of the old ends. Gaps of
// rounding size need no such care: a line end moves onto the other span's end, or an arc end
// onto the angle of a line's end. Returns false, leaving both spans unchanged, where the
// junction cannot be closed that way (e.g. a sub-tolerance S-step: the carriers meet only
// behind one span and ahead of the other).
bool snapJunction(Span& a, Span& b, const SnapContext& ctx) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    if (e == s) {
        return true;
    }
    const double gap = geometry::distance(e, s);
    const double rounding = 1e-9 * std::max({1.0, std::fabs(e.x), std::fabs(e.y)});
    const double reach = std::max(2.0 * ctx.tol.distance, 4.0 * gap);
    const Vec2 mid = (e + s) * 0.5;
    std::optional<Vec2> best;
    double bestGap = kInf;
    int bestExtensions = 3;
    // Candidates as near as each other (to the gap, or rounding) prefer the one reached by trimming:
    // where two circles cross twice close to a reversal (Round arcs about vertices almost exactly 2d
    // apart), meeting at the crossing that needs extensions leaves the arcs crossing at the other
    // one (T-0017, gen3 0x9300047).
    const double tie = std::max(gap, rounding);
    for (const Vec2 p : carrierIntersections(a, b)) {
        const std::optional<double> ta = extensionParam(a, p, true);
        const std::optional<double> tb = extensionParam(b, p, false);
        if (!ta || !tb || geometry::distance(p, e) > reach || geometry::distance(p, s) > reach) {
            continue;
        }
        const int extensions = (*ta > 1.0 ? 1 : 0) + (*tb < 0.0 ? 1 : 0);
        const double nearness = geometry::distance(p, mid);
        if (nearness > bestGap + tie || (nearness >= bestGap - tie && extensions >= bestExtensions)) {
            continue;
        }
        // Trimming keeps a span on itself. An extension beyond rounding size must stay clear of
        // the input (the dropped piece may have been a stretch of another part's offset cutting
        // across this corner) and, for Round, not stray beyond d.
        // Only a move at the level of the coordinates' last bits goes unchecked: at 1e4 mm a move
        // of 1e-9 x |x| = 1e-5 mm at a steep angle to another part of the input came 9.4e-7 mm
        // inside d (T-0017, gen3 0x9400bf7).
        auto extensionOk = [&](const Span& span, Vec2 end) {
            return geometry::distance(p, end) <= 1e-12 * std::max({1.0, std::fabs(e.x), std::fabs(e.y)}) ||
                   stretchClear(&span, end, p, ctx);
        };
        if ((*ta <= 1.0 || extensionOk(a, e)) && (*tb >= 0.0 || extensionOk(b, s))) {
            bestGap = nearness;
            bestExtensions = extensions;
            best = p;
        }
    }
    if (best && moveJunction(a, b, *best)) {
        return true;
    }
    if (gap > rounding) {
        return false;
    }
    // Moving a line end tilts the whole line, so the end that moves outwards (away from the
    // input) is moved: moving one inwards took a 185 mm line 1e-8 inside d near its end where the
    // gap (two copies of a nearly collinear chamfer, 97 mm from the origin) passed as rounding.
    const bool lineA = std::holds_alternative<LineSpan>(a);
    const bool lineB = std::holds_alternative<LineSpan>(b);
    const bool bOutwards = (e - s).dot(tangentAt(b, 0.0).perp() * ctx.sign) >= 0.0;
    if (lineB && (bOutwards || !lineA)) {
        std::get<LineSpan>(b).start = e;
    } else if (lineA) {
        std::get<LineSpan>(a).end = s;
    } else {
        // Two arcs whose circles do not meet here: only a rounding-level residual may remain
        // (exactJunction then removes it); anything larger is bridged instead.
        if (gap > 1e-12 * std::max({1.0, std::fabs(e.x), std::fabs(e.y), std::get<ArcSpan>(b).radius})) {
            return false;
        }
        b = withStart(b, e);
    }
    return true;
}

// Closes a junction snapJunction could not (a sub-tolerance step sideways, left where a
// tolerance-sized piece was dropped) by bending the last tolerance of one neighbour onto the
// other span's end point: the neighbour is cut at P, one tolerance along it from the junction,
// and the straight P -> (other end point) replaces the stretch beyond P (a short line moves as
// a whole). Only a neighbour whose end then moves away from the input (towards the offset side)
// and whose new stretch stays at least d from it is bent; the offset moves by at most the gap,
// and only within the tolerance of the junction. `extra` receives the new span between `a` and
// `b`, if any. Returns false where neither neighbour can be bent.
bool stepJunction(Span& a, Span& b, const SnapContext& ctx, std::optional<Span>& extra) {
    extra.reset();
    const double tol = ctx.tol.distance;
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    if (geometry::distance(e, s) > tol) {
        return false;
    }
    // Where `span` is cut when its `atEnd` end moves to `to` (nothing when it may not bend):
    // the parameter of P, or the far end itself for a short line.
    auto cut = [&](const Span& span, bool atEnd, Vec2 to) -> std::optional<double> {
        const double len = length(span);
        const bool line = std::holds_alternative<LineSpan>(span);
        if (len <= (line ? tol : 2.0 * tol)) {
            return std::nullopt;
        }
        const Vec2 from = atEnd ? endPoint(span) : startPoint(span);
        const Vec2 outward = tangentAt(span, atEnd ? 1.0 : 0.0).perp() * ctx.sign;
        if ((to - from).dot(outward) < 0.0) {
            return std::nullopt;
        }
        const double t = len <= 2.0 * tol ? (atEnd ? 0.0 : 1.0) : (atEnd ? 1.0 - tol / len : tol / len);
        const Vec2 p = pointAt(span, t);
        if (geometry::distance(p, to) <= tol) {
            return std::nullopt;  // the new stretch would itself be shorter than the tolerance
        }
        if (!stretchClear(nullptr, p, to, ctx)) {
            return std::nullopt;
        }
        return t;
    };
    if (const auto t = cut(a, true, s)) {
        if (*t == 0.0) {
            std::get<LineSpan>(a).end = s;
        } else {
            const Vec2 p = pointAt(a, *t);
            a = withEnd(a, p);
            extra = LineSpan{p, s};
        }
        return true;
    }
    if (const auto t = cut(b, false, e)) {
        if (*t == 1.0) {
            std::get<LineSpan>(b).start = e;
        } else {
            const Vec2 p = pointAt(b, *t);
            b = withStart(b, p);
            extra = LineSpan{e, p};
        }
        return true;
    }
    return false;
}

// Makes the junction of `a` and `b` bit-identical, changing only a's end or b's start. A line
// end simply takes the other span's computed end point. Between two arcs, whose end points are
// computed from centre, radius and angles (ADR-0005), b's centre is nudged by a few ulps until
// its computed start equals a's computed end; that can fail where the centre's coordinates are
// much larger than the point's, leaving a rounding-level residual (far below 1e-12 mm).
void exactJunction(Span& a, Span& b) {
    if (auto* line = std::get_if<LineSpan>(&a)) {
        line->end = startPoint(b);
        return;
    }
    const Vec2 j = endPoint(a);
    if (auto* line = std::get_if<LineSpan>(&b)) {
        line->start = j;
        return;
    }
    auto& arc = std::get<ArcSpan>(b);
    if (arc.startPoint() == j) {
        return;
    }
    const double scale = std::max({1.0, std::fabs(j.x), std::fabs(j.y), arc.radius});
    if (geometry::distance(arc.startPoint(), j) > 1e-12 * scale) {
        return;  // not a rounding-level residual: leave the arc where it is
    }
    ArcSpan moved = arc;
    moved.centre = j - Vec2::fromAngle(arc.angleAt(0.0), arc.radius);
    for (int iter = 0; iter < 8 && moved.startPoint() != j; ++iter) {
        const Vec2 got = moved.startPoint();
        if (got.x != j.x) {
            moved.centre.x = std::nextafter(moved.centre.x, got.x < j.x ? kInf : -kInf);
        }
        if (got.y != j.y) {
            moved.centre.y = std::nextafter(moved.centre.y, got.y < j.y ? kInf : -kInf);
        }
    }
    if (moved.startPoint() == j) {
        arc = moved;
    }
}

// The span that closes a junction nothing else could: a straight line from a's end to b's
// start. Where that line would leave the offset (beyond d for Round: a needle tip of two offset
// lines whose rounded end, an arc about an input point, was not among the kept pieces), the
// missing arc is put in instead if it can be: about the input point nearest one end, through that
// end, with the other span trimmed or extended onto its circle like any snapped junction, and
// every point of it passing the distance checks (T-0017, gen3 0x93010bf: the line ran 1.3e-5 mm
// beyond d across the tip). `a` or `b` may be changed.
std::optional<Span> gapBridge(Span& a, Span& b, const SnapContext& ctx) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    const LineSpan line{e, s};
    {
        // The straight bridge as documented: it may come inside d by its sagitta against the
        // round neighbourhood (at most tol^2 / (8d)), and for Round it must not go beyond d.
        const double tol = ctx.tol.distance;
        const double inside = std::max(ctx.allowance, tol * tol / (8.0 * ctx.d));
        const double len = geometry::distance(e, s);
        const int n = static_cast<int>(std::clamp(std::ceil(len / (0.125 * tol)), 2.0, 512.0));
        bool ok = true;
        for (int k = 1; ok && k < n; ++k) {
            const Vec2 q = lerp(e, s, static_cast<double>(k) / n);
            const double slack = 1e-12 * std::max({1.0, ctx.d, std::fabs(q.x), std::fabs(q.y)});
            const double dq = ctx.distance(q);
            ok = dq >= ctx.d - inside - slack && (!ctx.round || dq <= ctx.d + ctx.allowance + slack);
        }
        if (ok) {
            return Span{line};
        }
    }
    const Vec2 tin = tangentAt(a, 1.0);
    const Vec2 tout = tangentAt(b, 0.0);
    auto arcOk = [&](const ArcSpan& arc) {
        if (std::fabs(arc.sweepAngle()) > kPi / 2.0 || isDegenerate(Span{arc}, ctx.tol)) {
            return false;
        }
        const int n =
            static_cast<int>(std::clamp(std::ceil(arc.length() / (0.125 * ctx.tol.distance)), 4.0, 512.0));
        for (int k = 1; k < n; ++k) {
            const Vec2 q = arc.pointAt(static_cast<double>(k) / n);
            if (!ctx.clear(q) || !ctx.notFar(q)) {
                return false;
            }
        }
        return true;
    };
    const double gap = geometry::distance(e, s);
    const double reach = std::max(2.0 * ctx.tol.distance, 4.0 * gap);
    for (const bool aboutEnd : {false, true}) {
        // An arc about the input point nearest b's start, ending there, meeting a's carrier; else
        // one about the point nearest a's end, starting there, meeting b's carrier.
        const Vec2 through = aboutEnd ? e : s;
        const Vec2 v = ctx.nearest(through);
        const double r = geometry::distance(through, v);
        if (r <= 1e-9 * std::max({1.0, std::fabs(v.x), std::fabs(v.y)})) {
            continue;
        }
        const Span& other = aboutEnd ? b : a;
        const Vec2 otherEnd = aboutEnd ? s : e;
        const Vec2 t = aboutEnd ? tin : tout;
        const ArcDirection dir = (through - v).cross(t) > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
        std::optional<ArcSpan> best;
        std::optional<Span> moved;
        double bestDist = kInf;
        for (const Vec2 y : carrierIntersections(other, Span{ArcSpan::fullCircle(v, r)})) {
            const std::optional<double> ty = extensionParam(other, y, !aboutEnd);
            const double dy = geometry::distance(y, otherEnd);
            if (!ty || dy > reach || dy >= bestDist ||
                (aboutEnd ? *ty >= 1.0 - kParamEps : *ty <= kParamEps)) {
                continue;
            }
            const bool extends = aboutEnd ? *ty < 0.0 : *ty > 1.0;
            if (extends && !stretchClear(&other, otherEnd, y, ctx)) {
                continue;
            }
            const ArcSpan arc =
                aboutEnd ? ArcSpan::fromCentre(v, e, y, dir) : ArcSpan::fromCentre(v, y, s, dir);
            const Span changed = aboutEnd ? withStart(other, y) : withEnd(other, y);
            const Vec2 before = aboutEnd ? tin : tangentAt(changed, 1.0);
            const Vec2 after = aboutEnd ? tangentAt(changed, 0.0) : tout;
            if (isDegenerate(changed, ctx.tol) || arc.tangentAt(0.0).dot(before) <= 0.0 ||
                arc.tangentAt(1.0).dot(after) <= 0.0 || !arcOk(arc)) {
                continue;
            }
            best = arc;
            moved = changed;
            bestDist = dy;
        }
        if (best) {
            (aboutEnd ? b : a) = *moved;
            return Span{*best};
        }
    }
    return std::nullopt;
}

// Snaps the junctions of an output chain (and its closing junction when `closed`) so that
// consecutive spans share their end point exactly. `dropped[k]` holds the sub-tolerance pieces
// left out before span k (`dropped[0]`: those across the closing junction of a closed chain).
// Each junction is first snapped geometrically (snapJunction, else stepJunction); where both
// fail the dropped pieces are put back (they are the true offset there), and where there are
// none a straight bridge spans the gap (gapBridge). Where even that cannot stay on the offset (a
// Round needle tip whose rounded end is missing, T-0017) the chain is cut there instead: the
// result then holds several chains, open, each continuous. Then the junctions are made
// bit-identical in order (exactJunction), which never disturbs an end matched before; a closed
// chain is walked from a line if it has one, so its closing junction is exact too.
std::vector<std::vector<Span>> snapChain(const std::vector<Span>& spans,
                                         const std::vector<std::vector<Span>>& dropped, bool closed,
                                         const SnapContext& ctx, bool& stillClosed) {
    std::vector<std::vector<Span>> result;
    stillClosed = false;
    if (spans.empty()) {
        return result;
    }
    // A cut between two spans is marked by a Bezier span, which never occurs in an offset chain.
    const Span cut = BezierSpan{};
    auto isCut = [](const Span& sp) { return std::holds_alternative<BezierSpan>(sp); };
    // Closes the junction a -> b; `between` receives any spans that go between them.
    auto join = [&](Span& a, Span& b, const std::vector<Span>& before, std::vector<Span>& between) {
        std::optional<Span> extra;
        if (snapJunction(a, b, ctx)) {
            return;
        }
        if (stepJunction(a, b, ctx, extra)) {
            if (extra) {
                between.push_back(*extra);
            }
            return;
        }
        std::vector<Span> chain{a};
        chain.insert(chain.end(), before.begin(), before.end());
        chain.push_back(b);
        std::vector<Span> joined{chain.front()};
        for (std::size_t k = 1; k < chain.size(); ++k) {
            if (!snapJunction(joined.back(), chain[k], ctx)) {
                const std::optional<Span> bridge = gapBridge(joined.back(), chain[k], ctx);
                joined.push_back(bridge ? *bridge : cut);
            }
            joined.push_back(chain[k]);
        }
        a = joined.front();
        b = joined.back();
        between.assign(joined.begin() + 1, joined.end() - 1);
    };
    std::vector<Span> body = spans;
    std::vector<Span> closing;
    if (closed && body.size() > 1) {
        // The closing junction first, so that every later junction sees both ends of its spans
        // in their final place (no span can then be trimmed past its own other end).
        join(body.back(), body.front(), dropped[0], closing);
    }
    std::vector<Span> out{body.front()};
    for (std::size_t k = 1; k < body.size(); ++k) {
        std::vector<Span> between;
        join(out.back(), body[k], dropped[k], between);
        out.insert(out.end(), between.begin(), between.end());
        out.push_back(body[k]);
    }
    out.insert(out.end(), closing.begin(), closing.end());
    // A closed chain that was cut becomes open, starting after its first cut.
    const auto firstCut = std::find_if(out.begin(), out.end(), isCut);
    const bool wasCut = firstCut != out.end();
    if (wasCut && closed) {
        std::rotate(out.begin(), firstCut + 1, out.end());
    }
    std::vector<std::vector<Span>> pieces(1);
    for (const Span& sp : out) {
        if (isCut(sp)) {
            pieces.emplace_back();
        } else {
            pieces.back().push_back(sp);
        }
    }
    stillClosed = closed && !wasCut;
    for (std::vector<Span>& piece : pieces) {
        const std::size_t n = piece.size();
        if (n == 0) {
            continue;
        }
        std::size_t first = 0;
        if (stillClosed) {
            const auto line = std::find_if(piece.begin(), piece.end(), [](const Span& sp) {
                return std::holds_alternative<LineSpan>(sp);
            });
            first = line == piece.end() ? 0 : static_cast<std::size_t>(line - piece.begin());
        }
        const std::size_t junctions = stillClosed ? n : n - 1;
        for (std::size_t k = 0; k < junctions && n > 1; ++k) {
            const std::size_t i = (first + k) % n;
            exactJunction(piece[i], piece[(i + 1) % n]);
        }
        result.push_back(std::move(piece));
    }
    return result;
}

// True when the short span `s` steps back between `prev` and `next`: no longer than the
// tolerance, and its chord runs against both neighbours while they go the same general way, or it
// sets off straight back along `prev` (a retrace), or `next` sets off straight back along it.
bool stepsBack(const Span& prev, const Span& s, const Span& next, const Tolerance& tol) {
    if (!isDegenerate(s, tol)) {
        return false;
    }
    const Vec2 chord = endPoint(s) - startPoint(s);
    const Vec2 tin = tangentAt(prev, 1.0);
    const Vec2 tout = tangentAt(next, 0.0);
    if (tin.dot(tout) > 0.0 && chord.dot(tin) < 0.0 && chord.dot(tout) < 0.0) {
        return true;
    }
    return tin.dot(tangentAt(s, 0.0)) < -0.9999 || tangentAt(s, 1.0).dot(tout) < -0.9999;
}

// Safety net after snapChain (T-0006): where three or more offset carriers nearly meet within the
// tolerance (e.g. lines a few hundredths of a radian apart crossing in a sub-tolerance triangle),
// the pieces between their crossings can leave a span shorter than the tolerance that steps back
// or sideways against the direction of travel. Such a span is removed, with a short neighbour
// if need be, and the spans on either side are joined: by snapJunction (trimmed or extended along
// their carriers to where they meet, extensions at least d from the input and, for Round, not
// beyond it), else by a straight bridge (at most 2 x tol long) if that runs forwards with both and
// stays clear in the same way (the two sides on one carrier, say), possibly taking over the first
// or last tolerance of one of them. Two arcs on nearly one circle on either side are merged into
// one arc instead (T-0017). Where none of this works the chain is left as it is. Chains of any
// length from three spans are checked (T-0017: [arc, backward line, arc] was skipped). New
// junctions are made exact like the others.
void removeBackSteps(std::vector<Span>& spans, bool closed, const SnapContext& ctx) {
    auto bridgeOk = [&](const Span& a, const Span& b) {
        const Vec2 e = endPoint(a);
        const Vec2 s = startPoint(b);
        const Vec2 chord = s - e;
        if (geometry::distance(e, s) > 2.0 * ctx.tol.distance || chord.dot(tangentAt(a, 1.0)) <= 0.0 ||
            chord.dot(tangentAt(b, 0.0)) <= 0.0) {
            return false;
        }
        // Where the input has a gap a Round join next to it lies up to about that gap beyond d
        // (see offset.hpp); a bridge from it may stay that far out, but no farther (T-0017, gen3
        // 0x930046a: a 1.4e-5 mm input gap left a 3.2e-6 mm step back that no bridge could close).
        return stretchClear(nullptr, e, s, ctx, std::max(beyond(ctx, e), beyond(ctx, s)));
    };
    for (int pass = 0; pass < 4; ++pass) {
        bool changed = false;
        for (std::size_t i = 0; i < spans.size() && spans.size() >= 3; ++i) {
            const std::size_t n = spans.size();
            if (!closed && (i == 0 || i + 1 == n)) {
                continue;
            }
            if (!stepsBack(spans[(i + n - 1) % n], spans[i], spans[(i + 1) % n], ctx.tol)) {
                continue;
            }
            // Candidate windows (first, last) of spans to remove, all no longer than the tolerance:
            // the span itself, then with its previous or next neighbour.
            std::vector<std::pair<std::size_t, std::size_t>> windows{{i, i}};
            if (closed || i >= 2) {
                windows.emplace_back((i + n - 1) % n, i);
            }
            if (closed || i + 2 < n) {
                windows.emplace_back(i, (i + 1) % n);
            }
            for (const auto& [first, last] : windows) {
                if (!isDegenerate(spans[first], ctx.tol) || !isDegenerate(spans[last], ctx.tol)) {
                    continue;
                }
                const std::size_t left = (first + n - 1) % n;
                const std::size_t right = (last + 1) % n;
                if (left == right || left == last || right == first) {
                    continue;
                }
                // What goes in each slot of the chain afterwards (removed spans leave theirs empty).
                std::vector<std::vector<Span>> slots(n);
                for (std::size_t k = 0; k < n; ++k) {
                    slots[k].push_back(spans[k]);
                }
                slots[first].clear();
                slots[last].clear();
                bool joined = false;
                // Two arcs on nearly the same circle (e.g. the offsets of an input arc split in two
                // with ~1e-8 mm of noise, whose circles do not meet) become one arc on the circle of
                // either, where every point of it stays clear like a snapped extension; the
                // junction its moved end then has with the next (previous) span is snapped again.
                const auto* arcA = std::get_if<ArcSpan>(&spans[left]);
                const auto* arcB = std::get_if<ArcSpan>(&spans[right]);
                if (arcA && arcB && arcA->direction == arcB->direction &&
                    geometry::distance(arcA->centre, arcB->centre) + std::fabs(arcA->radius - arcB->radius) <=
                        0.5 * ctx.tol.distance) {
                    const double want = std::fabs(arcA->sweepAngle()) + std::fabs(arcB->sweepAngle());
                    for (const bool onA : {true, false}) {
                        ArcSpan merged = onA ? *arcA : *arcB;
                        if (onA) {
                            merged.endAngle = (arcB->endPoint() - merged.centre).angle();
                        } else {
                            merged.startAngle = (arcA->startPoint() - merged.centre).angle();
                        }
                        // The far neighbour of the end that moved, if there is one.
                        const std::size_t outer = onA ? (right + 1) % n : (left + n - 1) % n;
                        const bool hasFar =
                            closed ? outer != left && outer != right && outer != first && outer != last
                                   : (onA ? right + 1 < n : left > 0);
                        if (closed && !hasFar) {
                            continue;
                        }
                        bool ok = std::fabs(std::fabs(merged.sweepAngle()) - want) <=
                                  4.0 * ctx.tol.distance / merged.radius;
                        const int samples = static_cast<int>(
                            std::clamp(std::ceil(merged.length() / (0.125 * ctx.tol.distance)), 16.0, 512.0));
                        for (int k = 0; ok && k <= samples; ++k) {
                            const Vec2 q = merged.pointAt(static_cast<double>(k) / samples);
                            ok = ctx.clear(q) && ctx.notFar(q);
                        }
                        Span m = merged;
                        Span neighbour = hasFar ? spans[outer] : m;
                        if (ok && hasFar) {
                            ok = onA ? snapJunction(m, neighbour, ctx) : snapJunction(neighbour, m, ctx);
                            ok = ok && !isDegenerate(neighbour, ctx.tol);
                        }
                        if (!ok || isDegenerate(m, ctx.tol)) {
                            continue;
                        }
                        slots[left].assign(1, m);
                        slots[right].clear();
                        if (hasFar) {
                            slots[outer].assign(1, neighbour);
                        }
                        joined = true;
                        break;
                    }
                }
                if (!joined) {
                    Span a = spans[left];
                    Span b = spans[right];
                    std::optional<Span> bridge;
                    if (!snapJunction(a, b, ctx) || isDegenerate(a, ctx.tol) || isDegenerate(b, ctx.tol)) {
                        a = spans[left];
                        b = spans[right];
                        // A straight bridge between them, else one that also takes over the first
                        // tolerance of `b` or the last of `a` (a sub-tolerance notch of the exact
                        // offset that a direct bridge would cut inside d).
                        // The cut is tried at a tolerance, then shorter (a chord across a small arc
                        // dips inside d by its sagitta, so a tolerance-long one may not pass).
                        const double tol = ctx.tol.distance;
                        bool bridged = bridgeOk(a, b);
                        for (double cut = tol; !bridged && cut >= 0.1 * tol; cut *= 0.5) {
                            if (length(b) > 2.0 * cut) {
                                const Span cutB = withStart(b, pointAt(b, cut / length(b)));
                                if (bridgeOk(a, cutB)) {
                                    b = cutB;
                                    bridged = true;
                                    break;
                                }
                            }
                            if (length(a) > 2.0 * cut) {
                                const Span cutA = withEnd(a, pointAt(a, 1.0 - cut / length(a)));
                                if (bridgeOk(cutA, b)) {
                                    a = cutA;
                                    bridged = true;
                                }
                            }
                        }
                        if (!bridged) {
                            continue;
                        }
                        bridge = LineSpan{endPoint(a), startPoint(b)};
                    }
                    slots[left].assign(1, a);
                    slots[right].assign(1, b);
                    if (bridge) {
                        slots[last].push_back(*bridge);  // in place of the removed spans
                    }
                }
                std::vector<Span> next;
                next.reserve(n);
                for (const std::vector<Span>& slot : slots) {
                    next.insert(next.end(), slot.begin(), slot.end());
                }
                spans.swap(next);
                const std::size_t m = spans.size();
                for (std::size_t k = 0; k + 1 < m || (closed && k < m); ++k) {
                    exactJunction(spans[k], spans[(k + 1) % m]);
                }
                changed = true;
                break;
            }
        }
        if (!changed) {
            return;
        }
    }
}

// Closes the gap between consecutive spans `a` and `b` left by removeRetraces (at most a few
// rounding units of the coordinates): snapped like any junction, else a straight bridge that stays
// clear of the input. Returns whether it closed; `bridge` receives the bridge, if one is used.
bool closeSmallGap(Span& a, Span& b, const SnapContext& ctx, std::optional<Span>& bridge) {
    bridge.reset();
    Span x = a;
    Span y = b;
    if (snapJunction(x, y, ctx) && !isDegenerate(x, ctx.tol) && !isDegenerate(y, ctx.tol)) {
        a = x;
        b = y;
        return true;
    }
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    if (!stretchClear(nullptr, e, s, ctx)) {
        return false;
    }
    bridge = LineSpan{e, s};
    return true;
}

// A span that sets off straight back along its predecessor (or that its successor sets off back
// along) and lies wholly within a rounding unit of the coordinates (2e-9 x max(1, d, |x|, |y|))
// of it (two arcs also on one circle to that unit): a needle tip so thin that, at those
// coordinates, the two sides are the same stretch traversed twice (T-0017, 0x840117a: 2.9e-3 mm
// at 3.6e-6 mm sideways, ~1e4 mm from the origin). The span is removed, its neighbour cut back to
// where the span's far end lies alongside it, and the new junction closed (closeSmallGap). The
// geometry moves by at most that unit.
void removeRetraces(std::vector<Span>& spans, bool closed, const SnapContext& ctx, double d) {
    for (int pass = 0; pass < 8; ++pass) {
        bool changed = false;
        const std::size_t n = spans.size();
        if (n < (closed ? 3u : 2u)) {
            return;
        }
        for (std::size_t k = closed ? 0 : 1; k < n && !changed; ++k) {
            const std::size_t ia = (k + n - 1) % n;
            const std::size_t ib = k;
            const Span& a = spans[ia];
            const Span& b = spans[ib];
            if (tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) > -0.99) {
                continue;
            }
            const Vec2 j = startPoint(b);
            const double unit = 2e-9 * std::max({1.0, d, std::fabs(j.x), std::fabs(j.y)});
            auto alongside = [&](const Span& x, const Span& y) {
                // Arcs only on one circle to that unit: two different circles crossing at a shallow
                // angle make a real (if thin) needle, e.g. 0.3 mm long and 5e-5 mm wide.
                const auto* ax = std::get_if<ArcSpan>(&x);
                const auto* ay = std::get_if<ArcSpan>(&y);
                if (ax && ay &&
                    geometry::distance(ax->centre, ay->centre) + std::fabs(ax->radius - ay->radius) > unit) {
                    return false;
                }
                for (const double f : {0.25, 0.5, 0.75, 1.0}) {
                    if (distanceToSpan(pointAt(x, f), y) > unit) {
                        return false;
                    }
                }
                return true;
            };
            // Remove b (cutting a back) or a (cutting b forward); the cut neighbour must keep a
            // positive length, and the span beyond the removed one must exist and differ from it.
            for (const bool dropB : {true, false}) {
                const Span& gone = dropB ? b : a;
                const Span& kept = dropB ? a : b;
                if (!alongside(gone, kept)) {
                    continue;
                }
                const std::size_t ik = dropB ? ia : ib;
                const std::size_t ig = dropB ? ib : ia;
                const bool hasBeyond = closed || (dropB ? ig + 1 < n : ig > 0);
                const std::size_t beyond = dropB ? (ig + 1) % n : (ig + n - 1) % n;
                if (hasBeyond && beyond == ik) {
                    continue;
                }
                const double t =
                    std::clamp(carrierParam(kept, dropB ? endPoint(gone) : startPoint(gone)), 0.0, 1.0);
                Span cut = dropB ? subSpan(kept, 0.0, t) : subSpan(kept, t, 1.0);
                if ((dropB ? t : 1.0 - t) <= kParamEps || isDegenerate(cut, ctx.tol)) {
                    continue;
                }
                std::vector<std::vector<Span>> slots(n);
                for (std::size_t q = 0; q < n; ++q) {
                    slots[q].push_back(spans[q]);
                }
                slots[ig].clear();
                if (hasBeyond) {
                    Span other = spans[beyond];
                    std::optional<Span> bridge;
                    const bool ok = dropB ? closeSmallGap(cut, other, ctx, bridge)
                                          : closeSmallGap(other, cut, ctx, bridge);
                    if (!ok) {
                        continue;
                    }
                    slots[beyond].assign(1, other);
                    if (bridge) {
                        slots[ig].push_back(*bridge);
                    }
                }
                slots[ik].assign(1, cut);
                std::vector<Span> next;
                for (const std::vector<Span>& slot : slots) {
                    next.insert(next.end(), slot.begin(), slot.end());
                }
                spans.swap(next);
                const std::size_t m = spans.size();
                for (std::size_t q = 0; q + 1 < m || (closed && q < m); ++q) {
                    exactJunction(spans[q], spans[(q + 1) % m]);
                }
                changed = true;
                break;
            }
        }
        if (!changed) {
            return;
        }
    }
}

// Consecutive arcs of a chain on one circle to rounding (centres and radii within
// 1e-12 x max(1, |x|, |y|, r), so the junctions stay exact) and running the same way become one arc on the
// first one's circle: a split there is only an artefact of the pieces (e.g. offsets of an input arc cut in
// three, with noise), and the rounding between their circles can make the next arc start a hair behind the
// previous one's end (T-0017, gen3 0x94010e5: a full circle as three arcs). A closed chain that
// is then one arc going all the way round becomes a full circle.
void mergeCoCircularArcs(std::vector<Span>& spans, bool closed) {
    auto same = [](const Span& a, const Span& b) {
        const auto* x = std::get_if<ArcSpan>(&a);
        const auto* y = std::get_if<ArcSpan>(&b);
        if (!x || !y || x->direction != y->direction) {
            return false;
        }
        const double scale =
            1e-12 * std::max({1.0, std::fabs(x->centre.x), std::fabs(x->centre.y), x->radius});
        return geometry::distance(x->centre, y->centre) + std::fabs(x->radius - y->radius) <= scale;
    };
    bool changed = false;
    for (std::size_t k = 0; k + 1 < spans.size();) {
        if (!same(spans[k], spans[k + 1])) {
            ++k;
            continue;
        }
        auto& a = std::get<ArcSpan>(spans[k]);
        const auto& b = std::get<ArcSpan>(spans[k + 1]);
        const double sweep = std::fabs(a.sweepAngle()) + std::fabs(b.sweepAngle());
        if (sweep >= kTwoPi - 1e-9) {
            ++k;  // would wrap round: only a closed chain's last merge may close the circle
            continue;
        }
        ArcSpan merged = a;
        merged.endAngle = b.endAngle;  // the same circle to rounding: b's end, as b states it
        if (std::fabs(std::fabs(merged.sweepAngle()) - sweep) > 1e-9) {
            ++k;
            continue;
        }
        a = merged;
        spans.erase(spans.begin() + static_cast<std::ptrdiff_t>(k) + 1);
        changed = true;
    }
    if (closed && spans.size() == 2 && same(spans[0], spans[1])) {
        const ArcSpan a = std::get<ArcSpan>(spans[0]);
        spans.assign(1, Span{ArcSpan{a.centre, a.radius, a.startAngle, a.startAngle, a.direction}});
        changed = true;
    }
    if (changed) {
        const std::size_t m = spans.size();
        for (std::size_t q = 0; q + 1 < m || (closed && q < m && m > 1); ++q) {
            exactJunction(spans[q], spans[(q + 1) % m]);
        }
    }
}

// A closed chain that encloses no width a tool could follow and would only retrace itself,
// dropped like the opposite pairs of mergeOnCarriers: two spans between the same two points that
// never part by more than half the tolerance (two lines the snapping closed up, or a lens of two
// arcs on nearly one circle running opposite ways, T-0017 0x82004c8), or any loop whose mean
// width, 2 x area / perimeter, is below tol / 8 and whose every point lies within tol / 8 of
// another of its spans (a lens of two nearly co-circular opposite arcs, gen3 0x9300698: 1.45 mm
// long, ~1e-5 mm wide; what is left between two nearly tangent circles once the sliver along
// them is dropped, 0x9100821: 0.017 mm round, crossing itself).
bool isSliverLoop(const Contour& c, const Tolerance& tol) {
    if (!c.closed || c.spans.size() < 2) {
        return false;
    }
    const double half = 0.5 * tol.distance;
    if (c.spans.size() == 2) {
        bool apart = false;
        for (int k = 1; !apart && k < 16; ++k) {
            apart = distanceToSpan(pointAt(c.spans[0], k / 16.0), c.spans[1]) > half ||
                    distanceToSpan(pointAt(c.spans[1], k / 16.0), c.spans[0]) > half;
        }
        if (!apart) {
            return true;
        }
    }
    double perimeter = 0.0;
    for (const Span& sp : c.spans) {
        perimeter += length(sp);
    }
    const Vec2 o = startPoint(c.spans.front());
    double twiceArea = 0.0;
    for (const Span& sp : c.spans) {
        for (int k = 0; k < 16; ++k) {
            twiceArea += (pointAt(sp, k / 16.0) - o).cross(pointAt(sp, (k + 1) / 16.0) - o);
        }
    }
    if (perimeter <= 0.0 || std::fabs(twiceArea) / perimeter > 0.125 * tol.distance) {
        return false;
    }
    for (std::size_t i = 0; i < c.spans.size(); ++i) {
        for (int k = 1; k < 16; ++k) {
            const Vec2 q = pointAt(c.spans[i], k / 16.0);
            double nearest = kInf;
            for (std::size_t j = 0; j < c.spans.size(); ++j) {
                if (j != i) {
                    nearest = std::min(nearest, distanceToSpan(q, c.spans[j]));
                }
            }
            if (nearest > 0.125 * tol.distance) {
                return false;
            }
        }
    }
    return true;
}

// The raw offset chain of one side, with each element's position along the input path (span
// index + fraction at its start and end) for ordering the output chains.
struct RawChain {
    std::vector<Span> spans;
    std::vector<double> posA;
    std::vector<double> posB;
};

// Raw offset of one side, in input order: every element offset on its own, chained with joins
// (outside corners) and straight connectors (inside corners). An arc whose offset radius would
// be negative is inverted (radius d - r on the far side of its centre); it is pruned later but
// keeps the corner geometry, so the joins about its end points survive.
RawChain buildRaw(const std::vector<Span>& input, double d, double sign, JoinType join, double mitreLimit,
                  const Tolerance& tol, bool closed, std::vector<Wedge>& wedges) {
    RawChain rc;
    std::vector<Span>& raw = rc.spans;
    auto record = [&](double a, double b) {
        while (rc.posA.size() < raw.size()) {
            rc.posA.push_back(a);
            rc.posB.push_back(b);
        }
    };
    for (std::size_t i = 0; i < input.size(); ++i) {
        std::optional<Span> next = offsetSpan(input[i], d, sign);
        if (!next) {
            next = invertedOffset(input[i], d, sign);
            if (!next) {
                continue;  // offsets to (nearly) the arc's centre point: nothing to add
            }
        }
        const double at = static_cast<double>(i);
        if (!raw.empty()) {
            connect(raw, *next, startPoint(input[i]), tangentAt(input[i - 1], 1.0), tangentAt(input[i], 0.0),
                    d, sign, join, mitreLimit, tol, wedges);
            record(at, at);  // joins and connectors sit at the vertex
        }
        raw.push_back(*next);
        record(at, at + 1.0);
    }
    if (closed && raw.size() > 1) {
        Span first = raw.front();
        connect(raw, first, startPoint(input.front()), tangentAt(input.back(), 1.0),
                tangentAt(input.front(), 0.0), d, sign, join, mitreLimit, tol, wedges);
        raw.front() = first;
        const double at = static_cast<double>(input.size());
        record(at, at);
    }
    return rc;
}

// Kept pieces of raw elements that lie on one line or circle, merged into one boundary piece per
// stretch. Overlapping elements (the mitre extensions of a slot narrower than 2d, the chamfers
// or mitres of neighbouring peaks of a steep zig-zag, the inside-corner connectors of a dense
// zig-zag at a large d) keep copies of the same stretch. A copy running the same way is the
// same boundary piece; two copies running opposite ways bound a region of zero width inside the
// offset shape and both go. Matching copies by their end points failed where the copies were
// cut differently: each element computes its own boundary crossings, and a nearly tangent one
// (the bottom of a vertex circle touching a chamfer line) lands up to ~1e-7 mm apart on two
// copies, and pieces shorter than the tolerance could not be matched to the tolerance at all.
// Both copies of such an overlap survived and the chain stepped back over it (T-0006).
//
// So every group of elements on one carrier (pairs from `overlapping`, joined transitively) is
// swept along a common coordinate: distance along the line, or angle round the circle. All
// piece ends of the group are merged where they agree to rounding (1e-9 x max(1, d, |x|, |y|),
// or that over the radius for angles), and each elementary stretch between consecutive ends is
// kept once if more copies cover it one way than the other, on the copy outermost along it (see
// the sweep below), and dropped otherwise. Every piece end stays a piece end, so the stitching
// can still turn at any point where another piece starts or ends. Cost O(P log P) per group of P
// pieces, plus per stretch a pass over the copies covering it for each hand-over; pieces of elements outside
// every group are untouched. The result is ordered by element and parameter, like the input.
std::vector<Piece> mergeOnCarriers(const std::vector<Span>& raw, const std::vector<Piece>& kept,
                                   const std::vector<std::pair<std::size_t, std::size_t>>& overlapping,
                                   double d, double sign, double lateral, double opposite) {
    const std::size_t m = raw.size();
    std::vector<std::size_t> parent(m);
    for (std::size_t i = 0; i < m; ++i) {
        parent[i] = i;
    }
    auto find = [&](std::size_t i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    };
    std::vector<bool> shared(m, false);
    auto unite = [&](std::size_t i, std::size_t j) {
        shared[i] = true;
        shared[j] = true;
        const std::size_t a = find(i);
        const std::size_t b = find(j);
        if (a != b) {
            parent[std::max(a, b)] = std::min(a, b);  // the root is the group's lowest element
        }
    };
    for (const auto& [i, j] : overlapping) {
        unite(i, j);
    }
    // Nearly the same carrier: kept pieces of two elements that overlap along a common stretch
    // and stay within `lateral` of each other's line or circle there. Raw elements whose carriers
    // differ by more than rounding but less than the tolerance (vertex noise of 1e-9..1e-5 mm in
    // DXF or computed input, coordinates far from the origin) meet at ill-conditioned crossings
    // that do not cut their copies consistently either; both copies are boundary (each passed the
    // distance pruning), so keeping the outer one per stretch keeps the offset at least d from the
    // input and moves it by at most `lateral`. Only nearly parallel elements qualify, by a real
    // angle limit (kGroupSin) as well as by their sideways drift over their length: a short piece
    // crossing a carrier at 11-20 degrees (T-0017 R4) is a different boundary piece, not a copy.
    // Pieces that merely continue each other end to end share no stretch and are not grouped, and
    // an element joins a group only if it is also nearly parallel to the group's first element,
    // so a gently turning run of short elements cannot chain into one group (T-0017).
    constexpr double kGroupSin = 1e-3;
    std::vector<Span> pieceSpans;
    std::vector<Box2> pieceBoxes;
    pieceSpans.reserve(kept.size());
    pieceBoxes.reserve(kept.size());
    for (const Piece& p : kept) {
        pieceSpans.push_back(subSpan(raw[p.element], p.t0, p.t1));
        pieceBoxes.push_back(bounds(pieceSpans.back()).inflated(lateral));
    }
    auto onCarrierOf = [&](Vec2 q, const Span& s, double band) {
        if (const auto* line = std::get_if<LineSpan>(&s)) {
            const Vec2 u = line->end - line->start;
            return std::fabs(u.cross(q - line->start)) <= band * u.length();
        }
        const auto& arc = std::get<ArcSpan>(s);
        return std::fabs(geometry::distance(q, arc.centre) - arc.radius) <= band;
    };
    // Elements nearly parallel or antiparallel (lines) or nearly concentric (arcs); with a
    // non-negative `band`, also within it sideways over their whole length (a line's drift, an
    // arc's centre and radius).
    auto nearlyParallel = [&](const Span& ea, const Span& eb, double angleLimit, double band) {
        if (ea.index() != eb.index()) {
            return false;
        }
        if (const auto* arcA = std::get_if<ArcSpan>(&ea)) {
            const auto& arcB = std::get<ArcSpan>(eb);
            const double off = geometry::distance(arcA->centre, arcB.centre);
            return off <= angleLimit * std::min(arcA->radius, arcB.radius) &&
                   (band < 0.0 || (off <= 2.0 * band && std::fabs(arcA->radius - arcB.radius) <= band));
        }
        const double s = std::fabs(tangentAt(ea, 0.0).cross(tangentAt(eb, 0.0)));
        return s <= angleLimit && (band < 0.0 || s * std::max(length(ea), length(eb)) <= band);
    };
    // Some point of `y` (an end or its middle) lies strictly inside `x`'s extent, measured along
    // x's carrier: the two share a stretch of positive length.
    auto reachesInto = [&](const Span& x, const Span& y) {
        const double len = length(x);
        if (len <= 0.0) {
            return false;
        }
        for (const double f : {0.0, 0.5, 1.0}) {
            const Vec2 q = pointAt(y, f);
            const double margin = 1e-9 * std::max({1.0, std::fabs(q.x), std::fabs(q.y)}) / len;
            const double t = carrierParam(x, q);
            if (t > margin && t < 1.0 - margin) {
                return true;
            }
        }
        return false;
    };
    // Copies running the same way are grouped within `lateral`. Copies running opposite ways
    // bound a sliver that the merge drops (as many copies run each way), so their neighbours are
    // then joined sideways across its width: only within `opposite` (the snapping allowance) is
    // that a rounding repair. Wider opposite pairs are a real thin feature, e.g. a needle between
    // two strands 1e-4 mm apart (T-0017 R6), and stay apart.
    auto nearCarrier = [&](std::size_t a, std::size_t b) {
        const Span& sa = pieceSpans[a];
        const Span& sb = pieceSpans[b];
        const Span& ea = raw[kept[a].element];
        const Span& eb = raw[kept[b].element];
        if (ea.index() != eb.index()) {
            return false;
        }
        const bool same = std::holds_alternative<LineSpan>(ea)
                              ? tangentAt(ea, 0.0).dot(tangentAt(eb, 0.0)) > 0.0
                              : std::get<ArcSpan>(ea).direction == std::get<ArcSpan>(eb).direction;
        const double band = same ? lateral : opposite;
        if (!nearlyParallel(ea, eb, kGroupSin, band)) {
            return false;
        }
        if (std::holds_alternative<LineSpan>(sa) &&
            (!onCarrierOf(startPoint(sb), ea, band) || !onCarrierOf(endPoint(sb), ea, band) ||
             !onCarrierOf(startPoint(sa), eb, band) || !onCarrierOf(endPoint(sa), eb, band))) {
            return false;
        }
        return reachesInto(sa, sb) || reachesInto(sb, sa);
    };
    {
        BoxGrid pieceGrid(pieceBoxes);
        for (std::size_t a = 0; a < kept.size(); ++a) {
            pieceGrid.query(pieceBoxes[a], [&](std::size_t b) {
                const std::size_t ea = kept[a].element;
                const std::size_t eb = kept[b].element;
                if (b <= a || ea == eb || find(ea) == find(eb)) {
                    return;
                }
                if (nearCarrier(a, b) && nearlyParallel(raw[find(ea)], raw[eb], 2.0 * kGroupSin, -1.0) &&
                    nearlyParallel(raw[find(eb)], raw[ea], 2.0 * kGroupSin, -1.0) &&
                    nearlyParallel(raw[find(ea)], raw[find(eb)], 2.0 * kGroupSin, -1.0)) {
                    unite(ea, eb);
                }
            });
        }
    }
    std::vector<Piece> out;
    std::map<std::size_t, std::vector<Piece>> groups;
    for (const Piece& p : kept) {
        if (shared[p.element]) {
            groups[find(p.element)].push_back(p);
        } else {
            out.push_back(p);  // not on a shared carrier
        }
    }

    // A stretch [lo, hi] of the common coordinate covered by one piece, which way it runs, and
    // the piece it came from (element and parameter range).
    struct Stretch {
        double lo;
        double hi;
        bool forward;
        Piece piece;
    };
    for (const auto& [root, pieces] : groups) {
        const Span& ref = raw[root];
        const auto* refLine = std::get_if<LineSpan>(&ref);
        const auto* refArc = std::get_if<ArcSpan>(&ref);
        if (!refLine && !refArc) {
            out.insert(out.end(), pieces.begin(), pieces.end());
            continue;
        }
        const Vec2 origin = refLine ? refLine->start : refArc->centre;
        const Vec2 axis = refLine ? (refLine->end - refLine->start).normalized() : Vec2{};
        const double radius = refArc ? refArc->radius : 1.0;
        const double refAngle = refArc ? refArc->startAngle : 0.0;
        auto coordOf = [&](Vec2 p) {
            return refLine ? (p - origin).dot(axis) : normalizeAngle((p - origin).angle() - refAngle);
        };
        auto pointOf = [&](double c) {
            return refLine ? origin + axis * c : origin + Vec2::fromAngle(refAngle + c, radius);
        };
        double scale = std::max(1.0, d);
        std::vector<Stretch> stretches;
        for (const Piece& p : pieces) {
            const Span& s = raw[p.element];
            const Vec2 a = pointAt(s, p.t0);
            const Vec2 b = pointAt(s, p.t1);
            scale = std::max({scale, std::fabs(a.x), std::fabs(a.y), std::fabs(b.x), std::fabs(b.y)});
            if (refLine) {
                const double ca = coordOf(a);
                const double cb = coordOf(b);
                stretches.push_back({std::min(ca, cb), std::max(ca, cb), cb > ca, p});
                continue;
            }
            // Angles about the reference centre: a piece runs CCW (increasing angle) or CW from
            // its start, by about its own sweep; one that passes the reference angle is cut there.
            const auto& arc = std::get<ArcSpan>(s);
            const bool ccw = arc.direction == ArcDirection::CCW;
            const double own = std::fabs(arc.sweepAngle()) * (p.t1 - p.t0);
            const double ca = coordOf(a);
            double span = normalizeAngle(ccw ? coordOf(b) - ca : ca - coordOf(b));
            if (std::fabs(span + kTwoPi - own) < std::fabs(span - own)) {
                span += kTwoPi;  // a (nearly) full circle: the end angle wrapped round to the start
            }
            const double lo = ccw ? ca : ca - span;
            const double hi = ccw ? ca + span : ca;
            if (lo < 0.0) {
                stretches.push_back({lo + kTwoPi, kTwoPi, ccw, p});
                stretches.push_back({0.0, hi, ccw, p});
            } else if (hi > kTwoPi) {
                stretches.push_back({lo, kTwoPi, ccw, p});
                stretches.push_back({0.0, hi - kTwoPi, ccw, p});
            } else {
                stretches.push_back({lo, hi, ccw, p});
            }
        }
        // Parameter on a stretch's own element of the point at coordinate c: the exact inverse of
        // coordOf restricted to that element (a line's point whose projection is c; an arc's point
        // on the ray from the reference centre at angle c). Every stretch handed to an owner lies
        // within its own piece's coordinates, up to the merging of nearby ends into one node, so a
        // merged stretch is part of a kept piece of its owner (extended by at most that rounding,
        // which keeps it continuous with a neighbour whose sub-rounding piece was merged away),
        // never a projection of it onto someone else's carrier (T-0017 R4).
        auto paramAt = [&](const Stretch& st, double c) {
            const Span& s = raw[st.piece.element];
            double t = 0.0;
            if (const auto* line = std::get_if<LineSpan>(&s)) {
                const double along = (line->end - line->start).dot(axis);
                t = along != 0.0 ? (c - (line->start - origin).dot(axis)) / along : st.piece.t0;
            } else {
                const auto& arc = std::get<ArcSpan>(s);
                const Vec2 u = Vec2::fromAngle(refAngle + c);
                const Vec2 w = origin - arc.centre;
                const double b = w.dot(u);
                const double rho =
                    -b + std::sqrt(std::max(0.0, b * b - (w.lengthSquared() - arc.radius * arc.radius)));
                const double ang = (origin + u * rho - arc.centre).angle();
                const double a0 = arc.angleAt(st.piece.t0);
                const double sweep = std::fabs(arc.sweepAngle());
                const double pieceSweep = sweep * (st.piece.t1 - st.piece.t0);
                double adv =
                    arc.direction == ArcDirection::CCW ? normalizeAngle(ang - a0) : normalizeAngle(a0 - ang);
                if (adv > pieceSweep + 0.5 * (kTwoPi - pieceSweep)) {
                    adv -= kTwoPi;  // just before the piece's start
                }
                t = st.piece.t0 + adv / sweep;
            }
            return std::clamp(t, 0.0, 1.0);
        };
        // How far the copy lies towards the offset side of the reference at coordinate c.
        auto outwardAt = [&](const Stretch& st, double c) {
            const Span& s = raw[st.piece.element];
            const double t = paramAt(st, c);
            return (pointAt(s, t) - pointOf(c)).dot(tangentAt(s, t).perp() * sign);
        };
        const double eps = 1e-9 * scale / (refLine ? 1.0 : radius);
        // Merged end coordinates; each stretch then covers ends [first, last).
        std::vector<double> ends;
        for (const Stretch& st : stretches) {
            ends.push_back(st.lo);
            ends.push_back(st.hi);
        }
        std::sort(ends.begin(), ends.end());
        std::vector<double> nodes;
        for (const double c : ends) {
            if (nodes.empty() || c > nodes.back() + eps) {
                nodes.push_back(c);
            }
        }
        auto nodeOf = [&](double c) {
            // The last node at or below c: the node c was merged into (a node is the smallest
            // value of its run, and the next node lies more than eps above that run's start).
            const auto it = std::upper_bound(nodes.begin(), nodes.end(), c);
            return static_cast<std::size_t>(it - nodes.begin()) - 1;
        };
        // Sweep: copies (element, stretch index) entering and leaving at each node.
        using Copy = std::pair<std::size_t, std::size_t>;
        std::vector<std::vector<std::pair<bool, Copy>>> enter(nodes.size());
        std::vector<std::vector<std::pair<bool, Copy>>> leave(nodes.size());
        for (std::size_t i = 0; i < stretches.size(); ++i) {
            const Stretch& st = stretches[i];
            const std::size_t a = nodeOf(st.lo);
            const std::size_t b = nodeOf(st.hi);
            if (a < b) {
                enter[a].emplace_back(st.forward, Copy{st.piece.element, i});
                leave[b].emplace_back(st.forward, Copy{st.piece.element, i});
            }
        }
        std::set<Copy> forward;
        std::set<Copy> backward;
        std::optional<std::pair<bool, std::size_t>> previous;  // direction and element of the last stretch
        const double level = 1e-12 * scale;  // rounding only: any real difference picks the outer
        auto emitOn = [&](std::size_t st, double c0, double c1, bool fw) {
            const Stretch& s = stretches[st];
            const double t0 = paramAt(s, fw ? c0 : c1);
            const double t1 = paramAt(s, fw ? c1 : c0);
            if (t1 > t0) {
                out.push_back(Piece{s.piece.element, t0, t1});
            }
            previous = std::pair{fw, s.piece.element};
        };
        for (std::size_t k = 0; k + 1 < nodes.size(); ++k) {
            for (const auto& [fw, copy] : leave[k]) {
                (fw ? forward : backward).erase(copy);
            }
            for (const auto& [fw, copy] : enter[k]) {
                (fw ? forward : backward).insert(copy);
            }
            if (forward.size() == backward.size()) {
                previous.reset();
                continue;  // nothing, or opposite copies enclosing nothing
            }
            const bool fw = forward.size() > backward.size();
            const std::set<Copy>& covering = fw ? forward : backward;
            // Keep the outermost copy (farthest towards the offset side), which is at least d
            // from the inputs of all the copies, along the whole stretch: start with the one
            // outermost at the stretch's start (level to rounding: the previous stretch's element,
            // so no sideways jog, then the one outermost at the far end, then the lowest-numbered)
            // and hand over wherever another copy overtakes it (their carriers cross).
            double c0 = nodes[k];
            const double c1 = nodes[k + 1];
            std::size_t cur = covering.begin()->second;
            {
                double best0 = -kInf;
                double best1 = -kInf;
                bool bestSticky = false;
                for (const Copy& copy : covering) {
                    const double v0 = outwardAt(stretches[copy.second], c0);
                    const double v1 = outwardAt(stretches[copy.second], c1);
                    const bool sticky = previous && previous->first == fw && previous->second == copy.first;
                    const bool better =
                        v0 > best0 + level ||
                        (v0 >= best0 - level &&
                         ((sticky && !bestSticky) || (sticky == bestSticky && v1 > best1 + level)));
                    if (better) {
                        best0 = std::max(best0, v0);
                        best1 = v1;
                        bestSticky = sticky;
                        cur = copy.second;
                    }
                }
            }
            for (std::size_t guard = 0; guard <= covering.size(); ++guard) {
                std::optional<std::size_t> next;
                double handOver = c1;
                const double cur1 = outwardAt(stretches[cur], c1);
                for (const Copy& copy : covering) {
                    if (copy.second == cur || outwardAt(stretches[copy.second], c1) <= cur1 + level) {
                        continue;
                    }
                    // Overtakes `cur` before c1: bisect for where the two cross.
                    double lo = c0;
                    double hi = c1;
                    for (int it = 0; it < 60 && hi - lo > eps * 1e-3; ++it) {
                        const double mid = 0.5 * (lo + hi);
                        if (outwardAt(stretches[copy.second], mid) > outwardAt(stretches[cur], mid) + level) {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    if (hi < handOver) {
                        handOver = hi;
                        next = copy.second;
                    }
                }
                if (!next) {
                    break;
                }
                emitOn(cur, c0, handOver, fw);
                c0 = handOver;
                cur = *next;
            }
            emitOn(cur, c0, c1, fw);
        }
    }
    std::sort(out.begin(), out.end(), [](const Piece& a, const Piece& b) {
        return a.element != b.element ? a.element < b.element : a.t0 < b.t0;
    });
    return out;
}

}  // namespace

std::vector<Contour> offsetOpenContour(const Contour& contour, double distance, OffsetSide side,
                                       JoinType join, const Tolerance& tol, double mitreLimit) {
    std::vector<Contour> result;
    const std::vector<Span> input = flattenForOffset(contour, tol);
    if (input.empty() || !std::isfinite(distance)) {
        return result;
    }
    double sign = side == OffsetSide::Left ? 1.0 : -1.0;
    if (distance < 0.0) {
        sign = -sign;
    }
    const double d = std::fabs(distance);
    if (tol.isZero(d)) {
        Contour same;
        same.closed = contour.closed;
        same.spans = input;
        result.push_back(std::move(same));
        return result;
    }

    // 1. Raw offset (see buildRaw); Sharp/Square corners record the region they add.
    std::vector<Wedge> wedges;
    const bool cyclic = contour.closed && input.size() > 1;
    const RawChain rc = buildRaw(input, d, sign, join, mitreLimit, tol, contour.closed, wedges);
    const std::vector<Span>& raw = rc.spans;
    const std::size_t m = raw.size();
    if (m == 0) {
        return result;
    }

    // The margin only absorbs rounding (pieces on the boundary sit at d to ~1e-15 relative). It
    // stays below tol^2 / (8d), the depth of the shallowest connector or overshoot longer than
    // the tolerance at a nearly straight inside corner, so no such sliver survives as a cusp.
    const double keepAbove = d - std::min(1e-9 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d);
    std::vector<Box2> inputBoxes;
    inputBoxes.reserve(input.size());
    for (const Span& s : input) {
        inputBoxes.push_back(bounds(s));
    }
    BoxGrid inputGrid(inputBoxes);

    // Dead elements: distance to a segment is a convex function of the point, so a raw element
    // whose convex hull points (see hullPoints) are all closer than keepAbove to one input line
    // lies wholly that close to it, and every piece of it would be pruned. At a large distance
    // relative to the feature size most of the raw chain is like that; dropping these elements
    // before splitting removes most of the boundary-split and pruning work (T-0006). Splits a
    // live element would get from crossing a dead one lie strictly inside the neighbourhood,
    // between two pieces that are pruned anyway.
    std::vector<bool> dead(m, false);
    std::vector<Vec2> hull;
    for (std::size_t i = 0; i < m; ++i) {
        hullPoints(raw[i], hull);
        const Vec2 a = hull.front();
        dead[i] = inputGrid.queryUntil(
            Box2{a - Vec2{keepAbove, keepAbove}, a + Vec2{keepAbove, keepAbove}}, [&](std::size_t j) {
                if (!std::holds_alternative<LineSpan>(input[j])) {
                    return false;
                }
                return std::all_of(hull.begin(), hull.end(),
                                   [&](Vec2 p) { return distanceToSpan(p, input[j]) < keepAbove; });
            });
    }

    // 2. Split the raw chain at its self-intersections (candidate pairs from a uniform grid).
    std::vector<std::vector<double>> splits(m);
    std::vector<Box2> boxes(m);
    for (std::size_t i = 0; i < m; ++i) {
        if (!dead[i]) {
            boxes[i] = bounds(raw[i]).inflated(1e-9);
        }
    }
    BoxGrid rawGrid(boxes);
    std::vector<std::pair<std::size_t, std::size_t>> overlapping;  // element pairs on one carrier
    auto adjacentShared = [&](std::size_t i, std::size_t j, double ti, double tj) {
        if (j == i + 1 && ti >= 1.0 - kParamEps && tj <= kParamEps) {
            return true;
        }
        return cyclic && m > 1 && i == 0 && j + 1 == m && ti <= kParamEps && tj >= 1.0 - kParamEps;
    };
    for (std::size_t i = 0; i < m; ++i) {
        if (dead[i]) {
            continue;
        }
        rawGrid.query(boxes[i], [&](std::size_t j) {
            if (j <= i) {
                return;
            }
            if (sameCarrier(raw[i], raw[j])) {
                // Overlapping elements (e.g. the mitre extensions of a slot narrower than 2d):
                // split each at the other's end points so the overlap becomes whole pieces. The
                // carriers may still cross (sameCarrier allows ~1e-9 of the length sideways, 1e-7
                // on a 100 mm chamfer); that crossing is split too, below, so that on each side of
                // it the outer copy can be kept (mergeOnCarriers) and nothing dips inside d.
                overlapping.emplace_back(i, j);
                for (const auto& [a, b, splitsOfA] :
                     {std::tuple{i, j, &splits[i]}, std::tuple{j, i, &splits[j]}}) {
                    for (const Vec2 p : {startPoint(raw[b]), endPoint(raw[b])}) {
                        const double t = carrierParam(raw[a], p);
                        if (t > kParamEps && t < 1.0 - kParamEps) {
                            splitsOfA->push_back(t);
                        }
                    }
                }
            }
            for (const Vec2 p : carrierIntersections(raw[i], raw[j])) {
                const double ti = carrierParam(raw[i], p);
                const double tj = carrierParam(raw[j], p);
                if (!inUnit(ti) || !inUnit(tj) || adjacentShared(i, j, ti, tj)) {
                    continue;
                }
                splits[i].push_back(ti);
                splits[j].push_back(tj);
            }
        });
    }
    // The rest of the d-neighbourhood boundary: circles about every input vertex, the offsets
    // on both sides and, for arcs with r < d, the inverted offsets beyond the centre. A piece
    // can only enter the neighbourhood by crossing that boundary, so after these splits each
    // piece lies wholly inside or outside and an interior sample decides it.
    //    Of a vertex circle only the arc between the two neighbours' offsets on the outer side
    //    of the turn can be on that boundary: every other point of it lies in the interior of the
    //    neighbourhood of one of the two spans meeting there (the d-neighbourhood of a span is
    //    bounded by its two offsets and the halves of its end circles facing away from it). So
    //    a vertex contributes that arc (angle = turn angle, widened by a hair so the split set
    //    is a superset), a reversal a full circle, and the path's two ends their full circles.
    //    So does a "vertex" where the flattened input is not continuous (a sub-tolerance span
    //    was left out): both ends there get full circles. Splitting at whole circles cost one
    //    split per raw element per vertex within d, which dominated dense input at a large d
    //    (T-0006).
    std::vector<Span> boundary;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const Span& s = input[i];
        const Span& prev = input[i == 0 ? input.size() - 1 : i - 1];
        if (i == 0 && !cyclic) {
            boundary.emplace_back(ArcSpan::fullCircle(startPoint(s), d));
        } else if (geometry::distance(endPoint(prev), startPoint(s)) > 1e-9 * std::max(1.0, d)) {
            boundary.emplace_back(ArcSpan::fullCircle(endPoint(prev), d));
            boundary.emplace_back(ArcSpan::fullCircle(startPoint(s), d));
        } else {
            const Vec2 tin = tangentAt(prev, 1.0);
            const Vec2 tout = tangentAt(s, 0.0);
            const double turn = std::atan2(tin.cross(tout), tin.dot(tout));
            constexpr double kWiden = 1e-6;
            if (std::fabs(turn) >= kPi - 2.0 * kWiden) {
                boundary.emplace_back(ArcSpan::fullCircle(startPoint(s), d));
            } else {
                // Outer side: right of travel for a left turn, left for a right turn; the
                // outward normal turns with the tangent, from n(tin) to n(tout).
                const double outer = turn > 0.0 ? -1.0 : 1.0;
                const double a0 = (tin.perp() * outer).angle();
                const double grow = turn > 0.0 ? kWiden : -kWiden;
                boundary.emplace_back(ArcSpan{startPoint(s), d, a0 - grow, a0 + turn + grow,
                                              turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW});
            }
        }
        for (const double sd : {1.0, -1.0}) {
            if (auto o = offsetSpan(s, d, sd)) {
                boundary.push_back(*o);
            } else if (auto inv = invertedOffset(s, d, sd)) {
                boundary.push_back(*inv);
            }
        }
    }
    if (!cyclic) {
        boundary.emplace_back(ArcSpan::fullCircle(endPoint(input.back()), d));
    }
    for (const Span& b : boundary) {
        const auto* arc = std::get_if<ArcSpan>(&b);
        const bool wholeCarrier = arc && arc->isFullCircle();
        rawGrid.query(bounds(b).inflated(1e-9), [&](std::size_t i) {
            for (const Vec2 p : carrierIntersections(raw[i], b)) {
                const double t = carrierParam(raw[i], p);
                if (inUnit(t) && (wholeCarrier || inUnit(carrierParam(b, p)))) {
                    splits[i].push_back(t);
                }
            }
        });
    }

    // 3. Keep the pieces that stay at least d from the input and outside every Sharp/Square
    //    corner region.
    std::vector<Box2> wedgeBoxes;
    for (const Wedge& w : wedges) {
        Box2 b;
        for (const Span& s : w) {
            b.expand(bounds(s));
        }
        wedgeBoxes.push_back(b);
    }
    BoxGrid wedgeGrid(wedgeBoxes);
    // Strictly inside a corner region; points on its boundary (the extensions themselves) do
    // not count.
    const double onBoundary = 1e-9 * std::max(1.0, d);
    auto inWedge = [&](Vec2 p) {
        bool hit = false;
        wedgeGrid.query(Box2{p, p}, [&](std::size_t w) {
            if (hit || !insideRegion(p, wedges[w])) {
                return;
            }
            double gap = std::numeric_limits<double>::infinity();
            for (const Span& s : wedges[w]) {
                gap = std::min(gap, distanceToSpan(p, s));
            }
            hit = gap > onBoundary;
        });
        return hit;
    };
    std::vector<Piece> kept;
    std::size_t witness = input.size();
    for (std::size_t i = 0; i < m; ++i) {
        if (dead[i]) {
            continue;
        }
        std::vector<double>& ts = splits[i];
        ts.push_back(0.0);
        ts.push_back(1.0);
        for (double& t : ts) {
            t = std::clamp(t, 0.0, 1.0);
        }
        std::sort(ts.begin(), ts.end());
        const double len = length(raw[i]);
        for (std::size_t k = 0; k + 1 < ts.size(); ++k) {
            if ((ts[k + 1] - ts[k]) * len <= 1e-9) {
                continue;
            }
            const Piece piece{i, ts[k], ts[k + 1]};
            bool ok = true;
            for (const double f : {0.25, 0.5, 0.75}) {
                const Vec2 q = pointAt(raw[i], piece.t0 + (piece.t1 - piece.t0) * f);
                if (closerThan(q, keepAbove, input, inputGrid, witness) || inWedge(q)) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                kept.push_back(piece);
            }
        }
    }
    // Two points computed separately at the same place on the offset (a piece's end and its
    // continuation's start) agree to rounding: much closer than this, while distinct split
    // points lie much further apart.
    auto coincide = [&](Vec2 p) { return 1e-9 * std::max({1.0, d, std::fabs(p.x), std::fabs(p.y)}); };
    // Overlapping elements on one carrier (see mergeOnCarriers): one boundary piece per stretch.
    kept = mergeOnCarriers(raw, kept, overlapping, d, sign, 0.5 * tol.distance,
                           std::min(5e-7 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d));
    if (kept.empty()) {
        return result;
    }

    // 4. Stitch the kept pieces into components by following their end points. Usually a
    //    piece continues with its successor along the raw chain, but where the offset turns onto
    //    a later or earlier part of it (around a mitre that pokes out over a neighbour, say) it
    //    continues with whichever unused kept piece starts where it ends. Components are started
    //    from pieces nothing leads into, in raw order; pieces left over form cycles.
    auto pieceStart = [&](const Piece& p) { return pointAt(raw[p.element], p.t0); };
    auto pieceEnd = [&](const Piece& p) { return pointAt(raw[p.element], p.t1); };
    const std::size_t nk = kept.size();
    PointIndex starts(tol.distance);
    for (std::size_t k = 0; k < nk; ++k) {
        starts.add(pieceStart(kept[k]), k);
    }
    std::vector<bool> hasIncoming(nk, false);
    for (std::size_t k = 0; k < nk; ++k) {
        starts.forEachNear(pieceEnd(kept[k]), [&](std::size_t j) {
            if (j != k) {
                hasIncoming[j] = true;
            }
        });
    }
    std::vector<bool> used(nk, false);
    std::vector<std::vector<Piece>> components;
    auto walk = [&](std::size_t k) {
        std::vector<Piece> comp;
        for (;;) {
            used[k] = true;
            comp.push_back(kept[k]);
            const std::size_t successor = (k + 1) % nk;
            std::optional<std::size_t> next;
            std::size_t bestRank = nk;
            double bestGap = kInf;
            const Vec2 end = pieceEnd(kept[k]);
            const double same = coincide(end);
            starts.forEachNear(end, [&](std::size_t j) {
                if (used[j]) {
                    return;
                }
                // A piece starting exactly here (to rounding) beats one merely within the
                // tolerance: where split points lie closer together than the tolerance (the
                // overlapping chamfers of a dense zig-zag) a start a little behind the end would
                // be a backward step (T-0006). Among equally near starts, prefer the raw
                // successor, then the nearest piece after k in raw order.
                const double gap = geometry::distance(pieceStart(kept[j]), end);
                const std::size_t rank = j == successor ? 0 : (j + nk - k) % nk;
                const bool nearer = gap + same < bestGap;
                const bool asNear = !nearer && gap <= bestGap + same;
                if (nearer || (asNear && rank < bestRank)) {
                    bestGap = std::min(bestGap, gap);
                    bestRank = rank;
                    next = j;
                }
            });
            if (!next) {
                break;
            }
            k = *next;
        }
        components.push_back(std::move(comp));
    };
    for (std::size_t k = 0; k < nk; ++k) {
        if (!used[k] && !hasIncoming[k]) {
            walk(k);
        }
    }
    for (std::size_t k = 0; k < nk; ++k) {
        if (!used[k]) {
            walk(k);
        }
    }

    // 5. Order: a component that closes on itself starts at its piece earliest along the input,
    //    and the components are sorted along the input by where they start.
    auto pathPos = [&](const Piece& p) {
        return rc.posA[p.element] + (rc.posB[p.element] - rc.posA[p.element]) * p.t0;
    };
    for (std::vector<Piece>& comp : components) {
        if (comp.size() > 1 && tol.equalPoints(pieceEnd(comp.back()), pieceStart(comp.front()))) {
            std::size_t best = 0;
            for (std::size_t k = 1; k < comp.size(); ++k) {
                if (pathPos(comp[k]) < pathPos(comp[best])) {
                    best = k;
                }
            }
            std::rotate(comp.begin(), comp.begin() + static_cast<std::ptrdiff_t>(best), comp.end());
        }
    }
    std::stable_sort(components.begin(), components.end(),
                     [&](const std::vector<Piece>& a, const std::vector<Piece>& b) {
                         return pathPos(a.front()) < pathPos(b.front());
                     });

    // Junction snapping (step 6) may move the offset towards the input by a hair more than the
    // pruning margin: up to 5e-7 relative, and below 0.1 tol^2 / d like that margin.
    const double snapAbove =
        d - std::max(d - keepAbove, std::min(5e-7 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d));
    const double snapBelow = 2.0 * d - snapAbove;
    std::size_t snapWitness = input.size();
    std::size_t farWitness = input.size();
    const SnapContext snap{
        sign,
        tol,
        [&](Vec2 p) { return !closerThan(p, snapAbove, input, inputGrid, snapWitness) && !inWedge(p); },
        [&](Vec2 p) {
            return join != JoinType::Round || closerThan(p, snapBelow, input, inputGrid, farWitness);
        },
        [&](Vec2 p) {
            const double reach = 2.0 * d + 2.0 * tol.distance;
            double nearest = kInf;
            inputGrid.query(Box2{p - Vec2{reach, reach}, p + Vec2{reach, reach}},
                            [&](std::size_t i) { nearest = std::min(nearest, distanceToSpan(p, input[i])); });
            return nearest;
        },
        [&](Vec2 p) {
            const double reach = 2.0 * d + 2.0 * tol.distance;
            double best = kInf;
            Vec2 at = p;
            inputGrid.query(Box2{p - Vec2{reach, reach}, p + Vec2{reach, reach}}, [&](std::size_t i) {
                Vec2 q;
                if (const auto* line = std::get_if<LineSpan>(&input[i])) {
                    const Vec2 ab = line->end - line->start;
                    const double len2 = ab.lengthSquared();
                    q = line->start +
                        ab * (len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0);
                } else {
                    const auto& arc = std::get<ArcSpan>(input[i]);
                    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
                        q = arc.centre + (p - arc.centre).normalized() * arc.radius;
                    } else {
                        q = geometry::distance(p, arc.startPoint()) <= geometry::distance(p, arc.endPoint())
                                ? arc.startPoint()
                                : arc.endPoint();
                    }
                }
                const double dq = geometry::distance(p, q);
                if (dq < best) {
                    best = dq;
                    at = q;
                }
            });
            return at;
        },
        d,
        d - snapAbove,
        join == JoinType::Round};
    auto emit = [&](std::vector<Piece>::const_iterator b, std::vector<Piece>::const_iterator e, bool loop) {
        std::vector<Piece> merged;
        for (auto it = b; it != e; ++it) {
            if (!merged.empty() && merged.back().element == it->element &&
                it->t0 >= merged.back().t1 - kParamEps) {
                // Same element split at a touching point (or around a sub-tolerance sliver the
                // chain stepped over): re-merge.
                merged.back().t1 = it->t1;
            } else {
                merged.push_back(*it);
            }
        }
        // Pieces no longer than the tolerance are dropped, except where dropping one would open
        // a gap wider than the tolerance (a run of tiny pieces round a sub-tolerance bump); the
        // chain thus stays continuous within the tolerance.
        std::vector<Span> spans;
        std::vector<bool> tiny;
        for (const Piece& p : merged) {
            spans.push_back(subSpan(raw[p.element], p.t0, p.t1));
            tiny.push_back(isDegenerate(spans.back(), tol));
        }
        // The dropped pieces are remembered next to the span they preceded, in case the junction
        // they leave cannot be snapped shut (see snapChain).
        const auto lastSolid = std::find(tiny.rbegin(), tiny.rend(), false);
        const std::size_t solidEnd = static_cast<std::size_t>(tiny.rend() - lastSolid);  // one past
        double total = 0.0;
        std::vector<Span> solid;
        std::vector<std::vector<Span>> dropped(1);
        std::vector<Span> leading;
        for (std::size_t k = 0; k < solidEnd; ++k) {
            if (tiny[k] && (solid.empty() ||
                            geometry::distance(endPoint(solid.back()), endPoint(spans[k])) <= tol.distance)) {
                (solid.empty() ? leading : dropped.back()).push_back(spans[k]);
                continue;
            }
            total += length(spans[k]);
            solid.push_back(spans[k]);
            dropped.emplace_back();
        }
        if (total > tol.distance * (loop ? 4.0 : 1.0)) {
            // dropped[k] precedes solid[k]; dropped[0] crosses the closing junction (trailing
            // pieces, then leading ones).
            dropped.pop_back();
            dropped[0].assign(spans.begin() + static_cast<std::ptrdiff_t>(solidEnd), spans.end());
            dropped[0].insert(dropped[0].end(), leading.begin(), leading.end());
            bool stillClosed = false;
            std::vector<std::vector<Span>> pieces = snapChain(solid, dropped, loop, snap, stillClosed);
            for (std::vector<Span>& spansOfPiece : pieces) {
                Contour piece;
                piece.closed = stillClosed;
                piece.spans = std::move(spansOfPiece);
                removeBackSteps(piece.spans, piece.closed, snap);
                removeRetraces(piece.spans, piece.closed, snap, d);
                mergeCoCircularArcs(piece.spans, piece.closed);
                if (!isSliverLoop(piece, tol)) {
                    result.push_back(std::move(piece));
                }
            }
        }
    };
    // 6. Every crossing or touching point of the raw chain is a split point, so a component
    //    can only meet itself at piece boundaries ("nodes"): walk each one and cut it whenever
    //    it returns to a node it already visited. The part before the node becomes an open
    //    chain, the part from the node back to itself a closed loop, and the walk continues
    //    from the node, so each emitted chain visits every node once and is simple.
    PointIndex index(tol.distance);
    for (const std::vector<Piece>& comp : components) {
        std::vector<Vec2> nodes{pieceStart(comp.front())};
        index.clear();
        index.add(nodes.front(), 0);
        auto chainBegin = comp.begin();
        for (auto it = comp.begin(); it != comp.end(); ++it) {
            const Vec2 e = pieceEnd(*it);
            if (tol.equalPoints(e, nodes.back())) {
                // A piece shorter than the tolerance: keep it in the chain (it is dropped as
                // degenerate on output) and treat its end as the same node.
                nodes.push_back(nodes.back());
                continue;
            }
            if (const std::optional<std::size_t> hit = index.findFirst(e)) {
                // Back at an earlier node: split off the tail before it and the loop through it.
                const auto loopBegin = chainBegin + static_cast<std::ptrdiff_t>(*hit);
                if (*hit > 0) {
                    emit(chainBegin, loopBegin, false);
                }
                emit(loopBegin, it + 1, true);
                chainBegin = it + 1;
                nodes.assign(1, e);
                index.clear();
                index.add(e, 0);
                continue;
            }
            nodes.push_back(e);
            index.add(e, nodes.size() - 1);
        }
        if (chainBegin != comp.end()) {
            emit(chainBegin, comp.end(), false);
        }
    }
    return result;
}

}  // namespace carvetto::geometry
