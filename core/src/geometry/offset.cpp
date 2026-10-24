#include "carvetto/geometry/offset.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

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
//  2. Split the raw chain at all its self-intersections and where it crosses the rest of the
//     d-neighbourhood boundary (circles about the input vertices, opposite-side offsets).
//  3. Discard every piece that comes closer than d to the input. What remains is exactly the
//     part of the raw offset that bounds the d-neighbourhood on the requested side, so spans
//     consumed between inside corners vanish and nothing backtracks.
//  4. Stitch the surviving pieces back together, re-merging pieces of the same element.
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double kParamEps = 1e-9;

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

// Offsets one line or arc span by `d` towards `sign` (+1 left, -1 right). Returns nullopt when
// an arc collapses (its offset radius is not positive).
std::optional<Span> offsetSpan(const Span& s, double d, double sign, const Tolerance& tol) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 n = line->tangentAt(0.0).perp() * (d * sign);
        return Span{LineSpan{line->start + n, line->end + n}};
    }
    const auto& arc = std::get<ArcSpan>(s);
    // On a CCW arc the left normal points at the centre; on a CW arc it points away.
    const double towardsCentre = arc.direction == ArcDirection::CCW ? sign : -sign;
    const double r = arc.radius - towardsCentre * d;
    if (r <= tol.distance) {
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
// b's start, nearest `vertex` and within `maxDist` of it.
std::optional<Vec2> extensionMeet(const Span& a, const Span& b, Vec2 vertex, double maxDist) {
    std::optional<Vec2> best;
    for (const Vec2 p : carrierIntersections(a, b)) {
        if (carrierParam(a, p) < 1.0 - kParamEps || carrierParam(b, p) > kParamEps) {
            continue;
        }
        if (geometry::distance(p, vertex) > maxDist) {
            continue;
        }
        if (!best || geometry::distance(p, vertex) < geometry::distance(*best, vertex)) {
            best = p;
        }
    }
    return best;
}

// Square join: extends `a` and `b` to the chamfer line at distance d from the vertex and
// returns the chamfer; nullopt when an element cannot reach it (caller falls back to Round).
std::optional<LineSpan> squareJoin(Span& a, Span& b, Vec2 vertex, Vec2 tangentIn, double d) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    Vec2 bisector = ((e - vertex) + (s - vertex)).normalized();
    if (bisector.lengthSquared() == 0.0) {
        bisector = tangentIn;  // full reversal: cap perpendicular to the direction of travel
    }
    const Vec2 q = vertex + bisector * d;
    const Span chamferCarrier = LineSpan{q, q + bisector.perp()};
    const double reach = 1.5 * d;  // chamfer ends lie within d / cos(45 deg) of the vertex
    std::optional<Vec2> h1;
    for (const Vec2 p : carrierIntersections(a, chamferCarrier)) {
        if (carrierParam(a, p) >= 1.0 - kParamEps && geometry::distance(p, vertex) <= reach &&
            (!h1 || carrierParam(a, p) < carrierParam(a, *h1))) {
            h1 = p;
        }
    }
    std::optional<Vec2> h2;
    for (const Vec2 p : carrierIntersections(b, chamferCarrier)) {
        if (carrierParam(b, p) <= kParamEps && geometry::distance(p, vertex) <= reach &&
            (!h2 || carrierParam(b, p) > carrierParam(b, *h2))) {
            h2 = p;
        }
    }
    if (!h1 || !h2 || (*h2 - *h1).dot(s - e) < 0.0) {
        return std::nullopt;
    }
    a = withEnd(a, *h1);
    b = withStart(b, *h2);
    return LineSpan{*h1, *h2};
}

// Joins `prev` (the last raw element) to `next` around an outside corner at `vertex`. Sharp and
// Square modify the two elements in place (extension); any join element is appended to `raw`.
void outsideCorner(std::vector<Span>& raw, Span& next, Vec2 vertex, Vec2 tangentIn, double d, double sign,
                   JoinType join, double mitreLimit, const Tolerance& tol) {
    Span& prev = raw.back();
    const Vec2 e = endPoint(prev);
    const Vec2 s = startPoint(next);
    if (join == JoinType::Sharp) {
        if (const auto m = extensionMeet(prev, next, vertex, std::max(mitreLimit, 1.0) * d)) {
            prev = withEnd(prev, *m);
            next = withStart(next, *m);
            return;
        }
        // Beyond the mitre limit (or the carriers never meet): square off the corner.
    }
    if (join != JoinType::Round) {
        Span a = prev;
        Span b = next;
        if (const auto chamfer = squareJoin(a, b, vertex, tangentIn, d)) {
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
             double sign, JoinType join, double mitreLimit, const Tolerance& tol) {
    const Vec2 e = endPoint(raw.back());
    if (tol.equalPoints(e, startPoint(next))) {
        return;
    }
    if (classifyCorner(tangentIn, tangentOut, sign) == CornerKind::Outside) {
        outsideCorner(raw, next, vertex, tangentIn, d, sign, join, mitreLimit, tol);
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

double distanceToPath(Vec2 p, const std::vector<Span>& path) {
    double best = std::numeric_limits<double>::infinity();
    for (const Span& s : path) {
        best = std::min(best, distanceToSpan(p, s));
    }
    return best;
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

struct Piece {
    std::size_t element;
    double t0;
    double t1;
};

}  // namespace

Contour offsetOpenContour(const Contour& contour, double distance, OffsetSide side, JoinType join,
                          const Tolerance& tol, double mitreLimit) {
    Contour result;
    result.closed = contour.closed;
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
        result.spans = input;
        return result;
    }

    // 1. Raw offset: every element offset on its own, chained with joins (outside corners) and
    //    straight connectors (inside corners). An arc whose offset radius would be negative is
    //    inverted (radius d - r on the far side of its centre); it is discarded by pruning but
    //    keeps the chain's corner geometry, so the round joins about its end points survive.
    std::vector<Span> raw;
    for (std::size_t i = 0; i < input.size(); ++i) {
        std::optional<Span> next = offsetSpan(input[i], d, sign, tol);
        if (!next) {
            ArcSpan inverted = std::get<ArcSpan>(input[i]);
            const double towardsCentre = inverted.direction == ArcDirection::CCW ? sign : -sign;
            inverted.radius = towardsCentre * d - inverted.radius;
            inverted.startAngle += kPi;
            inverted.endAngle += kPi;
            if (inverted.radius <= tol.distance) {
                continue;  // offsets to (nearly) the arc's centre point: nothing to add
            }
            next = Span{inverted};
        }
        if (raw.empty()) {
            raw.push_back(*next);
            continue;
        }
        connect(raw, *next, startPoint(input[i]), tangentAt(input[i - 1], 1.0), tangentAt(input[i], 0.0), d,
                sign, join, mitreLimit, tol);
        raw.push_back(*next);
    }
    if (raw.empty()) {
        return result;
    }
    if (contour.closed && raw.size() > 1) {
        Span first = raw.front();
        connect(raw, first, startPoint(input.front()), tangentAt(input.back(), 1.0),
                tangentAt(input.front(), 0.0), d, sign, join, mitreLimit, tol);
        raw.front() = first;
    }
    const bool cyclic = contour.closed && raw.size() > 1;
    const std::size_t m = raw.size();

    // 2. Split the raw chain at its self-intersections.
    std::vector<std::vector<double>> splits(m);
    std::vector<Box2> boxes(m);
    for (std::size_t i = 0; i < m; ++i) {
        boxes[i] = bounds(raw[i]).inflated(1e-9);
    }
    auto adjacentShared = [&](std::size_t i, std::size_t j, double ti, double tj) {
        if (j == i + 1 && ti >= 1.0 - kParamEps && tj <= kParamEps) {
            return true;
        }
        return cyclic && i == 0 && j + 1 == m && ti <= kParamEps && tj >= 1.0 - kParamEps;
    };
    for (std::size_t i = 0; i < m; ++i) {
        for (std::size_t j = i + 1; j < m; ++j) {
            if (!boxes[i].intersects(boxes[j])) {
                continue;
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
        }
    }
    // The rest of the d-neighbourhood boundary: circles about every input vertex and the
    // opposite-side offsets. A raw piece can only enter the neighbourhood by crossing the
    // boundary, so after these splits one interior sample decides each piece.
    std::vector<Span> boundary;
    for (const Span& s : input) {
        boundary.emplace_back(ArcSpan::fullCircle(startPoint(s), d));
        if (auto opposite = offsetSpan(s, d, -sign, tol)) {
            boundary.push_back(*opposite);
        }
    }
    boundary.emplace_back(ArcSpan::fullCircle(endPoint(input.back()), d));
    for (const Span& b : boundary) {
        const Box2 bb = bounds(b).inflated(1e-9);
        const auto* arc = std::get_if<ArcSpan>(&b);
        const bool wholeCarrier = arc && arc->isFullCircle();
        for (std::size_t i = 0; i < m; ++i) {
            if (!boxes[i].intersects(bb)) {
                continue;
            }
            for (const Vec2 p : carrierIntersections(raw[i], b)) {
                const double t = carrierParam(raw[i], p);
                if (inUnit(t) && (wholeCarrier || inUnit(carrierParam(b, p)))) {
                    splits[i].push_back(t);
                }
            }
        }
    }

    // 3. Keep the pieces that stay at least d from the input.
    const double keepAbove = d - 1e-7 * std::max(1.0, d);
    std::vector<Piece> pieces;
    std::vector<bool> valid;
    for (std::size_t i = 0; i < m; ++i) {
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
                const double t = piece.t0 + (piece.t1 - piece.t0) * f;
                if (distanceToPath(pointAt(raw[i], t), input) < keepAbove) {
                    ok = false;
                    break;
                }
            }
            pieces.push_back(piece);
            valid.push_back(ok);
        }
    }

    // 4. Stitch consecutive valid pieces into components; keep the longest.
    auto pieceStart = [&](const Piece& p) { return pointAt(raw[p.element], p.t0); };
    auto pieceEnd = [&](const Piece& p) { return pointAt(raw[p.element], p.t1); };
    std::vector<Piece> kept;
    for (std::size_t k = 0; k < pieces.size(); ++k) {
        if (valid[k]) {
            kept.push_back(pieces[k]);
        }
    }
    if (kept.empty()) {
        return result;
    }
    const std::size_t nk = kept.size();
    auto continues = [&](std::size_t from, std::size_t to) {
        return tol.equalPoints(pieceEnd(kept[from]), pieceStart(kept[to]));
    };
    std::size_t first = 0;
    bool fullLoop = false;
    if (cyclic) {
        // Start right after a break so that a component never wraps around the array end.
        fullLoop = true;
        for (std::size_t k = 0; k < nk; ++k) {
            if (!continues((k + nk - 1) % nk, k)) {
                fullLoop = false;
                first = k;
                break;
            }
        }
    }
    std::vector<std::vector<Piece>> components;
    for (std::size_t n = 0; n < nk; ++n) {
        const std::size_t k = (first + n) % nk;
        if (n > 0 && continues((k + nk - 1) % nk, k)) {
            Piece& last = components.back().back();
            if (last.element == kept[k].element && last.t1 >= kept[k].t0 - kParamEps) {
                last.t1 = kept[k].t1;  // same element split at a touching point: re-merge
            } else {
                components.back().push_back(kept[k]);
            }
        } else {
            components.push_back({kept[k]});
        }
    }
    if (components.empty()) {
        return result;
    }
    auto componentLength = [&](const std::vector<Piece>& c) {
        double total = 0.0;
        for (const Piece& p : c) {
            total += length(raw[p.element]) * (p.t1 - p.t0);
        }
        return total;
    };
    const std::vector<Piece>* best = &components.front();
    for (const auto& c : components) {
        if (componentLength(c) > componentLength(*best) + 1e-12) {
            best = &c;
        }
    }
    result.closed = fullLoop;
    for (const Piece& p : *best) {
        const Span s = subSpan(raw[p.element], p.t0, p.t1);
        if (!isDegenerate(s, tol)) {
            result.spans.push_back(s);
        }
    }
    return result;
}

}  // namespace carvetto::geometry
