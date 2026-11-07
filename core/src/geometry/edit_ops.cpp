#include "carvetto/geometry/edit_ops.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>

#include "box_index.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"

namespace carvetto::geometry {

const char* toString(EditStatus s) {
    switch (s) {
        case EditStatus::Ok: return "ok";
        case EditStatus::InvalidArgument: return "invalid argument";
        case EditStatus::EmptyContour: return "empty contour";
        case EditStatus::BadNode: return "node index out of range";
        case EditStatus::Discontinuous: return "spans at the node do not meet";
        case EditStatus::DegenerateSpan: return "degenerate span";
        case EditStatus::NoCorner: return "no corner at this node";
        case EditStatus::Cusp: return "spans fold back onto each other";
        case EditStatus::RadiusTooLarge: return "radius does not fit";
        case EditStatus::NotClosed: return "boundary contour is not closed";
        case EditStatus::NothingToTrim: return "no intersection to trim to";
        case EditStatus::ClosedContour: return "contour is closed";
        case EditStatus::NoIntersection: return "no intersection ahead of the end";
    }
    return "unknown";
}

namespace {

bool finite(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool tolValid(const Tolerance& tol) {
    return std::isfinite(tol.distance) && tol.distance >= 0.0 && std::isfinite(tol.chordal) && tol.chordal > 0.0;
}

bool spanFinite(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return finite(l->start) && finite(l->end);
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        return finite(a->centre) && std::isfinite(a->radius) && a->radius >= 0.0 && std::isfinite(a->startAngle) &&
               std::isfinite(a->endAngle);
    }
    const auto& b = std::get<BezierSpan>(s);
    return finite(b.p0) && finite(b.p1) && finite(b.p2) && finite(b.p3);
}

bool contourFinite(const Contour& c) {
    return std::all_of(c.spans.begin(), c.spans.end(), [](const Span& s) { return spanFinite(s); });
}

/// Curve intersection with a Bezier costs about extent / tol.chordal chord pairs, so a drawing far
/// larger than its chordal tolerance is refused instead of grinding (ADR-0028).
constexpr double kMaxCurveCostRatio = 1e6;

bool tooCostly(const std::vector<const std::vector<Contour>*>& sets, const Contour* extra, const Tolerance& tol) {
    Box2 box;
    bool bezier = false;
    auto scan = [&](const Contour& c) {
        for (const Span& sp : c.spans) {
            bezier = bezier || std::holds_alternative<BezierSpan>(sp);
        }
        box.expand(c.bounds());
    };
    if (extra != nullptr) {
        scan(*extra);
    }
    for (const std::vector<Contour>* set : sets) {
        for (const Contour& c : *set) {
            scan(c);
        }
    }
    return bezier && !box.isEmpty() && distance(box.min, box.max) / tol.chordal > kMaxCurveCostRatio;
}

Span degenerateAt(Vec2 p) {
    return LineSpan{p, p};
}

/// The part of `s` between own parameters t0 and t1 (t1 <= t0 gives a zero-length line).
Span subSpan(const Span& s, double t0, double t1) {
    t0 = std::max(t0, 0.0);
    t1 = std::min(t1, 1.0);
    if (t0 <= 0.0 && t1 >= 1.0) {
        return s;
    }
    if (!(t1 > t0)) {
        return degenerateAt(pointAt(s, std::clamp(t0, 0.0, 1.0)));
    }
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return LineSpan{t0 <= 0.0 ? l->start : l->pointAt(t0), t1 >= 1.0 ? l->end : l->pointAt(t1)};
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        ArcSpan r = *a;
        r.startAngle = t0 <= 0.0 ? a->startAngle : a->angleAt(t0);
        r.endAngle = t1 >= 1.0 ? a->endAngle : a->angleAt(t1);
        return r;
    }
    const auto& b = std::get<BezierSpan>(s);
    if (t0 <= 0.0) {
        return b.split(t1).first;
    }
    if (t1 >= 1.0) {
        return b.split(t0).second;
    }
    return b.split(t1).first.split(t0 / t1).second;
}

Vec2 leftNormal(Vec2 t) {
    return Vec2{-t.y, t.x};
}

Vec2 offsetPoint(const Span& s, double t, double signedRadius) {
    return pointAt(s, t) + leftNormal(tangentAt(s, t)) * signedRadius;
}

std::size_t sampleCount(const Span& s) {
    if (std::holds_alternative<LineSpan>(s)) {
        return 1;
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        const double n = std::ceil(std::fabs(a->sweepAngle()) / (kPi / 16.0));
        return static_cast<std::size_t>(std::clamp(n, 1.0, 64.0));
    }
    return 32;
}

double approxLength(const Span& s) {
    const std::size_t n = sampleCount(s);
    double len = 0.0;
    Vec2 prev = pointAt(s, 0.0);
    for (std::size_t i = 1; i <= n; ++i) {
        const Vec2 p = pointAt(s, static_cast<double>(i) / static_cast<double>(n));
        len += distance(prev, p);
        prev = p;
    }
    return len;
}

double approxLength(const std::vector<Span>& spans) {
    double len = 0.0;
    for (const Span& s : spans) {
        len += approxLength(s);
    }
    return len;
}

struct FilletSolution {
    double ta{0.0};
    double tb{0.0};
    Vec2 centre;
};

/// Intersection of the offset curves of `a` and `b` (both offset by `sr` to their left) nearest
/// to the corner: sampled polylines find the crossings, Newton polishes the best one.
std::optional<FilletSolution> solveNormalFillet(const Span& a, const Span& b, double sr, double scale) {
    const std::size_t na = sampleCount(a);
    const std::size_t nb = sampleCount(b);
    std::vector<Vec2> pa(na + 1);
    std::vector<Vec2> pb(nb + 1);
    for (std::size_t i = 0; i <= na; ++i) {
        pa[i] = offsetPoint(a, static_cast<double>(i) / static_cast<double>(na), sr);
    }
    for (std::size_t i = 0; i <= nb; ++i) {
        pb[i] = offsetPoint(b, static_cast<double>(i) / static_cast<double>(nb), sr);
    }
    const double lenA = approxLength(a);
    const double lenB = approxLength(b);
    constexpr double kSlack = 1e-9;
    bool found = false;
    double bestScore = std::numeric_limits<double>::infinity();
    double bestA = 0.0;
    double bestB = 0.0;
    for (std::size_t i = 0; i < na; ++i) {
        const Vec2 d1 = pa[i + 1] - pa[i];
        for (std::size_t j = 0; j < nb; ++j) {
            const Vec2 d2 = pb[j + 1] - pb[j];
            const double den = d1.cross(d2);
            if (!(std::fabs(den) > 1e-18 * d1.length() * d2.length()) || den == 0.0) {
                continue;
            }
            const Vec2 w = pb[j] - pa[i];
            const double u = w.cross(d2) / den;
            const double v = w.cross(d1) / den;
            if (u < -kSlack || u > 1.0 + kSlack || v < -kSlack || v > 1.0 + kSlack) {
                continue;
            }
            const double ta = (static_cast<double>(i) + std::clamp(u, 0.0, 1.0)) / static_cast<double>(na);
            const double tb = (static_cast<double>(j) + std::clamp(v, 0.0, 1.0)) / static_cast<double>(nb);
            const double score = (1.0 - ta) * lenA + tb * lenB;
            if (score < bestScore) {
                bestScore = score;
                bestA = ta;
                bestB = tb;
                found = true;
            }
        }
    }
    if (!found) {
        return std::nullopt;
    }
    // Newton on OA(ta) - OB(tb) = 0 with central-difference derivatives.
    double ta = bestA;
    double tb = bestB;
    auto residual = [&](double x, double y) { return offsetPoint(a, x, sr) - offsetPoint(b, y, sr); };
    double best = residual(ta, tb).length();
    const double goal = 1e-13 * scale;
    for (int it = 0; it < 30 && best > goal; ++it) {
        const Vec2 f = residual(ta, tb);
        const double h = 1e-6;
        const Vec2 da = (offsetPoint(a, ta + h, sr) - offsetPoint(a, ta - h, sr)) / (2.0 * h);
        const Vec2 db = (offsetPoint(b, tb + h, sr) - offsetPoint(b, tb - h, sr)) / (2.0 * h);
        const double det = -da.x * db.y + db.x * da.y;
        if (det == 0.0 || !std::isfinite(det)) {
            break;
        }
        // Solve [da, -db] * [x, y] = -f.
        double x = (-f.x * -db.y - -db.x * -f.y) / det;
        double y = (da.x * -f.y - -f.x * da.y) / det;
        x = std::clamp(x, -0.25, 0.25);
        y = std::clamp(y, -0.25, 0.25);
        const double nta = ta + x;
        const double ntb = tb + y;
        const double r = residual(nta, ntb).length();
        if (!(r < best)) {
            break;
        }
        best = r;
        ta = nta;
        tb = ntb;
    }
    if (ta < -kSlack || ta > 1.0 + kSlack || tb < -kSlack || tb > 1.0 + kSlack) {
        return std::nullopt;
    }
    ta = std::clamp(ta, 0.0, 1.0);
    tb = std::clamp(tb, 0.0, 1.0);
    const Vec2 ca = offsetPoint(a, ta, sr);
    const Vec2 cb = offsetPoint(b, tb, sr);
    if (distance(ca, cb) > 1e-6 * std::max(scale, 1.0)) {
        return std::nullopt;
    }
    return FilletSolution{ta, tb, (ca + cb) * 0.5};
}

/// Parameter where the span is `d` away from `p`, searching from the span's end (fromEnd) or
/// start towards the other end; nullopt when the span never gets that far.
std::optional<double> paramAtDistance(const Span& s, Vec2 p, double d, bool fromEnd) {
    constexpr int kSteps = 64;
    auto at = [&](int i) { return fromEnd ? 1.0 - static_cast<double>(i) / kSteps : static_cast<double>(i) / kSteps; };
    double prev = at(0);
    for (int i = 1; i <= kSteps; ++i) {
        const double t = at(i);
        if (distance(pointAt(s, t), p) >= d) {
            double lo = prev;  // closer than d
            double hi = t;     // at least d
            for (int k = 0; k < 60; ++k) {
                const double mid = 0.5 * (lo + hi);
                if (distance(pointAt(s, mid), p) >= d) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            return hi;
        }
        prev = t;
    }
    return std::nullopt;
}

struct CornerGeom {
    Span arc;
    double ta{1.0};  // A keeps [0, ta]
    double tb{0.0};  // B keeps [tb, 1]
};

std::optional<CornerGeom> cornerGeometry(const Span& a, const Span& b, Vec2 corner, double sign, double r,
                                         FilletKind kind, const FilletOptions& opt) {
    const ArcDirection dir = sign > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
    const double scale = std::max({r, std::fabs(corner.x), std::fabs(corner.y)});
    switch (kind) {
        case FilletKind::Normal: {
            const auto sol = solveNormalFillet(a, b, sign * r, scale);
            if (!sol) {
                return std::nullopt;
            }
            const Vec2 pa = pointAt(a, sol->ta);
            const Vec2 pb = pointAt(b, sol->tb);
            ArcSpan arc = ArcSpan::fromCentre(sol->centre, pa, pb, dir);
            arc.radius = r;
            return CornerGeom{arc, sol->ta, sol->tb};
        }
        case FilletKind::DogBone: {
            const auto ta = paramAtDistance(a, corner, r, true);
            const auto tb = paramAtDistance(b, corner, r, false);
            if (!ta || !tb) {
                return std::nullopt;
            }
            ArcSpan arc = ArcSpan::fromCentre(corner, pointAt(a, *ta), pointAt(b, *tb), dir);
            arc.radius = r;
            return CornerGeom{arc, *ta, *tb};
        }
        case FilletKind::TBone: {
            if (opt.tBoneAlongIncoming) {
                const auto ta = paramAtDistance(a, corner, 2.0 * r, true);
                if (!ta) {
                    return std::nullopt;
                }
                const Vec2 s = pointAt(a, *ta);
                const Vec2 mid = (s + corner) * 0.5;
                ArcSpan arc = ArcSpan::fromCentre(mid, s, corner, dir);
                arc.radius = distance(mid, s);
                return CornerGeom{arc, *ta, 0.0};
            }
            const auto tb = paramAtDistance(b, corner, 2.0 * r, false);
            if (!tb) {
                return std::nullopt;
            }
            const Vec2 e = pointAt(b, *tb);
            const Vec2 mid = (e + corner) * 0.5;
            ArcSpan arc = ArcSpan::fromCentre(mid, corner, e, dir);
            arc.radius = distance(mid, e);
            return CornerGeom{arc, 1.0, *tb};
        }
    }
    return std::nullopt;
}

/// Applies one corner to a working span list. `consumed` marks spans reduced to nothing (erased
/// by the caller once all corners are done so node indices stay valid).
EditStatus applyCorner(std::vector<Span>& spans, std::vector<char>& consumed, std::vector<std::optional<Span>>& arcs,
                       bool closed, std::size_t node,
                       double radius, FilletKind kind, const FilletOptions& opt, const Tolerance& tol,
                       double& applied, bool& clamped, std::size_t& arcPos) {
    const std::size_t n = spans.size();
    if (n == 0) {
        return EditStatus::EmptyContour;
    }
    if (n < 2) {
        return closed ? EditStatus::NoCorner : EditStatus::BadNode;
    }
    if (closed ? node >= n : (node < 1 || node >= n)) {
        return EditStatus::BadNode;
    }
    const std::size_t ia = (node + n - 1) % n;
    const std::size_t ib = node;
    const Span a = spans[ia];
    const Span b = spans[ib];
    const Vec2 corner = endPoint(a);
    if (distance(corner, startPoint(b)) > tol.distance) {
        return EditStatus::Discontinuous;
    }
    if (!(approxLength(a) > tol.distance) || !(approxLength(b) > tol.distance)) {
        return EditStatus::DegenerateSpan;
    }
    const Vec2 ta = tangentAt(a, 1.0);
    const Vec2 tb = tangentAt(b, 0.0);
    if (ta.lengthSquared() == 0.0 || tb.lengthSquared() == 0.0) {
        return EditStatus::DegenerateSpan;
    }
    const double turn = std::atan2(ta.cross(tb), ta.dot(tb));
    if (std::fabs(turn) < 1e-6) {
        return EditStatus::NoCorner;
    }
    if (kPi - std::fabs(turn) < 1e-6) {
        return EditStatus::Cusp;
    }
    const double sign = turn > 0.0 ? 1.0 : -1.0;

    std::optional<CornerGeom> geom = cornerGeometry(a, b, corner, sign, radius, kind, opt);
    double used = radius;
    clamped = false;
    if (!geom) {
        if (!opt.clampToFit) {
            return EditStatus::RadiusTooLarge;
        }
        double lo = 0.0;
        double hi = radius;
        std::optional<CornerGeom> bestGeom;
        double bestR = 0.0;
        for (int it = 0; it < 60; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (!(mid > tol.distance)) {
                break;
            }
            auto g = cornerGeometry(a, b, corner, sign, mid, kind, opt);
            if (g) {
                lo = mid;
                bestGeom = g;
                bestR = mid;
            } else {
                hi = mid;
            }
        }
        if (!bestGeom) {
            return EditStatus::RadiusTooLarge;
        }
        geom = bestGeom;
        used = bestR;
        clamped = true;
    }
    applied = used;

    const double eps = tol.distance * 1e-3;
    Span newA = subSpan(a, 0.0, geom->ta);
    Span newB = subSpan(b, geom->tb, 1.0);
    spans[ia] = newA;
    spans[ib] = newB;
    if (geom->ta < 1.0 && approxLength(newA) <= eps) {
        consumed[ia] = 1;
    }
    if (geom->tb > 0.0 && approxLength(newB) <= eps) {
        consumed[ib] = 1;
    }
    // The arc goes before span `node` (after the last span for node 0); no insertion, so every
    // span index stays valid while more corners are applied.
    arcPos = node == 0 ? n : node;
    arcs[arcPos] = Span{geom->arc};
    return EditStatus::Ok;
}

/// Spans in order with each corner's arc before the span it precedes; spans reduced to nothing are
/// dropped. `track` (a slot index on entry) becomes the arc's index in the result.
std::vector<Span> assemble(const std::vector<Span>& spans, const std::vector<char>& consumed,
                           const std::vector<std::optional<Span>>& arcs, std::size_t* track = nullptr) {
    std::vector<Span> out;
    out.reserve(spans.size() + arcs.size());
    std::size_t newTrack = 0;
    for (std::size_t i = 0; i <= spans.size(); ++i) {
        if (arcs[i]) {
            if (track != nullptr && i == *track) {
                newTrack = out.size();
            }
            out.push_back(*arcs[i]);
        }
        if (i < spans.size() && !consumed[i]) {
            out.push_back(spans[i]);
        }
    }
    if (track != nullptr) {
        *track = newTrack;
    }
    return out;
}

bool radiusOk(double radius, const Tolerance& tol) {
    return std::isfinite(radius) && radius > tol.distance && tolValid(tol);
}

}  // namespace

// ------------------------------------------------------------------------------------ fillet

FilletResult filletCorner(const Contour& contour, std::size_t nodeIndex, double radius, FilletKind kind,
                          const FilletOptions& options, const Tolerance& tol) {
    FilletResult res;
    if (contour.isEmpty()) {
        res.status = EditStatus::EmptyContour;
        return res;
    }
    if (!radiusOk(radius, tol) || !contourFinite(contour)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    std::vector<Span> spans = contour.spans;
    std::vector<char> consumed(spans.size(), 0);
    std::vector<std::optional<Span>> arcs(spans.size() + 1);
    std::size_t arcPos = 0;
    res.status = applyCorner(spans, consumed, arcs, contour.closed, nodeIndex, radius, kind, options, tol,
                             res.appliedRadius, res.clamped, arcPos);
    if (res.status != EditStatus::Ok) {
        res.appliedRadius = 0.0;
        res.clamped = false;
        return res;
    }
    res.contour.spans = assemble(spans, consumed, arcs, &arcPos);
    res.contour.closed = contour.closed;
    res.arcSpan = arcPos;
    return res;
}

namespace {

FilletAllResult filletNodes(const Contour& contour, double radius, FilletKind kind, std::vector<std::size_t> nodes,
                            const FilletOptions& options, const Tolerance& tol) {
    FilletAllResult res;
    if (contour.isEmpty()) {
        res.status = EditStatus::EmptyContour;
        return res;
    }
    if (!radiusOk(radius, tol) || !contourFinite(contour)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    std::sort(nodes.begin(), nodes.end());
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
    std::vector<Span> spans = contour.spans;
    std::vector<char> consumed(spans.size(), 0);
    std::vector<std::optional<Span>> arcs(spans.size() + 1);
    // Highest node first, so a corner sees its lower neighbour untouched; the neighbour above it
    // is already trimmed.
    for (std::size_t k = nodes.size(); k-- > 0;) {
        CornerReport rep;
        rep.node = nodes[k];
        std::size_t arcPos = 0;
        rep.status = applyCorner(spans, consumed, arcs, contour.closed, nodes[k], radius, kind, options, tol,
                                 rep.appliedRadius, rep.clamped, arcPos);
        if (rep.status == EditStatus::Ok) {
            ++res.applied;
        } else {
            rep.appliedRadius = 0.0;
            rep.clamped = false;
        }
        res.corners.push_back(rep);
    }
    std::reverse(res.corners.begin(), res.corners.end());
    res.contour.spans = assemble(spans, consumed, arcs);
    res.contour.closed = contour.closed;
    return res;
}

}  // namespace

FilletAllResult filletAll(const Contour& contour, double radius, FilletKind kind, const FilletOptions& options,
                          const Tolerance& tol) {
    std::vector<std::size_t> nodes;
    const std::size_t n = contour.spans.size();
    for (std::size_t i = contour.closed ? 0 : 1; i < n; ++i) {
        nodes.push_back(i);
    }
    return filletNodes(contour, radius, kind, std::move(nodes), options, tol);
}

FilletAllResult filletAll(const Contour& contour, double radius, FilletKind kind, const std::vector<std::size_t>& nodes,
                          const FilletOptions& options, const Tolerance& tol) {
    return filletNodes(contour, radius, kind, nodes, options, tol);
}

// -------------------------------------------------------------------------------------- refit

namespace {

/// Merges neighbouring collinear lines and neighbouring arcs of one circle (same direction).
void mergeNeighbours(std::vector<Span>& spans, const Tolerance& tol) {
    const double eps = tol.distance * 0.01;
    std::vector<Span> out;
    out.reserve(spans.size());
    for (Span& s : spans) {
        if (!out.empty()) {
            Span& last = out.back();
            if (auto* l1 = std::get_if<LineSpan>(&last)) {
                if (const auto* l2 = std::get_if<LineSpan>(&s)) {
                    const Vec2 d1 = l1->end - l1->start;
                    const Vec2 d2 = l2->end - l2->start;
                    const double len1 = d1.length();
                    if (len1 > 0.0 && d2.lengthSquared() > 0.0 && d1.dot(d2) > 0.0 &&
                        distance(l1->end, l2->start) <= tol.distance &&
                        std::fabs(d1.cross(l2->end - l1->start)) / len1 <= eps &&
                        std::fabs(d1.cross(l2->start - l1->start)) / len1 <= eps) {
                        l1->end = l2->end;
                        continue;
                    }
                }
            } else if (auto* a1 = std::get_if<ArcSpan>(&last)) {
                if (const auto* a2 = std::get_if<ArcSpan>(&s)) {
                    if (a1->direction == a2->direction && distance(a1->centre, a2->centre) <= eps &&
                        std::fabs(a1->radius - a2->radius) <= eps &&
                        distance(a1->endPoint(), a2->startPoint()) <= tol.distance &&
                        std::fabs(a1->sweepAngle()) + std::fabs(a2->sweepAngle()) < kTwoPi - 1e-9) {
                        a1->endAngle = a2->endAngle;
                        continue;
                    }
                }
            }
        }
        out.push_back(std::move(s));
    }
    spans = std::move(out);
}

// ------------------------------------------------------------------------------------- trim

/// Closed boundary regions: polygonised spans in a box index for winding and distance queries.
class BoundarySet {
   public:
    BoundarySet(const std::vector<Contour>& contours, const Tolerance& tol, FillRule rule)
        : tol_(tol), rule_(rule), index_(collectBoxes(contours, tol)) {
        std::size_t k = 0;
        for (const Contour& c : contours) {
            // Vertices at span joints (and the closing joint) are shared exactly, so the winding
            // count never sees a sliver gap between two spans.
            const std::size_t first = polys_.size();
            for (const Span& s : c.spans) {
                spans_.push_back(s);
                std::vector<Vec2> pts;
                pts.push_back(polys_.size() > first ? polys_.back().back() : startPoint(s));
                polyInto(s, pts);
                polys_.push_back(std::move(pts));
                extent_.expand(boxes_[k]);
                ++k;
            }
            if (c.closed && polys_.size() > first) {
                polys_.back().back() = polys_[first].front();
            }
        }
        buildEdges();
    }

    std::size_t size() const { return spans_.size(); }
    const Span& span(std::size_t i) const { return spans_[i]; }
    std::size_t contourOf(std::size_t i) const { return owner_[i]; }

    template <class F>
    void query(const Box2& q, F&& f) {
        index_.query(q, f);
    }

    enum class Loc { Inside, Outside, On };

    Loc classify(Vec2 p) {
        if (spans_.empty() || extent_.isEmpty()) {
            return Loc::Outside;
        }
        const double d = tol_.distance;
        bool on = false;
        index_.query(Box2{Vec2{p.x - d, p.y - d}, Vec2{p.x + d, p.y + d}}, [&](std::size_t i) {
            if (!on && closestPoint(spans_[i], p).distance <= d) {
                on = true;
            }
        });
        if (on) {
            return Loc::On;
        }
        int winding = 0;
        stab(0, edges_.size(), p, winding);
        const bool inside = rule_ == FillRule::NonZero ? winding != 0 : (winding % 2) != 0;
        return inside ? Loc::Inside : Loc::Outside;
    }

   private:
    std::vector<Box2> collectBoxes(const std::vector<Contour>& contours, const Tolerance& tol) {
        for (std::size_t ci = 0; ci < contours.size(); ++ci) {
            for (const Span& s : contours[ci].spans) {
                boxes_.push_back(bounds(s).inflated(tol.distance));
                owner_.push_back(ci);
            }
        }
        return boxes_;
    }

    void polyInto(const Span& s, std::vector<Vec2>& pts) const {
        if (const auto* a = std::get_if<ArcSpan>(&s)) {
            const std::size_t n = a->segmentCountFor(tol_.chordal);
            if (n > 4096) {
                for (std::size_t i = 1; i <= 4096; ++i) {
                    pts.push_back(a->pointAt(static_cast<double>(i) / 4096.0));
                }
                return;
            }
        }
        polygoniseInto(s, tol_, pts);
    }

    struct Edge {
        double lo;  // y range [lo, hi)
        double hi;
        Vec2 a;
        Vec2 b;
    };

    // Edges sorted by lo; maxHi_[m] is the largest hi in the implicit subtree centred on m, so a
    // scanline query visits O(log n + edges crossing the scanline) edges.
    void buildEdges() {
        for (const std::vector<Vec2>& pts : polys_) {
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                const Vec2 a = pts[k];
                const Vec2 b = pts[k + 1];
                if (a.y != b.y) {
                    edges_.push_back({std::min(a.y, b.y), std::max(a.y, b.y), a, b});
                }
            }
        }
        std::stable_sort(edges_.begin(), edges_.end(), [](const Edge& x, const Edge& y) { return x.lo < y.lo; });
        maxHi_.assign(edges_.size(), 0.0);
        buildMax(0, edges_.size());
    }

    double buildMax(std::size_t l, std::size_t r) {
        if (l >= r) {
            return -std::numeric_limits<double>::infinity();
        }
        const std::size_t m = l + (r - l) / 2;
        maxHi_[m] = std::max({edges_[m].hi, buildMax(l, m), buildMax(m + 1, r)});
        return maxHi_[m];
    }

    void stab(std::size_t l, std::size_t r, Vec2 p, int& winding) const {
        if (l >= r) {
            return;
        }
        const std::size_t m = l + (r - l) / 2;
        if (maxHi_[m] <= p.y) {
            return;
        }
        stab(l, m, p, winding);
        const Edge& e = edges_[m];
        if (e.lo > p.y) {
            return;  // everything to the right starts above p
        }
        if (p.y < e.hi) {
            const double side = (e.b - e.a).cross(p - e.a);
            if (e.a.y <= p.y) {
                if (e.b.y > p.y && side > 0.0) {
                    ++winding;
                }
            } else if (e.b.y <= p.y && side < 0.0) {
                --winding;
            }
        }
        stab(m + 1, r, p, winding);
    }

    Tolerance tol_;
    FillRule rule_;
    std::vector<Edge> edges_;
    std::vector<double> maxHi_;
    std::vector<Box2> boxes_;
    std::vector<std::size_t> owner_;
    detail::BoxIndex index_;
    std::vector<Span> spans_;
    std::vector<std::vector<Vec2>> polys_;
    Box2 extent_;
};

struct Cut {
    double t{0.0};
    bool soft{true};  // tangent touch only: the span stays whole if both sides are kept
};

struct Piece {
    std::size_t span{0};
    double t0{0.0};
    double t1{1.0};
    bool softStart{false};
    bool kept{true};
};

std::vector<Cut> normaliseCuts(const Span& s, std::vector<Cut> cuts, const Tolerance& tol) {
    std::sort(cuts.begin(), cuts.end(), [](const Cut& a, const Cut& b) { return a.t < b.t; });
    std::vector<Cut> out;
    const Vec2 p0 = startPoint(s);
    const Vec2 p1 = endPoint(s);
    for (const Cut& c : cuts) {
        if (!(c.t > 0.0 && c.t < 1.0)) {
            continue;
        }
        const Vec2 p = pointAt(s, c.t);
        if (distance(p, p0) <= tol.distance || distance(p, p1) <= tol.distance) {
            continue;
        }
        if (!out.empty() && distance(p, pointAt(s, out.back().t)) <= tol.distance) {
            out.back().soft = out.back().soft && c.soft;
            continue;
        }
        out.push_back(c);
    }
    return out;
}

}  // namespace

TrimResult trimToBoundary(const std::vector<Contour>& contours, const std::vector<Contour>& boundaries, TrimSide side,
                          bool refitCurves, const Tolerance& tol, FillRule rule) {
    TrimResult res;
    if (!tolValid(tol)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    for (const Contour& b : boundaries) {
        if (!b.closed || b.isEmpty()) {
            res.status = EditStatus::NotClosed;
            return res;
        }
        if (!contourFinite(b)) {
            res.status = EditStatus::InvalidArgument;
            return res;
        }
    }
    for (const Contour& c : contours) {
        if (!contourFinite(c)) {
            res.status = EditStatus::InvalidArgument;
            return res;
        }
    }
    if (tooCostly({&contours, &boundaries}, nullptr, tol)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    BoundarySet set(boundaries, tol, rule);
    const auto keepLoc = [&](BoundarySet::Loc loc) {
        if (loc == BoundarySet::Loc::On) {
            return true;
        }
        return side == TrimSide::ClearInside ? loc == BoundarySet::Loc::Outside : loc == BoundarySet::Loc::Inside;
    };

    for (std::size_t ci = 0; ci < contours.size(); ++ci) {
        const Contour& c = contours[ci];
        if (c.isEmpty()) {
            continue;
        }
        std::vector<Piece> pieces;
        for (std::size_t si = 0; si < c.spans.size(); ++si) {
            const Span& s = c.spans[si];
            std::vector<Cut> cuts;
            if (set.size() > 0) {
                set.query(bounds(s).inflated(tol.distance), [&](std::size_t bi) {
                    const IntersectionResult r = intersect(s, set.span(bi), tol);
                    for (const Intersection& h : r.points) {
                        cuts.push_back({h.tA, h.tangent});
                    }
                    for (const Overlap& o : r.overlaps) {
                        cuts.push_back({o.tA0, false});
                        cuts.push_back({o.tA1, false});
                    }
                });
            }
            cuts = normaliseCuts(s, std::move(cuts), tol);
            double t0 = 0.0;
            bool soft = false;
            for (std::size_t k = 0; k <= cuts.size(); ++k) {
                const double t1 = k < cuts.size() ? cuts[k].t : 1.0;
                Piece p;
                p.span = si;
                p.t0 = t0;
                p.t1 = t1;
                p.softStart = k > 0 && soft;
                p.kept = keepLoc(set.classify(pointAt(s, 0.5 * (t0 + t1))));
                pieces.push_back(p);
                if (k < cuts.size()) {
                    soft = cuts[k].soft;
                    t0 = t1;
                }
            }
        }
        const bool allKept = std::all_of(pieces.begin(), pieces.end(), [](const Piece& p) { return p.kept; });
        if (allKept) {
            Contour out = c;
            if (refitCurves) {
                mergeNeighbours(out.spans, tol);
            }
            res.contours.push_back(std::move(out));
            res.sources.push_back(ci);
            continue;
        }
        // Order the pieces so that a closed contour starts right after a removed piece.
        std::vector<Piece> seq;
        if (c.closed) {
            std::size_t first = 0;
            while (pieces[first].kept) {
                ++first;
            }
            for (std::size_t k = 1; k <= pieces.size(); ++k) {
                seq.push_back(pieces[(first + k) % pieces.size()]);
            }
        } else {
            seq = pieces;
        }
        std::vector<Piece> run;
        auto flush = [&]() {
            if (run.empty()) {
                return;
            }
            Contour out;
            for (const Piece& p : run) {
                out.spans.push_back(subSpan(c.spans[p.span], p.t0, p.t1));
            }
            if (refitCurves) {
                mergeNeighbours(out.spans, tol);
            }
            res.contours.push_back(std::move(out));
            res.sources.push_back(ci);
            run.clear();
        };
        for (const Piece& p : seq) {
            if (!p.kept) {
                flush();
                continue;
            }
            if (!run.empty() && run.back().span == p.span && p.softStart) {
                run.back().t1 = p.t1;
            } else {
                run.push_back(p);
            }
        }
        flush();
    }
    return res;
}

// ---------------------------------------------------------------------------- trim at a point

namespace {

/// Position along a contour: span index + own parameter.
double posOf(std::size_t span, double t) {
    return static_cast<double>(span) + std::clamp(t, 0.0, 1.0);
}

Contour extractRange(const Contour& c, double a, double b) {
    Contour out;
    const std::size_t n = c.spans.size();
    constexpr double kMin = 1e-12;
    for (auto i = static_cast<std::size_t>(std::floor(a)); static_cast<double>(i) < b; ++i) {
        const double base = static_cast<double>(i);
        const double t0 = std::max(a - base, 0.0);
        const double t1 = std::min(b - base, 1.0);
        if (t1 - t0 < kMin) {
            continue;
        }
        out.spans.push_back(subSpan(c.spans[i % n], t0, t1));
    }
    return out;
}

}  // namespace

TrimAtResult trimSpansAt(const Contour& contour, Vec2 pick, const std::vector<Contour>& others,
                         const TrimAtOptions& options, const Tolerance& tol) {
    TrimAtResult res;
    if (contour.isEmpty()) {
        res.status = EditStatus::EmptyContour;
        return res;
    }
    if (!finite(pick) || !tolValid(tol) || !contourFinite(contour)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    for (const Contour& o : others) {
        if (!contourFinite(o)) {
            res.status = EditStatus::InvalidArgument;
            return res;
        }
    }
    if (tooCostly({&others}, &contour, tol)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    const std::size_t n = contour.spans.size();
    const double len = static_cast<double>(n);
    std::vector<double> cuts;
    auto addCut = [&](double pos) {
        if (contour.closed) {
            if (pos >= len) {
                pos -= len;
            }
        } else if (pos <= 1e-12 || pos >= len - 1e-12) {
            return;
        }
        cuts.push_back(pos);
    };
    {
        const ContourIntersections self = selfIntersections(contour, tol);
        for (const ContourIntersection& h : self.points) {
            addCut(posOf(h.spanA, h.hit.tA));
            addCut(posOf(h.spanB, h.hit.tB));
        }
        for (const ContourOverlap& o : self.overlaps) {
            addCut(posOf(o.spanA, o.overlap.tA0));
            addCut(posOf(o.spanA, o.overlap.tA1));
            addCut(posOf(o.spanB, o.overlap.tB0));
            addCut(posOf(o.spanB, o.overlap.tB1));
        }
    }
    for (const Contour& o : others) {
        if (o.isEmpty()) {
            continue;
        }
        const ContourIntersections hits = intersect(contour, o, tol);
        for (const ContourIntersection& h : hits.points) {
            addCut(posOf(h.spanA, h.hit.tA));
        }
        for (const ContourOverlap& ov : hits.overlaps) {
            addCut(posOf(ov.spanA, ov.overlap.tA0));
            addCut(posOf(ov.spanA, ov.overlap.tA1));
        }
    }
    std::sort(cuts.begin(), cuts.end());
    {
        std::vector<double> uniq;
        for (double c : cuts) {
            if (uniq.empty() || c - uniq.back() > 1e-9) {
                uniq.push_back(c);
            }
        }
        cuts = std::move(uniq);
    }
    const ContourClosestPoint cp = closestPoint(contour, pick);
    const double pos = posOf(cp.spanIndex, cp.t);
    constexpr double kEps = 1e-9;

    // Nearest cut at or before / strictly after the pick.
    std::optional<double> prev;
    std::optional<double> next;
    for (double c : cuts) {
        if (c <= pos + kEps) {
            prev = c;
        } else if (!next) {
            next = c;
        }
    }
    if (contour.closed) {
        if (cuts.size() < 2) {
            res.status = EditStatus::NothingToTrim;
            return res;
        }
        if (!prev) {
            prev = cuts.back() - len;
        }
        if (!next) {
            next = cuts.front() + len;
        }
        // Remaining: from next round to prev (one wrap).
        Contour rest = extractRange(contour, *next, *prev + len);
        if (rest.isEmpty()) {
            res.status = EditStatus::NothingToTrim;
            return res;
        }
        if (options.rejoin && distance(startPoint(rest.spans.front()), endPoint(rest.spans.back())) <= tol.distance &&
            approxLength(rest.spans) > 2.0 * tol.distance) {
            rest.closed = true;
        }
        res.contours.push_back(std::move(rest));
        return res;
    }
    if (!prev && !next) {
        res.status = EditStatus::NothingToTrim;
        return res;
    }
    Contour head;
    Contour tail;
    if (prev) {
        head = extractRange(contour, 0.0, *prev);
    }
    if (next) {
        tail = extractRange(contour, *next, len);
    }
    if (options.rejoin && !head.isEmpty() && !tail.isEmpty() &&
        distance(endPoint(head.spans.back()), startPoint(tail.spans.front())) <= tol.distance) {
        for (Span& s : tail.spans) {
            head.spans.push_back(std::move(s));
        }
        tail.spans.clear();
        if (distance(startPoint(head.spans.front()), endPoint(head.spans.back())) <= tol.distance &&
            approxLength(head.spans) > 2.0 * tol.distance) {
            head.closed = true;
        }
    }
    if (!head.isEmpty()) {
        res.contours.push_back(std::move(head));
    }
    if (!tail.isEmpty()) {
        res.contours.push_back(std::move(tail));
    }
    if (res.contours.empty()) {
        res.status = EditStatus::NothingToTrim;
    }
    return res;
}

// ---------------------------------------------------------------------------------- extend

ExtendResult extendSpan(const Contour& contour, ContourEnd end, const std::vector<Contour>& targets,
                        const Tolerance& tol) {
    ExtendResult res;
    if (contour.isEmpty()) {
        res.status = EditStatus::EmptyContour;
        return res;
    }
    if (!tolValid(tol) || !contourFinite(contour)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    for (const Contour& t : targets) {
        if (!contourFinite(t)) {
            res.status = EditStatus::InvalidArgument;
            return res;
        }
    }
    if (contour.closed) {
        res.status = EditStatus::ClosedContour;
        return res;
    }
    if (tooCostly({&targets}, &contour, tol)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    Contour work = end == ContourEnd::Start ? contour.reversed() : contour;
    Span& last = work.spans.back();
    const Vec2 tip = endPoint(last);
    const Vec2 dir = tangentAt(last, 1.0);
    if (!(approxLength(last) > tol.distance) || dir.lengthSquared() == 0.0) {
        res.status = EditStatus::DegenerateSpan;
        return res;
    }

    Box2 extent;
    extent.expand(tip);
    for (const Contour& t : targets) {
        extent.expand(t.bounds());
    }
    const double reach = 2.0 * std::max(distance(extent.min, extent.max), 1.0);

    // The curve the end travels along: a ray, or the rest of the circle for an arc.
    Span probe;
    double probeLen = reach;
    if (const auto* arc = std::get_if<ArcSpan>(&last)) {
        if (arc->isFullCircle()) {
            res.status = EditStatus::DegenerateSpan;
            return res;
        }
        ArcSpan rest = *arc;
        rest.startAngle = arc->endAngle;
        rest.endAngle = arc->startAngle;
        probe = rest;
        probeLen = rest.length();
    } else {
        probe = LineSpan{tip, tip + dir * reach};
    }
    const Box2 probeBox = bounds(probe).inflated(tol.distance);

    bool found = false;
    double bestAlong = std::numeric_limits<double>::infinity();
    Vec2 bestPoint;
    std::size_t bestC = 0;
    std::size_t bestS = 0;
    for (std::size_t ci = 0; ci < targets.size(); ++ci) {
        if (!targets[ci].bounds().inflated(tol.distance).intersects(probeBox)) {
            continue;
        }
        for (std::size_t si = 0; si < targets[ci].spans.size(); ++si) {
            const Span& ts = targets[ci].spans[si];
            if (!bounds(ts).inflated(tol.distance).intersects(probeBox)) {
                continue;
            }
            const IntersectionResult r = intersect(probe, ts, tol);
            for (const Intersection& h : r.points) {
                const double along = h.tA * probeLen;
                if (along > tol.distance && distance(h.point, tip) > tol.distance && along < bestAlong) {
                    bestAlong = along;
                    bestPoint = h.point;
                    bestC = ci;
                    bestS = si;
                    found = true;
                }
            }
        }
    }
    if (!found) {
        res.status = EditStatus::NoIntersection;
        return res;
    }
    if (auto* l = std::get_if<LineSpan>(&last)) {
        l->end = bestPoint;
    } else if (auto* a = std::get_if<ArcSpan>(&last)) {
        a->endAngle = (bestPoint - a->centre).angle();
    } else {
        work.spans.push_back(LineSpan{tip, bestPoint});
    }
    res.contour = end == ContourEnd::Start ? work.reversed() : std::move(work);
    res.point = bestPoint;
    res.targetContour = bestC;
    res.targetSpan = bestS;
    res.distance = bestAlong;
    return res;
}

// ----------------------------------------------------------------------------- join / close

namespace {

struct Endpoint {
    std::size_t contour;
    bool atEnd;
    Vec2 p;
};

class EndpointGrid {
   public:
    EndpointGrid(double cell) : cell_(cell) {}

    void add(const Endpoint& e) { cells_[key(cellOf(e.p.x), cellOf(e.p.y))].push_back(e); }

    template <class F>
    void near(Vec2 p, F&& f) const {
        const std::int64_t cx = cellOf(p.x);
        const std::int64_t cy = cellOf(p.y);
        for (std::int64_t dx = -1; dx <= 1; ++dx) {
            for (std::int64_t dy = -1; dy <= 1; ++dy) {
                const auto it = cells_.find(key(cx + dx, cy + dy));
                if (it == cells_.end()) {
                    continue;
                }
                for (const Endpoint& e : it->second) {
                    f(e);
                }
            }
        }
    }

   private:
    std::int64_t cellOf(double v) const { return static_cast<std::int64_t>(std::floor(v / cell_)); }
    static std::uint64_t key(std::int64_t x, std::int64_t y) {
        return (static_cast<std::uint64_t>(x) * 73856093ULL) ^ (static_cast<std::uint64_t>(y) * 19349663ULL);
    }

    double cell_;
    std::unordered_map<std::uint64_t, std::vector<Endpoint>> cells_;
};

}  // namespace

JoinResult joinContours(const std::vector<Contour>& contours, const Tolerance& tol) {
    JoinResult res;
    if (!tolValid(tol)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    const std::size_t n = contours.size();
    std::vector<char> joinable(n, 0);
    double maxCoord = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Contour& c = contours[i];
        if (c.isEmpty() || c.closed || !contourFinite(c)) {
            continue;
        }
        const Vec2 a = c.startPoint();
        const Vec2 b = c.endPoint();
        joinable[i] = 1;
        maxCoord = std::max({maxCoord, std::fabs(a.x), std::fabs(a.y), std::fabs(b.x), std::fabs(b.y)});
    }
    const double cell = std::max({tol.distance, maxCoord * 1e-12, 1e-12});
    EndpointGrid grid(cell);
    for (std::size_t i = 0; i < n; ++i) {
        if (joinable[i]) {
            grid.add({i, false, contours[i].startPoint()});
            grid.add({i, true, contours[i].endPoint()});
        }
    }
    std::vector<char> used(n, 0);

    auto findPartner = [&](Vec2 at, Endpoint& best) {
        bool any = false;
        double bestD = 0.0;
        grid.near(at, [&](const Endpoint& e) {
            if (used[e.contour]) {
                return;
            }
            const double d = distance(e.p, at);
            if (d > tol.distance) {
                return;
            }
            const bool better = !any || d < bestD ||
                                (d == bestD && (e.contour < best.contour ||
                                                (e.contour == best.contour && !e.atEnd && best.atEnd)));
            if (better) {
                any = true;
                bestD = d;
                best = e;
            }
        });
        return any;
    };

    for (std::size_t i = 0; i < n; ++i) {
        if (used[i]) {
            continue;
        }
        used[i] = 1;
        if (!joinable[i]) {
            res.contours.push_back(contours[i]);
            res.sources.push_back({i});
            continue;
        }
        std::deque<std::pair<std::size_t, bool>> chain;  // (contour, reversed)
        chain.emplace_back(i, false);
        Vec2 head = contours[i].startPoint();
        Vec2 tail = contours[i].endPoint();
        Endpoint e{};
        while (findPartner(tail, e)) {
            used[e.contour] = 1;
            if (!e.atEnd) {
                chain.emplace_back(e.contour, false);
                tail = contours[e.contour].endPoint();
            } else {
                chain.emplace_back(e.contour, true);
                tail = contours[e.contour].startPoint();
            }
        }
        while (findPartner(head, e)) {
            used[e.contour] = 1;
            if (e.atEnd) {
                chain.emplace_front(e.contour, false);
                head = contours[e.contour].startPoint();
            } else {
                chain.emplace_front(e.contour, true);
                head = contours[e.contour].endPoint();
            }
        }
        Contour out;
        std::vector<std::size_t> src;
        for (const auto& [ci, rev] : chain) {
            src.push_back(ci);
            if (rev) {
                const Contour r = contours[ci].reversed();
                out.spans.insert(out.spans.end(), r.spans.begin(), r.spans.end());
            } else {
                out.spans.insert(out.spans.end(), contours[ci].spans.begin(), contours[ci].spans.end());
            }
        }
        if (distance(head, tail) <= tol.distance && approxLength(out.spans) > 2.0 * tol.distance) {
            out.closed = true;
        }
        if (src.size() == 1 && !out.closed) {
            res.unmatched.push_back(res.contours.size());
        }
        res.contours.push_back(std::move(out));
        res.sources.push_back(std::move(src));
    }
    return res;
}

CloseResult closeContour(const Contour& contour, const Tolerance& tol) {
    CloseResult res;
    if (contour.isEmpty()) {
        res.status = EditStatus::EmptyContour;
        return res;
    }
    if (!tolValid(tol) || !contourFinite(contour)) {
        res.status = EditStatus::InvalidArgument;
        return res;
    }
    res.contour = contour;
    if (contour.closed) {
        res.action = CloseAction::AlreadyClosed;
        return res;
    }
    const Vec2 a = contour.startPoint();
    const Vec2 b = contour.endPoint();
    res.contour.closed = true;
    if (distance(a, b) <= tol.distance) {
        res.action = CloseAction::Snapped;
        Span& last = res.contour.spans.back();
        Span& first = res.contour.spans.front();
        if (auto* l = std::get_if<LineSpan>(&last)) {
            l->end = a;
        } else if (auto* bz = std::get_if<BezierSpan>(&last)) {
            bz->p3 = a;
        } else if (auto* l0 = std::get_if<LineSpan>(&first)) {
            l0->start = b;
        } else if (auto* bz0 = std::get_if<BezierSpan>(&first)) {
            bz0->p0 = b;
        }
        return res;
    }
    res.action = CloseAction::LineAdded;
    res.contour.spans.push_back(LineSpan{b, a});
    return res;
}

}  // namespace carvetto::geometry
