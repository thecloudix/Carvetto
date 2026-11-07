#include "carvetto/viewmodel/snapping.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <variant>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"

namespace carvetto::viewmodel {

using document::ObjectId;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Span;
using geometry::Vec2;

namespace {

constexpr double kMergeEpsMm = 1e-7;
constexpr int kRootSamples = 32;

// ----------------------------------------------------------------------- visibility

bool layerPickable(const document::Document& doc, document::LayerId id, bool includeLocked) {
    const document::Layer* layer = doc.findLayer(id);
    return layer != nullptr && layer->visible && (includeLocked || !layer->locked);
}

bool chainSnappable(const document::Document& doc, const document::Object& obj, bool includeLocked) {
    const document::Object* cur = &obj;
    for (std::size_t guard = 0; cur != nullptr && guard <= doc.objectCount(); ++guard) {
        if (!layerPickable(doc, cur->layer, includeLocked)) {
            return false;
        }
        if (!cur->parent.has_value()) {
            return true;
        }
        cur = doc.findObject(*cur->parent);
    }
    return false;
}

// ----------------------------------------------------------------------- span helpers

/// Arc-length integral of |B'| over [t0,t1] by 3-point Gauss-Legendre.
double speedIntegral(const BezierSpan& b, double t0, double t1) {
    constexpr double kNode = 0.7745966692414834;  // sqrt(3/5)
    const double mid = 0.5 * (t0 + t1);
    const double half = 0.5 * (t1 - t0);
    const double s0 = b.derivativeAt(mid).length();
    const double s1 = b.derivativeAt(mid - half * kNode).length();
    const double s2 = b.derivativeAt(mid + half * kNode).length();
    return half * (8.0 / 9.0 * s0 + 5.0 / 9.0 * (s1 + s2));
}

/// Point halfway along the arc length of a Bezier.
Vec2 bezierMidpoint(const BezierSpan& b) {
    constexpr int kPieces = 64;
    std::array<double, kPieces> piece{};
    double total = 0.0;
    for (int i = 0; i < kPieces; ++i) {
        piece[static_cast<std::size_t>(i)] = speedIntegral(b, static_cast<double>(i) / kPieces,
                                                           static_cast<double>(i + 1) / kPieces);
        total += piece[static_cast<std::size_t>(i)];
    }
    const double target = 0.5 * total;
    double acc = 0.0;
    int k = 0;
    while (k < kPieces - 1 && acc + piece[static_cast<std::size_t>(k)] < target) {
        acc += piece[static_cast<std::size_t>(k)];
        ++k;
    }
    double lo = static_cast<double>(k) / kPieces;
    double hi = static_cast<double>(k + 1) / kPieces;
    for (int it = 0; it < 60; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (acc + speedIntegral(b, static_cast<double>(k) / kPieces, mid) < target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return b.pointAt(0.5 * (lo + hi));
}

/// Zero-length span (a point); cheap, unlike `geometry::length` on Beziers.
bool isDegenerateSpan(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return l->start == l->end;
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        return !(a->radius > 0.0);
    }
    const auto& b = std::get<BezierSpan>(s);
    return b.p0 == b.p1 && b.p1 == b.p2 && b.p2 == b.p3;
}

Vec2 midpointOf(const Span& s) {
    if (const auto* b = std::get_if<BezierSpan>(&s)) {
        return bezierMidpoint(*b);
    }
    return geometry::pointAt(s, 0.5);
}

/// Roots of f on [0,1] by sampling and bisection; f must be continuous.
template <class F>
std::vector<double> sampledRoots(F&& f) {
    std::vector<double> roots;
    double prevT = 0.0;
    double prevV = f(0.0);
    if (prevV == 0.0) {
        roots.push_back(0.0);
    }
    for (int i = 1; i <= kRootSamples; ++i) {
        const double t = static_cast<double>(i) / kRootSamples;
        const double v = f(t);
        if (v == 0.0) {
            roots.push_back(t);
        } else if (prevV != 0.0 && ((prevV < 0.0) != (v < 0.0))) {
            double lo = prevT;
            double hi = t;
            bool loNeg = prevV < 0.0;
            for (int it = 0; it < 60; ++it) {
                const double mid = 0.5 * (lo + hi);
                const double mv = f(mid);
                if (mv == 0.0) {
                    lo = hi = mid;
                    break;
                }
                if ((mv < 0.0) == loNeg) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            roots.push_back(0.5 * (lo + hi));
        }
        prevT = t;
        prevV = v;
    }
    return roots;
}

/// Points Q of `s` with (Q - p) perpendicular to the tangent at Q.
void perpendicularFeet(const Span& s, Vec2 p, std::vector<Vec2>& out) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        const Vec2 d = l->end - l->start;
        const double len2 = d.lengthSquared();
        if (len2 <= 0.0) {
            return;
        }
        const double t = (p - l->start).dot(d) / len2;
        if (t >= 0.0 && t <= 1.0) {
            out.push_back(l->pointAt(t));
        }
    } else if (const auto* a = std::get_if<ArcSpan>(&s)) {
        const Vec2 dir = p - a->centre;
        if (dir.lengthSquared() <= 0.0) {
            return;
        }
        const double base = dir.angle();
        for (const double ang : {base, base + geometry::kPi}) {
            if (a->containsAngle(geometry::normalizeAngle(ang))) {
                out.push_back(a->centre + Vec2::fromAngle(ang, a->radius));
            }
        }
    } else {
        const auto& b = std::get<BezierSpan>(s);
        for (const double t : sampledRoots([&](double u) { return (b.pointAt(u) - p).dot(b.derivativeAt(u)); })) {
            out.push_back(b.pointAt(t));
        }
    }
}

/// Points Q of an arc or Bezier such that the line p-Q touches the curve at Q.
void tangentPoints(const Span& s, Vec2 p, std::vector<Vec2>& out) {
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        const Vec2 dir = p - a->centre;
        const double d = dir.length();
        if (d <= a->radius) {
            return;
        }
        const double off = std::acos(a->radius / d);
        const double base = dir.angle();
        for (const double ang : {base + off, base - off}) {
            if (a->containsAngle(geometry::normalizeAngle(ang))) {
                out.push_back(a->centre + Vec2::fromAngle(ang, a->radius));
            }
        }
    } else if (const auto* b = std::get_if<BezierSpan>(&s)) {
        for (const double t : sampledRoots([&](double u) { return (b->pointAt(u) - p).cross(b->derivativeAt(u)); })) {
            out.push_back(b->pointAt(t));
        }
    }
}

/// Junction points shared by the spans `i` and `j` of one contour (none unless adjacent).
std::vector<Vec2> sharedJunctions(const Contour& c, std::size_t i, std::size_t j) {
    std::vector<Vec2> out;
    const std::size_t n = c.spans.size();
    if (i > j) {
        std::swap(i, j);
    }
    if (j == i + 1) {
        out.push_back(geometry::endPoint(c.spans[i]));
    }
    if (c.closed && i == 0 && j == n - 1) {
        out.push_back(geometry::endPoint(c.spans[j]));
    }
    return out;
}

// ----------------------------------------------------------------------- candidates

struct Candidate {
    SnapKind kind{SnapKind::Endpoint};
    Vec2 point{};
    std::vector<ObjectId> objects;
    int guide{-1};
    int guide2{-1};
    double distancePx{0.0};
};

std::int64_t quantised(double v) {
    return static_cast<std::int64_t>(std::llround(v * 1e6));
}

bool betterThan(const Candidate& a, const Candidate& b) {
    if (a.kind != b.kind) {
        return a.kind < b.kind;
    }
    const std::int64_t da = quantised(a.distancePx);
    const std::int64_t db = quantised(b.distancePx);
    if (da != db) {
        return da < db;
    }
    if (a.objects != b.objects) {
        return a.objects < b.objects;
    }
    if (a.point.x != b.point.x) {
        return a.point.x < b.point.x;
    }
    if (a.point.y != b.point.y) {
        return a.point.y < b.point.y;
    }
    if (a.guide != b.guide) {
        return a.guide < b.guide;
    }
    return a.guide2 < b.guide2;
}

void mergeObjects(std::vector<ObjectId>& into, const std::vector<ObjectId>& from) {
    into.insert(into.end(), from.begin(), from.end());
    std::sort(into.begin(), into.end());
    into.erase(std::unique(into.begin(), into.end()), into.end());
}

/// Merges candidates of one kind at the same position (and the same guides).
std::vector<Candidate> mergeDuplicates(std::vector<Candidate> in) {
    std::sort(in.begin(), in.end(), [](const Candidate& a, const Candidate& b) {
        if (a.kind != b.kind) {
            return a.kind < b.kind;
        }
        if (a.point.x != b.point.x) {
            return a.point.x < b.point.x;
        }
        return a.point.y < b.point.y;
    });
    std::vector<Candidate> out;
    for (Candidate& c : in) {
        bool merged = false;
        for (std::size_t k = out.size(); k-- > 0;) {
            Candidate& o = out[k];
            if (o.kind != c.kind || c.point.x - o.point.x > kMergeEpsMm) {
                break;
            }
            if (std::fabs(c.point.y - o.point.y) <= kMergeEpsMm && o.guide == c.guide && o.guide2 == c.guide2) {
                mergeObjects(o.objects, c.objects);
                o.distancePx = std::min(o.distancePx, c.distancePx);
                merged = true;
                break;
            }
        }
        if (!merged) {
            out.push_back(std::move(c));
        }
    }
    return out;
}

struct NearSpan {
    std::size_t entry{0};
    double distanceMm{0.0};
};

}  // namespace

// ----------------------------------------------------------------------- index

SnapIndex::SnapIndex(const document::Document& doc, bool includeLocked) {
    std::vector<Box2> spanBoxes;
    std::vector<Box2> pointBoxes;
    auto addPoint = [&](SnapKind kind, Vec2 p, ObjectId object) {
        points_.push_back({kind, p, object});
        pointBoxes.push_back(Box2{p, p});
    };

    for (const auto& [id, obj] : doc.objects()) {
        const document::VectorObject* vec = obj.asVector();
        if (vec == nullptr || obj.sheet != doc.activeSheet() || !chainSnappable(doc, obj, includeLocked)) {
            continue;
        }
        for (const Contour& contour : vec->contours) {
            for (std::size_t i = 0; i < contour.spans.size(); ++i) {
                const Span& s = contour.spans[i];
                if (isDegenerateSpan(s)) {
                    continue;
                }
                spans_.push_back({id, &contour, i});
                spanBoxes.push_back(geometry::bounds(s));

                addPoint(SnapKind::Endpoint, geometry::startPoint(s), id);
                if (i + 1 == contour.spans.size() ||
                    geometry::endPoint(s) != geometry::startPoint(contour.spans[i + 1])) {
                    addPoint(SnapKind::Endpoint, geometry::endPoint(s), id);
                }
                addPoint(SnapKind::Midpoint, midpointOf(s), id);
                if (const auto* a = std::get_if<ArcSpan>(&s)) {
                    addPoint(SnapKind::Centre, a->centre, id);
                    const std::array<Vec2, 4> quads{Vec2{a->centre.x + a->radius, a->centre.y},
                                                    Vec2{a->centre.x, a->centre.y + a->radius},
                                                    Vec2{a->centre.x - a->radius, a->centre.y},
                                                    Vec2{a->centre.x, a->centre.y - a->radius}};
                    for (std::size_t q = 0; q < 4; ++q) {
                        if (a->containsAngle(static_cast<double>(q) * geometry::kPi * 0.5)) {
                            addPoint(SnapKind::Quadrant, quads[q], id);
                        }
                    }
                }
            }
        }
        std::visit(
            [&](const auto& p) {
                using P = std::decay_t<decltype(p)>;
                if constexpr (std::is_same_v<P, document::RectangleParams> ||
                              std::is_same_v<P, document::CircleParams> ||
                              std::is_same_v<P, document::PolygonParams> ||
                              std::is_same_v<P, document::StarParams>) {
                    addPoint(SnapKind::Centre, p.centreMm, id);
                } else if constexpr (std::is_same_v<P, document::EllipseParams>) {
                    addPoint(SnapKind::Centre, p.centreMm, id);
                    const Vec2 ax = Vec2::fromAngle(p.angle, p.widthMm * 0.5);
                    const Vec2 ay = Vec2::fromAngle(p.angle + geometry::kPi * 0.5, p.heightMm * 0.5);
                    addPoint(SnapKind::Quadrant, p.centreMm + ax, id);
                    addPoint(SnapKind::Quadrant, p.centreMm + ay, id);
                    addPoint(SnapKind::Quadrant, p.centreMm - ax, id);
                    addPoint(SnapKind::Quadrant, p.centreMm - ay, id);
                }
            },
            vec->params);
    }
    spanIndex_ = toolpath::BoxIndex(std::move(spanBoxes));
    pointIndex_ = toolpath::BoxIndex(std::move(pointBoxes));
}

std::vector<Snap> SnapIndex::candidates(const ViewTransform& view, Vec2 cursorPx, const SnapSettings& settings,
                                        const SnapQuery& query) const {
    std::vector<Snap> result;
    const double scale = view.scale();
    if (!(settings.tolerancePx >= 0.0) || !(scale > 0.0)) {
        return result;
    }
    const Vec2 cursor = view.screenToWorld(cursorPx);
    const double tolMm = settings.tolerancePx / scale;
    const Box2 window{cursor - Vec2{tolMm, tolMm}, cursor + Vec2{tolMm, tolMm}};
    auto within = [&](Vec2 p) { return geometry::distance(p, cursor) * scale <= settings.tolerancePx; };
    auto excluded = [&](ObjectId id) {
        return std::find(query.exclude.begin(), query.exclude.end(), id) != query.exclude.end();
    };

    std::vector<Candidate> found;
    auto add = [&](SnapKind kind, Vec2 p, std::vector<ObjectId> objects, int guide = -1, int guide2 = -1) {
        Candidate c;
        c.kind = kind;
        c.point = p;
        c.objects = std::move(objects);
        c.guide = guide;
        c.guide2 = guide2;
        c.distancePx = geometry::distance(p, cursor) * scale;
        found.push_back(std::move(c));
    };

    // Static points.
    for (const std::size_t i : pointIndex_.query(window)) {
        const PointEntry& pe = points_[i];
        if (settings.isEnabled(pe.kind) && !excluded(pe.object) && within(pe.point)) {
            add(pe.kind, pe.point, {pe.object});
        }
    }

    // Spans near the cursor, for the curve-based kinds.
    const bool needSpans = settings.isEnabled(SnapKind::Nearest) || settings.isEnabled(SnapKind::Intersection) ||
                           (query.pendingStartMm.has_value() && (settings.isEnabled(SnapKind::Perpendicular) ||
                                                                 settings.isEnabled(SnapKind::Tangent)));
    if (needSpans) {
        std::vector<NearSpan> closeSpans;
        for (const std::size_t i : spanIndex_.query(window)) {
            const SpanEntry& se = spans_[i];
            if (excluded(se.object)) {
                continue;
            }
            const geometry::ClosestPoint cp = geometry::closestPoint(se.contour->spans[se.spanIndex], cursor);
            if (cp.distance * scale <= settings.tolerancePx) {
                closeSpans.push_back({i, cp.distance});
            }
        }
        if (closeSpans.size() > kMaxNearSpans) {
            std::stable_sort(closeSpans.begin(), closeSpans.end(),
                             [](const NearSpan& a, const NearSpan& b) { return a.distanceMm < b.distanceMm; });
            closeSpans.resize(kMaxNearSpans);
            std::sort(closeSpans.begin(), closeSpans.end(), [](const NearSpan& a, const NearSpan& b) { return a.entry < b.entry; });
        }

        if (settings.isEnabled(SnapKind::Nearest)) {
            for (const NearSpan& ns : closeSpans) {
                const SpanEntry& se = spans_[ns.entry];
                const geometry::ClosestPoint cp = geometry::closestPoint(se.contour->spans[se.spanIndex], cursor);
                add(SnapKind::Nearest, cp.point, {se.object});
            }
        }

        if (settings.isEnabled(SnapKind::Intersection)) {
            // Pairwise search on the `kMaxPairSpans` spans nearest to the cursor (T-0204 bound: the
            // work is at most kMaxPairSpans^2 / 2 span pairs however dense the drawing is). A pair
            // whose boxes overlap only outside the cursor window cannot produce a snap and is
            // skipped without loss.
            std::vector<NearSpan> pairSpans = closeSpans;
            if (pairSpans.size() > kMaxPairSpans) {
                std::stable_sort(pairSpans.begin(), pairSpans.end(), [](const NearSpan& a, const NearSpan& b) {
                    return a.distanceMm < b.distanceMm;
                });
                pairSpans.resize(kMaxPairSpans);
                std::sort(pairSpans.begin(), pairSpans.end(),
                          [](const NearSpan& a, const NearSpan& b) { return a.entry < b.entry; });
            }
            const Box2 clip = window.inflated(settings.tolerance.distance);
            for (std::size_t a = 0; a < pairSpans.size(); ++a) {
                const SpanEntry& ea = spans_[pairSpans[a].entry];
                const Span& sa = ea.contour->spans[ea.spanIndex];
                const Box2 ba = geometry::bounds(sa).inflated(settings.tolerance.distance);
                if (!ba.intersects(clip)) {
                    continue;
                }
                for (std::size_t b = a + 1; b < pairSpans.size(); ++b) {
                    const SpanEntry& eb = spans_[pairSpans[b].entry];
                    const Span& sb = eb.contour->spans[eb.spanIndex];
                    const Box2 bb = geometry::bounds(sb);
                    if (!ba.intersects(bb)) {
                        continue;
                    }
                    // Any crossing lies in both boxes; if that common box misses the window, so does it.
                    const Box2 common{Vec2{std::max(ba.min.x, bb.min.x), std::max(ba.min.y, bb.min.y)},
                                      Vec2{std::min(ba.max.x, bb.max.x), std::min(ba.max.y, bb.max.y)}};
                    if (!common.intersects(clip)) {
                        continue;
                    }
                    std::vector<Vec2> junctions;
                    if (ea.contour == eb.contour) {
                        junctions = sharedJunctions(*ea.contour, ea.spanIndex, eb.spanIndex);
                    }
                    const geometry::IntersectionResult r = geometry::intersect(sa, sb, settings.tolerance);
                    for (const geometry::Intersection& hit : r.points) {
                        if (!within(hit.point)) {
                            continue;
                        }
                        bool atJunction = false;
                        for (const Vec2& j : junctions) {
                            atJunction = atJunction || geometry::distance(j, hit.point) <= settings.tolerance.distance;
                        }
                        if (atJunction) {
                            continue;
                        }
                        std::vector<ObjectId> ids{ea.object};
                        mergeObjects(ids, {eb.object});
                        add(SnapKind::Intersection, hit.point, std::move(ids));
                    }
                }
            }
        }

        if (query.pendingStartMm.has_value()) {
            const Vec2 start = *query.pendingStartMm;
            std::vector<Vec2> pts;
            for (const SnapKind kind : {SnapKind::Perpendicular, SnapKind::Tangent}) {
                if (!settings.isEnabled(kind)) {
                    continue;
                }
                for (const NearSpan& ns : closeSpans) {
                    const SpanEntry& se = spans_[ns.entry];
                    pts.clear();
                    if (kind == SnapKind::Perpendicular) {
                        perpendicularFeet(se.contour->spans[se.spanIndex], start, pts);
                    } else {
                        tangentPoints(se.contour->spans[se.spanIndex], start, pts);
                    }
                    for (const Vec2& q : pts) {
                        if (geometry::distance(q, start) > settings.tolerance.distance && within(q)) {
                            add(kind, q, {se.object});
                        }
                    }
                }
            }
        }
    }

    // Guides.
    if (settings.isEnabled(SnapKind::Guide)) {
        const std::size_t n = settings.guides.size();
        for (std::size_t g = 0; g < n; ++g) {
            const Guide& gd = settings.guides[g];
            const double len2 = gd.direction.lengthSquared();
            if (!(len2 > 0.0)) {
                continue;
            }
            const double t = (cursor - gd.pointMm).dot(gd.direction) / len2;
            const Vec2 foot = gd.pointMm + gd.direction * t;
            if (within(foot)) {
                add(SnapKind::Guide, foot, {}, static_cast<int>(g));
            }
            for (std::size_t h = g + 1; h < n; ++h) {
                const Guide& go = settings.guides[h];
                const double den = gd.direction.cross(go.direction);
                if (!(len2 > 0.0) || go.direction.lengthSquared() <= 0.0 || den == 0.0) {
                    continue;
                }
                const double u = (go.pointMm - gd.pointMm).cross(go.direction) / den;
                const Vec2 x = gd.pointMm + gd.direction * u;
                if (std::isfinite(x.x) && std::isfinite(x.y) && within(x)) {
                    add(SnapKind::Guide, x, {}, static_cast<int>(g), static_cast<int>(h));
                }
            }
        }
    }

    // Grid.
    if (settings.isEnabled(SnapKind::Grid) && settings.gridSpacingMm > 0.0) {
        const double s = settings.gridSpacingMm;
        const double gx = std::round((cursor.x - settings.gridOriginMm.x) / s);
        const double gy = std::round((cursor.y - settings.gridOriginMm.y) / s);
        if (std::fabs(gx) < 1e15 && std::fabs(gy) < 1e15) {
            const Vec2 node{settings.gridOriginMm.x + gx * s, settings.gridOriginMm.y + gy * s};
            if (within(node)) {
                add(SnapKind::Grid, node, {});
            }
        }
    }

    std::vector<Candidate> merged = mergeDuplicates(std::move(found));
    std::sort(merged.begin(), merged.end(), betterThan);
    result.reserve(merged.size());
    for (Candidate& c : merged) {
        Snap s;
        s.kind = c.kind;
        s.pointMm = c.point;
        s.distancePx = c.distancePx;
        s.objects = std::move(c.objects);
        s.guide = c.guide;
        s.guide2 = c.guide2;
        result.push_back(std::move(s));
    }
    return result;
}

std::optional<Snap> SnapIndex::best(const ViewTransform& view, Vec2 cursorPx, const SnapSettings& settings,
                                    const SnapQuery& query) const {
    std::vector<Snap> all = candidates(view, cursorPx, settings, query);
    if (all.empty()) {
        return std::nullopt;
    }
    return std::move(all.front());
}

std::optional<Snap> snapAt(const document::Document& doc, const ViewTransform& view, Vec2 cursorPx,
                           const SnapSettings& settings, const SnapQuery& query) {
    return SnapIndex(doc).best(view, cursorPx, settings, query);
}

}  // namespace carvetto::viewmodel
