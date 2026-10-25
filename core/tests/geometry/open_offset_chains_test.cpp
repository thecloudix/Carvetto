// T-0004: offsetOpenContour returns every chain; no chain intersects itself; coverage of the
// exact offset locus; cost for dense input.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/transform2.hpp"

using namespace carvetto::geometry;

// Trials per generator and distance in the generative test; raise it locally for soak runs.
#ifndef CARVETTO_OFFSET_TRIALS
#define CARVETTO_OFFSET_TRIALS 8
#endif

namespace {

const Tolerance kTol{};  // distance 0.001, chordal 0.01

const JoinType kJoins[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};
const OffsetSide kSides[] = {OffsetSide::Left, OffsetSide::Right};
constexpr double kMitre = 2.0;
constexpr double kDeg = kPi / 180.0;

// ---------------------------------------------------------------------------------------------
// Exact geometry on line and arc spans, written independently of the implementation.
// ---------------------------------------------------------------------------------------------

double distanceToSpan(Vec2 p, const Span& s) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 ab = line->end - line->start;
        const double len2 = ab.lengthSquared();
        const double t = len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0;
        return distance(p, line->start + ab * t);
    }
    const auto& arc = std::get<ArcSpan>(s);
    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
        return std::fabs(distance(p, arc.centre) - arc.radius);
    }
    return std::min(distance(p, arc.startPoint()), distance(p, arc.endPoint()));
}

double distanceToSpans(Vec2 p, const std::vector<Span>& spans) {
    double best = 1e300;
    for (const Span& s : spans) {
        best = std::min(best, distanceToSpan(p, s));
    }
    return best;
}

double distanceToChains(Vec2 p, const std::vector<Contour>& chains) {
    double best = 1e300;
    for (const Contour& c : chains) {
        best = std::min(best, distanceToSpans(p, c.spans));
    }
    return best;
}

double distanceToSpanEnds(Vec2 p, const std::vector<Contour>& chains) {
    double best = 1e300;
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            best = std::min({best, distance(p, startPoint(s)), distance(p, endPoint(s))});
        }
    }
    return best;
}

// Length of the stretch of `s` around parameter t that stays at least d from the input,
// measured in steps of a tenth of the tolerance and capped at twice the tolerance.
double validExtent(const Span& s, double t, const Contour& input, double d) {
    const double step = 0.1 * kTol.distance / length(s);
    double extent = 0.0;
    for (const double dir : {-1.0, 1.0}) {
        for (int k = 1; k <= 20; ++k) {
            const double u = t + dir * step * k;
            if (u < 0.0 || u > 1.0 ||
                distanceToSpans(pointAt(s, u), input.spans) < d - 1e-9 * std::max(1.0, d)) {
                break;
            }
            extent += 0.1 * kTol.distance;
        }
    }
    return extent;
}

// Points on the carriers of two spans (infinite line / full circle) where they meet.
std::vector<Vec2> carrierMeets(const Span& a, const Span& b) {
    std::vector<Vec2> out;
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    if (la && lb) {
        const Vec2 u = la->end - la->start;
        const Vec2 v = lb->end - lb->start;
        const double den = u.cross(v);
        if (std::fabs(den) > 1e-14 * u.length() * v.length()) {
            out.push_back(la->start + u * ((lb->start - la->start).cross(v) / den));
        }
        return out;
    }
    if (la || lb) {
        const LineSpan& line = la ? *la : *lb;
        const ArcSpan& arc = la ? std::get<ArcSpan>(b) : std::get<ArcSpan>(a);
        const Vec2 u = (line.end - line.start).normalized();
        const Vec2 w = line.start - arc.centre;
        const double along = -w.dot(u);
        const Vec2 foot = line.start + u * along;
        const double h2 = arc.radius * arc.radius - (foot - arc.centre).lengthSquared();
        if (h2 < -1e-12 * arc.radius * arc.radius) {
            return out;
        }
        const double h = std::sqrt(std::max(0.0, h2));
        out.push_back(foot - u * h);
        if (h > 0.0) {
            out.push_back(foot + u * h);
        }
        return out;
    }
    const auto& ca = std::get<ArcSpan>(a);
    const auto& cb = std::get<ArcSpan>(b);
    const Vec2 delta = cb.centre - ca.centre;
    const double dist = delta.length();
    if (dist < 1e-12 || dist > ca.radius + cb.radius + 1e-12 ||
        dist < std::fabs(ca.radius - cb.radius) - 1e-12) {
        return out;
    }
    const Vec2 u = delta / dist;
    const double along = (ca.radius * ca.radius - cb.radius * cb.radius + dist * dist) / (2.0 * dist);
    const double h = std::sqrt(std::max(0.0, ca.radius * ca.radius - along * along));
    out.push_back(ca.centre + u * along + u.perp() * h);
    if (h > 0.0) {
        out.push_back(ca.centre + u * along - u.perp() * h);
    }
    return out;
}

// True when the two spans share a stretch of positive length (same line or same circle).
bool overlaps(const Span& a, const Span& b) {
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    const auto* ca = std::get_if<ArcSpan>(&a);
    const auto* cb = std::get_if<ArcSpan>(&b);
    if (la && lb) {
        const Vec2 u = la->end - la->start;
        if (distanceToSpan(lb->start, LineSpan{la->start - u * 1e6, la->end + u * 1e6}) > 1e-9 ||
            distanceToSpan(lb->end, LineSpan{la->start - u * 1e6, la->end + u * 1e6}) > 1e-9) {
            return false;
        }
    } else if (ca && cb) {
        if (distance(ca->centre, cb->centre) > 1e-9 || std::fabs(ca->radius - cb->radius) > 1e-9) {
            return false;
        }
    } else {
        return false;
    }
    for (int k = 1; k < 20; ++k) {
        if (distanceToSpan(pointAt(a, k / 20.0), b) < 1e-9 ||
            distanceToSpan(pointAt(b, k / 20.0), a) < 1e-9) {
            return true;
        }
    }
    return false;
}

// How far from the junction of `a` (ending) and `b` (starting) a contact still counts as the
// junction itself.
double junctionReach(const Span& a, const Span& b) {
    if (tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) < 1.0 - 1e-6) {
        return 1e-6;
    }
    double r = 1.0;
    for (const Span* s : {&a, &b}) {
        if (const auto* arc = std::get_if<ArcSpan>(s)) {
            r = std::max(r, arc->radius);
        }
    }
    return 1e-6 + 4.0 * std::sqrt(2.0 * r * 1e-9);
}

// Exact span-vs-span self-intersection test of one chain. Consecutive spans may only meet at
// their shared junction (and the last and first at the closing junction of a closed chain);
// any other contact — crossing, touching, overlap — is a self-intersection.
bool selfIntersects(const Contour& chain, std::string* why = nullptr) {
    const std::vector<Span>& s = chain.spans;
    const std::size_t n = s.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const bool adjacent = j == i + 1;
            const bool closing = chain.closed && i == 0 && j + 1 == n && n > 1;
            if (overlaps(s[i], s[j])) {
                if (why) {
                    *why = "overlap " + std::to_string(i) + "/" + std::to_string(j);
                }
                return true;
            }
            for (const Vec2 p : carrierMeets(s[i], s[j])) {
                if (distanceToSpan(p, s[i]) > 1e-9 || distanceToSpan(p, s[j]) > 1e-9) {
                    continue;  // on a carrier but outside a span
                }
                // At a tangent-continuous junction the two carriers touch, so the 1e-9 on-span
                // band admits contacts up to ~sqrt(2 r 1e-9) away from the junction itself.
                if (adjacent && distance(p, endPoint(s[i])) <= junctionReach(s[i], s[j])) {
                    continue;
                }
                if (closing && distance(p, startPoint(s[0])) <= junctionReach(s[j], s[i])) {
                    continue;
                }
                if (why) {
                    *why = "spans " + std::to_string(i) + "/" + std::to_string(j) + " meet at (" +
                           std::to_string(p.x) + "," + std::to_string(p.y) + ")";
                }
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------------------------
// Result checks.
// ---------------------------------------------------------------------------------------------

struct ChainStats {
    double minDistance{1e300};
    double maxDistance{0.0};
};

ChainStats sampleChain(const Contour& chain, const Contour& input) {
    ChainStats st;
    for (const Span& s : chain.spans) {
        for (int i = 0; i <= 40; ++i) {
            const double dist = distanceToSpans(pointAt(s, i / 40.0), input.spans);
            st.minDistance = std::min(st.minDistance, dist);
            st.maxDistance = std::max(st.maxDistance, dist);
        }
    }
    return st;
}

// Per chain: exactly continuous, sub-tolerance spans only where continuity needs them, no
// retracing at a junction, a closed chain really closes, no self-intersection (exact), every
// point >= d (to 1e-6), Round points exactly d.
void checkChains(const std::vector<Contour>& chains, const Contour& input, double d, JoinType join) {
    const Tolerance exact{1e-12, 1e-9, 1e-12};
    for (std::size_t k = 0; k < chains.size(); ++k) {
        const Contour& c = chains[k];
        CAPTURE(k);
        REQUIRE_FALSE(c.isEmpty());
        // ADR-0012 A1 (T-0006): consecutive spans meet exactly (to 1e-12), for G-code.
        CHECK(c.isContinuous(exact));
        const std::size_t n = c.spans.size();
        for (std::size_t i = 0; i < n; ++i) {
            // A span no longer than the tolerance. T-0004 allowed one only where leaving it out
            // opened a gap wider than the tolerance, because chains were continuous only within
            // the tolerance. Since ADR-0012 A1 (T-0006) chains are exactly continuous, so such a
            // span is also kept where the offset really has a sub-tolerance feature that its
            // neighbours cannot be joined across. It must still have length, sit between two
            // spans (never at an open chain's ends), meet both exactly and not step backwards.
            if (isDegenerate(c.spans[i], kTol)) {
                CHECK(length(c.spans[i]) > 1e-12);
                CHECK((c.closed || (i > 0 && i + 1 < n)));
                const Span& prev = c.spans[(i + n - 1) % n];
                const Span& next = c.spans[(i + 1) % n];
                if (c.closed || i > 0) {
                    CHECK(distance(endPoint(prev), startPoint(c.spans[i])) <= 1e-12);
                }
                if (c.closed || i + 1 < n) {
                    CHECK(distance(endPoint(c.spans[i]), startPoint(next)) <= 1e-12);
                    // Not a backward step: the next span does not set off back along this one.
                    CHECK(tangentAt(c.spans[i], 1.0).dot(tangentAt(next, 0.0)) > -0.9999);
                }
            }
            if (i > 0 && tangentAt(c.spans[i - 1], 1.0).dot(tangentAt(c.spans[i], 0.0)) < -0.9999) {
                // A near-reversal is only legitimate as the tip of a needle-shaped feature; the
                // chain must not run back over itself.
                const Span& a = c.spans[i - 1];
                const Span& b = c.spans[i];
                const Span& shorter = length(a) < length(b) ? a : b;
                const Span& longer = length(a) < length(b) ? b : a;
                CHECK(distanceToSpan(pointAt(shorter, 0.5), longer) > 1e-6);
            }
        }
        if (c.closed) {
            CHECK(distance(c.startPoint(), c.endPoint()) <= kTol.distance);
        }
        std::string why;
        const bool crossed = selfIntersects(c, &why);
        CAPTURE(why);
        CHECK_FALSE(crossed);
        const ChainStats st = sampleChain(c, input);
        CHECK(st.minDistance >= d - 1e-6 * std::max(1.0, d));
        if (join == JoinType::Round) {
            CHECK(st.maxDistance <= d + 1e-6 * std::max(1.0, d));
        } else {
            CHECK(st.maxDistance <= std::max(kMitre, 1.5) * d + 1e-6);
        }
    }
}

// Offset carrier of one input span on the requested side (sign +1 left, -1 right), or nothing
// when an arc collapses.
std::optional<Span> sideOffset(const Span& s, double d, double sign) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 n = (line->end - line->start).normalized().perp() * (d * sign);
        return Span{LineSpan{line->start + n, line->end + n}};
    }
    ArcSpan arc = std::get<ArcSpan>(s);
    const double towardsCentre = arc.direction == ArcDirection::CCW ? sign : -sign;
    arc.radius -= towardsCentre * d;
    if (arc.radius <= 1e-9) {
        return std::nullopt;
    }
    return Span{arc};
}

bool isOutsideCorner(Vec2 tin, Vec2 tout, double sign) {
    const double cross = tin.cross(tout);
    if (std::fabs(cross) <= 1e-9) {
        return tin.dot(tout) < 0.0;
    }
    return sign * cross < 0.0;
}

// Coverage (AC2): densely samples the exact d-offset of every input span on the requested side
// (plus, for Round, the round corner arcs) and requires every sample that is at least d from the
// whole input (to rounding) to lie on a returned chain within 1e-6 d (or, inside a dropped
// sub-tolerance sliver next to a span end, within the tolerance). For Sharp/Square, samples within the reach
// of an outside corner's mitre or chamfer are skipped: those corners deliberately replace the round corner,
// and their extensions may cover a neighbour's offset there (they are checked by the distance property
// instead). Returns the number of samples checked.
std::size_t checkCoverage(const std::vector<Contour>& chains, const Contour& input, double d, OffsetSide side,
                          JoinType join) {
    const double sign = side == OffsetSide::Left ? 1.0 : -1.0;
    const double margin = 1e-6 * std::max(1.0, d);
    std::vector<Vec2> outsideVertices;
    std::vector<Span> locus;
    for (std::size_t i = 0; i < input.spans.size(); ++i) {
        if (auto o = sideOffset(input.spans[i], d, sign)) {
            locus.push_back(*o);
        }
        if (i == 0) {
            continue;
        }
        const Vec2 v = startPoint(input.spans[i]);
        const Vec2 tin = tangentAt(input.spans[i - 1], 1.0);
        const Vec2 tout = tangentAt(input.spans[i], 0.0);
        if (!isOutsideCorner(tin, tout, sign)) {
            continue;
        }
        outsideVertices.push_back(v);
        if (join == JoinType::Round) {
            const Vec2 e = v + tin.perp() * (d * sign);
            const Vec2 s = v + tout.perp() * (d * sign);
            locus.emplace_back(
                ArcSpan::fromCentre(v, e, s, sign > 0.0 ? ArcDirection::CW : ArcDirection::CCW));
        }
    }
    const double reach = std::max(kMitre, 1.5) * d + margin;
    std::size_t checked = 0;
    for (const Span& s : locus) {
        const int samples = std::clamp(static_cast<int>(length(s) / (d / 12.0)), 16, 400);
        for (int k = 0; k <= samples; ++k) {
            const Vec2 q = pointAt(s, static_cast<double>(k) / samples);
            if (distanceToSpans(q, input.spans) < d - 1e-9 * std::max(1.0, d)) {
                continue;  // closer than d to another part of the input: not part of the offset
            }
            if (join != JoinType::Round && std::any_of(outsideVertices.begin(), outsideVertices.end(),
                                                       [&](Vec2 v) { return distance(q, v) <= reach; })) {
                continue;
            }
            ++checked;
            const double gap = distanceToChains(q, chains);
            // Spans shorter than the tolerance are dropped from the output by contract: a sample
            // may miss by up to that much right next to a span end, or entirely when the valid
            // stretch of offset it lies on is itself no longer than the tolerance.
            if (gap > margin && !(gap <= kTol.distance && distanceToSpanEnds(q, chains) <= kTol.distance) &&
                validExtent(s, static_cast<double>(k) / samples, input, d) > 1.5 * kTol.distance) {
                CAPTURE(q.x, q.y, gap);
                CHECK(gap <= margin);
                return checked;  // one report per case is enough
            }
        }
    }
    return checked;
}

// Arc-length position along the input of the input point nearest to p (for ordering checks).
double pathPosition(Vec2 p, const Contour& input) {
    double best = 1e300;
    double pos = 0.0;
    double before = 0.0;
    for (const Span& s : input.spans) {
        const double len = length(s);
        for (int k = 0; k <= 400; ++k) {
            const double t = k / 400.0;
            const double dist = distance(p, pointAt(s, t));
            if (dist < best) {
                best = dist;
                pos = before + t * len;
            }
        }
        before += len;
    }
    return pos;
}

// ---------------------------------------------------------------------------------------------
// Fixed-seed generators with headings over the full +-180 degrees.
// ---------------------------------------------------------------------------------------------

// std::mt19937's sequence is fixed by the standard (the distributions are not).
struct Rng {
    std::mt19937 engine;
    explicit Rng(std::uint32_t seed) : engine(seed) {}
    double unit() { return static_cast<double>(engine()) / 4294967296.0; }
    double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
    int count(int lo, int hi) { return lo + static_cast<int>(unit() * static_cast<double>(hi - lo + 1)); }
    double sign() { return unit() < 0.5 ? -1.0 : 1.0; }
};

Contour polyline(const std::vector<Vec2>& pts) {
    Contour c;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        c.spans.emplace_back(LineSpan{pts[i - 1], pts[i]});
    }
    return c;
}

// Span lengths: a quarter are tiny (< d/10), the rest moderate.
double randomLength(Rng& rng, double d) {
    return rng.unit() < 0.25 ? rng.range(0.02, 0.09) * d : rng.range(0.3, 4.0) * d;
}

// Random walk: every heading drawn afresh from the full circle (so U-turns and near reversals
// are common), lengths mixed.
Contour randomWalk(Rng& rng, double d) {
    std::vector<Vec2> pts{{0.0, 0.0}};
    const int n = rng.count(4, 14);
    for (int i = 0; i < n; ++i) {
        pts.push_back(pts.back() + Vec2::fromAngle(rng.range(-180.0, 180.0) * kDeg, randomLength(rng, d)));
    }
    return polyline(pts);
}

// Archimedean spiral polyline, inwards or outwards, CW or CCW; ring spacing around 1-3 d so
// the offset of one ring runs into the next.
Contour spiralPolyline(Rng& rng, double d) {
    const double spacing = rng.range(0.8, 3.0) * d;
    const double r0 = rng.range(0.3, 2.0) * d;
    const double turns = rng.range(1.5, 3.5);
    const double step = rng.range(12.0, 40.0) * kDeg;
    const double dir = rng.sign();
    const double rot = rng.range(-180.0, 180.0) * kDeg;
    std::vector<Vec2> pts;
    for (double th = 0.0; th <= turns * 2.0 * kPi; th += step) {
        const double r = r0 + spacing * th / (2.0 * kPi);
        pts.push_back(Vec2::fromAngle(rot + dir * th, r));
    }
    if (rng.unit() < 0.5) {
        std::reverse(pts.begin(), pts.end());
    }
    return polyline(pts);
}

// Spiral of tangent half-circle arcs with shrinking radius (CW or CCW), optionally led in and
// out by lines.
Contour arcSpiral(Rng& rng, double d) {
    Contour c;
    const ArcDirection dir = rng.unit() < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
    const double turn = dir == ArcDirection::CCW ? 1.0 : -1.0;
    double heading = rng.range(-180.0, 180.0) * kDeg;
    Vec2 p{0.0, 0.0};
    if (rng.unit() < 0.5) {
        const Vec2 q = p + Vec2::fromAngle(heading, rng.range(1.0, 5.0) * d);
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
    }
    double r = rng.range(3.0, 6.0) * d;
    const double shrink = rng.range(0.6, 0.9);
    const int halves = rng.count(3, 7);
    for (int i = 0; i < halves && r > 0.05 * d; ++i) {
        const Vec2 centre = p + Vec2::fromAngle(heading).perp() * (r * turn);
        const double sweep = rng.range(150.0, 180.0) * kDeg;
        const Vec2 q = centre + Vec2::fromAngle((p - centre).angle() + turn * sweep, r);
        c.spans.emplace_back(ArcSpan::fromCentre(centre, p, q, dir));
        heading += turn * sweep;
        p = q;
        r *= shrink;
    }
    if (rng.unit() < 0.5) {
        const Vec2 q = p + Vec2::fromAngle(heading, rng.range(1.0, 5.0) * d);
        c.spans.emplace_back(LineSpan{p, q});
    }
    return c;
}

// One to three hairpins narrower than 2d: long legs joined by a U-turn made of an arc of
// radius < d or of a short cap line (square or slanted), legs slightly splayed.
Contour hairpins(Rng& rng, double d) {
    Contour c;
    Vec2 p{0.0, 0.0};
    double heading = rng.range(-180.0, 180.0) * kDeg;
    const int pins = rng.count(1, 3);
    for (int i = 0; i < pins; ++i) {
        const Vec2 q = p + Vec2::fromAngle(heading, rng.range(4.0, 10.0) * d);
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
        const double width = rng.range(0.1, 1.9) * d;
        const double turn = rng.sign();
        if (rng.unit() < 0.5) {
            const Vec2 centre = p + Vec2::fromAngle(heading).perp() * (0.5 * width * turn);
            const Vec2 r = centre + (centre - p);
            c.spans.emplace_back(
                ArcSpan::fromCentre(centre, p, r, turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW));
            p = r;
        } else {
            const Vec2 r = p + Vec2::fromAngle(heading + turn * rng.range(60.0, 120.0) * kDeg, width);
            c.spans.emplace_back(LineSpan{p, r});
            p = r;
        }
        heading += kPi + rng.range(-8.0, 8.0) * kDeg;
    }
    const Vec2 q = p + Vec2::fromAngle(heading, rng.range(4.0, 10.0) * d);
    c.spans.emplace_back(LineSpan{p, q});
    return c;
}

// Figure-eights: a lemniscate polyline (crosses itself once), or an "alpha" loop made of a
// line, an arc sweeping 250-340 degrees and a line that crosses the first one.
Contour figureEight(Rng& rng, double d) {
    const double rot = rng.range(-180.0, 180.0) * kDeg;
    const Transform2 m = Transform2::rotation(rot);
    if (rng.unit() < 0.5) {
        const double a = rng.range(2.0, 6.0) * d;
        const int n = rng.count(16, 48);
        std::vector<Vec2> pts;
        const double t0 = rng.range(0.0, 2.0 * kPi);
        const double span = rng.range(1.2, 1.9) * 2.0 * kPi * 0.5;
        for (int k = 0; k <= n; ++k) {
            const double t = t0 + span * k / n;
            const double den = 1.0 + std::sin(t) * std::sin(t);
            pts.push_back(m.apply(Vec2{a * std::cos(t) / den, a * std::sin(t) * std::cos(t) / den}));
        }
        return polyline(pts);
    }
    const double r = rng.range(0.5, 4.0) * d;
    const double turn = rng.sign();
    Contour c;
    const Vec2 a = m.apply(Vec2{-rng.range(2.0, 6.0) * d - r, 0.0});
    const Vec2 b = m.apply(Vec2{0.0, 0.0});
    c.spans.emplace_back(LineSpan{a, b});
    const Vec2 centre = m.apply(Vec2{0.0, r * turn});
    const double sweep = rng.range(250.0, 340.0) * kDeg;
    const Vec2 e = centre + Vec2::fromAngle((b - centre).angle() + turn * sweep, r);
    c.spans.emplace_back(
        ArcSpan::fromCentre(centre, b, e, turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW));
    const Vec2 heading = tangentAt(c.spans.back(), 1.0);
    c.spans.emplace_back(LineSpan{e, e + heading * (rng.range(1.5, 4.0) * r + d)});
    return c;
}

// Zig-zags whose turns are near-reversals (170 to 179.9 degrees), with some tiny spans.
Contour nearReversals(Rng& rng, double d) {
    std::vector<Vec2> pts{{0.0, 0.0}};
    double heading = rng.range(-180.0, 180.0) * kDeg;
    const int n = rng.count(3, 9);
    for (int i = 0; i < n; ++i) {
        pts.push_back(pts.back() + Vec2::fromAngle(heading, rng.unit() < 0.2 ? rng.range(0.02, 0.09) * d
                                                                             : rng.range(0.5, 5.0) * d));
        heading += rng.sign() * rng.range(170.0, 179.9) * kDeg;
    }
    return polyline(pts);
}

// Free mixture of lines and arcs: junctions are tangent or kinked by any angle, arcs sweep up to
// 300 degrees with radii from well below to well above d, tiny spans included.
Contour mixedFree(Rng& rng, double d) {
    Contour c;
    Vec2 p{0.0, 0.0};
    double heading = rng.range(-180.0, 180.0) * kDeg;
    const int n = rng.count(3, 10);
    for (int i = 0; i < n; ++i) {
        if (i > 0 && rng.unit() < 0.5) {
            heading += rng.range(-180.0, 180.0) * kDeg;  // non-tangent junction
        }
        if (rng.unit() < 0.5) {
            const Vec2 q = p + Vec2::fromAngle(heading, randomLength(rng, d));
            c.spans.emplace_back(LineSpan{p, q});
            p = q;
            continue;
        }
        const double radius = rng.unit() < 0.2 ? rng.range(0.03, 0.09) * d : rng.range(0.2, 5.0) * d;
        const double turn = rng.sign();
        const double sweep = rng.range(10.0, 300.0) * kDeg;
        const Vec2 centre = p + Vec2::fromAngle(heading).perp() * (radius * turn);
        const Vec2 q = centre + Vec2::fromAngle((p - centre).angle() + turn * sweep, radius);
        c.spans.emplace_back(
            ArcSpan::fromCentre(centre, p, q, turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW));
        heading += turn * sweep;
        p = q;
    }
    return c;
}

using Generator = Contour (*)(Rng&, double);

struct NamedGenerator {
    const char* name;
    Generator make;
    std::uint32_t seed;
};

const NamedGenerator kGenerators[] = {
    {"random walk", randomWalk, 1001u},   {"spiral polyline", spiralPolyline, 1002u},
    {"arc spiral", arcSpiral, 1003u},     {"hairpins", hairpins, 1004u},
    {"figure eight", figureEight, 1005u}, {"near reversals", nearReversals, 1006u},
    {"mixed line/arc", mixedFree, 1007u},
};

Contour keyhole() {
    // Neck 1 mm wide (narrower than 2d = 2) leading from a flange down into a 8 x 6 chamber.
    return polyline({{-10.0, 0.0},
                     {-0.5, 0.0},
                     {-0.5, -6.0},
                     {-4.0, -6.0},
                     {-4.0, -12.0},
                     {4.0, -12.0},
                     {4.0, -6.0},
                     {0.5, -6.0},
                     {0.5, 0.0},
                     {10.0, 0.0}});
}

Contour reproA() {
    Contour c;
    c.spans.emplace_back(
        LineSpan{{-6.4312190988360092, 1.1428523003734297}, {-12.846939679779521, 3.9348207104935331}});
    c.spans.emplace_back(
        LineSpan{{-12.846939679779521, 3.9348207104935331}, {-9.7434797348853266, -1.9260892800402662}});
    c.spans.emplace_back(ArcSpan{{-17.535585621268488, -1.0001000247398364},
                                 7.846933811721553,
                                 -0.11828212233321031,
                                 -5.0521988806306926,
                                 ArcDirection::CW});
    c.spans.emplace_back(ArcSpan{{-17.021348114572877, 0.45450829964057693},
                                 6.3041034287405493,
                                 1.2309864265488941,
                                 0.23263422704120162,
                                 ArcDirection::CW});
    return c;
}

}  // namespace

// --- AC1: a hairpin narrower than 2d returns every chain, in path order ----------------------

TEST_CASE("T-0004 AC1: inside of a narrow hairpin returns every chain in path order",
          "[geometry][offset][open][t0004]") {
    // The neck's interior is on the left of the path. It is narrower than 2d, so the left
    // offset falls apart into the flange chain over the neck mouth and the chamber chain.
    const Contour in = keyhole();
    const double d = 1.0;
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, join, kTol);
        REQUIRE(chains.size() == 2);
        checkChains(chains, in, d, join);
        CHECK(checkCoverage(chains, in, d, OffsetSide::Left, join) > 100);
        // Ordered along the input: the flange chain starts at the path start, the chamber
        // chain starts down the neck.
        CHECK(pathPosition(chains[0].startPoint(), in) < pathPosition(chains[1].startPoint(), in));
        CHECK_FALSE(chains[0].closed);
        CHECK(chains[1].closed);  // the chamber interior is a closed loop
        CHECK(chains[0].startPoint().x < -9.99);
        CHECK(chains[0].endPoint().x > 9.99);
        CHECK(chains[1].bounds().max.y < -6.0 + 1e-9);
        // The outside stays one chain.
        const std::vector<Contour> right = offsetOpenContour(in, d, OffsetSide::Right, join, kTol);
        REQUIRE(right.size() == 1);
        checkChains(right, in, d, join);
    }
}

TEST_CASE("T-0004 AC1: a pinched U comes back as a funnel chain and a chamber loop",
          "[geometry][offset][open][t0004]") {
    // Hourglass: the path comes down a funnel to a 1 mm pinch, round a chamber, back up
    // through the pinch and out. On the inside the funnel and the chamber separate.
    const Contour in =
        polyline({{-6.0, 10.0}, {-0.5, 5.0}, {-5.0, 0.0}, {5.0, 0.0}, {0.5, 5.0}, {6.0, 10.0}});
    const double d = 1.0;
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, join, kTol);
        REQUIRE(chains.size() == 2);
        checkChains(chains, in, d, join);
        CHECK(checkCoverage(chains, in, d, OffsetSide::Left, join) > 100);
        CHECK(pathPosition(chains[0].startPoint(), in) < pathPosition(chains[1].startPoint(), in));
        // The funnel above the pinch (its two legs meet in a V), then the chamber below it.
        CHECK_FALSE(chains[0].closed);
        CHECK(chains[0].bounds().min.y > 5.0);
        CHECK(chains[1].closed);
        CHECK(chains[1].bounds().max.y < 5.0);
    }
}

TEST_CASE("T-0004: a slot narrower than 2d with sharp joins returns no overlapping copies",
          "[geometry][offset][open][t0004]") {
    // The mitre extensions on both sides of the slot coincide over the slot mouth.
    const Contour in =
        polyline({{-10.0, 0.0}, {0.0, 0.0}, {0.0, 10.0}, {1.0, 10.0}, {1.0, 0.0}, {11.0, 0.0}});
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, 1.0, side, join, kTol);
            REQUIRE(chains.size() == 1);
            checkChains(chains, in, 1.0, join);
            checkCoverage(chains, in, 1.0, side, join);
        }
    }
    const std::vector<Contour> right = offsetOpenContour(in, 1.0, OffsetSide::Right, JoinType::Sharp, kTol);
    REQUIRE(right.size() == 1);
    CHECK_THAT(right[0].length(), Catch::Matchers::WithinAbs(21.0, 1e-9));
}

// --- AC4: no chain intersects itself -----------------------------------------------------------

TEST_CASE("T-0004 AC4: regression repro A splits the loop from the tail", "[geometry][offset][open][t0004]") {
    const Contour in = reproA();
    const double d = 2.5;
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol);
            REQUIRE_FALSE(chains.empty());
            checkChains(chains, in, d, join);
            checkCoverage(chains, in, d, side, join);
        }
    }
    // The reported case: Right side, Sharp and Square. The offset curls into a loop; it comes
    // back as an open chain plus a closed loop, neither crossing itself.
    for (const JoinType join : {JoinType::Sharp, JoinType::Square}) {
        const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Right, join, kTol);
        REQUIRE(chains.size() == 2);
        CHECK_FALSE(chains[0].closed);
        CHECK(chains[1].closed);
        for (const Contour& c : chains) {
            CHECK_FALSE(selfIntersects(c));
        }
    }
}

TEST_CASE("T-0004 AC4: a crossing offset is cut into tail, loop and tail",
          "[geometry][offset][open][t0004]") {
    // An alpha loop: line, 300 degree arc, line crossing the first line. On the outside of the
    // loop the offset crosses itself near the input crossing.
    Contour in;
    in.spans.emplace_back(LineSpan{{-10.0, 0.0}, {0.0, 0.0}});
    in.spans.emplace_back(ArcSpan::fromCentre(
        {0.0, 3.0}, {0.0, 0.0}, Vec2{0.0, 3.0} + Vec2::fromAngle(-kPi / 2.0 + 300.0 * kDeg, 3.0),
        ArcDirection::CCW));
    const Vec2 e = endPoint(in.spans.back());
    in.spans.emplace_back(LineSpan{e, e + tangentAt(in.spans.back(), 1.0) * 12.0});
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, 0.5, side, join, kTol);
            REQUIRE(chains.size() >= 2);
            checkChains(chains, in, 0.5, join);
            CHECK(checkCoverage(chains, in, 0.5, side, join) > 100);
        }
        // Inside the loop (left): lead-in tail, the loop itself, lead-out tail, in path order.
        const std::vector<Contour> left = offsetOpenContour(in, 0.5, OffsetSide::Left, join, kTol);
        REQUIRE(left.size() == 3);
        CHECK_FALSE(left[0].closed);
        CHECK(left[1].closed);
        CHECK_FALSE(left[2].closed);
        CHECK(left[0].startPoint().x < -9.99);
        CHECK(distance(left[1].startPoint(), left[0].endPoint()) > 0.5);
        CHECK(pathPosition(left[2].endPoint(), in) > pathPosition(left[1].startPoint(), in));
    }
}

// --- AC2 + AC4 generative ----------------------------------------------------------------------

TEST_CASE("T-0004 AC2/AC4 generative: every chain is simple, at distance d, and nothing is dropped",
          "[geometry][offset][open][t0004][generative]") {
    std::size_t cases = 0;
    std::size_t multi = 0;
    std::size_t loops = 0;
    std::size_t samples = 0;
    for (const NamedGenerator& g : kGenerators) {
        Rng rng(g.seed);
        for (int trial = 0; trial < CARVETTO_OFFSET_TRIALS; ++trial) {
            for (const double d : {0.3, 1.0, 2.5}) {
                const Contour in = g.make(rng, d);
                if (in.isEmpty()) {
                    continue;
                }
                for (const JoinType join : kJoins) {
                    for (const OffsetSide side : kSides) {
                        CAPTURE(g.name, trial, d, static_cast<int>(join), static_cast<int>(side));
                        const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol);
                        checkChains(chains, in, d, join);
                        samples += checkCoverage(chains, in, d, side, join);
                        ++cases;
                        multi += chains.size() > 1 ? 1u : 0u;
                        loops += static_cast<std::size_t>(std::count_if(
                            chains.begin(), chains.end(), [](const Contour& c) { return c.closed; }));
                    }
                }
            }
        }
    }
    // The generators really exercise splitting and loops.
    CHECK(cases >= 1000);
    CHECK(multi > 200);
    CHECK(loops > 40);
    CHECK(samples > 100000);
}

TEST_CASE("T-0004: the result is deterministic and independent of the distance sign",
          "[geometry][offset][open][t0004]") {
    Rng rng(99u);
    for (int trial = 0; trial < 20; ++trial) {
        const Contour in = mixedFree(rng, 1.0);
        for (const JoinType join : kJoins) {
            const auto a = offsetOpenContour(in, 1.0, OffsetSide::Left, join, kTol);
            const auto b = offsetOpenContour(in, 1.0, OffsetSide::Left, join, kTol);
            const auto c = offsetOpenContour(in, -1.0, OffsetSide::Right, join, kTol);
            CHECK(a == b);
            CHECK(a == c);
        }
    }
}

// --- AC3: dense input ------------------------------------------------------------------------

namespace {

// 2000-span polyline: a polygonised wavy spiral, so many parts of the path lie within 2d of
// each other and the offset splits and prunes heavily.
Contour densePolyline(std::size_t spans) {
    std::vector<Vec2> pts;
    for (std::size_t k = 0; k <= spans; ++k) {
        const double t = static_cast<double>(k) / static_cast<double>(spans);
        const double th = t * 10.0 * 2.0 * kPi;
        const double r = 5.0 + 40.0 * t + 0.6 * std::sin(37.0 * th);
        pts.push_back(Vec2::fromAngle(th, r));
    }
    return polyline(pts);
}

}  // namespace

TEST_CASE("T-0004 AC3: a 2000-span polyline offsets correctly (timing checked in Release)",
          "[geometry][offset][open][t0004][perf]") {
    const Contour in = densePolyline(2000);
    REQUIRE(in.spanCount() == 2000);
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            // Process CPU time, not wall-clock, so parallel ctest load does not fail it (T-0006).
            const std::clock_t t0 = std::clock();
            const std::vector<Contour> chains = offsetOpenContour(in, 1.5, side, join, kTol);
            const double seconds =
                static_cast<double>(std::clock() - t0) / static_cast<double>(CLOCKS_PER_SEC);
            REQUIRE_FALSE(chains.empty());
#ifdef NDEBUG
            CHECK(seconds < 0.5);  // AC3: Release build
#else
            (void)seconds;  // Debug: the complexity ratio below guards the cost instead
#endif
            // Spot-check the distance property on every seventh span.
            std::size_t k = 0;
            for (const Contour& c : chains) {
                for (const Span& s : c.spans) {
                    if (k++ % 7 == 0) {
                        CHECK(distanceToSpans(pointAt(s, 0.5), in.spans) >= 1.5 - 1e-6 * 1.5);
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0004 AC3: offset cost grows about linearly with the span count (CPU-time ratio)",
          "[geometry][offset][open][t0004][perf]") {
    // Same spiral sampled with 1000 and 4000 spans: linear cost gives a ratio of ~4, quadratic
    // ~16. A ratio of CPU times holds in Debug and Release alike (T-0006).
    const Contour small = densePolyline(1000);
    const Contour large = densePolyline(4000);
    auto cost = [](const Contour& in, int reps) {
        const std::clock_t t0 = std::clock();
        std::size_t chains = 0;
        for (int r = 0; r < reps; ++r) {
            chains += offsetOpenContour(in, 1.5, OffsetSide::Left, JoinType::Round, kTol).size();
        }
        CHECK(chains > 0);
        return static_cast<double>(std::clock() - t0) / static_cast<double>(CLOCKS_PER_SEC) / reps;
    };
    const double smallCost = std::max(cost(small, 4), 1e-4);
    const double largeCost = cost(large, 1);
    CAPTURE(smallCost, largeCost);
    CHECK(largeCost < 8.0 * smallCost);
}
