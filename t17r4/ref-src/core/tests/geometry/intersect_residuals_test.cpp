// T-0005: intersection residuals from T-0002's third verification round.
//   AC1  closed contours of one or two spans ignore every shared junction;
//   AC2  nearly concentric arcs follow the general "tangent within tolerance" rule;
//   AC3  shallow double crossings involving a Bezier are symmetric, merged hits are tangent.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {
const Tolerance tol{};

Contour closedOf(std::vector<Span> spans) {
    Contour c;
    c.spans = std::move(spans);
    c.closed = true;
    return c;
}

// The same closed contour starting at span `k` instead of span 0.
Contour rotated(const Contour& c, std::size_t k) {
    Contour r = c;
    std::rotate(r.spans.begin(), r.spans.begin() + static_cast<std::ptrdiff_t>(k), r.spans.end());
    return r;
}

// Checks a contour that must not self-intersect, in both directions and from every start span.
void requireSimple(const Contour& c) {
    REQUIRE(c.isContinuous(tol));
    for (const bool rev : {false, true}) {
        const Contour base = rev ? c.reversed() : c;
        for (std::size_t k = 0; k < base.spans.size(); ++k) {
            const Contour v = rotated(base, k);
            INFO("reversed=" << rev << " start span=" << k);
            CHECK(selfIntersections(v, tol).isEmpty());
            CHECK_FALSE(hasSelfIntersections(v, tol));
        }
    }
}

// A cubic Bezier through the parabola y = k*x^2 - g over x in [-w, w], shifted by `o`.
BezierSpan parabola(double k, double g, double w, Vec2 o) {
    const Vec2 q0{-w, k * w * w - g};
    const Vec2 q1{0.0, -k * w * w - g};
    const Vec2 q2{w, k * w * w - g};
    return {o + q0, o + q0 + (q1 - q0) * (2.0 / 3.0), o + q2 + (q1 - q2) * (2.0 / 3.0), o + q2};
}

// intersect(a, b) and intersect(b, a) must be mirror images: same points (bitwise up to 1e-12),
// swapped parameters, same tangent flags, and every point on both spans. Returns false (and
// records a failure) on any asymmetry.
bool symmetricPair(const Span& a, const Span& b, IntersectionResult* result = nullptr) {
    const IntersectionResult ab = intersect(a, b, tol);
    if (result != nullptr) {
        *result = ab;
    }
    const IntersectionResult ba = intersect(b, a, tol);
    bool ok = ab.points.size() == ba.points.size() && ab.overlaps.size() == ba.overlaps.size();
    for (const Intersection& p : ab.points) {
        ok = ok && distance(p.point, pointAt(a, p.tA)) <= tol.distance &&
             distance(p.point, pointAt(b, p.tB)) <= tol.distance;
        const bool matched = std::any_of(ba.points.begin(), ba.points.end(), [&](const Intersection& q) {
            return approxEqual(p.point, q.point, 1e-12) && std::fabs(p.tA - q.tB) <= 1e-12 &&
                   std::fabs(p.tB - q.tA) <= 1e-12 && p.tangent == q.tangent;
        });
        ok = ok && matched;
    }
    // Points closer than tol.distance are always merged.
    for (std::size_t i = 0; i < ab.points.size(); ++i) {
        for (std::size_t j = i + 1; j < ab.points.size(); ++j) {
            ok = ok && distance(ab.points[i].point, ab.points[j].point) > tol.distance;
        }
    }
    return ok;
}

// T-0008: an end of either span (a full circle has none) within tolerance of the other span is a
// contact of its own, reported unless another hit is within tolerance of it. True when `p` is such
// an end contact.
bool isEndContact(const Span& a, const Span& b, const Intersection& p) {
    const auto hasEnd = [](const Span& s, double t) {
        const auto* arc = std::get_if<ArcSpan>(&s);
        return (t == 0.0 || t == 1.0) && (arc == nullptr || !arc->isFullCircle());
    };
    return (hasEnd(a, p.tA) && closestPoint(b, pointAt(a, p.tA)).distance <= tol.distance) ||
           (hasEnd(b, p.tB) && closestPoint(a, pointAt(b, p.tB)).distance <= tol.distance);
}
}  // namespace

// ============================================================================ AC1

TEST_CASE("T-0005 AC1: closed contours of two spans ignore both junctions", "[geometry][intersect][t0005]") {
    SECTION("half-disc: line + arc") {
        requireSimple(closedOf(
            {LineSpan{{-5.0, 0.0}, {5.0, 0.0}}, ArcSpan{{0.0, 0.0}, 5.0, 0.0, kPi, ArcDirection::CCW}}));
    }
    SECTION("Bezier lens") {
        requireSimple(closedOf({BezierSpan{{0.0, 0.0}, {2.0, 2.0}, {4.0, 2.0}, {4.0, 0.0}},
                                BezierSpan{{4.0, 0.0}, {4.0, -2.0}, {2.0, -2.0}, {0.0, 0.0}}}));
    }
    SECTION("smooth two-Bezier oval (tangent-continuous junctions)") {
        requireSimple(closedOf({BezierSpan{{-6.0, 0.0}, {-6.0, 4.0}, {6.0, 4.0}, {6.0, 0.0}},
                                BezierSpan{{6.0, 0.0}, {6.0, -4.0}, {-6.0, -4.0}, {-6.0, 0.0}}}));
    }
    SECTION("D-shape: line + Bezier") {
        requireSimple(closedOf({LineSpan{{0.0, -5.0}, {0.0, 5.0}},
                                BezierSpan{{0.0, 5.0}, {7.0, 5.0}, {7.0, -5.0}, {0.0, -5.0}}}));
    }
    SECTION("arc + arc lens and arc + Bezier") {
        requireSimple(closedOf({ArcSpan::fromCentre({0.0, -3.0}, {-4.0, 0.0}, {4.0, 0.0}, ArcDirection::CW),
                                ArcSpan::fromCentre({0.0, 3.0}, {4.0, 0.0}, {-4.0, 0.0}, ArcDirection::CW)}));
        requireSimple(closedOf({ArcSpan{{0.0, 0.0}, 5.0, kPi, 0.0, ArcDirection::CCW},
                                BezierSpan{{5.0, 0.0}, {5.0, 6.0}, {-5.0, 6.0}, {-5.0, 0.0}}}));
    }
}

TEST_CASE("T-0005 AC1: a two-span closed contour that crosses itself still reports the crossing",
          "[geometry][intersect][t0005]") {
    // Point-symmetric S-curve and its mirror image: they meet at both junctions and cross once,
    // at (5, 0), away from either junction.
    const Contour eight = closedOf({BezierSpan{{0.0, 0.0}, {3.0, 6.0}, {7.0, -6.0}, {10.0, 0.0}},
                                    BezierSpan{{10.0, 0.0}, {7.0, 6.0}, {3.0, -6.0}, {0.0, 0.0}}});
    REQUIRE(eight.isContinuous(tol));
    for (const bool rev : {false, true}) {
        for (std::size_t k = 0; k < 2; ++k) {
            const Contour v = rotated(rev ? eight.reversed() : eight, k);
            CHECK(hasSelfIntersections(v, tol));
            const ContourIntersections r = selfIntersections(v, tol);
            REQUIRE(r.points.size() == 1);
            CHECK(r.points[0].spanA == 0);
            CHECK(r.points[0].spanB == 1);
            CHECK(approxEqual(r.points[0].hit.point, {5.0, 0.0}, 1e-9));
        }
    }
    // Line + Bezier where the Bezier swings back across the line between the junctions.
    const Contour cross = closedOf(
        {LineSpan{{0.0, 0.0}, {10.0, 0.0}}, BezierSpan{{10.0, 0.0}, {10.0, -8.0}, {0.0, 8.0}, {0.0, 0.0}}});
    REQUIRE(cross.isContinuous(tol));
    const ContourIntersections rc = selfIntersections(cross, tol);
    REQUIRE(rc.points.size() == 1);
    CHECK(approxEqual(rc.points[0].hit.point, {5.0, 0.0}, 1e-9));
}

TEST_CASE("T-0005 AC1: closed contours of one span", "[geometry][intersect][t0005]") {
    requireSimple(Contour::circle({1.0, 2.0}, 3.0));
    requireSimple(Contour::circle({1.0, 2.0}, 3.0, ArcDirection::CW));
    // A full circle whose seam is not at angle 0.
    requireSimple(closedOf({ArcSpan{{0.0, 0.0}, 2.0, 1.3, 1.3, ArcDirection::CW}}));

    // A Bezier whose end returns to its start (a teardrop loop): closed, the meeting point is the
    // closing junction; open, it is a genuine contact of the curve with itself.
    const BezierSpan drop{{0.0, 0.0}, {10.0, 10.0}, {-10.0, 10.0}, {0.0, 0.0}};
    Contour c = closedOf({drop});
    requireSimple(c);
    c.closed = false;
    const ContourIntersections open = selfIntersections(c, tol);
    REQUIRE(open.points.size() == 1);
    CHECK(approxEqual(open.points[0].hit.point, {0.0, 0.0}, 1e-9));
    CHECK(hasSelfIntersections(c, tol));
}

// ============================================================================ AC2

TEST_CASE("T-0005 AC2: nearly concentric circles within tolerance of touching give one tangent hit",
          "[geometry][intersect][t0005]") {
    const ArcSpan a = ArcSpan::fullCircle({644.61470300472024, 365.33035654442392}, 181.1984687011373);
    const ArcSpan b = ArcSpan::fullCircle({644.61399749030329, 365.33092243365013}, 181.1966002797156);
    REQUIRE(distance(a.centre, b.centre) <= tol.distance);
    REQUIRE(std::fabs(a.radius - b.radius) > tol.distance);
    REQUIRE(std::fabs(a.radius - b.radius) - distance(a.centre, b.centre) <= tol.distance);
    for (const bool swapArgs : {false, true}) {
        const ArcSpan& x = swapArgs ? b : a;
        const ArcSpan& y = swapArgs ? a : b;
        const IntersectionResult r = intersect(x, y, tol);
        REQUIRE(r.points.size() == 1);
        CHECK(r.overlaps.empty());
        const Intersection& p = r.points[0];
        CHECK(p.tangent);
        CHECK(std::fabs(distance(p.point, x.centre) - x.radius) <= tol.distance);
        CHECK(std::fabs(distance(p.point, y.centre) - y.radius) <= tol.distance);
        CHECK(distance(p.point, x.pointAt(p.tA)) <= tol.distance);
        CHECK(distance(p.point, y.pointAt(p.tB)) <= tol.distance);
    }
    CHECK(symmetricPair(a, b));
}

TEST_CASE("T-0005 AC2: generative near-concentric arc pairs around the tolerance boundary",
          "[geometry][intersect][t0005][property]") {
    SplitMix rng{0x7005AC2ULL};
    std::size_t touching = 0;
    std::size_t missing = 0;
    std::size_t coincident = 0;
    for (int i = 0; i < 6000; ++i) {
        const Vec2 ca{rng.uniform(-1000.0, 1000.0), rng.uniform(-1000.0, 1000.0)};
        const double r1 = rng.logUniform(0.5, 1000.0);
        const double d = (i % 7 == 0) ? 0.0 : rng.uniform(0.0, 1.0) * tol.distance;
        const double dr = rng.uniform(-2.0, 2.0) * tol.distance;
        const Vec2 cb = ca + Vec2::fromAngle(rng.uniform(0.0, kTwoPi), d);
        const double r2 = r1 + dr;
        const double s1 = rng.uniform(-kPi, kPi);
        const double s2 = rng.uniform(-kPi, kPi);
        const ArcDirection d1 = (i % 2) != 0 ? ArcDirection::CW : ArcDirection::CCW;
        const ArcDirection d2 = (i % 3) != 0 ? ArcDirection::CW : ArcDirection::CCW;
        const ArcSpan fa{ca, r1, s1, s1, d1};
        const ArcSpan fb{cb, r2, s2, s2, d2};
        const double dd = (cb - ca).length();
        const double gap = std::fabs(r1 - r2) - dd;  // closest approach of the two circles
        INFO("case " << i << " d=" << dd << " r1=" << r1 << " r2=" << r2);
        // Classification at exactly the boundary depends on rounding; stay clear of it.
        if (std::fabs(std::fabs(r1 - r2) - tol.distance) < 1e-12 || std::fabs(gap - tol.distance) < 1e-12) {
            continue;
        }
        CHECK(symmetricPair(fa, fb));
        const IntersectionResult r = intersect(fa, fb, tol);
        if (dd <= tol.distance && std::fabs(r1 - r2) <= tol.distance) {
            CHECK(r.points.empty());
            CHECK_FALSE(r.overlaps.empty());
            ++coincident;
        } else if (gap <= tol.distance) {
            REQUIRE(r.points.size() == 1);
            CHECK(r.overlaps.empty());
            CHECK(r.points[0].tangent);
            CHECK(std::fabs(distance(r.points[0].point, ca) - r1) <= tol.distance);
            CHECK(std::fabs(distance(r.points[0].point, cb) - r2) <= tol.distance);
            ++touching;
        } else {
            CHECK(r.isEmpty());
            ++missing;
        }
        // Partial arcs: whatever is reported lies on both arcs, in either order.
        const ArcSpan pa{ca, r1, s1, rng.uniform(-kPi, kPi), d1};
        const ArcSpan pb{cb, r2, s2, rng.uniform(-kPi, kPi), d2};
        CHECK(symmetricPair(pa, pb));
    }
    CHECK(touching > 500);
    CHECK(missing > 500);
    CHECK(coincident > 500);
}

// ============================================================================ AC3

TEST_CASE("T-0005 AC3: arc/Bezier shallow double crossing is found in both argument orders",
          "[geometry][intersect][t0005]") {
    const ArcSpan arc{{-3.2305423907849837, -8.7399422212084588},
                      9.3248612306533616,
                      -0.86772800461480459,
                      -5.0533095891647033,
                      ArcDirection::CW};
    const BezierSpan bz{{-14.222937128613282, -10.138318879522913},
                        {-12.121499048115544, -10.250528577090371},
                        {-11.910081639881342, -6.2911507142599774},
                        {-14.011519720379081, -6.1789410166925203}};
    CHECK(symmetricPair(arc, bz));
    for (const bool swapArgs : {false, true}) {
        const IntersectionResult r =
            swapArgs ? intersect(Span{bz}, Span{arc}, tol) : intersect(Span{arc}, Span{bz}, tol);
        // Two genuine crossings ~0.13 mm apart; between them the curves stay within ~tol.
        REQUIRE(r.points.size() == 2);
        CHECK(approxEqual(r.points[0].point, {-12.545392095108, -8.307956160307}, 1e-9));
        CHECK(approxEqual(r.points[1].point, {-12.538428157215, -8.177537091408}, 1e-9));
        CHECK_FALSE(r.points[0].tangent);
        CHECK_FALSE(r.points[1].tangent);
    }
}

TEST_CASE("T-0005 AC3: two crossings closer than tolerance merge into one tangent hit",
          "[geometry][intersect][t0005]") {
    // y = x^2 - g crosses y = 0 at x = +-sqrt(g).
    const LineSpan line{{-3.0, 0.0}, {3.0, 0.0}};
    const BezierSpan flat{{-3.0, 0.0}, {-1.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}};  // the same line as a Bezier
    const ArcSpan big{{0.0, 1e4}, 1e4, -0.5 * kPi - 3e-4, -0.5 * kPi + 3e-4, ArcDirection::CCW};
    SECTION("separation 0.6*tol: one merged tangent hit") {
        const BezierSpan bz = parabola(1.0, 9e-8, 1.0, {0.0, 0.0});
        for (const Span& other : {Span{line}, Span{flat}}) {
            for (const bool swapArgs : {false, true}) {
                const IntersectionResult r =
                    swapArgs ? intersect(Span{bz}, other, tol) : intersect(other, Span{bz}, tol);
                REQUIRE(r.points.size() == 1);
                CHECK(r.points[0].tangent);
                CHECK(std::fabs(r.points[0].point.x) <= tol.distance);
            }
            CHECK(symmetricPair(other, bz));
        }
        // Against a huge arc that is locally almost the line y = 0 (sagitta ~ 2e-5 over the span).
        const BezierSpan lifted = parabola(1.0, 9e-8, 1.0, {0.0, 0.0});
        CHECK(symmetricPair(big, lifted));
    }
    SECTION("separation 1.5*tol: two crossings") {
        const BezierSpan bz = parabola(1.0, 0.75e-3 * 0.75e-3, 1.0, {0.0, 0.0});
        for (const Span& other : {Span{line}, Span{flat}}) {
            for (const bool swapArgs : {false, true}) {
                const IntersectionResult r =
                    swapArgs ? intersect(Span{bz}, other, tol) : intersect(other, Span{bz}, tol);
                REQUIRE(r.points.size() == 2);
                CHECK(std::fabs(std::fabs(r.points[0].point.x) - 0.75e-3) <= 1e-9);
                CHECK(std::fabs(std::fabs(r.points[1].point.x) - 0.75e-3) <= 1e-9);
            }
            CHECK(symmetricPair(other, bz));
        }
    }
}

namespace {
BezierSpan randomBezier(SplitMix& rng, double extent) {
    const auto pt = [&] { return Vec2{rng.uniform(-extent, extent), rng.uniform(-extent, extent)}; };
    return {pt(), pt(), pt(), pt()};
}

ArcSpan randomArc(SplitMix& rng, double extent) {
    return {{rng.uniform(-extent, extent), rng.uniform(-extent, extent)},
            rng.logUniform(0.5, 2.0 * extent),
            rng.uniform(-kPi, kPi),
            rng.uniform(-kPi, kPi),
            rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CW : ArcDirection::CCW};
}

// Moves every control point by up to `amount` in a random direction.
BezierSpan jiggled(const BezierSpan& b, SplitMix& rng, double amount) {
    const auto j = [&](Vec2 p) {
        return p + Vec2::fromAngle(rng.uniform(0.0, kTwoPi), rng.uniform(0.0, amount));
    };
    return {j(b.p0), j(b.p1), j(b.p2), j(b.p3)};
}

// A Bezier hugging `arc` within a few tolerances: the Bezier approximation of a piece of the arc
// (radial error <= 2.7e-4 r per 90 degrees), extended past both ends so that it crosses rather
// than ends on the arc, with its radius and control points nudged by up to ~2 tol.
BezierSpan hugging(const ArcSpan& arc, SplitMix& rng) {
    const double sweep = arc.sweepAngle();
    const double a0 = arc.startAngle + sweep * rng.uniform(0.0, 0.6);
    const double piece = std::copysign(rng.uniform(0.05, 0.6), sweep);
    const double scale = 1.0 + rng.uniform(-2.0, 2.0) * tol.distance / arc.radius;
    const ArcSpan sub{arc.centre, arc.radius * scale, a0, a0 + piece, arc.direction};
    const BezierSpan bz = sub.toBeziers().front();
    const BezierSpan b = jiggled(bz, rng, rng.uniform(0.0, 2.0) * tol.distance);
    return rng.uniform(0.0, 1.0) < 0.5 ? b : b.reversed();
}
}  // namespace

TEST_CASE("T-0005 AC3: symmetry fuzz over arc/Bezier and Bezier/Bezier pairs",
          "[geometry][intersect][t0005][property]") {
    SplitMix rng{0x7005AC3ULL};
    std::size_t asymmetric = 0;
    std::size_t pairs = 0;
    std::size_t multiHit = 0;
    std::size_t tangentHits = 0;
    const auto run = [&](const Span& a, const Span& b, int i) {
        ++pairs;
        IntersectionResult r;
        if (!symmetricPair(a, b, &r)) {
            ++asymmetric;
            UNSCOPED_INFO("asymmetric case " << i);
        }
        multiHit += r.points.size() >= 2 ? 1u : 0u;
        tangentHits += static_cast<std::size_t>(
            std::count_if(r.points.begin(), r.points.end(), [](const Intersection& p) { return p.tangent; }));
    };
    // Debug builds (no NDEBUG) run a lighter mix (about 6,700 pairs); optimised builds run the
    // full >= 9,000-pair count several times over and the expensive near-coincident family more often.
#ifdef NDEBUG
    constexpr int kRounds = 5 * 2950;
    constexpr int kJiggleEvery = 4;
#else
    constexpr int kRounds = 2200;
    constexpr int kJiggleEvery = 16;
#endif
    for (int i = 0; i < kRounds; ++i) {
        // Random arc/Bezier and Bezier/Bezier pairs.
        const ArcSpan arc = randomArc(rng, 10.0);
        run(arc, randomBezier(rng, 10.0), i);
        run(randomBezier(rng, 10.0), randomBezier(rng, 10.0), i);
        // Shallow contacts: a Bezier hugging an arc, and (every few rounds, as these near-coincident
        // pairs are the slowest in Debug) a Bezier against a jiggled copy of itself: control points
        // moved by up to a few tolerances, so the curves cross repeatedly with gaps around 2*tol.
        const ArcSpan base = randomArc(rng, 10.0);
        run(base, hugging(base, rng), i);
        if (i % kJiggleEvery == 0) {
            const BezierSpan b = randomBezier(rng, 4.0);
            const BezierSpan c = jiggled(b, rng, rng.uniform(0.5, 3.0) * tol.distance);
            run(b, rng.uniform(0.0, 1.0) < 0.5 ? c : c.reversed(), i);
        }
    }
    // The AC3 count (>= 9,000 pairs) is met by the optimised build; Debug runs a lighter mix.
#ifdef NDEBUG
    CHECK(pairs >= 9000);
#else
    CHECK(pairs >= 6000);
#endif
    CHECK(asymmetric == 0);
    CHECK(multiHit > kRounds / 6);
    CHECK(tangentHits > kRounds / 30);
}

TEST_CASE("T-0005 AC3: every clearly separated crossing of a hugging Bezier is reported",
          "[geometry][intersect][t0005][property]") {
    // Brute force: the signed radial distance of the Bezier to the circle, sampled densely. Each
    // sign change whose neighbours are more than 3*tol away, at a crossing angle above 1e-4, is a
    // crossing that must have a reported hit within tolerance.
    SplitMix rng{0x7005AC4ULL};
    std::size_t checked = 0;
#ifdef NDEBUG
    constexpr int kCases = 6000;
#else
    constexpr int kCases = 1100;
#endif
    for (int i = 0; i < kCases; ++i) {
        const ArcSpan circle = ArcSpan::fullCircle({rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)},
                                                   rng.logUniform(1.0, 50.0));
        const BezierSpan bz = hugging(circle, rng);
        constexpr int kSamples = 4000;
        std::vector<std::pair<Vec2, double>> roots;  // crossing point, sin of crossing angle
        double prevT = 0.0;
        double prevF = distance(bz.p0, circle.centre) - circle.radius;
        for (int k = 1; k <= kSamples; ++k) {
            const double t = static_cast<double>(k) / kSamples;
            const double f = distance(bz.pointAt(t), circle.centre) - circle.radius;
            if ((f > 0.0) != (prevF > 0.0)) {
                const double tr = prevT + (t - prevT) * prevF / (prevF - f);
                const Vec2 p = bz.pointAt(tr);
                const Vec2 radial = (p - circle.centre) / distance(p, circle.centre);
                roots.emplace_back(p, std::fabs(radial.dot(bz.tangentAt(tr))));
            }
            prevT = t;
            prevF = f;
        }
        const IntersectionResult r = intersect(Span{circle}, Span{bz}, tol);
        for (std::size_t k = 0; k < roots.size(); ++k) {
            bool isolated = roots[k].second > 1e-4;
            for (std::size_t m = 0; m < roots.size(); ++m) {
                isolated =
                    isolated && (m == k || distance(roots[k].first, roots[m].first) > 3.0 * tol.distance);
            }
            if (!isolated) {
                continue;
            }
            ++checked;
            const bool found = std::any_of(r.points.begin(), r.points.end(), [&](const Intersection& p) {
                return distance(p.point, roots[k].first) <= tol.distance;
            });
            INFO("case " << i << " root " << k);
            CHECK(found);
        }
    }
    CHECK(checked > static_cast<std::size_t>(kCases) / 9);
}

// ======================================================= AC1 generative (brute force)

namespace {
// Dense reference polygonisation of one span, parameter-uniform for Beziers and angle-uniform for
// arcs (chordal error well below the margins used for classification).
std::vector<Vec2> densePoints(const Span& s) {
    std::vector<Vec2> pts;
    std::size_t n = 1;
    if (const auto* arc = std::get_if<ArcSpan>(&s)) {
        n = std::max<std::size_t>(16, arc->segmentCountFor(1e-4));
    } else if (std::holds_alternative<BezierSpan>(s)) {
        n = 160;
    }
    for (std::size_t k = 0; k <= n; ++k) {
        pts.push_back(pointAt(s, static_cast<double>(k) / static_cast<double>(n)));
    }
    return pts;
}

struct Seg {
    Vec2 a;
    Vec2 b;
    std::size_t span;
    double s0;  // arc length along the contour at a
};

double segDistance(const Seg& p, const Seg& q) {
    const auto pointSeg = [](Vec2 x, Vec2 a, Vec2 b) {
        const Vec2 d = b - a;
        const double l2 = d.lengthSquared();
        const double t = l2 > 0.0 ? std::clamp((x - a).dot(d) / l2, 0.0, 1.0) : 0.0;
        return distance(x, a + d * t);
    };
    return std::min(
        {pointSeg(p.a, q.a, q.b), pointSeg(p.b, q.a, q.b), pointSeg(q.a, p.a, p.b), pointSeg(q.b, p.a, p.b)});
}

enum class Verdict { Crossing, Clear, Ambiguous };

// Brute-force classification of a closed contour. Each span is densely polygonised and
// consecutive spans share their junction vertex exactly; segment pairs that share a vertex
// (neighbours within a span, or the two segments meeting at a junction) are the only contacts
// excluded. Crossings near a junction, shallow crossings and near misses are Ambiguous.
Verdict classify(const Contour& c) {
    const std::size_t n = c.spans.size();
    std::vector<std::vector<Vec2>> pts(n);
    for (std::size_t i = 0; i < n; ++i) {
        pts[i] = densePoints(c.spans[i]);
    }
    for (std::size_t i = 0; i < n; ++i) {
        pts[i].back() = pts[(i + 1) % n].front();  // exact shared junction vertex
    }
    std::vector<Seg> segs;
    double s = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t k = 1; k < pts[i].size(); ++k) {
            segs.push_back({pts[i][k - 1], pts[i][k], i, s});
            s += distance(pts[i][k - 1], pts[i][k]);
        }
    }
    const double total = s;
    std::vector<Vec2> junctions;
    for (std::size_t i = 0; i < n; ++i) {
        junctions.push_back(pts[i].front());
        // Spans leaving a junction at a small angle (a cusp) make the neighbourhood ambiguous.
        const Vec2 back = tangentAt(c.spans[(i + n - 1) % n], 1.0) * -1.0;
        const Vec2 out = tangentAt(c.spans[i], 0.0);
        if (std::fabs(std::atan2(back.cross(out), back.dot(out))) < 0.25) {
            return Verdict::Ambiguous;
        }
    }
    constexpr double kJunctionBall = 0.05;
    constexpr double kClear = 0.02;
    constexpr double kMinGap = 0.3;  // arc length between two pieces of the same span
    const auto nearJunction = [&](Vec2 p) {
        return std::any_of(junctions.begin(), junctions.end(),
                           [&](Vec2 j) { return distance(p, j) <= kJunctionBall; });
    };
    bool crossing = false;
    bool ambiguous = false;
    const std::size_t m = segs.size();
    for (std::size_t x = 0; x < m; ++x) {
        const Seg& p = segs[x];
        const Box2 bp = Box2{p.a, p.b}.inflated(kClear);
        for (std::size_t y = x + 1; y < m; ++y) {
            const Seg& q = segs[y];
            const bool adjacent = y == x + 1 || (x == 0 && y == m - 1);
            if (adjacent || !bp.intersects(Box2{q.a, q.b})) {
                continue;
            }
            const Vec2 dp = p.b - p.a;
            const Vec2 dq = q.b - q.a;
            const double o1 = dp.cross(q.a - p.a);
            const double o2 = dp.cross(q.b - p.a);
            const double o3 = dq.cross(p.a - q.a);
            const double o4 = dq.cross(p.b - q.a);
            const double gap = std::min(q.s0 - p.s0, total - (q.s0 - p.s0));
            const bool sameSpanClose = p.span == q.span && gap < kMinGap;
            if (o1 * o2 < 0.0 && o3 * o4 < 0.0) {
                const Vec2 hit = p.a + dp * (o3 / (o3 - o4));
                const double sinAngle = std::fabs(dp.cross(dq)) / (dp.length() * dq.length());
                if (nearJunction(hit) || sameSpanClose || sinAngle < 0.05) {
                    ambiguous = true;
                } else {
                    crossing = true;
                }
                continue;
            }
            if (o1 * o2 <= 0.0 && o3 * o4 <= 0.0) {
                ambiguous = true;  // touching or collinear polygon pieces
                continue;
            }
            if (sameSpanClose || (nearJunction(p.a) && nearJunction(p.b)) ||
                (nearJunction(q.a) && nearJunction(q.b))) {
                continue;
            }
            if (segDistance(p, q) <= kClear) {
                ambiguous = true;
            }
        }
    }
    if (crossing) {
        return Verdict::Crossing;
    }
    return ambiguous ? Verdict::Ambiguous : Verdict::Clear;
}

Vec2 randomPoint(SplitMix& rng) {
    return {rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)};
}

// A span from p to q: kind 0 line, 1 arc (sweep up to 1.6*pi either way), 2 Bezier.
Span spanBetween(SplitMix& rng, int kind, Vec2 p, Vec2 q) {
    if (kind == 0) {
        return LineSpan{p, q};
    }
    if (kind == 1) {
        const double sweep = rng.uniform(0.3, 1.6 * kPi);
        const ArcDirection dir = rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
        const Vec2 chord = q - p;
        const Vec2 left = (chord / chord.length()).perp();
        const double off = 0.5 * chord.length() / std::tan(0.5 * sweep);
        const Vec2 centre = (p + q) * 0.5 + left * (dir == ArcDirection::CCW ? off : -off);
        return ArcSpan::fromCentre(centre, p, q, dir);
    }
    const auto ctrl = [&](Vec2 base) {
        return base + Vec2::fromAngle(rng.uniform(0.0, kTwoPi), rng.uniform(0.5, 9.0));
    };
    return BezierSpan{p, ctrl(p), ctrl(q), q};
}

std::optional<Contour> randomClosed(SplitMix& rng, std::size_t spans) {
    if (spans == 1) {
        if (rng.uniform(0.0, 1.0) < 0.2) {
            const double a = rng.uniform(-kPi, kPi);
            return closedOf({ArcSpan{randomPoint(rng), rng.logUniform(0.5, 10.0), a, a,
                                     rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CW : ArcDirection::CCW}});
        }
        const Vec2 p = randomPoint(rng);
        return closedOf({BezierSpan{p, randomPoint(rng), randomPoint(rng), p}});
    }
    std::vector<Vec2> v;
    for (std::size_t i = 0; i < spans; ++i) {
        v.push_back(randomPoint(rng));
    }
    for (std::size_t i = 0; i < spans; ++i) {
        for (std::size_t j = i + 1; j < spans; ++j) {
            if (distance(v[i], v[j]) < 1.0) {
                return std::nullopt;
            }
        }
    }
    Contour c;
    c.closed = true;
    int lines = 0;
    for (std::size_t i = 0; i < spans; ++i) {
        const int kind = static_cast<int>(rng.next() % 3);
        lines += kind == 0 ? 1 : 0;
        c.append(spanBetween(rng, kind, v[i], v[(i + 1) % spans]));
    }
    if (spans == 2 && lines == 2) {
        return std::nullopt;  // a line retracing itself: a fold-back overlap, not a junction case
    }
    return c;
}
}  // namespace

TEST_CASE("T-0005 AC1: generative 1-span to 3-span closed contours match a brute-force reference",
          "[geometry][intersect][t0005][property]") {
    SplitMix rng{0x7005AC1ULL};
    std::array<std::size_t, 4> clear{};
    std::array<std::size_t, 4> crossing{};
    for (int i = 0; i < 1500; ++i) {
        const std::size_t spans = 1 + static_cast<std::size_t>(i % 3);
        const std::optional<Contour> c = randomClosed(rng, spans);
        if (!c) {
            continue;
        }
        const Verdict v = classify(*c);
        if (v == Verdict::Ambiguous) {
            continue;
        }
        INFO("case " << i << " spans=" << spans
                     << " verdict=" << (v == Verdict::Crossing ? "crossing" : "clear"));
        const ContourIntersections r = selfIntersections(*c, tol);
        CHECK(hasSelfIntersections(*c, tol) == !r.isEmpty());
        if (v == Verdict::Crossing) {
            CHECK_FALSE(r.isEmpty());
            ++crossing[spans];
        } else {
            CHECK(r.isEmpty());
            ++clear[spans];
        }
    }
    CHECK(clear[1] > 200);
    CHECK(clear[2] > 100);
    CHECK(crossing[2] > 30);
    CHECK(clear[3] > 50);
    CHECK(crossing[3] > 50);
}

// ============================================ AC3 round 2: crossings near a cubic's loop

TEST_CASE("T-0005 AC3: crossing between different branches near a cubic's loop is found",
          "[geometry][intersect][t0005]") {
    // Two nearly identical cubics with a tight loop near t ~ 0.63. A crossing of A's incoming
    // branch with B's outgoing branch (tA 0.5809, tB 0.6657) used to be absorbed by crossings of
    // nearby branches (verifier regression, round 2).
    const BezierSpan a{{5.8325634714396291, -2.578300261192763},
                       {0.764443054540072, 0.35273779267821048},
                       {8.135068096369956, -9.9568634983255482},
                       {1.1159080799683156, 1.3804952478029442}};
    const BezierSpan b{{5.8322141843730844, -2.5784758056482553},
                       {0.76415639146360825, 0.35347044054822863},
                       {8.1357061808389961, -9.9573664441958165},
                       {1.115994241592503, 1.3808138036843201}};
    CHECK(symmetricPair(a, b));
    const IntersectionResult r = intersect(Span{a}, Span{b}, tol);
    const auto has = [&](double tA, double tB) {
        return std::any_of(r.points.begin(), r.points.end(), [&](const Intersection& p) {
            return std::fabs(p.tA - tA) <= 1e-6 && std::fabs(p.tB - tB) <= 1e-6 && !p.tangent;
        });
    };
    // Every crossing of the two curves (dense brute force), plus the genuine closest approaches
    // within tolerance: a dip of the gap to 2.08e-4 near t = 0.63 and the ends, which are 3.3e-4
    // apart (closest 2.4e-4 just before B's end). Since T-0008 the starts, whose gap is 3.3e-4 to
    // 3.9e-4, are a contact too (an end within tolerance of the other span). Nothing else.
    CHECK(has(0.109388826, 0.109351239));
    CHECK(has(0.375244198, 0.375270185));
    CHECK(has(0.580903743, 0.665673934));
    CHECK(has(0.633110476, 0.615930502));
    CHECK(has(0.657644445, 0.589863425));
    REQUIRE(r.points.size() == 8);
    CHECK(r.points[0].tangent);
    CHECK(r.points[0].tB == 0.0);
    CHECK_THAT(r.points[0].tA, Catch::Matchers::WithinAbs(0.0000122, 1e-6));
    CHECK(approxEqual(r.points[3].point, {4.333550126, -4.035616762}, 1e-8));
    CHECK(r.points[4].tangent);
    CHECK_THAT(r.points[4].tA, Catch::Matchers::WithinAbs(0.629703, 1e-5));
    CHECK_THAT(r.points[4].tB, Catch::Matchers::WithinAbs(0.629877, 1e-5));
    CHECK(r.points[7].tA == 1.0);
    CHECK_THAT(r.points[7].tB, Catch::Matchers::WithinAbs(0.999994, 1e-5));
    CHECK(r.points[7].tangent);
}

namespace {
// Exact crossings of two cubics by brute force: dense parameter-uniform polylines (chord error
// ~5e-6 here), proper segment crossings found through a uniform grid, each refined by Newton on
// the true curves. Returns (tA, tB, sin of crossing angle) for the refined roots. (Two crossings
// within one pair of polyline segments, ~0.01 mm, can be missed; the test only checks roots found.)
struct BruteRoot {
    double tA;
    double tB;
    double sinAngle;
};

std::vector<BruteRoot> bruteCrossings(const BezierSpan& a, const BezierSpan& b) {
    constexpr std::size_t kN = 2000;
    constexpr double kCell = 0.05;
    std::vector<Vec2> pa(kN + 1);
    std::vector<Vec2> pb(kN + 1);
    for (std::size_t k = 0; k <= kN; ++k) {
        pa[k] = a.pointAt(static_cast<double>(k) / kN);
        pb[k] = b.pointAt(static_cast<double>(k) / kN);
    }
    Box2 all = a.bounds();
    all.expand(b.bounds());
    const auto cellOf = [&](double v, double lo) { return static_cast<long>(std::floor((v - lo) / kCell)); };
    const long nx = cellOf(all.max.x, all.min.x) + 1;
    const long ny = cellOf(all.max.y, all.min.y) + 1;
    std::vector<std::vector<std::size_t>> grid(static_cast<std::size_t>(nx * ny));
    for (std::size_t j = 0; j < kN; ++j) {
        const Box2 bx{pb[j], pb[j + 1]};
        for (long x = cellOf(bx.min.x, all.min.x); x <= cellOf(bx.max.x, all.min.x); ++x) {
            for (long y = cellOf(bx.min.y, all.min.y); y <= cellOf(bx.max.y, all.min.y); ++y) {
                grid[static_cast<std::size_t>(y * nx + x)].push_back(j);
            }
        }
    }
    std::vector<BruteRoot> roots;
    std::vector<std::size_t> seen;
    for (std::size_t i = 0; i < kN; ++i) {
        const Box2 ax{pa[i], pa[i + 1]};
        seen.clear();
        for (long x = cellOf(ax.min.x, all.min.x); x <= cellOf(ax.max.x, all.min.x); ++x) {
            for (long y = cellOf(ax.min.y, all.min.y); y <= cellOf(ax.max.y, all.min.y); ++y) {
                for (const std::size_t j : grid[static_cast<std::size_t>(y * nx + x)]) {
                    if (std::find(seen.begin(), seen.end(), j) != seen.end()) {
                        continue;
                    }
                    seen.push_back(j);
                    const Vec2 da = pa[i + 1] - pa[i];
                    const Vec2 db = pb[j + 1] - pb[j];
                    const double o1 = da.cross(pb[j] - pa[i]);
                    const double o2 = da.cross(pb[j + 1] - pa[i]);
                    const double o3 = db.cross(pa[i] - pb[j]);
                    const double o4 = db.cross(pa[i + 1] - pb[j]);
                    if (!(o1 * o2 < 0.0 && o3 * o4 < 0.0)) {
                        continue;
                    }
                    double t = (static_cast<double>(i) + o3 / (o3 - o4)) / kN;
                    double u = (static_cast<double>(j) + o1 / (o1 - o2)) / kN;
                    const double t0 = t;
                    const double u0 = u;
                    for (int iter = 0; iter < 50; ++iter) {
                        const Vec2 r = a.pointAt(t) - b.pointAt(u);
                        const Vec2 ta = a.derivativeAt(t);
                        const Vec2 tb = b.derivativeAt(u);
                        const double det = -ta.cross(tb);
                        if (det == 0.0) {
                            break;
                        }
                        // Solve [ta, -tb] (dt, du) = -r by Cramer's rule.
                        t += r.cross(tb) / det;
                        u -= ta.cross(r) / det;
                    }
                    if (std::fabs(t - t0) > 1e-3 || std::fabs(u - u0) > 1e-3 || t <= 1e-6 ||
                        t >= 1.0 - 1e-6 || u <= 1e-6 || u >= 1.0 - 1e-6 ||
                        distance(a.pointAt(t), b.pointAt(u)) > 1e-10) {
                        continue;  // Newton did not settle on this crossing: not a clear-cut root
                    }
                    const Vec2 ta = a.tangentAt(t);
                    roots.push_back({t, u, std::fabs(ta.cross(b.tangentAt(u)))});
                }
            }
        }
    }
    return roots;
}
}  // namespace

TEST_CASE("T-0005 AC3: generative perturbed copies of looping and tightly bent cubics vs brute force",
          "[geometry][intersect][t0005][property]") {
#ifdef NDEBUG
    constexpr int kCases = 1500;
#else
    constexpr int kCases = 140;
#endif
    SplitMix rng{0x7005B0BULL};
    std::size_t checked = 0;
    std::size_t looping = 0;
    int i = 0;
    for (int made = 0; made < kCases; ++i) {
        const auto pt = [&](double e) { return Vec2{rng.uniform(-e, e), rng.uniform(-e, e)}; };
        const BezierSpan a{pt(5.0), pt(10.0), pt(10.0), pt(5.0)};
        // Keep cubics with a loop or a tight bend: the speed |B'| drops well below its maximum.
        double vmin = 1e300;
        double vmax = 0.0;
        for (int k = 0; k <= 64; ++k) {
            const double v = a.derivativeAt(static_cast<double>(k) / 64.0).length();
            vmin = std::min(vmin, v);
            vmax = std::max(vmax, v);
        }
        Contour single;
        single.append(a);
        const bool loop = hasSelfIntersections(single, tol);
        if (!loop && vmin > 0.15 * vmax) {
            continue;
        }
        ++made;
        looping += loop ? 1u : 0u;
        const BezierSpan b = jiggled(a, rng, rng.uniform(0.2, 1.0) * tol.distance);
        const std::vector<BruteRoot> roots = bruteCrossings(a, b);
        IntersectionResult r;
        CHECK(symmetricPair(a, b, &r));
        for (std::size_t k = 0; k < roots.size(); ++k) {
            const Vec2 pk = a.pointAt(roots[k].tA);
            bool isolated = roots[k].sinAngle > 1e-4;
            for (std::size_t m = 0; m < roots.size(); ++m) {
                isolated = isolated && (m == k || distance(pk, a.pointAt(roots[m].tA)) > 2.0 * tol.distance);
            }
            if (!isolated) {
                continue;
            }
            ++checked;
            const bool found = std::any_of(r.points.begin(), r.points.end(), [&](const Intersection& p) {
                return distance(p.point, pk) <= tol.distance;
            });
            INFO("case " << i << " root tA=" << roots[k].tA << " tB=" << roots[k].tB);
            CHECK(found);
        }
    }
    CHECK(looping > kCases / 4);
    CHECK(checked > static_cast<std::size_t>(kCases));
}

// ===================================================== AC3 round 3: genuine touches within tol

namespace {
// Convex cubics (curvature of one sign) used as the touched curve.
const std::array<BezierSpan, 3> kConvex{{
    BezierSpan{{0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {3.0, 0.0}},
    BezierSpan{{0.0, 0.0}, {0.0, 2.5}, {4.0, 2.5}, {4.0, 0.0}},
    BezierSpan{{-1.0, 0.0}, {0.5, 3.0}, {1.0, 3.0}, {3.0, -1.0}},
}};

// Unit normal at A(t) pointing to the convex side (away from the centre of curvature).
Vec2 convexNormal(const BezierSpan& a, double t) {
    Vec2 n = a.tangentAt(t).perp();
    const Vec2 ahead = a.pointAt(std::min(1.0, t + 0.05)) - a.pointAt(t);
    return n.dot(ahead) > 0.0 ? n * -1.0 : n;
}

double curvatureAt(const BezierSpan& a, double t) {
    const Vec2 d1 = a.derivativeAt(t);
    const Vec2 d2 = ((a.p2 - a.p1 * 2.0 + a.p0) * (1.0 - t) + (a.p3 - a.p2 * 2.0 + a.p1) * t) * 6.0;
    return std::fabs(d1.cross(d2)) / std::pow(d1.length(), 3.0);
}

// Spans that touch the convex cubic `a` at A(t) with a gap of `h` and never cross it (the internal
// circle sits on the concave side, everything else on the convex side).
struct Toucher {
    const char* name;
    Span span;
};

std::vector<Toucher> touchers(const BezierSpan& a, double t, double h) {
    const Vec2 p = a.pointAt(t);
    const Vec2 tg = a.tangentAt(t);
    const Vec2 n = convexNormal(a, t);
    const Vec2 q = p + n * h;  // the touching point on the other span
    std::vector<Toucher> out;
    out.push_back({"line", LineSpan{q - tg * 0.7, q + tg * 0.7}});
    // External circle on the convex side (any radius).
    const double rExt = 0.8;
    out.push_back({"external circle", ArcSpan::fullCircle(q + n * rExt, rExt)});
    // Internal circle on the concave side, more tightly curved than the curve anywhere so that the
    // curve stays outside it.
    double kMax = 0.0;
    for (int k = 0; k <= 50; ++k) {
        kMax = std::max(kMax, curvatureAt(a, k / 50.0));
    }
    const double rInt = 0.6 / kMax;
    out.push_back({"internal circle", ArcSpan::fullCircle(p - n * (h + rInt), rInt)});
    // Mirror image of the cubic in the tangent line at A(t), shifted out by h.
    const auto mirror = [&](Vec2 x) {
        const Vec2 d = x - p;
        return q + tg * d.dot(tg) - n * d.dot(n);
    };
    out.push_back({"mirrored Bezier", BezierSpan{mirror(a.p0), mirror(a.p1), mirror(a.p2), mirror(a.p3)}});
    return out;
}
}  // namespace

TEST_CASE("T-0005 AC3: a tangent line shifted 0.1*tol off a convex cubic is one tangent hit",
          "[geometry][intersect][t0005]") {
    // Verifier repro (round 3): the tangent line at A(0.3), shifted 1e-4 to the convex side.
    const BezierSpan a{{0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {3.0, 0.0}};
    const LineSpan b{{0.43572451548973479, 0.4443975094920366}, {1.364201206374994, 0.81578818584614032}};
    IntersectionResult r;
    CHECK(symmetricPair(a, b, &r));
    REQUIRE(r.points.size() == 1);
    CHECK(r.points[0].tangent);
    CHECK_THAT(r.points[0].tA, Catch::Matchers::WithinAbs(0.3, 1e-6));
    CHECK_THAT(distance(a.pointAt(r.points[0].tA), b.pointAt(r.points[0].tB)),
               Catch::Matchers::WithinAbs(1e-4, 1e-7));
    const IntersectionResult rb = intersect(Span{b}, Span{a}, tol);
    REQUIRE(rb.points.size() == 1);
    CHECK(rb.points[0].tangent);
}

TEST_CASE("T-0005 AC3: touches within tolerance are reported once and gaps beyond it are not (h-sweep)",
          "[geometry][intersect][t0005][property]") {
    // Tangent lines, external and internal circles and mirrored cubics at t = i/40 on three convex
    // cubics, offset by h to the convex side: exactly one tangent hit for every h within tolerance
    // (0, 1e-5, 1e-4, 5e-4, 0.99*tol), none at 1.05*tol. Since T-0008 an end of either span that
    // comes within tolerance of the other further along (e.g. the end of the cubic next to a long
    // tangent line) is reported as well, as its own end contact.
    std::size_t touches = 0;
    std::size_t misses = 0;
    for (const BezierSpan& a : kConvex) {
        for (int i = 2; i < 40; ++i) {
            const double t = i / 40.0;
            for (const double h : {0.0, 1e-5, 1e-4, 5e-4, 0.99 * tol.distance, 1.05 * tol.distance}) {
                for (const Toucher& tc : touchers(a, t, h)) {
                    INFO(tc.name << " t=" << t << " h=" << h);
                    IntersectionResult r;
                    CHECK(symmetricPair(a, tc.span, &r));
                    if (h <= tol.distance) {
                        std::vector<Intersection> touch;
                        for (const Intersection& p : r.points) {
                            if (std::fabs(p.tA - t) <= 2e-3) {
                                touch.push_back(p);
                            } else {
                                CHECK(isEndContact(a, tc.span, p));
                                CHECK(p.tangent);
                            }
                        }
                        REQUIRE(touch.size() == 1);
                        CHECK(touch[0].tangent);
                        CHECK(distance(a.pointAt(touch[0].tA), pointAt(tc.span, touch[0].tB)) <= h + 1e-6);
                        ++touches;
                    } else {
                        CHECK(r.isEmpty());
                        ++misses;
                    }
                }
            }
        }
    }
    CHECK(touches == 3 * 38 * 5 * 4);
    CHECK(misses == 3 * 38 * 4);
}

TEST_CASE("T-0005 AC3: curves hugging within tolerance report crossings and true closest approaches only",
          "[geometry][intersect][t0005][property]") {
    // Perturbed copies of random cubics (control points moved by up to 0.9*tol). The reference is
    // the global gap g(t) = dist(A(t), B) sampled densely: every reported hit must sit at a local
    // minimum of g (a crossing has g = 0) or be an end contact (T-0008: an end of either span within
    // tolerance of the other), and every local minimum with g < 0.9*tol that is not within 3*tol
    // of another one must be reported.
#ifdef NDEBUG
    constexpr int kCases = 400;
#else
    constexpr int kCases = 60;
#endif
    SplitMix rng{0x7005B0CULL};
    std::size_t checkedMinima = 0;
    std::size_t touchHits = 0;
    for (int i = 0; i < kCases; ++i) {
        const auto pt = [&](double e) { return Vec2{rng.uniform(-e, e), rng.uniform(-e, e)}; };
        const BezierSpan a{pt(4.0), pt(6.0), pt(6.0), pt(4.0)};
        const BezierSpan b = jiggled(a, rng, rng.uniform(0.1, 0.9) * tol.distance);
        IntersectionResult r;
        CHECK(symmetricPair(a, b, &r));
        constexpr int kN = 800;
        std::vector<double> g(kN + 1);
        for (int k = 0; k <= kN; ++k) {
            g[static_cast<std::size_t>(k)] = closestPoint(b, a.pointAt(static_cast<double>(k) / kN)).distance;
        }
        std::vector<double> minima;  // parameters of local minima (ends included when they dip)
        for (int k = 0; k <= kN; ++k) {
            const auto at = [&](int m) { return g[static_cast<std::size_t>(m)]; };
            const bool lowerLeft = k == 0 || at(k) <= at(k - 1);
            const bool lowerRight = k == kN || at(k) < at(k + 1);
            if (lowerLeft && lowerRight) {
                minima.push_back(static_cast<double>(k) / kN);
            }
        }
        // Every hit is a crossing (zero gap) or a local minimum of the gap: g at the hit equals its
        // residual and does not drop within a step either side (a branch crossing is a V of width
        // ~1e-5 in t, which the sample grid cannot see, hence the direct check).
        for (const Intersection& p : r.points) {
            const double res = distance(a.pointAt(p.tA), b.pointAt(p.tB));
            const double gHere = closestPoint(b, a.pointAt(p.tA)).distance;
            INFO("case " << i << " hit tA=" << p.tA << " tangent=" << p.tangent << " res=" << res);
            if (isEndContact(a, b, p)) {
                // An end paired with its nearest point on the other span: the residual is the end's
                // distance to that span (the other point's own nearest point may lie a little closer).
                const bool endOfA = (p.tA == 0.0 || p.tA == 1.0) &&
                                    std::fabs(closestPoint(b, a.pointAt(p.tA)).distance - res) <= 1e-12;
                const bool endOfB = (p.tB == 0.0 || p.tB == 1.0) &&
                                    std::fabs(closestPoint(a, b.pointAt(p.tB)).distance - res) <= 1e-12;
                CHECK((endOfA || endOfB));
            } else {
                CHECK(gHere >= res - 1e-9);
            }
            if (res > 1e-9) {
                CHECK(p.tangent);
            }
            if (res > 1e-9 && !isEndContact(a, b, p)) {
                for (const double d : {-1e-4, -1e-5, 1e-5, 1e-4}) {
                    const double t = p.tA + d;
                    if (t >= 0.0 && t <= 1.0) {
                        CHECK(closestPoint(b, a.pointAt(t)).distance >= gHere - 1e-12);
                    }
                }
            }
            touchHits += res > 1e-9 ? 1u : 0u;
        }
        // Every isolated deep-enough minimum is reported.
        for (std::size_t k = 0; k < minima.size(); ++k) {
            const Vec2 pk = a.pointAt(minima[k]);
            const double gk = closestPoint(b, pk).distance;
            bool isolated = gk < 0.9 * tol.distance;
            for (std::size_t m = 0; m < minima.size(); ++m) {
                isolated = isolated && (m == k || distance(pk, a.pointAt(minima[m])) > 3.0 * tol.distance);
            }
            if (!isolated) {
                continue;
            }
            ++checkedMinima;
            // Found by position: two crossings of different branches at (nearly) the same place
            // merge into one hit that carries the parameters of only one of them.
            const bool found = std::any_of(r.points.begin(), r.points.end(), [&](const Intersection& p) {
                return std::fabs(p.tA - minima[k]) <= 1.5 / kN || distance(p.point, pk) <= tol.distance;
            });
            INFO("case " << i << " minimum tA=" << minima[k] << " gap=" << gk);
            CHECK(found);
        }
    }
    CHECK(checkedMinima > static_cast<std::size_t>(kCases));
    CHECK(touchHits > static_cast<std::size_t>(kCases) / 4);
}
