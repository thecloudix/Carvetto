// T-0007: intersection residuals from T-0005's third verification round.
//   AC1  external circles and arcs touching a convex cubic within tolerance, at every position;
//   AC2  end touches are reported whichever span's end comes near the other;
//   AC3  thin (sliver) self-loops of a single cubic;
//   AC4  a Bezier re-crossing the arc it joins, close to the junction.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {
const Tolerance tol{};

// intersect(a, b) and intersect(b, a) must be exact mirror images (bit-identical points and
// parameters, same flags), and every reported point lies within tolerance of both spans.
bool mirrored(const Span& a, const Span& b, IntersectionResult* result = nullptr) {
    const IntersectionResult ab = intersect(a, b, tol);
    const IntersectionResult ba = intersect(b, a, tol);
    if (result != nullptr) {
        *result = ab;
    }
    bool ok = ab.points.size() == ba.points.size() && ab.overlaps.size() == ba.overlaps.size();
    for (const Intersection& p : ab.points) {
        ok = ok && distance(p.point, pointAt(a, p.tA)) <= tol.distance &&
             distance(p.point, pointAt(b, p.tB)) <= tol.distance;
        ok = ok && std::any_of(ba.points.begin(), ba.points.end(), [&](const Intersection& q) {
                 return p.point.x == q.point.x && p.point.y == q.point.y && p.tA == q.tB && p.tB == q.tA &&
                        p.tangent == q.tangent;
             });
    }
    return ok;
}

// Unit normal at A(t) pointing to the convex side (away from the centre of curvature).
Vec2 convexNormal(const BezierSpan& a, double t) {
    const Vec2 n = a.tangentAt(t).perp();
    // The second derivative's normal component points to the concave side.
    const Vec2 d2 = ((a.p2 - a.p1 * 2.0 + a.p0) * (1.0 - t) + (a.p3 - a.p2 * 2.0 + a.p1) * t) * 6.0;
    return n.dot(d2) > 0.0 ? n * -1.0 : n;
}

// Convex cubics (curvature of one sign) used as the touched curve. The last one is the steep
// parabola-like cubic of the verifier's regressions, whose pieces are cut at multiples of 1/16.
const std::array<BezierSpan, 4> kConvex{{
    BezierSpan{{0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {3.0, 0.0}},
    BezierSpan{{0.0, 0.0}, {0.0, 2.5}, {4.0, 2.5}, {4.0, 0.0}},
    BezierSpan{{-1.0, 0.0}, {0.5, 3.0}, {1.0, 3.0}, {3.0, -1.0}},
    BezierSpan{{-3.0, 9.0}, {-1.0, -3.0}, {1.0, -3.0}, {3.0, 9.0}},
}};

}  // namespace

// ============================================================================ AC1

TEST_CASE("T-0007 AC1: verifier regressions, circles touching a cubic near piece cuts and the span start",
          "[geometry][intersect][t0007]") {
    struct Case {
        BezierSpan a;
        Vec2 centre;
        double tA;
    };
    const BezierSpan a1{{-3, 9}, {-1, -3}, {1, -3}, {3, 9}};
    const BezierSpan a2{{-1, 0}, {0.5, 3}, {1, 3}, {3, -1}};
    const std::array<Case, 5> cases{{
        {a1, {-2.4572958060271235, 5.0734751657764976}, 0.123},
        {a1, {-2.0592815815728511, 3.4301656512398564}, 0.189},
        {a2, {-0.92142472309476964, 0.59261447867484451}, 0.059},
        {a2, {2.2044174888982013, 0.91064133105013123}, 0.816},
        {a2, {-1.1657867390182135, 0.11673084174268154}, 0.003},
    }};
    for (const Case& c : cases) {
        const ArcSpan circle = ArcSpan::fullCircle(c.centre, 0.2);
        INFO("tA=" << c.tA);
        IntersectionResult r;
        CHECK(mirrored(c.a, circle, &r));
        REQUIRE(r.points.size() == 1);
        CHECK(r.points[0].tangent);
        CHECK_THAT(r.points[0].tA, Catch::Matchers::WithinAbs(c.tA, 1e-6));
        const double gap = distance(c.a.pointAt(r.points[0].tA), circle.pointAt(r.points[0].tB));
        CHECK(gap <= tol.distance);
        CHECK(gap > 0.0);
    }
}

TEST_CASE("T-0007 AC1: fine sweep of external circles and arcs touching convex cubics (t = i/1000)",
          "[geometry][intersect][t0007][property]") {
    // Every position t = i/1000 on four convex cubics, external circles of radius 0.2 and 1.0
    // touching at gap h on the convex side: exactly one tangent hit near A(t), in both argument
    // orders (bit-identical mirror images). A short arc of the same circle around the touch point
    // must give the same, as must the circle with its seam turned to face the cubic.
    //
    // Optimised builds run every position with every variant in both orders. Debug builds (no
    // NDEBUG) take every 3rd position, offset per configuration (0.003 apart, still well inside
    // the ~0.007 wide bands this guards against), the arc variants every 21st, and check the
    // mirrored order on every 5th call.
    std::size_t touches = 0;
    std::size_t drops = 0;
#ifdef NDEBUG
    constexpr int kStride = 1;
    constexpr int kArcEvery = 1;
    constexpr std::size_t kMirrorEvery = 1;
#else
    constexpr int kStride = 3;
    constexpr int kArcEvery = 21;
    constexpr std::size_t kMirrorEvery = 5;
#endif
    int config = 0;
    for (std::size_t ci = 0; ci < kConvex.size(); ++ci) {
        const BezierSpan& a = kConvex[ci];
        for (const double radius : {0.2, 1.0}) {
            for (const double h : {1e-5, 1e-4, 0.99 * tol.distance}) {
                for (int i = config++ % kStride; i <= 1000; i += kStride) {
                    const double t = i / 1000.0;
                    const Vec2 n = convexNormal(a, t);
                    const Vec2 centre = a.pointAt(t) + n * (h + radius);
                    const double facing = (n * -1.0).angle();  // angle of the touch point on the circle
                    std::vector<Span> others{ArcSpan::fullCircle(centre, radius)};
                    if (i % kArcEvery == 0) {
                        others.push_back(
                            ArcSpan{centre, radius, facing - 0.4, facing + 0.5, ArcDirection::CCW});
                        others.push_back(
                            ArcSpan{centre, radius, facing + 0.5, facing - 0.4, ArcDirection::CW});
                        others.push_back(
                            ArcSpan{centre, radius, facing, facing, ArcDirection::CW});  // seam at touch
                    }
                    for (const Span& other : others) {
                        IntersectionResult r;
                        bool symmetric = true;
                        if (touches % kMirrorEvery == 0) {
                            symmetric = mirrored(a, other, &r);
                        } else {
                            r = intersect(Span{a}, other, tol);
                        }
                        const bool ok =
                            symmetric && r.points.size() == 1 && r.points[0].tangent &&
                            std::fabs(r.points[0].tA - t) <= 2e-3 &&
                            distance(a.pointAt(r.points[0].tA), pointAt(other, r.points[0].tB)) <= h + 1e-9;
                        if (!ok) {
                            ++drops;
                            UNSCOPED_INFO("cubic " << ci << " R=" << radius << " h=" << h << " t=" << t
                                                   << " hits=" << r.points.size()
                                                   << " symmetric=" << symmetric);
                        }
                        ++touches;
                    }
                }
            }
        }
    }
    CHECK(drops == 0);
    CHECK(touches >= 4u * 2u * 3u * (1000u / static_cast<std::size_t>(kStride)));
}

TEST_CASE("T-0007 AC1: circles just beyond tolerance give nothing", "[geometry][intersect][t0007]") {
    for (const BezierSpan& a : kConvex) {
        for (int i = 0; i <= 200; ++i) {
            const double t = i / 200.0;
            const Vec2 n = convexNormal(a, t);
            const ArcSpan c = ArcSpan::fullCircle(a.pointAt(t) + n * (1.05 * tol.distance + 0.2), 0.2);
            INFO("t=" << t);
            CHECK(intersect(Span{a}, Span{c}, tol).isEmpty());
        }
    }
}

// ============================================================================ AC2

TEST_CASE("T-0007 AC2: verifier regression, both end touches of an arc and a Bezier",
          "[geometry][intersect][t0007]") {
    const ArcSpan arc{{1.2115827528094094, 3.0195632709834515},
                      5.9592789586434449,
                      5.5749720172249306,
                      7.0509709398542197,
                      ArcDirection::CCW};
    const BezierSpan bz{{5.4971827427603293, 7.1603566376986816},
                        {7.6315225436121663, 4.949460611577611},
                        {7.7363570760098641, 1.4786963115444329},
                        {5.7389721228483266, -0.85517614189571634}};
    IntersectionResult r;
    CHECK(mirrored(arc, bz, &r));
    REQUIRE(r.points.size() == 6);  // two end touches and four crossings
    // The arc's start near the Bezier's end, and the arc's end near the Bezier's start.
    CHECK(r.points.front().tangent);
    CHECK_THAT(r.points.front().tA, Catch::Matchers::WithinAbs(0.0, 1e-3));
    CHECK(r.points.front().tB == 1.0);
    CHECK(r.points.back().tangent);
    CHECK(r.points.back().tA == 1.0);
    CHECK_THAT(r.points.back().tB, Catch::Matchers::WithinAbs(0.000277534, 1e-6));
    for (std::size_t k = 1; k + 1 < r.points.size(); ++k) {
        CHECK_FALSE(r.points[k].tangent);
    }
}

TEST_CASE("T-0007 AC2: an end within tolerance of the other span is a touch whichever span it belongs to",
          "[geometry][intersect][t0007][property]") {
    // A span S ends at gap h from the interior of a curve C (on C's convex side) and leaves it at up
    // to 57 degrees from the normal. C is a convex cubic or an arc, S a line, an arc or a Bezier
    // (at least one of them a Bezier). Whichever is A, there is exactly one hit near S's end,
    // flagged tangent, within h of both; at 1.05 tol there is none.
    SplitMix rng{0x7007AC2ULL};
#ifdef NDEBUG
    constexpr int kCases = 12000;
#else
    constexpr int kCases = 2500;
#endif
    std::size_t touches = 0;
    std::size_t misses = 0;
    std::size_t bad = 0;
    for (int i = 0; i < kCases; ++i) {
        const int ck = i % 4;
        const int sk = static_cast<int>(rng.next() % 3);
        Span c;
        if (ck < 3) {
            c = kConvex[static_cast<std::size_t>(ck)];
        } else {
            c = ArcSpan{{0.0, 0.0},
                        rng.logUniform(0.3, 20.0),
                        rng.uniform(-3.0, 3.0),
                        rng.uniform(-3.0, 3.0),
                        rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CW : ArcDirection::CCW};
        }
        const double t = rng.uniform(0.03, 0.97);
        const Vec2 p = pointAt(c, t);
        Vec2 n = tangentAt(c, t).perp();
        const Vec2 chordMid =
            (pointAt(c, std::max(0.0, t - 0.02)) + pointAt(c, std::min(1.0, t + 0.02))) * 0.5;
        if (n.dot(chordMid - p) > 0.0) {
            n = n * -1.0;
        }
        const std::array<double, 5> gaps{1e-5, 1e-4, 5e-4, 0.99 * tol.distance, 1.05 * tol.distance};
        const double h = gaps[rng.next() % gaps.size()];
        const Vec2 q = p + n * h;
        const double heading = std::atan2(n.y, n.x) + rng.uniform(-1.0, 1.0);
        const Vec2 dir = Vec2::fromAngle(heading, 1.0);
        const double len = rng.uniform(0.3, 2.0);
        Span s;
        if (sk == 0) {
            s = LineSpan{q, q + dir * len};
        } else if (sk == 1) {
            const double r = rng.uniform(2.0, 5.0);
            const bool ccw = rng.uniform(0.0, 1.0) < 0.5;
            const Vec2 centre = q + (ccw ? dir.perp() : dir.perp() * -1.0) * r;
            const double a0 = (q - centre).angle();
            const double sweep = len / r;
            s = ArcSpan{centre, r, a0, ccw ? a0 + sweep : a0 - sweep,
                        ccw ? ArcDirection::CCW : ArcDirection::CW};
        } else {
            const Vec2 side = Vec2::fromAngle(heading + kPi / 2.0, rng.uniform(-0.15, 0.15));
            s = BezierSpan{q, q + dir * (len / 3.0), q + dir * (2.0 * len / 3.0) + side, q + dir * len};
        }
        if (ck == 3 && sk != 2) {
            continue;  // analytic pairs are not part of this test
        }
        if (rng.uniform(0.0, 1.0) < 0.5) {
            s = reversed(s);
        }
        for (const bool sIsA : {false, true}) {
            const Span& a = sIsA ? s : c;
            const Span& b = sIsA ? c : s;
            IntersectionResult r;
            const bool symmetric = mirrored(a, b, &r);
            std::size_t near = 0;
            bool ok = symmetric;
            for (const Intersection& hit : r.points) {
                if (distance(hit.point, q) <= 2.0 * tol.distance) {
                    ++near;
                    ok = ok && hit.tangent && distance(pointAt(a, hit.tA), pointAt(b, hit.tB)) <= h + 1e-9;
                }
            }
            if (h <= tol.distance) {
                ok = ok && near == 1;
                ++touches;
            } else {
                ok = ok && near == 0;
                ++misses;
            }
            if (!ok) {
                ++bad;
                UNSCOPED_INFO("case " << i << " curve " << ck << " span kind " << sk << " h=" << h
                                      << " S is A=" << sIsA << " near hits=" << near);
            }
        }
    }
    CHECK(bad == 0);
    CHECK(touches > static_cast<std::size_t>(kCases));
    CHECK(misses > static_cast<std::size_t>(kCases) / 5);
}

TEST_CASE("T-0007 AC2: an arc ending 16 um from a hugging Bezier's interior, next to a crossing",
          "[geometry][intersect][t0007]") {
    // Along the Bezier the distance keeps falling past the arc's end towards a crossing 0.06 mm
    // away, so the end touch is a minimum along the arc only (found while checking AC2).
    const ArcSpan arc{{7.615754897922443, 2.060748289683028},
                      1.2732541187119284,
                      -0.046424241812903055,
                      -0.63347190916599461,
                      ArcDirection::CW};
    const BezierSpan bz{{8.8242740913118212, 1.6605526140813853},
                        {8.7773397068828647, 1.5168625224733479},
                        {8.7044912219698638, 1.3839224389548523},
                        {8.610537711424648, 1.2659908954420165}};
    IntersectionResult r;
    CHECK(mirrored(arc, bz, &r));
    REQUIRE(r.points.size() == 3);
    CHECK_FALSE(r.points[0].tangent);
    CHECK_FALSE(r.points[1].tangent);
    CHECK(r.points[2].tangent);
    CHECK(r.points[2].tA == 1.0);
    CHECK_THAT(r.points[2].tB, Catch::Matchers::WithinAbs(0.885567757, 1e-8));
    CHECK_THAT(distance(arc.pointAt(1.0), bz.pointAt(r.points[2].tB)),
               Catch::Matchers::WithinAbs(1.653e-5, 1e-8));
}

TEST_CASE("T-0007 AC2: a double crossing confined to the last sample piece before a span end",
          "[geometry][intersect][t0007]") {
    // The Bezier swings across the arc and back within its last ~0.007 of parameter (0.15 mm),
    // ending 0.2 mm inside: both crossings are reported (found while checking AC2).
    const ArcSpan arc{{-3.1052154736786743, 2.2654113437212198},
                      1.3414515037303174,
                      -1.3951806466022543,
                      -2.4180261730015582,
                      ArcDirection::CCW};
    const BezierSpan bz{{-8.8159291567016052, 4.4948356447618334},
                        {-5.6362284890055232, 2.9739326413144602},
                        {-10.723822643183038, 2.2699812908738557},
                        {-3.2616336475393179, 3.5979244888280042}};
    IntersectionResult r;
    CHECK(mirrored(arc, bz, &r));
    REQUIRE(r.points.size() == 2);
    CHECK_THAT(r.points[0].tB, Catch::Matchers::WithinAbs(0.999839726, 1e-8));
    CHECK_THAT(r.points[1].tB, Catch::Matchers::WithinAbs(0.993069174, 1e-8));
    CHECK_FALSE(r.points[0].tangent);
    CHECK_FALSE(r.points[1].tangent);
}

// ============================================================================ AC3

namespace {
// The cubic P(t) = (L t^2, e (t^3 - k t)) on [t0, t1], rotated by `rot` and moved to `at`. It
// crosses itself at t = -sqrt(k) and t = +sqrt(k): a loop L*k long and
// e * 2 k^(3/2) / (3 sqrt(3)) thick (squashed by the small factor e).
BezierSpan loopCubic(double L, double e, double k, double t0, double t1, double rot, Vec2 at) {
    const auto pos = [&](double t) { return Vec2{L * t * t, e * (t * t * t - k * t)}; };
    const auto vel = [&](double t) { return Vec2{2.0 * L * t, e * (3.0 * t * t - k)} * (t1 - t0); };
    const auto place = [&](Vec2 v) {
        return Vec2{v.x * std::cos(rot) - v.y * std::sin(rot), v.x * std::sin(rot) + v.y * std::cos(rot)} +
               at;
    };
    return {place(pos(t0)), place(pos(t0) + vel(t0) / 3.0), place(pos(t1) - vel(t1) / 3.0), place(pos(t1))};
}
}  // namespace

TEST_CASE("T-0007 AC3: thin sliver self-loops of a single cubic are found", "[geometry][intersect][t0007]") {
    // Loops 0.05-0.3 mm long and 0.3-5 um thick, with arms of random length on both sides.
    SplitMix rng{0x7007AC3ULL};
#ifdef NDEBUG
    constexpr int kCases = 20000;
#else
    constexpr int kCases = 3000;
#endif
    std::size_t found = 0;
    for (int i = 0; i < kCases; ++i) {
        const double length = rng.uniform(0.05, 0.3);
        const double thickness = rng.uniform(0.3e-3, 5e-3);
        const double k = rng.uniform(0.2, 2.0);
        const double sk = std::sqrt(k);
        const double e = thickness * 3.0 * std::sqrt(3.0) / (2.0 * k * sk);
        const double t0 = -sk * (1.0 + rng.uniform(0.0, 3.0));
        const double t1 = sk * (1.0 + rng.uniform(0.0, 3.0));
        const BezierSpan b = loopCubic(length / k, e, k, t0, t1, rng.uniform(0.0, kTwoPi),
                                       {rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)});
        Contour c;
        c.append(b);
        const ContourIntersections r = selfIntersections(c, tol);
        const double s = (-sk - t0) / (t1 - t0);
        const double u = (sk - t0) / (t1 - t0);
        const bool ok = r.points.size() == 1 && r.points[0].spanA == 0 && r.points[0].spanB == 0 &&
                        std::fabs(r.points[0].hit.tA - s) <= 1e-6 &&
                        std::fabs(r.points[0].hit.tB - u) <= 1e-6 && !r.points[0].hit.tangent &&
                        hasSelfIntersections(c, tol);
        if (!ok) {
            UNSCOPED_INFO("case " << i << " length=" << length << " thickness=" << thickness
                                  << " hits=" << r.points.size());
        }
        found += ok ? 1u : 0u;
    }
    CHECK(found == static_cast<std::size_t>(kCases));
}

TEST_CASE("T-0007 AC3: loops no larger than the tolerance are cusps, not self-intersections",
          "[geometry][intersect][t0007]") {
    // A loop 0.5 um long: the curve crosses itself, but the whole loop is within tolerance of the
    // crossing point.
    const BezierSpan tiny = loopCubic(0.5e-3, 0.5e-3, 1.0, -3.0, 3.0, 0.3, {1.0, 2.0});
    Contour c;
    c.append(tiny);
    CHECK(selfIntersections(c, tol).isEmpty());
    // The same shape scaled up 1000 times is a proper loop.
    const BezierSpan big = loopCubic(0.5, 0.5, 1.0, -3.0, 3.0, 0.3, {1.0, 2.0});
    Contour d;
    d.append(big);
    const ContourIntersections r = selfIntersections(d, tol);
    REQUIRE(r.points.size() == 1);
    CHECK_THAT(r.points[0].hit.tA, Catch::Matchers::WithinAbs(1.0 / 3.0, 1e-9));
    CHECK_THAT(r.points[0].hit.tB, Catch::Matchers::WithinAbs(2.0 / 3.0, 1e-9));
}

// ============================================================================ AC4

TEST_CASE("T-0007 AC4: verifier regression, Bezier bulging past the arc it joins and re-crossing it",
          "[geometry][intersect][t0007]") {
    const BezierSpan bz{{-0.18860739840346241, 3.8634722338314074},
                        {0.0522167808921572, 4.3830641573127442},
                        {1.6272051316098195, -2.0179279547519577},
                        {1.9578386933824259, -2.3369534285717775}};
    const ArcSpan arc{{6.4363713372296427, 1.9356191918458676},
                      6.1896794294008179,
                      -2.3797254616523524,
                      3.0863225397506588,
                      ArcDirection::CW};
    IntersectionResult r;
    CHECK(mirrored(bz, arc, &r));
    REQUIRE(r.points.size() == 2);  // the re-crossing and the junction
    CHECK_FALSE(r.points[0].tangent);
    CHECK_THAT(r.points[0].tA, Catch::Matchers::WithinAbs(0.994105366, 1e-8));
    CHECK_THAT(r.points[0].tB, Catch::Matchers::WithinAbs(0.0017129725, 1e-8));
    CHECK_THAT(r.points[1].tA, Catch::Matchers::WithinAbs(1.0, 1e-12));
    CHECK_THAT(r.points[1].tB, Catch::Matchers::WithinAbs(0.0, 1e-12));

    Contour c;
    c.append(bz);
    c.append(arc);
    REQUIRE(c.isContinuous(tol));
    const ContourIntersections self = selfIntersections(c, tol);
    REQUIRE(self.points.size() == 1);
    CHECK(self.points[0].spanA == 0);
    CHECK(self.points[0].spanB == 1);
    CHECK_THAT(self.points[0].hit.tA, Catch::Matchers::WithinAbs(0.994105366, 1e-8));
    CHECK(hasSelfIntersections(c, tol));
}

TEST_CASE("T-0007 AC4: generative re-crossings within 2-10 tol of a Bezier/arc junction",
          "[geometry][intersect][t0007][property]") {
    // In the frame of the arc's start P (x along the arc, y towards its outside) the arc is
    // y = -x^2 / (2r) to fourth order. The Bezier runs back along it to P as
    // y = -x^2 / (2r) + beta x (d - x): it bulges up to `bulge` (<= tol) to one side of the arc and
    // re-crosses it at x = d, then falls away to the other side. Expected: exactly one
    // self-intersection, at x = d, whether the Bezier comes before the arc or after it.
    SplitMix rng{0x7007AC4ULL};
    constexpr int kCases = 1500;
    std::size_t good = 0;
    for (int i = 0; i < kCases; ++i) {
        const double r = rng.uniform(3.0, 20.0);
        const double d = rng.uniform(2.0, 10.0) * tol.distance;
        const double bulge =
            rng.uniform(0.1, 1.0) * tol.distance * (rng.uniform(0.0, 1.0) < 0.5 ? 1.0 : -1.0);
        const double beta = 4.0 * bulge / (d * d);
        const double x0 = rng.uniform(0.03, 0.05);
        const double rot = rng.uniform(-kPi, kPi);
        const Vec2 origin{rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)};
        const Vec2 ex = Vec2::fromAngle(rot, 1.0);
        const Vec2 ey = ex.perp();
        const auto place = [&](double x, double y) { return origin + ex * x + ey * y; };
        // The arc: centre below P, travelling from P towards +x (clockwise in the local frame).
        const Vec2 centre = place(0.0, -r);
        const double a0 = (origin - centre).angle();
        const ArcSpan arc{centre, r, a0, a0 - 0.8, ArcDirection::CW};
        // The Bezier: x from x0 down to 0, y quadratic in x (exact as a cubic).
        const auto y = [&](double x) { return -x * x / (2.0 * r) + beta * x * (d - x); };
        const auto dy = [&](double x) { return -x / r + beta * (d - 2.0 * x); };
        const Vec2 b0 = place(x0, y(x0));
        const Vec2 b3 = origin;
        const Vec2 d0 = (ex + ey * dy(x0)) * (-x0);  // dB/du at u = 0 (x decreases by x0)
        const Vec2 d3 = (ex + ey * dy(0.0)) * (-x0);
        const BezierSpan bz{b0, b0 + d0 / 3.0, b3 - d3 / 3.0, b3};
        const Vec2 expected = place(d, y(d));
        for (const bool bezierFirst : {true, false}) {
            Contour c;
            if (bezierFirst) {
                c.append(bz);
                c.append(arc);
            } else {
                c.append(arc.reversed());
                c.append(bz.reversed());
            }
            const ContourIntersections self = selfIntersections(c, tol);
            const bool ok = self.points.size() == 1 && distance(self.points[0].hit.point, expected) <= 1e-6 &&
                            !self.points[0].hit.tangent;
            if (!ok) {
                UNSCOPED_INFO("case " << i << " d/tol=" << d / tol.distance
                                      << " bulge/tol=" << bulge / tol.distance
                                      << " bezierFirst=" << bezierFirst << " hits=" << self.points.size());
            }
            good += ok ? 1u : 0u;
        }
    }
    CHECK(good == 2u * kCases);
}
