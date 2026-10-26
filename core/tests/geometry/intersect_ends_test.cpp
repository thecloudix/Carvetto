// T-0008: exact line/arc intersections, end touches and shared end points.
//   AC1/AC2  an end of either span within tolerance of the other gives exactly one hit;
//   AC3      near-tangent crossings further apart than the tolerance are both reported, and a
//            re-crossing next to a contour junction is not erased with it;
//   AC5      closestPoint(ArcSpan) parameters a rounding error outside the sweep;
//   AC6      ends inside stretches where a Bezier hugs an arc.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {
const Tolerance tol{};

// intersect(a, b) and intersect(b, a) are exact mirror images (bit-identical points and
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
             distance(p.point, pointAt(b, p.tB)) <= tol.distance &&
             distance(pointAt(a, p.tA), pointAt(b, p.tB)) <= tol.distance;
        ok = ok && std::any_of(ba.points.begin(), ba.points.end(), [&](const Intersection& q) {
                 return p.point.x == q.point.x && p.point.y == q.point.y && p.tA == q.tB && p.tB == q.tA &&
                        p.tangent == q.tangent;
             });
    }
    return ok;
}

// Brute-force distance from `p` to `s` (independent of closestPoint): 2000 samples, then golden
// section around the best one.
double bruteDistance(const Span& s, Vec2 p, double* tOut = nullptr) {
    constexpr int kN = 2000;
    int best = 0;
    double bestD = std::numeric_limits<double>::infinity();
    for (int k = 0; k <= kN; ++k) {
        const double d = distance(p, pointAt(s, static_cast<double>(k) / kN));
        if (d < bestD) {
            bestD = d;
            best = k;
        }
    }
    double lo = std::max(0.0, (best - 1.0) / kN);
    double hi = std::min(1.0, (best + 1.0) / kN);
    for (int it = 0; it < 80; ++it) {
        const double m1 = lo + (hi - lo) * 0.381966011250105;
        const double m2 = lo + (hi - lo) * 0.618033988749895;
        if (distance(p, pointAt(s, m1)) < distance(p, pointAt(s, m2))) {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    double t = 0.5 * (lo + hi);
    double d = distance(p, pointAt(s, t));
    if (bestD < d) {
        d = bestD;
        t = static_cast<double>(best) / kN;
    }
    if (tOut != nullptr) {
        *tOut = t;
    }
    return d;
}

std::size_t hitsNear(const IntersectionResult& r, Vec2 q, double radius) {
    return static_cast<std::size_t>(
        std::count_if(r.points.begin(), r.points.end(),
                      [&](const Intersection& p) { return distance(p.point, q) <= radius; }));
}

const Intersection* nearestHit(const IntersectionResult& r, Vec2 q) {
    const Intersection* best = nullptr;
    for (const Intersection& p : r.points) {
        if (best == nullptr || distance(p.point, q) < distance(best->point, q)) {
            best = &p;
        }
    }
    return best;
}

ArcSpan arcThrough(Vec2 centre, double radius, double a0, double sweep) {
    return {centre, radius, a0, a0 + sweep, sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW};
}

// A partial arc covering the directions within 0.3 rad of `facing`, its ends 0.3-1 rad away.
ArcSpan arcAround(SplitMix& rng, Vec2 centre, double radius, double facing) {
    const double before = rng.uniform(0.3, 1.0);
    return arcThrough(centre, radius, facing - before, before + rng.uniform(0.3, 1.0));
}
}  // namespace

// ============================================================================ AC1

TEST_CASE("T-0008 AC1: regression, a line starting exactly where an arc ends",
          "[geometry][intersect][t0008]") {
    // The end points are 1.8e-15 apart. The line is within tolerance of tangency to the circle, but
    // the foot of the perpendicular lies 1.6 um beyond the arc's end, so the tangent rule found
    // nothing and the shared end point was lost (0 hits).
    const LineSpan line{{-2.092227545811626, -4.3793988799678552}, {9.7310280804032239, -4.1770917092683604}};
    const ArcSpan arc{{-2.185139788351961, 0.95685533589696803},
                      5.3370630257800924,
                      2.4383258356681337,
                      4.7297987310031928,
                      ArcDirection::CW};
    REQUIRE(distance(line.start, arc.endPoint()) < 1e-14);
    for (const bool lineFirst : {true, false}) {
        const Span a = lineFirst ? Span{line} : Span{arc};
        const Span b = lineFirst ? Span{arc} : Span{line};
        IntersectionResult r;
        CHECK(mirrored(a, b, &r));
        REQUIRE(r.points.size() == 1);
        const Intersection& p = r.points[0];
        CHECK(distance(p.point, line.start) <= 1e-12);
        CHECK_THAT(lineFirst ? p.tA : p.tB, Catch::Matchers::WithinAbs(0.0, 1e-12));
        CHECK_THAT(lineFirst ? p.tB : p.tA, Catch::Matchers::WithinAbs(1.0, 1e-12));
        CHECK_FALSE(p.tangent);  // an exact contact of non-parallel spans
    }
}

TEST_CASE("T-0008 AC1: oblique end touches of lines and arcs, both orders", "[geometry][intersect][t0008]") {
    // A span ends 0.5 um from the interior (or the end) of another, leaving it at 40 degrees from
    // the normal: one tangent hit at the end, whichever span is A. Before T-0008 the analytic pairs
    // only caught an end whose extension crossed the other span within tolerance along the line.
    const double h = 5e-4;
    const LineSpan base{{-2.0, 0.0}, {2.0, 0.0}};
    const ArcSpan arcBase{{0.0, -3.0}, 3.0, kPi / 2.0 + 0.6, kPi / 2.0 - 0.6, ArcDirection::CW};
    const ArcSpan circle = ArcSpan::fullCircle({0.0, -3.0}, 3.0);
    const Vec2 q{0.0, h};  // the end, h above both bases (their top point is the origin)
    const Vec2 dir = Vec2::fromAngle(kPi / 2.0 - 0.7);
    const LineSpan leaving{q, q + dir * 1.5};
    // An arc leaving q along `dir`, turning left (its centre on the left, travelling CCW).
    const ArcSpan leavingArc = arcThrough(q + dir.perp() * 2.0, 2.0, (dir.perp() * -1.0).angle(), 0.6);
    const std::vector<Span> bases{base, arcBase, circle};
    const std::vector<Span> enders{leaving, leavingArc, Span{reversed(Span{leaving})},
                                   Span{reversed(Span{leavingArc})}};
    for (const Span& c : bases) {
        for (const Span& s : enders) {
            for (const bool sFirst : {true, false}) {
                INFO("base kind " << c.index() << " ender kind " << s.index() << " sFirst " << sFirst);
                IntersectionResult r;
                CHECK(mirrored(sFirst ? s : c, sFirst ? c : s, &r));
                REQUIRE(r.points.size() == 1);
                CHECK(r.points[0].tangent);
                CHECK(distance(r.points[0].point, q) <= h);
                const double sParam = sFirst ? r.points[0].tA : r.points[0].tB;
                CHECK((sParam == 0.0 || sParam == 1.0));
            }
        }
    }
    // End to end: the line's end 0.7 um from the arc's start, the line leaving obliquely.
    const ArcSpan arc{{0.0, 0.0}, 1.0, 0.0, 1.0, ArcDirection::CCW};  // starts at (1, 0)
    const Vec2 e = Vec2{1.0, 0.0} + Vec2::fromAngle(-0.3, 7e-4);
    const LineSpan line{e + Vec2::fromAngle(-0.9, 1.0), e};
    IntersectionResult r;
    CHECK(mirrored(line, arc, &r));
    REQUIRE(r.points.size() == 1);
    CHECK(r.points[0].tangent);
    CHECK((r.points[0].tA == 1.0 || r.points[0].tB == 0.0));
    // At 1.05 tol nothing.
    const Vec2 far = Vec2{0.0, 1.05 * tol.distance};
    CHECK(intersect(Span{LineSpan{far, far + dir}}, Span{base}, tol).isEmpty());
    CHECK(intersect(Span{LineSpan{far, far + dir}}, Span{circle}, tol).isEmpty());
}

// ============================================================================ AC2

namespace {
struct EndCase {
    Span c;    // the span whose interior is approached
    Span s;    // the span that ends near it
    Vec2 q;    // S's end
    double h;  // constructed gap
    int approach;
};

// S ends at gap h from C's interior at C(t), t in [0.02, 0.98] and at least 0.01 mm from C's ends,
// on either side of C, leaving it
// obliquely (up to 69 degrees from the normal), along the normal (within 1.1 degrees) or nearly
// parallel (1.1 to 8.6 degrees from C's tangent). C is a line (kind 0), a partial arc (1) or a full
// circle (2); S is a line (0) or an arc (1), travelling away from the end or towards it.
EndCase makeEndCase(SplitMix& rng, int cKind, int sKind, double h) {
    EndCase out;
    const Vec2 o{rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)};
    if (cKind == 0) {
        out.c = LineSpan{o, o + Vec2::fromAngle(rng.uniform(-kPi, kPi), rng.uniform(0.5, 8.0))};
    } else {
        const double r = rng.logUniform(0.2, 20.0);
        const double a0 = rng.uniform(-kPi, kPi);
        if (cKind == 1) {
            const double sweep = rng.uniform(std::min(0.3 + 0.03 / r, 3.0), kTwoPi - 0.3) *
                                 (rng.uniform(0.0, 1.0) < 0.5 ? 1.0 : -1.0);
            out.c = arcThrough(o, r, a0, sweep);
        } else {
            out.c = ArcSpan{o, r, a0, a0, rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CW : ArcDirection::CCW};
        }
    }
    // Keep the touch point at least 0.01 mm from C's ends (an end of C next to the touch would be
    // a contact of its own).
    const double margin = 0.02 + 0.01 / length(out.c);
    const double t = rng.uniform(margin, 1.0 - margin);
    const Vec2 p = pointAt(out.c, t);
    Vec2 n = tangentAt(out.c, t).perp();
    if (rng.uniform(0.0, 1.0) < 0.5) {
        n = n * -1.0;
    }
    out.h = h;
    out.q = p + n * h;
    out.approach = static_cast<int>(rng.next() % 3);
    double off = 0.0;
    if (out.approach == 0) {
        off = rng.uniform(-1.2, 1.2);
    } else if (out.approach == 1) {
        off = rng.uniform(-0.02, 0.02);
    } else {
        off = (rng.uniform(0.0, 1.0) < 0.5 ? 1.0 : -1.0) * (kPi / 2.0 - rng.uniform(0.02, 0.15));
    }
    const Vec2 dir = Vec2::fromAngle(std::atan2(n.y, n.x) + off);
    const double len = rng.uniform(0.3, 3.0);
    if (sKind == 0) {
        out.s = LineSpan{out.q, out.q + dir * len};
    } else {
        const double r = rng.logUniform(0.5, 20.0);
        const bool ccw = rng.uniform(0.0, 1.0) < 0.5;
        const Vec2 centre = out.q + (ccw ? dir.perp() : dir.perp() * -1.0) * r;
        const double a0 = (out.q - centre).angle();
        const double sweep = std::min(len / r, 2.0);
        out.s = arcThrough(centre, r, a0, ccw ? sweep : -sweep);
    }
    if (rng.uniform(0.0, 1.0) < 0.5) {
        out.s = reversed(out.s);
    }
    return out;
}
}  // namespace

TEST_CASE("T-0008 AC2: generative end touches of analytic pairs against brute force",
          "[geometry][intersect][t0008][property]") {
    // 10,000 cases for each of L/L, L/A, A/L, A/A (the approached span listed first; arcs include
    // full circles), h in [0, 0.99 tol] (a tenth exactly 0, a tenth 1e-9..1e-4) or [1.05, 2] tol,
    // both argument orders (bit-exact mirror images). The gap of the end is measured by brute force:
    // within tolerance there is exactly one hit within 2 tol of the end, within the gap of both
    // spans and tangent unless the gap is exact; beyond 1.05 tol there is none. Every hit of every
    // case has its residual within tolerance.
    SplitMix rng{0x7008AC2ULL};
    constexpr int kPerCombination = 10000;
    std::size_t touches = 0;
    std::size_t clear = 0;
    std::size_t failures = 0;
    std::array<std::size_t, 3> approaches{};
    for (int combo = 0; combo < 4; ++combo) {
        const int sKind = combo % 2;     // the span that ends: a line (0) or an arc (1)
        const bool cIsArc = combo >= 2;  // the approached span: a line, or an arc or full circle
        for (int i = 0; i < kPerCombination; ++i) {
            const int cKind = cIsArc ? 1 + static_cast<int>(rng.next() % 2) : 0;
            const double band = rng.uniform(0.0, 1.0);
            double h = 0.0;
            if (band < 0.1) {
                h = 0.0;
            } else if (band < 0.2) {
                h = std::pow(10.0, rng.uniform(-9.0, -4.0));
            } else if (band < 0.7) {
                h = rng.uniform(0.0, 0.99) * tol.distance;
            } else {
                h = rng.uniform(1.05, 2.0) * tol.distance;
            }
            const EndCase ec = makeEndCase(rng, cKind, sKind, h);
            ++approaches[static_cast<std::size_t>(ec.approach)];
            const bool sFirst = rng.uniform(0.0, 1.0) < 0.5;
            const Span& a = sFirst ? ec.s : ec.c;
            const Span& b = sFirst ? ec.c : ec.s;
            IntersectionResult r;
            const bool symmetric = mirrored(a, b, &r);
            const double gap = bruteDistance(ec.c, ec.q);
            bool ok = symmetric;
            for (const Intersection& p : r.points) {
                ok = ok && distance(pointAt(a, p.tA), pointAt(b, p.tB)) <= tol.distance;
            }
            const std::size_t near = hitsNear(r, ec.q, 2.0 * tol.distance);
            if (gap <= 0.99 * tol.distance + 1e-12) {
                ++touches;
                ok = ok && near == 1;
                if (near == 1) {
                    const Intersection& p = *nearestHit(r, ec.q);
                    const double res = distance(pointAt(a, p.tA), pointAt(b, p.tB));
                    ok = ok && res <= gap + 1e-9 && (p.tangent || gap <= 1e-6 * tol.distance);
                }
            } else if (gap >= 1.05 * tol.distance - 1e-12) {
                ++clear;
                ok = ok && near == 0;
            }
            if (!ok) {
                ++failures;
                UNSCOPED_INFO("combo " << combo << " case " << i << " cKind " << cKind << " approach "
                                       << ec.approach << " h=" << h << " gap=" << gap << " hits="
                                       << r.points.size() << " near=" << near << " symmetric=" << symmetric);
            }
        }
    }
    CHECK(failures == 0);
    CHECK(touches > 4u * kPerCombination / 2u);
    CHECK(clear > 4u * kPerCombination / 5u);
    CHECK(approaches[0] > 10000u);
    CHECK(approaches[1] > 10000u);
    CHECK(approaches[2] > 10000u);
}

// ============================================================================ AC3

TEST_CASE("T-0008 AC3: near-tangent line/circle and circle/circle crossings further apart than tol",
          "[geometry][intersect][t0008][property]") {
    // Two crossings c apart (the common chord): c > 1.05 tol gives both, untangent, at the right
    // places; c < 0.95 tol gives one tangent hit (the tolerance cannot separate them). Before
    // T-0008 any pair within tolerance of tangency gave one tangent hit, so crossings up to
    // 2 sqrt(2 r tol) apart were folded into one (0.9 mm for r = 100 mm).
    SplitMix rng{0x7008AC3ULL};
    std::size_t failures = 0;
    std::size_t pairs = 0;
    for (int i = 0; i < 4000; ++i) {
        const bool wide = i % 2 == 0;
        const double c =
            wide ? rng.logUniform(1.05, 1000.0) * tol.distance : rng.uniform(0.0, 0.95) * tol.distance;
        const double r = rng.logUniform(std::max(0.1, 2.0 * c), 200.0);
        const Vec2 m{rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)};
        const Vec2 u = Vec2::fromAngle(rng.uniform(-kPi, kPi));  // along the chord
        const Vec2 x1 = m - u * (0.5 * c);
        const Vec2 x2 = m + u * (0.5 * c);
        const double toCentre = std::sqrt(std::max(0.0, r * r - 0.25 * c * c));
        const Vec2 centre = m + u.perp() * toCentre;
        const double facing = (m - centre).angle();
        // A: the circle (full, or an arc around the chord); B: the chord's line, or a second circle
        // through x1 and x2 with its centre on either side.
        const Span circleA = rng.uniform(0.0, 1.0) < 0.3 ? Span{ArcSpan::fullCircle(centre, r)}
                                                         : Span{arcAround(rng, centre, r, facing)};
        Span other;
        if (i % 4 < 2) {
            // Beyond the crossings the line leaves the circle quadratically; it is extended far
            // enough (s^2 / 2r >= 3 tol) that its ends are not within tolerance of the circle.
            const double ext = std::sqrt(6.0 * r * tol.distance);
            other = LineSpan{m - u * (ext + rng.uniform(0.5, 3.0) + 0.5 * c),
                             m + u * (ext + rng.uniform(0.5, 3.0) + 0.5 * c)};
        } else {
            const double r2 = rng.logUniform(std::max(0.1, 2.0 * c), 200.0);
            const double side = rng.uniform(0.0, 1.0) < 0.5 ? 1.0 : -1.0;
            const Vec2 centre2 = m + u.perp() * (side * std::sqrt(std::max(0.0, r2 * r2 - 0.25 * c * c)));
            if (distance(centre, centre2) <= 0.05) {
                continue;  // nearly the same circle: the arcs' ends would lie within tolerance too
            }
            other = arcAround(rng, centre2, r2, (m - centre2).angle());
        }
        if (rng.uniform(0.0, 1.0) < 0.5) {
            other = reversed(other);
        }
        ++pairs;
        IntersectionResult res;
        bool ok = mirrored(circleA, other, &res);
        if (wide) {
            ok = ok && res.points.size() == 2;
            for (const Intersection& p : res.points) {
                const double err = std::min(distance(p.point, x1), distance(p.point, x2));
                // Exact crossings (residual ~1e-11 at crossing angles down to 0.01 rad), so along the
                // chord they can sit ~1e-9 from the constructed points.
                ok = ok && !p.tangent && err <= 1e-7 &&
                     distance(pointAt(circleA, p.tA), pointAt(other, p.tB)) <= 1e-9;
            }
            ok = ok && res.points.size() == 2 && distance(res.points[0].point, res.points[1].point) > 0.5 * c;
        } else {
            ok = ok && res.points.size() == 1 && res.points[0].tangent &&
                 distance(res.points[0].point, m) <= tol.distance;
        }
        if (!ok) {
            ++failures;
            UNSCOPED_INFO("case " << i << " c/tol=" << c / tol.distance << " r=" << r
                                  << " hits=" << res.points.size());
        }
    }
    CHECK(failures == 0);
    CHECK(pairs > 3500u);
}

namespace {
// A line (or a large arc) ending at the junction J, then an arc that starts at J nearly tangent,
// folds back over the first span and crosses it again at X, d from J. In the frame of J (x along
// the first span's direction of travel, y to its left) the second circle passes through J and
// X = (-d, 0); `side` puts its centre below (+1) or above (-1) the x axis.
Contour junctionContour(double d, double r, double side, bool firstIsArc, Vec2 origin, double rot, Vec2* x) {
    const Vec2 ex = Vec2::fromAngle(rot);
    const Vec2 ey = ex.perp();
    const auto place = [&](double px, double py) { return origin + ex * px + ey * py; };
    const Vec2 centre = place(-0.5 * d, -side * std::sqrt(r * r - 0.25 * d * d));
    const double aJ = (origin - centre).angle();
    const double sweep = 2.0 * std::asin(0.5 * d / r) + 0.5;
    const ArcSpan second = arcThrough(centre, r, aJ, side > 0.0 ? sweep : -sweep);
    Contour c;
    const double len = 1.0 + d;
    if (firstIsArc) {
        // A circle of radius 200 through J and X as well, its centre on the other side, travelling
        // towards J (along +x: CCW below its centre, CW above it).
        const double big = 200.0;
        const Vec2 c1 = place(-0.5 * d, side * std::sqrt(big * big - 0.25 * d * d));
        const double a1 = (origin - c1).angle();
        const double sw = side * len / big;
        c.append(arcThrough(c1, big, a1 - sw, sw));
    } else {
        c.append(LineSpan{place(-len, 0.0), origin});
    }
    c.append(second);
    *x = place(-d, 0.0);
    return c;
}
}  // namespace

TEST_CASE("T-0008 AC3: a re-crossing next to a contour junction is not erased with the junction",
          "[geometry][intersect][t0008][property]") {
    // Before T-0008 the junction and the re-crossing folded into one tangent hit midway, which was
    // erased with the junction when d <= 2 tol, and reported at the wrong place (tangent) beyond.
    // Now the re-crossing is reported as a crossing whenever d > tol; closer than that it is the
    // junction within tolerance and nothing is reported.
    SplitMix rng{0x7008AC33ULL};
    std::size_t failures = 0;
    for (int i = 0; i < 2000; ++i) {
        const bool apart = i % 2 == 0;
        const double d =
            apart ? rng.uniform(1.2, 20.0) * tol.distance : rng.uniform(0.05, 0.8) * tol.distance;
        const double r = rng.logUniform(0.2, 50.0);
        const double side = rng.uniform(0.0, 1.0) < 0.5 ? 1.0 : -1.0;
        const bool firstIsArc = i % 4 >= 2;
        Vec2 x;
        Contour c = junctionContour(d, r, side, firstIsArc, {rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)},
                                    rng.uniform(-kPi, kPi), &x);
        if (rng.uniform(0.0, 1.0) < 0.5) {
            c.reverse();
        }
        REQUIRE(c.isContinuous(tol));
        const ContourIntersections self = selfIntersections(c, tol);
        bool ok = false;
        if (apart) {
            ok = self.points.size() == 1 && !self.points[0].hit.tangent &&
                 distance(self.points[0].hit.point, x) <= 2e-7 && hasSelfIntersections(c, tol);
        } else {
            ok = self.points.empty();
        }
        if (!ok) {
            ++failures;
            UNSCOPED_INFO("case " << i << " d/tol=" << d / tol.distance << " r=" << r << " side=" << side
                                  << " firstIsArc=" << firstIsArc << " hits=" << self.points.size());
        }
    }
    CHECK(failures == 0);
}

// ============================================================================ AC5

TEST_CASE("T-0008 AC5: closestPoint(ArcSpan) parameter a rounding error outside the sweep",
          "[geometry][contour_ops][t0008]") {
    // Query directions a few ulps to 1e-9 rad (the slack of containsAngle) before the start or past
    // the end of CW and CCW partial arcs: the parameter is that of the nearer end and agrees with the
    // returned point. Before T-0008 a direction just before the start got parameter 1.
    SplitMix rng{0x7008AC5ULL};
    std::size_t checked = 0;
    for (int i = 0; i < 400; ++i) {
        const Vec2 centre{rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)};
        const double r = rng.logUniform(0.01, 100.0);
        const double a0 = rng.uniform(-10.0, 10.0);
        const double sweep = rng.uniform(0.01, kTwoPi - 0.01) * (i % 2 == 0 ? 1.0 : -1.0);
        const ArcSpan arc = arcThrough(centre, r, a0, sweep);
        const double dirSign = sweep > 0.0 ? 1.0 : -1.0;
        for (const double eps : {0.0, 1e-17, 1e-16, 4e-16, 1e-15, 1e-13, 1e-11, 5e-10, 9e-10}) {
            for (const double radial : {0.0, 0.3 * r, -0.3 * r}) {
                // Just before the start (against the direction of travel) and just past the end.
                const double before = arc.startAngle - dirSign * eps;
                const double past = arc.endAngle + dirSign * eps;
                const ClosestPoint s = closestPoint(arc, centre + Vec2::fromAngle(before, r + radial));
                const ClosestPoint e = closestPoint(arc, centre + Vec2::fromAngle(past, r + radial));
                INFO("arc " << i << " eps=" << eps << " t(start)=" << s.t << " t(end)=" << e.t);
                CHECK(s.t <= 1e-9);
                CHECK(e.t >= 1.0 - 1e-9);
                CHECK(distance(arc.pointAt(s.t), s.point) <= 1e-8 * std::max(1.0, r));
                CHECK(distance(arc.pointAt(e.t), e.point) <= 1e-8 * std::max(1.0, r));
                checked += 2;
            }
        }
        // Just inside both ends nothing changes.
        const ClosestPoint in0 = closestPoint(arc, arc.pointAt(1e-9));
        const ClosestPoint in1 = closestPoint(arc, arc.pointAt(1.0 - 1e-9));
        CHECK(in0.t > 0.0);
        CHECK(in0.t < 1e-8);
        CHECK(in1.t < 1.0);
        CHECK(in1.t > 1.0 - 1e-8);
    }
    CHECK(checked > 20000u);
    // The contour-level query reports the same consistent parameter.
    const ArcSpan arc{{0.0, 0.0}, 2.0, 0.5, -1.0, ArcDirection::CW};
    Contour c;
    c.append(arc);
    const ContourClosestPoint cp = closestPoint(c, Vec2::fromAngle(0.5 + 1e-16, 2.0));
    CHECK(cp.t == 0.0);
    CHECK(distance(arc.pointAt(cp.t), cp.point) <= 1e-12);
    CHECK(distanceTo(c, Vec2::fromAngle(0.5 + 1e-16, 2.0)) <= 1e-12);
}

// ============================================================================ AC6

TEST_CASE("T-0008 AC6: ends of Beziers hugging arcs that extend past them are reported",
          "[geometry][intersect][t0008][property]") {
    // A cubic approximating a sub-arc (0.05-0.6 rad) of a circle scaled by up to 0.9 tol / r and
    // jiggled by up to 0.5 tol, against the arc extending 0-0.3 rad past it on both sides (the
    // generator of the T-0007 round-3 verification, where 38-45% of these end touches were missed).
    // Every end of either span whose brute-force gap to the other span is within tolerance, with
    // its foot at least 0.01 mm from the other span's ends, has a hit within tol + gap/2: a tangent
    // hit, or an exact one where the end lies on the other span or within tolerance of a crossing
    // (which then represents it). Both argument orders, bit-exact mirror images.
    SplitMix rng{0x7008AC6ULL};
    constexpr int kPairs = 6000;
    std::size_t ends = 0;
    std::size_t misses = 0;
    std::size_t asymmetric = 0;
    for (int i = 0; i < kPairs; ++i) {
        const Vec2 centre{rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)};
        const double r = rng.logUniform(0.5, 20.0);
        const double a0 = rng.uniform(-kPi, kPi);
        const double piece = rng.uniform(0.05, 0.6) * (rng.uniform(0.0, 1.0) < 0.5 ? 1.0 : -1.0);
        const double scale = 1.0 + rng.uniform(-0.9, 0.9) * tol.distance / r;
        const BezierSpan approx = arcThrough(centre, r * scale, a0, piece).toBeziers().front();
        const auto jiggle = [&](Vec2 p) {
            return p + Vec2::fromAngle(rng.uniform(0.0, kTwoPi),
                                       rng.uniform(0.0, 0.5) * rng.uniform(0.0, 1.0) * tol.distance);
        };
        BezierSpan bz{jiggle(approx.p0), jiggle(approx.p1), jiggle(approx.p2), jiggle(approx.p3)};
        if (rng.uniform(0.0, 1.0) < 0.5) {
            bz = bz.reversed();
        }
        const double lo = std::min(a0, a0 + piece) - rng.uniform(0.0, 0.3);
        const double hi = std::max(a0, a0 + piece) + rng.uniform(0.0, 0.3);
        const bool ccw = rng.uniform(0.0, 1.0) < 0.5;
        const ArcSpan arc{centre, r, ccw ? lo : hi, ccw ? hi : lo,
                          ccw ? ArcDirection::CCW : ArcDirection::CW};
        const bool arcFirst = rng.uniform(0.0, 1.0) < 0.5;
        const Span a = arcFirst ? Span{arc} : Span{bz};
        const Span b = arcFirst ? Span{bz} : Span{arc};
        IntersectionResult res;
        if (!mirrored(a, b, &res)) {
            ++asymmetric;
        }
        for (const bool endOfA : {true, false}) {
            const Span& s = endOfA ? a : b;
            const Span& o = endOfA ? b : a;
            for (const double t : {0.0, 1.0}) {
                const Vec2 e = pointAt(s, t);
                double tFoot = 0.0;
                const double gap = bruteDistance(o, e, &tFoot);
                const Vec2 foot = pointAt(o, tFoot);
                if (gap > 0.999 * tol.distance || distance(foot, startPoint(o)) < 0.01 ||
                    distance(foot, endPoint(o)) < 0.01) {
                    continue;
                }
                ++ends;
                const Intersection* p = nearestHit(res, e);
                const bool exact =
                    p != nullptr && distance(pointAt(a, p->tA), pointAt(b, p->tB)) <= 1e-6 * tol.distance;
                const bool found = p != nullptr &&
                                   distance(p->point, e) <= tol.distance + 0.5 * gap + 1e-12 &&
                                   (p->tangent || exact);
                if (!found) {
                    ++misses;
                    UNSCOPED_INFO("pair " << i << " end of " << (endOfA ? "A" : "B") << " t=" << t
                                          << " gap=" << gap << " hits=" << res.points.size());
                }
            }
        }
    }
    CHECK(misses == 0);
    CHECK(asymmetric == 0);
    CHECK(ends > static_cast<std::size_t>(kPairs));
}
