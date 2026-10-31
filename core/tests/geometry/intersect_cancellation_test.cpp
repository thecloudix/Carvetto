// T-0021: arc/arc and line/arc crossings without cancellation.
//   AC1  the pair reported by the T-0017 coder (radius 39785 against 0.09, far from the origin) and
//        generative arc/arc and line/arc crossings over radius ratios 1e-6..1e6 and translations up
//        to 1e4 mm, checked against a double-double reference computed from the same inputs.
// (AC2, no result worse than main on the T-0007/T-0008 families, was measured with a separate
// driver linking main's geometry; see the T-0021 amendment of ADR-0009.)

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {
const Tolerance tol{};
constexpr double kEps = std::numeric_limits<double>::epsilon();

// ------------------------------------------------- double-double reference arithmetic
// A value hi + lo with |lo| <= ulp(hi)/2: about 106 bits. Error-free transformations (two-sum,
// fma-based two-product) make the reference crossings exact to far below the double rounding of
// the inputs, so the comparison measures the error of the double algorithm alone.
struct DD {
    double hi{0.0};
    double lo{0.0};
};

DD quickTwoSum(double a, double b) {
    const double s = a + b;
    return {s, b - (s - a)};
}

DD twoSum(double a, double b) {
    const double s = a + b;
    const double bb = s - a;
    return {s, (a - (s - bb)) + (b - bb)};
}

DD dd(double v) { return {v, 0.0}; }

DD operator+(DD a, DD b) {
    DD s = twoSum(a.hi, b.hi);
    const DD t = twoSum(a.lo, b.lo);
    s.lo += t.hi;
    s = quickTwoSum(s.hi, s.lo);
    s.lo += t.lo;
    return quickTwoSum(s.hi, s.lo);
}

DD operator-(DD a) { return {-a.hi, -a.lo}; }
DD operator-(DD a, DD b) { return a + (-b); }

DD operator*(DD a, DD b) {
    const double p = a.hi * b.hi;
    double e = std::fma(a.hi, b.hi, -p);
    e += a.hi * b.lo + a.lo * b.hi;
    return quickTwoSum(p, e);
}

DD operator/(DD a, DD b) {
    const double q1 = a.hi / b.hi;
    DD r = a - b * dd(q1);
    const double q2 = r.hi / b.hi;
    r = r - b * dd(q2);
    const double q3 = r.hi / b.hi;
    return dd(q1) + dd(q2) + dd(q3);
}

DD sqrtDD(DD a) {
    if (a.hi <= 0.0) {
        return {};
    }
    const double q = std::sqrt(a.hi);
    const DD r = a - dd(q) * dd(q);
    return dd(q) + dd(r.hi / (2.0 * q));
}

double toDouble(DD a) { return a.hi + a.lo; }

// A reference crossing and the sine of the angle between the curves there (the conditioning).
struct RefCrossing {
    Vec2 p;
    double sinAngle{0.0};
    bool onSegment{true};  // line/circle: false for a crossing of the infinite line beyond an end
};

// Crossings of two circles, in double-double from the smaller circle (the radical-line distance
// and the half chord carry ~1e-32 relative error).
std::vector<RefCrossing> circleCircleRef(Vec2 c1, double r1, Vec2 c2, double r2) {
    const bool firstSmall = r1 <= r2;
    const Vec2 cs = firstSmall ? c1 : c2;
    const Vec2 co = firstSmall ? c2 : c1;
    const double rs = firstSmall ? r1 : r2;
    const double ro = firstSmall ? r2 : r1;
    const DD dx = dd(co.x) - dd(cs.x);
    const DD dy = dd(co.y) - dd(cs.y);
    const DD d2 = dx * dx + dy * dy;
    const DD d = sqrtDD(d2);
    const DD along = (dd(rs) * dd(rs) - dd(ro) * dd(ro) + d2) / (dd(2.0) * d);
    const DD h2 = dd(rs) * dd(rs) - along * along;
    std::vector<RefCrossing> out;
    if (h2.hi <= 0.0) {
        return out;
    }
    const DD h = sqrtDD(h2);
    const DD ux = dx / d;
    const DD uy = dy / d;
    const DD bx = dd(cs.x) + ux * along;
    const DD by = dd(cs.y) + uy * along;
    for (const double side : {1.0, -1.0}) {
        const DD px = bx - uy * h * dd(side);
        const DD py = by + ux * h * dd(side);
        const Vec2 n1{toDouble((px - dd(c1.x)) / dd(r1)), toDouble((py - dd(c1.y)) / dd(r1))};
        const Vec2 n2{toDouble((px - dd(c2.x)) / dd(r2)), toDouble((py - dd(c2.y)) / dd(r2))};
        out.push_back({{toDouble(px), toDouble(py)}, std::fabs(n1.cross(n2))});
    }
    return out;
}

// Crossings of a segment's infinite line with a circle, in double-double from the foot of the
// perpendicular; `onSegment` tells whether each lies on the segment.
std::vector<RefCrossing> lineCircleRef(Vec2 a, Vec2 b, Vec2 c, double r) {
    const DD dx = dd(b.x) - dd(a.x);
    const DD dy = dd(b.y) - dd(a.y);
    const DD fx = dd(a.x) - dd(c.x);
    const DD fy = dd(a.y) - dd(c.y);
    const DD len2 = dx * dx + dy * dy;
    const DD tFoot = -((fx * dx + fy * dy) / len2);
    const DD footX = fx + tFoot * dx;
    const DD footY = fy + tFoot * dy;
    const DD half2 = dd(r) * dd(r) - (footX * footX + footY * footY);
    std::vector<RefCrossing> out;
    if (half2.hi <= 0.0) {
        return out;
    }
    const DD dt = sqrtDD(half2 / len2);
    const double len = std::sqrt(toDouble(len2));
    for (const double side : {-1.0, 1.0}) {
        const DD t = tFoot + dt * dd(side);
        const DD px = dd(a.x) + t * dx;
        const DD py = dd(a.y) + t * dy;
        const Vec2 n{toDouble((px - dd(c.x)) / dd(r)), toDouble((py - dd(c.y)) / dd(r))};
        const Vec2 dir{toDouble(dx) / len, toDouble(dy) / len};
        out.push_back({{toDouble(px), toDouble(py)}, std::fabs(n.dot(dir)), t.hi >= 0.0 && t.hi <= 1.0});
    }
    return out;
}

// intersect(a, b) and intersect(b, a) are exact mirror images.
bool mirrored(const Span& a, const Span& b) {
    const IntersectionResult ab = intersect(a, b, tol);
    const IntersectionResult ba = intersect(b, a, tol);
    if (ab.points.size() != ba.points.size() || ab.overlaps.size() != ba.overlaps.size()) {
        return false;
    }
    return std::all_of(ab.points.begin(), ab.points.end(), [&](const Intersection& p) {
        return std::any_of(ba.points.begin(), ba.points.end(), [&](const Intersection& q) {
            return p.point.x == q.point.x && p.point.y == q.point.y && p.tA == q.tB && p.tB == q.tA &&
                   p.tangent == q.tangent;
        });
    });
}

// Error of the reported crossing nearest `ref`, in units of eps * scale / sin(angle): the error a
// backward-stable algorithm makes when the inputs are perturbed by a few ulps of `scale` (the
// largest coordinate or radius). Infinity when no hit is within tolerance of `ref`.
double scaledError(const IntersectionResult& r, const RefCrossing& ref, double scale) {
    double best = std::numeric_limits<double>::infinity();
    for (const Intersection& p : r.points) {
        best = std::min(best, distance(p.point, ref.p));
    }
    if (best > tol.distance) {
        return std::numeric_limits<double>::infinity();
    }
    return best * ref.sinAngle / (kEps * scale);
}

double maxAbs(std::initializer_list<double> values) {
    double m = 0.0;
    for (const double v : values) {
        m = std::max(m, std::fabs(v));
    }
    return m;
}

// The bound on scaledError. A backward-stable solve measures 0.2-3 here (the worst of ~100,000
// random cases of each kind: 1.8 for arc/arc, 3.1 for line/arc); main's arc/arc formula reached 1e6.
constexpr double kStableBound = 16.0;

// The pair the T-0017 coder found (verifier seed 0xe400137f, lensSlot, Round): the closed chain's
// first span, an arc of radius 39785.2, and its last span, an arc of radius 0.0896, cross at the
// chain's closing junction and again 0.0108 mm away.
const ArcSpan kBigArc{{-6308.8745429246874, 31581.569216038675}, 39785.217135614701, -1.5706767134589792,
                      -1.5706742410393422, ArcDirection::CCW};
const ArcSpan kSmallArc{{-6304.1211345246902, -8203.558166433495}, 0.089633408917572641, 0.10399344190285414,
                        -1.5101329545047752, ArcDirection::CW};
// The two crossings of the circles, to 50 digits (Python decimal, 60-digit precision), rounded.
const Vec2 kJunctionCrossing{-6304.11570039429003663311666415046435, -8203.64763496551032264896124638961745};
const Vec2 kOtherCrossing{-6304.12654727606862975261797729814460, -8203.64763626146341749454975174584722};

}  // namespace

TEST_CASE("T-0021 AC1: regression, arcs of radius 39785 and 0.09 cross where they are",
          "[geometry][intersect][t0021]") {
    SECTION("the double-double reference agrees with the 50-digit crossings") {
        const auto ref = circleCircleRef(kBigArc.centre, kBigArc.radius, kSmallArc.centre, kSmallArc.radius);
        REQUIRE(ref.size() == 2);
        for (const Vec2 expected : {kJunctionCrossing, kOtherCrossing}) {
            const double err = std::min(distance(ref[0].p, expected), distance(ref[1].p, expected));
            CHECK(err <= 1e-12);
        }
    }
    SECTION("full circles: both crossings, untangent, to rounding (main: 1.0e-5 mm off)") {
        const Span a = ArcSpan::fullCircle(kBigArc.centre, kBigArc.radius);
        const Span b = ArcSpan::fullCircle(kSmallArc.centre, kSmallArc.radius);
        REQUIRE(mirrored(a, b));
        const IntersectionResult r = intersect(a, b, tol);
        REQUIRE(r.points.size() == 2);
        for (const Vec2 expected : {kJunctionCrossing, kOtherCrossing}) {
            const auto hit = std::min_element(r.points.begin(), r.points.end(), [&](const auto& l, const auto& rr) {
                return distance(l.point, expected) < distance(rr.point, expected);
            });
            CHECK_FALSE(hit->tangent);
            // sin(angle) ~ 0.06 here, so eps * scale / sin ~ 1.5e-10.
            CHECK(distance(hit->point, expected) <= 1e-9);
            CHECK(distance(hit->point, pointAt(a, hit->tA)) <= 1e-9);
            CHECK(distance(hit->point, pointAt(b, hit->tB)) <= 1e-9);
        }
    }
    SECTION("the partial arcs meet once, at the junction (main: a crossing 1.0e-5 mm along the big arc)") {
        const Span a = kBigArc;
        const Span b = kSmallArc;
        REQUIRE(mirrored(a, b));
        const IntersectionResult r = intersect(a, b, tol);
        REQUIRE(r.points.size() == 1);
        const Intersection& hit = r.points.front();
        CHECK_FALSE(hit.tangent);
        CHECK(distance(hit.point, kJunctionCrossing) <= 1e-9);
        // The junction is the big arc's start and the small arc's end: parameters at those ends to
        // rounding (main reported tA = 1.06e-4, i.e. 1.04e-5 mm along an arc of 0.098 mm).
        CHECK(hit.tA * kBigArc.length() <= 1e-9);
        CHECK((1.0 - hit.tB) * kSmallArc.length() <= 1e-9);
    }
    SECTION("the closed chain the pair belongs to does not intersect itself") {
        Contour c;
        c.spans.emplace_back(kBigArc);
        c.spans.emplace_back(LineSpan{{-6304.0173346428537, -8203.6476230780536}, {-6303.0314685996682, -8193.4356806539108}});
        c.spans.emplace_back(LineSpan{{-6303.0314685996682, -8193.4356806539108}, {-6305.1829714443538, -8192.5209172138584}});
        c.spans.emplace_back(LineSpan{{-6305.1829714443538, -8192.5209172138584}, {-6304.0319853554738, -8203.5488619387634}});
        c.spans.emplace_back(kSmallArc);
        c.closed = true;
        CHECK(selfIntersections(c, tol).isEmpty());
    }
}

TEST_CASE("T-0021 AC1: arc/arc crossings over radius ratios 1e-6..1e6 match a double-double reference",
          "[geometry][intersect][t0021]") {
    SplitMix rng{0x7002100A};
    int checked = 0;
    int worse = 0;
    int asymmetric = 0;
    double worst = 0.0;
    for (int i = 0; i < 6000; ++i) {
        const Vec2 shift{rng.uniform(-1e4, 1e4), rng.uniform(-1e4, 1e4)};
        const double phi = rng.uniform(-kPi, kPi);
        double rSmall = rng.logUniform(1e-3, 10.0);
        double rLarge = std::min(rSmall * rng.logUniform(1.0, 1e6), 1e6);
        double centreDistance = rLarge + rng.uniform(-0.999, 0.999) * rSmall;  // the small circle straddles the large one
        if (i % 4 == 0) {  // comparable radii at any crossing distance
            rLarge = rSmall * rng.logUniform(1.0, 10.0);
            centreDistance = (rLarge - rSmall) + rng.uniform(0.0, 1.0) * 2.0 * rSmall;
        }
        if (i % 2 == 1) {  // both large: ratios near 1 with radii up to 1e6
            rSmall *= rng.logUniform(1.0, 1e5);
            rLarge = std::max(rLarge, rSmall) * rng.logUniform(1.0, 2.0);
            centreDistance = (rLarge - rSmall) + rng.uniform(0.0, 1.0) * 2.0 * rSmall;
        }
        const Vec2 cLarge = shift;
        const Vec2 cSmall = shift + Vec2::fromAngle(phi, centreDistance);
        const bool smallFirst = rng.uniform(0.0, 1.0) < 0.5;
        const Vec2 c1 = smallFirst ? cSmall : cLarge;
        const Vec2 c2 = smallFirst ? cLarge : cSmall;
        const double r1 = smallFirst ? rSmall : rLarge;
        const double r2 = smallFirst ? rLarge : rSmall;
        const auto ref = circleCircleRef(c1, r1, c2, r2);
        if (ref.size() != 2 || distance(ref[0].p, ref[1].p) <= 2.0 * tol.distance ||
            (distance(c1, c2) <= tol.distance && std::fabs(r1 - r2) <= tol.distance)) {
            continue;  // folded into one tangent hit, or the same circle within tolerance (overlap)
        }
        const Span a = ArcSpan::fullCircle(c1, r1);
        const Span b = ArcSpan::fullCircle(c2, r2);
        const IntersectionResult r = intersect(a, b, tol);
        const double scale = maxAbs({c1.x, c1.y, c2.x, c2.y, r1, r2});
        ++checked;
        asymmetric += mirrored(a, b) ? 0 : 1;
        bool bad = r.points.size() != 2;
        for (const Intersection& p : r.points) {
            bad = bad || p.tangent;
        }
        for (const RefCrossing& q : ref) {
            const double e = scaledError(r, q, scale);
            worst = std::max(worst, e);
            bad = bad || !(e <= kStableBound);
        }
        worse += bad ? 1 : 0;
    }
    INFO("checked " << checked << ", worst error " << worst << " x eps * scale / sin(angle)");
    CHECK(checked > 5000);
    CHECK(worse == 0);
    CHECK(asymmetric == 0);
    CHECK(worst <= kStableBound);
}

TEST_CASE("T-0021 AC1: line/arc crossings over length/radius ratios 1e-6..1e6 match a double-double reference",
          "[geometry][intersect][t0021]") {
    SplitMix rng{0x7002100B};
    int checked = 0;
    int worse = 0;
    int asymmetric = 0;
    double worst = 0.0;
    for (int i = 0; i < 6000; ++i) {
        const Vec2 centre{rng.uniform(-1e4, 1e4), rng.uniform(-1e4, 1e4)};
        const double r = rng.logUniform(3e-3, 1e6);
        const double len = std::clamp(r * rng.logUniform(1e-6, 1e6), 3e-3, 1e6);
        const double phi = rng.uniform(-kPi, kPi);
        // Angle from the circle's tangent at the crossing: a fifth nearly tangent (1e-4..1 rad).
        const double psi = i % 5 == 0 ? rng.logUniform(1e-4, 1.0) : rng.uniform(0.0, kPi);
        const Vec2 onCircle = centre + Vec2::fromAngle(phi, r);
        const Vec2 dir = Vec2::fromAngle(phi + 0.5 * kPi + psi);
        const double before = rng.uniform(0.0, 1.0) * len;
        const Vec2 s = onCircle - dir * before;
        const Vec2 e = onCircle + dir * (len - before);
        const auto line = lineCircleRef(s, e, centre, r);
        if (line.size() != 2 || distance(line[0].p, line[1].p) <= 2.0 * tol.distance) {
            continue;  // folded into one tangent hit at the foot
        }
        const bool nearEnd = std::any_of(line.begin(), line.end(), [&](const RefCrossing& q) {
            return distance(q.p, s) <= 2.0 * tol.distance || distance(q.p, e) <= 2.0 * tol.distance;
        });
        if (nearEnd) {
            continue;  // an end contact may represent it (T-0008 rules), not the crossing formula
        }
        std::vector<RefCrossing> ref;
        std::copy_if(line.begin(), line.end(), std::back_inserter(ref), [](const RefCrossing& q) { return q.onSegment; });
        const Span a = LineSpan{s, e};
        const Span b = ArcSpan::fullCircle(centre, r);
        const IntersectionResult res = intersect(a, b, tol);
        const double scale = maxAbs({s.x, s.y, e.x, e.y, centre.x, centre.y, r});
        ++checked;
        asymmetric += mirrored(a, b) ? 0 : 1;
        // The crossings, untangent; besides them only end contacts of the segment (a short segment
        // along a large circle has its ends within tolerance of it: tangent hits at tA = 0 or 1).
        const auto crossings = std::count_if(res.points.begin(), res.points.end(),
                                             [](const Intersection& p) { return !p.tangent; });
        bool bad = static_cast<std::size_t>(crossings) != ref.size();
        for (const Intersection& p : res.points) {
            bad = bad || (p.tangent && p.tA != 0.0 && p.tA != 1.0);
        }
        for (const RefCrossing& q : ref) {
            const double err = scaledError(res, q, scale);
            worst = std::max(worst, err);
            bad = bad || !(err <= kStableBound);
        }
        worse += bad ? 1 : 0;
    }
    INFO("checked " << checked << ", worst error " << worst << " x eps * scale / sin(angle)");
    CHECK(checked > 4000);
    CHECK(worse == 0);
    CHECK(asymmetric == 0);
    CHECK(worst <= kStableBound);
}

TEST_CASE("T-0021 AC1: a small arc ending on a large arc's start is one exact hit at the junction",
          "[geometry][intersect][t0021]") {
    // The offset-junction shape of the regression, generated: the large arc starts at a crossing
    // computed in double-double (rounded), the small arc ends there. One hit, exact (the end lies
    // on the other arc to rounding), at the junction, whichever order.
    SplitMix rng{0x7002100C};
    int checked = 0;
    int failures = 0;
    for (int i = 0; i < 3000; ++i) {
        const Vec2 cLarge{rng.uniform(-1e4, 1e4), rng.uniform(-1e4, 1e4)};
        const double rSmall = rng.logUniform(0.01, 2.0);
        const double rLarge = rSmall * rng.logUniform(1e2, 1e6);
        const double phi = rng.uniform(-kPi, kPi);
        const Vec2 cSmall = cLarge + Vec2::fromAngle(phi, rLarge + rng.uniform(-0.9, 0.9) * rSmall);
        const auto ref = circleCircleRef(cLarge, rLarge, cSmall, rSmall);
        if (ref.size() != 2) {
            continue;
        }
        const Vec2 j = ref[static_cast<std::size_t>(i % 2)].p;
        const double angLarge = (j - cLarge).angle();
        const double angSmall = (j - cSmall).angle();
        const double sweepLarge = std::min(rng.logUniform(5e-3, 10.0) / rLarge, 3.0);
        const double sweepSmall = rng.uniform(0.2, 3.0);
        const ArcDirection dl = rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
        const ArcDirection ds = rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
        const double endLarge = dl == ArcDirection::CCW ? angLarge + sweepLarge : angLarge - sweepLarge;
        const double startSmall = ds == ArcDirection::CCW ? angSmall - sweepSmall : angSmall + sweepSmall;
        const Span large = ArcSpan{cLarge, rLarge, angLarge, std::remainder(endLarge, 2.0 * kPi), dl};
        const Span small = ArcSpan{cSmall, rSmall, std::remainder(startSmall, 2.0 * kPi), angSmall, ds};
        const IntersectionResult r = intersect(large, small, tol);
        // Hits other than the junction are genuine (the small arc may come back across the large
        // one); the junction itself must be one exact hit.
        int atJunction = 0;
        bool exact = true;
        for (const Intersection& p : r.points) {
            if (distance(p.point, j) <= tol.distance) {
                ++atJunction;
                const double scale = maxAbs({cLarge.x, cLarge.y, cSmall.x, cSmall.y, rLarge});
                exact = exact && distance(p.point, j) <= 1e3 * kEps * scale;
            }
        }
        ++checked;
        failures += (atJunction == 1 && exact && mirrored(large, small)) ? 0 : 1;
    }
    CHECK(checked > 2500);
    CHECK(failures == 0);
}
