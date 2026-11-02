// T-0022: distance-based arc slack and threshold crossings.
//   AC1  an end lying beyond an arc's end by more than rounding is an end contact flagged tangent,
//        on arcs of any radius: the regression (r = 125593), closestPoint(ArcSpan) against a
//        double-double reference, and a radius sweep 1e-3..1e6 with ends placed 0..2 tol beyond
//        the arc's ends (lines and Beziers, both orders).
//   AC2  two crossings that the tolerance cannot separate are one tangent hit, also where the
//        chord exceeds the tolerance only by rounding (line/circle and circle/circle).
// (AC3, no result worse than main on the T-0007/T-0008/T-0021 families, was measured with a
// separate driver linking main's geometry; see the T-0022 amendment of ADR-0009.)

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {
const Tolerance tol{};
constexpr double kEps = std::numeric_limits<double>::epsilon();
// A contact whose gap is at most this is exact (ADR-0009, T-0008 item 1).
const double kExactGap = 1e-6 * tol.distance;

// ------------------------------------------------- double-double reference arithmetic
// hi + lo with |lo| <= ulp(hi)/2 (~106 bits), from error-free transformations.
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

struct DDVec {
    DD x;
    DD y;
};

DDVec sub(DDVec a, DDVec b) { return {a.x - b.x, a.y - b.y}; }
DDVec ddv(Vec2 v) { return {dd(v.x), dd(v.y)}; }
DD dot(DDVec a, DDVec b) { return a.x * b.x + a.y * b.y; }
DD cross(DDVec a, DDVec b) { return a.x * b.y - a.y * b.x; }
DD norm(DDVec a) { return sqrtDD(dot(a, a)); }

// cos and sin of a double angle to ~1e-30: reduction by pi/2 in double-double, then Taylor series.
DDVec unitDD(double angle) {
    const DD halfPi{1.5707963267948966, 6.123233995736766e-17};
    const double k = std::nearbyint(angle / 1.5707963267948966);
    const DD y = dd(angle) - halfPi * dd(k);
    const DD y2 = y * y;
    DD s = y;
    DD c = dd(1.0);
    DD termS = y;
    DD termC = dd(1.0);
    for (int n = 1; n <= 20; ++n) {
        termS = -(termS * y2) / dd(static_cast<double>((2 * n) * (2 * n + 1)));
        termC = -(termC * y2) / dd(static_cast<double>((2 * n - 1) * (2 * n)));
        s = s + termS;
        c = c + termC;
    }
    const int q = ((static_cast<int>(k) % 4) + 4) % 4;
    switch (q) {
        case 0: return {c, s};
        case 1: return {-s, c};
        case 2: return {-c, -s};
        default: return {s, -c};
    }
}

// End point of the arc at parameter 0 or 1 exactly as the arc defines it (angleAt), in DD.
DDVec arcEndDD(const ArcSpan& arc, double t) {
    const DDVec u = unitDD(arc.angleAt(t));
    return {dd(arc.centre.x) + dd(arc.radius) * u.x, dd(arc.centre.y) + dd(arc.radius) * u.y};
}

// Distance from `p` to the arc, for a point whose direction lies within 0.5 rad of the arc's end
// `t` (0 or 1) and an arc whose gap to a full circle exceeds 1 rad: the radial distance when the
// direction lies inside the sweep, otherwise the distance to that end.
DD arcDistanceNearEnd(const ArcSpan& arc, Vec2 p, double t) {
    const DDVec rel = sub(ddv(p), ddv(arc.centre));
    const DDVec u = unitDD(arc.angleAt(t));
    // Travel direction sign: inside means "after the start" / "before the end" along travel.
    const double travel = arc.direction == ArcDirection::CCW ? 1.0 : -1.0;
    const double side = toDouble(cross(u, rel)) * travel;
    const bool inside = t == 0.0 ? side >= 0.0 : side <= 0.0;
    if (inside) {
        const DD radial = norm(rel) - dd(arc.radius);
        return radial.hi < 0.0 ? -radial : radial;
    }
    return norm(sub(ddv(p), arcEndDD(arc, t)));
}

// Distance from a DD point to a segment, in DD.
DD segmentDistanceDD(const LineSpan& s, DDVec p) {
    const DDVec d = sub(ddv(s.end), ddv(s.start));
    const DDVec w = sub(p, ddv(s.start));
    DD t = dot(w, d) / dot(d, d);
    if (t.hi < 0.0) {
        t = dd(0.0);
    } else if (t.hi > 1.0) {
        t = dd(1.0);
    }
    return norm(sub(w, {d.x * t, d.y * t}));
}

// intersect(a, b) and intersect(b, a) are exact mirror images, every point within tolerance of
// both spans at its parameters.
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

ArcSpan arcThrough(Vec2 centre, double radius, double a0, double sweep) {
    return {centre, radius, a0, a0 + sweep, sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW};
}

// A partial arc covering the directions within 0.3 rad of `facing` (its ends 0.3-1 rad away),
// travelling CCW or CW.
ArcSpan arcAround(SplitMix& rng, Vec2 centre, double radius, double facing, bool ccw) {
    const double before = rng.uniform(0.3, 1.0);
    const double sweep = before + rng.uniform(0.3, 1.0);
    return ccw ? arcThrough(centre, radius, facing - before, sweep) : arcThrough(centre, radius, facing + before, -sweep);
}

// Rounding allowance of an arc end's position: the end angle carries ~ulp(angle) and the
// coordinates ~ulp(|centre| + r), each a few times over.
double endRounding(const ArcSpan& arc) {
    const double angle = std::max({1.0, std::fabs(arc.startAngle), std::fabs(arc.endAngle)});
    return 8.0 * kEps * (std::max(std::fabs(arc.centre.x), std::fabs(arc.centre.y)) + arc.radius * (1.0 + angle));
}
}  // namespace

// ============================================================================ AC1

TEST_CASE("T-0022 AC1: regression, a radial line ending 4e-5 mm before a large arc's start",
          "[geometry][intersect][t0022]") {
    // The end E lies on the circle 3.2e-10 rad before the start: 4.0e-5 mm from the arc. Before
    // T-0022 closestPoint accepted the direction within containsAngle's 1e-9 rad slack and projected
    // E onto the circle (gap 0), so the end contact was exact and reported untangent.
    const double r = 125593.0;
    const ArcSpan arc{{0.0, 0.0}, r, 1.0, 1.5, ArcDirection::CCW};
    const double a = 1.0 - 3.2e-10;
    const Vec2 dir{std::cos(a), std::sin(a)};
    const Vec2 e = dir * r;
    const LineSpan line{e - dir * 0.5, e};
    const double gap = toDouble(arcDistanceNearEnd(arc, e, 0.0));
    REQUIRE(gap > 3.9e-5);
    REQUIRE(gap < 4.1e-5);

    const ClosestPoint cp = closestPoint(arc, e);
    CHECK(cp.t == 0.0);
    CHECK(cp.point.x == arc.startPoint().x);
    CHECK(cp.point.y == arc.startPoint().y);
    CHECK(std::fabs(cp.distance - gap) <= 1e-12);

    for (const bool lineFirst : {true, false}) {
        const Span sa = lineFirst ? Span{line} : Span{arc};
        const Span sb = lineFirst ? Span{arc} : Span{line};
        IntersectionResult res;
        CHECK(mirrored(sa, sb, &res));
        REQUIRE(res.points.size() == 1);
        const Intersection& hit = res.points[0];
        CHECK(hit.tangent);
        CHECK((lineFirst ? hit.tA : hit.tB) == 1.0);
        CHECK((lineFirst ? hit.tB : hit.tA) == 0.0);
        CHECK(distance(hit.point, e) <= gap);
        CHECK(distance(hit.point, arc.startPoint()) <= gap);
    }
}

TEST_CASE("T-0022 AC1: closestPoint(ArcSpan) beyond an end matches a double-double reference",
          "[geometry][contour_ops][t0022][property]") {
    // Query points beyond either end by 0..2 tol (and a few exactly at the end), on and off the
    // circle, for radii 1e-3..1e6: the distance is the reference distance to rounding. Before
    // T-0022 directions within 1e-9 rad of an end were projected radially, up to r * 1e-9 off.
    SplitMix rng{0x70022C1ULL};
    double worst = 0.0;
    int checked = 0;
    for (int i = 0; i < 20000; ++i) {
        const double r = rng.logUniform(1e-3, 1e6);
        const Vec2 centre{rng.uniform(-1e3, 1e3), rng.uniform(-1e3, 1e3)};
        const double sweep = rng.uniform(0.05, kTwoPi - 1.2) * (i % 2 == 0 ? 1.0 : -1.0);
        const ArcSpan arc = arcThrough(centre, r, rng.uniform(-kPi, kPi), sweep);
        const double t = (i / 2) % 2 == 0 ? 0.0 : 1.0;
        const double outward = (t == 0.0) == (sweep > 0.0) ? -1.0 : 1.0;  // angle sign beyond the end
        const double g = i % 10 == 0 ? 0.0 : (i % 10 < 4 ? rng.logUniform(1e-12, 1e-6) : rng.uniform(0.0, 2.0)) * tol.distance;
        const double delta = std::min(g / r, 0.5);
        const double radial = i % 3 == 0 ? 0.0 : rng.uniform(-1.0, 1.0) * std::min(g, 0.5 * r);
        const Vec2 p = centre + Vec2::fromAngle(arc.angleAt(t) + outward * delta, r + radial);
        const double ref = toDouble(arcDistanceNearEnd(arc, p, t));
        const ClosestPoint cp = closestPoint(arc, p);
        const double err = std::fabs(cp.distance - ref);
        const double allowance = endRounding(arc) + 4.0 * kEps * (std::fabs(p.x) + std::fabs(p.y));
        worst = std::max(worst, err / allowance);
        ++checked;
        if (err > allowance || distance(arc.pointAt(cp.t), cp.point) > allowance) {
            UNSCOPED_INFO("case " << i << " r=" << r << " g=" << g << " ref=" << ref << " got=" << cp.distance
                                  << " t=" << cp.t);
            CHECK(false);
        }
    }
    UNSCOPED_INFO("worst error / allowance " << worst);
    CHECK(checked == 20000);
}

TEST_CASE("T-0022 AC1: ends beyond arc ends over radii 1e-3..1e6 are end contacts, tangent unless exact",
          "[geometry][intersect][t0022][property]") {
    // A line or a Bezier ends at E, placed beyond an arc's start or end by a gap g in 0..2 tol
    // (along the circle, sometimes also outside it), arriving radially, obliquely or tangentially.
    // Against the double-double gap G: G <= 0.999 tol gives exactly one hit within tol + G of E,
    // flagged tangent when G exceeds the exact-contact gap by more than the rounding of the arc's
    // end, untangent when the contact is exact and transversal; G >= 1.001 tol gives no hit. Both
    // orders, bit-exact mirrors.
    SplitMix rng{0x70022A1ULL};
    int failures = 0;
    int within = 0;
    int beyond = 0;
    int tangentRequired = 0;
    int cleanCases = 0;
    for (int i = 0; i < 12000; ++i) {
        const double r = rng.logUniform(1e-3, 1e6);
        const Vec2 centre{rng.uniform(-1e3, 1e3), rng.uniform(-1e3, 1e3)};
        const double sweep = rng.uniform(0.05, kTwoPi - 1.2) * (i % 2 == 0 ? 1.0 : -1.0);
        const ArcSpan arc = arcThrough(centre, r, rng.uniform(-kPi, kPi), sweep);
        const double t = (i / 2) % 2 == 0 ? 0.0 : 1.0;
        const double outward = (t == 0.0) == (sweep > 0.0) ? -1.0 : 1.0;
        const int band = i % 10;
        const double g = band == 0 ? 0.0
                         : band < 3 ? rng.logUniform(1e-12, 1e-6) * tol.distance
                         : band < 5 ? rng.uniform(0.5, 2.0) * kExactGap
                                    : rng.uniform(0.0, 2.0) * tol.distance;
        const double delta = std::min(g / r, 0.5);
        const double endAngle = arc.angleAt(t) + outward * delta;
        // 0 radial in (from inside the circle, only where the line fits), 1 radial out, 2 tangent
        // away, 3 oblique (outwards and away), 4 a Bezier bending outwards and away.
        int kind = (i / 4) % 5;
        if (kind == 0 && 0.5 * r < 5.0 * tol.distance) {
            kind = 1;
        }
        const double lift = kind == 0 || i % 3 == 0 ? 0.0 : rng.uniform(0.0, 1.0) * std::min(g, 0.5 * r);
        const Vec2 radialDir = Vec2::fromAngle(endAngle);
        // The circle's tangent at E pointing further beyond the arc's end.
        const Vec2 awayDir = radialDir.perp() * outward;
        const Vec2 e = centre + radialDir * (r + lift);
        const double len = std::max(std::min(0.5 * r, 1.0), 5.0 * tol.distance) * rng.uniform(0.5, 1.0);
        Span other;
        switch (kind) {
            case 0: other = LineSpan{e - radialDir * len, e}; break;
            case 1: other = LineSpan{e + radialDir * len, e}; break;
            case 2: other = LineSpan{e + awayDir * len, e}; break;
            case 3: other = LineSpan{e + (awayDir + radialDir) * (len / std::sqrt(2.0)), e}; break;
            default:
                other = BezierSpan{e, e + radialDir * (len / 3.0), e + radialDir * (2.0 * len / 3.0) + awayDir * (len / 6.0),
                                   e + radialDir * len + awayDir * (len / 3.0)};
                break;
        }
        if (rng.uniform(0.0, 1.0) < 0.5) {
            other = reversed(other);
        }
        // Reference gap: E to the arc; a radial line coming in from inside passes the arc's end at
        // r sin(delta), slightly closer.
        DD gapDD = arcDistanceNearEnd(arc, e, t);
        if (kind == 0) {
            const DD toEnd = segmentDistanceDD(std::get<LineSpan>(other), arcEndDD(arc, t));
            if (toEnd.hi < gapDD.hi) {
                gapDD = toEnd;
            }
        }
        const double gap = toDouble(gapDD);
        const double rounding = endRounding(arc) + 4.0 * kEps * (std::fabs(e.x) + std::fabs(e.y));

        // On small arcs the other end can come within tolerance of the other span as well; such
        // cases are only checked for symmetry.
        // Spans no longer than the tolerance are handled as points (ADR-0009 decision 3), which
        // places such an arc at its start: judged like the small-arc cases.
        const bool clean = closestPoint(other, arc.pointAt(1.0 - t)).distance > 3.0 * tol.distance &&
                           arc.length() > 3.0 * tol.distance;
        cleanCases += clean ? 1 : 0;

        IntersectionResult res;
        bool ok = mirrored(arc, other, &res);
        std::size_t near = 0;
        const Intersection* hit = nullptr;
        for (const Intersection& p : res.points) {
            if (distance(p.point, e) <= tol.distance + gap) {
                ++near;
                hit = &p;
            }
        }
        if (gap <= 0.999 * tol.distance) {
            ++within;
            if (clean) {
                ok = ok && near == 1 && res.points.size() == 1;
                if (gap > kExactGap + rounding) {
                    ++tangentRequired;
                    ok = ok && hit->tangent;
                } else if (gap + rounding < kExactGap && (kind == 0 || kind == 1)) {
                    ok = ok && !hit->tangent;  // exact and transversal
                }
            }
        } else if (gap >= 1.001 * tol.distance) {
            ++beyond;
            ok = ok && (clean ? res.points.empty() : true);
        }
        if (!ok) {
            ++failures;
            UNSCOPED_INFO("case " << i << " kind=" << kind << " r=" << r << " g=" << g << " gap=" << gap
                                  << " hits=" << res.points.size() << " near=" << near
                                  << " tangent=" << (hit != nullptr && hit->tangent));
        }
    }
    UNSCOPED_INFO("within " << within << " beyond " << beyond << " tangent required " << tangentRequired
                            << " clean " << cleanCases);
    CHECK(failures == 0);
    CHECK(within > 6000);
    CHECK(beyond > 2000);
    CHECK(tangentRequired > 3500);
    CHECK(cleanCases > 11000);
}

// ============================================================================ AC2

namespace {
// One tangent hit, or two untangent hits further apart than the tolerance.
bool foldedOrSeparate(const IntersectionResult& r) {
    if (r.points.size() == 1) {
        return r.points[0].tangent;
    }
    return r.points.size() == 2 && !r.points[0].tangent && !r.points[1].tangent &&
           distance(r.points[0].point, r.points[1].point) > tol.distance;
}
}  // namespace

TEST_CASE("T-0022 AC2: regressions, crossings 4e-15 mm further apart than tol fold into one tangent hit",
          "[geometry][intersect][t0022]") {
    // Line/circle: the computed half chord gives 2s - tol = +4.4e-15 mm, but the two crossings
    // computed from it are 0.999999999999998 tol apart. Before T-0022 `addPoint` merged them into
    // one untangent hit at the first crossing; now they fold into one tangent hit at the foot.
    const LineSpan line{{-0x1.df87ce26aa716p+3, 0x1.08de41214af33p+5}, {-0x1.c24df54f3402cp+3, 0x1.00e2bff760c67p+5}};
    const ArcSpan circle = ArcSpan::fullCircle({-0x1.07d20be53b2f8p+4, 0x1.ed0c41a28808p+4}, 0x1.54206c56761f4p+1);
    IntersectionResult res;
    CHECK(mirrored(line, circle, &res));
    REQUIRE(res.points.size() == 1);
    CHECK(res.points[0].tangent);
    CHECK(std::fabs(res.points[0].tA - 0.5) <= 1e-12);  // the foot: the line is symmetric about it

    // Circle/circle (r = 0.053 against 0.007, 80 mm from the origin), same situation.
    const ArcSpan c1 = ArcSpan::fullCircle({0x1.3ca2c6d51ae6dp+6, 0x1.9f90a44326fbep+2}, 0x1.b4aa7ef7a5249p-5);
    const ArcSpan c2 = ArcSpan::fullCircle({0x1.3c9a66fc2fe49p+6, 0x1.a27bc0092410ep+2}, 0x1.ca3b92451746ep-8);
    CHECK(mirrored(c1, c2, &res));
    REQUIRE(res.points.size() == 1);
    CHECK(res.points[0].tangent);
}

TEST_CASE("T-0022 AC2: line/circle crossings a rounding error further apart than tol",
          "[geometry][intersect][t0022][property]") {
    // The chord 2s is tol (1 + eta), |eta| <= 1e-11 (|2s - tol| up to 1e-14 mm), both crossings
    // on both spans. Where 2s exceeds tol only by rounding the computed points can still be within
    // tol of each other; before T-0022 they were merged into one hit not flagged tangent. Small
    // full circles (r 0.005..0.1, no ends) far from the origin make the half chord accurate and
    // the points' rounding large, which is where that happens (~0.5 % of them; main: 108 of
    // 20,000 in a calibration run); larger partial arcs cover the rest of the threshold.
    SplitMix rng{0x70022A2ULL};
    int failures = 0;
    int single = 0;
    for (int i = 0; i < 20000; ++i) {
        const bool small = i % 2 == 0;
        const double r = small ? rng.logUniform(0.005, 0.1) : rng.logUniform(0.1, 100.0);
        const Vec2 centre{rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0)};
        const Vec2 u = Vec2::fromAngle(rng.uniform(-kPi, kPi));
        const Vec2 n = u.perp();
        const double s = 0.5 * tol.distance * (1.0 + rng.uniform(-1e-11, 1e-11));
        const double h = std::sqrt((r - s) * (r + s));
        const Vec2 foot = centre + n * h;
        const double ext = std::sqrt(6.0 * r * tol.distance) + rng.uniform(0.5, 3.0);
        const LineSpan line{foot - u * ext, foot + u * ext};
        const Span circle = small || i % 3 == 0 ? Span{ArcSpan::fullCircle(centre, r)}
                                                : Span{arcAround(rng, centre, r, n.angle(), i % 4 == 1)};
        IntersectionResult res;
        const bool ok = mirrored(line, circle, &res) && foldedOrSeparate(res);
        single += res.points.size() == 1 ? 1 : 0;
        if (!ok) {
            ++failures;
            UNSCOPED_INFO("case " << i << " r=" << r << " 2s-tol=" << 2.0 * s - tol.distance
                                  << " hits=" << res.points.size());
        }
    }
    CHECK(failures == 0);
    CHECK(single > 1000);
    CHECK(20000 - single > 1000);
}

TEST_CASE("T-0022 AC2: circle/circle crossings a rounding error further apart than tol",
          "[geometry][intersect][t0022][property]") {
    // Two circles through x1, x2 = m -+ u c/2 with c = tol (1 + eta), |eta| <= 1e-11, centres on
    // either side or the same side of the chord: one tangent hit, or two untangent hits further
    // apart than tol, never one untangent hit (the merge of two crossings by `addPoint`). Small
    // full circles far from the origin as above (main: 138 of 20,000 in a calibration run), and
    // partial arcs of radius 0.1..10.
    SplitMix rng{0x70022A3ULL};
    int failures = 0;
    int single = 0;
    int pairs = 0;
    for (int i = 0; i < 20000; ++i) {
        const bool small = i % 2 == 0;
        const double c = tol.distance * (1.0 + rng.uniform(-1e-11, 1e-11));
        const double r1 = small ? rng.logUniform(0.005, 0.1) : rng.logUniform(0.1, 10.0);
        const double r2 = small ? rng.logUniform(0.005, 0.1) : rng.logUniform(0.1, 10.0);
        const Vec2 m{rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0)};
        const Vec2 u = Vec2::fromAngle(rng.uniform(-kPi, kPi));
        const Vec2 n = u.perp();
        const Vec2 c1 = m + n * std::sqrt(r1 * r1 - 0.25 * c * c);
        const double side = i % 4 < 2 ? 1.0 : -1.0;
        const Vec2 c2 = m - n * (side * std::sqrt(r2 * r2 - 0.25 * c * c));
        if (distance(c1, c2) <= (small ? 0.005 : 0.05)) {
            continue;  // (nearly) the same circle
        }
        const auto around = [&](Vec2 cc, double rr) {
            return arcAround(rng, cc, rr, (m - cc).angle(), rng.uniform(0.0, 1.0) < 0.5);
        };
        const Span a = small ? Span{ArcSpan::fullCircle(c1, r1)} : Span{around(c1, r1)};
        const Span b = small ? Span{ArcSpan::fullCircle(c2, r2)} : Span{around(c2, r2)};
        IntersectionResult res;
        const bool ok = mirrored(a, b, &res) && foldedOrSeparate(res);
        ++pairs;
        single += res.points.size() == 1 ? 1 : 0;
        if (!ok) {
            ++failures;
            UNSCOPED_INFO("case " << i << " r1=" << r1 << " r2=" << r2 << " hits=" << res.points.size());
        }
    }
    CHECK(failures == 0);
    CHECK(pairs > 18000);
    CHECK(single > 1000);
    CHECK(pairs - single > 1000);
}
