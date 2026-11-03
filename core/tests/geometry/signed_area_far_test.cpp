// T-0147: signed area of small contours far from the origin, against exact oracles.
//
// Polygons: the shoelace sum of the stored double coordinates is evaluated exactly with
// floating-point expansions (error-free products via fma, error-free sums), so the oracle has no
// cancellation at all. Arcs and Beziers: the stored contour is translated near the origin by an
// exactly representable shift (every coordinate difference is exact), and its area is computed
// there by an independent formula (chord triangle + circular segment for arcs, the closed-form
// control-point formula for cubics, straight gaps between all span ends). The result must match
// to rounding relative to the contour's own size, wherever it lies up to 1e9 mm.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

// ------------------------------------------------------------ exact expansion arithmetic

struct TwoTerm {
    double hi;
    double lo;
};

TwoTerm twoSum(double a, double b) {
    const double s = a + b;
    const double bv = s - a;
    const double av = s - bv;
    return {s, (a - av) + (b - bv)};
}

TwoTerm twoProduct(double a, double b) {
    const double p = a * b;
    return {p, std::fma(a, b, -p)};
}

// Shewchuk's GROW-EXPANSION with zero elimination: `e` stays a non-overlapping expansion in
// increasing magnitude whose exact sum is the running total.
void grow(std::vector<double>& e, double b) {
    std::vector<double> out;
    out.reserve(e.size() + 1);
    double q = b;
    for (const double c : e) {
        const TwoTerm t = twoSum(q, c);
        if (t.lo != 0.0) {
            out.push_back(t.lo);
        }
        q = t.hi;
    }
    if (q != 0.0) {
        out.push_back(q);
    }
    e = std::move(out);
}

double estimate(const std::vector<double>& e) {
    double s = 0.0;
    for (const double c : e) {
        s += c;  // smallest first: the result is within about one ulp of the exact sum
    }
    return s;
}

// Exact (to the final rounding) signed area of the closed polygon through the stored points.
double exactPolygonArea(const std::vector<Vec2>& pts) {
    std::vector<double> e;
    for (std::size_t i = 0, n = pts.size(); i < n; ++i) {
        const Vec2 a = pts[i];
        const Vec2 b = pts[(i + 1) % n];
        const TwoTerm p = twoProduct(a.x, b.y);
        const TwoTerm q = twoProduct(-a.y, b.x);
        grow(e, p.lo);
        grow(e, p.hi);
        grow(e, q.lo);
        grow(e, q.hi);
    }
    return 0.5 * estimate(e);
}

// The old evaluation on absolute coordinates, to show the cases are hard ones. Each product is
// rounded on its own (volatile): a compiler that contracts x1 * y2 - y1 * x2 into one fma (GCC 15
// at -march=x86-64-v3, T-0159) computes the cross product more accurately, and the sanity check
// below would then measure the compiler instead of the fixture.
double naiveArea(const std::vector<Vec2>& pts) {
    double s = 0.0;
    for (std::size_t i = 0, n = pts.size(); i < n; ++i) {
        const Vec2 a = pts[i];
        const Vec2 b = pts[(i + 1) % n];
        const volatile double p = a.x * b.y;
        const volatile double q = a.y * b.x;
        s += p - q;
    }
    return 0.5 * s;
}

// ------------------------------------------------------------------- random placement

// A placement up to 1e9 mm from the origin, random sign per axis, including the origin region.
Vec2 randomBase(SplitMix& rng) {
    const double mag = rng.logUniform(1e-3, 1e9);
    const double ang = rng.uniform(0.0, 2.0 * kPi);
    return {mag * std::cos(ang), mag * std::sin(ang)};
}

// Random star-shaped polygon of `n` vertices and radius about `size`, stored at `base`.
std::vector<Vec2> randomStar(SplitMix& rng, Vec2 base, double size, std::size_t n) {
    std::vector<double> angles(n);
    for (double& a : angles) {
        a = rng.uniform(0.0, 2.0 * kPi);
    }
    std::sort(angles.begin(), angles.end());
    std::vector<Vec2> pts;
    for (const double a : angles) {
        const double r = size * rng.uniform(0.3, 1.0);
        pts.push_back(Vec2{base.x + r * std::cos(a), base.y + r * std::sin(a)});
    }
    return pts;
}

double extentOf(const std::vector<Vec2>& pts) {
    Box2 b;
    for (const Vec2 p : pts) {
        b.expand(p);
    }
    return std::max(b.width(), b.height());
}

// ------------------------------------------------- independent oracle near the origin

// Signed area of a contour lying near the origin: lines exactly (expansions), arcs as chord
// triangle + circular segment, cubics by the closed-form control-point formula, and a straight
// gap wherever a span does not start exactly where the previous one ended (incl. the closing one).
double oracleNearOrigin(const Contour& c) {
    std::vector<double> e;  // twice the polygon part, exact
    double curved = 0.0;    // area beyond the chords
    const auto chord = [&e](Vec2 a, Vec2 b) {
        const TwoTerm p = twoProduct(a.x, b.y);
        const TwoTerm q = twoProduct(-a.y, b.x);
        grow(e, p.lo);
        grow(e, p.hi);
        grow(e, q.lo);
        grow(e, q.hi);
    };
    Vec2 prevEnd = startPoint(c.spans.front());
    for (const Span& s : c.spans) {
        const Vec2 a = startPoint(s);
        const Vec2 b = endPoint(s);
        if (a.x != prevEnd.x || a.y != prevEnd.y) {
            chord(prevEnd, a);
        }
        chord(a, b);
        if (const auto* arc = std::get_if<ArcSpan>(&s)) {
            // Area between the arc and its chord (signed with the sweep), corrected for the end
            // points being evaluated (the triangle uses the computed end points a and b).
            const double sweep = arc->sweepAngle();
            const double r = arc->radius;
            const Vec2 pa = arc->centre + Vec2::fromAngle(arc->startAngle, r);
            const Vec2 pb = arc->centre + Vec2::fromAngle(arc->startAngle + sweep, r);
            curved += 0.5 * r * r * (sweep - std::sin(sweep));
            // The exact arc ends are pa and pb; join them to the computed ends a and b.
            curved += 0.5 * (a.cross(pa) + pb.cross(b));
            // Chord triangle with the exact ends instead of the computed ones.
            curved += 0.5 * (pa.cross(pb) - a.cross(b));
        } else if (const auto* bz = std::get_if<BezierSpan>(&s)) {
            // Area between the cubic and its chord: 3/20 * cross-product combination (exact).
            const Vec2 p0 = bz->p0;
            const Vec2 p1 = bz->p1;
            const Vec2 p2 = bz->p2;
            const Vec2 p3 = bz->p3;
            const double twice = (6.0 * p0.cross(p1) + 3.0 * p0.cross(p2) + 1.0 * p0.cross(p3) + 3.0 * p1.cross(p2) +
                                  3.0 * p1.cross(p3) + 6.0 * p2.cross(p3)) /
                                 10.0;
            curved += 0.5 * (twice - p0.cross(p3));
        }
        prevEnd = b;
    }
    const Vec2 first = startPoint(c.spans.front());
    if (prevEnd.x != first.x || prevEnd.y != first.y) {
        chord(prevEnd, first);
    }
    return 0.5 * estimate(e) + curved;
}

// Shift that is a multiple of 2^-20 (exact at every magnitude below 2^32), close to `p`.
Vec2 gridShift(Vec2 p) {
    constexpr double q = 1.0 / 1048576.0;
    return {std::round(p.x / q) * q, std::round(p.y / q) * q};
}

Span shifted(const Span& s, Vec2 by) {
    return std::visit(
        [by](const auto& v) -> Span {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return LineSpan{v.start - by, v.end - by};
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                ArcSpan a = v;
                a.centre = v.centre - by;
                return a;
            } else {
                return BezierSpan{v.p0 - by, v.p1 - by, v.p2 - by, v.p3 - by};
            }
        },
        s);
}

// The stored contour moved near the origin; for an arc only its centre moves, so its computed end
// points may differ from the moved ones: line ends are kept as the moved stored values, so the
// gaps (and hence the enclosed region) are those of the stored contour.
Contour shiftedContour(const Contour& c, Vec2 by) {
    Contour out;
    out.closed = c.closed;
    for (const Span& s : c.spans) {
        out.append(shifted(s, by));
    }
    return out;
}

// Rounded rectangle (lines + quarter arcs at arbitrary rotation) of size w x h, corner radius r.
// Line ends are the arcs' computed end points, as contour builders produce them.
Contour roundedRect(Vec2 base, double w, double h, double r, double rot, bool ccw) {
    const Vec2 ux = Vec2::fromAngle(rot, 1.0);
    const Vec2 uy = Vec2::fromAngle(rot + 0.5 * kPi, 1.0);
    const auto at = [&](double x, double y) { return base + ux * x + uy * y; };
    const std::array<Vec2, 4> centres{at(w - r, r), at(w - r, h - r), at(r, h - r), at(r, r)};
    Contour c;
    c.closed = true;
    std::vector<ArcSpan> arcs;
    for (int i = 0; i < 4; ++i) {
        const double a0 = rot - 0.5 * kPi + 0.5 * kPi * i;
        arcs.push_back(ArcSpan{centres[static_cast<std::size_t>(i)], r, a0, a0 + 0.5 * kPi, ArcDirection::CCW});
    }
    for (std::size_t i = 0; i < 4; ++i) {
        c.append(arcs[i]);
        c.append(LineSpan{arcs[i].endPoint(), arcs[(i + 1) % 4].startPoint()});
    }
    if (!ccw) {
        c.reverse();
    }
    return c;
}

// Closed shape of two cubic Beziers bulging either side of a chord, plus a straight cap.
Contour bezierLens(Vec2 base, double size, double rot, bool ccw) {
    const Vec2 ux = Vec2::fromAngle(rot, size);
    const Vec2 uy = Vec2::fromAngle(rot + 0.5 * kPi, size);
    const auto at = [&](double x, double y) { return base + ux * x + uy * y; };
    Contour c;
    c.closed = true;
    c.append(BezierSpan{at(0.0, 0.0), at(0.3, -0.4), at(0.8, -0.3), at(1.0, 0.0)});
    c.append(LineSpan{at(1.0, 0.0), at(1.0, 0.2)});
    c.append(BezierSpan{at(1.0, 0.2), at(0.7, 0.6), at(0.2, 0.5), at(0.0, 0.0)});
    if (!ccw) {
        c.reverse();
    }
    return c;
}

}  // namespace

TEST_CASE("T-0147 AC1: the repro rectangle 1 x 0.5 mm at 1e8 mm keeps its area", "[contour_ops][T-0147]") {
    int naiveWrong = 0;
    for (int k = 0; k < 20; ++k) {
        const double ox = 1e8 * (1.0 + 0.037 * k);
        const double oy = 0.7e8 * (1.0 + 0.051 * k);
        const std::vector<Vec2> pts{{ox, oy}, {ox + 1.0, oy}, {ox + 1.0, oy + 0.5}, {ox, oy + 0.5}};
        const double exact = exactPolygonArea(pts);
        const Contour c = Contour::fromPolyline(Polyline{pts, true});
        const double got = signedArea(c);
        CHECK(std::fabs(got - exact) <= 1e-12);
        CHECK(std::fabs(Polyline{pts, true}.signedArea() - exact) <= 1e-12);
        CHECK(orientation(c, Tolerance{}) == Orientation::CCW);
        if (std::fabs(naiveArea(pts) - exact) > 0.1) {
            ++naiveWrong;
        }
    }
    // The absolute-coordinate shoelace sum fails on most of these placements.
    CHECK(naiveWrong >= 10);
}

TEST_CASE("T-0147 AC1: random small polygons up to 1e9 mm match the exact shoelace sum",
          "[contour_ops][T-0147]") {
    SplitMix rng{0x7147A11CEULL};
    int hard = 0;
    for (int iter = 0; iter < 4000; ++iter) {
        const Vec2 base = randomBase(rng);
        const double size = rng.logUniform(1e-2, 10.0);
        const std::size_t n = 3 + static_cast<std::size_t>(rng.next() % 14);
        std::vector<Vec2> pts = randomStar(rng, base, size, n);
        if (rng.next() % 2 == 0) {
            std::reverse(pts.begin(), pts.end());
        }
        const double exact = exactPolygonArea(pts);
        const double ext = extentOf(pts);
        // Rounding of the local differences and of the n products and sums.
        const double bound = 4.0 * static_cast<double>(n + 2) * kEps * ext * ext;
        const Polyline pl{pts, true};
        const Contour c = Contour::fromPolyline(pl);
        const double got = signedArea(c);
        INFO("iter " << iter << " base " << base.x << "," << base.y << " size " << size << " n " << n);
        REQUIRE(std::fabs(got - exact) <= bound);
        REQUIRE(std::fabs(pl.signedArea() - exact) <= bound);
        if (std::fabs(exact) > 1e3 * bound) {
            REQUIRE((got > 0.0) == (exact > 0.0));
        }
        if (std::fabs(naiveArea(pts) - exact) > 1e-3 * std::fabs(exact)) {
            ++hard;
        }
    }
    // A fair share of the cases defeat the old absolute-coordinate evaluation.
    CHECK(hard > 400);
}

TEST_CASE("T-0147 AC1: rounded rectangles (lines + arcs) up to 1e9 mm match the oracle",
          "[contour_ops][T-0147]") {
    SplitMix rng{0xA4C5ULL};
    for (int iter = 0; iter < 3000; ++iter) {
        const Vec2 base = randomBase(rng);
        const double w = rng.logUniform(1e-2, 10.0);
        const double h = w * rng.uniform(0.2, 1.0);
        const double r = 0.5 * h * rng.uniform(0.05, 0.95);
        const double rot = rng.uniform(0.0, 2.0 * kPi);
        const bool ccw = rng.next() % 2 == 0;
        const Contour c = roundedRect(base, w, h, r, rot, ccw);
        const Vec2 shift = gridShift(c.startPoint());
        const double exact = oracleNearOrigin(shiftedContour(c, shift));
        const double got = signedArea(c);
        const double ext = std::max(w, h);
        INFO("iter " << iter << " base " << base.x << "," << base.y << " w " << w << " h " << h << " r " << r);
        REQUIRE(std::fabs(got - exact) <= 64.0 * kEps * ext * ext);
        REQUIRE((got > 0.0) == ccw);
        // The geometric area w h - (4 - pi) r^2 holds up to the rounding of the stored points.
        const double ideal = (w * h - (4.0 - kPi) * r * r) * (ccw ? 1.0 : -1.0);
        const double pointNoise = 8.0 * (std::fabs(base.x) + std::fabs(base.y) + ext) * kEps * 2.0 * (w + h);
        REQUIRE(std::fabs(got - ideal) <= pointNoise + 1e-12 * ext * ext);
    }
}

TEST_CASE("T-0147 AC1: full circles and Bezier shapes up to 1e9 mm match the oracle", "[contour_ops][T-0147]") {
    SplitMix rng{0xB3213ULL};
    for (int iter = 0; iter < 2000; ++iter) {
        const Vec2 base = randomBase(rng);
        const double size = rng.logUniform(1e-2, 10.0);
        const bool ccw = rng.next() % 2 == 0;
        const Contour circle = Contour::circle(base, size, ccw ? ArcDirection::CCW : ArcDirection::CW);
        const double gotCircle = signedArea(circle);
        INFO("iter " << iter << " base " << base.x << "," << base.y << " size " << size);
        REQUIRE(std::fabs(gotCircle - (ccw ? 1.0 : -1.0) * kPi * size * size) <= 64.0 * kEps * size * size);

        const Contour lens = bezierLens(base, size, rng.uniform(0.0, 2.0 * kPi), ccw);
        const Vec2 shift = gridShift(lens.startPoint());
        const double exact = oracleNearOrigin(shiftedContour(lens, shift));
        const double got = signedArea(lens);
        REQUIRE(std::fabs(got - exact) <= 64.0 * kEps * size * size);
        REQUIRE((got > 0.0) == ccw);
    }
}

TEST_CASE("T-0147: signedArea is invariant under exact translation of a mixed contour", "[contour_ops][T-0147]") {
    // A D shape (line, half circle, lines) and a Bezier shape, moved by exactly representable
    // shifts out to 1e9 mm: the area is the same to rounding of the local size.
    Contour d;
    d.closed = true;
    d.append(LineSpan{{0.0, 0.0}, {1.0, 0.0}});
    d.append(ArcSpan::fromCentre({1.0, 0.25}, {1.0, 0.0}, {1.0, 0.5}, ArcDirection::CCW));
    d.append(LineSpan{{1.0, 0.5}, {0.0, 0.5}});
    d.append(LineSpan{{0.0, 0.5}, {0.0, 0.0}});
    const double ref = signedArea(d);
    REQUIRE(std::fabs(ref - (0.5 + 0.5 * kPi * 0.0625)) <= 1e-14);
    for (double mag = 1.0; mag <= 1e9; mag *= 10.0) {
        for (const Vec2 dir : {Vec2{1.0, 0.0}, Vec2{-0.6, 0.8}, Vec2{-0.28, -0.96}}) {
            const Vec2 by = gridShift(dir * mag);
            const Contour moved = shiftedContour(d, Vec2{0.0, 0.0} - by);
            INFO("mag " << mag << " dir " << dir.x << "," << dir.y);
            CHECK(std::fabs(signedArea(moved) - ref) <= 1e-12);
        }
    }
}
