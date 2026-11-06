// T-0175: closed contours bounded by curves within the chordal tolerance of their chords (a thin
// lens, a crescent, a flat tab) must not polygonise onto those chords: the polygon would have zero
// area there, and the booleans (Clipper2) drop zero-area parts, so a Pocket cut straight through
// such an island.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"

using namespace carvetto::geometry;

namespace {

/// Arc from P to Q with sagitta s (s > 0 bulges to the left of P→Q).
ArcSpan arcWithSagitta(Vec2 P, Vec2 Q, double s) {
    const Vec2 d = Q - P;
    const double h = 0.5 * d.length();
    const double radius = (h * h + s * s) / (2.0 * std::fabs(s));
    const Vec2 n = (d * (1.0 / d.length())).perp();
    const Vec2 centre = (P + Q) * 0.5 - n * ((radius - std::fabs(s)) * (s > 0.0 ? 1.0 : -1.0));
    const double a0 = std::atan2(P.y - centre.y, P.x - centre.x);
    const double a1 = std::atan2(Q.y - centre.y, Q.x - centre.x);
    return ArcSpan{centre, radius, a0, a1, s > 0.0 ? ArcDirection::CW : ArcDirection::CCW};
}

Contour lens(double s) {
    Contour c;
    c.closed = true;
    c.append(arcWithSagitta({-2, 0}, {2, 0}, s));
    c.append(LineSpan{{2, 0}, {-2, 0}});
    return c;
}

double area(const std::vector<Contour>& cs) {
    double a = 0.0;
    for (const Contour& c : cs) {
        Polyline pl;
        pl.closed = true;
        for (const Span& s : c.spans) {
            pl.points.push_back(startPoint(s));
        }
        a += pl.signedArea();
    }
    return a;
}

double distanceToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double t = std::clamp((p - a).dot(ab) / ab.lengthSquared(), 0.0, 1.0);
    return distance(p, a + ab * t);
}

}  // namespace

TEST_CASE("T-0175: a thin lens polygonises to a polygon with area, within the chordal tolerance",
          "[geometry][polygonise][t0175]") {
    Tolerance tol;
    tol.chordal = 0.05;
    for (const double s : {0.04, 0.004, 1e-5, -0.02}) {
        INFO("sagitta " << s);
        const Contour c = lens(s);
        const Polyline pl = c.polygonise(tol);
        // The arc's midpoint (its farthest point from the chord) is kept.
        REQUIRE(pl.points.size() == 3);
        CHECK(distance(pl.points[1], Vec2{0.0, s}) < 1e-9);
        // Area: the triangle, between half and all of the true segment area (≈ 2/3·chord·s).
        const double trueArea = 2.0 / 3.0 * 4.0 * std::fabs(s);
        CHECK(std::fabs(pl.signedArea()) > 0.5 * trueArea);
        CHECK(std::fabs(pl.signedArea()) <= trueArea * 1.001);
        // The sign follows the contour's orientation (CW lens above the chord: negative).
        CHECK((pl.signedArea() < 0.0) == (s > 0.0));
        // The booleans and offsets keep it.
        const std::vector<Contour> u = unionContours({c}, tol, FillRule::EvenOdd);
        REQUIRE(u.size() == 1);
        CHECK(std::fabs(area(u)) > 0.5 * trueArea);
        CHECK(offsetContours({c}, 0.5, JoinType::Round, tol).size() == 1);
    }
}

TEST_CASE("T-0175: open contours and curves already off their chord polygonise as before",
          "[geometry][polygonise][t0175]") {
    Tolerance tol;
    tol.chordal = 0.05;
    // Open: one chord, as before (no region, nothing to keep).
    Contour open = lens(0.004);
    open.closed = false;
    CHECK(open.polygonise(tol).points.size() == 3);  // start, arc end, line end
    // A curve needing several chords already has vertices off its chord: unchanged when closed.
    Contour circle = Contour::circle({0, 0}, 1.0);
    std::vector<Vec2> pts;
    polygoniseInto(circle.spans[0], tol, pts);
    const Polyline pl = circle.polygonise(tol);
    CHECK(pl.points.size() == pts.size());
    Contour flatSide = lens(0.5);  // sagitta 0.5 > chordal: several chords
    std::vector<Vec2> arcPts;
    polygoniseInto(flatSide.spans[0], tol, arcPts);
    CHECK(flatSide.polygonise(tol).points.size() == arcPts.size() + 1);  // + the start point
}

TEST_CASE("T-0175: crescents and Bezier lenses (C- and S-shaped) keep area", "[geometry][polygonise][t0175]") {
    Tolerance tol;
    tol.chordal = 0.05;
    SECTION("crescent: two flat arcs bulging the same way") {
        Contour c;
        c.closed = true;
        c.append(arcWithSagitta({-2, 0}, {2, 0}, 0.02));
        c.append(arcWithSagitta({2, 0}, {-2, 0}, -0.01));
        const Polyline pl = c.polygonise(tol);
        CHECK(pl.points.size() == 4);
        // True area 2/3·4·(0.02 − 0.01); the polygon keeps 1/2·4·0.01.
        CHECK(std::fabs(pl.signedArea()) > 0.019);
        CHECK(unionContours({c}, tol, FillRule::EvenOdd).size() == 1);
    }
    SECTION("C-shaped Bezier lens") {
        const double s = 0.01;
        Contour c;
        c.closed = true;
        c.append(BezierSpan{{-2, 0}, {-2.0 / 3.0, 4.0 / 3.0 * s}, {2.0 / 3.0, 4.0 / 3.0 * s}, {2, 0}});
        c.append(LineSpan{{2, 0}, {-2, 0}});
        const Polyline pl = c.polygonise(tol);
        REQUIRE(pl.points.size() >= 3);
        CHECK(std::fabs(pl.signedArea()) > 0.5 * (2.0 / 3.0 * 4.0 * s));
        CHECK(unionContours({c}, tol, FillRule::EvenOdd).size() == 1);
        // Every vertex lies on the curve's side of the chord, within the tolerance of the curve.
        for (const Vec2 p : pl.points) {
            CHECK(p.y >= 0.0);
            CHECK(p.y <= s + 1e-12);
        }
    }
    SECTION("S-shaped Bezier lens: vertices either side of the chord") {
        const double h = 0.02;
        Contour c;
        c.closed = true;
        const BezierSpan b{{-2, 0}, {-2.0 / 3.0, h}, {2.0 / 3.0, -h}, {2, 0}};
        c.append(b);
        c.append(LineSpan{{2, 0}, {-2, 0}});
        const Polyline pl = c.polygonise(tol);
        bool above = false;
        bool below = false;
        for (const Vec2 p : pl.points) {
            above = above || p.y > 0.2 * h;
            below = below || p.y < -0.2 * h;
        }
        CHECK(above);
        CHECK(below);
        // Two lobes of opposite orientation: the even-odd union keeps both.
        const std::vector<Contour> u = unionContours({c}, tol, FillRule::EvenOdd);
        CHECK(u.size() == 2);
        // Within the chordal tolerance of the curve.
        double worst = 0.0;
        for (int k = 0; k <= 400; ++k) {
            const Vec2 q = b.pointAt(static_cast<double>(k) / 400.0);
            double best = 1e300;
            for (std::size_t i = 0; i < pl.points.size(); ++i) {
                best = std::min(best, distanceToSegment(q, pl.points[i], pl.points[(i + 1) % pl.points.size()]));
            }
            worst = std::max(worst, best);
        }
        CHECK(worst <= tol.chordal);
    }
}
