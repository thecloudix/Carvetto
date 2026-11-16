// T-0321: Vector Unwrapper (ADR-0114).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "carvetto/geometry/vector_unwrapper.hpp"

using namespace carvetto::geometry;
using Catch::Approx;

namespace {

Polyline regular(int n, Vec2 c, double r, bool closed = true) {
    Polyline p;
    p.closed = closed;
    for (int k = 0; k < n; ++k) {
        const double a = kTwoPi * k / n;
        p.points.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)});
    }
    return p;
}

UnwrapOptions opts(double radius = 10.0) {
    UnwrapOptions o;
    o.radius = radius;
    return o;
}

}  // namespace

TEST_CASE("unwrapper: a circle round its centre becomes one turn of constant distance and round-trips", "[geometry][rotary][unwrapper]") {
    const Vec2 c{3.0, 4.0};
    const Polyline circle = regular(64, c, 5.0);
    auto r = unwrapCrossSection(circle, opts(10.0));
    REQUIRE(r.ok());
    const UnwrapResult& u = r.value();
    CHECK(u.turns == 1);
    CHECK_FALSE(u.contour.closed);
    CHECK(u.centre.x == Approx(c.x).margin(1e-9));
    CHECK(u.centre.y == Approx(c.y).margin(1e-9));
    REQUIRE(u.contour.points.size() == 65);  // the start point repeated one turn further on
    for (const Vec2& p : u.contour.points) CHECK(p.y == Approx(5.0).margin(1e-9));
    CHECK(u.contour.points[1].x - u.contour.points[0].x == Approx(kTwoPi * 10.0 / 64.0));
    CHECK(u.contour.points.back().x - u.contour.points.front().x == Approx(kTwoPi * 10.0));
    const Polyline back = wrapCrossSection(u.contour, u.centre, 10.0, false);
    for (std::size_t k = 0; k < 64; ++k) {
        CHECK(back.points[k].x == Approx(circle.points[k].x).margin(1e-9));
        CHECK(back.points[k].y == Approx(circle.points[k].y).margin(1e-9));
    }
    CHECK(back.points[64].x == Approx(circle.points[0].x).margin(1e-9));
}

TEST_CASE("unwrapper: a clockwise circle turns the other way; flip negates the circumference", "[geometry][rotary][unwrapper]") {
    Polyline cw = regular(32, {0, 0}, 4.0);
    cw.reverse();
    auto r = unwrapCrossSection(cw, opts(5.0));
    REQUIRE(r.ok());
    CHECK(r.value().turns == -1);
    CHECK(r.value().contour.points.back().x - r.value().contour.points.front().x == Approx(-kTwoPi * 5.0));
    UnwrapOptions flipped = opts(5.0);
    flipped.flip = true;
    auto f = unwrapCrossSection(cw, flipped);
    REQUIRE(f.ok());
    CHECK(f.value().turns == -1);
    for (std::size_t k = 0; k < r.value().contour.points.size(); ++k) {
        CHECK(f.value().contour.points[k].x == Approx(-r.value().contour.points[k].x).margin(1e-9));
        CHECK(f.value().contour.points[k].y == Approx(r.value().contour.points[k].y).margin(1e-9));
    }
    const Polyline back = wrapCrossSection(f.value().contour, f.value().centre, 5.0, true);
    CHECK(back.points[5].x == Approx(cw.points[5].x).margin(1e-9));
    CHECK(back.points[5].y == Approx(cw.points[5].y).margin(1e-9));
}

TEST_CASE("unwrapper: a contour that does not enclose the centre stays closed; the seam never makes a jump", "[geometry][rotary][unwrapper]") {
    // A square to the left of the centre, straddling the -X axis (angle pi): atan2 jumps there.
    Polyline sq;
    sq.closed = true;
    sq.points = {{-6, -1}, {-3, -1}, {-3, 1}, {-6, 1}};
    UnwrapOptions o = opts(10.0);
    o.centre = UnwrapCentre::Custom;
    o.customCentre = {0, 0};
    auto r = unwrapCrossSection(sq, o);
    REQUIRE(r.ok());
    CHECK(r.value().turns == 0);
    CHECK(r.value().contour.closed);
    REQUIRE(r.value().contour.points.size() == 4);
    const auto& p = r.value().contour.points;
    for (std::size_t k = 0; k < 4; ++k) {
        CHECK(std::fabs(p[(k + 1) % 4].x - p[k].x) < kPi * 10.0 / 4.0);  // no jump of a turn
    }
    // Distances are the polar distances, angles continue past pi.
    CHECK(p[0].y == Approx(std::hypot(6.0, 1.0)));
    CHECK(p[1].y == Approx(std::hypot(3.0, 1.0)));
    CHECK(p[0].x / 10.0 == Approx(std::atan2(-1.0, -6.0)));  // starts at the principal angle of point 0
    const Polyline back = wrapCrossSection(r.value().contour, r.value().centre, 10.0, false);
    for (std::size_t k = 0; k < 4; ++k) {
        CHECK(back.points[k].x == Approx(sq.points[k].x).margin(1e-9));
        CHECK(back.points[k].y == Approx(sq.points[k].y).margin(1e-9));
    }
}

TEST_CASE("unwrapper: open cross-section, centre options", "[geometry][rotary][unwrapper]") {
    Polyline arc;
    arc.points = {{10, 0}, {0, 10}, {-10, 0}};
    UnwrapOptions o = opts(10.0);
    // Contour centre: the middle of the bounds = (0, 5).
    o.workspaceCentre = {0, 0};
    o.customCentre = {1, 2};
    CHECK(unwrapCentre(o, arc.bounds()).x == Approx(0.0));
    CHECK(unwrapCentre(o, arc.bounds()).y == Approx(5.0));
    o.centre = UnwrapCentre::Workspace;
    CHECK(unwrapCentre(o, arc.bounds()).y == Approx(0.0));
    o.centre = UnwrapCentre::Custom;
    CHECK(unwrapCentre(o, arc.bounds()).x == Approx(1.0));
    o.centre = UnwrapCentre::Workspace;
    auto r = unwrapCrossSection(arc, o);
    REQUIRE(r.ok());
    CHECK(r.value().turns == 0);
    CHECK_FALSE(r.value().contour.closed);
    const auto& p = r.value().contour.points;
    REQUIRE(p.size() == 3);
    CHECK(p[0].x == Approx(0.0).margin(1e-12));
    CHECK(p[1].x == Approx(kPi / 2 * 10.0));
    CHECK(p[2].x == Approx(kPi * 10.0));
    for (const Vec2& q : p) CHECK(q.y == Approx(10.0));
}

TEST_CASE("unwrapper: a point on the centre keeps the angle of the point before it", "[geometry][rotary][unwrapper]") {
    Polyline line;
    line.points = {{2, 2}, {0, 0}, {-2, 2}};
    UnwrapOptions o = opts(1.0);
    o.centre = UnwrapCentre::Custom;
    o.customCentre = {0, 0};
    auto r = unwrapCrossSection(line, o);
    REQUIRE(r.ok());
    const auto& p = r.value().contour.points;
    CHECK(p[1].y == 0.0);
    CHECK(p[1].x == Approx(p[0].x));
    CHECK(p[2].x == Approx(kPi * 3.0 / 4.0).margin(1e-12));
}

TEST_CASE("unwrapper: refinement keeps every point on the source and the unrolled curve within tolerance", "[geometry][rotary][unwrapper]") {
    Polyline seg;
    seg.points = {{10, -5}, {10, 5}};
    UnwrapOptions o = opts(10.0);
    o.centre = UnwrapCentre::Custom;
    o.customCentre = {0, 0};
    auto plain = unwrapCrossSection(seg, o);
    REQUIRE(plain.ok());
    CHECK(plain.value().contour.points.size() == 2);
    o.refine = true;
    o.tolerance = 0.01;
    auto fine = unwrapCrossSection(seg, o);
    REQUIRE(fine.ok());
    const auto& p = fine.value().contour.points;
    CHECK(p.size() > 4);
    CHECK(p.size() < 200);
    const Polyline back = wrapCrossSection(fine.value().contour, {0, 0}, 10.0, false);
    for (const Vec2& q : back.points) CHECK(q.x == Approx(10.0).margin(1e-9));  // on the source segment
    for (std::size_t k = 0; k + 1 < back.points.size(); ++k) {
        const double ym = 0.5 * (back.points[k].y + back.points[k + 1].y);
        const Vec2 truth{std::atan2(ym, 10.0) * 10.0, std::hypot(10.0, ym)};
        const Vec2 chord = (p[k] + p[k + 1]) * 0.5;
        CHECK(distance(truth, chord) <= 0.01 + 1e-9);
        CHECK(back.points[k + 1].y > back.points[k].y);  // in order, no duplicates
    }
    // Operation counter: a tighter tolerance needs more points, deterministically.
    o.tolerance = 0.001;
    auto finer = unwrapCrossSection(seg, o);
    REQUIRE(finer.ok());
    CHECK(finer.value().contour.points.size() > p.size());
    CHECK(unwrapCrossSection(seg, o).value().contour.points.size() == finer.value().contour.points.size());
    // A segment through the middle of the centre needs no extra points (it is straight in polar form too only
    // in angle: the distance is linear in the parameter on each side).
    Polyline radial;
    radial.points = {{1, 1}, {5, 5}};
    CHECK(unwrapCrossSection(radial, o).value().contour.points.size() == 2);
}

TEST_CASE("unwrapper: closed refined contour keeps its closing segment curved", "[geometry][rotary][unwrapper]") {
    Polyline tri;
    tri.closed = true;
    tri.points = {{10, -5}, {10, 5}, {14, 0}};
    UnwrapOptions o = opts(10.0);
    o.centre = UnwrapCentre::Custom;
    o.customCentre = {0, 0};
    o.refine = true;
    o.tolerance = 0.01;
    auto r = unwrapCrossSection(tri, o);
    REQUIRE(r.ok());
    CHECK(r.value().turns == 0);
    CHECK(r.value().contour.closed);
    const Polyline back = wrapCrossSection(r.value().contour, {0, 0}, 10.0, false);
    // Every refined point lies on the triangle's boundary.
    for (const Vec2& q : back.points) {
        const bool onA = std::fabs(q.x - 10.0) < 1e-9 && q.y >= -5 - 1e-9 && q.y <= 5 + 1e-9;
        const double dB = std::fabs((q.x - 10.0) * 5.0 + (q.y - 5.0) * 4.0) / std::hypot(5.0, 4.0);  // line (10,5)-(14,0)
        const double dC = std::fabs((q.x - 10.0) * -5.0 + (q.y + 5.0) * 4.0) / std::hypot(5.0, 4.0);  // line (14,0)-(10,-5)
        CHECK((onA || dB < 1e-9 || dC < 1e-9));
    }
    CHECK(r.value().contour.points.size() > 3);
}

TEST_CASE("unwrapper: refused inputs and the point cap", "[geometry][rotary][unwrapper]") {
    Polyline one;
    one.points = {{0, 0}};
    CHECK_FALSE(unwrapCrossSection(one, opts()).ok());
    Polyline bad;
    bad.points = {{0, 0}, {std::nan(""), 1}};
    CHECK_FALSE(unwrapCrossSection(bad, opts()).ok());
    Polyline ok;
    ok.points = {{0, 0}, {1, 1}};
    CHECK(unwrapCrossSection(ok, opts()).ok());
    for (const double r : {0.0, -1.0, std::nan("")}) CHECK_FALSE(unwrapCrossSection(ok, opts(r)).ok());
    UnwrapOptions t = opts();
    t.refine = true;
    t.tolerance = 0.0;
    CHECK_FALSE(unwrapCrossSection(ok, t).ok());
    t.refine = false;
    CHECK(unwrapCrossSection(ok, t).ok());  // the tolerance only matters when refining
    UnwrapOptions c = opts();
    c.centre = UnwrapCentre::Custom;
    c.customCentre = {std::nan(""), 0};
    CHECK_FALSE(unwrapCrossSection(ok, c).ok());
    c.centre = UnwrapCentre::Workspace;
    c.workspaceCentre = {0, std::nan("")};
    CHECK_FALSE(unwrapCrossSection(ok, c).ok());
    Polyline many;
    for (std::size_t i = 0; i <= kMaxUnwrapPoints; ++i) many.points.push_back({1.0 + static_cast<double>(i % 7), static_cast<double>(i % 5)});
    CHECK_FALSE(unwrapCrossSection(many, opts()).ok());
    many.points.pop_back();
    CHECK(unwrapCrossSection(many, opts()).ok());
    // A refinement that would need more than the cap is refused, not truncated.
    Polyline longSeg;
    longSeg.points = {{10.0, -100.0}, {10.0, 100.0}};
    UnwrapOptions tight = opts(1.0);
    tight.centre = UnwrapCentre::Custom;
    tight.customCentre = {0, 0};
    tight.refine = true;
    tight.tolerance = 1e-11;
    const auto huge = unwrapCrossSection(longSeg, tight);
    REQUIRE_FALSE(huge.ok());
    CHECK(huge.error().find("too many points") != std::string::npos);
    tight.tolerance = 1e-4;  // the same segment at a sane tolerance is fine
    CHECK(unwrapCrossSection(longSeg, tight).ok());
}
