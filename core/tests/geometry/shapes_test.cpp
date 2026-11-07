#include "carvetto/geometry/shapes.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>
#include <random>
#include <span>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/contour_ops.hpp"

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

const Tolerance kTol = Tolerance::defaults();

Contour need(const ShapeResult<Contour>& r) {
    INFO((r.ok() ? std::string() : r.error()));
    REQUIRE(r.ok());
    return r.value();
}

void requireClosedCcw(const Contour& c) {
    REQUIRE(c.closed);
    REQUIRE(c.isContinuous(kTol));
    REQUIRE(signedArea(c) > 0.0);
}

/// Numerical perimeter of an ellipse with semi-axes a, b (Simpson on the speed integrand).
double ellipsePerimeter(double a, double b) {
    const int n = 20000;
    const double h = kTwoPi / n;
    double sum = 0.0;
    for (int i = 0; i <= n; ++i) {
        const double t = h * i;
        const double f = std::hypot(a * std::sin(t), b * std::cos(t));
        sum += f * (i == 0 || i == n ? 1.0 : (i % 2 ? 4.0 : 2.0));
    }
    return sum * h / 3.0;
}

}  // namespace

TEST_CASE("rectangle: square corners, area, perimeter, start and winding", "[shapes]") {
    const Contour c = need(buildRectangle({10, 20}, 6, 4, RectCorner::Square, 0));
    requireClosedCcw(c);
    CHECK(c.spanCount() == 4);
    CHECK_THAT(signedArea(c), WithinAbs(24.0, 1e-12));
    CHECK_THAT(c.length(), WithinAbs(20.0, 1e-12));
    CHECK(c.startPoint() == Vec2{7, 18});
    // Corner size is ignored for square corners.
    CHECK(need(buildRectangle({10, 20}, 6, 4, RectCorner::Square, 99)) == c);
}

TEST_CASE("rectangle: rounded corners are exact arcs", "[shapes]") {
    const double w = 30, h = 20, r = 4;
    const Contour c = need(buildRectangle({1, 2}, w, h, RectCorner::Rounded, r));
    requireClosedCcw(c);
    CHECK(c.spanCount() == 8);
    CHECK_THAT(signedArea(c), WithinAbs(w * h - (4 - kPi) * r * r, 1e-9));
    CHECK_THAT(c.length(), WithinAbs(2 * (w + h) - 8 * r + kTwoPi * r, 1e-9));
    CHECK(kTol.equalPoints(c.startPoint(), {1 - 15 + r, 2 - 10}));
    for (const Span& s : c.spans) {
        if (const auto* a = std::get_if<ArcSpan>(&s)) {
            CHECK(a->direction == ArcDirection::CCW);
            CHECK_THAT(a->sweepAngle(), WithinAbs(kPi / 2, 1e-12));
        }
    }
}

TEST_CASE("rectangle: chamfered corners", "[shapes]") {
    const double w = 30, h = 20, k = 3;
    const Contour c = need(buildRectangle({0, 0}, w, h, RectCorner::Chamfered, k));
    requireClosedCcw(c);
    CHECK(c.spanCount() == 8);
    CHECK_THAT(signedArea(c), WithinAbs(w * h - 2 * k * k, 1e-9));
    CHECK_THAT(c.length(), WithinAbs(2 * (w + h) - 8 * k + 4 * k * std::sqrt(2.0), 1e-9));
}

TEST_CASE("rectangle: corner size consuming a whole side drops that side", "[shapes]") {
    const Contour stadium = need(buildRectangle({0, 0}, 20, 10, RectCorner::Rounded, 5));
    requireClosedCcw(stadium);
    CHECK(stadium.spanCount() == 6);  // the two short sides vanish
    CHECK_THAT(signedArea(stadium), WithinAbs(10 * 10 + kPi * 25, 1e-9));
    const Contour round = need(buildRectangle({0, 0}, 10, 10, RectCorner::Rounded, 5));
    CHECK(round.spanCount() == 4);
    CHECK_THAT(signedArea(round), WithinAbs(kPi * 25, 1e-9));
    CHECK(need(buildRectangle({0, 0}, 8, 8, RectCorner::Chamfered, 4)).spanCount() == 4);
    // Zero size is a plain rectangle.
    CHECK(need(buildRectangle({0, 0}, 8, 6, RectCorner::Rounded, 0)).spanCount() == 4);
}

TEST_CASE("rectangle: degenerate parameters are errors", "[shapes]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(buildRectangle({0, 0}, 0, 4, RectCorner::Square, 0).ok());
    CHECK_FALSE(buildRectangle({0, 0}, 4, -1, RectCorner::Square, 0).ok());
    CHECK_FALSE(buildRectangle({0, 0}, nan, 4, RectCorner::Square, 0).ok());
    CHECK_FALSE(buildRectangle({nan, 0}, 4, 4, RectCorner::Square, 0).ok());
    CHECK_FALSE(buildRectangle({0, 0}, 10, 4, RectCorner::Rounded, 2.01).ok());
    CHECK_FALSE(buildRectangle({0, 0}, 10, 4, RectCorner::Chamfered, 2.01).ok());
    CHECK_FALSE(buildRectangle({0, 0}, 10, 4, RectCorner::Rounded, -1).ok());
    CHECK_FALSE(buildRectangle({0, 0}, 10, 4, RectCorner::Rounded, nan).ok());
    CHECK_FALSE(buildRectangle({0, 0}, std::numeric_limits<double>::infinity(), 4, RectCorner::Square, 0).ok());
}

TEST_CASE("circle", "[shapes]") {
    const Contour c = need(buildCircle({3, 4}, 5));
    requireClosedCcw(c);
    CHECK(c.spanCount() == 1);
    CHECK_THAT(signedArea(c), WithinRel(kPi * 25, 1e-12));
    CHECK_THAT(c.length(), WithinRel(kTwoPi * 5, 1e-12));
    CHECK(c.startPoint() == Vec2{8, 4});
    CHECK_FALSE(buildCircle({0, 0}, 0).ok());
    CHECK_FALSE(buildCircle({0, 0}, -2).ok());
    CHECK_FALSE(buildCircle({0, 0}, std::numeric_limits<double>::quiet_NaN()).ok());
}

TEST_CASE("ellipse: area, perimeter and radial deviation within tolerance", "[shapes]") {
    const double w = 40, h = 10, ang = 0.7;
    const Vec2 centre{5, -3};
    const Contour c = need(buildEllipse(centre, w, h, ang));
    requireClosedCcw(c);
    CHECK_THAT(signedArea(c), WithinRel(kPi * 20 * 5, 1e-4));
    CHECK_THAT(c.length(), WithinRel(ellipsePerimeter(20, 5), 1e-4));
    CHECK(kTol.equalPoints(c.startPoint(), centre + Vec2::fromAngle(ang, 20)));
    // Every sampled point lies on the ellipse to within tol.distance.
    const double ca = std::cos(ang), sa = std::sin(ang);
    double worst = 0.0;
    for (const Span& s : c.spans) {
        for (int i = 0; i <= 50; ++i) {
            const Vec2 p = pointAt(s, i / 50.0) - centre;
            const double lx = p.x * ca + p.y * sa;
            const double ly = -p.x * sa + p.y * ca;
            const double q = std::hypot(lx / 20, ly / 5);  // 1 on the ellipse
            // Radial error along the ray through p (distance to the curve is <= this).
            worst = std::max(worst, std::fabs(1.0 - 1.0 / q) * p.length());
        }
    }
    CHECK(worst <= kTol.distance);
}

TEST_CASE("ellipse: tolerance drives the span count; equal axes give the exact circle", "[shapes]") {
    Tolerance coarse;
    coarse.distance = 1.0;
    Tolerance fine;
    fine.distance = 1e-6;
    const auto a = need(buildEllipse({0, 0}, 400, 100, 0, coarse));
    const auto b = need(buildEllipse({0, 0}, 400, 100, 0, fine));
    CHECK(a.spanCount() == 4);
    CHECK(b.spanCount() > a.spanCount());
    const Contour circ = need(buildEllipse({1, 1}, 8, 8, 0.3));
    CHECK(circ.spanCount() == 1);
    CHECK(std::holds_alternative<ArcSpan>(circ.spans[0]));
    CHECK_FALSE(buildEllipse({0, 0}, 0, 5, 0).ok());
    CHECK_FALSE(buildEllipse({0, 0}, 5, -5, 0).ok());
    CHECK_FALSE(buildEllipse({0, 0}, 5, 5, std::numeric_limits<double>::infinity()).ok());
}

TEST_CASE("polygon", "[shapes]") {
    for (int n : {3, 4, 5, 6, 17}) {
        const double r = 7.5;
        const Contour c = need(buildPolygon({1, 1}, n, r, 0.3));
        requireClosedCcw(c);
        CHECK(c.spanCount() == static_cast<std::size_t>(n));
        CHECK_THAT(signedArea(c), WithinRel(0.5 * n * r * r * std::sin(kTwoPi / n), 1e-12));
        CHECK_THAT(c.length(), WithinRel(2 * n * r * std::sin(kPi / n), 1e-12));
        CHECK(kTol.equalPoints(c.startPoint(), Vec2{1, 1} + Vec2::fromAngle(0.3, r)));
    }
    CHECK_FALSE(buildPolygon({0, 0}, 2, 5, 0).ok());
    CHECK_FALSE(buildPolygon({0, 0}, 0, 5, 0).ok());
    CHECK_FALSE(buildPolygon({0, 0}, -3, 5, 0).ok());
    CHECK_FALSE(buildPolygon({0, 0}, 5, 0, 0).ok());
    CHECK_FALSE(buildPolygon({0, 0}, 5, -1, 0).ok());
}

TEST_CASE("star", "[shapes]") {
    const int n = 5;
    const double ro = 10, ri = 4;
    const Contour c = need(buildStar({2, 3}, n, ro, ri, 0.5));
    requireClosedCcw(c);
    CHECK(c.spanCount() == 10);
    // 2n triangles (centre, tip, valley) of area 0.5 * ro * ri * sin(pi/n).
    CHECK_THAT(signedArea(c), WithinRel(n * ro * ri * std::sin(kPi / n), 1e-12));
    CHECK(kTol.equalPoints(c.startPoint(), Vec2{2, 3} + Vec2::fromAngle(0.5, ro)));
    CHECK_FALSE(buildStar({0, 0}, 5, 4, 4, 0).ok());
    CHECK_FALSE(buildStar({0, 0}, 5, 4, 5, 0).ok());
    CHECK_FALSE(buildStar({0, 0}, 5, 4, 0, 0).ok());
    CHECK_FALSE(buildStar({0, 0}, 2, 4, 1, 0).ok());
    CHECK_FALSE(buildStar({0, 0}, 5, 0, 0, 0).ok());
}

TEST_CASE("arc: three points", "[shapes]") {
    const Contour ccw = need(buildArcThreePoint({5, 0}, {0, 5}, {-5, 0}));
    CHECK_FALSE(ccw.closed);
    const auto& a = std::get<ArcSpan>(ccw.spans[0]);
    CHECK(a.direction == ArcDirection::CCW);
    CHECK_THAT(a.radius, WithinAbs(5.0, 1e-12));
    CHECK(kTol.equalPoints(a.centre, {0, 0}));
    CHECK_THAT(ccw.length(), WithinRel(kPi * 5, 1e-12));
    CHECK(kTol.equalPoints(ccw.startPoint(), {5, 0}));
    CHECK(kTol.equalPoints(ccw.endPoint(), {-5, 0}));

    const Contour below = need(buildArcThreePoint({5, 0}, {0, -5}, {-5, 0}));
    CHECK(std::get<ArcSpan>(below.spans[0]).direction == ArcDirection::CW);
    const Contour cw2 = need(buildArcThreePoint({-5, 0}, {0, 5}, {5, 0}));
    CHECK(std::get<ArcSpan>(cw2.spans[0]).direction == ArcDirection::CW);
    // The middle point lies on the arc.
    const ArcSpan& a2 = std::get<ArcSpan>(cw2.spans[0]);
    CHECK(a2.containsAngle(kPi / 2));

    CHECK_FALSE(buildArcThreePoint({0, 0}, {1, 1}, {2, 2}).ok());
    CHECK_FALSE(buildArcThreePoint({0, 0}, {0, 0}, {2, 2}).ok());
    CHECK_FALSE(buildArcThreePoint({0, 0}, {1, 1}, {0, 0}).ok());
}

TEST_CASE("arc: centre, start, end", "[shapes]") {
    const Contour c = need(buildArcCentreStartEnd({1, 1}, {4, 1}, {1, 9}, ArcDirection::CCW));
    CHECK_FALSE(c.closed);
    CHECK_THAT(c.length(), WithinRel(kPi / 2 * 3, 1e-12));
    CHECK(kTol.equalPoints(c.endPoint(), {1, 4}));  // end projected on the circle
    const Contour cw = need(buildArcCentreStartEnd({1, 1}, {4, 1}, {1, 9}, ArcDirection::CW));
    CHECK_THAT(cw.length(), WithinRel(3 * 1.5 * kPi, 1e-12));  // 270 degrees the other way
    CHECK_FALSE(buildArcCentreStartEnd({1, 1}, {1, 1}, {1, 9}, ArcDirection::CCW).ok());
    CHECK_FALSE(buildArcCentreStartEnd({1, 1}, {4, 1}, {1, 1}, ArcDirection::CCW).ok());
    CHECK_FALSE(buildArcCentreStartEnd({1, 1}, {4, 1}, {9, 1}, ArcDirection::CCW).ok());
}

TEST_CASE("arc: start, end, radius picks the requested arc", "[shapes]") {
    const Vec2 s{3, 0};
    const Vec2 e{0, 3};  // chord of a quarter circle of radius 3 about the origin
    const double r = 3;
    for (ArcDirection dir : {ArcDirection::CCW, ArcDirection::CW}) {
        for (bool large : {false, true}) {
            const Contour c = need(buildArcStartEndRadius(s, e, r, dir, large));
            const auto& a = std::get<ArcSpan>(c.spans[0]);
            CHECK(a.direction == dir);
            CHECK(kTol.equalPoints(c.startPoint(), s));
            CHECK(kTol.equalPoints(c.endPoint(), e));
            CHECK_THAT(a.radius, WithinAbs(r, 1e-12));
            const double sweep = std::fabs(a.sweepAngle());
            CHECK((sweep > kPi) == large);
            // Centre is the origin for CCW minor / CW major, else the reflected centre (3,3).
            const bool origin = (dir == ArcDirection::CCW) != large;
            CHECK(kTol.equalPoints(a.centre, origin ? Vec2{0, 0} : Vec2{3, 3}));
        }
    }
    // Chord exactly the diameter gives the semicircle, either way.
    const Contour semi = need(buildArcStartEndRadius({-2, 0}, {2, 0}, 2, ArcDirection::CCW, false));
    CHECK_THAT(semi.length(), WithinRel(kPi * 2, 1e-12));
    CHECK_FALSE(buildArcStartEndRadius({0, 0}, {10, 0}, 4, ArcDirection::CCW, false).ok());
    CHECK_FALSE(buildArcStartEndRadius({0, 0}, {0, 0}, 4, ArcDirection::CCW, false).ok());
    CHECK_FALSE(buildArcStartEndRadius({0, 0}, {1, 0}, 0, ArcDirection::CCW, false).ok());
    CHECK_FALSE(buildArcStartEndRadius({0, 0}, {1, 0}, -1, ArcDirection::CCW, false).ok());
}

TEST_CASE("property: random parameters give valid, continuous, correct-area contours", "[shapes]") {
    std::mt19937 rng(12345);
    auto uni = [&](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
    for (int it = 0; it < 300; ++it) {
        const Vec2 ctr{uni(-500, 500), uni(-500, 500)};
        const double w = uni(0.5, 300), h = uni(0.5, 300), ang = uni(-7, 7);
        const double r = uni(0, 0.5) * std::min(w, h);
        {
            const Contour c = need(buildRectangle(ctr, w, h, RectCorner::Rounded, r));
            requireClosedCcw(c);
            CHECK_THAT(signedArea(c), WithinRel(w * h - (4 - kPi) * r * r, 1e-9));
            const Contour k = need(buildRectangle(ctr, w, h, RectCorner::Chamfered, r));
            requireClosedCcw(k);
            CHECK_THAT(signedArea(k), WithinRel(w * h - 2 * r * r, 1e-9));
            const Box2 b = c.bounds();
            CHECK_THAT(b.width(), WithinAbs(w, 1e-9));
            CHECK_THAT(b.height(), WithinAbs(h, 1e-9));
        }
        {
            const Contour c = need(buildEllipse(ctr, w, h, ang));
            requireClosedCcw(c);
            CHECK_THAT(signedArea(c), WithinRel(kPi * w * h / 4, 2e-3));
        }
        {
            const int n = 3 + static_cast<int>(rng() % 40);
            const double rad = uni(0.5, 200);
            const Contour c = need(buildPolygon(ctr, n, rad, ang));
            requireClosedCcw(c);
            CHECK_THAT(signedArea(c), WithinRel(0.5 * n * rad * rad * std::sin(kTwoPi / n), 1e-9));
            const double ri = uni(0.05, 0.95) * rad;
            const Contour s = need(buildStar(ctr, n, rad, ri, ang));
            requireClosedCcw(s);
            CHECK_THAT(signedArea(s), WithinRel(n * rad * ri * std::sin(kPi / n), 1e-9));
        }
        {
            const Vec2 a = ctr, b{ctr.x + uni(-50, 50), ctr.y + uni(-50, 50)}, c{ctr.x + uni(-50, 50), ctr.y + uni(-50, 50)};
            const auto r3 = buildArcThreePoint(a, b, c);
            if (r3.ok()) {
                const ArcSpan& arc = std::get<ArcSpan>(r3.value().spans[0]);
                CHECK(kTol.equalPoints(arc.startPoint(), a));
                CHECK(kTol.equalPoints(arc.endPoint(), c));
                CHECK(kTol.equalPoints(arc.centre + (b - arc.centre).normalized() * arc.radius, b));
                CHECK(arc.containsAngle((b - arc.centre).angle(), 1e-6));
            }
        }
    }
}

TEST_CASE("params glue: rebuilding after a .carv save/load is bit-identical", "[shapes][document]") {
    using namespace carvetto::document;
    Document doc;
    const std::vector<CreationParams> all{
        RectangleParams{{3.3, -1.7}, 41.25, 17.125, RectangleParams::CornerStyle::Rounded, 3.1},
        RectangleParams{{0, 0}, 10, 8, RectangleParams::CornerStyle::Chamfered, 1.0 / 3.0},
        CircleParams{{0.1, 0.2}, 7.77},
        EllipseParams{{1.0 / 7.0, 2}, 33.3, 12.1, 0.123456789},
        PolygonParams{{5, 5}, 7, 12.34, 0.2},
        StarParams{{-5, 5}, 6, 20.5, 8.25, 1.0 / 3.0},
    };
    std::vector<std::vector<Contour>> built;
    for (const CreationParams& p : all) {
        auto r = buildContours(p);
        REQUIRE(r.ok());
        built.push_back(r.value());
        doc.insertObject(Object::vector(r.value(), doc.defaultLayer(), doc.activeSheet(), p));
    }
    auto bytes = saveCarv(doc);
    REQUIRE(bytes.ok());
    auto loaded = loadCarv(std::span<const std::uint8_t>(bytes.value()));
    REQUIRE(loaded.ok());
    const Document& d2 = loaded.value().document;
    REQUIRE(d2.objectCount() == all.size());
    std::size_t i = 0;
    for (const auto& [id, obj] : d2.objects()) {
        const VectorObject& vec = *obj.asVector();
        auto again = buildContours(vec.params);
        REQUIRE(again.ok());
        CHECK(again.value() == built[i]);  // exact equality, every coordinate bit-identical
        ++i;
    }
    CHECK_FALSE(buildContours(NoParams{}).ok());
    CHECK_FALSE(buildContours(CircleParams{{0, 0}, 0}).ok());
}
