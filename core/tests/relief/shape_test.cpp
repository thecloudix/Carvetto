#include <algorithm>
#include <atomic>
#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/shape.hpp"

using namespace carvetto::relief;
using carvetto::geometry::Polyline;
using carvetto::geometry::Vec2;
using Catch::Approx;

namespace {
const double kPi = 3.14159265358979323846;

Polyline poly(std::vector<Vec2> pts) {
    Polyline p;
    p.points = std::move(pts);
    p.closed = true;
    return p;
}
Polyline rect(double x0, double y0, double x1, double y1) {
    return poly({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
}
Polyline circle(double cx, double cy, double r, int n = 720) {
    std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * kPi * i / n;
        pts.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    return poly(pts);
}
Shape shapeOf(ShapeKind k, std::vector<Polyline> v, double cs = 0.25) {
    Shape s;
    s.params.kind = k;
    s.vectors = std::move(v);
    s.cellSize = cs;
    return s;
}
Component build(const Shape& s) {
    Component c;
    const ShapeStatus st = buildShape(s, nullptr, c);
    INFO(shapeStatusMessage(st));
    REQUIRE(st == ShapeStatus::Ok);
    return c;
}
Vec2 centre(const HeightField& f, std::size_t i, std::size_t j) {
    return {f.origin().x + static_cast<double>(i) * f.cellSize(), f.origin().y + static_cast<double>(j) * f.cellSize()};
}
double rectDist(double x, double y, double x0, double y0, double x1, double y1) {
    return std::min(std::min(x - x0, x1 - x), std::min(y - y0, y1 - y));
}
double peak(const Component& c) {
    float lo, hi;
    REQUIRE(c.field.minMax(lo, hi));
    return hi;
}
}  // namespace

TEST_CASE("Angled rectangle: d*tan(angle) analytic, outside transparent", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Angled, {rect(0.1, 0.1, 20.1, 10.1)});
    s.params.angleRad = kPi / 6.0;
    const Component c = build(s);
    std::size_t inside = 0;
    for (std::size_t j = 0; j < c.field.height(); ++j)
        for (std::size_t i = 0; i < c.field.width(); ++i) {
            const Vec2 p = centre(c.field, i, j);
            const bool in = p.x > 0.1 && p.x < 20.1 && p.y > 0.1 && p.y < 10.1;
            const float v = c.field.at(i, j);
            if (!in) {
                CHECK(std::isnan(v));
                continue;
            }
            ++inside;
            CHECK(static_cast<double>(v) == Approx(rectDist(p.x, p.y, 0.1, 0.1, 20.1, 10.1) * std::tan(kPi / 6.0)).margin(1e-4));
        }
    CHECK(inside == 80u * 40u);
}

TEST_CASE("Angled limit caps the height; angle changes the slope", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Angled, {rect(0.1, 0.1, 20.1, 10.1)});
    s.params.angleRad = kPi / 4.0;
    s.params.limitEnabled = true;
    s.params.limit = 2.0;
    const Component c = build(s);
    CHECK(peak(c) == Approx(2.0).margin(1e-6));
    for (std::size_t j = 0; j < c.field.height(); ++j)
        for (std::size_t i = 0; i < c.field.width(); ++i) {
            const Vec2 p = centre(c.field, i, j);
            if (p.x < 0.1 || p.x > 20.1 || p.y < 0.1 || p.y > 10.1) continue;
            CHECK(static_cast<double>(c.field.at(i, j)) ==
                  Approx(std::min(2.0, rectDist(p.x, p.y, 0.1, 0.1, 20.1, 10.1))).margin(1e-4));
        }
    s.params.angleRad = kPi / 3.0;
    const Component c2 = build(s);
    const std::size_t i = 3, j = 20;  // a cell near the left edge
    CHECK(c2.field.at(i, j) > c.field.at(i, j));
}

TEST_CASE("Round circle gives a spherical cap", "[relief][shape]") {
    const double r = 10.0;
    for (double ang : {kPi / 2.0, kPi / 3.0}) {
        Shape s = shapeOf(ShapeKind::Round, {circle(0.0, 0.0, r)}, 0.2);
        s.grid = Grid{101, 101, 0.2, {-10.0, -10.0}};  // a cell centre sits on the circle centre
        s.params.angleRad = ang;
        const Component c = build(s);
        const double R = r / std::sin(ang);
        for (std::size_t j = 0; j < c.field.height(); ++j)
            for (std::size_t i = 0; i < c.field.width(); ++i) {
                const Vec2 p = centre(c.field, i, j);
                const double rho = std::hypot(p.x, p.y);
                const float v = c.field.at(i, j);
                if (rho > r - 0.05) continue;
                REQUIRE_FALSE(std::isnan(v));
                const double off = R * std::sin(ang) - (r - rho);  // horizontal offset from the cap axis
                const double want = std::sqrt(R * R - off * off) - R * std::cos(ang);
                CHECK(static_cast<double>(v) == Approx(want).margin(0.06));
            }
        CHECK(peak(c) == Approx(R * (1.0 - std::cos(ang))).margin(0.06));
    }
}

TEST_CASE("Smooth interpolates concave to convex", "[relief][shape]") {
    auto run = [](double sm) {
        Shape s = shapeOf(ShapeKind::Smooth, {rect(0.1, 0.1, 20.1, 10.1)});
        s.params.smoothing = sm;
        return build(s);
    };
    const Component lin = run(0.0), cvx = run(1.0), ccv = run(-1.0);
    const double dMax = peak(lin);
    CHECK(peak(cvx) == Approx(dMax).margin(1e-5));
    CHECK(peak(ccv) == Approx(dMax).margin(1e-5));
    for (std::size_t j = 0; j < lin.field.height(); ++j)
        for (std::size_t i = 0; i < lin.field.width(); ++i) {
            const Vec2 p = centre(lin.field, i, j);
            if (p.x < 0.1 || p.x > 20.1 || p.y < 0.1 || p.y > 10.1) continue;
            const double d = rectDist(p.x, p.y, 0.1, 0.1, 20.1, 10.1);
            const double u = d / dMax;
            CHECK(static_cast<double>(lin.field.at(i, j)) == Approx(d).margin(1e-4));
            CHECK(static_cast<double>(cvx.field.at(i, j)) == Approx(std::pow(u, 0.25) * dMax).margin(1e-4));
            CHECK(static_cast<double>(ccv.field.at(i, j)) == Approx(std::pow(u, 4.0) * dMax).margin(1e-4));
        }
}

TEST_CASE("Custom profile is piecewise linear in depth", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Custom, {rect(0.1, 0.1, 20.1, 10.1)});
    s.params.profile = {{0.0, 0.0}, {0.5, 1.0}, {1.0, 0.5}};
    const Component c = build(s);
    // Cell centres lie half a cell off the rectangle edges: the deepest row is 4.875 from both edges.
    const double trueMax = 4.875;
    for (std::size_t j = 0; j < c.field.height(); ++j)
        for (std::size_t i = 0; i < c.field.width(); ++i) {
            const Vec2 p = centre(c.field, i, j);
            if (p.x < 0.1 || p.x > 20.1 || p.y < 0.1 || p.y > 10.1) continue;
            const double d = rectDist(p.x, p.y, 0.1, 0.1, 20.1, 10.1);
            const double u = std::min(1.0, d / trueMax);
            const double f = u <= 0.5 ? 2.0 * u : 1.0 - (u - 0.5);
            CHECK(static_cast<double>(c.field.at(i, j)) == Approx(f * trueMax).margin(0.03));
        }
}

TEST_CASE("Holes are transparent and lower the surface near them", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Angled, {rect(0.1, 0.1, 30.1, 30.1), rect(10.1, 10.1, 16.1, 16.1)});
    s.params.angleRad = kPi / 4.0;
    const Component c = build(s);
    for (std::size_t j = 0; j < c.field.height(); ++j)
        for (std::size_t i = 0; i < c.field.width(); ++i) {
            const Vec2 p = centre(c.field, i, j);
            const bool outer = p.x > 0.1 && p.x < 30.1 && p.y > 0.1 && p.y < 30.1;
            const bool hole = p.x > 10.1 && p.x < 16.1 && p.y > 10.1 && p.y < 16.1;
            const float v = c.field.at(i, j);
            if (!outer || hole) {
                CHECK(std::isnan(v));
                continue;
            }
            const double dOuter = rectDist(p.x, p.y, 0.1, 0.1, 30.1, 30.1);
            const double dx = std::max({10.1 - p.x, 0.0, p.x - 16.1});
            const double dy = std::max({10.1 - p.y, 0.0, p.y - 16.1});
            CHECK(static_cast<double>(v) == Approx(std::min(dOuter, std::hypot(dx, dy))).margin(1e-4));
        }
}

TEST_CASE("Island inside a hole is a shape again", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Angled,
                      {rect(0.1, 0.1, 30.1, 30.1), rect(5.1, 5.1, 25.1, 25.1), rect(12.1, 12.1, 18.1, 18.1)});
    s.params.angleRad = kPi / 4.0;
    const Component c = build(s);
    // Island centre cell (15.1 is not a centre; use the cell nearest): distance 3 to its edge.
    const std::size_t i = static_cast<std::size_t>(std::lround((15.1 - c.field.origin().x) / 0.25));
    const std::size_t j = i;
    const Vec2 p = centre(c.field, i, j);
    CHECK(static_cast<double>(c.field.at(i, j)) == Approx(rectDist(p.x, p.y, 12.1, 12.1, 18.1, 18.1)).margin(1e-4));
    // A cell in the ring between the hole and the island is transparent.
    const std::size_t k = static_cast<std::size_t>(std::lround((8.0 - c.field.origin().x) / 0.25));
    CHECK(std::isnan(c.field.at(k, k)));
}

TEST_CASE("Blend to inner vectors ignores holes as edges (Smooth/Custom only)", "[relief][shape]") {
    const std::vector<Polyline> v = {rect(0.1, 0.1, 30.1, 30.1), rect(10.1, 10.1, 16.1, 16.1)};
    Shape s = shapeOf(ShapeKind::Smooth, v);
    const Component plain = build(s);
    s.params.blendToInnerVectors = true;
    const Component blend = build(s);
    const std::size_t i = static_cast<std::size_t>(std::lround((8.0 - plain.field.origin().x) / 0.25));
    const std::size_t j = static_cast<std::size_t>(std::lround((13.0 - plain.field.origin().y) / 0.25));
    const Vec2 p = centre(plain.field, i, j);
    const double blendWant = rectDist(p.x, p.y, 0.1, 0.1, 30.1, 30.1);  // linear: height = distance to the outer edge
    CHECK(static_cast<double>(blend.field.at(i, j)) ==
          Approx(blendWant).margin(1e-3));
    CHECK(plain.field.at(i, j) < blend.field.at(i, j));
    const std::size_t hi = static_cast<std::size_t>(std::lround((13.0 - plain.field.origin().x) / 0.25));
    CHECK(std::isnan(blend.field.at(hi, j)));  // hole cells stay transparent
    // Angled ignores the flag.
    Shape a = shapeOf(ShapeKind::Angled, v);
    const Component a0 = build(a);
    a.params.blendToInnerVectors = true;
    const Component a1 = build(a);
    CHECK(a0.field.data().size() == a1.field.data().size());
    for (std::size_t k = 0; k < a0.field.data().size(); ++k) {
        const float x = a0.field.data()[k], y = a1.field.data()[k];
        CHECK(((std::isnan(x) && std::isnan(y)) || x == y));
    }
}

TEST_CASE("Preserve internal corners mitres reflex vertices only", "[relief][shape]") {
    const Polyline L = poly({{-10, -10}, {10, -10}, {10, 0}, {0, 0}, {0, 10}, {-10, 10}});
    Shape s = shapeOf(ShapeKind::Angled, {L}, 0.3);
    s.params.angleRad = kPi / 4.0;
    const Component round = build(s);
    s.params.preserveInternalCorners = true;
    const Component sharp = build(s);
    const std::size_t i = static_cast<std::size_t>(std::lround((-0.25 - round.field.origin().x) / 0.3));
    const Vec2 p = centre(round.field, i, i);
    REQUIRE(p.x == Approx(-0.25).margin(1e-9));
    CHECK(static_cast<double>(round.field.at(i, i)) == Approx(std::hypot(0.25, 0.25)).margin(1e-4));
    CHECK(static_cast<double>(sharp.field.at(i, i)) == Approx(0.25).margin(1e-4));
    for (std::size_t j = 0; j < round.field.height(); ++j)
        for (std::size_t k = 0; k < round.field.width(); ++k) {
            const Vec2 q = centre(round.field, k, j);
            const float a = round.field.at(k, j), b = sharp.field.at(k, j);
            if (std::isnan(a)) { CHECK(std::isnan(b)); continue; }
            CHECK(b <= a + 1e-6f);  // the mitre is never higher than the round value
            if (q.x > 0.0 || q.y > 0.0) {
                CHECK(a == b);  // identical outside the quadrant that the reflex vertex owns
            } else {
                // Inside it the mitre is the sharper of: the outer walls, and max(|x|,|y|).
                const double walls = std::min(q.x + 10.0, q.y + 10.0);
                CHECK(static_cast<double>(b) == Approx(std::min(walls, std::max(-q.x, -q.y))).margin(1e-4));
            }
        }
    // Convex-only shape: no change at all.
    Shape r = shapeOf(ShapeKind::Angled, {rect(0.1, 0.1, 10.1, 6.1)});
    const Component r0 = build(r);
    r.params.preserveInternalCorners = true;
    const Component r1 = build(r);
    for (std::size_t k = 0; k < r0.field.data().size(); ++k) {
        const float x = r0.field.data()[k], y = r1.field.data()[k];
        CHECK(((std::isnan(x) && std::isnan(y)) || x == y));
    }
}

TEST_CASE("Preserve internal corners mitres the corners of a hole", "[relief][shape]") {
    // Hole corners are reflex for the material, whatever the polygon orientation.
    for (bool reverseHole : {false, true}) {
        Shape s = shapeOf(ShapeKind::Angled, {rect(0.1, 0.1, 30.1, 30.1), rect(10.1, 10.1, 16.1, 16.1)});
        if (reverseHole) std::reverse(s.vectors[1].points.begin(), s.vectors[1].points.end());
        s.params.angleRad = kPi / 4.0;
        const Component round = build(s);
        s.params.preserveInternalCorners = true;
        const Component sharp = build(s);
        std::size_t differing = 0;
        for (std::size_t j = 0; j < round.field.height(); ++j)
            for (std::size_t i = 0; i < round.field.width(); ++i) {
                const Vec2 p = centre(round.field, i, j);
                const float a = round.field.at(i, j), b = sharp.field.at(i, j);
                if (std::isnan(a)) { CHECK(std::isnan(b)); continue; }
                const double dOuter = rectDist(p.x, p.y, 0.1, 0.1, 30.1, 30.1);
                const double dx = std::max({10.1 - p.x, 0.0, p.x - 16.1});
                const double dy = std::max({10.1 - p.y, 0.0, p.y - 16.1});
                // Round: hole distance is Euclidean; sharp: Chebyshev (square offset of a square hole).
                CHECK(static_cast<double>(a) == Approx(std::min(dOuter, std::hypot(dx, dy))).margin(1e-4));
                CHECK(static_cast<double>(b) == Approx(std::min(dOuter, std::max(dx, dy))).margin(1e-4));
                if (a != b) ++differing;
            }
        CHECK(differing > 100u);
    }
}

TEST_CASE("Polygon orientation does not matter", "[relief][shape]") {
    Shape a = shapeOf(ShapeKind::Angled, {rect(0.1, 0.1, 12.1, 8.1), rect(4.1, 3.1, 7.1, 5.1)});
    Shape b = a;
    std::reverse(b.vectors[0].points.begin(), b.vectors[0].points.end());
    const Component ca = build(a), cb = build(b);
    REQUIRE(ca.field.data().size() == cb.field.data().size());
    for (std::size_t k = 0; k < ca.field.data().size(); ++k) {
        const float x = ca.field.data()[k], y = cb.field.data()[k];
        CHECK(((std::isnan(x) && std::isnan(y)) || x == y));
    }
}

TEST_CASE("Plane: zero height inside, dynamic properties carried", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Plane, {rect(0.1, 0.1, 10.1, 6.1), rect(3.1, 2.1, 5.1, 4.1)});
    s.params.baseHeight = 1.5;
    s.params.name = "flat";
    s.params.mode = CombineMode::MergeHigh;
    s.params.fade = Fade{true, {0, 0}, {1, 0}, 50.0};
    s.params.tilt = Tilt{true, {0, 0}, {1, 0}, 0.1};
    const Component c = build(s);
    CHECK(c.name == "flat");
    CHECK(c.mode == CombineMode::MergeHigh);
    CHECK(c.baseHeight == 1.5);
    CHECK(c.fade.enabled);
    CHECK(c.fade.percent == 50.0);
    CHECK(c.tilt.angleRad == 0.1);
    for (std::size_t j = 0; j < c.field.height(); ++j)
        for (std::size_t i = 0; i < c.field.width(); ++i) {
            const Vec2 p = centre(c.field, i, j);
            const bool outer = p.x > 0.1 && p.x < 10.1 && p.y > 0.1 && p.y < 6.1;
            const bool hole = p.x > 3.1 && p.x < 5.1 && p.y > 2.1 && p.y < 4.1;
            if (outer && !hole) CHECK(c.field.at(i, j) == 0.0f); else CHECK(std::isnan(c.field.at(i, j)));
        }
}

TEST_CASE("Plane without vectors fills an explicit grid", "[relief][shape]") {
    Shape s;
    s.params.kind = ShapeKind::Plane;
    s.grid = Grid{7, 5, 2.0, {1.0, 1.0}};
    const Component c = build(s);
    CHECK(c.field.width() == 7u);
    for (float v : c.field.data()) CHECK(v == 0.0f);
    s.params.kind = ShapeKind::Angled;  // other kinds need vectors
    Component o;
    CHECK(buildShape(s, nullptr, o) == ShapeStatus::NoVectors);
}

TEST_CASE("Explicit grid smaller than the shape: distance is to the vector", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Angled, {rect(-100.0, -100.0, 100.0, 100.0)});
    s.grid = Grid{20, 20, 1.0, {0.0, 0.0}};
    s.params.angleRad = kPi / 4.0;
    const Component c = build(s);
    CHECK(c.field.at(0, 0) == Approx(100.0f));
    CHECK(c.field.at(10, 5) == Approx(90.0f));
}

TEST_CASE("Total height edits update scale and base consistently", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Angled, {rect(0.1, 0.1, 20.1, 10.1)});
    s.params.angleRad = kPi / 4.0;
    s.params.baseHeight = 1.0;
    s.params.scalePercent = 50.0;
    Component c = build(s);
    const double raw = peak(c);
    CHECK(totalHeight(c) == Approx(1.0 + raw * 0.5));
    // Scale mode: base kept.
    REQUIRE(setTotalHeight(c, 4.0, TotalHeightAdjust::Scale) == ShapeStatus::Ok);
    CHECK(c.baseHeight == 1.0);
    CHECK(c.shapeHeightPercent == Approx(300.0 / raw));
    CHECK(totalHeight(c) == Approx(4.0));
    // Base mode: scale kept.
    const double pct = c.shapeHeightPercent;
    REQUIRE(setTotalHeight(c, 7.0, TotalHeightAdjust::Base) == ShapeStatus::Ok);
    CHECK(c.shapeHeightPercent == pct);
    CHECK(c.baseHeight == Approx(7.0 - raw * pct / 100.0));
    // The composed top of the model really reaches the total.
    HeightField out;
    REQUIRE(composeComponent(c, c.field.grid(), out) == Status::Ok);
    float lo, hi;
    REQUIRE(out.minMax(lo, hi));
    CHECK(static_cast<double>(hi) == Approx(7.0).margin(1e-4));
    // Errors.
    double sp = 100.0, bh = 2.0;
    CHECK(setTotalHeight(0.0, 5.0, TotalHeightAdjust::Scale, sp, bh) == ShapeStatus::BadParameter);
    CHECK(setTotalHeight(3.0, 2.0, TotalHeightAdjust::Scale, sp, bh) == ShapeStatus::BadParameter);
    CHECK(setTotalHeight(3.0, 1.0, TotalHeightAdjust::Scale, sp, bh) == ShapeStatus::BadParameter);
    CHECK(setTotalHeight(NAN, 5.0, TotalHeightAdjust::Base, sp, bh) == ShapeStatus::BadParameter);
    CHECK(sp == 100.0);
    CHECK(bh == 2.0);
    CHECK(setTotalHeight(3.0, 5.0, TotalHeightAdjust::Scale, sp, bh) == ShapeStatus::Ok);
    CHECK(sp == Approx(100.0));
    Component empty;
    CHECK(setTotalHeight(empty, 5.0, TotalHeightAdjust::Base) == ShapeStatus::BadParameter);
}

TEST_CASE("Invalid input returns errors", "[relief][shape]") {
    Component o;
    auto st = [&](Shape s) { return buildShape(s, nullptr, o); };
    const Shape ok = shapeOf(ShapeKind::Angled, {rect(0, 0, 10, 10)});
    CHECK(st(ok) == ShapeStatus::Ok);
    { Shape s = ok; s.vectors[0].closed = false; CHECK(st(s) == ShapeStatus::BadVector); }
    { Shape s = ok; s.vectors[0].points.resize(2); CHECK(st(s) == ShapeStatus::BadVector); }
    { Shape s = ok; s.vectors[0].points[1].x = NAN; CHECK(st(s) == ShapeStatus::BadVector); }
    { Shape s = ok; s.vectors[0].points = {{0, 0}, {0, 0}, {5, 5}, {5, 5}}; CHECK(st(s) == ShapeStatus::BadVector); }
    { Shape s = ok; s.vectors.clear(); CHECK(st(s) == ShapeStatus::NoVectors); }
    { Shape s = ok; s.cellSize = 0.0; CHECK(st(s) == ShapeStatus::BadGrid); }
    { Shape s = ok; s.cellSize = NAN; CHECK(st(s) == ShapeStatus::BadGrid); }
    { Shape s = ok; s.cellSize = 1e-9; CHECK(st(s) == ShapeStatus::BadGrid); }
    { Shape s = ok; s.grid = Grid{0, 4, 1.0, {0, 0}}; CHECK(st(s) == ShapeStatus::BadGrid); }
    { Shape s = ok; s.params.angleRad = 0.0; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.angleRad = kPi / 2.0; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.angleRad = NAN; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.limitEnabled = true; s.params.limit = 0.0; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.scalePercent = 0.0; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.baseHeight = INFINITY; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.kind = ShapeKind::Round; s.params.angleRad = kPi / 2.0; CHECK(st(s) == ShapeStatus::Ok); }
    { Shape s = ok; s.params.kind = ShapeKind::Round; s.params.angleRad = kPi; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.kind = ShapeKind::Smooth; s.params.smoothing = 1.5; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.kind = ShapeKind::Smooth; s.params.smoothing = -1.0; CHECK(st(s) == ShapeStatus::Ok); }
    { Shape s = ok; s.params.kind = ShapeKind::Custom; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.kind = ShapeKind::Custom; s.params.profile = {{0.0, 0.0}, {0.0, 1.0}}; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.kind = ShapeKind::Custom; s.params.profile = {{0.0, 0.0}, {1.5, 1.0}}; CHECK(st(s) == ShapeStatus::BadParameter); }
    { Shape s = ok; s.params.kind = ShapeKind::Custom; s.params.profile = {{0.0, 0.0}, {1.0, 1.0}}; CHECK(st(s) == ShapeStatus::Ok); }
    { Shape s = shapeOf(ShapeKind::Angled, {rect(0, 0, 0.01, 0.01)}, 1.0); CHECK(st(s) == ShapeStatus::NoVectors); }
    CHECK(std::string(shapeStatusMessage(ShapeStatus::Cancelled)) == "cancelled");
}

TEST_CASE("Cancel, determinism and re-editing", "[relief][shape]") {
    Shape s = shapeOf(ShapeKind::Round, {circle(0, 0, 8.0, 90)}, 0.2);
    std::atomic<bool> stop{true};
    Component o;
    CHECK(buildShape(s, &stop, o) == ShapeStatus::Cancelled);
    CHECK(o.field.empty());
    stop = false;
    Component a, b;
    CHECK(buildShape(s, &stop, a) == ShapeStatus::Ok);
    CHECK(buildShape(s, nullptr, b) == ShapeStatus::Ok);
    REQUIRE(a.field.data().size() == b.field.data().size());
    CHECK(std::equal(a.field.data().begin(), a.field.data().end(), b.field.data().begin(),
                     [](float x, float y) { return (std::isnan(x) && std::isnan(y)) || x == y; }));
    // Re-edit: the stored shape regenerates with changed parameters.
    s.params.angleRad = kPi / 2.0;
    Component c;
    CHECK(buildShape(s, nullptr, c) == ShapeStatus::Ok);
    CHECK(peak(c) > peak(a));
}
