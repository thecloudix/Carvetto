#include <atomic>
#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/revolve.hpp"
#include "carvetto/relief/shape.hpp"

using namespace carvetto::relief;
using Catch::Approx;

namespace {
const double kPi = 3.14159265358979323846;

std::vector<Point> halfCircle(double r, int n) {
    std::vector<Point> p;
    for (int i = 0; i <= n; ++i) {
        const double a = kPi / 2.0 * (1.0 - static_cast<double>(i) / n);
        p.push_back({r * std::cos(a), r * std::sin(a)});
    }
    p.front().x = 0.0;
    p.back().y = 0.0;
    return p;
}

Revolve spin(std::vector<Point> prof, double cs = 0.5) {
    Revolve r;
    r.params.kind = RevolveKind::Spin;
    r.params.cellSize = cs;
    r.profile = std::move(prof);
    return r;
}
Revolve turn(std::vector<Point> prof, double cs = 0.5) {
    Revolve r = spin(std::move(prof), cs);
    r.params.kind = RevolveKind::Turn;
    return r;
}
}  // namespace

TEST_CASE("spin: half circle gives a hemisphere", "[relief][revolve]") {
    Component c;
    REQUIRE(buildRevolve(spin(halfCircle(10.0, 720)), nullptr, c) == RevolveStatus::Ok);
    const HeightField& f = c.field;
    REQUIRE(f.width() == 41);  // 2*20+1, centred on a cell
    CHECK(f.origin().x == Approx(-10.0));
    CHECK(f.at(20, 20) == Approx(10.0).margin(1e-4));
    for (std::size_t j = 0; j < f.height(); ++j)
        for (std::size_t i = 0; i < f.width(); ++i) {
            const double x = f.origin().x + static_cast<double>(i) * 0.5;
            const double y = f.origin().y + static_cast<double>(j) * 0.5;
            const double rho = std::hypot(x, y);
            if (rho > 10.0 + 1e-9) {
                CHECK(std::isnan(f.at(i, j)));
            } else {
                CHECK(f.at(i, j) == Approx(std::sqrt(std::max(100.0 - rho * rho, 0.0))).margin(2e-3));
            }
        }
    CHECK(f.at(30, 20) == Approx(std::sqrt(75.0)).margin(2e-3));  // rho = 5
    CHECK(f.at(40, 20) == Approx(0.0).margin(2e-3));             // rim
}

TEST_CASE("spin: sample values, centre and left end offset", "[relief][revolve]") {
    Revolve r = spin({{3, 4}, {5, 4}, {7, 0}}, 1.0);  // left end at x=3: radius 0..4
    r.params.center = {10, -2};
    Component c;
    REQUIRE(buildRevolve(r, nullptr, c) == RevolveStatus::Ok);
    const HeightField& f = c.field;
    CHECK(f.width() == 9);
    CHECK(f.origin().x == Approx(6.0));
    CHECK(f.origin().y == Approx(-6.0));
    CHECK(f.at(4, 4) == Approx(4.0));   // centre
    CHECK(f.at(5, 4) == Approx(4.0));   // rho 1
    CHECK(f.at(6, 4) == Approx(4.0));   // rho 2 (end of flat part)
    CHECK(f.at(7, 4) == Approx(2.0));   // rho 3: 4 -> 0 over rho 2..4
    CHECK(f.at(8, 4) == Approx(0.0));   // rho 4
    CHECK(f.at(5, 5) == Approx(4.0));   // rho sqrt2
    CHECK(std::isnan(f.at(7, 7)));  // rho 4.24 is beyond the profile
}

TEST_CASE("spin: outside the profile radius is transparent; steps take the larger value", "[relief][revolve]") {
    Revolve r = spin({{0, 1}, {2, 1}, {2, 3}, {4, 3}}, 1.0);
    Component c;
    REQUIRE(buildRevolve(r, nullptr, c) == RevolveStatus::Ok);
    const HeightField& f = c.field;
    CHECK(f.at(4, 4) == Approx(1.0));
    CHECK(f.at(5, 4) == Approx(1.0));
    CHECK(f.at(6, 4) == Approx(3.0));  // exactly at the step: larger value
    CHECK(f.at(7, 4) == Approx(3.0));
    CHECK(f.at(8, 4) == Approx(3.0));  // rho 4 = end
    CHECK(std::isnan(f.at(8, 8)));
    CHECK(std::isnan(f.at(0, 0)));
}

TEST_CASE("spin: exact height scales the analytic maximum", "[relief][revolve]") {
    Revolve r = spin(halfCircle(10.0, 90), 1.5);
    r.params.exactHeight = 25.0;
    r.params.baseHeight = 5.0;
    Component c;
    REQUIRE(buildRevolve(r, nullptr, c) == RevolveStatus::Ok);
    CHECK(c.shapeHeightPercent == Approx(200.0));  // (25-5)/10
    CHECK(c.baseHeight == Approx(5.0));
    CHECK(totalHeight(c) == Approx(25.0));
    r.params.exactHeight = 5.0;  // not above the base
    CHECK(buildRevolve(r, nullptr, c) == RevolveStatus::BadParameter);
}

TEST_CASE("turn: rectangle gives a cylinder", "[relief][revolve]") {
    Component c;
    REQUIRE(buildRevolve(turn({{0, 0}, {0, 5}, {20, 5}, {20, 0}}, 0.5), nullptr, c) == RevolveStatus::Ok);
    const HeightField& f = c.field;
    CHECK(f.width() == 41);
    CHECK(f.height() == 21);
    for (std::size_t j = 0; j < f.height(); ++j)
        for (std::size_t i = 0; i < f.width(); ++i) {
            const double x = static_cast<double>(i) * 0.5, w = static_cast<double>(j) * 0.5 - 5.0;
            if (std::fabs(w) > 5.0 + 1e-9) {
                CHECK(std::isnan(f.at(i, j)));
            } else {
                CHECK(f.at(i, j) == Approx(std::sqrt(std::max(25.0 - w * w, 0.0))).margin(1e-5));
            }
            (void)x;
        }
    CHECK(f.at(0, 10) == Approx(5.0));   // start of the part
    CHECK(f.at(40, 10) == Approx(5.0));  // end: step down to 0 takes the larger value
}

TEST_CASE("turn: a rotated axis and a tapered profile", "[relief][revolve]") {
    const double a = 30.0 * kPi / 180.0, ca = std::cos(a), sa = std::sin(a);
    auto rot = [&](double s, double w) { return Point{100.0 + s * ca - w * sa, 50.0 + s * sa + w * ca}; };
    Revolve r = turn({rot(0, 0), rot(0, 2), rot(10, 4), rot(10, 0)}, 0.25);
    Component c;
    REQUIRE(buildRevolve(r, nullptr, c) == RevolveStatus::Ok);
    const HeightField& f = c.field;
    // Probe through sample(): along the axis at s = 5 the radius is 3, so the ridge is 3.
    CHECK(f.sample(100.0 + 5 * ca, 50.0 + 5 * sa) == Approx(3.0).margin(0.02));
    const Point q = rot(5, 2);  // lateral offset 2 at radius 3
    CHECK(f.sample(q.x, q.y) == Approx(std::sqrt(5.0)).margin(0.02));
    const Point o = rot(5, 3.5);  // outside
    CHECK(std::isnan(f.sample(o.x, o.y)));
    const Point b = rot(-1, 0);  // before the start
    CHECK(std::isnan(f.sample(b.x, b.y)));
    float lo, hi;
    REQUIRE(f.minMax(lo, hi));
    CHECK(hi == Approx(4.0).margin(0.05));  // peak sits on the end edge; sampled within a cell
    CHECK(hi <= 4.0f + 1e-5f);
}

TEST_CASE("turn: exact height uses the largest radius", "[relief][revolve]") {
    Revolve r = turn({{0, 0}, {0, 5}, {20, 5}, {20, 0}}, 1.0);
    r.params.exactHeight = 10.0;
    Component c;
    REQUIRE(buildRevolve(r, nullptr, c) == RevolveStatus::Ok);
    CHECK(c.shapeHeightPercent == Approx(200.0));
    CHECK(totalHeight(c) == Approx(10.0));
}

TEST_CASE("revolve: degenerate profiles and bad input error cleanly", "[relief][revolve]") {
    Component c;
    c.name = "stale";
    CHECK(buildRevolve(spin({{0, 1}}), nullptr, c) == RevolveStatus::BadProfile);
    CHECK(c.name.empty());
    CHECK(c.field.empty());
    CHECK(buildRevolve(turn({}), nullptr, c) == RevolveStatus::BadProfile);
    CHECK(buildRevolve(turn({{0, 0}, {0, 0}}), nullptr, c) == RevolveStatus::Degenerate);
    CHECK(buildRevolve(turn({{1, 1}, {5, 5}, {3, 3}, {1, 1}}), nullptr, c) == RevolveStatus::Degenerate);
    CHECK(buildRevolve(turn({{0, 0}, {4, 0}}), nullptr, c) == RevolveStatus::Degenerate);  // on the axis
    CHECK(buildRevolve(spin({{2, 0}, {2, 5}}), nullptr, c) == RevolveStatus::Degenerate);   // no radius
    CHECK(buildRevolve(spin({{0, 1}, {5, 1}, {3, 1}}), nullptr, c) == RevolveStatus::BadProfile);  // backwards
    CHECK(buildRevolve(turn({{0, 0}, {5, 1}, {3, 1}, {10, 0}}), nullptr, c) == RevolveStatus::BadProfile);
    CHECK(buildRevolve(spin({{0, 1}, {std::nan(""), 1}}), nullptr, c) == RevolveStatus::BadProfile);
    CHECK(buildRevolve(spin({{0, 1}, {1, INFINITY}}), nullptr, c) == RevolveStatus::BadProfile);

    Revolve r = spin(halfCircle(10, 8));
    r.params.cellSize = 0.0;
    CHECK(buildRevolve(r, nullptr, c) == RevolveStatus::BadGrid);
    r.params.cellSize = 1e-6;  // too many cells
    CHECK(buildRevolve(r, nullptr, c) == RevolveStatus::BadGrid);
    r.params.cellSize = 1.0;
    r.params.baseHeight = std::nan("");
    CHECK(buildRevolve(r, nullptr, c) == RevolveStatus::BadParameter);
    r.params.baseHeight = 0.0;
    r.params.center = {INFINITY, 0};
    CHECK(buildRevolve(r, nullptr, c) == RevolveStatus::BadParameter);
    r.params.center = {};
    r.params.exactHeight = std::nan("");
    CHECK(buildRevolve(r, nullptr, c) == RevolveStatus::BadParameter);
    r.params.exactHeight.reset();
    Grid bad;
    r.params.grid = bad;  // zero size
    CHECK(buildRevolve(r, nullptr, c) == RevolveStatus::BadGrid);
    for (RevolveStatus s : {RevolveStatus::Ok, RevolveStatus::BadGrid, RevolveStatus::BadProfile, RevolveStatus::Degenerate,
                            RevolveStatus::BadParameter, RevolveStatus::Cancelled})
        CHECK(std::string(revolveStatusMessage(s)).size() > 1);
}

TEST_CASE("revolve: explicit grid, properties, cancel and determinism", "[relief][revolve]") {
    Revolve r = spin(halfCircle(10.0, 90), 1.0);
    Grid g;
    g.width = 5;
    g.height = 3;
    g.cellSize = 2.0;
    g.origin = {-4.0, -2.0};
    r.params.grid = g;
    r.params.name = "Dome";
    r.params.mode = CombineMode::MergeHigh;
    Component c;
    REQUIRE(buildRevolve(r, nullptr, c) == RevolveStatus::Ok);
    CHECK(c.name == "Dome");
    CHECK(c.mode == CombineMode::MergeHigh);
    CHECK(c.shapeHeightPercent == Approx(100.0));
    CHECK(c.field.width() == 5);
    CHECK(c.field.height() == 3);
    CHECK(c.field.at(2, 1) == Approx(10.0).margin(1e-3));

    Component d;
    REQUIRE(buildRevolve(r, nullptr, d) == RevolveStatus::Ok);
    CHECK(d.field.data().size() == c.field.data().size());
    for (std::size_t i = 0; i < d.field.data().size(); ++i) CHECK(d.field.data()[i] == c.field.data()[i]);

    std::atomic<bool> stop{true};
    CHECK(buildRevolve(r, &stop, c) == RevolveStatus::Cancelled);
    CHECK(c.field.empty());
}
