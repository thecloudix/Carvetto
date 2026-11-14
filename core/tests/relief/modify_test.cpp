#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/modify.hpp"

using namespace carvetto::relief;
using carvetto::geometry::Polyline;
using carvetto::geometry::Vec2;
using Catch::Approx;

namespace {
HeightField make(std::size_t w, std::size_t h, double cs, float fill, Point o = {0, 0}) {
    HeightField f;
    REQUIRE(HeightField::create(Grid{w, h, cs, o}, fill, f) == Status::Ok);
    return f;
}
Polyline rect(double x0, double y0, double x1, double y1, bool closed = true) {
    Polyline p;
    p.points = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
    p.closed = closed;
    return p;
}
bool same(float a, float b) { return (std::isnan(a) && std::isnan(b)) || a == b; }
}  // namespace

TEST_CASE("bake: modes, properties and union grid", "[relief][modify]") {
    Component a;
    a.name = "A";
    a.field = make(3, 3, 1.0, 2.0f);
    a.shapeHeightPercent = 50.0;  // 1
    a.baseHeight = 0.5;           // 1.5
    Component b;
    b.field = make(3, 3, 1.0, 4.0f, {2, 0});  // overlaps column 2, extends to x = 4
    b.mode = CombineMode::Subtract;
    std::vector<Component> sel{a, b};
    Component out;
    REQUIRE(bakeComponents(sel, out) == Status::Ok);
    CHECK(out.name == "A");
    CHECK(out.mode == CombineMode::Add);
    CHECK(out.shapeHeightPercent == 100.0);
    CHECK(out.baseHeight == 0.0);
    CHECK_FALSE(out.fade.enabled);
    CHECK_FALSE(out.tilt.enabled);
    REQUIRE(out.field.width() == 5);
    REQUIRE(out.field.height() == 3);
    CHECK(out.field.at(0, 1) == 1.5f);
    CHECK(out.field.at(1, 1) == 1.5f);
    CHECK(out.field.at(2, 1) == Approx(1.5 - 4.0));  // overlap: subtract
    CHECK(std::isnan(out.field.at(3, 1)));           // only b, subtract on transparent stays transparent
    CHECK(std::isnan(out.field.at(4, 1)));
    CHECK(sel[0].field.at(0, 0) == 2.0f);  // originals untouched
    CHECK(sel[1].field.at(0, 0) == 4.0f);
}

TEST_CASE("bake equals composeComponent with tilt; empty selection fails", "[relief][modify]") {
    Component a;
    a.field = make(4, 2, 1.0, 1.0f);
    a.tilt = Tilt{true, {0, 0}, {1, 0}, 0.3};
    Component out;
    REQUIRE(bakeComponents({a}, out) == Status::Ok);
    HeightField ref;
    REQUIRE(bake(a, ref) == Status::Ok);
    for (std::size_t i = 0; i < 4; ++i) CHECK(out.field.at(i, 0) == ref.at(i, 0));
    CHECK(out.field.at(3, 0) == Approx(1.0 + 3.0 * std::tan(0.3)));
    CHECK(bakeComponents({}, out) != Status::Ok);
    Component hidden = a;
    hidden.visible = false;
    CHECK(bakeComponents({hidden}, out) != Status::Ok);
}

TEST_CASE("componentFromField", "[relief][modify]") {
    Component like;
    like.name = "n";
    like.mode = CombineMode::Multiply;
    like.shapeHeightPercent = 10;
    const Component c = componentFromField(like, make(2, 2, 1.0, 3.0f));
    CHECK(c.name == "n");
    CHECK(c.mode == CombineMode::Add);
    CHECK(c.shapeHeightPercent == 100.0);
    CHECK(c.field.at(1, 1) == 3.0f);
}

TEST_CASE("replace below", "[relief][modify]") {
    HeightField f;
    REQUIRE(HeightField::fromData(Grid{5, 1, 1.0, {0, 0}}, {-1.0f, 0.0f, 1.0f, kTransparent, 3.0f}, f) == Status::Ok);
    HeightField o;
    ReplaceBelowParams p;
    p.threshold = 1.0;
    SECTION("transparent: strictly below only") {
        REQUIRE(replaceBelow(f, p, o) == Status::Ok);
        CHECK(std::isnan(o.at(0, 0)));
        CHECK(std::isnan(o.at(1, 0)));
        CHECK(o.at(2, 0) == 1.0f);  // equal to threshold is kept
        CHECK(std::isnan(o.at(3, 0)));
        CHECK(o.at(4, 0) == 3.0f);
    }
    SECTION("plane") {
        p.with = ReplaceWith::Plane;
        p.planeHeight = 0.25;
        REQUIRE(replaceBelow(f, p, o) == Status::Ok);
        CHECK(o.at(0, 0) == 0.25f);
        CHECK(o.at(1, 0) == 0.25f);
        CHECK(o.at(2, 0) == 1.0f);
        CHECK(std::isnan(o.at(3, 0)));  // transparent never becomes a plane
    }
    SECTION("lower by value applies to kept and plane cells") {
        p.with = ReplaceWith::Plane;
        p.planeHeight = 0.5;
        p.lowerBy = 2.0;
        REQUIRE(replaceBelow(f, p, o) == Status::Ok);
        CHECK(o.at(0, 0) == -1.5f);
        CHECK(o.at(2, 0) == -1.0f);
        CHECK(o.at(4, 0) == 1.0f);
    }
    SECTION("lower by without replacement matters when transparent") {
        p.lowerBy = 1.0;
        REQUIRE(replaceBelow(f, p, o) == Status::Ok);
        CHECK(o.at(2, 0) == 0.0f);
        CHECK(o.at(4, 0) == 2.0f);
        CHECK(std::isnan(o.at(0, 0)));
    }
    SECTION("input untouched, bad params") {
        REQUIRE(replaceBelow(f, p, o) == Status::Ok);
        CHECK(f.at(0, 0) == -1.0f);
        p.threshold = INFINITY;
        CHECK(replaceBelow(f, p, o) == Status::BadParameter);
        CHECK(o.empty());
        p.threshold = 0;
        p.lowerBy = NAN;
        CHECK(replaceBelow(f, p, o) == Status::BadParameter);
        p.lowerBy = 0;
        p.with = ReplaceWith::Plane;
        p.planeHeight = NAN;
        CHECK(replaceBelow(f, p, o) == Status::BadParameter);
        CHECK(replaceBelow(HeightField{}, ReplaceBelowParams{}, o) == Status::ZeroSize);
    }
}

TEST_CASE("reset inside / outside vectors", "[relief][modify]") {
    const HeightField f = make(6, 6, 1.0, 1.0f);  // centres 0..5
    HeightField o;
    SECTION("inside a rectangle; the bottom edge belongs inside, the top edge outside") {
        REQUIRE(resetByVectors(f, {rect(1.5, 1.0, 4.5, 4.0)}, ResetRegion::Inside, o) == Status::Ok);
        int cleared = 0;
        for (std::size_t j = 0; j < 6; ++j) {
            for (std::size_t i = 0; i < 6; ++i) {
                const bool in = i >= 2 && i <= 4 && j >= 1 && j <= 3;
                CHECK(std::isnan(o.at(i, j)) == in);
                cleared += in;
            }
        }
        CHECK(cleared == 9);
    }
    SECTION("outside is the exact complement") {
        const std::vector<Polyline> v{rect(1.5, 1.0, 4.5, 4.0)};
        HeightField in, out;
        REQUIRE(resetByVectors(f, v, ResetRegion::Inside, in) == Status::Ok);
        REQUIRE(resetByVectors(f, v, ResetRegion::Outside, out) == Status::Ok);
        for (std::size_t k = 0; k < 36; ++k) CHECK(std::isnan(in.data()[k]) != std::isnan(out.data()[k]));
    }
    SECTION("even-odd: a hole is outside") {
        REQUIRE(resetByVectors(f, {rect(-1, -1, 6, 6), rect(1.5, 1.5, 3.5, 3.5)}, ResetRegion::Inside, o) == Status::Ok);
        CHECK(std::isnan(o.at(0, 0)));
        CHECK(o.at(2, 2) == 1.0f);
        CHECK(o.at(3, 3) == 1.0f);
        CHECK(std::isnan(o.at(4, 4)));
    }
    SECTION("keeps already transparent cells and values") {
        HeightField g = f;
        g.set(0, 0, kTransparent);
        g.set(5, 5, 7.0f);
        REQUIRE(resetByVectors(g, {rect(1.5, 1.5, 3.5, 3.5)}, ResetRegion::Inside, o) == Status::Ok);
        CHECK(std::isnan(o.at(0, 0)));
        CHECK(o.at(5, 5) == 7.0f);
    }
    SECTION("no vectors: inside clears nothing, outside clears everything") {
        REQUIRE(resetByVectors(f, {}, ResetRegion::Inside, o) == Status::Ok);
        CHECK(same(o.at(3, 3), 1.0f));
        REQUIRE(resetByVectors(f, {}, ResetRegion::Outside, o) == Status::Ok);
        CHECK(std::isnan(o.at(3, 3)));
    }
    SECTION("bad vectors") {
        CHECK(resetByVectors(f, {rect(0, 0, 1, 1, false)}, ResetRegion::Inside, o) == Status::BadParameter);
        Polyline two;
        two.points = {{0, 0}, {1, 1}};
        two.closed = true;
        CHECK(resetByVectors(f, {two}, ResetRegion::Inside, o) == Status::BadParameter);
        Polyline nan = rect(0, 0, 1, 1);
        nan.points[2].x = NAN;
        CHECK(resetByVectors(f, {nan}, ResetRegion::Inside, o) == Status::BadParameter);
        Polyline big;
        big.closed = true;
        for (int k = 0; k < 40000; ++k) big.points.push_back({static_cast<double>(k % 2), static_cast<double>(k) * 1e-3});
        CHECK(resetByVectors(f, {big, big}, ResetRegion::Inside, o) == Status::BadParameter);
        CHECK(resetByVectors(HeightField{}, {}, ResetRegion::Inside, o) == Status::ZeroSize);
    }
}

TEST_CASE("offset model: analytic cases", "[relief][modify]") {
    HeightField o;
    OffsetParams p;
    SECTION("flat plane moves by d, either sign") {
        const HeightField f = make(9, 9, 0.5, 2.0f);
        p.distance = 1.0;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        for (float v : o.data()) CHECK(v == Approx(3.0));
        p.distance = -1.0;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        for (float v : o.data()) CHECK(v == Approx(1.0));
    }
    SECTION("zero distance is the identity") {
        HeightField f = make(4, 4, 1.0, 1.0f);
        f.set(1, 1, kTransparent);
        p.distance = 0.0;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        for (std::size_t k = 0; k < 16; ++k) CHECK(same(o.data()[k], f.data()[k]));
    }
    SECTION("single spike dilates to a ball cap") {
        HeightField f = make(11, 11, 1.0, 0.0f);
        f.set(5, 5, 10.0f);
        p.distance = 3.0;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        CHECK(o.at(5, 5) == Approx(13.0));
        CHECK(o.at(7, 5) == Approx(10.0 + std::sqrt(9.0 - 4.0)));  // 2 cells away
        CHECK(o.at(8, 5) == Approx(10.0));                        // exactly on the ball rim
        CHECK(o.at(9, 5) == Approx(3.0));                         // out of reach: the plane itself, +d
        CHECK(o.at(5, 5 + 3) == Approx(10.0));
        CHECK(o.at(7, 7) == Approx(10.0 + std::sqrt(9.0 - 8.0)));
    }
    SECTION("single spike erodes away: the plane drops by d, the spike is cut to its neighbours") {
        HeightField f = make(11, 11, 1.0, 0.0f);
        f.set(5, 5, 10.0f);
        p.distance = -3.0;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        CHECK(o.at(5, 5) == Approx(-std::sqrt(8.0)));  // lowest candidate: a 0-cell one step away, lift sqrt(9-1)
        CHECK(o.at(0, 0) == Approx(-3.0));
    }
    SECTION("clip to zero plane") {
        const HeightField f = make(5, 5, 1.0, 1.0f);
        p.distance = -2.0;
        p.clipToZero = true;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        for (float v : o.data()) CHECK(v == 0.0f);
        p.clipToZero = false;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        for (float v : o.data()) CHECK(v == Approx(-1.0));
        p.distance = 2.0;
        p.clipToZero = true;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        for (float v : o.data()) CHECK(v == Approx(3.0));
    }
    SECTION("transparent cells stay transparent and are ignored") {
        HeightField f = make(5, 1, 1.0, 1.0f);
        f.set(2, 0, kTransparent);
        p.distance = 1.0;
        REQUIRE(offsetModel(f, p, o) == Status::Ok);
        CHECK(std::isnan(o.at(2, 0)));
        CHECK(o.at(1, 0) == Approx(2.0));
    }
    SECTION("bad parameters and limits") {
        const HeightField f = make(4, 4, 1.0, 1.0f);
        p.distance = NAN;
        CHECK(offsetModel(f, p, o) == Status::BadParameter);
        p.distance = 1000.0;
        CHECK(offsetModel(f, p, o) == Status::BadParameter);
        const HeightField big = make(4096, 4096, 0.01, 0.0f);
        p.distance = 2.0;  // 200 cells -> 16M * 160k taps
        CHECK(offsetModel(big, p, o) == Status::TooLarge);
        CHECK(offsetModel(HeightField{}, OffsetParams{}, o) == Status::ZeroSize);
    }
}

TEST_CASE("offset by d then -d returns the original on a smooth field", "[relief][modify]") {
    // Spherical cap radius 20 mm: convex, curvature radius far above d, so closing recovers it.
    const double R = 20.0, cs = 0.25;
    const std::size_t n = 121;  // 30 mm square
    HeightField f = make(n, n, cs, 0.0f);
    for (std::size_t j = 0; j < n; ++j) {
        for (std::size_t i = 0; i < n; ++i) {
            const double x = (static_cast<double>(i) - 60.0) * cs, y = (static_cast<double>(j) - 60.0) * cs;
            f.set(i, j, static_cast<float>(std::sqrt(R * R - x * x - y * y)));
        }
    }
    HeightField up, back;
    REQUIRE(offsetModel(f, OffsetParams{2.0, false}, up) == Status::Ok);
    REQUIRE(up.at(60, 60) == Approx(R + 2.0).margin(1e-4));
    REQUIRE(offsetModel(up, OffsetParams{-2.0, false}, back) == Status::Ok);
    // Compare away from the border (erosion near the grid edge sees fewer neighbours).
    double worst = 0;
    for (std::size_t j = 12; j < n - 12; ++j) {
        for (std::size_t i = 12; i < n - 12; ++i) {
            worst = std::max(worst, std::fabs(static_cast<double>(back.at(i, j)) - static_cast<double>(f.at(i, j))));
        }
    }
    CHECK(worst < 0.5 * cs * cs / 1.0 + 0.05);  // stated tolerance: 0.05 mm + discretisation of the ball
    // And the other order (opening) on the same convex field.
    HeightField down, back2;
    REQUIRE(offsetModel(f, OffsetParams{-2.0, false}, down) == Status::Ok);
    REQUIRE(offsetModel(down, OffsetParams{2.0, false}, back2) == Status::Ok);
    worst = 0;
    for (std::size_t j = 12; j < n - 12; ++j) {
        for (std::size_t i = 12; i < n - 12; ++i) {
            worst = std::max(worst, std::fabs(static_cast<double>(back2.at(i, j)) - static_cast<double>(f.at(i, j))));
        }
    }
    CHECK(worst < 0.05);
}

TEST_CASE("smooth", "[relief][modify]") {
    HeightField o;
    SmoothParams p;
    p.radiusMm = 2.0;
    SECTION("zero percent is the identity, 100 percent of a constant is the constant") {
        HeightField f = make(7, 7, 1.0, 3.0f);
        f.set(3, 3, kTransparent);
        REQUIRE(smooth(f, p, o) == Status::Ok);
        for (std::size_t k = 0; k < 49; ++k) CHECK(same(o.data()[k], f.data()[k]));
        p.percent = 100.0;
        REQUIRE(smooth(f, p, o) == Status::Ok);
        for (std::size_t k = 0; k < 49; ++k) {
            if (k == 24) CHECK(std::isnan(o.data()[k]));
            else CHECK(o.data()[k] == Approx(3.0));
        }
    }
    SECTION("impulse reproduces the Gaussian weights and conserves mass away from the border") {
        HeightField f = make(31, 31, 1.0, 0.0f);
        f.set(15, 15, 1.0f);
        p.percent = 50.0;  // sigma = 1 mm = 1 cell
        REQUIRE(smooth(f, p, o) == Status::Ok);
        double sum = 0;
        for (float v : o.data()) sum += v;
        // Normalised convolution: value = w(c) / sum of weights in the window, window full => w / 2piSigma^2 form.
        const double k0 = 1.0, k1 = std::exp(-0.5), k2 = std::exp(-2.0), k3 = std::exp(-4.5);
        const double norm = (k0 + 2 * (k1 + k2 + k3)) * (k0 + 2 * (k1 + k2 + k3));
        CHECK(o.at(15, 15) == Approx(1.0 / norm).epsilon(1e-6));
        CHECK(o.at(16, 15) == Approx(k1 / norm).epsilon(1e-6));
        CHECK(o.at(17, 16) == Approx(k2 * k1 / norm).epsilon(1e-6));
        CHECK(o.at(18, 15) == Approx(k3 / norm).epsilon(1e-6));
        CHECK(o.at(19, 15) == 0.0f);  // beyond 3 sigma
        CHECK(sum == Approx(1.0).epsilon(1e-5));
    }
    SECTION("linear ramp is preserved in the interior") {
        HeightField f = make(41, 5, 0.5, 0.0f);
        for (std::size_t j = 0; j < 5; ++j) for (std::size_t i = 0; i < 41; ++i) f.set(i, j, static_cast<float>(i) * 0.1f);
        p.percent = 80.0;
        REQUIRE(smooth(f, p, o) == Status::Ok);
        CHECK(o.at(20, 2) == Approx(f.at(20, 2)).margin(1e-5));
        CHECK(o.at(25, 2) == Approx(f.at(25, 2)).margin(1e-5));
    }
    SECTION("transparency preserved vs not") {
        HeightField f = make(9, 1, 1.0, kTransparent);
        for (std::size_t i = 0; i < 5; ++i) f.set(i, 0, 2.0f);
        p.percent = 50.0;
        REQUIRE(smooth(f, p, o) == Status::Ok);
        for (std::size_t i = 5; i < 9; ++i) CHECK(std::isnan(o.at(i, 0)));
        CHECK(o.at(4, 0) == Approx(2.0));  // edge value unchanged: renormalised over opaque cells
        p.preserveTransparency = false;
        REQUIRE(smooth(f, p, o) == Status::Ok);
        // cell 5 sees weights 1,e^-.5,... on the opaque side: fraction well above half -> filled
        CHECK(o.at(5, 0) == Approx(2.0));
        CHECK(std::isnan(o.at(8, 0)));  // too far from the data
    }
    SECTION("deterministic and input untouched") {
        HeightField f = make(20, 20, 0.7, 0.0f);
        for (std::size_t k = 0; k < 400; ++k) f.set(k % 20, k / 20, static_cast<float>((k * 37) % 11));
        p.percent = 60.0;
        HeightField a, b;
        REQUIRE(smooth(f, p, a) == Status::Ok);
        REQUIRE(smooth(f, p, b) == Status::Ok);
        CHECK(a.data() == b.data());
        CHECK(f.at(1, 0) == 37.0f - 33.0f);
    }
    SECTION("bad parameters") {
        const HeightField f = make(4, 4, 1.0, 1.0f);
        p.percent = -1;
        CHECK(smooth(f, p, o) == Status::BadParameter);
        p.percent = 101;
        CHECK(smooth(f, p, o) == Status::BadParameter);
        p.percent = NAN;
        CHECK(smooth(f, p, o) == Status::BadParameter);
        p.percent = 10;
        p.radiusMm = 0;
        CHECK(smooth(f, p, o) == Status::BadParameter);
        p.radiusMm = INFINITY;
        CHECK(smooth(f, p, o) == Status::BadParameter);
        p.radiusMm = 5000;
        p.percent = 100;
        CHECK(smooth(f, p, o) == Status::BadParameter);  // kernel wider than 512 cells
        CHECK(smooth(HeightField{}, SmoothParams{}, o) == Status::ZeroSize);
    }
}

TEST_CASE("emboss", "[relief][modify]") {
    HeightField f = make(41, 5, 0.5, 0.0f);
    for (std::size_t j = 0; j < 5; ++j) for (std::size_t i = 0; i < 41; ++i) f.set(i, j, 1.0f + static_cast<float>(i) * 0.1f);
    f.set(0, 0, kTransparent);
    HeightField o;
    EmbossParams p;
    p.detailSmoothnessPercent = 80.0;
    SECTION("scale 1 is the identity") {
        p.scale = 1.0;
        REQUIRE(emboss(f, p, o) == Status::Ok);
        for (std::size_t k = 0; k < f.data().size(); ++k) CHECK(same(o.data()[k], f.data()[k]));
    }
    SECTION("scale s flattens a ramp to s * ramp in the interior") {
        p.scale = 0.25;
        REQUIRE(emboss(f, p, o) == Status::Ok);
        CHECK(o.at(20, 2) == Approx(0.25 * f.at(20, 2)).margin(1e-5));
        CHECK(std::isnan(o.at(0, 0)));
    }
    SECTION("detail survives: a bump on a plateau keeps its height at scale 0") {
        HeightField g = make(41, 41, 0.5, 5.0f);
        g.set(20, 20, 6.0f);
        p.scale = 0.0;
        p.detailSmoothnessPercent = 100.0;
        p.radiusMm = 1.0;
        REQUIRE(emboss(g, p, o) == Status::Ok);
        const double low = 5.0 + (6.0 - 5.0) * 0.0;  // plateau far from the bump
        (void)low;
        CHECK(o.at(2, 2) == Approx(0.0).margin(1e-5));          // plateau flattened to zero
        CHECK(o.at(20, 20) > 0.5);                              // the bump stays prominent
        CHECK(o.at(20, 20) < 1.0);
    }
    SECTION("scale above 1 exaggerates") {
        p.scale = 2.0;
        REQUIRE(emboss(f, p, o) == Status::Ok);
        CHECK(o.at(20, 2) == Approx(2.0 * f.at(20, 2)).margin(1e-5));
    }
    SECTION("bad parameters") {
        p.scale = -0.1;
        CHECK(emboss(f, p, o) == Status::BadParameter);
        p.scale = NAN;
        CHECK(emboss(f, p, o) == Status::BadParameter);
        p.scale = 1.0;
        p.detailSmoothnessPercent = 120.0;
        CHECK(emboss(f, p, o) == Status::BadParameter);
    }
}
