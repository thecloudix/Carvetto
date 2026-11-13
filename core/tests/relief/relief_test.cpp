#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/relief.hpp"

using namespace carvetto::relief;
using Catch::Approx;

namespace {
const float N = kTransparent;
Grid g3() { return Grid{3, 3, 1.0, {0.0, 0.0}}; }
HeightField mk(const Grid& g, std::vector<float> d) {
    HeightField f;
    REQUIRE(HeightField::fromData(g, std::move(d), f) == Status::Ok);
    return f;
}
bool same(float a, float b) { return (std::isnan(a) && std::isnan(b)) || a == b; }
void expectData(const HeightField& f, const std::vector<float>& e) {
    REQUIRE(f.data().size() == e.size());
    for (std::size_t k = 0; k < e.size(); ++k) {
        INFO("cell " << k << " got " << f.data()[k] << " want " << e[k]);
        CHECK(same(f.data()[k], e[k]));
    }
}
Component comp(HeightField f, CombineMode m = CombineMode::Add) {
    Component c;
    c.field = std::move(f);
    c.mode = m;
    return c;
}
Level lvl(std::vector<Component> cs) {
    Level l;
    l.components = std::move(cs);
    return l;
}
// A = 1 2 3 / 4 5 6 / 7 8 N ; B = 2 N 1 / 6 5 4 / N 9 N
const std::vector<float> A = {1, 2, 3, 4, 5, 6, 7, 8, N};
const std::vector<float> B = {2, N, 1, 6, 5, 4, N, 9, N};
HeightField run(CombineMode m) {
    HeightField out;
    REQUIRE(compose({lvl({comp(mk(g3(), A)), comp(mk(g3(), B), m)})}, g3(), out) == Status::Ok);
    return out;
}
}  // namespace

TEST_CASE("combine modes: hand-computed 3x3 goldens with transparent cells") {
    expectData(run(CombineMode::Add), {3, 2, 4, 10, 10, 10, 7, 17, N});
    expectData(run(CombineMode::Subtract), {-1, 2, 2, -2, 0, 2, 7, -1, N});
    expectData(run(CombineMode::MergeHigh), {2, 2, 3, 6, 5, 6, 7, 9, N});
    expectData(run(CombineMode::MergeLow), {1, 2, 1, 4, 5, 4, 7, 8, N});
    expectData(run(CombineMode::Multiply), {2, 2, 3, 24, 25, 24, 7, 72, N});
}

TEST_CASE("combine: transparent rules") {
    CHECK(combine(CombineMode::Add, N, 3.f) == 3.f);
    CHECK(combine(CombineMode::MergeLow, N, 3.f) == 3.f);
    CHECK(std::isnan(combine(CombineMode::Subtract, N, 3.f)));
    CHECK(std::isnan(combine(CombineMode::Multiply, N, 3.f)));
    for (auto m : {CombineMode::Add, CombineMode::Subtract, CombineMode::MergeHigh, CombineMode::MergeLow,
                   CombineMode::Multiply}) {
        CHECK(combine(m, 4.f, N) == 4.f);
        CHECK(std::isnan(combine(m, N, N)));
    }
}

TEST_CASE("field basics: sample, minMax, crop, resample") {
    const HeightField f = mk(g3(), {0, 1, 2, 10, 11, 12, 20, 21, N});
    CHECK(f.sample(1.0, 1.0) == 11.f);
    CHECK(f.sample(0.5, 0.0) == Approx(0.5));
    CHECK(f.sample(0.5, 0.5) == Approx(5.5));
    CHECK(std::isnan(f.sample(-0.1, 0.0)));
    CHECK(std::isnan(f.sample(2.5, 0.0)));
    CHECK(std::isnan(f.sample(1.5, 1.5)));  // touches the transparent cell
    CHECK(f.sample(2.0, 1.0) == 12.f);
    float lo = 0, hi = 0;
    REQUIRE(f.minMax(lo, hi));
    CHECK(lo == 0.f);
    CHECK(hi == 21.f);
    HeightField c;
    REQUIRE(f.crop(1, 1, 2, 2, c) == Status::Ok);
    expectData(c, {11, 12, 21, N});
    CHECK(c.origin().x == 1.0);
    CHECK(f.crop(2, 2, 2, 1, c) == Status::BadRegion);
    HeightField r;
    REQUIRE(f.resample(Grid{3, 2, 0.5, {0.0, 0.0}}, r) == Status::Ok);
    expectData(r, {0, 0.5f, 1, 5, 5.5f, 6});
    HeightField allN = mk(g3(), std::vector<float>(9, N));
    CHECK_FALSE(allN.minMax(lo, hi));
}

TEST_CASE("degenerate input and caps return errors") {
    HeightField f;
    CHECK(HeightField::create(Grid{0, 3, 1.0, {}}, 0.f, f) == Status::ZeroSize);
    CHECK(HeightField::create(Grid{3, 3, 0.0, {}}, 0.f, f) == Status::ZeroSize);
    CHECK(HeightField::create(Grid{3, 3, std::nan(""), {}}, 0.f, f) == Status::ZeroSize);
    CHECK(HeightField::create(Grid{kMaxCells, 2, 1.0, {}}, 0.f, f) == Status::TooLarge);
    CHECK(HeightField::create(Grid{std::size_t(-1), std::size_t(-1), 1.0, {}}, 0.f, f) == Status::TooLarge);
    CHECK(f.empty());
    CHECK(HeightField::fromData(g3(), {1, 2}, f) == Status::BadRegion);
    HeightField out;
    CHECK(compose({}, Grid{0, 0, 1, {}}, out) == Status::ZeroSize);
    CHECK(composeComponent(Component{}, g3(), out) == Status::ZeroSize);
    Grid u;
    CHECK(unionGrid({}, u) == Status::ZeroSize);
    Component bad = comp(mk(g3(), A));
    bad.fade = {true, {0, 0}, {0, 0}, 50.0};
    CHECK(bake(bad, out) == Status::BadParameter);
}

TEST_CASE("fade and tilt: goldens, non-destructive, bake equals composed") {
    Component c = comp(mk(g3(), std::vector<float>(9, 10.f)));
    c.fade = {true, {0, 0}, {2, 0}, 50.0};
    HeightField b;
    REQUIRE(bake(c, b) == Status::Ok);
    expectData(b, {10, 7.5f, 5, 10, 7.5f, 5, 10, 7.5f, 5});
    expectData(c.field, std::vector<float>(9, 10.f));  // stored field untouched
    Component t = comp(mk(g3(), std::vector<float>(9, 1.f)));
    t.tilt = {true, {0, 0}, {0, 2}, std::atan(0.5)};  // +0.5 mm per mm along +y
    REQUIRE(bake(t, b) == Status::Ok);
    expectData(b, {1, 1, 1, 1.5f, 1.5f, 1.5f, 2, 2, 2});
    // shape height, fade, tilt and base together: v*0.5 -> fade -> tilt -> base
    Component all = comp(mk(g3(), std::vector<float>(9, 10.f)));
    all.shapeHeightPercent = 50.0;
    all.fade = {true, {0, 0}, {2, 0}, 100.0};
    all.tilt = t.tilt;
    all.baseHeight = 3.0;
    HeightField composed, viaCompose;
    REQUIRE(bake(all, composed) == Status::Ok);
    expectData(composed, {8, 5.5f, 3 + 0, 8.5f, 6, 3.5f, 9, 6.5f, 4});
    REQUIRE(compose({lvl({all})}, g3(), viaCompose) == Status::Ok);
    CHECK(viaCompose.data().size() == composed.data().size());
    for (std::size_t k = 0; k < 9; ++k) CHECK(viaCompose.data()[k] == composed.data()[k]);
    // transparent stays transparent through fade/tilt/base
    Component tr = comp(mk(g3(), A));
    tr.baseHeight = 1.0;
    REQUIRE(bake(tr, b) == Status::Ok);
    CHECK(std::isnan(b.at(2, 2)));
    CHECK(b.at(0, 0) == 2.f);
}

TEST_CASE("level base height applies after combining and is not in bake") {
    Component a = comp(mk(g3(), std::vector<float>(9, 2.f)));
    Component m = comp(mk(g3(), std::vector<float>(9, 3.f)), CombineMode::Multiply);
    Level l = lvl({a, m});
    l.baseHeight = 1.0;
    HeightField out;
    REQUIRE(compose({l}, g3(), out) == Status::Ok);
    expectData(out, std::vector<float>(9, 7.f));  // (2*3)+1, not (2+1)*3
    HeightField b;
    REQUIRE(bake(l.components[0], b) == Status::Ok);
    expectData(b, std::vector<float>(9, 2.f));
}

TEST_CASE("levels combine with their own mode; hidden items skipped; union grid") {
    Level l1 = lvl({comp(mk(g3(), A))});
    Level l2 = lvl({comp(mk(g3(), B))});
    l2.mode = CombineMode::MergeHigh;
    HeightField out;
    REQUIRE(compose({l1, l2}, g3(), out) == Status::Ok);
    expectData(out, {2, 2, 3, 6, 5, 6, 7, 9, N});
    l2.visible = false;
    REQUIRE(compose({l1, l2}, g3(), out) == Status::Ok);
    expectData(out, A);
    Level l3 = lvl({comp(mk(g3(), A))});
    l3.components[0].visible = false;
    Grid u;
    CHECK(unionGrid({l3}, u) == Status::ZeroSize);
    Component off = comp(mk(Grid{2, 2, 0.5, {4.0, -1.0}}, {1, 1, 1, 1}));
    REQUIRE(unionGrid({l1, lvl({off})}, u) == Status::Ok);
    CHECK(u.cellSize == 0.5);
    CHECK(u.origin.x == 0.0);
    CHECK(u.origin.y == -1.0);
    CHECK(u.width == 10);   // x 0 .. 4.5 at 0.5
    CHECK(u.height == 7);   // y -1 .. 2 at 0.5
    // determinism
    HeightField o1, o2;
    REQUIRE(compose({l1, l2}, u, o1) == Status::Ok);
    REQUIRE(compose({l1, l2}, u, o2) == Status::Ok);
    CHECK(o1.data().size() == o2.data().size());
}
