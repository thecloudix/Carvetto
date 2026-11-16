// T-0321: the Wrapping level effect in core/relief (ADR-0114).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <vector>

#include "carvetto/relief/effects.hpp"

using namespace carvetto::relief;

namespace {

/// w x h grid, cell 1, origin (ox, oy); value from `fn(i, j)`.
template <typename Fn>
HeightField field(std::size_t w, std::size_t h, double ox, double oy, Fn fn, double cell = 1.0) {
    std::vector<float> d(w * h);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) d[j * w + i] = fn(i, j);
    }
    HeightField f;
    REQUIRE(HeightField::fromData(Grid{w, h, cell, {ox, oy}}, std::move(d), f) == Status::Ok);
    return f;
}

Wrapping wrapY(double circumference, double start = 0.0) {
    Wrapping w;
    w.enabled = true;
    w.circumferenceIsY = true;
    w.circumference = circumference;
    w.start = start;
    return w;
}

bool same(float a, float b) { return (std::isnan(a) && std::isnan(b)) || a == b; }

float srcValue(std::size_t i, std::size_t j) {
    if (i == 2 && j == 11) return kTransparent;  // a hole in the overhang
    return 1.0F + static_cast<float>(i) + 10.0F * static_cast<float>((j * 7) % 13);
}

}  // namespace

TEST_CASE("wrapping: equals the direct wrapped surface (max over the turns), exact on cell centres", "[relief][effects][wrapping]") {
    // 5 wide, 20 rows; circumference 8 cells: rows 0..7 are the window, 8..15 and 16..19 run past its edge.
    HeightField f = field(5, 20, 0, 0, srcValue);
    const HeightField src = f;
    REQUIRE(applyWrapping(wrapY(8.0), f) == Status::Ok);
    for (std::size_t j = 0; j < 20; ++j) {
        for (std::size_t i = 0; i < 5; ++i) {
            float expect = kTransparent;
            if (j < 8) {
                for (std::size_t k = j; k < 20; k += 8) expect = combine(CombineMode::MergeHigh, expect, src.at(i, k));
            }
            CHECK(same(f.at(i, j), expect));
        }
    }
    // The hole at (2, 11) is covered by the turn below (row 3 keeps its own value, as does row 19 -> 3).
    CHECK_FALSE(std::isnan(f.at(2, 3)));
}

TEST_CASE("wrapping: circumference along X is the transpose of along Y", "[relief][effects][wrapping]") {
    HeightField a = field(5, 20, 0, 0, srcValue);
    HeightField b = field(20, 5, 0, 0, [](std::size_t i, std::size_t j) { return srcValue(j, i); });
    Wrapping wx = wrapY(8.0);
    wx.circumferenceIsY = false;
    REQUIRE(applyWrapping(wrapY(8.0), a) == Status::Ok);
    REQUIRE(applyWrapping(wx, b) == Status::Ok);
    for (std::size_t j = 0; j < 20; ++j) {
        for (std::size_t i = 0; i < 5; ++i) CHECK(same(a.at(i, j), b.at(j, i)));
    }
}

TEST_CASE("wrapping: a window start moves the window; cells outside it are transparent", "[relief][effects][wrapping]") {
    HeightField f = field(2, 16, 0, 0, [](std::size_t, std::size_t j) { return static_cast<float>(j); });
    REQUIRE(applyWrapping(wrapY(6.0, 3.0), f) == Status::Ok);
    for (std::size_t j = 0; j < 16; ++j) {
        if (j < 3 || j >= 9) {
            CHECK(std::isnan(f.at(0, j)));
        } else {
            // window row j takes the highest of j, j + 6, j + 12 that exist in the field (j-6 < 0 or < start is below)
            float expect = static_cast<float>(j);
            while (expect + 6.0F <= 15.0F) expect += 6.0F;
            CHECK(f.at(1, j) == expect);
        }
    }
}

TEST_CASE("wrapping: a circumference that is not a whole number of cells interpolates", "[relief][effects][wrapping]") {
    HeightField f = field(1, 12, 0, 0, [](std::size_t, std::size_t j) { return 2.0F * static_cast<float>(j); });
    REQUIRE(applyWrapping(wrapY(7.25), f) == Status::Ok);
    // Row 2: copies at 2 and 9.25 (2 * 9.25 = 18.5, linear): the higher one. A quarter of the way from row 9 to 10.
    CHECK(f.at(0, 2) == 18.5F);
    // Row 0: copies at 0 and 7.25 (= 14.5); 14.5 is higher.
    CHECK(f.at(0, 0) == 14.5F);
    // Row 4: copy at 11.25 is past the last row centre (11), so only row 4 itself.
    CHECK(f.at(0, 4) == 8.0F);
    CHECK(std::isnan(f.at(0, 8)));  // outside the window [0, 7.25)
}

TEST_CASE("wrapping: a transparent neighbour makes an interpolated copy transparent", "[relief][effects][wrapping]") {
    HeightField f = field(1, 12, 0, 0, [](std::size_t, std::size_t j) { return j == 9 ? kTransparent : 1.0F; });
    REQUIRE(applyWrapping(wrapY(7.5), f) == Status::Ok);
    CHECK(f.at(0, 2) == 1.0F);  // copy at 9.5 straddles the hole at 9: transparent, row 2 itself stays
    HeightField g = field(1, 12, 0, 0, [](std::size_t, std::size_t j) { return j == 2 ? kTransparent : 1.0F; });
    REQUIRE(applyWrapping(wrapY(7.5), g) == Status::Ok);
    CHECK(g.at(0, 2) == 1.0F);  // row 2 is a hole but the copy at 9.5 fills it
}

TEST_CASE("wrapping: off, empty and invalid parameters", "[relief][effects][wrapping]") {
    HeightField f = field(3, 9, 0, 0, srcValue);
    const HeightField before = f;
    REQUIRE(applyWrapping(Wrapping{}, f) == Status::Ok);
    CHECK(f.data().size() == before.data().size());
    for (std::size_t k = 0; k < f.data().size(); ++k) CHECK(same(f.data()[k], before.data()[k]));
    HeightField none;
    CHECK(applyWrapping(wrapY(8.0), none) == Status::Ok);
    for (const double bad : {0.0, -1.0, std::nan(""), HUGE_VAL}) {
        CHECK(applyWrapping(wrapY(bad), f) == Status::BadParameter);
    }
    CHECK(applyWrapping(wrapY(8.0, std::nan("")), f) == Status::BadParameter);
    LevelEffects e;
    e.wrapping = wrapY(0.0);
    CHECK(validateEffects(e) == Status::BadParameter);
    e.wrapping = wrapY(8.0);
    CHECK(validateEffects(e) == Status::Ok);
    e.wrapping.enabled = false;
    e.wrapping.circumference = -3.0;  // ignored while off
    CHECK(validateEffects(e) == Status::Ok);
    for (std::size_t k = 0; k < f.data().size(); ++k) CHECK(same(f.data()[k], before.data()[k]));  // untouched on errors
}

TEST_CASE("wrapping: the number of turns is capped", "[relief][effects][wrapping]") {
    // Rows span (n - 1) cells; periods = span / C + 2 must not exceed kMaxWrapPeriods.
    const double c = 4.0;
    const std::size_t okRows = static_cast<std::size_t>((static_cast<double>(kMaxWrapPeriods) - 2.0) * c) + 1;
    HeightField ok = field(1, okRows, 0, 0, [](std::size_t, std::size_t) { return 1.0F; });
    CHECK(applyWrapping(wrapY(c), ok) == Status::Ok);
    HeightField over = field(1, okRows + 1, 0, 0, [](std::size_t, std::size_t) { return 1.0F; });
    CHECK(applyWrapping(wrapY(c), over) == Status::BadParameter);
    CHECK(over.data().size() == okRows + 1);
}

TEST_CASE("wrapping: order is mirror, wrapping, clip, and the level base height is added first", "[relief][effects][wrapping]") {
    Level l;
    l.name = "L";
    l.baseHeight = 2.0;
    Component c;
    c.name = "c";
    c.field = field(4, 12, 0, 0, [](std::size_t i, std::size_t j) { return 1.0F + static_cast<float>(i + 4 * j); });
    l.components.push_back(c);
    l.effects.mirror = MirrorMode::TopToBottom;  // along the circumference, so it does not commute with the wrapping
    l.effects.mirrorCenter = {1.5, 7.5};
    l.effects.wrapping = wrapY(8.0);
    l.effects.clip.polygons.push_back({{-0.5, -0.5}, {2.5, -0.5}, {2.5, 5.5}, {-0.5, 5.5}});
    const Grid grid{4, 12, 1.0, {0, 0}};
    HeightField composed;
    REQUIRE(compose({l}, grid, composed) == Status::Ok);

    HeightField manual = c.field;
    for (float& v : manual.mutableData()) v += 2.0F;
    REQUIRE(applyMirror(l.effects.mirror, l.effects.mirrorCenter, manual) == Status::Ok);
    REQUIRE(applyWrapping(l.effects.wrapping, manual) == Status::Ok);
    REQUIRE(applyClip(l.effects.clip, manual) == Status::Ok);
    for (std::size_t k = 0; k < manual.data().size(); ++k) CHECK(same(composed.data()[k], manual.data()[k]));
    // The wrapped rows beyond the window and the clip are gone, the window rows kept (row 0 carries the turn).
    CHECK_FALSE(std::isnan(composed.at(0, 0)));
    CHECK(std::isnan(composed.at(0, 9)));
    CHECK(std::isnan(composed.at(3, 0)));  // clipped (x = 3 > 2.5)
}

TEST_CASE("wrapping: bake equals the composition and the level no longer needs the effect", "[relief][effects][wrapping]") {
    Level l;
    l.name = "Rot";
    l.baseHeight = 4.0;
    Component c;
    c.name = "c";
    c.field = field(3, 14, 0, 0, srcValue);
    l.components.push_back(c);
    l.effects.wrapping = wrapY(6.0, 0.0);
    Component baked;
    REQUIRE(bakeLevel(l, baked) == Status::Ok);
    CHECK(baked.name == "Rot (baked)");
    // The bake has no base height; the composed level with the effect minus the base equals it.
    HeightField composed;
    REQUIRE(compose({l}, baked.field.grid(), composed) == Status::Ok);
    for (std::size_t k = 0; k < composed.data().size(); ++k) {
        const float v = composed.data()[k];
        CHECK(same(std::isnan(v) ? v : v - 4.0F, baked.field.data()[k]));
    }
    // Composing the baked component without the effect gives the same.
    Level plain;
    plain.components.push_back(baked);
    plain.baseHeight = 4.0;
    HeightField again;
    REQUIRE(compose({plain}, baked.field.grid(), again) == Status::Ok);
    for (std::size_t k = 0; k < again.data().size(); ++k) CHECK(same(again.data()[k], composed.data()[k]));
}

TEST_CASE("wrapping: a circumference of one cell folds every row onto the first", "[relief][effects][wrapping]") {
    // A whole circumference of one cell: every cell takes the highest value of its column.
    HeightField f = field(2, 5, 0, 0, [](std::size_t i, std::size_t j) { return static_cast<float>(i * 10 + (j == 3 ? 5 : j)); });
    REQUIRE(applyWrapping(wrapY(1.0), f) == Status::Ok);
    CHECK(f.at(0, 0) == 5.0F);
    CHECK(f.at(1, 0) == 15.0F);
    for (std::size_t j = 1; j < 5; ++j) CHECK(std::isnan(f.at(0, j)));
}
