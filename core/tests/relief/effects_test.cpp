// T-0295: level effects (clipping, mirror mode) in core/relief (ADR-0079).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <vector>

#include "carvetto/relief/effects.hpp"

using namespace carvetto::relief;

namespace {

/// w x h grid, cell 1, origin (0,0), value 1 + i + w*j: asymmetric in both axes.
HeightField ramp(std::size_t w, std::size_t h) {
    std::vector<float> d(w * h);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) d[j * w + i] = 1.0F + static_cast<float>(i + w * j);
    }
    HeightField f;
    REQUIRE(HeightField::fromData(Grid{w, h, 1.0, {0, 0}}, std::move(d), f) == Status::Ok);
    return f;
}

HeightField mirrored(MirrorMode m, std::size_t w, std::size_t h, Point c) {
    HeightField f = ramp(w, h);
    REQUIRE(applyMirror(m, c, f) == Status::Ok);
    return f;
}

float v(std::size_t i, std::size_t j) { return 1.0F + static_cast<float>(i + 4 * j); }

Clip square(double x0, double y0, double x1, double y1) {
    Clip c;
    c.polygons.push_back({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
    return c;
}

Level levelOf(const HeightField& f) {
    Level l;
    Component c;
    c.name = "c";
    c.field = f;
    l.components.push_back(c);
    l.name = "L";
    return l;
}

}  // namespace

TEST_CASE("mirror: half modes copy the source side onto the other, golden on an asymmetric field", "[relief][effects]") {
    const Point c{1.5, 1.5};
    {
        const HeightField f = mirrored(MirrorMode::LeftToRight, 4, 4, c);
        for (std::size_t j = 0; j < 4; ++j) {
            CHECK(f.at(0, j) == v(0, j));
            CHECK(f.at(1, j) == v(1, j));
            CHECK(f.at(2, j) == v(1, j));
            CHECK(f.at(3, j) == v(0, j));
        }
    }
    {
        const HeightField f = mirrored(MirrorMode::RightToLeft, 4, 4, c);
        for (std::size_t j = 0; j < 4; ++j) {
            CHECK(f.at(0, j) == v(3, j));
            CHECK(f.at(1, j) == v(2, j));
            CHECK(f.at(2, j) == v(2, j));
            CHECK(f.at(3, j) == v(3, j));
        }
    }
    {  // Top is the larger Y (row index).
        const HeightField f = mirrored(MirrorMode::TopToBottom, 4, 4, c);
        for (std::size_t i = 0; i < 4; ++i) {
            CHECK(f.at(i, 3) == v(i, 3));
            CHECK(f.at(i, 2) == v(i, 2));
            CHECK(f.at(i, 1) == v(i, 2));
            CHECK(f.at(i, 0) == v(i, 3));
        }
    }
    {
        const HeightField f = mirrored(MirrorMode::BottomToTop, 4, 4, c);
        for (std::size_t i = 0; i < 4; ++i) {
            CHECK(f.at(i, 0) == v(i, 0));
            CHECK(f.at(i, 1) == v(i, 1));
            CHECK(f.at(i, 2) == v(i, 1));
            CHECK(f.at(i, 3) == v(i, 0));
        }
    }
}

TEST_CASE("mirror: quadrant modes copy one quadrant onto the other three", "[relief][effects]") {
    const Point c{1.5, 1.5};
    struct Case {
        MirrorMode mode;
        std::size_t si, sj;  // the source cell that lands in the corner cell (0,0)'s quadrant group
    };
    // Source quadrant corner cell for each mode; the four images of it fill the field.
    const Case cases[] = {{MirrorMode::QuadTopRight, 3, 3},
                          {MirrorMode::QuadTopLeft, 0, 3},
                          {MirrorMode::QuadBottomLeft, 0, 0},
                          {MirrorMode::QuadBottomRight, 3, 0}};
    for (const Case& k : cases) {
        const HeightField f = mirrored(k.mode, 4, 4, c);
        const std::size_t oi = 3 - k.si, oj = 3 - k.sj;
        CHECK(f.at(k.si, k.sj) == v(k.si, k.sj));
        CHECK(f.at(oi, k.sj) == v(k.si, k.sj));
        CHECK(f.at(k.si, oj) == v(k.si, k.sj));
        CHECK(f.at(oi, oj) == v(k.si, k.sj));
    }
    // Inner cells of the top-right quadrant too.
    const HeightField f = mirrored(MirrorMode::QuadTopRight, 4, 4, c);
    CHECK(f.at(1, 1) == v(2, 2));
    CHECK(f.at(2, 1) == v(2, 2));
    CHECK(f.at(1, 2) == v(2, 2));
    CHECK(f.at(0, 1) == v(3, 2));
}

TEST_CASE("mirror: cells on an axis keep their value; None and bad centre", "[relief][effects]") {
    const HeightField f = mirrored(MirrorMode::LeftToRight, 5, 3, {2.0, 1.0});
    for (std::size_t j = 0; j < 3; ++j) {
        CHECK(f.at(2, j) == 1.0F + static_cast<float>(2 + 5 * j));
        CHECK(f.at(3, j) == f.at(1, j));
        CHECK(f.at(4, j) == f.at(0, j));
    }
    const HeightField same = mirrored(MirrorMode::None, 4, 4, {0, 0});
    CHECK(same.data() == ramp(4, 4).data());
    HeightField g = ramp(2, 2);
    CHECK(applyMirror(MirrorMode::LeftToRight, {std::nan(""), 0}, g) == Status::BadParameter);
}

TEST_CASE("mirror: a transparent source makes the target transparent; target values are discarded", "[relief][effects]") {
    HeightField f = ramp(4, 1);
    f.set(0, 0, kTransparent);
    REQUIRE(applyMirror(MirrorMode::LeftToRight, {1.5, 0.0}, f) == Status::Ok);
    CHECK(std::isnan(f.at(0, 0)));
    CHECK(std::isnan(f.at(3, 0)));
    CHECK(f.at(2, 0) == 2.0F);
}

TEST_CASE("mirror: a centre between cells samples bilinearly", "[relief][effects]") {
    // Centre at x = 1.25: cell 2 (x=2) mirrors to x = 0.5, halfway between cells 0 and 1.
    HeightField f = ramp(4, 1);
    REQUIRE(applyMirror(MirrorMode::LeftToRight, {1.25, 0.0}, f) == Status::Ok);
    CHECK(f.at(2, 0) == 1.5F);
    CHECK(std::isnan(f.at(3, 0)));  // mirrors to x = -0.5: outside the field
}

TEST_CASE("clip: keeps cells whose centre is inside", "[relief][effects]") {
    HeightField f = ramp(4, 4);
    REQUIRE(applyClip(square(0.5, 0.5, 2.5, 2.5), f) == Status::Ok);
    for (std::size_t j = 0; j < 4; ++j) {
        for (std::size_t i = 0; i < 4; ++i) {
            const bool in = i >= 1 && i <= 2 && j >= 1 && j <= 2;
            CHECK(std::isnan(f.at(i, j)) == !in);
            if (in) CHECK(f.at(i, j) == v(i, j));
        }
    }
}

TEST_CASE("clip: boundary cells, holes, concave and several polygons", "[relief][effects]") {
    {  // centres exactly on edges: left and bottom inside, right and top outside
        HeightField f = ramp(4, 4);
        REQUIRE(applyClip(square(1.0, 1.0, 3.0, 3.0), f) == Status::Ok);
        CHECK(f.at(1, 1) == v(1, 1));
        CHECK(f.at(2, 2) == v(2, 2));
        CHECK(std::isnan(f.at(3, 1)));
        CHECK(std::isnan(f.at(1, 3)));
        CHECK(std::isnan(f.at(0, 0)));
    }
    {  // hole by even-odd
        Clip c = square(-0.5, -0.5, 4.5, 4.5);
        const Clip hole = square(0.5, 0.5, 2.5, 2.5);
        c.polygons.push_back(hole.polygons[0]);
        HeightField f = ramp(5, 5);
        REQUIRE(applyClip(c, f) == Status::Ok);
        CHECK(f.at(0, 0) == 1.0F);
        CHECK(std::isnan(f.at(1, 1)));
        CHECK(std::isnan(f.at(2, 2)));
        CHECK(f.at(3, 3) == 1.0F + 3 + 15);
    }
    {  // concave L shape
        Clip c;
        c.polygons.push_back({{-0.5, -0.5}, {3.5, -0.5}, {3.5, 0.5}, {0.5, 0.5}, {0.5, 3.5}, {-0.5, 3.5}});
        HeightField f = ramp(4, 4);
        REQUIRE(applyClip(c, f) == Status::Ok);
        CHECK(f.at(3, 0) == v(3, 0));
        CHECK(f.at(0, 3) == v(0, 3));
        CHECK(std::isnan(f.at(1, 1)));
        CHECK(std::isnan(f.at(3, 3)));
    }
    {  // two disjoint polygons
        Clip c = square(-0.5, -0.5, 0.5, 0.5);
        c.polygons.push_back(square(2.5, 2.5, 3.5, 3.5).polygons[0]);
        HeightField f = ramp(4, 4);
        REQUIRE(applyClip(c, f) == Status::Ok);
        CHECK(f.at(0, 0) == v(0, 0));
        CHECK(f.at(3, 3) == v(3, 3));
        CHECK(std::isnan(f.at(1, 1)));
    }
    {  // winding does not matter
        Clip c;
        c.polygons.push_back({{0.5, 2.5}, {2.5, 2.5}, {2.5, 0.5}, {0.5, 0.5}});
        HeightField f = ramp(4, 4);
        REQUIRE(applyClip(c, f) == Status::Ok);
        CHECK(f.at(1, 1) == v(1, 1));
        CHECK(std::isnan(f.at(0, 0)));
    }
    {  // empty clip: no change
        HeightField f = ramp(4, 4);
        REQUIRE(applyClip(Clip{}, f) == Status::Ok);
        CHECK(f.data() == ramp(4, 4).data());
    }
}

TEST_CASE("effects: validation", "[relief][effects]") {
    LevelEffects e;
    CHECK(validateEffects(e) == Status::Ok);
    e.clip.polygons.push_back({{0, 0}, {1, 0}});
    CHECK(validateEffects(e) == Status::BadParameter);
    e.clip = square(0, 0, 1, 1);
    CHECK(validateEffects(e) == Status::Ok);
    e.clip.polygons[0][2].x = std::nan("");
    CHECK(validateEffects(e) == Status::BadParameter);
    e.clip = Clip{};
    for (std::size_t i = 0; i < kMaxClipPolygons; ++i) e.clip.polygons.push_back(square(0, 0, 1, 1).polygons[0]);
    CHECK(validateEffects(e) == Status::Ok);
    e.clip.polygons.push_back(square(0, 0, 1, 1).polygons[0]);
    CHECK(validateEffects(e) == Status::BadParameter);
    e.clip = Clip{};
    e.clip.polygons.push_back(std::vector<Point>(kMaxClipVertices, Point{0, 0}));
    CHECK(validateEffects(e) == Status::Ok);
    e.clip.polygons.push_back(square(0, 0, 1, 1).polygons[0]);
    CHECK(validateEffects(e) == Status::BadParameter);
    e.clip = Clip{};
    e.mirrorCenter.y = INFINITY;
    CHECK(validateEffects(e) == Status::BadParameter);
    HeightField f = ramp(2, 2);
    CHECK(applyEffects(e, f) == Status::BadParameter);
}

TEST_CASE("compose: effects apply per level, never change stored fields, mirror before clip", "[relief][effects]") {
    const HeightField stored = ramp(4, 4);
    Level l = levelOf(stored);
    l.baseHeight = 10.0;
    l.effects.mirror = MirrorMode::LeftToRight;
    l.effects.mirrorCenter = {1.5, 1.5};
    l.effects.clip = square(-0.5, -0.5, 2.5, 3.5);  // keeps columns 0..2
    const Grid grid = stored.grid();
    HeightField out;
    REQUIRE(compose({l}, grid, out) == Status::Ok);
    CHECK(out.at(0, 1) == v(0, 1) + 10.0F);
    CHECK(out.at(2, 1) == v(1, 1) + 10.0F);  // mirrored from column 1
    CHECK(std::isnan(out.at(3, 1)));          // mirrored cell, then clipped away
    CHECK(l.components[0].field.data() == stored.data());
    // Removing the effects restores the plain model.
    l.effects = LevelEffects{};
    HeightField plain;
    REQUIRE(compose({l}, grid, plain) == Status::Ok);
    CHECK(plain.at(3, 1) == v(3, 1) + 10.0F);
}

TEST_CASE("compose: effects of one level do not touch another level", "[relief][effects]") {
    Level a = levelOf(ramp(4, 4));
    Level b = levelOf(ramp(4, 4));
    b.effects.clip = square(-0.5, -0.5, 0.5, 0.5);
    b.mode = CombineMode::Add;
    HeightField out;
    REQUIRE(compose({a, b}, a.components[0].field.grid(), out) == Status::Ok);
    CHECK(out.at(0, 0) == 2.0F * v(0, 0));
    CHECK(out.at(3, 3) == v(3, 3));  // level b is transparent here
}

TEST_CASE("compose: invalid effects are an error", "[relief][effects]") {
    Level l = levelOf(ramp(2, 2));
    l.effects.clip.polygons.push_back({{0, 0}, {1, 1}});
    HeightField out;
    CHECK(compose({l}, l.components[0].field.grid(), out) == Status::BadParameter);
}

TEST_CASE("bakeLevel: the baked component reproduces the composed level, without base height", "[relief][effects]") {
    Level l = levelOf(ramp(4, 4));
    l.baseHeight = 7.0;
    l.effects.mirror = MirrorMode::QuadBottomLeft;
    l.effects.mirrorCenter = {1.5, 1.5};
    l.effects.clip = square(-0.5, -0.5, 3.5, 2.5);
    Component baked;
    REQUIRE(bakeLevel(l, baked) == Status::Ok);
    CHECK(baked.name == "L (baked)");
    HeightField expect;
    REQUIRE(compose({l}, baked.field.grid(), expect) == Status::Ok);
    for (std::size_t k = 0; k < expect.data().size(); ++k) {
        const float e = expect.data()[k];
        const float b = baked.field.data()[k];
        if (std::isnan(e)) {
            CHECK(std::isnan(b));
        } else {
            CHECK(b + 7.0F == e);
        }
    }
    // The baked component alone, with the effects dropped, equals the effect-bearing level.
    Level plain;
    plain.components.push_back(baked);
    plain.baseHeight = 7.0;
    HeightField again;
    REQUIRE(compose({plain}, baked.field.grid(), again) == Status::Ok);
    for (std::size_t k = 0; k < expect.data().size(); ++k) {
        if (std::isnan(expect.data()[k])) CHECK(std::isnan(again.data()[k]));
        else CHECK(again.data()[k] == expect.data()[k]);
    }
    Level empty;
    CHECK(bakeLevel(empty, baked) != Status::Ok);
}
