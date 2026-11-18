#include "carvetto/viewmodel/print_layout.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using namespace carvetto::viewmodel;
using Catch::Matchers::WithinAbs;

namespace {
PrintOptions trueSize(double overlap = 10.0) {
    PrintOptions o;
    o.mode = PrintScaleMode::TrueSize;
    o.overlapMm = overlap;
    return o;
}
}  // namespace

TEST_CASE("fit to page scales the content onto one centred page", "[viewmodel][print]") {
    const PrintPageSetup page;  // A4 portrait, 10 mm margin: 190 x 277 printable
    const Box2 content{{0, 0}, {380, 100}};
    const PrintLayout l = layoutPrint(content, page, PrintOptions{});
    REQUIRE(l.error.empty());
    REQUIRE(l.pageCount() == 1);
    CHECK_THAT(l.scale, WithinAbs(0.5, 1e-12));
    const PrintTile& t = l.tiles[0];
    // Centred: the content centre maps to the centre of the printable area.
    const Vec2 c = printWorldToPage(l, t, content.centre());
    CHECK_THAT(c.x, WithinAbs(105.0, 1e-9));
    CHECK_THAT(c.y, WithinAbs(148.5, 1e-9));
    CHECK(t.marks.empty());
    // A small design is scaled up to fill the page too.
    const PrintLayout small = layoutPrint(Box2{{0, 0}, {19, 10}}, page, PrintOptions{});
    CHECK_THAT(small.scale, WithinAbs(10.0, 1e-12));
}

TEST_CASE("true size tiles a large design with overlap and registration marks", "[viewmodel][print]") {
    const PrintPageSetup page;  // 190 x 277 printable
    const Box2 content{{0, 0}, {500, 400}};
    const PrintLayout l = layoutPrint(content, page, trueSize(10.0));
    REQUIRE(l.error.empty());
    CHECK_THAT(l.scale, WithinAbs(1.0, 1e-12));
    // Steps 180 and 267: ceil((500-190)/180)+1 = 3 columns, ceil((400-277)/267)+1 = 2 rows.
    CHECK(l.columns == 3);
    CHECK(l.rows == 2);
    REQUIRE(l.pageCount() == 6);
    // Row-major numbering from the top-left; the first window starts at the content's top-left.
    CHECK(l.tiles[0].page == 1);
    CHECK(l.tiles[5].page == 6);
    CHECK_THAT(l.tiles[0].window.min.x, WithinAbs(0.0, 1e-9));
    CHECK_THAT(l.tiles[0].window.max.y, WithinAbs(400.0, 1e-9));
    // Neighbouring windows overlap by 10 mm.
    CHECK_THAT(l.tiles[0].window.max.x - l.tiles[1].window.min.x, WithinAbs(10.0, 1e-9));
    CHECK_THAT(l.tiles[0].window.min.y - l.tiles[3].window.max.y, WithinAbs(-10.0, 1e-9));
    // The tiles cover the whole content.
    CHECK(l.tiles.back().window.max.x >= 500.0);
    CHECK(l.tiles.back().window.min.y <= 0.0);
    // Every mark of a tile has a mark of the right-hand neighbour at the same world position.
    const PrintTile& a = l.tiles[0];
    const PrintTile& b = l.tiles[1];
    int matched = 0;
    for (const Vec2& m : a.marks) {
        const Vec2 world{a.window.min.x + (m.x - l.marginMm) / l.scale, a.window.max.y - (m.y - l.marginMm) / l.scale};
        for (const Vec2& n : b.marks) {
            const Vec2 w2 = {b.window.min.x + (n.x - l.marginMm) / l.scale, b.window.max.y - (n.y - l.marginMm) / l.scale};
            if (std::fabs(world.x - w2.x) < 1e-9 && std::fabs(world.y - w2.y) < 1e-9) {
                ++matched;
                break;
            }
        }
    }
    CHECK(matched == 8);
    CHECK(a.marks.size() == 16);
}

TEST_CASE("true size at 1:1 of a design that fits prints one page without marks", "[viewmodel][print]") {
    const PrintLayout l = layoutPrint(Box2{{0, 0}, {100, 100}}, PrintPageSetup{}, trueSize());
    REQUIRE(l.pageCount() == 1);
    CHECK_THAT(l.scale, WithinAbs(1.0, 1e-12));
    CHECK(l.tiles[0].marks.empty());
}

TEST_CASE("zero overlap and marks off", "[viewmodel][print]") {
    PrintOptions o = trueSize(0.0);
    const PrintLayout l = layoutPrint(Box2{{0, 0}, {380, 100}}, PrintPageSetup{}, o);
    CHECK(l.columns == 2);
    CHECK(l.tiles[0].marks.size() == 4);
    o.registrationMarks = false;
    CHECK(layoutPrint(Box2{{0, 0}, {380, 100}}, PrintPageSetup{}, o).tiles[1].marks.empty());
}

TEST_CASE("refusals: nothing to print, tiny page, absurd scale, too many pages", "[viewmodel][print]") {
    CHECK_FALSE(layoutPrint(Box2{}, PrintPageSetup{}, PrintOptions{}).error.empty());
    PrintPageSetup tiny;
    tiny.pageWidthMm = 15.0;
    CHECK_FALSE(layoutPrint(Box2{{0, 0}, {1, 1}}, tiny, PrintOptions{}).error.empty());
    PrintOptions o = trueSize();
    o.trueScale = 0.0;
    CHECK_FALSE(layoutPrint(Box2{{0, 0}, {1, 1}}, PrintPageSetup{}, o).error.empty());
    o.trueScale = 1.0;
    const PrintLayout big = layoutPrint(Box2{{0, 0}, {100000, 100000}}, PrintPageSetup{}, o);
    CHECK(big.tiles.empty());
    CHECK_FALSE(big.error.empty());
}

TEST_CASE("a point-sized design fits to a page", "[viewmodel][print]") {
    const PrintLayout l = layoutPrint(Box2{{5, 5}, {5, 5}}, PrintPageSetup{}, PrintOptions{});
    REQUIRE(l.pageCount() == 1);
    CHECK_THAT(l.scale, WithinAbs(1.0, 1e-12));
}
