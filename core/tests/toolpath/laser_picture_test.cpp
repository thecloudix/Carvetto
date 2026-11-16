// T-0324: Laser Picture toolpath (laser_picture.hpp, ADR-0104): power from the pixels (reference rasters for the
// power mapping and the error diffusion), line placement and serpentine, overscan, beam-off gaps, hatch, selected
// vectors, passes, and refusals.

#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/laser_picture.hpp"
#include "carvetto/toolpath/move.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

const double kNan = std::numeric_limits<double>::quiet_NaN();

tooling::Tool beam(tooling::ToolType type = tooling::ToolType::EndMill) {
    tooling::Tool t;
    t.id = "laser";
    t.name = "Laser";
    t.type = type;
    t.diameter = 0.2;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 900.0;
    t.defaultCuttingData.plungeRate = 900.0;
    t.defaultCuttingData.spindleSpeed = 0.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.maxBurnRate = 2000.0;
    return t;
}

/// A gray image from rows (top row first).
GrayImage image(const std::vector<std::vector<int>>& rows) {
    GrayImage g;
    g.height = static_cast<int>(rows.size());
    g.width = static_cast<int>(rows.front().size());
    for (const auto& r : rows) {
        for (const int v : r) {
            g.pixels.push_back(static_cast<std::uint8_t>(v));
        }
    }
    return g;
}

/// One millimetre per pixel, lower-left corner at the origin.
Box2 unitPlacement(const GrayImage& g) {
    return Box2({0.0, 0.0}, {static_cast<double>(g.width), static_cast<double>(g.height)});
}

LaserPictureParams params() {
    LaserPictureParams p;
    p.lineSpacing = 1.0;
    p.power = 100.0;
    return p;
}

LaserPictureResult run(const GrayImage& g, const LaserPictureParams& p, const std::vector<Contour>& cs = {}) {
    return laserPictureToolpath(g, unitPlacement(g), beam(), std::nullopt, p, cs);
}

std::vector<int> bits(const GrayImage& g) {
    std::vector<int> out;
    for (const std::uint8_t v : g.pixels) {
        out.push_back(v == 255 ? 1 : v == 0 ? 0 : -1);
    }
    return out;
}

Contour line(Vec2 a, Vec2 b) {
    Contour c;
    c.append(LineSpan{a, b});
    return c;
}

}  // namespace

TEST_CASE("T-0324: error diffusion matches reference rasters", "[toolpath][laser][picture]") {
    // References computed with exact rational Floyd-Steinberg (7, 3, 5, 1 sixteenths, threshold 128, 1 = dark).
    CHECK(bits(laserDither(image({{100, 100, 100, 100, 100, 100, 100, 100}}))) ==
          std::vector<int>{0, 1, 0, 0, 1, 0, 0, 1});
    CHECK(bits(laserDither(image({{128, 128, 128, 128}, {128, 128, 128, 128}, {128, 128, 128, 128},
                                  {128, 128, 128, 128}}))) ==
          std::vector<int>{1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1});
    CHECK(bits(laserDither(image({{0, 64, 128, 192, 255}, {255, 191, 127, 63, 0}, {128, 128, 128, 128, 128}}))) ==
          std::vector<int>{0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0});
    CHECK(bits(laserDither(image({{64, 64, 64, 64, 64, 64}, {64, 64, 64, 64, 64, 64}, {64, 64, 64, 64, 64, 64}}))) ==
          std::vector<int>{0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0});
    // A 12 x 6 image with every kind of error path (reference from exact rational arithmetic).
    std::vector<std::vector<int>> busy;
    for (int y = 0; y < 6; ++y) {
        std::vector<int> row;
        for (int x = 0; x < 12; ++x) {
            row.push_back((x * 21 + y * 13 + x * y * 7) % 256);
        }
        busy.push_back(row);
    }
    const char* expected[6] = {"000001011111", "001010111000", "000111100001",
                               "010101000111", "001110011100", "011100110001"};
    std::vector<int> want;
    for (const char* row : expected) {
        for (const char* c = row; *c != '\0'; ++c) {
            want.push_back(*c == '1' ? 1 : 0);
        }
    }
    CHECK(bits(laserDither(image(busy))) == want);
    // An image where the last share must be the remainder, not a sixteenth (the two differ by a few 1/256 units, which
    // flips the last pixel of the bottom row).
    CHECK(bits(laserDither(image({{116, 141, 155, 233, 154, 127},
                                  {69, 107, 175, 214, 71, 201},
                                  {90, 146, 199, 44, 28, 184},
                                  {227, 182, 213, 246, 108, 147}}))) ==
          std::vector<int>{0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0});
    // Pure black and pure white stay as they are.
    CHECK(bits(laserDither(image({{0, 255}, {255, 0}}))) == std::vector<int>{0, 1, 1, 0});
    // An invalid image comes back unchanged.
    CHECK(laserDither(GrayImage{}).pixels.empty());
}

TEST_CASE("T-0324: the dithered picture burns only at full power on the dark pixels", "[toolpath][laser][picture]") {
    // Mid gray 127 (darkness 128): the checkerboard of the second reference (rows from the top).
    const GrayImage g = image({{127, 127, 127, 127}, {127, 127, 127, 127}, {127, 127, 127, 127}, {127, 127, 127, 127}});
    LaserPictureParams p = params();
    p.dither = true;
    p.power = 60.0;
    p.minPower = 10.0;  // ignored: a dithered pixel is fully dark
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    for (const Move& m : r.toolpath.moves) {
        CHECK((m.jetPower == 0.0 || m.jetPower == 60.0));
    }
    // Line y = 0.5 is the bottom row {0, 1, 0, 1} (bottom row = 4th: 0 1 0 1 from the reference).
    // Forward: rapid to the first dark pixel (x 1..2), beam-off 2..3 gap would be a feed, then 3..4 dark.
    REQUIRE(r.toolpath.moves.size() > 4);
    CHECK(r.toolpath.moves[0].kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves[0].to.x == Approx(1.0));
    CHECK(r.toolpath.moves[0].to.y == Approx(0.5));
    CHECK(r.toolpath.moves[1].to.x == Approx(2.0));
    CHECK(r.toolpath.moves[1].jetPower == 60.0);
    CHECK(r.toolpath.moves[2].to.x == Approx(3.0));
    CHECK(r.toolpath.moves[2].jetPower == 0.0);
    CHECK(r.toolpath.moves[3].to.x == Approx(4.0));
    CHECK(r.toolpath.moves[3].jetPower == 60.0);
}

TEST_CASE("T-0324: beam power follows the darkness, with power modulation or a threshold", "[toolpath][laser][picture]") {
    LaserPictureParams p;
    p.power = 80.0;
    p.minPower = 20.0;
    CHECK(laserPicturePower(p, 0) == 0.0);
    CHECK(laserPicturePower(p, 255) == Approx(80.0));
    CHECK(laserPicturePower(p, 1) == Approx(20.0 + 60.0 / 255.0));
    CHECK(laserPicturePower(p, 128) == Approx(20.0 + 60.0 * 128.0 / 255.0));
    p.minPower = 0.0;
    CHECK(laserPicturePower(p, 51) == Approx(16.0));
    p.modulatePower = false;
    CHECK(laserPicturePower(p, 0) == 0.0);
    CHECK(laserPicturePower(p, 127) == 0.0);
    CHECK(laserPicturePower(p, 128) == 80.0);
    CHECK(laserPicturePower(p, 255) == 80.0);
    // Out of range values are clamped.
    p.modulatePower = true;
    CHECK(laserPicturePower(p, 1000) == Approx(80.0));
    CHECK(laserPicturePower(p, -5) == 0.0);
}

TEST_CASE("T-0324: darkness is the complement of the gray value, or the gray value when inverted",
          "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 100, 255}});
    CHECK(laserDarkness(g, false, false).pixels == std::vector<std::uint8_t>{255, 155, 0});
    CHECK(laserDarkness(g, true, false).pixels == std::vector<std::uint8_t>{0, 100, 255});
    // Dithering the complement of {0, 100, 255}: 255 dark, 155 -> dark (155 + 0 >= 128), 0.
    CHECK(laserDarkness(g, false, true).pixels == std::vector<std::uint8_t>{255, 255, 0});
}

TEST_CASE("T-0324: each line of pixels becomes feed moves with the pixel's power", "[toolpath][laser][picture]") {
    // Rows from the top: white, two dark columns, mixed, black.
    const GrayImage g = image({{255, 255, 255, 255}, {0, 0, 255, 255}, {0, 128, 128, 0}, {0, 0, 0, 0}});
    LaserPictureParams p = params();
    p.power = 100.0;
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 3);
    REQUIRE(r.toolpath.tool.laser);
    CHECK(r.toolpath.tool.spindleSpeed == 0.0);
    CHECK(r.toolpath.tool.feedRate == 900.0);
    const auto& m = r.toolpath.moves;
    // Line y = 0.5 (bottom row, all black): one feed to x = 4 at full power.
    // Line y = 1.5 (row 2) runs back: 4 -> 3 full, 3 -> 1 at 127/255, 1 -> 0 full.
    // Line y = 2.5 runs forward again: 0 -> 2 at full power. Line y = 3.5 is white: no stroke.
    REQUIRE(m.size() == 1 + 1 + 1 + 3 + 1 + 1 + 1);
    CHECK(m[0].kind == MoveKind::Rapid);
    CHECK(m[0].to.x == Approx(0.0));
    CHECK(m[0].to.y == Approx(0.5));
    CHECK(m[1].kind == MoveKind::Feed);
    CHECK(m[1].to.x == Approx(4.0));
    CHECK(m[1].jetPower == Approx(100.0));
    CHECK(m[2].kind == MoveKind::Rapid);
    CHECK(m[2].to.x == Approx(4.0));
    CHECK(m[2].to.y == Approx(1.5));
    CHECK(m[3].to.x == Approx(3.0));
    CHECK(m[3].jetPower == Approx(100.0));
    CHECK(m[4].to.x == Approx(1.0));
    CHECK(m[4].jetPower == Approx(100.0 * 127.0 / 255.0));
    CHECK(m[5].to.x == Approx(0.0));
    CHECK(m[5].jetPower == Approx(100.0));
    CHECK(m[6].kind == MoveKind::Rapid);
    CHECK(m[6].to.x == Approx(0.0));
    CHECK(m[6].to.y == Approx(2.5));
    CHECK(m[7].to.x == Approx(2.0));
    CHECK(m[7].jetPower == Approx(100.0));
    CHECK(m[8].kind == MoveKind::Rapid);
    CHECK(m[8].to.z == p.safeZ);
    CHECK(m[8].to.x == Approx(2.0));
    for (const Move& mv : m) {
        if (mv.kind == MoveKind::Rapid) {
            CHECK(mv.jetPower == 0.0);
        }
        CHECK(mv.jetPower <= 100.0);
    }
}

TEST_CASE("T-0324: without power modulation a half dark pixel burns at full power", "[toolpath][laser][picture]") {
    const GrayImage g = image({{127, 128, 0}});  // darkness 128, 127, 255
    LaserPictureParams p = params();
    p.modulatePower = false;
    p.power = 70.0;
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    REQUIRE(r.toolpath.moves.size() == 5);
    // Pixel 0 (darkness 128) burns, pixel 1 (127) is a short gap, pixel 2 burns.
    CHECK(r.toolpath.moves[0].to.x == Approx(0.0));
    CHECK(r.toolpath.moves[1].to.x == Approx(1.0));
    CHECK(r.toolpath.moves[1].jetPower == 70.0);
    CHECK(r.toolpath.moves[2].to.x == Approx(2.0));
    CHECK(r.toolpath.moves[2].jetPower == 0.0);
    CHECK(r.toolpath.moves[3].to.x == Approx(3.0));
    CHECK(r.toolpath.moves[3].jetPower == 70.0);
}

TEST_CASE("T-0324: the faintest pixel burns at the minimum power", "[toolpath][laser][picture]") {
    const GrayImage g = image({{254, 0}});  // darkness 1 and 255
    LaserPictureParams p = params();
    p.power = 90.0;
    p.minPower = 30.0;
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    REQUIRE(r.toolpath.moves.size() == 4);
    CHECK(r.toolpath.moves[1].jetPower == Approx(30.0 + 60.0 / 255.0));
    CHECK(r.toolpath.moves[2].to.x == Approx(2.0));
    CHECK(r.toolpath.moves[2].jetPower == Approx(90.0));
}

TEST_CASE("T-0324: inverting burns the light pixels", "[toolpath][laser][picture]") {
    const GrayImage g = image({{255, 0}});
    LaserPictureParams p = params();
    p.invert = true;
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    CHECK(r.toolpath.moves[0].to.x == Approx(0.0));
    CHECK(r.toolpath.moves[1].to.x == Approx(1.0));
    CHECK(r.toolpath.moves[1].jetPower == 100.0);
    CHECK(r.toolpath.moves.size() == 3);  // rapid, one feed, rise
}

TEST_CASE("T-0324: overscan adds beam-off runs before and after each stroke", "[toolpath][laser][picture]") {
    const GrayImage g = image({{255, 0, 0, 255}});
    LaserPictureParams p = params();
    p.overscan = 0.4;
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    const auto& m = r.toolpath.moves;
    REQUIRE(m.size() == 5);
    CHECK(m[0].kind == MoveKind::Rapid);
    CHECK(m[0].to.x == Approx(0.6));
    CHECK(m[1].kind == MoveKind::Feed);
    CHECK(m[1].to.x == Approx(1.0));
    CHECK(m[1].jetPower == 0.0);
    CHECK(m[2].to.x == Approx(3.0));
    CHECK(m[2].jetPower == 100.0);
    CHECK(m[3].kind == MoveKind::Feed);
    CHECK(m[3].to.x == Approx(3.4));
    CHECK(m[3].jetPower == 0.0);
    CHECK(m[4].to.z == p.safeZ);
    CHECK(m[4].to.x == Approx(3.4));
}

TEST_CASE("T-0324: overscan runs along the line's direction on the return line", "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 0}, {0, 0}});  // two black lines: y = 0.5 forward, y = 1.5 back
    LaserPictureParams p = params();
    p.overscan = 1.0;
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    const auto& m = r.toolpath.moves;
    REQUIRE(m.size() == 9);
    CHECK(m[4].kind == MoveKind::Rapid);
    CHECK(m[4].to.x == Approx(3.0));  // start 2, one before it in travel direction (-x): 3
    CHECK(m[4].to.y == Approx(1.5));
    CHECK(m[6].to.x == Approx(0.0));
    CHECK(m[7].to.x == Approx(-1.0));
}

TEST_CASE("T-0324: a beam-off gap of 5 mm splits a line, a shorter one is crossed with the beam off",
          "[toolpath][laser][picture]") {
    std::vector<int> row(11, 255);
    row[0] = 0;
    row[6] = 0;  // gap of 5 pixels = 5 mm: split
    LaserPictureResult r = run(image({row}), params());
    REQUIRE(r.ok());
    CHECK(r.strokes == 2);
    row[6] = 255;
    row[5] = 0;  // gap of 4 mm: crossed
    r = run(image({row}), params());
    REQUIRE(r.ok());
    CHECK(r.strokes == 1);
    // rapid, feed to 1 (on), feed to 5 (off), feed to 6 (on), rise
    REQUIRE(r.toolpath.moves.size() == 5);
    CHECK(r.toolpath.moves[2].to.x == Approx(5.0));
    CHECK(r.toolpath.moves[2].jetPower == 0.0);
}

TEST_CASE("T-0324: beam-off stretches add up from the last burned pixel, not along the whole line",
          "[toolpath][laser][picture]") {
    // Two gaps of 3 mm, each shorter than the split length, with a burned pixel between them: one stroke.
    std::vector<int> row(10, 255);
    row[0] = 0;
    row[4] = 0;
    row[8] = 0;
    const LaserPictureResult r = run(image({row}), params());
    REQUIRE(r.ok());
    CHECK(r.strokes == 1);
}

TEST_CASE("T-0324: a line count that is whole up to rounding is not rounded up", "[toolpath][laser][picture]") {
    // 2.1 / 0.3 is 7.000000000000001 in floating point: still seven lines.
    const GrayImage g = image({{0, 0}, {0, 0}});
    LaserPictureParams p = params();
    p.lineSpacing = 0.3;
    const LaserPictureResult r = laserPictureToolpath(g, Box2({0, 0}, {2.1, 2.1}), beam(), std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 7);
}

TEST_CASE("T-0324: line spacing is shrunk so the lines tile the placement evenly", "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}});
    LaserPictureParams p = params();
    p.lineSpacing = 1.5;  // 4 mm / 1.5 -> 3 lines at pitch 4/3
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 3);
    std::vector<double> ys;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Rapid && m.to.z == 0.0) {
            ys.push_back(m.to.y);
        }
    }
    REQUIRE(ys.size() == 3);
    CHECK(ys[0] == Approx(2.0 / 3.0));
    CHECK(ys[1] == Approx(2.0));
    CHECK(ys[2] == Approx(10.0 / 3.0));
}

TEST_CASE("T-0324: the line angle turns the lines", "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}});
    LaserPictureParams p = params();
    p.lineAngle = 1.5707963267948966;
    const LaserPictureResult r = run(g, p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 4);
    // The first line is at x = 3.5 and runs up (+y).
    CHECK(r.toolpath.moves[0].to.x == Approx(3.5).margin(1e-9));
    CHECK(r.toolpath.moves[0].to.y == Approx(0.0).margin(1e-9));
    CHECK(r.toolpath.moves[1].to.x == Approx(3.5).margin(1e-9));
    CHECK(r.toolpath.moves[1].to.y == Approx(4.0).margin(1e-9));
}

TEST_CASE("T-0324: hatch adds a second pass at right angles", "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}});
    LaserPictureParams p = params();
    p.strategy = PhotoStrategy::Raster;
    const LaserPictureResult one = run(g, p);
    p.strategy = PhotoStrategy::Hatch;
    const LaserPictureResult two = run(g, p);
    REQUIRE(one.ok());
    REQUIRE(two.ok());
    CHECK(one.strokes == 3);
    CHECK(two.strokes == 6);
    // The second pass starts at the first vertical line (x = 2.5, running up).
    std::vector<Move> rapids;
    for (const Move& m : two.toolpath.moves) {
        if (m.kind == MoveKind::Rapid && m.to.z == 0.0) {
            rapids.push_back(m);
        }
    }
    REQUIRE(rapids.size() == 6);
    CHECK(rapids[3].to.x == Approx(2.5).margin(1e-9));
}

TEST_CASE("T-0324: passes repeat the whole path", "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 255, 0}, {0, 0, 0}});
    LaserPictureParams p = params();
    const LaserPictureResult one = run(g, p);
    p.passes = 3;
    const LaserPictureResult three = run(g, p);
    REQUIRE(one.ok());
    REQUIRE(three.ok());
    CHECK(three.strokes == one.strokes);
    CHECK(three.toolpath.moves.size() == 3 * (one.toolpath.moves.size() - 1) + 1);
    const std::size_t each = one.toolpath.moves.size() - 1;
    for (std::size_t k = 0; k < each; ++k) {
        CHECK(three.toolpath.moves[each + k] == one.toolpath.moves[k]);
    }
}

TEST_CASE("T-0324: selected vectors are followed, burning only where the pixels do", "[toolpath][laser][picture]") {
    // Four pixels wide, 2 high; a line along y = 1.5 reaching 1 mm past the placement on each side.
    const GrayImage g = image({{0, 128, 128, 0}, {255, 255, 255, 255}});
    LaserPictureParams p = params();
    p.strategy = PhotoStrategy::SelectedVectors;
    p.power = 100.0;
    const LaserPictureResult r = run(g, p, {line({-1.0, 1.5}, {5.0, 1.5})});
    REQUIRE(r.ok());
    CHECK(r.strokes == 1);
    const auto& m = r.toolpath.moves;
    REQUIRE(m.size() == 5);
    CHECK(m[0].kind == MoveKind::Rapid);
    CHECK(m[0].to.x == Approx(0.0).margin(1e-9));  // the part off the bitmap is not burned
    CHECK(m[1].to.x == Approx(1.0));
    CHECK(m[1].jetPower == 100.0);
    CHECK(m[2].to.x == Approx(3.0));
    CHECK(m[2].jetPower == Approx(100.0 * 127.0 / 255.0));
    CHECK(m[3].to.x == Approx(4.0));
    CHECK(m[3].jetPower == 100.0);
    CHECK(m[4].to.z == p.safeZ);
}

TEST_CASE("T-0324: a closed selected vector is followed all the way round without overscan",
          "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}});
    LaserPictureParams p = params();
    p.strategy = PhotoStrategy::SelectedVectors;
    p.overscan = 2.0;
    const LaserPictureResult r = run(g, p, {Contour::rectangle(Box2({1, 1}, {3, 3}))});
    REQUIRE(r.ok());
    CHECK(r.strokes == 1);
    std::size_t feeds = 0;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Feed) {
            CHECK(m.jetPower == 100.0);
            ++feeds;
        }
    }
    CHECK(feeds == 4);
    // The first move is the rapid to the vector's start, with no overscan point before it.
    CHECK(r.toolpath.moves.front().kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves.front().to.x >= 1.0 - 1e-9);
}

TEST_CASE("T-0324: selected vectors that miss the bitmap or burn nothing are reported", "[toolpath][laser][picture]") {
    const GrayImage g = image({{255, 255}, {255, 255}});
    LaserPictureParams p = params();
    p.strategy = PhotoStrategy::SelectedVectors;
    LaserPictureResult r = run(g, p, {line({0, 1}, {2, 1})});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::NoRegion));
    r = run(g, p, {});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::NoRegion));
    r = run(g, p, {Contour{}});
    CHECK(r.has(ToolpathIssue::EmptyContour));
}

TEST_CASE("T-0324: a blank picture has nothing to burn", "[toolpath][laser][picture]") {
    const LaserPictureResult r = run(image({{255, 255}, {255, 255}}), params());
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::NoRegion));
}

TEST_CASE("T-0324: bad parameters, tools and images are refused with a diagnostic", "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 0}, {0, 0}});
    const auto refused = [&](LaserPictureParams p) {
        const LaserPictureResult r = run(g, p);
        return !r.ok() && r.has(ToolpathIssue::InvalidParameters);
    };
    CHECK(refused([] { auto p = params(); p.power = 0.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.power = 100.5; return p; }()));
    CHECK(refused([] { auto p = params(); p.power = kNan; return p; }()));
    CHECK(refused([] { auto p = params(); p.minPower = -1.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.power = 50.0; p.minPower = 60.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.passes = 0; return p; }()));
    CHECK(refused([] { auto p = params(); p.passes = 1001; return p; }()));
    CHECK(refused([] { auto p = params(); p.overscan = -0.1; return p; }()));
    CHECK(refused([] { auto p = params(); p.lineSpacing = 0.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.lineAngle = kNan; return p; }()));
    CHECK(refused([] { auto p = params(); p.moveSpeed = -1.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.safeZ = 0.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.tolerance.chordal = 0.0; return p; }()));
    // The tool must be an end mill.
    LaserPictureResult r = laserPictureToolpath(g, unitPlacement(g), beam(tooling::ToolType::BallNose), std::nullopt,
                                                params());
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::InvalidTool));
    // The image and placement.
    r = laserPictureToolpath(GrayImage{}, Box2({0, 0}, {2, 2}), beam(), std::nullopt, params());
    CHECK(r.has(ToolpathIssue::InvalidParameters));
    r = laserPictureToolpath(g, Box2({0, 0}, {0, 2}), beam(), std::nullopt, params());
    CHECK(r.has(ToolpathIssue::InvalidParameters));
    r = laserPictureToolpath(g, Box2({0, 0}, {kNan, 2}), beam(), std::nullopt, params());
    CHECK(r.has(ToolpathIssue::InvalidParameters));
    // A tool without a feed rate and no move speed.
    tooling::Tool slow = beam();
    slow.defaultCuttingData.feedRate = 0.0;
    r = laserPictureToolpath(g, unitPlacement(g), slow, std::nullopt, params());
    CHECK_FALSE(r.ok());
}

TEST_CASE("T-0324: the move speed overrides the tool's feed rate", "[toolpath][laser][picture]") {
    LaserPictureParams p = params();
    p.moveSpeed = 1500.0;
    const LaserPictureResult r = run(image({{0, 0}}), p);
    REQUIRE(r.ok());
    CHECK(r.toolpath.tool.feedRate == 1500.0);
    CHECK(r.toolpath.tool.plungeRate == 1500.0);
}

TEST_CASE("T-0324: work and move limits are counted, a cancel flag stops the run", "[toolpath][laser][picture]") {
    const GrayImage g = image({{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}});
    ToolpathRunControl c;
    c.moveLimit = 5;
    LaserPictureResult r = laserPictureToolpath(g, unitPlacement(g), beam(), std::nullopt, params(), {}, c);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    CHECK(r.toolpath.moves.empty());
    c = {};
    c.workLimit = 10;
    r = laserPictureToolpath(g, unitPlacement(g), beam(), std::nullopt, params(), {}, c);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    // Fine line spacing over a large placement is refused up front, not after the work.
    LaserPictureParams fine = params();
    fine.lineSpacing = 0.001;
    r = laserPictureToolpath(g, Box2({0, 0}, {100000, 100000}), beam(), std::nullopt, fine);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    // Passes multiply the move estimate.
    c = {};
    c.moveLimit = 20;
    LaserPictureParams many = params();
    many.passes = 1000;
    r = laserPictureToolpath(g, unitPlacement(g), beam(), std::nullopt, many, {}, c);
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    std::atomic<bool> stop{true};
    c = {};
    c.cancel = &stop;
    r = laserPictureToolpath(g, unitPlacement(g), beam(), std::nullopt, params(), {}, c);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::Cancelled));
}

TEST_CASE("T-0324: the same input gives the same toolpath", "[toolpath][laser][picture]") {
    const GrayImage g = image({{10, 200, 90, 40}, {0, 128, 255, 77}, {255, 3, 3, 3}});
    LaserPictureParams p = params();
    p.lineAngle = 0.3;
    p.strategy = PhotoStrategy::Hatch;
    p.overscan = 0.5;
    const LaserPictureResult a = run(g, p);
    const LaserPictureResult b = run(g, p);
    REQUIRE(a.ok());
    CHECK(a.toolpath.moves == b.toolpath.moves);
    for (const Move& m : a.toolpath.moves) {
        CHECK(m.jetPower >= 0.0);
        CHECK(m.jetPower <= 100.0);
        CHECK(std::isfinite(m.to.x));
    }
}
