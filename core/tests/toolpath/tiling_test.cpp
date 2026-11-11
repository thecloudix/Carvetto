// Toolpath tiling (T-0242, ADR-0043).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

#include "carvetto/toolpath/tiling.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;
using Catch::Approx;

namespace {

Toolpath line(double x0, double y0, double x1, double y1, double z = -1.0) {
    Toolpath tp;
    tp.name = "t";
    tp.append(Move::rapid({x0, y0, 5.0}));
    tp.append(Move::plunge({x0, y0, z}));
    tp.append(Move::feed({x1, y1, z}));
    tp.append(Move::retract({x1, y1, 5.0}));
    return tp;
}

double cutLength(const Toolpath& tp) {
    double sum = 0.0;
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        if (m.kind == MoveKind::Feed || isArc(m.kind)) {
            sum += moveLength(m, tp.startOf(i));
        }
    }
    return sum;
}

double feedAndArcLength(const TilingResult& r) {
    double sum = 0.0;
    for (const Tile& t : r.tiles) {
        for (const Toolpath& tp : t.toolpaths) {
            sum += cutLength(tp);
        }
    }
    return sum;
}

}  // namespace

TEST_CASE("tileRects: grid with overlap, trimmed to the extent", "[tiling]") {
    TileSpec s;
    s.width = 100;
    s.height = 50;
    s.overlap = 10;
    const auto r = tileRects(Box2{{0, 0}, {250, 80}}, s);
    // x: starts 0, 90, 180 (180..250 trimmed); y: starts 0, 40 (40..80)
    REQUIRE(r.size() == 6);
    CHECK(r[0] == Box2{{0, 0}, {100, 50}});
    CHECK(r[1] == Box2{{90, 0}, {190, 50}});
    CHECK(r[2] == Box2{{180, 0}, {250, 50}});
    CHECK(r[3] == Box2{{0, 40}, {100, 80}});
    // a job that fits needs one tile
    CHECK(tileRects(Box2{{0, 0}, {100, 50}}, s).size() == 1);
    // one more mm needs a second tile
    CHECK(tileRects(Box2{{0, 0}, {101, 50}}, s).size() == 2);
    // exactly covered by overlapped tiles: 90 + 100 = 190
    CHECK(tileRects(Box2{{0, 0}, {190, 50}}, s).size() == 2);
}

TEST_CASE("tileRects: smallest first and feed-through strips", "[tiling]") {
    TileSpec s;
    s.width = 100;
    s.height = 100;
    s.smallestFirst = true;
    const auto r = tileRects(Box2{{0, 0}, {250, 100}}, s);
    REQUIRE(r.size() == 3);
    CHECK(r[0].width() == Approx(50));
    CHECK(r[2].width() == Approx(100));

    TileSpec x;
    x.strategy = TileStrategy::FeedThroughX;
    x.width = 100;
    const auto rx = tileRects(Box2{{0, 0}, {250, 700}}, x);
    REQUIRE(rx.size() == 3);
    CHECK(rx[1].min.y == 0.0);
    CHECK(rx[1].max.y == 700.0);
    CHECK(rx[1].min.x == 100.0);

    TileSpec y;
    y.strategy = TileStrategy::FeedThroughY;
    y.height = 100;
    y.overlap = 20;
    const auto ry = tileRects(Box2{{0, 0}, {900, 250}}, y);
    REQUIRE(ry.size() == 3);  // starts 0, 80, 160
    CHECK(ry[2].min.y == Approx(160));
    CHECK(ry[2].max.x == 900.0);
}

TEST_CASE("tileRects: invalid specs", "[tiling]") {
    std::string err;
    TileSpec s;
    CHECK(tileRects(Box2{{0, 0}, {10, 10}}, s, &err).empty());
    CHECK_FALSE(err.empty());
    s.width = 10;
    s.height = 10;
    s.overlap = 10;
    CHECK(tileRects(Box2{{0, 0}, {10, 10}}, s, &err).empty());
    CHECK_FALSE(err.empty());
    s.overlap = -1;
    CHECK(tileRects(Box2{{0, 0}, {10, 10}}, s, &err).empty());
    s.overlap = 0;
    CHECK(tileRects(Box2{{0, 0}, {10, 10}}, s, &err).size() == 1);
    CHECK(err.empty());
    s.width = 1e-6;
    s.height = 1e-6;
    CHECK(tileRects(Box2{{0, 0}, {10, 10}}, s, &err).empty());
    CHECK(err == "too many tiles");
    // a feed-through strip does not need the unused dimension
    TileSpec f;
    f.strategy = TileStrategy::FeedThroughX;
    f.width = 5;
    CHECK(tileRects(Box2{{0, 0}, {10, 10}}, f, &err).size() == 2);
    CHECK(tileRects(Box2{}, f, &err).empty());
}

TEST_CASE("clipToolpath: a line crossing the rectangle is cut at the edges with Z interpolated", "[tiling]") {
    Toolpath tp;
    tp.append(Move::rapid({-10, 5, 5}));
    tp.append(Move::plunge({-10, 5, -2}));
    tp.append(Move::feed({30, 5, -4}));
    tp.append(Move::retract({30, 5, 5}));
    const Toolpath c = clipToolpath(tp, Box2{{0, 0}, {20, 10}}, 5.0);
    // rapid, plunge to entry, feed, retract
    REQUIRE(c.moves.size() == 4);
    CHECK(c.moves[0].kind == MoveKind::Rapid);
    CHECK(c.moves[0].to == Vec3{0, 5, 5});
    CHECK(c.moves[1].kind == MoveKind::Plunge);
    CHECK(c.moves[1].to.z == Approx(-2.5));
    CHECK(c.moves[2].kind == MoveKind::Feed);
    CHECK(c.moves[2].to.x == Approx(20));
    CHECK(c.moves[2].to.z == Approx(-3.5));
    CHECK(c.moves[3].kind == MoveKind::Retract);
    CHECK(c.moves[3].to.z == 5.0);
}

TEST_CASE("clipToolpath: moves outside give nothing, inside are kept verbatim", "[tiling]") {
    const Toolpath tp = line(2, 2, 8, 2);
    CHECK(clipToolpath(tp, Box2{{50, 50}, {60, 60}}, 5.0).isEmpty());
    const Toolpath c = clipToolpath(tp, Box2{{0, 0}, {10, 10}}, 5.0);
    REQUIRE(c.moves.size() == 4);
    CHECK(c.moves[1] == tp.moves[1]);
    CHECK(c.moves[2] == tp.moves[2]);
    CHECK(c.moves[3] == tp.moves[3]);
    CHECK(c.name == "t");
}

TEST_CASE("clipToolpath: a line that only touches a corner is dropped", "[tiling]") {
    Toolpath tp;
    tp.append(Move::rapid({-5, 5, 5}));
    tp.append(Move::plunge({-5, 5, -1}));
    tp.append(Move::feed({5, -5, -1}));
    CHECK(clipToolpath(tp, Box2{{0, 0}, {10, 10}}, 5.0).isEmpty());
}

TEST_CASE("clipToolpath: pieces in one tile are relinked at safe Z", "[tiling]") {
    // A U shape: two vertical legs inside the rectangle joined outside it.
    Toolpath tp;
    tp.append(Move::rapid({5, 5, 5}));
    tp.append(Move::plunge({5, 5, -1}));
    tp.append(Move::feed({5, 30, -1}));
    tp.append(Move::feed({15, 30, -1}));
    tp.append(Move::feed({15, 5, -1}));
    const Toolpath c = clipToolpath(tp, Box2{{0, 0}, {20, 20}}, 5.0);
    // leg 1 (plunge, feed to y=20), retract, rapid, plunge, leg 2 (feed to y=5), retract
    REQUIRE(c.moves.size() == 8);
    CHECK(c.moves[0].kind == MoveKind::Rapid);
    CHECK(c.moves[1].kind == MoveKind::Plunge);
    CHECK(c.moves[2].kind == MoveKind::Feed);
    CHECK(c.moves[2].to.y == Approx(20));
    CHECK(c.moves[3].kind == MoveKind::Retract);
    CHECK(c.moves[3].to == Vec3{5, 20, 5});
    CHECK(c.moves[4].kind == MoveKind::Rapid);
    CHECK(c.moves[4].to == Vec3{15, 20, 5});
    CHECK(c.moves[5].kind == MoveKind::Plunge);
    CHECK(c.moves[5].to == Vec3{15, 20, -1});
    CHECK(c.moves[6].kind == MoveKind::Feed);
    CHECK(c.moves[6].to.y == Approx(5));
    CHECK(c.moves[7].kind == MoveKind::Retract);
}

TEST_CASE("clipToolpath: arcs are split at edges, helix Z interpolated", "[tiling]") {
    // Full circle radius 10 about the origin, starting at (10,0), CCW, Z 0 -> -4 (helix).
    Toolpath tp;
    tp.append(Move::rapid({10, 0, 5}));
    tp.append(Move::plunge({10, 0, 0}));
    tp.append(Move::arcCCW({10, 0, -4}, {0, 0}));
    // Right half plane x >= 0: the arc enters at (0,-10) (t = .75) and leaves at (0,10) (t = .25).
    const Toolpath c = clipToolpath(tp, Box2{{0, -20}, {20, 20}}, 5.0);
    // two pieces: [0, .25] and [.75, 1]
    std::vector<Move> arcs;
    std::vector<std::size_t> arcIdx;
    for (std::size_t k = 0; k < c.moves.size(); ++k) {
        if (isArc(c.moves[k].kind)) {
            arcs.push_back(c.moves[k]);
            arcIdx.push_back(k);
        }
    }
    REQUIRE(arcs.size() == 2);
    CHECK(arcs[0].to.x == Approx(0).margin(1e-9));
    CHECK(arcs[0].to.y == Approx(10));
    CHECK(arcs[0].to.z == Approx(-1));
    CHECK(arcs[1].to.x == Approx(10));
    CHECK(arcs[1].to.z == Approx(-4));
    // the second piece starts at (0,-10) with z = -3
    const Vec3 start = c.startOf(arcIdx[1]);
    CHECK(start.x == Approx(0).margin(1e-9));
    CHECK(start.y == Approx(-10));
    CHECK(start.z == Approx(-3));
    // arc length preserved
    CHECK(cutLength(c) == Approx(0.5 * 2 * carvetto::geometry::kPi * 10 * 1.0).epsilon(0.1));
}

TEST_CASE("clipToolpath: clockwise arc and arc wholly inside are kept", "[tiling]") {
    Toolpath tp;
    tp.append(Move::rapid({0, 10, 5}));
    tp.append(Move::plunge({0, 10, -1}));
    tp.append(Move::arcCW({10, 0, -1}, {0, 0}));  // quarter, CW
    const Toolpath whole = clipToolpath(tp, Box2{{-1, -1}, {11, 11}}, 5.0);
    REQUIRE(whole.moves.size() == 4);
    CHECK(whole.moves[2] == tp.moves[2]);
    // cut at x = 5: only the part with x >= 5 (CW from the top: starts at x=0, so enters at x=5)
    const Toolpath part = clipToolpath(tp, Box2{{5, -1}, {11, 11}}, 5.0);
    REQUIRE(part.moves.size() == 4);
    CHECK(part.moves[2].kind == MoveKind::ArcCW);
    CHECK(part.moves[1].to.x == Approx(5));
    CHECK(part.moves[1].to.y == Approx(std::sqrt(75.0)));
    CHECK(part.moves[2].to == Vec3{10, 0, -1});
    CHECK(part.moves[2].centre == Vec2{0, 0});
}

TEST_CASE("clipToolpath: dwell kept in its tile only", "[tiling]") {
    Toolpath tp;
    tp.append(Move::rapid({5, 5, 5}));
    tp.append(Move::plunge({5, 5, -1}));
    tp.append(Move::dwell({5, 5, -1}, 2.0));
    CHECK(clipToolpath(tp, Box2{{0, 0}, {10, 10}}, 5.0).count(MoveKind::Dwell) == 1);
    CHECK(clipToolpath(tp, Box2{{6, 0}, {10, 10}}, 5.0).isEmpty());
}

TEST_CASE("tileToolpaths: the tiles together cut the same length as the whole job", "[tiling]") {
    Toolpath tp;
    tp.name = "box";
    tp.append(Move::rapid({0, 0, 5}));
    tp.append(Move::plunge({0, 0, -1}));
    tp.append(Move::feed({250, 0, -1}));
    tp.append(Move::feed({250, 130, -1}));
    tp.append(Move::arcCCW({50, 130, -1}, {150, 130}));
    tp.append(Move::feed({0, 0, -1}));
    tp.append(Move::retract({0, 0, 5}));
    const double whole = cutLength(tp);

    TileSpec s;
    s.width = 100;
    s.height = 100;
    const TilingResult r0 = tileToolpaths({tp}, s);
    REQUIRE(r0.ok());
    CHECK(r0.tiles.size() > 1);
    CHECK(feedAndArcLength(r0) == Approx(whole).epsilon(1e-6));  // no overlap: no cut is doubled

    s.overlap = 10;
    const TilingResult r1 = tileToolpaths({tp}, s);
    REQUIRE(r1.ok());
    CHECK(feedAndArcLength(r1) >= whole - 1e-6);

    for (const Tile& t : r1.tiles) {
        CHECK(tileFitsWorkArea(t, 100, 100));
        // local coordinates: all within the tile size and Z untouched
        const Box2 b = t.toolpaths[0].bounds();
        CHECK(b.min.x >= -1e-6);
        CHECK(b.max.x <= 100 + 1e-6);
        CHECK(b.max.y <= 100 + 1e-6);
    }
    CHECK_FALSE(tileFitsWorkArea(r1.tiles[0], 50, 0));
    CHECK(tileFitsWorkArea(r1.tiles[0], 0, 0));
}

TEST_CASE("tileToolpaths: original position keeps coordinates, local shifts them", "[tiling]") {
    Toolpath tp = line(150, 20, 190, 20);
    tp.append(Move::rapid({0, 0, 5}));
    TileSpec s;
    s.width = 100;
    s.height = 100;
    s.originalPosition = true;
    // extent is from the toolpath (150..190 x 20): one tile
    const TilingResult a = tileToolpaths({tp}, s, Box2{{0, 0}, {200, 100}});
    REQUIRE(a.tiles.size() == 1);  // the second tile (100..200) only
    CHECK(a.tiles[0].column == 1);
    CHECK(a.tiles[0].rect.min.x == 100.0);
    CHECK(a.tiles[0].toolpaths[0].moves[1].to.x == 150.0);
    s.originalPosition = false;
    const TilingResult b = tileToolpaths({tp}, s, Box2{{0, 0}, {200, 100}});
    REQUIRE(b.tiles.size() == 1);
    CHECK(b.tiles[0].toolpaths[0].moves[1].to.x == 50.0);
    CHECK(b.tiles[0].toolpaths[0].moves[1].to.y == 20.0);
}

TEST_CASE("tileToolpaths: several toolpaths share the grid; smallest first order; errors", "[tiling]") {
    const Toolpath a = line(5, 5, 95, 5);
    const Toolpath b = line(105, 5, 245, 60);
    TileSpec s;
    s.width = 100;
    s.height = 100;
    s.smallestFirst = true;
    const TilingResult r = tileToolpaths({a, b}, s);
    REQUIRE(r.ok());
    REQUIRE(r.tiles.size() == 3);
    // extent 5..245: tiles 5..105, 105..205, 205..245 ; smallest (40 wide) first
    CHECK(r.tiles[0].rect.width() == Approx(40));
    CHECK(r.tiles[0].toolpaths[0].isEmpty());
    CHECK_FALSE(r.tiles[0].toolpaths[1].isEmpty());
    CHECK(r.tiles[0].column == 2);

    TileSpec bad;
    const TilingResult e = tileToolpaths({a}, bad);
    CHECK_FALSE(e.ok());
    CHECK(e.tiles.empty());
    CHECK(tileToolpaths({}, s).tiles.empty());
}

TEST_CASE("tileToolpaths: explicit safe Z is the link height", "[tiling]") {
    TileSpec s;
    s.width = 1000;
    s.height = 1000;
    s.safeZ = 12.0;
    const TilingResult r = tileToolpaths({line(0, 0, 10, 0)}, s);
    REQUIRE(r.tiles.size() == 1);
    CHECK(r.tiles[0].toolpaths[0].moves.front().to.z == 12.0);
    CHECK(r.tiles[0].toolpaths[0].moves.back().to.z == 12.0);
}
