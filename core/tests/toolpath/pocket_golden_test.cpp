// Golden G-code for the T-0121 AC1 pocket (50x30 rectangle, 6 mm end mill, 40 % stepover,
// offset strategy, climb, 6 mm deep in two 3 mm passes) through the shipped Grbl post.
//
// The golden core/tests/golden/toolpath/pocket_rect_grbl.nc is regenerated only through
// tools/regen_golden.py (hidden [.regen] test below) and was hand-checked as described in
// ADR-0017 ("Golden file"). The structural test re-derives the same move list by hand
// (independently of the offset code) so a regenerated golden cannot silently drift.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_test_util.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

Toolpath ac1Pocket() {
    const PocketTool t = carvetto::test::pocket::endMill("End Mill (6 mm)", 1, 6.0, 2.4, 3.0);
    PocketParams p;
    p.name = "Pocket 1";
    p.cutDepth = 6.0;
    p.safeZ = 5.0;
    const PocketResult r = pocketToolpath({Contour::rectangle(Box2{Vec2{0, 0}, Vec2{50, 30}})}, {t}, p);
    REQUIRE(r.toolpaths.size() == 1);
    REQUIRE(r.diagnostics.empty());
    return r.toolpaths[0];
}

std::string postGrbl(const Toolpath& tp) {
    auto pp = carvetto::post::PostProcessor::parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp"));
    REQUIRE(pp.ok());
    carvetto::post::PostContext ctx;
    ctx.fileName = "pocket_rect_grbl";
    ctx.home = {0, 0, 20};
    ctx.safeZ = 5.0;
    const auto out = pp.value().run({tp}, ctx);
    REQUIRE(out.ok());
    CHECK(out.value().warnings.empty());
    return out.value().text;
}

}  // namespace

TEST_CASE("AC6: AC1 pocket through the Grbl post matches the golden", "[toolpath][pocket][golden]") {
    const std::string golden = readFile(std::string(CARVETTO_GOLDEN_DIR) + "/toolpath/pocket_rect_grbl.nc");
    CHECK(postGrbl(ac1Pocket()) == golden);
}

TEST_CASE("AC6: the AC1 pocket move list equals its hand derivation", "[toolpath][pocket][golden]") {
    // Centre region: the rectangle inset by r + margin = 3 + 0.01 -> [3.01, 46.99] x [3.01, 26.99]
    // (half-height 11.99). Rings at insets k*2.4, k = 0..4 (k = 5 would need 12 > 11.99); stepover
    // 2.4 <= r, so no extra rings. Cut inside out, counter-clockwise (climb). The innermost ring
    // starts at its lowest-x, lowest-y vertex; each next ring starts at its point nearest to the
    // current position: straight down by 2.4 (the foot on the bottom edge). Level 1 (Z-3) ends on
    // the outer ring at (12.61, 3.01); the way back to the innermost start (12.61, 12.61) is
    // inside the centre region, so the tool feeds there at Z-3 and plunges to Z-6 — one retract.
    const double z1 = -3.0, z2 = -6.0, safe = 5.0;
    const double lo = 3.01, s = 2.4;
    auto ringMoves = [&](int k, double z, std::vector<Move>& out) {
        const double x0 = lo + k * s, y0 = lo + k * s, x1 = 50 - lo - k * s, y1 = 30 - lo - k * s;
        const double sx = lo + 4 * s;  // every ring starts at x = 12.61 (bottom edge)
        // Innermost ring (k = 4): starts at its own corner (12.61, 12.61).
        out.push_back(Move::feed(Vec3{x1, y0, z}));
        out.push_back(Move::feed(Vec3{x1, y1, z}));
        out.push_back(Move::feed(Vec3{x0, y1, z}));
        out.push_back(Move::feed(Vec3{x0, y0, z}));
        if (k != 4) {
            out.push_back(Move::feed(Vec3{sx, y0, z}));
        }
    };
    std::vector<Move> expect;
    const Vec2 start{lo + 4 * s, lo + 4 * s};
    expect.push_back(Move::rapid(Vec3{start, safe}));
    for (double z : {z1, z2}) {
        if (z == z2) {
            expect.push_back(Move::feed(Vec3{start, z1}));  // back to the innermost start at Z-3
        }
        expect.push_back(Move::plunge(Vec3{start, z}));
        for (int k = 4; k >= 0; --k) {
            if (k != 4) {
                expect.push_back(Move::feed(Vec3{lo + 4 * s, lo + k * s, z}));  // step down 2.4
            }
            ringMoves(k, z, expect);
        }
    }
    expect.push_back(Move::retract(Vec3{lo + 4 * s, lo, safe}));

    const Toolpath tp = ac1Pocket();
    REQUIRE(tp.moves.size() == expect.size());
    for (std::size_t i = 0; i < expect.size(); ++i) {
        INFO("move " << i);
        CHECK(tp.moves[i].kind == expect[i].kind);
        CHECK(std::fabs(tp.moves[i].to.x - expect[i].to.x) <= 1e-6);  // Clipper2's 1e-6 mm grid
        CHECK(std::fabs(tp.moves[i].to.y - expect[i].to.y) <= 1e-6);
        CHECK(tp.moves[i].to.z == expect[i].to.z);
    }
    CHECK(tp.count(MoveKind::Retract) == 1);
}

// Hidden test (tag [.regen], never run by ctest): rewrites pocket_rect_grbl.nc. Run through
// tools/regen_golden.py and hand-review the result before committing it.
TEST_CASE("Regenerate pocket_rect_grbl golden", "[.regen]") {
    std::ofstream out(std::string(CARVETTO_GOLDEN_DIR) + "/toolpath/pocket_rect_grbl.nc", std::ios::binary);
    REQUIRE(out);
    out << postGrbl(ac1Pocket());
}
