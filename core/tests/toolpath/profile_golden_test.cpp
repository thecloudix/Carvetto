// Golden files for the profile toolpath (T-0120 AC1 and AC6): the move list of the AC1 case and
// its G-code through the shipped Grbl post. Regenerate only with tools/regen_golden.py and
// hand-check the result (see the hand-check notes in ADR-0016).

#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;

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

void writeFile(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    REQUIRE(out);
    out << text;
}

const std::string kMovesGolden = std::string(CARVETTO_GOLDEN_DIR) + "/toolpath/profile_rect_tabs.moves";
const std::string kGrblGolden = std::string(CARVETTO_GOLDEN_DIR) + "/toolpath/profile_rect_tabs_grbl.nc";

/// AC1: outside profile of a 100x50 rectangle at the origin, 6 mm end mill, 12 mm deep in three
/// 4 mm passes, two tabs 10 mm long and 2 mm thick, climb, start at the vector start point.
Toolpath ac1Toolpath() {
    ProfileParams p;
    p.name = "Profile 100x50";
    p.passes.cutDepth = 12.0;
    p.passes.passDepth = 4.0;
    p.safeZ = 5.0;
    p.tabs.enabled = true;
    p.tabs.count = 2;
    p.tabs.length = 10.0;
    p.tabs.thickness = 2.0;
    const geometry::Contour rect = geometry::Contour::rectangle(geometry::Box2{{0, 0}, {100, 50}});
    const ProfileResult r = profileToolpath({rect}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    REQUIRE(r.diagnostics.empty());
    return r.toolpath;
}

std::string ac1Grbl() {
    auto pp = post::PostProcessor::parse(readFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp"));
    REQUIRE(pp.ok());
    post::PostContext ctx;
    ctx.fileName = "profile_rect_tabs";
    ctx.home = {0, 0, 20};
    ctx.safeZ = 5.0;
    const auto out = pp.value().run({ac1Toolpath()}, ctx);
    REQUIRE(out.ok());
    REQUIRE(out.value().warnings.empty());
    return out.value().text;
}

}  // namespace

TEST_CASE("AC1: profile move list matches the hand-checked golden", "[toolpath][profile][golden]") {
    CHECK(test::formatMoves(ac1Toolpath()) == readFile(kMovesGolden));
}

TEST_CASE("AC6: profile G-code through the Grbl post matches the golden", "[toolpath][profile][golden][post]") {
    CHECK(ac1Grbl() == readFile(kGrblGolden));
}

// Hidden (tag [.regen], never run by ctest): rewrites both goldens. Run via tools/regen_golden.py
// and hand-check the diff before committing.
TEST_CASE("Regenerate profile goldens", "[.regen]") {
    writeFile(kMovesGolden, test::formatMoves(ac1Toolpath()));
    writeFile(kGrblGolden, ac1Grbl());
}
