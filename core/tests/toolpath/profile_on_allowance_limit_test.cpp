// T-0137: Profile On the vector refuses an allowance above the tool radius too (owner decision
// 2026-11-02, ADR-0016 §4). On the vector the allowance is the tool centre's distance from it, so
// above +r the tool runs entirely off the vector, just as below -r (T-0129). Inside/Outside and
// open Left/Right keep only the -r limit: a large positive allowance there is an oversize cut.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

std::string mm(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr) + " mm";
}

tooling::Tool vbit60() {
    tooling::Tool v = test::endMill(12.0);
    v.type = tooling::ToolType::VBit;
    v.includedAngle = geometry::kPi / 3.0;  // 60 deg: radius at depth 3 is 3 tan 30 deg
    return v;
}

ProfileParams params(ProfileSide side, bool separate) {
    ProfileParams p;
    p.side = side;
    p.passes.cutDepth = 3.0;
    p.passes.passDepth = 1.5;
    p.safeZ = 5.0;
    p.separateLastPass = separate;
    p.lastPassAllowance = 0.2;
    return p;
}

std::vector<Contour> closedScene() {
    return {Contour::rectangle(Box2{{0, 0}, {40, 30}}), Contour::circle({80, 15}, 12)};
}

std::vector<Contour> openScene() {
    Contour c;
    c.append(LineSpan{{0, 0}, {30, 0}});
    c.append(LineSpan{{30, 0}, {30, 20}});
    c.append(LineSpan{{30, 20}, {60, 20}});
    return {c};
}

/// The single refusal diagnostic: InvalidParameters, no vector, naming the limit.
void checkRefusedAbove(const ProfileResult& r, double radius) {
    CHECK_FALSE(r.ok());
    CHECK(r.toolpath.moves.empty());
    REQUIRE(r.diagnostics.size() == 1);
    const ToolpathDiagnostic& d = r.diagnostics.front();
    CHECK(d.issue == ProfileIssue::InvalidParameters);
    CHECK_FALSE(d.contour.has_value());
    CHECK_FALSE(d.tool.has_value());
    CHECK_THAT(d.message, ContainsSubstring("above the tool radius (" + mm(radius) + " at the cut depth)"));
    CHECK_THAT(d.message, ContainsSubstring("run entirely off the vector; nothing is cut"));
}

struct NamedTool {
    const char* name;
    tooling::Tool tool;
};

std::vector<NamedTool> tools() {
    return {{"end mill 6", test::endMill(6.0)}, {"V-bit 60", vbit60()}};
}

}  // namespace

TEST_CASE("T-0137 AC1: On refuses an allowance above the tool radius, exactly +r is cut",
          "[toolpath][profile][allowance]") {
    for (const bool open : {false, true}) {
        const std::vector<Contour> cs = open ? openScene() : closedScene();
        for (const NamedTool& t : tools()) {
            for (const bool separate : {false, true}) {
                ProfileParams p = params(ProfileSide::On, separate);
                const double r = profileToolRadius(t.tool, p.passes);
                INFO((open ? "open" : "closed") << ", " << t.name << ", separate " << separate << ", r " << mm(r));

                // Exactly +r stays valid without a separate last pass; with one the roughing sits
                // at r + 0.2 and is refused, so the finishing allowance moves down to keep it at r.
                p.allowance = separate ? r - p.lastPassAllowance : r;
                const ProfileResult exact = profileToolpath(cs, t.tool, p);
                CHECK(exact.ok());
                CHECK_FALSE(exact.has(ProfileIssue::InvalidParameters));
                CHECK_FALSE(exact.toolpath.moves.empty());

                // One ulp, 1e-9 and 1 mm above +r are refused.
                for (const double above : {std::nextafter(r, kInf), r + 1e-9, r + 1.0}) {
                    p.allowance = above;
                    INFO("allowance " << mm(above));
                    const ProfileResult res = profileToolpath(cs, t.tool, p);
                    checkRefusedAbove(res, r);
                    if (res.diagnostics.empty()) {
                        continue;
                    }
                    const std::string& msg = res.diagnostics.front().message;
                    CHECK(mm(above) != mm(r));  // never prints the same as the limit
                    if (!separate) {
                        CHECK_THAT(msg, ContainsSubstring("allowance " + mm(above) + " is above"));
                        CHECK_THAT(msg, !ContainsSubstring("last pass"));
                    } else {
                        // Both passes are above: the message names both distances.
                        CHECK_THAT(msg, ContainsSubstring("allowance " + mm(above) + " and allowance + last pass allowance " +
                                                          mm(above + p.lastPassAllowance) + " are above"));
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0137 AC1: On refuses a roughing distance above the tool radius", "[toolpath][profile][allowance]") {
    // With a separate last pass the roughing runs at allowance + last-pass allowance: it can be
    // the only pass off the vector. Refused as a whole; the boundary is the exact double sum.
    for (const bool open : {false, true}) {
        const std::vector<Contour> cs = open ? openScene() : closedScene();
        for (const NamedTool& t : tools()) {
            ProfileParams p = params(ProfileSide::On, true);
            const double r = profileToolRadius(t.tool, p.passes);
            INFO((open ? "open" : "closed") << ", " << t.name << ", r " << mm(r));
            // Finishing exactly at +r with a zero last-pass allowance: both passes at r, valid.
            p.allowance = r;
            p.lastPassAllowance = 0.0;
            CHECK(profileToolpath(cs, t.tool, p).ok());
            // Finishing at r, roughing above: refused, naming the roughing distance.
            for (const double lpa : {std::nextafter(0.0, kInf), 1e-9, 0.3}) {
                p.allowance = r;
                p.lastPassAllowance = lpa;
                const double rough = r + lpa;
                INFO("last pass allowance " << mm(lpa));
                if (!(rough > r)) {
                    continue;  // a denormal lpa vanishes in the sum: identical to lpa 0
                }
                const ProfileResult res = profileToolpath(cs, t.tool, p);
                checkRefusedAbove(res, r);
                REQUIRE(res.diagnostics.size() == 1);
                const std::string& msg = res.diagnostics.front().message;
                CHECK_THAT(msg, ContainsSubstring("allowance + last pass allowance " + mm(rough) + " is above"));
                CHECK_THAT(msg, ContainsSubstring("the roughing would run entirely off the vector"));
                CHECK_THAT(msg, ContainsSubstring("the last pass at allowance " + mm(r) + " is not cut on its own"));
            }
            // A negative finishing allowance with a large last-pass allowance: the roughing crosses
            // to the far side of the vector and beyond +r. Refused too.
            p.allowance = -r;
            p.lastPassAllowance = 2.0 * r + 0.5;
            checkRefusedAbove(profileToolpath(cs, t.tool, p), r);
            // Roughing exactly at +r from a negative finishing allowance: valid.
            p.allowance = -1.0;
            p.lastPassAllowance = r + 1.0;
            if (p.allowance + p.lastPassAllowance <= r) {
                CHECK(profileToolpath(cs, t.tool, p).ok());
            }
        }
    }
}

TEST_CASE("T-0137 AC1: Inside/Outside and open Left/Right have no upper allowance limit",
          "[toolpath][profile][allowance]") {
    // A positive allowance on a sided profile is an oversize cut: the tool stays on its side of the
    // vector, so no allowance above +r is refused there (owner decision 2026-11-02).
    for (const bool open : {false, true}) {
        const std::vector<Contour> cs = open ? openScene() : closedScene();
        for (const ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
            for (const NamedTool& t : tools()) {
                for (const bool separate : {false, true}) {
                    ProfileParams p = params(side, separate);
                    const double r = profileToolRadius(t.tool, p.passes);
                    INFO((open ? "open" : "closed") << ", side " << static_cast<int>(side) << ", " << t.name
                                                    << ", separate " << separate);
                    for (const double above : {r, std::nextafter(r, kInf), r + 1e-9, r + 1.0}) {
                        p.allowance = above;
                        const ProfileResult res = profileToolpath(cs, t.tool, p);
                        CHECK_FALSE(res.has(ProfileIssue::InvalidParameters));
                        // Inside a closed scene a large allowance may leave nothing to cut
                        // (reported as too small); Outside and open vectors always cut.
                        if (open || side == ProfileSide::Outside) {
                            CHECK(res.ok());
                            CHECK_FALSE(res.toolpath.moves.empty());
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0137: the On upper limit follows the V-bit radius at the cut depth", "[toolpath][profile][allowance]") {
    const std::vector<Contour> cs = closedScene();
    ProfileParams p = params(ProfileSide::On, false);
    p.allowance = 2.5;
    p.passes.cutDepth = 3.0;
    const double r3 = profileToolRadius(vbit60(), p.passes);
    REQUIRE(r3 < 2.5);
    checkRefusedAbove(profileToolpath(cs, vbit60(), p), r3);
    p.passes.cutDepth = 6.0;
    REQUIRE(profileToolRadius(vbit60(), p.passes) > 2.5);
    CHECK(profileToolpath(cs, vbit60(), p).ok());
    // The start depth counts: the radius is taken at start + cut depth.
    p.passes.cutDepth = 3.0;
    p.passes.startDepth = 3.0;
    CHECK(profileToolpath(cs, vbit60(), p).ok());
}

TEST_CASE("T-0137: the -r limit still decides first On, and both limits are symmetric",
          "[toolpath][profile][allowance]") {
    const std::vector<Contour> cs = closedScene();
    const tooling::Tool tool = test::endMill(6.0);
    ProfileParams p = params(ProfileSide::On, false);
    const double r = profileToolRadius(tool, p.passes);
    for (const double a : {-r, -r / 2.0, 0.0, r / 2.0, r}) {
        p.allowance = a;
        INFO("allowance " << mm(a));
        CHECK(profileToolpath(cs, tool, p).ok());
    }
    p.allowance = -r - 1.0;
    const ProfileResult below = profileToolpath(cs, tool, p);
    REQUIRE(below.diagnostics.size() == 1);
    CHECK_THAT(below.diagnostics.front().message, ContainsSubstring("is below minus the tool radius"));
    // Finishing below -r and roughing above +r: the T-0129 message (finishing crosses) wins.
    p.separateLastPass = true;
    p.lastPassAllowance = 2.0 * r + 3.0;
    const ProfileResult both = profileToolpath(cs, tool, p);
    REQUIRE(both.diagnostics.size() == 1);
    CHECK_THAT(both.diagnostics.front().message, ContainsSubstring("the last pass would cross the vector"));
}
