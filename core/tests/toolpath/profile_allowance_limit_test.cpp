// T-0129: Profile refuses an allowance below minus the tool radius (owner decision 2026-11-01,
// ADR-0016 §4). Below -r the tool centre crosses the vector, so Outside would cut like Inside (and
// the reverse) and On, Left and Right would swap sides.

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
using carvetto::geometry::Vec2;
using Catch::Matchers::ContainsSubstring;

namespace {

std::string mm(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr) + " mm";
}

tooling::Tool vbit60() {
    tooling::Tool v = test::endMill(12.0);
    v.type = tooling::ToolType::VBit;
    v.includedAngle = geometry::kPi / 3.0;  // 60 deg: radius at depth 3 is 3·tan 30° = 1.7320508…
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

const char* sideName(ProfileSide s) {
    switch (s) {
        case ProfileSide::Outside:
            return "Outside/Right";
        case ProfileSide::Inside:
            return "Inside/Left";
        case ProfileSide::On:
            return "On";
    }
    return "?";
}

/// The single refusal diagnostic: InvalidParameters, no vector, naming the allowance and the limit.
void checkRefused(const ProfileResult& r, const ProfileParams& p, double radius) {
    CHECK_FALSE(r.ok());
    CHECK(r.toolpath.moves.empty());
    REQUIRE(r.diagnostics.size() == 1);
    const ToolpathDiagnostic& d = r.diagnostics.front();
    CHECK(d.issue == ProfileIssue::InvalidParameters);
    CHECK_FALSE(d.contour.has_value());
    CHECK_FALSE(d.tool.has_value());
    CHECK_THAT(d.message, ContainsSubstring("allowance " + mm(p.allowance)));
    CHECK_THAT(d.message, ContainsSubstring("minus the tool radius (" + mm(-radius) + " at the cut depth)"));
    CHECK_THAT(d.message, ContainsSubstring("nothing is cut"));
    // The allowance never prints the same as the limit.
    CHECK(mm(p.allowance) != mm(-radius));
}

}  // namespace

TEST_CASE("AC1/AC2: an allowance below minus the tool radius is refused, exactly -r is cut",
          "[toolpath][profile][allowance]") {
    struct Tool {
        const char* name;
        tooling::Tool tool;
    };
    const std::vector<Tool> tools{{"end mill 6", test::endMill(6.0)}, {"V-bit 60", vbit60()}};
    for (const bool open : {false, true}) {
        const std::vector<Contour> cs = open ? openScene() : closedScene();
        for (const ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside, ProfileSide::On}) {
            for (const Tool& t : tools) {
                for (const bool separate : {false, true}) {
                    ProfileParams p = params(side, separate);
                    const double r = profileToolRadius(t.tool, p.passes);
                    INFO((open ? "open" : "closed") << ", " << sideName(side) << ", " << t.name << ", separate "
                                                    << separate << ", r " << mm(r));
                    const double expected = t.tool.type == tooling::ToolType::VBit ? 3.0 * std::tan(geometry::kPi / 6.0) : 3.0;
                    CHECK(std::fabs(r - expected) < 1e-12);

                    // Exactly -r stays valid (T-0127).
                    p.allowance = -r;
                    const ProfileResult exact = profileToolpath(cs, t.tool, p);
                    CHECK(exact.ok());
                    CHECK_FALSE(exact.has(ProfileIssue::InvalidParameters));

                    // One ulp, 1e-9 and 1 mm below -r are refused.
                    for (const double below :
                         {std::nextafter(-r, -std::numeric_limits<double>::infinity()), -r - 1e-9, -r - 1.0}) {
                        p.allowance = below;
                        INFO("allowance " << mm(below));
                        const ProfileResult res = profileToolpath(cs, t.tool, p);
                        checkRefused(res, p, r);
                        if (res.diagnostics.empty()) {
                            continue;
                        }
                        const std::string& msg = res.diagnostics.front().message;
                        const double rough = below + p.lastPassAllowance;
                        if (!separate) {
                            CHECK_THAT(msg, ContainsSubstring("the tool would cross the vector"));
                            CHECK_THAT(msg, !ContainsSubstring("last pass"));
                        } else if (rough < -r) {
                            // Both passes cross: the message names both distances.
                            CHECK_THAT(msg, ContainsSubstring("and allowance + last pass allowance " + mm(rough)));
                        } else {
                            // Only the last pass crosses: the roughing is not emitted on its own.
                            CHECK_THAT(msg, ContainsSubstring("the last pass would cross the vector"));
                            CHECK_THAT(msg, ContainsSubstring("roughing at allowance + last pass allowance " +
                                                              mm(rough) + " is not cut on its own"));
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("A separate last pass is refused as a whole when only its finishing pass crosses the vector",
          "[toolpath][profile][allowance]") {
    // allowance -r - 0.1 with a last-pass allowance of 0.3: the roughing (at -r + 0.2) alone would
    // be valid, but the part is never left half-finished: nothing is cut.
    const std::vector<Contour> cs = closedScene();
    for (const ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside, ProfileSide::On}) {
        INFO(sideName(side));
        ProfileParams p = params(side, true);
        p.lastPassAllowance = 0.3;
        const double r = profileToolRadius(test::endMill(6.0), p.passes);
        p.allowance = -r - 0.1;
        const ProfileResult res = profileToolpath(cs, test::endMill(6.0), p);
        checkRefused(res, p, r);
        REQUIRE(res.diagnostics.size() == 1);
        CHECK_THAT(res.diagnostics.front().message, ContainsSubstring("is not cut on its own"));
        // The same settings without the separate last pass cut at the finishing distance: refused too.
        p.separateLastPass = false;
        checkRefused(profileToolpath(cs, test::endMill(6.0), p), p, r);
        // And the roughing distance on its own is valid.
        p.allowance = -r + 0.2;
        CHECK(profileToolpath(cs, test::endMill(6.0), p).ok());
    }
}

TEST_CASE("The allowance limit follows the V-bit radius at the cut depth", "[toolpath][profile][allowance]") {
    // A 60° V-bit cuts 2·depth·tan 30° wide; a deeper cut widens the limit, so the same allowance
    // is refused at depth 3 and accepted at depth 6.
    const std::vector<Contour> cs = closedScene();
    ProfileParams p = params(ProfileSide::Outside, false);
    p.allowance = -2.5;
    p.passes.cutDepth = 3.0;
    const double r3 = profileToolRadius(vbit60(), p.passes);
    REQUIRE(r3 < 2.5);
    checkRefused(profileToolpath(cs, vbit60(), p), p, r3);
    p.passes.cutDepth = 6.0;
    REQUIRE(profileToolRadius(vbit60(), p.passes) > 2.5);
    CHECK(profileToolpath(cs, vbit60(), p).ok());
    // The start depth counts: the radius is taken at start + cut depth.
    p.passes.cutDepth = 3.0;
    p.passes.startDepth = 3.0;
    CHECK(profileToolpath(cs, vbit60(), p).ok());
}
