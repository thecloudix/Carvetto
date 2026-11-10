// T-0157: small Profile parts far from the origin. Coordinates up to kProfileMaxCoordinateMm
// (1e5 mm) give the same toolpath as at the origin up to translation; beyond it the job is
// refused with WorkLimitExceeded (ADR-0016 section 19), never silently wrong.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using namespace carvetto::geometry;

namespace {

Contour poly(const std::vector<Vec2>& p, Vec2 o) {
    std::vector<Vec2> q = p;
    for (auto& v : q) v = v + o;
    return Contour::fromPolyline(Polyline{q, true});
}

std::vector<Contour> shape(int k, Vec2 o) {
    std::vector<Contour> v;
    switch (k) {
    case 0: v.push_back(poly({{0, 0}, {3, 0}, {3, 1.5}, {0, 1.5}}, o)); break;
    case 1: v.push_back(poly({{0, 0}, {1, 0}, {1, 0.5}, {0, 0.5}}, o)); break;
    case 2: v.push_back(Contour::circle(o + Vec2{1, 1}, 1.2)); break;
    case 3:
        v.push_back(poly({{0, 0}, {6, 0}, {6, 4}, {0, 4}}, o));
        v.push_back(poly({{2, 1.5}, {2, 2.5}, {4, 2.5}, {4, 1.5}}, o));
        break;
    default: break;
    }
    return v;
}

ProfileResult run(int k, Vec2 o, ProfileSide side) {
    ProfileParams p;
    p.side = side;
    p.passes.cutDepth = 1;
    p.passes.passDepth = 1;
    tooling::Tool t;
    t.id = "em";
    t.name = "EM";
    t.type = tooling::ToolType::EndMill;
    t.diameter = 0.5;
    t.defaultCuttingData.passDepth = 2;
    t.defaultCuttingData.feedRate = 1000;
    t.defaultCuttingData.plungeRate = 300;
    t.defaultCuttingData.spindleSpeed = 18000;
    t.defaultCuttingData.toolNumber = 1;
    return profileToolpath(shape(k, o), t, p);
}

} // namespace

TEST_CASE("T-0157: small parts at up to 1e5 mm match the origin up to translation", "[toolpath][profile]") {
    const double mags[] = {1e3, 1e4, 5e4, 8e4};
    for (int k = 0; k < 4; ++k)
        for (const ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
            const auto ref = run(k, {0, 0}, side);
            // (a 1 x 0.5 mm rectangle has no Inside cut for this tool: far placements must agree.)
            if (side == ProfileSide::Outside) REQUIRE(!ref.toolpath.moves.empty());
            for (double m : mags)
                for (int j = 0; j < 5; ++j) {
                    const Vec2 o{m * (1 + 0.037 * j) * (j % 2 ? -1 : 1), m * 0.7 * (1 + 0.051 * j)};
                    const auto r = run(k, o, side);
                    INFO("shape " << k << " mag " << m << " j " << j);
                    REQUIRE(r.toolpath.moves.size() == ref.toolpath.moves.size());
                    double dev = 0;
                    for (std::size_t i = 0; i < ref.toolpath.moves.size(); ++i) {
                        const auto& a = ref.toolpath.moves[i].to;
                        const auto& b = r.toolpath.moves[i].to;
                        dev = std::max({dev, std::fabs(b.x - o.x - a.x), std::fabs(b.y - o.y - a.y), std::fabs(b.z - a.z)});
                    }
                    CHECK(dev < 1e-3);
                }
        }
}

TEST_CASE("T-0157: coordinates beyond the limit are refused with a diagnostic", "[toolpath][profile]") {
    for (double m : {2e5, 1e6, 1e8, 3e8, 1e9})
        for (int k = 0; k < 4; ++k) {
            const auto r = run(k, {m, -0.7 * m}, ProfileSide::Outside);
            INFO("shape " << k << " mag " << m);
            CHECK(r.toolpath.moves.empty());
            REQUIRE(r.diagnostics.size() == 1);
            CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
        }
}
