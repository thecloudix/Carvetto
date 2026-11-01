// Toolpath scaling (T-0123 AC3, AC4): text-heavy jobs with thousands of small closed vectors
// profile and pocket in near-linear time, and a rest tool on a 200-tooth comb builds its pattern
// only over the rest areas.
//
// Time is process CPU time (`processCpuSeconds`, cpu_time.hpp), best of a few runs, so other
// processes under parallel ctest do not count. The absolute bounds are Release bounds and are
// checked only under NDEBUG; the scaling ratio (time for 2n over time for n, at most 2.5) is
// checked in every build, at smaller sizes in Debug. Reference numbers on the T-0123 machine
// (Release): 5,000 vectors profile 0.24 s, pocket 0.65 s; comb raster rest 0.27 s, against 23.7 s
// at the T-0121 merge (799dba3) for the same case.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "pocket_oracle.hpp"
#include "pocket_test_util.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far).
#include "../geometry/cpu_time.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Polyline;
using carvetto::geometry::Vec2;
namespace orc = carvetto::test::pocket;

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
constexpr std::size_t kSmall = 2500;
constexpr std::size_t kLarge = 5000;
#else
constexpr bool kRelease = false;
constexpr std::size_t kSmall = 1250;
constexpr std::size_t kLarge = 2500;
#endif

/// `n` small closed vectors on a 6 mm grid, like a page of text: 4 mm squares, L shapes, circles
/// and squares with a square hole (the hole counts as a vector).
std::vector<Contour> textLike(std::size_t n) {
    std::vector<Contour> cs;
    const auto cols = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(n))));
    for (std::size_t k = 0; cs.size() < n; ++k) {
        const double x = static_cast<double>(k % cols) * 6.0;
        const double y = static_cast<double>(k / cols) * 6.0;
        switch (k % 4) {
            case 0:
                cs.push_back(Contour::rectangle(Box2{{x, y}, {x + 4, y + 4}}));
                break;
            case 1:
                cs.push_back(Contour::fromPolyline(Polyline{
                    {{x, y}, {x + 4, y}, {x + 4, y + 1.5}, {x + 1.5, y + 1.5}, {x + 1.5, y + 4}, {x, y + 4}}, true}));
                break;
            case 2:
                cs.push_back(Contour::circle({x + 2, y + 2}, 2.0));
                break;
            default:
                cs.push_back(Contour::rectangle(Box2{{x, y}, {x + 4, y + 4}}));
                if (cs.size() < n) {
                    cs.push_back(Contour::rectangle(Box2{{x + 1.2, y + 1.2}, {x + 2.8, y + 2.8}}));
                }
                break;
        }
    }
    return cs;
}

carvetto::tooling::Tool profileTool() {
    carvetto::tooling::Tool t;
    t.name = "EM 1";
    t.diameter = 1.0;
    t.defaultCuttingData.passDepth = 3.0;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    return t;
}

/// Best-of-`runs` CPU seconds of `f()`; `f` stores its result for the checks.
template <class F>
double bestCpu(int runs, F&& f) {
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < runs; ++i) {
        const double t0 = carvetto::test::processCpuSeconds();
        f();
        best = std::min(best, carvetto::test::processCpuSeconds() - t0);
    }
    return best;
}

/// Lower bound for the small-size time in the ratio, so a coarse clock tick cannot fail it.
constexpr double kTimeFloor = 0.02;

}  // namespace

TEST_CASE("AC3: thousands of small closed vectors profile in near-linear time", "[toolpath][profile][perf][T-0123]") {
    ProfileParams p;
    p.passes.cutDepth = 3.0;
    const std::vector<Contour> small = textLike(kSmall);
    const std::vector<Contour> large = textLike(kLarge);
    ProfileResult rs;
    ProfileResult rl;
    const int runs = kRelease ? 3 : 2;
    const double ts = bestCpu(runs, [&] { rs = profileToolpath(small, profileTool(), p); });
    const double tl = bestCpu(runs, [&] { rl = profileToolpath(large, profileTool(), p); });
    INFO("CPU seconds: " << kSmall << " vectors " << ts << ", " << kLarge << " vectors " << tl);
    CHECK(rs.diagnostics.empty());
    CHECK(rl.diagnostics.empty());
    // Every vector is cut once (one retract per cut).
    CHECK(rs.toolpath.count(MoveKind::Retract) == kSmall);
    CHECK(rl.toolpath.count(MoveKind::Retract) == kLarge);
    CHECK(tl <= 2.5 * std::max(ts, kTimeFloor));
    if constexpr (kRelease) {
        CHECK(tl < 2.0);
    }
}

TEST_CASE("AC3: thousands of small closed vectors pocket in near-linear time", "[toolpath][pocket][perf][T-0123]") {
    PocketParams p;
    p.cutDepth = 3.0;
    const PocketTool tool = orc::endMill("EM 1", 1, 1.0, 0.4, 3.0);
    const std::vector<Contour> small = textLike(kSmall);
    const std::vector<Contour> large = textLike(kLarge);
    PocketResult rs;
    PocketResult rl;
    const int runs = kRelease ? 3 : 2;
    const double ts = bestCpu(runs, [&] { rs = pocketToolpath(small, {tool}, p); });
    const double tl = bestCpu(runs, [&] { rl = pocketToolpath(large, {tool}, p); });
    INFO("CPU seconds: " << kSmall << " vectors " << ts << ", " << kLarge << " vectors " << tl);
    CHECK(rs.diagnostics.empty());
    CHECK(rl.diagnostics.empty());
    // One cluster (and one retract) per pocketed region: every shape but the holes, whose
    // squares are pocketed as rings.
    REQUIRE_FALSE(rl.plans[0].paths.empty());
    CHECK(rl.toolpaths[0].count(MoveKind::Retract) == rl.plans[0].paths.back().cluster + 1);
    CHECK(rl.toolpaths[0].count(MoveKind::Retract) >= kLarge * 3 / 4);
    CHECK(tl <= 2.5 * std::max(ts, kTimeFloor));
    if constexpr (kRelease) {
        CHECK(tl < 2.0);
    }
}

TEST_CASE("AC4: a 0.1 mm rest tool on a 200-tooth comb works only over the rest areas",
          "[toolpath][pocket][rest][perf][T-0123]") {
    // The T-0121 verifier's comb: 200 teeth, 1 mm pitch, 0.5 mm gaps 20 mm deep, on a 5 mm base.
    // A 1 mm clearance tool cannot enter the gaps; a 0.1 mm rest tool (40 % stepover) cleans
    // them. At the T-0121 merge the rest raster built the whole pattern (625 lines x 200 teeth)
    // and clipped every piece against every gap: 23.7 s Release. Building the pattern over the
    // rest areas: 0.27 s.
    std::vector<Vec2> pts;
    const int teeth = 200;
    for (int i = 0; i < teeth; ++i) {
        pts.push_back({i * 1.0, 0});
        pts.push_back({i * 1.0, 20});
        pts.push_back({i + 0.5, 20});
        pts.push_back({i + 0.5, 0.0});
    }
    pts.push_back({double(teeth), -5});
    pts.push_back({0, -5});
    const Contour comb = Contour::fromPolyline(Polyline{pts, true});
    const std::vector<PocketTool> tools{orc::endMill("B", 1, 1.0, 0.4, 9.0), orc::endMill("S", 2, 0.1, 0.04, 9.0)};
    for (const PocketStrategy s : {PocketStrategy::Raster, PocketStrategy::Offset}) {
        INFO("strategy " << static_cast<int>(s));
        PocketParams p;
        p.cutDepth = 9.0;
        p.strategy = s;
        PocketResult r;
        const double t = bestCpu(1, [&] { r = pocketToolpath({comb}, tools, p); });
        INFO("CPU seconds " << t);
        CHECK(r.diagnostics.empty());
        REQUIRE_FALSE(r.toolpaths[1].isEmpty());
        // Every gap is a rest area of its own; the tool retracts between rest areas (owner
        // decision 2026-11-01), once per area.
        CHECK(r.plans[1].paths.back().cluster + 1 >= static_cast<std::size_t>(teeth));
        CHECK(r.toolpaths[1].count(MoveKind::Retract) == r.plans[1].paths.back().cluster + 1);
        if constexpr (kRelease) {
            CHECK(t < 4.0);  // >= 5x faster than the 23.7 s at the T-0121 merge
        }
    }
}
