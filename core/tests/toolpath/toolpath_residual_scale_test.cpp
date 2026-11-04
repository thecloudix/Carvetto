// T-0131: residuals of the T-0123 toolpath alignment.
//
// (a) `BoxIndex` answers like a brute-force loop when no box of the set can go into its grid
//     (huge, infinite or NaN boxes only).
// (c)-(f) near-linear CPU time: Raster zig-zag linking on a 1,000-tooth comb, Profile with a
//     separate last pass, Pocket with a rest tool and Profile tabs on a page of thousands of
//     small vectors.
//
// Time is process CPU time, the n and 2n (or 4n) runs alternating (`interleavedCpuSeconds`,
// cpu_scaling.hpp, T-0145), so other processes under parallel ctest and load swings do not count,
// and the small time is floored above the Windows process-clock tick. The absolute bounds are
// Release bounds on the least large-run time, checked only under NDEBUG; the ratio is checked in
// every build, at smaller sizes in Debug: time for 2n over time for n at most 2.5, or time for
// 4n over time for n at most 2.5² (2.5 per doubling over two doublings) where Clipper2's own
// sweep (T-0142) keeps the curve slightly superlinear and one doubling sits close to 2.5, or the
// run would be long. The comb (c) and rest-tool (e) cases sit at that limit and use 3.0 per
// doubling until T-0142 item 3. Reference numbers (Release, the
// T-0131 machine, CPU seconds) are in each test case; the numbers before T-0131 are from the same
// machine at the T-0123 merge.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "pocket_test_util.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far/small, and a GDI `Polyline`).
#include "../geometry/cpu_scaling.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;
namespace orc = carvetto::test::pocket;

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Indices of `boxes` that intersect `q`, by a loop over all of them.
std::vector<std::size_t> bruteQuery(const std::vector<Box2>& boxes, const Box2& q) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (boxes[i].intersects(q)) {
            out.push_back(i);
        }
    }
    return out;
}

/// A box with the given corners as stored (no min/max reordering, so NaN stays where it is put).
Box2 rawBox(double x0, double y0, double x1, double y1) {
    Box2 b;
    b.min = {x0, y0};
    b.max = {x1, y1};
    return b;
}

/// Small deterministic generator (portable: no std distributions).
struct Lcg {
    std::uint64_t state;
    double next() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(state >> 11) * (1.0 / 9007199254740992.0);
    }
    double in(double a, double b) { return a + (b - a) * next(); }
};

/// `n` small vectors on a jittered 6 mm grid, like a page of text: squares, L shapes, circles,
/// squares with a square hole (the hole counts as a vector), stars and open strokes, 2-4.5 mm.
std::vector<Contour> textPage(std::size_t n) {
    std::vector<Contour> cs;
    Lcg rng{77};
    const auto cols = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(n))));
    for (std::size_t k = 0; cs.size() < n; ++k) {
        const double s = rng.in(2.0, 4.5);
        const double x = static_cast<double>(k % cols) * 6.0 + rng.in(-0.5, 0.5);
        const double y = static_cast<double>(k / cols) * 6.0 + rng.in(-0.5, 0.5);
        const int kind = std::min(5, static_cast<int>(rng.in(0.0, 6.0)));
        switch (kind) {
            case 0:
                cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}}));
                break;
            case 1: {
                const double w = s * 0.375;
                const std::vector<Vec2> pts{{x, y},         {x + s, y},     {x + s, y + w},
                                            {x + w, y + w}, {x + w, y + s}, {x, y + s}};
                cs.push_back(Contour::fromPolyline(carvetto::geometry::Polyline{pts, true}));
                break;
            }
            case 2:
                cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2));
                break;
            case 3:
                cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}}));
                if (cs.size() < n) {
                    const double a = 0.3 * s;
                    const double b = 0.7 * s;
                    cs.push_back(Contour::rectangle(Box2{{x + a, y + a}, {x + b, y + b}}));
                }
                break;
            case 4: {
                std::vector<Vec2> pts;
                const int m = 2 * (3 + static_cast<int>(rng.in(0.0, 4.0)));
                for (int i = 0; i < m; ++i) {
                    const double t = 6.283185307179586 * i / m;
                    const double r = (i % 2 != 0 ? 0.25 : 0.5) * s;
                    pts.push_back({x + s / 2 + r * std::cos(t), y + s / 2 + r * std::sin(t)});
                }
                cs.push_back(Contour::fromPolyline(carvetto::geometry::Polyline{pts, true}));
                break;
            }
            default:
                cs.push_back(Contour::fromPolyline(
                    carvetto::geometry::Polyline{{{x, y}, {x + s, y + s}, {x + s, y}}, false}));
                break;
        }
    }
    return cs;
}

/// The T-0121 verifier's comb: `teeth` teeth at 1 mm pitch, 0.5 mm gaps 20 mm deep, on a 5 mm base.
Contour comb(int teeth) {
    std::vector<Vec2> pts;
    for (int i = 0; i < teeth; ++i) {
        pts.push_back({i * 1.0, 0});
        pts.push_back({i * 1.0, 20});
        pts.push_back({i + 0.5, 20});
        pts.push_back({i + 0.5, 0.0});
    }
    pts.push_back({double(teeth), -5});
    pts.push_back({0, -5});
    return Contour::fromPolyline(carvetto::geometry::Polyline{pts, true});
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


}  // namespace

TEST_CASE("T-0131 a: BoxIndex of only non-finite boxes answers like brute force",
          "[toolpath][box_index][T-0131]") {
    const std::vector<Box2> singles{
        Box2{{0, 0}, {1, 1}},                // finite
        Box2{{-1e308, 0}, {1e308, 1}},        // finite corners, infinite width
        Box2{{-kInf, -1}, {kInf, 1}},         // infinite
        Box2{{0, -kInf}, {1, kInf}},          // infinite height
        rawBox(kNaN, 0, 1, 1),                // NaN corner
        rawBox(0, 0, kNaN, kNaN),             // NaN corners
        Box2{},                               // empty
    };
    const std::vector<Box2> queries{Box2{{0, 0}, {1, 1}}, Box2{{0.5, 0.5}, {0.5, 0.5}}, Box2{{5, 5}, {6, 6}},
                                    Box2{{-1e300, -2}, {-1e300, -2}}, Box2{{-kInf, -kInf}, {kInf, kInf}}};
    const std::vector<Vec2> points{{0, 0}, {0.5, 0.5}, {7, 0.5}, {-1e307, 0.5}, {0.5, 1e307}};
    for (std::size_t s = 0; s < singles.size(); ++s) {
        INFO("box " << s);
        const std::vector<Box2> boxes{singles[s]};
        const BoxIndex index(boxes);
        for (const Box2& q : queries) {
            CHECK(index.query(q) == bruteQuery(boxes, q));
        }
        for (const Vec2 p : points) {
            CHECK(index.containing(p) == bruteQuery(boxes, Box2{p, p}));
        }
    }
    // The card's examples.
    const std::vector<std::size_t> first{0};
    CHECK(BoxIndex({Box2{{-1e308, 0}, {1e308, 1}}}).query(Box2{{0, 0}, {1, 1}}) == first);
    CHECK(BoxIndex({Box2{{-kInf, -1}, {kInf, 1}}}).query(Box2{{0, 0}, {1, 1}}) == first);
    CHECK(BoxIndex({Box2{{-kInf, -1}, {kInf, 1}}}).containing({0, 0}) == first);
    // Every pair and triple of the kinds above, in every order: the same answers as brute force.
    for (std::size_t a = 0; a < singles.size(); ++a) {
        for (std::size_t b = 0; b < singles.size(); ++b) {
            for (std::size_t c = 0; c < singles.size(); ++c) {
                const std::vector<Box2> boxes{singles[a], singles[b], singles[c]};
                const BoxIndex index(boxes);
                for (const Box2& q : queries) {
                    INFO("boxes " << a << " " << b << " " << c);
                    CHECK(index.query(q) == bruteQuery(boxes, q));
                }
            }
        }
    }
}

TEST_CASE("T-0131 c: Raster zig-zag linking on a 1000-tooth comb is near-linear",
          "[toolpath][pocket][perf][T-0131]") {
    // A 0.3 mm tool at 0.12 mm stepover: every raster row crosses every gap, and each step of
    // the zig-zag looked at the whole next row; each link and travel check walked a ray across
    // every tooth, and each travel route searched and walked the whole comb outline. Release,
    // 250 / 500 / 1,000 teeth: 0.07 / 0.15 / 0.36 s (T-0123 merge: 0.50 / 1.46 / 5.65 s). The
    // rest of the growth is Clipper2's inset of the comb outline (T-0142).
    const int nSmall = kRelease ? 250 : 62;
    const int nLarge = 4 * nSmall;
    PocketParams p;
    p.cutDepth = 9.0;
    p.strategy = PocketStrategy::Raster;
    const std::vector<PocketTool> tools{orc::endMill("B", 1, 0.3, 0.12, 9.0)};
    const Contour cs = comb(nSmall);
    const Contour cl = comb(nLarge);
    PocketResult rs;
    PocketResult rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        kRelease ? 3 : 2, [&] { rs = pocketToolpath({cs}, tools, p); },
        [&] { rl = pocketToolpath({cl}, tools, p); });
    INFO("CPU seconds: " << nSmall << " teeth " << t.smallSeconds << ", " << nLarge << " teeth "
                         << t.largeSeconds << ", ratio " << t.ratio());
    CHECK(rs.diagnostics.empty());
    CHECK(rl.diagnostics.empty());
    // Every gap gets its own runs of raster lines.
    CHECK(rl.plans[0].paths.size() >= static_cast<std::size_t>(nLarge));
    // Measured 4.9-6.0 under load, close to 2.5 * 2.5: the remaining growth is Clipper2's (T-0142
    // item 3), so the bound has headroom like (e); T-0142 should tighten it to 2.5 * 2.5.
    CHECK(t.ratio() <= 3.0 * 3.0);
    if constexpr (kRelease) {
        CHECK(t.largeSeconds < 3.0);  // T-0123 merge: 5.65 s
    }
}

TEST_CASE("T-0131 d: Profile with a separate last pass is near-linear in the vector count",
          "[toolpath][profile][perf][T-0131]") {
    // Each finishing cut looked for its roughing counterpart with 17 distances to every roughing
    // cut. Release, 2,500 -> 5,000 vectors: 1.1 -> 2.4 s (T-0123 merge, the verifier's page:
    // 16 -> 69 s at 1,250 -> 2,500).
    const std::size_t nSmall = kRelease ? 2500 : 312;
    const std::size_t nLarge = 2 * nSmall;
    ProfileParams p;
    p.passes.cutDepth = 3.0;
    p.separateLastPass = true;
    p.lastPassAllowance = 0.2;
    const std::vector<Contour> cs = textPage(nSmall);
    const std::vector<Contour> cl = textPage(nLarge);
    ProfileResult rs;
    ProfileResult rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        2, [&] { rs = profileToolpath(cs, profileTool(), p); },
        [&] { rl = profileToolpath(cl, profileTool(), p); });
    INFO("CPU seconds: " << nSmall << " vectors " << t.smallSeconds << ", " << nLarge << " vectors "
                         << t.largeSeconds << ", ratio " << t.ratio());
    // Every vector is cut twice (roughing and the separate last pass), one retract per cut, apart
    // from the few the tool cannot follow.
    CHECK(rs.toolpath.count(MoveKind::Retract) >= nSmall * 3 / 2);
    CHECK(rl.toolpath.count(MoveKind::Retract) >= nLarge * 3 / 2);
    CHECK(t.ratio() <= 2.5);
    if constexpr (kRelease) {
        CHECK(t.largeSeconds < 6.0);
    }
}

TEST_CASE("T-0131 d: tabs with a separate last pass are matched near-linearly",
          "[toolpath][profile][tabs][perf][T-0131]") {
    // Every finishing tab was mapped onto every roughing cut, and every cut's keep-clear points
    // came from every start of the other pass. The tab check itself (f) dominates the time, so
    // the sizes are small. Release, 312 -> 624 vectors: 1.6 -> 3.0 s.
    const std::size_t nSmall = kRelease ? 312 : 78;
    const std::size_t nLarge = 2 * nSmall;
    ProfileParams p;
    p.passes.cutDepth = 3.0;
    p.separateLastPass = true;
    p.lastPassAllowance = 0.2;
    p.tabs.enabled = true;
    p.tabs.length = 1.0;
    p.tabs.count = 1;
    p.tabs.thickness = 1.0;
    const std::vector<Contour> cs = textPage(nSmall);
    const std::vector<Contour> cl = textPage(nLarge);
    ProfileResult rs;
    ProfileResult rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        1, [&] { rs = profileToolpath(cs, profileTool(), p); },
        [&] { rl = profileToolpath(cl, profileTool(), p); });
    INFO("CPU seconds: " << nSmall << " vectors " << t.smallSeconds << ", " << nLarge << " vectors "
                         << t.largeSeconds << ", ratio " << t.ratio());
    CHECK(rl.toolpath.count(MoveKind::Retract) >= nLarge * 3 / 2);
    CHECK(t.ratio() <= 2.5);
    if constexpr (kRelease) {
        // A loose guard: the criterion is linearity (the ratio). Windows CI Release measures
        // ~10.5 s at 624 vectors since T-0159 places T-0127's tab candidate both ways.
        CHECK(t.largeSeconds < 30.0);
    }
}

TEST_CASE("T-0131 e: Pocket with a rest tool on a page of vectors is near-linear",
          "[toolpath][pocket][rest][perf][T-0131]") {
    // The rest areas were ordered by a nearest-first search whose square grew over the whole
    // page at the end of every part (one index over all parts). Release, this page, 1,250 /
    // 2,500 / 5,000 vectors: 0.84 / 2.0 / 4.9 s idle, up to 6.7 s at 5,000 under load (main:
    // 1.1 / 3.2 / 9.0 s; the verifier's page at the T-0123 merge: 2.3 / 7.5 / 22 s).
    //
    // Accepted at this limit until T-0142 (director, 2026-11-03): what remains superlinear is
    // Clipper2's boolean sweep over the whole page (BuildIntersectList; rest material and rest
    // areas), ~2.4-2.6x per doubling. So the bound is 3.0 per doubling (9.0 over the two), the
    // ratio taken over three interleaved pairs; T-0142 item 3 should tighten it to 2.5.
    //
    // No absolute bound: on this page the branch is under 2x faster than main at 5,000 vectors
    // (4.9 vs 9.0 s idle, the branch up to 6.7 s under load), so no bound sits safely between
    // them on every CI machine, and a bound above main guards nothing. The ratio is the guard.
    const std::size_t nSmall = kRelease ? 1250 : 156;
    const std::size_t nLarge = 4 * nSmall;
    PocketParams p;
    p.cutDepth = 3.0;
    const std::vector<PocketTool> tools{orc::endMill("EM 1", 1, 1.0, 0.4, 3.0),
                                        orc::endMill("EM .3", 2, 0.3, 0.12, 3.0)};
    const std::vector<Contour> cs = textPage(nSmall);
    const std::vector<Contour> cl = textPage(nLarge);
    PocketResult rs;
    PocketResult rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        3, [&] { rs = pocketToolpath(cs, tools, p); },
        [&] { rl = pocketToolpath(cl, tools, p); });
    INFO("CPU seconds: " << nSmall << " vectors " << t.smallSeconds << ", " << nLarge << " vectors "
                         << t.largeSeconds << ", ratio " << t.ratio());
    REQUIRE(rl.toolpaths.size() == 2);
    // The rest tool works in the corners of most shapes: one retract per rest area.
    CHECK(rl.toolpaths[1].count(MoveKind::Retract) >= nLarge);
    CHECK(rl.toolpaths[1].count(MoveKind::Retract) == rl.plans[1].paths.back().cluster + 1);
    CHECK(t.ratio() <= 3.0 * 3.0);  // T-0142 item 3: tighten to 2.5 * 2.5
}

TEST_CASE("T-0131 f: Profile tabs on a page of vectors stay linear",
          "[toolpath][profile][tabs][perf][T-0131]") {
    // Linear before T-0131 too, but slow (17 s for 5,000 vectors on the verifier's page): every
    // tab is judged on a 0.05 mm raster round it, a distance query per cell. The distance
    // queries now skip the spans that cannot be nearest. Release, one 1 mm tab per vector,
    // 1,250 / 2,500 / 5,000 vectors: 3.3 / 6.5 / 12.7 s. Checked over two doublings to keep the
    // run short.
    const std::size_t nSmall = kRelease ? 1250 : 78;
    const std::size_t nLarge = 4 * nSmall;
    ProfileParams p;
    p.passes.cutDepth = 3.0;
    p.tabs.enabled = true;
    p.tabs.length = 1.0;
    p.tabs.count = 1;
    p.tabs.thickness = 1.0;
    const std::vector<Contour> cs = textPage(nSmall);
    const std::vector<Contour> cl = textPage(nLarge);
    ProfileResult rs;
    ProfileResult rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        1, [&] { rs = profileToolpath(cs, profileTool(), p); },
        [&] { rl = profileToolpath(cl, profileTool(), p); });
    INFO("CPU seconds: " << nSmall << " vectors " << t.smallSeconds << ", " << nLarge << " vectors "
                         << t.largeSeconds << ", ratio " << t.ratio());
    CHECK(rl.toolpath.count(MoveKind::Retract) >= nLarge * 3 / 4);
    CHECK(t.ratio() <= 2.5 * 2.5);
    if constexpr (kRelease) {
        CHECK(t.largeSeconds < 30.0);  // a loose guard: the criterion here is linearity (the ratio)
    }
}
