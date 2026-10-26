// Pocket toolpath (T-0121): raster geometry and profile pass (AC3), linking (AC5), direction,
// allowance, ramps, multi-tool naming, messages and determinism.

#include "carvetto/toolpath/pocket.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pass_plan.hpp"
#include "pocket_oracle.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::kPi;
using carvetto::geometry::Polyline;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;
using Catch::Matchers::WithinAbs;
namespace orc = carvetto::test::pocket;

namespace {

PocketTool tool(double diameter, double stepoverFraction, double passDepth, const std::string& name = "EM") {
    PocketTool t;
    t.tool.name = name;
    t.tool.toolNumber = 1;
    t.tool.diameter = diameter;
    t.tool.feedRate = 1000.0;
    t.tool.plungeRate = 300.0;
    t.tool.spindleSpeed = 18000.0;
    t.stepover = stepoverFraction * diameter;
    t.passDepth = passDepth;
    return t;
}

Contour rect(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2{Vec2{x0, y0}, Vec2{x1, y1}});
}

double shoelace(const std::vector<Vec2>& pts) {
    double a = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        a += pts[i].cross(pts[(i + 1) % pts.size()]);
    }
    return 0.5 * a;
}

/// Distance from p to the boundary of an axis-aligned box.
double boxBoundaryDist(Vec2 p, double x0, double y0, double x1, double y1) {
    return orc::boundaryDist({orc::rectPoly(x0, y0, x1, y1)}, p);
}

PocketParams params(double cutDepth = 3.0) {
    PocketParams p;
    p.cutDepth = cutDepth;
    return p;
}

}  // namespace

TEST_CASE("AC3: raster at 45 degrees with a last profile pass", "[toolpath][pocket][raster]") {
    PocketParams p = params(3.0);
    p.strategy = PocketStrategy::Raster;
    p.rasterAngle = kPi / 4;
    p.profilePass = ProfilePass::Last;
    const PocketTool t = tool(6.0, 0.4, 3.0);
    const PocketResult res = pocketToolpath({rect(0, 0, 50, 30)}, {t}, p);
    REQUIRE(res.plans.size() == 1);
    const PocketToolPlan& plan = res.plans[0];
    // Centre boundary: the rectangle inset by radius + chordal margin.
    const double m = 3.0 + p.tolerance.chordal;
    const double x0 = m, y0 = m, x1 = 50 - m, y1 = 30 - m;
    const Vec2 u{std::cos(p.rasterAngle), std::sin(p.rasterAngle)};

    std::size_t rasters = 0;
    std::vector<double> offsets;
    for (std::size_t i = 0; i < plan.paths.size(); ++i) {
        const PocketPath& pp = plan.paths[i];
        if (pp.role == PocketPathRole::Profile) {
            CHECK(i + 1 == plan.paths.size());  // the profile pass comes last
            continue;
        }
        REQUIRE(pp.role == PocketPathRole::Raster);
        ++rasters;
        REQUIRE(pp.path.points.size() == 2);
        CHECK_FALSE(pp.path.closed);
        const Vec2 a = pp.path.points[0];
        const Vec2 b = pp.path.points[1];
        const Vec2 d = b - a;
        CHECK(std::fabs(d.cross(u)) <= 1e-9 * d.length());  // parallel to the raster angle
        CHECK(boxBoundaryDist(a, x0, y0, x1, y1) <= 1e-5);  // clipped exactly at the boundary
        CHECK(boxBoundaryDist(b, x0, y0, x1, y1) <= 1e-5);
        offsets.push_back(a.dot(u.perp()));
    }
    CHECK(rasters >= 10);
    std::sort(offsets.begin(), offsets.end());
    for (std::size_t i = 1; i < offsets.size(); ++i) {
        CHECK(offsets[i] - offsets[i - 1] <= t.stepover + 1e-9);
        CHECK(offsets[i] - offsets[i - 1] >= 0.5 * t.stepover);
    }

    // Exactly one profile path, tracing the centre boundary once (area and perimeter match).
    std::size_t profiles = 0;
    for (const PocketPath& pp : plan.paths) {
        if (pp.role != PocketPathRole::Profile) {
            continue;
        }
        ++profiles;
        CHECK(pp.path.closed);
        for (const Vec2 q : pp.path.points) {
            CHECK(boxBoundaryDist(q, x0, y0, x1, y1) <= 1e-5);
        }
        CHECK_THAT(shoelace(pp.path.points), WithinAbs((x1 - x0) * (y1 - y0), 1e-4));  // CCW = climb
        CHECK_THAT(pp.path.length(), WithinAbs(2 * ((x1 - x0) + (y1 - y0)), 1e-4));
    }
    CHECK(profiles == 1);

    // The move list ends with that loop: feeds through every profile vertex and back to its
    // start, then the final retract.
    const Toolpath& tp = res.toolpaths[0];
    const Polyline& loop = plan.paths.back().path;
    const std::size_t n = loop.points.size();
    REQUIRE(tp.moves.size() > n + 2);
    CHECK(tp.moves.back().kind == MoveKind::Retract);
    for (std::size_t k = 0; k <= n; ++k) {
        const Move& mv = tp.moves[tp.moves.size() - 2 - k];
        CHECK(mv.kind == MoveKind::Feed);
        CHECK(mv.to.z == -3.0);
        const Vec2 expect = loop.points[(n - k) % n];
        CHECK(distance(mv.to.xy(), expect) <= 1e-12);
    }
}

TEST_CASE("AC3: raster lines are clipped around an island; First puts the profile first",
          "[toolpath][pocket][raster]") {
    PocketParams p = params(3.0);
    p.strategy = PocketStrategy::Raster;
    p.rasterAngle = kPi / 4;
    p.profilePass = ProfilePass::First;
    const PocketResult res =
        pocketToolpath({rect(0, 0, 50, 30), rect(15, 10, 35, 20)}, {tool(6.0, 0.4, 3.0)}, p);
    const PocketToolPlan& plan = res.plans[0];
    const double m = 3.0 + p.tolerance.chordal;
    const std::vector<orc::Poly> centre{orc::rectPoly(m, m, 50 - m, 30 - m)};
    REQUIRE(plan.paths.size() > 3);
    CHECK(plan.paths[0].role == PocketPathRole::Profile);
    CHECK(plan.paths[1].role == PocketPathRole::Profile);  // outer wall and island wall
    CHECK(plan.paths[2].role == PocketPathRole::Raster);
    // Island walls are cut clockwise with climb milling.
    std::size_t ccw = 0, cw = 0;
    for (std::size_t i = 0; i < 2; ++i) {
        (shoelace(plan.paths[i].path.points) > 0 ? ccw : cw) += 1;
    }
    CHECK(ccw == 1);
    CHECK(cw == 1);
    std::size_t endsOnIsland = 0;
    for (const PocketPath& pp : plan.paths) {
        if (pp.role != PocketPathRole::Raster) {
            continue;
        }
        for (const Vec2 q : pp.path.points) {
            // Every end lies on the outer centre boundary or on the island's rounded offset.
            const double toOuter = orc::boundaryDist(centre, q);
            const double toIsland = orc::boundaryDist({orc::rectPoly(15, 10, 35, 20)}, q);
            const bool onIsland = std::fabs(toIsland - m) <= 0.006;  // round-join chords
            endsOnIsland += onIsland ? 1u : 0u;
            CHECK((toOuter <= 1e-5 || onIsland));
            CHECK(toIsland >= m - 0.006);
        }
    }
    CHECK(endsOnIsland >= 10);
}

TEST_CASE("AC5: offset strategy retracts at most once per disconnected region plus one",
          "[toolpath][pocket][linking]") {
    struct Case {
        const char* name;
        std::vector<orc::Poly> polys;  // even-odd: outer(s), then islands
        std::size_t regions;
    };
    const orc::Poly lShape{{0, 0}, {60, 0}, {60, 20}, {20, 20}, {20, 50}, {0, 50}};
    // Verifier repros (T-0121 round 2): concave U and C shapes and the AC2 island used to
    // retract at every level because the way back had to be one straight segment.
    const orc::Poly uShape{{0, 0}, {60, 0}, {60, 40}, {35, 40}, {35, 12}, {25, 12}, {25, 40}, {0, 40}};
    const orc::Poly cShape{{0, 0}, {50, 0}, {50, 12}, {14, 12}, {14, 38}, {50, 38}, {50, 50}, {0, 50}};
    const orc::Poly plus{{20, 0},  {40, 0},  {40, 20}, {60, 20}, {60, 40}, {40, 40},
                         {40, 60}, {20, 60}, {20, 40}, {0, 40},  {0, 20},  {20, 20}};
    auto circle = [](Vec2 c, double r) {
        orc::Poly p;
        for (int i = 0; i < 90; ++i) {
            p.push_back({c.x + r * std::cos(i * kPi / 45), c.y + r * std::sin(i * kPi / 45)});
        }
        return p;
    };
    const std::vector<Case> cases{
        {"rectangle", {orc::rectPoly(0, 0, 50, 30)}, 1},
        {"circle r 20", {circle({0, 0}, 20)}, 1},
        {"L shape", {lShape}, 1},
        {"U shape", {uShape}, 1},
        {"C shape", {cShape}, 1},
        {"plus", {plus}, 1},
        {"rectangle + 20x10 island", {orc::rectPoly(0, 0, 50, 30), orc::rectPoly(15, 10, 35, 20)}, 1},
        {"rectangle + two islands",
         {orc::rectPoly(0, 0, 70, 40), orc::rectPoly(10, 10, 25, 30), orc::rectPoly(40, 12, 60, 28)},
         1},
        {"two rectangles", {orc::rectPoly(0, 0, 30, 20), orc::rectPoly(40, 0, 70, 20)}, 2},
        {"three circles", {circle({0, 0}, 8), circle({30, 0}, 8), circle({60, 0}, 8)}, 3},
    };
    for (const Case& c : cases) {
        std::vector<Contour> input;
        for (const orc::Poly& poly : c.polys) {
            input.push_back(Contour::fromPolyline(Polyline{poly, true}));
        }
        for (double cutDepth : {3.0, 9.0}) {
            for (double stepover : {2.4, 4.5}) {
                for (ProfilePass pass : {ProfilePass::Last, ProfilePass::First}) {
                    for (bool ramp : {false, true}) {
                        INFO(c.name << " cut depth " << cutDepth << " stepover " << stepover << " pass "
                                    << static_cast<int>(pass) << " ramp " << ramp);
                        PocketParams p = params(cutDepth);
                        p.rampPlunge = ramp;
                        p.profilePass = pass;
                        PocketTool t = tool(6.0, 0.4, 3.0);
                        t.stepover = stepover;
                        const PocketResult res = pocketToolpath(input, {t}, p);
                        const Toolpath& tp = res.toolpaths[0];
                        REQUIRE_FALSE(tp.isEmpty());
                        CHECK(tp.count(MoveKind::Retract) <= c.regions + 1);
                        CHECK(tp.count(MoveKind::Retract) >= c.regions);
                        if (ramp || cutDepth < 9.0) {
                            continue;  // the exact gouge check below is costly; one variant suffices
                        }
                        // Travel inside the pocket never gouges, and lifts are vertical and stay
                        // below the surface.
                        const orc::GougeReport g = orc::gougeCheck(tp, c.polys, 0.0, p.safeZ);
                        CHECK(g.allInside);
                        CHECK(g.minDistance >= 3.0);
                        for (std::size_t i = 0; i < tp.moves.size(); ++i) {
                            const Move& mv = tp.moves[i];
                            const Vec3 from = tp.startOf(i);
                            if (mv.kind == MoveKind::Feed && mv.to.z > from.z) {
                                CHECK(distance(mv.to.xy(), from.xy()) == 0.0);
                                CHECK(mv.to.z <= 0.0);
                            }
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("Raster links along the boundary are feeds, not retracts", "[toolpath][pocket][linking][raster]") {
    // Verifier repro: at 45 degrees the projected piece ends sit a rounding error off the
    // boundary, and the link along it used to be rejected (~16 retracts per level; 1,374 on
    // 500 x 300).
    for (double angle : {0.0, kPi / 6, kPi / 4, kPi / 2, 2.0}) {
        for (double w : {50.0, 500.0}) {
            INFO("angle " << angle << " width " << w);
            PocketParams p = params(3.0);
            p.strategy = PocketStrategy::Raster;
            p.rasterAngle = angle;
            p.profilePass = ProfilePass::None;
            const PocketResult res = pocketToolpath({rect(0, 0, w, 0.6 * w)}, {tool(6.0, 0.4, 3.0)}, p);
            const Toolpath& tp = res.toolpaths[0];
            CHECK(tp.count(MoveKind::Retract) <= 2);
            const orc::GougeReport g = orc::gougeCheck(tp, {orc::rectPoly(0, 0, w, 0.6 * w)}, 0.0, p.safeZ);
            CHECK(g.allInside);
            CHECK(g.minDistance >= 3.0);
        }
    }
}

TEST_CASE("Owner decision 2026-10-26: Profile Pass First/Last/None for the Offset strategy",
          "[toolpath][pocket][profile]") {
    const std::vector<Contour> shape{rect(0, 0, 50, 30), rect(15, 10, 35, 20)};
    auto plan = [&](ProfilePass pass) {
        PocketParams p = params(3.0);
        p.profilePass = pass;
        return pocketToolpath(shape, {tool(6.0, 0.4, 3.0)}, p);
    };
    const PocketResult first = plan(ProfilePass::First);
    const PocketResult last = plan(ProfilePass::Last);
    const PocketResult none = plan(ProfilePass::None);
    auto roles = [](const PocketResult& r) {
        std::vector<PocketPathRole> out;
        for (const PocketPath& pp : r.plans[0].paths) {
            out.push_back(pp.role);
        }
        return out;
    };
    // The boundary pass is the outer wall ring and the island ring at the full inset.
    const std::vector<PocketPathRole> rf = roles(first), rl = roles(last), rn = roles(none);
    REQUIRE(rf.size() >= 4);
    CHECK(rf[0] == PocketPathRole::Profile);
    CHECK(rf[1] == PocketPathRole::Profile);
    CHECK(rf[2] == PocketPathRole::Ring);
    CHECK(std::count(rf.begin(), rf.end(), PocketPathRole::Profile) == 2);
    CHECK(rl[rl.size() - 1] == PocketPathRole::Profile);
    CHECK(rl[rl.size() - 2] == PocketPathRole::Profile);
    CHECK(std::count(rl.begin(), rl.end(), PocketPathRole::Profile) == 2);
    CHECK(std::count(rn.begin(), rn.end(), PocketPathRole::Profile) == 0);
    // None leaves the boundary as an ordinary (last) clearing ring: same geometry as Last.
    REQUIRE(none.plans[0].paths.size() == last.plans[0].paths.size());
    for (std::size_t i = 0; i < none.plans[0].paths.size(); ++i) {
        CHECK(none.plans[0].paths[i].path == last.plans[0].paths[i].path);
    }
    CHECK(none.toolpaths[0].moves == last.toolpaths[0].moves);
    // First cuts the boundary before clearing: the first cutting moves trace the wall at
    // radius + margin from the outer rectangle.
    const Polyline& wall = first.plans[0].paths[0].path;
    CHECK_THAT(std::fabs(shoelace(wall.points)), WithinAbs((50 - 6.02) * (30 - 6.02), 1e-3));
    CHECK(first.toolpaths[0].moves[1].kind == MoveKind::Plunge);
    CHECK(distance(first.toolpaths[0].moves[1].to.xy(), wall.points.front()) <= 1e-12);
    // Every variant is gouge-free and retracts once.
    for (const PocketResult* r : {&first, &last, &none}) {
        CHECK(r->toolpaths[0].count(MoveKind::Retract) == 1);
        const orc::GougeReport g = orc::gougeCheck(
            r->toolpaths[0], {orc::rectPoly(0, 0, 50, 30), orc::rectPoly(15, 10, 35, 20)}, 0.0, 5.0);
        CHECK(g.allInside);
        CHECK(g.minDistance >= 3.0);
    }
}

TEST_CASE("Pocket: degenerate parameters are refused before any work, never thrown",
          "[toolpath][pocket][robustness]") {
    // Verifier repros: pass depth 1e-9 (2e9 levels -> bad_alloc), stepover 1e-6 (unbounded run),
    // chordal 1e-7 on an r = 20 circle (unbounded run). Each must come back at once with an empty
    // toolpath, no plan and a message; the checks run before PassPlan, offsets or patterns, so
    // the work done is independent of how extreme the value is.
    struct Case {
        const char* name;
        std::vector<Contour> shape;
        PocketTool tool;
        PocketParams params;
    };
    std::vector<Case> cases;
    auto add = [&](const char* name, std::vector<Contour> shape, PocketTool t, PocketParams p) {
        cases.push_back({name, std::move(shape), std::move(t), std::move(p)});
    };
    PocketTool t = tool(6.0, 0.4, 3.0);
    PocketParams p = params(6.0);
    PocketTool tinyPass = t;
    tinyPass.passDepth = 1e-9;
    add("pass depth 1e-9", {rect(0, 0, 50, 30)}, tinyPass, p);
    PocketTool tinyStep = t;
    tinyStep.stepover = 1e-6;
    add("stepover 1e-6", {rect(0, 0, 50, 30)}, tinyStep, p);
    PocketParams fineTol = p;
    fineTol.tolerance.chordal = 1e-7;
    add("chordal 1e-7", {Contour::circle({0, 0}, 20)}, t, fineTol);
    PocketParams badDist = p;
    badDist.tolerance.distance = 0.0;
    add("distance 0", {rect(0, 0, 50, 30)}, t, badDist);
    PocketTool smallStep = t;
    smallStep.stepover = 0.02;
    add("stepover 0.02 on a 2 m pocket", {rect(0, 0, 2000, 2000)}, smallStep, p);
    add("huge circle at fine tolerance", {Contour::circle({0, 0}, 1e6)}, t, p);
    PocketTool hugeLevels = t;
    hugeLevels.passDepth = 0.5;
    PocketParams deep = p;
    deep.cutDepth = 1e5;
    add("100 m deep in 0.5 mm passes", {rect(0, 0, 50, 30)}, hugeLevels, deep);
    for (const Case& c : cases) {
        INFO(c.name);
        PocketResult r;
        REQUIRE_NOTHROW(r = pocketToolpath(c.shape, {c.tool}, c.params));
        REQUIRE(r.toolpaths.size() == 1);
        CHECK(r.toolpaths[0].isEmpty());
        CHECK(r.plans[0].paths.empty());
        CHECK(r.plans[0].levels.empty());
        CHECK(r.messages.size() == 1);
    }
}

TEST_CASE("Pocket: climb and conventional reverse the ring directions", "[toolpath][pocket]") {
    for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
        PocketParams p = params();
        p.direction = dir;
        const PocketResult res =
            pocketToolpath({rect(0, 0, 50, 30), rect(15, 10, 35, 20)}, {tool(6.0, 0.4, 3.0)}, p);
        std::size_t outers = 0, holes = 0;
        for (const PocketPath& pp : res.plans[0].paths) {
            REQUIRE(pp.role != PocketPathRole::Raster);  // default Last: the boundary rings are Profile
            REQUIRE(pp.path.closed);
            const double a = shoelace(pp.path.points);
            if (a > 0) {
                ++(dir == CutDirection::Climb ? outers : holes);
            } else {
                ++(dir == CutDirection::Climb ? holes : outers);
            }
        }
        CHECK(outers > 0);
        CHECK(holes > 0);
        // The very last ring is the outer wall: CCW for climb, CW for conventional.
        const double last = shoelace(res.plans[0].paths.back().path.points);
        CHECK((dir == CutDirection::Climb ? last > 0 : last < 0));
        CHECK_THAT(std::fabs(last), WithinAbs((50 - 6.02) * (30 - 6.02), 1e-3));
    }
}

TEST_CASE("Pocket: allowance shrinks the centre region; levels follow PassPlan", "[toolpath][pocket]") {
    PocketParams p = params(7.0);
    p.startDepth = 1.0;
    p.allowance = 0.4;
    p.maintainExactStep = false;
    p.lastPassThickness = 0.5;
    const PocketTool t = tool(6.0, 0.5, 2.5);
    const PocketResult res = pocketToolpath({rect(0, 0, 50, 30)}, {t}, p);
    REQUIRE(res.plans[0].centreRegion.size() == 1);
    const Box2 b = res.plans[0].centreRegion[0].bounds();
    const double m = 3.0 + 0.4 + p.tolerance.chordal;
    CHECK_THAT(b.min.x, WithinAbs(m, 2e-6));
    CHECK_THAT(b.min.y, WithinAbs(m, 2e-6));
    CHECK_THAT(b.max.x, WithinAbs(50 - m, 2e-6));
    CHECK_THAT(b.max.y, WithinAbs(30 - m, 2e-6));
    PassPlan pp;
    pp.startDepth = 1.0;
    pp.cutDepth = 7.0;
    pp.passDepth = 2.5;
    pp.maintainExactStep = false;
    pp.lastPassThickness = 0.5;
    CHECK(res.plans[0].levels == pp.levels());
    const Toolpath& tp = res.toolpaths[0];
    CHECK_THAT(*tp.minZ(), WithinAbs(-8.0, 1e-12));
    // Every move below the start depth is a cutting move; rapids stay at safe Z.
    for (const Move& mv : tp.moves) {
        if (mv.kind == MoveKind::Rapid) {
            CHECK(mv.to.z == p.safeZ);
        }
    }
    CHECK(tp.tool == t.tool);
    CHECK(tp.name == "Pocket");
}

TEST_CASE("Pocket: ramp plunge enters along the path and reaches the level exactly",
          "[toolpath][pocket][ramp]") {
    for (PocketStrategy s : {PocketStrategy::Offset, PocketStrategy::Raster}) {
        INFO("strategy " << static_cast<int>(s));
        PocketParams p = params(6.0);
        p.strategy = s;
        p.rampPlunge = true;
        p.rampDistance = 8.0;
        const PocketResult res = pocketToolpath({rect(0, 0, 50, 30)}, {tool(6.0, 0.4, 3.0)}, p);
        const Toolpath& tp = res.toolpaths[0];
        std::size_t ramps = 0;
        for (std::size_t i = 0; i < tp.moves.size(); ++i) {
            const Move& mv = tp.moves[i];
            const Vec3 from = tp.startOf(i);
            if (mv.kind == MoveKind::Plunge) {
                CHECK(mv.to.z >= -p.startDepth);  // plunges only through the air
                CHECK(distance(mv.to.xy(), from.xy()) == 0.0);
            }
            if (mv.kind == MoveKind::Feed && mv.to.z < from.z) {
                // Start of a descent: follow it to the bottom and measure the XY travel.
                if (i > 0 && tp.moves[i - 1].kind == MoveKind::Feed &&
                    tp.moves[i - 1].to.z < tp.startOf(i - 1).z) {
                    continue;
                }
                const Vec3 rampStart = from;
                double xy = 0.0;
                std::size_t j = i;
                while (j < tp.moves.size() && tp.moves[j].kind == MoveKind::Feed &&
                       tp.moves[j].to.z < tp.startOf(j).z) {
                    xy += distance(tp.moves[j].to.xy(), tp.startOf(j).xy());
                    ++j;
                }
                ++ramps;
                const Vec3 end = tp.moves[j - 1].to;
                CHECK(distance(end.xy(), rampStart.xy()) <= 1e-12);  // back at the start point
                CHECK((end.z == -3.0 || end.z == -6.0));             // exactly at a level
                CHECK(xy <= 2 * p.rampDistance + 1e-9);
                const bool longPath =
                    s == PocketStrategy::Offset;  // rings are long; raster lines may be short
                if (longPath) {
                    CHECK_THAT(xy, WithinAbs(2 * p.rampDistance, 1e-9));
                }
            }
        }
        CHECK(ramps >= 2);
        CHECK(tp.count(MoveKind::Plunge) >= 1);
    }
}

TEST_CASE("Pocket: multi-tool naming, rest messages, determinism", "[toolpath][pocket][rest]") {
    PocketParams p = params(3.0);
    p.name = "Clear";
    const std::vector<PocketTool> tools{tool(12.0, 0.4, 3.0, "Big"), tool(3.0, 0.4, 3.0, "Small"),
                                        tool(2.0, 0.4, 3.0, "Tiny")};
    const PocketResult a = pocketToolpath({rect(0, 0, 50, 30)}, tools, p);
    const PocketResult b = pocketToolpath({rect(0, 0, 50, 30)}, tools, p);
    REQUIRE(a.toolpaths.size() == 3);
    CHECK(a.toolpaths[0].name == "Clear [Big]");
    CHECK(a.toolpaths[1].name == "Clear [Small]");
    CHECK(a.toolpaths[2].name == "Clear [Tiny]");
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(a.toolpaths[i] == b.toolpaths[i]);
    }
    CHECK(a.plans[0].restMaterial.empty());
    CHECK_FALSE(a.plans[1].restMaterial.empty());
    CHECK(a.plans[1].restMaterial.size() == 4);  // the four corners
    // The rest tool's paths group into one cluster per corner.
    std::size_t maxCluster = 0;
    for (const PocketPath& pp : a.plans[1].paths) {
        maxCluster = std::max(maxCluster, pp.cluster);
    }
    CHECK(maxCluster == 3);
    // The 2 mm tool still finds the tips the 3 mm tool left in the corners.
    CHECK_FALSE(a.toolpaths[2].isEmpty());
}

TEST_CASE("Pocket: invalid input is reported, not cut", "[toolpath][pocket]") {
    SECTION("open contours are ignored") {
        Contour open;
        open.append(carvetto::geometry::LineSpan{{0, 0}, {10, 0}});
        const PocketResult r = pocketToolpath({open, rect(0, 0, 20, 20)}, {tool(6.0, 0.4, 3.0)}, params());
        CHECK_FALSE(r.toolpaths[0].isEmpty());
        REQUIRE(r.messages.size() == 1);
        CHECK(r.messages[0].find("open contour") != std::string::npos);
    }
    SECTION("only open contours") {
        Contour open;
        open.append(carvetto::geometry::LineSpan{{0, 0}, {10, 0}});
        const PocketResult r = pocketToolpath({open}, {tool(6.0, 0.4, 3.0)}, params());
        CHECK(r.toolpaths[0].isEmpty());
        CHECK(r.messages.size() == 2);
    }
    SECTION("tool too large") {
        const PocketResult r = pocketToolpath({rect(0, 0, 5, 5)}, {tool(6.0, 0.4, 3.0)}, params());
        CHECK(r.toolpaths[0].isEmpty());
        REQUIRE(r.messages.size() == 1);
        CHECK(r.messages[0].find("too large") != std::string::npos);
    }
    SECTION("stepover larger than the tool, zero pass depth") {
        const PocketResult r =
            pocketToolpath({rect(0, 0, 50, 50)}, {tool(6.0, 1.5, 3.0), tool(6.0, 0.5, 0.0)}, params());
        CHECK(r.toolpaths[0].isEmpty());
        CHECK(r.toolpaths[1].isEmpty());
        CHECK(r.messages.size() == 2);
    }
    SECTION("skipped first tool: the next one clears") {
        const PocketResult r =
            pocketToolpath({rect(0, 0, 50, 50)}, {tool(6.0, 1.5, 3.0), tool(6.0, 0.5, 3.0)}, params());
        CHECK(r.toolpaths[0].isEmpty());
        CHECK_FALSE(r.toolpaths[1].isEmpty());
        CHECK(r.plans[1].restMaterial.empty());
    }
    SECTION("no tools, no depth") {
        CHECK(pocketToolpath({rect(0, 0, 50, 50)}, {}, params()).messages.size() == 1);
        const PocketResult r = pocketToolpath({rect(0, 0, 50, 50)}, {tool(6.0, 0.4, 3.0)}, params(0.0));
        CHECK(r.toolpaths[0].isEmpty());
        CHECK(r.messages.size() == 1);
    }
    SECTION("nested contours alternate pocket / island / pocket (even-odd)") {
        const PocketResult r =
            pocketToolpath({rect(0, 0, 60, 60), rect(10, 10, 50, 50), rect(20, 20, 40, 40)},
                           {tool(4.0, 0.4, 3.0)}, params());
        // Nothing is cut between 10 and 20 (the island ring), the middle square is pocketed.
        const orc::GougeReport g = orc::gougeCheck(
            r.toolpaths[0],
            {orc::rectPoly(0, 0, 60, 60), orc::rectPoly(10, 10, 50, 50), orc::rectPoly(20, 20, 40, 40)}, 0.0,
            5.0);
        CHECK(g.allInside);
        CHECK(g.minDistance >= 2.0);
        bool middle = false;
        for (const Move& mv : r.toolpaths[0].moves) {
            middle = middle || (mv.to.z < 0 && mv.to.x > 21 && mv.to.x < 39 && mv.to.y > 21 && mv.to.y < 39);
        }
        CHECK(middle);
    }
}

TEST_CASE("Regression: travel never crosses a thin fin through its corner vertices",
          "[toolpath][pocket][gouge]") {
    // Round-2 gouge sweep, verifier scene 84: a U pocket around a 1.63 mm fin, raster with the
    // boundary first. The raster ends, projected onto their exact line, sat ~1e-7 mm off the
    // centre region's top edge, so a lifted travel along that line slipped through the fin's
    // corner vertices without registering a crossing and came 0.0022 mm too close to the fin.
    const orc::Poly u{{14.66107, 15.29806}, {46.90379, 15.29806}, {46.90379, 47.54078}, {31.59986, 47.54078},
                      {31.59986, 24.97087}, {29.96501, 24.97087}, {29.96501, 47.54078}, {14.66107, 47.54078}};
    PocketTool t1 = tool(2.1278, 1.0, 1.8905, "T1");
    t1.stepover = 1.6948;
    PocketTool t2 = tool(0.9164, 1.0, 3.8357, "T2");
    t2.stepover = 0.3921;
    for (ProfilePass pass : {ProfilePass::First, ProfilePass::Last, ProfilePass::None}) {
        for (PocketStrategy strategy : {PocketStrategy::Raster, PocketStrategy::Offset}) {
            INFO("pass " << static_cast<int>(pass) << " strategy " << static_cast<int>(strategy));
            PocketParams p = params(4.594);
            p.strategy = strategy;
            p.rasterAngle = -0.2235;
            p.profilePass = pass;
            p.rampPlunge = true;
            p.rampDistance = 17.076;
            const PocketResult res = pocketToolpath({Contour::fromPolyline(Polyline{u, true})}, {t1, t2}, p);
            for (std::size_t i = 0; i < 2; ++i) {
                const Toolpath& tp = res.toolpaths[i];
                if (tp.isEmpty()) {
                    continue;
                }
                const orc::GougeReport g = orc::gougeCheck(tp, {u}, 1e-9, p.safeZ);  // surface moves too
                CHECK(g.allInside);
                CHECK(g.minDistance >= 0.5 * tp.tool.diameter);
            }
        }
    }
}
