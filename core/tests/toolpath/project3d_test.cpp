#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/relief.hpp"
#include "carvetto/simulate/preview_component.hpp"
#include "carvetto/simulate/simulator.hpp"
#include "carvetto/toolpath/project3d.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using relief::HeightField;

namespace {

/// Model of `w` x `h` nodes at cell size `cs`, origin (0, 0), height f(x, y).
template <class F>
HeightField makeField(std::size_t w, std::size_t h, double cs, F f) {
    relief::Grid g{w, h, cs, {0.0, 0.0}};
    std::vector<float> d(w * h);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            d[j * w + i] = static_cast<float>(f(static_cast<double>(i) * cs, static_cast<double>(j) * cs));
        }
    }
    HeightField out;
    REQUIRE(HeightField::fromData(g, std::move(d), out) == relief::Status::Ok);
    return out;
}

tooling::Tool makeTool(tooling::ToolType type, double diameter) {
    tooling::Tool t;
    t.id = "T";
    t.name = "T";
    t.type = type;
    t.diameter = diameter;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 1.0;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 18000.0;
    t.defaultCuttingData.toolNumber = 1;
    return t;
}

Toolpath lineAt(double x0, double x1, double y, double z, double diameter = 2.0) {
    Toolpath tp;
    tp.name = "in";
    tp.tool.diameter = diameter;
    tp.append(Move::rapid({x0, y, 5.0}));
    tp.append(Move::plunge({x0, y, z}));
    tp.append(Move::feed({x1, y, z}));
    tp.append(Move::retract({x1, y, 5.0}));
    return tp;
}

bool hasIssueOf(const Project3dResult& r, ToolpathIssue i) {
    return hasIssue(r.diagnostics, i);
}

/// Z of the path at X along a straight path in X (feed moves only); the first matching piece.
double zAtX(const Toolpath& tp, double x) {
    for (std::size_t i = 1; i < tp.moves.size(); ++i) {
        if (tp.moves[i].kind != MoveKind::Feed) {
            continue;
        }
        const Vec3 a = tp.startOf(i);
        const Vec3 b = tp.moves[i].to;
        const double lo = std::min(a.x, b.x);
        const double hi = std::max(a.x, b.x);
        if (x >= lo - 1e-9 && x <= hi + 1e-9 && hi > lo) {
            const double t = (x - a.x) / (b.x - a.x);
            return a.z + t * (b.z - a.z);
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

TEST_CASE("project3d: project mode puts the depth below a tilted plane exactly", "[project3d]") {
    const HeightField m = makeField(12, 8, 1.0, [](double x, double) { return -2.0 + 0.5 * x; });
    Project3dParams p;
    p.mode = ProjectTipMode::Project;
    const Project3dResult r = projectToolpath(lineAt(1.0, 9.0, 3.0, -1.0), m,
                                              makeTool(tooling::ToolType::EndMill, 2.0), p);
    REQUIRE(r.diagnostics.empty());
    CHECK(r.path.name == "in (3D)");
    // Rapid and the safe heights are untouched.
    CHECK(r.path.moves.front() == Move::rapid({1.0, 3.0, 5.0}));
    CHECK(r.path.moves.back().to.z == 5.0);
    std::size_t feeds = 0;
    for (std::size_t i = 0; i < r.path.moves.size(); ++i) {
        const Move& mv = r.path.moves[i];
        if (mv.kind == MoveKind::Feed) {
            ++feeds;
            CHECK(std::fabs(mv.to.z - (-2.0 + 0.5 * mv.to.x - 1.0)) < 1e-6);
        }
    }
    CHECK(feeds == 8);  // one piece per cell: the surface is exact, only maxSegment splits
    // The plunge ends on the surface minus the depth.
    CHECK(std::fabs(r.path.moves[1].to.z - (-2.0 + 0.5 - 1.0)) < 1e-6);
    CHECK(r.path.moves[1].kind == MoveKind::Plunge);
}

TEST_CASE("project3d: drop mode lifts a flat tool over a step, project mode does not", "[project3d]") {
    const HeightField m = makeField(12, 8, 1.0, [](double x, double) { return x < 5.0 ? -5.0 : 0.0; });
    const tooling::Tool flat = makeTool(tooling::ToolType::EndMill, 2.0);
    Project3dParams p;
    p.mode = ProjectTipMode::DropTool;
    const Project3dResult d = projectToolpath(lineAt(2.0, 4.0, 3.0, 0.0), m, flat, p);
    REQUIRE(d.diagnostics.empty());
    // At x = 4 the flat tool (radius 1) already reaches the wall at x = 5: the tip is at 0.
    CHECK(std::fabs(zAtX(d.path, 4.0) - 0.0) < 1e-6);
    CHECK(std::fabs(zAtX(d.path, 3.0) - (-5.0)) < 1e-6);
    CHECK(std::fabs(zAtX(d.path, 2.0) - (-5.0)) < 1e-6);
    p.mode = ProjectTipMode::Project;
    const Project3dResult pr = projectToolpath(lineAt(2.0, 4.0, 3.0, 0.0), m, flat, p);
    CHECK(std::fabs(zAtX(pr.path, 4.0) - (-5.0)) < 1e-6);
    // A bigger tool reaches further.
    p.mode = ProjectTipMode::DropTool;
    const Project3dResult big = projectToolpath(lineAt(2.0, 4.0, 3.0, 0.0), m, makeTool(tooling::ToolType::EndMill, 4.0), p);
    CHECK(std::fabs(zAtX(big.path, 3.0) - 0.0) < 1e-6);
}

TEST_CASE("project3d: zOffset, allowance, floor, outside and safe heights", "[project3d]") {
    const HeightField m = makeField(10, 10, 1.0, [](double, double) { return -3.0; });
    const tooling::Tool t = makeTool(tooling::ToolType::BallNose, 2.0);
    Project3dParams p;
    p.mode = ProjectTipMode::Project;
    p.zOffset = 10.0;
    p.allowance = 0.5;
    p.stockBottom = -8.0;
    Toolpath in = lineAt(1.0, 5.0, 4.0, -1.0);
    in.moves.push_back(Move::dwell({5.0, 4.0, 5.0}, 1.0));
    Project3dResult r = projectToolpath(in, m, t, p);
    REQUIRE(r.diagnostics.empty());
    // Surface at machine Z = -3 + 10 + 0.5 = 7.5; depth 1 below it.
    CHECK(std::fabs(zAtX(r.path, 3.0) - 6.5) < 1e-6);
    CHECK(r.path.moves.back().kind == MoveKind::Dwell);
    CHECK(r.path.moves.back().to == Vec3{5.0, 4.0, 5.0});
    // Deep cut is capped at the material bottom.
    p.zOffset = 0.0;
    p.allowance = 0.0;
    r = projectToolpath(lineAt(1.0, 5.0, 4.0, -9.0), m, t, p);
    CHECK(zAtX(r.path, 3.0) == -8.0);
    // Outside the model the surface is `outsideSurface`: depth stays as given.
    p.outsideSurface = 1.0;
    r = projectToolpath(lineAt(11.0, 14.0, 4.0, -2.0), m, t, p);
    CHECK(std::fabs(zAtX(r.path, 12.0) - (-1.0)) < 1e-9);
    // A transparent node makes its neighbourhood "outside".
    HeightField hole = makeField(10, 10, 1.0, [](double, double) { return -3.0; });
    hole.set(5, 4, relief::kTransparent);
    r = projectToolpath(lineAt(1.0, 8.0, 4.0, -2.0), hole, t, p);
    CHECK(std::fabs(zAtX(r.path, 5.0) - (-1.0)) < 1e-9);
    CHECK(std::fabs(zAtX(r.path, 2.0) - (-5.0)) < 1e-9);
}

TEST_CASE("project3d: arcs become chords within the tolerance and keep the helix", "[project3d]") {
    const HeightField m = makeField(20, 20, 1.0, [](double, double) { return -2.0; });
    Project3dParams p;
    p.mode = ProjectTipMode::Project;
    p.tolerance = 0.01;
    Toolpath in;
    in.name = "circle";
    in.append(Move::rapid({13.0, 10.0, 5.0}));
    in.append(Move::plunge({13.0, 10.0, -1.0}));
    in.append(Move::arcCCW({13.0, 10.0, -2.0}, {10.0, 10.0}));
    const Project3dResult r = projectToolpath(in, m, makeTool(tooling::ToolType::EndMill, 2.0), p);
    REQUIRE(r.diagnostics.empty());
    std::size_t feeds = 0;
    double prevZ = -3.0;
    for (std::size_t i = 2; i < r.path.moves.size(); ++i) {
        const Move& mv = r.path.moves[i];
        REQUIRE(mv.kind == MoveKind::Feed);
        ++feeds;
        CHECK(std::fabs(std::hypot(mv.to.x - 10.0, mv.to.y - 10.0) - 3.0) < 1e-9);
        CHECK(mv.to.z <= prevZ + 1e-12);  // the helix only descends
        prevZ = mv.to.z;
        const Vec3 a = r.path.startOf(i);
        const double mid = std::hypot(0.5 * (a.x + mv.to.x) - 10.0, 0.5 * (a.y + mv.to.y) - 10.0);
        CHECK(3.0 - mid <= 0.01 + 1e-9);  // chordal tolerance
    }
    CHECK(feeds >= 20);
    CHECK(std::fabs(r.path.moves.back().to.z - (-4.0)) < 1e-9);
    CHECK(std::fabs(r.path.moves.back().to.x - 13.0) < 1e-9);
}

TEST_CASE("project3d: pieces stay on a curved surface within the tolerance", "[project3d]") {
    const HeightField m = makeField(30, 6, 1.0, [](double x, double) { return -4.0 + 3.0 * std::sin(x * 0.9); });
    Project3dParams p;
    p.mode = ProjectTipMode::Project;
    p.tolerance = 0.02;
    p.maxSegment = 5.0;  // longer than a cell: refinement must do the work
    const Project3dResult r = projectToolpath(lineAt(1.0, 27.0, 2.0, -0.5), m,
                                              makeTool(tooling::ToolType::BallNose, 2.0), p);
    REQUIRE(r.diagnostics.empty());
    std::size_t pieces = 0;
    for (std::size_t i = 2; i < r.path.moves.size(); ++i) {
        const Move& mv = r.path.moves[i];
        if (mv.kind != MoveKind::Feed) {
            continue;
        }
        ++pieces;
        const Vec3 a = r.path.startOf(i);
        for (const double t : {0.25, 0.5, 0.75}) {
            const double x = a.x + t * (mv.to.x - a.x);
            const double z = a.z + t * (mv.to.z - a.z);
            const double surf = m.sample(x, 2.0) - 0.5;
            CHECK(std::fabs(z - surf) <= 2.0 * 0.02 + 1e-6);
        }
        CHECK(std::fabs(mv.to.z - (m.sample(mv.to.x, 2.0) - 0.5)) < 1e-5);  // end points exactly on it
    }
    CHECK(pieces > 6);  // five-mm pieces would be six
}

TEST_CASE("project3d: drop mode with a ball nose never gouges a tilted plane (simulator)", "[project3d]") {
    const double slope = 0.5;
    const HeightField m = makeField(40, 12, 1.0, [&](double x, double) { return -20.0 + slope * x; });
    const tooling::Tool ball = makeTool(tooling::ToolType::BallNose, 4.0);
    Project3dParams p;
    p.mode = ProjectTipMode::DropTool;
    p.stockBottom = -28.0;
    const Project3dResult r = projectToolpath(lineAt(6.0, 30.0, 6.0, 0.0, 4.0), m, ball, p);
    REQUIRE(r.diagnostics.empty());
    simulate::Stock st;
    st.width = 39.75;
    st.height = 11.75;
    st.thickness = 30.0;
    st.cellSize = 1.0;
    st.origin = {-0.5, -0.5};
    st.topZ = 0.0;
    st.trackPassIds = false;
    simulate::Simulator sim(st);
    REQUIRE(sim.ok());
    const auto shape = simulate::ToolShape::fromTool(ball);
    REQUIRE(shape);
    REQUIRE(sim.addToolpath(r.path, *shape, 1));
    REQUIRE(sim.advanceAll());
    const double c0 = 2.0 * (std::sqrt(1.0 + slope * slope) - 1.0);  // tangent lift of the ball on the slope
    for (std::size_t i = 8; i <= 28; ++i) {
        const double h = sim.heightfield().height(i, 6);
        const double plane = -20.0 + slope * static_cast<double>(i);
        CHECK(h >= plane - 0.01);
        CHECK(h <= plane + c0 + 0.02);
    }
    // Project mode on the same plane gouges it, which is why the drop mode exists.
    p.mode = ProjectTipMode::Project;
    const Project3dResult g = projectToolpath(lineAt(6.0, 30.0, 6.0, 0.0, 4.0), m, ball, p);
    simulate::Simulator sim2(st);
    REQUIRE(sim2.addToolpath(g.path, *shape, 1));
    REQUIRE(sim2.advanceAll());
    CHECK(sim2.heightfield().height(18, 6) < -20.0 + slope * 18.0 - 0.1);
}

TEST_CASE("project3d: tabs raise the cut over the tab stretches (vertical steps)", "[project3d]") {
    const HeightField m = makeField(50, 8, 1.0, [](double, double) { return -2.0; });
    Project3dParams p;
    p.mode = ProjectTipMode::Project;
    p.tabs.enabled = true;
    p.tabs.length = 4.0;
    p.tabs.thickness = 1.5;
    p.tabs.count = 2;
    p.tabs.threeD = false;
    // Tool diameter 2: raised stretch 6 mm around centres 10 and 30 of the 40 mm run.
    const Project3dResult r = projectToolpath(lineAt(0.0, 40.0, 3.0, -3.0), m,
                                              makeTool(tooling::ToolType::EndMill, 2.0), p);
    REQUIRE(r.diagnostics.empty());
    const auto zAt = [&](double x, bool afterStep) {
        // Height at x; at a step X the later (afterStep) or earlier move's end.
        double found = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t i = 2; i < r.path.moves.size(); ++i) {
            const Move& mv = r.path.moves[i];
            if (mv.kind != MoveKind::Feed) {
                continue;
            }
            const Vec3 a = r.path.startOf(i);
            if (std::fabs(mv.to.x - x) < 1e-9 && std::fabs(a.x - x) < 1e-9) {
                found = afterStep ? mv.to.z : a.z;  // vertical piece: start is the low end
                if (afterStep) {
                    return found;
                }
            } else if (a.x - 1e-9 <= x && x <= mv.to.x + 1e-9 && mv.to.x > a.x) {
                found = a.z + (x - a.x) / (mv.to.x - a.x) * (mv.to.z - a.z);
                return found;
            }
        }
        return found;
    };
    CHECK(std::fabs(zAt(3.0, false) - (-5.0)) < 1e-6);
    CHECK(std::fabs(zAt(10.0, false) - (-3.5)) < 1e-6);
    CHECK(std::fabs(zAt(30.0, false) - (-3.5)) < 1e-6);
    CHECK(std::fabs(zAt(20.0, false) - (-5.0)) < 1e-6);
    CHECK(std::fabs(zAt(36.0, false) - (-5.0)) < 1e-6);
    // The steps are vertical: no slope leaks to either side of an edge.
    CHECK(std::fabs(zAt(6.99, false) - (-5.0)) < 1e-6);
    CHECK(std::fabs(zAt(7.01, false) - (-3.5)) < 1e-6);
    CHECK(std::fabs(zAt(12.99, false) - (-3.5)) < 1e-6);
    CHECK(std::fabs(zAt(13.01, false) - (-5.0)) < 1e-6);
    // The edges at 7, 13: a vertical Feed from -5 to -3.5 and back.
    std::size_t verticals = 0;
    for (std::size_t i = 2; i < r.path.moves.size(); ++i) {
        const Vec3 a = r.path.startOf(i);
        const Move& mv = r.path.moves[i];
        if (mv.kind == MoveKind::Feed && mv.to.x == a.x && mv.to.y == a.y && mv.to.z != a.z) {
            ++verticals;
            CHECK(std::fabs(std::fabs(mv.to.z - a.z) - 1.5) < 1e-6);
            const double x = a.x;
            CHECK((std::fabs(x - 7.0) < 1e-6 || std::fabs(x - 13.0) < 1e-6 || std::fabs(x - 27.0) < 1e-6 ||
                   std::fabs(x - 33.0) < 1e-6));
        }
    }
    CHECK(verticals == 4);
    // The plunge and the end are not raised.
    CHECK(r.path.moves[1].to.z == -5.0);
    CHECK(r.path.moves.back().to.z == 5.0);
}

TEST_CASE("project3d: 3D tabs ramp at 45 degrees and follow a sloped surface", "[project3d]") {
    const double slope = 0.25;
    const HeightField m = makeField(50, 8, 1.0, [&](double x, double) { return -2.0 + slope * x; });
    Project3dParams p;
    p.mode = ProjectTipMode::Project;
    p.tabs.enabled = true;
    p.tabs.length = 4.0;
    p.tabs.thickness = 1.5;
    p.tabs.count = 1;
    p.tabs.threeD = true;
    // One tab, centre 20, raised stretch [17, 23]; ramps 1.5 mm each side.
    const Project3dResult r = projectToolpath(lineAt(0.0, 40.0, 3.0, -3.0), m,
                                              makeTool(tooling::ToolType::EndMill, 2.0), p);
    REQUIRE(r.diagnostics.empty());
    const auto bottom = [&](double x) { return -2.0 + slope * x - 3.0; };
    CHECK(std::fabs(zAtX(r.path, 20.0) - (bottom(20.0) + 1.5)) < 1e-6);
    CHECK(std::fabs(zAtX(r.path, 17.0) - (bottom(17.0) + 1.5)) < 1e-6);
    CHECK(std::fabs(zAtX(r.path, 23.0) - (bottom(23.0) + 1.5)) < 1e-6);
    CHECK(std::fabs(zAtX(r.path, 15.5) - bottom(15.5)) < 1e-6);
    CHECK(std::fabs(zAtX(r.path, 24.5) - bottom(24.5)) < 1e-6);
    CHECK(std::fabs(zAtX(r.path, 16.25) - (bottom(16.25) + 0.75)) < 1e-6);
    CHECK(std::fabs(zAtX(r.path, 10.0) - bottom(10.0)) < 1e-6);
    // No vertical jumps.
    for (std::size_t i = 2; i < r.path.moves.size(); ++i) {
        const Vec3 a = r.path.startOf(i);
        const Move& mv = r.path.moves[i];
        if (mv.kind == MoveKind::Feed) {
            CHECK(mv.to.x != a.x);
        }
    }
}

TEST_CASE("project3d: tab diagnostics and parameter checks", "[project3d]") {
    const HeightField m = makeField(30, 8, 1.0, [](double, double) { return -2.0; });
    const tooling::Tool t = makeTool(tooling::ToolType::EndMill, 2.0);
    Project3dParams p;
    p.mode = ProjectTipMode::Project;
    p.tabs.enabled = true;
    // A surface cut (depth 0) has no bottom to raise from.
    Project3dResult r = projectToolpath(lineAt(1.0, 20.0, 3.0, 0.0), m, t, p);
    CHECK(hasIssueOf(r, ToolpathIssue::TabsNotPlaced));
    CHECK(!r.path.moves.empty());
    // A run shorter than a tab leaves none.
    p.tabs.length = 40.0;
    r = projectToolpath(lineAt(1.0, 5.0, 3.0, -1.0), m, t, p);
    CHECK(hasIssueOf(r, ToolpathIssue::TabsNotPlaced));
    for (const Move& mv : r.path.moves) {
        if (mv.kind == MoveKind::Feed) {
            CHECK(mv.to.z == -3.0);
        }
    }
    p.tabs.length = 4.0;
    p.tabs.thickness = 0.0;
    r = projectToolpath(lineAt(1.0, 20.0, 3.0, -1.0), m, t, p);
    CHECK(hasIssueOf(r, ToolpathIssue::InvalidParameters));
    CHECK(r.path.moves.empty());
}

TEST_CASE("project3d: invalid input and limits are refused", "[project3d]") {
    const HeightField m = makeField(10, 10, 1.0, [](double, double) { return -2.0; });
    const tooling::Tool t = makeTool(tooling::ToolType::EndMill, 2.0);
    const Toolpath in = lineAt(1.0, 8.0, 3.0, -1.0);
    {
        Project3dParams p;
        p.tolerance = 0.0;
        CHECK(hasIssueOf(projectToolpath(in, m, t, p), ToolpathIssue::InvalidParameters));
        p = Project3dParams{};
        p.allowance = -1.0;
        CHECK(hasIssueOf(projectToolpath(in, m, t, p), ToolpathIssue::InvalidParameters));
        p = Project3dParams{};
        p.zOffset = std::numeric_limits<double>::quiet_NaN();
        CHECK(hasIssueOf(projectToolpath(in, m, t, p), ToolpathIssue::InvalidParameters));
        p = Project3dParams{};
        CHECK(hasIssueOf(projectToolpath(in, HeightField{}, t, p), ToolpathIssue::InvalidParameters));
    }
    {
        Toolpath bad = in;
        bad.moves[2].to.x = std::numeric_limits<double>::infinity();
        CHECK(hasIssueOf(projectToolpath(bad, m, t, Project3dParams{}), ToolpathIssue::InvalidParameters));
    }
    {
        tooling::Tool none = t;
        none.type = tooling::ToolType::ThreadMill;
        CHECK(hasIssueOf(projectToolpath(in, m, none, Project3dParams{}), ToolpathIssue::InvalidTool));
        Project3dParams p;
        p.mode = ProjectTipMode::Project;  // the tool is not needed
        CHECK(projectToolpath(in, m, none, p).diagnostics.empty());
    }
    {
        std::atomic<bool> stop{true};
        ToolpathRunControl c;
        c.cancel = &stop;
        const Project3dResult r = projectToolpath(in, m, t, Project3dParams{}, c);
        CHECK(hasIssueOf(r, ToolpathIssue::Cancelled));
        CHECK(r.path.moves.empty());
    }
    {
        ToolpathRunControl c;
        c.moveLimit = 3;
        Project3dParams p;
        p.mode = ProjectTipMode::Project;
        const Project3dResult r = projectToolpath(in, m, t, p, c);
        CHECK(hasIssueOf(r, ToolpathIssue::WorkLimitExceeded));
        CHECK(r.path.moves.empty());
        ToolpathRunControl w;
        w.workLimit = 2;
        CHECK(hasIssueOf(projectToolpath(in, m, t, p, w), ToolpathIssue::WorkLimitExceeded));
        CHECK(hasIssueOf(projectToolpath(in, m, t, Project3dParams{}, w), ToolpathIssue::WorkLimitExceeded));
    }
}

TEST_CASE("preview component: matches the simulated heightfield", "[project3d][preview]") {
    simulate::Stock st;
    st.width = 20.0;
    st.height = 10.0;
    st.thickness = 10.0;
    st.cellSize = 0.5;
    st.origin = {3.0, -2.0};
    st.topZ = 0.0;
    simulate::Simulator sim(st);
    REQUIRE(sim.ok());
    const auto shape = simulate::ToolShape::ballNose(2.0);
    REQUIRE(shape);
    Toolpath tp;
    tp.tool.diameter = 2.0;
    tp.append(Move::rapid({6.0, 3.0, 5.0}));
    tp.append(Move::plunge({6.0, 3.0, -2.0}));
    tp.append(Move::feed({16.0, 3.0, -3.0}));
    tp.append(Move::retract({16.0, 3.0, 5.0}));
    REQUIRE(sim.addToolpath(tp, *shape, 1));
    REQUIRE(sim.advanceAll());
    const simulate::Heightfield& hf = sim.heightfield();

    relief::Component c;
    simulate::PreviewComponentOptions o;
    o.name = "Preview";
    REQUIRE(simulate::componentFromPreview(hf, o, c) == relief::Status::Ok);
    CHECK(c.name == "Preview");
    CHECK(c.mode == relief::CombineMode::Add);
    CHECK(c.field.width() == hf.nx());
    CHECK(c.field.height() == hf.ny());
    CHECK(c.field.cellSize() == hf.cellSize());
    CHECK(c.field.origin().x == 3.25);
    CHECK(c.field.origin().y == -1.75);
    CHECK(c.field.data() == hf.heights());
    // The component's node (i, j) is the centre of simulator cell (i, j).
    const geometry::Vec2 ctr = hf.centreOf(7, 9);
    CHECK(c.field.sample(ctr.x, ctr.y) == hf.height(7, 9));
    // Bake keeps the field as is.
    relief::HeightField baked;
    REQUIRE(relief::bake(c, baked) == relief::Status::Ok);
    CHECK(baked.data() == hf.heights());

    o.cells = simulate::PreviewCells::CutOnly;
    o.zOffset = 4.0;
    o.mode = relief::CombineMode::MergeLow;
    relief::Component cut;
    REQUIRE(simulate::componentFromPreview(hf, o, cut) == relief::Status::Ok);
    CHECK(cut.mode == relief::CombineMode::MergeLow);
    std::size_t opaque = 0;
    for (std::size_t j = 0; j < hf.ny(); ++j) {
        for (std::size_t i = 0; i < hf.nx(); ++i) {
            const float v = cut.field.at(i, j);
            if (hf.isCut(i, j)) {
                ++opaque;
                CHECK(v == static_cast<float>(static_cast<double>(hf.height(i, j)) - 4.0));
            } else {
                CHECK(std::isnan(v));
            }
        }
    }
    CHECK(opaque == hf.cutCells());
    CHECK(opaque > 0);
    CHECK(opaque < hf.cellCount());

    o.zOffset = std::numeric_limits<double>::infinity();
    relief::Component keep = c;
    CHECK(simulate::componentFromPreview(hf, o, keep) == relief::Status::BadParameter);
    CHECK(keep.name == "Preview");
    CHECK(simulate::componentFromPreview(simulate::Heightfield{}, simulate::PreviewComponentOptions{}, keep) ==
          relief::Status::ZeroSize);
}

TEST_CASE("preview component: a roughed model round-trips through the simulator", "[project3d][preview]") {
    // Cut a known flat pocket and read it back: the component is the pocket floor.
    simulate::Stock st;
    st.width = 10.0;
    st.height = 10.0;
    st.thickness = 5.0;
    st.cellSize = 1.0;
    simulate::Simulator sim(st);
    REQUIRE(sim.ok());
    const auto shape = simulate::ToolShape::flatEndMill(2.0);
    REQUIRE(shape);
    Toolpath tp;
    tp.append(Move::rapid({5.0, 2.5, 5.0}));
    tp.append(Move::plunge({5.0, 2.5, -2.0}));
    for (double y = 2.5; y <= 7.5; y += 1.0) {
        tp.append(Move::feed({5.0, y, -2.0}));
    }
    REQUIRE(sim.addToolpath(tp, *shape, 1));
    REQUIRE(sim.advanceAll());
    relief::Component c;
    REQUIRE(simulate::componentFromPreview(sim.heightfield(), simulate::PreviewComponentOptions{}, c) ==
            relief::Status::Ok);
    CHECK(c.field.at(5, 5) == -2.0f);
    CHECK(c.field.at(0, 0) == 0.0f);
    CHECK(c.field.at(9, 9) == 0.0f);
}
