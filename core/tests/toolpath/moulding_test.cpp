#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/simulate/simulator.hpp"
#include "carvetto/toolpath/moulding.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Polyline;
using geometry::Vec2;

namespace {

Rough3dTool makeTool(tooling::ToolType type, double diameter, double stepover = 1.0, double passDepth = 1.0) {
    tooling::Tool t;
    t.id = "T";
    t.name = "T";
    t.type = type;
    t.diameter = diameter;
    t.defaultCuttingData.passDepth = passDepth;
    t.defaultCuttingData.stepover = stepover;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 18000.0;
    t.defaultCuttingData.toolNumber = 1;
    Rough3dTool r;
    r.tool = t;
    return r;
}

Polyline open(std::vector<Vec2> pts) {
    Polyline p;
    p.points = std::move(pts);
    return p;
}

MouldingParams baseParams() {
    MouldingParams p;
    p.stockTop = 10.0;
    p.stockBottom = -20.0;
    p.safeZ = 15.0;
    p.rails = {open({{0.0, 0.0}, {20.0, 0.0}})};
    p.profile = open({{-5.0, -2.5}, {5.0, 2.5}});  // plane of slope 0.5
    return p;
}

/// One rapid..retract group of a straight rail along X.
struct PassRec {
    double z{0.0};
    double y{0.0};
    double xStart{0.0};
    double xEnd{0.0};
    std::size_t feeds{0};
};

std::vector<PassRec> passesOf(const Toolpath& tp) {
    std::vector<PassRec> out;
    PassRec cur;
    bool inPass = false;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Rapid) {
            cur = PassRec{};
            cur.xStart = m.to.x;
            inPass = true;
        } else if (m.kind == MoveKind::Feed && inPass) {
            cur.z = m.to.z;
            cur.y = m.to.y;
            cur.xEnd = m.to.x;
            ++cur.feeds;
        } else if (m.kind == MoveKind::Retract && inPass) {
            out.push_back(cur);
            inPass = false;
        }
    }
    return out;
}

double ballLift(double r, double slope) {
    return r * (std::sqrt(1.0 + slope * slope) - 1.0);
}

bool hasIssueOf(const MouldingResult& r, ToolpathIssue i) {
    return hasIssue(r.diagnostics, i);
}

/// Heights of the section (x across, y height), linear between points.
double heightAt(const std::vector<Vec2>& pts, double x) {
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        if (x >= pts[i].x && x <= pts[i + 1].x) {
            const double t = (x - pts[i].x) / (pts[i + 1].x - pts[i].x);
            return pts[i].y + t * (pts[i + 1].y - pts[i].y);
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

simulate::Stock simStock() {
    simulate::Stock st;
    st.width = 24.0;
    st.height = 24.0;
    st.thickness = 30.0;
    st.cellSize = 0.5;
    st.origin = {-2.0, -12.0};
    st.topZ = 10.0;
    st.trackPassIds = false;
    return st;
}

const std::vector<Vec2> kSection = {{-6.0, -3.0}, {-3.0, -1.0}, {0.0, -4.0}, {3.0, -1.0}, {6.0, -3.0}};

}  // namespace

TEST_CASE("moulding: drop height is exact for a ball on a plane and a flat tool on a step", "[moulding]") {
    const auto ball = simulate::ToolShape::ballNose(2.0);
    REQUIRE(ball);
    const std::vector<Vec2> plane = {{-10.0, -5.0}, {10.0, 5.0}};
    for (const double u : {-4.0, 0.0, 3.3}) {
        CHECK(mouldingDropHeight(plane, *ball, u) == Catch::Approx(0.5 * u + ballLift(1.0, 0.5)).margin(1e-9));
    }
    const auto flat = simulate::ToolShape::flatEndMill(2.0);
    REQUIRE(flat);
    const std::vector<Vec2> step = {{-5.0, 0.0}, {0.0, 0.0}, {0.0, 2.0}, {5.0, 2.0}};
    CHECK(mouldingDropHeight(step, *flat, -1.5) == Catch::Approx(0.0).margin(1e-12));
    CHECK(mouldingDropHeight(step, *flat, -1.0) == Catch::Approx(2.0).margin(1e-12));
    CHECK(mouldingDropHeight(step, *flat, -0.2) == Catch::Approx(2.0).margin(1e-12));
    CHECK(mouldingDropHeight(step, *flat, 4.0) == Catch::Approx(2.0).margin(1e-12));
    // A tool far from any section point sees nothing.
    CHECK(mouldingDropHeight(step, *flat, 20.0) == -std::numeric_limits<double>::infinity());
}

TEST_CASE("moulding: finish puts every pass at the exact drop height at its offset", "[moulding]") {
    MouldingParams p = baseParams();
    p.zOffset = 3.0;
    const MouldingResult r = mouldingToolpath(makeTool(tooling::ToolType::BallNose, 2.0), p);
    REQUIRE(r.toolpaths.size() == 1);
    CHECK(r.toolpaths[0].name == "Moulding");
    const std::vector<PassRec> ps = passesOf(r.toolpaths[0]);
    REQUIRE(ps.size() == 11);  // offsets -5 .. 5, stepover 1
    // Climb: the next pass is on the right of travel, so offsets descend; the first is the leftmost.
    CHECK(ps.front().y == Catch::Approx(5.0).margin(1e-9));
    CHECK(ps.back().y == Catch::Approx(-5.0).margin(1e-9));
    for (const PassRec& pr : ps) {
        if (std::fabs(pr.y) <= 4.0) {
            CHECK(pr.z == Catch::Approx(3.0 + 0.5 * pr.y + ballLift(1.0, 0.5)).margin(1e-8));
        }
        // Zig-zag alternates direction; each pass covers the whole rail.
        CHECK(std::fabs(pr.xEnd - pr.xStart) == Catch::Approx(20.0).margin(1e-9));
    }
    CHECK(ps[0].xEnd > ps[0].xStart);
    CHECK(ps[1].xEnd < ps[1].xStart);
    // The offset is to the left of the rail direction (+Y for a rail along +X).
    CHECK(r.toolpaths[0].tool.diameter == 2.0);
}

TEST_CASE("moulding: step order, travel and reverse steps", "[moulding]") {
    MouldingParams p = baseParams();
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0);
    p.direction = CutDirection::Conventional;
    auto ps = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    CHECK(ps.front().y == Catch::Approx(-5.0).margin(1e-9));
    p.reverseSteps = true;
    ps = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    CHECK(ps.front().y == Catch::Approx(5.0).margin(1e-9));
    p.travel = RasterTravel::OneWay;
    ps = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    for (const PassRec& pr : ps) {
        CHECK(pr.xEnd > pr.xStart);
    }
}

TEST_CASE("moulding: toolpath position anchors the section on the rail", "[moulding]") {
    MouldingParams p = baseParams();
    p.profile = open({{2.0, 0.0}, {6.0, 0.0}});
    const Rough3dTool em = makeTool(tooling::ToolType::EndMill, 1.0, 0.5);
    const auto range = [&](MouldingPosition pos) {
        p.position = pos;
        const auto ps = passesOf(mouldingToolpath(em, p).toolpaths[0]);
        double lo = 1e9;
        double hi = -1e9;
        for (const PassRec& pr : ps) {
            lo = std::min(lo, pr.y);
            hi = std::max(hi, pr.y);
        }
        return std::pair<double, double>{lo, hi};
    };
    CHECK(range(MouldingPosition::AsDrawn) == std::pair<double, double>{2.0, 6.0});
    CHECK(range(MouldingPosition::Start) == std::pair<double, double>{0.0, 4.0});
    CHECK(range(MouldingPosition::Centre) == std::pair<double, double>{-2.0, 2.0});
    CHECK(range(MouldingPosition::End) == std::pair<double, double>{-4.0, 0.0});
    // A profile drawn right to left: Start/End follow the drawn order.
    p.profile = open({{6.0, 0.0}, {2.0, 0.0}});
    CHECK(range(MouldingPosition::Start) == std::pair<double, double>{-4.0, 0.0});
    CHECK(range(MouldingPosition::End) == std::pair<double, double>{0.0, 4.0});
}

TEST_CASE("moulding: reverse profile mirrors the section about the anchor, reverse rails flips the side",
          "[moulding]") {
    MouldingParams p = baseParams();
    p.profile = open({{0.0, -4.0}, {4.0, -1.0}, {6.0, -3.0}});
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0, 0.5);
    const auto a = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    p.reverseProfile = true;
    const auto b = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    REQUIRE(a.size() == b.size());
    CHECK(a.size() > 10);
    for (const PassRec& pa : a) {
        const auto it = std::find_if(b.begin(), b.end(),
                                     [&](const PassRec& pb) { return std::fabs(pb.y + pa.y) < 1e-9; });
        REQUIRE(it != b.end());
        CHECK(it->z == Catch::Approx(pa.z).margin(1e-9));
    }
    p.reverseProfile = false;
    p.reverseRails = true;
    p.travel = RasterTravel::OneWay;
    const auto c = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    REQUIRE(c.size() == a.size());
    for (const PassRec& pc : c) {
        CHECK(pc.y <= 1e-9);  // section now on the right of the (reversed) direction: -Y for this rail
        CHECK(pc.xStart >= 20.0 - 1e-9);
    }
}

TEST_CASE("moulding: vary stepover keeps the step along the profile", "[moulding]") {
    MouldingParams p = baseParams();
    p.profile = open({{0.0, 0.0}, {10.0, 10.0}});  // 45 degrees
    p.stockTop = 20.0;
    p.safeZ = 25.0;
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 1.0, 1.0);
    p.travel = RasterTravel::OneWay;
    const auto fixedPasses = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    CHECK(fixedPasses.size() == 11);
    p.varyStepover = true;
    const auto varied = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    CHECK(varied.size() == 16);  // ceil(10 / (1 / sqrt 2)) steps, the last one short
    // Climb order is from the high offset down, so the short remainder step comes first.
    CHECK(varied[0].y - varied[1].y < 1.0 / std::sqrt(2.0));
    for (std::size_t i = 2; i < varied.size(); ++i) {
        CHECK(std::fabs(varied[i - 1].y - varied[i].y) == Catch::Approx(1.0 / std::sqrt(2.0)).margin(1e-9));
    }
    // A flat section is unaffected.
    p.profile = open({{0.0, 0.0}, {10.0, 0.0}});
    CHECK(passesOf(mouldingToolpath(ball, p).toolpaths[0]).size() == 11);
}

TEST_CASE("moulding: skip flat regions leaves out the passes strictly inside them", "[moulding]") {
    MouldingParams p = baseParams();
    p.profile = open({{0.0, -3.0}, {3.0, 0.0}, {8.0, 0.0}, {11.0, -3.0}});
    const Rough3dTool em = makeTool(tooling::ToolType::BallNose, 1.0, 1.0);
    const auto count = [&](bool skip) {
        p.skipFlatRegions = skip;
        std::vector<double> ys;
        for (const PassRec& pr : passesOf(mouldingToolpath(em, p).toolpaths[0])) {
            ys.push_back(pr.y);
        }
        return ys;
    };
    const auto all = count(false);
    const auto skipped = count(true);
    CHECK(all.size() == 12);
    CHECK(skipped.size() == 12 - 4);  // u = 4, 5, 6, 7
    for (const double y : skipped) {
        CHECK(!(y > 3.0 + 1e-9 && y < 8.0 - 1e-9));
    }
    CHECK(std::count_if(skipped.begin(), skipped.end(), [](double y) { return std::fabs(y - 3.0) < 1e-9; }) == 1);
    CHECK(std::count_if(skipped.begin(), skipped.end(), [](double y) { return std::fabs(y - 8.0) < 1e-9; }) == 1);
    // A flat stretch narrower than the stepover is not a flat region.
    p.profile = open({{0.0, -3.0}, {3.0, 0.0}, {4.5, 0.0}, {7.5, -3.0}});
    const Rough3dTool wide = makeTool(tooling::ToolType::BallNose, 2.0, 2.0);
    const auto countWide = [&](bool skip) {
        p.skipFlatRegions = skip;
        return passesOf(mouldingToolpath(wide, p).toolpaths[0]).size();
    };
    CHECK(countWide(true) == countWide(false));  // the flat is 1.5 wide, the stepover 2 (a pass at u = 4 stays)
}

TEST_CASE("moulding: sharp corners mitre the offset of a rail, round corners do not", "[moulding]") {
    MouldingParams p = baseParams();
    Polyline sq;
    sq.closed = true;
    sq.points = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}};
    p.rails = {sq};
    p.profile = open({{-3.0, 0.0}, {-3.0 + 1e-3, 0.0}});  // a pass at -3 only is enough
    p.profile = open({{-3.0, 0.0}, {-2.9, 0.0}});
    const Rough3dTool em = makeTool(tooling::ToolType::EndMill, 0.1, 0.1);
    const auto reaches = [&](bool sharp) {
        p.sharpCorners = sharp;
        const MouldingResult r = mouldingToolpath(em, p);
        double minX = 1e9;
        for (const Move& m : r.toolpaths[0].moves) {
            if (m.kind == MoveKind::Feed && std::fabs(m.to.x + 3.0) < 1e-6 && std::fabs(m.to.y + 3.0) < 1e-6) {
                return true;
            }
            minX = std::min(minX, m.to.x);
        }
        return false;
    };
    CHECK(reaches(true));
    CHECK(!reaches(false));
}

TEST_CASE("moulding: boundary offset extends the band; automatic uses the tool radius", "[moulding]") {
    MouldingParams p = baseParams();
    p.profile = open({{0.0, -1.0}, {4.0, -1.0}});
    p.travel = RasterTravel::OneWay;
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0, 1.0);
    const auto range = [&]() {
        const auto ps = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
        double lo = 1e9;
        double hi = -1e9;
        for (const PassRec& pr : ps) {
            lo = std::min(lo, pr.y);
            hi = std::max(hi, pr.y);
            CHECK(pr.z == Catch::Approx(-1.0).margin(1e-9));  // flat continuation at the end heights
        }
        return std::pair<double, double>{lo, hi};
    };
    CHECK(range() == std::pair<double, double>{0.0, 4.0});
    p.boundaryOffset = 2.0;
    CHECK(range() == std::pair<double, double>{-2.0, 6.0});
    p.boundaryOffset = 0.0;
    p.autoBoundaryOffset = true;
    CHECK(range() == std::pair<double, double>{-1.0, 5.0});
}

TEST_CASE("moulding: finish by simulation never gouges and stays near the section", "[moulding]") {
    MouldingParams p = baseParams();
    p.profile = open(kSection);
    p.profile.points.insert(p.profile.points.begin(), {-6.0, -3.0});  // duplicate start is harmless
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0, 0.5);
    const MouldingResult r = mouldingToolpath(ball, p);
    REQUIRE(r.toolpaths.size() == 1);
    simulate::Simulator sim(simStock());
    REQUIRE(sim.ok());
    const auto shape = simulate::ToolShape::fromTool(ball.tool);
    REQUIRE(shape);
    REQUIRE(sim.addToolpath(r.toolpaths[0], *shape, 1));
    REQUIRE(sim.advanceAll());
    const simulate::Heightfield& hf = sim.heightfield();
    std::size_t checked = 0;
    for (std::size_t j = 0; j < 48; ++j) {
        const double y = -12.0 + (static_cast<double>(j) + 0.5) * 0.5;
        if (std::fabs(y) > 5.5) {
            continue;
        }
        for (std::size_t i = 6; i < 38; ++i) {  // x = 1 .. 17, clear of the rail ends
            const double s = heightAt(kSection, y);
            CHECK(hf.height(i, j) >= s - 0.01);
            ++checked;
            if (y >= 4.0) {  // on the straight slope away from the kinks
                CHECK(hf.height(i, j) <= s + ballLift(1.0, 2.0 / 3.0) + 0.03);
            }
        }
    }
    CHECK(checked > 300);
}

TEST_CASE("moulding: rough clears level by level above the section plus allowance", "[moulding]") {
    MouldingParams p = baseParams();
    p.mode = MouldingMode::Rough;
    p.profile = open(kSection);
    p.allowance = 0.3;
    p.stockTop = 0.0;
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0, 1.0, 1.5);
    const MouldingResult r = mouldingToolpath(ball, p);
    REQUIRE(r.toolpaths.size() == 1);
    // Cut Z values are level heights (0 - 1.5 n) or the exact tip (drop + allowance) of a pass.
    const auto shape = simulate::ToolShape::ballNose(2.0);
    std::set<double> tips;
    for (double u = -6.0; u <= 6.0 + 1e-9; u += 1.0) {
        std::vector<Vec2> sec(kSection);
        tips.insert(mouldingDropHeight(sec, *shape, u) + 0.3);
    }
    double minZ = 1e9;
    for (const PassRec& pr : passesOf(r.toolpaths[0])) {
        minZ = std::min(minZ, pr.z);
        const double k = pr.z / -1.5;
        const bool onLevel = std::fabs(k - std::round(k)) < 1e-9;
        const bool onTip = std::any_of(tips.begin(), tips.end(), [&](double t) { return std::fabs(t - pr.z) < 1e-8; });
        CHECK((onLevel || onTip));
    }
    CHECK(minZ == Catch::Approx(*tips.begin()).margin(1e-8));
    simulate::Simulator sim(simStock());
    REQUIRE(sim.addToolpath(r.toolpaths[0], *shape, 1));
    REQUIRE(sim.advanceAll());
    for (std::size_t j = 0; j < 48; ++j) {
        const double y = -12.0 + (static_cast<double>(j) + 0.5) * 0.5;
        if (std::fabs(y) > 5.5) {
            continue;
        }
        for (std::size_t i = 6; i < 38; ++i) {
            const double s = heightAt(kSection, y);
            CHECK(sim.heightfield().height(i, j) >= s + 0.3 - 0.01);
        }
    }
}

TEST_CASE("moulding: rough machines flat regions only on request", "[moulding]") {
    MouldingParams p = baseParams();
    p.mode = MouldingMode::Rough;
    p.stockTop = 0.0;
    p.profile = open({{0.0, -3.0}, {3.0, -6.0}, {8.0, -6.0}, {11.0, -3.0}});
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 1.0, 1.0, 3.0);
    const auto inside = [&]() {
        std::size_t n = 0;
        for (const PassRec& pr : passesOf(mouldingToolpath(ball, p).toolpaths[0])) {
            n += pr.y > 3.0 + 1e-9 && pr.y < 8.0 - 1e-9 ? 1 : 0;
        }
        return n;
    };
    p.machineFlatRegions = true;
    CHECK(inside() > 0);
    p.machineFlatRegions = false;
    CHECK(inside() == 0);
}

TEST_CASE("moulding: a clearance tool clears first and the main tool only rests where it left stock",
          "[moulding]") {
    MouldingParams p = baseParams();
    p.mode = MouldingMode::Rough;
    p.stockTop = 0.0;
    // A wide flat floor at -5 with a narrow groove down to -9.
    p.profile = open({{-12.0, -5.0}, {-2.5, -5.0}, {-2.0, -9.0}, {2.0, -9.0}, {2.5, -5.0}, {12.0, -5.0}});
    const Rough3dTool small = makeTool(tooling::ToolType::EndMill, 2.0, 1.5, 2.0);
    const auto without = mouldingToolpath(small, p);
    p.clearTool = makeTool(tooling::ToolType::EndMill, 8.0, 6.0, 2.0);
    const MouldingResult r = mouldingToolpath(small, p);
    REQUIRE(r.toolpaths.size() == 2);
    CHECK(r.toolpaths[0].name == "Moulding [Clear]");
    CHECK(r.toolpaths[0].tool.diameter == 8.0);
    CHECK(r.toolpaths[1].tool.diameter == 2.0);
    CHECK(!r.toolpaths[0].moves.empty());
    const auto mainPasses = passesOf(r.toolpaths[1]);
    CHECK(!mainPasses.empty());
    CHECK(mainPasses.size() < passesOf(without.toolpaths[0]).size());
    // The clearance tool cut the flat floor to -5: the main tool leaves that area alone.
    for (const PassRec& pr : mainPasses) {
        CHECK(std::fabs(pr.y) < 4.5);
    }
    // Together they reach the groove and never go below the section.
    simulate::Simulator sim(simStock());
    REQUIRE(sim.ok());
    const auto sc = simulate::ToolShape::flatEndMill(8.0);
    const auto sm = simulate::ToolShape::flatEndMill(2.0);
    REQUIRE(sim.addToolpath(r.toolpaths[0], *sc, 1));
    REQUIRE(sim.addToolpath(r.toolpaths[1], *sm, 2));
    REQUIRE(sim.advanceAll());
    CHECK(sim.heightfield().height(20, 23) < -8.99);  // x = 8.75, y = -0.25: groove floor is -9
    CHECK(sim.heightfield().height(20, 23) >= -9.0 - 0.01);
    CHECK(sim.heightfield().height(20, 5) == Catch::Approx(-5.0).margin(0.01));  // the flat floor, y = -9.25
}

TEST_CASE("moulding: ramp plunge enters along the pass", "[moulding]") {
    MouldingParams p = baseParams();
    p.mode = MouldingMode::Rough;
    p.stockTop = 0.0;
    p.profile = open({{-3.0, -4.0}, {3.0, -4.0}});
    const Rough3dTool em = makeTool(tooling::ToolType::EndMill, 2.0, 2.0, 2.0);
    const auto stats = [&](bool ramp) {
        p.rampPlunge = ramp;
        const MouldingResult r = mouldingToolpath(em, p);
        double deepestPlunge = 0.0;
        std::size_t descending = 0;
        for (std::size_t i = 1; i < r.toolpaths[0].moves.size(); ++i) {
            const Move& m = r.toolpaths[0].moves[i];
            const Vec3 a = r.toolpaths[0].startOf(i);
            if (m.kind == MoveKind::Plunge) {
                deepestPlunge = std::min(deepestPlunge, m.to.z);
            }
            if (m.kind == MoveKind::Feed && m.to.z < a.z - 1e-9) {
                ++descending;
            }
        }
        return std::pair<double, std::size_t>{deepestPlunge, descending};
    };
    CHECK(stats(false) == std::pair<double, std::size_t>{-4.0, 0});
    const auto ramped = stats(true);
    CHECK(ramped.first == Catch::Approx(-2.0).margin(1e-9));  // only down to the top of the last level (-2), not -4
    CHECK(ramped.second > 0);
    // And it still reaches the bottom without gouging.
    p.rampPlunge = true;
    const MouldingResult r = mouldingToolpath(em, p);
    CHECK(r.toolpaths[0].minZ().value() == Catch::Approx(-4.0).margin(1e-9));
}

TEST_CASE("moulding: invalid input is refused with a diagnostic and no moves", "[moulding]") {
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0);
    const auto refused = [&](MouldingParams p, ToolpathIssue issue, const Rough3dTool& t) {
        const MouldingResult r = mouldingToolpath(t, p);
        for (const Toolpath& tp : r.toolpaths) {
            CHECK(tp.moves.empty());
        }
        return hasIssueOf(r, issue);
    };
    CHECK(refused(baseParams(), ToolpathIssue::InvalidTool, makeTool(tooling::ToolType::BallNose, 0.0)));
    MouldingParams p = baseParams();
    p.rails.clear();
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.profile.closed = true;
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.profile = open({{0.0, 0.0}});
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.profile = open({{0.0, 0.0}, {2.0, 1.0}, {1.0, 2.0}});  // turns back
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.profile = open({{1.0, 0.0}, {1.0, 3.0}});  // no width
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.safeZ = p.stockTop;
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.allowance = -1.0;
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.zOffset = std::numeric_limits<double>::quiet_NaN();
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    p = baseParams();
    p.rails = {open({{1.0, 1.0}, {1.0, 1.0}})};
    CHECK(refused(p, ToolpathIssue::InvalidParameters, ball));
    CHECK(hasIssueOf(mouldingToolpath(ball, p), ToolpathIssue::EmptyContour));
    p = baseParams();
    p.mode = MouldingMode::Rough;
    p.clearTool = makeTool(tooling::ToolType::EndMill, 0.0);
    CHECK(refused(p, ToolpathIssue::InvalidTool, ball));
}

TEST_CASE("moulding: nothing to cut gives NoPasses; cancel and limits give empty results", "[moulding]") {
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0);
    MouldingParams p = baseParams();
    p.stockTop = -10.0;
    p.stockBottom = -30.0;
    p.safeZ = 0.0;
    p.zOffset = 0.0;
    p.profile = open({{-5.0, 4.0}, {5.0, 4.0}});  // above the top of the stock
    const MouldingResult none = mouldingToolpath(ball, p);
    CHECK(hasIssueOf(none, ToolpathIssue::NoPasses));
    CHECK(none.toolpaths[0].moves.empty());

    p = baseParams();
    std::atomic<bool> stop{true};
    ToolpathRunControl c;
    c.cancel = &stop;
    MouldingResult r = mouldingToolpath(ball, p, c);
    CHECK(hasIssueOf(r, ToolpathIssue::Cancelled));
    CHECK(r.toolpaths[0].moves.empty());
    c = {};
    c.workLimit = 100;
    r = mouldingToolpath(ball, p, c);
    CHECK(hasIssueOf(r, ToolpathIssue::WorkLimitExceeded));
    CHECK(r.toolpaths[0].moves.empty());
    c = {};
    c.moveLimit = 10;
    r = mouldingToolpath(ball, p, c);
    CHECK(hasIssueOf(r, ToolpathIssue::WorkLimitExceeded));
    CHECK(r.toolpaths[0].moves.empty());
}

TEST_CASE("moulding: output is deterministic, stays in Z bounds and keeps plunges and retracts vertical",
          "[moulding]") {
    MouldingParams p = baseParams();
    p.mode = MouldingMode::Rough;
    p.profile = open(kSection);
    p.stockTop = 0.0;
    p.stockBottom = -3.5;  // floors the deepest passes
    p.rampPlunge = true;
    Polyline sq;
    sq.closed = true;
    sq.points = {{0.0, 0.0}, {30.0, 0.0}, {30.0, 30.0}, {0.0, 30.0}};
    p.rails.push_back(sq);
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0, 1.0, 1.0);
    const MouldingResult a = mouldingToolpath(ball, p);
    const MouldingResult b = mouldingToolpath(ball, p);
    REQUIRE(a.toolpaths.size() == 1);
    CHECK(a.toolpaths[0] == b.toolpaths[0]);
    CHECK(a.diagnostics == b.diagnostics);
    const Toolpath& tp = a.toolpaths[0];
    REQUIRE(!tp.moves.empty());
    CHECK(tp.minZ().value() >= -3.5 - 1e-12);
    CHECK(tp.maxZ().value() <= 15.0);
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        const Vec3 s = tp.startOf(i, Vec3{0.0, 0.0, 15.0});
        if (m.kind == MoveKind::Plunge) {
            CHECK(m.to.z <= s.z);
            CHECK(std::fabs(m.to.x - s.x) + std::fabs(m.to.y - s.y) < 1e-12);
        }
        if (m.kind == MoveKind::Retract) {
            CHECK(m.to.z == 15.0);
            CHECK(std::fabs(m.to.x - s.x) + std::fabs(m.to.y - s.y) < 1e-12);
        }
    }
}

TEST_CASE("moulding: several rails are all machined", "[moulding]") {
    MouldingParams p = baseParams();
    p.rails.push_back(open({{0.0, 20.0}, {20.0, 20.0}}));
    p.travel = RasterTravel::OneWay;
    const Rough3dTool ball = makeTool(tooling::ToolType::BallNose, 2.0);
    const auto ps = passesOf(mouldingToolpath(ball, p).toolpaths[0]);
    CHECK(ps.size() == 22);
    CHECK(std::count_if(ps.begin(), ps.end(), [](const PassRec& pr) { return pr.y > 14.0; }) == 11);
}

TEST_CASE("moulding: a stepover wider than the tool is limited to its diameter", "[moulding]") {
    MouldingParams p = baseParams();
    p.profile = open({{0.0, -1.0}, {6.0, -1.0}});
    const MouldingResult r = mouldingToolpath(makeTool(tooling::ToolType::EndMill, 1.0, 3.0), p);
    CHECK(hasIssueOf(r, ToolpathIssue::StepoverLimited));
    CHECK(passesOf(r.toolpaths[0]).size() == 7);  // offsets 0 .. 6 at the diameter (1), not at 3
}

TEST_CASE("moulding: the clearance tool clears flat regions even when the main tool does not", "[moulding]") {
    MouldingParams p = baseParams();
    p.mode = MouldingMode::Rough;
    p.stockTop = 0.0;
    p.machineFlatRegions = false;
    p.profile = open({{-12.0, -5.0}, {-2.5, -5.0}, {-2.0, -9.0}, {2.0, -9.0}, {2.5, -5.0}, {12.0, -5.0}});
    p.clearTool = makeTool(tooling::ToolType::EndMill, 8.0, 6.0, 2.0);
    const MouldingResult r = mouldingToolpath(makeTool(tooling::ToolType::EndMill, 2.0, 1.5, 2.0), p);
    REQUIRE(r.toolpaths.size() == 2);
    std::size_t clearOnFlat = 0;
    for (const PassRec& pr : passesOf(r.toolpaths[0])) {
        clearOnFlat += std::fabs(pr.y) > 5.0 && std::fabs(pr.y) < 7.0 ? 1 : 0;  // strictly inside the flats
    }
    CHECK(clearOnFlat > 0);
    for (const PassRec& pr : passesOf(r.toolpaths[1])) {
        CHECK(std::fabs(pr.y) < 3.5);
    }
}

TEST_CASE("moulding: the main tool ignores stock the clearance tool left within the rest tolerance", "[moulding]") {
    MouldingParams p = baseParams();
    p.mode = MouldingMode::Rough;
    p.stockTop = 0.0;
    // A floor at -5 with a 0.005 deep groove that the wide clearance tool cannot reach.
    p.profile = open({{-12.0, -5.0}, {-2.5, -5.0}, {-2.0, -5.005}, {2.0, -5.005}, {2.5, -5.0}, {12.0, -5.0}});
    p.clearTool = makeTool(tooling::ToolType::EndMill, 8.0, 6.0, 5.0);
    const Rough3dTool small = makeTool(tooling::ToolType::EndMill, 2.0, 1.5, 5.0);
    p.restTolerance = 0.01;
    CHECK(passesOf(mouldingToolpath(small, p).toolpaths[1]).empty());
    p.restTolerance = 0.001;
    CHECK(!passesOf(mouldingToolpath(small, p).toolpaths[1]).empty());
}
