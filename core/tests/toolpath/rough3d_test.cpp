#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/relief.hpp"
#include "carvetto/simulate/simulator.hpp"
#include "carvetto/toolpath/rough3d.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using relief::HeightField;

namespace {

const float kNaN = std::numeric_limits<float>::quiet_NaN();

/// A smooth bumpy model with a steep pit, in [-8, -1] mm, deterministic.
HeightField makeModel(std::size_t w, std::size_t h, double cs, bool holes = false) {
    relief::Grid g{w, h, cs, {2.0, -3.0}};
    std::vector<float> d(w * h);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            const double x = static_cast<double>(i) * cs;
            const double y = static_cast<double>(j) * cs;
            double z = -4.0 + 2.5 * std::sin(x * 0.45) * std::cos(y * 0.35);
            const double r = std::hypot(x - 8.0, y - 6.0);
            if (r < 2.0) {
                z = -7.5;  // a pit with vertical walls
            }
            if (holes && i % 7 == 3 && j % 5 == 2) {
                d[j * w + i] = kNaN;
                continue;
            }
            d[j * w + i] = static_cast<float>(z);
        }
    }
    HeightField f;
    REQUIRE(HeightField::fromData(g, std::move(d), f) == relief::Status::Ok);
    return f;
}

Rough3dTool makeTool(tooling::ToolType type, double diameter, double stepover, double passDepth,
                     const std::string& name = "T") {
    Rough3dTool t;
    t.tool.id = name;
    t.tool.name = name;
    t.tool.type = type;
    t.tool.diameter = diameter;
    t.tool.defaultCuttingData.passDepth = passDepth;
    t.tool.defaultCuttingData.stepover = stepover;
    t.tool.defaultCuttingData.feedRate = 1000.0;
    t.tool.defaultCuttingData.plungeRate = 300.0;
    t.tool.defaultCuttingData.spindleSpeed = 18000.0;
    t.tool.defaultCuttingData.toolNumber = 1;
    return t;
}

Rough3dParams baseParams() {
    Rough3dParams p;
    p.stockTop = 0.0;
    p.stockBottom = -10.0;
    p.safeZ = 5.0;
    return p;
}

/// Simulates the toolpaths on the model grid (cell centres coincide with the model nodes).
simulate::Heightfield simulateAll(const HeightField& m, const Rough3dParams& p, const Rough3dResult& r,
                                  const std::vector<Rough3dTool>& tools) {
    simulate::Stock st;
    const double cs = m.cellSize();
    st.width = (static_cast<double>(m.width()) - 0.25) * cs;
    st.height = (static_cast<double>(m.height()) - 0.25) * cs;
    st.thickness = p.stockTop - p.stockBottom;
    st.cellSize = cs;
    st.origin = {m.origin().x - 0.5 * cs, m.origin().y - 0.5 * cs};
    st.topZ = p.stockTop;
    st.trackPassIds = false;
    simulate::Simulator sim(st);
    REQUIRE(sim.ok());
    for (std::size_t k = 0; k < r.toolpaths.size(); ++k) {
        auto shape = simulate::ToolShape::fromTool(tools[k].tool);
        REQUIRE(shape);
        if (!r.toolpaths[k].moves.empty()) {
            REQUIRE(sim.addToolpath(r.toolpaths[k], *shape, static_cast<std::uint32_t>(k + 1)));
        }
    }
    REQUIRE(sim.advanceAll());
    for (const auto& dg : sim.diagnostics()) {
        CHECK(dg.kind != simulate::DiagnosticKind::BelowTable);
    }
    return sim.heightfield();
}

struct Check {
    double worstGouge{0.0};   // max (model + a - sim) over cells
    double worstExcess{0.0};  // max (sim - bound) over cells
};

Check checkResult(const HeightField& m, const simulate::Heightfield& hf, const Rough3dParams& p,
                  const std::vector<float>& bound, double slack) {
    Check c;
    c.worstGouge = -1e9;
    c.worstExcess = -1e9;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            const float mv = m.at(i, j);
            if (std::isnan(mv)) {
                continue;
            }
            const double target = std::max(static_cast<double>(mv) + p.allowance, p.stockBottom);
            c.worstGouge = std::max(c.worstGouge, target - hf.height(i, j));
            c.worstExcess = std::max(c.worstExcess, static_cast<double>(hf.height(i, j)) - (bound[j * m.width() + i] + slack));
        }
    }
    return c;
}

std::vector<float> dropOf(const HeightField& m, const Rough3dTool& t, const Rough3dParams& p) {
    std::vector<float> out;
    std::uint64_t work = 0;
    auto shape = simulate::ToolShape::fromTool(t.tool);
    REQUIRE(shape);
    REQUIRE(dropToolSurface(m, *shape, p.zOffset, out, 1ULL << 40, work) == DropStatus::Ok);
    for (float& v : out) {
        v = static_cast<float>(std::max(static_cast<double>(v) + p.allowance, p.stockBottom));
    }
    return out;
}

}  // namespace

TEST_CASE("rough3d: drop surface equals the brute-force definition", "[rough3d]") {
    const struct {
        std::size_t w, h;
        double cs;
        bool holes;
    } cases[] = {{37, 29, 0.5, true}, {50, 41, 0.4, false}, {5, 4, 1.0, false}};
    const simulate::ToolShape shapes[] = {*simulate::ToolShape::flatEndMill(3.0), *simulate::ToolShape::ballNose(3.0),
                                          *simulate::ToolShape::vBit(4.0, 1.2),
                                          *simulate::ToolShape::taperedBallNose(4.0, 0.5, 1.5)};
    for (const auto& cs : cases) {
        const HeightField m = makeModel(cs.w, cs.h, cs.cs, cs.holes);
        for (const auto& sh : shapes) {
            std::vector<float> got;
            std::uint64_t work = 0;
            REQUIRE(dropToolSurface(m, sh, 0.25, got, 1ULL << 40, work) == DropStatus::Ok);
            REQUIRE(got.size() == cs.w * cs.h);
            for (std::size_t j = 0; j < cs.h; ++j) {
                for (std::size_t i = 0; i < cs.w; ++i) {
                    double best = -std::numeric_limits<double>::infinity();
                    for (std::size_t qj = 0; qj < cs.h; ++qj) {
                        for (std::size_t qi = 0; qi < cs.w; ++qi) {
                            const float v = m.at(qi, qj);
                            const double dx = (static_cast<double>(qi) - static_cast<double>(i)) * cs.cs;
                            const double dy = (static_cast<double>(qj) - static_cast<double>(j)) * cs.cs;
                            const double d = std::hypot(dx, dy);
                            if (std::isnan(v) || d > sh.radius() * (1.0 + 1e-9) + 1e-9) {
                                continue;
                            }
                            best = std::max(best, static_cast<double>(v) + 0.25 - sh.heightAt(std::min(d, sh.radius())));
                        }
                    }
                    const float g = got[j * cs.w + i];
                    if (best == -std::numeric_limits<double>::infinity()) {
                        CHECK(g == -std::numeric_limits<float>::infinity());
                    } else {
                        INFO("cell " << i << "," << j);
                        CHECK(static_cast<double>(g) >= best);
                        CHECK(static_cast<double>(g) - best < 1e-5);
                    }
                }
            }
        }
    }
}

TEST_CASE("rough3d: drop surface is a pit-wall lift for a flat tool", "[rough3d]") {
    // One tall cell in a flat floor: a flat tool of diameter 2 must clear it within radius 1.
    relief::Grid g{9, 9, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -5.0f, m) == relief::Status::Ok);
    m.set(4, 4, -1.0f);
    const auto flat = *simulate::ToolShape::flatEndMill(2.0);
    std::vector<float> out;
    std::uint64_t work = 0;
    REQUIRE(dropToolSurface(m, flat, 0.0, out, 1ULL << 30, work) == DropStatus::Ok);
    for (std::size_t j = 0; j < 9; ++j) {
        for (std::size_t i = 0; i < 9; ++i) {
            const double d = std::hypot(static_cast<double>(i) - 4.0, static_cast<double>(j) - 4.0);
            CHECK(out[j * 9 + i] == (d <= 1.0 ? -1.0f : -5.0f));
        }
    }
}

TEST_CASE("rough3d: drop surface limits and cancel", "[rough3d]") {
    const HeightField m = makeModel(60, 60, 0.5);
    const auto ball = *simulate::ToolShape::ballNose(3.0);
    std::vector<float> out;
    std::uint64_t work = 0;
    CHECK(dropToolSurface(m, ball, 0.0, out, 1000, work) == DropStatus::WorkLimit);
    std::atomic<bool> stop{true};
    work = 0;
    CHECK(dropToolSurface(m, ball, 0.0, out, 1ULL << 40, work, &stop) == DropStatus::Cancelled);
    work = 0;
    CHECK(dropToolSurface(m, *simulate::ToolShape::ballNose(2000.0), 0.0, out, 1ULL << 40, work) == DropStatus::TooLarge);
    // Block pruning: a smooth model costs far fewer evaluations than the full footprint.
    work = 0;
    REQUIRE(dropToolSurface(m, ball, 0.0, out, 1ULL << 40, work) == DropStatus::Ok);
    const double full = 60.0 * 60.0 * 3.14159 * 6.0 * 6.0;
    CHECK(static_cast<double>(work) < 0.8 * full);
    // Pinned: 267835 evaluations with the exact block bound; using only the block maximum (no
    // profile term) costs 270068.
    CHECK(work < 269000);
    CHECK(work > 250000);
}

TEST_CASE("rough3d: raster never gouges and leaves at most the drop surface", "[rough3d]") {
    const HeightField m = makeModel(40, 32, 0.5, true);
    for (auto type : {tooling::ToolType::EndMill, tooling::ToolType::BallNose}) {
        for (double allowance : {0.0, 0.4}) {
            Rough3dParams p = baseParams();
            p.strategy = Rough3dStrategy::Raster;
            p.allowance = allowance;
            const std::vector<Rough3dTool> tools{makeTool(type, 3.0, 0.5, 2.0)};
            const Rough3dResult r = rough3dToolpath(m, tools, p);
            REQUIRE(r.diagnostics.empty());
            REQUIRE(!r.toolpaths[0].moves.empty());
            const auto hf = simulateAll(m, p, r, tools);
            const auto bound = dropOf(m, tools[0], p);
            const Check c = checkResult(m, hf, p, bound, 1e-4);
            INFO("type " << static_cast<int>(type) << " allowance " << allowance);
            CHECK(c.worstGouge <= 1e-4);
            CHECK(c.worstExcess <= 0.0);
            // The allowance is really left: a mid cell of the pit floor sits at pit + allowance.
            CHECK(hf.height(12, 9) >= -7.5 + allowance - 1e-4);
        }
    }
}

TEST_CASE("rough3d: z-level never gouges and leaves at most one stepdown above the drop surface", "[rough3d]") {
    const HeightField m = makeModel(40, 32, 0.5);
    for (auto type : {tooling::ToolType::EndMill, tooling::ToolType::BallNose}) {
        Rough3dParams p = baseParams();
        p.allowance = 0.3;
        const double sd = 1.5;
        const std::vector<Rough3dTool> tools{makeTool(type, 3.0, 0.5, sd)};
        const Rough3dResult r = rough3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        const auto hf = simulateAll(m, p, r, tools);
        const auto bound = dropOf(m, tools[0], p);
        const Check c = checkResult(m, hf, p, bound, sd + 1e-4);
        CHECK(c.worstGouge <= 1e-4);
        CHECK(c.worstExcess <= 0.0);
        // Z-level moves between rows are at level heights: every cutting Z is a level or the floor.
        double top = 0.0;
        for (const Move& mv : r.toolpaths[0].moves) {
            if (mv.kind == MoveKind::Feed) {
                const double steps = (top - mv.to.z) / sd;
                const bool onLevel = std::fabs(steps - std::round(steps)) < 1e-4;
                const bool onFloor = std::fabs(mv.to.z - *r.toolpaths[0].minZ()) < 1e-4;
                CHECK((onLevel || onFloor));
            }
        }
        // The levels are really used: the stock is cut to within a stepdown of the surface somewhere.
        CHECK(*r.toolpaths[0].minZ() < -6.0);
    }
}

TEST_CASE("rough3d: tolerance merges within bound and never gouges", "[rough3d]") {
    const HeightField m = makeModel(40, 32, 0.5);
    Rough3dParams p = baseParams();
    p.strategy = Rough3dStrategy::Raster;
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 3.0, 0.5, 2.0)};
    const auto exact = rough3dToolpath(m, tools, p);
    p.tolerance = 0.2;
    const auto loose = rough3dToolpath(m, tools, p);
    CHECK(loose.toolpaths[0].moves.size() < exact.toolpaths[0].moves.size());
    const auto hf = simulateAll(m, p, loose, tools);
    const auto bound = dropOf(m, tools[0], p);
    const Check c = checkResult(m, hf, p, bound, 0.2 + 1e-4);
    CHECK(c.worstGouge <= 1e-4);
    CHECK(c.worstExcess <= 0.0);
}

TEST_CASE("rough3d: ramp entry never gouges and descends gradually", "[rough3d]") {
    const HeightField m = makeModel(40, 32, 0.5);
    for (auto strat : {Rough3dStrategy::ZLevel, Rough3dStrategy::Raster}) {
        Rough3dParams p = baseParams();
        p.strategy = strat;
        p.rampPlunge = true;
        const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 3.0, 0.5, 2.0)};
        const auto r = rough3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        const auto hf = simulateAll(m, p, r, tools);
        const auto bound = dropOf(m, tools[0], p);
        const Check c = checkResult(m, hf, p, bound, 2.0 + 1e-4);
        CHECK(c.worstGouge <= 1e-4);
        CHECK(c.worstExcess <= 0.0);
        // Plunges only go from safe Z to the start of a ramp; the ramp slope is at most the angle.
        const auto& mv = r.toolpaths[0].moves;
        Vec3 prev = mv[0].to;
        std::size_t ramped = 0;
        for (std::size_t k = 1; k < mv.size(); ++k) {
            if (mv[k].kind == MoveKind::Feed) {
                const double dxy = std::hypot(mv[k].to.x - prev.x, mv[k].to.y - prev.y);
                const double dz = prev.z - mv[k].to.z;
                if (dxy > 1e-9 && dz > 1e-9) {
                    ++ramped;
                    CHECK(dz / dxy <= std::tan(p.rampAngle) + 1e-6);
                }
            }
            prev = mv[k].to;
        }
        CHECK(ramped > 0);
    }
}

TEST_CASE("rough3d: boundary modes restrict the cut region", "[rough3d]") {
    HeightField m = makeModel(40, 32, 0.5, true);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 2.0, 0.5, 2.0)};
    Rough3dParams p = baseParams();
    p.strategy = Rough3dStrategy::Raster;
    p.boundary = Rough3dBoundary::Vectors;
    geometry::Polyline sq;
    sq.closed = true;
    sq.points = {{4.0, -1.0}, {10.0, -1.0}, {10.0, 4.0}, {4.0, 4.0}};
    p.vectors = {sq};
    const auto r = rough3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    // Every cutting move lies inside the vector.
    for (const Move& mv : r.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Feed || mv.kind == MoveKind::Plunge) {
            CHECK(mv.to.x >= 4.0 - 1e-9);
            CHECK(mv.to.x <= 10.0 + 1e-9);
            CHECK(mv.to.y >= -1.0 - 1e-9);
            CHECK(mv.to.y <= 4.0 + 1e-9);
        }
    }
    // Growing the boundary grows the region; shrinking it shrinks it.
    auto extent = [&](double off) {
        Rough3dParams q = p;
        q.boundaryOffset = off;
        const auto rr = rough3dToolpath(m, tools, q);
        REQUIRE(rr.diagnostics.empty());
        return rr.toolpaths[0].bounds();
    };
    const auto e0 = extent(0.0);
    const auto ep = extent(1.0);
    const auto en = extent(-1.0);
    CHECK(ep.min.x < e0.min.x - 0.9);
    CHECK(ep.max.y > e0.max.y + 0.9);
    CHECK(en.min.x > e0.min.x + 0.9);
    CHECK(en.max.y < e0.max.y - 0.9);
    // No vector -> NoRegion.
    Rough3dParams q = p;
    q.vectors.clear();
    CHECK(hasIssue(rough3dToolpath(m, tools, q).diagnostics, ToolpathIssue::NoRegion));

    // Model boundary: only opaque cells. Blank out the left half.
    relief::Grid g = m.grid();
    std::vector<float> d = m.data();
    for (std::size_t j = 0; j < g.height; ++j) {
        for (std::size_t i = 0; i < 20; ++i) {
            d[j * g.width + i] = kNaN;
        }
    }
    HeightField half;
    REQUIRE(HeightField::fromData(g, d, half) == relief::Status::Ok);
    Rough3dParams pm = baseParams();
    pm.strategy = Rough3dStrategy::Raster;
    pm.boundary = Rough3dBoundary::Model;
    const auto rm = rough3dToolpath(half, tools, pm);
    REQUIRE(rm.diagnostics.empty());
    const double edge = g.origin.x + 20.0 * g.cellSize;
    for (const Move& mv : rm.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Feed || mv.kind == MoveKind::Plunge) {
            CHECK(mv.to.x >= edge - 1e-9);
        }
    }
    pm.boundaryOffset = 1.0;
    const auto rg = rough3dToolpath(half, tools, pm);
    CHECK(rg.toolpaths[0].bounds().min.x < edge - 0.9);
    CHECK(rg.toolpaths[0].bounds().min.x >= edge - 1.0 - 1e-9);
    pm.boundaryOffset = -1.0;
    const auto rs = rough3dToolpath(half, tools, pm);
    CHECK(rs.toolpaths[0].bounds().min.x > edge + 0.9);
}

TEST_CASE("rough3d: row order, axis, travel and direction", "[rough3d]") {
    relief::Grid g{9, 7, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 5.0)};
    auto rowsOf = [&](Rough3dParams p) {
        p.strategy = Rough3dStrategy::Raster;
        const auto r = rough3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        // Plunge positions in order = start of every chain.
        std::vector<Vec3> starts;
        for (const Move& mv : r.toolpaths[0].moves) {
            if (mv.kind == MoveKind::Plunge) {
                starts.push_back(mv.to);
            }
        }
        return starts;
    };
    Rough3dParams p = baseParams();
    p.travel = RasterTravel::OneWay;
    auto s = rowsOf(p);
    REQUIRE(s.size() == 7);
    CHECK(s[0].y == 6.0);  // Climb, along X: rows step towards -Y
    CHECK(s[1].y == 5.0);
    CHECK(s[0].x == 0.0);
    CHECK(s[1].x == 0.0);  // one way: every row starts at the same side
    p.direction = CutDirection::Conventional;
    s = rowsOf(p);
    CHECK(s[0].y == 0.0);
    p.reverseSteps = true;
    s = rowsOf(p);
    CHECK(s[0].y == 6.0);
    p.reverseSteps = false;
    p.direction = CutDirection::Climb;
    p.axis = RasterAxis::AlongY;
    s = rowsOf(p);
    REQUIRE(s.size() == 9);
    CHECK(s[0].x == 0.0);  // climb along Y: rows step towards +X
    CHECK(s[1].x == 1.0);
    CHECK(s[0].y == 0.0);
    // Zig-zag: one chain, alternate rows reverse, linked at the ends.
    p = baseParams();
    p.axis = RasterAxis::AlongX;
    s = rowsOf(p);
    CHECK(s.size() == 1);
    // Stepover of two cells skips rows but keeps the last.
    Rough3dParams q = baseParams();
    q.travel = RasterTravel::OneWay;
    const std::vector<Rough3dTool> wide{makeTool(tooling::ToolType::EndMill, 1.0, 2.5, 5.0)};
    q.strategy = Rough3dStrategy::Raster;
    const auto r = rough3dToolpath(m, wide, q);
    std::vector<double> ys;
    for (const Move& mv : r.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Plunge) {
            ys.push_back(mv.to.y);
        }
    }
    CHECK(ys == std::vector<double>{6.0, 4.0, 2.0, 0.0});
}

TEST_CASE("rough3d: zig-zag raster links rows and is one continuous cut", "[rough3d]") {
    relief::Grid g{6, 5, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 5.0)};
    Rough3dParams p = baseParams();
    p.strategy = Rough3dStrategy::Raster;
    const auto r = rough3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    const Toolpath& tp = r.toolpaths[0];
    CHECK(tp.count(MoveKind::Plunge) == 1);
    CHECK(tp.count(MoveKind::Rapid) == 1);
    CHECK(tp.count(MoveKind::Retract) == 1);
    // First row starts at the top (climb), the second reverses.
    CHECK(tp.moves[0].to.z == p.safeZ);
    CHECK(tp.moves[1].to == Vec3{0.0, 4.0, -3.0});
    CHECK(tp.moves[2].to == Vec3{5.0, 4.0, -3.0});
    CHECK(tp.moves[3].to == Vec3{5.0, 3.0, -3.0});
    CHECK(tp.moves[4].to == Vec3{0.0, 3.0, -3.0});
    CHECK(tp.length() > 29.0);
}

TEST_CASE("rough3d: drop tool versus project", "[rough3d]") {
    const HeightField m = makeModel(40, 32, 0.5);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 3.0, 0.5, 2.0)};
    Rough3dParams p = baseParams();
    p.strategy = Rough3dStrategy::Raster;
    p.tip = Rough3dTipMode::Project;
    const auto proj = rough3dToolpath(m, tools, p);
    REQUIRE(proj.diagnostics.empty());
    // Projected: every node's tip height is the model height; gouges are expected, so just compare.
    p.tip = Rough3dTipMode::DropTool;
    const auto drop = rough3dToolpath(m, tools, p);
    auto meanZ = [](const Toolpath& t) {
        double sum = 0.0;
        std::size_t n = 0;
        for (const Move& mv : t.moves) {
            if (mv.kind == MoveKind::Feed) {
                sum += mv.to.z;
                ++n;
            }
        }
        return sum / static_cast<double>(n);
    };
    CHECK(meanZ(proj.toolpaths[0]) < meanZ(drop.toolpaths[0]) - 0.3);
    p.tip = Rough3dTipMode::Project;
    const auto hf = simulateAll(m, p, proj, tools);
    const auto bound = dropOf(m, tools[0], p);
    const Check c = checkResult(m, hf, p, bound, 1e-4);
    CHECK(c.worstGouge > 0.1);  // projecting a ball tip onto a steep model does gouge
}

TEST_CASE("rough3d: multi-tool rest machining cuts only what the first tool left", "[rough3d]") {
    // A pit 2 mm wide: the 6 mm flat cannot enter it, the 1 mm flat can.
    relief::Grid g{40, 30, 0.5, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    for (std::size_t j = 12; j < 16; ++j) {
        for (std::size_t i = 18; i < 22; ++i) {
            m.set(i, j, -8.0f);
        }
    }
    Rough3dParams p = baseParams();
    p.strategy = Rough3dStrategy::Raster;
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 6.0, 1.0, 5.0, "big"),
                                         makeTool(tooling::ToolType::EndMill, 1.0, 0.5, 5.0, "small")};
    const auto r = rough3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    REQUIRE(r.toolpaths.size() == 2);
    CHECK(r.toolpaths[0].name == "3D Rough [big]");
    CHECK(r.toolpaths[1].name == "3D Rough [small]");
    // The first tool stays above -3 (it cannot enter the pit); the rest tool goes down to the pit
    // floor and only near it.
    CHECK(*r.toolpaths[0].minZ() > -3.0 - 1e-6);
    CHECK(*r.toolpaths[1].minZ() <= -8.0 + 1e-6);
    const Box2 b = r.toolpaths[1].bounds();
    CHECK(b.min.x >= 8.5 - 1e-6);
    CHECK(b.max.x <= 11.5 + 1e-6);
    CHECK(b.min.y >= 5.5 - 1e-6);
    CHECK(b.max.y <= 8.5 + 1e-6);
    const auto hf = simulateAll(m, p, r, tools);
    const Check c = checkResult(m, hf, p, std::vector<float>(40 * 30, 0.0f), 0.0);
    CHECK(c.worstGouge <= 1e-4);
    // The pit is cleaned to its floor.
    CHECK(hf.height(20, 14) <= -8.0f + 1e-4);
    // A rest tool as big as the first finds nothing.
    const std::vector<Rough3dTool> same{makeTool(tooling::ToolType::EndMill, 6.0, 1.0, 5.0, "a"),
                                        makeTool(tooling::ToolType::EndMill, 6.0, 1.0, 5.0, "b")};
    const auto r2 = rough3dToolpath(m, same, p);
    CHECK(hasIssue(r2.diagnostics, ToolpathIssue::NoRestMaterial));
    CHECK(r2.toolpaths[1].moves.empty());
}

TEST_CASE("rough3d: deterministic and cancellable", "[rough3d]") {
    const HeightField m = makeModel(40, 32, 0.5, true);
    Rough3dParams p = baseParams();
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 3.0, 0.5, 2.0)};
    const auto a = rough3dToolpath(m, tools, p);
    const auto b = rough3dToolpath(m, tools, p);
    CHECK(a.toolpaths[0] == b.toolpaths[0]);
    std::atomic<bool> stop{true};
    ToolpathRunControl rc;
    rc.cancel = &stop;
    const auto c = rough3dToolpath(m, tools, p, rc);
    CHECK(hasIssue(c.diagnostics, ToolpathIssue::Cancelled));
    CHECK(c.toolpaths[0].moves.empty());
    rc.cancel = nullptr;
    rc.workLimit = 100;
    CHECK(hasIssue(rough3dToolpath(m, tools, p, rc).diagnostics, ToolpathIssue::WorkLimitExceeded));
    rc.workLimit = 0;
    rc.moveLimit = 50;
    const auto d = rough3dToolpath(m, tools, p, rc);
    CHECK(hasIssue(d.diagnostics, ToolpathIssue::WorkLimitExceeded));
    CHECK(d.toolpaths[0].moves.empty());
}

TEST_CASE("rough3d: invalid input is refused", "[rough3d]") {
    const HeightField m = makeModel(10, 10, 0.5);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 2.0, 0.5, 1.0)};
    Rough3dParams p = baseParams();
    CHECK(hasIssue(rough3dToolpath(m, {}, p).diagnostics, ToolpathIssue::NoTool));
    CHECK(hasIssue(rough3dToolpath(HeightField{}, tools, p).diagnostics, ToolpathIssue::InvalidParameters));
    auto refused = [&](Rough3dParams q) { return hasIssue(rough3dToolpath(m, tools, q).diagnostics, ToolpathIssue::InvalidParameters); };
    Rough3dParams q = p;
    q.safeZ = 0.0;
    CHECK(refused(q));
    q = p;
    q.stockBottom = 1.0;
    CHECK(refused(q));
    q = p;
    q.allowance = -1.0;
    CHECK(refused(q));
    q = p;
    q.zOffset = std::numeric_limits<double>::quiet_NaN();
    CHECK(refused(q));
    q = p;
    q.rampAngle = 0.0;
    CHECK(refused(q));
    auto badTool = tools;
    badTool[0].tool.defaultCuttingData.stepover = 0.0;
    CHECK(hasIssue(rough3dToolpath(m, badTool, p).diagnostics, ToolpathIssue::InvalidTool));
    badTool = tools;
    badTool[0].tool.type = tooling::ToolType::RadiusedEndMill;
    CHECK(hasIssue(rough3dToolpath(m, badTool, p).diagnostics, ToolpathIssue::InvalidTool));
    // A model entirely below... nothing above floor: the model is the stock top.
    relief::Grid g{4, 4, 1.0, {0, 0}};
    HeightField hi;
    REQUIRE(HeightField::create(g, 3.0f, hi) == relief::Status::Ok);
    CHECK(hasIssue(rough3dToolpath(hi, tools, p).diagnostics, ToolpathIssue::NoRegion));
}
