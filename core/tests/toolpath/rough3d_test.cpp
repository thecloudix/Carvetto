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

// ---- T-0304: offset clearing, profile first/last, depth-first ordering, avoid machined areas ----

namespace {

std::vector<Vec3> chainStarts(const Toolpath& tp) {
    std::vector<Vec3> s;
    for (const Move& mv : tp.moves) {
        if (mv.kind == MoveKind::Plunge) {
            s.push_back(mv.to);
        }
    }
    return s;
}

std::vector<Vec3> feedsOf(const Toolpath& tp) {
    std::vector<Vec3> s;
    for (const Move& mv : tp.moves) {
        if (mv.kind == MoveKind::Feed) {
            s.push_back(mv.to);
        }
    }
    return s;
}

double sumHeights(const simulate::Heightfield& hf, std::size_t w, std::size_t h) {
    double s = 0.0;
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            s += static_cast<double>(hf.height(i, j));
        }
    }
    return s;
}

}  // namespace

TEST_CASE("rough3d: offset clearing never gouges in any clearing/profile/order combination", "[rough3d][t0304]") {
    const HeightField m = makeModel(40, 32, 0.5, true);
    for (auto type : {tooling::ToolType::EndMill, tooling::ToolType::BallNose}) {
        for (auto prof : {Rough3dProfile::None, Rough3dProfile::First, Rough3dProfile::Last}) {
            for (auto order : {Rough3dOrder::LevelByLevel, Rough3dOrder::DepthFirst}) {
                for (auto clearing : {Rough3dClearing::Raster, Rough3dClearing::Offset}) {
                    for (bool avoid : {false, true}) {
                        Rough3dParams p = baseParams();
                        p.allowance = 0.3;
                        p.profile = prof;
                        p.order = order;
                        p.clearing = clearing;
                        p.avoidMachined = avoid;
                        p.rampPlunge = (prof == Rough3dProfile::Last);
                        const double sd = 1.5;
                        const std::vector<Rough3dTool> tools{makeTool(type, 3.0, 0.5, sd)};
                        const Rough3dResult r = rough3dToolpath(m, tools, p);
                        INFO("type " << static_cast<int>(type) << " prof " << static_cast<int>(prof) << " order "
                                     << static_cast<int>(order) << " clearing " << static_cast<int>(clearing)
                                     << " avoid " << avoid);
                        REQUIRE(r.diagnostics.empty());
                        REQUIRE(!r.toolpaths[0].moves.empty());
                        const auto hf = simulateAll(m, p, r, tools);
                        const auto bound = dropOf(m, tools[0], p);
                        const Check c = checkResult(m, hf, p, bound, sd + 1e-4);
                        CHECK(c.worstGouge <= 1e-4);
                        CHECK(c.worstExcess <= 0.0);
                    }
                }
            }
        }
    }
}

TEST_CASE("rough3d: offset clearing leaves no more stock than raster clearing", "[rough3d][t0304]") {
    const HeightField m = makeModel(40, 32, 0.5);
    for (double stepover : {0.5, 1.5}) {
        Rough3dParams p = baseParams();
        p.allowance = 0.2;
        const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 3.0, stepover, 1.5)};
        Rough3dParams pr = p;
        pr.clearing = Rough3dClearing::Raster;
        Rough3dParams po = p;
        po.clearing = Rough3dClearing::Offset;
        const auto rr = rough3dToolpath(m, tools, pr);
        const auto ro = rough3dToolpath(m, tools, po);
        REQUIRE(rr.diagnostics.empty());
        REQUIRE(ro.diagnostics.empty());
        const auto hr = simulateAll(m, pr, rr, tools);
        const auto ho = simulateAll(m, po, ro, tools);
        const double sr = sumHeights(hr, 40, 32);
        const double so = sumHeights(ho, 40, 32);
        INFO("stepover " << stepover << " raster sum " << sr << " offset sum " << so);
        CHECK(so <= sr + 1e-3);
        // Cell by cell no more than a stepdown above raster would leave.
        for (std::size_t j = 0; j < 32; ++j) {
            for (std::size_t i = 0; i < 40; ++i) {
                CHECK(static_cast<double>(ho.height(i, j)) <= static_cast<double>(hr.height(i, j)) + 1.5 + 1e-4);
            }
        }
    }
}

TEST_CASE("rough3d: offset rings run wall-following inward, climb counter-clockwise", "[rough3d][t0304]") {
    relief::Grid g{9, 7, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 5.0)};
    Rough3dParams p = baseParams();
    p.clearing = Rough3dClearing::Offset;
    auto xy = [](const Vec3& v) { return std::vector<double>{v.x, v.y}; };
    {
        const auto r = rough3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        const auto st = chainStarts(r.toolpaths[0]);
        REQUIRE(st.size() == 1);  // the rings are linked into one continuous cut
        CHECK(xy(st[0]) == std::vector<double>{0.0, 0.0});
        CHECK(st[0].z == -3.0);
        const auto f = feedsOf(r.toolpaths[0]);
        REQUIRE(f.size() > 8);
        // Outer ring: bottom edge east, right edge north, top edge west, left edge south ...
        CHECK(xy(f[0]) == std::vector<double>{8.0, 0.0});
        CHECK(xy(f[1]) == std::vector<double>{8.0, 6.0});
        CHECK(xy(f[2]) == std::vector<double>{0.0, 6.0});
        CHECK(xy(f[3]) == std::vector<double>{0.0, 1.0});
        // ... then the next ring one cell in, same sense.
        // (collinear feeds are merged, so the link (0,1) -> (1,1) and the ring's first edge are one move)
        CHECK(xy(f[4]) == std::vector<double>{7.0, 1.0});
        CHECK(xy(f[5]) == std::vector<double>{7.0, 5.0});
        CHECK(xy(f[6]) == std::vector<double>{1.0, 5.0});
        CHECK(xy(f[7]) == std::vector<double>{1.0, 2.0});
        CHECK(xy(f[8]) == std::vector<double>{6.0, 2.0});
    }
    {
        Rough3dParams q = p;
        q.direction = CutDirection::Conventional;
        const auto r = rough3dToolpath(m, tools, q);
        const auto f = feedsOf(r.toolpaths[0]);
        REQUIRE(f.size() > 4);
        CHECK(xy(f[0]) == std::vector<double>{0.0, 6.0});  // clockwise: left edge first
        CHECK(xy(f[1]) == std::vector<double>{8.0, 6.0});
        CHECK(xy(f[2]) == std::vector<double>{8.0, 0.0});
        CHECK(xy(f[3]) == std::vector<double>{1.0, 0.0});
    }
    {
        // Profile Last cuts the outer ring after the inner ones: the cut starts one ring in and ends on the boundary.
        Rough3dParams q = p;
        q.profile = Rough3dProfile::Last;
        const auto r = rough3dToolpath(m, tools, q);
        const auto st = chainStarts(r.toolpaths[0]);
        REQUIRE(!st.empty());
        CHECK(xy(st[0]) == std::vector<double>{1.0, 1.0});
        const auto f = feedsOf(r.toolpaths[0]);
        const Vec3 last = f.back();
        CHECK((last.x == 0.0 || last.x == 8.0 || last.y == 0.0 || last.y == 6.0));
        for (const Vec3& v : f) {
            CHECK(v.z == -3.0);
        }
    }
    {
        // Profile First is the default order of offset clearing.
        Rough3dParams q = p;
        q.profile = Rough3dProfile::First;
        CHECK(rough3dToolpath(m, tools, q).toolpaths[0] == rough3dToolpath(m, tools, p).toolpaths[0]);
    }
}

TEST_CASE("rough3d: offset clearing circles an island and covers every node", "[rough3d][t0304]") {
    // A pillar in the middle: the ring around it is cut with the pillar on the right (climb).
    relief::Grid g{11, 11, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    for (std::size_t j = 4; j <= 6; ++j) {
        for (std::size_t i = 4; i <= 6; ++i) {
            m.set(i, j, 0.0f);
        }
    }
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 5.0)};
    Rough3dParams p = baseParams();
    p.clearing = Rough3dClearing::Offset;
    const auto r = rough3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    const auto hf = simulateAll(m, p, r, tools);
    const auto bound = dropOf(m, tools[0], p);
    const Check c = checkResult(m, hf, p, bound, 1e-4);
    CHECK(c.worstGouge <= 1e-4);
    CHECK(c.worstExcess <= 0.0);
    // The pillar (T = 0 on it) is untouched; its surroundings are cut to -3.
    CHECK(hf.height(5, 5) > -1e-4);
    CHECK(hf.height(3, 5) < -2.99);
    // Around the pillar the cut runs clockwise (pillar on the right): the island ring starts at its lowest
    // node (3,3), goes north along x = 3, then east along y = 7, south along x = 7 and back west along y = 3.
    const auto& mv = r.toolpaths[0].moves;
    std::size_t at = mv.size();
    for (std::size_t k = 0; k < mv.size(); ++k) {
        if (mv[k].kind == MoveKind::Plunge && mv[k].to.x == 3.0 && mv[k].to.y == 3.0) {
            at = k;
            break;
        }
    }
    REQUIRE(at + 4 < mv.size());
    CHECK(mv[at + 1].to.x == 3.0);
    CHECK(mv[at + 1].to.y == 7.0);
    CHECK(mv[at + 2].to.x == 7.0);
    CHECK(mv[at + 2].to.y == 7.0);
    CHECK(mv[at + 3].to.x == 7.0);
    CHECK(mv[at + 3].to.y == 3.0);
    CHECK(mv[at + 4].to.x == 4.0);
    CHECK(mv[at + 4].to.y == 3.0);
}

TEST_CASE("rough3d: raster clearing with a profile pass adds a boundary ring first or last", "[rough3d][t0304]") {
    relief::Grid g{9, 7, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 5.0)};
    Rough3dParams p = baseParams();
    const auto base = chainStarts(rough3dToolpath(m, tools, p).toolpaths[0]);
    REQUIRE(base.size() == 1);
    CHECK(base[0].y == 6.0);  // raster climb along X starts on the top row
    p.profile = Rough3dProfile::First;
    const auto first = chainStarts(rough3dToolpath(m, tools, p).toolpaths[0]);
    REQUIRE(first.size() == 2);
    CHECK(first[0].x == 0.0);
    CHECK(first[0].y == 0.0);  // the ring starts at the lowest node
    CHECK(first[1].y == 6.0);
    p.profile = Rough3dProfile::Last;
    const auto last = chainStarts(rough3dToolpath(m, tools, p).toolpaths[0]);
    REQUIRE(last.size() == 2);
    CHECK(last[0].y == 6.0);
    CHECK(last[1].y == 0.0);
    CHECK(last[1].x == 0.0);
}

TEST_CASE("rough3d: depth-first cuts each pocket to the bottom before the next", "[rough3d][t0304]") {
    // Two pockets (-9) separated by a wall column at the stock top.
    relief::Grid g{21, 9, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -9.0f, m) == relief::Status::Ok);
    for (std::size_t j = 0; j < 9; ++j) {
        m.set(10, j, 0.0f);
    }
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 3.0)};
    for (auto clearing : {Rough3dClearing::Raster, Rough3dClearing::Offset}) {
        Rough3dParams p = baseParams();
        p.clearing = clearing;
        const auto lbl = rough3dToolpath(m, tools, p);
        REQUIRE(lbl.diagnostics.empty());
        const auto a = chainStarts(lbl.toolpaths[0]);
        // Level by level: the depths never go up.
        for (std::size_t k = 1; k < a.size(); ++k) {
            CHECK(a[k].z <= a[k - 1].z);
        }
        p.order = Rough3dOrder::DepthFirst;
        const auto dfo = rough3dToolpath(m, tools, p);
        REQUIRE(dfo.diagnostics.empty());
        const auto b = chainStarts(dfo.toolpaths[0]);
        // Depth-first: every chain of the first pocket (x < 10) comes before any chain of the second.
        bool second = false;
        bool wentUp = false;
        double firstPocketMin = 0.0;
        for (std::size_t k = 0; k < b.size(); ++k) {
            if (b[k].x > 10.0) {
                second = true;
            } else {
                CHECK(!second);
                firstPocketMin = std::min(firstPocketMin, b[k].z);
            }
            if (k > 0 && b[k].z > b[k - 1].z) {
                wentUp = true;
            }
        }
        CHECK(second);
        CHECK(wentUp);
        CHECK(firstPocketMin == -9.0);
        // The first chain is at the first level, in the first pocket (lowest node index).
        CHECK(b[0].z == -3.0);
        CHECK(b[0].x < 10.0);
        // Same result in the simulator, nothing gouged.
        const auto h1 = simulateAll(m, p, lbl, tools);
        const auto h2 = simulateAll(m, p, dfo, tools);
        const auto bound = dropOf(m, tools[0], p);
        CHECK(checkResult(m, h2, p, bound, 1e-4).worstGouge <= 1e-4);
        CHECK(checkResult(m, h2, p, bound, 1e-4).worstExcess <= 0.0);
        for (std::size_t j = 0; j < 9; ++j) {
            for (std::size_t i = 0; i < 21; ++i) {
                CHECK(h1.height(i, j) == h2.height(i, j));
            }
        }
    }
}

TEST_CASE("rough3d: depth-first on nested pockets visits the deep part inside its parent first", "[rough3d][t0304]") {
    // Pocket A is shallow (-4) with a deeper pit (-9) inside; pocket B is at -9 on the other side of a wall.
    relief::Grid g{23, 9, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -4.0f, m) == relief::Status::Ok);
    for (std::size_t j = 0; j < 9; ++j) {
        m.set(10, j, 0.0f);
        for (std::size_t i = 12; i < 23; ++i) {
            m.set(i, j, -9.0f);
        }
    }
    for (std::size_t j = 3; j <= 5; ++j) {
        for (std::size_t i = 3; i <= 5; ++i) {
            m.set(i, j, -9.0f);
        }
    }
    m.set(11, 4, 0.0f);
    for (std::size_t j = 0; j < 9; ++j) {
        m.set(11, j, 0.0f);
    }
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 3.0)};
    Rough3dParams p = baseParams();
    p.order = Rough3dOrder::DepthFirst;
    const auto r = rough3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    const auto st = chainStarts(r.toolpaths[0]);
    // Pocket A first (all chains with x < 10), then B.
    bool second = false;
    for (const Vec3& v : st) {
        if (v.x > 11.0) {
            second = true;
        } else {
            CHECK(!second);
        }
    }
    CHECK(second);
    const auto hf = simulateAll(m, p, r, tools);
    const auto bound = dropOf(m, tools[0], p);
    const Check c = checkResult(m, hf, p, bound, 3.0 + 1e-4);
    CHECK(c.worstGouge <= 1e-4);
    CHECK(c.worstExcess <= 0.0);
    CHECK(hf.height(4, 4) < -8.99);
    CHECK(hf.height(16, 4) < -8.99);
}

TEST_CASE("rough3d: avoid machined areas retracts instead of linking across them", "[rough3d][t0304]") {
    // Three pillars in a row with one-cell gaps: the 1.5 mm-radius first tool cannot reach the two gap cells;
    // the rest tool cuts them, linking across the already machined nodes between them unless avoided.
    relief::Grid g{15, 7, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    m.set(4, 3, 0.0f);
    m.set(6, 3, 0.0f);
    m.set(8, 3, 0.0f);
    std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 3.0, 1.0, 5.0, "big"),
                                   makeTool(tooling::ToolType::EndMill, 0.5, 1.0, 5.0, "small")};
    Rough3dParams p = baseParams();
    p.clearing = Rough3dClearing::Offset;
    const auto a = rough3dToolpath(m, tools, p);
    std::string msgs;
    for (const auto& d : a.diagnostics) {
        msgs += d.message + "; ";
    }
    INFO(msgs);
    REQUIRE(a.diagnostics.empty());
    REQUIRE(a.toolpaths.size() == 2);
    const auto sa = chainStarts(a.toolpaths[1]);
    p.avoidMachined = true;
    const auto b = rough3dToolpath(m, tools, p);
    REQUIRE(b.diagnostics.empty());
    const auto sb = chainStarts(b.toolpaths[1]);
    CHECK(sa.size() == 1);
    CHECK(sb.size() == 2);
    // Both cut the same nodes: identical simulated result.
    const auto ha = simulateAll(m, p, a, tools);
    const auto hb = simulateAll(m, p, b, tools);
    for (std::size_t j = 0; j < 7; ++j) {
        for (std::size_t i = 0; i < 15; ++i) {
            CHECK(ha.height(i, j) == hb.height(i, j));
        }
    }
    CHECK(ha.height(5, 3) < -2.99);
    CHECK(ha.height(7, 3) < -2.99);
    CHECK(ha.height(6, 3) > -1e-4);
}

TEST_CASE("rough3d: transparent cells are ignored and cut to the floor under the material boundary", "[rough3d][t0304]") {
    // Left half opaque at -3, right half transparent: far from the opaque cells the tool goes to the floor.
    relief::Grid g{20, 6, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    for (std::size_t j = 0; j < 6; ++j) {
        for (std::size_t i = 10; i < 20; ++i) {
            m.set(i, j, kNaN);
        }
    }
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0, 1.0)};
    for (auto clearing : {Rough3dClearing::Raster, Rough3dClearing::Offset}) {
        Rough3dParams p = baseParams();
        p.clearing = clearing;
        const auto r = rough3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        const auto hf = simulateAll(m, p, r, tools);
        CHECK(hf.height(15, 3) < -9.99);  // floor: stockBottom = -10
        CHECK(hf.height(2, 3) < -2.99);
        CHECK(hf.height(2, 3) > -3.0 - 1e-4);  // the opaque side is not gouged
    }
}

TEST_CASE("rough3d: new z-level modes are deterministic, cancellable and bounded", "[rough3d][t0304]") {
    const HeightField m = makeModel(40, 32, 0.5, true);
    Rough3dParams p = baseParams();
    p.clearing = Rough3dClearing::Offset;
    p.order = Rough3dOrder::DepthFirst;
    p.profile = Rough3dProfile::Last;
    p.avoidMachined = true;
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 3.0, 0.5, 2.0)};
    const auto a = rough3dToolpath(m, tools, p);
    const auto b = rough3dToolpath(m, tools, p);
    REQUIRE(!a.toolpaths[0].moves.empty());
    CHECK(a.toolpaths[0] == b.toolpaths[0]);
    std::atomic<bool> stop{true};
    ToolpathRunControl rc;
    rc.cancel = &stop;
    const auto c = rough3dToolpath(m, tools, p, rc);
    CHECK(hasIssue(c.diagnostics, ToolpathIssue::Cancelled));
    CHECK(c.toolpaths[0].moves.empty());
    rc.cancel = nullptr;
    rc.moveLimit = 50;
    const auto d = rough3dToolpath(m, tools, p, rc);
    CHECK(hasIssue(d.diagnostics, ToolpathIssue::WorkLimitExceeded));
    CHECK(d.toolpaths[0].moves.empty());
    // Cancel arriving mid-run (polled per unit/ring).
    p.order = Rough3dOrder::LevelByLevel;
    rc.moveLimit = 0;
    rc.workLimit = 2000;
    CHECK(hasIssue(rough3dToolpath(m, tools, p, rc).diagnostics, ToolpathIssue::WorkLimitExceeded));
}

TEST_CASE("rough3d: offset stepover skips rings but always cuts the innermost one", "[rough3d][t0304]") {
    // 9x7 flat grid: Chebyshev rings 0..3. A two-cell stepover cuts rings 0 and 2 and the innermost ring 3,
    // like raster rows skip rows but keep the last; ring 1 is left alone (the 1 mm tool does not reach it).
    relief::Grid g{9, 7, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 2.0, 5.0)};
    Rough3dParams p = baseParams();
    p.clearing = Rough3dClearing::Offset;
    const auto r = rough3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    const auto hf = simulateAll(m, p, r, tools);
    CHECK(hf.height(0, 0) < -2.99);   // ring 0
    CHECK(hf.height(4, 0) < -2.99);   // ring 0
    CHECK(hf.height(4, 1) > -1e-4);   // ring 1: skipped (only the short link between rings crosses it)
    CHECK(hf.height(7, 4) > -1e-4);
    CHECK(hf.height(2, 2) < -2.99);   // ring 2
    CHECK(hf.height(4, 3) < -2.99);   // ring 3, the innermost (not on the stepover)
    CHECK(hf.height(3, 3) < -2.99);
}
