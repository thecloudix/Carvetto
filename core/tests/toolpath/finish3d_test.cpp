#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/relief.hpp"
#include "carvetto/simulate/simulator.hpp"
#include "carvetto/toolpath/finish3d.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using relief::HeightField;

namespace {

const float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kDomeR = 12.0;

/// A spherical dome (radius 12, apex -1) on a w x h grid centred on the dome axis.
HeightField makeDome(std::size_t n, double cs) {
    relief::Grid g{n, n, cs, {0.0, 0.0}};
    std::vector<float> d(n * n);
    const double c = 0.5 * static_cast<double>(n - 1) * cs;
    for (std::size_t j = 0; j < n; ++j) {
        for (std::size_t i = 0; i < n; ++i) {
            const double x = static_cast<double>(i) * cs - c;
            const double y = static_cast<double>(j) * cs - c;
            d[j * n + i] = static_cast<float>(-1.0 - kDomeR + std::sqrt(kDomeR * kDomeR - x * x - y * y));
        }
    }
    HeightField f;
    REQUIRE(HeightField::fromData(g, std::move(d), f) == relief::Status::Ok);
    return f;
}

/// Bumpy model with a steep pit and optional transparent cells, in [-8, -1] mm.
HeightField makeModel(std::size_t w, std::size_t h, double cs, bool holes = false) {
    relief::Grid g{w, h, cs, {2.0, -3.0}};
    std::vector<float> d(w * h);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            const double x = static_cast<double>(i) * cs;
            const double y = static_cast<double>(j) * cs;
            double z = -4.0 + 2.5 * std::sin(x * 0.45) * std::cos(y * 0.35);
            if (std::hypot(x - 8.0, y - 6.0) < 2.0) {
                z = -7.5;
            }
            d[j * w + i] = (holes && i % 7 == 3 && j % 5 == 2) ? kNaN : static_cast<float>(z);
        }
    }
    HeightField f;
    REQUIRE(HeightField::fromData(g, std::move(d), f) == relief::Status::Ok);
    return f;
}

Rough3dTool makeTool(tooling::ToolType type, double diameter, double stepover, const std::string& name = "T") {
    Rough3dTool t;
    t.tool.id = name;
    t.tool.name = name;
    t.tool.type = type;
    t.tool.diameter = diameter;
    t.tool.defaultCuttingData.passDepth = 2.0;
    t.tool.defaultCuttingData.stepover = stepover;
    t.tool.defaultCuttingData.feedRate = 1000.0;
    t.tool.defaultCuttingData.plungeRate = 300.0;
    t.tool.defaultCuttingData.spindleSpeed = 18000.0;
    t.tool.defaultCuttingData.toolNumber = 1;
    return t;
}

Finish3dParams baseParams() {
    Finish3dParams p;
    p.stockTop = 0.0;
    p.stockBottom = -10.0;
    p.safeZ = 5.0;
    return p;
}

simulate::Heightfield simulateOn(const HeightField& m, double stockTop, double stockBottom,
                                 const std::vector<std::pair<const Toolpath*, const Rough3dTool*>>& paths) {
    simulate::Stock st;
    const double cs = m.cellSize();
    st.width = (static_cast<double>(m.width()) - 0.25) * cs;
    st.height = (static_cast<double>(m.height()) - 0.25) * cs;
    st.thickness = stockTop - stockBottom;
    st.cellSize = cs;
    st.origin = {m.origin().x - 0.5 * cs, m.origin().y - 0.5 * cs};
    st.topZ = stockTop;
    st.trackPassIds = false;
    simulate::Simulator sim(st);
    REQUIRE(sim.ok());
    std::uint32_t id = 1;
    for (const auto& pr : paths) {
        auto shape = simulate::ToolShape::fromTool(pr.second->tool);
        REQUIRE(shape);
        if (!pr.first->moves.empty()) {
            REQUIRE(sim.addToolpath(*pr.first, *shape, id++));
        }
    }
    REQUIRE(sim.advanceAll());
    for (const auto& dg : sim.diagnostics()) {
        CHECK(dg.kind != simulate::DiagnosticKind::BelowTable);
    }
    return sim.heightfield();
}

simulate::Heightfield simulateAll(const HeightField& m, const Finish3dParams& p, const Finish3dResult& r,
                                  const std::vector<Rough3dTool>& tools) {
    std::vector<std::pair<const Toolpath*, const Rough3dTool*>> v;
    for (std::size_t k = 0; k < r.toolpaths.size(); ++k) {
        v.emplace_back(&r.toolpaths[k], &tools[k]);
    }
    return simulateOn(m, p.stockTop, p.stockBottom, v);
}

/// Worst (model + allowance - sim): > 0 is a gouge.
double worstGouge(const HeightField& m, const simulate::Heightfield& hf, const Finish3dParams& p) {
    double worst = -1e9;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            const float mv = m.at(i, j);
            if (std::isnan(mv)) {
                continue;
            }
            const double target = std::max(static_cast<double>(mv) + p.allowance, p.stockBottom);
            worst = std::max(worst, target - hf.height(i, j));
        }
    }
    return worst;
}

/// Worst (sim - (model + allowance)) over cells (the stock left above the model).
double worstExcess(const HeightField& m, const simulate::Heightfield& hf, const Finish3dParams& p,
                   bool (*skip)(std::size_t, std::size_t, std::size_t) = nullptr) {
    double worst = -1e9;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            if (std::isnan(m.at(i, j)) || (skip != nullptr && skip(i, j, m.width()))) {
                continue;
            }
            const double target = std::max(static_cast<double>(m.at(i, j)) + p.allowance, p.stockBottom);
            worst = std::max(worst, static_cast<double>(hf.height(i, j)) - target);
        }
    }
    return worst;
}

/// Cells nearer than the tool radius to the grid edge are not covered: tool centres stay in the model
/// rectangle (ADR-0072), so only the interior (rho <= 5.5 of the 7.5 half width) is compared.
/// Excess of the finished dome above its own model against the predicted scallop bound for a ball of
/// radius `r` and passes `s` apart on the surface (plan spacing for rasters is widened by 1/cos(slope)).
double domeExcessMargin(const HeightField& m, const simulate::Heightfield& hf, double r, double s, bool raster,
                        double tol) {
    const double c = 0.5 * static_cast<double>(m.width() - 1) * m.cellSize();
    double worstMargin = -1e9;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            const double x = static_cast<double>(i) * m.cellSize() - c;
            const double y = static_cast<double>(j) * m.cellSize() - c;
            const double rho = std::hypot(x, y);
            if (rho > 5.5) {
                continue;
            }
            const double sinA = rho / kDomeR;
            const double cosA = std::sqrt(1.0 - sinA * sinA);
            const double surfaceSpacing = raster ? s / cosA : s * 1.12;
            // Scallop normal to the surface; its vertical extent is that over cos(slope).
            const double bound = ballScallopHeight(r, surfaceSpacing) / cosA + tol;
            const double excess = static_cast<double>(hf.height(i, j)) - static_cast<double>(m.at(i, j));
            worstMargin = std::max(worstMargin, excess - bound);
        }
    }
    return worstMargin;
}

}  // namespace

TEST_CASE("finish3d: ball scallop height formula", "[finish3d]") {
    CHECK(ballScallopHeight(3.0, 0.0) == 0.0);
    CHECK(ballScallopHeight(3.0, 6.0) == 3.0);
    CHECK(ballScallopHeight(3.0, 10.0) == 3.0);
    CHECK(std::fabs(ballScallopHeight(3.0, 2.0) - (3.0 - std::sqrt(8.0))) < 1e-12);
    CHECK(ballScallopHeight(0.0, 1.0) == 0.0);
}

TEST_CASE("finish3d: raster and offset on a sphere stay within the predicted scallop bound", "[finish3d]") {
    const HeightField m = makeDome(61, 0.25);
    for (auto strat : {Finish3dStrategy::Raster, Finish3dStrategy::Offset}) {
        for (double stepover : {0.5, 1.0}) {
            Finish3dParams p = baseParams();
            p.strategy = strat;
            const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 3.0, stepover)};
            const Finish3dResult r = finish3dToolpath(m, tools, p);
            REQUIRE(r.diagnostics.empty());
            REQUIRE(!r.toolpaths[0].moves.empty());
            const auto hf = simulateAll(m, p, r, tools);
            INFO("strategy " << static_cast<int>(strat) << " stepover " << stepover);
            CHECK(worstGouge(m, hf, p) <= 1e-4);
            const double margin = domeExcessMargin(m, hf, 1.5, stepover, strat == Finish3dStrategy::Raster, p.tolerance);
            CHECK(margin <= 1e-3);
            // The bound is meaningful: a wider stepover really leaves more.
            CHECK(worstExcess(m, hf, p) > ballScallopHeight(1.5, stepover) * 0.2);
        }
    }
}

TEST_CASE("finish3d: never gouges with any tool shape, angle, strategy and allowance", "[finish3d]") {
    const HeightField m = makeModel(36, 30, 0.5, true);
    const tooling::ToolType types[] = {tooling::ToolType::EndMill, tooling::ToolType::BallNose,
                                       tooling::ToolType::VBit};
    for (auto type : types) {
        for (auto strat : {Finish3dStrategy::Raster, Finish3dStrategy::Offset}) {
            for (double allowance : {0.0, 0.3}) {
                Finish3dParams p = baseParams();
                p.strategy = strat;
                p.allowance = allowance;
                p.rasterAngle = 0.6;
                Rough3dTool tool = makeTool(type, 3.0, 0.7);
                if (type == tooling::ToolType::VBit) {
                    tool.tool.includedAngle = 1.2;
                }
                const std::vector<Rough3dTool> tools{tool};
                const Finish3dResult r = finish3dToolpath(m, tools, p);
                INFO("type " << static_cast<int>(type) << " strat " << static_cast<int>(strat) << " allowance "
                             << allowance);
                REQUIRE(!r.toolpaths[0].moves.empty());
                const auto hf = simulateAll(m, p, r, tools);
                CHECK(worstGouge(m, hf, p) <= 1e-4);
                // The pit floor is reached and the allowance is really left on it.
                CHECK(std::fabs(hf.height(16, 12) - (-7.5 + allowance)) < 1e-3);
            }
        }
    }
}

TEST_CASE("finish3d: the tolerance bounds the stock left above the drop surface", "[finish3d]") {
    const HeightField m = makeDome(61, 0.25);
    double prevMoves = 0.0;
    double prevExcess = 1e9;
    for (double tol : {0.2, 0.05, 0.005}) {
        Finish3dParams p = baseParams();
        p.tolerance = tol;
        const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 3.0, 0.2)};
        const Finish3dResult r = finish3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        const auto hf = simulateAll(m, p, r, tools);
        const double margin = domeExcessMargin(m, hf, 1.5, 0.2, true, 1.6 * tol);
        double ex = 0.0;  // worst interior excess above the model: non-increasing as the tolerance shrinks
        for (std::size_t j = 15; j < 46; ++j) {
            for (std::size_t i = 15; i < 46; ++i) {
                ex = std::max(ex, static_cast<double>(hf.height(i, j)) - static_cast<double>(m.at(i, j)));
            }
        }
        CHECK(worstGouge(m, hf, p) <= 1e-4);
        // Scallop (tiny at stepover 0.2) + the tolerance (+ a little for the unchecked quarter points).
        CHECK(margin <= 1e-3);
        CHECK(static_cast<double>(r.toolpaths[0].moves.size()) > prevMoves);
        CHECK(ex <= prevExcess + 1e-3);
        prevMoves = static_cast<double>(r.toolpaths[0].moves.size());
        prevExcess = ex;
    }
}

TEST_CASE("finish3d: raster line order and direction", "[finish3d]") {
    relief::Grid g{21, 21, 0.5, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 2.0, 2.0)};
    auto feeds = [&](const Finish3dParams& p) {
        const Finish3dResult r = finish3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        std::vector<geometry::Vec3> v;
        for (const Move& mv : r.toolpaths[0].moves) {
            if (mv.kind == MoveKind::Feed && (v.empty() || v.back().x != mv.to.x || v.back().y != mv.to.y)) {
                v.push_back(mv.to);
            }
        }
        return v;
    };
    Finish3dParams p = baseParams();
    // Climb, angle 0: lines step towards -Y, the first line runs towards +X at the top (y = 10).
    auto v = feeds(p);
    REQUIRE(v.size() >= 4);
    CHECK(v.front().y == 10.0);
    CHECK(v.back().y == 0.0);
    CHECK(v[1].x > v[0].x);
    std::vector<double> ys;
    for (const auto& q : v) {
        if (ys.empty() || ys.back() != q.y) {
            ys.push_back(q.y);
        }
    }
    CHECK(ys == std::vector<double>{10.0, 8.0, 6.0, 4.0, 2.0, 0.0});
    // Zig-zag: the second line runs back towards -X.
    {
        const Finish3dResult zz = finish3dToolpath(m, tools, p);
        const auto& all = zz.toolpaths[0].moves;
        double firstX = 0.0;
        double secondX = 0.0;
        bool seenSecond = false;
        for (const Move& mv : all) {
            if (mv.kind == MoveKind::Feed && mv.to.y == 8.0 && !seenSecond) {
                seenSecond = true;
                secondX = mv.to.x;
            }
            if (mv.kind == MoveKind::Feed && mv.to.y == 10.0) {
                firstX = mv.to.x;
            }
        }
        CHECK(firstX == 10.0);
        CHECK(secondX == 10.0);  // enters the second line at the end where the first one stopped
    }
    // Conventional: the opposite order; reverse steps flips it again.
    p.direction = CutDirection::Conventional;
    v = feeds(p);
    CHECK(v.front().y == 0.0);
    CHECK(v.back().y == 10.0);
    p.reverseSteps = true;
    v = feeds(p);
    CHECK(v.front().y == 10.0);
    // Angle 90 degrees: lines along Y; climb steps towards +X.
    p = baseParams();
    p.rasterAngle = 1.5707963267948966;
    v = feeds(p);
    CHECK(std::fabs(v.front().x - 0.0) < 1e-9);
    CHECK(std::fabs(v.back().x - 10.0) < 1e-9);
    CHECK(v[1].y > v[0].y);
    // One-way travel: every line runs towards +X.
    p = baseParams();
    p.travel = RasterTravel::OneWay;
    const Finish3dResult r = finish3dToolpath(m, tools, p);
    double lastX = -1.0;
    for (const Move& mv : r.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Feed) {
            CHECK(mv.to.x >= lastX - 1e-9);
            lastX = mv.to.x;
        } else if (mv.kind == MoveKind::Rapid) {
            lastX = -1.0;
        }
    }
}

TEST_CASE("finish3d: offset passes are a constant surface distance apart", "[finish3d]") {
    relief::Grid g{41, 41, 0.5, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const double s = 2.0;
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 4.0, s)};
    Finish3dParams p = baseParams();
    p.strategy = Finish3dStrategy::Offset;
    const Finish3dResult r = finish3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    // On the flat model every feed vertex lies on a square ring at a multiple of the stepover from the
    // edge (within the grid-distance error), ring k at k * s.
    std::vector<int> ringCount(8, 0);
    for (const Move& mv : r.toolpaths[0].moves) {
        if (mv.kind != MoveKind::Feed) {
            continue;
        }
        const double d = std::min({mv.to.x, mv.to.y, 20.0 - mv.to.x, 20.0 - mv.to.y});
        const double k = d / s;
        CHECK(std::fabs(k - std::round(k)) < 0.07);
        ++ringCount[static_cast<std::size_t>(std::round(k))];
    }
    for (int k = 0; k < 5; ++k) {
        CHECK(ringCount[static_cast<std::size_t>(k)] > 0);
    }
    // Outside in: the first pass is at the edge, the innermost last. Reverse steps: inside out.
    auto ringOf = [&](const Move& mv) {
        return std::min({mv.to.x, mv.to.y, 20.0 - mv.to.x, 20.0 - mv.to.y});
    };
    const Move* firstFeed = nullptr;
    const Move* lastFeed = nullptr;
    for (const Move& mv : r.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Feed) {
            firstFeed = firstFeed != nullptr ? firstFeed : &mv;
            lastFeed = &mv;
        }
    }
    CHECK(ringOf(*firstFeed) < 0.1);
    CHECK(ringOf(*lastFeed) > 7.0);
    p.reverseSteps = true;
    const Finish3dResult rv = finish3dToolpath(m, tools, p);
    firstFeed = nullptr;
    for (const Move& mv : rv.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Feed) {
            firstFeed = &mv;
            break;
        }
    }
    CHECK(ringOf(*firstFeed) > 7.0);
    // Climb vs conventional: the rings run in opposite senses (signed area).
    auto area = [&](const Finish3dResult& res) {
        double a = 0.0;
        geometry::Vec3 prev{};
        bool have = false;
        for (const Move& mv : res.toolpaths[0].moves) {
            if (mv.kind != MoveKind::Feed) {
                have = false;
                continue;
            }
            const double d = std::min({mv.to.x, mv.to.y, 20.0 - mv.to.x, 20.0 - mv.to.y});
            if (have && d > 3.9 && d < 4.1) {
                a += prev.x * mv.to.y - mv.to.x * prev.y;
            }
            prev = mv.to;
            have = true;
        }
        return a;
    };
    p = baseParams();
    p.strategy = Finish3dStrategy::Offset;
    const double climbA = area(finish3dToolpath(m, tools, p));
    p.direction = CutDirection::Conventional;
    const double convA = area(finish3dToolpath(m, tools, p));
    // Climb keeps the inside (higher distance) on the right: clockwise rings, negative area.
    CHECK(climbA < -10.0);
    CHECK(convA > 10.0);
}

TEST_CASE("finish3d: boundary modes restrict the cut region", "[finish3d]") {
    const HeightField m = makeModel(36, 30, 0.5, true);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 1.0, 0.5)};
    for (auto strat : {Finish3dStrategy::Raster, Finish3dStrategy::Offset}) {
        Finish3dParams p = baseParams();
        p.strategy = strat;
        p.boundary = Rough3dBoundary::Vectors;
        geometry::Polyline sq;
        sq.closed = true;
        sq.points = {{4.0, -1.0}, {9.0, -1.0}, {9.0, 4.0}, {4.0, 4.0}};
        p.vectors = {sq};
        const Finish3dResult r = finish3dToolpath(m, tools, p);
        REQUIRE(r.diagnostics.empty());
        for (const Move& mv : r.toolpaths[0].moves) {
            if (mv.kind == MoveKind::Feed) {
                CHECK(mv.to.x >= 4.0 - 0.26);
                CHECK(mv.to.x <= 9.0 + 0.26);
                CHECK(mv.to.y >= -1.0 - 0.26);
                CHECK(mv.to.y <= 4.0 + 0.26);
            }
        }
        // Growing the vectors by 1 mm widens the cut.
        p.boundaryOffset = 1.0;
        const Finish3dResult big = finish3dToolpath(m, tools, p);
        double minX = 1e9;
        for (const Move& mv : big.toolpaths[0].moves) {
            if (mv.kind == MoveKind::Feed) {
                minX = std::min(minX, mv.to.x);
            }
        }
        CHECK(minX < 3.3);
        CHECK(minX >= 3.0 - 0.26);
        // Model boundary: nothing is cut over transparent cells' neighbourhood shrunk by 1 mm.
        p = baseParams();
        p.strategy = strat;
        p.boundary = Rough3dBoundary::Model;
        p.boundaryOffset = -1.0;
        const Finish3dResult mr = finish3dToolpath(m, tools, p);
        REQUIRE(!mr.toolpaths[0].moves.empty());
        for (const Move& mv : mr.toolpaths[0].moves) {
            if (mv.kind == MoveKind::Feed) {
                CHECK(mv.to.x >= 2.0 + 1.0 - 0.26);
                CHECK(mv.to.y >= -3.0 + 1.0 - 0.26);
            }
        }
    }
    // Empty regions are reported.
    Finish3dParams p = baseParams();
    p.boundary = Rough3dBoundary::Vectors;
    CHECK(finish3dToolpath(m, tools, p).diagnostics.at(0).issue == ToolpathIssue::NoRegion);
}

TEST_CASE("finish3d: rest machining cuts only where the earlier tool left material", "[finish3d]") {
    const HeightField m = makeModel(40, 32, 0.5);
    // A rough flat tool leaves the pit and the fine detail; the ball then finishes only there.
    Rough3dParams rp;
    rp.stockTop = 0.0;
    rp.stockBottom = -10.0;
    rp.safeZ = 5.0;
    rp.strategy = Rough3dStrategy::Raster;
    const Rough3dTool big = makeTool(tooling::ToolType::EndMill, 5.0, 1.0, "big");
    const Rough3dResult rough = rough3dToolpath(m, {big}, rp);
    REQUIRE(rough.diagnostics.empty());

    Finish3dParams p = baseParams();
    p.priors.push_back({rough.toolpaths[0], big});
    const Rough3dTool fine = makeTool(tooling::ToolType::BallNose, 2.0, 0.5, "fine");
    const Finish3dResult all = finish3dToolpath(m, {fine}, baseParams());
    const Finish3dResult rest = finish3dToolpath(m, {fine}, p);
    REQUIRE(rest.diagnostics.empty());
    REQUIRE(!rest.toolpaths[0].moves.empty());
    CHECK(rest.toolpaths[0].moves.size() < all.toolpaths[0].moves.size());

    const auto left = simulateOn(m, 0.0, -10.0, {{&rough.toolpaths[0], &big}});
    const auto fin = simulateOn(m, 0.0, -10.0, {{&rough.toolpaths[0], &big}, {&rest.toolpaths[0], &fine}});
    CHECK(worstGouge(m, fin, p) <= 1e-4);
    // Eligible cells: where the ball can go lower than the stock left (by more than the rest tolerance).
    std::vector<float> T;
    std::uint64_t work = 0;
    const auto ballShape = *simulate::ToolShape::ballNose(2.0);
    REQUIRE(dropToolSurface(m, ballShape, 0.0, T, 1ULL << 40, work) == DropStatus::Ok);
    std::size_t eligible = 0;
    std::size_t cutCells = 0;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            const double t = std::max(static_cast<double>(T[j * m.width() + i]), -10.0);
            const bool e = static_cast<double>(left.height(i, j)) > t + p.restTolerance;
            eligible += e ? 1 : 0;
            const bool changed = fin.height(i, j) < left.height(i, j) - 1e-6;
            cutCells += changed ? 1 : 0;
            // Everything cut was within reach of an eligible cell: radius + stepover.
            if (changed) {
                bool near = false;
                for (std::size_t qj = 0; qj < m.height() && !near; ++qj) {
                    for (std::size_t qi = 0; qi < m.width() && !near; ++qi) {
                        const double tq = std::max(static_cast<double>(T[qj * m.width() + qi]), -10.0);
                        if (static_cast<double>(left.height(qi, qj)) > tq + p.restTolerance &&
                            std::hypot(static_cast<double>(qi) - static_cast<double>(i),
                                       static_cast<double>(qj) - static_cast<double>(j)) *
                                    0.5 <= 0.5 + 0.5 + 1.0 + 1e-9) {
                            near = true;
                        }
                    }
                }
                CHECK(near);
            }
        }
    }
    CHECK(eligible > 0);
    CHECK(cutCells > 0);
    // Cells the rough already finished (and far from eligible ones) are untouched: compare with the
    // full finish, which cuts many more cells.
    const auto fullFin = simulateOn(m, 0.0, -10.0, {{&rough.toolpaths[0], &big}, {&all.toolpaths[0], &fine}});
    std::size_t fullCut = 0;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            fullCut += fullFin.height(i, j) < left.height(i, j) - 1e-6 ? 1U : 0U;
        }
    }
    CHECK(cutCells < fullCut);
}

TEST_CASE("finish3d: multi-tool lists use the earlier tools' simulated result", "[finish3d]") {
    const HeightField m = makeDome(61, 0.25);
    const Rough3dTool big = makeTool(tooling::ToolType::BallNose, 6.0, 3.0, "big");
    const Rough3dTool small = makeTool(tooling::ToolType::BallNose, 1.0, 0.4, "small");
    Finish3dParams p = baseParams();
    const Finish3dResult r = finish3dToolpath(m, {big, small}, p);
    REQUIRE(r.toolpaths.size() == 2);
    REQUIRE(!r.toolpaths[0].moves.empty());
    // The big tool leaves wide scallops that the small one then cuts.
    REQUIRE(!r.toolpaths[1].moves.empty());
    const auto hf = simulateAll(m, p, r, {big, small});
    CHECK(worstGouge(m, hf, p) <= 1e-4);
    const auto only0 = simulateOn(m, 0.0, -10.0, {{&r.toolpaths[0], &big}});
    CHECK(worstExcess(m, hf, p) < worstExcess(m, only0, p));
    // A tool that finds nothing to do (the first tool left less than the rest tolerance) says so.
    p.tolerance = 0.002;
    Rough3dTool fineStep = small;
    fineStep.tool.defaultCuttingData.stepover = 0.05;
    fineStep.tool.diameter = 1.0;
    const Finish3dResult same = finish3dToolpath(m, {fineStep, small}, p);
    bool noRest = false;
    for (const auto& d : same.diagnostics) {
        noRest = noRest || d.issue == ToolpathIssue::NoRestMaterial;
    }
    CHECK(noRest);
}

TEST_CASE("finish3d: invalid input, limits, cancel and determinism", "[finish3d]") {
    const HeightField m = makeModel(30, 24, 0.5);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 2.0, 0.5)};
    Finish3dParams p = baseParams();
    CHECK(finish3dToolpath(m, {}, p).diagnostics.at(0).issue == ToolpathIssue::NoTool);
    {
        Finish3dParams q = p;
        q.tolerance = 0.0;
        CHECK(finish3dToolpath(m, tools, q).diagnostics.at(0).issue == ToolpathIssue::InvalidParameters);
        q = p;
        q.stockBottom = 1.0;
        CHECK(finish3dToolpath(m, tools, q).diagnostics.at(0).issue == ToolpathIssue::InvalidParameters);
        q = p;
        q.safeZ = -1.0;
        CHECK(finish3dToolpath(m, tools, q).diagnostics.at(0).issue == ToolpathIssue::InvalidParameters);
        q = p;
        q.allowance = -0.1;
        CHECK(finish3dToolpath(m, tools, q).diagnostics.at(0).issue == ToolpathIssue::InvalidParameters);
        q = p;
        q.rasterAngle = std::numeric_limits<double>::quiet_NaN();
        CHECK(finish3dToolpath(m, tools, q).diagnostics.at(0).issue == ToolpathIssue::InvalidParameters);
    }
    Rough3dTool bad = tools[0];
    bad.tool.defaultCuttingData.stepover = 0.0;
    CHECK(finish3dToolpath(m, {bad}, p).diagnostics.at(0).issue == ToolpathIssue::InvalidTool);
    // A stepover wider than the tool is limited.
    Rough3dTool wide = tools[0];
    wide.tool.defaultCuttingData.stepover = 5.0;
    const Finish3dResult wr = finish3dToolpath(m, {wide}, p);
    CHECK(wr.diagnostics.at(0).issue == ToolpathIssue::StepoverLimited);
    CHECK(!wr.toolpaths[0].moves.empty());

    for (auto strat : {Finish3dStrategy::Raster, Finish3dStrategy::Offset}) {
        p.strategy = strat;
        const Finish3dResult a = finish3dToolpath(m, tools, p);
        const Finish3dResult b = finish3dToolpath(m, tools, p);
        REQUIRE(a.toolpaths[0].moves.size() == b.toolpaths[0].moves.size());
        bool same = true;
        for (std::size_t k = 0; k < a.toolpaths[0].moves.size(); ++k) {
            same = same && a.toolpaths[0].moves[k].kind == b.toolpaths[0].moves[k].kind &&
                   a.toolpaths[0].moves[k].to == b.toolpaths[0].moves[k].to;
        }
        CHECK(same);

        ToolpathRunControl ctl;
        ctl.workLimit = 2000;
        const Finish3dResult lim = finish3dToolpath(m, tools, p, ctl);
        CHECK(lim.toolpaths[0].moves.empty());
        CHECK(lim.diagnostics.back().issue == ToolpathIssue::WorkLimitExceeded);
        ctl = {};
        ctl.moveLimit = 50;
        const Finish3dResult ml = finish3dToolpath(m, tools, p, ctl);
        CHECK(ml.toolpaths[0].moves.empty());
        CHECK(ml.diagnostics.back().issue == ToolpathIssue::WorkLimitExceeded);
        std::atomic<bool> stop{true};
        ctl = {};
        ctl.cancel = &stop;
        const Finish3dResult cr = finish3dToolpath(m, tools, p, ctl);
        CHECK(cr.toolpaths[0].moves.empty());
        CHECK(cr.diagnostics.back().issue == ToolpathIssue::Cancelled);
    }
}

TEST_CASE("finish3d: the tool is lifted over a pit wall by the exact piece height", "[finish3d]") {
    // A flat tool crossing the edge of a pit between two nodes: a piece spanning the wall must be
    // at the height that clears the higher side at the piece, not at its end heights only.
    relief::Grid g{11, 3, 1.0, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -5.0f, m) == relief::Status::Ok);
    for (std::size_t j = 0; j < 3; ++j) {
        m.set(5, j, -1.0f);  // a ridge at x = 5
    }
    Finish3dParams p = baseParams();
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::EndMill, 1.0, 1.0)};
    const Finish3dResult r = finish3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    const auto hf = simulateAll(m, p, r, tools);
    CHECK(worstGouge(m, hf, p) <= 1e-4);
    // The ridge cell is left at its own height, the floor beside it is cut down to -5.
    CHECK(std::fabs(hf.height(5, 1) - (-1.0)) < 1e-4);
    CHECK(std::fabs(hf.height(2, 1) - (-5.0)) < 1e-4);
}

TEST_CASE("finish3d: the last line lies on the far edge when the stepover does not divide the width", "[finish3d]") {
    relief::Grid g{21, 21, 0.5, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 4.0, 3.0)};
    const Finish3dResult r = finish3dToolpath(m, tools, baseParams());
    REQUIRE(r.diagnostics.empty());
    std::vector<double> ys;
    for (const Move& mv : r.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Feed && (ys.empty() || ys.back() != mv.to.y)) {
            ys.push_back(mv.to.y);
        }
    }
    CHECK(ys == std::vector<double>{10.0, 9.0, 6.0, 3.0, 0.0});
}

TEST_CASE("finish3d: the floor limits the tip and the allowance rides on top", "[finish3d]") {
    const HeightField m = makeModel(36, 30, 0.5);
    Finish3dParams p = baseParams();
    p.stockBottom = -5.0;
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 2.0, 0.6)};
    const Finish3dResult r = finish3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    double minZ = 1e9;
    for (const Move& mv : r.toolpaths[0].moves) {
        minZ = std::min(minZ, mv.to.z);
    }
    CHECK(minZ >= -5.0);
    CHECK(minZ < -4.9);
    const auto hf = simulateAll(m, p, r, tools);
    CHECK(worstGouge(m, hf, p) <= 1e-4);
}

TEST_CASE("finish3d: offset spacing is measured along the slope", "[finish3d]") {
    // A plane rising 0.5 per mm in Y: on the surface one mm of Y is 1.118 mm. Rings from the y = 0 edge
    // sit at y = k * s / 1.118 (checked away from the x edges and the middle ridge).
    relief::Grid g{41, 41, 0.5, {0.0, 0.0}};
    std::vector<float> d(41 * 41);
    for (std::size_t j = 0; j < 41; ++j) {
        for (std::size_t i = 0; i < 41; ++i) {
            d[j * 41 + i] = static_cast<float>(-9.0 + 0.5 * static_cast<double>(j) * 0.5);
        }
    }
    HeightField m;
    REQUIRE(HeightField::fromData(g, std::move(d), m) == relief::Status::Ok);
    const double s = 2.0;
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 4.0, s)};
    Finish3dParams p = baseParams();
    p.strategy = Finish3dStrategy::Offset;
    const Finish3dResult r = finish3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    std::size_t counted = 0;
    for (const Move& mv : r.toolpaths[0].moves) {
        if (mv.kind == MoveKind::Feed && mv.to.x > 8.0 && mv.to.x < 12.0 && mv.to.y > 0.5 && mv.to.y < 6.0) {
            const double k = mv.to.y * std::sqrt(1.25) / s;
            CHECK(std::fabs(k - std::round(k)) < 0.07);
            ++counted;
        }
    }
    CHECK(counted > 5);
}

TEST_CASE("finish3d: runs far apart are not linked", "[finish3d]") {
    // A round region: near its top the ends of consecutive lines jump by more than two stepovers (a chord
    // between them would be inside the region), so those lines are separate cuts.
    relief::Grid g{41, 41, 0.5, {0.0, 0.0}};
    HeightField m;
    REQUIRE(HeightField::create(g, -3.0f, m) == relief::Status::Ok);
    const std::vector<Rough3dTool> tools{makeTool(tooling::ToolType::BallNose, 2.0, 0.8)};
    Finish3dParams p = baseParams();
    p.boundary = Rough3dBoundary::Vectors;
    geometry::Polyline circle;
    circle.closed = true;
    for (int k = 0; k < 96; ++k) {
        const double a = 2.0 * 3.14159265358979323846 * k / 96.0;
        circle.points.push_back({10.0 + 10.0 * std::cos(a), 10.0 + 10.0 * std::sin(a)});
    }
    p.vectors = {circle};
    const Finish3dResult r = finish3dToolpath(m, tools, p);
    REQUIRE(r.diagnostics.empty());
    std::size_t rapids = 0;
    for (const Move& mv : r.toolpaths[0].moves) {
        rapids += mv.kind == MoveKind::Rapid ? 1U : 0U;
    }
    // Pinned: 24 cuts without any linking; links within two stepovers join the rest (the length limit
    // only shapes the path, any link is cut at its exact height, ADR-0072).
    CHECK(rapids == 5);
}

TEST_CASE("finish3d: rest passes reach cells between the passes", "[finish3d]") {
    const HeightField m = makeModel(40, 32, 0.5);
    Rough3dParams rp;
    rp.stockTop = 0.0;
    rp.stockBottom = -10.0;
    rp.safeZ = 5.0;
    rp.strategy = Rough3dStrategy::Raster;
    const Rough3dTool big = makeTool(tooling::ToolType::EndMill, 5.0, 1.0, "big");
    const Rough3dResult rough = rough3dToolpath(m, {big}, rp);
    REQUIRE(rough.diagnostics.empty());
    Finish3dParams p = baseParams();
    p.rasterAngle = 0.6;
    p.priors.push_back({rough.toolpaths[0], big});
    const Rough3dTool fine = makeTool(tooling::ToolType::BallNose, 2.0, 0.5, "fine");
    const Finish3dResult rest = finish3dToolpath(m, {fine}, p);
    REQUIRE(!rest.toolpaths[0].moves.empty());
    const auto left = simulateOn(m, 0.0, -10.0, {{&rough.toolpaths[0], &big}});
    const auto fin = simulateOn(m, 0.0, -10.0, {{&rough.toolpaths[0], &big}, {&rest.toolpaths[0], &fine}});
    std::vector<float> T;
    std::uint64_t work = 0;
    REQUIRE(dropToolSurface(m, *simulate::ToolShape::ballNose(2.0), 0.0, T, 1ULL << 40, work) == DropStatus::Ok);
    std::size_t eligible = 0;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            const double t = std::max(static_cast<double>(T[j * m.width() + i]), -10.0);
            if (static_cast<double>(left.height(i, j)) > t + p.restTolerance) {
                ++eligible;
                // Cut down to within a scallop (r = 1, spacing 0.5, steep slopes) of the tip target.
                CHECK(static_cast<double>(fin.height(i, j)) <= t + 0.2);
            }
        }
    }
    CHECK(eligible > 0);
    // Cut points reach past the eligible cells (the passes between them), but never beyond one stepover
    // (plus half a cell for the nearest-node rounding) from an eligible cell.
    std::size_t beyond = 0;
    for (const Move& mv : rest.toolpaths[0].moves) {
        if (mv.kind != MoveKind::Feed) {
            continue;
        }
        double nearest = 1e9;
        for (std::size_t j = 0; j < m.height(); ++j) {
            for (std::size_t i = 0; i < m.width(); ++i) {
                const double t = std::max(static_cast<double>(T[j * m.width() + i]), -10.0);
                if (static_cast<double>(left.height(i, j)) > t + p.restTolerance) {
                    nearest = std::min(nearest, std::hypot(mv.to.x - (2.0 + static_cast<double>(i) * 0.5),
                                                           mv.to.y - (-3.0 + static_cast<double>(j) * 0.5)));
                }
            }
        }
        CHECK(nearest <= 0.5 + 0.25 + 0.5 + 1e-9);
        beyond += nearest > 0.65 ? 1U : 0U;
    }
    CHECK(beyond > 0);
}
