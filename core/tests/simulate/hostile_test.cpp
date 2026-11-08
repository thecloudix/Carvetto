// T-0222 AC2/AC3: hostile and degenerate moves are refused up front or bounded; thousands of
// randomly corrupted jobs never produce a crash, a hang, a non-finite height or an out-of-range
// one (run under ASan/UBSan in the T-0222 verification).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "../geometry/cpu_time.hpp"
#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;
using simulate::SimStatus;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPosInf = std::numeric_limits<double>::infinity();

Toolpath pathOf(std::initializer_list<Move> moves) {
    Toolpath t;
    for (const Move& m : moves) t.append(m);
    return t;
}

bool sane(const Heightfield& f) {
    std::size_t cut = 0;
    for (std::size_t k = 0; k < f.heights().size(); ++k) {
        const float v = f.heights()[k];
        if (!std::isfinite(v) || v > static_cast<float>(f.topZ()) || v < static_cast<float>(f.bottomZ())) return false;
        cut += f.cutStates()[k];
        if (f.tracksPassIds() && (f.cutStates()[k] != 0) != (f.passIds()[k] != 0)) return false;
    }
    return cut == f.cutCells();
}

}  // namespace

TEST_CASE("simulate: non-finite and out-of-range coordinates are refused with the move index", "[simulate][hostile]") {
    const Stock stock = stockOf(10, 10, 0.5);
    const ToolShape tool = *ToolShape::flatEndMill(2.0);
    const double bad[] = {kNaN, kPosInf, -kPosInf, 1e300, -1e300, 1.0000001e7, -1.0000001e7};
    for (double b : bad) {
        INFO("value " << b);
        for (int field = 0; field < 5; ++field) {
            Simulator sim(stock);
            Toolpath good = pathOf({Move::rapid({1, 1, 5}), Move::feed({2, 2, -1})});
            REQUIRE(sim.addToolpath(good, tool, 1));
            Move m = Move::arcCCW({5, 5, -1}, {4, 4});
            if (field == 0) m.to.x = b;
            if (field == 1) m.to.y = b;
            if (field == 2) m.to.z = b;
            if (field == 3) m.centre.x = b;
            if (field == 4) m.centre.y = b;
            Toolpath t = pathOf({Move::feed({3, 3, -1}), m});
            CHECK_FALSE(sim.addToolpath(t, tool, 2));
            CHECK(sim.error().status == SimStatus::InvalidMove);
            REQUIRE(sim.error().move.has_value());
            CHECK(*sim.error().move == 3);
            CHECK(sim.moveCount() == 2);  // nothing of the refused toolpath was queued
            // The simulator stays usable.
            CHECK(sim.advanceAll());
            CHECK(sane(sim.heightfield()));
            Toolpath later = pathOf({Move::feed({4, 4, -1})});
            CHECK(sim.addToolpath(later, tool, 3));
            CHECK(sim.ok());
        }
    }
    // A non-finite start position, a bad feed override and a bad dwell are handled too.
    Simulator sim(stock);
    Toolpath t = pathOf({Move::feed({3, 3, -1})});
    CHECK_FALSE(sim.addToolpath(t, tool, 1, Vec3{kNaN, 0, 0}));
    CHECK(sim.error().status == SimStatus::InvalidMove);
    Toolpath odd = pathOf({Move::rapid({1, 1, 5}), Move::feed({3, 3, -1}, kNaN), Move::feed({4, 4, -1}, -5.0),
                           Move::dwell({4, 4, -1}, kNaN), Move::dwell({4, 4, -1}, -3.0), Move::dwell({4, 4, -1}, 1e300)});
    REQUIRE(sim.addToolpath(odd, tool, 2));
    CHECK(sim.advanceAll());
    for (double e : sim.moveEndTimes()) CHECK(std::isfinite(e));
    CHECK(sane(sim.heightfield()));
}

TEST_CASE("simulate: degenerate arcs, zero-length moves and unusable tools", "[simulate][hostile]") {
    const Stock stock = stockOf(10, 10, 0.5);
    const ToolShape tool = *ToolShape::ballNose(2.0);
    {  // zero radius with a different end point
        Simulator sim(stock);
        Toolpath t = pathOf({Move::rapid({5, 5, 5}), Move::arcCW({6, 5, 5}, {5, 5})});
        CHECK_FALSE(sim.addToolpath(t, tool, 1));
        CHECK(sim.error().status == SimStatus::InvalidMove);
        CHECK(*sim.error().move == 1);
    }
    {  // ends at its own centre
        Simulator sim(stock);
        Toolpath t = pathOf({Move::rapid({5, 5, 5}), Move::arcCCW({3, 5, 5}, {3, 5})});
        CHECK_FALSE(sim.addToolpath(t, tool, 1));
        CHECK(sim.error().status == SimStatus::InvalidMove);
    }
    {  // zero radius staying put (a helix of no radius) is a vertical move; zero-length moves are stamps
        Simulator sim(stock);
        Toolpath t = pathOf({Move::rapid({5, 5, 5}), Move::arcCW({5, 5, -1}, {5, 5}), Move::feed({5, 5, -1}),
                             Move::plunge({5, 5, -1}), Move::dwell({5, 5, -1}, 2.0), Move::retract({5, 5, 5})});
        REQUIRE(sim.addToolpath(t, tool, 1));
        REQUIRE(sim.advanceAll());
        std::size_t i = 0, j = 0;
        REQUIRE(sim.heightfield().cellAt({5.1, 5.1}, i, j));
        CHECK(sim.heightfield().height(i, j) < -0.8f);
        CHECK(sane(sim.heightfield()));
    }
    {  // microscopic arcs and segments
        Simulator sim(stock);
        Toolpath t = pathOf({Move::rapid({5, 5, 5}), Move::feed({5, 5, -1}), Move::arcCCW({5 + 1e-12, 5, -1}, {5, 5 + 1e-11}),
                             Move::feed({5 + 1e-300, 5, -1}), Move::arcCW({5, 5 + 2e-9, -1}, {5, 5 + 1e-9})});
        REQUIRE(sim.addToolpath(t, tool, 1));
        REQUIRE(sim.advanceAll());
        CHECK(sane(sim.heightfield()));
    }
    {  // invalid tools
        Simulator sim(stock);
        CHECK_FALSE(sim.addToolpath(pathOf({Move::feed({1, 1, 1})}), simulate::ToolShape(), 1));
        CHECK(sim.error().status == SimStatus::InvalidTool);
    }
    {  // queue limit
        simulate::SimOptions opt;
        opt.maxMoves = 5;
        Simulator sim(stock, opt);
        Toolpath big;
        for (int k = 0; k < 6; ++k) big.append(Move::rapid({1, 1, 5}));
        CHECK_FALSE(sim.addToolpath(big, tool, 1));
        CHECK(sim.error().status == SimStatus::TooManyMoves);
        Toolpath five;
        for (int k = 0; k < 5; ++k) five.append(Move::rapid({1, 1, 5}));
        CHECK(sim.addToolpath(five, tool, 1));
        CHECK_FALSE(sim.addToolpath(pathOf({Move::rapid({1, 1, 5})}), tool, 2));
    }
}

TEST_CASE("simulate: huge extents and radii are clipped to the stock and fast", "[simulate][hostile]") {
    const Stock stock = stockOf(20, 20, 0.25);
    const ToolShape tool = *ToolShape::vBit(3.0, 60 * kDeg);
    const double t0 = test::processCpuSeconds();
    {  // a line across the whole coordinate range
        const Simulator sim = run(stock, tool, {-1e7, -1e7, 5}, {Move::feed({-1e7, -1e7, -1}), Move::feed({1e7, 1e7, -1})});
        CHECK(sane(sim.heightfield()));
        CHECK(sim.heightfield().cutCells() > 50);
        std::size_t i = 0, j = 0;
        REQUIRE(sim.heightfield().cellAt({10.1, 10.1}, i, j));
        CHECK(sim.heightfield().height(i, j) < -0.99f);
        REQUIRE(sim.heightfield().cellAt({10.1, 15.1}, i, j));
        CHECK(sim.heightfield().height(i, j) == 0.0f);
    }
    {  // an arc of radius 1e7 through the stock
        const double rho = 1e7 - 12;
        const Vec2 c{10 - rho, 10};
        const double ang = 4e-6;
        const Vec3 s{10, 10, -1};
        const Vec3 e{c.x + rho * std::cos(ang), c.y + rho * std::sin(ang), -1};
        const Simulator sim = run(stock, tool, {s.x, s.y, 5}, {Move::feed(s), Move::arcCCW(e, c)});
        CHECK(sane(sim.heightfield()));
        CHECK(sim.heightfield().cutCells() > 50);
    }
    {  // a full circle of radius 5e6 passing near the stock, CW and CCW, with a helix
        for (int dir = 0; dir < 2; ++dir) {
            const double rho = 5e6;
            const Vec3 s{10, 10, -1};
            const Vec2 c{10 + rho, 10};
            const Move m = dir == 0 ? Move::arcCW({10, 10, -1}, c) : Move::arcCCW({10, 10, -2}, c);
            const Simulator sim = run(stock, tool, {s.x, s.y, 5}, {Move::feed(s), m});
            CHECK(sane(sim.heightfield()));
        }
    }
    {  // a tool as big as the stock and a million times the stock in diameter
        const Simulator sim = run(stock, *ToolShape::flatEndMill(10000.0), {10, 10, 5}, {Move::feed({10, 10, -1}), Move::feed({12, 12, -1})});
        CHECK(sane(sim.heightfield()));
        CHECK(sim.heightfield().cutCells() == sim.heightfield().cellCount());
    }
    const double t = test::processCpuSeconds() - t0;
    INFO("CPU " << t << " s");
    CHECK(t < 20.0);
}

TEST_CASE("simulate: the work budget stops a runaway job promptly", "[simulate][hostile]") {
    const Stock stock = stockOf(100, 100, 0.25);  // 160,000 cells
    simulate::SimOptions opt;
    opt.maxCellWork = 1e6;
    Simulator sim(stock, opt);
    Toolpath t;
    t.append(Move::rapid({50, 50, 5}));
    for (int k = 0; k < 100000; ++k) {
        t.append(Move::feed({50.0 + (k % 3), 50, -1.0 - 0.0001 * (k % 7)}));
    }
    REQUIRE(sim.addToolpath(t, *ToolShape::flatEndMill(200.0), 1));
    const double t0 = test::processCpuSeconds();
    CHECK_FALSE(sim.advanceAll());
    CHECK(test::processCpuSeconds() - t0 < 5.0);
    CHECK(sim.error().status == SimStatus::WorkBudgetExceeded);
    REQUIRE(sim.error().move.has_value());
    CHECK(*sim.error().move < 100);
    CHECK(sim.movesDone() < 100);
    CHECK_FALSE(sim.advanceToMove(1000000));
    CHECK_FALSE(sim.ok());
    sim.reset();  // the budget is per run
    CHECK(sim.ok());
    CHECK(sim.cellWork() == 0.0);
    CHECK(sim.advanceToMove(2));
}

TEST_CASE("simulate: fuzz: corrupted jobs never crash, hang or leave bad heights", "[simulate][hostile][fuzz]") {
    const std::vector<NamedTool> tools = allTools();
    const double specials[] = {kNaN, kPosInf, -kPosInf, 1e300, -1e300, 1e7, -1e7, 1.0000001e7, 1e-300, -1e-300,
                               5e-324, 0.0, -0.0, 1e-9, 1e-12, 1e6, 123456.789, 4.0, 4.5, 5.0};
    std::size_t refused = 0, ran = 0, budget = 0;
    const int seeds = 3000;
    for (int seed = 0; seed < seeds; ++seed) {
        Rng rng(static_cast<std::uint64_t>(seed) * 7919 + 13);
        Stock stock = stockOf(rng.uniform(2, 14), rng.uniform(2, 14), rng.uniform(0.2, 0.9), rng.uniform(0.5, 12));
        stock.topZ = rng.uniform() < 0.3 ? rng.uniform(0, 5) : 0.0;
        simulate::SimOptions opt;
        opt.maxCellWork = 5e6;
        Simulator sim(stock, opt);
        REQUIRE(sim.ok());
        const NamedTool& nt = tools[rng.next() % tools.size()];
        ToolShape shape = nt.shape;
        if (rng.uniform() < 0.3) shape = shape.withFluteLength(rng.uniform(0.5, 6));
        Toolpath t;
        t.tool.feedRate = rng.uniform() < 0.1 ? 0.0 : 800.0;
        t.tool.plungeRate = rng.uniform() < 0.1 ? kNaN : 200.0;
        const int n = 1 + static_cast<int>(rng.next() % 12);
        auto coord = [&](double lo, double hi) {
            return rng.uniform() < 0.12 ? specials[rng.next() % (sizeof(specials) / sizeof(specials[0]))] : rng.uniform(lo, hi);
        };
        for (int k = 0; k < n; ++k) {
            Move m;
            const int kind = static_cast<int>(rng.next() % 7);
            const Vec3 to{coord(-2, 16), coord(-2, 16), coord(-8, 6)};
            const Vec2 c{coord(-4, 18), coord(-4, 18)};
            const std::optional<double> fo = rng.uniform() < 0.2 ? std::optional<double>(coord(-10, 3000)) : std::nullopt;
            switch (kind) {
                case 0: m = Move::rapid(to); break;
                case 1: m = Move::feed(to, fo); break;
                case 2: m = Move::plunge(to, fo); break;
                case 3: m = Move::retract(to, fo); break;
                case 4: m = Move::arcCW(to, c, fo); break;
                case 5: m = Move::arcCCW(to, c, fo); break;
                default: m = Move::dwell(to, coord(-1, 5)); break;
            }
            t.append(m);
        }
        const std::optional<Vec3> from =
            rng.uniform() < 0.2 ? std::optional<Vec3>(Vec3{coord(0, 10), coord(0, 10), coord(0, 5)}) : std::nullopt;
        const bool queued = sim.addToolpath(t, shape, 1 + static_cast<std::uint32_t>(seed % 5), from);
        if (!queued) {
            ++refused;
            CHECK(sim.moveCount() == 0);
            CHECK(sim.error().status != SimStatus::Ok);
            continue;
        }
        ++ran;
        const bool finished = sim.advanceAll();
        budget += finished ? 0 : 1;
        REQUIRE(sane(sim.heightfield()));
        for (double e : sim.moveEndTimes()) REQUIRE(std::isfinite(e));
        for (const auto& d : sim.diagnostics()) REQUIRE(d.move < sim.moveCount());
        // time playback of the same job never misbehaves either
        sim.reset();
        sim.advanceToTime(sim.totalSeconds() * rng.uniform());
        REQUIRE(sane(sim.heightfield()));
        // two runs agree
        Simulator again(stock, opt);
        again.addToolpath(t, shape, 1 + static_cast<std::uint32_t>(seed % 5), from);
        sim.reset();
        sim.advanceAll();
        again.advanceAll();
        if (finished) {
            REQUIRE(again.heightfield().heights() == sim.heightfield().heights());
        }
    }
    INFO("refused " << refused << ", ran " << ran << ", stopped by the budget " << budget);
    CHECK(ran > 500);
    CHECK(refused > 500);
}
