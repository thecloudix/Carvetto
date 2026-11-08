// T-0222 AC1: incremental playback (up to move N / up to time t) equals a from-scratch run at the
// same point, partial moves are exact prefixes, and the queue of several toolpaths behaves.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <vector>

#include "carvetto/toolpath/time.hpp"
#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;

namespace {

/// A varied job: three toolpaths with different tools, rapids, plunges, feeds, arcs, helices,
/// retracts and dwells.
std::vector<std::pair<Toolpath, ToolShape>> variedJob(std::uint64_t seed, int movesPerPath) {
    Rng rng(seed);
    std::vector<std::pair<Toolpath, ToolShape>> out;
    const auto tools = allTools();
    for (int p = 0; p < 3; ++p) {
        Toolpath tp;
        tp.name = "p" + std::to_string(p);
        tp.tool.feedRate = 900.0;
        tp.tool.plungeRate = 300.0;
        tp.tool.toolNumber = p + 1;
        Vec3 cur{rng.uniform(4, 20), rng.uniform(4, 20), 5};
        tp.append(Move::rapid(cur));
        for (int k = 0; k < movesPerPath; ++k) {
            const double r = rng.uniform();
            Move m;
            if (r < 0.15) {
                m = Move::rapid({rng.uniform(3, 21), rng.uniform(3, 21), 5});
            } else if (r < 0.3) {
                m = Move::plunge({cur.x, cur.y, rng.uniform(-6, 0)});
            } else if (r < 0.6) {
                m = Move::feed({rng.uniform(3, 21), rng.uniform(3, 21), rng.uniform(-6, 0.5)});
            } else if (r < 0.85) {
                const double rho = rng.uniform(1, 5);
                const double a0 = rng.uniform(0, 2 * kPi);
                const double sw = rng.uniform(0.2, 1.9) * kPi * (rng.uniform() < 0.5 ? -1 : 1);
                const Vec2 c{cur.x - rho * std::cos(a0), cur.y - rho * std::sin(a0)};
                const Vec3 e{c.x + rho * std::cos(a0 + sw), c.y + rho * std::sin(a0 + sw),
                             rng.uniform() < 0.5 ? cur.z : rng.uniform(-6, 0)};
                m = sw > 0 ? Move::arcCCW(e, c) : Move::arcCW(e, c);
            } else if (r < 0.92) {
                m = Move::retract({cur.x, cur.y, 5});
            } else {
                m = Move::dwell(cur, 0.5);
            }
            cur = m.to;
            tp.append(m);
        }
        out.emplace_back(tp, tools[static_cast<std::size_t>(p) * 3 % tools.size()].shape);
    }
    return out;
}

Simulator build(const Stock& stock, const std::vector<std::pair<Toolpath, ToolShape>>& job) {
    Simulator sim(stock);
    std::uint32_t id = 1;
    for (const auto& [tp, shape] : job) {
        REQUIRE(sim.addToolpath(tp, shape, id++));
    }
    return sim;
}

bool sameState(const Simulator& a, const Simulator& b) {
    return a.heightfield().heights() == b.heightfield().heights() &&
           a.heightfield().cutStates() == b.heightfield().cutStates() &&
           a.heightfield().passIds() == b.heightfield().passIds() &&
           a.heightfield().cutCells() == b.heightfield().cutCells() && a.movesDone() == b.movesDone() &&
           a.diagnostics().size() == b.diagnostics().size();
}

}  // namespace

TEST_CASE("simulate: advancing in steps equals a full recompute at the same move index", "[simulate][playback]") {
    const Stock stock = stockOf(24, 24, 0.25, 8.0);
    for (std::uint64_t seed = 1; seed <= 6; ++seed) {
        INFO("seed " << seed);
        const auto job = variedJob(seed, 25);
        Simulator stepped = build(stock, job);
        const std::size_t total = stepped.moveCount();
        REQUIRE(total > 60);
        Rng rng(seed * 77);
        std::size_t n = 0;
        while (n < total) {
            n = std::min(total, n + 1 + static_cast<std::size_t>(rng.next() % 9));
            REQUIRE(stepped.advanceToMove(n));
            Simulator fresh = build(stock, job);
            REQUIRE(fresh.advanceToMove(n));
            REQUIRE(sameState(stepped, fresh));
        }
        CHECK(stepped.movesDone() == total);
        // A target behind the state does nothing; seekMove rewinds and replays.
        REQUIRE(stepped.advanceToMove(3));
        CHECK(stepped.movesDone() == total);
        REQUIRE(stepped.seekMove(total / 2));
        Simulator half = build(stock, job);
        half.advanceToMove(total / 2);
        CHECK(sameState(stepped, half));
        REQUIRE(stepped.seekMove(total));
        Simulator all = build(stock, job);
        all.advanceAll();
        CHECK(sameState(stepped, all));
        // Past the end clamps.
        CHECK(stepped.advanceToMove(total + 1000));
        CHECK(stepped.movesDone() == total);
    }
}

TEST_CASE("simulate: playback by time: completed moves and exact prefixes of the move in progress", "[simulate][playback][time]") {
    const Stock stock = stockOf(24, 16, 0.25, 8.0);
    const ToolShape tool = *ToolShape::ballNose(2.0);
    Toolpath tp;
    tp.tool.feedRate = 600.0;   // 10 mm/s
    tp.tool.plungeRate = 300.0; // 5 mm/s
    tp.append(Move::rapid({2, 5, 5}));
    tp.append(Move::plunge({2, 5, -1}));     // 6 mm at 5 mm/s = 1.2 s, ends at t = 1.2
    tp.append(Move::feed({12, 5, -1}));      // 1 s, ends at 2.2
    tp.append(Move::arcCCW({12, 13, -1}, {12, 9}));  // half circle radius 4, ~12.566 mm, ends at ~3.457
    simulate::SimOptions opt;
    opt.rapidRate = 6000.0;
    Simulator sim(stock, opt);
    REQUIRE(sim.addToolpath(tp, tool, 1));
    const std::vector<double>& ends = sim.moveEndTimes();
    REQUIRE(ends.size() == 4);
    CHECK(ends[0] == 0.0);
    CHECK(std::fabs(ends[1] - 1.2) < 1e-12);
    CHECK(std::fabs(ends[2] - 2.2) < 1e-12);
    CHECK(std::fabs(ends[3] - (2.2 + 4 * kPi / 10.0)) < 1e-9);
    CHECK(sim.totalSeconds() == ends.back());
    // The estimate of time.hpp agrees (same rates).
    const toolpath::TimeEstimate est = toolpath::estimateTime(tp, toolpath::MachineRates{6000.0, 600.0, 300.0});
    CHECK(std::fabs(est.total() - sim.totalSeconds()) < 1e-9);

    // Halfway through the feed: a prefix of 5 mm.
    REQUIRE(sim.advanceToTime(1.7));
    CHECK(sim.movesDone() == 2);
    CHECK(std::fabs(sim.partialFraction() - 0.5) < 1e-12);
    CHECK(std::fabs(sim.currentSeconds() - 1.7) < 1e-12);
    const Simulator expect = run(stock, tool, {2, 5, 5}, {Move::plunge({2, 5, -1}), Move::feed({7, 5, -1})});
    CHECK(sim.heightfield().heights() == expect.heightfield().heights());
    // Going on: the same call again does nothing; a later time continues without recomputing.
    REQUIRE(sim.advanceToTime(1.7));
    REQUIRE(sim.advanceToTime(1.5));
    CHECK(sim.heightfield().heights() == expect.heightfield().heights());
    // A quarter of the way along the half circle: 45 degrees of it.
    const double tArc = 2.2 + 0.25 * (4 * kPi / 10.0);
    REQUIRE(sim.advanceToTime(tArc));
    CHECK(sim.movesDone() == 3);
    CHECK(std::fabs(sim.partialFraction() - 0.25) < 1e-9);
    const double ang = -kPi / 2 + kPi / 4;  // start at 270 degrees about (12, 9), CCW
    const Vec3 mid{12 + 4 * std::cos(ang), 9 + 4 * std::sin(ang), -1};
    const Simulator expectArc = run(stock, tool, {2, 5, 5},
                                    {Move::plunge({2, 5, -1}), Move::feed({12, 5, -1}),
                                     Move::arcCCW(mid, {12, 9})});
    const auto& a = sim.heightfield().heights();
    const auto& b = expectArc.heightfield().heights();
    REQUIRE(a.size() == b.size());
    float worst = 0;
    for (std::size_t k = 0; k < a.size(); ++k) worst = std::max(worst, std::fabs(a[k] - b[k]));
    CHECK(worst < 1e-5f);
    // To the end: identical to a full run.
    REQUIRE(sim.advanceToTime(1e9));
    CHECK(sim.movesDone() == 4);
    CHECK(sim.partialFraction() == 0.0);
    Simulator full(stock, opt);
    full.addToolpath(tp, tool, 1);
    full.advanceAll();
    CHECK(sim.heightfield().heights() == full.heightfield().heights());
    // seekTime back to the middle of the feed reproduces the earlier state.
    REQUIRE(sim.seekTime(1.7));
    CHECK(sim.heightfield().heights() == expect.heightfield().heights());
    CHECK(sim.seekTime(0.0));
    CHECK(sim.heightfield().cutCells() == 0);
    // NaN is ignored, negative time cuts nothing.
    CHECK(sim.advanceToTime(std::nan("")));
    CHECK(sim.advanceToTime(-5.0));
    CHECK(sim.movesDone() <= 1);  // the zero-length first rapid completes at t = 0
}

TEST_CASE("simulate: a full circle cut part way is an arc of the elapsed fraction", "[simulate][playback][time]") {
    const Stock stock = stockOf(20, 20, 0.25, 8.0);
    const ToolShape tool = *ToolShape::flatEndMill(1.5);
    Toolpath tp;
    tp.tool.feedRate = 600.0;
    tp.append(Move::rapid({14, 10, -1}));
    tp.append(Move::arcCW({14, 10, -1}, {10, 10}));  // full circle, radius 4
    Simulator sim(stock);
    REQUIRE(sim.addToolpath(tp, tool, 1));
    const double total = sim.totalSeconds();
    REQUIRE(sim.advanceToTime(total * 0.5));  // half of it: 180 degrees clockwise from angle 0
    const Simulator ref = run(stock, tool, {14, 10, -1}, {Move::arcCW({6, 10, -1}, {10, 10})});
    // The only differing cells are those on the very edge of the half-circle's end cap.
    std::size_t differing = 0;
    for (std::size_t k = 0; k < ref.heightfield().heights().size(); ++k) {
        differing += ref.heightfield().heights()[k] != sim.heightfield().heights()[k];
    }
    CHECK(differing <= 3);
}

TEST_CASE("simulate: several toolpaths: positions carry over, ids label cells, from overrides", "[simulate][playback]") {
    const Stock stock = stockOf(20, 12, 0.25, 6.0);
    const ToolShape flat = *ToolShape::flatEndMill(2.0);
    Toolpath a;
    a.append(Move::rapid({3, 6, 5}));
    a.append(Move::plunge({3, 6, -1}));
    a.append(Move::feed({8, 6, -1}));
    a.append(Move::retract({8, 6, 5}));
    Toolpath b;  // starts where `a` ended: the first move needs no rapid
    b.append(Move::feed({8, 3, 5}));  // above the stock: cuts nothing
    b.append(Move::plunge({8, 3, -2}));
    Simulator sim(stock);
    REQUIRE(sim.addToolpath(a, flat, 11));
    REQUIRE(sim.addToolpath(b, flat, 12));
    CHECK(sim.moveCount() == 6);
    REQUIRE(sim.advanceToMove(4));
    CHECK(sim.heightfield().cutCells() > 0);
    std::size_t i = 0, j = 0;
    REQUIRE(sim.heightfield().cellAt({8, 3}, i, j));
    CHECK_FALSE(sim.heightfield().isCut(i, j));
    REQUIRE(sim.advanceAll());
    CHECK(sim.heightfield().isCut(i, j));
    CHECK(sim.heightfield().passId(i, j) == 12);
    REQUIRE(sim.heightfield().cellAt({5, 6}, i, j));
    CHECK(sim.heightfield().passId(i, j) == 11);
    // An explicit start position replaces the carried-over one.
    Simulator s2(stock);
    s2.addToolpath(a, flat, 1);
    Toolpath c;
    c.append(Move::plunge({15, 9, -3}));
    REQUIRE(s2.addToolpath(c, flat, 2, Vec3{15, 9, 5}));
    s2.advanceAll();
    REQUIRE(s2.heightfield().cellAt({15, 9}, i, j));
    CHECK(s2.heightfield().height(i, j) == -3.0f);
    REQUIRE(s2.heightfield().cellAt({12, 7.5}, i, j));  // a diagonal plunge from (8,6,5) would pass here
    CHECK(s2.heightfield().height(i, j) == 0.0f);
    // An empty toolpath queues nothing and is fine.
    CHECK(s2.addToolpath(Toolpath{}, flat, 3));
    CHECK(s2.moveCount() == 5);
    // reset restores the stock and keeps the queue.
    s2.reset();
    CHECK(s2.heightfield().cutCells() == 0);
    CHECK(s2.movesDone() == 0);
    CHECK(s2.moveCount() == 5);
    s2.advanceAll();
    CHECK(s2.heightfield().cutCells() > 0);
}

TEST_CASE("simulate: a cutting first move without a start position stamps the tool", "[simulate][playback]") {
    const Stock stock = stockOf(12, 12, 0.25, 6.0);
    const ToolShape ball = *ToolShape::ballNose(3.0);
    Toolpath a;
    a.append(Move::plunge({6, 6, -2}));
    Simulator sim(stock);
    REQUIRE(sim.addToolpath(a, ball, 1));
    sim.advanceAll();
    std::size_t i = 0, j = 0;
    REQUIRE(sim.heightfield().cellAt({6.1, 6.1}, i, j));
    CHECK(sim.heightfield().height(i, j) < -1.9f);
    Toolpath arc;  // an arc without a known start is a stamp too (not a full circle)
    arc.append(Move::arcCW({6, 6, -2}, {3, 6}));
    Simulator s2(stock);
    REQUIRE(s2.addToolpath(arc, ball, 1));
    s2.advanceAll();
    CHECK(s2.heightfield().cutCells() == sim.heightfield().cutCells());
}
