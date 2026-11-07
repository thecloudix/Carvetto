// T-0222 round 2: diagnostics independent of the playback history, arc end-radius validation,
// allocation failure plumbing, threshold and boundary pins.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <vector>

#include "sim_test_util.hpp"

using namespace carvetto;
using namespace carvetto::test::sim;
using simulate::DiagnosticKind;
using simulate::SimStatus;

namespace {

bool sameDiagnostics(const std::vector<simulate::SimDiagnostic>& a, const std::vector<simulate::SimDiagnostic>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].kind != b[i].kind || a[i].move != b[i].move || a[i].toolpathId != b[i].toolpathId ||
            !(a[i].position == b[i].position) || a[i].message != b[i].message) {
            return false;
        }
    }
    return true;
}

std::size_t count(const Simulator& s, DiagnosticKind k) {
    std::size_t n = 0;
    for (const auto& d : s.diagnostics()) n += d.kind == k;
    return n;
}

/// A job that produces every diagnostic kind: a flat 4 mm tool with 5 mm flutes dives (rapid in
/// material), plunges 8 mm into uncut stock (too deep), goes through the table; then another tool
/// starts inside the stock.
Simulator diagJob(const Stock& stock) {
    Simulator sim(stock);
    Toolpath a;
    a.tool.feedRate = 600.0;
    a.tool.plungeRate = 300.0;
    a.tool.toolNumber = 1;
    a.append(Move::rapid({10, 10, 5}));
    a.append(Move::rapid({10, 10, -1}));        // 1: rapid in material
    a.append(Move::retract({10, 10, 5}));
    a.append(Move::plunge({10, 10, -8}));       // 3: too deep (flute 5)
    a.append(Move::feed({25, 10, -8}));         // 4: too deep, feed through uncut stock
    a.append(Move::arcCCW({25, 10, -9}, {25, 15}));  // 5: helix, too deep? (start radius 5, end radius 5)
    a.append(Move::plunge({30, 30, -11}));      // 6: below the table (thickness 10)
    Toolpath b;
    b.tool.feedRate = 600.0;
    b.tool.plungeRate = 300.0;
    b.tool.toolNumber = 2;
    b.append(Move::feed({30, 25, -2}));         // 7: new tool while the old one is inside the stock
    b.append(Move::plunge({30, 25, -6}));
    sim.addToolpath(a, ToolShape::flatEndMill(4.0)->withFluteLength(5.0), 1);
    sim.addToolpath(b, *ToolShape::ballNose(4.0), 2);
    return sim;
}

}  // namespace

TEST_CASE("simulate: diagnostics are the same for every playback order", "[simulate][diagnostics][playback][round2]") {
    const Stock stock = stockOf(40, 40, 0.5, 10.0);
    Simulator oneShot = diagJob(stock);
    REQUIRE(oneShot.advanceAll());
    const auto expected = oneShot.diagnostics();
    for (DiagnosticKind k : {DiagnosticKind::RapidInMaterial, DiagnosticKind::BelowTable, DiagnosticKind::ToolTooDeep,
                             DiagnosticKind::ToolChangeWithoutRetract}) {
        CHECK(count(oneShot, k) >= 1);
    }
    // the reported repro: scrub to 90% of the plunge, then run to the end
    {
        Simulator sim = diagJob(stock);
        const std::vector<double>& e = sim.moveEndTimes();
        for (double f : {0.2, 0.9, 0.999}) {
            sim.reset();
            REQUIRE(sim.advanceToTime(e[2] + f * (e[3] - e[2])));
            REQUIRE(sim.advanceAll());
            CHECK(sameDiagnostics(sim.diagnostics(), expected));
        }
    }
    // random orders of advance and seek calls, ending with the whole job
    for (std::uint64_t seed = 1; seed <= 150; ++seed) {
        Simulator sim = diagJob(stock);
        const double total = sim.totalSeconds();
        const std::size_t n = sim.moveCount();
        Rng rng(seed);
        for (int op = 0; op < 12; ++op) {
            switch (rng.next() % 5) {
                case 0: sim.advanceToTime(rng.uniform() * total); break;
                case 1: sim.advanceToMove(sim.movesDone() + rng.next() % 3); break;
                case 2: sim.seekTime(rng.uniform() * total); break;
                case 3: sim.seekMove(rng.next() % (n + 1)); break;
                default: sim.advanceToTime(sim.currentSeconds() + rng.uniform() * 2.0); break;
            }
        }
        REQUIRE(sim.advanceAll());
        INFO("seed " << seed);
        CHECK(sameDiagnostics(sim.diagnostics(), expected));
        // and a prefix is the prefix of the one-shot list
        Simulator pre = diagJob(stock);
        const std::size_t upto = rng.next() % (n + 1);
        pre.advanceToTime(pre.moveEndTimes()[upto == 0 ? 0 : upto - 1] * 0.5);
        pre.advanceToMove(upto);
        std::vector<simulate::SimDiagnostic> want;
        for (const auto& d : expected) {
            if (d.move < upto) want.push_back(d);
        }
        CHECK(sameDiagnostics(pre.diagnostics(), want));
    }
}

TEST_CASE("simulate: arcs must end on their start radius", "[simulate][hostile][round2]") {
    const Stock stock = stockOf(30, 30, 0.5);
    const ToolShape tool = *ToolShape::flatEndMill(2.0);
    auto queue = [&](double startRadius, double endRadius) {
        Simulator sim(stock);
        Toolpath t;
        t.append(Move::rapid({15 + startRadius, 15, 5}));
        t.append(Move::arcCCW({15, 15 + endRadius, 5}, {15, 15}));
        const bool ok = sim.addToolpath(t, tool, 1);
        if (!ok) {
            CHECK(sim.error().status == SimStatus::InvalidMove);
            CHECK(sim.error().move == std::optional<std::size_t>(1));
            CHECK(sim.error().message.find("end radius") != std::string::npos);
            CHECK(sim.moveCount() == 0);
        }
        return ok;
    };
    CHECK(queue(3.0, 3.0));
    CHECK(queue(3.0, 3.004));    // just inside max(0.005 mm, 1e-6 r)
    CHECK(queue(3.0, 2.996));
    CHECK_FALSE(queue(3.0, 3.006));  // just outside
    CHECK_FALSE(queue(3.0, 2.994));
    CHECK_FALSE(queue(3.0, 10.0));
    // the relative part of the tolerance: radius 100000 allows 0.1 mm
    {
        Simulator sim(stockOf(10, 10, 1.0));
        auto big = [&](double endR) {
            Toolpath t;
            t.append(Move::rapid({100000, 0, 5}));
            t.append(Move::arcCCW({0, endR, 5}, {0, 0}));
            Simulator s2(stockOf(10, 10, 1.0));
            return s2.addToolpath(t, tool, 1);
        };
        CHECK(big(100000.09));
        CHECK_FALSE(big(100000.11));
    }
    // the zero-radius boundary (kMinRadius = 1e-9)
    {
        Simulator sim(stock);
        Toolpath t;
        t.append(Move::rapid({5.0 + 0.5e-9, 5, 5}));
        t.append(Move::arcCW({6, 5, 5}, {5, 5}));
        CHECK_FALSE(sim.addToolpath(t, tool, 1));  // radius 0.5 nm, goes 1 mm: zero radius
        Toolpath u;
        u.append(Move::rapid({5.0 + 2e-9, 5, 5}));
        u.append(Move::arcCW({5, 5.0 + 2e-9, 5}, {5, 5}));
        CHECK(sim.addToolpath(u, tool, 1));  // radius 2 nm is a (tiny) real arc
    }
}

TEST_CASE("simulate: a failed allocation is an error status, never an exception", "[simulate][hostile][round2]") {
    const Stock stock = stockOf(10, 10, 0.5);
    simulate::detail::failNextAllocation(1);
    REQUIRE_NOTHROW(Simulator(stock));
    simulate::detail::failNextAllocation(1);
    Simulator sim(stock);
    CHECK_FALSE(sim.ok());
    CHECK(sim.error().status == SimStatus::OutOfMemory);
    CHECK(sim.heightfield().cellCount() == 0);
    CHECK_FALSE(sim.advanceAll());
    CHECK_FALSE(sim.addToolpath(Toolpath{}, *ToolShape::flatEndMill(2.0), 1));
    CHECK(sim.error().status == SimStatus::OutOfMemory);
    Simulator fine(stock);  // the failure was consumed
    CHECK(fine.ok());

    simulate::detail::failNextAllocation(2);
    Toolpath t;
    t.append(Move::rapid({1, 1, 5}));
    t.append(Move::feed({2, 2, -1}));
    REQUIRE_NOTHROW(fine.addToolpath(t, *ToolShape::flatEndMill(2.0), 1));
    CHECK(fine.moveCount() == 0);
    CHECK(fine.error().status == SimStatus::OutOfMemory);
    CHECK(fine.addToolpath(t, *ToolShape::flatEndMill(2.0), 1));  // retry works, error cleared
    CHECK(fine.ok());
    CHECK(fine.moveCount() == 2);
    CHECK(fine.advanceAll());
}

TEST_CASE("simulate: stock cell cap boundary", "[simulate][stock][round2]") {
    Stock s = stockOf(10, 10, 1.0);
    s.maxCells = 100;
    CHECK(simulate::checkStock(s).ok());
    s.maxCells = 99;
    CHECK(simulate::checkStock(s).issue == simulate::StockIssue::TooManyCells);
    s.maxCells = 100;
    s.width = 10.5;  // 11 x 10 = 110
    CHECK_FALSE(simulate::checkStock(s).ok());
}

TEST_CASE("simulate: the 1 um thresholds of the diagnostics", "[simulate][diagnostics][round2]") {
    const ToolShape flat = *ToolShape::flatEndMill(3.0);
    Stock stock = stockOf(30, 12, 0.25, 8.0);
    // Rapid along a cut floor whose height is not a float: rounding must not flag it ...
    double floorZ = -3.0;
    for (int k = 1; k < 2000 && !(static_cast<double>(static_cast<float>(floorZ)) > floorZ); ++k) {
        floorZ = -3.0 - 0.0001237 * k;
    }
    REQUIRE(static_cast<double>(static_cast<float>(floorZ)) > floorZ);
    auto rapidAt = [&](double z) {
        Toolpath t;
        t.append(Move::rapid({5, 6, 5}));
        t.append(Move::plunge({5, 6, floorZ}));
        t.append(Move::feed({20, 6, floorZ}));
        t.append(Move::rapid({8, 6, z}));
        t.append(Move::rapid({18, 6, z}));
        Simulator sim(stock);
        sim.addToolpath(t, flat, 1);
        sim.advanceAll();
        return count(sim, DiagnosticKind::RapidInMaterial);
    };
    CHECK(rapidAt(floorZ) == 0);            // float rounding of the floor is below 1 um
    CHECK(rapidAt(floorZ + 0.0005) == 0);
    CHECK(rapidAt(floorZ - 0.0005) == 0);   // 0.5 um into the floor: within the tolerance
    CHECK(rapidAt(floorZ - 0.002) >= 1);    // 2 um into the floor: a collision
    CHECK(rapidAt(floorZ + 0.002) == 0);
    // Tool change: the previous end point 0.5 um / 2 um below the top; and a change of shape only.
    auto change = [&](double endZ, bool otherShape) {
        Toolpath a;
        a.tool.toolNumber = 1;
        a.append(Move::rapid({5, 6, 5}));
        a.append(Move::feed({5, 6, endZ}));
        Toolpath b;
        b.tool.toolNumber = 1;
        b.append(Move::rapid({9, 6, 5}));
        Simulator sim(stock);
        sim.addToolpath(a, flat, 1);
        sim.addToolpath(b, otherShape ? *ToolShape::ballNose(3.0) : flat, 2);
        sim.advanceAll();
        return count(sim, DiagnosticKind::ToolChangeWithoutRetract);
    };
    CHECK(change(-0.0005, true) == 0);
    CHECK(change(-0.002, true) == 1);     // differs only by shape (same tool number)
    CHECK(change(-0.002, false) == 0);
    CHECK(change(0.0, true) == 0);
    // Below the table
    auto below = [&](double z) {
        Simulator sim = run(stock, flat, {5, 6, 5}, {Move::plunge({5, 6, z})});
        return count(sim, DiagnosticKind::BelowTable);
    };
    CHECK(below(-8.0005) == 0);
    CHECK(below(-8.002) == 1);
    // Exactly at the flute depth is not too deep (pinned: the flute reaches the surface).
    const ToolShape fl = ToolShape::flatEndMill(3.0)->withFluteLength(4.0);
    CHECK(count(run(stock, fl, {5, 6, 5}, {Move::plunge({5, 6, -4.0})}), DiagnosticKind::ToolTooDeep) == 0);
    CHECK(count(run(stock, fl, {5, 6, 5}, {Move::plunge({5, 6, -4.002})}), DiagnosticKind::ToolTooDeep) == 1);
}

TEST_CASE("simulate: partial playback follows the move in Z and the time boundaries are inclusive", "[simulate][playback][round2]") {
    const Stock stock = stockOf(24, 16, 0.25, 8.0);
    const ToolShape tool = *ToolShape::ballNose(2.0);
    // segment: Z scales with the fraction
    {
        Toolpath tp;
        tp.tool.feedRate = 600.0;   // 10 mm/s
        tp.tool.plungeRate = 300.0; // 5 mm/s
        tp.append(Move::rapid({2, 5, -1}));
        tp.append(Move::feed({12, 5, -3}));  // length ~10.198 mm
        for (double f : {0.1, 0.3, 0.7}) {
            Simulator sim(stock);
            sim.addToolpath(tp, tool, 1);
            REQUIRE(sim.advanceToTime(sim.totalSeconds() * f));
            const Simulator ref = run(stock, tool, {2, 5, -1}, {Move::feed({2 + 10 * f, 5, -1 - 2 * f})});
            CHECK(sim.heightfield().heights() == ref.heightfield().heights());
        }
    }
    // helix: bounded between the states before and after, monotone in the fraction, close to the exact prefix
    {
        Toolpath tp;
        tp.tool.feedRate = 600.0;
        tp.append(Move::rapid({14, 8, -1}));
        tp.append(Move::arcCCW({14, 8, -3}, {10, 8}));  // full helix, 2 mm down
        Simulator full(stock);
        full.addToolpath(tp, tool, 1);
        full.advanceAll();
        for (double f : {0.15, 0.4, 0.65, 0.9}) {
            Simulator sim(stock);
            sim.addToolpath(tp, tool, 1);
            REQUIRE(sim.advanceToTime(sim.totalSeconds() * f));
            const auto& h = sim.heightfield().heights();
            const auto& hf = full.heightfield().heights();
            // The prefix is cut with a different chord subdivision than the whole arc: it can be
            // deeper than the finished cut by the documented partial-playback error (about 0.01 mm here).
            for (std::size_t k = 0; k < h.size(); ++k) {
                CHECK(h[k] >= hf[k] - 0.015f);
                CHECK(h[k] <= 0.0f);
            }
            // exact prefix: arc of fraction f of the circle, linear Z (the planar part is exact)
            const double ang = 2 * kPi * f;
            const Vec3 e{10 + 4 * std::cos(ang), 8 + 4 * std::sin(ang), -1 - 2 * f};
            const Simulator ref = run(stock, tool, {14, 8, -1}, {Move::arcCCW(e, {10, 8})});
            double worst = 0.0;
            for (std::size_t k = 0; k < h.size(); ++k) {
                worst = std::max(worst, static_cast<double>(std::fabs(h[k] - ref.heightfield().heights()[k])));
            }
            CHECK(worst < 0.02);
        }
    }
    // the same sequence on one simulator: monotone non-increasing heights
    {
        Toolpath tp;
        tp.tool.feedRate = 600.0;
        tp.append(Move::rapid({14, 8, -1}));
        tp.append(Move::arcCW({14, 8, -3}, {10, 8}));
        Simulator sim(stock);
        sim.addToolpath(tp, tool, 1);
        std::vector<float> before = sim.heightfield().heights();
        for (double f : {0.1, 0.35, 0.6, 0.85, 1.0}) {
            REQUIRE(sim.advanceToTime(sim.totalSeconds() * f));
            const auto& h = sim.heightfield().heights();
            for (std::size_t k = 0; k < h.size(); ++k) CHECK(h[k] <= before[k]);
            before = h;
        }
    }
    // a move boundary exactly at the requested time completes that move
    {
        Toolpath tp;
        tp.tool.feedRate = 600.0;
        tp.tool.plungeRate = 300.0;
        tp.append(Move::rapid({2, 5, 5}));
        tp.append(Move::plunge({2, 5, 0}));   // 5 mm at 5 mm/s = 1.0 s
        tp.append(Move::feed({12, 5, 0}));    // 10 mm at 10 mm/s = 1.0 s
        Simulator sim(stock);
        sim.addToolpath(tp, tool, 1);
        const auto& e = sim.moveEndTimes();
        REQUIRE(e.size() == 3);
        CHECK(e[1] == 1.0);
        CHECK(e[2] == 2.0);
        REQUIRE(sim.advanceToTime(0.0));
        CHECK(sim.movesDone() == 1);
        REQUIRE(sim.advanceToTime(1.0));
        CHECK(sim.movesDone() == 2);
        CHECK(sim.partialFraction() == 0.0);
        REQUIRE(sim.advanceToTime(2.0));
        CHECK(sim.movesDone() == 3);
    }
    // seekMove to the move being played partially rewinds to the exact state of that index
    {
        Toolpath tp;
        tp.tool.feedRate = 600.0;
        tp.append(Move::rapid({2, 5, -1}));
        tp.append(Move::feed({12, 5, -1}));
        tp.append(Move::feed({12, 12, -2}));
        Simulator sim(stock);
        sim.addToolpath(tp, tool, 1);
        REQUIRE(sim.advanceToTime(sim.moveEndTimes()[1] + 0.3));
        REQUIRE(sim.partialFraction() > 0.0);
        const std::size_t doneNow = sim.movesDone();
        REQUIRE(sim.seekMove(doneNow));
        Simulator fresh(stock);
        fresh.addToolpath(tp, tool, 1);
        fresh.advanceToMove(doneNow);
        CHECK(sim.heightfield().heights() == fresh.heightfield().heights());
        CHECK(sim.partialFraction() == 0.0);
    }
}

TEST_CASE("simulate: arcs count towards the work budget", "[simulate][hostile][round2]") {
    simulate::SimOptions opt;
    opt.maxCellWork = 5000.0;
    Simulator sim(stockOf(100, 100, 0.25), opt);
    Toolpath t;
    t.append(Move::rapid({60, 50, -1}));
    for (int k = 0; k < 50; ++k) {
        t.append(k % 2 == 0 ? Move::arcCW({60, 50, -1}, {50, 50}) : Move::arcCCW({60, 50, -1}, {50, 50}));
    }
    REQUIRE(sim.addToolpath(t, *ToolShape::flatEndMill(8.0), 1));
    CHECK_FALSE(sim.advanceAll());
    CHECK(sim.error().status == SimStatus::WorkBudgetExceeded);
    CHECK(sim.error().move.has_value());
    CHECK(sim.cellWork() > 5000.0);
    CHECK(sim.cellWork() < 5000.0 + 1e6);
}
