#include "carvetto/toolpath/time.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>

using namespace carvetto::toolpath;
using Catch::Matchers::WithinAbs;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

// One straight 1000 mm feed move starting at the origin.
Toolpath thousandMillimetres() {
    Toolpath tp;
    tp.append(Move::rapid({0, 0, 0}));
    tp.append(Move::feed({1000, 0, 0}));
    return tp;
}
}  // namespace

TEST_CASE("AC4: 1000 mm at 1000 mm/min", "[toolpath][time]") {
    const Toolpath tp = thousandMillimetres();

    SECTION("infinite acceleration is exactly 60 s") {
        const TimeEstimate t = estimateTime(tp, 3000.0, 1000.0);
        CHECK_THAT(t.feed, WithinAbs(60.0, 1e-12));
        CHECK(t.rapid == 0.0);  // the first rapid has no known start, so zero length
        CHECK_THAT(t.total(), WithinAbs(60.0, 1e-12));
    }
    SECTION("finite acceleration is strictly greater") {
        const TimeEstimate t = estimateTime(tp, 3000.0, 1000.0, 500.0);
        CHECK(t.total() > 60.0);
        // v = 16.667 mm/s, a = 500 mm/s²: ramp distance v²/a = 0.556 mm ≪ 1000, so the trapezoid
        // formula applies: d/v + v/a.
        CHECK_THAT(t.total(), WithinAbs(60.0 + (1000.0 / 60.0) / 500.0, 1e-9));
    }
    SECTION("lower acceleration takes longer") {
        const double a1 = estimateTime(tp, 3000.0, 1000.0, 1000.0).total();
        const double a2 = estimateTime(tp, 3000.0, 1000.0, 100.0).total();
        const double a3 = estimateTime(tp, 3000.0, 1000.0, 10.0).total();
        CHECK(a1 < a2);
        CHECK(a2 < a3);
    }
}

TEST_CASE("moveTime trapezoid and triangle regimes", "[toolpath][time]") {
    CHECK(moveTime(0.0, 1000.0, 100.0) == 0.0);
    CHECK(moveTime(-1.0, 1000.0, 100.0) == 0.0);
    CHECK(std::isinf(moveTime(10.0, 0.0, 100.0)));
    CHECK(std::isinf(moveTime(10.0, -5.0, kInf)));
    CHECK_THAT(moveTime(100.0, 600.0, kInf), WithinAbs(10.0, 1e-12));
    // Zero or negative acceleration is treated as "no acceleration model".
    CHECK_THAT(moveTime(100.0, 600.0, 0.0), WithinAbs(10.0, 1e-12));
    CHECK_THAT(moveTime(100.0, 600.0, -1.0), WithinAbs(10.0, 1e-12));

    // v = 10 mm/s, a = 10 mm/s²: ramp distance 10 mm.
    SECTION("exactly at the ramp distance both formulas agree") {
        CHECK_THAT(moveTime(10.0, 600.0, 10.0), WithinAbs(2.0, 1e-12));
        CHECK_THAT(2.0 * std::sqrt(10.0 / 10.0), WithinAbs(2.0, 1e-12));
    }
    SECTION("short move never reaches cruise: triangular profile") {
        CHECK_THAT(moveTime(2.5, 600.0, 10.0), WithinAbs(2.0 * std::sqrt(0.25), 1e-12));
        CHECK(moveTime(2.5, 600.0, 10.0) > 2.5 / 10.0);
    }
    SECTION("long move: cruise plus one full ramp time") {
        CHECK_THAT(moveTime(100.0, 600.0, 10.0), WithinAbs(10.0 + 1.0, 1e-12));
    }
    SECTION("time is monotonic in distance across the regime boundary") {
        double prev = 0.0;
        for (double d = 0.0; d <= 30.0; d += 0.5) {
            const double t = moveTime(d, 600.0, 10.0);
            CHECK(t >= prev);
            prev = t;
        }
    }
}

TEST_CASE("Estimate is split by kind and honours rates, overrides and dwells", "[toolpath][time]") {
    Toolpath tp;
    tp.append(Move::rapid({0, 0, 10}));                // zero length (no start)
    tp.append(Move::rapid({100, 0, 10}));              // 100 mm rapid
    tp.append(Move::plunge({100, 0, 0}));              // 10 mm plunge
    tp.append(Move::feed({100, 60, 0}));               // 60 mm feed
    tp.append(Move::feed({100, 90, 0}, 600.0));        // 30 mm at override 600 mm/min = 3 s
    tp.append(Move::arcCCW({100, 90, 0}, {100, 95}));  // full circle r=5: 10π mm at feed
    tp.append(Move::dwell({100, 90, 0}, 2.5));
    tp.append(Move::retract({100, 90, 10}));  // 10 mm retract at plunge rate

    const MachineRates rates{6000.0, 1200.0, 300.0, kInf};
    const TimeEstimate t = estimateTime(tp, rates);
    CHECK_THAT(t.rapid, WithinAbs(1.0, 1e-12));
    CHECK_THAT(t.plunge, WithinAbs(2.0, 1e-12));
    CHECK_THAT(t.feed, WithinAbs(3.0 + 3.0 + 10.0 * carvetto::geometry::kPi / 20.0, 1e-12));
    CHECK_THAT(t.dwell, WithinAbs(2.5, 1e-12));
    CHECK_THAT(t.retract, WithinAbs(2.0, 1e-12));
    CHECK_THAT(t.total(), WithinAbs(t.rapid + t.plunge + t.feed + t.dwell + t.retract, 1e-12));

    SECTION("explicit start position adds to the first rapid") {
        const TimeEstimate t2 = estimateTime(tp, rates, Vec3{0, 0, 110});
        CHECK_THAT(t2.rapid, WithinAbs(2.0, 1e-12));
    }
    SECTION("negative dwell counts as zero") {
        Toolpath d;
        d.append(Move::dwell({0, 0, 0}, -3.0));
        CHECK(estimateTime(d, rates).dwell == 0.0);
    }
    SECTION("feed override is ignored for rapids") {
        Toolpath r;
        r.append(Move::rapid({0, 0, 0}));
        Move m = Move::rapid({600, 0, 0});
        m.feedOverride = 60.0;
        r.append(m);
        CHECK_THAT(estimateTime(r, rates).rapid, WithinAbs(6.0, 1e-12));
    }
    SECTION("the card form uses feed for plunge and retract too") {
        const TimeEstimate c = estimateTime(tp, 6000.0, 1200.0);
        CHECK_THAT(c.plunge, WithinAbs(0.5, 1e-12));
        CHECK_THAT(c.retract, WithinAbs(0.5, 1e-12));
    }
}

TEST_CASE("MachineRates::fromTool takes feed and plunge from the snapshot", "[toolpath][time]") {
    ToolSnapshot tool;
    tool.feedRate = 1500.0;
    tool.plungeRate = 400.0;
    const MachineRates r = MachineRates::fromTool(tool, 5000.0, 800.0);
    CHECK(r == MachineRates{5000.0, 1500.0, 400.0, 800.0});
    CHECK(std::isinf(MachineRates::fromTool(tool, 5000.0).acceleration));
}

TEST_CASE("Empty toolpath and zero-length moves take no time", "[toolpath][time]") {
    CHECK(estimateTime(Toolpath{}, 3000.0, 1000.0, 100.0).total() == 0.0);
    Toolpath tp;
    tp.append(Move::rapid({1, 1, 1}));
    tp.append(Move::feed({1, 1, 1}));
    tp.append(Move::plunge({1, 1, 1}));
    CHECK(estimateTime(tp, 3000.0, 1000.0, 100.0).total() == 0.0);
}

TEST_CASE("Estimate is deterministic", "[toolpath][time]") {
    const Toolpath tp = thousandMillimetres();
    const TimeEstimate a = estimateTime(tp, 3000.0, 1000.0, 250.0);
    const TimeEstimate b = estimateTime(tp, 3000.0, 1000.0, 250.0);
    CHECK(a == b);
}
