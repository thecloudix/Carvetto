#include "carvetto/toolpath/pass_plan.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <vector>

using namespace carvetto::toolpath;
using Catch::Matchers::WithinAbs;

namespace {
void checkLevels(const std::vector<double>& got, const std::vector<double>& want, double tol = 1e-9) {
    REQUIRE(got.size() == want.size());
    for (std::size_t i = 0; i < got.size(); ++i) {
        CHECK_THAT(got[i], WithinAbs(want[i], tol));
    }
}
}  // namespace

TEST_CASE("AC1: 0 -> 12 mm by 5 mm", "[toolpath][passplan]") {
    PassPlan plan;
    plan.startDepth = 0.0;
    plan.cutDepth = 12.0;
    plan.passDepth = 5.0;

    SECTION("exact step keeps 5 mm passes and a 2 mm remainder") {
        plan.maintainExactStep = true;
        checkLevels(plan.levels(), {-5.0, -10.0, -12.0});
        CHECK(plan.passCount() == 3);
    }
    SECTION("equalised gives three 4 mm passes") {
        plan.maintainExactStep = false;
        checkLevels(plan.levels(), {-4.0, -8.0, -12.0});
    }
    CHECK(plan.bottomZ() == -12.0);
}

TEST_CASE("Equalised passes may grow the step by up to 15 percent", "[toolpath][passplan]") {
    PassPlan plan;
    plan.cutDepth = 11.0;
    plan.passDepth = 5.0;
    plan.maintainExactStep = false;
    // 3 passes of 3.67 would be the ceiling count; 2 passes of 5.5 is within +15 % so it wins.
    checkLevels(plan.levels(), {-5.5, -11.0});

    plan.cutDepth = 11.5;  // 2 passes would be 5.75 = +15 % exactly: allowed.
    checkLevels(plan.levels(), {-5.75, -11.5});

    plan.cutDepth = 11.6;  // 2 passes would be 5.8 = +16 %: not allowed, fall back to 3 equal passes.
    checkLevels(plan.levels(), {-11.6 / 3.0, -2.0 * 11.6 / 3.0, -11.6});
}

TEST_CASE("Start depth offsets every level", "[toolpath][passplan]") {
    PassPlan plan;
    plan.startDepth = 2.0;
    plan.cutDepth = 6.0;
    plan.passDepth = 4.0;
    checkLevels(plan.levels(), {-6.0, -8.0});
    plan.maintainExactStep = false;
    checkLevels(plan.levels(), {-5.0, -8.0});
    CHECK(plan.bottomZ() == -8.0);
}

TEST_CASE("Pass depth greater than or equal to total depth gives a single pass", "[toolpath][passplan]") {
    PassPlan plan;
    plan.cutDepth = 3.0;
    plan.passDepth = 5.0;
    checkLevels(plan.levels(), {-3.0});
    plan.maintainExactStep = false;
    checkLevels(plan.levels(), {-3.0});

    plan.passDepth = 3.0;  // exactly equal
    checkLevels(plan.levels(), {-3.0});
    plan.maintainExactStep = true;
    checkLevels(plan.levels(), {-3.0});
}

TEST_CASE("Exact multiples do not produce a zero-length final pass", "[toolpath][passplan]") {
    PassPlan plan;
    plan.cutDepth = 12.0;
    plan.passDepth = 4.0;
    checkLevels(plan.levels(), {-4.0, -8.0, -12.0});
    plan.maintainExactStep = false;
    checkLevels(plan.levels(), {-4.0, -8.0, -12.0});

    // Depths that are not exactly representable still count correctly.
    plan.cutDepth = 0.3;
    plan.passDepth = 0.1;
    plan.maintainExactStep = true;
    checkLevels(plan.levels(), {-0.1, -0.2, -0.3});
}

TEST_CASE("Degenerate inputs", "[toolpath][passplan]") {
    PassPlan plan;
    SECTION("no cut depth gives no passes") {
        plan.cutDepth = 0.0;
        plan.passDepth = 1.0;
        CHECK(plan.levels().empty());
        plan.cutDepth = -2.0;
        CHECK(plan.levels().empty());
        CHECK(plan.passCount() == 0);
    }
    SECTION("non-positive pass depth gives a single pass to the bottom") {
        plan.cutDepth = 7.0;
        plan.passDepth = 0.0;
        checkLevels(plan.levels(), {-7.0});
        plan.passDepth = -1.0;
        checkLevels(plan.levels(), {-7.0});
    }
}

TEST_CASE("Last pass thickness reserves the final pass", "[toolpath][passplan]") {
    PassPlan plan;
    plan.cutDepth = 12.0;
    plan.passDepth = 5.0;
    plan.lastPassThickness = 1.0;

    SECTION("exact step above the last pass") {
        checkLevels(plan.levels(), {-5.0, -10.0, -11.0, -12.0});
    }
    SECTION("equalised above the last pass") {
        plan.maintainExactStep = false;
        // 11 mm by 5 → two passes of 5.5 (within 15 %), then the 1 mm finishing pass.
        checkLevels(plan.levels(), {-5.5, -11.0, -12.0});
    }
    SECTION("last pass thickness at least the cut depth gives one pass") {
        plan.lastPassThickness = 12.0;
        checkLevels(plan.levels(), {-12.0});
        plan.lastPassThickness = 20.0;
        checkLevels(plan.levels(), {-12.0});
    }
    SECTION("last pass thickness with a start depth") {
        plan.startDepth = 1.0;
        plan.cutDepth = 4.0;
        plan.passDepth = 10.0;
        plan.lastPassThickness = 0.5;
        checkLevels(plan.levels(), {-4.5, -5.0});
    }
}

TEST_CASE("Levels are strictly decreasing and end at the bottom", "[toolpath][passplan]") {
    for (double cut : {0.7, 3.0, 7.9, 12.0, 25.3}) {
        for (double pass : {0.3, 1.0, 2.5, 4.0}) {
            for (bool exact : {true, false}) {
                PassPlan plan;
                plan.cutDepth = cut;
                plan.passDepth = pass;
                plan.maintainExactStep = exact;
                const auto lv = plan.levels();
                REQUIRE_FALSE(lv.empty());
                CHECK(lv.back() == plan.bottomZ());
                for (std::size_t i = 1; i < lv.size(); ++i) {
                    CHECK(lv[i] < lv[i - 1]);
                    // No step exceeds the nominal pass depth by more than the 15 % slack.
                    CHECK(lv[i - 1] - lv[i] <= pass * (1.0 + PassPlan::kStepSlack) + 1e-9);
                }
                CHECK(-lv.front() <= pass * (1.0 + PassPlan::kStepSlack) + 1e-9);
            }
        }
    }
}
