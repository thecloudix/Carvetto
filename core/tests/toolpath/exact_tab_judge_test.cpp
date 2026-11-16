// T-0279: the exact tab judge driven by hand-built fixtures (ADR-0096), one per branch the scene
// corpus cannot reach, so every line of its world is pinned: each mutant of it fails one of these.
//
// The world: two horizontal bands of radius 1 (y = 0, from the window's left side to x = -1.8 and from
// x = 1.8 to its right side) leave a gap in the middle, which joins the half of the window above them
// to the half below. A disc of radius 1 at the origin seals the gap (and so splits the window in two
// pieces); a disc of half the radius leaves it open.

#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "exact_tab_judge.hpp"

using namespace carvetto;
using namespace carvetto::toolpath::detail;
using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;

namespace {

constexpr double kRk = 1.0;

Box2 window() { return Box2({-10.0, -3.0}, {10.0, 3.0}); }

ExactTools twoBands() {
    ExactTools ex;
    ex.cutRuns.push_back({{-10.0, 0.0}, {-5.0, 0.0}, {-1.8, 0.0}});
    ex.cutRuns.push_back({{1.8, 0.0}, {5.0, 0.0}, {10.0, 0.0}});
    return ex;
}

bool holds(const ExactTools& ex) { return exactNoWasteHolds(window(), ex, kRk, geometry::Tolerance{}, 2.0); }

}  // namespace

TEST_CASE("T-0279: the exact judge: the gap between two bands, open and sealed", "[toolpath][profile][tabs]") {
    // Nothing seals the gap: one piece, no waste to tell apart: holds (the old answer).
    CHECK(holds(twoBands()));

    // A lone cut sample's disc of the full radius seals it: two pieces, nothing lowered joins them.
    ExactTools full = twoBands();
    full.cutRuns.push_back({{0.0, 0.0}});
    CHECK_FALSE(holds(full));
    // ... but a lone sample's disc of half the radius leaves it open (one piece: holds). The sample is
    // alone in its run, the case the judge draws as a disc itself.
    ExactTools off = twoBands();
    off.cutRuns.push_back({{0.0, 3.5}});  // far from the gap: touches nothing
    CHECK(holds(off));
}

TEST_CASE("T-0279: the exact judge: what lowering the tab adds", "[toolpath][profile][tabs]") {
    // A corner disc cut with the tab raised seals the gap: two pieces (above, below) before the tab is
    // lowered, and nothing lowered splits them further: severed.
    ExactTools base = twoBands();
    base.cutX.push_back({{0.0, 0.0}, kRk});
    CHECK_FALSE(holds(base));

    // Lowering it adds a corner-move disc across the upper piece: it splits it, so the tab joins pieces.
    ExactTools disc = base;
    disc.ownX.push_back({{-5.0, 2.0}, kRk});
    CHECK(holds(disc));

    // ... or a run of its own samples (a lone one is a disc, two a band).
    ExactTools run = base;
    run.ownRuns.push_back({{5.0, 1.0}, {5.0, 3.0}});
    CHECK(holds(run));
    ExactTools lone = base;
    lone.ownRuns.push_back({{5.0, 2.0}});
    CHECK(holds(lone));
}

TEST_CASE("T-0279: the exact judge: a corner disc's radius is its own", "[toolpath][profile][tabs]") {
    ExactTools big = twoBands();
    big.cutX.push_back({{0.0, 0.0}, kRk});
    CHECK_FALSE(holds(big));
    ExactTools small = twoBands();
    small.cutX.push_back({{0.0, 0.0}, 0.5});  // does not seal the 1.6 wide gap
    CHECK(holds(small));
}

TEST_CASE("T-0279: which tool positions the judge sees", "[toolpath][profile][tabs]") {
    const std::vector<bool> alive{true, true, false};
    // not covered: cut, plain or corner move
    CHECK(classifyTool(false, {}, 0, alive) == ToolSide::Cut);
    CHECK(classifyTool(true, {}, 0, alive) == ToolSide::Cut);
    // covered by this group alone: what lowering it adds
    CHECK(classifyTool(false, {0}, 0, alive) == ToolSide::Own);
    CHECK(classifyTool(true, {0}, 0, alive) == ToolSide::Own);
    // covered by another live tab too: a plain sample stays cut, a corner move is not made
    CHECK(classifyTool(false, {0, 1}, 0, alive) == ToolSide::Cut);
    CHECK(classifyTool(true, {0, 1}, 0, alive) == ToolSide::Skip);
    CHECK(classifyTool(true, {1}, 0, alive) == ToolSide::Skip);
    CHECK(classifyTool(false, {1}, 0, alive) == ToolSide::Cut);
    // a dropped group's cover counts for nothing
    CHECK(classifyTool(true, {0, 2}, 0, alive) == ToolSide::Own);
    CHECK(classifyTool(true, {2}, 0, alive) == ToolSide::Cut);
}
