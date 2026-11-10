// T-0184: huge arcs reach the clearance sampler and the offset with bounded cost and memory.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <vector>

#include "../geometry/cpu_time.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/clearance.hpp"
#include "carvetto/toolpath/path_walk.hpp"

using namespace carvetto;
using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::Contour;
using geometry::JoinType;
using geometry::Tolerance;

namespace {

Contour circleArc(double radius, bool closed) {
    Contour c;
    c.closed = closed;
    c.append(ArcSpan{{0.0, -radius}, radius, 0.0, 2.0 * std::numbers::pi, ArcDirection::CCW});
    return c;
}

}  // namespace

TEST_CASE("findViolations bounds its samples on a huge arc", "[toolpath][clearance][huge]") {
    // A circle of radius 1e9 mm (6.3e9 mm round) passing through the origin, an obstacle 5 mm away.
    const Tolerance tol = Tolerance::defaults();
    const toolpath::ArcLengthPath walk(circleArc(1e9, true), tol);
    const toolpath::ClearanceField field({Contour::circle({0.0, 5.0}, 1.0)});
    const double t0 = test::processCpuSeconds();
    const auto bad = toolpath::findViolations(walk, 0.0, walk.length(), field, 10.0, 1e-3);
    CHECK(test::processCpuSeconds() - t0 < 60.0);
    CHECK_FALSE(bad.empty());  // the stretch round the origin is within 10 mm of the obstacle
}

TEST_CASE("findViolations keeps the 1 mm grid within the cap", "[toolpath][clearance][huge]") {
    // 20 mm line, obstacle circle of radius 1 at (10, 2): violations at threshold 1.5, as before.
    Contour line;
    line.append(geometry::LineSpan{{0.0, 0.0}, {20.0, 0.0}});
    const toolpath::ArcLengthPath walk(line, Tolerance::defaults());
    const toolpath::ClearanceField field({Contour::circle({10.0, 2.0}, 1.0)});
    const auto bad = toolpath::findViolations(walk, 0.0, 20.0, field, 1.5, 1e-4);
    REQUIRE(bad.size() == 1);
    CHECK(bad[0].first == Catch::Approx(8.5).margin(2e-4));
    CHECK(bad[0].second == Catch::Approx(11.5).margin(2e-4));
}

TEST_CASE("offsetContours refuses an outline that would need hundreds of MB", "[geometry][offset][huge]") {
    const Tolerance tol = Tolerance::defaults();
    // Open circle of radius 1e9 mm: 700,000 vertices, 505 MB and 12 s before the bound.
    CHECK(geometry::offsetContours({circleArc(1e9, false)}, 3.0, JoinType::Round, tol).empty());
    // Radius 1e7 (70,000 vertices) is well inside the bound and is offset as before.
    const auto ok = geometry::offsetContours({circleArc(1e7, false)}, 3.0, JoinType::Round, tol);
    CHECK_FALSE(ok.empty());
    // The refused contour is only left out: an ordinary one beside it is still offset.
    const auto mixed = geometry::offsetContours(
        {circleArc(1e9, false), Contour::circle({0.0, 0.0}, 10.0)}, 3.0, JoinType::Round, tol);
    REQUIRE(mixed.size() == 1);
    CHECK(mixed[0].bounds().width() == Catch::Approx(26.0).margin(0.05));
}
