// T-0182: algorithms that polygonise input arcs terminate promptly on arcs of huge or infinite radius
// (the polygon is capped at kMaxArcSegments chords, ADR-0005 A2), and medialAxis ignores a contour
// with a non-finite or absurd (beyond 1e12 mm) coordinate or radius instead of hanging or crashing.
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "../geometry/cpu_time.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "pocket_test_util.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using carvetto::test::processCpuSeconds;

namespace {

// One full circle of the given radius as a closed, continuous contour.
geometry::Contour bigCircle(double radius) {
    geometry::Contour c;
    c.closed = true;
    geometry::ArcSpan a;
    a.radius = radius;
    a.centre = {0.0, -radius};
    a.startAngle = 0.0;
    a.endAngle = 0.0;
    a.direction = geometry::ArcDirection::CCW;
    c.spans.emplace_back(a);
    return c;
}

}  // namespace

TEST_CASE("profile, pocket and self-intersection terminate on a huge full circle", "[toolpath][polygonise][cap]") {
    for (const double radius : {1e10, 1e12}) {
        INFO("radius " << radius);
        const std::vector<geometry::Contour> cs{bigCircle(radius)};
        const auto& arc = std::get<geometry::ArcSpan>(cs[0].spans[0]);
        // The capped path is really taken for this arc.
        REQUIRE(arc.segmentCountIsCapped(geometry::Tolerance::defaults().chordal));
        const double t0 = processCpuSeconds();

        toolpath::ProfileParams pp;
        pp.side = toolpath::ProfileSide::Outside;
        pp.passes.cutDepth = 1.0;
        pp.passes.passDepth = 1.0;
        const toolpath::ProfileResult pr = toolpath::profileToolpath(cs, test::endMill(3.0), pp);
        CHECK((!pr.toolpath.moves.empty() || !pr.diagnostics.empty()));  // a result or a reason, never silence

        toolpath::PocketParams kp;
        kp.cutDepth = 1.0;
        const toolpath::PocketResult kr =
            toolpath::pocketToolpath(cs, {test::pocket::endMill("em", 1, 3.0, 1.5, 1.0)}, kp);
        CHECK((!kr.toolpaths.empty() || !kr.diagnostics.empty()));

        (void)geometry::selfIntersections(cs[0], geometry::Tolerance::defaults());
        CHECK(processCpuSeconds() - t0 < 30.0);
    }
}

TEST_CASE("medialAxis ignores a contour with a non-finite or absurd arc, promptly", "[medial][polygonise][cap]") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double radius : {1e12, 1e100, 1e300, 1.7e308, inf, nan, -5.0, 0.0, -1e300}) {
        INFO("radius " << radius);
        const double t0 = processCpuSeconds();
        const geometry::MedialAxis m = geometry::medialAxis({bigCircle(radius)}, geometry::Tolerance::defaults());
        CHECK(processCpuSeconds() - t0 < 5.0);
        // Beyond 1e12 mm (or not finite) the contour is ignored and counted.
        if (!(std::fabs(radius) <= 1e12)) {
            CHECK(m.ignoredContours == 1);
            CHECK(m.branches.empty());
            CHECK(m.accuracyCapped == std::isfinite(radius));  // finite but absurd is flagged
        }
    }
    // The same for a huge centre and for an arc inside a mixed region.
    geometry::Contour far = bigCircle(10.0);
    std::get<geometry::ArcSpan>(far.spans[0]).centre = {1e300, 0.0};
    const geometry::MedialAxis m = geometry::medialAxis({far}, geometry::Tolerance::defaults());
    CHECK(m.ignoredContours == 1);
}
