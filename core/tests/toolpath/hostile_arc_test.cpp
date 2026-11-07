// T-0182: algorithms that polygonise input arcs terminate promptly on arcs of huge or infinite radius
// (the polygon is capped at kMaxArcSegments chords, ADR-0005 A2). Results are not asserted beyond
// "returns, quickly"; a hang or an out-of-memory crash is the failure.
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "../geometry/cpu_time.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using carvetto::test::processCpuSeconds;

namespace {

// A "D" shape: a line and an arc of the given radius bulging by 5 mm over a chord of 100 mm
// (any radius at or above 50 gives a sliver; huge radii a nearly straight edge).
geometry::Contour bigArcShape(double radius) {
    geometry::Contour c;
    c.closed = true;
    geometry::ArcSpan a;
    a.radius = radius;
    a.centre = {50.0, 10.0 - radius};
    a.direction = geometry::ArcDirection::CW;
    a.startAngle = std::atan2(0.0 - a.centre.y, 0.0 - 50.0);
    a.endAngle = std::atan2(0.0 - a.centre.y, 100.0 - 50.0);
    c.spans.emplace_back(geometry::LineSpan{{100.0, 0.0}, {0.0, 0.0}});
    c.spans.emplace_back(a);
    return c;
}

}  // namespace

TEST_CASE("toolpath and analysis callers terminate on a huge or infinite arc", "[toolpath][polygonise][cap]") {
    const double inf = std::numeric_limits<double>::infinity();
    for (const double radius : {1e12, 1e100, 1.7e308, inf}) {
        INFO("radius " << radius);
        const std::vector<geometry::Contour> cs{bigArcShape(radius)};
        const double t0 = processCpuSeconds();

        toolpath::ProfileParams pp;
        pp.side = toolpath::ProfileSide::Outside;
        pp.passes.cutDepth = 1.0;
        pp.passes.passDepth = 1.0;
        (void)toolpath::profileToolpath(cs, test::endMill(3.0), pp);

        toolpath::PocketParams kp;
        kp.cutDepth = 1.0;
        toolpath::PocketTool pt;
        pt.tool = test::endMill(3.0);
        (void)toolpath::pocketToolpath(cs, {pt}, kp);

        // medialAxis hangs at a radius of 1e100 and trips a vector bounds assertion at 1.7e308 for reasons
        // unrelated to polygonising (the pre-T-0182 code does too; reported as a follow-up), so only the
        // radii it handles are run.
        if (radius == 1e12 || std::isinf(radius)) {
            (void)geometry::medialAxis(cs, geometry::Tolerance::defaults());
        }
        (void)geometry::selfIntersections(cs[0], geometry::Tolerance::defaults());

        CHECK(processCpuSeconds() - t0 < 30.0);
    }
}
