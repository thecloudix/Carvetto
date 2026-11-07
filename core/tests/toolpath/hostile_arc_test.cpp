// T-0182: algorithms that polygonise input arcs terminate promptly on arcs of huge or infinite radius
// (the polygon is capped at kMaxArcSegments chords, ADR-0005 A2), and medialAxis ignores a contour
// with a non-finite or absurd (beyond 1e12 mm) coordinate or radius instead of hanging or crashing.
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
#include "pocket_test_util.hpp"
#include "profile_test_util.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/offCentre).
#include "../geometry/fail_fast.hpp"

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
        const geometry::MedialAxis m = carvetto::test::failFast(
            20.0, "medialAxis(huge arc)", [&] { return geometry::medialAxis({bigCircle(radius)}, geometry::Tolerance::defaults()); });
        CHECK(processCpuSeconds() - t0 < 5.0);
        // Beyond 1e12 mm (or not finite) the contour is ignored and counted.
        if (!(std::fabs(radius) <= 1e12)) {
            CHECK(m.ignoredContours == 1);
            CHECK(m.branches.empty());
            CHECK(m.accuracyCapped == std::isfinite(radius));  // finite but absurd is flagged
        }
    }
    // The same for a huge centre and for an arc inside a mixed region.
    geometry::Contour offCentre = bigCircle(10.0);
    std::get<geometry::ArcSpan>(offCentre.spans[0]).centre = {1e300, 0.0};
    const geometry::MedialAxis m = geometry::medialAxis({offCentre}, geometry::Tolerance::defaults());
    CHECK(m.ignoredContours == 1);
}

namespace {

geometry::MedialAxis medialOf(const std::vector<geometry::Contour>& cs) {
    return carvetto::test::failFast(20.0, "medialAxis(boundary)",
                                    [&] { return geometry::medialAxis(cs, geometry::Tolerance::defaults()); });
}

}  // namespace

TEST_CASE("medialAxis magnitude boundary is exactly 1e12 mm", "[medial][polygonise][cap]") {
    using geometry::Box2;
    using geometry::Contour;
    const double lim = 1e12;
    const double over = std::nextafter(lim, std::numeric_limits<double>::infinity());
    SECTION("coordinates exactly at +-1e12 are accepted") {
        const geometry::MedialAxis hi = medialOf({Contour::rectangle(Box2{{lim - 20, lim - 8}, {lim, lim}})});
        CHECK(hi.ignoredContours == 0);
        const geometry::MedialAxis lo = medialOf({Contour::rectangle(Box2{{-lim, -lim}, {-lim + 20, -lim + 8}})});
        CHECK(lo.ignoredContours == 0);
    }
    SECTION("one ulp beyond is ignored and flagged") {
        const geometry::MedialAxis m = medialOf({Contour::rectangle(Box2{{over - 20, over - 8}, {over, over}})});
        CHECK(m.ignoredContours == 1);
        CHECK(m.accuracyCapped);
        CHECK(m.branches.empty());
        const geometry::MedialAxis n = medialOf({Contour::rectangle(Box2{{-over, -over}, {-over + 20, -over + 8}})});
        CHECK(n.ignoredContours == 1);
    }
    SECTION("the radius alone is checked") {
        geometry::Contour in = bigCircle(lim);
        std::get<geometry::ArcSpan>(in.spans[0]).centre = {0.0, 0.0};
        CHECK(medialOf({in}).ignoredContours == 0);
        geometry::Contour out = in;
        std::get<geometry::ArcSpan>(out.spans[0]).radius = over;
        const geometry::MedialAxis m = medialOf({out});
        CHECK(m.ignoredContours == 1);
        CHECK(m.accuracyCapped);
    }
    SECTION("an ignored contour next to a normal one") {
        const geometry::MedialAxis m =
            medialOf({Contour::rectangle(Box2{{0, 0}, {20, 8}}), bigCircle(1e300)});
        CHECK(m.ignoredContours == 1);
        CHECK(m.accuracyCapped);       // set in the final result too, not only on the early-return path
        CHECK_FALSE(m.branches.empty());  // the normal rectangle still has its skeleton
        const geometry::MedialAxis plain = medialOf({Contour::rectangle(Box2{{0, 0}, {20, 8}})});
        CHECK_FALSE(plain.accuracyCapped);
        CHECK(plain.branches.size() == m.branches.size());
    }
}
