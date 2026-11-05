// T-0141 (b): Profile refuses tolerances outside Pocket's bounds (1e-4 <= chordal <= 1 mm,
// 0 < distance <= chordal) with one InvalidParameters diagnostic and no toolpath (ADR-0016 §18).
// T-0142 (d): both refuse a distance tolerance below 1e-6 mm, one unit of the offset grid.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "pocket_test_util.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Tolerance;
using Catch::Matchers::ContainsSubstring;

namespace {

ProfileParams params(const Tolerance& tol) {
    ProfileParams p;
    p.side = ProfileSide::Outside;
    p.passes.cutDepth = 3.0;
    p.passes.passDepth = 1.5;
    p.safeZ = 5.0;
    p.tolerance = tol;
    return p;
}

std::vector<Contour> scene() {
    return {Contour::rectangle(Box2{{0, 0}, {40, 30}}), Contour::circle({80, 15}, 12)};
}

}  // namespace

TEST_CASE("T-0141 (b): Profile refuses a tolerance outside Pocket's bounds", "[toolpath][profile][tolerance]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<Tolerance> bad{
        {0.001, 1e-9, 0.0},     // chordal zero
        {5e-5, 1e-9, 9.9e-5},   // chordal just below 1e-4
        {0.001, 1e-9, 1.0001},  // chordal above 1 mm
        {0.001, 1e-9, nan},     {0.001, 1e-9, inf},  {0.0, 1e-9, 0.01},  // distance zero
        {-0.001, 1e-9, 0.01},   {nan, 1e-9, 0.01},   {inf, 1e-9, 0.01},
        {0.02, 1e-9, 0.01},  // distance above chordal
        {1e-9, 1e-9, 1e-4},  // distance below 1e-6 mm (T-0142)
        {9.99e-7, 1e-9, 1e-4},
        {1e-300, 1e-9, 0.01},
    };
    for (const Tolerance& tol : bad) {
        CAPTURE(tol.distance, tol.chordal);
        const ProfileResult r = profileToolpath(scene(), test::endMill(6.0), params(tol));
        CHECK_FALSE(r.ok());
        CHECK(r.toolpath.isEmpty());
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics.front().issue == ProfileIssue::InvalidParameters);
        CHECK_FALSE(r.diagnostics.front().contour.has_value());
        CHECK_THAT(r.diagnostics.front().message,
                   ContainsSubstring("invalid tolerance: chordal must be between 0.0001 and 1 mm, distance "
                                     "between 0.000001 mm and chordal"));
    }
}

TEST_CASE("T-0141 (b): Profile accepts tolerances on Pocket's bounds", "[toolpath][profile][tolerance]") {
    const std::vector<Tolerance> good{
        {1e-4, 1e-9, 1e-4},  // smallest chordal, distance equal to it
        {1e-6, 1e-9, 1e-4},  // smallest distance (T-0142)
        {1.0, 1e-9, 1.0},    // largest chordal, distance equal to it
        Tolerance::defaults(),
    };
    for (const Tolerance& tol : good) {
        CAPTURE(tol.distance, tol.chordal);
        const ProfileResult r = profileToolpath(scene(), test::endMill(6.0), params(tol));
        CHECK(r.ok());
        CHECK_FALSE(r.toolpath.isEmpty());
        CHECK_FALSE(r.has(ProfileIssue::InvalidParameters));
    }
}

namespace {

// The T-0141 coder's Bezier shape (harness t0141v1/bez.cpp): {distance 1e-8, chordal 1e-4} ran
// for minutes before T-0142 (Bezier spans were polygonised at distance / 2).
Contour bezierShape() {
    Contour c;
    c.closed = true;
    c.spans.emplace_back(geometry::BezierSpan{{0, 0}, {10, -5}, {20, 5}, {30, 0}});
    c.spans.emplace_back(geometry::LineSpan{{30, 0}, {30, 20}});
    c.spans.emplace_back(geometry::BezierSpan{{30, 20}, {20, 25}, {10, 15}, {0, 20}});
    c.spans.emplace_back(geometry::LineSpan{{0, 20}, {0, 0}});
    return c;
}

}  // namespace

TEST_CASE("T-0142 (d): Profile and Pocket refuse a distance tolerance below 1e-6 mm alike",
          "[toolpath][profile][pocket][tolerance]") {
    const std::string reason =
        "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance between 0.000001 mm and chordal";
    for (const double distance : {1e-8, 9.99e-7, 1e-6, 1e-5}) {
        CAPTURE(distance);
        const Tolerance tol{distance, 1e-9, 1e-4};
        const bool valid = distance >= 1e-6;
        const ProfileResult pr = profileToolpath({bezierShape()}, test::endMill(6.0), params(tol));
        CHECK(pr.ok() == valid);
        CHECK(pr.has(ProfileIssue::InvalidParameters) == !valid);
        toolpath::PocketParams pp;
        pp.cutDepth = 3.0;
        pp.tolerance = tol;
        const PocketResult kr =
            pocketToolpath({bezierShape()}, {test::pocket::endMill("EM 6", 1, 6.0, 2.4, 3.0)}, pp);
        CHECK(kr.ok() == valid);
        CHECK(kr.has(ToolpathIssue::InvalidParameters) == !valid);
        if (!valid) {
            REQUIRE(pr.diagnostics.size() == 1);
            CHECK_THAT(pr.diagnostics.front().message, ContainsSubstring(reason));
            REQUIRE(kr.diagnostics.size() == 1);
            CHECK_THAT(kr.diagnostics.front().message, ContainsSubstring(reason));
        }
    }
}
