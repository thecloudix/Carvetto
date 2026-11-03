// T-0141 (b): Profile refuses tolerances outside Pocket's bounds (1e-4 <= chordal <= 1 mm,
// 0 < distance <= chordal) with one InvalidParameters diagnostic and no toolpath (ADR-0016 §18).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/profile.hpp"
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
                   ContainsSubstring("invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 "
                                     "and <= chordal"));
    }
}

TEST_CASE("T-0141 (b): Profile accepts tolerances on Pocket's bounds", "[toolpath][profile][tolerance]") {
    const std::vector<Tolerance> good{
        {1e-4, 1e-9, 1e-4},  // smallest chordal, distance equal to it
        {1e-9, 1e-9, 1e-4},  // tiny distance (no lower bound beyond > 0)
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
