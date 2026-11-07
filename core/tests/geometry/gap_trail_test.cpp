// T-0178: the touch-settling bookkeeping (gap_trail.hpp) has a unique, deterministic order
// whatever the standard library does with equal keys, and the near-coincident Bezier pair that
// exposed the tie-breaks (corpus seed 87 of the T-0178 verifier) keeps its hit count.
#include <catch2/catch_test_macros.hpp>
#include <cstddef>

#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/span.hpp"
#include "gap_trail.hpp"

using namespace carvetto::geometry;
using detail::GapMinima;
using detail::GapSample;
using detail::GapTrail;

TEST_CASE("GapTrail neighbours: lower bound, latest recorded among equal t", "[gaptrail][geometry]") {
    GapTrail trail;
    REQUIRE_FALSE(trail.around(0.5).has_value());
    trail.record({0.25, 0.0, 1.0});
    REQUIRE_FALSE(trail.around(0.5).has_value());  // nothing to the right yet
    trail.record({0.75, 0.0, 3.0});
    trail.record({0.5, 0.1, 2.0});
    trail.record({0.5, 0.2, 2.5});  // equal t, recorded later: it comes first
    const auto mid = trail.around(0.5);
    REQUIRE(mid.has_value());
    REQUIRE(mid->first.t == 0.25);
    REQUIRE(mid->second.t == 0.5);
    REQUIRE(mid->second.tB == 0.2);  // the later of the two samples at t = 0.5
    const auto between = trail.around(0.6);
    REQUIRE(between->first.t == 0.5);
    REQUIRE(between->first.tB == 0.1);  // the earlier of the equal pair is the one next to the left
    REQUIRE(between->second.t == 0.75);
    REQUIRE_FALSE(trail.around(0.25).has_value());  // t at the first sample: no left neighbour
    REQUIRE_FALSE(trail.around(0.9).has_value());   // beyond the last
    const auto atRight = trail.around(0.75);
    REQUIRE(atRight->first.tB == 0.1);
    REQUIRE(atRight->second.t == 0.75);
    REQUIRE(trail.size() == 4);
}

TEST_CASE("GapTrail with many equal t keeps newest-first order", "[gaptrail][geometry]") {
    GapTrail trail;
    trail.record({0.0, -1.0, 0.0});
    trail.record({1.0, -2.0, 0.0});
    for (int i = 0; i < 10; ++i) {
        trail.record({0.5, static_cast<double>(i), 0.0});
    }
    REQUIRE(trail.around(0.5)->second.tB == 9.0);
    REQUIRE(trail.around(0.5000001)->first.tB == 0.0);
}

TEST_CASE("GapMinima range is open at both ends, lowest gap wins, earliest among equals", "[gaptrail][geometry]") {
    GapMinima minima;
    REQUIRE(minima.lowestIn(0.0, 1.0, 10.0) == nullptr);
    minima.push_back({0.2, 0.0, 5.0});  // index 0
    minima.push_back({0.4, 1.0, 3.0});  // index 1
    minima.push_back({0.4, 2.0, 3.0});  // index 2: same t and gap as index 1
    minima.push_back({0.6, 3.0, 3.0});  // index 3: same gap, later t
    minima.push_back({0.8, 4.0, 1.0});  // index 4
    // Ends excluded: 0.2 and 0.8 are not inside (0.2, 0.8).
    const GapSample* inside = minima.lowestIn(0.2, 0.8, 10.0);
    REQUIRE(inside != nullptr);
    REQUIRE(inside->tB == 1.0);  // gap 3 at t = 0.4, the earliest accepted of the equal ones
    // Closed ends would find index 4 (gap 1) or index 0.
    REQUIRE(minima.lowestIn(0.19, 0.8, 4.0)->tB == 1.0);  // index 0 has gap 5 > 4
    REQUIRE(minima.lowestIn(0.2, 0.81, 10.0)->tB == 4.0);
    REQUIRE(minima.lowestIn(0.1, 0.3, 10.0)->tB == 0.0);
    REQUIRE(minima.lowestIn(0.2, 0.3, 10.0) == nullptr);  // lower end excluded
    REQUIRE(minima.lowestIn(0.1, 0.2, 10.0) == nullptr);  // upper end excluded
    // The gap limit is inclusive.
    REQUIRE(minima.lowestIn(0.3, 0.5, 3.0) != nullptr);
    REQUIRE(minima.lowestIn(0.3, 0.5, 2.999) == nullptr);
    // Equal t and equal gap: the first accepted, however many follow.
    REQUIRE(minima.lowestIn(0.39, 0.41, 3.0)->tB == 1.0);
    REQUIRE(minima.size() == 5);
}

TEST_CASE("Near-coincident Beziers (verifier corpus seed 87) keep their hit count", "[gaptrail][intersect][geometry]") {
    const BezierSpan a{{1.1695751753846242, -0.88604445838327339},
                       {-0.99922524477883568, 0.35850538370472607},
                       {-1.0609980289416854, -0.66167352476561769},
                       {0.90321582389781074, -0.37316672813838381}};
    // The same curve reversed: every point is a touch, and the settling walks reuse each other's
    // trail and minima; changing a tie-break there moves the count from 26 to 25.
    const auto r = intersect(Span{a}, Span{a.reversed()}, Tolerance::defaults());
    REQUIRE(r.points.size() == 26);
    const auto swapped = intersect(Span{a.reversed()}, Span{a}, Tolerance::defaults());
    REQUIRE(swapped.points.size() == 26);
}
