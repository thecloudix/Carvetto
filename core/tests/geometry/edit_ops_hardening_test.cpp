// Hardening tests for the geometry edit operations (T-0179, ADR-0028 section 8): the huge-radius
// extend probe, the chordal-versus-coordinate relation, the cost rule and the input caps of every
// entry point, convexNodes' skip rules. Calls that must not hang run under failFast (CPU-time
// limit, assertions on the main thread), so a regression fails in seconds of CPU.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/edit_ops.hpp"
#include "fail_fast.hpp"

using namespace carvetto::geometry;
using carvetto::test::failFast;
using carvetto::test::processCpuSeconds;

namespace {

constexpr double kLimit = 10.0;  // CPU seconds before a call counts as hung (legit calls: milliseconds)
constexpr double kArcLimit = 4.0;  // huge-arc probes: ms when cut, tens of CPU seconds (Debug) when not

Contour openLine(Vec2 a, Vec2 b) {
    Contour c;
    c.append(LineSpan{a, b});
    return c;
}

Contour bezierContour(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3) {
    Contour c;
    c.append(BezierSpan{p0, p1, p2, p3});
    return c;
}

/// A Bezier whose control polygon spans about `big` (an honest drawing no more: refused by cost).
Contour hugeBezier(double big) {
    return bezierContour({0, 0}, {big, big}, {-big, big}, {10, 10});
}

Contour closedWith(Contour c) {
    const Vec2 e = c.endPoint();
    const Vec2 s = c.startPoint();
    c.append(LineSpan{e, s});
    c.closed = true;
    return c;
}

/// Arc end with a huge radius: centre (0, R), bottom of the circle at the origin, CCW, ending 5 mm
/// right of the bottom.
Contour bigArcEnd(double radius) {
    Contour c;
    c.append(ArcSpan{{0.0, radius}, radius, -kPi / 2 - 5.0 / radius, -kPi / 2 + 5.0 / radius, ArcDirection::CCW});
    return c;
}

/// A short, nearly vertical Bezier 30 mm right of the origin.
Contour wall() {
    return bezierContour({30, -1}, {30, -0.3}, {30, 0.3}, {30, 1});
}

}  // namespace

// ----------------------------------------------------------------------------------- (a) probe

TEST_CASE("extend: an arc end with a huge radius is probed in bounded time", "[edit][extend][guard]") {
    for (double radius : {1e3, 1e6, 1e9, 1e11, 1e12}) {
        INFO("radius = " << radius);
        const Contour arc = bigArcEnd(radius);
        const double t0 = processCpuSeconds();
        const ExtendResult r = failFast(kArcLimit, "extendSpan huge arc", [&] {
            return extendSpan(arc, ContourEnd::End, {wall()});
        });
        const double used = processCpuSeconds() - t0;
        REQUIRE(r.status == EditStatus::Ok);
        // The circle keeps heading right along the bottom, so it meets the wall at x = 30, 25 mm
        // of arc past the end of the arc (the circle rises only 450 / R mm over that distance).
        CHECK(std::fabs(r.point.x - 30.0) < 1e-3);
        CHECK(std::fabs(r.distance - 25.0) < 1e-3 + 5e3 / (radius * radius));  // arc vs x: 30^3 / (6 R^2)
        CHECK(r.targetContour == 0);
        CHECK(r.targetSpan == 0);
        const ArcSpan& a = std::get<ArcSpan>(r.contour.spans.back());
        CHECK(a.startAngle == std::get<ArcSpan>(arc.spans.front()).startAngle);
        CHECK(a.direction == ArcDirection::CCW);
        CHECK(r.contour.spans.size() == 1);
        CHECK(std::fabs(a.radius * (a.endAngle - (-kPi / 2)) - 30.0) < 1e-3 + 5e3 / (radius * radius) + radius * 1e-15);
        CHECK(used < 3.0);  // unbounded: 0.2 s at 1e9 and ~7 s at 1e12 in Release
    }
}

TEST_CASE("extend: a clockwise huge arc and a Start end are probed the same way", "[edit][extend][guard]") {
    // Mirror image: centre (0, R), CW, running left along the bottom; the wall is 30 mm to the left.
    const double radius = 1e12;
    Contour arc;
    arc.append(ArcSpan{{0.0, radius}, radius, -kPi / 2 + 5.0 / radius, -kPi / 2 - 5.0 / radius, ArcDirection::CW});
    const Contour leftWall = bezierContour({-30, -1}, {-30, -0.3}, {-30, 0.3}, {-30, 1});
    const double t0 = processCpuSeconds();
    const ExtendResult r = failFast(kArcLimit, "extendSpan huge CW arc",
                                    [&] { return extendSpan(arc, ContourEnd::End, {leftWall}); });
    CHECK(processCpuSeconds() - t0 < 3.0);  // a probe cut the wrong way is the whole circle again
    REQUIRE(r.status == EditStatus::Ok);
    CHECK(std::fabs(r.point.x + 30.0) < 1e-3);
    CHECK(std::fabs(r.distance - 25.0) < 1e-3);
    // The same arc reversed, extended at its Start, lands on the same wall.
    const ExtendResult s = failFast(kArcLimit, "extendSpan huge arc at Start", [&] {
        return extendSpan(arc.reversed(), ContourEnd::Start, {leftWall});
    });
    REQUIRE(s.status == EditStatus::Ok);
    CHECK(std::fabs(s.point.x + 30.0) < 1e-3);
    CHECK(std::fabs(s.distance - 25.0) < 1e-3);
}

TEST_CASE("extend: a huge circle with no target ahead still reports NoIntersection quickly", "[edit][extend][guard]") {
    const Contour arc = bigArcEnd(1e12);
    const Contour behind = bezierContour({-30, -1}, {-30, -0.3}, {-30, 0.3}, {-30, 1});
    const ExtendResult r =
        failFast(kArcLimit, "extendSpan huge arc, no target", [&] { return extendSpan(arc, ContourEnd::End, {behind}); });
    CHECK(r.status == EditStatus::NoIntersection);
}

TEST_CASE("extend: a huge circle is probed only `reach` far; wrap-around hits are NoIntersection", "[edit][extend][guard]") {
    // Documented contract (ADR-0028 section 8): a target behind the tip, or through the arc's own
    // start, could only be reached by travelling almost round a circle of millions of mm. The
    // unbounded probe reported such hits as Ok; the bounded one reports NoIntersection.
    for (double radius : {1e6, 1e9, 1e12}) {
        INFO("radius = " << radius);
        const Contour ccw = bigArcEnd(radius);
        // Wall 30 mm behind the start of the CCW arc (x = -30), and one through the arc's own start.
        const Contour behind = bezierContour({-30, -1}, {-30, -0.3}, {-30, 0.3}, {-30, 1});
        const Contour throughStart = bezierContour({-5, -1}, {-5, -0.3}, {-5, 0.3}, {-5, 1});
        for (const Contour& target : {behind, throughStart}) {
            CHECK(failFast(kArcLimit, "extendSpan wrap-around CCW",
                           [&] { return extendSpan(ccw, ContourEnd::End, {target}).status; }) ==
                  EditStatus::NoIntersection);
        }
        // Clockwise mirror: walls on the other side are behind.
        Contour cw;
        cw.append(ArcSpan{{0.0, radius}, radius, -kPi / 2 + 5.0 / radius, -kPi / 2 - 5.0 / radius, ArcDirection::CW});
        const Contour behindCw = bezierContour({30, -1}, {30, -0.3}, {30, 0.3}, {30, 1});
        CHECK(failFast(kArcLimit, "extendSpan wrap-around CW",
                       [&] { return extendSpan(cw, ContourEnd::End, {behindCw}).status; }) == EditStatus::NoIntersection);
        // The same circle still finds a wall ahead, within the probe, at the analytic point.
        const ExtendResult ahead = failFast(kArcLimit, "extendSpan ahead CCW",
                                            [&] { return extendSpan(ccw, ContourEnd::End, {wall()}); });
        REQUIRE(ahead.status == EditStatus::Ok);
        CHECK(std::fabs(ahead.point.x - 30.0) < 1e-3);
        const ExtendResult aheadCw = failFast(kArcLimit, "extendSpan ahead CW", [&] {
            return extendSpan(cw, ContourEnd::End, {bezierContour({-30, -1}, {-30, -0.3}, {-30, 0.3}, {-30, 1})});
        });
        REQUIRE(aheadCw.status == EditStatus::Ok);
        CHECK(std::fabs(aheadCw.point.x + 30.0) < 1e-3);
    }
}

// -------------------------------------------------------------------------- (b) chordal vs ulps

TEST_CASE("curve edits refuse a chordal tolerance within ulps of far-away coordinates", "[edit][guard]") {
    // Coordinates of 1e11 mm have an ulp of 1.5e-5 mm; a chordal tolerance of 1e-5 mm cannot be
    // reached by subdividing (4 s .. 29 s before the relation check).
    const double o = 1e11;
    const Contour bz = bezierContour({o, o}, {o + 1.0 / 3, o + 2}, {o + 2.0 / 3, o - 1}, {o + 1, o});
    const Contour bz2 = bezierContour({o, o + 0.5}, {o + 1.0 / 3, o - 1}, {o + 2.0 / 3, o + 2}, {o + 1, o + 0.5});
    const Contour sq = Contour::rectangle(Box2{{o, o}, {o + 1, o + 1}});
    const Contour line = openLine({o - 1, o + 0.25}, {o + 2, o + 0.25});
    for (double chordal : {1e-5, 3e-5}) {
        INFO("chordal = " << chordal);
        const Tolerance t{0.01, 1e-9, chordal};
        CHECK(failFast(kLimit, "trimSpansAt ulp", [&] { return trimSpansAt(bz, {o + 0.5, o}, {bz2}, {true}, t).status; }) ==
              EditStatus::InvalidArgument);
        CHECK(failFast(kLimit, "trimToBoundary ulp",
                       [&] { return trimToBoundary({bz, line}, {sq}, TrimSide::ClearInside, false, t).status; }) ==
              EditStatus::InvalidArgument);
        CHECK(failFast(kLimit, "extendSpan ulp", [&] { return extendSpan(line, ContourEnd::End, {bz2}, t).status; }) ==
              EditStatus::InvalidArgument);
    }
    // A few ulps more is reachable and quick: accepted, computed, within a small CPU budget.
    const Tolerance ok{0.01, 1e-9, 2e-4};
    const double t0 = processCpuSeconds();
    const TrimAtResult r =
        failFast(kLimit, "trimSpansAt ulp ok", [&] { return trimSpansAt(bz, {o + 0.5, o}, {bz2}, {true}, ok); });
    CHECK(r.status != EditStatus::InvalidArgument);
    CHECK(processCpuSeconds() - t0 < 3.0);
    // The relation only concerns curves: lines and arcs alone are not refused at tiny tolerances.
    const Tolerance tiny{0.01, 1e-9, 1e-12};
    CHECK(trimSpansAt(line, {o + 0.5, o + 0.25}, {Contour::rectangle(Box2{{o + 0.4, o}, {o + 0.6, o + 1}})}, {}, tiny)
              .status != EditStatus::InvalidArgument);
}

TEST_CASE("the chordal-versus-ulp threshold is 4 eps x the largest coordinate magnitude", "[edit][guard]") {
    // Placements with the large coordinate in x or y, positive or negative, the other near 0: a
    // "largest coordinate" taken from one component or one sign only gets one of these wrong.
    const double o = 1e11;
    const double eps = std::numeric_limits<double>::epsilon();
    const Vec2 places[] = {{0.0, o}, {o, 0.0}, {-o, 0.0}, {0.0, -o}, {-o, o}, {o, -o}};
    for (const Vec2 at : places) {
        const Contour bz = bezierContour(at, at + Vec2{1.0 / 3, 2}, at + Vec2{2.0 / 3, -1}, at + Vec2{1, 0});
        const Contour bz2 = bezierContour(at + Vec2{0, 0.5}, at + Vec2{1.0 / 3, -1}, at + Vec2{2.0 / 3, 2}, at + Vec2{1, 0.5});
        double largest = 0.0;
        for (const Contour* c : {&bz, &bz2}) {
            const BezierSpan& b = std::get<BezierSpan>(c->spans.front());
            for (const Vec2 p : {b.p0, b.p1, b.p2, b.p3}) {
                largest = std::max({largest, std::fabs(p.x), std::fabs(p.y)});
            }
        }
        for (const double k : {3.9, 4.1}) {
            INFO("place = " << at.x << "," << at.y << " k = " << k);
            const Tolerance t{0.01, 1e-9, k * eps * largest};
            const EditStatus st = failFast(kLimit, "trimSpansAt threshold",
                                           [&] { return trimSpansAt(bz, at + Vec2{0.5, 0}, {bz2}, {true}, t).status; });
            if (k < 4.0) {
                CHECK(st == EditStatus::InvalidArgument);
            } else {
                CHECK(st != EditStatus::InvalidArgument);
            }
        }
    }
}

TEST_CASE("the cost rule's extent ratio is 1e6 x tol.chordal", "[edit][guard]") {
    // A nearly straight Bezier (few chords) along x, cut by a vertical line; chordal tolerance 1.
    const Tolerance t{0.001, 1e-9, 1.0};
    auto along = [](double length) {
        return bezierContour({0, 0}, {length / 3, 0.1}, {2 * length / 3, -0.1}, {length, 0});
    };
    const Contour cut = openLine({0.4, -1}, {0.4, 1});
    // Under the ratio: accepted and computed (the pick is on the Bezier, the vertical line crosses it).
    const TrimAtResult ok = failFast(kLimit, "trimSpansAt ratio ok",
                                     [&] { return trimSpansAt(along(5e5), {10, 0}, {cut}, {}, t); });
    CHECK(ok.status != EditStatus::InvalidArgument);
    // Over the ratio, up to 1e8 and beyond: refused.
    for (double length : {2e6, 5e6, 5e7, 5e8}) {
        INFO("length = " << length);
        CHECK(failFast(kLimit, "trimSpansAt ratio refused",
                       [&] { return trimSpansAt(along(length), {10, 0}, {cut}, {}, t).status; }) ==
              EditStatus::InvalidArgument);
    }
}

// ------------------------------------------------------------------------- (c) cost rule, wide

TEST_CASE("every curve edit refuses a Bezier drawing far larger than its chordal tolerance", "[edit][guard]") {
    // Within the magnitude cap, far beyond 1e6 x chordal: refused before any intersection work.
    const Contour ln = openLine({-5, 5}, {15, 5});
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {10, 10}});
    for (double big : {1e9, 9e11}) {
        INFO("big = " << big);
        const Contour bz = hugeBezier(big);
        const Contour closedBz = closedWith(bz);
        const EditStatus refused = EditStatus::InvalidArgument;
        CHECK(failFast(kLimit, "trimSpansAt curve", [&] { return trimSpansAt(bz, {1, 1}, {ln}).status; }) == refused);
        CHECK(failFast(kLimit, "trimSpansAt other", [&] { return trimSpansAt(ln, {1, 5}, {bz}).status; }) == refused);
        CHECK(failFast(kLimit, "trimToBoundary curve",
                       [&] { return trimToBoundary({bz}, {sq}, TrimSide::ClearInside).status; }) == refused);
        CHECK(failFast(kLimit, "trimToBoundary boundary",
                       [&] { return trimToBoundary({ln}, {closedBz}, TrimSide::ClearOutside).status; }) == refused);
        CHECK(failFast(kLimit, "extendSpan target",
                       [&] { return extendSpan(openLine({-30, 0}, {-25, 0}), ContourEnd::End, {bz}).status; }) ==
              refused);
        CHECK(failFast(kLimit, "extendSpan contour",
                       [&] { return extendSpan(bz, ContourEnd::End, {ln, sq}).status; }) == refused);
        CHECK(failFast(kLimit, "extendSpan arc end", [&] {
                  Contour arc;
                  arc.append(ArcSpan{{0, -50}, 20, 0.5, 1.5, ArcDirection::CCW});
                  return extendSpan(arc, ContourEnd::End, {bz}).status;
              }) == refused);
    }
}

TEST_CASE("the cost rule is about Beziers: huge lines and arcs alone are processed", "[edit][guard]") {
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {1e9, 1e9}});
    const Contour ln = openLine({-5e8, 5e8}, {2e9, 5e8});
    const TrimResult t = failFast(kLimit, "trimToBoundary lines",
                                  [&] { return trimToBoundary({ln}, {sq}, TrimSide::ClearOutside); });
    CHECK(t.status == EditStatus::Ok);
    CHECK(t.contours.size() == 1);
    const ExtendResult e = failFast(kLimit, "extendSpan lines", [&] {
        return extendSpan(openLine({-3e8, 1e8}, {-2e8, 1e8}), ContourEnd::End, {sq});
    });
    CHECK(e.status == EditStatus::Ok);
    CHECK(std::fabs(e.point.x) < 1e-3);
}

// ------------------------------------------------------------------------------ (c) input caps

TEST_CASE("arc radius caps: over the magnitude cap or negative arcs are refused everywhere", "[edit][guard]") {
    const Contour line = openLine({0, 0}, {10, 0});
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {10, 10}});
    // Every end point and centre lies within the cap, only the radius is over it (or negative).
    struct ArcCase {
        double radius;
        Vec2 centre;
    };
    for (const ArcCase ac : {ArcCase{1.5e12, {1e12, 0.0}}, ArcCase{-1.0, {0.0, 0.0}}}) {
        const double radius = ac.radius;
        INFO("radius = " << radius);
        Contour arc;
        arc.append(ArcSpan{ac.centre, radius, kPi - 1e-9, kPi + 1e-9, ArcDirection::CCW});
        arc.append(LineSpan{arc.endPoint(), {5, 5}});
        Contour closedArc = arc;
        closedArc.append(LineSpan{{5, 5}, arc.startPoint()});
        closedArc.closed = true;
        const EditStatus refused = EditStatus::InvalidArgument;
        CHECK(failFast(kLimit, "trimSpansAt", [&] { return trimSpansAt(arc, {1, 1}, {line}).status; }) == refused);
        CHECK(failFast(kLimit, "trimSpansAt other", [&] { return trimSpansAt(line, {1, 0}, {arc}).status; }) == refused);
        CHECK(failFast(kLimit, "trimToBoundary curve",
                       [&] { return trimToBoundary({arc}, {sq}, TrimSide::ClearInside).status; }) == refused);
        CHECK(failFast(kLimit, "trimToBoundary boundary",
                       [&] { return trimToBoundary({line}, {closedArc}, TrimSide::ClearInside).status; }) == refused);
        CHECK(failFast(kLimit, "extendSpan", [&] { return extendSpan(arc, ContourEnd::End, {line}).status; }) == refused);
        CHECK(failFast(kLimit, "extendSpan target",
                       [&] { return extendSpan(openLine({-5, 0}, {-1, 0}), ContourEnd::End, {arc}).status; }) ==
              refused);
        CHECK(failFast(kLimit, "filletAll", [&] { return filletAll(closedArc, 1.0, FilletKind::Normal).status; }) ==
              refused);
        CHECK(failFast(kLimit, "filletCorner",
                       [&] { return filletCorner(closedArc, 1, 1.0, FilletKind::Normal).status; }) == refused);
        CHECK(failFast(kLimit, "closeContour", [&] { return closeContour(arc).status; }) == refused);
        CHECK(failFast(kLimit, "convexNodes", [&] { return convexNodes(closedArc).size(); }) == 0);
        // joinContours: the unusable contour passes through unjoined, status Ok.
        const JoinResult j = failFast(kLimit, "joinContours", [&] { return joinContours({arc, line}); });
        CHECK(j.status == EditStatus::Ok);
        CHECK(j.contours.size() == 2);
        CHECK(j.unmatched.size() == 1);  // only the line is a joinable contour, without a partner
    }
    // The cap itself is inclusive: a 1e12 radius arc is a valid argument.
    Contour edge;
    edge.append(ArcSpan{{0, 0}, 1e12, 0.0, 1e-12, ArcDirection::CCW});
    CHECK(failFast(kLimit, "closeContour at the cap", [&] { return closeContour(edge).status; }) == EditStatus::Ok);
}

TEST_CASE("tolerance caps: tol.distance and tol.chordal above 1e12 are refused everywhere", "[edit][guard]") {
    const Contour line = openLine({0, 0}, {10, 0});
    const Contour cross = openLine({5, -5}, {5, 5});
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {10, 10}});
    const Contour tri = Contour::rectangle(Box2{{0, 0}, {10, 10}});
    const Tolerance bad[] = {
        Tolerance{2e12, 1e-9, 0.01},
        Tolerance{0.001, 1e-9, 2e12},
    };
    for (const Tolerance& t : bad) {
        INFO("distance = " << t.distance << " chordal = " << t.chordal);
        const EditStatus refused = EditStatus::InvalidArgument;
        CHECK(failFast(kLimit, "trimSpansAt", [&] { return trimSpansAt(line, {1, 0}, {cross}, {}, t).status; }) == refused);
        CHECK(failFast(kLimit, "trimToBoundary", [&] {
                  return trimToBoundary({line}, {sq}, TrimSide::ClearInside, false, t).status;
              }) == refused);
        CHECK(failFast(kLimit, "extendSpan", [&] {
                  return extendSpan(openLine({-5, 5}, {-1, 5}), ContourEnd::End, {sq}, t).status;
              }) == refused);
        CHECK(failFast(kLimit, "filletAll", [&] { return filletAll(tri, 1.0, FilletKind::Normal, {}, t).status; }) ==
              refused);
        CHECK(failFast(kLimit, "filletCorner",
                       [&] { return filletCorner(tri, 1, 1.0, FilletKind::Normal, {}, t).status; }) == refused);
        CHECK(failFast(kLimit, "closeContour", [&] { return closeContour(line, t).status; }) == refused);
        CHECK(failFast(kLimit, "joinContours", [&] { return joinContours({line, cross}, t).status; }) == refused);
        CHECK(failFast(kLimit, "convexNodes", [&] { return convexNodes(sq, t).size(); }) == 0);
    }
}

TEST_CASE("extend and interactive trim refuse unusable target / other contours at once", "[edit][guard]") {
    const Contour line = openLine({0, 0}, {10, 0});
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // Lines only (no Bezier, so the cost rule cannot catch them): non-finite or over the cap.
    for (const Contour& bad : {openLine({5, -1}, {5, 1e300}), openLine({5, -1}, {5, 2e12}), openLine({nan, -1}, {5, 1}),
                               openLine({5, -inf}, {5, 1})}) {
        const EditStatus refused = EditStatus::InvalidArgument;
        CHECK(failFast(kLimit, "extendSpan target", [&] {
                  return extendSpan(openLine({-5, 0}, {-1, 0}), ContourEnd::End, {Contour::rectangle(Box2{{0, 0}, {1, 1}}), bad})
                      .status;
              }) == refused);
        CHECK(failFast(kLimit, "trimSpansAt other",
                       [&] { return trimSpansAt(line, {1, 0}, {openLine({3, -1}, {3, 1}), bad}).status; }) == refused);
    }
}

// ------------------------------------------------------------------------------- convexNodes

TEST_CASE("convexNodes skips gaps, cusps, tangent-continuous nodes and non-corner open ends", "[edit][fillet][dogbone]") {
    using Nodes = std::vector<std::size_t>;
    SECTION("a gap larger than the tolerance at a convex node leaves it out") {
        Contour c;
        c.closed = true;
        c.append(LineSpan{{0, 0}, {10, 0}});
        c.append(LineSpan{{10, 1}, {10, 10}});  // 1 mm gap at node 1
        c.append(LineSpan{{10, 10}, {0, 10}});
        c.append(LineSpan{{0, 10}, {0, 0}});
        CHECK(convexNodes(c) == Nodes{0, 2, 3});
        // Within the tolerance the same node is a corner again.
        c.spans[1] = LineSpan{{10, 1e-6}, {10, 10}};
        CHECK(convexNodes(c) == Nodes{0, 1, 2, 3});
    }
    SECTION("a cusp (turn within 1e-6 of pi) is not a relief corner") {
        // A needle: A(0,0) -> B(100,0) -> C(0,1e-5); the turn at B is pi - 1e-7.
        Contour c;
        c.closed = true;
        c.append(LineSpan{{0, 0}, {100, 0}});
        c.append(LineSpan{{100, 0}, {0, 1e-5}});
        c.append(LineSpan{{0, 1e-5}, {0, 0}});
        CHECK(convexNodes(c) == Nodes{0, 2});
    }
    SECTION("a turn under 1e-6 is tangent-continuous") {
        Contour c;
        c.closed = true;
        c.append(LineSpan{{0, 0}, {10, 0}});
        c.append(LineSpan{{10, 0}, {20, 5e-6}});  // turn 5e-7 at node 1
        c.append(LineSpan{{20, 5e-6}, {20, 10}});
        c.append(LineSpan{{20, 10}, {0, 10}});
        c.append(LineSpan{{0, 10}, {0, 0}});
        CHECK(convexNodes(c) == Nodes{0, 2, 3, 4});
    }
    SECTION("an open contour has no corner at its ends even when they coincide") {
        Contour c;  // three sides of a counter-clockwise square, the end back on the start
        c.append(LineSpan{{0, 0}, {10, 0}});
        c.append(LineSpan{{10, 0}, {10, 10}});
        c.append(LineSpan{{10, 10}, {0, 10}});
        c.append(LineSpan{{0, 10}, {0, 0}});
        c.closed = false;
        CHECK(convexNodes(c) == Nodes{1, 2, 3});
    }
    SECTION("a closed contour of zero area has no inside") {
        Contour c;  // bow tie: signed area 0
        c.closed = true;
        c.append(LineSpan{{0, 0}, {10, 10}});
        c.append(LineSpan{{10, 10}, {10, 0}});
        c.append(LineSpan{{10, 0}, {0, 10}});
        c.append(LineSpan{{0, 10}, {0, 0}});
        REQUIRE(std::fabs(signedArea(c)) < 1e-9);
        CHECK(convexNodes(c).empty());
    }
}
