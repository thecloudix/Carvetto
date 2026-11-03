// T-0152 AC1: medial axis of regions with holes, sharp and rounded corners, thin necks, arcs and
// Beziers, checked against brute-force oracles (medial_axis_oracle.hpp) that share no code with
// the implementation beyond the span closest-point functions:
// - radius at every output point = its distance to the boundary, within tol.chordal;
// - every output point is equidistant (within tol.chordal) from two boundary features;
// - the radius interpolated linearly along every segment stays within tol.chordal;
// - every point lies inside the region; the topology is consistent;
// - for shapes with a known axis, the output and the analytic axis are close both ways.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "medial_axis_oracle.hpp"

using namespace carvetto::geometry;
namespace orc = carvetto::test::medial;

namespace {

const Tolerance kTol{};  // distance 0.001, chordal 0.01

struct Stats {
    std::size_t points{0};
    std::size_t notMedial{0};
    double maxRadiusError{0.0};
    double maxInterpError{0.0};
};

/// Checks every AC1 property; `maxChecks` bounds the (expensive) equidistance checks.
Stats checkAxis(const std::vector<Contour>& contours, const MedialAxis& ax, const Tolerance& tol,
                std::size_t maxChecks = 400) {
    Stats st;
    REQUIRE_FALSE(ax.accuracyCapped);
    const double c = tol.chordal;
    // Topology.
    for (std::size_t b = 0; b < ax.branches.size(); ++b) {
        const MedialBranch& br = ax.branches[b];
        REQUIRE(br.points.size() >= 2);
        REQUIRE(br.sides.size() == br.points.size() - 1);
        REQUIRE(br.startNode < ax.nodes.size());
        REQUIRE(br.endNode < ax.nodes.size());
        CHECK(br.points.front().point == ax.nodes[br.startNode].point);
        CHECK(br.points.back().point == ax.nodes[br.endNode].point);
        const auto& sb = ax.nodes[br.startNode].branches;
        const auto& eb = ax.nodes[br.endNode].branches;
        CHECK(std::find(sb.begin(), sb.end(), b) != sb.end());
        CHECK(std::find(eb.begin(), eb.end(), b) != eb.end());
        for (const auto& side : br.sides) {
            for (const MedialBoundaryRef& r : side) {
                REQUIRE(r.contour < contours.size());
                REQUIRE(r.span < contours[r.contour].spans.size());
                CHECK(r.t >= 0.0);
                CHECK(r.t <= 1.0);
            }
        }
    }
    for (const MedialNode& n : ax.nodes) {
        if (n.branches.size() == 2) {
            // Only loop anchors have degree two: one branch listed twice.
            CHECK(n.branches[0] == n.branches[1]);
        }
    }
    const std::vector<MedialPoint> pts = orc::allPoints(ax);
    st.points = pts.size();
    const orc::DenseBoundary dense = orc::sampleBoundary(contours, c);
    const std::size_t stride = std::max<std::size_t>(1, pts.size() / maxChecks);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const MedialPoint& mp = pts[i];
        const double d = orc::bruteDistance(contours, mp.point);
        st.maxRadiusError = std::max(st.maxRadiusError, std::fabs(d - mp.radius));
        CHECK(isInside(contours, mp.point, FillRule::EvenOdd, tol));
        if (i % stride == 0 && !orc::isMedial(dense, mp.point, mp.radius, c)) {
            ++st.notMedial;
            UNSCOPED_INFO("not medial: (" << mp.point.x << ", " << mp.point.y << ") r=" << mp.radius);
        }
    }
    for (const MedialBranch& br : ax.branches) {
        for (std::size_t i = 0; i + 1 < br.points.size(); ++i) {
            for (double f : {0.25, 0.5, 0.75}) {
                const Vec2 p = lerp(br.points[i].point, br.points[i + 1].point, f);
                const double interp = br.points[i].radius + f * (br.points[i + 1].radius - br.points[i].radius);
                st.maxInterpError = std::max(st.maxInterpError, std::fabs(orc::bruteDistance(contours, p) - interp));
            }
        }
    }
    CHECK(st.maxRadiusError <= c);
    CHECK(st.maxInterpError <= c);
    CHECK(st.notMedial == 0);
    return st;
}

/// Samples of an analytic axis given as segments: every sample with radius >= minRadius must be
/// within `tolerance` of the computed axis, and every computed point within `tolerance` of the
/// analytic one.
void compareWithAnalytic(const MedialAxis& ax, const std::vector<std::array<MedialPoint, 2>>& segs, double tolerance,
                         double minRadius) {
    double worstMissing = 0.0;
    for (const auto& s : segs) {
        for (int k = 0; k <= 200; ++k) {
            const double f = k / 200.0;
            const Vec2 p = lerp(s[0].point, s[1].point, f);
            const double r = s[0].radius + f * (s[1].radius - s[0].radius);
            if (r < minRadius) {
                continue;
            }
            worstMissing = std::max(worstMissing, orc::distanceToAxis(ax, p));
        }
    }
    double worstExtra = 0.0;
    for (const MedialPoint& mp : orc::allPoints(ax)) {
        double best = 1e300;
        for (const auto& s : segs) {
            best = std::min(best, closestPoint(LineSpan{s[0].point, s[1].point}, mp.point).distance);
        }
        worstExtra = std::max(worstExtra, best);
    }
    INFO("missing " << worstMissing << " extra " << worstExtra);
    CHECK(worstMissing <= tolerance);
    CHECK(worstExtra <= tolerance);
}

Contour roundedRect(Box2 b, double r) {
    Contour c;
    c.closed = true;
    const double x0 = b.min.x;
    const double y0 = b.min.y;
    const double x1 = b.max.x;
    const double y1 = b.max.y;
    c.append(LineSpan{{x0 + r, y0}, {x1 - r, y0}});
    c.append(ArcSpan{{x1 - r, y1 - (y1 - y0) + r}, r, -kPi / 2, 0.0, ArcDirection::CCW});
    c.append(LineSpan{{x1, y0 + r}, {x1, y1 - r}});
    c.append(ArcSpan{{x1 - r, y1 - r}, r, 0.0, kPi / 2, ArcDirection::CCW});
    c.append(LineSpan{{x1 - r, y1}, {x0 + r, y1}});
    c.append(ArcSpan{{x0 + r, y1 - r}, r, kPi / 2, kPi, ArcDirection::CCW});
    c.append(LineSpan{{x0, y1 - r}, {x0, y0 + r}});
    c.append(ArcSpan{{x0 + r, y0 + r}, r, kPi, 1.5 * kPi, ArcDirection::CCW});
    return c;
}

Contour polygon(std::vector<Vec2> pts) {
    return Contour::fromPolyline(Polyline{std::move(pts), true});
}

/// Ellipse from four cubic Beziers (standard quarter construction).
Contour bezierEllipse(Vec2 c, double a, double b) {
    const double k = 0.5522847498;
    Contour e;
    e.closed = true;
    e.append(BezierSpan{{c.x + a, c.y}, {c.x + a, c.y + k * b}, {c.x + k * a, c.y + b}, {c.x, c.y + b}});
    e.append(BezierSpan{{c.x, c.y + b}, {c.x - k * a, c.y + b}, {c.x - a, c.y + k * b}, {c.x - a, c.y}});
    e.append(BezierSpan{{c.x - a, c.y}, {c.x - a, c.y - k * b}, {c.x - k * a, c.y - b}, {c.x, c.y - b}});
    e.append(BezierSpan{{c.x, c.y - b}, {c.x + k * a, c.y - b}, {c.x + a, c.y - k * b}, {c.x + a, c.y}});
    return e;
}

}  // namespace

TEST_CASE("medial axis: rectangle with sharp corners matches the analytic axis", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {40, 20}})};
    const MedialAxis ax = medialAxis(cs, kTol);
    const Stats st = checkAxis(cs, ax, kTol);
    INFO("points " << st.points << " samples " << ax.boundarySamples);
    // Four corner branches meet the central segment in two junctions of degree 3.
    std::size_t junctions = 0;
    std::size_t ends = 0;
    for (const MedialNode& n : ax.nodes) {
        if (n.branches.size() == 3) {
            ++junctions;
        }
        if (n.branches.size() == 1) {
            ++ends;
        }
    }
    CHECK(junctions == 2);
    CHECK(ends == 4);
    CHECK(ax.branches.size() == 5);
    compareWithAnalytic(ax,
                        {{MedialPoint{{10, 10}, 10}, MedialPoint{{30, 10}, 10}},
                         {MedialPoint{{0, 0}, 0}, MedialPoint{{10, 10}, 10}},
                         {MedialPoint{{0, 20}, 0}, MedialPoint{{10, 10}, 10}},
                         {MedialPoint{{40, 0}, 0}, MedialPoint{{30, 10}, 10}},
                         {MedialPoint{{40, 20}, 0}, MedialPoint{{30, 10}, 10}}},
                        kTol.chordal, kTol.chordal);
    // On the central segment the radius is half the height, less the half-chordal slack of the
    // equidistance guarantee (samples on the two walls need not face each other).
    for (const MedialBranch& br : ax.branches) {
        for (const MedialPoint& p : br.points) {
            if (p.point.x > 10.5 && p.point.x < 29.5) {
                CHECK(std::fabs(p.radius - 10.0) <= 0.5 * kTol.chordal);
            }
        }
    }
}

TEST_CASE("medial axis: circle is one point at the centre", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{Contour::circle({3, -2}, 5.0)};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
    for (const MedialPoint& p : orc::allPoints(ax)) {
        CHECK(distance(p.point, {3, -2}) <= kTol.chordal);
        CHECK(std::fabs(p.radius - 5.0) <= kTol.chordal);
    }
    REQUIRE_FALSE(ax.nodes.empty());
}

TEST_CASE("medial axis: rounded rectangle (lines and arcs)", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{roundedRect(Box2{{0, 0}, {30, 12}}, 3.0)};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
    // Corner branches end at the arc centres (radius 3), not in the corners.
    compareWithAnalytic(ax,
                        {{MedialPoint{{6, 6}, 6}, MedialPoint{{24, 6}, 6}},
                         {MedialPoint{{3, 3}, 3}, MedialPoint{{6, 6}, 6}},
                         {MedialPoint{{3, 9}, 3}, MedialPoint{{6, 6}, 6}},
                         {MedialPoint{{27, 3}, 3}, MedialPoint{{24, 6}, 6}},
                         {MedialPoint{{27, 9}, 3}, MedialPoint{{24, 6}, 6}}},
                        kTol.chordal, 0.0);
}

TEST_CASE("medial axis: L shape with a reflex corner", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{polygon({{0, 0}, {20, 0}, {20, 6}, {6, 6}, {6, 20}, {0, 20}})};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
}

TEST_CASE("medial axis: square with a square hole and a circular hole", "[geometry][medial][ac1]") {
    std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {30, 20}}), Contour::rectangle(Box2{{4, 4}, {12, 16}}),
                            Contour::circle({21, 10}, 4.0)};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
    // Two holes make the skeleton contain cycles: more edges than a tree on its nodes.
    std::size_t degreeSum = 0;
    for (const MedialNode& n : ax.nodes) {
        degreeSum += n.branches.size();
    }
    CHECK(ax.branches.size() >= ax.nodes.size() + 1);
    CHECK(degreeSum == 2 * ax.branches.size());
    // Hole orientation does not matter (even-odd).
    cs[1].reverse();
    const MedialAxis ax2 = medialAxis(cs, kTol);
    CHECK(ax2.branches.size() == ax.branches.size());
}

TEST_CASE("medial axis: annulus is one closed loop", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{Contour::circle({0, 0}, 10.0), Contour::circle({0, 0}, 6.0, ArcDirection::CW)};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
    for (const MedialPoint& p : orc::allPoints(ax)) {
        CHECK(std::fabs(p.point.length() - 8.0) <= kTol.chordal);
        CHECK(std::fabs(p.radius - 2.0) <= kTol.chordal);
    }
}

TEST_CASE("medial axis: thin neck between two lobes", "[geometry][medial][ac1]") {
    // Two 10 mm squares joined by a 0.2 mm wide, 8 mm long channel.
    const std::vector<Contour> cs{polygon({{0, 0}, {10, 0}, {10, 4.9}, {18, 4.9}, {18, 0}, {28, 0}, {28, 10},
                                           {18, 10}, {18, 5.1}, {10, 5.1}, {10, 10}, {0, 10}})};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
    // The axis runs through the channel at radius 0.1.
    for (double x : {12.0, 14.0, 16.0}) {
        CHECK(orc::distanceToAxis(ax, {x, 5.0}) <= kTol.chordal);
    }
}

TEST_CASE("medial axis: Bezier ellipse", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{bezierEllipse({5, 5}, 12.0, 5.0)};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
    // The axis of an ellipse lies on the major axis, between the centres of curvature of the
    // vertices (x = +-(a - b^2/a)).
    const double e = 12.0 - 25.0 / 12.0;
    for (const MedialPoint& p : orc::allPoints(ax)) {
        CHECK(std::fabs(p.point.y - 5.0) <= 0.05);
        CHECK(std::fabs(p.point.x - 5.0) <= e + 0.05);
    }
}

TEST_CASE("medial axis: five-pointed star with sharp corners", "[geometry][medial][ac1]") {
    std::vector<Vec2> pts;
    for (int k = 0; k < 10; ++k) {
        const double r = k % 2 == 0 ? 10.0 : 3.8;
        const double a = kPi / 2 + k * kPi / 5;
        pts.push_back({r * std::cos(a), r * std::sin(a)});
    }
    const std::vector<Contour> cs{polygon(pts)};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
}

TEST_CASE("medial axis: glyph-like shape mixing lines, arcs and Beziers with a hole", "[geometry][medial][ac1]") {
    // A "D": straight stem, Bezier bowl, and an inner counter of a line and an arc.
    Contour outer;
    outer.closed = true;
    outer.append(LineSpan{{0, 0}, {6, 0}});
    outer.append(BezierSpan{{6, 0}, {13, 0}, {13, 14}, {6, 14}});
    outer.append(LineSpan{{6, 14}, {0, 14}});
    outer.append(LineSpan{{0, 14}, {0, 0}});
    Contour counter;
    counter.closed = true;
    counter.append(LineSpan{{2.5, 11.5}, {2.5, 2.5}});
    counter.append(LineSpan{{2.5, 2.5}, {6, 2.5}});
    counter.append(ArcSpan{{6, 7}, 4.5, -kPi / 2, kPi / 2, ArcDirection::CCW});
    counter.append(LineSpan{{6, 11.5}, {2.5, 11.5}});
    counter.reverse();
    const std::vector<Contour> cs{outer, counter};
    const MedialAxis ax = medialAxis(cs, kTol);
    checkAxis(cs, ax, kTol);
}

TEST_CASE("medial axis: coarser and finer chordal tolerances", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{bezierEllipse({0, 0}, 8.0, 3.0), Contour::circle({-3, 0}, 1.0)};
    for (double c : {0.05, 0.002}) {
        Tolerance tol;
        tol.chordal = c;
        const MedialAxis ax = medialAxis(cs, tol);
        INFO("chordal " << c);
        checkAxis(cs, ax, tol, 150);
    }
}

TEST_CASE("medial axis: ignored contours and empty input", "[geometry][medial][ac1]") {
    CHECK(medialAxis({}, kTol).nodes.empty());
    Contour open;
    open.append(LineSpan{{0, 0}, {5, 0}});
    open.append(LineSpan{{5, 0}, {5, 5}});
    const MedialAxis ax = medialAxis({open, Contour::rectangle(Box2{{0, 0}, {4, 2}})}, kTol);
    CHECK(ax.ignoredContours == 1);
    CHECK_FALSE(ax.branches.empty());
    // Boundary references point at the input index of the rectangle.
    for (const MedialBranch& br : ax.branches) {
        for (const auto& side : br.sides) {
            CHECK(side[0].contour == 1);
            CHECK(side[1].contour == 1);
        }
    }
}

TEST_CASE("medial axis: non-zero fill rule", "[geometry][medial][ac1]") {
    // Two nested CCW squares: under NonZero the inner one does not bound anything.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {20, 20}}), Contour::rectangle(Box2{{5, 5}, {15, 15}})};
    MedialAxisOptions opt;
    opt.fillRule = FillRule::NonZero;
    const MedialAxis nz = medialAxis(cs, kTol, opt);
    CHECK(nz.ignoredContours == 1);
    const MedialAxis eo = medialAxis(cs, kTol);
    CHECK(eo.ignoredContours == 0);
    // Even-odd: a frame, whose axis is a loop at radius 2.5 along the sides; at each corner the
    // parabolic arcs round the inner corner meet the outer corner's bisector at radius
    // 10 - 5 sqrt 2 (equidistant from both outer walls and the inner corner). T-0153: before the
    // on-boundary circumcentre fix those corner junctions were missed and every radius was <= 2.5.
    const double junction = 10.0 - 5.0 * std::sqrt(2.0);
    double rMax = 0.0;
    for (const MedialPoint& p : orc::allPoints(eo)) {
        CHECK(p.radius <= junction + 1e-9);
        rMax = std::max(rMax, p.radius);
    }
    CHECK(rMax >= junction - kTol.chordal);
}

TEST_CASE("medial axis: output is deterministic", "[geometry][medial][ac1]") {
    const std::vector<Contour> cs{bezierEllipse({0, 0}, 8.0, 3.0), Contour::circle({-3, 0}, 1.0)};
    const MedialAxis a = medialAxis(cs, kTol);
    const MedialAxis b = medialAxis(cs, kTol);
    REQUIRE(a.branches.size() == b.branches.size());
    for (std::size_t i = 0; i < a.branches.size(); ++i) {
        REQUIRE(a.branches[i].points.size() == b.branches[i].points.size());
        for (std::size_t k = 0; k < a.branches[i].points.size(); ++k) {
            CHECK(a.branches[i].points[k].point == b.branches[i].points[k].point);
            CHECK(a.branches[i].points[k].radius == b.branches[i].points[k].radius);
        }
    }
}

TEST_CASE("medial axis oracle: off-axis points are rejected", "[geometry][medial][ac1]") {
    // The equidistance oracle must not be vacuous: points off the true axis fail it.
    const std::vector<Contour> rect{Contour::rectangle(Box2{{0, 0}, {40, 20}})};
    const orc::DenseBoundary db = orc::sampleBoundary(rect, kTol.chordal);
    CHECK(orc::isMedial(db, {20, 10}, 10.0, kTol.chordal));
    CHECK(orc::isMedial(db, {5, 5}, 5.0, kTol.chordal));      // corner bisector
    CHECK_FALSE(orc::isMedial(db, {20, 7}, 7.0, kTol.chordal));  // 3 mm off the centre line
    CHECK_FALSE(orc::isMedial(db, {20, 9.9}, 9.9, kTol.chordal));
    CHECK_FALSE(orc::isMedial(db, {6, 4}, 4.0, kTol.chordal));  // beside a corner branch
    const std::vector<Contour> circle{Contour::circle({0, 0}, 5.0)};
    const orc::DenseBoundary dc = orc::sampleBoundary(circle, kTol.chordal);
    CHECK(orc::isMedial(dc, {0, 0}, 5.0, kTol.chordal));
    CHECK_FALSE(orc::isMedial(dc, {0.5, 0}, 4.5, kTol.chordal));
}
