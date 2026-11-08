#include "carvetto/viewmodel/node_ops.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/shapes.hpp"

using namespace carvetto::geometry;
namespace nodeops = carvetto::viewmodel::nodeops;
using Catch::Matchers::WithinAbs;

namespace {

Contour rect(double w = 10.0, double h = 6.0) {
    return Contour::rectangle(Box2{{0, 0}, {w, h}});
}

Contour openLines() {
    Contour c;
    c.append(LineSpan{{0, 0}, {10, 0}});
    c.append(LineSpan{{10, 0}, {10, 10}});
    c.append(LineSpan{{10, 10}, {0, 10}});
    return c;
}

/// Largest distance from points sampled on `a` to the contour `b` (one-sided Hausdorff estimate).
double maxDeviation(const Contour& a, const Contour& b, int samples = 200) {
    double worst = 0.0;
    for (const Span& s : a.spans) {
        for (int i = 0; i <= samples; ++i) {
            const Vec2 p = pointAt(s, static_cast<double>(i) / samples);
            worst = std::max(worst, distanceTo(b, p));
        }
    }
    return worst;
}

double tangentAngle(const Contour& c, std::size_t node) {
    const std::size_t n = c.spans.size();
    const Vec2 in = tangentAt(c.spans[(node + n - 1) % n], 1.0);
    const Vec2 out = tangentAt(c.spans[node % n], 0.0);
    return std::atan2(std::fabs(in.cross(out)), in.dot(out));
}

BezierSpan wave() {
    return {{0, 0}, {3, 8}, {7, -8}, {10, 0}};
}

}  // namespace

TEST_CASE("node numbering of open and closed contours", "[viewmodel][node]") {
    CHECK(nodeops::nodeCount(rect()) == 4);
    CHECK(nodeops::nodeCount(openLines()) == 4);
    CHECK(nodeops::nodeCount(Contour{}) == 0);
    CHECK(nodeops::nodePoint(rect(), 2) == Vec2{10, 6});
    CHECK(nodeops::nodePoint(openLines(), 3) == Vec2{0, 10});
}

TEST_CASE("a full circle is normalised into two half arcs of the same circle", "[viewmodel][node]") {
    const Contour circle = Contour::circle({3, 4}, 5.0);
    CHECK_FALSE(nodeops::isNormalised(circle));
    const Contour n = nodeops::normalised(circle);
    REQUIRE(n.spans.size() == 2);
    CHECK(n.closed);
    CHECK(nodeops::isNormalised(n));
    CHECK(nodeops::nodeCount(n) == 2);
    CHECK(maxDeviation(circle, n) < 1e-12);
    CHECK(maxDeviation(n, circle) < 1e-12);
    CHECK_THAT(length(n.spans[0]) + length(n.spans[1]), WithinAbs(2 * kPi * 5.0, 1e-9));
    // Endpoints chain.
    CHECK(approxEqual(endPoint(n.spans[0]), startPoint(n.spans[1]), 1e-12));
    CHECK(approxEqual(endPoint(n.spans[1]), startPoint(n.spans[0]), 1e-12));
    // Idempotent and CW circles work too.
    CHECK(nodeops::normalised(n) == n);
    const Contour cw = nodeops::normalised(Contour::circle({0, 0}, 2.0, ArcDirection::CW));
    CHECK(cw.spans.size() == 2);
    CHECK_THAT(length(cw.spans[0]), WithinAbs(kPi * 2.0, 1e-9));
}

TEST_CASE("arc construction helpers", "[viewmodel][node]") {
    SECTION("sweep") {
        const auto arc = nodeops::arcFromSweep({1, 0}, {0, 1}, kPi / 2);
        REQUIRE(arc.has_value());
        CHECK(approxEqual(arc->centre, {0, 0}, 1e-12));
        CHECK_THAT(arc->radius, WithinAbs(1.0, 1e-12));
        CHECK(arc->direction == ArcDirection::CCW);
        const auto cw = nodeops::arcFromSweep({1, 0}, {0, 1}, -3 * kPi / 2);
        REQUIRE(cw.has_value());
        CHECK(cw->direction == ArcDirection::CW);
        CHECK_THAT(cw->sweepAngle(), WithinAbs(-3 * kPi / 2, 1e-9));
        CHECK(approxEqual(cw->startPoint(), {1, 0}, 1e-9));
        CHECK(approxEqual(cw->endPoint(), {0, 1}, 1e-9));
        CHECK_FALSE(nodeops::arcFromSweep({1, 0}, {1, 0}, 1.0).has_value());
        CHECK_FALSE(nodeops::arcFromSweep({1, 0}, {0, 1}, 0.0).has_value());
        CHECK_FALSE(nodeops::arcFromSweep({1, 0}, {0, 1}, 2 * kPi).has_value());
    }
    SECTION("through three points") {
        const auto arc = nodeops::arcThrough({0, 0}, {1, 1}, {2, 0});
        REQUIRE(arc.has_value());
        CHECK(approxEqual(arc->centre, {1, 0}, 1e-12));
        CHECK(arc->direction == ArcDirection::CW);  // over the top from left to right
        CHECK(distanceTo(Contour{{*arc}, false}, {1, 1}) < 1e-12);
        const auto ccw = nodeops::arcThrough({0, 0}, {1, -1}, {2, 0});
        REQUIRE(ccw.has_value());
        CHECK(ccw->direction == ArcDirection::CCW);
        CHECK(distanceTo(Contour{{*ccw}, false}, {1, -1}) < 1e-12);
        CHECK_FALSE(nodeops::arcThrough({0, 0}, {1, 0}, {2, 0}).has_value());
        CHECK_FALSE(nodeops::arcThrough({0, 0}, {0, 0}, {2, 0}).has_value());
    }
    SECTION("bulge") {
        const auto arc = nodeops::arcFromBulge({0, 0}, {2, 0}, 1.0);  // semicircle, counter-clockwise
        REQUIRE(arc.has_value());
        CHECK_THAT(arc->sweepAngle(), WithinAbs(kPi, 1e-9));
        CHECK(approxEqual(arc->centre, {1, 0}, 1e-12));
        CHECK(arc->pointAt(0.5).y < 0.0);  // counter-clockwise from (0,0) to (2,0) dips below
        const auto neg = nodeops::arcFromBulge({0, 0}, {2, 0}, -1.0);
        REQUIRE(neg.has_value());
        CHECK(neg->pointAt(0.5).y > 0.0);
        CHECK_FALSE(nodeops::arcFromBulge({0, 0}, {2, 0}, std::numeric_limits<double>::quiet_NaN()).has_value());
    }
}

TEST_CASE("moveNodes: lines end exactly at the target and neighbours follow", "[viewmodel][node]") {
    Contour c = rect();
    std::string why;
    REQUIRE(nodeops::moveNodes(c, {{2, {12.5, 7.25}}}, &why));
    CHECK(c.spans[1] == Span{LineSpan{{10, 0}, {12.5, 7.25}}});
    CHECK(c.spans[2] == Span{LineSpan{{12.5, 7.25}, {0, 6}}});
    CHECK(c.spans[0] == Span{LineSpan{{0, 0}, {10, 0}}});
    CHECK(c.isContinuous(Tolerance::defaults()));
    // Moving node 0 of a closed contour changes the last and the first span.
    REQUIRE(nodeops::moveNodes(c, {{0, {-1, -2}}}));
    CHECK(startPoint(c.spans[0]) == Vec2{-1, -2});
    CHECK(endPoint(c.spans[3]) == Vec2{-1, -2});
    // Open contour: node n is the end of the last span.
    Contour o = openLines();
    REQUIRE(nodeops::moveNodes(o, {{3, {1, 11}}}));
    CHECK(endPoint(o.spans[2]) == Vec2{1, 11});
}

TEST_CASE("moveNodes: arcs keep their bulge, Beziers keep their handle offsets", "[viewmodel][node]") {
    SECTION("arc") {
        Contour c;
        c.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.5));
        c.append(LineSpan{{10, 0}, {0, 0}});
        c.closed = true;
        const double sweep = std::get<ArcSpan>(c.spans[0]).sweepAngle();
        REQUIRE(nodeops::moveNodes(c, {{1, {12, 3}}}));
        const auto& arc = std::get<ArcSpan>(c.spans[0]);
        CHECK_THAT(arc.sweepAngle(), WithinAbs(sweep, 1e-9));
        CHECK(approxEqual(arc.startPoint(), {0, 0}, 1e-9));
        CHECK(approxEqual(arc.endPoint(), {12, 3}, 1e-9));
        CHECK(approxEqual(startPoint(c.spans[1]), {12, 3}, 1e-12));
    }
    SECTION("arc translated as a whole stays an exact translation") {
        Contour c;
        c.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.5));
        c.append(LineSpan{{10, 0}, {0, 0}});
        c.closed = true;
        const ArcSpan before = std::get<ArcSpan>(c.spans[0]);
        REQUIRE(nodeops::moveNodes(c, {{0, {1, 2}}, {1, {11, 2}}}));
        const auto& arc = std::get<ArcSpan>(c.spans[0]);
        CHECK(arc.centre == before.centre + Vec2{1, 2});
        CHECK(arc.radius == before.radius);
    }
    SECTION("bezier") {
        Contour c;
        c.append(wave());
        c.append(LineSpan{{10, 0}, {0, 0}});
        c.closed = true;
        REQUIRE(nodeops::moveNodes(c, {{0, {1, 1}}, {1, {9, -2}}}));
        const auto& b = std::get<BezierSpan>(c.spans[0]);
        CHECK(b.p0 == Vec2{1, 1});
        CHECK(b.p3 == Vec2{9, -2});
        CHECK(b.p1 == Vec2{3, 8} + Vec2{1, 1});
        CHECK(b.p2 == Vec2{7, -8} + Vec2{-1, -2});
    }
}

TEST_CASE("moveNodes refuses degenerate or out-of-range results and changes nothing", "[viewmodel][node]") {
    Contour c = rect();
    const Contour before = c;
    CHECK_FALSE(nodeops::moveNodes(c, {{1, {0, 0}}}));  // onto node 0: zero-length span
    CHECK_FALSE(nodeops::moveNodes(c, {{1, {std::numeric_limits<double>::quiet_NaN(), 0}}}));
    CHECK_FALSE(nodeops::moveNodes(c, {{1, {2e9, 0}}}));
    CHECK_FALSE(nodeops::moveNodes(c, {{9, {1, 1}}}));
    CHECK(c == before);
    Contour empty;
    CHECK_FALSE(nodeops::moveNodes(empty, {{0, {1, 1}}}));
    // An arc whose ends meet cannot be re-fitted.
    Contour arcs;
    arcs.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.5));
    arcs.append(*nodeops::arcFromBulge({10, 0}, {0, 0}, 0.5));
    arcs.closed = true;
    const Contour arcsBefore = arcs;
    CHECK_FALSE(nodeops::moveNodes(arcs, {{1, {0, 0}}}));
    CHECK(arcs == arcsBefore);
}

TEST_CASE("setControlPoint with and without keeping tangency", "[viewmodel][node]") {
    Contour c;
    c.append(BezierSpan{{0, 0}, {2, 4}, {4, 4}, {6, 0}});
    c.append(BezierSpan{{6, 0}, {8, -4}, {10, -4}, {12, 0}});
    // Node 1 = (6,0): handles (4,4) and (8,-4) are collinear through the node.
    Contour plain = c;
    REQUIRE(nodeops::setControlPoint(plain, 0, 1, {5, 2}, false));
    CHECK(std::get<BezierSpan>(plain.spans[0]).p2 == Vec2{5, 2});
    CHECK(std::get<BezierSpan>(plain.spans[1]).p1 == Vec2{8, -4});  // untouched

    Contour kept = c;
    REQUIRE(nodeops::setControlPoint(kept, 0, 1, {5, 2}, true));
    const Vec2 node{6, 0};
    const Vec2 in = std::get<BezierSpan>(kept.spans[0]).p2 - node;
    const Vec2 out = std::get<BezierSpan>(kept.spans[1]).p1 - node;
    CHECK_THAT(in.cross(out), WithinAbs(0.0, 1e-12));  // still collinear
    CHECK(in.dot(out) < 0.0);                          // on opposite sides
    CHECK_THAT(out.length(), WithinAbs(Vec2(2, -4).length(), 1e-12));  // length kept

    CHECK_FALSE(nodeops::setControlPoint(c, 0, 2, {1, 1}, false));
    CHECK_FALSE(nodeops::setControlPoint(c, 5, 0, {1, 1}, false));
    Contour lines = rect();
    CHECK_FALSE(nodeops::setControlPoint(lines, 0, 0, {1, 1}, false));
}

TEST_CASE("setControlPoint wraps around the start of a closed contour", "[viewmodel][node]") {
    Contour c;
    c.append(BezierSpan{{0, 0}, {1, 3}, {5, 3}, {6, 0}});
    c.append(BezierSpan{{6, 0}, {5, -3}, {1, -3}, {0, 0}});
    c.closed = true;
    REQUIRE(nodeops::setControlPoint(c, 0, 0, {2, 2}, true));  // p1 of span 0 at node 0; opposite = p2 of span 1
    const Vec2 node{0, 0};
    const Vec2 a = std::get<BezierSpan>(c.spans[0]).p1 - node;
    const Vec2 b = std::get<BezierSpan>(c.spans[1]).p2 - node;
    CHECK_THAT(a.cross(b), WithinAbs(0.0, 1e-12));
    CHECK(a.dot(b) < 0.0);
}

TEST_CASE("splitSpan describes exactly the same curve", "[viewmodel][node]") {
    const Span spans[] = {LineSpan{{0, 0}, {7, 3}}, *nodeops::arcFromBulge({0, 0}, {10, 0}, 0.7),
                          ArcSpan::fullCircle({1, 1}, 2.0, ArcDirection::CW), wave()};
    for (const Span& s : spans) {
        for (const double t : {0.1, 0.5, 0.83}) {
            const auto [a, b] = nodeops::splitSpan(s, t);
            CHECK(approxEqual(endPoint(a), startPoint(b), 1e-12));
            CHECK(approxEqual(startPoint(a), startPoint(s), 1e-12));
            CHECK(approxEqual(endPoint(b), endPoint(s), 1e-12));
            Contour original;
            original.append(s);
            Contour parts;
            parts.append(a);
            parts.append(b);
            CHECK(maxDeviation(original, parts) < 1e-9);
            CHECK(maxDeviation(parts, original) < 1e-9);
        }
    }
}

TEST_CASE("insertPoint lies on the span, preserves the shape and indexes the new node", "[viewmodel][node]") {
    SECTION("line") {
        const auto r = nodeops::insertPoint(rect(), 1, 0.25);
        REQUIRE(r.has_value());
        CHECK(r->spans.size() == 5);
        CHECK(nodeops::nodePoint(*r, 2) == Vec2{10, 1.5});
        CHECK(maxDeviation(rect(), *r) < 1e-12);
    }
    SECTION("bezier and arc") {
        Contour c;
        c.append(wave());
        c.append(*nodeops::arcFromBulge({10, 0}, {0, 0}, -0.4));
        c.closed = true;
        for (std::size_t span = 0; span < 2; ++span) {
            const auto r = nodeops::insertPoint(c, span, 0.37);
            REQUIRE(r.has_value());
            const Vec2 p = nodeops::nodePoint(*r, span + 1);
            CHECK(distanceTo(c, p) < 1e-9);
            CHECK(maxDeviation(c, *r) < 1e-9);
            CHECK(maxDeviation(*r, c) < 1e-9);
            CHECK(r->isContinuous(Tolerance::defaults()));
        }
    }
    SECTION("refusals") {
        CHECK_FALSE(nodeops::insertPoint(rect(), 9, 0.5).has_value());
        CHECK_FALSE(nodeops::insertPoint(rect(), 0, 0.0).has_value());
        CHECK_FALSE(nodeops::insertPoint(rect(), 0, 1.0).has_value());
        CHECK_FALSE(nodeops::insertPoint(rect(), 0, std::numeric_limits<double>::quiet_NaN()).has_value());
        CHECK_FALSE(nodeops::insertPoint(rect(), 0, 1e-9).has_value());  // too close to the end
    }
}

TEST_CASE("deleteNodes re-joins the spans around a removed node", "[viewmodel][node]") {
    SECTION("rectangle corner gives a triangle with exact vertices") {
        const auto r = nodeops::deleteNodes(rect(), {2});
        REQUIRE(r.has_value());
        REQUIRE(r->spans.size() == 3);
        CHECK(r->closed);
        CHECK(r->spans[1] == Span{LineSpan{{10, 0}, {0, 6}}});
        CHECK(r->isContinuous(Tolerance::defaults()));
    }
    SECTION("node 0 of a closed contour moves the start") {
        const auto r = nodeops::deleteNodes(rect(), {0});
        REQUIRE(r.has_value());
        CHECK(startPoint(r->spans[0]) == Vec2{10, 0});
        CHECK(r->isContinuous(Tolerance::defaults()));
    }
    SECTION("open contour: end nodes drop their span, inner nodes merge") {
        const auto first = nodeops::deleteNodes(openLines(), {0});
        REQUIRE(first.has_value());
        CHECK(first->spans.size() == 2);
        CHECK(startPoint(first->spans[0]) == Vec2{10, 0});
        const auto last = nodeops::deleteNodes(openLines(), {3});
        REQUIRE(last.has_value());
        CHECK(endPoint(last->spans.back()) == Vec2{10, 10});
        const auto mid = nodeops::deleteNodes(openLines(), {1, 2});
        REQUIRE(mid.has_value());
        REQUIRE(mid->spans.size() == 1);
        CHECK(mid->spans[0] == Span{LineSpan{{0, 0}, {0, 10}}});
    }
    SECTION("two Beziers merge keeping the outer handles") {
        Contour c;
        c.append(BezierSpan{{0, 0}, {1, 3}, {3, 3}, {4, 0}});
        c.append(BezierSpan{{4, 0}, {5, -3}, {7, -3}, {8, 0}});
        const auto r = nodeops::deleteNodes(c, {1});
        REQUIRE(r.has_value());
        REQUIRE(r->spans.size() == 1);
        CHECK(std::get<BezierSpan>(r->spans[0]) == BezierSpan{{0, 0}, {1, 3}, {7, -3}, {8, 0}});
    }
    SECTION("a Bezier next to a line gets a third-of-chord handle on the line side") {
        Contour c;
        c.append(BezierSpan{{0, 0}, {1, 3}, {3, 3}, {4, 0}});
        c.append(LineSpan{{4, 0}, {8, 0}});
        const auto r = nodeops::deleteNodes(c, {1});
        REQUIRE(r.has_value());
        const auto& b = std::get<BezierSpan>(r->spans[0]);
        CHECK(b.p1 == Vec2{1, 3});
        CHECK(approxEqual(b.p2, {16.0 / 3.0, 0}, 1e-12));
    }
    SECTION("refusals") {
        CHECK_FALSE(nodeops::deleteNodes(rect(), {0, 1, 2}).has_value());   // one node left
        CHECK_FALSE(nodeops::deleteNodes(rect(), {1, 3}).has_value());      // closed lines would need 3
        CHECK_FALSE(nodeops::deleteNodes(rect(), {7}).has_value());
        CHECK_FALSE(nodeops::deleteNodes(rect(), {}).has_value());
        Contour one;
        one.append(LineSpan{{0, 0}, {5, 0}});
        CHECK_FALSE(nodeops::deleteNodes(one, {1}).has_value());
        // A lens of two arcs can lose neither node: one span would remain.
        Contour lens;
        lens.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.4));
        lens.append(*nodeops::arcFromBulge({10, 0}, {0, 0}, 0.4));
        lens.closed = true;
        CHECK_FALSE(nodeops::deleteNodes(lens, {0}).has_value());
    }
}

TEST_CASE("deleteSpans opens a closed contour and splits an open one", "[viewmodel][node]") {
    const Contour r = rect();
    SECTION("closed") {
        const auto pieces = nodeops::deleteSpans(r, {1});
        REQUIRE(pieces.size() == 1);
        CHECK_FALSE(pieces[0].closed);
        REQUIRE(pieces[0].spans.size() == 3);
        CHECK(pieces[0].spans[0] == r.spans[2]);
        CHECK(pieces[0].spans[1] == r.spans[3]);
        CHECK(pieces[0].spans[2] == r.spans[0]);
        const auto two = nodeops::deleteSpans(r, {0, 2});
        REQUIRE(two.size() == 2);
        CHECK(two[0].spans == std::vector<Span>{r.spans[1]});
        CHECK(two[1].spans == std::vector<Span>{r.spans[3]});
    }
    SECTION("open") {
        const Contour o = openLines();
        const auto mid = nodeops::deleteSpans(o, {1});
        REQUIRE(mid.size() == 2);
        CHECK(mid[0].spans == std::vector<Span>{o.spans[0]});
        CHECK(mid[1].spans == std::vector<Span>{o.spans[2]});
        CHECK(nodeops::deleteSpans(o, {0}).size() == 1);
        CHECK(nodeops::deleteSpans(o, {0, 1, 2}).empty());
        CHECK(nodeops::deleteSpans(o, {5}).empty());
    }
}

TEST_CASE("cutAtNodes yields open pieces that cover exactly the original spans", "[viewmodel][node]") {
    const Contour o = openLines();
    SECTION("open contour at an inner node") {
        const auto pieces = nodeops::cutAtNodes(o, {2});
        REQUIRE(pieces.size() == 2);
        CHECK(pieces[0].spans == std::vector<Span>{o.spans[0], o.spans[1]});
        CHECK(pieces[1].spans == std::vector<Span>{o.spans[2]});
        CHECK_FALSE(pieces[0].closed);
        CHECK(nodeops::cutAtNodes(o, {0}).empty());
        CHECK(nodeops::cutAtNodes(o, {3}).empty());
        CHECK(nodeops::cutAtNodes(o, {1, 2}).size() == 3);
    }
    SECTION("closed contour") {
        const Contour r = rect();
        const auto one = nodeops::cutAtNodes(r, {2});
        REQUIRE(one.size() == 1);
        CHECK(one[0].spans == std::vector<Span>{r.spans[2], r.spans[3], r.spans[0], r.spans[1]});
        const auto two = nodeops::cutAtNodes(r, {1, 3});
        REQUIRE(two.size() == 2);
        CHECK(two[0].spans == std::vector<Span>{r.spans[1], r.spans[2]});
        CHECK(two[1].spans == std::vector<Span>{r.spans[3], r.spans[0]});
        for (const Contour& piece : two) {
            CHECK_FALSE(piece.closed);
            CHECK(piece.isContinuous(Tolerance::defaults()));
        }
        CHECK(nodeops::cutAtNodes(r, {9}).empty());
    }
}

TEST_CASE("convertSpan preserves end points and honours the stated through point or bulge", "[viewmodel][node]") {
    const Contour o = openLines();
    SECTION("line to arc through a point") {
        nodeops::ConvertSpec spec;
        spec.target = nodeops::SpanTarget::Arc;
        spec.through = Vec2{5, -2};
        const auto r = nodeops::convertSpan(o, 0, spec);
        REQUIRE(r.has_value());
        const auto* arc = std::get_if<ArcSpan>(&r->spans[0]);
        REQUIRE(arc != nullptr);
        CHECK(approxEqual(arc->startPoint(), {0, 0}, 1e-9));
        CHECK(approxEqual(arc->endPoint(), {10, 0}, 1e-9));
        CHECK(distanceTo(Contour{{*arc}, false}, {5, -2}) < 1e-9);
        CHECK(r->spans[1] == o.spans[1]);
    }
    SECTION("line to arc by bulge") {
        nodeops::ConvertSpec spec;
        spec.target = nodeops::SpanTarget::Arc;
        spec.bulge = 0.5;
        const auto r = nodeops::convertSpan(o, 0, spec);
        REQUIRE(r.has_value());
        const auto& arc = std::get<ArcSpan>(r->spans[0]);
        CHECK_THAT(arc.sweepAngle(), WithinAbs(4.0 * std::atan(0.5), 1e-9));
        // Sagitta of a bulge b over chord c is b * c / 2.
        CHECK_THAT(std::fabs(arc.pointAt(0.5).y), WithinAbs(0.5 * 10.0 / 2.0, 1e-9));
    }
    SECTION("line to arc without input uses the default bulge; a collinear point is refused") {
        nodeops::ConvertSpec spec;
        spec.target = nodeops::SpanTarget::Arc;
        const auto r = nodeops::convertSpan(o, 0, spec);
        REQUIRE(r.has_value());
        CHECK_THAT(std::get<ArcSpan>(r->spans[0]).sweepAngle(), WithinAbs(4.0 * std::atan(nodeops::kDefaultBulge), 1e-9));
        spec.through = Vec2{5, 0};
        std::string why;
        CHECK_FALSE(nodeops::convertSpan(o, 0, spec, nullptr, &why).has_value());
        CHECK_FALSE(why.empty());
    }
    SECTION("bezier to arc goes through the Bezier's midpoint") {
        Contour c;
        c.append(BezierSpan{{0, 0}, {2, 6}, {8, 6}, {10, 0}});
        nodeops::ConvertSpec spec;
        spec.target = nodeops::SpanTarget::Arc;
        const auto r = nodeops::convertSpan(c, 0, spec);
        REQUIRE(r.has_value());
        const Vec2 mid = std::get<BezierSpan>(c.spans[0]).pointAt(0.5);
        CHECK(distanceTo(*r, mid) < 1e-9);
        CHECK(approxEqual(startPoint(r->spans[0]), {0, 0}, 1e-9));
        CHECK(approxEqual(endPoint(r->spans[0]), {10, 0}, 1e-9));
    }
    SECTION("to line") {
        Contour c;
        c.append(wave());
        nodeops::ConvertSpec spec;
        const auto r = nodeops::convertSpan(c, 0, spec);
        REQUIRE(r.has_value());
        CHECK(r->spans[0] == Span{LineSpan{{0, 0}, {10, 0}}});
        const auto a = nodeops::convertSpan(Contour{{*nodeops::arcFromBulge({0, 0}, {4, 0}, 1)}, false}, 0, spec);
        REQUIRE(a.has_value());
        CHECK(std::holds_alternative<LineSpan>(a->spans[0]));
    }
    SECTION("line to bezier: straight by default, through the stated point otherwise") {
        nodeops::ConvertSpec spec;
        spec.target = nodeops::SpanTarget::Bezier;
        const auto straight = nodeops::convertSpan(o, 0, spec);
        REQUIRE(straight.has_value());
        const auto& b0 = std::get<BezierSpan>(straight->spans[0]);
        CHECK(approxEqual(b0.p1, {10.0 / 3.0, 0}, 1e-12));
        CHECK(maxDeviation(Contour{{o.spans[0]}, false}, *straight) < 1e-12);

        spec.through = Vec2{5, 3};
        const auto bent = nodeops::convertSpan(o, 0, spec);
        REQUIRE(bent.has_value());
        const auto& b = std::get<BezierSpan>(bent->spans[0]);
        CHECK(b.p0 == Vec2{0, 0});
        CHECK(b.p3 == Vec2{10, 0});
        CHECK(approxEqual(b.pointAt(0.5), {5, 3}, 1e-12));
    }
    SECTION("arc to bezier approximates the arc and counts the pieces") {
        const Contour c{{*nodeops::arcFromBulge({0, 0}, {20, 0}, 1.0)}, false};  // semicircle, r = 10
        nodeops::ConvertSpec spec;
        spec.target = nodeops::SpanTarget::Bezier;
        std::size_t produced = 0;
        const auto r = nodeops::convertSpan(c, 0, spec, &produced);
        REQUIRE(r.has_value());
        CHECK(produced == r->spans.size());
        CHECK(produced >= 2);
        CHECK(maxDeviation(c, *r) < 5e-4 * 10.0);
        CHECK(maxDeviation(*r, c) < 5e-4 * 10.0);
        CHECK(approxEqual(startPoint(r->spans.front()), {0, 0}, 1e-9));
        CHECK(approxEqual(endPoint(r->spans.back()), {20, 0}, 1e-9));
    }
    SECTION("bad span") {
        nodeops::ConvertSpec spec;
        CHECK_FALSE(nodeops::convertSpan(o, 9, spec).has_value());
    }
}

TEST_CASE("smoothNodes gives G1 continuity (angle below 1e-9)", "[viewmodel][node]") {
    SECTION("two lines at a corner") {
        const auto r = nodeops::smoothNodes(openLines(), {1});
        REQUIRE(r.has_value());
        CHECK(tangentAngle(*r, 1) < 1e-9);
        CHECK(std::holds_alternative<BezierSpan>(r->spans[0]));
        CHECK(std::holds_alternative<BezierSpan>(r->spans[1]));
        CHECK(r->spans.size() == 3);
        CHECK(approxEqual(endPoint(r->spans[0]), {10, 0}, 1e-12));  // the node does not move
        CHECK(r->isContinuous(Tolerance::defaults()));
    }
    SECTION("closed rectangle, every node") {
        const auto r = nodeops::smoothNodes(rect(), {0, 1, 2, 3});
        REQUIRE(r.has_value());
        for (std::size_t n = 0; n < 4; ++n) {
            CHECK(tangentAngle(*r, n) < 1e-9);
        }
    }
    SECTION("two Beziers share the bisector and keep their handle lengths") {
        Contour c;
        c.append(BezierSpan{{0, 0}, {2, 1}, {4, 3}, {6, 4}});
        c.append(BezierSpan{{6, 4}, {7, 9}, {9, 9}, {12, 4}});
        const auto r = nodeops::smoothNodes(c, {1});
        REQUIRE(r.has_value());
        CHECK(tangentAngle(*r, 1) < 1e-9);
        const auto& a = std::get<BezierSpan>(r->spans[0]);
        const auto& b = std::get<BezierSpan>(r->spans[1]);
        CHECK_THAT(distance(a.p2, a.p3), WithinAbs(distance({4, 3}, {6, 4}), 1e-12));
        CHECK_THAT(distance(b.p0, b.p1), WithinAbs(distance({6, 4}, {7, 9}), 1e-12));
        CHECK(a.p0 == Vec2{0, 0});
        CHECK(b.p3 == Vec2{12, 4});
    }
    SECTION("a Bezier next to a line follows the line") {
        Contour c;
        c.append(LineSpan{{0, 0}, {5, 0}});
        c.append(BezierSpan{{5, 0}, {7, 4}, {9, 4}, {10, 0}});
        const auto r = nodeops::smoothNodes(c, {1});
        REQUIRE(r.has_value());
        CHECK(tangentAngle(*r, 1) < 1e-9);
        CHECK(std::holds_alternative<LineSpan>(r->spans[0]));  // the line is not touched
        CHECK(r->spans[0] == c.spans[0]);
    }
    SECTION("an arc is kept and the neighbouring line becomes a Bezier") {
        Contour c;
        c.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.5));
        c.append(LineSpan{{10, 0}, {10, 10}});
        const auto r = nodeops::smoothNodes(c, {1});
        REQUIRE(r.has_value());
        CHECK(tangentAngle(*r, 1) < 1e-9);
        CHECK(r->spans[0] == c.spans[0]);
        CHECK(std::holds_alternative<BezierSpan>(r->spans[1]));
        CHECK(r->spans.size() == 2);
    }
    SECTION("arc to arc: the second arc becomes Beziers, the first is untouched") {
        Contour c;
        c.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.5));
        c.append(*nodeops::arcFromBulge({10, 0}, {12, 9}, -0.3));
        const auto r = nodeops::smoothNodes(c, {1});
        REQUIRE(r.has_value());
        CHECK(tangentAngle(*r, 1) < 1e-9);
        CHECK(r->spans[0] == c.spans[0]);
        CHECK(std::holds_alternative<BezierSpan>(r->spans[1]));
    }
    SECTION("the wrap-around node of a closed contour, with an expanding arc") {
        Contour c;
        c.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.5));
        c.append(LineSpan{{10, 0}, {10, 10}});
        c.append(*nodeops::arcFromBulge({10, 10}, {0, 0}, 0.3));
        c.closed = true;
        const auto r = nodeops::smoothNodes(c, {0, 1, 2});
        REQUIRE(r.has_value());
        for (std::size_t n = 0; n < r->spans.size(); ++n) {
            CHECK(tangentAngle(*r, n) < 1e-9);
        }
        CHECK(r->isContinuous(Tolerance::defaults()));
    }
    SECTION("refusals: ends, already smooth, cusp") {
        std::string why;
        CHECK_FALSE(nodeops::smoothNodes(openLines(), {0}, &why).has_value());
        CHECK_FALSE(nodeops::smoothNodes(openLines(), {3}).has_value());
        Contour straight;
        straight.append(LineSpan{{0, 0}, {5, 0}});
        straight.append(LineSpan{{5, 0}, {10, 0}});
        CHECK_FALSE(nodeops::smoothNodes(straight, {1}, &why).has_value());
        CHECK(why.find("smooth") != std::string::npos);
        Contour cusp;
        cusp.append(LineSpan{{0, 0}, {5, 0}});
        cusp.append(LineSpan{{5, 0}, {0, 0}});
        CHECK_FALSE(nodeops::smoothNodes(cusp, {1}).has_value());
        CHECK_FALSE(nodeops::smoothNodes(openLines(), {9}).has_value());
    }
}

TEST_CASE("rotateStart keeps the geometry", "[viewmodel][node]") {
    const Contour r = rect();
    const auto rotated = nodeops::rotateStart(r, 2);
    REQUIRE(rotated.has_value());
    CHECK(startPoint(rotated->spans[0]) == Vec2{10, 6});
    CHECK(rotated->closed);
    CHECK(rotated->spans.size() == 4);
    CHECK(maxDeviation(r, *rotated) < 1e-12);
    CHECK_THAT(rotated->length(), WithinAbs(r.length(), 1e-12));
    CHECK_FALSE(nodeops::rotateStart(r, 0).has_value());
    CHECK_FALSE(nodeops::rotateStart(r, 4).has_value());
    CHECK_FALSE(nodeops::rotateStart(openLines(), 1).has_value());
}

TEST_CASE("reshapeSpan and moveSpan", "[viewmodel][node]") {
    SECTION("bezier: the curve point at the grabbed parameter moves by the delta") {
        Contour c;
        c.append(wave());
        const BezierSpan before = wave();
        REQUIRE(nodeops::reshapeSpan(c, 0, 0.3, {1.5, -2}));
        const auto& b = std::get<BezierSpan>(c.spans[0]);
        CHECK(approxEqual(b.pointAt(0.3), before.pointAt(0.3) + Vec2{1.5, -2}, 1e-12));
        CHECK(b.p0 == before.p0);
        CHECK(b.p3 == before.p3);
        // Near the ends the parameter is clamped, the move is still the requested delta.
        Contour d;
        d.append(wave());
        REQUIRE(nodeops::reshapeSpan(d, 0, 0.001, {0, 1}));
        CHECK(approxEqual(std::get<BezierSpan>(d.spans[0]).pointAt(0.05), wave().pointAt(0.05) + Vec2{0, 1}, 1e-12));
    }
    SECTION("arc: re-fitted through the moved point") {
        Contour c;
        c.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.4));
        const Vec2 grab = std::get<ArcSpan>(c.spans[0]).pointAt(0.5);
        REQUIRE(nodeops::reshapeSpan(c, 0, 0.5, {0, -2}));
        const auto& arc = std::get<ArcSpan>(c.spans[0]);
        CHECK(approxEqual(arc.startPoint(), {0, 0}, 1e-9));
        CHECK(approxEqual(arc.endPoint(), {10, 0}, 1e-9));
        CHECK(distanceTo(c, grab + Vec2{0, -2}) < 1e-9);
        // Moving onto the chord has no arc.
        Contour flat;
        flat.append(*nodeops::arcFromBulge({0, 0}, {10, 0}, 0.4));
        const Vec2 top = std::get<ArcSpan>(flat.spans[0]).pointAt(0.5);
        CHECK_FALSE(nodeops::reshapeSpan(flat, 0, 0.5, Vec2{0, 0} - top + Vec2{5, 0}));
    }
    SECTION("line and moveSpan translate both end nodes and the neighbours follow") {
        Contour c = rect();
        REQUIRE(nodeops::reshapeSpan(c, 1, 0.5, {2, 0}));
        CHECK(c.spans[1] == Span{LineSpan{{12, 0}, {12, 6}}});
        CHECK(c.spans[0] == Span{LineSpan{{0, 0}, {12, 0}}});
        CHECK(c.spans[2] == Span{LineSpan{{12, 6}, {0, 6}}});
        Contour d = rect();
        REQUIRE(nodeops::moveSpan(d, 3, {0, 1}));  // last span of a closed contour wraps to node 0
        CHECK(d.spans[3] == Span{LineSpan{{0, 7}, {0, 1}}});
        CHECK(startPoint(d.spans[0]) == Vec2{0, 1});
        CHECK_FALSE(nodeops::moveSpan(d, 9, {0, 1}));
    }
    SECTION("moving a whole Bezier span translates it exactly") {
        Contour c;
        c.append(wave());
        c.append(LineSpan{{10, 0}, {0, 0}});
        c.closed = true;
        REQUIRE(nodeops::moveSpan(c, 0, {3, 4}));
        CHECK(std::get<BezierSpan>(c.spans[0]) == wave().transformed(Transform2::translation({3, 4})));
    }
}

TEST_CASE("validContour rejects NaN, huge and degenerate geometry", "[viewmodel][node]") {
    CHECK(nodeops::validContour(rect()));
    CHECK_FALSE(nodeops::validContour(Contour{}));
    Contour bad = rect();
    bad.spans[0] = LineSpan{{0, 0}, {std::numeric_limits<double>::infinity(), 0}};
    CHECK_FALSE(nodeops::validContour(bad));
    bad.spans[0] = LineSpan{{0, 0}, {1e9 + 1, 0}};
    CHECK_FALSE(nodeops::validContour(bad));
    bad.spans[0] = LineSpan{{0, 0}, {0, 0}};
    CHECK_FALSE(nodeops::validContour(bad));
    bad.spans[0] = BezierSpan{{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    CHECK_FALSE(nodeops::validContour(bad));
}
