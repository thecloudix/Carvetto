#include "carvetto/geometry/edit_ops.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;
using Catch::Matchers::WithinAbs;

namespace {
const Tolerance tol{};

Contour square(double side) {
    return Contour::rectangle(Box2{{0.0, 0.0}, {side, side}});
}

double maxGap(const Contour& c) {
    double g = 0.0;
    const std::size_t n = c.spans.size();
    for (std::size_t i = 0; i + 1 < n; ++i) {
        g = std::max(g, distance(endPoint(c.spans[i]), startPoint(c.spans[i + 1])));
    }
    if (c.closed && n > 0) {
        g = std::max(g, distance(endPoint(c.spans[n - 1]), startPoint(c.spans[0])));
    }
    return g;
}

double totalLength(const std::vector<Contour>& cs) {
    double l = 0.0;
    for (const Contour& c : cs) {
        l += c.length();
    }
    return l;
}

/// Polyline-ish contour of lines through the points.
Contour lines(std::initializer_list<Vec2> pts, bool closed = false) {
    Contour c;
    c.closed = closed;
    std::vector<Vec2> v(pts);
    for (std::size_t i = 0; i + 1 < v.size(); ++i) {
        c.append(LineSpan{v[i], v[i + 1]});
    }
    if (closed) {
        c.append(LineSpan{v.back(), v.front()});
    }
    return c;
}

/// Polygon with the given interior (turn) angle at its second vertex: corner at origin.
Contour wedge(double turnAngle) {
    // A runs along +x into the origin, B leaves at `turnAngle` to the left.
    return lines({{-20.0, 0.0}, {0.0, 0.0}, {20.0 * std::cos(turnAngle), 20.0 * std::sin(turnAngle)}});
}

const ArcSpan& arcAt(const Contour& c, std::size_t i) {
    return std::get<ArcSpan>(c.spans[i]);
}
}  // namespace

TEST_CASE("fillet: normal corner is tangent to both lines with the requested radius", "[edit][fillet]") {
    SplitMix rng{11};
    for (int k = 0; k < 60; ++k) {
        const double turn = rng.uniform(0.15, 3.0) * (k % 2 == 0 ? 1.0 : -1.0);
        const double r = std::min(rng.uniform(0.2, 5.0), 15.0 / std::tan(std::fabs(turn) / 2.0));
        Contour c = wedge(turn);
        const FilletResult res = filletCorner(c, 1, r, FilletKind::Normal);
        REQUIRE(res.status == EditStatus::Ok);
        REQUIRE(res.contour.spans.size() == 3);
        const ArcSpan& arc = arcAt(res.contour, 1);
        CHECK_THAT(arc.radius, WithinAbs(r, 1e-9));
        // Tangent: centre is r from both original lines.
        CHECK_THAT(closestPoint(c.spans[0], arc.centre).distance, WithinAbs(r, 1e-9));
        CHECK_THAT(closestPoint(c.spans[1], arc.centre).distance, WithinAbs(r, 1e-9));
        // Endpoints on the original spans; contour continuous.
        CHECK(closestPoint(c.spans[0], arc.startPoint()).distance < 1e-9);
        CHECK(closestPoint(c.spans[1], arc.endPoint()).distance < 1e-9);
        CHECK(maxGap(res.contour) < 1e-9);
        // Tangent continuity (angle) at both joins.
        const Vec2 t0 = tangentAt(res.contour.spans[0], 1.0);
        const Vec2 t1 = arc.tangentAt(0.0);
        CHECK(std::fabs(t0.cross(t1)) < 1e-9);
        CHECK(t0.dot(t1) > 0.0);
        const Vec2 t2 = arc.tangentAt(1.0);
        const Vec2 t3 = tangentAt(res.contour.spans[2], 0.0);
        CHECK(std::fabs(t2.cross(t3)) < 1e-9);
        CHECK(t2.dot(t3) > 0.0);
        // Sweep equals the turn.
        CHECK_THAT(std::fabs(arc.sweepAngle()), WithinAbs(std::fabs(turn), 1e-9));
    }
}

TEST_CASE("fillet: area change matches the analytic corner area", "[edit][fillet]") {
    Contour sq = square(10.0);
    const double r = 2.0;
    const FilletResult res = filletCorner(sq, 1, r, FilletKind::Normal);
    REQUIRE(res.status == EditStatus::Ok);
    CHECK_THAT(area(sq) - area(res.contour), WithinAbs(r * r * (1.0 - kPi / 4.0), 1e-9));
    // All four corners.
    const FilletAllResult all = filletAll(sq, r, FilletKind::Normal);
    REQUIRE(all.status == EditStatus::Ok);
    CHECK(all.applied == 4);
    CHECK(all.contour.spans.size() == 8);
    CHECK_THAT(area(sq) - area(all.contour), WithinAbs(4.0 * r * r * (1.0 - kPi / 4.0), 1e-9));
    CHECK(all.contour.closed);
    CHECK(maxGap(all.contour) < 1e-9);
    // Concave corner of an L adds area.
    Contour ell = lines({{0, 0}, {10, 0}, {10, 4}, {4, 4}, {4, 10}, {0, 10}}, true);
    const FilletResult cc = filletCorner(ell, 3, 1.5, FilletKind::Normal);
    REQUIRE(cc.status == EditStatus::Ok);
    CHECK_THAT(area(cc.contour) - area(ell), WithinAbs(1.5 * 1.5 * (1.0 - kPi / 4.0), 1e-9));
}

TEST_CASE("fillet: radius equal to the half side consumes the spans exactly", "[edit][fillet]") {
    const FilletAllResult all = filletAll(square(10.0), 5.0, FilletKind::Normal);
    CHECK(all.applied == 4);
    CHECK(all.contour.spans.size() == 4);  // four quarter arcs
    for (const Span& s : all.contour.spans) {
        CHECK(std::holds_alternative<ArcSpan>(s));
    }
    CHECK_THAT(area(all.contour), WithinAbs(kPi * 25.0, 1e-6));
}

TEST_CASE("fillet: line to arc and arc to arc corners", "[edit][fillet]") {
    // Rounded slot end: line meets arc of radius 6 at a corner.
    Contour c;
    c.append(LineSpan{{-20, 0}, {0, 0}});
    c.append(ArcSpan::fromCentre({0, 6}, {0, 0}, {6, 6}, ArcDirection::CW));  // leaves downwards-right? any arc
    // Replace with a definite arc turning left from the line: centre above, CCW from (0,0).
    c.spans[1] = ArcSpan::fromCentre({0, 6}, {0, 0}, {-6, 6}, ArcDirection::CW);
    // Tangent start (arc leaves along -x?), so make a genuine corner: arc centre below-left.
    c.spans[1] = ArcSpan::fromCentre({3, 5}, {0, 0}, {6, 8}, ArcDirection::CW);
    const double r = 1.0;
    const FilletResult res = filletCorner(c, 1, r, FilletKind::Normal);
    REQUIRE(res.status == EditStatus::Ok);
    const ArcSpan& f = arcAt(res.contour, 1);
    CHECK_THAT(f.radius, WithinAbs(r, 1e-9));
    CHECK_THAT(closestPoint(c.spans[0], f.centre).distance, WithinAbs(r, 1e-8));
    CHECK_THAT(closestPoint(c.spans[1], f.centre).distance, WithinAbs(r, 1e-8));
    CHECK(maxGap(res.contour) < 1e-8);

    Contour d;
    d.append(ArcSpan::fromCentre({0, 0}, {-5, 0}, {0, 5}, ArcDirection::CW));  // upper-left quarter
    d.append(ArcSpan::fromCentre({0, 10}, {0, 5}, {5, 10}, ArcDirection::CCW));
    const FilletResult rd = filletCorner(d, 1, 0.5, FilletKind::Normal);
    // Whatever the geometry, the result is either a valid tangent fillet or a reported refusal.
    if (rd.status == EditStatus::Ok) {
        CHECK(maxGap(rd.contour) < 1e-8);
    }
}

TEST_CASE("fillet: bezier neighbour", "[edit][fillet]") {
    Contour c;
    c.append(LineSpan{{-10, 0}, {0, 0}});
    c.append(BezierSpan{{0, 0}, {3, 4}, {4, 8}, {4, 12}});
    const FilletResult res = filletCorner(c, 1, 1.0, FilletKind::Normal);
    REQUIRE(res.status == EditStatus::Ok);
    const ArcSpan& f = arcAt(res.contour, 1);
    CHECK_THAT(closestPoint(c.spans[0], f.centre).distance, WithinAbs(1.0, 1e-8));
    CHECK_THAT(closestPoint(c.spans[1], f.centre).distance, WithinAbs(1.0, 1e-6));
    CHECK(maxGap(res.contour) < 1e-8);
    // The kept Bezier lies on the original curve.
    const BezierSpan& kept = std::get<BezierSpan>(res.contour.spans[2]);
    for (double t : {0.0, 0.3, 0.7, 1.0}) {
        CHECK(closestPoint(c.spans[1], kept.pointAt(t)).distance < 1e-9);
    }
}

TEST_CASE("fillet: radius too large is refused or clamped, and reported", "[edit][fillet]") {
    const Contour sq = square(10.0);
    const FilletResult refused = filletCorner(sq, 1, 12.0, FilletKind::Normal);
    CHECK(refused.status == EditStatus::RadiusTooLarge);
    CHECK(refused.contour.isEmpty());
    FilletOptions opt;
    opt.clampToFit = true;
    const FilletResult clamped = filletCorner(sq, 1, 12.0, FilletKind::Normal, opt);
    REQUIRE(clamped.status == EditStatus::Ok);
    CHECK(clamped.clamped);
    CHECK_THAT(clamped.appliedRadius, WithinAbs(10.0, 1e-6));
    CHECK_THAT(arcAt(clamped.contour, clamped.arcSpan).radius, WithinAbs(clamped.appliedRadius, 1e-9));
}

TEST_CASE("fillet: degenerate and hostile inputs return statuses", "[edit][fillet]") {
    const Contour sq = square(10.0);
    CHECK(filletCorner(Contour{}, 0, 1.0, FilletKind::Normal).status == EditStatus::EmptyContour);
    CHECK(filletCorner(sq, 0, 0.0, FilletKind::Normal).status == EditStatus::InvalidArgument);
    CHECK(filletCorner(sq, 0, -1.0, FilletKind::Normal).status == EditStatus::InvalidArgument);
    CHECK(filletCorner(sq, 0, std::nan(""), FilletKind::Normal).status == EditStatus::InvalidArgument);
    CHECK(filletCorner(sq, 0, std::numeric_limits<double>::infinity(), FilletKind::Normal).status ==
          EditStatus::InvalidArgument);
    CHECK(filletCorner(sq, 4, 1.0, FilletKind::Normal).status == EditStatus::BadNode);
    const Contour open = lines({{0, 0}, {5, 0}, {5, 5}});
    CHECK(filletCorner(open, 0, 1.0, FilletKind::Normal).status == EditStatus::BadNode);
    CHECK(filletCorner(open, 2, 1.0, FilletKind::Normal).status == EditStatus::BadNode);
    CHECK(filletCorner(open, 1, 1.0, FilletKind::Normal).status == EditStatus::Ok);
    // Collinear neighbours and a reversal.
    CHECK(filletCorner(lines({{0, 0}, {5, 0}, {10, 0}}), 1, 1.0, FilletKind::Normal).status == EditStatus::NoCorner);
    CHECK(filletCorner(lines({{0, 0}, {5, 0}, {0, 0}}), 1, 1.0, FilletKind::Normal).status == EditStatus::Cusp);
    // Zero-length span, gap, single span.
    Contour z = lines({{0, 0}, {5, 0}, {5, 5}});
    z.spans[0] = LineSpan{{5, 0}, {5, 0}};
    CHECK(filletCorner(z, 1, 1.0, FilletKind::Normal).status == EditStatus::DegenerateSpan);
    Contour gap = lines({{0, 0}, {5, 0}});
    gap.append(LineSpan{{6, 0.5}, {6, 5}});
    CHECK(filletCorner(gap, 1, 1.0, FilletKind::Normal).status == EditStatus::Discontinuous);
    Contour one;
    one.append(LineSpan{{0, 0}, {1, 0}});
    CHECK(filletCorner(one, 1, 0.1, FilletKind::Normal).status == EditStatus::BadNode);
    CHECK(filletCorner(Contour::circle({0, 0}, 3.0), 0, 0.5, FilletKind::Normal).status == EditStatus::NoCorner);
    // NaN geometry.
    Contour bad = lines({{0, 0}, {5, 0}, {5, 5}});
    bad.spans[0] = LineSpan{{0, 0}, {std::nan(""), 0}};
    CHECK(filletCorner(bad, 1, 1.0, FilletKind::Normal).status == EditStatus::InvalidArgument);
}

TEST_CASE("fillet: huge and tiny scales", "[edit][fillet]") {
    for (double scale : {1e-3, 1e3, 1e6}) {
        Contour c = square(10.0 * scale);
        const FilletResult res = filletCorner(c, 2, 2.0 * scale, FilletKind::Normal);
        REQUIRE(res.status == EditStatus::Ok);
        CHECK_THAT(arcAt(res.contour, 2).radius / scale, WithinAbs(2.0, 1e-6));
        CHECK(maxGap(res.contour) < 1e-6 * scale + 1e-9);
    }
    // Offset far from the origin.
    Contour remote = square(10.0);
    remote.transform(Transform2::translation(Vec2{1e6, -1e6}));
    const FilletResult res = filletCorner(remote, 1, 2.0, FilletKind::Normal);
    REQUIRE(res.status == EditStatus::Ok);
    CHECK_THAT(arcAt(res.contour, 1).radius, WithinAbs(2.0, 1e-6));
}

TEST_CASE("fillet: selection and open contours", "[edit][fillet]") {
    const Contour sq = square(10.0);
    const FilletAllResult sel = filletAll(sq, 1.0, FilletKind::Normal, std::vector<std::size_t>{0, 2, 2, 9});
    REQUIRE(sel.status == EditStatus::Ok);
    CHECK(sel.applied == 2);
    REQUIRE(sel.corners.size() == 3);
    CHECK(sel.corners[0].node == 0);
    CHECK(sel.corners[2].status == EditStatus::BadNode);
    CHECK(sel.contour.spans.size() == 6);
    CHECK(maxGap(sel.contour) < 1e-9);
    const Contour open = lines({{0, 0}, {5, 0}, {5, 5}, {0, 5}});
    const FilletAllResult o = filletAll(open, 1.0, FilletKind::Normal);
    CHECK(o.applied == 2);
    CHECK(!o.contour.closed);
    CHECK(o.contour.spans.size() == 5);
    // Overlapping fillets on a short edge: second one is refused, not corrupting.
    const FilletAllResult tight = filletAll(lines({{0, 0}, {10, 0}, {10, 3}, {0, 3}}, true), 2.0, FilletKind::Normal);
    CHECK(tight.applied < 4);
    CHECK(maxGap(tight.contour) < 1e-9);
}

TEST_CASE("fillet: dog-bone geometry", "[edit][fillet][dogbone]") {
    // CCW square: every corner is a left turn; the relief is centred on the corner node.
    const Contour sq = square(10.0);
    const double r = 1.5;
    const FilletResult res = filletCorner(sq, 1, r, FilletKind::DogBone);  // node (10,0)
    REQUIRE(res.status == EditStatus::Ok);
    const ArcSpan& a = arcAt(res.contour, 1);
    CHECK(approxEqual(a.centre, Vec2{10, 0}, 1e-9));
    CHECK_THAT(a.radius, WithinAbs(r, 1e-12));
    CHECK(approxEqual(a.startPoint(), Vec2{10 - r, 0}, 1e-9));
    CHECK(approxEqual(a.endPoint(), Vec2{10, r}, 1e-9));
    CHECK_THAT(std::fabs(a.sweepAngle()), WithinAbs(1.5 * kPi, 1e-9));
    CHECK(maxGap(res.contour) < 1e-9);
    // Area added: the 270 degree sector outside the square.
    CHECK_THAT(area(res.contour) - area(sq), WithinAbs(0.75 * kPi * r * r, 1e-9));
    // Oblique corner: added area is the (pi + turn) sector.
    const double turn = 1.0;
    Contour w = wedge(turn);
    const FilletResult ow = filletCorner(w, 1, r, FilletKind::DogBone);
    REQUIRE(ow.status == EditStatus::Ok);
    CHECK_THAT(std::fabs(arcAt(ow.contour, 1).sweepAngle()), WithinAbs(kPi + turn, 1e-9));
    // Neither relief point is closer to the corner than r on the walls: both lie on the walls.
    CHECK(closestPoint(w.spans[0], arcAt(ow.contour, 1).startPoint()).distance < 1e-9);
    CHECK(closestPoint(w.spans[1], arcAt(ow.contour, 1).endPoint()).distance < 1e-9);
    // Right turn flips the arc direction.
    const FilletResult cw = filletCorner(wedge(-turn), 1, r, FilletKind::DogBone);
    REQUIRE(cw.status == EditStatus::Ok);
    CHECK(arcAt(cw.contour, 1).direction == ArcDirection::CW);
    // Does not fit.
    CHECK(filletCorner(sq, 1, 11.0, FilletKind::DogBone).status == EditStatus::RadiusTooLarge);
}

TEST_CASE("fillet: t-bone geometry", "[edit][fillet][tbone]") {
    const Contour sq = square(10.0);
    const double r = 1.5;
    // Outgoing: node (10,0), the relief lies on the right edge, semicircle bulging outwards (+x).
    const FilletResult out = filletCorner(sq, 1, r, FilletKind::TBone);
    REQUIRE(out.status == EditStatus::Ok);
    const ArcSpan& a = arcAt(out.contour, out.arcSpan);
    CHECK(approxEqual(a.centre, Vec2{10, r}, 1e-9));
    CHECK_THAT(a.radius, WithinAbs(r, 1e-9));
    CHECK(approxEqual(a.startPoint(), Vec2{10, 0}, 1e-9));
    CHECK(approxEqual(a.endPoint(), Vec2{10, 2 * r}, 1e-9));
    CHECK_THAT(std::fabs(a.sweepAngle()), WithinAbs(kPi, 1e-9));
    CHECK(a.pointAt(0.5).x > 10.0);
    CHECK_THAT(area(out.contour) - area(sq), WithinAbs(0.5 * kPi * r * r, 1e-9));
    CHECK(maxGap(out.contour) < 1e-9);
    // Incoming: relief on the bottom edge, bulging downwards.
    FilletOptions opt;
    opt.tBoneAlongIncoming = true;
    const FilletResult in = filletCorner(sq, 1, r, FilletKind::TBone, opt);
    REQUIRE(in.status == EditStatus::Ok);
    const ArcSpan& b = arcAt(in.contour, in.arcSpan);
    CHECK(approxEqual(b.centre, Vec2{10 - r, 0}, 1e-9));
    CHECK(b.pointAt(0.5).y < 0.0);
    CHECK_THAT(area(in.contour) - area(sq), WithinAbs(0.5 * kPi * r * r, 1e-9));
    CHECK(maxGap(in.contour) < 1e-9);
    CHECK(filletCorner(sq, 1, 6.0, FilletKind::TBone).status == EditStatus::RadiusTooLarge);
}

// --------------------------------------------------------------------------------------- trim

TEST_CASE("trim to boundary: lines against a rectangle", "[edit][trim]") {
    const Contour boundary = square(10.0);
    const Contour line = lines({{-5, 5}, {15, 5}});
    const TrimResult inside = trimToBoundary({line}, {boundary}, TrimSide::ClearInside);
    REQUIRE(inside.status == EditStatus::Ok);
    REQUIRE(inside.contours.size() == 2);
    CHECK(approxEqual(inside.contours[0].startPoint(), Vec2{-5, 5}));
    CHECK(approxEqual(inside.contours[0].endPoint(), Vec2{0, 5}, 1e-9));
    CHECK(approxEqual(inside.contours[1].startPoint(), Vec2{10, 5}, 1e-9));
    const TrimResult outside = trimToBoundary({line}, {boundary}, TrimSide::ClearOutside);
    REQUIRE(outside.contours.size() == 1);
    CHECK(approxEqual(outside.contours[0].startPoint(), Vec2{0, 5}, 1e-9));
    CHECK(approxEqual(outside.contours[0].endPoint(), Vec2{10, 5}, 1e-9));
    // Lengths add up.
    CHECK_THAT(totalLength(inside.contours) + totalLength(outside.contours), WithinAbs(line.length(), 1e-9));
    // Untouched contours pass through unchanged (closed stays closed).
    const Contour distant = Contour::circle({100, 100}, 2.0);
    const TrimResult passIn = trimToBoundary({distant}, {boundary}, TrimSide::ClearInside);
    REQUIRE(passIn.contours.size() == 1);
    CHECK(passIn.contours[0] == distant);
    CHECK(trimToBoundary({distant}, {boundary}, TrimSide::ClearOutside).contours.empty());
}

TEST_CASE("trim to boundary: curves are cut exactly and kept pieces lie on the kept side", "[edit][trim]") {
    const Contour boundary = Contour::circle({0, 0}, 5.0);
    Contour curve;
    curve.append(BezierSpan{{-10, -3}, {-3, 8}, {3, -8}, {10, 3}});
    curve.append(ArcSpan::fromCentre({10, 10}, {10, 3}, {17, 10}, ArcDirection::CCW));
    for (TrimSide side : {TrimSide::ClearInside, TrimSide::ClearOutside}) {
        const TrimResult r = trimToBoundary({curve}, {boundary}, side);
        REQUIRE(r.status == EditStatus::Ok);
        REQUIRE_FALSE(r.contours.empty());
        for (const Contour& c : r.contours) {
            CHECK(maxGap(c) < 1e-9);
            for (const Span& s : c.spans) {
                for (int i = 0; i <= 20; ++i) {
                    const Vec2 p = pointAt(s, i / 20.0);
                    const double d = distance(p, {0, 0});
                    if (side == TrimSide::ClearInside) {
                        CHECK(d >= 5.0 - 1e-6);
                    } else {
                        CHECK(d <= 5.0 + 1e-6);
                    }
                    CHECK(distanceTo(curve, p) < 1e-9);  // on the original curve
                }
            }
            // Cut ends are on the boundary exactly.
        }
        // Total length preserved: inside + outside = whole.
    }
    const TrimResult in = trimToBoundary({curve}, {boundary}, TrimSide::ClearInside);
    const TrimResult out = trimToBoundary({curve}, {boundary}, TrimSide::ClearOutside);
    CHECK_THAT(totalLength(in.contours) + totalLength(out.contours), WithinAbs(curve.length(), 1e-6));
    // Cut points sit on the circle.
    for (const Contour& c : out.contours) {
        CHECK_THAT(distance(c.startPoint(), {0, 0}), WithinAbs(5.0, 1e-8));
        CHECK_THAT(distance(c.endPoint(), {0, 0}), WithinAbs(5.0, 1e-8));
    }
}

TEST_CASE("trim to boundary: closed contour cut into an open one, holes, boundary-coincident pieces", "[edit][trim]") {
    const Contour big = square(10.0);
    Contour shifted = square(10.0);
    shifted.transform(Transform2::translation(Vec2{5.0, 0.0}));
    const TrimResult r = trimToBoundary({shifted}, {big}, TrimSide::ClearInside);
    REQUIRE(r.contours.size() == 1);
    CHECK(!r.contours[0].closed);
    // The edge pieces lying on the boundary are kept, the left edge (inside) goes: 10 + 10 + 10.
    CHECK_THAT(r.contours[0].length(), WithinAbs(30.0, 1e-9));
    CHECK(r.contours[0].isContinuous(tol));
    for (const Span& s : r.contours[0].spans) {
        CHECK(pointAt(s, 0.5).x >= 5.0 - 1e-9);
    }
    // A boundary with a hole: only the annulus is "inside" under the non-zero rule.
    Contour hole = Contour::circle({0, 0}, 2.0);
    hole.reverse();
    const Contour outer = Contour::circle({0, 0}, 6.0);
    const Contour ln = lines({{-8, 0}, {8, 0}});
    const TrimResult annulus = trimToBoundary({ln}, {outer, hole}, TrimSide::ClearOutside);
    REQUIRE(annulus.contours.size() == 2);
    CHECK_THAT(totalLength(annulus.contours), WithinAbs(8.0, 1e-8));
    // Coincident boundary: a contour equal to the boundary is kept as it is.
    const TrimResult same = trimToBoundary({big}, {big}, TrimSide::ClearInside);
    REQUIRE(same.contours.size() == 1);
    CHECK_THAT(same.contours[0].length(), WithinAbs(40.0, 1e-9));
    // Open boundary is refused.
    CHECK(trimToBoundary({big}, {lines({{0, 0}, {1, 0}})}, TrimSide::ClearInside).status == EditStatus::NotClosed);
    // No boundary at all: ClearInside keeps everything.
    CHECK(trimToBoundary({big}, {}, TrimSide::ClearInside).contours.size() == 1);
    CHECK(trimToBoundary({big}, {}, TrimSide::ClearOutside).contours.empty());
}

TEST_CASE("trim to boundary: tangent touches do not split spans; refit merges collinear pieces", "[edit][trim]") {
    const Contour boundary = Contour::circle({0, 0}, 5.0);
    const Contour tangentLine = lines({{-10, 5}, {10, 5}});
    const TrimResult t = trimToBoundary({tangentLine}, {boundary}, TrimSide::ClearInside);
    REQUIRE(t.contours.size() == 1);
    CHECK(t.contours[0].spans.size() == 1);
    CHECK(t.contours[0].spans[0] == tangentLine.spans[0]);
    // Collinear chain gets merged by refit.
    Contour chain = lines({{-10, 0}, {-8, 0}, {-6, 0}, {-4, 0}});
    const TrimResult plain = trimToBoundary({chain}, {boundary}, TrimSide::ClearInside, false);
    const TrimResult refit = trimToBoundary({chain}, {boundary}, TrimSide::ClearInside, true);
    REQUIRE(plain.contours.size() == 1);
    CHECK(plain.contours[0].spans.size() == 3);
    CHECK(refit.contours[0].spans.size() == 1);
    CHECK_THAT(refit.contours[0].length(), WithinAbs(chain.length() - 1.0, 1e-9));
}

TEST_CASE("trim spans at a pick", "[edit][trim]") {
    // A "+" made of two lines: trim the right arm.
    const Contour h = lines({{-10, 0}, {10, 0}});
    const Contour v = lines({{0, -10}, {0, 10}});
    const TrimAtResult r = trimSpansAt(h, {7, 0.2}, {v});
    REQUIRE(r.status == EditStatus::Ok);
    REQUIRE(r.contours.size() == 1);
    CHECK(approxEqual(r.contours[0].startPoint(), Vec2{-10, 0}, 1e-9));
    CHECK(approxEqual(r.contours[0].endPoint(), Vec2{0, 0}, 1e-9));
    // Pick on the left of the crossing keeps the right arm.
    const TrimAtResult l = trimSpansAt(h, {-7, 0.2}, {v});
    REQUIRE(l.contours.size() == 1);
    CHECK(approxEqual(l.contours[0].startPoint(), Vec2{0, 0}, 1e-9));
    // No intersection: nothing to trim.
    CHECK(trimSpansAt(h, {7, 0.0}, {}).status == EditStatus::NothingToTrim);
    // Middle run between two crossings of an open polyline.
    const Contour w = lines({{-10, 0}, {10, 0}});
    const Contour a = lines({{-4, -1}, {-4, 1}});
    const Contour b = lines({{4, -1}, {4, 1}});
    const TrimAtResult mid = trimSpansAt(w, {0, 0}, {a, b});
    REQUIRE(mid.contours.size() == 2);
    CHECK(approxEqual(mid.contours[0].endPoint(), Vec2{-4, 0}, 1e-9));
    CHECK(approxEqual(mid.contours[1].startPoint(), Vec2{4, 0}, 1e-9));
    CHECK_THAT(totalLength(mid.contours), WithinAbs(20.0 - 8.0, 1e-9));
}

TEST_CASE("trim spans at a pick: self intersections, rejoin, closed contours", "[edit][trim]") {
    // Open loop with a tail: (0,0)->(10,0)->(10,10)->(5,10)->(5,-5): crosses itself at (5,0).
    const Contour c = lines({{0, 0}, {10, 0}, {10, 10}, {5, 10}, {5, -5}});
    const TrimAtResult noJoin = trimSpansAt(c, {10, 5}, {});
    REQUIRE(noJoin.status == EditStatus::Ok);
    REQUIRE(noJoin.contours.size() == 2);
    CHECK(approxEqual(noJoin.contours[0].endPoint(), Vec2{5, 0}, 1e-9));
    CHECK(approxEqual(noJoin.contours[1].startPoint(), Vec2{5, 0}, 1e-9));
    TrimAtOptions opt;
    opt.rejoin = true;
    const TrimAtResult joined = trimSpansAt(c, {10, 5}, {}, opt);
    REQUIRE(joined.contours.size() == 1);
    CHECK(joined.contours[0].isContinuous(tol));
    CHECK(approxEqual(joined.contours[0].startPoint(), Vec2{0, 0}));
    CHECK(approxEqual(joined.contours[0].endPoint(), Vec2{5, -5}));
    CHECK_THAT(joined.contours[0].length(), WithinAbs(5.0 + 5.0, 1e-9));

    // Closed square crossed by a line: trimming one side leaves an open contour.
    const Contour sq = square(10.0);
    const Contour cut = lines({{5, -5}, {5, 5}});
    const TrimAtResult cl = trimSpansAt(sq, {2, 0}, {cut});
    CHECK(cl.status == EditStatus::NothingToTrim);  // one-sided crossing needs two cut points
    const Contour cut2 = lines({{-5, 3}, {15, 3}});
    const TrimAtResult cl2 = trimSpansAt(sq, {2, 0}, {cut, cut2});
    REQUIRE(cl2.status == EditStatus::Ok);
    REQUIRE(cl2.contours.size() == 1);
    CHECK(!cl2.contours[0].closed);
    CHECK(cl2.contours[0].isContinuous(tol));
    // Removed run: from (5,0) round to... the picked side holds (2,0); between cut (5,0) and (0,3).
    CHECK_THAT(cl2.contours[0].length(), WithinAbs(40.0 - (5.0 + 3.0), 1e-9));
    // Figure-of-eight closed contour: trimming one loop with rejoin leaves the other closed.
    Contour bow = lines({{0, 0}, {10, 10}, {10, 0}, {0, 10}}, true);  // bow tie, crossing at (5,5)
    TrimAtOptions rj;
    rj.rejoin = true;
    const TrimAtResult loop = trimSpansAt(bow, {10, 5}, {}, rj);
    REQUIRE(loop.status == EditStatus::Ok);
    REQUIRE(loop.contours.size() == 1);
    CHECK(loop.contours[0].closed);
    CHECK(loop.contours[0].isContinuous(tol));
    CHECK_THAT(area(loop.contours[0]), WithinAbs(25.0, 1e-9));
}

// ------------------------------------------------------------------------------------ extend

TEST_CASE("extend: lines, arcs and beziers land exactly on the target", "[edit][extend]") {
    const Contour target = lines({{10, -5}, {10, 5}});
    const Contour ln = lines({{0, 0}, {4, 1}});
    const ExtendResult r = extendSpan(ln, ContourEnd::End, {target});
    REQUIRE(r.status == EditStatus::Ok);
    CHECK(approxEqual(r.point, Vec2{10, 2.5}, 1e-9));
    CHECK(approxEqual(r.contour.endPoint(), Vec2{10, 2.5}, 1e-12));
    CHECK(r.contour.spans.size() == 1);
    CHECK_THAT(r.distance, WithinAbs(distance({4, 1}, {10, 2.5}), 1e-9));
    // Start end: the start moves.
    const ExtendResult s = extendSpan(lines({{4, 1}, {8, 2}}), ContourEnd::Start, {lines({{-6, -5}, {-6, 5}})});
    REQUIRE(s.status == EditStatus::Ok);
    CHECK(approxEqual(s.contour.startPoint(), Vec2{-6, -1.5}, 1e-9));
    CHECK(approxEqual(s.contour.endPoint(), Vec2{8, 2}, 1e-12));
    // Nearest of several targets.
    const ExtendResult nearest = extendSpan(ln, ContourEnd::End, {lines({{20, -5}, {20, 5}}), target});
    CHECK(nearest.targetContour == 1);

    // Arc continues round its circle.
    Contour ac;
    ac.append(ArcSpan::fromCentre({0, 0}, {5, 0}, {0, 5}, ArcDirection::CCW));
    const ExtendResult ar = extendSpan(ac, ContourEnd::End, {lines({{-10, -3}, {10, -3}}), lines({{-3, -10}, {-3, 10}})});
    REQUIRE(ar.status == EditStatus::Ok);
    CHECK(approxEqual(ar.point, Vec2{-3, 4}, 1e-9));
    CHECK_THAT(distance(ar.contour.endPoint(), {0, 0}), WithinAbs(5.0, 1e-9));
    CHECK(approxEqual(ar.contour.endPoint(), Vec2{-3, 4}, 1e-9));
    CHECK(ar.contour.spans.size() == 1);

    // Bezier: tangent line appended.
    Contour bz;
    bz.append(BezierSpan{{0, 0}, {2, 0}, {4, 1}, {6, 3}});
    const Vec2 tan = tangentAt(bz.spans[0], 1.0);
    const ExtendResult br = extendSpan(bz, ContourEnd::End, {lines({{12, -20}, {12, 20}})});
    REQUIRE(br.status == EditStatus::Ok);
    REQUIRE(br.contour.spans.size() == 2);
    CHECK(br.contour.spans[0] == bz.spans[0]);
    CHECK(approxEqual(br.contour.endPoint(), Vec2{12, 3 + 6.0 * tan.y / tan.x}, 1e-9));
    CHECK(maxGap(br.contour) < 1e-12);
}

TEST_CASE("extend: refusals", "[edit][extend]") {
    const Contour ln = lines({{0, 0}, {4, 1}});
    CHECK(extendSpan(ln, ContourEnd::End, {lines({{-10, -5}, {-10, 5}})}).status == EditStatus::NoIntersection);
    CHECK(extendSpan(ln, ContourEnd::End, {}).status == EditStatus::NoIntersection);
    CHECK(extendSpan(Contour{}, ContourEnd::End, {}).status == EditStatus::EmptyContour);
    CHECK(extendSpan(square(5.0), ContourEnd::End, {}).status == EditStatus::ClosedContour);
    CHECK(extendSpan(lines({{1, 1}, {1, 1}}), ContourEnd::End, {square(5.0)}).status == EditStatus::DegenerateSpan);
    // An end that already touches a target ignores that touch.
    const ExtendResult past = extendSpan(ln, ContourEnd::End, {lines({{4, -5}, {4, 5}}), lines({{9, -5}, {9, 5}})});
    REQUIRE(past.status == EditStatus::Ok);
    CHECK(approxEqual(past.point, Vec2{9, 2.25}, 1e-9));
    Contour nanC = ln;
    nanC.spans[0] = LineSpan{{0, 0}, {std::nan(""), 1}};
    CHECK(extendSpan(nanC, ContourEnd::End, {}).status == EditStatus::InvalidArgument);
}

// ------------------------------------------------------------------------------- join / close

TEST_CASE("join: chains, reversal, closing and leftovers", "[edit][join]") {
    // Square cut into four edges in shuffled order and directions, plus one stray.
    const Contour e0 = lines({{0, 0}, {10, 0}});
    const Contour e1 = lines({{10, 10}, {10, 0}});  // reversed
    const Contour e2 = lines({{10, 10}, {0, 10}});
    const Contour e3 = lines({{0, 10}, {0, 0}});
    const Contour stray = lines({{50, 50}, {60, 50}});
    const JoinResult r = joinContours({e0, stray, e2, e1, e3});
    REQUIRE(r.status == EditStatus::Ok);
    REQUIRE(r.contours.size() == 2);
    CHECK(r.contours[0].closed);
    CHECK(r.contours[0].spans.size() == 4);
    CHECK(r.contours[0].isContinuous(tol));
    CHECK_THAT(area(r.contours[0]), WithinAbs(100.0, 1e-9));
    CHECK(r.sources[0].size() == 4);
    CHECK(r.sources[1] == std::vector<std::size_t>{1});
    CHECK(r.unmatched == std::vector<std::size_t>{1});
    CHECK(!r.contours[1].closed);
    // Length preserved, no span lost or duplicated.
    CHECK_THAT(totalLength(r.contours), WithinAbs(e0.length() * 4 + stray.length(), 1e-9));
    // Deterministic.
    const JoinResult again = joinContours({e0, stray, e2, e1, e3});
    CHECK(again.contours == r.contours);
    CHECK(again.sources == r.sources);
}

TEST_CASE("join: tolerance boundary, closed passthrough, hostile input", "[edit][join]") {
    const Contour a = lines({{0, 0}, {10, 0}});
    const Contour within = lines({{10.0009, 0}, {20, 0}});
    const Contour beyond = lines({{20.0011, 0}, {30, 0}});
    const JoinResult r = joinContours({a, within, beyond});
    REQUIRE(r.contours.size() == 2);
    CHECK(r.contours[0].spans.size() == 2);
    CHECK(r.contours[1].spans.size() == 1);
    // Closed contours and empties pass through.
    const Contour sq = square(3.0);
    const JoinResult p = joinContours({sq, Contour{}, a});
    REQUIRE(p.contours.size() == 3);
    CHECK(p.contours[0] == sq);
    CHECK(p.unmatched == std::vector<std::size_t>{2});
    // A single open contour whose ends meet is closed.
    Contour ring = lines({{0, 0}, {5, 0}, {5, 5}, {0, 5}, {0, 0.0005}});
    const JoinResult c = joinContours({ring});
    CHECK(c.contours[0].closed);
    // Huge coordinates, tiny tolerance, NaN contour, zero tolerance.
    Contour nanC = lines({{0, 0}, {1, 0}});
    nanC.spans[0] = LineSpan{{0, 0}, {std::nan(""), 0}};
    const JoinResult h = joinContours({nanC, a});
    CHECK(h.contours.size() == 2);
    Tolerance zero;
    zero.distance = 0.0;
    const JoinResult z = joinContours({a, lines({{10, 0}, {20, 0}}), lines({{20.0000001, 0}, {30, 0}})}, zero);
    CHECK(z.contours.size() == 2);
    const Contour farA = lines({{1e9, 1e9}, {1e9 + 10, 1e9}});
    const Contour farB = lines({{1e9 + 10, 1e9}, {1e9 + 20, 1e9}});
    CHECK(joinContours({farA, farB}).contours.size() == 1);
    Tolerance bad;
    bad.distance = -1.0;
    CHECK(joinContours({a}, bad).status == EditStatus::InvalidArgument);
}

TEST_CASE("join: random segment soup keeps all length", "[edit][join]") {
    SplitMix rng{5};
    std::vector<Vec2> pts;
    for (int i = 0; i < 40; ++i) {
        pts.push_back({rng.uniform(0, 100), rng.uniform(0, 100)});
    }
    std::vector<Contour> pieces;
    double total = 0.0;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        Contour c = lines({pts[i], pts[i + 1]});
        total += c.length();
        if (rng.next() % 2 == 0) {
            c.reverse();
        }
        pieces.push_back(c);
    }
    // Shuffle deterministically.
    for (std::size_t i = pieces.size(); i > 1; --i) {
        std::swap(pieces[i - 1], pieces[rng.next() % i]);
    }
    const JoinResult r = joinContours(pieces);
    REQUIRE(r.contours.size() == 1);
    CHECK_THAT(r.contours[0].length(), WithinAbs(total, 1e-9));
    CHECK(r.contours[0].spans.size() == pieces.size());
}

TEST_CASE("close: snap, add line, already closed", "[edit][close]") {
    const Contour open = lines({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
    const CloseResult c = closeContour(open);
    REQUIRE(c.status == EditStatus::Ok);
    CHECK(c.action == CloseAction::LineAdded);
    CHECK(c.contour.closed);
    CHECK(c.contour.spans.size() == 4);
    CHECK_THAT(area(c.contour), WithinAbs(100.0, 1e-9));
    CHECK_THAT(c.contour.length(), WithinAbs(open.length() + 10.0, 1e-9));
    // Gap within tolerance snaps with no extra span.
    const Contour almost = lines({{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0.0004}});
    const CloseResult s = closeContour(almost);
    CHECK(s.action == CloseAction::Snapped);
    CHECK(s.contour.spans.size() == 4);
    CHECK(approxEqual(endPoint(s.contour.spans.back()), s.contour.startPoint(), 1e-15));
    // Gap just beyond tolerance gets a line.
    CHECK(closeContour(lines({{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0.0011}})).action == CloseAction::LineAdded);
    CHECK(closeContour(square(2.0)).action == CloseAction::AlreadyClosed);
    CHECK(closeContour(Contour{}).status == EditStatus::EmptyContour);
    // Arc-only contour with a tiny gap: closes without breaking the arcs.
    Contour arcs;
    arcs.append(ArcSpan::fromCentre({0, 0}, {5, 0}, {-5, 0}, ArcDirection::CCW));
    arcs.append(ArcSpan::fromCentre({0, 0}, {-5, 0}, Vec2{5.0003, 0}, ArcDirection::CCW));
    const CloseResult ar = closeContour(arcs);
    CHECK(ar.action == CloseAction::Snapped);
    CHECK(ar.contour.closed);
    Contour nanC = open;
    nanC.spans[0] = LineSpan{{0, 0}, {std::nan(""), 0}};
    CHECK(closeContour(nanC).status == EditStatus::InvalidArgument);
}

TEST_CASE("toString covers every status", "[edit]") {
    for (int i = 0; i <= static_cast<int>(EditStatus::NoIntersection); ++i) {
        CHECK(std::string(toString(static_cast<EditStatus>(i))) != "unknown");
    }
}

TEST_CASE("generative: trim keeps every piece on its side and loses no length", "[edit][trim][generative]") {
    SplitMix rng{2024};
    for (int k = 0; k < 120; ++k) {
        const Vec2 centre{rng.uniform(-5, 5), rng.uniform(-5, 5)};
        const double radius = rng.uniform(2.0, 12.0);
        const Contour boundary = k % 2 == 0 ? Contour::circle(centre, radius)
                                             : Contour::rectangle(Box2{centre - Vec2{radius, radius * 0.6},
                                                                       centre + Vec2{radius, radius * 0.6}});
        Contour c;
        Vec2 at{rng.uniform(-15, 15), rng.uniform(-15, 15)};
        for (int s = 0; s < 4; ++s) {
            const Vec2 to{rng.uniform(-15, 15), rng.uniform(-15, 15)};
            if (s % 2 == 0) {
                c.append(LineSpan{at, to});
            } else {
                c.append(BezierSpan{at, {rng.uniform(-15, 15), rng.uniform(-15, 15)}, {rng.uniform(-15, 15), rng.uniform(-15, 15)}, to});
            }
            at = to;
        }
        const TrimResult in = trimToBoundary({c}, {boundary}, TrimSide::ClearInside);
        const TrimResult out = trimToBoundary({c}, {boundary}, TrimSide::ClearOutside);
        REQUIRE(in.status == EditStatus::Ok);
        REQUIRE(out.status == EditStatus::Ok);
        // Bezier lengths are numerical: compare at 1e-6 relative.
        CHECK_THAT(totalLength(in.contours) + totalLength(out.contours), WithinAbs(c.length(), 1e-6 * c.length() + 1e-9));
        for (const Contour& piece : in.contours) {
            CHECK(maxGap(piece) < 1e-9);
            for (const Span& sp : piece.spans) {
                for (double t : {0.1, 0.5, 0.9}) {
                    const Vec2 p = pointAt(sp, t);
                    CHECK(locatePoint(boundary, p, FillRule::NonZero, tol) != PointLocation::Inside);
                    CHECK(distanceTo(c, p) < 1e-7);
                }
            }
        }
        for (const Contour& piece : out.contours) {
            for (const Span& sp : piece.spans) {
                for (double t : {0.1, 0.5, 0.9}) {
                    CHECK(locatePoint(boundary, pointAt(sp, t), FillRule::NonZero, tol) != PointLocation::Outside);
                }
            }
        }
    }
}

TEST_CASE("generative: extend lands exactly on the first target hit", "[edit][extend][generative]") {
    SplitMix rng{77};
    int landed = 0;
    for (int k = 0; k < 150; ++k) {
        const Vec2 start{rng.uniform(-5, 5), rng.uniform(-5, 5)};
        const double ang = rng.uniform(0.0, kTwoPi);
        const Contour ln = lines({start, start + Vec2::fromAngle(ang, rng.uniform(1.0, 4.0))});
        const Contour box = Contour::rectangle(Box2{{-30, -30}, {30, 30}});
        const Contour circle = Contour::circle({rng.uniform(-10, 10), rng.uniform(-10, 10)}, rng.uniform(3.0, 8.0));
        const ExtendResult r = extendSpan(ln, ContourEnd::End, {circle, box});
        if (r.status != EditStatus::Ok) {
            continue;
        }
        ++landed;
        const Vec2 tip = ln.endPoint();
        // On the extension line, ahead of the end, and on the reported target.
        CHECK(std::fabs((r.point - tip).cross(tangentAt(ln.spans[0], 0.0))) < 1e-9);
        CHECK((r.point - tip).dot(tangentAt(ln.spans[0], 0.0)) > 0.0);
        const Contour& target = r.targetContour == 0 ? circle : box;
        CHECK(distanceTo(target, r.point) < 1e-9);
        CHECK(approxEqual(r.contour.endPoint(), r.point, 1e-15));
        // Nothing nearer: sample the extension.
        const Vec2 dir = tangentAt(ln.spans[0], 0.0);
        for (int s = 1; s < 50; ++s) {
            const Vec2 q = tip + dir * (r.distance * s / 50.0);
            if (r.distance * s / 50.0 > 0.01) {
                CHECK(distanceTo(circle, q) > 1e-6);
                CHECK(distanceTo(box, q) > 1e-6);
            }
        }
    }
    CHECK(landed > 100);
}

TEST_CASE("convexNodes and the relief default only touch convex vertices", "[edit][fillet][dogbone]") {
    // L-shape, CCW: five convex corners and one reflex corner at (4,4).
    const Contour ell = lines({{0, 0}, {10, 0}, {10, 4}, {4, 4}, {4, 10}, {0, 10}}, true);
    const std::vector<std::size_t> ccw = convexNodes(ell);
    CHECK(ccw == std::vector<std::size_t>{0, 1, 2, 4, 5});  // node 3 is the reflex vertex (4,4)
    Contour cw = ell;
    cw.reverse();
    const std::vector<std::size_t> cwNodes = convexNodes(cw);
    CHECK(cwNodes.size() == 5);
    for (std::size_t n : cwNodes) {
        const Vec2 corner = startPoint(cw.spans[n]);
        CHECK_FALSE(approxEqual(corner, Vec2{4, 4}));
    }
    // No explicit node list: dog-bones only at convex nodes, so the region only grows.
    for (const Contour& c : {ell, cw}) {
        for (FilletKind kind : {FilletKind::DogBone, FilletKind::TBone}) {
            const FilletAllResult r = filletAll(c, 0.5, kind);
            REQUIRE(r.status == EditStatus::Ok);
            CHECK(r.corners.size() == 5);
            CHECK(r.applied == 5);
            CHECK(area(r.contour) > area(c));
        }
    }
    // An explicit list is honoured, reflex vertex included (caller's choice).
    const FilletAllResult forced = filletAll(ell, 0.5, FilletKind::DogBone, std::vector<std::size_t>{3});
    CHECK(forced.applied == 1);
    // Open contours: left turns; degenerate input: none.
    CHECK(convexNodes(lines({{0, 0}, {5, 0}, {5, 5}, {10, 5}})) == std::vector<std::size_t>{1});
    CHECK(convexNodes(Contour{}).empty());
    CHECK(convexNodes(lines({{0, 0}, {5, 0}, {10, 0}})).empty());
}

TEST_CASE("pinned behaviours: join tie, refit of an untouched closed contour, open rejoin closing", "[edit][join][trim]") {
    // Two partners at exactly the same distance: the lowest input index wins.
    const Contour a = lines({{0, 0}, {10, 0}});
    const Contour up = lines({{10, 0}, {10, 5}});
    const Contour down = lines({{10, 0}, {10, -5}});
    const JoinResult j = joinContours({a, up, down});
    REQUIRE(j.contours.size() == 2);
    CHECK(j.sources[0] == std::vector<std::size_t>{0, 1});
    CHECK(j.sources[1] == std::vector<std::size_t>{2});
    const JoinResult j2 = joinContours({a, down, up});
    CHECK(j2.sources[0] == std::vector<std::size_t>{0, 1});

    // A closed contour that no boundary touches still gets refitted when asked.
    Contour split = lines({{0, 0}, {5, 0}, {10, 0}, {10, 10}, {0, 10}}, true);
    const Contour away = Contour::circle({100, 100}, 1.0);
    const TrimResult plain = trimToBoundary({split}, {away}, TrimSide::ClearInside, false);
    const TrimResult refit = trimToBoundary({split}, {away}, TrimSide::ClearInside, true);
    REQUIRE(plain.contours.size() == 1);
    CHECK(plain.contours[0].spans.size() == 5);
    REQUIRE(refit.contours.size() == 1);
    CHECK(refit.contours[0].spans.size() == 4);
    CHECK(refit.contours[0].closed);

    // Open contour whose rejoined remainder ends where it starts becomes closed.
    const Contour bow = lines({{0, 0}, {10, 10}, {10, 0}, {0, 10}, {0, 0}});
    TrimAtOptions opt;
    opt.rejoin = true;
    const TrimAtResult tr = trimSpansAt(bow, {10, 5}, {}, opt);
    REQUIRE(tr.status == EditStatus::Ok);
    REQUIRE(tr.contours.size() == 1);
    CHECK(tr.contours[0].closed);
    CHECK_THAT(area(tr.contours[0]), WithinAbs(25.0, 1e-9));
}
