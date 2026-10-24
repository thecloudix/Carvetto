#include "carvetto/geometry/span.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <variant>

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
bool near(Vec2 a, Vec2 b, double tol = 1e-9) {
    return approxEqual(a, b, tol);
}

// Brute-force chord-sum estimate of a Bezier's length.
double chordLength(const BezierSpan& b, int segments) {
    double total = 0.0;
    Vec2 prev = b.p0;
    for (int i = 1; i <= segments; ++i) {
        const Vec2 p = b.pointAt(static_cast<double>(i) / static_cast<double>(segments));
        total += distance(prev, p);
        prev = p;
    }
    return total;
}
}  // namespace

// ------------------------------------------------------------------- Line

TEST_CASE("LineSpan basics", "[geometry][span]") {
    const LineSpan l{{0.0, 0.0}, {3.0, 4.0}};
    CHECK_THAT(l.length(), WithinAbs(5.0, 1e-12));
    CHECK(l.pointAt(0.5) == Vec2{1.5, 2.0});
    CHECK(near(l.tangentAt(0.3), {0.6, 0.8}));
    CHECK(l.reversed() == LineSpan{{3.0, 4.0}, {0.0, 0.0}});
    CHECK(l.bounds() == Box2{{0.0, 0.0}, {3.0, 4.0}});
    CHECK(LineSpan{{1.0, 1.0}, {1.0, 1.0}}.tangentAt(0.0) == Vec2{});
    const LineSpan t = l.transformed(Transform2::translation({1.0, 1.0}));
    CHECK(t == LineSpan{{1.0, 1.0}, {4.0, 5.0}});
}

// -------------------------------------------------------------------- Arc

// Acceptance criterion 2 (full circle length).
TEST_CASE("ArcSpan full circle r=10 has length 2*pi*10", "[geometry][span][acceptance]") {
    const ArcSpan a = ArcSpan::fullCircle({0.0, 0.0}, 10.0);
    CHECK(a.isFullCircle());
    CHECK_THAT(a.length(), WithinAbs(kTwoPi * 10.0, 1e-9));
    CHECK_THAT(a.sweepAngle(), WithinAbs(kTwoPi, 1e-12));
    const ArcSpan cw = ArcSpan::fullCircle({0.0, 0.0}, 10.0, ArcDirection::CW);
    CHECK_THAT(cw.length(), WithinAbs(kTwoPi * 10.0, 1e-9));
    CHECK_THAT(cw.sweepAngle(), WithinAbs(-kTwoPi, 1e-12));
}

// Acceptance criterion 2 (pointAt on a CCW half circle).
TEST_CASE("ArcSpan pointAt(0.5) on CCW half circle (10,0)->(-10,0) is (0,10)",
          "[geometry][span][acceptance]") {
    const ArcSpan a = ArcSpan::fromCentre({0.0, 0.0}, {10.0, 0.0}, {-10.0, 0.0}, ArcDirection::CCW);
    CHECK_THAT(a.sweepAngle(), WithinAbs(kPi, 1e-12));
    const Vec2 mid = a.pointAt(0.5);
    CHECK_THAT(mid.x, WithinAbs(0.0, 1e-9));
    CHECK_THAT(mid.y, WithinAbs(10.0, 1e-9));
    CHECK(near(a.startPoint(), {10.0, 0.0}));
    CHECK(near(a.endPoint(), {-10.0, 0.0}));
    CHECK_THAT(a.length(), WithinAbs(kPi * 10.0, 1e-9));
    // The CW arc between the same points goes the other way round.
    const ArcSpan cw = ArcSpan::fromCentre({0.0, 0.0}, {10.0, 0.0}, {-10.0, 0.0}, ArcDirection::CW);
    CHECK(near(cw.pointAt(0.5), {0.0, -10.0}));
    CHECK_THAT(cw.sweepAngle(), WithinAbs(-kPi, 1e-12));
}

TEST_CASE("ArcSpan sweep wraps across the +X axis", "[geometry][span]") {
    // CCW from 350 degrees to 10 degrees is a 20 degree arc, not 340.
    const ArcSpan a{{0.0, 0.0}, 1.0, 350.0 * kPi / 180.0, 10.0 * kPi / 180.0, ArcDirection::CCW};
    CHECK_THAT(a.sweepAngle(), WithinAbs(20.0 * kPi / 180.0, 1e-12));
    const ArcSpan b{{0.0, 0.0}, 1.0, 10.0 * kPi / 180.0, 350.0 * kPi / 180.0, ArcDirection::CW};
    CHECK_THAT(b.sweepAngle(), WithinAbs(-20.0 * kPi / 180.0, 1e-12));
    CHECK(a.containsAngle(0.0));
    CHECK(a.containsAngle(a.startAngle));
    CHECK(a.containsAngle(a.endAngle));
    CHECK_FALSE(a.containsAngle(kPi));
}

TEST_CASE("ArcSpan tangent, reverse and bounds", "[geometry][span]") {
    const ArcSpan q = ArcSpan::fromCentre({0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, ArcDirection::CCW);
    CHECK(near(q.tangentAt(0.0), {0.0, 1.0}));
    CHECK(near(q.tangentAt(1.0), {-1.0, 0.0}));
    const ArcSpan r = q.reversed();
    CHECK(r.direction == ArcDirection::CW);
    CHECK(near(r.startPoint(), q.endPoint()));
    CHECK(near(r.endPoint(), q.startPoint()));
    CHECK(near(r.tangentAt(0.0), {1.0, 0.0}));
    CHECK_THAT(r.length(), WithinAbs(q.length(), 1e-12));
    // Quarter arc bounds are just its end points.
    CHECK(approxEqual(q.bounds(), Box2{{0.0, 0.0}, {1.0, 1.0}}, 1e-12));
    // A 3/4 arc from (1,0) CCW to (0,-1) reaches x=-1 and y=1.
    const ArcSpan big = ArcSpan::fromCentre({2.0, 3.0}, {3.0, 3.0}, {2.0, 2.0}, ArcDirection::CCW);
    CHECK(approxEqual(big.bounds(), Box2{{1.0, 2.0}, {3.0, 4.0}}, 1e-12));
    // Full circle bounds.
    CHECK(approxEqual(ArcSpan::fullCircle({0.0, 0.0}, 10.0).bounds(), Box2{{-10.0, -10.0}, {10.0, 10.0}},
                      1e-12));
    // The small arc that crosses +X but not +Y.
    const ArcSpan tiny{{0.0, 0.0}, 1.0, -0.1, 0.1, ArcDirection::CCW};
    const Box2 tb = tiny.bounds();
    CHECK_THAT(tb.max.x, WithinAbs(1.0, 1e-12));
    CHECK_THAT(tb.min.x, WithinAbs(std::cos(0.1), 1e-12));
}

TEST_CASE("ArcSpan segment count honours chordal tolerance", "[geometry][span]") {
    const ArcSpan a = ArcSpan::fullCircle({0.0, 0.0}, 10.0);
    const std::size_t n = a.segmentCountFor(0.01);
    CHECK(n < 200);
    CHECK(n > 10);
    const double step = kTwoPi / static_cast<double>(n);
    CHECK(10.0 * (1.0 - std::cos(step / 2.0)) <= 0.01 + 1e-12);
    CHECK(a.segmentCountFor(0.0) >= 4);
    CHECK(a.segmentCountFor(100.0) >= 4);
    CHECK(ArcSpan{{0.0, 0.0}, 0.0, 0.0, 1.0, ArcDirection::CCW}.segmentCountFor(0.01) == 1);
}

TEST_CASE("ArcSpan to Beziers approximates the circle", "[geometry][span]") {
    const ArcSpan a = ArcSpan::fullCircle({1.0, 2.0}, 5.0);
    const auto bz = a.toBeziers();
    REQUIRE(bz.size() == 4);
    CHECK(near(bz.front().p0, a.startPoint()));
    CHECK(near(bz.back().p3, a.endPoint()));
    for (std::size_t i = 1; i < bz.size(); ++i) {
        CHECK(near(bz[i - 1].p3, bz[i].p0));
    }
    // Radial error of the standard 90-degree approximation is about 2.7e-4 * r.
    for (const BezierSpan& b : bz) {
        for (int i = 0; i <= 20; ++i) {
            const double r = distance(b.pointAt(i / 20.0), a.centre);
            CHECK_THAT(r, WithinAbs(5.0, 5.0 * 3e-4));
        }
    }
    const ArcSpan half = ArcSpan::fromCentre({0.0, 0.0}, {1.0, 0.0}, {-1.0, 0.0}, ArcDirection::CW);
    const auto hb = half.toBeziers();
    REQUIRE(hb.size() == 2);
    CHECK(near(hb[0].p3, {0.0, -1.0}, 1e-9));
}

// ----------------------------------------------------------------- Bezier

// Acceptance criterion 3.
TEST_CASE("BezierSpan adaptive length agrees with a 10k-segment estimate", "[geometry][span][acceptance]") {
    const BezierSpan curves[] = {
        {{0.0, 0.0}, {10.0, 20.0}, {30.0, -20.0}, {40.0, 0.0}},
        {{0.0, 0.0}, {0.0, 0.0}, {10.0, 10.0}, {10.0, 10.0}},    // straight line, degenerate controls
        {{-5.0, 3.0}, {50.0, 3.0}, {-50.0, 3.0}, {5.0, 3.0}},    // self-overlapping on a line
        {{0.0, 0.0}, {0.0, 55.0}, {100.0, 55.0}, {100.0, 0.0}},  // U shape
        {{0.0, 0.0}, {30.0, 30.0}, {-30.0, 30.0}, {0.0, 0.0}},   // loop
    };
    for (const BezierSpan& b : curves) {
        const double estimate = chordLength(b, 10000);
        const double adaptive = b.length();
        CHECK_THAT(adaptive, WithinRel(estimate, 1e-6));
    }
    CHECK(BezierSpan{{1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}}.length() == 0.0);
}

TEST_CASE("BezierSpan evaluation, split, reverse, tangent", "[geometry][span]") {
    const BezierSpan b{{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}};
    CHECK(b.pointAt(0.0) == b.p0);
    CHECK(b.pointAt(1.0) == b.p3);
    CHECK(near(b.pointAt(0.5), {5.0, 7.5}));
    CHECK(near(b.tangentAt(0.0), {0.0, 1.0}));
    CHECK(near(b.tangentAt(1.0), {0.0, -1.0}));
    CHECK(near(b.tangentAt(0.5), {1.0, 0.0}));
    const auto [l, r] = b.split(0.5);
    CHECK(l.p0 == b.p0);
    CHECK(r.p3 == b.p3);
    CHECK(near(l.p3, r.p0));
    CHECK(near(l.p3, b.pointAt(0.5)));
    CHECK(near(l.pointAt(0.5), b.pointAt(0.25)));
    CHECK(near(r.pointAt(0.5), b.pointAt(0.75)));
    CHECK_THAT(l.length() + r.length(), WithinRel(b.length(), 1e-9));
    const BezierSpan rev = b.reversed();
    CHECK(rev.p0 == b.p3);
    CHECK(rev.p3 == b.p0);
    CHECK(near(rev.pointAt(0.25), b.pointAt(0.75)));
    // Degenerate derivative at the start still yields a direction.
    const BezierSpan deg{{0.0, 0.0}, {0.0, 0.0}, {5.0, 5.0}, {10.0, 10.0}};
    CHECK(near(deg.tangentAt(0.0), Vec2{1.0, 1.0}.normalized()));
    const BezierSpan deg2{{0.0, 0.0}, {5.0, 5.0}, {10.0, 10.0}, {10.0, 10.0}};
    CHECK(near(deg2.tangentAt(1.0), Vec2{1.0, 1.0}.normalized()));
}

TEST_CASE("BezierSpan bounds are tight", "[geometry][span]") {
    const BezierSpan b{{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}};
    const Box2 box = b.bounds();
    CHECK_THAT(box.min.x, WithinAbs(0.0, 1e-12));
    CHECK_THAT(box.max.x, WithinAbs(10.0, 1e-12));
    CHECK_THAT(box.min.y, WithinAbs(0.0, 1e-12));
    CHECK_THAT(box.max.y, WithinAbs(7.5, 1e-12));  // y peaks at t=0.5
    // Compare to a brute-force sample on a wiggly curve.
    const BezierSpan w{{0.0, 0.0}, {40.0, 30.0}, {-30.0, -20.0}, {10.0, 5.0}};
    Box2 sampled;
    for (int i = 0; i <= 100000; ++i) {
        sampled.expand(w.pointAt(i / 100000.0));
    }
    CHECK(approxEqual(w.bounds(), sampled, 1e-6));
    // Linear (quadratic coefficient vanishes) case.
    const BezierSpan lin{{0.0, 0.0}, {1.0, 2.0}, {2.0, 4.0}, {3.0, 6.0}};
    CHECK(approxEqual(lin.bounds(), Box2{{0.0, 0.0}, {3.0, 6.0}}, 1e-12));
}

// --------------------------------------------------------- Span (variant)

TEST_CASE("Span free functions dispatch to each alternative", "[geometry][span]") {
    const Span line = LineSpan{{0.0, 0.0}, {4.0, 0.0}};
    const Span arc = ArcSpan::fromCentre({0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, ArcDirection::CCW);
    const Span bez = BezierSpan{{0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {3.0, 0.0}};

    CHECK_THAT(length(line), WithinAbs(4.0, 1e-12));
    CHECK_THAT(length(arc), WithinAbs(kPi / 2.0, 1e-12));
    CHECK(length(bez) > 3.0);
    CHECK(startPoint(line) == Vec2{0.0, 0.0});
    CHECK(endPoint(line) == Vec2{4.0, 0.0});
    CHECK(near(startPoint(arc), {1.0, 0.0}));
    CHECK(near(endPoint(arc), {0.0, 1.0}));
    CHECK(startPoint(bez) == Vec2{0.0, 0.0});
    CHECK(endPoint(bez) == Vec2{3.0, 0.0});
    CHECK(pointAt(line, 0.25) == Vec2{1.0, 0.0});
    CHECK(near(tangentAt(arc, 0.0), {0.0, 1.0}));
    CHECK(approxEqual(bounds(arc), Box2{{0.0, 0.0}, {1.0, 1.0}}, 1e-12));

    CHECK(std::holds_alternative<LineSpan>(reversed(line)));
    CHECK(std::holds_alternative<ArcSpan>(reversed(arc)));
    CHECK(std::holds_alternative<BezierSpan>(reversed(bez)));
    CHECK(near(startPoint(reversed(arc)), endPoint(arc)));

    const Tolerance tol;
    CHECK(isDegenerate(Span{LineSpan{{1.0, 1.0}, {1.0, 1.0 + 1e-4}}}, tol));
    CHECK_FALSE(isDegenerate(line, tol));
}

TEST_CASE("Span transformed keeps arcs under conformal maps", "[geometry][span]") {
    const Span arc = ArcSpan::fromCentre({0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}, ArcDirection::CCW);
    const Transform2 m =
        Transform2::translation({5.0, 5.0}) * Transform2::rotation(kPi / 2.0) * Transform2::scaling(2.0);
    const auto out = transformed(arc, m);
    REQUIRE(out.size() == 1);
    REQUIRE(std::holds_alternative<ArcSpan>(out[0]));
    const ArcSpan& a = std::get<ArcSpan>(out[0]);
    CHECK(near(a.centre, {5.0, 5.0}));
    CHECK_THAT(a.radius, WithinAbs(20.0, 1e-12));
    CHECK(a.direction == ArcDirection::CCW);
    CHECK(near(a.startPoint(), m.apply({10.0, 0.0})));
    CHECK(near(a.endPoint(), m.apply({0.0, 10.0})));
    CHECK(near(a.pointAt(0.5), m.apply(pointAt(arc, 0.5))));

    // Mirroring flips direction but keeps the point set.
    const auto mir = transformed(arc, Transform2::mirrorX());
    REQUIRE(mir.size() == 1);
    const ArcSpan& ma = std::get<ArcSpan>(mir[0]);
    CHECK(ma.direction == ArcDirection::CW);
    CHECK(near(ma.startPoint(), {-10.0, 0.0}));
    CHECK(near(ma.endPoint(), {0.0, 10.0}));
    CHECK(near(ma.pointAt(0.5), {-10.0 / std::sqrt(2.0), 10.0 / std::sqrt(2.0)}));

    // Full circles stay full circles.
    const auto fc = transformed(Span{ArcSpan::fullCircle({0.0, 0.0}, 3.0)}, Transform2::rotation(1.0));
    REQUIRE(fc.size() == 1);
    CHECK(std::get<ArcSpan>(fc[0]).isFullCircle());
    CHECK_THAT(length(fc[0]), WithinAbs(kTwoPi * 3.0, 1e-9));
}

TEST_CASE("Span transformed converts arcs to Beziers under non-uniform scale", "[geometry][span]") {
    const Span circle = ArcSpan::fullCircle({0.0, 0.0}, 10.0);
    const Transform2 m = Transform2::scaling(2.0, 1.0);
    const auto out = transformed(circle, m);
    REQUIRE(out.size() == 4);
    for (const Span& s : out) {
        CHECK(std::holds_alternative<BezierSpan>(s));
    }
    // Resulting ellipse x^2/400 + y^2/100 = 1 within the Bezier approximation error.
    for (const Span& s : out) {
        for (int i = 0; i <= 10; ++i) {
            const Vec2 p = pointAt(s, i / 10.0);
            CHECK_THAT(p.x * p.x / 400.0 + p.y * p.y / 100.0, WithinAbs(1.0, 1e-3));
        }
    }
    CHECK(near(startPoint(out.front()), {20.0, 0.0}));
    CHECK(near(endPoint(out.back()), {20.0, 0.0}));
}

TEST_CASE("polygoniseInto excludes start and includes end", "[geometry][span]") {
    const Tolerance tol{0.001, 1e-9, 0.01};
    std::vector<Vec2> pts;
    polygoniseInto(Span{LineSpan{{0.0, 0.0}, {1.0, 0.0}}}, tol, pts);
    REQUIRE(pts.size() == 1);
    CHECK(pts[0] == Vec2{1.0, 0.0});

    pts.clear();
    const ArcSpan arc = ArcSpan::fromCentre({0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}, ArcDirection::CCW);
    polygoniseInto(Span{arc}, tol, pts);
    REQUIRE(pts.size() >= 2);
    CHECK(near(pts.back(), {0.0, 10.0}));
    for (const Vec2 p : pts) {
        CHECK_THAT(p.length(), WithinAbs(10.0, 1e-9));
    }

    pts.clear();
    const BezierSpan bz{{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}};
    polygoniseInto(Span{bz}, tol, pts);
    REQUIRE(pts.size() >= 2);
    CHECK(near(pts.back(), bz.p3));
    // Each polyline vertex lies on the curve, and sampled curve points stay within tolerance of the chords.
    std::vector<Vec2> poly{bz.p0};
    poly.insert(poly.end(), pts.begin(), pts.end());
    for (int i = 0; i <= 1000; ++i) {
        const Vec2 c = bz.pointAt(i / 1000.0);
        double best = 1e9;
        for (std::size_t k = 1; k < poly.size(); ++k) {
            const Vec2 a = poly[k - 1];
            const Vec2 b = poly[k];
            const Vec2 ab = b - a;
            const double t = std::clamp((c - a).dot(ab) / ab.lengthSquared(), 0.0, 1.0);
            best = std::min(best, distance(c, a + ab * t));
        }
        CHECK(best <= tol.chordal + 1e-9);
    }
}
