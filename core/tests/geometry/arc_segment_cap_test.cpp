// T-0182: polygonising an arc is bounded (kMaxArcSegments chords), reports when it is coarser than
// the tolerance, leaves every arc under the cap exactly as before, and the callers that need the
// tolerance refuse a coarse contour (ADR-0005 A2).
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "cpu_time.hpp"

using namespace carvetto::geometry;
using carvetto::test::processCpuSeconds;
namespace geometry = carvetto::geometry;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

ArcSpan makeArc(Vec2 centre, double radius, double a0, double a1, ArcDirection dir = ArcDirection::CCW) {
    ArcSpan a;
    a.centre = centre;
    a.radius = radius;
    a.startAngle = a0;
    a.endAngle = a1;
    a.direction = dir;
    return a;
}

Tolerance tolWith(double chordal) {
    Tolerance t = Tolerance::defaults();
    t.chordal = chordal;
    return t;
}

// The pre-T-0182 polygonisation, kept here as the reference for arcs under the cap.
std::vector<Vec2> referencePolygon(const ArcSpan& v, double chordal) {
    const double sweep = std::fabs(v.sweepAngle());
    std::size_t n = 1;
    if (v.radius > 0.0) {
        if (chordal <= 0.0 || chordal >= v.radius) {
            n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(sweep / (kPi * 0.5))));
        } else {
            const double maxStep = 2.0 * std::acos(1.0 - chordal / v.radius);
            n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(sweep / maxStep - 1e-9)));
        }
    }
    std::vector<Vec2> pts;
    for (std::size_t i = 1; i <= n; ++i) {
        pts.push_back(v.pointAt(static_cast<double>(i) / static_cast<double>(n)));
    }
    return pts;
}

Contour closedWith(const ArcSpan& a) {
    Contour c;
    c.closed = true;
    c.spans.emplace_back(a);
    return c;
}

Contour squareAt(double x, double y, double s) {
    Contour c;
    c.closed = true;
    c.spans = {LineSpan{{x, y}, {x + s, y}}, LineSpan{{x + s, y}, {x + s, y + s}},
               LineSpan{{x + s, y + s}, {x, y + s}}, LineSpan{{x, y + s}, {x, y}}};
    return c;
}

}  // namespace

TEST_CASE("huge arcs polygonise in bounded time and memory", "[polygonise][arc][geometry][cap]") {
    for (const double radius : {1e9, 1e12, 1e15, 1e100, 1e300, 1.7e308, kInf}) {
        for (const double chordal : {0.001, 1e-9}) {
            for (const bool full : {true, false}) {
                const ArcSpan arc = makeArc({0, 0}, radius, 0.0, full ? 0.0 : 3.0);
                REQUIRE(arc.segmentCountFor(chordal) <= kMaxArcSegments);
                const double t0 = processCpuSeconds();
                std::vector<Vec2> pts;
                bool coarse = false;
                polygoniseInto(Span{arc}, tolWith(chordal), pts, &coarse);
                const double dt = processCpuSeconds() - t0;
                INFO("radius " << radius << " chordal " << chordal << " full " << full);
                CHECK(dt < 1.0);  // ~0.05 s in Release; fails fast on a hang
                CHECK(pts.size() <= kMaxArcSegments);
                CHECK(coarse);
            }
        }
    }
}

TEST_CASE("the cap is exact: at the limit nothing is cut, one more chord is", "[polygonise][arc][geometry][cap]") {
    const double chordal = 0.01;
    const double radius = 3e9;
    // The chord angle the arc code uses; sweeps that are whole multiples of it need exactly that many chords.
    const double maxStep = 2.0 * std::acos(1.0 - chordal / radius);
    const double capSweep = static_cast<double>(kMaxArcSegments) * maxStep;
    REQUIRE(capSweep < 6.0);
    const ArcSpan atLimit = makeArc({0, 0}, radius, 0.0, capSweep);
    const ArcSpan beyond = makeArc({0, 0}, radius, 0.0, static_cast<double>(kMaxArcSegments + 1) * maxStep);
    const ArcSpan below = makeArc({0, 0}, radius, 0.0, static_cast<double>(kMaxArcSegments - 1) * maxStep);
    REQUIRE(atLimit.segmentCountFor(chordal) == kMaxArcSegments);
    REQUIRE(beyond.segmentCountFor(chordal) == kMaxArcSegments);
    REQUIRE(below.segmentCountFor(chordal) == kMaxArcSegments - 1);
    REQUIRE_FALSE(atLimit.segmentCountIsCapped(chordal));
    REQUIRE_FALSE(below.segmentCountIsCapped(chordal));
    REQUIRE(beyond.segmentCountIsCapped(chordal));

    std::vector<Vec2> pts;
    bool coarse = false;
    polygoniseInto(Span{atLimit}, tolWith(chordal), pts, &coarse);
    CHECK(pts.size() == kMaxArcSegments);
    CHECK_FALSE(coarse);
    CHECK(pts == referencePolygon(atLimit, chordal));
    pts.clear();
    polygoniseInto(Span{below}, tolWith(chordal), pts, &coarse);
    CHECK_FALSE(coarse);
    CHECK(pts == referencePolygon(below, chordal));
    pts.clear();
    polygoniseInto(Span{beyond}, tolWith(chordal), pts, &coarse);
    CHECK(coarse);
    CHECK(pts.size() == kMaxArcSegments);
}

TEST_CASE("a capped polygon has the documented deviation and ends where the arc does", "[polygonise][arc][geometry][cap]") {
    const double radius = 1e12;
    const ArcSpan arc = makeArc({0, 0}, radius, 0.0, kPi);
    std::vector<Vec2> pts;
    polygoniseInto(Span{arc}, tolWith(0.001), pts);
    REQUIRE(pts.size() == kMaxArcSegments);
    CHECK(pts.back() == arc.endPoint());
    const double step = kPi / static_cast<double>(kMaxArcSegments);
    const double expected = radius * (1.0 - std::cos(step / 2.0));  // sagitta, ~ R * step^2 / 8
    for (const std::size_t i : {std::size_t{1}, std::size_t{1000}, kMaxArcSegments / 2}) {
        const Vec2 mid = (pts[i - 1] + pts[i]) * 0.5;
        const double deviation = radius - mid.length();
        CHECK(std::fabs(deviation - expected) <= 1e-3 * expected + 1e-3);
    }
    CHECK(expected < radius * step * step / 8.0 * 1.0001);
}

TEST_CASE("arcs under the cap polygonise exactly as before", "[polygonise][arc][geometry][cap]") {
    std::uint64_t seed = 12345;
    auto next = [&seed]() {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(seed >> 11) / 9007199254740992.0;
    };
    int checked = 0;
    for (int i = 0; i < 400; ++i) {
        const double radius = std::pow(10.0, -6.0 + 15.0 * next());
        const double chordal = std::pow(10.0, -9.0 * next());
        const Vec2 centre{(next() - 0.5) * std::pow(10.0, 1.0 + 8.0 * next()), (next() - 0.5) * 1e6};
        const double a0 = (next() - 0.5) * 20.0;
        const double a1 = i % 5 == 0 ? a0 : (next() - 0.5) * 20.0;
        const ArcDirection dir = i % 2 == 0 ? ArcDirection::CCW : ArcDirection::CW;
        const double sweepFactor = i % 7 == 0 ? 1e-6 : 1.0;
        const ArcSpan arc = makeArc(centre, radius, a0, a0 + (a1 - a0) * sweepFactor, dir);
        if (arc.segmentCountIsCapped(chordal)) {
            continue;
        }
        std::vector<Vec2> pts;
        bool coarse = false;
        polygoniseInto(Span{arc}, tolWith(chordal), pts, &coarse);
        CHECK_FALSE(coarse);
        CHECK(pts == referencePolygon(arc, chordal));
        ++checked;
    }
    CHECK(checked > 200);
}

TEST_CASE("non-finite arcs are bounded and flagged", "[polygonise][arc][geometry][cap]") {
    const std::vector<ArcSpan> arcs = {
        makeArc({0, 0}, kInf, 0.0, 1.0),  makeArc({0, 0}, kNaN, 0.0, 1.0), makeArc({0, 0}, 5.0, 0.0, kInf),
        makeArc({0, 0}, 5.0, kNaN, 1.0),  makeArc({0, 0}, 5.0, kInf, -kInf), makeArc({kInf, 0}, 5.0, 0.0, 1.0),
        makeArc({0, 0}, -kInf, 0.0, 1.0),
    };
    for (const ArcSpan& arc : arcs) {
        const double t0 = processCpuSeconds();
        std::vector<Vec2> pts;
        bool coarse = false;
        polygoniseInto(Span{arc}, tolWith(1e-9), pts, &coarse);
        CHECK(processCpuSeconds() - t0 < 0.5);
        CHECK(pts.size() == 1);
        CHECK(coarse);
        Contour c = closedWith(arc);
        bool contourCoarse = false;
        const geometry::Polyline pl = c.polygonise(tolWith(1e-9), &contourCoarse);
        CHECK(contourCoarse);
        CHECK(pl.points.size() <= 3);
    }
    // A bad tolerance is a degenerate one, not a hang.
    // (the documented degenerate-tolerance count: a quarter turn per chord, 4 for a full circle)
    for (const double bad : {kNaN, kInf, -1.0, 0.0}) {
        std::vector<Vec2> pts;
        polygoniseInto(Span{makeArc({0, 0}, 10.0, 0.0, 0.0)}, tolWith(bad), pts);
        CHECK(pts.size() == 4);
    }
}

TEST_CASE("Contour::polygonise and the region variant report a coarse polygon", "[polygonise][arc][geometry][cap]") {
    const Tolerance tol = tolWith(0.001);
    SECTION("closed") {
        bool coarse = false;
        const geometry::Polyline pl = closedWith(makeArc({0, 0}, 1e13, 0.0, 0.0)).polygonise(tol, &coarse);
        CHECK(coarse);
        CHECK(pl.points.size() <= kMaxArcSegments + 1);
    }
    SECTION("open") {
        Contour c;
        c.spans.emplace_back(makeArc({0, 0}, 1e13, 0.0, 1.0));
        bool coarse = false;
        c.polygonise(tol, &coarse);
        CHECK(coarse);
    }
    SECTION("a small arc is not coarse and the flag is never reset") {
        bool coarse = false;
        closedWith(makeArc({0, 0}, 10.0, 0.0, 0.0)).polygonise(tol, &coarse);
        CHECK_FALSE(coarse);
        coarse = true;
        closedWith(makeArc({0, 0}, 10.0, 0.0, 0.0)).polygonise(tol, &coarse);
        CHECK(coarse);
    }
}

TEST_CASE("a Bezier past its split cap is reported coarse", "[polygonise][bezier][geometry][cap]") {
    const BezierSpan b{{0, 0}, {1e7, 1e7}, {-1e7, 1e7}, {0, 0}};
    std::vector<Vec2> pts;
    bool coarse = false;
    polygoniseInto(Span{b}, tolWith(1e-9), pts, &coarse);
    CHECK(coarse);
    coarse = false;
    pts.clear();
    polygoniseInto(Span{BezierSpan{{0, 0}, {1, 2}, {3, 2}, {4, 0}}}, tolWith(0.01), pts, &coarse);
    CHECK_FALSE(coarse);
}

TEST_CASE("booleans refuse a contour that cannot meet the tolerance", "[boolean][polygonise][geometry][cap]") {
    const Tolerance tol = tolWith(0.001);
    const Contour huge = closedWith(makeArc({0, 0}, 1e13, 0.0, 0.0));
    const Contour sq = squareAt(0, 0, 10);
    SECTION("as the subject") {
        const BooleanResult r = booleanOp(BooleanOp::Union, {huge, sq}, {}, tol);
        CHECK(r.refusedContours == 1);
        CHECK(r.closed.size() == 1);
    }
    SECTION("as the clip") {
        const BooleanResult r = booleanOp(BooleanOp::Difference, {sq}, {huge}, tol);
        CHECK(r.refusedContours == 1);
        CHECK(r.closed.size() == 1);
    }
    SECTION("a non-finite arc") {
        const BooleanResult r =
            booleanOp(BooleanOp::Union, {closedWith(makeArc({0, 0}, kInf, 0.0, 0.0)), sq}, {}, tol);
        CHECK(r.refusedContours == 1);
        CHECK(r.closed.size() == 1);
    }
    SECTION("nothing refused for ordinary input") {
        const BooleanResult r = booleanOp(BooleanOp::Union, {closedWith(makeArc({0, 0}, 5.0, 0.0, 0.0)), sq}, {}, tol);
        CHECK(r.refusedContours == 0);
        CHECK_FALSE(r.closed.empty());
    }
    SECTION("open-path clipper counts clip and subject") {
        OpenPathClipper clipper({huge, sq}, {1.0, 0.0}, tol);
        CHECK(clipper.refusedContours() == 1);
        Contour open;
        open.spans.emplace_back(makeArc({0, 0}, 1e13, 0.0, 1.0));
        clipper.clip(BooleanOp::Intersection, {open});
        CHECK(clipper.refusedContours() == 2);
    }
    SECTION("offset leaves it out") {
        CHECK(offsetContours({huge}, 1.0, JoinType::Round, tol).empty());
        CHECK_FALSE(offsetContours({sq}, 1.0, JoinType::Round, tol).empty());
    }
}
