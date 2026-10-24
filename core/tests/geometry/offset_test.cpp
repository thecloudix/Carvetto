#include "carvetto/geometry/offset.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <utility>
#include <variant>

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

const Tolerance kTol{};  // distance 0.001, chordal 0.01

double area(const Contour& c) {
    return c.polygonise(kTol).signedArea();
}

Contour square10() {
    return Contour::rectangle(Box2{{0.0, 0.0}, {10.0, 10.0}});
}

// Distance from p to the nearest point on the polyline's segments.
double distanceToPolyline(Vec2 p, const Polyline& pl) {
    double best = 1e300;
    for (std::size_t k = 0; k < pl.segmentCount(); ++k) {
        const Vec2 a = pl.segmentStart(k);
        const Vec2 b = pl.segmentEnd(k);
        const Vec2 ab = b - a;
        const double len2 = ab.lengthSquared();
        const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
        best = std::min(best, distance(p, a + ab * t));
    }
    return best;
}

// Symmetric Hausdorff distance between two closed contours, sampled densely along each.
double hausdorff(const Contour& a, const Contour& b, int samples = 4000) {
    const Polyline pa = a.polygonise(Tolerance{0.001, 1e-9, 0.0005});
    const Polyline pb = b.polygonise(Tolerance{0.001, 1e-9, 0.0005});
    double worst = 0.0;
    for (int i = 0; i < samples; ++i) {
        const double t = static_cast<double>(i) / samples;
        worst = std::max(worst, distanceToPolyline(a.pointAt(t), pb));
        worst = std::max(worst, distanceToPolyline(b.pointAt(t), pa));
    }
    return worst;
}

std::size_t countArcs(const Contour& c) {
    return static_cast<std::size_t>(std::count_if(
        c.spans.begin(), c.spans.end(), [](const Span& s) { return std::holds_alternative<ArcSpan>(s); }));
}

// Rounded rectangle 30x20 with corner radius 4, CCW, starting at the bottom edge.
Contour roundedRect() {
    Contour c;
    c.closed = true;
    const double r = 4.0;
    c.spans.emplace_back(LineSpan{{r, 0.0}, {30.0 - r, 0.0}});
    c.spans.emplace_back(ArcSpan::fromCentre({30.0 - r, r}, {30.0 - r, 0.0}, {30.0, r}, ArcDirection::CCW));
    c.spans.emplace_back(LineSpan{{30.0, r}, {30.0, 20.0 - r}});
    c.spans.emplace_back(
        ArcSpan::fromCentre({30.0 - r, 20.0 - r}, {30.0, 20.0 - r}, {30.0 - r, 20.0}, ArcDirection::CCW));
    c.spans.emplace_back(LineSpan{{30.0 - r, 20.0}, {r, 20.0}});
    c.spans.emplace_back(ArcSpan::fromCentre({r, 20.0 - r}, {r, 20.0}, {0.0, 20.0 - r}, ArcDirection::CCW));
    c.spans.emplace_back(LineSpan{{0.0, 20.0 - r}, {0.0, r}});
    c.spans.emplace_back(ArcSpan::fromCentre({r, r}, {0.0, r}, {r, 0.0}, ArcDirection::CCW));
    return c;
}

}  // namespace

// --- Acceptance criterion 1 -----------------------------------------------------------------

TEST_CASE("Offset square outwards with round joins", "[geometry][offset]") {
    const auto out = offsetContours({square10()}, 1.0, JoinType::Round, kTol);
    REQUIRE(out.size() == 1);
    CHECK(out[0].closed);
    CHECK(out[0].isContinuous(kTol));
    const double expected = 100.0 + 40.0 + kPi;
    CHECK_THAT(area(out[0]), WithinRel(expected, 0.001));
    const Box2 b = out[0].bounds();
    CHECK_THAT(b.min.x, WithinAbs(-1.0, 1e-6));
    CHECK_THAT(b.max.y, WithinAbs(11.0, 1e-6));
}

TEST_CASE("Offset square outwards with sharp and square joins", "[geometry][offset]") {
    SECTION("sharp joins give a 12x12 square") {
        const auto out = offsetContours({square10()}, 1.0, JoinType::Sharp, kTol);
        REQUIRE(out.size() == 1);
        CHECK(out[0].spanCount() == 4);
        CHECK_THAT(area(out[0]), WithinAbs(144.0, 1e-6));
    }
    SECTION("square joins chamfer the corners at the offset distance") {
        const auto out = offsetContours({square10()}, 1.0, JoinType::Square, kTol);
        REQUIRE(out.size() == 1);
        CHECK(out[0].spanCount() == 8);
        // Each corner loses a triangle; the chamfer line is 1 mm from the original vertex.
        const double chamferLeg = 2.0 - std::sqrt(2.0);
        const double expected = 144.0 - 4.0 * 0.5 * chamferLeg * chamferLeg;
        CHECK_THAT(area(out[0]), WithinRel(expected, 1e-6));
    }
    SECTION("mitre limit turns a sharp acute corner into a square one") {
        Contour spike;
        spike.closed = true;
        spike.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
        spike.spans.emplace_back(LineSpan{{10.0, 0.0}, {0.0, 1.0}});
        spike.spans.emplace_back(LineSpan{{0.0, 1.0}, {0.0, 0.0}});
        const auto limited = offsetContours({spike}, 1.0, JoinType::Sharp, kTol, 2.0);
        const auto unlimited = offsetContours({spike}, 1.0, JoinType::Sharp, kTol, 100.0);
        REQUIRE(limited.size() == 1);
        REQUIRE(unlimited.size() == 1);
        // The unlimited mitre reaches far past the tip; the limited one stops near 2*distance.
        CHECK(unlimited[0].bounds().max.x > 20.0);
        CHECK(limited[0].bounds().max.x < 13.0);
    }
}

// --- Acceptance criterion 2 -----------------------------------------------------------------

TEST_CASE("Offset square inwards past its half-size vanishes", "[geometry][offset]") {
    CHECK(offsetContours({square10()}, -6.0, JoinType::Round, kTol).empty());
    CHECK(offsetContours({square10()}, -5.0, JoinType::Sharp, kTol).empty());
    const auto small = offsetContours({square10()}, -4.0, JoinType::Sharp, kTol);
    REQUIRE(small.size() == 1);
    CHECK_THAT(area(small[0]), WithinAbs(4.0, 1e-6));
}

// --- Acceptance criterion 4 -----------------------------------------------------------------

TEST_CASE("Offset out then in returns to the original rounded shape", "[geometry][offset]") {
    const Contour original = roundedRect();
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        const auto out = offsetContours({original}, 2.5, join, kTol);
        REQUIRE(out.size() == 1);
        const auto back = offsetContours(out, -2.5, JoinType::Round, kTol);
        REQUIRE(back.size() == 1);
        CHECK(hausdorff(original, back[0]) <= 2.0 * kTol.chordal);
    }
}

TEST_CASE("Offset direction follows the outermost contour's orientation", "[geometry][offset]") {
    const Contour cw = square10().reversed();
    // A lone clockwise square is the outermost contour, so +1 still grows it and keeps it CW.
    const auto out = offsetContours({cw}, 1.0, JoinType::Sharp, kTol);
    REQUIRE(out.size() == 1);
    CHECK_THAT(area(out[0]), WithinAbs(-144.0, 1e-6));
    // Nested inside a CCW outer the same square is a hole and shrinks.
    const Contour outer = Contour::rectangle(Box2{{-10.0, -10.0}, {20.0, 20.0}});
    const auto nested = offsetContours({outer, cw}, 1.0, JoinType::Sharp, kTol);
    REQUIRE(nested.size() == 2);
    const auto hole =
        std::find_if(nested.begin(), nested.end(), [](const Contour& c) { return area(c) < 0.0; });
    REQUIRE(hole != nested.end());
    CHECK_THAT(area(*hole), WithinAbs(-64.0, 1e-6));
}

TEST_CASE("Offset of an outer with a hole keeps both", "[geometry][offset]") {
    const Contour outer = Contour::rectangle(Box2{{0.0, 0.0}, {20.0, 20.0}});
    const Contour hole = Contour::rectangle(Box2{{8.0, 8.0}, {12.0, 12.0}}).reversed();
    SECTION("outward offset grows the outer and shrinks the hole") {
        const auto out = offsetContours({outer, hole}, 1.0, JoinType::Sharp, kTol);
        REQUIRE(out.size() == 2);
        double total = 0.0;
        for (const auto& c : out) {
            total += area(c);
        }
        CHECK_THAT(total, WithinAbs(22.0 * 22.0 - 2.0 * 2.0, 1e-6));
    }
    SECTION("inward offset shrinks the outer and grows the hole") {
        const auto out = offsetContours({outer, hole}, -2.5, JoinType::Sharp, kTol);
        REQUIRE(out.size() == 2);
        double total = 0.0;
        for (const auto& c : out) {
            total += area(c);
        }
        CHECK_THAT(total, WithinAbs(15.0 * 15.0 - 9.0 * 9.0, 1e-6));
    }
    SECTION("inward offset past the wall thickness removes everything") {
        CHECK(offsetContours({outer, hole}, -4.5, JoinType::Sharp, kTol).empty());
    }
}

TEST_CASE("Offset merges overlapping results", "[geometry][offset]") {
    const Contour a = Contour::rectangle(Box2{{0.0, 0.0}, {10.0, 10.0}});
    const Contour b = Contour::rectangle(Box2{{11.0, 0.0}, {21.0, 10.0}});
    CHECK(offsetContours({a, b}, 0.25, JoinType::Sharp, kTol).size() == 2);
    const auto merged = offsetContours({a, b}, 1.0, JoinType::Sharp, kTol);
    REQUIRE(merged.size() == 1);
    CHECK_THAT(area(merged[0]), WithinAbs(23.0 * 12.0, 1e-6));
}

TEST_CASE("Offset of a circle contour is a circle", "[geometry][offset]") {
    // Polygonal results are inscribed, so the area deficit scales with the chordal tolerance
    // (about 2/3 * sag * perimeter); a fine tolerance keeps it well under 0.1%.
    const Tolerance fine{0.001, 1e-9, 0.001};
    const auto out = offsetContours({Contour::circle({0.0, 0.0}, 10.0)}, 2.0, JoinType::Round, fine);
    REQUIRE(out.size() == 1);
    CHECK_THAT(out[0].polygonise(fine).signedArea(), WithinRel(kPi * 144.0, 0.001));
    for (const Span& s : out[0].spans) {
        CHECK_THAT(startPoint(s).length(), WithinAbs(12.0, 2.0 * fine.chordal));
    }
    // At the default tolerance the deficit is larger but still bounded by the chordal sag.
    const auto coarse = offsetContours({Contour::circle({0.0, 0.0}, 10.0)}, 2.0, JoinType::Round, kTol);
    REQUIRE(coarse.size() == 1);
    CHECK_THAT(area(coarse[0]), WithinRel(kPi * 144.0, 0.005));
}

// --- Edge cases -----------------------------------------------------------------------------

TEST_CASE("Offset edge cases", "[geometry][offset]") {
    SECTION("empty input") {
        CHECK(offsetContours({}, 1.0, JoinType::Round, kTol).empty());
        CHECK(offsetContours({Contour{}}, 1.0, JoinType::Round, kTol).empty());
    }
    SECTION("degenerate closed contour (two points) contributes nothing") {
        Contour sliver;
        sliver.closed = true;
        sliver.spans.emplace_back(LineSpan{{0.0, 0.0}, {5.0, 0.0}});
        sliver.spans.emplace_back(LineSpan{{5.0, 0.0}, {0.0, 0.0}});
        CHECK(offsetContours({sliver}, 1.0, JoinType::Round, kTol).empty());
    }
    SECTION("zero distance returns the polygonised input unchanged") {
        const auto out = offsetContours({square10()}, 0.0, JoinType::Round, kTol);
        REQUIRE(out.size() == 1);
        CHECK(out[0].spanCount() == 4);
        CHECK_THAT(area(out[0]), WithinAbs(100.0, 1e-9));
    }
    SECTION("self-touching figure (two squares sharing a vertex) offsets to one contour") {
        const Contour a = Contour::rectangle(Box2{{0.0, 0.0}, {10.0, 10.0}});
        const Contour b = Contour::rectangle(Box2{{10.0, 10.0}, {20.0, 20.0}});
        const auto out = offsetContours({a, b}, 1.0, JoinType::Sharp, kTol);
        REQUIRE(out.size() == 1);
        CHECK_THAT(area(out[0]), WithinAbs(2.0 * 144.0 - 4.0, 1e-6));
    }
    SECTION("self-touching bow-tie offsets without crashing and keeps positive area") {
        Contour bow;
        bow.closed = true;
        bow.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 10.0}});
        bow.spans.emplace_back(LineSpan{{10.0, 10.0}, {10.0, 0.0}});
        bow.spans.emplace_back(LineSpan{{10.0, 0.0}, {0.0, 10.0}});
        bow.spans.emplace_back(LineSpan{{0.0, 10.0}, {0.0, 0.0}});
        const auto out = offsetContours({bow}, 0.5, JoinType::Round, kTol);
        REQUIRE_FALSE(out.empty());
        for (const auto& c : out) {
            CHECK(c.closed);
            CHECK(c.isContinuous(kTol));
        }
    }
    SECTION("open contour is outlined into a closed shape") {
        Contour line;
        line.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
        const Tolerance fine{0.001, 1e-9, 0.001};
        const auto round = offsetContours({line}, 1.0, JoinType::Round, fine);
        REQUIRE(round.size() == 1);
        CHECK(round[0].closed);
        CHECK_THAT(round[0].polygonise(fine).signedArea(), WithinRel(20.0 + kPi, 0.001));
        const auto squared = offsetContours({line}, 1.0, JoinType::Square, kTol);
        REQUIRE(squared.size() == 1);
        CHECK_THAT(area(squared[0]), WithinAbs(24.0, 1e-6));
        CHECK(offsetContours({line}, 0.0, JoinType::Round, kTol).empty());
    }
}

// --- Acceptance criterion 5: open-contour single-side offsets --------------------------------

TEST_CASE("Open polyline offset left/right keeps the span count", "[geometry][offset][open]") {
    Contour pl;
    pl.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    pl.spans.emplace_back(LineSpan{{10.0, 0.0}, {10.0, 10.0}});

    SECTION("left of an L turning left is the inside: two trimmed lines") {
        const Contour left = offsetOpenContour(pl, 1.0, OffsetSide::Left, JoinType::Round, kTol);
        REQUIRE_FALSE(left.closed);
        REQUIRE(left.spanCount() == 2);
        CHECK(left.isContinuous(kTol));
        CHECK(approxEqual(left.startPoint(), {0.0, 1.0}));
        CHECK(approxEqual(endPoint(left.spans[0]), {9.0, 1.0}));
        CHECK(approxEqual(left.endPoint(), {9.0, 10.0}));
    }
    SECTION("right of an L turning left is the outside: round join adds one arc") {
        const Contour right = offsetOpenContour(pl, 1.0, OffsetSide::Right, JoinType::Round, kTol);
        REQUIRE(right.spanCount() == 3);
        CHECK(right.isContinuous(kTol));
        CHECK(countArcs(right) == 1);
        const auto& arc = std::get<ArcSpan>(right.spans[1]);
        CHECK(approxEqual(arc.centre, {10.0, 0.0}));
        CHECK_THAT(arc.radius, WithinAbs(1.0, 1e-12));
        CHECK_THAT(std::fabs(arc.sweepAngle()), WithinAbs(kPi / 2.0, 1e-9));
        CHECK(approxEqual(right.startPoint(), {0.0, -1.0}));
        CHECK(approxEqual(right.endPoint(), {11.0, 10.0}));
    }
    SECTION("sharp join on the outside keeps the span count exactly") {
        const Contour right = offsetOpenContour(pl, 1.0, OffsetSide::Right, JoinType::Sharp, kTol);
        REQUIRE(right.spanCount() == 2);
        CHECK(right.isContinuous(kTol));
        CHECK(approxEqual(endPoint(right.spans[0]), {11.0, -1.0}));
    }
    SECTION("square join on the outside adds one chamfer span") {
        const Contour right = offsetOpenContour(pl, 1.0, OffsetSide::Right, JoinType::Square, kTol);
        REQUIRE(right.spanCount() == 3);
        CHECK(right.isContinuous(kTol));
        // Chamfer line is 1 mm from the corner along the bisector direction (1,-1)/sqrt2.
        const Vec2 mid = pointAt(right.spans[1], 0.5);
        CHECK_THAT(distance(mid, {10.0, 0.0}), WithinAbs(1.0, 1e-9));
    }
    SECTION("negative distance flips the side") {
        const Contour a = offsetOpenContour(pl, -1.0, OffsetSide::Left, JoinType::Sharp, kTol);
        const Contour b = offsetOpenContour(pl, 1.0, OffsetSide::Right, JoinType::Sharp, kTol);
        CHECK(a == b);
    }
}

TEST_CASE("Open zig-zag offset with sharp joins keeps the span count", "[geometry][offset][open]") {
    Contour zig;
    zig.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 5.0}});
    zig.spans.emplace_back(LineSpan{{10.0, 5.0}, {20.0, 0.0}});
    zig.spans.emplace_back(LineSpan{{20.0, 0.0}, {30.0, 5.0}});
    zig.spans.emplace_back(LineSpan{{30.0, 5.0}, {40.0, 0.0}});
    for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
        const Contour out = offsetOpenContour(zig, 0.5, side, JoinType::Sharp, kTol);
        CHECK(out.spanCount() == 4);
        CHECK(out.isContinuous(kTol));
        CHECK_FALSE(out.closed);
        // Every point of the result is at least 0.5 from the original polyline.
        const Polyline orig = zig.polygonise(kTol);
        for (int i = 0; i <= 50; ++i) {
            CHECK(distanceToPolyline(out.pointAt(i / 50.0), orig) >= 0.5 - 1e-9);
        }
    }
}

TEST_CASE("Open contour offset handles arcs exactly", "[geometry][offset][open]") {
    // Line, quarter fillet (r=2), line: a G1-continuous path turning left.
    Contour c;
    c.spans.emplace_back(LineSpan{{0.0, 0.0}, {8.0, 0.0}});
    c.spans.emplace_back(ArcSpan::fromCentre({8.0, 2.0}, {8.0, 0.0}, {10.0, 2.0}, ArcDirection::CCW));
    c.spans.emplace_back(LineSpan{{10.0, 2.0}, {10.0, 10.0}});

    SECTION("outside offset grows the arc radius, same span count") {
        const Contour out = offsetOpenContour(c, 1.0, OffsetSide::Right, JoinType::Round, kTol);
        REQUIRE(out.spanCount() == 3);
        CHECK(out.isContinuous(kTol));
        const auto& arc = std::get<ArcSpan>(out.spans[1]);
        CHECK_THAT(arc.radius, WithinAbs(3.0, 1e-12));
        CHECK(approxEqual(arc.centre, {8.0, 2.0}));
    }
    SECTION("inside offset smaller than the radius shrinks the arc") {
        const Contour out = offsetOpenContour(c, 1.0, OffsetSide::Left, JoinType::Round, kTol);
        REQUIRE(out.spanCount() == 3);
        CHECK(out.isContinuous(kTol));
        CHECK_THAT(std::get<ArcSpan>(out.spans[1]).radius, WithinAbs(1.0, 1e-12));
    }
    SECTION("inside offset beyond the radius collapses the arc into a sharp corner") {
        const Contour out = offsetOpenContour(c, 3.0, OffsetSide::Left, JoinType::Round, kTol);
        REQUIRE(out.spanCount() == 2);
        CHECK(out.isContinuous(kTol));
        CHECK(countArcs(out) == 0);
        CHECK(approxEqual(endPoint(out.spans[0]), {7.0, 3.0}));
    }
}

TEST_CASE("Open contour offset edge cases", "[geometry][offset][open]") {
    SECTION("empty contour") {
        const Contour out = offsetOpenContour(Contour{}, 1.0, OffsetSide::Left, JoinType::Round, kTol);
        CHECK(out.isEmpty());
    }
    SECTION("degenerate spans are ignored") {
        Contour c;
        c.spans.emplace_back(LineSpan{{0.0, 0.0}, {0.0, 0.0}});
        c.spans.emplace_back(LineSpan{{0.0, 0.0}, {5.0, 0.0}});
        const Contour out = offsetOpenContour(c, 1.0, OffsetSide::Left, JoinType::Round, kTol);
        REQUIRE(out.spanCount() == 1);
        CHECK(approxEqual(out.startPoint(), {0.0, 1.0}));
    }
    SECTION("zero distance returns the input geometry") {
        Contour c;
        c.spans.emplace_back(LineSpan{{0.0, 0.0}, {5.0, 0.0}});
        const Contour out = offsetOpenContour(c, 0.0, OffsetSide::Left, JoinType::Round, kTol);
        CHECK(out.spans == c.spans);
    }
    SECTION("Bezier spans are polygonised, result stays within the chordal tolerance band") {
        Contour c;
        c.spans.emplace_back(BezierSpan{{0.0, 0.0}, {5.0, 10.0}, {15.0, 10.0}, {20.0, 0.0}});
        const Contour out = offsetOpenContour(c, 1.0, OffsetSide::Right, JoinType::Round, kTol);
        REQUIRE(out.spanCount() >= 2);
        CHECK(out.isContinuous(kTol));
        const Polyline orig = c.polygonise(Tolerance{0.001, 1e-9, 0.0005});
        for (int i = 0; i <= 100; ++i) {
            CHECK_THAT(distanceToPolyline(out.pointAt(i / 100.0), orig), WithinAbs(1.0, 2.0 * kTol.chordal));
        }
    }
    SECTION("full reversal (spike) gets a half-circle round join") {
        Contour c;
        c.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
        c.spans.emplace_back(LineSpan{{10.0, 0.0}, {0.0, 0.0}});
        const Contour out = offsetOpenContour(c, 1.0, OffsetSide::Left, JoinType::Round, kTol);
        REQUIRE(out.spanCount() == 3);
        CHECK(out.isContinuous(kTol));
        CHECK_THAT(std::fabs(std::get<ArcSpan>(out.spans[1]).sweepAngle()), WithinAbs(kPi, 1e-9));
    }
    SECTION("closed input wraps the corner at the start and stays closed") {
        const Contour out = offsetOpenContour(square10(), 1.0, OffsetSide::Right, JoinType::Sharp, kTol);
        CHECK(out.closed);
        REQUIRE(out.spanCount() == 4);
        CHECK(out.isContinuous(kTol));
        CHECK_THAT(area(out), WithinAbs(144.0, 1e-9));
        const Contour in = offsetOpenContour(square10(), 1.0, OffsetSide::Left, JoinType::Sharp, kTol);
        REQUIRE(in.spanCount() == 4);
        CHECK_THAT(area(in), WithinAbs(64.0, 1e-9));
    }
}

// --- Acceptance criterion 5 (amended wording) -----------------------------------------------
//
// Span count = n - (spans consumed at inside corners) + one join span per outside corner for
// Round/Square (and per mitre-limit fallback for Sharp). Every result point is >= d - tol from
// the input; offset vertices and arcs lie at exactly d.

namespace {

Contour openPolyline(std::initializer_list<Vec2> pts) {
    const std::vector<Vec2> v(pts);
    Contour c;
    for (std::size_t i = 1; i < v.size(); ++i) {
        c.spans.emplace_back(LineSpan{v[i - 1], v[i]});
    }
    return c;
}

// Smallest distance from p to the input polyline (exact for line-only inputs).
double distanceToInput(Vec2 p, const Contour& input) {
    return distanceToPolyline(p, input.polygonise(kTol));
}

// Densely samples the result and returns {min, max} distance to the input.
std::pair<double, double> distanceRange(const Contour& result, const Contour& input) {
    double lo = 1e300;
    double hi = 0.0;
    for (int i = 0; i <= 4000; ++i) {
        const double dist = distanceToInput(result.pointAt(i / 4000.0), input);
        lo = std::min(lo, dist);
        hi = std::max(hi, dist);
    }
    return {lo, hi};
}

}  // namespace

TEST_CASE("AC5: U-shape offset to the outside adds one join span per outside corner",
          "[geometry][offset][open][ac5]") {
    // Verifier repro: (0,0)->(10,0)->(10,10)->(0,10), Right side, d=1. Both corners turn left,
    // so the right side is outside at both: n=3, no trimmed corners, +2 joins.
    const Contour u = openPolyline({{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}});
    const double d = 1.0;

    SECTION("Round: 3 + 2 arc joins = 5 spans, every point exactly at d") {
        const Contour out = offsetOpenContour(u, d, OffsetSide::Right, JoinType::Round, kTol);
        CHECK_FALSE(out.closed);
        REQUIRE(out.spanCount() == 5);
        CHECK(out.isContinuous(kTol));
        CHECK(countArcs(out) == 2);
        const auto& a1 = std::get<ArcSpan>(out.spans[1]);
        const auto& a2 = std::get<ArcSpan>(out.spans[3]);
        CHECK(approxEqual(a1.centre, {10.0, 0.0}));
        CHECK(approxEqual(a2.centre, {10.0, 10.0}));
        CHECK_THAT(a1.radius, WithinAbs(d, 1e-12));
        CHECK_THAT(a2.radius, WithinAbs(d, 1e-12));
        const auto [lo, hi] = distanceRange(out, u);
        CHECK_THAT(lo, WithinAbs(d, 1e-9));
        CHECK_THAT(hi, WithinAbs(d, 1e-9));
        CHECK(approxEqual(out.startPoint(), {0.0, -1.0}));
        CHECK(approxEqual(out.endPoint(), {0.0, 11.0}));
    }
    SECTION("Square: 3 + 2 chamfer joins = 5 spans, chamfers exactly d from the corners") {
        const Contour out = offsetOpenContour(u, d, OffsetSide::Right, JoinType::Square, kTol);
        REQUIRE(out.spanCount() == 5);
        CHECK(out.isContinuous(kTol));
        CHECK(countArcs(out) == 0);
        // Chamfer midpoints sit on the corner bisector at exactly d.
        CHECK_THAT(distance(pointAt(out.spans[1], 0.5), {10.0, 0.0}), WithinAbs(d, 1e-9));
        CHECK_THAT(distance(pointAt(out.spans[3], 0.5), {10.0, 10.0}), WithinAbs(d, 1e-9));
        const auto [lo, hi] = distanceRange(out, u);
        CHECK(lo >= d - kTol.distance);
        CHECK_THAT(lo, WithinAbs(d, 1e-9));
        CHECK(hi <= d * std::sqrt(2.0) + 1e-9);  // never beyond the mitre point
    }
    SECTION("Sharp (right angles are within the default mitre limit): 3 + 0 = 3 spans") {
        const Contour out = offsetOpenContour(u, d, OffsetSide::Right, JoinType::Sharp, kTol);
        REQUIRE(out.spanCount() == 3);
        CHECK(approxEqual(endPoint(out.spans[0]), {11.0, -1.0}));
        CHECK(approxEqual(endPoint(out.spans[1]), {11.0, 11.0}));
        const auto [lo, hi] = distanceRange(out, u);
        CHECK_THAT(lo, WithinAbs(d, 1e-9));
        CHECK(hi <= d * std::sqrt(2.0) + 1e-9);
    }
    SECTION("Left side: both corners inside, trimmed, no spans consumed -> 3 spans at exactly d") {
        for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
            const Contour out = offsetOpenContour(u, d, OffsetSide::Left, join, kTol);
            REQUIRE(out.spanCount() == 3);
            CHECK(approxEqual(endPoint(out.spans[0]), {9.0, 1.0}));
            CHECK(approxEqual(endPoint(out.spans[1]), {9.0, 9.0}));
            const auto [lo, hi] = distanceRange(out, u);
            CHECK_THAT(lo, WithinAbs(d, 1e-9));
            CHECK_THAT(hi, WithinAbs(d, 1e-9));
        }
    }
}

TEST_CASE("AC5: acute zig-zag with Sharp joins falls back to chamfers past the mitre limit",
          "[geometry][offset][open][ac5]") {
    // Verifier repro: (0,0)->(1,10)->(2,0)->(3,10)->(4,0), Left side, d=0.5, mitreLimit=2.
    // Peaks (1,10) and (3,10) turn right, so the left side is outside there; the mitre would
    // reach ~5.0 = 10*d from the vertex, beyond 2*d, so each falls back to a square chamfer
    // (+1 span each). The valley (2,0) turns left: inside, trimmed, no span consumed.
    // Expected: 4 - 0 + 2 = 6 spans.
    const Contour zig = openPolyline({{0.0, 0.0}, {1.0, 10.0}, {2.0, 0.0}, {3.0, 10.0}, {4.0, 0.0}});
    const double d = 0.5;

    const Contour out = offsetOpenContour(zig, d, OffsetSide::Left, JoinType::Sharp, kTol, 2.0);
    CHECK_FALSE(out.closed);
    REQUIRE(out.spanCount() == 6);
    CHECK(out.isContinuous(kTol));
    // The chamfer spans are horizontal at y = 10 + d, exactly d above each peak.
    CHECK_THAT(pointAt(out.spans[1], 0.5).y, WithinAbs(10.0 + d, 1e-9));
    CHECK_THAT(pointAt(out.spans[4], 0.5).y, WithinAbs(10.0 + d, 1e-9));
    CHECK_THAT(distance(pointAt(out.spans[1], 0.5), {1.0, 10.0}), WithinAbs(d, 1e-9));
    CHECK_THAT(distance(pointAt(out.spans[4], 0.5), {3.0, 10.0}), WithinAbs(d, 1e-9));
    // The valley trim point lies at exactly d from both adjacent input segments.
    const Vec2 valley = endPoint(out.spans[2]);
    CHECK_THAT(distanceToInput(valley, zig), WithinAbs(d, 1e-9));
    CHECK(approxEqual(valley, startPoint(out.spans[3])));
    // Distance property: never closer than d, never past the mitre-limit bound.
    const auto [lo, hi] = distanceRange(out, zig);
    CHECK(lo >= d - kTol.distance);
    CHECK_THAT(lo, WithinAbs(d, 1e-9));
    CHECK(hi <= 2.0 * d + 1e-9);

    SECTION("a mitre limit large enough keeps the sharp corners: 4 + 0 = 4 spans") {
        const Contour sharp = offsetOpenContour(zig, d, OffsetSide::Left, JoinType::Sharp, kTol, 20.0);
        REQUIRE(sharp.spanCount() == 4);
        CHECK(distanceRange(sharp, zig).first >= d - 1e-9);
    }
    SECTION("Round joins: 4 + 2 arcs = 6 spans, all points exactly at d") {
        const Contour round = offsetOpenContour(zig, d, OffsetSide::Left, JoinType::Round, kTol);
        REQUIRE(round.spanCount() == 6);
        CHECK(countArcs(round) == 2);
        const auto [rlo, rhi] = distanceRange(round, zig);
        CHECK_THAT(rlo, WithinAbs(d, 1e-9));
        CHECK_THAT(rhi, WithinAbs(d, 1e-9));
    }
    SECTION("Right side: valleys at (2,0) is outside, peaks inside -> 4 + 1 = 5 spans") {
        const Contour right = offsetOpenContour(zig, d, OffsetSide::Right, JoinType::Sharp, kTol, 2.0);
        REQUIRE(right.spanCount() == 5);
        CHECK(distanceRange(right, zig).first >= d - kTol.distance);
    }
}

TEST_CASE("AC5: an inside corner that consumes a span reduces the count", "[geometry][offset][open][ac5]") {
    // Line, r=2 fillet, line turning left. Offsetting 3 mm to the left (inside) collapses the
    // fillet, so the two lines are trimmed to meet directly: 3 - 1 = 2 spans.
    Contour c;
    c.spans.emplace_back(LineSpan{{0.0, 0.0}, {8.0, 0.0}});
    c.spans.emplace_back(ArcSpan::fromCentre({8.0, 2.0}, {8.0, 0.0}, {10.0, 2.0}, ArcDirection::CCW));
    c.spans.emplace_back(LineSpan{{10.0, 2.0}, {10.0, 10.0}});
    const double d = 3.0;

    const Contour in = offsetOpenContour(c, d, OffsetSide::Left, JoinType::Round, kTol);
    REQUIRE(in.spanCount() == 2);
    CHECK(in.isContinuous(kTol));
    CHECK(approxEqual(endPoint(in.spans[0]), {7.0, 3.0}));
    const auto [lo, hi] = distanceRange(in, c);
    CHECK(lo >= d - kTol.chordal);  // input arc is polygonised for the measurement
    CHECK(hi <= d + kTol.chordal + 1e-9);

    // The outside of the same path keeps every span and offsets the arc to exactly r + d.
    const Contour out = offsetOpenContour(c, d, OffsetSide::Right, JoinType::Round, kTol);
    REQUIRE(out.spanCount() == 3);
    const auto& arc = std::get<ArcSpan>(out.spans[1]);
    CHECK_THAT(arc.radius, WithinAbs(2.0 + d, 1e-12));
    CHECK(approxEqual(arc.centre, {8.0, 2.0}));
    for (int i = 0; i <= 20; ++i) {
        CHECK_THAT(distance(arc.pointAt(i / 20.0), arc.centre) - 2.0, WithinAbs(d, 1e-12));
    }
}

TEST_CASE("Offsets reject non-finite distances", "[geometry][offset]") {
    const double nan = std::nan("");
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(offsetContours({square10()}, nan, JoinType::Round, kTol).empty());
    CHECK(offsetContours({square10()}, inf, JoinType::Round, kTol).empty());
    Contour line;
    line.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    CHECK(offsetOpenContour(line, nan, OffsetSide::Left, JoinType::Round, kTol).isEmpty());
    CHECK(offsetOpenContour(line, -inf, OffsetSide::Left, JoinType::Round, kTol).isEmpty());
}

// --- AC5 round 3: arc corners, consumed spans, generative coverage ---------------------------

namespace {

// Exact distance from p to a line or arc span.
double exactDistanceToSpan(Vec2 p, const Span& s) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 ab = line->end - line->start;
        const double t = std::clamp((p - line->start).dot(ab) / ab.lengthSquared(), 0.0, 1.0);
        return distance(p, line->start + ab * t);
    }
    const auto& arc = std::get<ArcSpan>(s);
    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
        return std::fabs(distance(p, arc.centre) - arc.radius);
    }
    return std::min(distance(p, arc.startPoint()), distance(p, arc.endPoint()));
}

double exactDistanceToPath(Vec2 p, const Contour& path) {
    double best = 1e300;
    for (const Span& s : path.spans) {
        best = std::min(best, exactDistanceToSpan(p, s));
    }
    return best;
}

struct OffsetCheck {
    double minDistance{1e300};
    double maxDistance{0.0};
    bool continuous{false};
    bool degenerateSpan{false};
    bool cusp{false};
    bool selfIntersecting{false};
};

// Samples every output span against the input (exact distances) and checks the shape of the
// chain: continuity, no degenerate spans, no 180-degree reversal between consecutive spans and
// no self-intersection (which is how backtracking shows up).
OffsetCheck inspectOffset(const Contour& out, const Contour& input) {
    OffsetCheck r;
    r.continuous = out.isContinuous(kTol);
    for (const Span& s : out.spans) {
        r.degenerateSpan = r.degenerateSpan || isDegenerate(s, kTol);
        for (int i = 0; i <= 40; ++i) {
            const double dist = exactDistanceToPath(pointAt(s, i / 40.0), input);
            r.minDistance = std::min(r.minDistance, dist);
            r.maxDistance = std::max(r.maxDistance, dist);
        }
    }
    for (std::size_t k = 1; k < out.spans.size(); ++k) {
        if (tangentAt(out.spans[k - 1], 1.0).dot(tangentAt(out.spans[k], 0.0)) < -0.9999) {
            r.cusp = true;
        }
    }
    const Polyline pl = out.polygonise(Tolerance{0.001, 1e-9, 0.002});
    const std::size_t n = pl.segmentCount();
    for (std::size_t i = 0; i < n && !r.selfIntersecting; ++i) {
        for (std::size_t j = i + 2; j < n; ++j) {
            const Vec2 a = pl.segmentStart(i);
            const Vec2 u = pl.segmentEnd(i) - a;
            const Vec2 c = pl.segmentStart(j);
            const Vec2 v = pl.segmentEnd(j) - c;
            const double den = u.cross(v);
            if (std::fabs(den) < 1e-9 * u.length() * v.length()) {
                continue;
            }
            const double t = (c - a).cross(v) / den;
            const double w = (c - a).cross(u) / den;
            if (t > 1e-9 && t < 1.0 - 1e-9 && w > 1e-9 && w < 1.0 - 1e-9) {
                r.selfIntersecting = true;
                break;
            }
        }
    }
    return r;
}

// Asserts the AC5 distance property and chain sanity for one offset result.
void requireValidOpenOffset(const Contour& out, const Contour& input, double d, JoinType join,
                            double mitreLimit) {
    const OffsetCheck c = inspectOffset(out, input);
    CHECK_FALSE(out.closed);
    CHECK_FALSE(out.isEmpty());
    CHECK(c.continuous);
    CHECK_FALSE(c.degenerateSpan);
    CHECK_FALSE(c.cusp);
    CHECK_FALSE(c.selfIntersecting);
    CHECK(c.minDistance >= d - 1e-6);
    if (join == JoinType::Round) {
        CHECK(c.maxDistance <= d + 1e-6);  // round offsets lie at exactly d everywhere
    } else if (join == JoinType::Sharp) {
        CHECK(c.maxDistance <= std::max(mitreLimit, 1.5) * d + 1e-6);
    } else {
        CHECK(c.maxDistance <= 1.5 * d + 1e-6);
    }
}

Contour repro1() {
    Contour c;
    c.spans.emplace_back(LineSpan{{0.0, 0.0}, {10.0, 0.0}});
    c.spans.emplace_back(ArcSpan::fromCentre({10.0, 5.0}, {10.0, 0.0}, {15.0, 5.0}, ArcDirection::CCW));
    c.spans.emplace_back(LineSpan{{15.0, 5.0}, {20.0, 15.0}});
    c.spans.emplace_back(ArcSpan::fromCentre({25.0, 15.0}, {20.0, 15.0}, {30.0, 15.0}, ArcDirection::CW));
    c.spans.emplace_back(LineSpan{{30.0, 15.0}, {40.0, 5.0}});
    return c;
}

Contour repro2() {
    Contour c;
    c.spans.emplace_back(ArcSpan::fromCentre({0.0, 10.0}, {0.0, 0.0}, {10.0, 10.0}, ArcDirection::CCW));
    c.spans.emplace_back(ArcSpan::fromCentre({20.0, 10.0}, {10.0, 10.0}, {20.0, 20.0}, ArcDirection::CW));
    c.spans.emplace_back(LineSpan{{20.0, 20.0}, {30.0, 10.0}});
    return c;
}

struct ExpectedCounts {
    std::size_t roundLeft, roundRight, sharpLeft, sharpRight, squareLeft, squareRight;
    std::size_t get(JoinType j, OffsetSide s) const {
        const bool left = s == OffsetSide::Left;
        switch (j) {
            case JoinType::Round:
                return left ? roundLeft : roundRight;
            case JoinType::Sharp:
                return left ? sharpLeft : sharpRight;
            case JoinType::Square:
                return left ? squareLeft : squareRight;
        }
        return 0;
    }
};

const JoinType kAllJoins[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};
const OffsetSide kBothSides[] = {OffsetSide::Left, OffsetSide::Right};

}  // namespace

TEST_CASE("AC5: outside corners involving arcs cost at most one join span", "[geometry][offset][open][ac5]") {
    // Repro 1 (d=1): corners are line->arc (tangent), arc->line (right turn), line->arc (left
    // turn), arc->line (left turn). Round/Square add one span per outside corner; Sharp extends
    // the line or the arc itself to the mitre point and adds none.
    SECTION("repro 1") {
        const Contour in = repro1();
        const ExpectedCounts expected{6, 7, 5, 5, 6, 7};
        for (const JoinType join : kAllJoins) {
            for (const OffsetSide side : kBothSides) {
                CAPTURE(static_cast<int>(join), static_cast<int>(side));
                const Contour out = offsetOpenContour(in, 1.0, side, join, kTol);
                CHECK(out.spanCount() == expected.get(join, side));
                requireValidOpenOffset(out, in, 1.0, join, 2.0);
            }
        }
    }
    // Repro 2 (d=1.5): arc->arc is tangent-continuous (no corner, no join); arc->line turns right.
    SECTION("repro 2") {
        const Contour in = repro2();
        const ExpectedCounts expected{4, 3, 3, 3, 4, 3};
        for (const JoinType join : kAllJoins) {
            for (const OffsetSide side : kBothSides) {
                CAPTURE(static_cast<int>(join), static_cast<int>(side));
                const Contour out = offsetOpenContour(in, 1.5, side, join, kTol);
                CHECK(out.spanCount() == expected.get(join, side));
                requireValidOpenOffset(out, in, 1.5, join, 2.0);
            }
        }
    }
    SECTION("Sharp at an arc corner extends the arc itself: no tangent stub") {
        const Contour out = offsetOpenContour(repro1(), 1.0, OffsetSide::Left, JoinType::Sharp, kTol);
        REQUIRE(out.spanCount() == 5);
        CHECK(countArcs(out) == 2);
        // Every arc keeps its centre and offset radius (r - d on the left of the CCW arc,
        // r + d on the left of the CW arc).
        CHECK_THAT(std::get<ArcSpan>(out.spans[1]).radius, WithinAbs(4.0, 1e-12));
        CHECK_THAT(std::get<ArcSpan>(out.spans[3]).radius, WithinAbs(6.0, 1e-12));
    }
}

TEST_CASE("AC5: a span consumed between two inside corners is removed", "[geometry][offset][open][ac5]") {
    // Verifier repro: the 1 mm middle span is swallowed when offsetting 2 mm to the left.
    Polyline pl;
    pl.points = {{-10.0, 10.0}, {0.0, 0.0}, {1.0, 0.0}, {11.0, 10.0}};
    const Contour in = Contour::fromPolyline(pl);
    for (const JoinType join : kAllJoins) {
        CAPTURE(static_cast<int>(join));
        const Contour left = offsetOpenContour(in, 2.0, OffsetSide::Left, join, kTol);
        REQUIRE(left.spanCount() == 2);
        // The neighbours meet where x + y = 2*sqrt(2) and y - x = 2*sqrt(2) - 1.
        CHECK(approxEqual(endPoint(left.spans[0]), {0.5, 2.0 * std::sqrt(2.0) - 0.5}, 1e-9));
        requireValidOpenOffset(left, in, 2.0, join, 2.0);
        // Right side: both corners are outside; the middle span survives.
        const Contour right = offsetOpenContour(in, 2.0, OffsetSide::Right, join, kTol);
        CHECK(right.spanCount() == (join == JoinType::Sharp ? 3u : 5u));
        requireValidOpenOffset(right, in, 2.0, join, 2.0);
    }
    SECTION("several consecutive short spans in a narrow valley are all consumed") {
        Polyline valley;
        valley.points = {{-10.0, 10.0}, {0.0, 0.0}, {0.3, -0.1}, {0.6, -0.1}, {0.9, 0.0}, {10.9, 10.0}};
        const Contour v = Contour::fromPolyline(valley);
        for (const JoinType join : kAllJoins) {
            const Contour out = offsetOpenContour(v, 2.0, OffsetSide::Left, join, kTol);
            CHECK(out.spanCount() == 2);
            requireValidOpenOffset(out, v, 2.0, join, 2.0);
        }
    }
    SECTION("a short first or last span pointing inwards is consumed at the path ends") {
        Polyline hook;
        hook.points = {{0.3, -0.5}, {0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {9.7, 10.4}};
        const Contour h = Contour::fromPolyline(hook);
        for (const JoinType join : kAllJoins) {
            const Contour out = offsetOpenContour(h, 1.0, OffsetSide::Left, join, kTol);
            requireValidOpenOffset(out, h, 1.0, join, 2.0);
        }
    }
}

namespace {

// Deterministic generator: std::mt19937's output sequence is specified by the standard, unlike
// the std:: distributions, so the generated shapes are identical on every platform.
struct TestRng {
    std::mt19937 engine;
    explicit TestRng(std::uint32_t seed) : engine(seed) {}
    double unit() { return static_cast<double>(engine()) / 4294967296.0; }
    double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
    int count(int lo, int hi) { return lo + static_cast<int>(unit() * static_cast<double>(hi - lo + 1)); }
    double signedRange(double lo, double hi) { return range(lo, hi) * (unit() < 0.5 ? -1.0 : 1.0); }
};

constexpr double kDeg = kPi / 180.0;

// x-monotone random polyline (headings within +-80 deg), so the one-sided offset is well
// defined and only local interactions occur. `longSegments` keeps every span long enough that
// no corner trim can consume it, and keeps outside turns away from the 120 deg Sharp/Square
// switch-over of mitreLimit = 2 so the expected span count is unambiguous.
Contour randomPolyline(TestRng& rng, bool longSegments, double d) {
    Contour c;
    const int n = rng.count(3, 10);
    Vec2 p{0.0, 0.0};
    double prev = 0.0;
    for (int i = 0; i < n; ++i) {
        double h = 0.0;
        for (;;) {
            h = rng.range(-80.0, 80.0) * kDeg;
            const double turn = std::fabs(h - prev);
            if (i == 0 || (turn > 5.0 * kDeg && std::fabs(turn - 120.0 * kDeg) > 1.0 * kDeg)) {
                break;
            }
        }
        double len = 0.0;
        if (longSegments) {
            len = rng.range(12.0, 20.0) * d;
        } else {
            len = rng.unit() < 0.35 ? rng.range(0.05, 0.5) * d : rng.range(0.5, 4.0) * d;
        }
        const Vec2 q = p + Vec2::fromAngle(h, len);
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
        prev = h;
    }
    return c;
}

// x-monotone random path of lines and arcs. Junctions are either tangent-continuous or kinked.
// `tame` keeps radii well above d, kinks moderate and spans long so the expected span count is
// computable; otherwise arcs may collapse (r < d) and spans may be consumed.
Contour randomMixedPath(TestRng& rng, bool tame, double d) {
    Contour c;
    const int n = rng.count(3, 9);
    const double limit = 80.0 * kDeg;
    Vec2 p{0.0, 0.0};
    double h = rng.range(-60.0, 60.0) * kDeg;
    for (int i = 0; i < n; ++i) {
        if (i > 0 && rng.unit() < 0.5) {
            for (int tries = 0; tries < 50; ++tries) {
                const double kink =
                    tame ? rng.signedRange(10.0, 60.0) * kDeg : rng.signedRange(5.0, 90.0) * kDeg;
                if (std::fabs(h + kink) <= limit) {
                    h += kink;
                    break;
                }
            }
        }
        if (rng.unit() < 0.5) {
            const double len = tame
                                   ? rng.range(12.0, 20.0) * d
                                   : (rng.unit() < 0.35 ? rng.range(0.05, 0.5) * d : rng.range(0.5, 4.0) * d);
            const Vec2 q = p + Vec2::fromAngle(h, len);
            c.spans.emplace_back(LineSpan{p, q});
            p = q;
            continue;
        }
        const double radius = tame ? rng.range(3.0, 6.0) * d : rng.range(0.2, 5.0) * d;
        double sweep = tame ? rng.signedRange(40.0, 70.0) * kDeg : rng.signedRange(10.0, 120.0) * kDeg;
        if (std::fabs(h + sweep) > limit) {
            sweep = -sweep;
        }
        if (std::fabs(h + sweep) > limit) {
            continue;
        }
        const ArcDirection dir = sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
        const Vec2 normal = Vec2::fromAngle(h).perp();
        const Vec2 centre = sweep > 0.0 ? p + normal * radius : p - normal * radius;
        const Vec2 q = centre + Vec2::fromAngle((p - centre).angle() + sweep, radius);
        c.spans.emplace_back(ArcSpan::fromCentre(centre, p, q, dir));
        p = q;
        h += sweep;
    }
    return c;
}

// AC5 count rule: n plus one join span per outside corner for Round/Square, and per mitre-limit
// fallback for Sharp. Valid only when no span is consumed (long spans, tame arcs).
std::size_t expectedSpanCount(const Contour& in, OffsetSide side, JoinType join, double mitreLimit) {
    const double sign = side == OffsetSide::Left ? 1.0 : -1.0;
    std::size_t n = in.spanCount();
    for (std::size_t i = 1; i < in.spans.size(); ++i) {
        const Vec2 a = tangentAt(in.spans[i - 1], 1.0);
        const Vec2 b = tangentAt(in.spans[i], 0.0);
        const double cross = a.cross(b);
        if (std::fabs(cross) < 1e-9 && a.dot(b) > 0.0) {
            continue;  // tangent-continuous: no corner
        }
        if (sign * cross >= 0.0) {
            continue;  // inside corner: trimmed, nothing added
        }
        const double turn = std::acos(std::clamp(a.dot(b), -1.0, 1.0));
        if (join != JoinType::Sharp || 1.0 / std::cos(turn / 2.0) > mitreLimit) {
            ++n;
        }
    }
    return n;
}

}  // namespace

TEST_CASE("AC5 generative: random polylines with short spans keep the distance property",
          "[geometry][offset][open][ac5][generative]") {
    TestRng rng(20261024u);
    for (int trial = 0; trial < 60; ++trial) {
        for (const double d : {0.5, 1.0, 2.5}) {
            const Contour in = randomPolyline(rng, false, d);
            for (const JoinType join : kAllJoins) {
                for (const OffsetSide side : kBothSides) {
                    CAPTURE(trial, d, static_cast<int>(join), static_cast<int>(side));
                    const Contour out = offsetOpenContour(in, d, side, join, kTol, 2.0);
                    requireValidOpenOffset(out, in, d, join, 2.0);
                }
            }
        }
    }
}

TEST_CASE("AC5 generative: random line/arc paths keep the distance property",
          "[geometry][offset][open][ac5][generative]") {
    TestRng rng(424242u);
    for (int trial = 0; trial < 60; ++trial) {
        for (const double d : {0.5, 1.0, 2.5}) {
            const Contour in = randomMixedPath(rng, false, d);
            if (in.isEmpty()) {
                continue;
            }
            for (const JoinType join : kAllJoins) {
                for (const OffsetSide side : kBothSides) {
                    CAPTURE(trial, d, static_cast<int>(join), static_cast<int>(side));
                    const Contour out = offsetOpenContour(in, d, side, join, kTol, 2.0);
                    requireValidOpenOffset(out, in, d, join, 2.0);
                }
            }
        }
    }
}

TEST_CASE("AC5 generative: span count follows the rule when nothing is consumed",
          "[geometry][offset][open][ac5][generative]") {
    TestRng rng(7u);
    for (int trial = 0; trial < 60; ++trial) {
        for (const double d : {0.5, 1.0, 2.5}) {
            const Contour poly = randomPolyline(rng, true, d);
            const Contour mixed = randomMixedPath(rng, true, d);
            for (const JoinType join : kAllJoins) {
                for (const OffsetSide side : kBothSides) {
                    CAPTURE(trial, d, static_cast<int>(join), static_cast<int>(side));
                    const Contour a = offsetOpenContour(poly, d, side, join, kTol, 2.0);
                    CHECK(a.spanCount() == expectedSpanCount(poly, side, join, 2.0));
                    requireValidOpenOffset(a, poly, d, join, 2.0);
                    if (mixed.isEmpty()) {
                        continue;
                    }
                    const Contour b = offsetOpenContour(mixed, d, side, join, kTol, 2.0);
                    CHECK(b.spanCount() == expectedSpanCount(mixed, side, join, 2.0));
                    requireValidOpenOffset(b, mixed, d, join, 2.0);
                }
            }
        }
    }
}
