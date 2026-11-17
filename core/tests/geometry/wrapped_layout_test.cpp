// T-0322: Wrapped Fluting and Spiral layouts (wrapped_layout.hpp, ADR-0115): exact flute counts and spacing,
// outline shapes, strand counts, pitch, seam splitting, twist on the cylinder as wrapped by CylinderMap, and
// hostile input.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <numbers>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/cylinder_map.hpp"
#include "carvetto/geometry/wrapped_layout.hpp"

using namespace carvetto::geometry;
using carvetto::geometry::kPi;
using Catch::Approx;

namespace {


const LineSpan& line(const Contour& c, std::size_t i = 0) { return std::get<LineSpan>(c.spans[i]); }

}  // namespace

TEST_CASE("fluting layout: flutes are evenly spaced centre lines along the axis", "[wrapped][layout]") {
    WrappedFlutingParams p;
    p.diameter = 40.0;  // circumference 40 pi
    p.flutes = 8;
    p.alongStart = 10.0;
    p.alongEnd = 90.0;
    const WrappedFlutingResult r = wrappedFlutingLayout(p);
    REQUIRE(r.ok());
    CHECK(r.spacing == Approx(40.0 * kPi / 8.0));
    REQUIRE(r.contours.size() == 8);
    for (std::size_t i = 0; i < 8; ++i) {
        const Contour& c = r.contours[i];
        REQUIRE(c.spans.size() == 1);
        CHECK(!c.closed);
        CHECK(line(c).start.x == 10.0);
        CHECK(line(c).end.x == 90.0);
        CHECK(line(c).start.y == Approx(r.spacing * static_cast<double>(i)));
        CHECK(line(c).end.y == line(c).start.y);
    }
    CHECK(r.crossingSeam.empty());
}

TEST_CASE("fluting layout: along Y swaps the axes; start and offset", "[wrapped][layout]") {
    WrappedFlutingParams p;
    p.alongX = false;
    p.diameter = 20.0;
    p.flutes = 3;
    p.alongStart = 0.0;
    p.alongEnd = 50.0;
    p.circStart = 5.0;
    p.offset = 2.0;
    const WrappedFlutingResult r = wrappedFlutingLayout(p);
    REQUIRE(r.ok());
    REQUIRE(r.contours.size() == 3);
    const double s = 20.0 * kPi / 3.0;
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(line(r.contours[i]).start.y == 0.0);
        CHECK(line(r.contours[i]).end.y == 50.0);
        CHECK(line(r.contours[i]).start.x == Approx(5.0 + 2.0 + s * static_cast<double>(i)));
    }
    // An offset beyond the turn, or negative, wraps modulo the turn into [start, start + turn).
    p.offset = 20.0 * kPi + 2.0;
    CHECK(line(wrappedFlutingLayout(p).contours[0]).start.x == Approx(7.0));
    p.offset = -2.0;
    CHECK(line(wrappedFlutingLayout(p).contours[0]).start.x == Approx(5.0 + 20.0 * kPi - 2.0));
    CHECK(wrappedFlutingLayout(p).contours[0].spans.size() == 1);
}

TEST_CASE("fluting layout: rectangle and stadium outlines", "[wrapped][layout]") {
    WrappedFlutingParams p;
    p.diameter = 40.0;
    p.flutes = 4;
    p.alongStart = 0.0;
    p.alongEnd = 60.0;
    p.offset = 0.5 * 40.0 * kPi / 4.0;  // flutes centred mid-way, clear of the seam
    p.width = 6.0;
    p.shape = FluteShape::Rectangle;
    WrappedFlutingResult r = wrappedFlutingLayout(p);
    REQUIRE(r.ok());
    REQUIRE(r.contours.size() == 4);
    CHECK(r.crossingSeam.empty());
    const double c0 = 0.5 * 40.0 * kPi / 4.0;
    Contour& rect = r.contours[0];
    CHECK(rect.closed);
    CHECK(rect.spans.size() == 4);
    CHECK(rect.isContinuous(Tolerance::defaults()));
    CHECK(rect.bounds().min.x == 0.0);
    CHECK(rect.bounds().max.x == 60.0);
    CHECK(rect.bounds().min.y == Approx(c0 - 3.0));
    CHECK(rect.bounds().max.y == Approx(c0 + 3.0));
    CHECK(rect.length() == Approx(2.0 * 60.0 + 2.0 * 6.0));

    p.shape = FluteShape::Stadium;
    r = wrappedFlutingLayout(p);
    REQUIRE(r.ok());
    const Contour st = r.contours[1];
    CHECK(st.closed);
    REQUIRE(st.spans.size() == 4);
    CHECK(st.isContinuous(Tolerance::defaults()));
    CHECK(std::holds_alternative<ArcSpan>(st.spans[1]));
    CHECK(std::holds_alternative<ArcSpan>(st.spans[3]));
    CHECK(std::get<ArcSpan>(st.spans[1]).radius == Approx(3.0));
    CHECK(st.bounds().min.x == Approx(0.0));
    CHECK(st.bounds().max.x == Approx(60.0));
    const double c1 = c0 + 40.0 * kPi / 4.0;
    CHECK(st.bounds().min.y == Approx(c1 - 3.0));
    CHECK(st.bounds().max.y == Approx(c1 + 3.0));
    CHECK(st.length() == Approx(2.0 * (60.0 - 6.0) + 2.0 * kPi * 3.0));

    // Along Y the same outline is mirrored in the swap; arcs stay continuous and bounds follow.
    p.alongX = false;
    r = wrappedFlutingLayout(p);
    REQUIRE(r.ok());
    const Contour& sy = r.contours[1];
    CHECK(sy.isContinuous(Tolerance::defaults()));
    CHECK(sy.bounds().min.y == Approx(0.0));
    CHECK(sy.bounds().max.y == Approx(60.0));
    CHECK(sy.bounds().min.x == Approx(c1 - 3.0));
    CHECK(sy.length() == Approx(st.length()));
    // The arc ends bulge outwards (the outline's mid points lie within the bounds, tangent to them).
    const ArcSpan& cap = std::get<ArcSpan>(sy.spans[1]);
    CHECK(cap.bounds().max.y == Approx(60.0));
    CHECK(cap.bounds().min.y == Approx(57.0));
}

TEST_CASE("fluting layout: seam crossing is reported", "[wrapped][layout]") {
    WrappedFlutingParams p;
    p.diameter = 40.0;
    p.flutes = 4;
    p.shape = FluteShape::Rectangle;
    p.width = 6.0;
    p.offset = 0.0;  // flute 0 sits on the start of the turn: half its width is outside
    const WrappedFlutingResult r = wrappedFlutingLayout(p);
    REQUIRE(r.ok());
    REQUIRE(r.crossingSeam.size() == 1);
    CHECK(r.crossingSeam[0] == 0);
    // A flute whose upper edge reaches past the end of the turn.
    p.offset = 40.0 * kPi / 4.0 * 0.5;
    p.circStart = 1.0;
    CHECK(wrappedFlutingLayout(p).crossingSeam.empty());
    p.width = 40.0 * kPi / 4.0;  // as wide as the spacing: touching neighbours, still allowed
    const WrappedFlutingResult wide = wrappedFlutingLayout(p);
    REQUIRE(wide.ok());
    CHECK(wide.crossingSeam.empty());  // flutes centred mid-way fit exactly
    p.offset = 0.5 * 40.0 * kPi / 4.0 + 0.1;
    CHECK(wrappedFlutingLayout(p).crossingSeam.size() == 1);
    // Centre lines never cross.
    p.shape = FluteShape::CentreLine;
    CHECK(wrappedFlutingLayout(p).crossingSeam.empty());
}

TEST_CASE("fluting layout: refusals", "[wrapped][layout]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto bad = [&](auto edit) {
        WrappedFlutingParams p;
        edit(p);
        const WrappedFlutingResult r = wrappedFlutingLayout(p);
        CHECK(!r.ok());
        CHECK(r.contours.empty());
    };
    bad([&](WrappedFlutingParams& p) { p.diameter = nan; });
    bad([](WrappedFlutingParams& p) { p.diameter = 0.0; });
    bad([](WrappedFlutingParams& p) { p.diameter = 1e6; });
    bad([](WrappedFlutingParams& p) { p.flutes = 0; });
    bad([](WrappedFlutingParams& p) { p.flutes = kMaxLayoutStrands + 1; });
    bad([](WrappedFlutingParams& p) { p.alongEnd = p.alongStart; });
    bad([&](WrappedFlutingParams& p) { p.offset = nan; });
    bad([](WrappedFlutingParams& p) { p.depth = 0.0; });
    bad([](WrappedFlutingParams& p) { p.depth = p.diameter / 2.0; });  // reaches the axis
    bad([](WrappedFlutingParams& p) { p.shape = FluteShape::Rectangle; p.width = 0.0; });
    bad([&](WrappedFlutingParams& p) { p.shape = FluteShape::Rectangle; p.width = nan; });
    bad([](WrappedFlutingParams& p) {  // wider than the spacing 50 pi / 4 = 39.3
        p.shape = FluteShape::Rectangle;
        p.width = 40.0;
    });
    bad([](WrappedFlutingParams& p) {  // shorter than wide: no room for the round ends
        p.shape = FluteShape::Stadium;
        p.alongEnd = 3.0;
        p.width = 4.0;
    });
    WrappedFlutingParams ok;
    ok.shape = FluteShape::Stadium;
    ok.alongEnd = 4.0;
    ok.width = 4.0;  // exactly its width: two touching semicircles
    CHECK(wrappedFlutingLayout(ok).ok());
    ok.flutes = kMaxLayoutStrands;
    ok.shape = FluteShape::CentreLine;
    CHECK(wrappedFlutingLayout(ok).contours.size() == static_cast<std::size_t>(kMaxLayoutStrands));
}

TEST_CASE("spiral layout: one strand makes a straight flat line per turn", "[wrapped][layout]") {
    WrappedSpiralParams p;
    p.diameter = 40.0;  // circumference C = 40 pi
    p.strands = 1;
    p.alongStart = 0.0;
    p.alongEnd = 100.0;
    p.mode = SpiralPitchMode::Pitch;
    p.value = 25.0;  // four turns
    p.twist = SpiralTwist::LeftHand;  // alongX, no flip: the circumference coordinate increases with X
    const WrappedSpiralResult r = wrappedSpiralLayout(p);
    REQUIRE(r.ok());
    const double C = 40.0 * kPi;
    CHECK(r.pitch == 25.0);
    CHECK(r.turns == Approx(4.0));
    REQUIRE(r.segments.size() == 1);
    CHECK(r.segments[0] == 4);  // starts on the seam: four pieces of one turn each
    REQUIRE(r.contours.size() == 4);
    for (std::size_t k = 0; k < 4; ++k) {
        const LineSpan& s = line(r.contours[k]);
        CHECK(s.start.x == Approx(25.0 * static_cast<double>(k)));
        CHECK(s.end.x == Approx(25.0 * static_cast<double>(k + 1)));
        CHECK(s.start.y == Approx(0.0));
        CHECK(s.end.y == Approx(C));
        CHECK(!r.contours[k].closed);
    }
    CHECK(line(r.contours[3]).end.x == 100.0);
}

TEST_CASE("spiral layout: a start inside the turn splits the strand at the seam", "[wrapped][layout]") {
    WrappedSpiralParams p;
    p.diameter = 40.0;
    p.alongStart = 0.0;
    p.alongEnd = 60.0;
    p.value = 30.0;  // two turns
    p.twist = SpiralTwist::LeftHand;
    p.offset = 0.25 * 40.0 * kPi;  // starts a quarter way round
    WrappedSpiralResult r = wrappedSpiralLayout(p);
    REQUIRE(r.ok());
    const double C = 40.0 * kPi;
    // Reaches the end of the turn after 3/4 of a pitch, then every pitch: pieces at 0..22.5, 22.5..52.5, 52.5..60.
    REQUIRE(r.contours.size() == 3);
    CHECK(line(r.contours[0]).start.x == 0.0);
    CHECK(line(r.contours[0]).start.y == Approx(0.25 * C));
    CHECK(line(r.contours[0]).end.x == Approx(22.5));
    CHECK(line(r.contours[0]).end.y == Approx(C));
    CHECK(line(r.contours[1]).start.x == Approx(22.5));
    CHECK(line(r.contours[1]).start.y == Approx(0.0));
    CHECK(line(r.contours[1]).end.x == Approx(52.5));
    CHECK(line(r.contours[1]).end.y == Approx(C));
    CHECK(line(r.contours[2]).start.x == Approx(52.5));
    CHECK(line(r.contours[2]).start.y == Approx(0.0));
    CHECK(line(r.contours[2]).end.x == 60.0);
    CHECK(line(r.contours[2]).end.y == Approx(0.25 * C));
    // The slope is the same everywhere: circumference C per pitch.
    for (const Contour& c : r.contours) {
        const LineSpan& s = line(c);
        CHECK((s.end.y - s.start.y) / (s.end.x - s.start.x) == Approx(C / 30.0));
    }
    // The other twist runs the other way: starts at the same place, crossing the start of the turn first.
    p.twist = SpiralTwist::RightHand;
    r = wrappedSpiralLayout(p);
    REQUIRE(r.contours.size() == 3);
    CHECK(line(r.contours[0]).start.y == Approx(0.25 * C));
    CHECK(line(r.contours[0]).end.y == Approx(0.0));
    CHECK(line(r.contours[0]).end.x == Approx(7.5));  // a quarter of the way down, a quarter of the pitch
    CHECK(line(r.contours[1]).start.y == Approx(C));
    CHECK(line(r.contours[1]).end.y == Approx(0.0));
    CHECK(line(r.contours[1]).end.x == Approx(37.5));
    CHECK(line(r.contours[2]).start.y == Approx(C));
    CHECK(line(r.contours[2]).end.x == 60.0);
    CHECK(line(r.contours[2]).end.y == Approx(0.25 * C));  // two whole turns: back where it started
}

TEST_CASE("spiral layout: a sliver at the seam is not a vector", "[wrapped][layout]") {
    WrappedSpiralParams p;
    p.diameter = 40.0;
    p.alongEnd = 2.0;
    p.value = 0.5;  // four turns
    p.twist = SpiralTwist::LeftHand;
    const double C = 40.0 * kPi;
    p.offset = C * (1.0 - 1.5e-9);  // reaches the seam after 0.75 nanometres
    const WrappedSpiralResult r = wrappedSpiralLayout(p);
    REQUIRE(r.ok());
    CHECK(r.contours.size() == 4);
    CHECK(r.segments[0] == 4);
    for (const Contour& c : r.contours) {
        CHECK(std::fabs(line(c).end.x - line(c).start.x) > 0.4);
    }
}

TEST_CASE("spiral layout: strands, spacing and turns", "[wrapped][layout]") {
    WrappedSpiralParams p;
    p.diameter = 40.0;
    p.strands = 3;
    p.alongStart = 10.0;
    p.alongEnd = 70.0;
    p.mode = SpiralPitchMode::Spacing;
    p.value = 10.0;  // pitch = spacing x strands = 30
    p.twist = SpiralTwist::LeftHand;
    WrappedSpiralResult r = wrappedSpiralLayout(p);
    REQUIRE(r.ok());
    CHECK(r.pitch == 30.0);
    CHECK(r.turns == Approx(2.0));
    REQUIRE(r.segments.size() == 3);
    const double C = 40.0 * kPi;
    // Strand starts are evenly spaced round the circumference.
    std::size_t at = 0;
    for (int s = 0; s < 3; ++s) {
        CHECK(line(r.contours[at]).start.x == 10.0);
        CHECK(line(r.contours[at]).start.y == Approx(C * s / 3.0));
        at += static_cast<std::size_t>(r.segments[static_cast<std::size_t>(s)]);
    }
    CHECK(at == r.contours.size());
    // Neighbouring strands are 10 mm apart along the axis at any circumference coordinate: strand 1's start is
    // reached by strand 0 a third of a pitch further.
    const LineSpan& s0 = line(r.contours[0]);
    const double slope = (s0.end.y - s0.start.y) / (s0.end.x - s0.start.x);
    CHECK((C / 3.0) / slope == Approx(10.0));

    p.mode = SpiralPitchMode::Turns;
    p.value = 3.0;  // 60 mm over 3 turns: pitch 20
    p.strands = 1;
    r = wrappedSpiralLayout(p);
    REQUIRE(r.ok());
    CHECK(r.pitch == Approx(20.0));
    CHECK(r.turns == Approx(3.0));
    CHECK(r.contours.size() == 3);
    p.value = 0.5;  // half a turn: a single piece
    r = wrappedSpiralLayout(p);
    REQUIRE(r.contours.size() == 1);
    CHECK(line(r.contours[0]).end.y == Approx(0.5 * C));
    CHECK(r.segments[0] == 1);
}

TEST_CASE("spiral layout: along Y swaps the axes", "[wrapped][layout]") {
    WrappedSpiralParams p;
    p.alongX = false;
    p.diameter = 40.0;
    p.alongEnd = 50.0;
    p.value = 25.0;
    p.twist = SpiralTwist::RightHand;  // alongY, no flip: circumference increases along Y
    const WrappedSpiralResult r = wrappedSpiralLayout(p);
    REQUIRE(r.ok());
    REQUIRE(r.contours.size() == 2);
    CHECK(line(r.contours[0]).start.y == 0.0);
    CHECK(line(r.contours[0]).end.y == Approx(25.0));
    CHECK(line(r.contours[0]).start.x == Approx(0.0));
    CHECK(line(r.contours[0]).end.x == Approx(40.0 * kPi));
}

TEST_CASE("spiral layout: the twist wraps to the asked handedness on the cylinder", "[wrapped][layout]") {
    // Handedness of the helix a strand makes on the cylinder as CylinderMap wraps it: advancing along the axis
    // while turning counter-clockwise seen from the end it advances towards is right-handed.
    for (const bool alongX : {true, false}) {
        for (const bool flip : {false, true}) {
            for (const SpiralTwist twist : {SpiralTwist::RightHand, SpiralTwist::LeftHand}) {
                WrappedSpiralParams p;
                p.alongX = alongX;
                p.flip = flip;
                p.diameter = 40.0;
                p.alongEnd = 20.0;
                p.value = 100.0;  // under a quarter turn: a single piece
                p.twist = twist;
                p.offset = 0.5 * 40.0 * kPi;  // half way round: clear of the seam
                const WrappedSpiralResult r = wrappedSpiralLayout(p);
                REQUIRE(r.ok());
                REQUIRE(r.contours.size() == 1);
                const LineSpan& s = line(r.contours[0]);
                CylinderMap map;
                map.alongX = alongX;
                map.flip = flip;
                map.radius = 20.0;
                map.axisZ = 0.0;
                const Vec3 a = map.wrap({s.start.x, s.start.y, 20.0});
                const Vec3 b = map.wrap({s.end.x, s.end.y, 20.0});
                // Advance along +axis; the turn about the axis (+X or +Y) is the axis component of a x b.
                const double advance = alongX ? b.x - a.x : b.y - a.y;
                const double turn = alongX ? a.y * b.z - a.z * b.y : a.z * b.x - a.x * b.z;
                const bool right = advance * turn > 0.0;
                CHECK(right == (twist == SpiralTwist::RightHand));
            }
        }
    }
}

TEST_CASE("spiral layout: limits and refusals", "[wrapped][layout]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto bad = [&](auto edit) {
        WrappedSpiralParams p;
        edit(p);
        const WrappedSpiralResult r = wrappedSpiralLayout(p);
        CHECK(!r.ok());
        CHECK(r.contours.empty());
    };
    bad([&](WrappedSpiralParams& p) { p.value = nan; });
    bad([](WrappedSpiralParams& p) { p.value = 0.0; });
    bad([](WrappedSpiralParams& p) { p.value = -5.0; });
    bad([](WrappedSpiralParams& p) { p.strands = 0; });
    bad([](WrappedSpiralParams& p) { p.strands = kMaxLayoutStrands + 1; });
    bad([](WrappedSpiralParams& p) { p.diameter = -1.0; });
    bad([](WrappedSpiralParams& p) { p.alongEnd = p.alongStart - 1.0; });
    bad([](WrappedSpiralParams& p) { p.value = 1e-4; });                     // pitch too small
    bad([](WrappedSpiralParams& p) { p.mode = SpiralPitchMode::Turns; p.value = 1e9; });
    bad([](WrappedSpiralParams& p) {  // 1000 strands x 101 pieces would be over the limit
        p.strands = 1000;
        p.value = 1.0;
    });
    WrappedSpiralParams edge;
    edge.value = 0.001;  // the smallest pitch: 100000 turns over 100 mm is refused by the vector limit
    CHECK(!wrappedSpiralLayout(edge).ok());
    edge.value = 25.0;
    edge.strands = kMaxLayoutStrands;
    edge.alongEnd = 1.0;
    const WrappedSpiralResult many = wrappedSpiralLayout(edge);
    REQUIRE(many.ok());
    CHECK(many.segments.size() == static_cast<std::size_t>(kMaxLayoutStrands));
    CHECK(many.contours.size() >= static_cast<std::size_t>(kMaxLayoutStrands));
}
