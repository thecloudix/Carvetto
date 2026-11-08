#include <cmath>
#include <limits>
#include <string>
#include <variant>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "text_test_util.hpp"

using namespace carvetto;
using namespace carvetto::text;
using Catch::Approx;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;
using testutil::hashLayout;
using testutil::loadTestFont;

namespace {

TextStyle style10() {
    TextStyle s;
    s.sizeMm = 10.0;
    return s;
}

Contour lineAlongX(double length) {
    Contour c;
    c.spans.emplace_back(LineSpan{{0, 0}, {length, 0}});
    return c;
}

// Closed counter-clockwise circle made of cubic Beziers (<= 90 degrees each), centred at the origin.
Contour bezierCircle(double radius, bool reverse = false) {
    Contour c;
    for (const BezierSpan& b : ArcSpan::fullCircle({0, 0}, radius).toBeziers()) {
        c.spans.emplace_back(b);
    }
    c.closed = true;
    if (reverse) {
        c.reverse();
    }
    return c;
}

double angleBetween(Vec2 a, Vec2 b) {
    return std::atan2(a.cross(b), a.dot(b));
}

}  // namespace

TEST_CASE("Text on a straight path equals the flat layout", "[text][path]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    const TextLayout flat = layoutText(f, "TO IT", s);
    const TextLayout on = layoutTextOnPath(f, "TO IT", s, lineAlongX(500));
    REQUIRE(on.ok());
    REQUIRE(flat.ok());
    REQUIRE(on.glyphs.size() == flat.glyphs.size());
    REQUIRE(on.contours.size() == flat.contours.size());
    CHECK_FALSE(on.truncated);
    for (std::size_t i = 0; i < on.glyphs.size(); ++i) {
        CHECK(on.glyphs[i].origin.x == Approx(flat.glyphs[i].origin.x).margin(1e-12));
        CHECK(on.glyphs[i].origin.y == Approx(0.0).margin(1e-12));
        CHECK(on.glyphs[i].baselineDir == Vec2{1.0, 0.0});
    }
    for (std::size_t i = 0; i < on.contours.size(); ++i) {
        const geometry::Box2 a = on.contours[i].bounds();
        const geometry::Box2 b = flat.contours[i].bounds();
        CHECK(a.min.x == Approx(b.min.x).margin(1e-9));
        CHECK(a.max.y == Approx(b.max.y).margin(1e-9));
    }
}

TEST_CASE("Glyph baselines are tangent to a circular path", "[text][path][oracle]") {
    const Font f = loadTestFont();
    const double radius = 60.0;
    TextStyle s = style10();
    s.kerning = false;  // a kerned glyph is deliberately drawn off its advance midpoint (next test)
    const TextLayout l = layoutTextOnPath(f, "ITOTITOT", s, bezierCircle(radius));
    REQUIRE(l.ok());
    REQUIRE(l.glyphs.size() == 8);
    CHECK_FALSE(l.truncated);

    Vec2 previous;
    double previousHalf = 0.0;
    for (std::size_t i = 0; i < l.glyphs.size(); ++i) {
        const PlacedGlyph& g = l.glyphs[i];
        CHECK(g.baselineDir.length() == Approx(1.0).margin(1e-12));
        // The advance midpoint of the baseline lies on the path (a 4-Bezier circle is within
        // 2.7e-4 of the true one) ...
        const Vec2 anchor = g.origin + g.baselineDir * (g.advance / 2.0);
        CHECK(anchor.length() == Approx(radius).margin(0.02));
        // ... and the baseline there is tangent: perpendicular to the radius (counter-clockwise).
        const Vec2 trueTangent = anchor.perp().normalized();
        CHECK(std::fabs(angleBetween(trueTangent, g.baselineDir)) < 3e-3);  // the 4-Bezier circle's own tangent error is ~1.3e-3
        // Arc-length placement: consecutive midpoints are half-advance + half-advance apart.
        if (i > 0) {
            const double angle = std::fabs(angleBetween(previous, anchor));
            const double expectedArc = previousHalf + g.advance / 2.0;
            CHECK(angle * radius == Approx(expectedArc).epsilon(2e-3));
        }
        previous = anchor;
        previousHalf = g.advance / 2.0;
    }
}

TEST_CASE("Kerning shifts a glyph along the tangent, not off the path", "[text][path]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    const Contour line = lineAlongX(200);
    const TextLayout kerned = layoutTextOnPath(f, "TO", s, line);
    s.kerning = false;
    const TextLayout plain = layoutTextOnPath(f, "TO", s, line);
    REQUIRE(kerned.glyphs.size() == 2);
    // Same word, 1.2 mm tighter: the O sits 1.2 mm closer to the T and still on the baseline.
    CHECK(kerned.glyphs[1].origin.x == Approx(plain.glyphs[1].origin.x - 1.2).margin(1e-9));
    CHECK(kerned.glyphs[1].origin.y == Approx(0.0).margin(1e-12));
}

TEST_CASE("Glyphs are rigid and stand on the left side of the path", "[text][path][oracle]") {
    const Font f = loadTestFont();
    const double radius = 60.0;
    const TextStyle s = style10();
    for (const bool reversed : {false, true}) {
        const TextLayout l = layoutTextOnPath(f, "I", s, bezierCircle(radius, reversed));
        REQUIRE(l.ok());
        REQUIRE(l.contours.size() == 1);
        // The I bar: 4 corners, local (1,0), (1,7), (3,7), (3,0) mm.
        const Contour& c = l.contours[0];
        std::vector<Vec2> p;
        for (const geometry::Span& sp : c.spans) {
            p.push_back(std::get<LineSpan>(sp).start);
        }
        REQUIRE(p.size() == 4);
        double minR = 1e9;
        double maxR = 0.0;
        for (const Vec2 v : p) {
            minR = std::min(minR, v.length());
            maxR = std::max(maxR, v.length());
        }
        const Vec2 anchor = l.glyphs[0].origin + l.glyphs[0].baselineDir * (l.glyphs[0].advance / 2.0);
        const Vec2 up = l.glyphs[0].baselineDir.perp();
        // Every corner is within the 7 mm tall, 2 mm wide bar rotated onto the tangent frame.
        for (const Vec2 v : p) {
            const Vec2 d = v - anchor;
            const double across = d.dot(l.glyphs[0].baselineDir);
            const double height = d.dot(up);
            CHECK(std::fabs(std::fabs(across) - 1.0) < 1e-9);  // x from -1 to +1 around the midpoint
            CHECK((std::fabs(height) < 1e-9 || std::fabs(height - 7.0) < 1e-9));
        }
        // Counter-clockwise path: left is inside, so the glyph tops point at the centre; the
        // reversed (clockwise) path puts the text outside.
        const double topRadius = (reversed ? maxR : minR);
        CHECK(topRadius == Approx(reversed ? radius + 7.0 : radius - 7.0).margin(0.03));
    }
}

TEST_CASE("Alignment and start offset position the text along the path", "[text][path]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    PathTextOptions o;
    o.startOffsetMm = 100.0;
    // "IT" is 11 mm wide.
    const TextLayout left = layoutTextOnPath(f, "IT", s, lineAlongX(300), o);
    CHECK(left.glyphs[0].origin.x == Approx(100.0));
    s.alignment = Alignment::Centre;
    const TextLayout centre = layoutTextOnPath(f, "IT", s, lineAlongX(300), o);
    CHECK(centre.glyphs[0].origin.x == Approx(94.5));
    s.alignment = Alignment::Right;
    const TextLayout right = layoutTextOnPath(f, "IT", s, lineAlongX(300), o);
    CHECK(right.glyphs[0].origin.x == Approx(89.0));
    CHECK(right.lines[0].widthMm == Approx(11.0));
    CHECK(right.lines[0].baselineStart.x == Approx(89.0));
}

TEST_CASE("Text on a Bezier path of mixed curvature follows arc length", "[text][path][oracle]") {
    const Font f = loadTestFont();
    // S-shaped cubic; reference arc length by dense chord sum, independent of the library.
    const BezierSpan bz{{0, 0}, {60, 80}, {120, -80}, {180, 0}};
    Contour path;
    path.spans.emplace_back(bz);
    std::vector<double> cum{0.0};
    Vec2 prev = bz.p0;
    for (int i = 1; i <= 20000; ++i) {
        const double t = i / 20000.0;
        const double u = 1 - t;
        const Vec2 p{u * u * u * bz.p0.x + 3 * u * u * t * bz.p1.x + 3 * u * t * t * bz.p2.x + t * t * t * bz.p3.x,
                     u * u * u * bz.p0.y + 3 * u * u * t * bz.p1.y + 3 * u * t * t * bz.p2.y + t * t * t * bz.p3.y};
        cum.push_back(cum.back() + (p - prev).length());
        prev = p;
    }
    const TextLayout l = layoutTextOnPath(f, "IIIIIIII", style10(), path);
    REQUIRE(l.ok());
    REQUIRE(l.glyphs.size() == 8);
    for (std::size_t i = 0; i < 8; ++i) {
        const PlacedGlyph& g = l.glyphs[i];
        const Vec2 anchor = g.origin + g.baselineDir * (g.advance / 2.0);
        const double wantArc = 4.0 * static_cast<double>(i) + 2.0;  // each I advances 4 mm
        // Find the reference point at that arc length.
        std::size_t k = 0;
        while (k + 1 < cum.size() && cum[k + 1] < wantArc) {
            ++k;
        }
        const double cellFrac = (wantArc - cum[k]) / (cum[k + 1] - cum[k]);
        const double t = (static_cast<double>(k) + cellFrac) / 20000.0;
        const double u = 1 - t;
        const Vec2 want{u * u * u * bz.p0.x + 3 * u * u * t * bz.p1.x + 3 * u * t * t * bz.p2.x + t * t * t * bz.p3.x,
                        u * u * u * bz.p0.y + 3 * u * u * t * bz.p1.y + 3 * u * t * t * bz.p2.y + t * t * t * bz.p3.y};
        CHECK((anchor - want).length() < 1e-4);
        // Tangent: finite difference of the reference curve.
        const double h = 1e-4;
        const auto at = [&](double tt) {
            const double uu = 1 - tt;
            return Vec2{uu * uu * uu * bz.p0.x + 3 * uu * uu * tt * bz.p1.x + 3 * uu * tt * tt * bz.p2.x + tt * tt * tt * bz.p3.x,
                        uu * uu * uu * bz.p0.y + 3 * uu * uu * tt * bz.p1.y + 3 * uu * tt * tt * bz.p2.y + tt * tt * tt * bz.p3.y};
        };
        const Vec2 tangent = (at(t + h) - at(t - h)).normalized();
        CHECK(std::fabs(angleBetween(tangent, g.baselineDir)) < 2e-3);
    }
}

TEST_CASE("Open paths drop glyphs past the ends, closed paths wrap", "[text][path]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    SECTION("open path too short") {
        const TextLayout l = layoutTextOnPath(f, "IIIIIIII", s, lineAlongX(20));
        REQUIRE(l.ok());
        CHECK(l.truncated);
        // Midpoints at 2, 6, 10, 14, 18 mm fit; 22 does not.
        CHECK(l.glyphs.size() == 5);
        CHECK(l.lines[0].glyphCount == 5);
    }
    SECTION("negative start drops leading glyphs") {
        PathTextOptions o;
        o.startOffsetMm = -5.0;
        const TextLayout l = layoutTextOnPath(f, "IIII", s, lineAlongX(100), o);
        CHECK(l.truncated);
        CHECK(l.glyphs.size() == 3);  // midpoints -3 (dropped), 1, 5, 9
    }
    SECTION("closed square wraps") {
        Contour sq = Contour::rectangle(geometry::Box2{{0, 0}, {10, 10}});  // length 40
        const TextLayout l = layoutTextOnPath(f, "IIIIIIIIIIII", s, sq);
        REQUIRE(l.ok());
        CHECK_FALSE(l.truncated);
        CHECK(l.glyphs.size() == 12);
        for (const PlacedGlyph& g : l.glyphs) {
            const Vec2 a = g.origin + g.baselineDir * (g.advance / 2.0);
            const bool onEdge = std::fabs(a.x) < 1e-9 || std::fabs(a.x - 10) < 1e-9 || std::fabs(a.y) < 1e-9 ||
                                std::fabs(a.y - 10) < 1e-9;
            CHECK(onEdge);
        }
    }
}

TEST_CASE("Text on a path: line breaks become spaces, missing and empty text", "[text][path]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    const TextLayout l = layoutTextOnPath(f, "I\r\nI\nI", s, lineAlongX(200));
    REQUIRE(l.ok());
    CHECK(l.glyphs.size() == 5);  // I, space, I, space, I
    CHECK(l.glyphs[1].glyphId == 1);
    CHECK(layoutTextOnPath(f, "", s, lineAlongX(200)).glyphs.empty());
    CHECK(layoutTextOnPath(f, "", s, lineAlongX(200)).ok());
    CHECK(layoutTextOnPath(f, "Z", s, lineAlongX(200)).glyphs[0].missing);
}

TEST_CASE("Hostile guide paths are refused", "[text][path][hostile]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(layoutTextOnPath(f, "I", s, Contour{}).status == TextStatus::InvalidPath);
    Contour zero;
    zero.spans.emplace_back(LineSpan{{5, 5}, {5, 5}});
    CHECK(layoutTextOnPath(f, "I", s, zero).status == TextStatus::InvalidPath);
    Contour bad;
    bad.spans.emplace_back(LineSpan{{0, 0}, {nan, 1}});
    CHECK(layoutTextOnPath(f, "I", s, bad).status == TextStatus::InvalidPath);
    Contour infinite;
    infinite.spans.emplace_back(LineSpan{{0, 0}, {inf, 0}});
    CHECK(layoutTextOnPath(f, "I", s, infinite).status == TextStatus::InvalidPath);
    Contour badBezier;
    badBezier.spans.emplace_back(BezierSpan{{0, 0}, {nan, 0}, {1, 1}, {2, 2}});
    CHECK(layoutTextOnPath(f, "I", s, badBezier).status == TextStatus::InvalidPath);
    Contour badArc;
    badArc.spans.emplace_back(ArcSpan{{0, 0}, inf, 0.0, 1.0, geometry::ArcDirection::CCW});
    CHECK(layoutTextOnPath(f, "I", s, badArc).status == TextStatus::InvalidPath);

    PathTextOptions o;
    o.startOffsetMm = nan;
    CHECK(layoutTextOnPath(f, "I", s, lineAlongX(10), o).status == TextStatus::InvalidStyle);

    TextLimits lim;
    lim.maxPathSpans = 2;
    Contour three;
    for (int i = 0; i < 3; ++i) {
        three.spans.emplace_back(LineSpan{{double(i), 0}, {double(i + 1), 0}});
    }
    const TextLayout r = layoutTextOnPath(f, "I", s, three, {}, lim);
    CHECK(r.status == TextStatus::PathTooComplex);
    CHECK(r.glyphs.empty());
    lim.maxPathSpans = 3;
    CHECK(layoutTextOnPath(f, "I", s, three, {}, lim).ok());
}

TEST_CASE("Text on a path is deterministic", "[text][path][determinism]") {
    const Font f = loadTestFont();
    TextStyle s;
    s.sizeMm = 9.0;
    s.letterSpacingMm = 0.25;
    // Beziers and lines only (arcs use libm trig, which is not bit-identical across platforms).
    Contour path;
    path.spans.emplace_back(LineSpan{{0, 0}, {30, 0}});
    path.spans.emplace_back(BezierSpan{{30, 0}, {60, 0}, {90, 40}, {90, 80}});
    path.spans.emplace_back(LineSpan{{90, 80}, {10, 100}});
    const TextLayout a = layoutTextOnPath(f, "TOI IT TOO", s, path);
    const TextLayout b = layoutTextOnPath(loadTestFont(), "TOI IT TOO", s, path);
    REQUIRE(a.ok());
    CHECK(hashLayout(a) == hashLayout(b));
    CHECK(hashLayout(a) == 0xaeaf74a71756a6acull);  // golden, see layout_test.cpp
}
