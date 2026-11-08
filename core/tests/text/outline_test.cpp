#include <cmath>
#include <variant>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "text_test_util.hpp"

using namespace carvetto;
using namespace carvetto::text;
using Catch::Approx;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Tolerance;
using geometry::Vec2;
using testutil::loadTestFont;
using testutil::signedArea;

// Glyph ids of the test font (core/tests/fixtures/text/README.md).
namespace {
constexpr std::uint32_t kNotdef = 0, kSpace = 1, kI = 2, kT = 3, kO = 4, kAlef = 5, kAcute = 6, kEacute = 7;

double netArea(const GlyphOutline& g) {
    double a = 0.0;
    for (const Contour& c : g.contours) {
        a += signedArea(c);
    }
    return a;
}
}  // namespace

TEST_CASE("Glyph outlines match the hand-computed areas of the test font", "[text][outline][oracle]") {
    const Font f = loadTestFont();
    // The expected values are derived by hand in core/tests/fixtures/text/README.md.
    CHECK(netArea(f.outline(kNotdef)) == Approx(180000.0).margin(1e-6));
    CHECK(netArea(f.outline(kI)) == Approx(140000.0).margin(1e-6));
    CHECK(netArea(f.outline(kT)) == Approx(120000.0).margin(1e-6));
    CHECK(netArea(f.outline(kO)) == Approx(275000.0).margin(1e-6));
    CHECK(netArea(f.outline(kAlef)) == Approx(100000.0).margin(1e-6));
    CHECK(netArea(f.outline(kAcute)) == Approx(20000.0).margin(1e-6));
    CHECK(netArea(f.outline(kEacute)) == Approx(160000.0).margin(1e-6));  // composite: I + shifted acute
}

TEST_CASE("Glyph outlines: winding, closure, bounds", "[text][outline][oracle]") {
    const Font f = loadTestFont();
    const Tolerance tol;
    for (std::uint32_t id = 0; id < 8; ++id) {
        const GlyphOutline g = f.outline(id);
        REQUIRE(g.status == OutlineStatus::Ok);
        for (const Contour& c : g.contours) {
            CHECK(c.closed);
            CHECK(c.isContinuous(tol));
            CHECK_FALSE(c.spans.empty());
        }
    }
    // Space is empty.
    CHECK(f.outline(kSpace).contours.empty());

    // Holes wind clockwise, outer contours counter-clockwise, although TrueType stores the reverse.
    const GlyphOutline o = f.outline(kO);
    REQUIRE(o.contours.size() == 2);
    CHECK(signedArea(o.contours[0]) == Approx(408333.3333333333).epsilon(1e-12));
    CHECK(signedArea(o.contours[1]) == Approx(-133333.3333333333).epsilon(1e-12));
    const GlyphOutline nd = f.outline(kNotdef);
    REQUIRE(nd.contours.size() == 2);
    CHECK(signedArea(nd.contours[0]) == Approx(280000.0));
    CHECK(signedArea(nd.contours[1]) == Approx(-100000.0));

    // Exact bounds in font units.
    const auto bounds = [&](std::uint32_t id) {
        geometry::Box2 b;
        for (const Contour& c : f.outline(id).contours) {
            b.expand(c.bounds());
        }
        return b;
    };
    CHECK(bounds(kI).min == Vec2{100, 0});
    CHECK(bounds(kI).max == Vec2{300, 700});
    CHECK(bounds(kT).min == Vec2{0, 0});
    CHECK(bounds(kT).max == Vec2{600, 700});
    CHECK(bounds(kO).min.x == Approx(0.0).margin(1e-9));
    CHECK(bounds(kO).max.x == Approx(700.0).epsilon(1e-12));
    CHECK(bounds(kAlef).min == Vec2{100, 0});
    CHECK(bounds(kAlef).max == Vec2{500, 700});
    CHECK(bounds(kAcute).min == Vec2{-300, 750});
    CHECK(bounds(kEacute).max == Vec2{300, 850});

    // The alef is an L: its stem is on the left. A mirrored outline would put the foot on the left.
    const GlyphOutline alef = f.outline(kAlef);
    REQUIRE(alef.contours.size() == 1);
    bool hasFoot = false;
    for (const geometry::Span& s : alef.contours[0].spans) {
        if (const auto* l = std::get_if<LineSpan>(&s)) {
            hasFoot = hasFoot || (l->start == Vec2{500, 100} && l->end == Vec2{500, 0}) ||
                      (l->start == Vec2{500, 0} && l->end == Vec2{500, 100});
        }
    }
    CHECK(hasFoot);
}

TEST_CASE("Quadratic curves are elevated to cubics exactly", "[text][outline][oracle]") {
    const Font f = loadTestFont();
    const GlyphOutline o = f.outline(kO);
    REQUIRE(o.contours.size() == 2);
    // Source quadratics (P0, Q, P2) of the font's 'O' (core/tests/fixtures/text/README.md).
    struct Quad {
        Vec2 p0, q, p2;
    };
    const Quad quads[8] = {
        {{0, 350}, {0, 700}, {350, 700}},     {{350, 700}, {700, 700}, {700, 350}},
        {{700, 350}, {700, 0}, {350, 0}},     {{350, 0}, {0, 0}, {0, 350}},
        {{150, 350}, {150, 150}, {350, 150}}, {{350, 150}, {550, 150}, {550, 350}},
        {{550, 350}, {550, 550}, {350, 550}}, {{350, 550}, {150, 550}, {150, 350}},
    };
    const auto quadAt = [](const Quad& q, double t) {
        const double u = 1.0 - t;
        return Vec2{u * u * q.p0.x + 2 * u * t * q.q.x + t * t * q.p2.x,
                    u * u * q.p0.y + 2 * u * t * q.q.y + t * t * q.p2.y};
    };
    int matched = 0;
    for (const Contour& c : o.contours) {
        CHECK(c.spans.size() == 4);
        for (const geometry::Span& s : c.spans) {
            const auto* b = std::get_if<BezierSpan>(&s);
            REQUIRE(b != nullptr);
            bool found = false;
            for (const Quad& q : quads) {
                const bool forward = b->p0 == q.p0 && b->p3 == q.p2;
                const bool backward = b->p0 == q.p2 && b->p3 == q.p0;
                if (!forward && !backward) {
                    continue;
                }
                found = true;
                for (const double t : {0.1, 0.3, 0.5, 0.77, 0.9}) {
                    const Vec2 want = quadAt(q, forward ? t : 1.0 - t);
                    const Vec2 got = b->pointAt(t);
                    CHECK(got.x == Approx(want.x).margin(1e-9));
                    CHECK(got.y == Approx(want.y).margin(1e-9));
                }
            }
            CHECK(found);
            matched += found ? 1 : 0;
        }
    }
    CHECK(matched == 8);
}

TEST_CASE("Outline requests are bounded", "[text][outline][hostile]") {
    const Font f = loadTestFont();
    // 'I' has 4 points.
    CHECK(f.outline(kI, 4).status == OutlineStatus::Ok);
    CHECK(f.outline(kI, 3).status == OutlineStatus::TooComplex);
    CHECK(f.outline(kI, 3).contours.empty());
    CHECK(f.outline(kI, 3).pointCount == 4);
    // Glyph ids beyond the font.
    CHECK(f.outline(8).status == OutlineStatus::LoadFailed);
    CHECK(f.outline(0xFFFFFFFFu).status == OutlineStatus::LoadFailed);
}
