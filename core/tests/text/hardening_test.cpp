#include <chrono>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <string>
#include <variant>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "text_test_util.hpp"

// Verifier round 1 of T-0130: CPU-bound hostile input, environment independence, the OpenType/CFF
// path, and boundary conditions of the path layout and the limits.

using namespace carvetto;
using namespace carvetto::text;
using Catch::Approx;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;
using testutil::fontBytes;
using testutil::hashLayout;
using testutil::loadCffFont;
using testutil::loadTestFont;
using testutil::signedArea;

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

Vec2 anchorOf(const PlacedGlyph& g) {
    return g.origin + g.baselineDir * (g.advance / 2.0);
}

// Runs `fn`, requires that it finishes within `seconds` (a generous bound for a Debug build; the
// quadratic implementations this guards against took 8 to 36 s on these inputs).
template <class Fn>
void requireFast(double seconds, Fn&& fn) {
    const auto start = std::chrono::steady_clock::now();
    fn();
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
    CHECK(elapsed.count() < seconds);
}

void setEnv(const char* name, const char* value) {
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}
void clearEnv(const char* name) {
#ifdef _WIN32
    _putenv_s(name, "");
#else
    unsetenv(name);
#endif
}

}  // namespace

TEST_CASE("Line-break floods are linear time on both entry points", "[text][hostile][limits]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    const Contour path = lineAlongX(100);
    std::string crlf;
    for (int i = 0; i < 524288; ++i) {
        crlf += "\r\n";  // 1 MiB: the default byte limit
    }
    std::string crlfBig;
    for (int i = 0; i < 1100000; ++i) {
        crlfBig += "\r\n";  // above the default limit
    }
    std::string mixed;
    for (int i = 0; i < 300000; ++i) {
        mixed += "\r\r\n\n\n\r";
    }

    TextLimits wide;  // a caller raising the byte limit gets other limits, not a CPU sink
    wide.maxTextBytes = 64u << 20;
    for (const std::string* text : {&crlf, &crlfBig, &mixed}) {
        requireFast(3.0, [&] {
            const TextLayout l = layoutText(f, *text, s);
            CHECK_FALSE(l.ok());
            CHECK(l.glyphs.empty());
        });
        requireFast(3.0, [&] {
            const TextLayout l = layoutTextOnPath(f, *text, s, path);
            CHECK_FALSE(l.ok());
            CHECK(l.glyphs.empty());
        });
        requireFast(3.0, [&] {
            CHECK_FALSE(layoutText(f, *text, s, wide).ok());
            CHECK_FALSE(layoutTextOnPath(f, *text, s, path, {}, wide).ok());
        });
    }
    // The refusal is the size check, before any copying (path layout), not an accident.
    CHECK(layoutTextOnPath(f, crlfBig, s, path).status == TextStatus::TextTooLong);
    CHECK(layoutText(f, crlfBig, s).status == TextStatus::TextTooLong);
    // Small inputs: CRLF, CR and LF each become exactly one space on a path.
    const TextLayout small = layoutTextOnPath(f, "I\r\nI\rI\nI", s, path);
    REQUIRE(small.ok());
    CHECK(small.glyphs.size() == 7);
}

TEST_CASE("Layout does not depend on HarfBuzz or FreeType environment variables", "[text][determinism]") {
    const std::string text = "TOI IT\xC3\x89\n\xD7\x90 Z\xCC\x81 fi";
    TextStyle s;
    s.sizeMm = 12.5;
    Contour path;
    path.spans.emplace_back(LineSpan{{0, 0}, {300, 0}});
    const auto run = [&] {
        const Font f = loadTestFont();  // FreeType reads FREETYPE_PROPERTIES when it initialises
        const TextLayout a = layoutText(f, text, s);
        const TextLayout b = layoutTextOnPath(f, text, s, path);
        REQUIRE(a.ok());
        REQUIRE(b.ok());
        return std::pair{hashLayout(a), hashLayout(b)};
    };
    const auto baseline = run();
    const char* const vars[][2] = {
        {"HB_SHAPER_LIST", "fallback"},
        {"HB_SHAPER_LIST", "nonexistent"},
        {"HB_FACE_LOADER", "nonexistent"},
        {"HB_FONT_FUNCS", "nonexistent"},
        {"FREETYPE_PROPERTIES", "truetype:interpreter-version=35 cff:no-stem-darkening=0 cff:darkening-parameters=500,500,1000,500,2000,500,4000,0"},
        {"FREETYPE_PROPERTIES", "autofitter:warping=1"},
        {"FT2_DEBUG", "any:7"},
    };
    for (const auto& v : vars) {
        setEnv(v[0], v[1]);
        CHECK(run() == baseline);
        clearEnv(v[0]);
    }
    CHECK(run() == baseline);
}

TEST_CASE("OpenType/CFF fonts: PostScript winding and real cubics", "[text][outline][oracle][cff]") {
    const Font f = loadCffFont();
    REQUIRE(f.valid());
    CHECK(f.metrics().unitsPerEm == 1000);
    CHECK(f.glyphIndex(U'A') == 1);
    CHECK(f.glyphIndex(U'B') == 2);
    CHECK(f.glyphIndex(U'C') == 3);

    // A: box 600x700, hole 400x500. CFF outers are counter-clockwise already: no flip.
    const GlyphOutline a = f.outline(1);
    REQUIRE(a.status == OutlineStatus::Ok);
    REQUIRE(a.contours.size() == 2);
    CHECK(signedArea(a.contours[0]) == Approx(420000.0));
    CHECK(signedArea(a.contours[1]) == Approx(-200000.0));

    // B: line, a cubic that must pass through unchanged, line. Area 250000 + bulge 81000 (by hand).
    const GlyphOutline b = f.outline(2);
    REQUIRE(b.contours.size() == 1);
    CHECK(signedArea(b.contours[0]) == Approx(331000.0).epsilon(1e-12));
    int cubics = 0;
    for (const geometry::Span& sp : b.contours[0].spans) {
        if (const auto* bz = std::get_if<BezierSpan>(&sp)) {
            ++cubics;
            CHECK(bz->p0 == Vec2{500, 0});
            CHECK(bz->p1 == Vec2{800, 100});
            CHECK(bz->p2 == Vec2{800, 400});
            CHECK(bz->p3 == Vec2{500, 500});
        }
    }
    CHECK(cubics == 1);

    // C: a Bezier circle r = 500 with a clockwise hole r = 200; pi (r1^2 - r2^2) within the integer-rounded
    // the four-cubic approximation.
    const GlyphOutline c = f.outline(3);
    REQUIRE(c.contours.size() == 2);
    const double pi = std::numbers::pi;
    CHECK(signedArea(c.contours[0]) == Approx(pi * 500 * 500).epsilon(2e-3));
    CHECK(signedArea(c.contours[1]) == Approx(-pi * 200 * 200).epsilon(2e-3));

    // And through layout, scaled: A at 10 mm is 22 mm^2, positive.
    const TextLayout l = layoutText(f, "AB", style10());
    REQUIRE(l.ok());
    CHECK(l.glyphs[1].origin.x == Approx(7.0));
    CHECK(testutil::netArea(l.contours, l.glyphs[0].firstContour, l.glyphs[0].contourCount) == Approx(22.0));
    // Kerning-free font: same answer with and without.
    TextStyle s = style10();
    s.kerning = false;
    CHECK(hashLayout(layoutText(f, "AB", s)) == hashLayout(l));
}

TEST_CASE("Closed paths wrap exactly, including negative offsets", "[text][path]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    const Contour square = Contour::rectangle(geometry::Box2{{0, 0}, {10, 10}});  // counter-clockwise, length 40
    SECTION("past the end") {
        const TextLayout l = layoutTextOnPath(f, "IIIIIIIIIIII", s, square);  // midpoints 2, 6, ..., 46
        REQUIRE(l.glyphs.size() == 12);
        CHECK(anchorOf(l.glyphs[10]).x == Approx(2.0).margin(1e-9));  // 42 -> 2
        CHECK(anchorOf(l.glyphs[10]).y == Approx(0.0).margin(1e-9));
        CHECK(anchorOf(l.glyphs[11]).x == Approx(6.0).margin(1e-9));  // 46 -> 6
        CHECK(anchorOf(l.glyphs[11]).y == Approx(0.0).margin(1e-9));
    }
    SECTION("negative start offset") {
        PathTextOptions o;
        o.startOffsetMm = -6.0;  // midpoint -4 -> 36: left edge, 4 mm above the origin
        const TextLayout l = layoutTextOnPath(f, "I", s, square, o);
        REQUIRE(l.glyphs.size() == 1);
        CHECK_FALSE(l.truncated);
        CHECK(anchorOf(l.glyphs[0]).x == Approx(0.0).margin(1e-9));
        CHECK(anchorOf(l.glyphs[0]).y == Approx(4.0).margin(1e-9));
        o.startOffsetMm = -46.0;  // more than a lap back
        const TextLayout m = layoutTextOnPath(f, "I", s, square, o);
        CHECK(anchorOf(m.glyphs[0]).y == Approx(4.0).margin(1e-9));
    }
}

TEST_CASE("Open path ends are inclusive", "[text][path]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    // 'I' advances 4 mm, so its midpoint is at start + 2.
    CHECK(layoutTextOnPath(f, "I", s, lineAlongX(2.0)).glyphs.size() == 1);   // midpoint == length: kept
    CHECK_FALSE(layoutTextOnPath(f, "I", s, lineAlongX(2.0)).truncated);
    const TextLayout past = layoutTextOnPath(f, "I", s, lineAlongX(1.999));   // just past the end
    CHECK(past.glyphs.empty());
    CHECK(past.truncated);
    PathTextOptions o;
    o.startOffsetMm = -2.0;  // midpoint == 0: kept
    CHECK(layoutTextOnPath(f, "I", s, lineAlongX(50), o).glyphs.size() == 1);
    o.startOffsetMm = -2.001;
    CHECK(layoutTextOnPath(f, "I", s, lineAlongX(50), o).glyphs.empty());
}

TEST_CASE("Alignment width counts letter spacing between glyphs only", "[text][layout][path]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    s.letterSpacingMm = 1.0;  // "IT" = 4 + 7 + one gap of 1 = 12
    s.alignment = Alignment::Centre;
    CHECK(layoutText(f, "IT", s).glyphs[0].origin.x == Approx(-6.0));
    CHECK(layoutTextOnPath(f, "IT", s, lineAlongX(100)).lines[0].widthMm == Approx(12.0));
    PathTextOptions o;
    o.startOffsetMm = 50.0;
    CHECK(layoutTextOnPath(f, "IT", s, lineAlongX(100), o).glyphs[0].origin.x == Approx(44.0));
    s.alignment = Alignment::Right;
    CHECK(layoutText(f, "IT", s).glyphs[0].origin.x == Approx(-12.0));
    CHECK(layoutTextOnPath(f, "IT", s, lineAlongX(100), o).glyphs[0].origin.x == Approx(38.0));
    // A single glyph has no gap.
    s.alignment = Alignment::Right;
    CHECK(layoutText(f, "I", s).glyphs[0].origin.x == Approx(-4.0));
}

TEST_CASE("Font limits: ceilings and em size range", "[text][font][hostile]") {
    SECTION("the byte ceiling cannot be lifted by the caller") {
        // Only the length is inspected before the refusal; the bytes are never read.
        const std::vector<std::uint8_t> one(16, 0);
        FontLimits lim;
        lim.maxFileBytes = static_cast<std::size_t>(-1);
        const FontLoad r = Font::load(std::span<const std::uint8_t>(one.data(), kAbsoluteMaxFontBytes + 1), lim);
        CHECK(r.status == FontStatus::TooLarge);
    }
    SECTION("units per em outside 16..16384 is refused") {
        std::vector<std::uint8_t> bytes = fontBytes();
        std::size_t head = 0;
        const std::size_t tables = (static_cast<std::size_t>(bytes[4]) << 8) | bytes[5];
        for (std::size_t i = 0; i < tables; ++i) {
            const std::size_t rec = 12 + 16 * i;
            if (bytes[rec] == 'h' && bytes[rec + 1] == 'e' && bytes[rec + 2] == 'a' && bytes[rec + 3] == 'd') {
                head = (static_cast<std::size_t>(bytes[rec + 8]) << 24) | (static_cast<std::size_t>(bytes[rec + 9]) << 16) |
                       (static_cast<std::size_t>(bytes[rec + 10]) << 8) | bytes[rec + 11];
            }
        }
        REQUIRE(head != 0);
        const auto setUpem = [&](unsigned v) {
            bytes[head + 18] = static_cast<std::uint8_t>(v >> 8);
            bytes[head + 19] = static_cast<std::uint8_t>(v);
        };
        for (const unsigned bad : {0u, 8u, 15u, 16385u, 20000u, 65535u}) {
            setUpem(bad);
            CHECK_FALSE(Font::load(bytes).ok());
        }
        for (const unsigned good : {16u, 1000u, 2048u, 16384u}) {
            setUpem(good);
            CHECK(Font::load(bytes).ok());
        }
    }
}
