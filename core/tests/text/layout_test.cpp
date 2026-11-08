#include <cmath>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "text_test_util.hpp"

using namespace carvetto;
using namespace carvetto::text;
using Catch::Approx;
using geometry::Vec2;
using testutil::hashLayout;
using testutil::loadTestFont;
using testutil::netArea;

namespace {
// Size 10 mm on a 1000-unit em: 0.01 mm per font unit.
TextStyle style10() {
    TextStyle s;
    s.sizeMm = 10.0;
    return s;
}
}  // namespace

TEST_CASE("A single glyph is placed at the origin with oracle area and bounds", "[text][layout][oracle]") {
    const Font f = loadTestFont();
    const TextLayout l = layoutText(f, "I", style10());
    REQUIRE(l.ok());
    REQUIRE(l.glyphs.size() == 1);
    REQUIRE(l.lines.size() == 1);
    CHECK(l.glyphs[0].glyphId == 2);
    CHECK_FALSE(l.glyphs[0].missing);
    CHECK(l.glyphs[0].advance == Approx(4.0));
    CHECK(l.glyphs[0].origin == Vec2{0, 0});
    CHECK(l.lines[0].widthMm == Approx(4.0));
    // 140000 font units^2 * (0.01 mm/unit)^2 = 14 mm^2; bar from x = 1 to 3, y = 0 to 7.
    CHECK(netArea(l.contours, 0, l.contours.size()) == Approx(14.0).epsilon(1e-12));
    CHECK(l.bounds.min.x == Approx(1.0));
    CHECK(l.bounds.max.x == Approx(3.0));
    CHECK(l.bounds.min.y == Approx(0.0).margin(1e-12));
    CHECK(l.bounds.max.y == Approx(7.0));
    CHECK(l.work == 1 + 4 + 4 + 4);  // 1 code point, 4 per glyph, 4 outline points, 4 spans
}

TEST_CASE("Size scales the outlines linearly", "[text][layout]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    const TextLayout a = layoutText(f, "TO", s);
    s.sizeMm = 25.0;
    const TextLayout b = layoutText(f, "TO", s);
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    CHECK(b.bounds.max.x == Approx(a.bounds.max.x * 2.5).epsilon(1e-12));
    CHECK(b.bounds.max.y == Approx(a.bounds.max.y * 2.5).epsilon(1e-12));
    CHECK(netArea(b.contours, 0, b.contours.size()) == Approx(netArea(a.contours, 0, a.contours.size()) * 6.25).epsilon(1e-12));
}

TEST_CASE("Advances, kerning and letter spacing", "[text][layout]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    SECTION("kerning pulls O under the bar of T") {
        const TextLayout k = layoutText(f, "TO", s);
        REQUIRE(k.glyphs.size() == 2);
        CHECK(k.glyphs[1].origin.x == Approx(5.8));  // (700 - 120) * 0.01
        // HarfBuzz splits a 'kern' pair between the first glyph's advance and the second's advance
        // and offset: the pen moves 6.4 + 7.4 but O is drawn 0.6 left of its pen position.
        CHECK(k.glyphs[0].advance == Approx(6.4));
        CHECK(k.glyphs[1].advance == Approx(7.4));
        CHECK(k.lines[0].widthMm == Approx(5.8 + 8.0));
        s.kerning = false;
        const TextLayout n = layoutText(f, "TO", s);
        CHECK(n.glyphs[1].origin.x == Approx(7.0));
        CHECK(n.lines[0].widthMm == Approx(15.0));
    }
    SECTION("a pair without kerning is unaffected") {
        const TextLayout l = layoutText(f, "OT", s);
        CHECK(l.glyphs[1].origin.x == Approx(8.0));
    }
    SECTION("letter spacing is added after each glyph") {
        s.letterSpacingMm = 1.5;
        const TextLayout l = layoutText(f, "ITI", s);
        REQUIRE(l.glyphs.size() == 3);
        CHECK(l.glyphs[1].origin.x == Approx(4.0 + 1.5));
        CHECK(l.glyphs[2].origin.x == Approx(4.0 + 7.0 + 3.0));
        CHECK(l.lines[0].widthMm == Approx(4.0 + 7.0 + 4.0 + 3.0));  // spacing between glyphs only
        s.letterSpacingMm = -1.0;
        CHECK(layoutText(f, "II", s).glyphs[1].origin.x == Approx(3.0));
    }
    SECTION("space has an advance and no outline") {
        const TextLayout l = layoutText(f, "I I", s);
        REQUIRE(l.glyphs.size() == 3);
        CHECK(l.glyphs[1].contourCount == 0);
        CHECK(l.glyphs[1].advance == Approx(3.0));
        CHECK(l.glyphs[2].origin.x == Approx(4.0 + 3.0));
        CHECK(l.contours.size() == 2);
    }
}

TEST_CASE("Alignment moves each line about the anchor", "[text][layout]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    // "IT" is 11 mm wide.
    CHECK(layoutText(f, "IT", s).glyphs[0].origin.x == Approx(0.0));
    s.alignment = Alignment::Centre;
    CHECK(layoutText(f, "IT", s).glyphs[0].origin.x == Approx(-5.5));
    s.alignment = Alignment::Right;
    const TextLayout r = layoutText(f, "IT", s);
    CHECK(r.glyphs[0].origin.x == Approx(-11.0));
    CHECK(r.glyphs[1].origin.x == Approx(-7.0));
    CHECK(r.lines[0].baselineStart.x == Approx(-11.0));
}

TEST_CASE("Lines break at LF, CRLF and CR with the font's line pitch", "[text][layout]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    for (const char* text : {"I\nT", "I\r\nT", "I\rT"}) {
        const TextLayout l = layoutText(f, text, s);
        REQUIRE(l.ok());
        REQUIRE(l.lines.size() == 2);
        REQUIRE(l.glyphs.size() == 2);
        CHECK(l.glyphs[0].line == 0);
        CHECK(l.glyphs[1].line == 1);
        CHECK(l.glyphs[1].origin.y == Approx(-11.0));  // 1100 units = 11 mm
        CHECK(l.lines[1].baselineStart.y == Approx(-11.0));
        CHECK(l.glyphs[1].cluster == std::string(text).size() - 1);
    }
    s.lineSpacing = 2.0;
    CHECK(layoutText(f, "I\nT\nI", s).glyphs[2].origin.y == Approx(-44.0));
    s.lineSpacing = 0.0;
    CHECK(layoutText(f, "I\nT", s).glyphs[1].origin.y == Approx(0.0).margin(1e-12));
    // Each line is aligned on its own.
    s = style10();
    s.alignment = Alignment::Right;
    const TextLayout l = layoutText(f, "IT\nI", s);
    CHECK(l.glyphs[2].origin.x == Approx(-4.0));
}

TEST_CASE("Empty text, blank lines and missing glyphs", "[text][layout][hostile]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    SECTION("empty") {
        const TextLayout l = layoutText(f, "", s);
        CHECK(l.ok());
        CHECK(l.glyphs.empty());
        CHECK(l.contours.empty());
        CHECK(l.bounds.isEmpty());
        CHECK(l.lines.size() == 1);
    }
    SECTION("only line breaks") {
        const TextLayout l = layoutText(f, "\n\n", s);
        CHECK(l.ok());
        CHECK(l.lines.size() == 3);
        CHECK(l.glyphs.empty());
    }
    SECTION("a character the font lacks shows .notdef") {
        const TextLayout l = layoutText(f, "Z", s);
        REQUIRE(l.ok());
        REQUIRE(l.glyphs.size() == 1);
        CHECK(l.glyphs[0].missing);
        CHECK(l.glyphs[0].glyphId == 0);
        CHECK(l.glyphs[0].contourCount == 2);
        CHECK(netArea(l.contours, 0, 2) == Approx(18.0).epsilon(1e-12));  // 180000 * 1e-4
        CHECK(l.glyphs[0].advance == Approx(5.0));
    }
    SECTION("emoji and other non-BMP characters are missing, not fatal") {
        const TextLayout l = layoutText(f, "I\xF0\x9F\x98\x80I", s);
        REQUIRE(l.ok());
        REQUIRE(l.glyphs.size() == 3);
        CHECK(l.glyphs[1].missing);
    }
    SECTION("invalid UTF-8 is replaced and flagged") {
        for (const char* bad : {"\xFF", "I\xC3", "\xED\xA0\x80", "\xF4\x90\x80\x80", "\xC0\x80", "\xE0\x80\x80"}) {
            const TextLayout l = layoutText(f, bad, s);
            CHECK(l.ok());
            CHECK(l.replacedInvalidUtf8);
        }
        CHECK_FALSE(layoutText(f, "I\xC3\xA9", s).replacedInvalidUtf8);
    }
    SECTION("embedded NUL and control characters are bounded input like any other") {
        const std::string text("I\0T\x01\x1F", 5);
        const TextLayout l = layoutText(f, text, s);
        CHECK(l.ok());
        CHECK(l.glyphs.size() >= 2);
    }
}

TEST_CASE("Right-to-left text is shaped in visual order without mirroring", "[text][layout][rtl]") {
    const Font f = loadTestFont();
    TextStyle s = style10();
    const std::string text = "\xD7\x90\xD7\x90T";  // alef alef T
    SECTION("auto direction follows the first strong character") {
        const TextLayout l = layoutText(f, text, s);
        REQUIRE(l.glyphs.size() == 3);
        // RTL run: logical order is reversed on the page (T is leftmost).
        CHECK(l.glyphs[0].cluster == 4);
        CHECK(l.glyphs[2].cluster == 0);
        CHECK(l.glyphs[0].glyphId == 3);
        CHECK(l.glyphs[1].origin.x == Approx(7.0));
        CHECK(l.glyphs[2].origin.x == Approx(13.0));
        // The alef stays an upright L: foot on the right of its stem, bounds not mirrored.
        const PlacedGlyph& a = l.glyphs[2];
        geometry::Box2 b;
        for (std::size_t i = 0; i < a.contourCount; ++i) {
            b.expand(l.contours[a.firstContour + i].bounds());
        }
        CHECK(b.min.x == Approx(13.0 + 1.0));
        CHECK(b.max.x == Approx(13.0 + 5.0));
    }
    SECTION("forced directions") {
        s.direction = TextDirection::LeftToRight;
        const TextLayout ltr = layoutText(f, text, s);
        CHECK(ltr.glyphs[0].cluster == 0);
        CHECK(ltr.glyphs[2].cluster == 4);
        s.direction = TextDirection::RightToLeft;
        const TextLayout rtl = layoutText(f, "IT", s);
        CHECK(rtl.glyphs[0].cluster == 1);
        CHECK(rtl.glyphs[1].cluster == 0);
    }
    SECTION("latin text stays left to right") {
        const TextLayout l = layoutText(f, "IT", s);
        CHECK(l.glyphs[0].cluster == 0);
    }
}

TEST_CASE("Combining marks add no advance and keep their cluster", "[text][layout]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    const TextLayout l = layoutText(f, "I\xCC\x81T", s);  // I, U+0301, T
    REQUIRE(l.ok());
    REQUIRE(l.glyphs.size() == 3);
    CHECK(l.glyphs[1].glyphId == 6);
    CHECK(l.glyphs[1].advance == Approx(0.0).margin(1e-12));
    CHECK(l.glyphs[1].cluster == 0);  // a mark joins its base's cluster
    CHECK(l.glyphs[2].origin.x == Approx(4.0));
    CHECK(l.lines[0].widthMm == Approx(11.0));
    // HarfBuzz's fallback mark placement puts the bar (unplaced: y 7.5 to 8.5 mm) above the base's
    // 7 mm height; the exact nudge is HarfBuzz's, so assert only "above the base and not lower".
    REQUIRE(l.glyphs[1].contourCount == 1);
    const geometry::Box2 mark = l.contours[l.glyphs[1].firstContour].bounds();
    CHECK(mark.min.y >= 7.0);
    CHECK(mark.max.y >= 8.5 - 1e-9);
    CHECK(mark.height() == Approx(1.0));
}

TEST_CASE("A composite glyph is resolved", "[text][layout]") {
    const Font f = loadTestFont();
    const TextLayout l = layoutText(f, "\xC3\x89", style10());  // U+00C9
    REQUIRE(l.ok());
    REQUIRE(l.glyphs.size() == 1);
    CHECK(l.glyphs[0].glyphId == 7);
    CHECK(l.contours.size() == 2);
    CHECK(netArea(l.contours, 0, 2) == Approx(16.0).epsilon(1e-12));
    CHECK(l.bounds.max.y == Approx(8.5));
}

TEST_CASE("Layout output is deterministic", "[text][layout][determinism]") {
    const Font f = loadTestFont();
    TextStyle s;
    s.sizeMm = 12.5;
    s.letterSpacingMm = 0.3;
    s.alignment = Alignment::Centre;
    const std::string text = "TOI IT\xC3\x89\n\xD7\x90 Z\xCC\x81";
    const TextLayout a = layoutText(f, text, s);
    const TextLayout b = layoutText(f, text, s);
    const TextLayout c = layoutText(loadTestFont(), text, s);
    REQUIRE(a.ok());
    CHECK(hashLayout(a) == hashLayout(b));
    CHECK(hashLayout(a) == hashLayout(c));
    CHECK(a.contours == b.contours);
    // Golden: identical on every platform and compiler (integer font units, IEEE +-*/ only, no
    // libm, contraction off). Recorded on Linux/GCC; CI checks Linux and Windows against it.
    CHECK(hashLayout(a) == 0x33c4912177b82920ull);
}
