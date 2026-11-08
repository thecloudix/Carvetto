#include <cmath>
#include <limits>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "text_test_util.hpp"

using namespace carvetto;
using namespace carvetto::text;
using Catch::Approx;
using testutil::loadTestFont;

namespace {
TextStyle style10() {
    TextStyle s;
    s.sizeMm = 10.0;
    return s;
}

void requireRefused(const TextLayout& l, TextStatus status) {
    CHECK(l.status == status);
    CHECK_FALSE(l.ok());
    CHECK_FALSE(l.message.empty());
    // A refusal never returns partial output.
    CHECK(l.glyphs.empty());
    CHECK(l.contours.empty());
    CHECK(l.lines.empty());
    CHECK(l.bounds.isEmpty());
}
}  // namespace

TEST_CASE("Invalid styles are refused", "[text][limits][hostile]") {
    const Font f = loadTestFont();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    TextStyle s = style10();
    for (const double bad : {nan, inf, -inf, 0.0, -1.0, 10001.0, 1e300}) {
        s.sizeMm = bad;
        requireRefused(layoutText(f, "I", s), TextStatus::InvalidStyle);
    }
    s = style10();
    for (const double bad : {nan, inf, 10001.0, -10001.0}) {
        s.letterSpacingMm = bad;
        requireRefused(layoutText(f, "I", s), TextStatus::InvalidStyle);
    }
    s = style10();
    for (const double bad : {nan, inf, -0.5, 100.5}) {
        s.lineSpacing = bad;
        requireRefused(layoutText(f, "I", s), TextStatus::InvalidStyle);
    }
    s = style10();
    for (const char* bad : {"", "e n", "en_US", "this-language-tag-is-way-too-long-for-bcp47"}) {
        s.language = bad;
        requireRefused(layoutText(f, "I", s), TextStatus::InvalidStyle);
    }
    s.language = "de-CH";
    CHECK(layoutText(f, "I", s).ok());

    // The edge of the size range works and stays finite.
    s = style10();
    s.sizeMm = 10000.0;
    const TextLayout big = layoutText(f, "O", s);
    REQUIRE(big.ok());
    CHECK(std::isfinite(big.bounds.max.x));
    CHECK(big.bounds.max.x == Approx(7000.0).epsilon(1e-12));
    // A caller limit can lower the size cap, and cannot raise it past 1e6.
    TextLimits lim;
    lim.maxSizeMm = 20.0;
    s.sizeMm = 21.0;
    requireRefused(layoutText(f, "I", s, lim), TextStatus::InvalidStyle);
    lim.maxSizeMm = 1e30;
    s.sizeMm = 2e6;
    requireRefused(layoutText(f, "I", s, lim), TextStatus::InvalidStyle);
}

TEST_CASE("Size limits refuse with a status", "[text][limits][hostile]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    TextLimits lim;

    SECTION("bytes") {
        lim.maxTextBytes = 3;
        requireRefused(layoutText(f, "IIII", s, lim), TextStatus::TextTooLong);
        CHECK(layoutText(f, "III", s, lim).ok());
    }
    SECTION("the byte ceiling cannot be lifted") {
        lim.maxTextBytes = static_cast<std::size_t>(-1);
        const std::string huge(kAbsoluteMaxTextBytes + 1, 'I');
        requireRefused(layoutText(f, huge, s, lim), TextStatus::TextTooLong);
    }
    SECTION("code points count characters, not bytes") {
        lim.maxCodePoints = 3;
        requireRefused(layoutText(f, "IIII", s, lim), TextStatus::TooManyCodePoints);
        CHECK(layoutText(f, "I\xC3\xA9I", s, lim).ok());  // 3 characters, 4 bytes
        requireRefused(layoutText(f, "\xFF\xFF\xFF\xFF", s, lim), TextStatus::TooManyCodePoints);
    }
    SECTION("lines") {
        lim.maxLines = 2;
        requireRefused(layoutText(f, "I\nI\nI", s, lim), TextStatus::TooManyLines);
        CHECK(layoutText(f, "I\nI", s, lim).ok());
        requireRefused(layoutText(f, "I\nI\n", s, lim), TextStatus::TooManyLines);  // trailing empty line is a line
        requireRefused(layoutText(f, "\n\n\n\n\n\n", s, lim), TextStatus::TooManyLines);
    }
    SECTION("glyphs") {
        lim.maxGlyphs = 3;
        requireRefused(layoutText(f, "IIII", s, lim), TextStatus::TooManyGlyphs);
        CHECK(layoutText(f, "III", s, lim).ok());
        // The cap is on the whole text, not per line.
        requireRefused(layoutText(f, "II\nII", s, lim), TextStatus::TooManyGlyphs);
    }
    SECTION("spans: decoded and emitted") {
        // 'O' has 8 curve spans.
        lim.maxSpans = 7;
        requireRefused(layoutText(f, "O", s, lim), TextStatus::TooManySpans);  // decoding alone is too much
        lim.maxSpans = 8;
        CHECK(layoutText(f, "O", s, lim).ok());
        requireRefused(layoutText(f, "OO", s, lim), TextStatus::TooManySpans);  // decoded once, emitted twice
        lim.maxSpans = 16;
        CHECK(layoutText(f, "OO", s, lim).ok());
    }
    SECTION("glyph points: an oversized glyph is skipped, not fatal") {
        lim.maxGlyphPoints = 3;  // 'I' has 4
        const TextLayout l = layoutText(f, "II", s, lim);
        REQUIRE(l.ok());
        CHECK(l.skippedGlyphs == 2);
        CHECK(l.glyphs.size() == 2);
        CHECK(l.contours.empty());
        CHECK(l.glyphs[1].origin.x == Approx(4.0));  // layout still advances
        lim.maxGlyphPoints = 4;
        const TextLayout ok = layoutText(f, "II", s, lim);
        CHECK(ok.skippedGlyphs == 0);
        CHECK(ok.contours.size() == 2);
    }
    SECTION("work meter is deterministic and bounded") {
        // 1 per code point + 4 per glyph + outline points once per distinct glyph + 1 per span emitted.
        const TextLayout one = layoutText(f, "II", s);
        CHECK(one.work == 2 + 8 + 4 + 8);
        lim.maxWork = one.work;
        CHECK(layoutText(f, "II", s, lim).ok());
        lim.maxWork = one.work - 1;
        requireRefused(layoutText(f, "II", s, lim), TextStatus::WorkLimit);
        // The same meter is reported for the same input every time.
        CHECK(layoutText(f, "II", s).work == one.work);
    }
    SECTION("a long text is processed within the defaults") {
        std::string text;
        for (int i = 0; i < 2000; ++i) {
            text += "TOI IT ";
        }
        const TextLayout l = layoutText(f, text, s);
        REQUIRE(l.ok());
        CHECK(l.glyphs.size() == 14000);
        // And a text that would need more than the span budget is refused whole.
        const std::string many(200000, 'O');  // 200k glyphs * 8 spans > 1M
        requireRefused(layoutText(f, many, s), TextStatus::TooManySpans);
    }
}

TEST_CASE("Hostile characters cannot make shaping run away", "[text][limits][hostile]") {
    const Font f = loadTestFont();
    const TextStyle s = style10();
    TextLimits lim;
    lim.maxWork = 2'000'000;
    // Long runs of combining marks and zero-width / joiner characters, in every direction.
    std::string marks;
    for (int i = 0; i < 20000; ++i) {
        marks += "I\xCC\x81\xCC\x82\xCC\x83\xE2\x80\x8D\xD7\x90\xD6\xB0";
    }
    for (const TextDirection d : {TextDirection::Auto, TextDirection::LeftToRight, TextDirection::RightToLeft}) {
        TextStyle t = s;
        t.direction = d;
        const TextLayout l = layoutText(f, marks, t, lim);
        CHECK((l.ok() || l.status == TextStatus::WorkLimit || l.status == TextStatus::TooManySpans ||
               l.status == TextStatus::TooManyGlyphs));
    }
}
