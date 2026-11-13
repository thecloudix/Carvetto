#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "text_test_util.hpp"

using namespace carvetto;
using namespace carvetto::text;
using testutil::fontBytes;
using testutil::loadTestFont;
using testutil::signedArea;

TEST_CASE("Font loads the OFL test font from bytes and from a file", "[text][font]") {
    const Font f = loadTestFont();
    REQUIRE(f.valid());
    CHECK(f.metrics().unitsPerEm == 1000);
    CHECK(f.metrics().ascender == 800);
    CHECK(f.metrics().descender == -200);
    CHECK(f.metrics().lineHeight == 1100);  // ascender - descender + line gap
    CHECK(f.metrics().glyphCount == 8);
    CHECK(f.familyName() == "Carvetto Test");

    FontLoad fromFile = Font::loadFile(testutil::fixturePath());
    REQUIRE(fromFile.ok());
    CHECK(fromFile.font.metrics().glyphCount == 8);
}

TEST_CASE("Font maps code points to glyph ids", "[text][font]") {
    const Font f = loadTestFont();
    CHECK(f.glyphIndex(U' ') == 1);
    CHECK(f.glyphIndex(U'I') == 2);
    CHECK(f.glyphIndex(U'T') == 3);
    CHECK(f.glyphIndex(U'O') == 4);
    CHECK(f.glyphIndex(0x5D0) == 5);
    CHECK(f.glyphIndex(0x301) == 6);
    CHECK(f.glyphIndex(0xC9) == 7);
    CHECK(f.glyphIndex(U'Z') == 0);  // missing: .notdef
    CHECK(Font{}.glyphIndex(U'I') == 0);
}

TEST_CASE("Font refuses hostile or unsupported input with a status", "[text][font][hostile]") {
    SECTION("empty") {
        CHECK(Font::load({}).status == FontStatus::Empty);
    }
    SECTION("not a font") {
        const std::vector<std::uint8_t> junk(2000, 0x41);
        const FontLoad r = Font::load(junk);
        CHECK(r.status == FontStatus::UnsupportedFormat);
        CHECK_FALSE(r.font.valid());
    }
    SECTION("a Type 1 / WOFF header never reaches FreeType") {
        std::vector<std::uint8_t> woff = {'w', 'O', 'F', 'F', 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        CHECK(Font::load(woff).status == FontStatus::UnsupportedFormat);
        std::vector<std::uint8_t> t1 = {'%', '!', 'P', 'S', '-', 'A', 'd', 'o', 'b', 'e', 'F', 'o', 'n', 't'};
        CHECK(Font::load(t1).status == FontStatus::UnsupportedFormat);
    }
    SECTION("sfnt magic with garbage behind it") {
        for (const std::uint32_t magic : {0x00010000u, 0x4F54544Fu, 0x74727565u, 0x74746366u}) {
            std::vector<std::uint8_t> junk(300, 0xFF);
            junk[0] = static_cast<std::uint8_t>(magic >> 24);
            junk[1] = static_cast<std::uint8_t>(magic >> 16);
            junk[2] = static_cast<std::uint8_t>(magic >> 8);
            junk[3] = static_cast<std::uint8_t>(magic);
            CHECK(Font::load(junk).status == FontStatus::Corrupt);
        }
    }
    SECTION("file size limit") {
        const std::vector<std::uint8_t> bytes = fontBytes();
        FontLimits lim;
        lim.maxFileBytes = bytes.size() - 1;
        CHECK(Font::load(bytes, lim).status == FontStatus::TooLarge);
        lim.maxFileBytes = bytes.size();
        CHECK(Font::load(bytes, lim).ok());
        // The caller cannot lift the ceiling.
        lim.maxFileBytes = static_cast<std::size_t>(-1);
        CHECK(Font::load(bytes, lim).ok());
    }
    SECTION("glyph count limit") {
        FontLimits lim;
        lim.maxGlyphs = 7;
        CHECK(Font::load(fontBytes(), lim).status == FontStatus::Corrupt);
        lim.maxGlyphs = 8;
        CHECK(Font::load(fontBytes(), lim).ok());
    }
    SECTION("missing file") {
        CHECK(Font::loadFile(std::string(CARVETTO_TEXT_FIXTURE_DIR) + "/nope.ttf").status == FontStatus::IoError);
    }
    SECTION("every truncation of a valid font is refused or loads cleanly") {
        const std::vector<std::uint8_t> bytes = fontBytes();
        for (std::size_t n = 0; n < bytes.size(); ++n) {
            const FontLoad r = Font::load(std::span<const std::uint8_t>(bytes.data(), n));
            if (r.ok()) {
                // A font that loads must still lay out without crashing.
                const TextLayout l = layoutText(r.font, "ITO\xD7\x90 Z");
                CHECK((l.ok() || l.status != TextStatus::Ok));
            }
        }
        // Cutting into the glyph data (the directory points past the end) never loads.
        CHECK_FALSE(Font::load(std::span<const std::uint8_t>(bytes.data(), 200)).ok());
    }
}

TEST_CASE("Font is movable and an empty Font is inert", "[text][font]") {
    Font a = loadTestFont();
    Font b = std::move(a);
    CHECK(b.valid());
    CHECK_FALSE(a.valid());  // NOLINT(bugprone-use-after-move): documented moved-from state
    CHECK(b.outline(2).contours.size() == 1);
    const Font none;
    CHECK(none.outline(2).status == OutlineStatus::LoadFailed);
    CHECK(none.metrics().glyphCount == 0);
    CHECK(none.familyName().empty());
    const TextLayout l = layoutText(none, "I");
    CHECK(l.status == TextStatus::InvalidFont);
}

TEST_CASE("loadFile clamps the size limit to the absolute ceiling", "[text][font][hostile]") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "carvetto_font_ceiling_test";
    fs::create_directories(dir);
    const fs::path file = dir / "big.ttf";
    {
        std::ofstream out(file, std::ios::binary);
        out.put('x');
    }
    std::error_code ec;
    fs::resize_file(file, kAbsoluteMaxFontBytes + 1, ec);  // sparse where the file system allows
    if (!ec) {
        FontLimits lim;
        lim.maxFileBytes = static_cast<std::size_t>(-1);  // "no limit" still hits the ceiling
        CHECK(Font::loadFile(file.string(), lim).status == FontStatus::TooLarge);
    }
    fs::remove_all(dir, ec);
    CHECK_FALSE(ec);
}
