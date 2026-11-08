// DXF TEXT / MTEXT import onto Text objects (T-0211).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;
using Catch::Approx;

namespace {

std::vector<std::uint8_t> fontBytes() {
    std::ifstream in(std::string(CARVETTO_TEXT_FIXTURE_DIR) + "/CarvettoTest-Regular.ttf", std::ios::binary);
    REQUIRE(in.good());
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

geometry::Box2 boundsOf(const std::vector<geometry::Contour>& cs) {
    geometry::Box2 b;
    for (const auto& c : cs) {
        b.expand(c.bounds());
    }
    return b;
}

const std::string kDxf =
    "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
    "0\nSECTION\n2\nENTITIES\n"
    "0\nTEXT\n8\n0\n10\n50\n20\n20\n40\n7\n50\n90\n1\nTOI\n"
    "0\nMTEXT\n8\n0\n10\n0\n20\n200\n40\n7\n41\n12\n71\n1\n1\nI I I I\n"
    "0\nENDSEC\n0\nEOF\n";

}  // namespace

TEST_CASE("DXF text without a font is skipped with a warning", "[text][dxf]") {
    const std::vector<std::uint8_t> bytes(kDxf.begin(), kDxf.end());
    Document doc;
    const DxfImportOutcome out = importDxfIntoDocument(doc, bytes);
    CHECK(out.ok);
    CHECK(out.textSkipped == 2);
    CHECK(out.textAdded == 0);
    CHECK(doc.objectCount() == 0);
    CHECK_FALSE(out.warnings.empty());
}

TEST_CASE("DXF TEXT and MTEXT become text objects when a font is given", "[text][dxf]") {
    const std::vector<std::uint8_t> bytes(kDxf.begin(), kDxf.end());
    Document doc;
    const std::size_t steps = doc.undoStack().count();
    const auto font = fontBytes();
    const DxfImportOutcome out = importDxfIntoDocument(doc, bytes, std::nullopt, font);
    REQUIRE(out.ok);
    CHECK(out.textAdded == 2);
    CHECK(out.textSkipped == 0);
    REQUIRE(doc.objectCount() == 2);
    CHECK(doc.fonts().size() == 1);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.checkIntegrity().empty());
    CHECK(doc.selection().size() == 2);

    const Object& single = doc.objects().begin()->second;
    const auto* tp = std::get_if<TextParams>(&single.asVector()->params);
    REQUIRE(tp != nullptr);
    CHECK(tp->text == "TOI");
    CHECK(tp->sizeMm == Approx(10.0));  // 7 mm cap height / 0.7 em
    // Rotated 90 degrees about (50, 20): the 7 mm glyph height runs to the left of the anchor.
    const geometry::Box2 b = boundsOf(single.asVector()->contours);
    CHECK(b.max.x == Approx(50.0).margin(1e-9));
    CHECK(b.min.x == Approx(43.0).margin(1e-9));
    CHECK(b.min.y == Approx(20.0).margin(1e-9));

    const Object& boxed = doc.objects().rbegin()->second;
    const auto* bp = std::get_if<TextParams>(&boxed.asVector()->params);
    REQUIRE(bp != nullptr);
    CHECK(bp->mode == TextMode::Box);
    CHECK(bp->boxWidthMm == Approx(12.0));
    const geometry::Box2 bb = boundsOf(boxed.asVector()->contours);
    CHECK(bb.max.y == Approx(199.0));  // first baseline one ascender below the box top
    CHECK(bb.min.y == Approx(181.0));  // wrapped to two lines

    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.fonts().empty());
    REQUIRE(doc.redo());
    CHECK(doc.objectCount() == 2);
}

TEST_CASE("DXF text with bytes that are not a font is skipped", "[text][dxf]") {
    const std::vector<std::uint8_t> bytes(kDxf.begin(), kDxf.end());
    Document doc;
    const std::vector<std::uint8_t> junk{1, 2, 3, 4, 5};
    const DxfImportOutcome out = importDxfIntoDocument(doc, bytes, std::nullopt, junk);
    CHECK(out.ok);
    CHECK(out.textSkipped == 2);
    CHECK(doc.objectCount() == 0);
}

namespace {
std::string justified(int h, int v, const char* p1, const char* p2) {
    return std::string("0\nSECTION\n2\nENTITIES\n0\nTEXT\n8\n0\n10\n") + p1 + "\n40\n5\n1\nI\n50\n0\n41\n1\n72\n" +
           std::to_string(h) + "\n73\n" + std::to_string(v) + "\n11\n" + p2 + "\n0\nENDSEC\n0\nEOF\n";
}
geometry::Box2 importOne(const std::string& dxf, bool* ok = nullptr) {
    const std::vector<std::uint8_t> bytes(dxf.begin(), dxf.end());
    Document doc;
    const auto font = fontBytes();
    const DxfImportOutcome out = importDxfIntoDocument(doc, bytes, std::nullopt, font);
    if (ok != nullptr) {
        *ok = out.ok && doc.objectCount() == 1;
    }
    if (doc.objectCount() != 1) {
        return {};
    }
    return boundsOf(doc.objects().begin()->second.asVector()->contours);
}
}  // namespace

TEST_CASE("justified DXF TEXT is anchored at the second alignment point", "[text][dxf]") {
    // 'I' at height 5 -> em 7.143 mm; the bar is 1..3 of a 4 mm-per-10 advance: 0.714 mm margins.
    const double em = 5.0 / 0.7;
    SECTION("centre: the advance is centred on group 11") {
        const geometry::Box2 b = importOne(justified(1, 0, "0\n20\n0", "50\n21\n0"));
        CHECK((b.min.x + b.max.x) * 0.5 == Approx(50.0).margin(1e-9));
        CHECK(b.min.y == Approx(0.0).margin(1e-9));
    }
    SECTION("right: the advance ends on group 11") {
        const geometry::Box2 b = importOne(justified(2, 0, "0\n20\n0", "50\n21\n0"));
        CHECK(b.max.x == Approx(50.0 - 0.1 * em).margin(1e-9));
    }
    SECTION("left with both points equal is unchanged") {
        const geometry::Box2 b = importOne(justified(0, 0, "10\n20\n0", "10\n21\n0"));
        CHECK(b.min.x == Approx(10.0 + 0.1 * em).margin(1e-9));
    }
    SECTION("middle: centred horizontally and vertically on the anchor") {
        const geometry::Box2 b = importOne(justified(4, 0, "0\n20\n0", "50\n21\n30"));
        CHECK((b.min.x + b.max.x) * 0.5 == Approx(50.0).margin(1e-9));
        CHECK(b.max.y == Approx(30.0 + 2.5).margin(1e-9));  // 7 mm bar is 5 mm cap height; 2.5 above
        CHECK(b.min.y == Approx(30.0 - 2.5).margin(1e-9));
    }
    SECTION("bottom attachment lifts the baseline by the descender") {
        const geometry::Box2 b = importOne(justified(1, 1, "0\n20\n0", "50\n21\n0"));
        CHECK(b.min.y == Approx(0.2 * em).margin(1e-9));
    }
    SECTION("fit and aligned run the baseline from group 10 to group 11") {
        const geometry::Box2 fit = importOne(justified(5, 0, "0\n20\n0", "40\n21\n0"));
        CHECK(fit.max.x == Approx(40.0).margin(1e-9));
        CHECK(fit.max.y == Approx(5.0 / 0.7 * 0.7).margin(1e-9));  // height unchanged
        const geometry::Box2 aligned = importOne(justified(3, 0, "0\n20\n0", "40\n21\n0"));
        CHECK(aligned.max.x == Approx(40.0).margin(1e-9));
        CHECK(aligned.max.y > fit.max.y);  // taller: scaled with the width
    }
}

TEST_CASE("DXF text in a font without the glyphs warns", "[text][dxf]") {
    const std::string dxf = justified(0, 0, "0\n20\n0", "0\n21\n0");
    std::string z = dxf;
    z.replace(z.find("\nI\n"), 3, "\nZ\n");
    const std::vector<std::uint8_t> bytes(z.begin(), z.end());
    Document doc;
    const auto font = fontBytes();
    const DxfImportOutcome out = importDxfIntoDocument(doc, bytes, std::nullopt, font);
    CHECK(out.textAdded == 1);
    bool warned = false;
    for (const std::string& w : out.warnings) {
        warned = warned || w.find("no glyph") != std::string::npos;
    }
    CHECK(warned);
}

TEST_CASE("DXF text is skipped with a warning when the font table is full", "[text][dxf][caps]") {
    Document doc;
    for (std::size_t i = 0; i < kMaxEmbeddedFonts; ++i) {
        char key[17];
        std::snprintf(key, sizeof key, "%016zx", i + 1);
        REQUIRE(doc.insertFont(FontAsset{key, {1}}));
    }
    const std::vector<std::uint8_t> bytes(kDxf.begin(), kDxf.end());
    const std::size_t steps = doc.undoStack().count();
    const auto font = fontBytes();
    const DxfImportOutcome out = importDxfIntoDocument(doc, bytes, std::nullopt, font);
    CHECK(out.textAdded == 0);
    CHECK(out.textSkipped == 2);
    CHECK(doc.objectCount() == 0);
    CHECK(doc.undoStack().count() == steps);
    bool named = false;
    for (const std::string& w : out.warnings) {
        named = named || w.find("64 fonts") != std::string::npos;
    }
    CHECK(named);
    CHECK(doc.checkIntegrity().empty());
}
