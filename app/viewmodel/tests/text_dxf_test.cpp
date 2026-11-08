// DXF TEXT / MTEXT import onto Text objects (T-0211).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
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
