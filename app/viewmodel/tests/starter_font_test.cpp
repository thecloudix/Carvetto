// The compiled-in starter font, DejaVu Serif (T-0233): coverage, layout speed, embedding in .carv,
// and the Text tool / DXF import working with the full font.

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/text/layout.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"
#include "carvetto/viewmodel/starter_font.hpp"
#include "carvetto/viewmodel/tools.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

std::vector<std::uint8_t> starterBytes() {
    const auto s = starterFontBytes();
    return std::vector<std::uint8_t>(s.begin(), s.end());
}

const char* const kSample = "Hello World 123 \xC3\x84\xC3\x96\xC3\x9C \xC3\x9F \xC3\xA9 \xC3\xB1 \xCE\xA9 \xD0\x96";

}  // namespace

TEST_CASE("the starter font is DejaVu Serif and covers Latin, Latin-1, Greek and Cyrillic", "[text][font][starter]") {
    const auto bytes = starterBytes();
    CHECK(bytes.size() == 380132);
    CHECK(std::string(starterFontName()) == "DejaVu Serif");
    text::FontLoad load = text::Font::load(bytes);
    REQUIRE(load.ok());
    CHECK(load.font.familyName() == "DejaVu Serif");

    for (char32_t c = 0x20; c < 0x7F; ++c) {
        CHECK(load.font.glyphIndex(c) != 0);
    }
    for (char32_t c = 0xA1; c <= 0xFF; ++c) {
        CHECK(load.font.glyphIndex(c) != 0);
    }
    for (char32_t c = 0x391; c <= 0x3C9; ++c) {
        if (c != 0x3A2) {
            CHECK(load.font.glyphIndex(c) != 0);
        }
    }
    for (char32_t c = 0x410; c <= 0x44F; ++c) {
        CHECK(load.font.glyphIndex(c) != 0);
    }

    const text::TextLayout l = text::layoutText(load.font, kSample);
    REQUIRE(l.ok());
    CHECK_FALSE(l.contours.empty());
    CHECK(l.skippedGlyphs == 0);
    for (const auto& g : l.glyphs) {
        CHECK_FALSE(g.missing);
    }
}

TEST_CASE("first use of the starter font loads and lays out quickly", "[text][font][starter]") {
    const auto t0 = std::chrono::steady_clock::now();
    text::FontLoad load = text::Font::load(starterBytes());
    REQUIRE(load.ok());
    const text::TextLayout l = text::layoutText(load.font, kSample);
    REQUIRE(l.ok());
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // Generous so a loaded CI machine or a sanitizer build never flakes; the real cost is a few ms.
    CHECK(ms < 5000.0);
}

TEST_CASE("Text tool with the full starter font: place, save, load, undo, convert, text on a curve",
          "[text][tool][starter]") {
    Document doc(Job::singleSided(1000.0, 1000.0, 18.0, Units::Millimetres));
    ViewTransform view;
    view.setViewport(1000.0, 1000.0);
    view.set(2.0, {100.0, 900.0});
    ToolController tool;
    tool.setDocument(&doc);
    tool.setView(view);
    tool.setTool(ToolKind::Text);
    auto& ed = tool.textEditor();
    REQUIRE(ed.setFont(starterBytes(), starterFontName()).ok);
    CHECK(ed.fontName() == "DejaVu Serif");

    const auto click = [&](geometry::Vec2 mm) {
        tool.pointerMove(view.worldToScreen(mm));
        tool.pointerPress(view.worldToScreen(mm));
        tool.pointerRelease(view.worldToScreen(mm));
    };
    click({100.0, 100.0});
    REQUIRE(ed.hasDraft());
    ed.insertText(kSample);
    CHECK(tool.key(ToolKey::Enter).committed);
    REQUIRE(doc.objectCount() == 1);
    const VectorObject* v = doc.objects().begin()->second.asVector();
    const auto* tp = std::get_if<TextParams>(&v->params);
    REQUIRE(tp != nullptr);
    CHECK(tp->text == kSample);
    CHECK_FALSE(v->contours.empty());
    CHECK(doc.checkIntegrity().empty());
    REQUIRE(doc.fonts().size() == 1);

    // The whole font is embedded: about 380 KB, far below the caps.
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    CHECK(saved.value().size() > 100000);
    CHECK(saved.value().size() < 1000000);
    auto loaded = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().document.fonts().size() == 1);
    CHECK(loaded.value().document.checkIntegrity().empty());
    auto again = saveCarv(loaded.value().document, loaded.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());

    // Convert to curves, then undo.
    const auto contours = v->contours;
    CHECK(ed.convertSelectionToCurves().committed);
    CHECK(std::holds_alternative<NoParams>(doc.objects().begin()->second.asVector()->params));
    CHECK(doc.objects().begin()->second.asVector()->contours == contours);
    REQUIRE(doc.undo());
    CHECK(std::holds_alternative<TextParams>(doc.objects().begin()->second.asVector()->params));
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.fonts().empty());
    REQUIRE(doc.redo());
    CHECK(doc.fonts().size() == 1);

    // Text on a curve.
    geometry::Contour line;
    line.closed = false;
    line.spans.push_back(geometry::LineSpan{{100.0, 300.0}, {400.0, 300.0}});
    doc.insertObject(Object::vector(line, doc.activeLayer(), doc.activeSheet()));
    ed.setMode(TextMode::Curve);
    click({150.0, 300.0});
    REQUIRE(ed.hasDraft());
    ed.insertText("Hello World");
    CHECK_FALSE(ed.previewContours().empty());
    CHECK(tool.key(ToolKey::Enter).committed);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("DXF text imports with real glyphs and no .notdef warning", "[text][dxf][starter]") {
    const std::string dxf =
        "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n"
        "0\nSECTION\n2\nENTITIES\n"
        "0\nTEXT\n8\n0\n10\n50\n20\n20\n40\n7\n1\nHello World 123\n"
        "0\nMTEXT\n8\n0\n10\n0\n20\n200\n40\n7\n41\n60\n71\n1\n1\nThe quick brown fox\n"
        "0\nENDSEC\n0\nEOF\n";
    const std::vector<std::uint8_t> bytes(dxf.begin(), dxf.end());
    Document doc;
    const DxfImportOutcome out = importDxfIntoDocument(doc, bytes, std::nullopt, starterBytes());
    REQUIRE(out.ok);
    CHECK(out.textAdded == 2);
    CHECK(out.textSkipped == 0);
    for (const std::string& w : out.warnings) {
        CHECK(w.find("notdef") == std::string::npos);
        CHECK(w.find("glyph") == std::string::npos);
    }
    REQUIRE(doc.objectCount() == 2);
    for (const auto& entry : doc.objects()) {
        CHECK(entry.second.asVector()->contours.size() > 5);
    }
}
