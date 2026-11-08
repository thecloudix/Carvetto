// Text tool (T-0211): point text, text in a box, text on a curve, editing in place, converting to
// curves, undo as one step, and document replacement in the middle of a gesture.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/viewmodel/starter_font.hpp"
#include "carvetto/viewmodel/tools.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;
using Catch::Approx;
using geometry::Vec2;

namespace {

std::vector<std::uint8_t> fontBytes() {
    std::ifstream in(std::string(CARVETTO_TEXT_FIXTURE_DIR) + "/CarvettoTest-Regular.ttf", std::ios::binary);
    REQUIRE(in.good());
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

double area(const std::vector<geometry::Contour>& cs) {
    // Independent oracle: shoelace over a fine polyline of every span.
    double a = 0.0;
    for (const auto& c : cs) {
        std::vector<Vec2> pts;
        for (const auto& s : c.spans) {
            for (int i = 0; i < 64; ++i) {
                pts.push_back(geometry::pointAt(s, i / 64.0));
            }
        }
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2 p = pts[i];
            const Vec2 q = pts[(i + 1) % pts.size()];
            a += 0.5 * (p.x * q.y - q.x * p.y);
        }
    }
    return a;
}

geometry::Box2 boundsOf(const std::vector<geometry::Contour>& cs) {
    geometry::Box2 b;
    for (const auto& c : cs) {
        b.expand(c.bounds());
    }
    return b;
}

struct Fixture {
    Document doc;
    ViewTransform view;
    ToolController tool;
    Fixture() : doc(Job::singleSided(1000.0, 1000.0, 18.0, Units::Millimetres)) {
        view.setViewport(1000.0, 1000.0);
        view.set(2.0, {100.0, 900.0});
        tool.setDocument(&doc);
        tool.setView(view);
        tool.setTool(ToolKind::Text);
        REQUIRE(tool.textEditor().setFont(fontBytes(), "Test").ok);
    }
    Vec2 px(Vec2 mm) const { return view.worldToScreen(mm); }
    ToolResult click(Vec2 mm) {
        tool.pointerMove(px(mm));
        const ToolResult p = tool.pointerPress(px(mm));
        const ToolResult r = tool.pointerRelease(px(mm));
        ToolResult out = r;
        out.committed = p.committed || r.committed;
        if (out.message.empty()) {
            out.message = p.message;
        }
        return out;
    }
    ToolResult drag(Vec2 a, Vec2 b) {
        tool.pointerMove(px(a));
        tool.pointerPress(px(a));
        tool.pointerMove(px((a + b) * 0.5));
        tool.pointerMove(px(b));
        return tool.pointerRelease(px(b));
    }
    const VectorObject* onlyVector() const {
        REQUIRE(doc.objectCount() >= 1);
        return doc.objects().rbegin()->second.asVector();
    }
};

}  // namespace

TEST_CASE("point text: click, type, commit is one undo step", "[text][tool]") {
    Fixture f;
    const std::uint64_t before = f.doc.undoStack().generation();
    CHECK(f.click({100.0, 100.0}).message.empty());
    REQUIRE(f.tool.textEditor().hasDraft());
    CHECK(f.tool.active());
    CHECK(f.tool.wantsKeys());
    CHECK(f.tool.textEditor().insertText("TOI").draftChanged);
    // Typing never touches the document.
    CHECK(f.doc.undoStack().generation() == before);
    CHECK(f.doc.objectCount() == 0);
    const ToolPreview pv = f.tool.preview();
    CHECK(pv.valid);
    CHECK(area(pv.contours) == Approx(53.5).epsilon(1e-3));
    const geometry::Box2 box = boundsOf(pv.contours);
    CHECK(box.min.x == Approx(100.0));  // T bar starts at the origin
    CHECK(box.min.y == Approx(100.0));

    const ToolResult done = f.tool.key(ToolKey::Enter);
    CHECK(done.committed);
    CHECK(done.selectionChanged);
    CHECK_FALSE(f.tool.textEditor().hasDraft());
    REQUIRE(f.doc.objectCount() == 1);
    CHECK(f.doc.fonts().size() == 1);
    CHECK(f.doc.checkIntegrity().empty());
    const auto* tp = std::get_if<TextParams>(&f.onlyVector()->params);
    REQUIRE(tp != nullptr);
    CHECK(tp->text == "TOI");
    CHECK(tp->mode == TextMode::Point);
    CHECK(f.doc.selection().size() == 1);
    CHECK(area(f.onlyVector()->contours) == Approx(53.5).epsilon(1e-3));

    const Object committed = f.doc.objects().begin()->second;
    REQUIRE(f.doc.undo());
    CHECK(f.doc.objectCount() == 0);
    CHECK(f.doc.fonts().empty());
    CHECK_FALSE(f.doc.undo());  // exactly one step
    REQUIRE(f.doc.redo());
    CHECK(f.doc.objects().begin()->second == committed);
    CHECK(f.doc.fonts().size() == 1);
}

TEST_CASE("typing keys: backspace removes whole code points, escape drops the draft", "[text][tool]") {
    Fixture f;
    f.click({10.0, 10.0});
    auto& ed = f.tool.textEditor();
    ed.insertText("I\xC3\x89");  // I, E-acute (2 bytes)
    CHECK(ed.draftParams()->text == "I\xC3\x89");
    f.tool.key(ToolKey::Backspace);
    CHECK(ed.draftParams()->text == "I");
    f.tool.key(ToolKey::Backspace);
    f.tool.key(ToolKey::Backspace);  // already empty: harmless
    CHECK(ed.draftParams()->text.empty());
    ed.insertText("a\r\nb\rc\x01");
    CHECK(ed.draftParams()->text == "a\nb\nc");
    const std::uint64_t gen = f.doc.undoStack().generation();
    f.tool.key(ToolKey::Escape);
    CHECK_FALSE(ed.hasDraft());
    CHECK(f.doc.objectCount() == 0);
    CHECK(f.doc.undoStack().generation() == gen);
    // Empty text is not placed.
    f.click({10.0, 10.0});
    const ToolResult r = f.tool.key(ToolKey::Enter);
    CHECK_FALSE(r.committed);
    CHECK(f.doc.objectCount() == 0);
    CHECK_FALSE(ed.hasDraft());
}

TEST_CASE("the compiled-in starter font is the bundled OFL font and loads", "[text][tool][font]") {
    const auto bytes = starterFontBytes();
    REQUIRE_FALSE(bytes.empty());
    CHECK(std::vector<std::uint8_t>(bytes.begin(), bytes.end()) == fontBytes());
    Document doc;
    ToolController tool(&doc);
    REQUIRE(tool.textEditor().setFont(std::vector<std::uint8_t>(bytes.begin(), bytes.end()), starterFontName()).ok);
    CHECK(tool.textEditor().fontName() == "Carvetto Starter");
    CHECK(tool.textEditor().fontKey() == fontKeyFor(fontBytes()));
}

TEST_CASE("text needs a font, a writable layer and sane input", "[text][tool]") {
    Document doc;
    ViewTransform view;
    view.setViewport(1000.0, 1000.0);
    view.set(2.0, {0.0, 900.0});
    ToolController tool(&doc);
    tool.setView(view);
    tool.setTool(ToolKind::Text);
    CHECK_FALSE(tool.pointerPress(view.worldToScreen({5, 5})).message.empty());  // no font yet
    CHECK_FALSE(tool.textEditor().hasDraft());
    auto& ed = tool.textEditor();
    CHECK_FALSE(ed.setFont({}).ok);
    CHECK_FALSE(ed.setFont({1, 2, 3, 4, 5, 6, 7, 8}).ok);  // not a font
    CHECK_FALSE(ed.setFont(std::vector<std::uint8_t>(kMaxEmbeddedFontBytes + 1, 0)).ok);
    CHECK_FALSE(ed.setFontFile("/nonexistent/font.ttf").ok);
    CHECK_FALSE(ed.hasFont());
    REQUIRE(ed.setFont(fontBytes()).ok);
    CHECK_FALSE(ed.setSize(0.0).ok);
    CHECK_FALSE(ed.setSize(std::nan("")).ok);
    CHECK_FALSE(ed.setLineSpacing(-1.0).ok);
    CHECK_FALSE(ed.setLetterSpacing(1e9).ok);
    CHECK(ed.settings().sizeMm == 10.0);

    doc.findLayer(doc.activeLayer())->locked = true;
    tool.pointerPress(view.worldToScreen({5, 5}));
    REQUIRE(ed.hasDraft());
    ed.insertText("I");
    const ToolResult r = tool.key(ToolKey::Enter);
    CHECK_FALSE(r.committed);
    CHECK_FALSE(r.message.empty());
    CHECK(doc.objectCount() == 0);
    CHECK(ed.hasDraft());  // kept so the user can unlock the layer and retry
    doc.findLayer(doc.activeLayer())->locked = false;
    CHECK(tool.key(ToolKey::Enter).committed);

    CHECK_FALSE(ed.insertText("x").ok);  // no draft
    // Hostile length: refused, draft intact.
    tool.pointerPress(view.worldToScreen({50, 50}));
    REQUIRE(ed.hasDraft());
    CHECK_FALSE(ed.insertText(std::string(kMaxTextObjectBytes + 1, 'I')).ok);
    CHECK(ed.draftParams()->text.empty());
}

TEST_CASE("text in a box: drag, wrap, commit", "[text][tool][box]") {
    Fixture f;
    f.tool.textEditor().setMode(TextMode::Box);
    // A click without a drag is refused.
    CHECK_FALSE(f.click({10.0, 10.0}).message.empty());
    CHECK_FALSE(f.tool.textEditor().hasDraft());
    // Drag the box from its top-left (100, 200) to (112, 160): width 12, height 40.
    f.tool.pointerMove(f.px({100, 200}));
    f.tool.pointerPress(f.px({100, 200}));
    f.tool.pointerMove(f.px({106, 180}));
    CHECK(f.tool.preview().marqueePx.has_value());
    f.tool.pointerMove(f.px({112, 160}));
    f.tool.pointerRelease(f.px({112, 160}));
    auto& ed = f.tool.textEditor();
    REQUIRE(ed.hasDraft());
    REQUIRE(ed.draftParams()->mode == TextMode::Box);
    CHECK(ed.draftParams()->boxWidthMm == Approx(12.0));
    CHECK(ed.draftParams()->boxHeightMm == Approx(40.0));
    REQUIRE(ed.boxMm().has_value());
    CHECK(ed.boxMm()->min.x == Approx(100.0));
    CHECK(ed.boxMm()->max.y == Approx(200.0));
    ed.insertText("I I I I");
    CHECK_FALSE(ed.previewOverflow());
    const geometry::Box2 b = boundsOf(ed.previewContours());
    CHECK(b.max.y == Approx(199.0));   // bar top: 8 mm ascender below the top, 7 mm up
    CHECK(b.min.y == Approx(181.0));   // second line 11 mm lower
    CHECK(b.min.x == Approx(101.0));
    ed.setAlign(TextAlign::Right);
    CHECK(boundsOf(ed.previewContours()).max.x == Approx(111.0));
    CHECK(f.tool.key(ToolKey::Enter).committed);
    const auto* tp = std::get_if<TextParams>(&f.onlyVector()->params);
    REQUIRE(tp != nullptr);
    CHECK(tp->align == TextAlign::Right);
    CHECK(tp->mode == TextMode::Box);
    CHECK(f.doc.undoStack().canUndo());
    f.doc.undo();
    CHECK(f.doc.objectCount() == 0);
}

TEST_CASE("text on a curve: click a curve, run the text along it", "[text][tool][curve]") {
    Fixture f;
    geometry::Contour line;
    line.closed = false;
    line.spans.push_back(geometry::LineSpan{{100.0, 300.0}, {200.0, 300.0}});
    const ObjectId guide =
        f.doc.insertObject(Object::vector(line, f.doc.activeLayer(), f.doc.activeSheet()));
    auto& ed = f.tool.textEditor();
    ed.setMode(TextMode::Curve);
    // Clicking away from any curve is refused.
    CHECK_FALSE(f.click({500.0, 500.0}).message.empty());
    CHECK_FALSE(ed.hasDraft());
    f.click({150.0, 300.0});
    REQUIRE(ed.hasDraft());
    CHECK(ed.draftParams()->mode == TextMode::Curve);
    CHECK(ed.draftParams()->guideId == guide);
    ed.insertText("I");
    ed.setAlign(TextAlign::Centre);
    ed.setStartOffset(50.0);
    // The advance midpoint (2 mm) of the glyph is at arc length 50: bar from x = 149 to 151.
    geometry::Box2 b = boundsOf(ed.previewContours());
    CHECK(b.min.x == Approx(149.0));
    CHECK(b.max.x == Approx(151.0));
    CHECK(b.min.y == Approx(300.0));
    CHECK(b.max.y == Approx(307.0));
    // Reversing the guide sends the glyph the other way up the line: upside down below it.
    CHECK(ed.reverseGuide().ok);
    b = boundsOf(ed.previewContours());
    CHECK(b.max.y == Approx(300.0));
    CHECK(b.min.y == Approx(293.0));
    CHECK(ed.reverseGuide().ok);
    CHECK(f.tool.key(ToolKey::Enter).committed);
    REQUIRE(f.doc.objectCount() == 2);
    const auto* tp = std::get_if<TextParams>(&f.onlyVector()->params);
    REQUIRE(tp != nullptr);
    CHECK(tp->mode == TextMode::Curve);
    CHECK(tp->guide.spans.size() == 1);
    CHECK(f.doc.checkIntegrity().empty());
    // The guide stays where it is.
    CHECK(std::holds_alternative<NoParams>(f.doc.findObject(guide)->asVector()->params));
}

TEST_CASE("editing text in place is one undo step and undo restores it exactly", "[text][tool][undo]") {
    Fixture f;
    f.click({100.0, 100.0});
    f.tool.textEditor().insertText("TOI");
    f.tool.key(ToolKey::Enter);
    const Object original = f.doc.objects().begin()->second;
    const ObjectId id = original.id;
    const std::uint64_t gen = f.doc.undoStack().generation();

    // Click on the stem of the T (x 2.5..3.5 mm, y 0..7 mm from the anchor).
    f.doc.selection().clear();
    f.click({103.0, 103.0});
    auto& ed = f.tool.textEditor();
    REQUIRE(ed.hasDraft());
    CHECK(ed.editedObject() == id);
    CHECK(f.doc.undoStack().generation() == gen);  // opening changes nothing
    ed.setText("O");
    ed.setSize(20.0);
    CHECK(f.doc.undoStack().generation() == gen);  // still a draft
    CHECK(f.tool.key(ToolKey::Enter).committed);
    REQUIRE(f.doc.objectCount() == 1);  // edited, not duplicated
    const Object edited = f.doc.objects().begin()->second;
    CHECK(area(edited.asVector()->contours) == Approx(110.0).epsilon(1e-3));
    CHECK(std::get<TextParams>(edited.asVector()->params).sizeMm == 20.0);

    REQUIRE(f.doc.undo());
    CHECK(f.doc.objects().begin()->second == original);
    REQUIRE(f.doc.redo());
    CHECK(f.doc.objects().begin()->second == edited);

    // Opening and committing without a change records nothing.
    ed.beginEdit(id);
    const std::uint64_t g2 = f.doc.undoStack().generation();
    const ToolResult same = f.tool.key(ToolKey::Enter);
    CHECK_FALSE(same.committed);
    CHECK(f.doc.undoStack().generation() == g2);
}

TEST_CASE("changing the font of the draft embeds the new font on commit", "[text][tool]") {
    Fixture f;
    f.click({10.0, 10.0});
    auto& ed = f.tool.textEditor();
    ed.insertText("I");
    // The CFF test font is a different file; I is not in it, but A is.
    std::ifstream in(std::string(CARVETTO_TEXT_FIXTURE_DIR) + "/CarvettoTestCFF-Regular.otf", std::ios::binary);
    std::vector<std::uint8_t> otf((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    REQUIRE_FALSE(otf.empty());
    const std::string otfKey = fontKeyFor(otf);
    REQUIRE(ed.setFont(otf, "cff").ok);
    CHECK(ed.draftParams()->fontKey == otfKey);
    ed.setText("A");
    CHECK(area(ed.previewContours()) == Approx(22.0).epsilon(1e-6));  // 220000 font units^2 * 1e-4
    CHECK(f.tool.key(ToolKey::Enter).committed);
    CHECK(f.doc.findFont(otfKey) != nullptr);
    CHECK(f.doc.fonts().size() == 1);
    CHECK(f.doc.checkIntegrity().empty());
    f.doc.undo();
    CHECK(f.doc.fonts().empty());
}

TEST_CASE("convert to curves: toolpath-ready geometry, one undo step", "[text][tool][undo]") {
    Fixture f;
    f.click({100.0, 100.0});
    f.tool.textEditor().insertText("TOI");
    f.tool.key(ToolKey::Enter);
    const std::vector<geometry::Contour> contours = f.onlyVector()->contours;
    {
        Fixture g;
        CHECK_FALSE(g.tool.textEditor().convertSelectionToCurves().message.empty());
    }

    auto& ed = f.tool.textEditor();
    CHECK(ed.convertSelectionToCurves().committed);
    CHECK(std::holds_alternative<NoParams>(f.onlyVector()->params));
    CHECK(f.onlyVector()->contours == contours);
    // Selected vectors feed toolpaths: the object is a plain vector now.
    CHECK(f.doc.selectedVectors(VectorSelection::allVectors()).size() == 1);
    REQUIRE(f.doc.undo());
    CHECK(std::holds_alternative<TextParams>(f.onlyVector()->params));
    CHECK(ed.convertSelectionToCurves().committed);  // the selection is still the text
}

TEST_CASE("document replacement in the middle of a gesture", "[text][tool][lifetime]") {
    auto doc = std::make_unique<Document>(Job::singleSided(1000.0, 1000.0, 18.0, Units::Millimetres));
    ViewTransform view;
    view.setViewport(1000.0, 1000.0);
    view.set(2.0, {100.0, 900.0});
    ToolController tool(doc.get());
    tool.setView(view);
    tool.setTool(ToolKind::Text);
    auto& ed = tool.textEditor();
    REQUIRE(ed.setFont(fontBytes()).ok);

    SECTION("a draft is dropped when the document is replaced") {
        tool.pointerPress(view.worldToScreen({10, 10}));
        ed.insertText("TOI");
        REQUIRE(ed.hasDraft());
        Document other(Job::singleSided(500.0, 500.0, 18.0, Units::Millimetres));
        doc.reset();  // the old document is gone before the tool is rebound
        tool.setDocument(&other);
        CHECK_FALSE(ed.hasDraft());
        CHECK(ed.previewContours().empty());
        CHECK_FALSE(tool.active());
        CHECK(other.objectCount() == 0);
        CHECK(other.undoStack().canUndo() == false);
        // The new document works, and the font survived.
        tool.pointerPress(view.worldToScreen({20, 20}));
        ed.insertText("I");
        CHECK(tool.key(ToolKey::Enter).committed);
        CHECK(other.objectCount() == 1);
        CHECK(other.fonts().size() == 1);
    }
    SECTION("a box drag in progress is dropped") {
        ed.setMode(TextMode::Box);
        tool.pointerPress(view.worldToScreen({10, 100}));
        tool.pointerMove(view.worldToScreen({50, 50}));
        REQUIRE(ed.pressed());
        Document other;
        doc.reset();
        tool.setDocument(&other);
        CHECK_FALSE(ed.pressed());
        const ToolResult r = tool.pointerRelease(view.worldToScreen({50, 50}));
        CHECK(r.message.empty());
        CHECK_FALSE(ed.hasDraft());
        CHECK(other.objectCount() == 0);
    }
    SECTION("an edit in place on a replaced document is dropped") {
        tool.pointerPress(view.worldToScreen({100, 100}));
        ed.insertText("TOI");
        tool.key(ToolKey::Enter);
        tool.pointerPress(view.worldToScreen({103, 103}));
        REQUIRE_FALSE(ed.editedObject().isNull());
        ed.setText("O");
        Document other;
        doc.reset();
        tool.setDocument(&other);
        CHECK_FALSE(ed.hasDraft());
        CHECK_FALSE(tool.key(ToolKey::Enter).committed);
        CHECK(other.objectCount() == 0);
    }
    SECTION("the document is replaced with no document") {
        tool.pointerPress(view.worldToScreen({10, 10}));
        ed.insertText("I");
        doc.reset();
        tool.setDocument(nullptr);
        CHECK_FALSE(ed.hasDraft());
        CHECK_FALSE(tool.pointerPress(view.worldToScreen({10, 10})).committed);
        CHECK_FALSE(ed.insertText("I").ok);
    }
}

TEST_CASE("a text made with the tool survives save and load and lays out the same", "[text][tool][carv]") {
    Fixture f;
    f.click({100.0, 100.0});
    f.tool.textEditor().insertText("TO\nI");
    f.tool.textEditor().setLetterSpacing(0.5);
    f.tool.key(ToolKey::Enter);
    auto saved = saveCarv(f.doc);
    REQUIRE(saved.ok());
    auto loaded = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().document.objects().begin()->second == f.doc.objects().begin()->second);
    auto again = saveCarv(loaded.value().document, loaded.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());

    // Editing the loaded document works: its font came from the file, not from the editor.
    ToolController tool(&loaded.value().document);
    ViewTransform view = f.view;
    tool.setView(view);
    tool.setTool(ToolKind::Text);
    loaded.value().document.selection().assign({loaded.value().document.objects().begin()->first});
    REQUIRE(tool.textEditor().beginEditSelected().ok);
    CHECK(tool.textEditor().hasFont());
    CHECK(tool.textEditor().previewContours() == f.onlyVector()->contours);
}

TEST_CASE("a full font table refuses the commit with a message and changes nothing", "[text][tool][caps]") {
    SECTION("font count cap") {
        Fixture f;
        for (std::size_t i = 0; i < kMaxEmbeddedFonts; ++i) {
            char key[17];
            std::snprintf(key, sizeof key, "%016zx", i + 1);
            REQUIRE(f.doc.insertFont(FontAsset{key, {1}}));
        }
        f.click({10.0, 10.0});
        f.tool.textEditor().insertText("TOI");
        const std::size_t steps = f.doc.undoStack().count();
        const ToolResult r = f.tool.key(ToolKey::Enter);
        CHECK_FALSE(r.committed);
        CHECK(r.message.find("64") != std::string::npos);
        CHECK(f.doc.objectCount() == 0);
        CHECK(f.doc.undoStack().count() == steps);
        CHECK(f.doc.checkIntegrity().empty());
        CHECK(f.tool.textEditor().hasDraft());  // kept
    }
    SECTION("total size cap") {
        Fixture f;
        const std::size_t chunk = kMaxEmbeddedFontBytes;
        for (std::size_t i = 0; i < 4; ++i) {
            char key[17];
            std::snprintf(key, sizeof key, "%016zx", i + 1);
            REQUIRE(f.doc.insertFont(FontAsset{key, std::vector<std::uint8_t>(chunk, 1)}));
        }
        f.click({10.0, 10.0});
        f.tool.textEditor().insertText("I");
        const ToolResult r = f.tool.key(ToolKey::Enter);
        CHECK_FALSE(r.committed);
        CHECK(r.message.find("MiB") != std::string::npos);
        CHECK(f.doc.objectCount() == 0);
    }
    SECTION("an already embedded font is fine at the cap") {
        Fixture f;
        f.click({10.0, 10.0});
        f.tool.textEditor().insertText("I");
        REQUIRE(f.tool.key(ToolKey::Enter).committed);
        for (std::size_t i = 1; i < kMaxEmbeddedFonts; ++i) {
            char key[17];
            std::snprintf(key, sizeof key, "%016zx", i + 1);
            REQUIRE(f.doc.insertFont(FontAsset{key, {1}}));
        }
        f.click({50.0, 50.0});
        f.tool.textEditor().insertText("T");
        CHECK(f.tool.key(ToolKey::Enter).committed);
        CHECK(f.doc.objectCount() == 2);
    }
}

TEST_CASE("characters outside the BMP are typed and deleted whole", "[text][tool]") {
    Fixture f;
    f.click({10.0, 10.0});
    auto& ed = f.tool.textEditor();
    ed.insertText("I\xF0\x9F\x98\x80\xF0\xA0\x80\x80");  // emoji, CJK Ext-B
    CHECK(ed.draftParams()->text.size() == 9);
    CHECK(ed.previewMessage().empty());
    f.tool.key(ToolKey::Backspace);
    CHECK(ed.draftParams()->text == "I\xF0\x9F\x98\x80");
    f.tool.key(ToolKey::Backspace);
    CHECK(ed.draftParams()->text == "I");
}
