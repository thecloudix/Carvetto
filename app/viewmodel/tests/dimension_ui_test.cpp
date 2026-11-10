// Dimensions wired into the tool controller (T-0255): placing, shift-select editing with control
// points, reflex angular placement, options edits, delete, text outlines in the scene and DXF.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/dimension.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/dimension_text.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

using namespace carvetto;
using carvetto::geometry::Vec2;
using carvetto::viewmodel::DimensionOptions;
using carvetto::viewmodel::Modifiers;
using carvetto::viewmodel::ToolKind;
using carvetto::viewmodel::ToolResult;
using Catch::Approx;

namespace {

struct Fixture {
    document::Document doc{document::Job::singleSided(1000.0, 1000.0, 18.0, document::Units::Millimetres)};
    viewmodel::ViewTransform view;
    viewmodel::ToolController tool;
    Fixture() {
        view.setViewport(1000.0, 1000.0);
        view.set(2.0, {100.0, 900.0});  // 2 px per mm
        tool.setDocument(&doc);
        tool.setView(view);
        tool.setTool(ToolKind::Dimension);
    }
    Vec2 px(Vec2 mm) const { return view.worldToScreen(mm); }
    ToolResult click(Vec2 mm, Modifiers m = {}) {
        tool.pointerMove(px(mm), m);
        ToolResult p = tool.pointerPress(px(mm), m);
        const ToolResult r = tool.pointerRelease(px(mm), m);
        p.committed = p.committed || r.committed;
        if (p.message.empty()) {
            p.message = r.message;
        }
        return p;
    }
    ToolResult drag(Vec2 a, Vec2 b) {
        tool.pointerMove(px(a));
        tool.pointerPress(px(a));
        tool.pointerMove(px((a + b) * 0.5));
        tool.pointerMove(px(b));
        return tool.pointerRelease(px(b));
    }
    document::ObjectId place(document::DimensionKind kind, std::vector<Vec2> pts) {
        DimensionOptions o;
        o.kind = kind;
        REQUIRE(tool.dimensionTool().setOptions(o).ok);
        ToolResult r;
        for (const Vec2 p : pts) {
            r = click(p);
        }
        REQUIRE(r.committed);
        return doc.selection().single();
    }
    const document::DimensionObject& dim(document::ObjectId id) const { return *doc.findObject(id)->asDimension(); }
};

}  // namespace

TEST_CASE("the dimension tool is a ToolController tool: place, preview, one undo step", "[dimension][tools]") {
    Fixture f;
    CHECK(std::string(viewmodel::toolName(ToolKind::Dimension)) == "Dimension");
    DimensionOptions o;
    o.kind = document::DimensionKind::Aligned;
    REQUIRE(f.tool.dimensionTool().setOptions(o).ok);
    const std::size_t steps = f.doc.undoStack().count();
    CHECK_FALSE(f.tool.active());
    f.click({0, 0});
    CHECK(f.tool.active());
    CHECK(f.tool.wantsKeys());
    f.click({40, 0});
    f.tool.pointerMove(f.px({20, 10}));
    const viewmodel::ToolPreview pv = f.tool.preview();
    CHECK(pv.valid);
    CHECK(pv.contours.size() > 5);  // lines, arrowheads and the text outlines
    const ToolResult r = f.click({20, 10});
    CHECK(r.committed);
    CHECK_FALSE(f.tool.active());
    CHECK(f.doc.undoStack().count() == steps + 1);  // layer + object are one step
    const document::ObjectId id = f.doc.selection().single();
    REQUIRE_FALSE(id.isNull());
    CHECK(f.dim(id).offsetMm == Approx(10.0));
    CHECK(f.dim(id).points[1].pos.x == Approx(40.0));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findObject(id) == nullptr);
    REQUIRE(f.doc.redo());
    CHECK(f.doc.findObject(id) != nullptr);
}

TEST_CASE("Escape, Backspace and switching tools drop a dimension draft", "[dimension][tools]") {
    Fixture f;
    f.click({0, 0});
    f.click({40, 0});
    CHECK(f.tool.dimensionTool().pickedCount() == 2);
    f.tool.key(viewmodel::ToolKey::Backspace);
    CHECK(f.tool.dimensionTool().pickedCount() == 1);
    f.tool.key(viewmodel::ToolKey::Escape);
    CHECK(f.tool.dimensionTool().pickedCount() == 0);
    CHECK(f.doc.objects().empty());
    f.click({0, 0});
    f.tool.setTool(ToolKind::Select);
    CHECK(f.tool.dimensionTool().pickedCount() == 0);
    f.tool.setTool(ToolKind::Dimension);
    CHECK(f.tool.commitNumeric().message.size() > 0);  // Enter has nothing to commit
    // A degenerate pick (two identical points) is refused at the placement click, nothing is added.
    f.click({5, 5});
    f.click({5, 5});
    const ToolResult bad = f.click({5, 9});
    CHECK_FALSE(bad.committed);
    CHECK_FALSE(bad.message.empty());
    CHECK(f.doc.objects().empty());
}

TEST_CASE("Shift-click selects a dimension and its handles edit it in one undo step", "[dimension][tools][handles]") {
    Fixture f;
    const auto id = f.place(document::DimensionKind::Aligned, {{0, 0}, {40, 0}, {20, 10}});
    f.tool.setTool(ToolKind::Select);
    f.tool.setTool(ToolKind::Dimension);
    f.doc.selection().clear();
    CHECK_FALSE(f.tool.dimensionTool().editTarget().has_value());
    CHECK(f.tool.dimensionTool().handles().empty());

    Modifiers shift;
    shift.shift = true;
    const ToolResult sel = f.click({20, 10}, shift);  // on the dimension line
    CHECK(sel.selectionChanged);
    REQUIRE(f.tool.dimensionTool().editTarget().has_value());
    CHECK(*f.tool.dimensionTool().editTarget() == id);
    CHECK(f.doc.selection().single() == id);
    const auto handles = f.tool.dimensionTool().handles();
    CHECK(handles.size() == 5);
    CHECK(std::count_if(handles.begin(), handles.end(), [](const document::DimensionHandle& h) { return h.white; }) == 3);

    // Drag the leg handle (end of the extension lines) up: the dimension line moves, the size stays.
    const std::size_t steps = f.doc.undoStack().count();
    f.drag({40, 10}, {40, 25});
    CHECK(f.dim(id).offsetMm == Approx(25.0));
    CHECK(f.dim(id).points[1].pos.x == Approx(40.0));
    CHECK(f.doc.undoStack().count() == steps + 1);

    // Drag a size handle: the measured text follows.
    f.drag({40, 0}, {60, 0});
    CHECK(document::measureDimension(f.doc, f.dim(id)).value() == Approx(60.0));
    CHECK(f.doc.undoStack().count() == steps + 2);
    REQUIRE(f.doc.undo());
    CHECK(document::measureDimension(f.doc, f.dim(id)).value() == Approx(40.0));
    REQUIRE(f.doc.undo());
    CHECK(f.dim(id).offsetMm == Approx(10.0));

    // A click on a handle without moving is no edit; Escape during a drag reverts it.
    const std::size_t before = f.doc.undoStack().count();
    f.click({40, 0});
    CHECK(f.doc.undoStack().count() == before);
    f.tool.pointerMove(f.px({40, 10}));
    f.tool.pointerPress(f.px({40, 10}));
    f.tool.pointerMove(f.px({40, 30}));
    CHECK(f.tool.dimensionTool().dragging());
    CHECK(f.tool.preview().valid);
    f.tool.key(viewmodel::ToolKey::Escape);
    CHECK_FALSE(f.tool.dimensionTool().dragging());
    f.tool.pointerRelease(f.px({40, 30}));
    CHECK(f.doc.undoStack().count() == before);
    CHECK(f.dim(id).offsetMm == Approx(10.0));

    // A second Escape drops the edit target; a miss with Shift too.
    f.tool.key(viewmodel::ToolKey::Escape);
    CHECK_FALSE(f.tool.dimensionTool().editTarget().has_value());
    f.click({20, 10}, shift);
    CHECK(f.tool.dimensionTool().editTarget().has_value());
    f.click({300, 300}, shift);
    CHECK_FALSE(f.tool.dimensionTool().editTarget().has_value());
}

TEST_CASE("dragging a point handle onto a part vertex anchors it", "[dimension][tools][handles]") {
    Fixture f;
    geometry::Box2 b;
    b.expand(Vec2{100, 100});
    b.expand(Vec2{140, 130});
    const auto rect = f.doc.insertObject(
        document::Object::vector(geometry::Contour::rectangle(b), f.doc.activeLayer(), f.doc.activeSheet()));
    f.tool.snapSettings().setEnabled(viewmodel::SnapKind::Endpoint, false);
    const auto id = f.place(document::DimensionKind::Aligned, {{0, 0}, {30, 0}, {15, 6}});
    REQUIRE_FALSE(f.dim(id).points[1].anchor.has_value());
    Modifiers shift;
    shift.shift = true;
    f.click({15, 6}, shift);
    f.drag({30, 0}, {140.5, 100.4});  // within the pick tolerance of the corner (140, 100)
    REQUIRE(f.dim(id).points[1].anchor.has_value());
    CHECK(f.dim(id).points[1].anchor->object == rect);
    CHECK(f.dim(id).points[1].pos.x == Approx(140.0));
}

TEST_CASE("placing an angular dimension outside the angle measures the reflex angle", "[dimension][tools][reflex]") {
    Fixture f;
    const auto inner = f.place(document::DimensionKind::Angular, {{0, 0}, {30, 0}, {0, 30}, {10, 10}});
    CHECK(document::measureDimension(f.doc, f.dim(inner)).value() == Approx(geometry::kPi / 2));
    const auto outer = f.place(document::DimensionKind::Angular, {{0, 0}, {30, 0}, {0, 30}, {-10, -10}});
    CHECK(f.dim(outer).offsetMm < 0.0);
    CHECK(document::measureDimension(f.doc, f.dim(outer)).value() == Approx(3 * geometry::kPi / 2));
    CHECK(document::dimensionText(f.doc, f.dim(outer), document::Units::Millimetres) == "270.00\xC2\xB0");
    // Flip it with the options (the form's "reflex" box) and size the arc with the offset field.
    auto o = f.tool.dimensionTool().optionsOf(outer);
    REQUIRE(o);
    CHECK(o->reflex.value_or(false));
    CHECK(*o->offsetMm == Approx(std::sqrt(200.0)));
    o->reflex = false;
    o->offsetMm = 12.0;
    REQUIRE(f.tool.dimensionTool().edit(outer, *o).committed);
    CHECK(f.dim(outer).offsetMm == Approx(12.0));
    CHECK(document::measureDimension(f.doc, f.dim(outer)).value() == Approx(geometry::kPi / 2));
}

TEST_CASE("dimension option edits refuse hostile input and are one undo step each", "[dimension][tools]") {
    Fixture f;
    const auto id = f.place(document::DimensionKind::Linear, {{0, 0}, {30, 5}, {10, 20}});
    auto o = f.tool.dimensionTool().optionsOf(id);
    REQUIRE(o);
    const std::size_t steps = f.doc.undoStack().count();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double bad : {inf, -inf, std::nan(""), 2e9}) {
        auto b = *o;
        b.offsetMm = bad;
        CHECK_FALSE(f.tool.dimensionTool().edit(id, b).ok);
        b = *o;
        b.textHeightMm = bad;
        CHECK_FALSE(f.tool.dimensionTool().edit(id, b).ok);
    }
    auto b = *o;
    b.decimals = 9;
    CHECK_FALSE(f.tool.dimensionTool().edit(id, b).ok);
    b = *o;
    b.angleDecimals = 7;
    CHECK_FALSE(f.tool.dimensionTool().edit(id, b).ok);
    b = *o;
    b.customText = std::string(5000, 'x');
    CHECK_FALSE(f.tool.dimensionTool().edit(id, b).ok);
    CHECK(f.doc.undoStack().count() == steps);

    o->decimals = 1;
    o->offsetMm = -7.5;
    o->useCustomText = true;
    o->customText = "30 +/- 0.1";
    REQUIRE(f.tool.dimensionTool().edit(id, *o).committed);
    CHECK(f.doc.undoStack().count() == steps + 1);
    CHECK(f.dim(id).offsetMm == Approx(-7.5));
    // Delete: one undo step, restores everything.
    const auto del = f.tool.dimensionTool().removeSelected();
    REQUIRE(del.committed);
    CHECK(f.doc.findObject(id) == nullptr);
    CHECK_FALSE(f.tool.dimensionTool().editTarget().has_value());
    CHECK_FALSE(f.tool.dimensionTool().removeSelected().ok);  // nothing selected any more
    REQUIRE(f.doc.undo());
    CHECK(f.dim(id).customText == "30 +/- 0.1");
}

TEST_CASE("each dimension kind can be placed, edited by handles and deleted", "[dimension][tools][handles]") {
    Fixture f;
    const auto lin = f.place(document::DimensionKind::Linear, {{0, 0}, {30, 20}, {10, 40}});
    const auto ang = f.place(document::DimensionKind::Angular, {{100, 100}, {130, 100}, {100, 130}, {110, 110}});
    const auto rad = f.place(document::DimensionKind::Radial, {{200, 200}, {220, 200}, {240, 200}});
    for (const auto id : {lin, ang, rad}) {
        f.tool.dimensionTool().setEditTarget(id);
        const auto hs = f.tool.dimensionTool().handles();
        REQUIRE_FALSE(hs.empty());
        for (const document::DimensionHandle& h : hs) {
            const document::DimensionObject before = f.dim(id);
            REQUIRE(f.tool.dimensionTool().beginHandle(h.pos, 0.5));
            // Moves a little: either the edit is valid and one undo step, or it is ignored.
            const std::size_t steps = f.doc.undoStack().count();
            f.tool.dimensionTool().dragTo(h.pos + Vec2{1.0, 2.0});
            const auto r = f.tool.dimensionTool().endDrag(h.pos + Vec2{1.0, 2.0});
            CHECK(f.doc.undoStack().count() == steps + (r.committed ? 1u : 0u));
            if (r.committed) {
                CHECK_FALSE(f.dim(id) == before);
                REQUIRE(f.doc.undo());
                CHECK(f.dim(id) == before);
            }
        }
        f.doc.selection().assign({id});
        REQUIRE(f.tool.dimensionTool().removeSelected().committed);
        CHECK(f.doc.findObject(id) == nullptr);
        REQUIRE(f.doc.undo());
        CHECK(f.doc.findObject(id) != nullptr);
    }
}

TEST_CASE("pickAt finds lines and text, skips hidden layers and other sheets' dimensions", "[dimension][tools]") {
    Fixture f;
    const auto id = f.place(document::DimensionKind::Aligned, {{0, 0}, {40, 0}, {20, 10}});
    auto& dt = f.tool.dimensionTool();
    CHECK(dt.pickAt({20, 10.5}, 1.0) == id);                              // dimension line
    CHECK(dt.pickAt({0, 5}, 1.0) == id);                                  // extension line
    CHECK(dt.pickAt({20, 10 + 2.25}, 0.1) == id);                         // the text
    CHECK_FALSE(dt.pickAt({20, 30}, 1.0).has_value());
    CHECK_FALSE(dt.pickAt({std::nan(""), 0}, 1.0).has_value());
    CHECK_FALSE(dt.pickAt({20, 10}, -1.0).has_value());
    document::Layer l = *f.doc.findLayer(f.doc.findObject(id)->layer);
    l.visible = false;
    REQUIRE(f.doc.execute(std::make_unique<document::SetLayerPropertiesCommand>(l)));
    CHECK_FALSE(dt.pickAt({20, 10}, 1.0).has_value());
}

TEST_CASE("the scene carries dimension text as outlines", "[dimension][viewmodel][scene]") {
    document::Document doc;
    document::DimensionObject d;
    d.points[1].pos = {40, 0};
    d.offsetMm = 8.0;
    d.textHeightMm = 4.0;
    REQUIRE(document::addDimension(doc, d, doc.activeSheet()));
    viewmodel::ViewTransform view;
    view.setViewport(400, 300);
    view.set(5.0, {100.0, 200.0});
    const viewmodel::CanvasScene scene = viewmodel::buildCanvasScene(doc, view);
    REQUIRE(scene.labels.size() == 1);
    const viewmodel::DrawLabel& label = scene.labels[0];
    CHECK(label.text == "40.00");
    CHECK(label.outlinesPx.size() >= 5);  // at least one contour per glyph
    CHECK(label.heightPx == Approx(20.0));
    // The outlines are centred on the label position and about one text height tall.
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& poly : label.outlinesPx) {
        for (const Vec2 p : poly) {
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }
    }
    CHECK((minX + maxX) / 2 == Approx(label.centrePx.x).margin(0.5));
    CHECK((minY + maxY) / 2 == Approx(label.centrePx.y).margin(0.5));
    CHECK(maxY - minY == Approx(label.heightPx).margin(label.heightPx * 0.15));
    CHECK(maxX - minX > maxY - minY);  // five characters are wider than tall

    // Custom text replaces the measurement; hidden layers and off-screen dimensions have no label.
    document::DimensionObject e = d;
    e.useCustomText = true;
    e.customText = "TYP";
    d = e;
    const auto id = doc.objects().rbegin()->first;
    REQUIRE(doc.execute(std::make_unique<document::SetDimensionCommand>(id, e)));
    CHECK(viewmodel::buildCanvasScene(doc, view).labels[0].text == "TYP");
    view.set(5.0, {-5000.0, 200.0});
    CHECK(viewmodel::buildCanvasScene(doc, view).labels.empty());
}

TEST_CASE("an unknown or empty font key falls back to the starter font; empty text has no outlines",
          "[dimension][viewmodel][scene]") {
    document::Document doc;
    document::DimensionObject d;
    d.points[1].pos = {40, 0};
    d.fontKey = "0123456789abcdef";  // not embedded
    const auto fallback = viewmodel::dimensionTextContours(doc, d);
    d.fontKey.clear();
    CHECK_FALSE(fallback.empty());
    CHECK(fallback.size() == viewmodel::dimensionTextContours(doc, d).size());
    d.useCustomText = true;
    d.customText.clear();
    CHECK(viewmodel::dimensionTextContours(doc, d).empty());
    d.useCustomText = false;
    d.points[1].pos = d.points[0].pos;  // degenerate
    CHECK(viewmodel::dimensionTextContours(doc, d).empty());
    CHECK_FALSE(viewmodel::dimensionFontName(doc, "").empty());
}

TEST_CASE("DXF export of a dimension writes its text outlines", "[dimension][viewmodel][dxf]") {
    document::Document doc;
    document::DimensionObject d;
    d.points[1].pos = {10, 0};
    d.offsetMm = 5.0;
    REQUIRE(document::addDimension(doc, d, doc.activeSheet()));
    const auto text = viewmodel::dimensionTextContours(doc, *doc.objects().rbegin()->second.asDimension());
    REQUIRE_FALSE(text.empty());
    const auto out = viewmodel::exportDocumentDxf(doc, false);
    REQUIRE(out.ok);
    CHECK(out.contoursWritten == 5 + text.size());
}
