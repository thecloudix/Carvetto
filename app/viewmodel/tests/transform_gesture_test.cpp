#include "carvetto/viewmodel/transform_gesture.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/transform_ops.hpp"

using namespace carvetto;
using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Transform2;
using geometry::Vec2;
using namespace carvetto::viewmodel;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool nearPt(Vec2 a, Vec2 b, double tol = 1e-9) { return near(a.x, b.x, tol) && near(a.y, b.y, tol); }

Box2 boundsOf(const Document& doc, ObjectId id) {
    Box2 b;
    for (const Contour& c : doc.findObject(id)->asVector()->contours) {
        b.expand(c.bounds());
    }
    return b;
}

std::vector<std::uint8_t> bytes(const Document& doc) {
    auto r = document::saveCarv(doc);
    REQUIRE(r.ok());
    return r.value();
}

bool sameGeometry(const Document& a, const Document& b, ObjectId id, double tol = 1e-9) {
    const auto& ca = a.findObject(id)->asVector()->contours;
    const auto& cb = b.findObject(id)->asVector()->contours;
    if (ca.size() != cb.size()) {
        return false;
    }
    for (std::size_t i = 0; i < ca.size(); ++i) {
        if (ca[i].spanCount() != cb[i].spanCount()) {
            return false;
        }
        for (int k = 0; k <= 16; ++k) {
            if (!nearPt(ca[i].pointAt(k / 16.0), cb[i].pointAt(k / 16.0), tol)) {
                return false;
            }
        }
    }
    return true;
}

struct World {
    Document doc{document::Job::singleSided(1000.0, 1000.0, 18.0, document::Units::Millimetres)};
    ViewTransform view;
    ToolController tool;
    ObjectId rect;
    World() {
        view.setViewport(1000.0, 1000.0);
        view.set(2.0, {100.0, 900.0});  // 2 px per mm; px = (100 + 2x, 900 - 2y)
        rect = doc.insertObject(Object::vector(Contour::rectangle(Box2{{100, 100}, {200, 150}}), doc.activeLayer(),
                                               doc.activeSheet()));
        doc.selection().assign({rect});
        tool.setDocument(&doc);
        tool.setView(view);
    }
    Vec2 px(Vec2 mm) const { return view.worldToScreen(mm); }
    void press(Vec2 mm, Modifiers m = {}) {
        tool.pointerMove(px(mm), m);
        tool.pointerPress(px(mm), m);
    }
    void move(Vec2 mm, Modifiers m = {}) { tool.pointerMove(px(mm), m); }
    ToolResult release(Vec2 mm, Modifiers m = {}) { return tool.pointerRelease(px(mm), m); }
    void click(Vec2 mm, Modifiers m = {}) {
        press(mm, m);
        release(mm, m);
    }
};

}  // namespace

TEST_CASE("corner drag scales proportionally in one undo step, equal to the numeric scale", "[transform][tool]") {
    World w;
    const auto before = bytes(w.doc);
    w.press({200, 150});  // top-right handle; the anchor is the bottom-left corner
    for (double s : {1.1, 1.3, 1.45, 1.7, 1.9, 2.0}) {
        w.move({100 + 100 * s, 100 + 50 * s});
    }
    REQUIRE(w.tool.transforming());
    const ToolResult r = w.release({300, 200});
    CHECK_FALSE(w.tool.transforming());
    CHECK(w.doc.undoStack().count() == 1);
    (void)r;

    World twin;
    ScaleOptions so;
    so.scaleX = so.scaleY = 2.0;
    so.anchor = HandleKind::BottomLeft;
    REQUIRE(scaleSelection(twin.doc, so).ok);
    CHECK(sameGeometry(w.doc, twin.doc, w.rect));

    REQUIRE(w.doc.undo());
    CHECK(bytes(w.doc) == before);
    REQUIRE(w.doc.redo());
    CHECK(sameGeometry(w.doc, twin.doc, w.rect));
}

TEST_CASE("scale modifiers: Alt free, Shift about the centre, edge handles", "[transform][tool]") {
    {  // Alt: non-proportional
        World w;
        Modifiers alt;
        alt.alt = true;
        w.press({200, 150}, alt);
        w.move({250, 130}, alt);
        w.release({300, 200}, alt);
        const Box2 b = boundsOf(w.doc, w.rect);
        CHECK(nearPt(b.min, {100, 100}));
        CHECK(nearPt(b.max, {300, 200}));
    }
    {  // Shift: about the centre (and no snapping)
        World w;
        Modifiers sh;
        sh.shift = true;
        w.press({200, 150}, sh);
        w.move({250, 175}, sh);
        w.release({250, 175}, sh);
        const Box2 b = boundsOf(w.doc, w.rect);
        CHECK(nearPt(b.centre(), {150, 125}));
        CHECK(nearPt(b.max, {250, 175}));
        CHECK(nearPt(b.min, {50, 75}));
    }
    {  // Edge handle: one axis; Shift makes it proportional
        World w;
        w.press({200, 125});
        w.move({300, 125});
        w.release({300, 125});
        const Box2 b = boundsOf(w.doc, w.rect);
        CHECK(nearPt(b.min, {100, 100}));
        CHECK(nearPt(b.max, {300, 150}));
        World v;
        Modifiers sh;
        sh.shift = true;
        v.press({200, 125}, sh);
        v.move({300, 125}, sh);
        v.release({300, 125}, sh);
        CHECK(near(boundsOf(v.doc, v.rect).height(), 100.0));
        CHECK(near(boundsOf(v.doc, v.rect).width(), 200.0));
    }
    {  // Dragging past the anchor holds the last valid size; a plain click changes nothing.
        World w;
        w.press({200, 125});
        w.move({150, 125});
        w.move({50, 125});
        w.release({50, 125});
        CHECK(near(boundsOf(w.doc, w.rect).width(), 50.0));
        World c;
        c.click({200, 150});
        CHECK(c.doc.undoStack().count() == 0);
    }
}

TEST_CASE("drag inside moves; Alt constrains to an axis; the middle handle moves", "[transform][tool]") {
    World w;
    const auto before = bytes(w.doc);
    w.press({120, 110});  // inside the rectangle, on its fill
    w.move({130, 115});
    w.move({160, 150});
    w.move({170, 140});
    w.release({170, 140});
    CHECK(w.doc.undoStack().count() == 1);
    CHECK(nearPt(boundsOf(w.doc, w.rect).min, {150, 130}));
    REQUIRE(w.doc.undo());
    CHECK(bytes(w.doc) == before);

    World c;
    Modifiers alt;
    alt.alt = true;
    c.press({150, 125}, alt);  // the centre handle
    c.move({190, 130}, alt);
    c.release({190, 130}, alt);
    CHECK(nearPt(boundsOf(c.doc, c.rect).min, {140, 100}));  // only X moved

    // Pressing on empty canvas still draws a marquee and selects.
    World m;
    m.press({500, 500});
    m.move({600, 600});
    CHECK_FALSE(m.tool.transforming());
    CHECK(m.tool.preview().marqueePx.has_value());
}

TEST_CASE("click toggles Scale and Rotate; rotate drag with Alt steps; pivot drag", "[transform][tool]") {
    World w;
    CHECK(w.tool.transformMode() == TransformMode::Scale);
    w.click({120, 110});  // click on the selected object
    CHECK(w.tool.transformMode() == TransformMode::Rotate);
    CHECK(w.doc.undoStack().count() == 0);
    CHECK(w.doc.selection().size() == 1);

    // Rotate by 90 degrees about the centre with the top-right corner; Alt snaps to 15 degrees.
    Modifiers alt;
    alt.alt = true;
    const Vec2 centre{150, 125};
    const Vec2 corner{200, 150};
    w.press(corner, alt);
    for (double a : {0.2, 0.7, 1.2, 1.5, kPi / 2.0}) {
        const Vec2 d = Transform2::rotation(a, centre).apply(corner);
        w.move(d, alt);
    }
    w.release(Transform2::rotation(kPi / 2.0, centre).apply(corner), alt);
    CHECK(w.doc.undoStack().count() == 1);
    World twin;
    RotateOptions ro;
    ro.angleRad = kPi / 2.0;
    REQUIRE(rotateSelection(twin.doc, ro).ok);
    CHECK(sameGeometry(w.doc, twin.doc, w.rect, 1e-7));
    REQUIRE(w.doc.undo());

    // 20 degrees without Alt is free; with Alt it steps to 15.
    World free;
    free.click({120, 110});
    free.press(corner);
    free.move(Transform2::rotation(20.0 * kPi / 180.0, centre).apply(corner));
    free.release(Transform2::rotation(20.0 * kPi / 180.0, centre).apply(corner));
    World stepped;
    stepped.click({120, 110});
    stepped.press(corner, alt);
    stepped.move(Transform2::rotation(20.0 * kPi / 180.0, centre).apply(corner), alt);
    stepped.release(Transform2::rotation(20.0 * kPi / 180.0, centre).apply(corner), alt);
    World ref15;
    RotateOptions r15;
    r15.angleRad = 15.0 * kPi / 180.0;
    REQUIRE(rotateSelection(ref15.doc, r15).ok);
    CHECK(sameGeometry(stepped.doc, ref15.doc, stepped.rect, 1e-7));
    CHECK_FALSE(sameGeometry(free.doc, ref15.doc, free.rect, 1e-7));

    // Dragging the centre handle in Rotate mode moves the pivot, not the document.
    World p;
    p.click({120, 110});
    p.press({150, 125});
    p.move({120, 110});
    p.move({100, 100});
    p.release({100, 100});
    CHECK(p.doc.undoStack().count() == 0);
    REQUIRE(p.tool.pivotMm().has_value());
    CHECK(nearPt(*p.tool.pivotMm(), {100, 100}));
    p.press(corner);
    p.move(Transform2::rotation(kPi / 2.0, {100, 100}).apply(corner));
    p.release(Transform2::rotation(kPi / 2.0, {100, 100}).apply(corner));
    World pt;
    RotateOptions pr;
    pr.angleRad = kPi / 2.0;
    pr.pivotMm = Vec2{100, 100};
    REQUIRE(rotateSelection(pt.doc, pr).ok);
    CHECK(sameGeometry(p.doc, pt.doc, p.rect, 1e-7));
    p.tool.setPivot(std::nullopt);
    p.tool.setPivotAnchor(HandleKind::TopRight);
    CHECK(nearPt(*p.tool.pivotMm(), boundsOf(p.doc, p.rect).max));
}

TEST_CASE("Escape reverts an interactive transform; a document swap is safe", "[transform][tool]") {
    World w;
    const auto before = bytes(w.doc);
    w.press({120, 110});
    w.move({150, 140});
    w.move({170, 160});
    REQUIRE(w.tool.transforming());
    CHECK(w.tool.wantsKeys());
    const ToolResult r = w.tool.key(ToolKey::Escape);
    CHECK(r.transformed);
    CHECK_FALSE(w.tool.transforming());
    CHECK(bytes(w.doc) == before);
    CHECK_FALSE(w.doc.undoStack().canRedo());  // the cancelled step is gone, not redoable
    CHECK(w.doc.undoStack().count() == 0);
    w.release({170, 160});
    CHECK(bytes(w.doc) == before);

    // An outside edit during the drag ends the gesture without further changes.
    World e;
    e.press({120, 110});
    e.move({150, 140});
    REQUIRE(e.doc.undo());
    e.move({170, 160});
    CHECK_FALSE(e.tool.transforming());
    e.release({170, 160});
    CHECK(bytes(e.doc) == before);
}

TEST_CASE("transform drags snap the pointer; Shift disables it; moving objects do not self-snap",
          "[transform][tool][snapping]") {
    World w;
    Contour target;
    target.append(geometry::LineSpan{{400, 300}, {450, 300}});
    w.doc.insertObject(Object::vector(target, w.doc.activeLayer(), w.doc.activeSheet()));
    // Drag from a point on the fill by (+252,+173): the release is within 10 px of the endpoint
    // (400,300) of the other object when started at (150,125) -> (402,298).
    w.press({150, 125});
    w.move({300, 200});
    w.move({402, 298});
    w.release({402, 298});
    REQUIRE(w.tool.snap().has_value());
    CHECK(nearPt(boundsOf(w.doc, w.rect).min, {350, 275}));  // centre went exactly to (400,300)
    CHECK(w.doc.undoStack().count() == 1);

    World s;
    s.doc.insertObject(Object::vector(target, s.doc.activeLayer(), s.doc.activeSheet()));
    Modifiers sh;
    sh.shift = true;
    s.press({150, 125});
    s.move({402, 298}, sh);
    s.release({402, 298}, sh);
    CHECK(nearPt(boundsOf(s.doc, s.rect).min, {352, 273}));
}

TEST_CASE("cursor hints and handles follow the mode", "[transform][tool]") {
    World w;
    const auto handles = w.tool.selectionHandles();
    REQUIRE(handles.has_value());
    CHECK(near(handles->boundsMm.width(), 100.0));
    w.move({200, 150});
    CHECK(w.tool.cursorHint() == CursorHint::ScaleDiagonalUp);
    w.move({100, 150});
    CHECK(w.tool.cursorHint() == CursorHint::ScaleDiagonalDown);
    w.move({150, 150});
    CHECK(w.tool.cursorHint() == CursorHint::ScaleVertical);
    w.move({200, 125});
    CHECK(w.tool.cursorHint() == CursorHint::ScaleHorizontal);
    w.move({150, 125});
    CHECK(w.tool.cursorHint() == CursorHint::Move);
    w.move({600, 600});
    CHECK(w.tool.cursorHint() == CursorHint::Default);
    w.tool.setTransformMode(TransformMode::Rotate);
    w.move({200, 150});
    CHECK(w.tool.cursorHint() == CursorHint::Rotate);
    w.move({150, 125});
    CHECK(w.tool.cursorHint() == CursorHint::Pivot);
    w.move({150, 150});
    CHECK(w.tool.cursorHint() == CursorHint::Default);
    w.tool.setTool(ToolKind::Rectangle);
    CHECK_FALSE(w.tool.selectionHandles().has_value());

    // Locked layers show no handles.
    World l;
    l.doc.findLayer(l.doc.activeLayer())->locked = true;
    CHECK_FALSE(l.tool.selectionHandles().has_value());
}

TEST_CASE("Ctrl/Shift clicks still select instead of transforming", "[transform][tool]") {
    World w;
    const ObjectId other = w.doc.insertObject(Object::vector(Contour::rectangle(Box2{{400, 400}, {450, 450}}),
                                                             w.doc.activeLayer(), w.doc.activeSheet()));
    Modifiers sh;
    sh.shift = true;
    w.click({400, 425}, sh);
    CHECK(w.doc.selection().size() == 2);
    CHECK(w.doc.selection().contains(other));
    CHECK(w.doc.undoStack().count() == 0);
    // Plain click on empty space clears.
    w.click({800, 800});
    CHECK(w.doc.selection().empty());
}

TEST_CASE("drags survive undo, redo, deletion and a document swap (sanitizer target)", "[transform][tool]") {
    World w;
    Contour other;
    other.append(geometry::LineSpan{{400, 300}, {450, 300}});
    w.doc.insertObject(Object::vector(other, w.doc.activeLayer(), w.doc.activeSheet()));
    w.press({150, 125});
    w.move({160, 135});
    w.move({401, 301});  // snapped against the frozen index
    REQUIRE(w.doc.undo());
    w.move({410, 310});
    CHECK_FALSE(w.tool.transforming());
    REQUIRE(w.doc.redo());
    w.move({420, 320});
    w.release({420, 320});

    World d;
    d.press({150, 125});
    d.move({170, 145});
    REQUIRE(d.doc.execute(std::make_unique<document::RemoveObjectsCommand>(std::vector<ObjectId>{d.rect})));
    d.move({180, 155});
    d.move({190, 165});
    d.release({190, 165});
    CHECK(d.doc.objectCount() == 0);

    World s;
    Document replacement{document::Job::singleSided(500.0, 500.0, 18.0, document::Units::Millimetres)};
    s.press({150, 125});
    s.move({170, 145});
    s.tool.setDocument(&replacement);
    s.move({180, 155});
    s.release({180, 155});
    CHECK(replacement.undoStack().count() == 0);
}

TEST_CASE("a frozen drag snap index is not used after an outside edit (also run under ASan)",
          "[transform][tool][snapping]") {
    World w;
    Contour target;
    target.append(geometry::LineSpan{{400, 300}, {450, 300}});
    const ObjectId other = w.doc.insertObject(Object::vector(target, w.doc.activeLayer(), w.doc.activeSheet()));
    w.press({150, 125});
    w.move({160, 135});
    w.move({401, 301});
    REQUIRE(w.tool.transforming());
    REQUIRE(w.tool.snap().has_value());  // snapped to the other object's endpoint
    CHECK(nearPt(w.tool.pointMm(), {400, 300}));

    // The other object is deleted behind the gesture's back: its geometry must not snap any more
    // and the stale index must not be dereferenced.
    REQUIRE(w.doc.execute(std::make_unique<document::RemoveObjectsCommand>(std::vector<ObjectId>{other})));
    w.move({401, 301});
    if (w.tool.snap().has_value()) {
        CHECK_FALSE(nearPt(w.tool.snap()->pointMm, {400, 300}));
        for (ObjectId id : w.tool.snap()->objects) {
            CHECK(id != other);
        }
    }
    CHECK_FALSE(nearPt(w.tool.pointMm(), {400, 300}));
    w.release({401, 301});
}
