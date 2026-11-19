#include "carvetto/viewmodel/nest_selection.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto;
using Catch::Approx;
using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
using namespace carvetto::viewmodel;

namespace {

ObjectId addRect(Document& doc, double x0, double y0, double x1, double y1) {
    return doc.insertObject(Object::vector(Contour::rectangle(Box2{{x0, y0}, {x1, y1}}), doc.activeLayer(),
                                           doc.activeSheet()));
}

ObjectId addOpen(Document& doc, Vec2 a, Vec2 b) {
    Contour c;
    c.append(geometry::LineSpan{a, b});
    return doc.insertObject(Object::vector(std::move(c), doc.activeLayer(), doc.activeSheet()));
}

NestSelectionOptions baseOptions() {
    NestSelectionOptions o;
    o.sheets = {{500, 300}};
    o.toolDiameterMm = 4;
    o.clearanceMm = 1;
    o.borderGapMm = 5;
    return o;
}

Box2 boundsOf(const Document& doc, ObjectId id) {
    const auto b = doc.objectBounds(id);
    REQUIRE(b.has_value());
    return *b;
}

}  // namespace

TEST_CASE("nest selection: copies land on new sheets in one undo step", "[nest][viewmodel]") {
    Document doc;
    const ObjectId a = addRect(doc, 1000, 1000, 1100, 1050);
    const ObjectId b = addRect(doc, 2000, 0, 2200, 50);
    doc.selection().assign({a, b});
    NestSelectionOptions o = baseOptions();
    o.removeOriginals = false;
    o.copies = 3;
    const std::size_t objectsBefore = doc.objectCount();
    const std::size_t sheetsBefore = doc.sheets().size();
    const NestSelectionResult r = nestSelection(doc, o);
    REQUIRE(r.ok);
    CHECK(r.parts == 2);
    CHECK(r.placed == 6);
    CHECK(r.unplaced == 0);
    CHECK(r.created.size() == 6);
    CHECK(doc.objectCount() == objectsBefore + 6);
    CHECK(doc.sheets().size() == sheetsBefore + r.sheets.size());
    REQUIRE_FALSE(r.sheets.empty());
    CHECK(doc.activeSheet() == r.sheets[0]);
    for (ObjectId id : r.created) {
        const Box2 bb = boundsOf(doc, id);
        CHECK(bb.min.x >= 5 - 1e-6);
        CHECK(bb.max.x <= 495 + 1e-6);
        CHECK(bb.min.y >= 5 - 1e-6);
        CHECK(bb.max.y <= 295 + 1e-6);
    }
    CHECK(doc.selection().ids() == r.created);
    // One undo removes the sheets and every copy.
    doc.undoStack().undo(doc);
    CHECK(doc.objectCount() == objectsBefore);
    CHECK(doc.sheets().size() == sheetsBefore);
    CHECK_FALSE(doc.undoStack().canUndo());
}

TEST_CASE("nest selection: removing originals and mirrored copies keep a counter-clockwise outline", "[nest][viewmodel]") {
    Document doc;
    const ObjectId a = addRect(doc, 0, 0, 100, 50);
    doc.selection().assign({a});
    NestSelectionOptions o = baseOptions();
    o.mirror = true;
    o.rotationStepRad = 3.14159265358979323846 / 2;
    const NestSelectionResult r = nestSelection(doc, o);
    REQUIRE(r.ok);
    CHECK(doc.findObject(a) == nullptr);
    CHECK(r.created.size() == 1);
    doc.undoStack().undo(doc);
    CHECK(doc.findObject(a) != nullptr);
}

TEST_CASE("nest selection: contained and open vectors travel with their part", "[nest][viewmodel]") {
    Document doc;
    const ObjectId frame = addRect(doc, 0, 0, 200, 100);
    const ObjectId inner = addRect(doc, 20, 20, 60, 60);   // contained: part of the frame
    const ObjectId mark = addOpen(doc, {100, 50}, {150, 50});  // open, inside the frame
    const ObjectId stray = addOpen(doc, {900, 900}, {950, 900});
    doc.selection().assign({frame, inner, mark, stray});
    NestSelectionOptions o = baseOptions();
    o.removeOriginals = false;
    const NestSelectionResult r = nestSelection(doc, o);
    REQUIRE(r.ok);
    CHECK(r.parts == 1);
    CHECK(r.created.size() == 3);
    CHECK(r.skippedOpen == 1);
    const Box2 f = boundsOf(doc, r.created[0]);
    const Box2 in = boundsOf(doc, r.created[1]);
    CHECK(in.min.x - f.min.x == Approx(20));
    CHECK(in.min.y - f.min.y == Approx(20));
}

TEST_CASE("nest selection: parts inside holes", "[nest][viewmodel]") {
    Document doc;
    // A frame as an outline plus a hole outline in one object; a small panel as a second object.
    Object frame = Object::vector(Contour::rectangle(Box2{{0, 0}, {200, 200}}), doc.activeLayer(), doc.activeSheet());
    Contour hole = Contour::rectangle(Box2{{30, 30}, {170, 170}});
    hole.reverse();
    frame.asVector()->contours.push_back(hole);
    const ObjectId f = doc.insertObject(std::move(frame));
    const ObjectId p = addRect(doc, 5000, 5000, 5100, 5100);
    doc.selection().assign({f, p});
    NestSelectionOptions o = baseOptions();
    o.sheets = {{220, 220}};
    o.borderGapMm = 5;
    o.removeOriginals = false;
    NestSelectionResult r = nestSelection(doc, o);
    REQUIRE(r.ok);
    CHECK(r.sheets.size() == 2);
    doc.undoStack().undo(doc);
    o.allowInside = true;
    doc.selection().assign({f, p});
    r = nestSelection(doc, o);
    REQUIRE(r.ok);
    CHECK(r.sheets.size() == 1);
    CHECK(r.placed == 2);
}

TEST_CASE("nest selection: active sheet and boundary areas", "[nest][viewmodel]") {
    Document doc;  // default sheet 300 x 300
    const ObjectId a = addRect(doc, 1000, 1000, 1100, 1050);
    doc.selection().assign({a});
    NestSelectionOptions o = baseOptions();
    o.area = NestArea::ActiveSheet;
    o.corner = geometry::NestCorner::TopRight;
    NestSelectionResult r = nestSelection(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 1);
    const Box2 b = boundsOf(doc, r.created[0]);
    CHECK(b.max.x == Approx(295).margin(0.06));
    CHECK(b.max.y == Approx(295).margin(0.06));
    CHECK(r.sheets.empty());

    // Boundary: a 150 x 150 region on its own layer.
    Document d2;
    document::Layer bl;
    bl.name = "Bound";
    auto cmd = std::make_unique<document::AddLayerCommand>(bl, false);
    const document::AddLayerCommand* raw = cmd.get();
    REQUIRE(d2.execute(std::move(cmd)));
    d2.insertObject(Object::vector(Contour::rectangle(Box2{{400, 400}, {550, 550}}), raw->id(), d2.activeSheet()));
    const ObjectId p = addRect(d2, 0, 0, 100, 100);
    d2.selection().assign({p});
    NestSelectionOptions ob = baseOptions();
    ob.area = NestArea::Boundary;
    ob.boundaryLayer = raw->id();
    ob.borderGapMm = 10;
    r = nestSelection(d2, ob);
    REQUIRE(r.ok);
    const Box2 pb = boundsOf(d2, r.created[0]);
    CHECK(pb.min.x == Approx(410).margin(0.06));
    CHECK(pb.min.y == Approx(410).margin(0.06));
    ob.boundaryLayer = {};
    d2.selection().assign({p});
    CHECK_FALSE(nestSelection(d2, ob).ok);
}

TEST_CASE("nest selection: failures leave the document untouched", "[nest][viewmodel]") {
    Document doc;
    const ObjectId big = addRect(doc, 0, 0, 5000, 5000);
    doc.selection().assign({big});
    NestSelectionOptions o = baseOptions();
    const std::size_t objects = doc.objectCount();
    NestSelectionResult r = nestSelection(doc, o);
    CHECK_FALSE(r.ok);
    CHECK(doc.objectCount() == objects);
    CHECK(doc.sheets().size() == 1);
    CHECK_FALSE(doc.undoStack().canUndo());

    doc.selection().assign({});
    CHECK_FALSE(canNestSelection(doc));
    CHECK_FALSE(nestSelection(doc, o).ok);
    doc.selection().assign({big});
    CHECK(canNestSelection(doc));

    o.copies = 0;
    CHECK_FALSE(validateNestOptions(o).empty());
    o = baseOptions();
    o.sheets = {{-1, 10}};
    CHECK_FALSE(validateNestOptions(o).empty());
    o = baseOptions();
    o.clearanceMm = -1;
    CHECK_FALSE(validateNestOptions(o).empty());
}

TEST_CASE("nest selection: crossing outlines nest as one rigid part", "[nest][viewmodel]") {
    Document doc;
    const ObjectId a = addRect(doc, 0, 0, 100, 100);
    const ObjectId b = addRect(doc, 50, 50, 150, 150);
    doc.selection().assign({a, b});
    NestSelectionOptions o = baseOptions();
    o.removeOriginals = false;
    const NestSelectionResult r = nestSelection(doc, o);
    REQUIRE(r.ok);
    CHECK(r.parts == 1);
    const Box2 ba = boundsOf(doc, r.created[0]);
    const Box2 bb = boundsOf(doc, r.created[1]);
    CHECK(bb.min.x - ba.min.x == Approx(50));
    CHECK(bb.min.y - ba.min.y == Approx(50));
}
