#include "carvetto/viewmodel/relief_modelling.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

ObjectId addRect(Document& doc, double x0, double y0, double x1, double y1) {
    Object o;
    o.layer = doc.defaultLayer();
    o.sheet = doc.activeSheet();
    VectorObject v;
    v.contours.push_back(geometry::Contour::rectangle(geometry::Box2({x0, y0}, {x1, y1})));
    o.data = std::move(v);
    auto cmd = std::make_unique<AddObjectCommand>(std::move(o));
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

ObjectId addBitmap(Document& doc) {
    const std::string pgm = "P2\n2 2\n255\n0 255\n128 255\n";
    BitmapImport in;
    in.bytes.assign(pgm.begin(), pgm.end());
    in.widthPx = 2;
    in.heightPx = 2;
    in.placementMm = geometry::Box2({10.0, 20.0}, {30.0, 40.0});
    in.layer = doc.defaultLayer();
    in.sheet = doc.activeSheet();
    in.name = "img";
    auto cmd = makeImportBitmapCommand(doc, std::move(in));
    REQUIRE(cmd != nullptr);
    REQUIRE(doc.execute(std::move(cmd)));
    return doc.objects().rbegin()->first;
}

float maxHeight(const relief::Component& c) {
    float lo = 0.0F;
    float hi = 0.0F;
    REQUIRE(c.field.minMax(lo, hi));
    return hi;
}

}  // namespace

TEST_CASE("modelling: Create Shape previews selected closed vectors without touching the document",
          "[viewmodel][relief][modelling]") {
    Document doc;
    CreateShapeForm form;
    CHECK_FALSE(previewCreateShape(doc, form).ok);  // nothing selected
    const ObjectId r = addRect(doc, 0, 0, 20, 10);
    doc.selection().assign({r});
    CHECK(selectedShapeVectorCount(doc) == 1);
    const std::size_t steps = doc.undoStack().count();
    const ModellingPreview p = previewCreateShape(doc, form);
    REQUIRE(p.ok);
    CHECK(p.component.name == "Shape");
    CHECK(maxHeight(p.component) > 4.0F);  // 45 degrees on a 10 mm wide rectangle: ~5 mm
    CHECK(doc.undoStack().count() == steps);
    CHECK(doc.reliefComponentCount() == 0);

    form.limitEnabled = true;
    form.limit = "2";
    CHECK(maxHeight(previewCreateShape(doc, form).component) <= 2.0F + 1e-4F);
    form.kind = relief::ShapeKind::Plane;
    CHECK(maxHeight(previewCreateShape(doc, form).component) == 0.0F);
}

TEST_CASE("modelling: Create Shape refuses bad numbers and open vectors", "[viewmodel][relief][modelling]") {
    Document doc;
    const ObjectId r = addRect(doc, 0, 0, 20, 10);
    doc.selection().assign({r});
    CreateShapeForm form;
    form.angleDeg = "abc";
    ModellingPreview p = previewCreateShape(doc, form);
    CHECK_FALSE(p.ok);
    CHECK(p.message.find("Angle") != std::string::npos);
    form.angleDeg = "45";
    form.cellSize = "0";
    CHECK_FALSE(previewCreateShape(doc, form).ok);
    form.cellSize = "1";
    form.angleDeg = "120";
    CHECK_FALSE(previewCreateShape(doc, form).ok);
}

TEST_CASE("modelling: bitmap relief from the selected bitmap object", "[viewmodel][relief][modelling]") {
    Document doc;
    CHECK_FALSE(previewBitmapRelief(doc, BitmapReliefForm{}, {}).ok);  // no image
    const ObjectId b = addBitmap(doc);
    doc.selection().assign({b});
    BitmapReliefForm form = defaultBitmapForm(doc);
    CHECK(form.widthMm == "20");
    CHECK(form.cornerX == "10");
    CHECK(form.cornerY == "20");
    form.zHeightMm = "10";
    const ModellingPreview p = previewBitmapRelief(doc, form, {});
    REQUIRE(p.ok);
    CHECK(p.component.field.width() == 2);
    CHECK(maxHeight(p.component) == 10.0F);
    // Top row of the image is the highest Y row: pixels "0 255" -> cell row 1.
    CHECK(p.component.field.at(0, 1) == 0.0F);
    CHECK(p.component.field.at(1, 1) == 10.0F);
    form.invert = true;
    CHECK(previewBitmapRelief(doc, form, {}).component.field.at(0, 1) == 10.0F);
    form.widthMm = "-1";
    CHECK_FALSE(previewBitmapRelief(doc, form, {}).ok);
    form.widthMm = "20";
    form.filePath = "/nonexistent/image.png";
    CHECK_FALSE(previewBitmapRelief(doc, form, {}).ok);
}

TEST_CASE("modelling: zero plane covers the job", "[viewmodel][relief][modelling]") {
    Document doc;  // default job 300 x 300
    ZeroPlaneForm form;
    form.cellSize = "10";
    const ModellingPreview p = previewZeroPlane(doc, form);
    REQUIRE(p.ok);
    CHECK(p.component.field.width() == 31);
    CHECK(p.component.name == "Zero Plane");
    CHECK(maxHeight(p.component) == 0.0F);
    form.cellSize = "-2";
    CHECK_FALSE(previewZeroPlane(doc, form).ok);
}

TEST_CASE("modelling: session adds, re-applies in place and starts new", "[viewmodel][relief][modelling]") {
    Document doc;
    ModellingSession session;
    ZeroPlaneForm form;
    form.cellSize = "50";
    REQUIRE(session.apply(doc, previewZeroPlane(doc, form).component).changed);
    CHECK(doc.reliefLevels().size() == 1);  // a level was made
    CHECK(doc.reliefComponentCount() == 1);
    CHECK(doc.undoStack().count() == 1);   // level and component are one step
    const ComponentId first = session.current(doc);
    CHECK_FALSE(first.isNull());
    CHECK(doc.componentSelection().single() == first);

    form.cellSize = "100";
    REQUIRE(session.apply(doc, previewZeroPlane(doc, form).component).changed);
    CHECK(doc.reliefComponentCount() == 1);
    CHECK(doc.findReliefComponent(first)->data.field.cellSize() == 100.0);
    CHECK(doc.undoStack().count() == 2);
    CHECK(doc.undo());
    CHECK(doc.findReliefComponent(first)->data.field.cellSize() == 50.0);

    session.startNew();
    CHECK(session.current(doc).isNull());
    REQUIRE(session.apply(doc, previewZeroPlane(doc, form).component).changed);
    CHECK(doc.reliefComponentCount() == 2);
    CHECK(doc.reliefLevels().size() == 1);
    CHECK(doc.undo());
    CHECK(doc.reliefComponentCount() == 1);

    // A component the user deleted is not re-applied to.
    REQUIRE(doc.execute(std::make_unique<DeleteReliefComponentsCommand>(std::vector<ComponentId>{first})));
    CHECK(session.current(doc).isNull());
}

TEST_CASE("modelling: Scale Z previews and applies as one undo step", "[viewmodel][relief][modelling]") {
    Document doc;
    ModellingSession session;
    CreateShapeForm sf;
    const ObjectId r = addRect(doc, 0, 0, 20, 10);
    doc.selection().assign({r});
    REQUIRE(session.apply(doc, previewCreateShape(doc, sf).component).changed);
    const ComponentId a = session.current(doc);
    session.startNew();
    REQUIRE(session.apply(doc, previewCreateShape(doc, sf).component).changed);
    const ComponentId b = session.current(doc);
    const float hiA = maxHeight(doc.findReliefComponent(a)->data);

    ScaleZForm form;
    form.value = "50";
    doc.componentSelection().assign({a});
    const ScaleZPreview p = previewScaleZ(doc, form);
    REQUIRE(p.ok);
    CHECK(p.ids.size() == 1);
    CHECK(p.factor == 0.5);
    CHECK(maxHeight(doc.findReliefComponent(a)->data) == hiA);  // preview does not edit

    const std::size_t steps = doc.undoStack().count();
    REQUIRE(applyScaleZ(doc, form).changed);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(std::fabs(maxHeight(doc.findReliefComponent(a)->data) - hiA * 0.5F) < 1e-5F);
    CHECK(maxHeight(doc.findReliefComponent(b)->data) == hiA);  // unselected: untouched
    CHECK(doc.undo());
    CHECK(maxHeight(doc.findReliefComponent(a)->data) == hiA);

    // Nothing selected: the whole sheet's model.
    doc.componentSelection().clear();
    REQUIRE(applyScaleZ(doc, form).changed);
    CHECK(std::fabs(maxHeight(doc.findReliefComponent(b)->data) - hiA * 0.5F) < 1e-5F);

    // Exact height.
    form.kind = relief::ScaleZ::Kind::ExactHeight;
    form.value = "3";
    REQUIRE(applyScaleZ(doc, form).changed);
    CHECK(std::fabs(maxHeight(doc.findReliefComponent(a)->data) - 3.0F) < 1e-4F);

    form.value = "x";
    CHECK_FALSE(applyScaleZ(doc, form).ok);
    form.value = "0";
    CHECK_FALSE(applyScaleZ(doc, form).ok);
}

TEST_CASE("modelling: Scale Z on a document without a model is refused", "[viewmodel][relief][modelling]") {
    Document doc;
    const EditResult r = applyScaleZ(doc, ScaleZForm{});
    CHECK_FALSE(r.ok);
    CHECK_FALSE(r.changed);
    CHECK(doc.undoStack().count() == 0);
}
