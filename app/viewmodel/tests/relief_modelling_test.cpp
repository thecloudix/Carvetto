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

// --- T-0306 modify forms ---------------------------------------------------------------------------------

namespace {

/// A 20 x 20 mm Add shape (45 degrees, 1 mm cells) from a rectangle, applied through the session.
ComponentId addModel(Document& doc) {
    const ObjectId v = addRect(doc, 10.0, 10.0, 30.0, 30.0);
    doc.selection().assign({v});
    CreateShapeForm f;
    const ModellingPreview p = previewCreateShape(doc, f);
    REQUIRE(p.ok);
    ModellingSession s;
    REQUIRE(s.apply(doc, p.component).ok);
    doc.selection().assign({});
    return s.current(doc);
}

}  // namespace

TEST_CASE("modelling: custom shape profile text is parsed and validated", "[viewmodel][relief][modelling]") {
    std::vector<relief::ProfilePoint> pts;
    std::string err;
    REQUIRE(parseShapeProfile("0,0; 0.5,0.8; 1,1;", pts, err));
    REQUIRE(pts.size() == 3);
    CHECK(pts[1].h == 0.8);
    CHECK_FALSE(parseShapeProfile("0,0", pts, err));
    CHECK_FALSE(parseShapeProfile("0,0; 0,1", pts, err));
    CHECK_FALSE(parseShapeProfile("0,0; 2,1", pts, err));
    CHECK_FALSE(parseShapeProfile("0,0; x,1", pts, err));

    Document doc;
    doc.selection().assign({addRect(doc, 10.0, 10.0, 30.0, 30.0)});
    CreateShapeForm f;
    f.kind = relief::ShapeKind::Custom;
    f.profile = "0,0; 0.5,1; 1,0.5";
    CHECK(previewCreateShape(doc, f).ok);
    f.profile = "nonsense";
    CHECK_FALSE(previewCreateShape(doc, f).ok);
}

TEST_CASE("modelling: every modify tool has fields and defaults", "[viewmodel][relief][modelling]") {
    Document doc;
    for (int i = 0; i < kModifyToolCount; ++i) {
        const auto tool = static_cast<ModifyTool>(i);
        CHECK(std::string(modifyToolName(tool)).size() > 0);
        CHECK(std::string(modifyHelp(tool)).size() > 0);
        const ModifyForm f = defaultModifyForm(doc, tool);
        CHECK(f.values.size() == modifyFields(tool).size());
    }
    CHECK_FALSE(previewModify(doc, defaultModifyForm(doc, ModifyTool::Smooth)).ok);   // no model
}

TEST_CASE("modelling: replace below, offset, smooth and emboss change the component in place",
          "[viewmodel][relief][modelling]") {
    Document doc;
    const ComponentId id = addModel(doc);
    const float top = maxHeight(doc.findReliefComponent(id)->data);

    ModifyForm rb = defaultModifyForm(doc, ModifyTool::ReplaceBelow);
    rb.values["Threshold"] = "3";
    ModifyResult r = previewModify(doc, rb);
    REQUIRE(r.ok);
    CHECK(r.previewField() != nullptr);
    CHECK(doc.findReliefComponent(id)->data.field.at(0, 0) != r.replacements.front().field.at(0, 0));  // untouched
    const std::size_t steps = doc.undoStack().count();
    REQUIRE(applyModify(doc, rb).ok);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.reliefComponentCount() == 1);
    CHECK(maxHeight(doc.findReliefComponent(id)->data) == top);
    CHECK(doc.undo());

    ModifyForm off = defaultModifyForm(doc, ModifyTool::Offset);
    off.values["Distance"] = "bad";
    CHECK_FALSE(previewModify(doc, off).ok);
    CHECK(previewModify(doc, off).message.find("Distance") != std::string::npos);
    off.values["Distance"] = "-1";
    REQUIRE(applyModify(doc, off).ok);

    ModifyForm sm = defaultModifyForm(doc, ModifyTool::Smooth);
    REQUIRE(applyModify(doc, sm).ok);
    ModifyForm em = defaultModifyForm(doc, ModifyTool::Emboss);
    REQUIRE(applyModify(doc, em).ok);
    CHECK(doc.reliefComponentCount() == 1);
}

TEST_CASE("modelling: reset inside vectors needs closed vectors and makes cells transparent",
          "[viewmodel][relief][modelling]") {
    Document doc;
    const ComponentId id = addModel(doc);
    ModifyForm f = defaultModifyForm(doc, ModifyTool::ResetByVectors);
    CHECK_FALSE(previewModify(doc, f).ok);
    doc.selection().assign({addRect(doc, 15.0, 15.0, 25.0, 25.0)});
    const ModifyResult r = previewModify(doc, f);
    REQUIRE(r.ok);
    const relief::HeightField& field = r.replacements.front().field;
    const relief::HeightField& orig = doc.findReliefComponent(id)->data.field;
    const std::size_t ci = static_cast<std::size_t>((20.0 - orig.origin().x) / orig.cellSize());
    const std::size_t cj = static_cast<std::size_t>((20.0 - orig.origin().y) / orig.cellSize());
    CHECK(std::isnan(field.at(ci, cj)));
    CHECK_FALSE(std::isnan(orig.at(ci, cj)));
    f.values["Region"] = "Outside";
    const ModifyResult out = previewModify(doc, f);
    REQUIRE(out.ok);
    CHECK_FALSE(std::isnan(out.replacements.front().field.at(ci, cj)));
}

TEST_CASE("modelling: slice, draft and split add components in one undo step", "[viewmodel][relief][modelling]") {
    Document doc;
    const ComponentId id = addModel(doc);
    const std::size_t steps = doc.undoStack().count();

    ModifyForm sl = defaultModifyForm(doc, ModifyTool::Slice);
    sl.values["Thickness"] = "4";
    sl.values["Boundary vectors"] = "true";
    const ModifyResult sp = previewModify(doc, sl);
    REQUIRE(sp.ok);
    CHECK(sp.additions.size() >= 2);
    CHECK_FALSE(sp.vectors.empty());
    const std::size_t objects = doc.objects().size();
    REQUIRE(applyModify(doc, sl).ok);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.reliefComponentCount() == 1 + sp.additions.size());
    CHECK(doc.objects().size() == objects + 1);
    CHECK(doc.undo());
    CHECK(doc.reliefComponentCount() == 1);
    CHECK(doc.objects().size() == objects);

    ModifyForm dr = defaultModifyForm(doc, ModifyTool::Draft);
    REQUIRE(applyModify(doc, dr).ok);
    CHECK(doc.reliefComponentCount() == 2);
    CHECK(doc.undo());

    // Split with an open vertical line at x = 20: original becomes part A, part B is added.
    Object line;
    line.layer = doc.defaultLayer();
    line.sheet = doc.activeSheet();
    VectorObject vo;
    geometry::Polyline pl;
    pl.points = {{20.0, 0.0}, {20.0, 40.0}};
    vo.contours.push_back(geometry::Contour::fromPolyline(pl));
    line.data = std::move(vo);
    auto add = std::make_unique<AddObjectCommand>(std::move(line));
    REQUIRE(doc.execute(std::move(add)));
    ModifyForm sp2 = defaultModifyForm(doc, ModifyTool::Split);
    CHECK_FALSE(previewModify(doc, sp2).ok);   // no vector selected
    doc.selection().assign({doc.objects().rbegin()->first});
    REQUIRE(applyModify(doc, sp2).ok);
    CHECK(doc.reliefComponentCount() == 2);
    CHECK(doc.findReliefComponent(id)->data.name.find("A") != std::string::npos);
    CHECK(doc.undo());
    CHECK(doc.reliefComponentCount() == 1);
}

TEST_CASE("modelling: vector boundary, cross section and distort", "[viewmodel][relief][modelling]") {
    Document doc;
    const ComponentId id = addModel(doc);
    const std::size_t objects = doc.objects().size();

    REQUIRE(applyModify(doc, defaultModifyForm(doc, ModifyTool::VectorBoundary)).ok);
    CHECK(doc.objects().size() == objects + 1);

    ModifyForm cs = defaultModifyForm(doc, ModifyTool::CrossSection);   // starts from the bounds
    const ModifyResult r = previewModify(doc, cs);
    REQUIRE(r.ok);
    CHECK_FALSE(r.vectors.empty());
    cs.values["Spacing"] = "-1";
    CHECK_FALSE(previewModify(doc, cs).ok);

    ModifyForm di = defaultModifyForm(doc, ModifyTool::Distort);
    const double x1 = std::stod(di.values["Right-bottom X"]);
    di.values["Right-bottom X"] = std::to_string(x1 + 10.0);
    di.values["Right-top X"] = std::to_string(x1 + 10.0);
    const std::size_t steps = doc.undoStack().count();
    REQUIRE(applyModify(doc, di).ok);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.findReliefComponent(id)->data.field.width() > 20);
    di.values["Right-top X"] = "bad";
    CHECK_FALSE(applyModify(doc, di).ok);
}
