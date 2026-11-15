// Self-test of the modelling forms (T-0300): drives the real MainWindow offscreen through the
// widgets of the Modelling dock (Create Shape, Bitmap, Zero Plane, Scale Z).

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <memory>
#include <string>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "main_window.hpp"
#include "modelling_panel.hpp"
#include "carvetto/viewmodel/relief_modelling.hpp"
#include "relief_panels.hpp"

namespace carvetto::app {

namespace {

#define MD_REQUIRE(cond, text)                         \
    do {                                               \
        if (!(cond)) {                                 \
            return std::string("modelling: ") + (text); \
        }                                              \
    } while (false)

float topOf(const document::Document& doc, document::ComponentId id) {
    float lo = 0.0F;
    float hi = 0.0F;
    const document::ReliefComponent* c = doc.findReliefComponent(id);
    return c != nullptr && c->data.field.minMax(lo, hi) ? hi : -1.0F;
}

}  // namespace

std::string modellingSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres)));
    ModellingPanel* panel = window.modelling();
    ModelView3D* view = window.modelView();
    MD_REQUIRE(panel != nullptr && view != nullptr, "the panel is missing");
    document::Document* doc = &window.currentDocument();

    // Create Shape from a selected rectangle.
    panel->setTool(ModellingPanel::Tool::CreateShape);
    document::Object rect;
    rect.layer = doc->defaultLayer();
    rect.sheet = doc->activeSheet();
    document::VectorObject vec;
    vec.contours.push_back(geometry::Contour::rectangle(geometry::Box2({10.0, 10.0}, {50.0, 30.0})));
    rect.data = std::move(vec);
    auto add = std::make_unique<document::AddObjectCommand>(std::move(rect));
    const document::ObjectId rectId = add.get()->id();
    MD_REQUIRE(doc->execute(std::move(add)), "adding the vector failed");
    const document::ObjectId vecId = doc->objects().rbegin()->first;
    (void)rectId;
    MD_REQUIRE(!panel->preview() && !panel->status()->text().isEmpty(), "preview without a selection was accepted");
    doc->selection().assign({vecId});
    MD_REQUIRE(panel->preview(), "the shape preview failed");
    MD_REQUIRE(view->previewing() && view->surface().ok(), "the preview is not shown in the Model view");
    MD_REQUIRE(doc->reliefComponentCount() == 0, "previewing changed the document");
    panel->field(QStringLiteral("Shape: Name"))->setText(QStringLiteral("Roof"));
    const std::size_t steps = doc->undoStack().count();
    MD_REQUIRE(panel->apply(), "applying the shape failed");
    MD_REQUIRE(!view->previewing(), "the preview stayed after Apply");
    MD_REQUIRE(doc->reliefComponentCount() == 1 && doc->undoStack().count() == steps + 1,
               "Apply is not one undo step");
    const document::ComponentId shape = panel->session().current(*doc);
    MD_REQUIRE(shape && doc->findReliefComponent(shape)->data.name == "Roof", "the shape was not added");
    const float shapeTop = topOf(*doc, shape);
    MD_REQUIRE(shapeTop > 8.0F, "the 45 degree shape is too low");

    // Apply again with a limit updates the same component; Start New Component adds another.
    panel->check(QStringLiteral("Shape: Limit height"))->setChecked(true);
    panel->field(QStringLiteral("Shape: Limit"))->setText(QStringLiteral("3"));
    MD_REQUIRE(panel->apply() && doc->reliefComponentCount() == 1 && topOf(*doc, shape) <= 3.001F,
               "re-applying did not update the component");
    panel->startNew();
    MD_REQUIRE(panel->apply() && doc->reliefComponentCount() == 2, "Start New Component did not add another");
    panel->field(QStringLiteral("Shape: Angle"))->setText(QStringLiteral("bad"));
    MD_REQUIRE(!panel->apply() && panel->status()->text().contains(QStringLiteral("Angle")),
               "a bad number was accepted");
    panel->reset();
    MD_REQUIRE(panel->field(QStringLiteral("Shape: Angle"))->text() == QStringLiteral("45") &&
                   !panel->check(QStringLiteral("Shape: Limit height"))->isChecked(),
               "Reset did not restore the defaults");

    // Zero plane.
    panel->setTool(ModellingPanel::Tool::ZeroPlane);
    panel->field(QStringLiteral("Zero Plane: Cell size"))->setText(QStringLiteral("5"));
    const std::size_t before = doc->reliefComponentCount();
    panel->startNew();
    MD_REQUIRE(panel->apply() && doc->reliefComponentCount() == before + 1, "the zero plane was not added");
    MD_REQUIRE(topOf(*doc, panel->session().current(*doc)) == 0.0F, "the zero plane is not flat");

    // Bitmap from an embedded Netpbm image.
    panel->setTool(ModellingPanel::Tool::Bitmap);
    const std::string pgm = "P2\n2 2\n255\n0 255\n128 255\n";
    document::BitmapImport in;
    in.bytes.assign(pgm.begin(), pgm.end());
    in.widthPx = 2;
    in.heightPx = 2;
    in.placementMm = geometry::Box2({0.0, 0.0}, {20.0, 20.0});
    in.layer = doc->defaultLayer();
    in.sheet = doc->activeSheet();
    in.name = "img";
    auto imp = document::makeImportBitmapCommand(*doc, std::move(in));
    MD_REQUIRE(imp != nullptr && doc->execute(std::move(imp)), "importing the bitmap failed");
    doc->selection().assign({doc->objects().rbegin()->first});
    panel->useSelectedBitmap();
    MD_REQUIRE(panel->field(QStringLiteral("Bitmap: Width"))->text() == QStringLiteral("20"),
               "Use selected bitmap did not fill the width");
    panel->field(QStringLiteral("Bitmap: Z height"))->setText(QStringLiteral("4"));
    panel->startNew();
    MD_REQUIRE(panel->apply(), "applying the bitmap failed");
    MD_REQUIRE(topOf(*doc, panel->session().current(*doc)) == 4.0F, "the bitmap height is wrong");

    // Scale Z on the selected component, then on the whole model.
    const document::ComponentId bmp = panel->session().current(*doc);
    panel->setTool(ModellingPanel::Tool::ScaleZ);
    panel->field(QStringLiteral("Scale Z: Value"))->setText(QStringLiteral("50"));
    MD_REQUIRE(panel->preview() && view->previewing(), "the Scale Z preview failed");
    const std::size_t scaleSteps = doc->undoStack().count();
    MD_REQUIRE(panel->apply() && doc->undoStack().count() == scaleSteps + 1, "Scale Z is not one undo step");
    MD_REQUIRE(topOf(*doc, bmp) == 2.0F, "Scale Z did not halve the bitmap");
    MD_REQUIRE(window.undo() && topOf(*doc, bmp) == 4.0F, "undo did not restore the height");
    panel->combo(QStringLiteral("Scale Z: Kind"))->setCurrentIndex(1);
    panel->field(QStringLiteral("Scale Z: Value"))->setText(QStringLiteral("1"));
    MD_REQUIRE(panel->apply() && topOf(*doc, bmp) == 1.0F, "Scale Z to an exact height failed");
    panel->field(QStringLiteral("Scale Z: Value"))->setText(QStringLiteral("-3"));
    MD_REQUIRE(!panel->apply(), "a negative height was accepted");
    panel->clearPreview();
    window.undo();

    // Modify forms (T-0306): offset and slice on the bitmap component, then a custom shape profile.
    doc->componentSelection().assign({bmp});
    panel->setModifyTool(viewmodel::ModifyTool::Smooth);
    MD_REQUIRE(panel->field(QStringLiteral("Smooth: Radius")) != nullptr &&
                   panel->combo(QStringLiteral("Smooth: Radius")) == nullptr,
               "the Smooth page was not built");
    panel->field(QStringLiteral("Smooth: Percent"))->setText(QStringLiteral("100"));
    panel->field(QStringLiteral("Smooth: Radius"))->setText(QStringLiteral("50"));   // cells are 10 mm here
    MD_REQUIRE(panel->preview() && view->previewing(), "the Smooth preview failed");
    const std::size_t modSteps = doc->undoStack().index();   // not count(): a redo tail would be dropped
    const std::size_t modCount = doc->reliefComponentCount();
    const bool applied = panel->apply();
    MD_REQUIRE(applied && doc->undoStack().index() == modSteps + 1 && doc->reliefComponentCount() == modCount,
               "Smooth is not one in-place undo step");
    MD_REQUIRE(window.undo(), "undoing Smooth failed");

    panel->setModifyTool(viewmodel::ModifyTool::Slice);
    panel->field(QStringLiteral("Slice Model: Thickness"))->setText(QStringLiteral("1"));
    panel->check(QStringLiteral("Slice Model: Boundary vectors"))->setChecked(true);
    MD_REQUIRE(panel->preview(), "the Slice preview failed");
    const std::size_t objectsBefore = doc->objects().size();
    MD_REQUIRE(panel->apply() && doc->reliefComponentCount() > modCount && doc->objects().size() == objectsBefore + 1 &&
                   doc->undoStack().index() == modSteps + 1,
               "Slice did not add components and boundaries as one step");
    MD_REQUIRE(window.undo() && doc->reliefComponentCount() == modCount, "undoing Slice failed");

    panel->setModifyTool(viewmodel::ModifyTool::Offset);
    panel->field(QStringLiteral("Offset Model: Distance"))->setText(QStringLiteral("x"));
    MD_REQUIRE(!panel->preview() && panel->status()->text().contains(QStringLiteral("Distance")),
               "a bad offset distance was accepted");
    panel->setModifyTool(viewmodel::ModifyTool::VectorBoundary);
    MD_REQUIRE(panel->apply() && doc->objects().size() == objectsBefore + 1, "Create Vector Boundary failed");
    window.undo();

    panel->setTool(ModellingPanel::Tool::CreateShape);
    panel->combo(QStringLiteral("Shape: Kind"))->setCurrentIndex(static_cast<int>(relief::ShapeKind::Custom));
    panel->field(QStringLiteral("Shape: Profile"))->setText(QStringLiteral("0,0; 0.5,1; 1,0.5"));
    doc->selection().assign({vecId});
    MD_REQUIRE(panel->preview(), "a custom profile preview failed");
    panel->field(QStringLiteral("Shape: Profile"))->setText(QStringLiteral("0,0"));
    MD_REQUIRE(!panel->preview(), "a one-point profile was accepted");

    // Build forms (T-0308): Two Rail Sweep, Turn and Spin, Textured Area from the selection.
    const auto addLine = [&](std::initializer_list<geometry::Vec2> pts, bool closed) {
        document::Object o;
        o.layer = doc->defaultLayer();
        o.sheet = doc->activeSheet();
        document::VectorObject v;
        geometry::Polyline pl;
        pl.points.assign(pts.begin(), pts.end());
        pl.closed = closed;
        v.contours.push_back(geometry::Contour::fromPolyline(pl));
        o.data = std::move(v);
        const bool ok = doc->execute(std::make_unique<document::AddObjectCommand>(std::move(o)));
        return ok ? doc->objects().rbegin()->first : document::ObjectId{};
    };
    const document::ObjectId rail1 = addLine({{0, 60}, {40, 60}}, false);
    const document::ObjectId rail2 = addLine({{0, 70}, {40, 70}}, false);
    MD_REQUIRE(rail1 && rail2, "adding the rails failed");
    doc->selection().assign({rail1, rail2});
    panel->setBuildTool(viewmodel::BuildTool::TwoRailSweep);
    MD_REQUIRE(panel->field(QStringLiteral("Two Rail Sweep: Section profile")) != nullptr &&
                   panel->combo(QStringLiteral("Two Rail Sweep: Combine mode")) != nullptr,
               "the sweep page was not built");
    MD_REQUIRE(panel->preview() && view->previewing(), "the sweep preview failed");
    panel->startNew();
    const std::size_t buildCount = doc->reliefComponentCount();
    const std::size_t buildSteps = doc->undoStack().index();
    MD_REQUIRE(panel->apply() && doc->reliefComponentCount() == buildCount + 1 &&
                   doc->undoStack().index() == buildSteps + 1,
               "the sweep is not one undo step");
    const document::ComponentId sweepId = panel->session().current(*doc);
    MD_REQUIRE(topOf(*doc, sweepId) > 3.5F, "the swept surface is too low");
    panel->field(QStringLiteral("Two Rail Sweep: Exact height"))->setText(QStringLiteral("2"));
    MD_REQUIRE(panel->apply() && doc->reliefComponentCount() == buildCount + 1 && topOf(*doc, sweepId) < 2.01F,
               "re-applying the sweep did not update it");
    panel->field(QStringLiteral("Two Rail Sweep: Cell size"))->setText(QStringLiteral("zero"));
    MD_REQUIRE(!panel->apply() && panel->status()->text().contains(QStringLiteral("Cell size")),
               "a bad cell size was accepted");
    MD_REQUIRE(window.undo() && window.undo(), "undoing the sweep failed");

    panel->setBuildTool(viewmodel::BuildTool::TurnSpin);
    panel->startNew();
    MD_REQUIRE(panel->preview() && panel->apply() && doc->reliefComponentCount() == buildCount + 1,
               "the Spin default did not apply");
    MD_REQUIRE(window.undo(), "undoing the spin failed");

    panel->setBuildTool(viewmodel::BuildTool::TexturedArea);
    const document::ObjectId boundary = addLine({{0, 0}, {30, 0}, {30, 20}, {0, 20}}, true);
    doc->selection().assign({boundary});
    doc->componentSelection().assign({bmp});
    panel->startNew();
    MD_REQUIRE(panel->preview(), "the textured area preview failed");
    MD_REQUIRE(panel->apply() && doc->reliefComponentCount() == buildCount + 1, "the textured area was not added");
    MD_REQUIRE(doc->componentSelection().ids().size() == 1 && doc->componentSelection().ids().front() == bmp,
               "the seed selection was not kept");
    window.undo();
    panel->clearPreview();
    return {};
}


}  // namespace carvetto::app
