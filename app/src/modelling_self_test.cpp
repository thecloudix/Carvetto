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
    return {};
}

}  // namespace carvetto::app
