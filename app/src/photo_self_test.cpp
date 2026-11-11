// Self-test of Photo V-carve in the real app (T-0250b), offscreen: File > Import Bitmap (a PNG,
// decoded by QImage), the bitmap painted on the canvas, the Photo V-Carve form of the Toolpaths dock,
// calculation off the UI thread, edit, stale after moving the bitmap, undo, save and reload.

#include <QAction>
#include <QColor>
#include <QDir>
#include <QImage>
#include <QPushButton>
#include <QPoint>
#include <QSize>
#include <QString>
#include <QTemporaryDir>
#include <QTreeWidget>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/view_transform.hpp"
#include "canvas_widget.hpp"
#include "main_window.hpp"
#include "toolpath_form_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define PH_REQUIRE(cond, text)                         \
    do {                                               \
        if (!(cond)) {                                 \
            return std::string("photo v-carve: ") + (text); \
        }                                              \
    } while (false)

double deepest(const document::ToolpathRecord& rec) {
    double z = 0.0;
    if (rec.moves != nullptr) {
        for (const toolpath::Move& m : *rec.moves) {
            z = std::min(z, m.to.z);
        }
    }
    return z;
}

bool accept(ToolpathsPanel* panel) {
    ToolpathFormDialog* dlg = panel->formDialog();
    if (dlg == nullptr || !dlg->okButton()->isEnabled()) {
        return false;
    }
    dlg->accept();
    return true;
}

}  // namespace

std::string photoSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 100.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    ToolpathsPanel* panel = window.toolpaths();
    PH_REQUIRE(panel != nullptr, "no Toolpaths panel");
    PH_REQUIRE(window.findChild<QAction*>(QStringLiteral("importBitmapAction")) != nullptr,
               "File has no Import Bitmap action");

    // 1. A PNG (left half black, right half white) imported through the window.
    QTemporaryDir dir;
    PH_REQUIRE(dir.isValid(), "no temporary directory");
    QImage png(64, 32, QImage::Format_RGB32);
    png.fill(Qt::white);
    for (int y = 0; y < png.height(); ++y) {
        for (int x = 0; x < png.width() / 2; ++x) {
            png.setPixelColor(x, y, Qt::black);
        }
    }
    const QString pngPath = dir.filePath(QStringLiteral("half.png"));
    PH_REQUIRE(png.save(pngPath), "could not write the test PNG");
    const std::size_t steps0 = doc.undoStack().index();
    PH_REQUIRE(window.importBitmapFile(std::filesystem::path(pngPath.toStdString())), "importing the PNG failed");
    PH_REQUIRE(doc.undoStack().index() == steps0 + 1, "the import was not one undo step");
    PH_REQUIRE(doc.bitmaps().size() == 1 && doc.selection().size() == 1, "the image is not embedded and selected");
    const document::ObjectId bitmap = doc.selection().ids().front();
    PH_REQUIRE(doc.findObject(bitmap) != nullptr && doc.findObject(bitmap)->asBitmap() != nullptr,
               "the selected object is not a bitmap");
    const geometry::Box2 placed = doc.findObject(bitmap)->asBitmap()->placementMm;

    // 2. The bitmap is painted: dark over the left half of its placement.
    viewmodel::ViewTransform view;
    const QImage frame = window.canvas()->renderFitted(QSize(800, 400), &view);
    const viewmodel::CanvasScene scene = viewmodel::buildCanvasScene(doc, view);
    PH_REQUIRE(scene.bitmaps.size() == 1, "the scene lists no bitmap");
    const geometry::Vec2 left = view.worldToScreen(
        geometry::Vec2{placed.min.x + 0.25 * placed.width(), 0.5 * (placed.min.y + placed.max.y)});
    const QColor px = frame.pixelColor(QPoint(static_cast<int>(left.x), static_cast<int>(left.y)));
    PH_REQUIRE(px.red() < 60 && px.green() < 60 && px.blue() < 60, "the bitmap is not painted on the canvas");

    // 3. Add a Photo V-Carve through the form.
    window.documentChanged();
    const std::size_t steps = doc.undoStack().index();
    QAction* add = panel->addAction(document::ToolpathOperation::PhotoVCarve);
    PH_REQUIRE(add != nullptr && add->isEnabled(), "Add Photo V-Carve is missing or disabled");
    add->trigger();
    ToolpathFormDialog* dlg = panel->formDialog();
    PH_REQUIRE(dlg != nullptr, "Add Photo V-Carve opened no form");
    PH_REQUIRE(!dlg->okButton()->isEnabled(), "OK is enabled before a tool is chosen");
    PH_REQUIRE(dlg->pickTool("v-bit-90-1-4in"), "the V-bit cannot be picked");
    PH_REQUIRE(dlg->fieldEdit("maxDepth") != nullptr && dlg->fieldEdit("lineSpacing") != nullptr &&
                   dlg->fieldEdit("lineAngle") != nullptr && dlg->fieldEdit("stepoverRetract") != nullptr,
               "the form lacks a Max Carving Depth, Line Spacing, Line Angle or Stepover Retract field");
    PH_REQUIRE(!dlg->typeField("maxDepth", "0"), "a zero depth was accepted");
    PH_REQUIRE(dlg->typeField("maxDepth", "1") && dlg->typeField("lineSpacing", "100"), "valid entries rejected");
    PH_REQUIRE(dlg->chooseOption("strategy", 2), "the strategy cannot be set to Selected Vectors");
    PH_REQUIRE(!dlg->okButton()->isEnabled(), "Selected Vectors without vectors can be accepted");
    PH_REQUIRE(dlg->chooseOption("strategy", 1) && dlg->chooseOption("strategy", 0), "the strategy cannot be set to Raster");
    PH_REQUIRE(dlg->okButton()->isEnabled(), "OK is disabled for a complete photo form");
    PH_REQUIRE(accept(panel) && panel->busy(), "accepting the form did not start a calculation");
    PH_REQUIRE(panel->waitIdle(120000), "the calculation did not finish");
    PH_REQUIRE(doc.toolpaths().size() == 1 && doc.undoStack().index() == steps + 1,
               "adding the photo V-carve was not one undo step: " + panel->statusText().toStdString());
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    PH_REQUIRE(rec.operation() == document::ToolpathOperation::PhotoVCarve && rec.hasMoves(), "wrong record");
    PH_REQUIRE(deepest(rec) < -0.99 && deepest(rec) > -1.01, "the deepest cut is not the Max Carving Depth");

    // 4. Edit the depth: recalculated, one step.
    panel->selectToolpath(rec.id);
    panel->editAction()->trigger();
    dlg = panel->formDialog();
    PH_REQUIRE(dlg != nullptr && dlg->typeField("maxDepth", "0.5"), "the edit form rejected a depth of 0.5");
    PH_REQUIRE(accept(panel) && panel->waitIdle(120000), "the edit did not finish");
    PH_REQUIRE(doc.undoStack().index() == steps + 2 && deepest(doc.toolpaths().front()) > -0.51 &&
                   deepest(doc.toolpaths().front()) < -0.49,
               "editing the depth did not recalculate in one step: " + panel->statusText().toStdString());

    // 5. Moving the bitmap makes it stale; recalculate.
    PH_REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
                   std::vector<document::ObjectId>{bitmap}, geometry::Transform2::translation({3.0, 0.0}))),
               "could not move the bitmap");
    window.documentChanged();
    PH_REQUIRE(panel->list()->topLevelItem(0)->text(3) == QStringLiteral("Stale"), "moving the bitmap did not mark it stale");
    panel->selectToolpath(doc.toolpaths().front().id);
    panel->recalculateAction()->trigger();
    PH_REQUIRE(panel->waitIdle(120000) && panel->list()->topLevelItem(0)->text(3) == QStringLiteral("Valid"),
               "recalculation did not clear the stale state");

    // 6. Undo back to the imported bitmap (recalculation, move, edit, add), then redo.
    for (int i = 0; i < 4; ++i) {
        PH_REQUIRE(window.undo(), "undo failed");
    }
    PH_REQUIRE(doc.toolpaths().empty() && doc.bitmaps().size() == 1 && doc.findObject(bitmap) != nullptr &&
                   doc.findObject(bitmap)->asBitmap()->placementMm == placed,
               "undo did not leave just the imported bitmap");
    for (int i = 0; i < 4; ++i) {
        PH_REQUIRE(window.redo(), "redo failed");
    }
    PH_REQUIRE(doc.toolpaths().size() == 1 && panel->list()->topLevelItem(0)->text(3) == QStringLiteral("Valid"),
               "redo did not bring back a valid toolpath");

    // 7. Save, reload: bitmap, record and state come back.
    const std::filesystem::path file(dir.filePath(QStringLiteral("photo.carv")).toStdString());
    PH_REQUIRE(window.saveFileAs(file), "saving failed");
    PH_REQUIRE(window.openFile(file), "reloading failed");
    document::Document& loaded = window.currentDocument();
    PH_REQUIRE(loaded.bitmaps().size() == 1 && loaded.toolpaths().size() == 1, "the reloaded file lost the image or the toolpath");
    PH_REQUIRE(loaded.toolpaths().front().operation() == document::ToolpathOperation::PhotoVCarve &&
                   loaded.toolpaths().front().hasMoves(),
               "the reloaded record is wrong");
    PH_REQUIRE(window.toolpaths()->list()->topLevelItemCount() == 1 &&
                   window.toolpaths()->list()->topLevelItem(0)->text(3) == QStringLiteral("Valid"),
               "the reloaded toolpath is not valid");

    return std::string();
}

}  // namespace carvetto::app
