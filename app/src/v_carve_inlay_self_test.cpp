// Self-test of the V-carve inlay in the real app (T-0265), offscreen: the form of the Toolpaths dock
// (piece-dependent fields, V-bit only), a pocket and a mirrored plug of the same vector, calculation
// off the UI thread, undo and a save and reload.

#include <QAction>
#include <QDir>
#include <QPushButton>
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
#include "main_window.hpp"
#include "toolpath_form_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define VI_REQUIRE(cond, text)                              \
    do {                                                    \
        if (!(cond)) {                                      \
            return std::string("v-carve inlay: ") + (text); \
        }                                                   \
    } while (false)

geometry::Box2 cutBounds(const document::ToolpathRecord& rec) {
    geometry::Box2 b;
    if (rec.moves != nullptr) {
        for (const toolpath::Move& m : *rec.moves) {
            if (toolpath::isCutting(m.kind)) {
                b.expand(geometry::Vec2{m.to.x, m.to.y});
            }
        }
    }
    return b;
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

std::string vCarveInlaySelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 100.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    ToolpathsPanel* panel = window.toolpaths();
    VI_REQUIRE(panel != nullptr, "no Toolpaths panel");

    // 1. A rectangle selected.
    const document::ObjectId vec = doc.insertObject(document::Object::vector(
        geometry::Contour::rectangle(geometry::Box2{{40, 30}, {70, 50}}), doc.defaultLayer(), doc.activeSheet()));
    doc.selection().assign({vec});
    window.documentChanged();
    const std::size_t steps = doc.undoStack().index();

    // 2. The pocket through the form: V-bit only, no plug fields.
    QAction* add = panel->addAction(document::ToolpathOperation::VCarveInlay);
    VI_REQUIRE(add != nullptr && add->isEnabled(), "Add V-Carve Inlay is missing or disabled");
    add->trigger();
    ToolpathFormDialog* dlg = panel->formDialog();
    VI_REQUIRE(dlg != nullptr, "Add V-Carve Inlay opened no form");
    VI_REQUIRE(!dlg->okButton()->isEnabled(), "OK is enabled before a tool is chosen");
    VI_REQUIRE(!dlg->pickTool("end-mill-3mm"), "an end mill was accepted as the carving tool");
    VI_REQUIRE(dlg->pickTool("v-bit-90-1-4in"), "the V-bit cannot be picked");
    VI_REQUIRE(dlg->fieldEdit("pocketDepth") != nullptr && dlg->fieldEdit("glueGap") != nullptr,
               "the form lacks a Pocket Depth or Glue Gap field");
    VI_REQUIRE(dlg->fieldEdit("surfaceClearance") == nullptr && dlg->fieldEdit("plugSheet") == nullptr,
               "the pocket shows plug fields");
    VI_REQUIRE(!dlg->typeField("pocketDepth", "0"), "a zero pocket depth was accepted");
    VI_REQUIRE(dlg->typeField("pocketDepth", "1.5") && dlg->typeField("glueGap", "0.1"), "valid entries rejected");
    VI_REQUIRE(dlg->okButton()->isEnabled(), "OK is disabled for a complete form");
    VI_REQUIRE(accept(panel) && panel->busy(), "accepting the form did not start a calculation");
    VI_REQUIRE(panel->waitIdle(120000), "the calculation did not finish");
    VI_REQUIRE(doc.toolpaths().size() == 1 && doc.undoStack().index() == steps + 1,
               "adding the pocket was not one undo step: " + panel->statusText().toStdString());
    const document::ToolpathRecord pocket = doc.toolpaths().front();
    VI_REQUIRE(pocket.operation() == document::ToolpathOperation::VCarveInlay && pocket.hasMoves(), "wrong pocket record");
    const geometry::Box2 pocketBox = cutBounds(pocket);
    VI_REQUIRE(pocketBox.min.x >= 40.0 - 1e-6 && pocketBox.max.x <= 70.0 + 1e-6, "the pocket leaves the vector");

    // 3. The plug: plug fields appear, the carved region reaches past the vector.
    panel->addAction(document::ToolpathOperation::VCarveInlay)->trigger();
    dlg = panel->formDialog();
    VI_REQUIRE(dlg != nullptr && dlg->pickTool("v-bit-90-1-4in"), "the second form cannot be set up");
    VI_REQUIRE(dlg->chooseOption("piece", 1), "the piece cannot be set to Plug");
    VI_REQUIRE(dlg->fieldEdit("surfaceClearance") != nullptr && dlg->fieldEdit("plugSheet") != nullptr &&
                   dlg->fieldEdit("boundaryOffset") != nullptr,
               "the plug form lacks Surface Clearance, Plug Sheet or Boundary Offset");
    VI_REQUIRE(dlg->typeField("pocketDepth", "1.5") && dlg->typeField("glueGap", "0.1") &&
                   dlg->typeField("boundaryOffset", "6"),
               "valid plug entries rejected");
    VI_REQUIRE(!dlg->typeField("plugSheet", "1.5"), "a fractional sheet id was accepted");
    VI_REQUIRE(dlg->typeField("plugSheet", "0"), "sheet 0 (this sheet) was rejected");
    VI_REQUIRE(accept(panel) && panel->waitIdle(120000), "the plug calculation did not finish");
    VI_REQUIRE(doc.toolpaths().size() == 2 && doc.undoStack().index() == steps + 2,
               "adding the plug was not one undo step: " + panel->statusText().toStdString());
    const geometry::Box2 plugBox = cutBounds(doc.toolpaths().back());
    VI_REQUIRE(plugBox.min.x < 40.0 && plugBox.max.x > 70.0 && plugBox.max.y > 50.0, "the plug does not carve round the vector");

    // 4. Undo both, redo, save and reload.
    VI_REQUIRE(window.undo() && window.undo() && doc.toolpaths().empty(), "undo did not remove the toolpaths");
    VI_REQUIRE(window.redo() && window.redo() && doc.toolpaths().size() == 2, "redo did not bring them back");
    QTemporaryDir dir;
    VI_REQUIRE(dir.isValid(), "no temporary directory");
    const std::filesystem::path file(dir.filePath(QStringLiteral("inlay.carv")).toStdString());
    VI_REQUIRE(window.saveFileAs(file), "saving failed");
    VI_REQUIRE(window.openFile(file), "reloading failed");
    document::Document& loaded = window.currentDocument();
    VI_REQUIRE(loaded.toolpaths().size() == 2 &&
                   loaded.toolpaths().front().operation() == document::ToolpathOperation::VCarveInlay &&
                   loaded.toolpaths().front().hasMoves() &&
                   std::get<toolpath::VCarveInlayParams>(loaded.toolpaths().back().params).piece ==
                       toolpath::VCarveInlayPiece::Plug,
               "the reloaded records are wrong");
    VI_REQUIRE(window.toolpaths()->list()->topLevelItemCount() == 2 &&
                   window.toolpaths()->list()->topLevelItem(0)->text(3) == QStringLiteral("Valid"),
               "the reloaded toolpaths are not valid");
    return std::string();
}

}  // namespace carvetto::app
