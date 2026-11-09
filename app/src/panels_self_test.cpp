// Self-test of the Properties, Layers, Sheets and History panels (T-0210): drives the real
// MainWindow offscreen through the panels' widgets and actions, like a user without scripts.

#include <QAction>
#include <QComboBox>
#include <QCoreApplication>
#include <QDockWidget>
#include <QListWidget>
#include <QString>
#include <QTreeWidget>
#include <cmath>
#include <memory>
#include <string>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/contour.hpp"
#include "document_panels.hpp"
#include "main_window.hpp"
#include "properties_panel.hpp"

namespace carvetto::app {

namespace {

#define PT_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("panels: ") + (text);    \
        }                                               \
    } while (false)

document::ObjectId addRectangle(document::Document& doc, double cx, double cy, double w, double h) {
    const document::RectangleParams params{{cx, cy}, w, h};
    auto built = document::buildContours(params);
    if (!built.ok() || !doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                           built.value(), doc.activeLayer(), doc.activeSheet(), params)))) {
        return {};
    }
    return doc.objectIds().back();
}

const document::RectangleParams* rectOf(const document::Document& doc, document::ObjectId id) {
    const document::Object* obj = doc.findObject(id);
    const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
    return vec != nullptr ? std::get_if<document::RectangleParams>(&vec->params) : nullptr;
}

}  // namespace

std::string panelsSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    LayersPanel* layers = window.layersPanel();
    SheetsPanel* sheets = window.sheetsPanel();
    PropertiesPanel* props = window.propertiesPanel();
    HistoryPanel* history = window.historyPanel();
    PT_REQUIRE(layers != nullptr && sheets != nullptr && props != nullptr && history != nullptr, "a panel is missing");
    PT_REQUIRE(window.propertiesDock() != nullptr && window.historyDock() != nullptr, "a dock is missing");
    PT_REQUIRE(layers->rowCount() == 1 && sheets->rowCount() == 1 && history->rowCount() == 1,
               "the panels do not list the new document");
    PT_REQUIRE(props->title() == "No selection" && props->fieldIds().empty(), "properties of nothing are not empty");

    // --- Layers through the widgets -------------------------------------------------------
    PT_REQUIRE(layers->addLayer("Cutouts"), "adding a layer failed");
    document::Document* doc = &window.currentDocument();
    PT_REQUIRE(doc->layers().size() == 2 && layers->rowCount() == 2, "the new layer is not listed");
    PT_REQUIRE(window.undoAction()->text().contains(QStringLiteral("Add layer")), "Edit > Undo does not name the step");
    PT_REQUIRE(layers->currentRow() == 1 && doc->activeLayer() == doc->layers()[1].id, "the new layer is not active");

    // Rename by editing the name cell, hide and lock by clicking the check boxes.
    QTreeWidgetItem* item = layers->tree()->topLevelItem(1);
    item->setText(0, QStringLiteral("Pockets"));
    PT_REQUIRE(doc->layers()[1].name == "Pockets", "renaming through the list did not reach the document");
    layers->tree()->topLevelItem(1)->setCheckState(1, Qt::Unchecked);
    PT_REQUIRE(!doc->layers()[1].visible, "unchecking Show did not hide the layer");
    layers->tree()->topLevelItem(1)->setCheckState(2, Qt::Checked);
    PT_REQUIRE(doc->layers()[1].locked, "checking Lock did not lock the layer");
    const document::Colour teal{1, 2, 3};
    PT_REQUIRE(layers->setColour(1, teal) && doc->layers()[1].colour == teal, "setting the colour failed");
    PT_REQUIRE(!layers->renameLayer(1, ""), "an empty layer name was accepted");
    PT_REQUIRE(doc->layers()[1].name == "Pockets", "a refused rename changed the layer");
    PT_REQUIRE(layers->setLocked(1, false) && layers->setVisible(1, true), "unlocking / showing failed");
    PT_REQUIRE(layers->moveLayer(1, 0) && doc->layers()[0].name == "Pockets", "reordering layers failed");
    PT_REQUIRE(layers->tree()->topLevelItem(0)->text(0) == QStringLiteral("Pockets"), "the list did not follow the reorder");
    PT_REQUIRE(window.undo() && doc->layers()[0].name == "Layer 1", "undoing the reorder failed");

    // --- Properties of a selected rectangle -------------------------------------------------
    const document::ObjectId rect = addRectangle(*doc, 100, 100, 40, 20);
    PT_REQUIRE(!rect.isNull(), "could not add a rectangle");
    doc->selection().assign({rect});
    window.documentChanged();
    PT_REQUIRE(props->title() == "Rectangle", "the properties title does not name the shape");
    PT_REQUIRE(props->textOf("width") == "40.00" && props->textOf("height") == "20.00", "the size fields are wrong");
    const std::size_t steps = doc->undoStack().count();
    PT_REQUIRE(props->commit("width", "60") && rectOf(*doc, rect)->widthMm == 60.0, "editing the width failed");
    PT_REQUIRE(doc->undoStack().count() == steps + 1, "a width edit is not one undo step");
    PT_REQUIRE(window.undoAction()->text().contains(QStringLiteral("Edit Rectangle")), "Edit > Undo does not name the edit");
    PT_REQUIRE(history->rowCount() == static_cast<int>(doc->undoStack().count()) + 1, "the history list is out of date");
    PT_REQUIRE(!props->commit("width", "abc") && rectOf(*doc, rect)->widthMm == 60.0, "a bad number was applied");
    PT_REQUIRE(props->textOf("width") == "60.00", "a refused entry did not restore the old value");
    PT_REQUIRE(!props->commit("width", "-3") && !props->commit("width", "nan"), "a negative or NaN width was accepted");
    PT_REQUIRE(doc->undoStack().count() == steps + 1, "a refused edit added an undo step");
    PT_REQUIRE(props->commit("x", "10") && props->commit("y", "5"), "editing the position failed");
    PT_REQUIRE(doc->findObject(rect)->asVector()->bounds().min.x == 10.0, "the X edit did not move the object");
    PT_REQUIRE(props->commit("name", "Panel") && doc->findObject(rect)->name == "Panel", "renaming the object failed");
    // Move to the other layer through the Layer combo (index 1).
    PT_REQUIRE(props->commit("layer", "1") && doc->findObject(rect)->layer == doc->layers()[1].id,
               "moving the object to another layer failed");

    // A locked layer blocks edits of its objects.
    PT_REQUIRE(layers->setLocked(1, true), "locking the layer failed");
    PT_REQUIRE(!props->commit("width", "70") && rectOf(*doc, rect)->widthMm == 60.0, "a locked object was edited");
    PT_REQUIRE(layers->setLocked(1, false), "unlocking the layer failed");

    // --- History: jump back and forward ---------------------------------------------------
    const std::size_t total = doc->undoStack().count();
    PT_REQUIRE(history->jumpTo(steps), "jumping in the history failed");
    PT_REQUIRE(rectOf(*doc, rect)->widthMm == 40.0 && doc->undoStack().index() == steps &&
                   doc->findObject(rect)->name.empty(),
               "the history jump did not stop at the step");
    PT_REQUIRE(history->currentRow() == static_cast<int>(steps), "the history list does not show the current step");
    PT_REQUIRE(history->jumpTo(total) && doc->findObject(rect)->name == "Panel", "jumping forward failed");
    PT_REQUIRE(!history->jumpTo(total + 5), "a jump beyond the history was accepted");

    // --- Delete layer with objects ---------------------------------------------------------
    PT_REQUIRE(layers->deleteLayer(1, false) && doc->layers().size() == 1 &&
                   doc->findObject(rect)->layer == doc->layers()[0].id,
               "deleting a layer did not move its objects");
    PT_REQUIRE(!layers->deleteLayer(0, false), "the last layer was deleted");

    // --- Sheets ---------------------------------------------------------------------------
    PT_REQUIRE(sheets->addSheet("Back") && doc->sheets().size() == 2 && sheets->rowCount() == 2, "adding a sheet failed");
    PT_REQUIRE(doc->activeSheet() == doc->sheets()[1].id, "the new sheet is not active");
    PT_REQUIRE(sheets->setMaterial(1, "400", "250 mm", "", 1, 4), "editing the material block failed");
    PT_REQUIRE(doc->sheets()[1].sizeMm.x == 400.0 && doc->sheets()[1].sizeMm.y == 250.0 &&
                   doc->sheets()[1].zZero == document::ZZero::MachineBed &&
                   doc->sheets()[1].datum == document::Datum::Centre,
               "the material block has the wrong values");
    PT_REQUIRE(doc->sheets()[0].sizeMm.x == 300.0, "the other sheet's material changed");
    PT_REQUIRE(!sheets->setMaterial(1, "0", "", ""), "a zero material width was accepted");
    PT_REQUIRE(!sheets->setMaterial(1, "nan", "", ""), "a NaN material width was accepted");
    PT_REQUIRE(!sheets->setMaterial(1, "1e30", "", ""), "a huge material width was accepted");
    sheets->list()->item(1)->setText(QStringLiteral("Back side"));
    PT_REQUIRE(doc->sheets()[1].name == "Back side", "renaming a sheet through the list failed");
    sheets->setCurrentRow(0);
    PT_REQUIRE(doc->activeSheet() == doc->sheets()[0].id, "picking a sheet did not activate it");
    PT_REQUIRE(sheets->deleteSheet(1) && doc->sheets().size() == 1, "deleting a sheet failed");
    PT_REQUIRE(!sheets->deleteSheet(0), "the last sheet was deleted");
    PT_REQUIRE(doc->checkIntegrity().empty(), "the document is inconsistent after the panel edits");

    // --- Replacing the document: panels follow, stale ids do nothing -------------------------
    doc->selection().assign({rect});
    window.documentChanged();
    PT_REQUIRE(props->title() == "Rectangle", "the properties did not show the selection");
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 200.0, 10.0, document::Units::Inches)));
    QCoreApplication::processEvents();
    PT_REQUIRE(props->title() == "No selection" && props->fieldIds().empty(), "properties kept the old selection");
    PT_REQUIRE(layers->rowCount() == 1 && sheets->rowCount() == 1 && history->rowCount() == 1,
               "the panels kept the old document's rows");
    PT_REQUIRE(!props->commit("width", "5"), "a stale properties edit was accepted");
    PT_REQUIRE(!layers->renameLayer(5, "x") && !layers->deleteLayer(7, true), "an edit of a missing layer was accepted");
    PT_REQUIRE(!history->jumpTo(3), "a jump in an empty history was accepted");
    // Leave the plain 300 mm job the next sections expect.
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    return std::string();
}

}  // namespace carvetto::app
