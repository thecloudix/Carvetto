// Self-test of document variables (T-0257): drives the real Document Variables dialog and the
// toolpath form offscreen. A variable defined from the job width is used in a toolpath's depth box,
// changed (the toolpath recalculates in the same undo step), refused deletion while used (also while
// the form is open), import / export, and clear errors in the calculation box.

#include <QAction>
#include <QCoreApplication>
#include <QLineEdit>
#include <QPushButton>
#include <QTemporaryDir>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/viewmodel/variables_model.hpp"
#include "main_window.hpp"
#include "toolpath_form_dialog.hpp"
#include "toolpaths_panel.hpp"
#include "variable_menu.hpp"
#include "variables_dialog.hpp"

namespace carvetto::app {

namespace {

#define VS_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("variables: ") + (text); \
        }                                               \
    } while (false)

double pocketDepth(const document::Document& doc, std::size_t index) {
    return std::get<toolpath::PocketParams>(doc.toolpaths().at(index).params).cutDepth;
}

}  // namespace

std::string variablesSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    ToolpathsPanel* panel = window.toolpaths();
    VS_REQUIRE(window.variablesAction() != nullptr && window.variablesAction()->text().contains(QStringLiteral("Variables")),
               "Edit > Document Variables is missing");

    VS_REQUIRE(doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                   geometry::Contour::rectangle(geometry::Box2{{40, 40}, {100, 80}}), doc.activeLayer(),
                   doc.activeSheet()))),
               "could not add the rectangle");
    doc.selection().assign({doc.objectIds().back()});
    window.documentChanged();
    doc.undoStack().clear();

    viewmodel::VariablesModel model(&doc, &panel->session());
    model.setDeferRecalculation(true);
    model.setBusyCheck([panel]() { return panel->busy(); });
    VariablesDialog vars(model, &window);
    vars.onRecalculate = [panel]() { panel->recalculateBound(); };
    int edits = 0;
    vars.onEdited = [&edits]() { ++edits; };

    // 1. Create variables, one defined from the job width (letter variable W).
    VS_REQUIRE(vars.setVariable("Margin", "=W/10"), "creating Margin = W/10 failed: " + vars.message().toStdString());
    VS_REQUIRE(doc.variables().get("Margin") == 30.0, "Margin is not 30 (W = 300): " + std::to_string(doc.variables().get("Margin").value_or(-1.0)));
    VS_REQUIRE(doc.undoStack().index() == 1 && edits == 1, "creating a variable is not one undo step");
    VS_REQUIRE(vars.setVariable("Depth", "3"), "creating Depth failed");
    VS_REQUIRE(vars.rowCount() == 2, "the table does not list the variables");
    VS_REQUIRE(!vars.setVariable("1bad", "1") && !vars.message().isEmpty(), "an invalid name was accepted");
    VS_REQUIRE(!vars.setVariable("Bad", "x") && !vars.setVariable("Bad", "={Nope}"), "an invalid value was accepted");
    VS_REQUIRE(doc.undoStack().index() == 2 && vars.rowCount() == 2, "a refused edit changed the document");
    VS_REQUIRE(vars.editValueCell("Margin", "12") && doc.variables().get("Margin") == 12.0,
               "editing the value cell did not change the variable");
    VS_REQUIRE(doc.undoStack().index() == 3, "a cell edit is not one undo step");

    // 1b. Insert New Document Variable works on any numeric box (T-0271), not only the toolpath forms.
    {
        QLineEdit box;
        attachVariableMenu(&box);
        box.setText(QStringLiteral("5"));
        box.setCursorPosition(1);
        VS_REQUIRE(insertNewVariable(&box, "Gap", "2").empty() && doc.variables().get("Gap") == 2.0 &&
                       box.text() == QStringLiteral("=5{Gap}"),
                   "Insert New Document Variable on a plain numeric box failed");
        VS_REQUIRE(!insertNewVariable(&box, "9bad", "1").empty(), "an invalid name was accepted on a plain box");
        VS_REQUIRE(window.undo() && !doc.variables().contains("Gap"), "creating the variable is not an undo step");
        VS_REQUIRE(window.redo() && doc.variables().contains("Gap"), "redo did not restore the variable");
    }

    // 2. A Pocket whose depth box holds ={Depth}; the open form holds the variable.
    panel->addAction(document::ToolpathOperation::Pocket)->trigger();
    ToolpathFormDialog* dlg = panel->formDialog();
    VS_REQUIRE(dlg != nullptr && dlg->pickTool("end-mill-3mm"), "the Pocket form did not open");
    VS_REQUIRE(!dlg->typeField("cutDepth", "={Nope}") && !dlg->message().isEmpty(),
               "an unknown variable in a box shows no error");
    VS_REQUIRE(!dlg->typeField("cutDepth", "=1/0") && !dlg->okButton()->isEnabled(), "a division by zero was accepted");
    VS_REQUIRE(dlg->typeField("cutDepth", "={Depth}"), "={Depth} was rejected: " + dlg->message().toStdString());
    VS_REQUIRE(doc.variables().referenceCount("Depth") == 1, "the open form does not hold Depth");
    VS_REQUIRE(!vars.deleteVariable("Depth") && doc.variables().contains("Depth"),
               "a variable used by an open form was deleted");
    // Insert New Document Variable on the start depth box.
    VS_REQUIRE(dlg->insertNewVariable("startDepth", "Top", "0.5").empty(), "Insert New Document Variable failed");
    VS_REQUIRE(doc.variables().get("Top") == 0.5 && dlg->fieldEdit("startDepth")->text() == QStringLiteral("={Top}"),
               "the new variable was not created or inserted");
    VS_REQUIRE(!dlg->insertNewVariable("startDepth", "9bad", "1").empty(), "an invalid new variable name was accepted");
    VS_REQUIRE(dlg->okButton()->isEnabled(), "OK is disabled for a complete form");
    const std::size_t before = doc.undoStack().index();
    dlg->accept();
    VS_REQUIRE(panel->waitIdle(120000) && doc.toolpaths().size() == 1, "the pocket was not calculated");
    VS_REQUIRE(pocketDepth(doc, 0) == 3.0, "the pocket depth is not the value of Depth");
    VS_REQUIRE(doc.undoStack().index() == before + 1, "adding the toolpath is not one undo step");

    // 3. The toolpath now holds the variables: deletion is refused, and says which toolpath.
    VS_REQUIRE(doc.variables().referenceCount("Depth") == 1 && doc.variables().referenceCount("Top") == 1,
               "the toolpath does not hold its variables");
    VS_REQUIRE(!vars.deleteVariable("Depth") && vars.message().contains(QStringLiteral("used by")),
               "a variable used by a toolpath was deleted or the refusal is unclear");
    VS_REQUIRE(doc.variables().contains("Depth"), "the used variable is gone");

    // 4. Changing the variable recalculates the toolpath in the background (T-0271): the variable
    //    edit is one undo step and the recalculation a second one.
    const auto movesBefore = doc.toolpaths().front().moves;
    const std::size_t steps = doc.undoStack().index();
    VS_REQUIRE(vars.setVariable("Depth", "5"), "changing Depth failed: " + vars.message().toStdString());
    VS_REQUIRE(panel->busy() || pocketDepth(doc, 0) == 5.0, "the recalculation did not start");
    VS_REQUIRE(!vars.setVariable("Depth", "6"), "an edit was accepted while recalculating");
    VS_REQUIRE(panel->waitIdle(120000), "the recalculation did not finish");
    VS_REQUIRE(pocketDepth(doc, 0) == 5.0 && doc.toolpaths().front().moves != movesBefore,
               "the toolpath was not recalculated for the new Depth");
    VS_REQUIRE(doc.undoStack().index() == steps + 2, "the variable edit and the recalculation are not two undo steps");
    VS_REQUIRE(window.undo() && window.undo() && doc.variables().get("Depth") == 3.0 && pocketDepth(doc, 0) == 3.0,
               "undo did not restore the variable and the toolpath");
    VS_REQUIRE(window.redo() && window.redo() && pocketDepth(doc, 0) == 5.0, "redo did not restore Depth = 5");

    // 5. Editing the toolpath shows the expression; closing the form releases its hold.
    panel->selectToolpath(doc.toolpaths().front().id);
    panel->editAction()->trigger();
    dlg = panel->formDialog();
    VS_REQUIRE(dlg != nullptr && dlg->fieldEdit("cutDepth") != nullptr &&
                   dlg->fieldEdit("cutDepth")->text() == QStringLiteral("={Depth}"),
               "the edit form does not show the expression");
    VS_REQUIRE(doc.variables().referenceCount("Depth") == 2, "an open edit form does not add its hold");
    dlg->reject();
    VS_REQUIRE(doc.variables().referenceCount("Depth") == 1, "closing the form did not release its hold");

    // 6. Export, then import overwrites same-named variables, in one undo step.
    QTemporaryDir dir;
    VS_REQUIRE(dir.isValid(), "no temporary directory");
    const std::filesystem::path file = std::filesystem::path(dir.path().toStdString()) / "vars.txt";
    VS_REQUIRE(vars.exportTo(file), "export failed: " + vars.message().toStdString());
    VS_REQUIRE(vars.setVariable("Margin", "99") && doc.variables().get("Margin") == 99.0, "changing Margin failed");
    const std::size_t beforeImport = doc.undoStack().index();
    VS_REQUIRE(vars.importFrom(file) && doc.variables().get("Margin") == 12.0, "import did not overwrite Margin");
    VS_REQUIRE(doc.undoStack().index() == beforeImport + 1, "import is not one undo step");
    VS_REQUIRE(!vars.importFrom(file.parent_path() / "missing.txt"), "importing a missing file succeeded");

    // 7. Deleting the toolpath frees the variables; then deletion works and is undoable.
    VS_REQUIRE(panel->session().deleteToolpaths({doc.toolpaths().front().id}).ok, "could not delete the toolpath");
    VS_REQUIRE(vars.deleteVariable("Depth") && !doc.variables().contains("Depth"),
               "a variable no longer used could not be deleted: " + vars.message().toStdString());
    VS_REQUIRE(window.undo() && doc.variables().contains("Depth"), "undo did not restore the deleted variable");
    return {};
}

}  // namespace carvetto::app
