// Self-test of the Toolpaths dock (T-0221): drives the real MainWindow offscreen through the dock's
// actions and the form dialog's widgets.

#include <QAction>
#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QImage>
#include <QPushButton>
#include <QTreeWidget>
#include <cmath>
#include <memory>
#include <string>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "main_window.hpp"
#include "preview_panel.hpp"
#include "relief_panels.hpp"
#include "toolpath_form_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define ST_REQUIRE(cond, text)                 \
    do {                                       \
        if (!(cond)) {                         \
            return std::string("toolpaths: ") + (text); \
        }                                      \
    } while (false)

document::ObjectId addRect(document::Document& doc, double x, double y, double w, double h) {
    const bool ok = doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
        geometry::Contour::rectangle(geometry::Box2{{x, y}, {x + w, y + h}}), doc.activeLayer(), doc.activeSheet())));
    return ok ? doc.objectIds().back() : document::ObjectId{};
}

QTreeWidgetItem* rowOf(ToolpathsPanel* panel, int row) {
    return row < panel->list()->topLevelItemCount() ? panel->list()->topLevelItem(row) : nullptr;
}

QString stateOf(ToolpathsPanel* panel, int row) {
    const QTreeWidgetItem* item = rowOf(panel, row);
    return item == nullptr ? QString() : item->text(3);
}

double profileDepth(const document::Document& doc, std::size_t index) {
    return std::get<toolpath::ProfileParams>(doc.toolpaths().at(index).params).passes.cutDepth;
}

bool accept(ToolpathsPanel* panel);

/// Adds a Pocket on the (large) selected vectors that takes a while to calculate; returns its id.
document::ToolpathId addHeavyPocket(MainWindow& window, document::ObjectId big) {
    document::Document& doc = window.currentDocument();
    ToolpathsPanel* panel = window.toolpaths();
    doc.selection().assign({big});
    window.documentChanged();
    panel->addAction(document::ToolpathOperation::Pocket)->trigger();
    ToolpathFormDialog* dlg = panel->formDialog();
    if (dlg == nullptr || !dlg->pickTool("end-mill-3mm") || !dlg->typeField("cutDepth", "12") || !accept(panel) ||
        !panel->waitIdle(300000)) {
        return document::ToolpathId{};
    }
    return doc.toolpaths().empty() ? document::ToolpathId{} : doc.toolpaths().back().id;
}

/// Confirms the open form like the OK button.
bool accept(ToolpathsPanel* panel) {
    ToolpathFormDialog* dlg = panel->formDialog();
    if (dlg == nullptr || !dlg->okButton()->isEnabled()) {
        return false;
    }
    dlg->accept();
    return true;
}

}  // namespace

std::string toolpathsSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    ToolpathsPanel* panel = window.toolpaths();
    ST_REQUIRE(panel != nullptr && window.toolpathsDock()->widget() == panel, "the dock holds no ToolpathsPanel");
    ST_REQUIRE(panel->list()->topLevelItemCount() == 0, "the list is not empty for a new document");

    // 1. Two rectangles selected, a Profile toolpath added through the form.
    const document::ObjectId a = addRect(doc, 40, 40, 60, 40);
    const document::ObjectId b = addRect(doc, 140, 40, 40, 30);
    ST_REQUIRE(!a.isNull() && !b.isNull(), "could not add the rectangles");
    doc.selection().assign({a, b});
    window.documentChanged();
    const std::size_t stepsBefore = doc.undoStack().index();

    ST_REQUIRE(panel->addAction(document::ToolpathOperation::Profile)->isEnabled(), "Add Profile is disabled");
    panel->addAction(document::ToolpathOperation::Profile)->trigger();
    ToolpathFormDialog* dlg = panel->formDialog();
    ST_REQUIRE(dlg != nullptr, "Add Profile opened no form");
    ST_REQUIRE(!dlg->okButton()->isEnabled(), "OK is enabled before a tool and depth are chosen");
    ST_REQUIRE(dlg->pickTool("end-mill-3mm"), "the 3 mm end mill cannot be picked");
    ST_REQUIRE(dlg->fieldEdit("cutDepth") != nullptr, "the form has no cut depth field");
    ST_REQUIRE(!dlg->typeField("cutDepth", "abc"), "a non-numeric depth was accepted");
    ST_REQUIRE(!dlg->okButton()->isEnabled() && !dlg->message().isEmpty(), "bad input shows no inline message");
    ST_REQUIRE(!dlg->typeField("cutDepth", "-2"), "a negative depth was accepted");
    ST_REQUIRE(dlg->typeField("cutDepth", "3"), "a valid depth was rejected");
    ST_REQUIRE(dlg->okButton()->isEnabled(), "OK is disabled for a complete form");
    ST_REQUIRE(accept(panel), "accepting the form failed");
    ST_REQUIRE(panel->busy(), "the panel is not busy after accepting");
    ST_REQUIRE(!panel->addAction(document::ToolpathOperation::Pocket)->isEnabled() && panel->cancelAction()->isEnabled(),
               "actions are not disabled while busy");
    ST_REQUIRE(panel->waitIdle(120000), "the calculation did not finish");
    ST_REQUIRE(doc.toolpaths().size() == 1 && doc.undoStack().index() == stepsBefore + 1,
               "adding a Profile was not exactly one undo step: " + panel->statusText().toStdString());
    ST_REQUIRE(doc.toolpaths().front().hasMoves() && profileDepth(doc, 0) == 3.0, "the toolpath has no moves or wrong depth");
    ST_REQUIRE(panel->list()->topLevelItemCount() == 1 && stateOf(panel, 0) == QStringLiteral("Valid"),
               "the list does not show a valid toolpath");
    ST_REQUIRE(panel->summaryLabel()->text().contains(QStringLiteral("1 toolpath")), "the summary is not updated");

    // 2. Edit the depth.
    const auto movesBefore = doc.toolpaths().front().moves;
    panel->selectToolpath(doc.toolpaths().front().id);
    ST_REQUIRE(panel->editAction()->isEnabled(), "Edit is disabled with a toolpath selected");
    panel->editAction()->trigger();
    dlg = panel->formDialog();
    ST_REQUIRE(dlg != nullptr, "Edit opened no form");
    ST_REQUIRE(dlg->typeField("cutDepth", "5"), "the edit form rejected a depth of 5");
    ST_REQUIRE(accept(panel) && panel->waitIdle(120000), "the edit did not finish");
    ST_REQUIRE(doc.toolpaths().size() == 1 && profileDepth(doc, 0) == 5.0 && doc.toolpaths().front().moves != movesBefore,
               "editing the depth did not replace the record: " + panel->statusText().toStdString());
    ST_REQUIRE(doc.undoStack().index() == stepsBefore + 2, "the edit was not one undo step");

    // 3. Moving a source rectangle marks the toolpath stale; recalculate fixes it.
    ST_REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
                   std::vector<document::ObjectId>{a}, geometry::Transform2::translation({10, 0}))),
               "could not move a rectangle");
    window.documentChanged();
    ST_REQUIRE(stateOf(panel, 0) == QStringLiteral("Stale"), "moving a vector did not mark the toolpath stale");
    panel->selectToolpath(doc.toolpaths().front().id);
    panel->recalculateAction()->trigger();
    ST_REQUIRE(panel->busy() && panel->waitIdle(120000), "recalculate did not finish");
    ST_REQUIRE(stateOf(panel, 0) == QStringLiteral("Valid"), "recalculation did not clear the stale state");

    // 4. Clicking the toolpath selects its vectors.
    doc.selection().clear();
    window.documentChanged();
    panel->list()->setCurrentItem(nullptr);
    panel->list()->setCurrentItem(rowOf(panel, 0));
    ST_REQUIRE(doc.selection().size() == 2 && doc.selection().contains(a) && doc.selection().contains(b),
               "clicking the toolpath did not select its two vectors");

    // 5. Undo (the recalculation, the move, the edit, the add) through the window.
    ST_REQUIRE(window.undo() && window.undo() && window.undo(), "undo failed");
    ST_REQUIRE(profileDepth(doc, 0) == 3.0 && stateOf(panel, 0) == QStringLiteral("Valid"),
               "undoing the edit did not restore the depth of 3");
    ST_REQUIRE(window.undo() && doc.toolpaths().empty() && panel->list()->topLevelItemCount() == 0,
               "undoing the add left a toolpath behind");
    ST_REQUIRE(window.redo() && doc.toolpaths().size() == 1 && panel->list()->topLevelItemCount() == 1,
               "redo did not bring the toolpath back");

    // 6. A refusal is shown inline and changes nothing: a 6 mm pocket does not fit a 4 mm square.
    const document::ObjectId tiny = addRect(doc, 200, 200, 4, 4);
    ST_REQUIRE(!tiny.isNull(), "could not add the tiny square");
    doc.selection().assign({tiny});
    window.documentChanged();
    const std::size_t steps = doc.undoStack().index();
    panel->addAction(document::ToolpathOperation::Pocket)->trigger();
    dlg = panel->formDialog();
    ST_REQUIRE(dlg != nullptr && dlg->pickTool("end-mill-6mm") && dlg->typeField("cutDepth", "2"), "pocket form setup failed");
    ST_REQUIRE(accept(panel) && panel->waitIdle(120000), "the pocket calculation did not finish");
    ST_REQUIRE(doc.undoStack().index() == steps && doc.toolpaths().size() == 1,
               "a refused pocket changed the document");
    ST_REQUIRE(!panel->statusText().isEmpty() && panel->diagnostics()->count() > 0,
               "a refused pocket shows no message");

    // A heavy pocket for the mid-calculation tests (long enough for the worker to be still running).
    const document::ObjectId big = addRect(doc, 20, 100, 260, 190);
    ST_REQUIRE(!big.isNull(), "could not add the big rectangle");
    const document::ToolpathId heavy = addHeavyPocket(window, big);
    ST_REQUIRE(!heavy.isNull(), "could not add the heavy pocket: " + panel->statusText().toStdString());

    // 7. Stale result: the vectors change while the job runs; the result is refused with a message.
    doc.selection().clear();
    panel->selectToolpath(heavy);
    const std::size_t before7 = doc.undoStack().index();
    const auto record7 = doc.toolpaths().back();
    panel->recalculateAction()->trigger();
    ST_REQUIRE(panel->busy(), "recalculate did not start");
    ST_REQUIRE(!window.undo(), "Undo ran during a calculation");
    ST_REQUIRE(!window.undoAction()->isEnabled() && !window.redoAction()->isEnabled(),
               "Undo / Redo actions are enabled during a calculation");
    ST_REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
                   std::vector<document::ObjectId>{big}, geometry::Transform2::translation({0, -5}))),
               "could not edit during the calculation");
    ST_REQUIRE(panel->waitIdle(120000), "the stale calculation did not finish");
    ST_REQUIRE(doc.undoStack().index() == before7 + 1 && doc.toolpaths().back() == record7,
               "a stale result was applied: " + panel->statusText().toStdString());
    ST_REQUIRE(!panel->statusText().isEmpty(), "a refused stale result shows no message");
    window.documentChanged();

    // 8. Cancel: either cancelled (nothing changes) or already done (one step), never both.
    {
        ST_REQUIRE(window.undo() && window.redoAction()->isEnabled(), "could not set up a redoable step");
        const std::size_t n = doc.undoStack().index();
        panel->selectToolpath(heavy);
        panel->recalculateAction()->trigger();
        ST_REQUIRE(panel->busy(), "recalculate did not start (cancel test)");
        ST_REQUIRE(!window.redo() && !window.redoAction()->isEnabled(), "Redo ran or is enabled during a calculation");
        panel->cancelAction()->trigger();
        ST_REQUIRE(panel->waitIdle(120000), "the cancelled calculation did not finish");
        const std::size_t now = doc.undoStack().index();
        ST_REQUIRE(now == n || now == n + 1, "cancel left an inconsistent state");
    }

    // 9. The document is replaced while a calculation runs: the result is discarded, nothing of the
    // old document is touched.
    panel->selectToolpath(heavy);
    panel->recalculateAction()->trigger();
    ST_REQUIRE(panel->busy(), "recalculate did not start (replace test)");
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 200.0, 18.0, document::Units::Millimetres)));
    ST_REQUIRE(panel->waitIdle(120000), "the replaced-document calculation did not finish");
    ST_REQUIRE(window.currentDocument().toolpaths().empty() && panel->list()->topLevelItemCount() == 0 &&
                   window.currentDocument().undoStack().index() == 0,
               "a result was applied to the replacement document");
    ST_REQUIRE(panel->statusText().contains(QStringLiteral("replaced")), "no message about the replaced document");
    ST_REQUIRE(panel->addAction(document::ToolpathOperation::Profile)->isEnabled(), "the panel stayed disabled");

    // 10. The window is closed (destroyed) while a calculation runs.
    {
        MainWindow other;
        other.setQuiet(true);
        other.show();
        document::Document& d = other.currentDocument();
        const document::ObjectId r = addRect(d, 10, 10, 280, 280);
        d.selection().assign({r});
        other.documentChanged();
        ToolpathsPanel* p = other.toolpaths();
        p->addAction(document::ToolpathOperation::Pocket)->trigger();
        ToolpathFormDialog* f = p->formDialog();
        if (f == nullptr || !f->pickTool("end-mill-3mm") || !f->typeField("cutDepth", "12") || !accept(p)) {
            return "toolpaths: could not set up the close-window test";
        }
        if (!p->busy()) {
            return "toolpaths: the close-window test did not start a calculation";
        }
        other.close();
    }  // destroyed while the worker may still run: it is cancelled and joined
    QCoreApplication::processEvents();
    return std::string();
}

std::string toolpaths3dSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    ToolpathsPanel* panel = window.toolpaths();
    ST_REQUIRE(panel != nullptr, "3D: the dock holds no ToolpathsPanel");

    // Without a relief model the form opens but cannot be accepted.
    ST_REQUIRE(panel->addAction(document::ToolpathOperation::Rough3d) != nullptr &&
                   panel->addAction(document::ToolpathOperation::Finish3d) != nullptr,
               "3D: the Add menu has no 3D Rough / 3D Finish entry");
    panel->addAction(document::ToolpathOperation::Rough3d)->trigger();
    ToolpathFormDialog* dlg = panel->formDialog();
    ST_REQUIRE(dlg != nullptr, "3D: Add 3D Rough opened no form");
    ST_REQUIRE(dlg->pickTool("end-mill-3mm"), "3D: the 3 mm end mill cannot be picked");
    ST_REQUIRE(!dlg->okButton()->isEnabled(), "3D: OK is enabled without a relief model");
    dlg->reject();

    // A wavy model, 61 x 61 cells of 1 mm, 0 .. 6 mm high (not a plain ramp: a .carv file holds at most one
    // move per byte, and a perfectly regular model compresses to almost nothing).
    auto level = std::make_unique<document::AddReliefLevelCommand>(doc.activeSheet(), "Level");
    const document::AddReliefLevelCommand* rawLevel = level.get();
    ST_REQUIRE(doc.execute(std::move(level)), "3D: could not add a level");
    relief::Component c;
    c.name = "Waves";
    std::vector<float> data;
    for (int j = 0; j < 61; ++j) {
        for (int i = 0; i < 61; ++i) {
            data.push_back(static_cast<float>(3.0 + 3.0 * std::sin(i * 0.21) * std::cos(j * 0.17)));
        }
    }
    ST_REQUIRE(relief::HeightField::fromData(relief::Grid{61, 61, 1.0, {20.0, 20.0}}, std::move(data), c.field) ==
                   relief::Status::Ok,
               "3D: could not build the model");
    ST_REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(rawLevel->createdId(), std::move(c))),
               "3D: could not add the model");
    window.documentChanged();
    const std::size_t steps = doc.undoStack().index();

    // 3D Rough through the form: the stock defaults come from the job.
    panel->addAction(document::ToolpathOperation::Rough3d)->trigger();
    dlg = panel->formDialog();
    ST_REQUIRE(dlg != nullptr && dlg->pickTool("end-mill-3mm"), "3D: no rough form or tool");
    ST_REQUIRE(dlg->fieldEdit("stockBottom") != nullptr && dlg->fieldEdit("stockBottom")->text() == QStringLiteral("-18"),
               "3D: the stock bottom does not default to the job thickness");
    ST_REQUIRE(dlg->fieldEdit("zOffset") != nullptr && dlg->fieldEdit("zOffset")->text().toDouble() < -5.5 &&
                   dlg->fieldEdit("zOffset")->text().toDouble() > -6.01,
               "3D: the model Z offset does not put the model top at the stock top");
    ST_REQUIRE(!dlg->typeField("stockTop", "abc"), "3D: a non-numeric stock top was accepted");
    ST_REQUIRE(dlg->typeField("stockTop", "0") && dlg->okButton()->isEnabled(), "3D: the complete form cannot be accepted");
    ST_REQUIRE(accept(panel) && panel->waitIdle(300000), "3D: the rough calculation did not finish");
    ST_REQUIRE(doc.toolpaths().size() == 1 && doc.undoStack().index() == steps + 1 &&
                   doc.toolpaths().front().operation() == document::ToolpathOperation::Rough3d &&
                   doc.toolpaths().front().moveCount() > 10,
               "3D: adding a 3D Rough was not one undo step with moves: " + panel->statusText().toStdString());

    // 3D Finish.
    panel->addAction(document::ToolpathOperation::Finish3d)->trigger();
    dlg = panel->formDialog();
    ST_REQUIRE(dlg != nullptr && dlg->pickTool("end-mill-3mm") && dlg->typeField("tolerance", "0.05"),
               "3D: no finish form or tool");
    ST_REQUIRE(accept(panel) && panel->waitIdle(300000), "3D: the finish calculation did not finish");
    ST_REQUIRE(doc.toolpaths().size() == 2 && doc.toolpaths()[1].operation() == document::ToolpathOperation::Finish3d,
               "3D: adding a 3D Finish failed: " + panel->statusText().toStdString());

    // The Model view draws the toolpaths over the surface.
    ModelView3D* view = window.modelView();
    ST_REQUIRE(view != nullptr, "3D: no Model view");
    view->resize(320, 240);
    view->camera().viewTop();
    view->refresh();
    const QImage& picture = view->image();
    std::size_t red = 0;
    for (int y = 0; y < picture.height(); ++y) {
        for (int x = 0; x < picture.width(); ++x) {
            if (picture.pixel(x, y) == 0xFFD02020u) {
                ++red;
            }
        }
    }
    ST_REQUIRE(red > 50, "3D: the Model view does not show the toolpath");

    // Simulation with the existing simulator.
    PreviewPanel* preview = window.preview();
    ST_REQUIRE(preview != nullptr && preview->startSimulation() && preview->waitSimulation(300000),
               "3D: the simulation did not run: " + (preview != nullptr ? preview->statusLabel()->text().toStdString() : std::string()));

    // One undo step removes the finish, the model stays.
    ST_REQUIRE(window.undo() && doc.toolpaths().size() == 1 && !doc.reliefLevels().empty(), "3D: undo did not remove the finish");
    return std::string();
}

}  // namespace carvetto::app
