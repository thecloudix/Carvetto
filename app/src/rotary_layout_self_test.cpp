// Self-test of the Wrapped Fluting / Spiral Layout dialogs (T-0340): drives the Setup menu actions, the real
// dialog widgets and the wrapped Model view of the MainWindow offscreen.

#include "rotary_layout_self_test.hpp"

#include <QAction>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QPushButton>
#include <cmath>
#include <memory>
#include <string>
#include <variant>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/rotary_layout_form.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"
#include "main_window.hpp"
#include "relief_panels.hpp"
#include "rotary_layout_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define RL_REQUIRE(cond, text)                              \
    do {                                                    \
        if (!(cond)) {                                      \
            return std::string("rotary layout: ") + (text); \
        }                                                   \
    } while (false)

std::size_t contourCount(const document::Document& doc, document::ObjectId id) {
    const document::Object* obj = doc.findObject(id);
    return obj != nullptr && obj->asVector() != nullptr ? obj->asVector()->contours.size() : 0;
}

}  // namespace

std::string rotaryLayoutSelfTest(MainWindow& window) {
    window.setQuiet(true);
    QAction* fluting = window.flutingLayoutAction();
    QAction* spiral = window.spiralLayoutAction();
    RL_REQUIRE(fluting != nullptr && spiral != nullptr, "the Setup menu has no layout entries");

    // 1. A flat job offers neither layout.
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 100.0, 18.0, document::Units::Millimetres)));
    window.documentChanged();
    RL_REQUIRE(!fluting->isEnabled() && !spiral->isEnabled(), "the layouts are enabled for a flat job");
    RL_REQUIRE(window.makeRotaryLayoutDialog(viewmodel::RotaryLayoutKind::Fluting) == nullptr,
               "a flat job opened a layout dialog");

    // 2. A rotary job enables both.
    window.setDocument(std::make_unique<document::Document>(
        document::Job::rotary(120.0, 40.0, document::Units::Millimetres, document::RotaryAxis::X,
                              document::ZZero::MaterialSurface)));
    window.documentChanged();
    document::Document& doc = window.currentDocument();
    RL_REQUIRE(fluting->isEnabled() && spiral->isEnabled(), "the layouts are disabled for a rotary job");
    ModelView3D* model = window.modelView();
    model->resize(480, 360);
    model->refresh();
    RL_REQUIRE(model->showingWrapped() && !model->hasLayoutPreview(), "the Model view is not wrapped or has a stale preview");
    const std::size_t runsBefore = model->wrappedPathRuns();

    // 3. Fluting: 6 centre-line flutes of depth 3 mm; the preview is drawn wrapped; one undo step.
    {
        RotaryLayoutDialog* dlg = window.makeRotaryLayoutDialog(viewmodel::RotaryLayoutKind::Fluting);
        RL_REQUIRE(dlg != nullptr, "no Fluting dialog for a rotary job");
        RL_REQUIRE(dlg->canAccept() && dlg->lastPreview().contours.size() == 4, "the default Fluting preview is not 4 flutes");
        RL_REQUIRE(model->hasLayoutPreview() && model->wrappedPathRuns() == runsBefore + 4,
                   "the default preview is not drawn on the wrapped Model view");
        RL_REQUIRE(dlg->typeField("flutes", "6") && dlg->typeField("depth", "3"), "the Fluting fields were refused");
        RL_REQUIRE(dlg->lastPreview().contours.size() == 6 && model->wrappedPathRuns() == runsBefore + 6,
                   "the preview did not follow the flute count");
        // A bad entry is refused, the field keeps its value and the form is unchanged.
        RL_REQUIRE(!dlg->typeField("flutes", "0") && dlg->fieldEdit("flutes")->text() == QStringLiteral("6") &&
                       !dlg->message().isEmpty(),
                   "a zero flute count was accepted or left in the field");
        RL_REQUIRE(dlg->buttons()->button(QDialogButtonBox::Ok)->isEnabled(), "OK is disabled for a valid layout");
        const std::size_t steps = doc.undoStack().index();
        RL_REQUIRE(window.applyRotaryLayout(dlg->form()), "applying the Fluting layout failed");
        delete dlg;
        RL_REQUIRE(!model->hasLayoutPreview(), "the preview outlived the dialog");
        RL_REQUIRE(doc.undoStack().index() == steps + 1, "the Fluting layout was not exactly one undo step");
        RL_REQUIRE(doc.selection().ids().size() == 1 && contourCount(doc, doc.selection().ids().front()) == 6,
                   "the layout is not one selected object of 6 vectors");
        // The Fluting toolpath form for this selection starts at the layout's depth.
        viewmodel::ToolpathSession& session = window.toolpaths()->session();
        const viewmodel::ToolpathForm form = session.newForm(document::ToolpathOperation::Fluting);
        const auto* params = std::get_if<toolpath::FlutingParams>(&form.definition().params);
        RL_REQUIRE(params != nullptr && std::fabs(params->fluteDepth - 3.0) < 1e-9,
                   "the Fluting toolpath form does not take the layout's depth");
        window.documentChanged();
        RL_REQUIRE(model->wrappedPathRuns() == runsBefore + 6, "the applied vectors are not drawn wrapped");
        // Without the layout selected the form keeps its own default.
        doc.selection().clear();
        const viewmodel::ToolpathForm plain = session.newForm(document::ToolpathOperation::Fluting);
        const auto* plainParams = std::get_if<toolpath::FlutingParams>(&plain.definition().params);
        RL_REQUIRE(plainParams != nullptr && std::fabs(plainParams->fluteDepth - 3.0) > 1e-9,
                   "an unrelated selection took the layout's depth");
        RL_REQUIRE(doc.undo() && doc.objectIds().empty(), "undo did not remove the Fluting layout");
        window.documentChanged();
    }

    // 4. Spiral: choose the twist mode, one strand over 240 degrees; a refused entry leaves the preview intact.
    {
        RotaryLayoutDialog* dlg = window.makeRotaryLayoutDialog(viewmodel::RotaryLayoutKind::Spiral);
        RL_REQUIRE(dlg != nullptr && dlg->canAccept(), "no usable Spiral dialog for a rotary job");
        RL_REQUIRE(dlg->choiceBox("mode") != nullptr && dlg->chooseOption("mode", 2), "the spiral has no mode choice");
        RL_REQUIRE(dlg->fieldEdit("twist") != nullptr && dlg->fieldEdit("pitch") == nullptr,
                   "the fields did not follow the spiral mode");
        RL_REQUIRE(dlg->typeField("strands", "2") && dlg->typeField("twist", "720") && dlg->chooseOption("direction", 1),
                   "the Spiral fields were refused");
        RL_REQUIRE(dlg->canAccept() && !dlg->lastPreview().contours.empty() && std::fabs(dlg->lastPreview().turns - 2.0) < 1e-6,
                   "the spiral preview does not show 2 turns");
        RL_REQUIRE(!dlg->typeField("alongEnd", "abc") && dlg->canAccept(), "a non-number extent was accepted");
        // A valid entry that makes the layout impossible disables OK; fixing it brings OK back.
        RL_REQUIRE(dlg->typeField("alongEnd", "-5") && !dlg->canAccept() &&
                       !dlg->buttons()->button(QDialogButtonBox::Ok)->isEnabled() && !model->hasLayoutPreview(),
                   "an end before the start left the layout acceptable");
        RL_REQUIRE(dlg->typeField("alongEnd", "120") && dlg->canAccept() && model->hasLayoutPreview(),
                   "restoring the extent did not restore the preview");
        const std::size_t steps = doc.undoStack().index();
        RL_REQUIRE(window.applyRotaryLayout(dlg->form()), "applying the Spiral layout failed");
        delete dlg;
        RL_REQUIRE(doc.undoStack().index() == steps + 1, "the Spiral layout was not exactly one undo step");
        RL_REQUIRE(doc.undo() && doc.objectIds().empty(), "undo did not remove the Spiral layout");
        window.documentChanged();
    }
    return {};
}

}  // namespace carvetto::app
