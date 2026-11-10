// Self-test of double-sided jobs (T-0261): drives the real MainWindow offscreen through Job Setup, the
// Edit > Double-sided actions, the multi-sided view, copy to the other side, per-side Save Toolpaths
// output and a .carv round trip.

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QImage>
#include <QKeySequence>
#include <QTemporaryDir>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/double_sided.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/double_sided_ops.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"
#include "double_sided_menu.hpp"
#include "main_window.hpp"
#include "new_job_dialog.hpp"
#include "toolpaths_panel.hpp"

#ifndef CARVETTO_SHIPPED_POSTS_DIR
#define CARVETTO_SHIPPED_POSTS_DIR ""
#endif

namespace carvetto::app {

namespace {

#define DS_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("double-sided: ") + (text); \
        }                                               \
    } while (false)

std::string readAll(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

std::string doubleSidedSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(document::Job::singleSided(
        200.0, 100.0, 18.0, document::Units::Millimetres, document::Datum::BottomLeft, document::ZZero::MachineBed)));
    document::Document& doc = window.currentDocument();
    DoubleSidedMenu* menu = window.doubleSided();
    ToolpathsPanel* panel = window.toolpaths();
    DS_REQUIRE(menu != nullptr, "the Double-sided menu is missing");
    DS_REQUIRE(!menu->swapSidesAction()->isEnabled() && !menu->singleSidedAction()->isEnabled(),
               "the side actions are enabled for a single-sided job");
    DS_REQUIRE(menu->topSideAction()->shortcut() == QKeySequence(Qt::Key_1) &&
                   menu->bottomSideAction()->shortcut() == QKeySequence(Qt::Key_2) &&
                   menu->multiSidedAction()->shortcut() == QKeySequence(Qt::Key_Equal),
               "the keys 1, 2 and = are not bound");

    // 1. Job Setup page: the double-sided fields reach the job, and applying it pairs the sheets.
    NewJobDialog dialog(viewmodel::formFromJob(doc.job()), &window);
    auto* flip = dialog.findChild<QComboBox*>(QStringLiteral("flipDirection"));
    auto* zeroOff = dialog.findChild<QCheckBox*>(QStringLiteral("zeroOffSameSide"));
    DS_REQUIRE(flip != nullptr && zeroOff != nullptr, "Job Setup has no flip / zero-off-same-side controls");
    DS_REQUIRE(!zeroOff->isEnabled(), "zero off same side is enabled for a single-sided job");
    viewmodel::NewJobForm form = dialog.form();
    form.type = document::JobType::DoubleSided;
    form.flip = document::FlipDirection::Horizontal;
    form.zeroOffSameSide = true;
    dialog.setForm(form);
    DS_REQUIRE(zeroOff->isEnabled() && zeroOff->isChecked(), "zero off same side is not editable for a double-sided job");
    dialog.accept();
    DS_REQUIRE(dialog.job().has_value() && dialog.job()->zeroOffSameSide &&
                   dialog.job()->type == document::JobType::DoubleSided,
               "the dialog did not produce a double-sided job");
    const std::size_t steps = doc.undoStack().count();
    const viewmodel::EditResult setup = viewmodel::applyJobSetup(doc, *dialog.job());
    DS_REQUIRE(setup.ok && doc.undoStack().count() == steps + 1, "Job Setup is not one undo step");
    window.documentChanged();
    DS_REQUIRE(doc.sheets().size() == 2 && doc.sheets()[1].side == document::Side::Back &&
                   doc.sheets()[0].partner == doc.sheets()[1].id,
               "Job Setup did not pair a back sheet");
    const document::SheetId front = doc.sheets()[0].id;
    const document::SheetId back = doc.sheets()[1].id;
    DS_REQUIRE(menu->swapSidesAction()->isEnabled() && menu->singleSidedAction()->isEnabled(),
               "the side actions are disabled for a double-sided job");

    // 2. A rectangle on the top side, a Profile on it, copied to the bottom side.
    DS_REQUIRE(doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                   geometry::Contour::rectangle(geometry::Box2{{20, 20}, {70, 60}}), doc.activeLayer(), doc.activeSheet()))),
               "could not add the rectangle");
    const document::ObjectId rect = doc.objectIds().back();
    viewmodel::ToolpathForm profile(document::ToolpathOperation::Profile, document::Units::Millimetres);
    DS_REQUIRE(profile.setTool(panel->toolDatabase(), document::ToolSource{"end-mill-3mm", {}, {}}).ok &&
                   profile.setField("cutDepth", "3").ok,
               "could not fill the Profile form");
    doc.selection().assign({rect});
    window.documentChanged();
    DS_REQUIRE(panel->session().add(profile.definition()).ok, "the Profile toolpath could not be added");
    window.documentChanged();
    DS_REQUIRE(doc.toolpaths().size() == 1 && doc.toolpaths()[0].sheet == front, "the toolpath is not linked to the top sheet");

    doc.selection().assign({rect});
    menu->copyObjectsAction()->trigger();
    DS_REQUIRE(doc.objectsOnSheet(back).size() == 1, "Copy Selection to Other Side did not copy the rectangle");
    const auto mirrored = doc.objectBounds(doc.objectsOnSheet(back).front());
    DS_REQUIRE(mirrored.has_value() && std::abs(mirrored->min.x - 130.0) < 1e-6 && std::abs(mirrored->max.x - 180.0) < 1e-6,
               "the copy is not mirrored about the material centre");
    panel->selectToolpath(doc.toolpaths()[0].id);
    menu->copyToolpathAction()->trigger();
    DS_REQUIRE(doc.toolpaths().size() == 2 && doc.toolpaths()[1].sheet == back, "Copy Toolpath to Other Side made no back toolpath");

    // 3. Swap Sides and the keys 1 / 2; the multi-sided view paints something extra.
    menu->swapSidesAction()->trigger();
    DS_REQUIRE(doc.activeSheet() == back, "Swap Sides did not activate the bottom side");
    menu->topSideAction()->trigger();
    DS_REQUIRE(doc.activeSheet() == front, "key 1 did not activate the top side");
    menu->bottomSideAction()->trigger();
    DS_REQUIRE(doc.activeSheet() == back, "key 2 did not activate the bottom side");
    menu->topSideAction()->trigger();
    const QImage plain = window.canvas()->renderFitted(QSize(500, 300));
    menu->multiSidedAction()->setChecked(true);
    DS_REQUIRE(window.canvas()->multiSidedView(), "the multi-sided view did not switch on");
    const QImage multi = window.canvas()->renderFitted(QSize(500, 300));
    DS_REQUIRE(plain != multi, "the multi-sided view draws nothing for the other side");
    const auto outlines = viewmodel::multiSidedView(doc);
    DS_REQUIRE(outlines.size() == 2 && !outlines[0].otherSideContent.isEmpty(), "multiSidedView has no other-side content");
    menu->multiSidedAction()->setChecked(false);

    // 4. Save Toolpaths: one file per side, each with its own Z zero (bed zero + same-side zero:
    // top side cut face at Z 18, bottom side at Z 0; cut depth 3).
    QTemporaryDir dir;
    DS_REQUIRE(dir.isValid(), "no temporary directory");
    const auto posts = viewmodel::discoverPosts(viewmodel::pathFromUtf8(CARVETTO_SHIPPED_POSTS_DIR), {});
    const viewmodel::PostInfo* grbl = nullptr;
    for (const viewmodel::PostInfo& p : posts) {
        if (p.path.filename() == "grbl_mm.pp") {
            grbl = &p;
        }
    }
    DS_REQUIRE(grbl != nullptr, "the grbl post is missing");
    viewmodel::SaveOptions options;
    options.directory = viewmodel::pathFromUtf8(dir.path().toStdString());
    options.jobName = "part";
    const viewmodel::SaveJob job = viewmodel::prepareSave(viewmodel::makeSaveSource(panel->session()), *grbl, options);
    const viewmodel::SaveOutput out = viewmodel::generateOutput(job);
    DS_REQUIRE(out.ok && out.files.size() == 2, "Save Toolpaths did not make one file per side: " + out.error);
    DS_REQUIRE(out.files[0].path.filename() == "part_front.nc" && out.files[1].path.filename() == "part_back.nc",
               "the side files are not named per side");
    DS_REQUIRE(out.files[0].text.find("Z15") != std::string::npos && out.files[0].text.find("Z-3") == std::string::npos,
               "the top side file does not use the shifted Z zero");
    DS_REQUIRE(out.files[1].text.find("Z-3") != std::string::npos && out.files[1].text.find("Z15") == std::string::npos,
               "the bottom side file does not cut to Z-3");
    DS_REQUIRE(viewmodel::writeOutput(out, false).ok, "writing the side files failed");

    // 5. A .carv round trip keeps the pairing and the toolpaths' sheet links.
    const std::filesystem::path carv = options.directory / "sides.carv";
    DS_REQUIRE(window.saveFileAs(carv), "saving the .carv failed");
    DS_REQUIRE(window.openFile(carv), "reopening the .carv failed");
    document::Document& loaded = window.currentDocument();
    DS_REQUIRE(loaded.job().type == document::JobType::DoubleSided && loaded.sheets().size() == 2 &&
                   loaded.toolpaths().size() == 2 && loaded.toolpaths()[0].sheet == loaded.sheets()[0].id &&
                   loaded.toolpaths()[1].sheet == loaded.sheets()[1].id,
               "the reopened document lost the sides or the toolpath links");

    // 6. Back to single-sided, in one undo step.
    const std::size_t before = loaded.undoStack().count();
    window.doubleSided()->singleSidedAction()->trigger();
    DS_REQUIRE(loaded.job().type == document::JobType::SingleSided && loaded.undoStack().count() == before + 1 &&
                   loaded.sheets()[0].partner.isNull() && loaded.sheets()[1].side == document::Side::Front,
               "switching back to single-sided did not unpair the sheets in one step");
    DS_REQUIRE(!window.doubleSided()->swapSidesAction()->isEnabled(), "Swap Sides is enabled for a single-sided job");
    window.undo();
    DS_REQUIRE(window.currentDocument().job().type == document::JobType::DoubleSided, "undo did not restore the double-sided job");
    return {};
}

}  // namespace carvetto::app
