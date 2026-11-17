// Self-test of the Auto-Wrapping View (T-0334): drives the View menu toggle, the Model view, the simulation view and
// the rotary post dialog of the real MainWindow offscreen.

#include "wrapped_view_self_test.hpp"

#include <QAction>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QListWidget>
#include <QPushButton>
#include <QTemporaryDir>
#include <QThread>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/rotary_post_report.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"
#include "main_window.hpp"
#include "preview_panel.hpp"
#include "relief_panels.hpp"
#include "relief_view.hpp"
#include "rotary_post_dialog.hpp"
#include "save_toolpaths_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define WV_REQUIRE(cond, text)                         \
    do {                                               \
        if (!(cond)) {                                 \
            return std::string("wrapped view: ") + (text); \
        }                                              \
    } while (false)

constexpr QRgb kVectorColour = 0xFF1A5FB4u;

int countColour(const QImage& image, QRgb colour) {
    int n = 0;
    for (int y = 0; y < image.height(); ++y) {
        const QRgb* row = reinterpret_cast<const QRgb*>(image.constScanLine(y));
        for (int x = 0; x < image.width(); ++x) {
            if (row[x] == colour) {
                ++n;
            }
        }
    }
    return n;
}

bool sameImage(const QImage& a, const QImage& b) {
    return a.size() == b.size() && a == b;
}

}  // namespace

std::string wrappedViewSelfTest(MainWindow& window) {
    window.setQuiet(true);
    QAction* action = window.wrappedViewAction();
    WV_REQUIRE(action != nullptr && action->isCheckable(), "View > Wrapped View is missing");

    // 1. A flat job has no wrapped view.
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 100.0, 18.0, document::Units::Millimetres)));
    window.documentChanged();
    WV_REQUIRE(!action->isEnabled(), "Wrapped View is enabled for a flat job");
    WV_REQUIRE(!window.modelView()->wrappedAvailable() && !window.modelView()->showingWrapped(),
               "the Model view offers the wrapped view for a flat job");

    // 2. A rotary job with a vector and a profile toolpath.
    window.setDocument(std::make_unique<document::Document>(
        document::Job::rotary(60.0, 20.0, document::Units::Millimetres, document::RotaryAxis::X,
                                                  document::ZZero::MaterialSurface)));
    document::Document& doc = window.currentDocument();
    WV_REQUIRE(doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                   geometry::Contour::rectangle(geometry::Box2{{10, 10}, {50, 50}}), doc.activeLayer(),
                   doc.activeSheet()))),
               "could not add a vector");
    {
        viewmodel::ToolpathSession& session = window.toolpaths()->session();
        viewmodel::ToolpathForm form = session.newForm(document::ToolpathOperation::Profile);
        WV_REQUIRE(form.setTool(session.toolDatabase(), document::ToolSource{"end-mill-3mm", {}, {}}).ok &&
                       form.setField("cutDepth", "2").ok,
                   "the profile form rejected the tool or depth");
        const viewmodel::ToolpathOutcome out = session.add(form.definition());
        WV_REQUIRE(out.ok, "adding the profile failed: " + out.message);
    }
    window.documentChanged();
    WV_REQUIRE(action->isEnabled() && action->isChecked() && window.wrappedView(),
               "Wrapped View is not on and enabled for a rotary job");

    // 3. The Model view: wrapped by default, flat after the toggle; vectors are drawn on the cylinder.
    ModelView3D* model = window.modelView();
    model->resize(480, 360);
    model->refresh();
    WV_REQUIRE(model->wrappedAvailable() && model->showingWrapped(), "the Model view is not wrapped for a rotary job");
    model->camera().viewTop();
    model->cameraChanged();
    const QImage wrappedModel = model->image();
    WV_REQUIRE(countColour(wrappedModel, kVectorColour) > 20, "no vector is drawn on the wrapped model");
    action->setChecked(false);
    WV_REQUIRE(!window.wrappedView() && !model->showingWrapped(), "the toggle did not switch the Model view to flat");
    const QImage flatModel = model->image();
    WV_REQUIRE(!sameImage(wrappedModel, flatModel), "the flat and wrapped Model pictures are the same");
    // The view's own toggle button drives the same state.
    auto* button = model->findChild<QPushButton*>(QStringLiteral("wrappedToggle"));
    WV_REQUIRE(button != nullptr && button->isCheckable(), "the Model view has no wrapped toggle button");
    button->click();
    WV_REQUIRE(window.wrappedView() && action->isChecked() && model->showingWrapped(),
               "the Model view's button did not turn the wrapped view on");
    WV_REQUIRE(sameImage(model->image(), wrappedModel), "the wrapped Model picture is not repeatable");

    // 4. The simulation, played wrapped.
    PreviewPanel* preview = window.preview();
    WV_REQUIRE(preview->startSimulation() && preview->waitSimulation(120000), "the simulation did not run");
    ReliefView* relief = window.reliefView();
    relief->resize(480, 360);
    WV_REQUIRE(relief->showingWrapped(), "the simulation view is not wrapped for a rotary job");
    relief->camera().viewTop();
    relief->cameraChanged();
    const auto meta = preview->simulation().meta();
    WV_REQUIRE(meta && meta->moveCount > 0, "the simulation has no moves");
    const std::size_t lastMove = meta->moveCount;
    const QImage wrappedStart = relief->wrappedImage();  // the uncut cylinder (move 0)
    WV_REQUIRE(!wrappedStart.isNull() && wrappedStart.width() > 100, "no wrapped simulation picture");
    preview->scrubTo(lastMove);
    WV_REQUIRE(preview->waitSimulation(120000), "the wrapped simulation did not seek to the end");
    WV_REQUIRE(relief->frame() && relief->frame()->move == lastMove && relief->frame()->cutCells > 0,
               "the wrapped simulation cut nothing");
    const QImage wrappedEnd = relief->wrappedImage();
    WV_REQUIRE(!sameImage(wrappedStart, wrappedEnd), "playing the simulation does not change the wrapped picture");
    preview->scrubTo(0);
    WV_REQUIRE(preview->waitSimulation(120000), "the wrapped simulation did not seek back to the start");
    WV_REQUIRE(sameImage(relief->wrappedImage(), wrappedStart), "the wrapped start picture is not repeatable");
    preview->scrubTo(lastMove);
    WV_REQUIRE(preview->waitSimulation(120000) && sameImage(relief->wrappedImage(), wrappedEnd),
               "the wrapped end picture is not repeatable");
    action->setChecked(false);
    WV_REQUIRE(!relief->showingWrapped() && relief->hasPicture(), "the simulation view did not go flat");
    action->setChecked(true);
    WV_REQUIRE(relief->showingWrapped(), "the simulation view did not go wrapped again");
    preview->clearSimulation();

    // 5. The post dialog: a post that does not wrap is a warning, one that wraps the wrong axis a refusal.
    viewmodel::ToolpathSession& session = window.toolpaths()->session();
    QTemporaryDir tmp;
    WV_REQUIRE(tmp.isValid(), "no temporary folder");
    const std::filesystem::path outPath = viewmodel::pathFromUtf8(tmp.path().toStdString());
    {
        SaveToolpathsDialog dialog(viewmodel::makeSaveSource(session), "job", {}, outPath);
        dialog.setAcceptStale(true);
        WV_REQUIRE(dialog.waitIdle() && dialog.selectPost("grbl_mm.pp") && dialog.waitIdle() && dialog.output().ok,
                   "the Save Toolpaths dialog did not generate for the rotary job");
        WV_REQUIRE(dialog.rotaryReport().issues.size() == 1 && !dialog.rotaryReport().refused(),
                   "a post that does not wrap gave no rotary warning");
        WV_REQUIRE(!dialog.rotaryDetailsButton()->isHidden(), "the Rotary details button is hidden");
        const std::unique_ptr<RotaryPostDialog> details = dialog.makeRotaryDialog();
        WV_REQUIRE(details->issueCount() == 1 && !details->refused() && details->list()->count() == 2,
                   "the rotary dialog does not list the warning and its advice");
    }
    {
        // The cylinder lies along X: a post that wraps X onto the rotary axis is refused.
        std::ifstream in(shippedPostsDir() / "grbl_mm.pp", std::ios::binary);
        WV_REQUIRE(in.good(), "the shipped GRBL post is not found");
        std::ostringstream text;
        text << in.rdbuf();
        const std::filesystem::path postPath = outPath / "wrong_axis.pp";
        {
            std::ofstream out(postPath, std::ios::binary);
            out << text.str() << "\nROTARY_WRAP_X = \"A\"\n";
        }
        viewmodel::PostInfo post;
        post.name = "wrong_axis";
        post.path = postPath;
        post.extension = "nc";
        viewmodel::SaveOptions options;
        options.directory = outPath;
        options.jobName = "job";
        const viewmodel::SaveSource source = viewmodel::makeSaveSource(session);
        const viewmodel::SaveJob job = viewmodel::prepareSave(source, post, options);
        WV_REQUIRE(job.error.empty(), "the wrong-axis post could not be prepared: " + job.error);
        const viewmodel::SaveOutput out = viewmodel::generateOutput(job);
        WV_REQUIRE(!out.ok, "the post wrapping the cylinder's own axis was not refused");
        const viewmodel::RotaryPostReport report = viewmodel::rotaryPostReport(out, doc.job());
        WV_REQUIRE(report.refused() && report.issues.size() == 1, "the refusal is not in the rotary report");
        RotaryPostDialog refusal(report);
        WV_REQUIRE(refusal.refused() && refusal.issueCount() == 1 && refusal.list()->count() == 2 &&
                       refusal.list()->item(0)->text().startsWith(QStringLiteral("Refused: ")),
                   "the rotary dialog does not show the refusal");
    }
    return {};
}

}  // namespace carvetto::app
