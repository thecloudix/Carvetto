// Self-test of the toolpath preview and the simulation view (T-0223): drives the real MainWindow
// offscreen through the Preview dock's widgets, the canvas overlay and the simulation view.

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QImage>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSlider>
#include <QThread>
#include <QTreeWidget>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <variant>

#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"
#include "main_window.hpp"
#include "preview_panel.hpp"
#include "relief_view.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define PT_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("preview: ") + (text);   \
        }                                               \
    } while (false)

document::ObjectId addRect(document::Document& doc, double x, double y, double w, double h) {
    const bool ok = doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
        geometry::Contour::rectangle(geometry::Box2{{x, y}, {x + w, y + h}}), doc.activeLayer(), doc.activeSheet())));
    return ok ? doc.objectIds().back() : document::ObjectId{};
}

/// Pixels of `image` within `tol` (per channel) of `c`.
int countNear(const QImage& image, const viewmodel::PreviewColour& c, int tol = 30) {
    int n = 0;
    for (int y = 0; y < image.height(); ++y) {
        const QRgb* row = reinterpret_cast<const QRgb*>(image.constScanLine(y));
        for (int x = 0; x < image.width(); ++x) {
            const QRgb p = row[x];
            if (std::abs(qRed(p) - c.r) <= tol && std::abs(qGreen(p) - c.g) <= tol && std::abs(qBlue(p) - c.b) <= tol) {
                ++n;
            }
        }
    }
    return n;
}

/// Adds a Profile toolpath (3 mm end mill, cut depth `depth`) on the whole drawing through the session.
bool addProfile(MainWindow& window, double depth, std::string* why) {
    viewmodel::ToolpathSession& session = window.toolpaths()->session();
    viewmodel::ToolpathForm form = session.newForm(document::ToolpathOperation::Profile);
    if (!form.setTool(session.toolDatabase(), document::ToolSource{"end-mill-3mm", {}, {}}).ok ||
        !form.setField("cutDepth", std::to_string(depth)).ok) {
        *why = "the profile form rejected the tool or depth";
        return false;
    }
    const viewmodel::ToolpathOutcome out = session.add(form.definition());
    if (!out.ok) {
        *why = "adding the profile failed: " + out.message;
        return false;
    }
    window.documentChanged();
    return true;
}

bool pump(int ms) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(2);
    }
    return true;
}

}  // namespace

std::string previewSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    PreviewPanel* panel = window.preview();
    CanvasWidget* canvas = window.canvas();
    ReliefView* relief = window.reliefView();
    PT_REQUIRE(panel != nullptr && relief != nullptr && window.previewDock()->widget() == panel,
               "the window has no Preview dock");
    PT_REQUIRE(panel->model().enabled() && panel->model().scene().toolpaths.empty(), "bad initial preview state");
    PT_REQUIRE(!panel->simulateButton()->isEnabled() || !panel->startSimulation(),
               "a simulation started without toolpaths");
    PT_REQUIRE(!panel->statusLabel()->text().isEmpty(), "no message for a simulation without toolpaths");

    // 1. A toolpath appears on the canvas; the preview toggles on and off.
    const document::ObjectId a = addRect(doc, 40, 40, 60, 40);
    const document::ObjectId b = addRect(doc, 160, 60, 40, 30);
    PT_REQUIRE(!a.isNull() && !b.isNull(), "could not add the rectangles");
    doc.selection().clear();
    std::string why;
    PT_REQUIRE(addProfile(window, 3.0, &why), why);
    PT_REQUIRE(doc.toolpaths().size() == 1 && panel->model().scene().toolpaths.size() == 1,
               "the preview does not know the new toolpath");
    const QSize size(900, 700);
    const viewmodel::PreviewColour feed = viewmodel::colourOf(viewmodel::MoveClass::Feed);
    QImage on = canvas->renderFitted(size);
    const int feedOn = countNear(on, feed);
    PT_REQUIRE(feedOn > 100, "no feed-coloured toolpath on the canvas");
    panel->showAction()->setChecked(false);
    PT_REQUIRE(!panel->model().enabled() && !panel->showCheck()->isChecked(), "the toggle did not reach the checkbox");
    const int feedOff = countNear(canvas->renderFitted(size), feed);
    PT_REQUIRE(feedOff + 100 < feedOn, "turning the preview off left the toolpath on the canvas");
    panel->showCheck()->setChecked(true);
    PT_REQUIRE(panel->model().enabled() && panel->showAction()->isChecked(), "the checkbox did not reach the action");
    PT_REQUIRE(countNear(canvas->renderFitted(size), feed) > 100, "turning the preview on did not draw the toolpath");

    // 2. Per-toolpath visibility and selection.
    QTreeWidgetItem* row = panel->toolpathList()->topLevelItem(0);
    PT_REQUIRE(row != nullptr, "the visibility list is empty");
    row->setCheckState(0, Qt::Unchecked);
    PT_REQUIRE(countNear(canvas->renderFitted(size), feed) + 100 < feedOn, "hiding the toolpath did not hide it");
    row->setCheckState(0, Qt::Checked);
    panel->selectedOnlyCheck()->setChecked(true);
    PT_REQUIRE(countNear(canvas->renderFitted(size), feed) + 100 < feedOn, "selected-only drew an unselected toolpath");
    window.toolpaths()->selectToolpath(doc.toolpaths().front().id);
    PT_REQUIRE(panel->model().selected() == doc.toolpaths().front().id, "selecting the toolpath did not select it in the preview");
    PT_REQUIRE(countNear(canvas->renderFitted(size), feed) > 100, "the selected toolpath is not drawn");
    panel->selectedOnlyCheck()->setChecked(false);
    doc.selection().clear();
    window.documentChanged();
    panel->classCheck(viewmodel::MoveClass::Feed)->setChecked(false);
    PT_REQUIRE(countNear(canvas->renderFitted(size), feed) + 100 < feedOn, "hiding the feed class did not hide feed moves");
    panel->classCheck(viewmodel::MoveClass::Feed)->setChecked(true);

    // 3. The time readout; the rapid rate completes it.
    const QString before = panel->timeLabel()->text();
    PT_REQUIRE(before.contains(QStringLiteral("Estimated machining time")), "no time readout");
    PT_REQUIRE(before.contains(QStringLiteral("rapid")), "the readout does not say that rapids are left out");
    panel->rapidRate()->setValue(3000.0);
    PT_REQUIRE(!panel->timeLabel()->text().contains(QStringLiteral("without rapids")),
               "setting the rapid rate did not complete the estimate");
    PT_REQUIRE(window.toolpaths()->session().machineSetup().rapidRateMmPerMin == 3000.0, "the rate did not reach the session");

    // 4. Simulation: start, scrub, step, play.
    panel->qualityCombo()->setCurrentIndex(0);  // coarse
    PT_REQUIRE(panel->startSimulation(), "the simulation did not start: " + panel->statusLabel()->text().toStdString());
    PT_REQUIRE(panel->waitSimulation(120000), "the first simulation frame never arrived: " + panel->statusLabel()->text().toStdString());
    PT_REQUIRE(relief->hasPicture() && panel->simulationCurrent(), "no relief picture after the simulation started");
    const std::size_t total = panel->playback().moveCount();
    PT_REQUIRE(total == doc.toolpaths().front().moveCount() && total > 10, "the playback does not cover the toolpath");
    window.showView(true);
    PT_REQUIRE(window.simulationViewShown(), "the simulation view did not show");
    const auto& moves = *doc.toolpaths().front().moves;
    geometry::Vec2 cutPoint{0, 0};
    bool found = false;
    for (const toolpath::Move& m : moves) {
        if (m.kind == toolpath::MoveKind::Feed && m.to.z < -2.9) {
            cutPoint = {m.to.x, m.to.y};
            found = true;
            break;
        }
    }
    PT_REQUIRE(found, "the toolpath has no cutting move at depth");
    const QColor uncut = relief->pixelAtStock({280, 280});
    PT_REQUIRE(uncut.isValid() && relief->pixelAtStock(cutPoint) == uncut, "the first frame is not the uncut stock");

    panel->scrubTo(total);
    PT_REQUIRE(panel->waitSimulation(120000), "the final frame never arrived");
    PT_REQUIRE(relief->pixelAtStock(cutPoint) != uncut && relief->pixelAtStock({280, 280}) == uncut,
               "the cut is not visible at the end of the toolpath");
    PT_REQUIRE(panel->scrubber()->value() == panel->scrubber()->maximum(), "the scrubber is not at the end");
    panel->scrubTo(total / 2);
    PT_REQUIRE(panel->waitSimulation(120000) && panel->simulation().frame()->move == total / 2, "scrubbing back failed");
    panel->stepForwardButton()->click();
    PT_REQUIRE(panel->waitSimulation(120000) && panel->simulation().frame()->move == total / 2 + 1, "step forward failed");
    panel->stepBackButton()->click();
    PT_REQUIRE(panel->waitSimulation(120000) && panel->simulation().frame()->move == total / 2, "step back failed");
    // The canvas shows the tool and splits played from pending (played layer changes the picture).
    const QImage mid = canvas->renderFitted(size);
    panel->scrubTo(total);
    PT_REQUIRE(panel->waitSimulation(120000), "the final frame did not come again");
    PT_REQUIRE(countNear(canvas->renderFitted(size), feed) > countNear(mid, feed),
               "playback does not draw the played part in full colour");
    // Playing at 500x runs to the end by itself.
    panel->scrubTo(0);
    PT_REQUIRE(panel->waitSimulation(120000), "rewinding failed");
    panel->speedCombo()->setCurrentIndex(panel->speedCombo()->count() - 1);
    panel->playButton()->click();
    PT_REQUIRE(panel->playback().playing(), "Play did not start the playback");
    QElapsedTimer t;
    t.start();
    while (panel->playback().playing() && t.elapsed() < 120000) {
        pump(20);
    }
    PT_REQUIRE(panel->playback().position() == total, "the playback did not reach the end");
    PT_REQUIRE(panel->waitSimulation(120000), "the playback end frame never arrived");
    // Colour by toolpath re-shades.
    const QColor depthColour = relief->pixelAtStock(cutPoint);
    panel->colourCombo()->setCurrentIndex(1);
    pump(300);
    PT_REQUIRE(panel->waitSimulation(120000) && relief->pixelAtStock(cutPoint) != depthColour,
               "colouring by toolpath did not change the picture");
    panel->colourCombo()->setCurrentIndex(0);

    // 5. An edit makes the toolpath stale: the preview flags it, the simulation is out of date.
    PT_REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
                   std::vector<document::ObjectId>{a}, geometry::Transform2::translation({10, 0}))),
               "could not move a rectangle");
    window.documentChanged();
    PT_REQUIRE(panel->model().scene().toolpaths.front().stale && panel->model().staleDrawn() == 1,
               "the preview does not flag the stale toolpath");
    PT_REQUIRE(!panel->simulationCurrent() && !relief->banner().isEmpty() &&
                   relief->banner().contains(QStringLiteral("Out of date")),
               "the stale simulation is not flagged");
    PT_REQUIRE(countNear(canvas->renderFitted(size), feed) < feedOn / 2, "the stale toolpath is still drawn like a valid one");
    window.toolpaths()->selectToolpath(doc.toolpaths().front().id);
    window.toolpaths()->recalculateAction()->trigger();
    PT_REQUIRE(window.toolpaths()->waitIdle(120000), "recalculation did not finish");
    PT_REQUIRE(!panel->model().scene().toolpaths.front().stale, "the preview is still stale after recalculating");
    PT_REQUIRE(!panel->simulationCurrent() && !relief->banner().isEmpty(),
               "the simulation of the old moves passes as current after the toolpath was recalculated");
    PT_REQUIRE(panel->startSimulation() && panel->waitSimulation(120000) && panel->simulationCurrent() &&
                   relief->banner().isEmpty(),
               "simulating again did not clear the out-of-date state");

    // 6. Heavy preview paints fast: a toolpath of a million moves.
    {
        std::vector<toolpath::Move> big;
        big.reserve(1'002'000);
        big.push_back(toolpath::Move::rapid({0, 0, 5}));
        for (int row2 = 0; row2 < 1000; ++row2) {
            const double y = row2 * 0.3;
            const bool fwd = row2 % 2 == 0;
            for (int i = 0; i < 1000; ++i) {
                big.push_back(toolpath::Move::feed({(fwd ? i + 1 : 999 - i) * 0.3, y, -1}));
            }
            big.push_back(toolpath::Move::feed({fwd ? 300.0 : 0.0, y + 0.3, -1}));
        }
        document::ToolpathRecord rec;
        rec.name = "Million moves";
        rec.tool.diameter = 3;
        rec.tool.feedRate = 1000;
        rec.tool.plungeRate = 300;
        rec.moves = document::shareMoves(std::move(big));
        PT_REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(rec)), "could not add the big record");
        QElapsedTimer sync;
        sync.start();
        window.documentChanged();
        const double syncMs = static_cast<double>(sync.nsecsElapsed()) / 1e6;
        PT_REQUIRE(panel->model().scene().totalMoves > 1'000'000, "the big toolpath is not in the preview");
        QElapsedTimer paint;
        paint.start();
        const QImage whole = canvas->renderFitted(QSize(1200, 800));
        const double wholeMs = static_cast<double>(paint.nsecsElapsed()) / 1e6;
        const std::size_t pts = canvas->lastPreviewPoints();
        paint.restart();
        const QImage again = canvas->renderFitted(QSize(1200, 800));
        const double cachedMs = static_cast<double>(paint.nsecsElapsed()) / 1e6;
        PT_REQUIRE(countNear(whole, feed) > 1000 && again == whole, "the big toolpath did not paint (or not the same twice)");
        std::printf("preview perf (self-test): %zu moves, sync %.1f ms, first paint %.1f ms (%zu points drawn), repaint from cache %.1f ms\n",
                    panel->model().scene().totalMoves, syncMs, wholeMs, pts, cachedMs);
        std::fflush(stdout);
        PT_REQUIRE(wholeMs < 5000.0 && cachedMs < 1000.0, "painting a million moves is too slow");
        // Remove it again.
        PT_REQUIRE(window.undo(), "could not undo adding the big record");
        PT_REQUIRE(panel->model().scene().toolpaths.size() == 1, "the preview kept the removed toolpath");
    }

    // 7. Edit, recalculate and replace the document while a simulation is running.
    {
        std::vector<toolpath::Move> zig;
        zig.push_back(toolpath::Move::rapid({0, 0, 5}));
        for (int r = 0; r < 1500; ++r) {
            zig.push_back(toolpath::Move::feed({r % 2 == 0 ? 280.0 : 20.0, 20.0 + r * 0.17, -1.0}));
            zig.push_back(toolpath::Move::feed({r % 2 == 0 ? 280.0 : 20.0, 20.17 + r * 0.17, -1.0}));
        }
        document::ToolpathRecord rec;
        rec.name = "Zigzag";
        rec.tool.diameter = 3;
        rec.tool.feedRate = 1000;
        rec.tool.plungeRate = 300;
        rec.moves = document::shareMoves(std::move(zig));
        PT_REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(rec)), "could not add the zigzag record");
        window.documentChanged();
        PT_REQUIRE(panel->startSimulation() && panel->waitSimulation(120000), "the two-toolpath simulation did not run");
    }
    panel->simulation().setChunkDelayForTest(20);
    panel->scrubTo(panel->playback().moveCount());
    pump(100);
    PT_REQUIRE(panel->simulation().progress().state == viewmodel::SimState::Working, "the simulation is not running");
    PT_REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
                   std::vector<document::ObjectId>{b}, geometry::Transform2::translation({0, 10}))),
               "could not move the second rectangle");
    window.documentChanged();
    window.toolpaths()->selectToolpath(doc.toolpaths().front().id);
    window.toolpaths()->recalculateAction()->trigger();
    PT_REQUIRE(window.toolpaths()->waitIdle(120000), "recalculation during the simulation did not finish");
    PT_REQUIRE(!panel->simulationCurrent(), "a simulation of replaced moves passes as current");
    panel->stopButton()->click();
    pump(300);
    PT_REQUIRE(panel->waitSimulation(120000), "stopping the simulation left it working");
    PT_REQUIRE(panel->playback().position() == panel->simulation().frame()->move, "the scrubber did not follow the stop");
    // Replace the document while it runs again.
    panel->simulation().setChunkDelayForTest(20);
    panel->scrubTo(panel->playback().moveCount());
    pump(100);
    PT_REQUIRE(panel->simulation().progress().state == viewmodel::SimState::Working, "the simulation is not running again");
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    PT_REQUIRE(!panel->simActive() && !relief->hasPicture() && panel->model().scene().toolpaths.empty() &&
                   panel->toolpathList()->topLevelItemCount() == 0,
               "replacing the document left the simulation or preview behind");
    pump(100);
    PT_REQUIRE(!panel->startSimulation(), "a simulation started on an empty document");

    // 8. Close while a simulation is running (the caller closes the window).
    document::Document& doc2 = window.currentDocument();
    PT_REQUIRE(!addRect(doc2, 40, 40, 80, 60).isNull(), "could not add a rectangle to the new document");
    PT_REQUIRE(addProfile(window, 3.0, &why), why);
    PT_REQUIRE(panel->startSimulation() && panel->waitSimulation(120000), "the simulation did not run on the new document");
    panel->simulation().setChunkDelayForTest(15);
    panel->scrubTo(panel->playback().moveCount());
    pump(100);
    window.showView(false);
    return {};
}

}  // namespace carvetto::app
