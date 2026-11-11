// Self-test of the keep-out zones UI (T-0256): drives the real MainWindow offscreen through the
// Keep-out Zones dock and menu actions, the canvas drawing and the routed preview.

#include <QAction>
#include <QCoreApplication>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QImage>
#include <QMenu>
#include <QMenuBar>
#include <cmath>
#include <memory>
#include <string>

#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "carvetto/viewmodel/time_readout.hpp"
#include "carvetto/viewmodel/toolpath_preview.hpp"
#include "keepout_panel.hpp"
#include "main_window.hpp"
#include "preview_panel.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define KT_REQUIRE(cond, text)                              \
    do {                                                    \
        if (!(cond)) {                                      \
            return std::string("keep-out: ") + (text);      \
        }                                                   \
    } while (false)

int countNear(const QImage& image, QRect region, int r, int g, int b, int tol) {
    int n = 0;
    region = region.intersected(image.rect());
    for (int y = region.top(); y <= region.bottom(); ++y) {
        for (int x = region.left(); x <= region.right(); ++x) {
            const QRgb p = image.pixel(x, y);
            if (std::abs(qRed(p) - r) <= tol && std::abs(qGreen(p) - g) <= tol && std::abs(qBlue(p) - b) <= tol) {
                ++n;
            }
        }
    }
    return n;
}

/// Pixels of `region` that differ between the two images.
int diffCount(const QImage& a, const QImage& b, QRect region) {
    int n = 0;
    region = region.intersected(a.rect());
    for (int y = region.top(); y <= region.bottom(); ++y) {
        for (int x = region.left(); x <= region.right(); ++x) {
            if (a.pixel(x, y) != b.pixel(x, y)) {
                ++n;
            }
        }
    }
    return n;
}

QRect pixelRect(const viewmodel::ViewTransform& view, double x0, double y0, double x1, double y1) {
    const geometry::Box2 px = view.worldToScreen(geometry::Box2{{x0, y0}, {x1, y1}});
    return QRect(QPoint(static_cast<int>(px.min.x), static_cast<int>(px.min.y)),
                 QPoint(static_cast<int>(px.max.x), static_cast<int>(px.max.y)));
}

}  // namespace

std::string keepOutSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    KeepOutPanel* panel = window.keepOutPanel();
    KT_REQUIRE(panel != nullptr && window.keepOutDock() != nullptr && window.keepOutDock()->widget() == panel,
               "the window has no Keep-out Zones dock");

    // The Edit menu holds the submenu with the same actions.
    bool inMenu = false;
    for (QAction* top : window.menuBar()->actions()) {
        if (top->menu() == nullptr || top->text().remove(QLatin1Char('&')) != QStringLiteral("Edit")) {
            continue;
        }
        for (QAction* sub : top->menu()->actions()) {
            if (sub->menu() != nullptr && sub->text().remove(QLatin1Char('&')) == QStringLiteral("Keep-out Zones")) {
                inMenu = sub->menu()->actions().contains(panel->createAction()) &&
                         sub->menu()->actions().contains(panel->clearAction()) &&
                         sub->menu()->actions().contains(panel->showAction());
            }
        }
    }
    KT_REQUIRE(inMenu, "the Edit menu has no Keep-out Zones submenu with its actions");

    // A toolpath whose rapid crosses the zone-to-be: along y = 120 from x = 10 to x = 290.
    document::ToolpathRecord rec;
    rec.name = "Crossing";
    rec.moves = document::shareMoves({toolpath::Move::rapid({10, 120, 10}), toolpath::Move::rapid({290, 120, 10}),
                                      toolpath::Move::plunge({290, 120, -1}), toolpath::Move::feed({290, 140, -1})});
    KT_REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(std::move(rec))), "could not add a toolpath");
    window.documentChanged();

    // The zone: a rectangle 100..140 x 100..140 mm.
    KT_REQUIRE(doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                   geometry::Contour::rectangle(geometry::Box2{{100, 100}, {140, 140}}), doc.activeLayer(),
                   doc.activeSheet()))),
               "could not add the zone vector");
    doc.selection().clear();
    window.documentChanged();
    KT_REQUIRE(!panel->createAction()->isEnabled(), "Create is enabled with nothing selected");
    KT_REQUIRE(!panel->clearAction()->isEnabled(), "Clear is enabled without zones");

    const QSize size(900, 900);
    viewmodel::ViewTransform view;
    PreviewPanel* preview = window.preview();
    // Toolpath pixels = what differs between the preview on and off (a hand-made record is
    // "stale" and drawn grey, so its colour is not relied on).
    const auto toolpathPixels = [&](const QRect& region) {
        preview->showAction()->setChecked(true);
        const QImage on = window.canvas()->renderFitted(size, &view);
        preview->showAction()->setChecked(false);
        const QImage off = window.canvas()->renderFitted(size, &view);
        preview->showAction()->setChecked(true);
        return diffCount(on, off, region);
    };
    window.canvas()->renderFitted(size, &view);
    const QRect inside = pixelRect(view, 104, 104, 136, 136);
    const int rapidBefore = toolpathPixels(inside);
    KT_REQUIRE(rapidBefore > 10, "the straight rapid is not drawn across the zone-to-be");

    // 1. Create from Selection: enabled, one undo step, drawn.
    doc.selection().assign({doc.objectIds().back()});
    panel->refresh();
    KT_REQUIRE(panel->createAction()->isEnabled(), "Create is not enabled for a closed vector");
    const std::size_t steps = doc.undoStack().count();
    panel->createAction()->trigger();
    KT_REQUIRE(doc.keepOutZones().size() == 1 && doc.undoStack().count() == steps + 1,
               "Create from Selection did not add one zone in one undo step");
    KT_REQUIRE(panel->clearAction()->isEnabled(), "Clear is not enabled with a zone");
    const QImage drawn = window.canvas()->renderFitted(size, &view);
    KT_REQUIRE(countNear(drawn, pixelRect(view, 100, 100, 140, 140), 200, 30, 30, 40) > 50,
               "the zone is not drawn on the canvas");

    // 2. The preview routes the rapid round the zone; the summary reports it.
    KT_REQUIRE(preview->model().scene().routing.routed == 1, "the preview did not route the rapid");
    KT_REQUIRE(toolpathPixels(inside) < rapidBefore / 4,
               "the routed rapid still crosses the zone in the preview");
    const viewmodel::JobSummary summary = window.toolpaths()->session().summary();
    KT_REQUIRE(summary.keepOutRouted == 1, "the job summary does not count the routed rapid");

    // 3. Clearance: the field edits it (display units are mm here), one undo step, halo drawn.
    panel->clearanceField()->setValue(8.0);
    KT_REQUIRE(doc.keepOutClearance() == 8.0 && doc.undoStack().count() == steps + 2,
               "editing the clearance field did not set the clearance in one undo step");
    const QImage halo = window.canvas()->renderFitted(size, &view);
    KT_REQUIRE(countNear(halo, pixelRect(view, 88, 88, 152, 152), 220, 60, 60, 60) >
                   countNear(drawn, pixelRect(view, 88, 88, 152, 152), 220, 60, 60, 60) + 20,
               "the clearance halo is not drawn");
    KT_REQUIRE(preview->model().scene().routing.routed == 1, "the rapid is not routed round the grown zone");

    // 4. Hide and show.
    panel->showAction()->setChecked(false);
    KT_REQUIRE(!doc.keepOutVisible(), "Show Zones off did not hide the zones");
    const QImage hidden = window.canvas()->renderFitted(size, &view);
    KT_REQUIRE(countNear(hidden, pixelRect(view, 100, 100, 140, 140), 200, 30, 30, 40) <
                   countNear(drawn, pixelRect(view, 100, 100, 140, 140), 200, 30, 30, 40) / 4,
               "hidden zones are still drawn");
    KT_REQUIRE(preview->model().scene().routing.routed == 1, "hiding the zones changed the routing");
    panel->showAction()->setChecked(true);
    KT_REQUIRE(doc.keepOutVisible(), "Show Zones on did not show the zones");

    // 5. Save Toolpaths takes the zones too.
    const viewmodel::SaveSource source = viewmodel::makeSaveSource(window.toolpaths()->session());
    KT_REQUIRE(source.keepOutZones.size() == 1 && source.keepOutClearance == 8.0,
               "Save Toolpaths does not carry the zones");

    // 6. Clear Zones: one undo step; the rapid is straight again; undo brings the zone back.
    panel->clearAction()->trigger();
    KT_REQUIRE(doc.keepOutZones().empty() && doc.undoStack().count() == steps + 3, "Clear Zones was not one undo step");
    KT_REQUIRE(preview->model().scene().routing.routed == 0, "the preview still routes without zones");
    KT_REQUIRE(toolpathPixels(inside) > rapidBefore / 2,
               "the rapid is not straight again after Clear Zones");
    KT_REQUIRE(window.undo() && doc.keepOutZones().size() == 1, "undoing Clear Zones did not restore the zone");
    KT_REQUIRE(window.undo() && doc.keepOutClearance() == 0.0, "undoing the clearance edit did not restore it");
    KT_REQUIRE(window.undo() && doc.keepOutZones().empty(), "undoing Create did not remove the zone");
    return {};
}

}  // namespace carvetto::app
