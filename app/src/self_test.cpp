#include "self_test.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QFileInfo>
#include <QImage>
#include <QPixmap>
#include <QPoint>
#include <QSettings>
#include <QSize>
#include <QString>
#include <QTemporaryDir>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>

#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/pixel_probe.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"
#include "carvetto/viewmodel/view_transform.hpp"
#include "main_window.hpp"
#include "vector_ops_menu.hpp"

namespace carvetto::app {

namespace {

int fail(const std::string& reason) {
    std::fprintf(stderr, "carvetto self-test FAILED: %s\n", reason.c_str());
    std::fflush(stderr);
    return 1;
}

/// Pixels in `region` of `image` that differ from the material fill.
std::size_t inkPixels(const QImage& image, const viewmodel::PixelRect& region) {
    const auto* pixels = reinterpret_cast<const std::uint32_t*>(image.constBits());
    const auto stride = static_cast<std::ptrdiff_t>(image.bytesPerLine() / 4);
    return viewmodel::countPixelsDiffering(pixels, image.width(), image.height(), stride, region,
                                           static_cast<std::uint32_t>(materialFill().rgb()));
}

}  // namespace

int runSelfTest() {
    MainWindow window;
    window.resize(1280, 800);
    window.show();
    QCoreApplication::processEvents();

    // 1. Create a job through the same form validation the New Job dialog uses.
    viewmodel::NewJobForm form;
    form.units = document::Units::Inches;
    form.width = 10.0;
    form.height = 10.0;
    form.thickness = 0.75;
    const viewmodel::NewJobResult created = viewmodel::validateNewJob(form);
    if (!created.ok()) {
        return fail("new-job form rejected a valid 10 x 10 x 0.75 in job");
    }
    window.setDocument(std::make_unique<document::Document>(*created.job));
    document::Document& doc = window.currentDocument();
    const geometry::Box2 material = viewmodel::activeMaterialMm(doc);
    if (material.isEmpty()) {
        return fail("the new document has no material rectangle");
    }

    // 2. Add a circle in the middle of the material (undoable).
    const geometry::Vec2 centre = material.centre();
    const double radius = material.width() * 0.25;
    const bool added = doc.execute(std::make_unique<document::AddObjectCommand>(
        document::Object::vector(geometry::Contour::circle(centre, radius), doc.activeLayer(),
                                 doc.activeSheet(), document::CircleParams{centre, radius})));
    window.documentChanged();
    if (!added || doc.objectCount() != 1) {
        return fail("adding a circle to the document failed");
    }

    // 3. Render one frame and look for the circle inside the material.
    const QSize frameSize(800, 600);
    viewmodel::ViewTransform view;
    const QImage frame = window.canvas()->renderFitted(frameSize, &view);
    if (frame.isNull() || frame.size() != frameSize || frame.format() != QImage::Format_RGB32) {
        return fail("rendering the 2D canvas did not produce an 800x600 RGB32 image");
    }
    const viewmodel::CanvasScene scene = viewmodel::buildCanvasScene(doc, view);
    const viewmodel::PixelRect inside =
        viewmodel::insetPixelRect(scene.materialPx, 3, frame.width(), frame.height());
    const geometry::Box2 circlePx =
        view.worldToScreen(geometry::Box2::fromCentre(centre, 2.0 * radius, 2.0 * radius));
    const viewmodel::PixelRect region =
        viewmodel::intersect(inside, viewmodel::toPixelRect(circlePx, 2, frame.width(), frame.height()));
    if (region.isEmpty()) {
        return fail("the circle is not inside the rendered material rectangle");
    }
    const QPoint materialCentre(static_cast<int>(view.worldToScreen(centre).x),
                                static_cast<int>(view.worldToScreen(centre).y));
    if (frame.pixel(materialCentre) != materialFill().rgb()) {
        return fail("the material rectangle was not painted");
    }
    if (frame.pixel(QPoint(0, 0)) == materialFill().rgb()) {
        return fail("the canvas background was painted as material");
    }
    const std::size_t before = inkPixels(frame, region);
    if (before == 0) {
        return fail("no circle pixels were drawn inside the material rectangle");
    }

    // 3b. Click-to-select through the canvas: a click on the circle's outline selects it, a click
    // on empty material clears the selection, Shift adds.
    {
        CanvasWidget* canvas = window.canvas();
        const QPointF onCircle = QPointF(canvas->view().worldToScreen({centre.x + radius, centre.y}).x,
                                         canvas->view().worldToScreen({centre.x + radius, centre.y}).y);
        const QPointF onEmpty = QPointF(canvas->view().worldToScreen(centre).x,
                                        canvas->view().worldToScreen(centre).y);
        if (!canvas->selectAt(onCircle) || doc.selection().size() != 1) {
            return fail("clicking the circle outline did not select it");
        }
        if (canvas->selectAt(onEmpty) || !doc.selection().empty()) {
            return fail("clicking empty canvas did not clear the selection");
        }
        canvas->selectAt(onCircle, true);
        if (doc.selection().size() != 1) {
            return fail("shift-click did not add the circle");
        }
        canvas->selectAt(onCircle, false, true);
        if (!doc.selection().empty()) {
            return fail("ctrl-click did not toggle the circle off");
        }
    }

    // 3c. Vectors menu: weld the circle with an overlapping rectangle through the real window
    // (enabling rules, one undo step, selection = the result), then an offset, then undo both.
    {
        VectorOpsMenu* vectors = window.vectorOps();
        if (vectors == nullptr) {
            return fail("the main window has no Vectors menu");
        }
        const std::size_t objectsBefore = doc.objectCount();
        const document::ObjectId circleId = doc.selection().empty() ? doc.objectIds().front() : doc.selection().ids().front();
        if (!doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                geometry::Contour::rectangle(geometry::Box2{{centre.x, centre.y - radius}, {centre.x + 2.0 * radius, centre.y + radius}}),
                doc.activeLayer(), doc.activeSheet())))) {
            return fail("adding the overlapping rectangle failed");
        }
        doc.selection().assign(doc.objectIds());
        vectors->refresh();
        if (!vectors->booleanAction(viewmodel::BooleanKind::Weld)->isEnabled() || !vectors->offsetAction()->isEnabled()) {
            return fail("Weld and Offset are not enabled for two closed vectors");
        }
        const std::size_t steps = doc.undoStack().count();
        const viewmodel::VectorOpResult welded = vectors->runBoolean(viewmodel::BooleanKind::Weld, false);
        if (!welded.ok || welded.created.size() != 1 || doc.objectCount() != 1 ||
            doc.selection().ids() != welded.created || doc.undoStack().count() != steps + 1) {
            return fail("welding the circle and the rectangle gave the wrong result: " + welded.message);
        }
        viewmodel::OffsetOptions offsetOptions;
        offsetOptions.distanceMm = 2.0;
        const viewmodel::VectorOpResult grown = vectors->runOffset(offsetOptions, false);
        if (!grown.ok || grown.created.size() != 1 || doc.objectCount() != 2) {
            return fail("offsetting the welded vector failed: " + grown.message);
        }
        if (!window.undo() || !window.undo() || doc.objectCount() != objectsBefore + 1) {
            return fail("undoing the offset and the weld did not restore the objects");
        }
        if (!doc.hasObject(circleId) || !doc.undo() || doc.objectCount() != objectsBefore) {
            return fail("undoing the rectangle failed");
        }
        doc.selection().clear();
        window.documentChanged();
    }

    // 4. Undo the add, re-render, and check the circle is gone.
    if (!window.undo() || doc.objectCount() != 0) {
        return fail("undoing the circle failed");
    }
    const QImage after = window.canvas()->renderFitted(frameSize, nullptr);
    const std::size_t remaining = inkPixels(after, region);
    if (remaining != 0) {
        return fail("circle pixels remain after undo (" + std::to_string(remaining) + " of " +
                    std::to_string(before) + ")");
    }

    // The live widget tree must also render headlessly.
    QCoreApplication::processEvents();
    if (window.grab().isNull()) {
        return fail("grabbing the main window produced no image");
    }

    // 5. Save the dock layout to a temporary INI file, disturb it, and restore it.
    QTemporaryDir tempDir;
    if (!tempDir.isValid()) {
        return fail("could not create a temporary directory for the layout file");
    }
    const QString iniPath = tempDir.filePath(QStringLiteral("layout.ini"));
    {
        QSettings settings(iniPath, QSettings::IniFormat);
        window.saveLayout(settings);
        settings.sync();
        if (settings.status() != QSettings::NoError) {
            return fail("writing the layout settings failed");
        }
    }
    if (!QFileInfo::exists(iniPath)) {
        return fail("the layout settings file was not written");
    }
    QDockWidget* toolpaths = window.toolpathsDock();
    if (window.dockWidgetArea(toolpaths) != Qt::RightDockWidgetArea) {
        return fail("the toolpaths panel does not start on the right");
    }
    window.addDockWidget(Qt::LeftDockWidgetArea, toolpaths);
    QCoreApplication::processEvents();
    if (window.dockWidgetArea(toolpaths) != Qt::LeftDockWidgetArea) {
        return fail("moving the toolpaths panel for the layout test failed");
    }
    {
        QSettings settings(iniPath, QSettings::IniFormat);
        if (!window.restoreLayout(settings)) {
            return fail("restoring the dock layout from settings failed");
        }
    }
    QCoreApplication::processEvents();
    if (window.dockWidgetArea(toolpaths) != Qt::RightDockWidgetArea) {
        return fail("the restored layout did not put the toolpaths panel back on the right");
    }

    // 6. File session: open -> modify -> save -> reopen, all headless (no dialogs).
    window.setQuiet(true);
    const std::filesystem::path carvPath = viewmodel::pathFromUtf8(tempDir.filePath(QStringLiteral("job.carv")).toStdString());
    const std::filesystem::path dxfPath = viewmodel::pathFromUtf8(tempDir.filePath(QStringLiteral("part.dxf")).toStdString());
    if (!window.saveFileAs(carvPath) || window.session().isDirty() || !window.windowTitle().startsWith(QStringLiteral("job.carv"))) {
        return fail("saving the job as job.carv failed or left it dirty");
    }
    if (!window.openFile(carvPath) || window.currentDocument().objectCount() != 0) {
        return fail("reopening the saved job failed");
    }
    document::Document& reopened = window.currentDocument();
    if (!reopened.execute(std::make_unique<document::AddObjectCommand>(
            document::Object::vector(geometry::Contour::circle(centre, radius), reopened.activeLayer(),
                                     reopened.activeSheet(), document::CircleParams{centre, radius})))) {
        return fail("adding a circle to the reopened job failed");
    }
    window.documentChanged();
    if (!window.session().isDirty() || !window.windowTitle().contains(QStringLiteral("job.carv*"))) {
        return fail("the modified job is not marked as unsaved in the title");
    }
    if (!window.saveFile() || window.session().isDirty()) {
        return fail("saving the modified job failed");
    }
    if (!window.openFile(carvPath) || window.currentDocument().objectCount() != 1) {
        return fail("the saved modification did not survive reopening");
    }
    if (!window.exportDxfFile(dxfPath, false)) {
        return fail("exporting the sheet as DXF failed: " + window.session().lastError());
    }
    if (!window.importDxfFile(dxfPath, std::nullopt) || window.currentDocument().objectCount() != 2 ||
        !window.session().isDirty()) {
        return fail("importing the exported DXF failed: " + window.session().lastError());
    }
    if (!window.undo() || window.session().isDirty() || window.currentDocument().objectCount() != 1) {
        return fail("undoing the DXF import did not return to the saved state");
    }
    if (window.openFile(viewmodel::pathFromUtf8(tempDir.filePath(QStringLiteral("missing.carv")).toStdString())) ||
        window.currentDocument().objectCount() != 1) {
        return fail("opening a missing file did not fail cleanly");
    }

    window.close();
    std::printf("carvetto self-test: OK\n");
    std::fflush(stdout);
    return 0;
}

}  // namespace carvetto::app
