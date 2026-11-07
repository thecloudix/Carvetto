#include "self_test.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QFileInfo>
#include <QImage>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPixmap>
#include <QPoint>
#include <QSettings>
#include <QSize>
#include <QString>
#include <QTemporaryDir>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>

#include "array_copy_menu.hpp"
#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/array_copies.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/pixel_probe.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/view_transform.hpp"
#include "main_window.hpp"
#include "tool_panel.hpp"
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

void sendMouse(CanvasWidget* canvas, QEvent::Type type, QPointF pos, Qt::MouseButton button = Qt::LeftButton,
               Qt::KeyboardModifiers mods = Qt::NoModifier) {
    const Qt::MouseButtons buttons = type == QEvent::MouseButtonRelease ? Qt::NoButton : button;
    QMouseEvent event(type, pos, canvas->mapToGlobal(pos), type == QEvent::MouseMove ? Qt::NoButton : button,
                      buttons, mods);
    QCoreApplication::sendEvent(canvas, &event);
}

void sendKey(CanvasWidget* canvas, Qt::Key key) {
    QKeyEvent press(QEvent::KeyPress, key, Qt::NoModifier);
    QCoreApplication::sendEvent(canvas, &press);
}

QPointF screenOf(const CanvasWidget* canvas, geometry::Vec2 mm) {
    const geometry::Vec2 p = canvas->view().worldToScreen(mm);
    return QPointF(p.x, p.y);
}

/// Drag with the left button through the real widget event path.
void dragOnCanvas(CanvasWidget* canvas, geometry::Vec2 fromMm, geometry::Vec2 toMm) {
    const QPointF a = screenOf(canvas, fromMm);
    const QPointF b = screenOf(canvas, toMm);
    sendMouse(canvas, QEvent::MouseMove, a, Qt::NoButton);
    sendMouse(canvas, QEvent::MouseButtonPress, a);
    sendMouse(canvas, QEvent::MouseMove, (a + b) * 0.5, Qt::LeftButton);
    sendMouse(canvas, QEvent::MouseMove, b, Qt::LeftButton);
    sendMouse(canvas, QEvent::MouseButtonRelease, b);
}

void clickOnCanvas(CanvasWidget* canvas, geometry::Vec2 mm) {
    const QPointF p = screenOf(canvas, mm);
    sendMouse(canvas, QEvent::MouseMove, p, Qt::NoButton);
    sendMouse(canvas, QEvent::MouseButtonPress, p);
    sendMouse(canvas, QEvent::MouseButtonRelease, p);
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

    // 4b. Drawing tools through the real window: toolbar, mouse events, key events, numeric entry.
    {
        CanvasWidget* canvas = window.canvas();
        const double w = material.width();
        const geometry::Vec2 lo{material.min.x + w * 0.1, material.min.y + w * 0.1};
        const geometry::Vec2 hi{material.min.x + w * 0.4, material.min.y + w * 0.3};

        if (window.toolAction(viewmodel::ToolKind::Rectangle) == nullptr || window.toolPanel() == nullptr) {
            return fail("the tool palette is missing");
        }
        window.selectTool(viewmodel::ToolKind::Rectangle);
        if (canvas->tools().tool() != viewmodel::ToolKind::Rectangle ||
            !window.toolAction(viewmodel::ToolKind::Rectangle)->isChecked()) {
            return fail("choosing Rectangle in the toolbar did not activate the tool");
        }
        dragOnCanvas(canvas, lo, hi);
        if (doc.objectCount() != 1 || doc.undoStack().count() != 1) {
            return fail("dragging a rectangle did not add exactly one object / undo step");
        }
        const document::Object& rect = doc.objects().begin()->second;
        const auto* rp = std::get_if<document::RectangleParams>(&rect.asVector()->params);
        if (rp == nullptr || std::fabs(rp->widthMm - (hi.x - lo.x)) > 1e-6 ||
            std::fabs(rp->heightMm - (hi.y - lo.y)) > 1e-6) {
            return fail("the rectangle has the wrong parameters");
        }
        const auto rebuilt = document::buildContours(rect.asVector()->params);
        if (!rebuilt.ok() || !(rebuilt.value() == rect.asVector()->contours)) {
            return fail("the rectangle does not rebuild from its parameters");
        }
        if (!window.undo() || doc.objectCount() != 0 || !window.redo() || doc.objectCount() != 1) {
            return fail("undo/redo of the drawn rectangle failed");
        }

        // A shape in progress is painted by the overlay and Escape cancels it without an edit.
        window.selectTool(viewmodel::ToolKind::Circle);
        const geometry::Vec2 centreMm{material.min.x + w * 0.7, material.min.y + w * 0.5};
        const QImage idle = canvas->grab().toImage();
        sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, centreMm), Qt::NoButton);
        sendMouse(canvas, QEvent::MouseButtonPress, screenOf(canvas, centreMm));
        sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, centreMm + geometry::Vec2{w * 0.1, 0.0}),
                  Qt::LeftButton);
        const QImage drawing = canvas->grab().toImage();
        if (idle == drawing) {
            return fail("the tool preview was not painted on the canvas");
        }
        sendKey(canvas, Qt::Key_Escape);
        if (canvas->tools().active() || doc.objectCount() != 1 || doc.undoStack().count() != 1) {
            return fail("Escape did not cancel the circle without changing the document");
        }
        sendMouse(canvas, QEvent::MouseButtonRelease, screenOf(canvas, centreMm));

        // Numeric entry: a polygon whose sides and radius are typed (display units are inches).
        window.selectTool(viewmodel::ToolKind::Polygon);
        clickOnCanvas(canvas, centreMm);
        if (!window.toolPanel()->typeInto("sides", "5") || !window.toolPanel()->typeInto("radius", "1")) {
            return fail("valid numeric entries were rejected");
        }
        if (window.toolPanel()->typeInto("radius", "abc") || window.toolPanel()->typeInto("sides", "2")) {
            return fail("invalid numeric entries were accepted");
        }
        window.toolPanel()->typeInto("radius", "1");
        sendKey(canvas, Qt::Key_Return);
        if (doc.objectCount() != 2) {
            return fail("Enter did not complete the polygon from the typed values");
        }
        const auto* pp = std::get_if<document::PolygonParams>(&doc.objects().rbegin()->second.asVector()->params);
        if (pp == nullptr || pp->sides != 5 || std::fabs(pp->radiusMm - 25.4) > 1e-9) {
            return fail("the typed polygon parameters were not applied (sides 5, radius 1 in)");
        }

        // Polyline: two clicks and a double-click finish one open contour.
        window.selectTool(viewmodel::ToolKind::Polyline);
        const geometry::Vec2 q{material.min.x + w * 0.1, material.min.y + w * 0.7};
        clickOnCanvas(canvas, q);
        clickOnCanvas(canvas, q + geometry::Vec2{w * 0.1, 0.0});
        {
            const QPointF last = screenOf(canvas, q + geometry::Vec2{w * 0.1, w * 0.1});
            sendMouse(canvas, QEvent::MouseMove, last, Qt::NoButton);
            sendMouse(canvas, QEvent::MouseButtonDblClick, last);
        }
        if (doc.objectCount() != 3 ||
            doc.objects().rbegin()->second.asVector()->contours.at(0).spans.size() != 2) {
            return fail("the polyline tool did not create a two-span contour");
        }

        // The select tool picks the rectangle by its outline and clears on empty canvas.
        window.selectTool(viewmodel::ToolKind::Select);
        clickOnCanvas(canvas, geometry::Vec2{lo.x, (lo.y + hi.y) * 0.5});
        if (doc.selection().size() != 1) {
            return fail("the select tool did not pick the rectangle");
        }
        clickOnCanvas(canvas, geometry::Vec2{material.max.x - 1.0, material.max.y - 1.0});
        if (!doc.selection().empty()) {
            return fail("the select tool did not clear the selection on empty canvas");
        }
        while (doc.undoStack().canUndo()) {
            window.undo();
        }
        if (doc.objectCount() != 0) {
            return fail("undoing the drawn objects did not empty the document");
        }
        window.selectTool(viewmodel::ToolKind::Select);
    }

    // 4c. Edit > Copies: a 2 x 2 array of a rectangle through the real window, then undo.
    {
        CopiesMenu* copies = window.copies();
        if (copies == nullptr) {
            return fail("the main window has no Copies menu");
        }
        if (!doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                geometry::Contour::rectangle(geometry::Box2{{0.0, 0.0}, {10.0, 5.0}}), doc.activeLayer(),
                doc.activeSheet())))) {
            return fail("could not add a rectangle for the array copy");
        }
        doc.selection().assign(doc.objectIds());
        copies->refresh();
        if (!copies->arrayAction()->isEnabled() || !copies->circularAction()->isEnabled()) {
            return fail("Array Copy and Circular Copy are not enabled for a selected rectangle");
        }
        viewmodel::ArrayOptions options;
        options.rows = 2;
        options.columns = 2;
        options.xMm = 20.0;
        options.yMm = 20.0;
        const viewmodel::CopyResult made = copies->runArray(options, false);
        if (!made.ok || made.created.size() != 3 || doc.objectCount() != 4) {
            return fail("the array copy failed: " + made.message);
        }
        if (!window.undo() || doc.objectCount() != 1) {
            return fail("undoing the array copy did not remove all copies in one step");
        }
        if (!window.undo() || doc.objectCount() != 0) {
            return fail("undoing the rectangle failed after the array copy");
        }
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
