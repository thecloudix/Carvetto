#include "self_test.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QAction>
#include <QKeyEvent>
#include <QKeySequence>
#include <QMenu>
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
#include <vector>

#include "array_copy_menu.hpp"
#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/array_copies.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/pixel_probe.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/view_transform.hpp"
#include "main_window.hpp"
#include "node_edit_menu.hpp"
#include "save_toolpaths_dialog.hpp"
#include "tool_panel.hpp"
#include "transform_menu.hpp"
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

    // 4d. Node editing through the real window: select, N, drag a node, convert a span, insert a
    // point, Escape reverts a drag, undo everything (T-0206).
    {
        CanvasWidget* canvas = window.canvas();
        NodeEditMenu* nodeMenu = window.nodeEdit();
        QAction* nodeAction = window.toolAction(viewmodel::ToolKind::NodeEdit);
        if (nodeMenu == nullptr || nodeAction == nullptr ||
            nodeAction->shortcut() != QKeySequence(QStringLiteral("N"))) {
            return fail("the Node Edit tool is missing or has no N shortcut");
        }
        const double w = material.width();
        const geometry::Vec2 lo{material.min.x + w * 0.5, material.min.y + w * 0.5};
        const geometry::Vec2 hi{lo.x + w * 0.2, lo.y + w * 0.1};
        document::RectangleParams rp;
        rp.centreMm = (lo + hi) * 0.5;
        rp.widthMm = hi.x - lo.x;
        rp.heightMm = hi.y - lo.y;
        if (!doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                geometry::Contour::rectangle(geometry::Box2{lo, hi}), doc.activeLayer(), doc.activeSheet(), rp)))) {
            return fail("could not add a rectangle for node editing");
        }
        const document::ObjectId rectId = doc.objectIds().back();
        const auto objectsBefore = doc.objects();
        const std::size_t stepsBefore = doc.undoStack().count();
        doc.selection().assign({rectId});
        window.documentChanged();

        window.selectTool(viewmodel::ToolKind::NodeEdit);
        if (canvas->tools().tool() != viewmodel::ToolKind::NodeEdit || !nodeAction->isChecked()) {
            return fail("choosing Node Edit did not activate the tool");
        }
        // Drag the top right node: one undo step, the parameters are gone, the node is where dropped.
        const QImage idle = canvas->grab().toImage();
        const geometry::Vec2 moved{hi.x + w * 0.05, hi.y + w * 0.05};
        dragOnCanvas(canvas, hi, moved);
        if (doc.undoStack().count() != stepsBefore + 1) {
            return fail("dragging a node did not make exactly one undo step");
        }
        const auto* edited = doc.findObject(rectId)->asVector();
        if (std::holds_alternative<document::RectangleParams>(edited->params) || edited->contours.at(0).spans.size() != 4 ||
            geometry::distance(geometry::endPoint(edited->contours.at(0).spans[1]), moved) > 1e-6) {
            return fail("the dragged node did not end where it was dropped");
        }
        if (canvas->grab().toImage() == idle) {
            return fail("the node handles / edit were not painted");
        }

        // Escape reverts a drag in progress exactly and leaves nothing to redo.
        const auto afterDrag = doc.objects();
        sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, lo), Qt::NoButton);
        sendMouse(canvas, QEvent::MouseButtonPress, screenOf(canvas, lo));
        sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, lo - geometry::Vec2{w * 0.03, w * 0.03}),
                  Qt::LeftButton);
        sendKey(canvas, Qt::Key_Escape);
        sendMouse(canvas, QEvent::MouseButtonRelease, screenOf(canvas, lo));
        if (!(doc.objects() == afterDrag) || doc.undoStack().canRedo() || doc.undoStack().count() != stepsBefore + 1) {
            return fail("Escape did not revert the node drag exactly");
        }

        // Select a span by clicking near the start of the bottom edge, convert it to an arc (menu glue
        // and the viewmodel operation behind it), then insert a point with the I key.
        clickOnCanvas(canvas, geometry::Vec2{lo.x + (hi.x - lo.x) * 0.2, lo.y});
        QMenu* spanMenu = nodeMenu->buildMenu(viewmodel::HandleType::Span, lo);
        bool arcEnabled = false;
        for (QAction* a : spanMenu->actions()) {
            if (a->text().contains(QStringLiteral("Convert to &Arc"))) {
                arcEnabled = a->isEnabled();
            }
        }
        delete spanMenu;
        if (!arcEnabled) {
            return fail("Convert to Arc is not enabled for a selected span");
        }
        viewmodel::NodeOpArgs through;
        through.throughMm = geometry::Vec2{(lo.x + hi.x) * 0.5, lo.y - w * 0.03};
        if (!nodeMenu->run(viewmodel::NodeOp::ConvertArc, through).ok ||
            !std::holds_alternative<geometry::ArcSpan>(doc.findObject(rectId)->asVector()->contours.at(0).spans[0])) {
            return fail("converting the span to an arc failed");
        }
        sendKey(canvas, Qt::Key_I);
        if (doc.findObject(rectId)->asVector()->contours.at(0).spans.size() != 5) {
            return fail("the I key did not insert a point");
        }
        if (doc.undoStack().count() != stepsBefore + 3) {
            return fail("convert and insert were not one undo step each");
        }
        // Undo all three steps: the rectangle, parameters included, is back byte-identically.
        for (int i = 0; i < 3; ++i) {
            if (!window.undo()) {
                return fail("undoing the node edits failed");
            }
        }
        if (!(doc.objects() == objectsBefore)) {
            return fail("undoing the node edits did not restore the rectangle exactly");
        }
        window.selectTool(viewmodel::ToolKind::Select);
        doc.selection().clear();
        if (!window.undo() || doc.objectCount() != 0) {
            return fail("undoing the node-edit rectangle failed");
        }
        window.documentChanged();
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

    // 4e. H / V collision (T-0206 / T-0205): in NodeEdit the plain letters win over the window
    // shortcuts (ShortcutOverride) and H toggles the mirror mode without a Mirror undo step; in
    // Select mode the same key is left to the Transform shortcut, which mirrors the selection.
    {
        CanvasWidget* canvas = window.canvas();
        const geometry::Box2 box{{material.min.x + 5.0, material.min.y + 5.0}, {material.min.x + 25.0, material.min.y + 15.0}};
        if (!doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                geometry::Contour::rectangle(box), doc.activeLayer(), doc.activeSheet())))) {
            return fail("could not add a rectangle for the H/V test");
        }
        doc.selection().assign({doc.objectIds().back()});
        window.documentChanged();
        const std::size_t steps = doc.undoStack().count();
        auto overrideAccepted = [&](Qt::Key key) {
            QKeyEvent ev(QEvent::ShortcutOverride, key, Qt::NoModifier, QString(QChar('a' + (key - Qt::Key_A))));
            QCoreApplication::sendEvent(canvas, &ev);
            return ev.isAccepted();
        };
        window.selectTool(viewmodel::ToolKind::NodeEdit);
        if (!overrideAccepted(Qt::Key_H) || !overrideAccepted(Qt::Key_V)) {
            return fail("NodeEdit does not claim the H / V keys from the window shortcuts");
        }
        sendKey(canvas, Qt::Key_H);
        if (canvas->tools().nodeEditor().options().mirror != viewmodel::MirrorMode::Horizontal ||
            doc.undoStack().count() != steps) {
            return fail("H in NodeEdit did not toggle the mirror mode without an undo step");
        }
        sendKey(canvas, Qt::Key_H);
        window.selectTool(viewmodel::ToolKind::Select);
        if (overrideAccepted(Qt::Key_H)) {
            return fail("Select mode claims H although it belongs to the Transform shortcut");
        }
        QAction* flip = window.transformMenu()->mirrorAction(viewmodel::MirrorAxis::Horizontal, false, false);
        if (flip == nullptr || flip->shortcut() != QKeySequence(QStringLiteral("H")) || !flip->isEnabled()) {
            return fail("the Transform menu has no enabled H mirror action");
        }
        flip->trigger();
        if (doc.undoStack().count() != steps + 1) {
            return fail("H in Select mode did not mirror the selection in one undo step");
        }
        if (!window.undo() || !window.undo() || doc.objectCount() != 0) {
            return fail("undoing the H/V test objects failed");
        }
        doc.selection().clear();
        window.documentChanged();
    }

    // 5b. Transforms through the real window: handle drag, numeric move, align, undo (T-0205).
    if (const std::string why = transformSelfTest(window); !why.empty()) {
        return fail(why);
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

    // 7. Replacing the document during a node drag (File > New order: the old document is freed
    // first, then the canvas rebinds the tools); the app must not touch the freed document.
    {
        CanvasWidget* canvas = window.canvas();
        document::Document& cur = window.currentDocument();
        const geometry::Box2 box = viewmodel::activeMaterialMm(cur);
        const double w = box.width();
        const geometry::Vec2 lo{box.min.x + w * 0.2, box.min.y + w * 0.2};
        const geometry::Vec2 hi{lo.x + w * 0.2, lo.y + w * 0.1};
        if (!cur.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                geometry::Contour::rectangle(geometry::Box2{lo, hi}), cur.activeLayer(), cur.activeSheet())))) {
            return fail("could not add a rectangle for the document replacement test");
        }
        cur.selection().assign({cur.objectIds().back()});
        window.documentChanged();
        window.selectTool(viewmodel::ToolKind::NodeEdit);
        sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, hi), Qt::NoButton);
        sendMouse(canvas, QEvent::MouseButtonPress, screenOf(canvas, hi));
        sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, hi + geometry::Vec2{w * 0.05, w * 0.05}), Qt::LeftButton);
        if (!canvas->tools().nodeEditor().gestureActive()) {
            return fail("the node drag for the replacement test did not start");
        }
        window.setDocument(std::make_unique<document::Document>(document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
        sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, lo), Qt::LeftButton);
        sendMouse(canvas, QEvent::MouseButtonRelease, screenOf(canvas, lo));
        sendKey(canvas, Qt::Key_Escape);
        if (window.currentDocument().objectCount() != 0 || canvas->tools().nodeEditor().gestureActive()) {
            return fail("replacing the document during a node drag left state behind");
        }
        window.selectTool(viewmodel::ToolKind::Select);
    }

    // 8. Save Toolpaths (T-0224): two calculated toolpaths with different tools, post them through
    // the dialog (generation and writing run on a worker thread), re-read the files.
    {
        document::Document& cur = window.currentDocument();
        const auto addRecord = [&](const std::string& name, int toolNumber, double diameter) {
            document::ToolpathRecord r;
            r.name = name;
            r.tool.name = "Tool " + std::to_string(toolNumber);
            r.tool.toolNumber = toolNumber;
            r.tool.diameter = diameter;
            r.tool.feedRate = 800.0;
            r.tool.plungeRate = 300.0;
            r.tool.spindleSpeed = 12000.0;
            std::vector<toolpath::Move> moves;
            moves.push_back(toolpath::Move::rapid({0.0, 0.0, 5.0}));
            moves.push_back(toolpath::Move::plunge({0.0, 0.0, -1.0}));
            moves.push_back(toolpath::Move::feed({20.0, 0.0, -1.0}));
            moves.push_back(toolpath::Move::feed({20.0, 20.0, -1.0}));
            moves.push_back(toolpath::Move::retract({20.0, 20.0, 5.0}));
            r.moves = document::shareMoves(std::move(moves));
            return cur.execute(std::make_unique<document::AddToolpathCommand>(std::move(r)));
        };
        if (!addRecord("Outline", 1, 3.0) || !addRecord("Pocket", 2, 6.0)) {
            return fail("could not add toolpaths for the Save Toolpaths test");
        }
        if (window.saveToolpathsAction() == nullptr) {
            return fail("the File menu has no Save Toolpaths action");
        }
        QTemporaryDir outDir;
        if (!outDir.isValid()) {
            return fail("no temporary directory for the Save Toolpaths test");
        }
        const std::filesystem::path outPath = viewmodel::pathFromUtf8(outDir.path().toStdString());
        SaveToolpathsDialog dialog(cur, {}, "job", {}, outPath);
        dialog.show();
        if (!dialog.waitIdle() || dialog.postCount() < 3 || !dialog.output().ok) {
            return fail("the Save Toolpaths dialog did not generate: " + dialog.output().error);
        }
        if (!dialog.selectPost("grbl_mm.pp") || !dialog.waitIdle() || !dialog.output().ok ||
            dialog.output().files.size() != 1 || dialog.output().toolChanges != 1) {
            return fail("the Save Toolpaths dialog gave no single Grbl file with one tool change");
        }
        if (dialog.previewShown().find("G21") == std::string::npos || !dialog.canSave()) {
            return fail("the Save Toolpaths preview is empty or Save is disabled");
        }
        if (!dialog.saveNow() || dialog.written().size() != 1 || dialog.written().front() != outPath / "job.nc") {
            return fail("saving the toolpaths through the dialog failed");
        }
        std::string reread;
        {
            QFile f(outDir.filePath(QStringLiteral("job.nc")));
            if (!f.open(QIODevice::ReadOnly)) {
                return fail("the saved G-code file cannot be read");
            }
            reread = QString::fromUtf8(f.readAll()).toStdString();
        }
        if (reread != dialog.output().files.front().text || reread.find("(Change to tool 2: Tool 2)") == std::string::npos) {
            return fail("the saved file differs from the generated G-code");
        }
        // The file exists now: the dialog says so and a second save is refused until Replace is ticked.
        dialog.setNameTemplate("job");  // regenerate to see the file
        if (!dialog.waitIdle() || !dialog.output().files.front().exists || dialog.saveNow()) {
            return fail("the dialog did not refuse to overwrite the saved file");
        }
        dialog.setOverwrite(true);
        if (!dialog.waitIdle() || !dialog.saveNow()) {
            return fail("replacing the saved file failed");
        }
        // One file per toolpath with a name template.
        dialog.setOutputMode(viewmodel::OutputMode::FilePerToolpath);
        dialog.setNameTemplate("{n}-{toolpath}");
        if (!dialog.waitIdle() || dialog.output().files.size() != 2 || !dialog.saveNow() || dialog.written().size() != 2 ||
            !QFileInfo::exists(outDir.filePath(QStringLiteral("01-Outline.nc"))) ||
            !QFileInfo::exists(outDir.filePath(QStringLiteral("02-Pocket.nc")))) {
            return fail("one file per toolpath was not written");
        }
        // A bad template is reported, not saved.
        dialog.setNameTemplate("{nope}");
        if (!dialog.waitIdle() || dialog.canSave() || dialog.messageShown().find("{nope}") == std::string::npos) {
            return fail("a bad file name template was not reported");
        }
        dialog.reject();
        window.undo();
        window.undo();
        if (!cur.toolpaths().empty()) {
            return fail("undoing the test toolpaths did not remove them");
        }
    }

    window.close();
    std::printf("carvetto self-test: OK\n");
    std::fflush(stdout);
    return 0;
}

}  // namespace carvetto::app
