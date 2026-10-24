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
#include <memory>
#include <string>

#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/pixel_probe.hpp"
#include "carvetto/viewmodel/view_transform.hpp"
#include "main_window.hpp"

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

    window.close();
    std::printf("carvetto self-test: OK\n");
    std::fflush(stdout);
    return 0;
}

}  // namespace carvetto::app
