// Self-test of printing (T-0349).
#include "print_self_test.hpp"

#include <QAction>
#include <QFile>
#include <QImage>
#include <QPainter>
#include <QPdfWriter>
#include <QMarginsF>
#include <QPageLayout>
#include <QPageSize>
#include <QRegularExpression>
#include <QTemporaryDir>
#include <filesystem>

#include "canvas_widget.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/print_layout.hpp"
#include "main_window.hpp"
#include "print_support.hpp"

namespace carvetto::app {

namespace {

/// Page objects of a PDF written by QPdfWriter ("/Type /Page" but not "/Type /Pages").
int pdfPages(const std::filesystem::path& p) {
    QFile f(QString::fromStdString(viewmodel::pathToUtf8(p)));
    if (!f.open(QIODevice::ReadOnly)) {
        return -1;
    }
    const QString text = QString::fromLatin1(f.readAll());
    int n = 0;
    QRegularExpressionMatchIterator it = QRegularExpression(QStringLiteral("/Type\\s*/Page(?![A-Za-z])")).globalMatch(text);
    while (it.hasNext()) {
        it.next();
        ++n;
    }
    return n;
}

}  // namespace

std::string printSelfTest(MainWindow& window) {
    PrintController* print = window.printing();
    if (print == nullptr || print->printAction() == nullptr || print->previewAction() == nullptr ||
        print->printPdfAction() == nullptr) {
        return "the File menu has no Print, Print Preview or Print to PDF action";
    }
    QTemporaryDir dir;
    if (!dir.isValid()) {
        return "no temporary directory for the print test";
    }
    const std::filesystem::path base = viewmodel::pathFromUtf8(dir.path().toStdString());

    // Fit to page: one page.
    PrintRequest fit;
    int pages = 0;
    if (const std::string why = print->printToPdf(base / "fit.pdf", fit, false, &pages); !why.empty()) {
        return "printing to PDF failed: " + why;
    }
    if (pages != 1 || pdfPages(base / "fit.pdf") != 1) {
        return "a fit-to-page print is not one page";
    }

    // True size, scaled so the design needs at least three columns: page count = the layout's.
    PrintRequest tiled;
    tiled.options.mode = viewmodel::PrintScaleMode::TrueSize;
    const geometry::Box2 box = print->contentBox(PrintKind::Design);
    if (box.isEmpty() || box.width() <= 0.0) {
        return "the design has no printable extent";
    }
    tiled.options.trueScale = 3.0 * 190.0 / box.width();
    QPdfWriter probe(QString::fromStdString((base / "probe.pdf").string()));
    probe.setPageSize(QPageSize(QPageSize::A4));
    probe.setPageMargins(QMarginsF(0, 0, 0, 0), QPageLayout::Millimeter);
    const viewmodel::PrintLayout layout = print->layoutFor(probe, tiled);
    if (!layout.error.empty() || layout.pageCount() < 3) {
        return "the 1:1 layout does not tile: " + layout.error;
    }
    if (const std::string why = print->printToPdf(base / "tiled.pdf", tiled, false, &pages); !why.empty()) {
        return "printing the tiled design failed: " + why;
    }
    if (pages != layout.pageCount() || pdfPages(base / "tiled.pdf") != layout.pageCount()) {
        return "a tiled print does not have the layout's page count";
    }
    // Toolpath preview and job sheet.
    PrintRequest toolpaths;
    toolpaths.kind = PrintKind::Toolpaths;
    if (const std::string why = print->printToPdf(base / "tp.pdf", toolpaths, false, &pages); !why.empty() || pages < 1) {
        return "printing the toolpath preview failed: " + why;
    }
    PrintRequest sheet;
    sheet.kind = PrintKind::JobSheet;
    sheet.landscape = true;
    if (const std::string why = print->printToPdf(base / "sheet.pdf", sheet, false, &pages); !why.empty() || pages < 1) {
        return "printing the job sheet failed: " + why;
    }
    // Refusals: existing file, '..', wrong extension, nothing to print.
    if (print->printToPdf(base / "fit.pdf", fit, false, nullptr).empty()) {
        return "an existing PDF was replaced without overwrite";
    }
    if (print->printToPdf(base / ".." / "x.pdf", fit, true, nullptr).empty() ||
        print->printToPdf(base / "x.html", fit, true, nullptr).empty()) {
        return "a hostile print path was accepted";
    }
    PrintRequest absurd = tiled;
    absurd.options.trueScale = 100000.0;
    if (print->printToPdf(base / "absurd.pdf", absurd, true, nullptr).empty() ||
        std::filesystem::exists(base / "absurd.pdf")) {
        return "a print needing too many pages was not refused";
    }
    // The print paints white paper, whatever the canvas colours.
    QImage image(120, 120, QImage::Format_RGB32);
    image.fill(Qt::black);
    viewmodel::ViewTransform view;
    view.setViewport(120.0, 120.0);
    view.zoomToFit(box, 0.2);
    {
        QPainter painter(&image);
        window.canvas()->paintForPrint(painter, view, image.size(), false);
    }
    if (image.pixelColor(1, 1) != QColor(Qt::white)) {
        return "the print does not paint white paper";
    }
    return {};
}

}  // namespace carvetto::app
