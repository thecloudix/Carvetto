// File > Print, Print Preview and Print to PDF (T-0349).
#include "print_support.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QMarginsF>
#include <QMenu>
#include <QMessageBox>
#include <QPageLayout>
#include <QPageSize>
#include <QPagedPaintDevice>
#include <QPainter>
#include <QPdfWriter>
#include <QRegularExpression>
#include <QSaveFile>
#include <QTextDocument>
#include <QVBoxLayout>

#ifdef CARVETTO_HAVE_PRINTSUPPORT
#include <QPrintDialog>
#include <QPrintPreviewDialog>
#include <QPrinter>
#endif

#include "canvas_widget.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/job_sheet.hpp"
#include "carvetto/viewmodel/toolpath_preview.hpp"
#include "carvetto/viewmodel/view_transform.hpp"
#include "main_window.hpp"
#include "preview_panel.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

constexpr int kPdfDpi = 300;
constexpr double kMmPerInch = 25.4;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

QString jobTitle(MainWindow& window) {
    const std::filesystem::path current = window.session().path();
    return current.empty() ? QString() : qs(viewmodel::pathToUtf8(current.stem()));
}

/// The job sheet as printable rich text (no inline SVG: rich text has none), as the PDF export does.
void setSheetHtml(QTextDocument& doc, MainWindow& window) {
    const viewmodel::JobSheet sheet = viewmodel::makeJobSheet(
        window.currentDocument(), window.toolpaths()->session().summary(), jobTitle(window).toStdString());
    QString html = qs(viewmodel::renderJobSheetHtml(sheet));
    html.remove(QRegularExpression(QStringLiteral("<svg.*?</svg>"), QRegularExpression::DotMatchesEverythingOption));
    doc.setHtml(html);
}

void drawCross(QPainter& painter, double cx, double cy, double armDots) {
    painter.drawLine(QPointF(cx - armDots, cy), QPointF(cx + armDots, cy));
    painter.drawLine(QPointF(cx, cy - armDots), QPointF(cx, cy + armDots));
}

}  // namespace

PrintController::PrintController(MainWindow& window, QMenu* fileMenu) : window_(window) {
    printAction_ = fileMenu->addAction(QCoreApplication::translate("PrintController", "&Print..."));
    printAction_->setObjectName(QStringLiteral("printAction"));
    previewAction_ = fileMenu->addAction(QCoreApplication::translate("PrintController", "Print Pre&view..."));
    previewAction_->setObjectName(QStringLiteral("printPreviewAction"));
    printPdfAction_ = fileMenu->addAction(QCoreApplication::translate("PrintController", "Print to P&DF..."));
    printPdfAction_->setObjectName(QStringLiteral("printPdfAction"));
#ifdef CARVETTO_HAVE_PRINTSUPPORT
    QObject::connect(printAction_, &QAction::triggered, fileMenu, [this]() { printDialog(); });
    QObject::connect(previewAction_, &QAction::triggered, fileMenu, [this]() { previewDialog(); });
#else
    // Built without Qt PrintSupport: only the PDF output exists.
    printAction_->setEnabled(false);
    previewAction_->setEnabled(false);
#endif
    QObject::connect(printPdfAction_, &QAction::triggered, fileMenu, [this]() { printPdfDialog(); });
}

geometry::Box2 PrintController::contentBox(PrintKind kind) const {
    geometry::Box2 box = viewmodel::activeMaterialMm(window_.currentDocument());
    if (kind == PrintKind::Toolpaths && window_.preview() != nullptr) {
        box.expand(window_.preview()->model().scene().bounds);
    }
    return box;
}

viewmodel::PrintLayout PrintController::layoutFor(const QPagedPaintDevice& dev, const PrintRequest& request) const {
    const QRectF rect = dev.pageLayout().paintRect(QPageLayout::Millimeter);
    viewmodel::PrintPageSetup page;
    page.pageWidthMm = rect.width();
    page.pageHeightMm = rect.height();
    page.marginMm = 10.0;
    return viewmodel::layoutPrint(contentBox(request.kind), page, request.options);
}

int PrintController::render(QPagedPaintDevice& dev, const PrintRequest& request, std::string& error) const {
    error.clear();
    if (request.kind == PrintKind::JobSheet) {
        QTextDocument doc;
        setSheetHtml(doc, window_);
        doc.print(&dev);
        return std::max(1, doc.pageCount());
    }
    const viewmodel::PrintLayout layout = layoutFor(dev, request);
    if (!layout.error.empty() || layout.tiles.empty()) {
        error = layout.error.empty() ? "there is nothing to print" : layout.error;
        return 0;
    }
    QPainter painter(&dev);
    if (!painter.isActive()) {
        error = "cannot start printing";
        return 0;
    }
    const double dpi = static_cast<double>(dev.logicalDpiX());
    const double dotsPerMm = dpi / kMmPerInch;
    const double scaleDots = layout.scale * dotsPerMm;
    const double marginDots = layout.marginMm * dotsPerMm;
    const bool tiled = layout.pageCount() > 1;
    const bool withToolpaths = request.kind == PrintKind::Toolpaths;
    for (const viewmodel::PrintTile& tile : layout.tiles) {
        if (tile.page > 1) {
            dev.newPage();
        }
        const QSize size(painter.device()->width(), painter.device()->height());
        viewmodel::ViewTransform view;
        view.setViewport(size.width(), size.height());
        view.set(scaleDots, {marginDots - tile.window.min.x * scaleDots, marginDots + tile.window.max.y * scaleDots});
        painter.save();
        painter.setClipRect(QRectF(marginDots, marginDots, layout.printableWidthMm * dotsPerMm,
                                   layout.printableHeightMm * dotsPerMm));
        window_.canvas()->paintForPrint(painter, view, size, withToolpaths);
        painter.restore();
        painter.setRenderHint(QPainter::Antialiasing, true);
        QPen markPen(Qt::black);
        markPen.setWidthF(0.2 * dotsPerMm);
        painter.setPen(markPen);
        painter.setBrush(Qt::NoBrush);
        for (const geometry::Vec2& m : tile.marks) {
            drawCross(painter, m.x * dotsPerMm, m.y * dotsPerMm, viewmodel::kPrintMarkArmMm * dotsPerMm);
        }
        if (tiled) {
            QFont font = painter.font();
            font.setPixelSize(std::max(6, static_cast<int>(3.0 * dotsPerMm)));
            painter.setFont(font);
            const QString label = QCoreApplication::translate("PrintController", "Page %1 of %2 (row %3, column %4)")
                                      .arg(tile.page)
                                      .arg(layout.pageCount())
                                      .arg(tile.row + 1)
                                      .arg(tile.column + 1);
            painter.drawText(QPointF(marginDots, std::max(marginDots * 0.6, 4.0 * dotsPerMm)), label);
        }
    }
    painter.end();
    return layout.pageCount();
}

std::string PrintController::printToPdf(const std::filesystem::path& path, const PrintRequest& request,
                                        bool overwrite, int* pages) {
    if (pages != nullptr) {
        *pages = 0;
    }
    if (const std::string why = viewmodel::checkJobSheetPath(path, viewmodel::JobSheetFormat::Pdf); !why.empty()) {
        return why;
    }
    std::error_code ec;
    if (!overwrite && std::filesystem::exists(path, ec)) {
        return "'" + viewmodel::pathToUtf8(path) + "' exists already";
    }
    if (std::filesystem::is_symlink(std::filesystem::symlink_status(path, ec))) {
        return "'" + viewmodel::pathToUtf8(path) + "' is a symbolic link; choose a regular file name";
    }
    QSaveFile file(qs(viewmodel::pathToUtf8(path)));
    if (!file.open(QIODevice::WriteOnly)) {
        return "cannot write '" + viewmodel::pathToUtf8(path) + "': " + file.errorString().toStdString();
    }
    std::string error;
    int count = 0;
    {
        QPdfWriter pdf(&file);
        pdf.setResolution(kPdfDpi);
        pdf.setPageSize(QPageSize(QPageSize::A4));
        pdf.setPageOrientation(request.landscape ? QPageLayout::Landscape : QPageLayout::Portrait);
        // The layout keeps its own margin for drawings; the job sheet text needs a real one.
        pdf.setPageMargins(request.kind == PrintKind::JobSheet ? QMarginsF(10, 10, 10, 10) : QMarginsF(0, 0, 0, 0),
                           QPageLayout::Millimeter);
        count = render(pdf, request, error);
    }
    if (count == 0) {
        file.cancelWriting();
        return error;
    }
    if (!file.commit()) {
        return "cannot write '" + viewmodel::pathToUtf8(path) + "': " + file.errorString().toStdString();
    }
    if (pages != nullptr) {
        *pages = count;
    }
    return {};
}

bool PrintController::askRequest(PrintRequest& request) {
    QDialog dialog(&window_);
    dialog.setObjectName(QStringLiteral("printOptionsDialog"));
    dialog.setWindowTitle(QCoreApplication::translate("PrintController", "Print"));
    auto* form = new QFormLayout;
    auto* kind = new QComboBox;
    kind->setObjectName(QStringLiteral("printKind"));
    kind->addItem(QCoreApplication::translate("PrintController", "Design (2D view)"), static_cast<int>(PrintKind::Design));
    kind->addItem(QCoreApplication::translate("PrintController", "Toolpath preview"), static_cast<int>(PrintKind::Toolpaths));
    kind->addItem(QCoreApplication::translate("PrintController", "Job sheet"), static_cast<int>(PrintKind::JobSheet));
    kind->setCurrentIndex(static_cast<int>(last_.kind));
    auto* mode = new QComboBox;
    mode->setObjectName(QStringLiteral("printMode"));
    mode->addItem(QCoreApplication::translate("PrintController", "Fit to page"), static_cast<int>(viewmodel::PrintScaleMode::FitToPage));
    mode->addItem(QCoreApplication::translate("PrintController", "True size (tiled over pages)"),
                  static_cast<int>(viewmodel::PrintScaleMode::TrueSize));
    mode->setCurrentIndex(last_.options.mode == viewmodel::PrintScaleMode::TrueSize ? 1 : 0);
    auto* percent = new QDoubleSpinBox;
    percent->setObjectName(QStringLiteral("printScalePercent"));
    percent->setRange(1.0, 1000.0);
    percent->setDecimals(1);
    percent->setSuffix(QStringLiteral(" %"));
    percent->setValue(last_.options.trueScale * 100.0);
    auto* overlap = new QDoubleSpinBox;
    overlap->setObjectName(QStringLiteral("printOverlap"));
    overlap->setRange(0.0, 50.0);
    overlap->setDecimals(1);
    overlap->setSuffix(QStringLiteral(" mm"));
    overlap->setValue(last_.options.overlapMm);
    auto* marks = new QCheckBox(QCoreApplication::translate("PrintController", "Registration marks on tiled pages"));
    marks->setObjectName(QStringLiteral("printMarks"));
    marks->setChecked(last_.options.registrationMarks);
    auto* landscape = new QCheckBox(QCoreApplication::translate("PrintController", "Landscape (PDF)"));
    landscape->setObjectName(QStringLiteral("printLandscape"));
    landscape->setChecked(last_.landscape);
    form->addRow(QCoreApplication::translate("PrintController", "Print:"), kind);
    form->addRow(QCoreApplication::translate("PrintController", "Scale:"), mode);
    form->addRow(QCoreApplication::translate("PrintController", "True size scale:"), percent);
    form->addRow(QCoreApplication::translate("PrintController", "Tile overlap:"), overlap);
    form->addRow(marks);
    form->addRow(landscape);
    const auto sync = [=]() {
        const bool graphic = kind->currentData().toInt() != static_cast<int>(PrintKind::JobSheet);
        const bool tiled = graphic && mode->currentData().toInt() == static_cast<int>(viewmodel::PrintScaleMode::TrueSize);
        mode->setEnabled(graphic);
        percent->setEnabled(tiled);
        overlap->setEnabled(tiled);
        marks->setEnabled(tiled);
    };
    QObject::connect(kind, &QComboBox::currentIndexChanged, &dialog, sync);
    QObject::connect(mode, &QComboBox::currentIndexChanged, &dialog, sync);
    sync();
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    auto* layout = new QVBoxLayout(&dialog);
    layout->addLayout(form);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted) {
        return false;
    }
    request.kind = static_cast<PrintKind>(kind->currentData().toInt());
    request.options.mode = static_cast<viewmodel::PrintScaleMode>(mode->currentData().toInt());
    request.options.trueScale = percent->value() / 100.0;
    request.options.overlapMm = overlap->value();
    request.options.registrationMarks = marks->isChecked();
    request.landscape = landscape->isChecked();
    last_ = request;
    return true;
}

void PrintController::printPdfDialog() {
    PrintRequest request = last_;
    if (!askRequest(request)) {
        return;
    }
    const std::filesystem::path current = window_.session().path();
    const QString folder = current.empty() ? QDir::homePath() : qs(viewmodel::pathToUtf8(current.parent_path()));
    const QString suggested = QDir(folder).filePath(jobTitle(window_).isEmpty() ? QStringLiteral("print.pdf")
                                                                                  : jobTitle(window_) + QStringLiteral(".pdf"));
    const QString name = QFileDialog::getSaveFileName(
        &window_, QCoreApplication::translate("PrintController", "Print to PDF"), suggested,
        QCoreApplication::translate("PrintController", "PDF (*.pdf)"));
    if (name.isEmpty()) {
        return;
    }
    std::filesystem::path path = viewmodel::pathFromUtf8(name.toStdString());
    if (QFileInfo(name).suffix().isEmpty()) {
        path += ".pdf";
    }
    int pages = 0;
    const std::string why = printToPdf(path, request, true, &pages);
    if (!why.empty()) {
        QMessageBox::critical(&window_, QCoreApplication::translate("PrintController", "Print to PDF"), qs(why));
    }
}

#ifdef CARVETTO_HAVE_PRINTSUPPORT
void PrintController::printDialog() {
    PrintRequest request = last_;
    if (!askRequest(request)) {
        return;
    }
    QPrinter printer(QPrinter::HighResolution);
    QPrintDialog dialog(&printer, &window_);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    std::string error;
    if (render(printer, request, error) == 0) {
        QMessageBox::critical(&window_, QCoreApplication::translate("PrintController", "Print"), qs(error));
    }
}

void PrintController::previewDialog() {
    PrintRequest request = last_;
    if (!askRequest(request)) {
        return;
    }
    QPrinter printer(QPrinter::HighResolution);
    QPrintPreviewDialog preview(&printer, &window_);
    preview.setObjectName(QStringLiteral("printPreviewDialog"));
    QObject::connect(&preview, &QPrintPreviewDialog::paintRequested, &preview, [this, request](QPrinter* p) {
        std::string error;
        render(*p, request, error);
    });
    preview.exec();
}
#else
void PrintController::printDialog() {}
void PrintController::previewDialog() {}
#endif

}  // namespace carvetto::app
