#pragma once

#include <filesystem>
#include <string>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/viewmodel/print_layout.hpp"

class QAction;
class QMenu;
class QPagedPaintDevice;

namespace carvetto::app {

class MainWindow;

/// What File > Print prints.
enum class PrintKind { Design, Toolpaths, JobSheet };

struct PrintRequest {
    PrintKind kind{PrintKind::Design};
    viewmodel::PrintOptions options;
    /// Page orientation of a PDF (a printer's orientation is set in its own dialog).
    bool landscape{false};
};

/// File > Print..., Print Preview... and Print to PDF... (T-0349). The page layout and tiling are
/// viewmodel::layoutPrint (Qt-free); this class only paints the pages onto a QPagedPaintDevice. The PDF
/// path (QPdfWriter) is what the self-test drives; QPrinter-based printing and the preview dialog exist
/// when Qt PrintSupport was found at build time (CARVETTO_HAVE_PRINTSUPPORT).
class PrintController {
   public:
    PrintController(MainWindow& window, QMenu* fileMenu);

    QAction* printAction() const { return printAction_; }
    QAction* previewAction() const { return previewAction_; }
    QAction* printPdfAction() const { return printPdfAction_; }

    /// The world box printed for `kind` (material, plus the preview's bounds for toolpaths).
    carvetto::geometry::Box2 contentBox(PrintKind kind) const;
    /// The layout of a graphic print on `dev`'s paintable page.
    viewmodel::PrintLayout layoutFor(const QPagedPaintDevice& dev, const PrintRequest& request) const;
    /// Paints the request onto `dev` (a new page per tile). Returns the page count, or 0 with `error`.
    int render(QPagedPaintDevice& dev, const PrintRequest& request, std::string& error) const;
    /// Writes the request as an A4 PDF (checked path, atomic write). Returns "" or why not; `pages`
    /// receives the page count.
    std::string printToPdf(const std::filesystem::path& path, const PrintRequest& request, bool overwrite, int* pages);

    /// The three interactive entry points (modal dialogs).
    void printDialog();
    void previewDialog();
    void printPdfDialog();

   private:
    bool askRequest(PrintRequest& request);

    MainWindow& window_;
    QAction* printAction_{nullptr};
    QAction* previewAction_{nullptr};
    QAction* printPdfAction_{nullptr};
    PrintRequest last_;
};

}  // namespace carvetto::app
