// Self-test of File > Create Job Sheet (T-0259).
#include "job_sheet_self_test.hpp"

#include <QAction>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <filesystem>

#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/job_sheet.hpp"
#include "main_window.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

std::string readAll(const std::filesystem::path& p) {
    QFile f(QString::fromStdString(viewmodel::pathToUtf8(p)));
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    const QByteArray b = f.readAll();
    return std::string(b.constData(), static_cast<std::size_t>(b.size()));
}

}  // namespace

std::string jobSheetSelfTest(MainWindow& window) {
    using viewmodel::JobSheetFormat;
    if (window.createJobSheetAction() == nullptr) {
        return "the File menu has no Create Job Sheet action";
    }
    QTemporaryDir dir;
    if (!dir.isValid()) {
        return "no temporary directory for the job sheet test";
    }
    const std::filesystem::path base = viewmodel::pathFromUtf8(dir.path().toStdString());
    const viewmodel::JobSheet sheet =
        viewmodel::makeJobSheet(window.currentDocument(), window.toolpaths()->session().summary(), "");

    const std::filesystem::path html = base / "sheet.html";
    if (const std::string why = window.createJobSheetFile(html, JobSheetFormat::Html, false, false); !why.empty()) {
        return "writing the HTML job sheet failed: " + why;
    }
    if (readAll(html) != viewmodel::renderJobSheetHtml(sheet)) {
        return "the HTML job sheet differs from makeJobSheet's output";
    }
    const std::filesystem::path txt = base / "sheet.txt";
    if (!window.createJobSheetFile(txt, JobSheetFormat::Text, false, false).empty() ||
        readAll(txt) != viewmodel::renderJobSheetText(sheet)) {
        return "the text job sheet is missing or differs from makeJobSheet's output";
    }
    const std::filesystem::path pdf = base / "sheet.pdf";
    if (const std::string why = window.createJobSheetFile(pdf, JobSheetFormat::Pdf, false, false); !why.empty()) {
        return "writing the PDF job sheet failed: " + why;
    }
    if (readAll(pdf).rfind("%PDF", 0) != 0) {
        return "the PDF job sheet is not a PDF";
    }
    // Refusals: existing file without overwrite, '..', wrong extension, control characters.
    if (window.createJobSheetFile(html, JobSheetFormat::Html, false, false).empty()) {
        return "an existing job sheet was replaced without overwrite";
    }
    if (window.createJobSheetFile(pdf, JobSheetFormat::Pdf, false, false).empty()) {
        return "an existing PDF job sheet was replaced without overwrite";
    }
    if (window.createJobSheetFile(base / ".." / "x.html", JobSheetFormat::Html, true, false).empty() ||
        window.createJobSheetFile(base / "x.exe", JobSheetFormat::Html, true, false).empty() ||
        window.createJobSheetFile(base / "a\nb.txt", JobSheetFormat::Text, true, false).empty() ||
        window.createJobSheetFile(base / "x.html", JobSheetFormat::Pdf, true, false).empty()) {
        return "a hostile job sheet path was accepted";
    }
    std::size_t count = 0;
    for (const auto& e : std::filesystem::directory_iterator(base)) {
        (void)e;
        ++count;
    }
    if (count != 3) {
        return "the job sheet test left stray files";
    }
    return {};
}

}  // namespace carvetto::app
