// Self-test of the Import Parts List dialog (T-0336): drives the File menu entry's dialog, the diagnostics, the
// partial-import switch, the tool pickers and the one-undo-step apply on the real MainWindow offscreen.

#include <QAction>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QTemporaryDir>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "main_window.hpp"
#include "parts_import_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define PI_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("parts import: ") + (text); \
        }                                               \
    } while (false)

const char* kMapping =
    "units = mm\ncolumn.length = Length\ncolumn.width = Width\ncolumn.thickness = Thickness\ncolumn.material = Material\n"
    "column.label = Label\ncolumn.quantity = Qty\ncolumn.holes = Holes\ncolumn.dados = Dados\n"
    "sheet.length_mm = 1200\nsheet.width_mm = 800\nnest.gap_mm = 10\nnest.margin_mm = 10\n";

const char* kParts =
    "Label,Length,Width,Thickness,Material,Qty,Holes,Dados\n"
    "Side,600,300,18,Birch,2,50:50:3|550:50:3:6,L:100:10:6\n"
    "Shelf,500,280,18,Birch,1,,W:250:12:5\n"
    "Back,600,500,6,Hardboard,1,,\n";

// One row has a bad length, the others are fine.
const char* kPartsOneBad =
    "Label,Length,Width,Thickness,Material,Qty,Holes,Dados\n"
    "Side,600,300,18,Birch,2,,\n"
    "Broken,abc,300,18,Birch,1,,\n"
    "Back,600,500,6,Hardboard,1,,\n";

bool writeFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out << text;
    return static_cast<bool>(out);
}

}  // namespace

std::string partsImportSelfTest(MainWindow& window) {
    window.setQuiet(true);
    QAction* action = window.importPartsAction();
    PI_REQUIRE(action != nullptr && action->isEnabled(), "File > Import Parts List is missing");

    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(1200.0, 800.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    const tooling::ToolDatabase db = window.toolpaths()->toolDatabase();

    // 1. Nothing entered: nothing to import.
    {
        PartsImportDialog dlg(db);
        PI_REQUIRE(!dlg.canApply(), "OK is enabled with nothing entered");
        dlg.typeMapping(QStringLiteral("column.length = Length\ncolumn.length ="));  // broken mapping
        dlg.typeParts(QString::fromLatin1(kParts));
        PI_REQUIRE(!dlg.canApply(), "OK is enabled with a broken mapping");
        PI_REQUIRE(dlg.diagnosticsList()->count() > 0, "a broken mapping shows no diagnostics");
        bool numbered = false;
        for (int i = 0; i < dlg.diagnosticsList()->count(); ++i) {
            numbered = numbered || dlg.diagnosticsList()->item(i)->text().contains(QStringLiteral("line"));
        }
        PI_REQUIRE(numbered, "mapping diagnostics carry no line number");
    }

    // 2. Files: read through the file buttons' loader; a missing file reports why.
    QTemporaryDir dir;
    PI_REQUIRE(dir.isValid(), "no temporary directory");
    const std::filesystem::path base = viewmodel::pathFromUtf8(dir.path().toStdString());
    PI_REQUIRE(writeFile(base / "map.txt", kMapping) && writeFile(base / "parts.csv", kParts) &&
                   writeFile(base / "bad.csv", kPartsOneBad),
               "could not write the sample files");
    {
        PartsImportDialog dlg(db);
        PI_REQUIRE(!dlg.chooseMappingFile(QString::fromStdString((base / "nope.txt").string())).empty(),
                   "a missing mapping file loaded");
        PI_REQUIRE(dlg.chooseMappingFile(QString::fromStdString((base / "map.txt").string())).empty(),
                   "the mapping file did not load");
        PI_REQUIRE(dlg.choosePartsFile(QString::fromStdString((base / "bad.csv").string())).empty(),
                   "the parts file did not load");
        // 3. A bad row blocks, with a line-numbered diagnostic, until partial import is allowed.
        PI_REQUIRE(!dlg.canApply(), "OK is enabled although a row has an error");
        bool rowShown = false;
        for (int i = 0; i < dlg.diagnosticsList()->count(); ++i) {
            rowShown = rowShown || dlg.diagnosticsList()->item(i)->text().startsWith(QStringLiteral("Error (parts"));
        }
        PI_REQUIRE(rowShown, "the bad row's diagnostic is not shown");
        dlg.setAllowPartial(true);
        PI_REQUIRE(dlg.canApply(), "allowing a partial import did not enable OK");
        dlg.setAllowPartial(false);
        PI_REQUIRE(!dlg.canApply(), "turning partial import off did not disable OK");
        PI_REQUIRE(dlg.choosePartsFile(QString::fromStdString((base / "parts.csv").string())).empty(),
                   "the good parts file did not load");
        PI_REQUIRE(dlg.canApply(), "OK is disabled for a good parts list");
        PI_REQUIRE(!dlg.pickTool(document::ToolpathOperation::Profile, "no-such-tool") &&
                       !dlg.pickTool(document::ToolpathOperation::Engrave, "end-mill-3mm"),
                   "a tool that is not offered was picked");
    }

    // 4. The window's own dialog flow: build the dialog as the menu does and apply through the window.
    const std::size_t steps = doc.undoStack().count();
    {
        PartsImportDialog dlg(db);
        dlg.typeMapping(QString::fromLatin1(kMapping));
        dlg.typeParts(QString::fromLatin1(kParts));
        PI_REQUIRE(dlg.pickTool(document::ToolpathOperation::Profile, "end-mill-3mm") &&
                       dlg.pickTool(document::ToolpathOperation::Drill, "drill-1-8in") &&
                       dlg.pickTool(document::ToolpathOperation::Pocket, "end-mill-3mm"),
                   "the tools cannot be picked");
        PI_REQUIRE(dlg.canApply(), "OK is disabled for a good import");
        const viewmodel::PartsImportOutcome o = window.applyPartsImport(dlg.form());
        PI_REQUIRE(o.ok, "the import failed: " + o.message);
        PI_REQUIRE(o.panels == 4 && o.sheets == 2, "unexpected panel or sheet count");
        PI_REQUIRE(o.toolpaths.size() == 6, "expected 2 pocket, 2 drill and 2 profile toolpaths");
        PI_REQUIRE(doc.undoStack().count() == steps + 1, "the import is not one undo step");
        PI_REQUIRE(window.session().isDirty(), "the document is not dirty after the import");
    }
    PI_REQUIRE(window.undo(), "undo failed");
    PI_REQUIRE(doc.toolpaths().empty() && doc.sheets().size() == 1, "undo did not remove the whole import");
    PI_REQUIRE(window.redo() && doc.sheets().size() == 2, "redo did not restore the import");
    PI_REQUIRE(window.undo(), "final undo failed");

    // 5. Without toolpaths: the panels only.
    {
        PartsImportDialog dlg(db);
        dlg.typeMapping(QString::fromLatin1(kMapping));
        dlg.typeParts(QString::fromLatin1(kParts));
        dlg.setCreateToolpaths(false);
        const viewmodel::PartsImportOutcome o = window.applyPartsImport(dlg.form());
        PI_REQUIRE(o.ok && o.panels == 4 && o.toolpaths.empty() && doc.toolpaths().empty(),
                   "an import without toolpaths created some");
    }
    PI_REQUIRE(window.undo(), "undo of the panels-only import failed");
    return {};
}

}  // namespace carvetto::app
