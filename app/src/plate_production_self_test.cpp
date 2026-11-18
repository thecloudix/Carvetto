// Self-test of Plate Production (T-0347): drives the menu action, the real dialog widgets and the merge of a
// 20-row CSV on the MainWindow offscreen.

#include "plate_production_self_test.hpp"

#include <QString>
#include <cstdint>
#include <memory>
#include <string>
#include <variant>

#include "array_copy_menu.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/viewmodel/data_merge.hpp"
#include "carvetto/viewmodel/starter_font.hpp"
#include "main_window.hpp"
#include "plate_production_dialog.hpp"

namespace carvetto::app {

namespace {

#define PP_REQUIRE(cond, text)                              \
    do {                                                    \
        if (!(cond)) {                                      \
            return std::string("plate production: ") + (text); \
        }                                                   \
    } while (false)

}  // namespace

std::string plateProductionSelfTest(MainWindow& window) {
    document::Document& doc = window.currentDocument();
    CopiesMenu* copies = window.copies();
    PP_REQUIRE(copies != nullptr && copies->plateAction() != nullptr, "the Copies menu has no Plate Production entry");

    const std::size_t objectsBefore = doc.objectCount();
    const std::size_t selectionBefore = doc.selection().ids().size();
    const auto selectedBefore = doc.selection().ids();

    // A template: a 60 x 30 plate with a label holding two placeholders.
    const auto bytes = viewmodel::starterFontBytes();
    document::FontAsset asset;
    asset.bytes.assign(bytes.begin(), bytes.end());
    asset.key = document::fontKeyFor(asset.bytes);
    PP_REQUIRE(doc.insertFont(asset) || doc.findFont(asset.key) != nullptr, "could not embed the font");
    document::TextParams params;
    params.text = "!!NAME!! !!SERIAL!!";
    params.fontKey = asset.key;
    params.sizeMm = 6.0;
    params.placement = geometry::Transform2::translation({4.0, 12.0});
    text::FontLoad load = text::Font::load(asset.bytes);
    PP_REQUIRE(load.ok(), "the starter font does not load");
    document::TextBuild built = document::buildTextContours(load.font, params);
    PP_REQUIRE(built.ok(), "the template text cannot be laid out");
    auto rect = std::make_unique<document::AddObjectCommand>(document::Object::vector(
        geometry::Contour::rectangle(geometry::Box2{{0.0, 0.0}, {60.0, 30.0}}), doc.activeLayer(), doc.activeSheet()));
    const document::AddObjectCommand* rectRaw = rect.get();
    PP_REQUIRE(doc.execute(std::move(rect)), "could not add the template plate");
    auto label = std::make_unique<document::AddObjectCommand>(
        document::Object::vector(std::move(built.contours), doc.activeLayer(), doc.activeSheet(), params));
    const document::AddObjectCommand* labelRaw = label.get();
    PP_REQUIRE(doc.execute(std::move(label)), "could not add the template label");
    doc.selection().assign({rectRaw->id(), labelRaw->id()});
    copies->refresh();
    PP_REQUIRE(copies->plateAction()->isEnabled(), "Plate Production is not enabled for a selected template");
    const std::size_t undoDepth = doc.undoStack().index();
    const std::size_t objectsWithTemplate = doc.objectCount();

    std::string csv = "Name,Qty\n";
    for (int i = 1; i <= 20; ++i) {
        csv += "Plate " + std::to_string(i) + "," + std::to_string(i) + "\n";
    }

    // The dialog assigns NAME from the header, lists SERIAL as unassigned and previews 20 plates.
    {
        PlateProductionDialog dialog(doc, &window);
        dialog.setCsvText(QString::fromStdString(csv));
        PP_REQUIRE(dialog.assignmentRows() == 2, "the dialog does not list both placeholders");
        dialog.refreshPreview();
        PP_REQUIRE(dialog.summary().contains(QStringLiteral("SERIAL")),
                   "the unassigned placeholder is not reported: " + dialog.summary().toStdString());
        dialog.setCsvText(QString::fromStdString("Name,SERIAL\n" + csv.substr(csv.find('\n') + 1)));
        dialog.refreshPreview();
        PP_REQUIRE(dialog.summary().startsWith(QStringLiteral("20 plate(s)")),
                   "the preview does not report 20 plates: " + dialog.summary().toStdString());
        PP_REQUIRE(doc.objectCount() == objectsWithTemplate, "the preview changed the document");
    }

    // Merge through the window with a counter.
    viewmodel::DataMergeOptions options;
    options.assignments = {{"NAME", 0}};
    viewmodel::CounterSpec serial;
    serial.name = "SERIAL";
    serial.width = 3;
    serial.prefix = "SN";
    options.counters = {serial};
    const viewmodel::DataMergeResult made = runPlateProduction(window, csv, options, false);
    PP_REQUIRE(made.ok, "the merge failed: " + made.message);
    PP_REQUIRE(made.plates == 20 && made.created.size() == 40 && doc.objectCount() == objectsWithTemplate + 40,
               "the merge did not make 20 plates");
    PP_REQUIRE(doc.undoStack().index() == undoDepth + 1, "the merge is not one undo step");
    {
        const document::Object* last = doc.findObject(made.created.back());
        const auto* tp = last != nullptr && last->asVector() != nullptr
                             ? std::get_if<document::TextParams>(&last->asVector()->params)
                             : nullptr;
        PP_REQUIRE(tp != nullptr && tp->text == "Plate 20 SN020", "the last plate has the wrong text");
    }
    PP_REQUIRE(window.undo() && doc.objectCount() == objectsWithTemplate, "undoing the merge did not remove the plates");

    doc.selection().assign({rectRaw->id(), labelRaw->id()});

    // A malformed file is refused with line numbers and nothing changes.
    const viewmodel::DataMergeResult refused = runPlateProduction(window, "Name,Qty\nA,1\nB\n", options, false);
    PP_REQUIRE(!refused.ok && !refused.diagnostics.empty() && refused.diagnostics.front().line == 3,
               "the malformed row was not reported at line 3");
    PP_REQUIRE(doc.objectCount() == objectsWithTemplate, "a refused merge changed the document");

    // Restore the document for the tests that follow.
    while (doc.objectCount() > objectsBefore && doc.undoStack().canUndo()) {
        window.undo();
    }
    PP_REQUIRE(doc.objectCount() == objectsBefore, "could not remove the template again");
    doc.selection().assign(selectedBefore);
    PP_REQUIRE(doc.selection().ids().size() == selectionBefore, "the selection was not restored");
    return {};
}

}  // namespace carvetto::app
