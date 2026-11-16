#include "carvetto/viewmodel/parts_import_form.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>

#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using document::ToolpathOperation;

namespace {

const char* kMapping =
    "units = mm\ncolumn.length = Length\ncolumn.width = Width\ncolumn.thickness = Thickness\ncolumn.material = Material\n"
    "column.label = Label\ncolumn.quantity = Qty\ncolumn.holes = Holes\ncolumn.dados = Dados\n"
    "sheet.length_mm = 1200\nsheet.width_mm = 800\nnest.gap_mm = 10\nnest.margin_mm = 10\n";

const char* kParts =
    "Label,Length,Width,Thickness,Material,Qty,Holes,Dados\n"
    "Side,600,300,18,Birch,2,50:50:3|550:50:3:6,L:100:10:6\n"
    "Shelf,500,280,18,Birch,1,,W:250:12:5\n"
    "Back,600,500,6,Hardboard,1,,\n";

struct Fixture {
    document::Document doc;
    ToolpathSession session{&doc};
    PartsImportForm form;
    Fixture() {
        session.setToolDatabase(testing::cliToolDatabase());
        form.setMappingText(kMapping);
        form.setPartsText(kParts);
        form.setTool(ToolpathOperation::Profile, testing::source("end-mill-3mm"));
        form.setTool(ToolpathOperation::Drill, testing::source("drill-3mm"));
        form.setTool(ToolpathOperation::Pocket, testing::source("end-mill-3mm"));
    }
};

}  // namespace

TEST_CASE("parts import form: the preview reports counts and diagnostics", "[parts][viewmodel]") {
    PartsImportForm form;
    form.setMappingText(kMapping);
    form.setPartsText(kParts);
    const PartsImportPreview& p = form.preview();
    CHECK(p.mappingOk);
    CHECK(p.partsRead);
    CHECK(p.parts == 3);
    CHECK(p.instances == 4);
    CHECK(p.errors == 0);
    CHECK(p.canApply);

    form.setPartsText(std::string(kParts) + "Bad,abc,300,18,Birch,1,,\n");
    const PartsImportPreview& q = form.preview();
    CHECK(q.errors == 1);
    CHECK(q.rowsSkipped == 1);
    CHECK(q.parts == 3);
    CHECK_FALSE(q.canApply);
    form.setAllowPartial(true);
    CHECK(form.preview().canApply);
}

TEST_CASE("parts import form: mapping errors block the import with line diagnostics", "[parts][viewmodel]") {
    Fixture f;
    f.form.setMappingText("units = mm\ncolumn.length = Length\nbogus = 1\n");
    const PartsImportPreview& p = f.form.preview();
    CHECK_FALSE(p.mappingOk);
    CHECK_FALSE(p.canApply);
    CHECK(p.parts == 0);
    const std::size_t steps = f.doc.undoStack().count();
    const PartsImportOutcome o = f.form.apply(f.doc, &f.session);
    CHECK_FALSE(o.ok);
    CHECK(f.doc.undoStack().count() == steps);
    bool line3 = false;
    for (const auto& d : o.diagnostics) line3 = line3 || (d.line == 3 && d.text.find("unknown key") != std::string::npos);
    CHECK(line3);
}

TEST_CASE("parts import form: panels, nesting and toolpaths in one undo step", "[parts][viewmodel]") {
    Fixture f;
    const std::size_t steps = f.doc.undoStack().count();
    const PartsImportOutcome o = f.form.apply(f.doc, &f.session);
    REQUIRE(o.ok);
    CHECK(o.panels == 4);
    CHECK(o.sheets == 2);  // Birch 18 and Hardboard 6
    CHECK(f.doc.undoStack().count() == steps + 1);

    // Birch sheet: dados (L 6 and W 5), holes (through 18 and blind 6), profile; Hardboard sheet: profile.
    const auto items = f.session.list();
    REQUIRE(items.size() == o.toolpaths.size());
    std::size_t pockets = 0, drills = 0, profiles = 0;
    for (const auto& it : items) {
        if (it.operation == ToolpathOperation::Pocket) ++pockets;
        if (it.operation == ToolpathOperation::Drill) ++drills;
        if (it.operation == ToolpathOperation::Profile) ++profiles;
        CHECK(it.moveCount > 0);
    }
    CHECK(pockets == 2);   // two distinct dado depths
    CHECK(drills == 2);    // through (18) and blind (6)
    CHECK(profiles == 2);  // one per sheet
    CHECK(f.doc.activeSheet() == f.doc.sheets().front().id);

    // One undo removes everything: panels, sheets, layers and toolpaths.
    REQUIRE(f.doc.undoStack().undo(f.doc));
    CHECK(f.doc.toolpaths().empty());
    CHECK(f.doc.sheets().size() == 1);
    CHECK(f.doc.objectsOnSheet(f.doc.activeSheet()).empty());
    REQUIRE(f.doc.undoStack().redo(f.doc));
    CHECK(f.doc.toolpaths().size() == o.toolpaths.size());
}

TEST_CASE("parts import form: no tool, no toolpath of that kind, with a note", "[parts][viewmodel]") {
    Fixture f;
    f.form.setTool(ToolpathOperation::Drill, {});
    const PartsImportOutcome o = f.form.apply(f.doc, &f.session);
    REQUIRE(o.ok);
    const auto items = f.session.list();
    CHECK(std::none_of(items.begin(), items.end(),
                       [](const ToolpathListItem& i) { return i.operation == ToolpathOperation::Drill; }));
    CHECK(std::any_of(o.diagnostics.begin(), o.diagnostics.end(), [](const io::PartsDiagnostic& d) {
        return d.source == "toolpath" && d.text.find("no drilling tool chosen") != std::string::npos;
    }));
}

TEST_CASE("parts import form: a refused tool is reported and the panels stay", "[parts][viewmodel]") {
    Fixture f;
    f.form.setTool(ToolpathOperation::Profile, testing::source("no-such-tool"));
    const PartsImportOutcome o = f.form.apply(f.doc, &f.session);
    REQUIRE(o.ok);
    CHECK(o.panels == 4);
    CHECK(std::any_of(o.diagnostics.begin(), o.diagnostics.end(),
                      [](const io::PartsDiagnostic& d) { return d.source == "toolpath" && d.text.find("profile tool") == 0; }));
}

TEST_CASE("parts import form: toolpaths can be switched off or need a session", "[parts][viewmodel]") {
    {
        Fixture f;
        f.form.setCreateToolpaths(false);
        const PartsImportOutcome o = f.form.apply(f.doc, &f.session);
        REQUIRE(o.ok);
        CHECK(o.toolpaths.empty());
        CHECK(f.doc.toolpaths().empty());
    }
    {
        Fixture f;
        const PartsImportOutcome o = f.form.apply(f.doc, nullptr);
        REQUIRE(o.ok);
        CHECK(f.doc.toolpaths().empty());
        CHECK(o.panels == 4);
    }
}

TEST_CASE("parts import form: errors block unless partial import is allowed", "[parts][viewmodel]") {
    Fixture f;
    f.form.setPartsText(std::string(kParts) + "Bad,abc,300,18,Birch,1,,\n");
    const std::size_t steps = f.doc.undoStack().count();
    PartsImportOutcome o = f.form.apply(f.doc, &f.session);
    CHECK_FALSE(o.ok);
    CHECK(f.doc.undoStack().count() == steps);
    f.form.setAllowPartial(true);
    o = f.form.apply(f.doc, &f.session);
    REQUIRE(o.ok);
    CHECK(o.panels == 4);
}

TEST_CASE("parts import form: a panel too big for the sheet blocks the import", "[parts][viewmodel]") {
    Fixture f;
    f.form.setPartsText("Label,Length,Width,Thickness,Material,Qty,Holes,Dados\nHuge,2000,300,18,Birch,1,,\n");
    const std::size_t steps = f.doc.undoStack().count();
    const PartsImportOutcome o = f.form.apply(f.doc, &f.session);
    CHECK_FALSE(o.ok);
    CHECK(f.doc.undoStack().count() == steps);
    CHECK(f.doc.toolpaths().empty());
}

TEST_CASE("parts import form: files are read with a size bound", "[parts][viewmodel]") {
    const std::filesystem::path dir = std::filesystem::path(CARVETTO_VM_WORK_DIR) / "parts_import";
    std::filesystem::create_directories(dir);
    {
        std::ofstream(dir / "map.txt", std::ios::binary) << kMapping;
        std::ofstream(dir / "parts.csv", std::ios::binary) << kParts;
        std::ofstream big(dir / "big.txt", std::ios::binary);
        big << std::string(io::kMaxMappingBytes + 1, '#');
    }
    PartsImportForm form;
    CHECK(form.loadMappingFile((dir / "map.txt").string()).empty());
    CHECK(form.loadPartsFile((dir / "parts.csv").string()).empty());
    CHECK(form.preview().canApply);
    CHECK_FALSE(form.loadMappingFile((dir / "big.txt").string()).empty());
    CHECK_FALSE(form.loadPartsFile((dir / "missing.csv").string()).empty());
    CHECK(form.preview().canApply);  // the failed loads changed nothing
}
