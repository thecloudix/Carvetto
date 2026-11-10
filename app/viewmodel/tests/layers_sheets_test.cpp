#include "carvetto/viewmodel/layers_sheets.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

ObjectId addCircle(Document& doc, LayerId layer, SheetId sheet) {
    auto cmd = std::make_unique<AddObjectCommand>(
        Object::vector(carvetto::geometry::Contour::circle({20, 20}, 5), layer, sheet, CircleParams{{20, 20}, 5}));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

}  // namespace

TEST_CASE("addLayer adds one undo step, names itself and becomes active", "[viewmodel][layers]") {
    Document doc;
    const auto r = addLayer(doc);
    REQUIRE(r.ok);
    REQUIRE(doc.layers().size() == 2);
    CHECK(doc.layers().back().name == "Layer 2");
    CHECK(doc.activeLayer() == doc.layers().back().id);
    CHECK(doc.undoStack().undoText() == "Add layer");
    REQUIRE(addLayer(doc, "Toolpath guides").ok);
    CHECK(doc.layers().back().name == "Toolpath guides");
    CHECK(doc.undo());
    CHECK(doc.layers().size() == 2);
}

TEST_CASE("layer edits are validated and each is its own undo step", "[viewmodel][layers]") {
    Document doc;
    const LayerId id = doc.layers().front().id;
    const std::size_t steps = doc.undoStack().count();

    CHECK_FALSE(renameLayer(doc, id, "").ok);
    CHECK_FALSE(renameLayer(doc, id, "tab\there").ok);
    CHECK_FALSE(renameLayer(doc, id, std::string(10000, 'a')).ok);
    CHECK_FALSE(renameLayer(doc, LayerId{9999}, "x").ok);
    CHECK(doc.undoStack().count() == steps);
    CHECK(doc.layers().front().name == "Layer 1");

    REQUIRE(renameLayer(doc, id, "Outline").ok);
    REQUIRE(setLayerVisible(doc, id, false).ok);
    REQUIRE(setLayerLocked(doc, id, true).ok);
    REQUIRE(setLayerColour(doc, id, {10, 20, 30}).ok);
    CHECK(doc.undoStack().count() == steps + 4);  // none merged
    const Layer& l = doc.layers().front();
    CHECK(l.name == "Outline");
    CHECK_FALSE(l.visible);
    CHECK(l.locked);
    CHECK(l.colour == Colour{10, 20, 30});

    const std::size_t after = doc.undoStack().count();
    CHECK(setLayerVisible(doc, id, false).ok);  // already hidden: no-op, no step
    CHECK(doc.undoStack().count() == after);
    REQUIRE(doc.undo());
    CHECK(l.colour == Colour{});
}

TEST_CASE("deleting layers moves or deletes contents; the last layer stays", "[viewmodel][layers]") {
    Document doc;
    REQUIRE(addLayer(doc, "B").ok);
    const LayerId b = doc.layers()[1].id;
    const ObjectId o = addCircle(doc, b, doc.activeSheet());

    REQUIRE(deleteLayer(doc, b, false).ok);
    CHECK(doc.layers().size() == 1);
    CHECK(doc.findObject(o)->layer == doc.layers().front().id);
    REQUIRE(doc.undo());
    CHECK(doc.findObject(o)->layer == b);

    REQUIRE(deleteLayer(doc, b, true).ok);
    CHECK_FALSE(doc.hasObject(o));
    CHECK_FALSE(deleteLayer(doc, doc.layers().front().id, false).ok);
    CHECK_FALSE(deleteLayer(doc, LayerId{9999}, false).ok);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("moveLayer reorders and moveSelectionToLayer respects locks", "[viewmodel][layers]") {
    Document doc;
    REQUIRE(addLayer(doc, "B").ok);
    REQUIRE(addLayer(doc, "C").ok);
    const LayerId a = doc.layers()[0].id;
    const LayerId c = doc.layers()[2].id;
    REQUIRE(moveLayer(doc, c, 0).ok);
    CHECK(doc.layers()[0].id == c);
    CHECK_FALSE(moveLayer(doc, c, 7).ok);
    REQUIRE(doc.undo());
    CHECK(doc.layers()[0].id == a);

    const ObjectId o = addCircle(doc, a, doc.activeSheet());
    doc.selection().assign({o});
    REQUIRE(setLayerLocked(doc, c, true).ok);
    CHECK_FALSE(moveSelectionToLayer(doc, c).ok);
    CHECK(doc.findObject(o)->layer == a);
    REQUIRE(setLayerLocked(doc, c, false).ok);
    REQUIRE(moveSelectionToLayer(doc, c).ok);
    CHECK(doc.findObject(o)->layer == c);
    // From a locked layer: refused.
    REQUIRE(setLayerLocked(doc, c, true).ok);
    CHECK_FALSE(moveSelectionToLayer(doc, a).ok);
    CHECK(doc.findObject(o)->layer == c);
}

TEST_CASE("layer count is capped", "[viewmodel][layers]") {
    Document doc;
    for (std::size_t i = doc.layers().size(); i < kMaxLayers; ++i) {
        REQUIRE(addLayer(doc).ok);
    }
    const std::size_t steps = doc.undoStack().count();
    CHECK_FALSE(addLayer(doc).ok);
    CHECK(doc.undoStack().count() == steps);
}

TEST_CASE("sheets: add, rename, switch material, delete", "[viewmodel][sheets]") {
    Document doc;
    const SheetId first = doc.sheets().front().id;
    REQUIRE(addSheet(doc).ok);
    REQUIRE(doc.sheets().size() == 2);
    CHECK(doc.sheets()[1].name == "Sheet 2");
    CHECK(doc.activeSheet() == doc.sheets()[1].id);
    CHECK(doc.sheets()[1].sizeMm == doc.sheets()[0].sizeMm);
    const SheetId second = doc.sheets()[1].id;

    REQUIRE(renameSheet(doc, second, "Back side").ok);
    CHECK_FALSE(renameSheet(doc, second, "").ok);
    CHECK_FALSE(renameSheet(doc, SheetId{9999}, "x").ok);

    SheetMaterialEdit e;
    e.widthMm = 400;
    e.thicknessMm = 12;
    e.zZero = ZZero::MachineBed;
    e.datum = Datum::Centre;
    REQUIRE(setSheetMaterial(doc, second, e).ok);
    CHECK(doc.findSheet(second)->sizeMm.x == 400);
    CHECK(doc.findSheet(second)->sizeMm.z == 12);
    CHECK(doc.findSheet(second)->zZero == ZZero::MachineBed);
    CHECK(doc.findSheet(first)->sizeMm.x == 300);  // the other sheet is untouched
    CHECK(doc.undoStack().undoText() == "Edit sheet");

    const std::size_t steps = doc.undoStack().count();
    SheetMaterialEdit bad;
    bad.widthMm = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(setSheetMaterial(doc, second, bad).ok);
    bad.widthMm = -1;
    CHECK_FALSE(setSheetMaterial(doc, second, bad).ok);
    bad.widthMm = 0;
    CHECK_FALSE(setSheetMaterial(doc, second, bad).ok);
    bad.widthMm = 1e12;
    CHECK_FALSE(setSheetMaterial(doc, second, bad).ok);
    bad.widthMm = std::numeric_limits<double>::infinity();
    CHECK_FALSE(setSheetMaterial(doc, second, bad).ok);
    CHECK(doc.undoStack().count() == steps);

    const ObjectId o = addCircle(doc, doc.activeLayer(), second);
    REQUIRE(deleteSheet(doc, second).ok);
    CHECK_FALSE(doc.hasObject(o));
    CHECK(doc.activeSheet() == first);
    CHECK_FALSE(deleteSheet(doc, first).ok);  // last sheet
    REQUIRE(doc.undo());
    CHECK(doc.hasObject(o));
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("history lists the steps and jumps both ways", "[viewmodel][history]") {
    Document doc;
    REQUIRE(addLayer(doc, "A").ok);
    REQUIRE(addLayer(doc, "B").ok);
    REQUIRE(addLayer(doc, "C").ok);
    auto entries = historyEntries(doc);
    REQUIRE(entries.size() == 3);
    CHECK(entries[0].text == "Add layer");
    CHECK(entries[2].applied);

    REQUIRE(jumpToHistory(doc, 1).ok);
    CHECK(doc.layers().size() == 2);
    entries = historyEntries(doc);
    CHECK(entries[0].applied);
    CHECK_FALSE(entries[1].applied);
    CHECK_FALSE(entries[2].applied);

    REQUIRE(jumpToHistory(doc, 3).ok);
    CHECK(doc.layers().size() == 4);
    REQUIRE(jumpToHistory(doc, 0).ok);
    CHECK(doc.layers().size() == 1);
    CHECK_FALSE(jumpToHistory(doc, 4).ok);
    CHECK(doc.layers().size() == 1);

    // A new edit after a jump drops the redo tail.
    REQUIRE(jumpToHistory(doc, 1).ok);
    REQUIRE(addLayer(doc, "D").ok);
    CHECK(historyEntries(doc).size() == 2);
}

TEST_CASE("replacing the document between calls leaves nothing dangling", "[viewmodel][layers]") {
    auto doc = std::make_unique<Document>();
    REQUIRE(addLayer(*doc, "A").ok);
    const LayerId stale = doc->layers().back().id;
    doc = std::make_unique<Document>();  // File > New: ids from the old document mean nothing
    CHECK_FALSE(renameLayer(*doc, stale, "x").ok);
    CHECK_FALSE(deleteLayer(*doc, stale, true).ok);
    CHECK_FALSE(setLayerVisible(*doc, stale, false).ok);
    CHECK(historyEntries(*doc).empty());
}

TEST_CASE("makeDoubleSided pairs a back sheet in one undo step", "[viewmodel][sheets][double-sided]") {
    Document doc;
    const SheetId front = doc.sheets().front().id;
    const std::size_t steps = doc.undoStack().count();
    REQUIRE(makeDoubleSided(doc, FlipDirection::Vertical, true).ok);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.job().type == JobType::DoubleSided);
    CHECK(doc.job().flipDirection == FlipDirection::Vertical);
    CHECK(doc.job().zeroOffSameSide);
    REQUIRE(doc.sheets().size() == 2);
    const Sheet& back = doc.sheets()[1];
    CHECK(back.side == Side::Back);
    CHECK(back.name == "Sheet 1 (back)");
    CHECK(back.partner == front);
    CHECK(doc.findSheet(front)->partner == back.id);
    CHECK(doc.activeSheet() == front);
    CHECK(doc.checkIntegrity().empty());

    // Again: only the flip settings change, no second back sheet.
    REQUIRE(makeDoubleSided(doc, FlipDirection::Horizontal, false).ok);
    CHECK(doc.sheets().size() == 2);
    CHECK(doc.job().flipDirection == FlipDirection::Horizontal);

    REQUIRE(doc.undoStack().canUndo());
    doc.undo();
    doc.undo();
    CHECK(doc.sheets().size() == 1);
    CHECK(doc.findSheet(front)->partner == SheetId{});
    CHECK(doc.job().type == JobType::SingleSided);
}

TEST_CASE("makeDoubleSided is refused for rotary jobs and for a lone back sheet", "[viewmodel][sheets][double-sided]") {
    Document rotary(Job::rotary(400, 60, Units::Millimetres));
    CHECK_FALSE(makeDoubleSided(rotary, FlipDirection::Horizontal, false).ok);
    CHECK(rotary.job().type == JobType::Rotary);

    Document doc;
    doc.findSheet(doc.activeSheet())->side = Side::Back;
    CHECK_FALSE(makeDoubleSided(doc, FlipDirection::Horizontal, false).ok);
}

TEST_CASE("swapSides activates the other face; setFlipDirection needs a double-sided job",
          "[viewmodel][sheets][double-sided]") {
    Document doc;
    CHECK_FALSE(swapSides(doc).ok);
    CHECK_FALSE(setFlipDirection(doc, FlipDirection::Vertical).ok);
    const SheetId front = doc.activeSheet();
    REQUIRE(makeDoubleSided(doc, FlipDirection::Horizontal, false).ok);
    REQUIRE(swapSides(doc).ok);
    CHECK(doc.activeSheet() == doc.sheets()[1].id);
    REQUIRE(swapSides(doc).ok);
    CHECK(doc.activeSheet() == front);
    REQUIRE(setFlipDirection(doc, FlipDirection::Vertical).ok);
    CHECK(doc.job().flipDirection == FlipDirection::Vertical);
    CHECK(setFlipDirection(doc, FlipDirection::Vertical).message.empty());
}

TEST_CASE("deleting one face unpairs the other, and undo restores the pair", "[viewmodel][sheets][double-sided]") {
    Document doc;
    const SheetId front = doc.activeSheet();
    REQUIRE(makeDoubleSided(doc, FlipDirection::Horizontal, false).ok);
    const SheetId back = doc.sheets()[1].id;
    REQUIRE(deleteSheet(doc, back).ok);
    CHECK(doc.findSheet(front)->partner == SheetId{});
    CHECK(doc.checkIntegrity().empty());
    doc.undo();
    CHECK(doc.findSheet(front)->partner == back);
    CHECK(doc.findSheet(back) != nullptr);
}

TEST_CASE("addSheet never copies the pairing", "[viewmodel][sheets][double-sided]") {
    Document doc;
    REQUIRE(makeDoubleSided(doc, FlipDirection::Horizontal, false).ok);
    REQUIRE(swapSides(doc).ok);  // active = back sheet
    REQUIRE(addSheet(doc).ok);
    const Sheet& added = *doc.findSheet(doc.activeSheet());
    CHECK(added.side == Side::Front);
    CHECK(added.partner == SheetId{});
}

TEST_CASE("multiSidedView shows the partner's drawing mirrored into the active frame",
          "[viewmodel][sheets][double-sided]") {
    Document doc(Job::doubleSided(100, 50, 10, Units::Millimetres));
    CHECK(multiSidedView(doc).size() == 1);
    const SheetId front = doc.activeSheet();
    REQUIRE(makeDoubleSided(doc, FlipDirection::Horizontal, false).ok);
    const SheetId back = doc.sheets()[1].id;
    addCircle(doc, doc.layers().front().id, back);  // centre (20,20) r 5 -> x 15..25

    const auto view = multiSidedView(doc);
    REQUIRE(view.size() == 2);
    CHECK(view[0].sheet == front);
    CHECK(view[0].active);
    CHECK(view[0].content.isEmpty());
    CHECK(view[0].otherSideContent.min.x == Catch::Approx(75));
    CHECK(view[0].otherSideContent.max.x == Catch::Approx(85));
    CHECK(view[1].sheet == back);
    CHECK_FALSE(view[1].active);
    CHECK(view[1].content.min.x == Catch::Approx(15));
    CHECK(view[1].otherSideContent.isEmpty());
}

TEST_CASE("sheetTopZ follows the job's zero settings per side", "[viewmodel][sheets][double-sided]") {
    Document doc(Job::doubleSided(100, 50, 12, Units::Millimetres, Datum::BottomLeft, ZZero::MaterialSurface));
    REQUIRE(makeDoubleSided(doc, FlipDirection::Horizontal, true).ok);
    CHECK(sheetTopZ(doc, doc.sheets()[0].id) == 0.0);
    CHECK(sheetTopZ(doc, doc.sheets()[1].id) == 12.0);
    REQUIRE(makeDoubleSided(doc, FlipDirection::Horizontal, false).ok);
    CHECK(sheetTopZ(doc, doc.sheets()[1].id) == 0.0);
    CHECK(sheetTopZ(doc, SheetId{424242}) == 0.0);
}
