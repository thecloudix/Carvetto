#include "carvetto/io/parts_build.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"

using namespace carvetto;
using namespace carvetto::io;

namespace {

const char* kMapping =
    "units = mm\ncolumn.length = L\ncolumn.width = W\ncolumn.thickness = T\ncolumn.material = M\n"
    "column.label = N\ncolumn.quantity = Q\ncolumn.holes = H\ncolumn.dados = D\n"
    "sheet.length_mm = 1000\nsheet.width_mm = 600\nnest.gap_mm = 5\nnest.margin_mm = 10\n";

PartsMapping mapping(const std::string& extra = {}) {
    const MappingResult r = parsePartsMapping(std::string(kMapping) + extra);
    REQUIRE(r.ok);
    return r.mapping;
}

PartsList parts(const std::string& rows, const PartsMapping& m) {
    const PartsList l = readPartsText("N,L,W,T,M,Q,H,D\n" + rows, m);
    REQUIRE_FALSE(l.hasErrors());
    return l;
}

geometry::Box2 boundsOf(const document::Document& doc, document::ObjectId id) {
    return doc.findObject(id)->asVector()->bounds();
}

}  // namespace

TEST_CASE("build: panels, holes and dados land on their layers, on a sheet sized from the mapping", "[parts][build]") {
    document::Document doc;
    const PartsMapping m = mapping();
    const PartsList l = parts("side,400,200,18,Oak,1,50:60:8|100:60:8:10,L:20:10:6\n", m);
    const BuildResult r = buildPanels(doc, l, m);
    REQUIRE(r.ok);
    REQUIRE(r.sheets.size() == 1);
    const BuiltSheet& s = r.sheets[0];
    CHECK(s.material == "Oak");
    CHECK(s.thicknessMm == 18.0);
    const document::Sheet* sheet = doc.findSheet(s.sheet);
    REQUIRE(sheet);
    CHECK(sheet->sizeMm.x == 1000.0);
    CHECK(sheet->sizeMm.y == 600.0);
    CHECK(sheet->sizeMm.z == 18.0);
    CHECK(sheet->name == "Oak 18mm #1");
    CHECK(doc.activeSheet() == s.sheet);
    CHECK(doc.sheets().size() == 1);  // the empty default sheet was reused

    REQUIRE(s.panels.size() == 1);
    const geometry::Box2 pb = boundsOf(doc, s.panels[0]);
    CHECK(pb.min.x == 10.0);
    CHECK(pb.min.y == 10.0);
    CHECK(pb.max.x == 410.0);
    CHECK(pb.max.y == 210.0);
    CHECK(doc.findObject(s.panels[0])->layer == r.panelLayer);
    CHECK(doc.findObject(s.panels[0])->name == "side");

    REQUIRE(s.holes.size() == 2);
    CHECK(doc.findObject(s.holes[0].object)->layer == r.holeLayer);
    const geometry::Box2 hb = boundsOf(doc, s.holes[0].object);
    CHECK(std::abs(hb.centre().x - 60.0) < 1e-6);
    CHECK(std::abs(hb.centre().y - 70.0) < 1e-6);
    CHECK(std::abs(hb.width() - 8.0) < 1e-6);
    CHECK(s.holes[0].depthMm == 0.0);
    CHECK(s.holes[1].depthMm == 10.0);

    REQUIRE(s.dados.size() == 1);
    CHECK(doc.findObject(s.dados[0].object)->layer == r.dadoLayer);
    const geometry::Box2 db = boundsOf(doc, s.dados[0].object);
    CHECK(std::abs(db.min.x - 10.0) < 1e-9);
    CHECK(std::abs(db.max.x - 410.0) < 1e-9);
    CHECK(std::abs(db.min.y - 30.0) < 1e-9);  // 10 + 20
    CHECK(std::abs(db.max.y - 40.0) < 1e-9);
    CHECK(s.dados[0].depthMm == 6.0);

    // Layers exist with the mapping's names.
    CHECK(doc.findLayer(r.panelLayer)->name == "Panels");
    CHECK(doc.findLayer(r.holeLayer)->name == "Holes");
    CHECK(doc.findLayer(r.dadoLayer)->name == "Dados");
}

TEST_CASE("build: a W dado crosses the width at the given distance", "[parts][build]") {
    document::Document doc;
    const PartsMapping m = mapping();
    const BuildResult r = buildPanels(doc, parts("a,400,200,18,,1,,W:30:12:5\n", m), m);
    REQUIRE(r.ok);
    const geometry::Box2 db = boundsOf(doc, r.sheets[0].dados[0].object);
    CHECK(std::abs(db.min.x - 40.0) < 1e-9);
    CHECK(std::abs(db.max.x - 52.0) < 1e-9);
    CHECK(std::abs(db.min.y - 10.0) < 1e-9);
    CHECK(std::abs(db.max.y - 210.0) < 1e-9);
}

TEST_CASE("build: a rotated panel maps holes and dados with a quarter turn", "[parts][build]") {
    document::Document doc;
    // A 500-long, 300-wide panel on a 400 x 600 sheet only fits turned (margin 10, usable 380 x 580).
    PartsMapping m = mapping();
    m.rotate = true;
    m.sheetLengthMm = 400;
    m.sheetWidthMm = 600;
    const PartsList l = parts("r,500,300,18,,1,100:50:20,L:20:10:5\n", m);
    const BuildResult r = buildPanels(doc, l, m);
    REQUIRE(r.ok);
    const BuiltSheet& s = r.sheets[0];
    const geometry::Box2 pb = boundsOf(doc, s.panels[0]);
    CHECK(std::abs(pb.width() - 300.0) < 1e-9);   // the width runs along X now
    CHECK(std::abs(pb.height() - 500.0) < 1e-9);
    // Hole (x=100 along the length, y=50 along the width) -> (pb.min.x + width - y, pb.min.y + x).
    const geometry::Box2 hb = boundsOf(doc, s.holes[0].object);
    CHECK(std::abs(hb.centre().x - (pb.min.x + 300.0 - 50.0)) < 1e-6);
    CHECK(std::abs(hb.centre().y - (pb.min.y + 100.0)) < 1e-6);
    // L dado (y 20..30 along the width, spans the whole length) -> x from 270 down to 280... runs along Y.
    const geometry::Box2 db = boundsOf(doc, s.dados[0].object);
    CHECK(std::abs(db.height() - 500.0) < 1e-9);
    CHECK(std::abs(db.width() - 10.0) < 1e-9);
    CHECK(std::abs(db.min.x - (pb.min.x + 300.0 - 30.0)) < 1e-9);
}

TEST_CASE("build: quantities and names, grouping by material and thickness", "[parts][build]") {
    document::Document doc;
    const PartsMapping m = mapping();
    const PartsList l = parts("a,100,100,18,Oak,3,,\nb,100,100,12,Oak,1,,\nc,100,100,18,Pine,1,,\nd,100,100,18.0004,Oak,1,,\n", m);
    const BuildResult r = buildPanels(doc, l, m);
    REQUIRE(r.ok);
    REQUIRE(r.sheets.size() == 3);  // Oak 18 (incl. 18.0004: same micrometre bucket? no: see below), Oak 12, Pine 18
    CHECK(r.placed == 6);
    // 18.0004 rounds to 18000 um like 18: one group.
    CHECK(r.sheets[0].panels.size() == 4);
    CHECK(r.sheets[0].material == "Oak");
    CHECK(r.sheets[1].thicknessMm == 12.0);
    CHECK(r.sheets[2].material == "Pine");
    CHECK(doc.findObject(r.sheets[0].panels[0])->name == "a (1/3)");
    CHECK(doc.findObject(r.sheets[0].panels[2])->name == "a (3/3)");
    CHECK(doc.findObject(r.sheets[1].panels[0])->name == "b");
    CHECK(doc.sheets().size() == 3);
}

TEST_CASE("build: thicknesses a micrometre apart are different groups", "[parts][build]") {
    document::Document doc;
    const PartsMapping m = mapping();
    const BuildResult r = buildPanels(doc, parts("a,100,100,18,,1,,\nb,100,100,18.002,,1,,\n", m), m);
    REQUIRE(r.ok);
    CHECK(r.sheets.size() == 2);
}

TEST_CASE("build: overflow adds sheets, one undo step for everything, redo restores", "[parts][build]") {
    document::Document doc;
    const PartsMapping m = mapping();
    const PartsList l = parts("big,900,500,18,,3,10:10:5,L:10:5:5\n", m);
    const std::size_t stepsBefore = doc.undoStack().count();
    const BuildResult r = buildPanels(doc, l, m);
    REQUIRE(r.ok);
    CHECK(r.sheets.size() == 3);
    CHECK(doc.sheets().size() == 3);
    CHECK(doc.layers().size() == 4);
    CHECK(doc.undoStack().count() == stepsBefore + 1);
    CHECK(doc.undoStack().undoText() == "Import parts");
    const std::size_t objects = doc.objectsOnSheet(r.sheets[0].sheet).size() + doc.objectsOnSheet(r.sheets[1].sheet).size() +
                                doc.objectsOnSheet(r.sheets[2].sheet).size();
    CHECK(objects == 9);  // 3 panels + 3 holes + 3 dados

    const document::SheetId firstSheet = doc.sheets().front().id;
    const document::SheetId activeBefore = r.sheets[0].sheet;
    CHECK(doc.activeSheet() == activeBefore);
    REQUIRE(doc.undoStack().undo(doc));
    CHECK(doc.sheets().size() == 1);
    CHECK(doc.layers().size() == 1);
    CHECK(doc.objectsOnSheet(firstSheet).empty());
    CHECK(doc.sheets().front().name == "Sheet 1");
    CHECK(doc.sheets().front().sizeMm.x == 300.0);
    REQUIRE(doc.undoStack().redo(doc));
    CHECK(doc.sheets().size() == 3);
    CHECK(doc.layers().size() == 4);
}

TEST_CASE("build: existing layers are reused, hidden or locked ones refuse", "[parts][build]") {
    document::Document doc;
    document::Layer layer;
    layer.name = "Panels";
    const document::LayerId existing = doc.insertLayer(layer);
    const PartsMapping m = mapping();
    BuildResult r = buildPanels(doc, parts("a,100,100,18,,1,,\n", m), m);
    REQUIRE(r.ok);
    CHECK(r.panelLayer == existing);
    CHECK(doc.layers().size() == 4);  // default + Panels + Holes + Dados

    document::Document doc2;
    layer.locked = true;
    doc2.insertLayer(layer);
    const std::size_t steps = doc2.undoStack().count();
    r = buildPanels(doc2, parts("a,100,100,18,,1,,\n", m), m);
    CHECK_FALSE(r.ok);
    CHECK(doc2.undoStack().count() == steps);
    REQUIRE_FALSE(r.diagnostics.empty());
    CHECK(r.diagnostics[0].text.find("hidden or locked") != std::string::npos);

    document::Document doc3;
    layer.locked = false;
    layer.visible = false;
    doc3.insertLayer(layer);
    CHECK_FALSE(buildPanels(doc3, parts("a,100,100,18,,1,,\n", m), m).ok);
}

TEST_CASE("build: a panel that fits no sheet refuses the whole import and changes nothing", "[parts][build]") {
    document::Document doc;
    const PartsMapping m = mapping();
    const std::size_t steps = doc.undoStack().count();
    const BuildResult r = buildPanels(doc, parts("ok,100,100,18,,1,,\nhuge,990,100,18,,1,,\n", m), m);
    CHECK_FALSE(r.ok);
    CHECK(r.unplaced == 1);
    CHECK(r.placed == 1);
    CHECK(doc.undoStack().count() == steps);
    CHECK(doc.sheets().size() == 1);
    REQUIRE_FALSE(r.diagnostics.empty());
    CHECK(r.diagnostics[0].text.find("\"huge\"") != std::string::npos);
    CHECK(r.diagnostics[0].line == 3);
}

TEST_CASE("build: nothing to build is an error", "[parts][build]") {
    document::Document doc;
    const BuildResult r = buildPanels(doc, PartsList{}, mapping());
    CHECK_FALSE(r.ok);
    CHECK(r.diagnostics[0].text.find("no panels") != std::string::npos);
}

TEST_CASE("build: a non-empty active sheet is kept; new sheets are added", "[parts][build]") {
    document::Document doc;
    document::Object o = document::Object::vector(
        geometry::Contour::rectangle(geometry::Box2{{0, 0}, {5, 5}}), doc.defaultLayer(), doc.activeSheet());
    REQUIRE(doc.execute(std::make_unique<document::AddObjectCommand>(o)));
    const document::SheetId old = doc.activeSheet();
    const PartsMapping m = mapping();
    const BuildResult r = buildPanels(doc, parts("a,100,100,18,,1,,\n", m), m);
    REQUIRE(r.ok);
    CHECK(doc.sheets().size() == 2);
    CHECK(r.sheets[0].sheet != old);
    CHECK(doc.activeSheet() == r.sheets[0].sheet);
    REQUIRE(doc.undoStack().undo(doc));
    CHECK(doc.activeSheet() == old);
}
