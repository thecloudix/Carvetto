// T-0265: the V-carve inlay form (piece-dependent fields, V-bit rule, clearance tools) and a pocket
// and a plug through the session: calculation, plug sheet, clearance groups, undo, save/load,
// recalculation.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <string>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using Catch::Approx;
using document::Document;
using document::ToolpathOperation;
using document::Units;

namespace {
const ToolpathField* find(const std::vector<ToolpathField>& fields, const std::string& id) {
    for (const ToolpathField& f : fields) {
        if (f.id == id) {
            return &f;
        }
    }
    return nullptr;
}
bool hasChoice(const ToolpathForm& form, const std::string& id) {
    const auto cs = form.choices();
    return std::any_of(cs.begin(), cs.end(), [&](const ToolpathChoice& c) { return c.id == id; });
}
geometry::Box2 cutBounds(const document::ToolpathRecord& rec) {
    geometry::Box2 b;
    for (const toolpath::Move& m : *rec.moves) {
        if (toolpath::isCutting(m.kind)) {
            b.expand(geometry::Vec2{m.to.x, m.to.y});
        }
    }
    return b;
}
}  // namespace

TEST_CASE("V-carve inlay form: piece-dependent fields, bounds, tools", "[toolpath][form][v_carve_inlay]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::VCarveInlay, Units::Millimetres);
    CHECK(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("end-mill-3mm")).ok);  // a V-bit carves the inlay
    CHECK(form.setTool(db, source("v-bit-90")).ok);
    CHECK(form.setClearanceTools(db, {source("end-mill-3mm")}).ok);
    CHECK_FALSE(form.setClearanceTools(db, {source("v-bit-90")}).ok);
    CHECK(form.problems().empty());

    // The pocket has no plug fields.
    CHECK(find(form.fields(), "pocketDepth") != nullptr);
    CHECK(find(form.fields(), "glueGap") != nullptr);
    CHECK(find(form.fields(), "surfaceClearance") == nullptr);
    CHECK(find(form.fields(), "plugSheet") == nullptr);
    CHECK(find(form.fields(), "boundaryOffset") == nullptr);
    CHECK_FALSE(hasChoice(form, "plugBoundary"));
    REQUIRE(form.setChoice("piece", 1).ok);
    CHECK(find(form.fields(), "surfaceClearance") != nullptr);
    CHECK(find(form.fields(), "plugSheet") != nullptr);
    CHECK(find(form.fields(), "boundaryOffset") != nullptr);
    CHECK(hasChoice(form, "plugBoundary"));
    REQUIRE(form.setChoice("plugBoundary", 0).ok);  // Sheet Limits needs no offset
    CHECK(find(form.fields(), "boundaryOffset") == nullptr);
    REQUIRE(form.setChoice("plugBoundary", 2).ok);

    REQUIRE(form.setField("glueGap", "0.15").ok);
    CHECK(find(form.fields(), "glueGap")->text == "0.15");
    REQUIRE(form.setField("plugSheet", "3").ok);
    CHECK(find(form.fields(), "plugSheet")->text == "3");
    CHECK_FALSE(form.setField("plugSheet", "1.5").ok);
    CHECK_FALSE(form.setField("plugSheet", "-1").ok);
    CHECK_FALSE(form.setField("pocketDepth", "0").ok);
    CHECK_FALSE(form.setField("glueGap", "-1").ok);
    CHECK_FALSE(form.setField("boundaryOffset", "1e9").ok);
    CHECK_FALSE(form.setChoice("plugBoundary", 3).ok);
    CHECK_FALSE(form.setChoice("piece", 2).ok);
    CHECK(std::get<toolpath::VCarveInlayParams>(form.definition().params).plugSheet == 3);
    // An empty text resets the field.
    REQUIRE(form.setField("plugSheet", "").ok);
    CHECK(std::get<toolpath::VCarveInlayParams>(form.definition().params).plugSheet == 0);
}

TEST_CASE("V-carve inlay session: pocket and plug, clearance groups, undo, save, load, recalculate",
          "[toolpath][session][v_carve_inlay]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::rectangle(geometry::Box2{{10, 10}, {50, 40}}));
    doc.undoStack().clear();

    for (const int piece : {0, 1}) {
        ToolpathForm form(ToolpathOperation::VCarveInlay, Units::Millimetres);
        REQUIRE(form.setTool(db, source("v-bit-90")).ok);
        REQUIRE(form.setClearanceTools(db, {source("end-mill-3mm")}).ok);
        REQUIRE(form.setChoice("piece", piece).ok);
        REQUIRE(form.setField("pocketDepth", "2").ok);
        REQUIRE(form.setField("glueGap", "0.1").ok);
        const ToolpathOutcome out = session.add(form.definition());
        INFO(out.message);
        REQUIRE(out.ok);
    }
    // Each piece: the end mill's clearance toolpath, then the V-bit's.
    REQUIRE(doc.toolpaths().size() == 4);
    for (const document::ToolpathRecord& rec : doc.toolpaths()) {
        CHECK(rec.operation() == ToolpathOperation::VCarveInlay);
        CHECK(rec.hasMoves());
    }
    CHECK(doc.toolpaths()[0].tool.name != doc.toolpaths()[1].tool.name);
    CHECK(doc.toolpaths()[2].tool.name == doc.toolpaths()[0].tool.name);
    // The pocket is inside the vector; the plug's carving covers a boundary beyond it.
    const geometry::Box2 pocket = cutBounds(doc.toolpaths()[1]);
    const geometry::Box2 plug = cutBounds(doc.toolpaths()[3]);
    CHECK(pocket.min.x >= 10.0 - 1e-6);
    CHECK(pocket.max.x <= 50.0 + 1e-6);
    CHECK(plug.min.x < 10.0);
    CHECK(plug.max.x > 50.0);
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "v_carve_inlay";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 4);
    CHECK(list[0].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[1].id).ok);
    CHECK(doc2.toolpaths() == before);

    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
}

TEST_CASE("V-carve inlay session: the plug on another sheet is mirrored and centred there",
          "[toolpath][session][v_carve_inlay]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    document::Sheet other;
    other.name = "Plugs";
    other.sizeMm = {200.0, 100.0, 18.0};
    other.datumOffsetMm = {500.0, 300.0};
    const document::SheetId second = doc.insertSheet(other);
    const geometry::Box2 sheetBox = doc.findSheet(second)->materialBoundsMm();
    // An asymmetric vector on the first sheet.
    addVector(doc, geometry::Contour::fromPolyline(geometry::Polyline{{{10, 10}, {70, 10}, {10, 40}}, true}));
    doc.undoStack().clear();

    ToolpathForm form(ToolpathOperation::VCarveInlay, Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    REQUIRE(form.setChoice("piece", 1).ok);
    REQUIRE(form.setField("pocketDepth", "3").ok);
    REQUIRE(form.setField("boundaryOffset", "5").ok);
    REQUIRE(form.setField("plugSheet", std::to_string(second.value)).ok);
    ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const geometry::Box2 cut = cutBounds(doc.toolpaths()[0]);
    CHECK(cut.centre().x == Approx(sheetBox.centre().x).margin(0.5));
    CHECK(cut.centre().y == Approx(sheetBox.centre().y).margin(0.5));

    // Sheet Limits: the whole sheet is carved.
    REQUIRE(form.setChoice("plugBoundary", 0).ok);
    REQUIRE(form.setClearanceTools(db, {source("end-mill-3mm")}).ok);  // the big flat area
    out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 3);  // clearance + carving
    const geometry::Box2 whole = cutBounds(doc.toolpaths()[1]);
    CHECK(whole.min.x < sheetBox.min.x + 6.0);
    CHECK(whole.max.x > sheetBox.max.x - 6.0);
    CHECK(whole.min.x >= sheetBox.min.x - 1e-6);

    // A sheet that does not exist is refused.
    REQUIRE(form.setField("plugSheet", "999999").ok);
    out = session.add(form.definition());
    CHECK_FALSE(out.ok);
    CHECK(out.message.find("does not exist") != std::string::npos);
    CHECK(doc.toolpaths().size() == 3);
}
