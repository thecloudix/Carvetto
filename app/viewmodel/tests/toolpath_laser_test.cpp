// T-0323: the Laser Cut / Fill form (fields shown by strategy, choices, bounds, tool restriction) and a laser
// toolpath through the session: calculation, power on the moves, undo, save/load, recalculation.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

#include "carvetto/document/carv_io.hpp"
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
}  // namespace

TEST_CASE("laser form: fields follow the strategy, bounds and choices are checked", "[toolpath][form][laser]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::LaserCutFill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.problems().empty());
    CHECK(find(form.fields(), "power") != nullptr);
    CHECK(find(form.fields(), "moveSpeed") != nullptr);
    CHECK(find(form.fields(), "repeats") != nullptr);
    CHECK(find(form.fields(), "stepover") == nullptr);  // the default strategy cuts the outside
    CHECK(find(form.fields(), "overscan") == nullptr);
    CHECK(find(form.fields(), "passDepth") == nullptr);  // no cut depth: one pass
    REQUIRE(form.setChoice("strategy", 3).ok);            // hatch fill
    CHECK(find(form.fields(), "stepover") != nullptr);
    CHECK(find(form.fields(), "hatchAngle") != nullptr);
    CHECK(find(form.fields(), "overscan") != nullptr);
    REQUIRE(form.setChoice("crossHatch", 1).ok);
    REQUIRE(form.setChoice("fillMode", 1).ok);
    REQUIRE(form.setChoice("outline", 1).ok);
    REQUIRE(form.setField("cutDepth", "3").ok);
    CHECK(find(form.fields(), "passDepth") != nullptr);
    REQUIRE(form.setField("power", "80").ok);
    REQUIRE(form.setField("repeats", "2").ok);
    CHECK_FALSE(form.setField("power", "0").ok);
    CHECK_FALSE(form.setField("power", "101").ok);
    CHECK_FALSE(form.setField("power", "abc").ok);
    CHECK_FALSE(form.setField("repeats", "0").ok);
    CHECK_FALSE(form.setField("repeats", "1001").ok);
    CHECK_FALSE(form.setField("stepover", "0").ok);
    CHECK_FALSE(form.setField("overscan", "-1").ok);
    CHECK_FALSE(form.setField("cutDepth", "-1").ok);
    CHECK_FALSE(form.setChoice("strategy", 4).ok);
    const auto& p = std::get<toolpath::LaserCutFillParams>(form.definition().params);
    CHECK(p.power == 80.0);
    CHECK(p.repeats == 2);
    CHECK(p.strategy == toolpath::LaserStrategy::HatchFill);
    CHECK(p.crossHatch);
}

TEST_CASE("laser form: only an end mill is accepted as the beam", "[toolpath][form][laser]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::LaserCutFill, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
}

TEST_CASE("laser session: add, undo, save, load and recalculate", "[toolpath][session][laser]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::circle({20, 20}, 10.0));
    doc.undoStack().clear();
    ToolpathForm form(ToolpathOperation::LaserCutFill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setChoice("strategy", 3).ok);
    REQUIRE(form.setField("stepover", "1").ok);
    REQUIRE(form.setField("power", "45").ok);
    REQUIRE(form.setField("moveSpeed", "700").ok);
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::LaserCutFill);
    CHECK(rec.tool.laser);
    CHECK(rec.tool.feedRate == 700.0);
    REQUIRE(rec.hasMoves());
    std::size_t lit = 0;
    for (const toolpath::Move& m : *rec.moves) {
        if (m.jetPower > 0.0) {
            CHECK(m.jetPower == 45.0);
            ++lit;
        }
    }
    CHECK(lit > 10);
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "laser";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 1);
    CHECK(list[0].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[0].id).ok);
    CHECK(doc2.toolpaths() == before);

    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
}
