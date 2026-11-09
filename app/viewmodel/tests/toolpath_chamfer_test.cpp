// T-0236: the chamfer form (linked angle / width / cut depth, V-bit angle from the tool), and a
// chamfer through the session: calculation, undo, save/load, recalculation.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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

TEST_CASE("chamfer form: width and cut depth are linked through the V-bit's angle", "[toolpath][form][chamfer]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Chamfer, Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    CHECK(form.toolSummary().maxCutDepthMm > 0.0);
    // The V-bit's angle is the tool's: the field is hidden and cannot be set.
    CHECK(find(form.fields(), "angle") == nullptr);
    CHECK_FALSE(form.setField("angle", "30").ok);
    REQUIRE(form.setField("width", "2").ok);
    REQUIRE(find(form.fields(), "cutDepth") != nullptr);
    CHECK(find(form.fields(), "cutDepth")->text == "1");
    REQUIRE(form.setField("cutDepth", "1.5").ok);
    CHECK(find(form.fields(), "width")->text == "3");
    CHECK(find(form.fields(), "cutDepth")->text == "1.5");
    CHECK_FALSE(form.setField("cutDepth", "0").ok);
    CHECK_FALSE(form.setField("cutDepth", "-1").ok);
    CHECK_FALSE(form.setField("cutDepth", "abc").ok);
    CHECK(find(form.fields(), "width")->text == "3");
    REQUIRE(form.setField("cutDepth", "").ok);  // reset
    CHECK(find(form.fields(), "width")->text == "2");
    CHECK_FALSE(form.setField("width", "0").ok);
    CHECK_FALSE(form.setField("overcut", "-1").ok);
    CHECK(form.problems().empty());
    REQUIRE(form.setChoice("type", 1).ok);
    REQUIRE(form.setChoice("slope", 1).ok);
    CHECK_FALSE(form.setChoice("slope", 2).ok);
}

TEST_CASE("chamfer form: a round-nosed tool has an editable angle", "[toolpath][form][chamfer]") {
    ToolpathForm form(ToolpathOperation::Chamfer, Units::Millimetres);
    CHECK_FALSE(form.setField("cutDepth", "1").ok);  // no angle yet
    REQUIRE(form.setField("angle", "45").ok);
    REQUIRE(form.setField("width", "2").ok);
    CHECK(find(form.fields(), "cutDepth")->text == "1");
    REQUIRE(form.setField("cutDepth", "2").ok);
    CHECK(find(form.fields(), "width")->text == "4");
    CHECK_FALSE(form.setField("angle", "0").ok);
    CHECK_FALSE(form.setField("angle", "90").ok);
    CHECK_FALSE(form.setField("angle", "1e9").ok);
}

TEST_CASE("chamfer form: only chamfer tools are accepted", "[toolpath][form][chamfer]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Chamfer, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("v-bit-90")).ok);
}

TEST_CASE("chamfer session: add, undo, save, load and recalculate", "[toolpath][session][chamfer]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, square(0, 0, 20));
    doc.undoStack().clear();
    ToolpathDefinition def = chamferDefinition(db);
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::Chamfer);
    REQUIRE(rec.hasMoves());
    double minZ = 0.0;
    for (const toolpath::Move& m : *rec.moves) {
        minZ = std::min(minZ, m.to.z);
    }
    CHECK(minZ == Approx(-1.0).margin(1e-9));
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "chamfer";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "c.carv", {}));
    auto loaded = document::loadCarv(dir / "c.carv");
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
