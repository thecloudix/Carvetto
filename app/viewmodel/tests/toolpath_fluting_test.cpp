// T-0239: the fluting form (ramp fields shown by flute type and basis, choices, bounds, tool
// restriction) and a flute through the session: calculation, undo, save/load, recalculation.

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

TEST_CASE("fluting form: fields, visibility and bounds", "[toolpath][form][fluting]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Fluting, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.problems().empty());
    CHECK(find(form.fields(), "fluteDepth") != nullptr);
    CHECK(find(form.fields(), "rampLength") != nullptr);
    CHECK(find(form.fields(), "rampPercent") == nullptr);
    REQUIRE(form.setChoice("rampBasis", 1).ok);
    CHECK(find(form.fields(), "rampLength") == nullptr);
    CHECK(find(form.fields(), "rampPercent") != nullptr);
    REQUIRE(form.setChoice("fluteType", 0).ok);  // ramp over the complete length: no ramp fields
    CHECK(find(form.fields(), "rampPercent") == nullptr);
    CHECK(find(form.fields(), "rampLength") == nullptr);
    REQUIRE(form.setChoice("fluteType", 2).ok);
    REQUIRE(form.setChoice("rampType", 1).ok);
    REQUIRE(form.setField("rampPercent", "25").ok);
    CHECK_FALSE(form.setField("rampPercent", "0").ok);
    CHECK_FALSE(form.setField("rampPercent", "101").ok);
    CHECK_FALSE(form.setField("fluteDepth", "0").ok);
    CHECK_FALSE(form.setField("fluteDepth", "-1").ok);
    CHECK_FALSE(form.setField("fluteDepth", "abc").ok);
    CHECK_FALSE(form.setField("fluteDepth", "1e9").ok);
    CHECK_FALSE(form.setChoice("fluteType", 3).ok);
    CHECK_FALSE(form.setChoice("nothing", 0).ok);
}

TEST_CASE("fluting form: end mill and ball nose are accepted, a V-bit is not", "[toolpath][form][fluting]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Fluting, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
}

TEST_CASE("fluting session: add, undo, save, load and recalculate", "[toolpath][session][fluting]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::circle({20, 20}, 10.0));
    doc.undoStack().clear();
    ToolpathForm form(ToolpathOperation::Fluting, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setField("fluteDepth", "1").ok);
    REQUIRE(form.setField("rampLength", "15").ok);
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::Fluting);
    REQUIRE(rec.hasMoves());
    double minZ = 0.0;
    for (const toolpath::Move& m : *rec.moves) {
        minZ = std::min(minZ, m.to.z);
    }
    CHECK(minZ == Approx(-1.0).margin(1e-9));
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "fluting";
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
