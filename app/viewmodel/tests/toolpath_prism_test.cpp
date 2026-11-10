// T-0240: the prism form and a prism through the session: calculation, undo, save/load, recalculation.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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

TEST_CASE("prism form: fields, full-prism hides the depth, bounds", "[toolpath][form][prism]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Prism, Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    CHECK(form.problems().empty());
    CHECK(find(form.fields(), "depth") != nullptr);
    REQUIRE(form.setField("depth", "3").ok);
    CHECK_FALSE(form.setField("depth", "0").ok);
    CHECK_FALSE(form.setField("depth", "-1").ok);
    CHECK_FALSE(form.setField("depth", "abc").ok);
    CHECK_FALSE(form.setField("stepover", "0").ok);
    CHECK_FALSE(form.setField("stepover", "1000").ok);
    REQUIRE(form.setField("stepover", "0.5").ok);
    REQUIRE(form.setChoice("direction", 1).ok);
    CHECK_FALSE(form.setChoice("direction", 2).ok);
    REQUIRE(form.setChoice("fullPrism", 1).ok);
    CHECK(find(form.fields(), "depth") == nullptr);
}

TEST_CASE("prism form: only V-bits and ball noses", "[toolpath][form][prism]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Prism, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("v-bit-90")).ok);
}

TEST_CASE("prism session: add, undo, save, load and recalculate", "[toolpath][session][prism]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::rectangle(geometry::Box2{{0, 0}, {20, 20}}));
    doc.undoStack().clear();
    ToolpathForm form(ToolpathOperation::Prism, Units::Millimetres);
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    REQUIRE(form.setField("depth", "3").ok);
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::Prism);
    REQUIRE(rec.hasMoves());
    double minZ = 0.0;
    for (const toolpath::Move& m : *rec.moves) {
        minZ = std::min(minZ, m.to.z);
    }
    CHECK(minZ == Approx(-3.0).margin(1e-9));
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "prism";
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
