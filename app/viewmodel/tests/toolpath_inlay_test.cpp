// T-0238: the inlay form (piece-dependent fields, choices, bounds) and an inlay (male and female)
// through the session: calculation, undo, save/load, recalculation, tool rule.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
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
bool hasChoice(const ToolpathForm& form, const std::string& id) {
    const auto cs = form.choices();
    return std::any_of(cs.begin(), cs.end(), [&](const ToolpathChoice& c) { return c.id == id; });
}
}  // namespace

TEST_CASE("inlay form: piece-dependent fields and bounds", "[toolpath][form][inlay]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Inlay, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.problems().empty());
    CHECK(find(form.fields(), "stepDepth") == nullptr);  // straight male
    CHECK(hasChoice(form, "maleStyle"));
    CHECK(!hasChoice(form, "femaleStyle"));
    REQUIRE(form.setChoice("maleStyle", 1).ok);
    CHECK(find(form.fields(), "stepDepth") != nullptr);
    CHECK(find(form.fields(), "stepWidth") != nullptr);
    REQUIRE(form.setChoice("piece", 1).ok);
    CHECK(find(form.fields(), "stepDepth") == nullptr);
    CHECK(!hasChoice(form, "maleStyle"));
    CHECK(hasChoice(form, "femaleStyle"));
    REQUIRE(form.setChoice("femaleStyle", 1).ok);
    REQUIRE(form.setField("allowance", "0.15").ok);
    CHECK(find(form.fields(), "allowance")->text == "0.15");
    CHECK_FALSE(form.setField("allowance", "-1").ok);
    CHECK_FALSE(form.setField("allowance", "1e9").ok);
    CHECK_FALSE(form.setField("cutDepth", "0").ok);
    CHECK_FALSE(form.setField("startDepth", "-1").ok);
    CHECK_FALSE(form.setField("allowance", "abc").ok);
    CHECK_FALSE(form.setChoice("piece", 2).ok);
    CHECK_FALSE(form.setChoice("nothing", 0).ok);
}

TEST_CASE("inlay form: only an end mill is accepted", "[toolpath][form][inlay]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Inlay, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
}

TEST_CASE("inlay session: male and female for the same vector, undo, save, load, recalculate",
          "[toolpath][session][inlay]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::rectangle(geometry::Box2{{10, 10}, {40, 30}}));
    doc.undoStack().clear();

    for (const int piece : {0, 1}) {
        ToolpathForm form(ToolpathOperation::Inlay, Units::Millimetres);
        REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
        REQUIRE(form.setChoice("piece", piece).ok);
        REQUIRE(form.setField("allowance", "0.1").ok);
        REQUIRE(form.setField("cutDepth", "2").ok);
        const ToolpathOutcome out = session.add(form.definition());
        INFO(out.message);
        REQUIRE(out.ok);
    }
    REQUIRE(doc.toolpaths().size() == 2);
    for (const document::ToolpathRecord& rec : doc.toolpaths()) {
        CHECK(rec.operation() == ToolpathOperation::Inlay);
        REQUIRE(rec.hasMoves());
        double minZ = 0.0;
        for (const toolpath::Move& m : *rec.moves) {
            minZ = std::min(minZ, m.to.z);
        }
        CHECK(minZ == Approx(-2.0).margin(1e-9));
    }
    // The male runs outside the vector, the female inside it.
    const auto extent = [](const document::ToolpathRecord& rec) {
        double minX = 1e300;
        for (const toolpath::Move& m : *rec.moves) {
            if (m.kind == toolpath::MoveKind::Feed) {
                minX = std::min(minX, m.to.x);
            }
        }
        return minX;
    };
    CHECK(extent(doc.toolpaths()[0]) < 10.0);
    CHECK(extent(doc.toolpaths()[1]) > 10.0);
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "inlay";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 2);
    CHECK(list[0].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[0].id).ok);
    CHECK(doc2.toolpaths() == before);

    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
}
