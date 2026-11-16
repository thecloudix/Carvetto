// T-0322: the Rounding form (fields, bounds, choices, no vectors) and a rounding toolpath through the session on a
// rotary job: defaults from the job, calculation, undo, save/load, recalculation, refusal on a flat job.

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
using document::Job;
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
const toolpath::RoundingParams& params(const ToolpathForm& form) {
    return std::get<toolpath::RoundingParams>(form.definition().params);
}
}  // namespace

TEST_CASE("rounding form: fields, bounds and choices", "[toolpath][form][rounding]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Rounding, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-6mm")).ok);
    CHECK(form.problems().empty());
    CHECK_FALSE(needsVectors(form.definition().params));
    CHECK(needsVectors(toolpath::FlutingParams{}));
    for (const char* id : {"blankRadius", "targetRadius", "startRadius", "allowance", "passDepth", "stepover",
                           "alongStart", "alongEnd", "circStart", "surfaceZ", "safeZ"}) {
        CHECK(find(form.fields(), id) != nullptr);
    }
    REQUIRE(form.setField("blankRadius", "30").ok);
    REQUIRE(form.setField("targetRadius", "12.5").ok);
    REQUIRE(form.setField("startRadius", "20").ok);
    REQUIRE(form.setField("allowance", "0.25").ok);
    REQUIRE(form.setField("alongStart", "-5").ok);  // may be negative
    REQUIRE(form.setField("alongEnd", "120").ok);
    REQUIRE(form.setField("surfaceZ", "-4").ok);
    REQUIRE(form.setChoice("method", 1).ok);
    REQUIRE(form.setChoice("blank", 1).ok);
    REQUIRE(form.setChoice("alongX", 0).ok);
    CHECK_FALSE(form.setField("blankRadius", "0").ok);
    CHECK_FALSE(form.setField("targetRadius", "-1").ok);
    CHECK_FALSE(form.setField("startRadius", "-1").ok);
    CHECK_FALSE(form.setField("allowance", "-0.1").ok);
    CHECK_FALSE(form.setField("passDepth", "-1").ok);
    CHECK_FALSE(form.setField("stepover", "abc").ok);
    CHECK_FALSE(form.setChoice("method", 3).ok);
    CHECK_FALSE(form.setChoice("blank", 2).ok);
    const toolpath::RoundingParams& p = params(form);
    CHECK(p.blankRadius == 30.0);
    CHECK(p.targetRadius == 12.5);
    CHECK(p.startRadius == 20.0);
    CHECK(p.allowance == 0.25);
    CHECK(p.alongStart == -5.0);
    CHECK(p.surfaceZ == -4.0);
    CHECK(p.method == toolpath::RoundingMethod::Raster);
    CHECK(p.blank == toolpath::RoundingBlank::Square);
    CHECK_FALSE(p.alongX);
    // Pass depth and stepover follow the tool when left at 0.
    const ToolpathField* pass = find(form.fields(), "passDepth");
    REQUIRE(pass != nullptr);
    CHECK(pass->fromTool);
    CHECK(find(form.fields(), "stepover")->fromTool);
}

TEST_CASE("rounding form: only an end mill or ball nose is accepted", "[toolpath][form][rounding]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Rounding, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
}

TEST_CASE("rounding session: from a rotary job to a saved toolpath", "[toolpath][session][rounding]") {
    namespace fs = std::filesystem;
    Document doc(Job::rotary(120.0, 50.0, Units::Millimetres));  // axis zero: the surface is at Z 25
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    doc.undoStack().clear();
    ToolpathForm form = session.newForm(ToolpathOperation::Rounding);
    // Defaults come from the job.
    CHECK(params(form).blankRadius == Approx(25.0));
    CHECK(params(form).targetRadius == Approx(20.0));
    CHECK(params(form).alongStart == 0.0);
    CHECK(params(form).alongEnd == Approx(120.0));
    CHECK(params(form).surfaceZ == Approx(25.0));
    CHECK(params(form).safeZ == Approx(30.0));
    CHECK(params(form).alongX);
    REQUIRE(form.setTool(db, source("end-mill-6mm")).ok);
    REQUIRE(form.setChoice("method", 1).ok);
    REQUIRE(form.setField("targetRadius", "15").ok);
    REQUIRE(form.setField("passDepth", "5").ok);
    REQUIRE(form.setField("stepover", "6").ok);
    CHECK(form.problems().empty());
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::Rounding);
    CHECK(rec.selection.objects.empty());
    REQUIRE(rec.hasMoves());
    // Two passes (10 mm in steps of 5), 21 turns each (120 mm in steps of 6).
    CHECK(rec.moveCount() == 2 * 21 * 4 + 0);  // rapid, plunge, feed, retract per stroke; the raster lifts each turn
    double lowest = 1e9;
    for (const toolpath::Move& m : *rec.moves) {
        lowest = std::min(lowest, m.to.z);
    }
    CHECK(lowest == Approx(15.0));  // axis at Z 0, the finished radius is 15
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "rounding";
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

TEST_CASE("rounding session: a flat job is refused; the vectors selected are not needed", "[toolpath][session][rounding]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    Document flat;
    ToolpathSession session(&flat);
    session.setToolDatabase(db);
    addVector(flat, geometry::Contour::circle({20, 20}, 10.0));
    ToolpathDefinition def = roundingDefinition(db);
    const ToolpathOutcome out = session.add(def);
    CHECK_FALSE(out.ok);
    CHECK(out.message.find("rotary") != std::string::npos);
    CHECK(flat.toolpaths().empty());
    // On a rotary job a selected vector makes no difference.
    Document rotary(Job::rotary(100.0, 40.0, Units::Millimetres));
    ToolpathSession s2(&rotary);
    s2.setToolDatabase(db);
    addVector(rotary, geometry::Contour::circle({20, 20}, 5.0));
    ToolpathForm form = s2.newForm(ToolpathOperation::Rounding);
    REQUIRE(form.setTool(db, source("end-mill-6mm")).ok);
    REQUIRE(s2.add(form.definition()).ok);
    REQUIRE(rotary.toolpaths().size() == 1);
    CHECK(rotary.toolpaths().front().selection.objects.empty());
    // A nothing-to-remove result is a refusal with the reason, not an empty toolpath.
    ToolpathForm none = s2.newForm(ToolpathOperation::Rounding);
    REQUIRE(none.setTool(db, source("end-mill-6mm")).ok);
    REQUIRE(none.setField("targetRadius", "20").ok);
    const ToolpathOutcome nothing = s2.add(none.definition());
    CHECK_FALSE(nothing.ok);
    CHECK(rotary.toolpaths().size() == 1);
}
