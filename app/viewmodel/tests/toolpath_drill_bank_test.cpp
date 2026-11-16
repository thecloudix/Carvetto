// T-0328: the Drill Bank form (fields, choices, bounds, per-spindle lists, tool restriction) and a drill bank
// toolpath through the session: calculation, refusal of unmatched holes, undo, save/load, recalculation.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
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
const toolpath::DrillBankParams& params(const ToolpathForm& form) {
    return std::get<toolpath::DrillBankParams>(form.definition().params);
}
}  // namespace

TEST_CASE("drill bank form: fields, choices and bounds", "[toolpath][form][drill_bank]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::DrillBank, Units::Millimetres);
    CHECK(form.name() == "Drill bank");
    REQUIRE(form.setTool(db, source("drill-3mm")).ok);
    CHECK_FALSE(form.problems().empty());  // no cut depth yet
    REQUIRE(form.setField("cutDepth", "12").ok);
    CHECK(form.problems().empty());
    for (const char* id : {"startDepth", "cutDepth", "safeZ", "horizontalDrills", "verticalDrills", "pitch", "rowOffset",
                           "columnOffset", "referenceDrill", "pitchTolerance", "minHoles", "startX", "startY",
                           "dwellTime"}) {
        INFO(id);
        CHECK(find(form.fields(), id) != nullptr);
    }
    CHECK(find(form.fields(), "retractGap") == nullptr);  // no peck
    CHECK(find(form.fields(), "diameterTolerance") == nullptr);  // diameters are not matched
    REQUIRE(form.setChoice("peck", 1).ok);
    REQUIRE(form.setChoice("matchDiameters", 1).ok);
    REQUIRE(form.setChoice("unmatched", 1).ok);
    CHECK(find(form.fields(), "retractGap") != nullptr);
    CHECK(find(form.fields(), "diameterTolerance") != nullptr);
    CHECK_FALSE(form.setChoice("unmatched", 2).ok);

    REQUIRE(form.setField("horizontalDrills", "6").ok);
    REQUIRE(form.setField("verticalDrills", "2").ok);
    REQUIRE(form.setField("pitch", "25.5").ok);
    REQUIRE(form.setField("rowOffset", "-4").ok);
    REQUIRE(form.setField("columnOffset", "6").ok);
    REQUIRE(form.setField("referenceDrill", "3").ok);
    REQUIRE(form.setField("pitchTolerance", "0.2").ok);
    REQUIRE(form.setField("minHoles", "3").ok);
    const toolpath::DrillBankParams& p = params(form);
    CHECK(p.horizontalDrills == 6);
    CHECK(p.verticalDrills == 2);
    CHECK(p.pitch == Approx(25.5));
    CHECK(p.rowOffset == Approx(-4.0));
    CHECK(p.columnOffset == Approx(6.0));
    CHECK(p.referenceDrill == 3);
    CHECK(p.pitchTolerance == Approx(0.2));
    CHECK(p.minHoles == 3);
    CHECK(p.peck);
    CHECK(p.matchDiameters);
    CHECK(p.unmatched == toolpath::BankUnmatched::Skip);
    CHECK(form.problems().empty());

    CHECK_FALSE(form.setField("horizontalDrills", "33").ok);
    CHECK_FALSE(form.setField("horizontalDrills", "-1").ok);
    CHECK_FALSE(form.setField("horizontalDrills", "2.5").ok);
    CHECK_FALSE(form.setField("pitch", "0").ok);
    CHECK_FALSE(form.setField("pitch", "abc").ok);
    CHECK_FALSE(form.setField("pitchTolerance", "0").ok);
    CHECK_FALSE(form.setField("referenceDrill", "0").ok);
    CHECK_FALSE(form.setField("referenceDrill", "65").ok);
    CHECK_FALSE(form.setField("minHoles", "0").ok);
    CHECK_FALSE(form.setField("cutDepth", "0").ok);
    CHECK(params(form).horizontalDrills == 6);  // a bad entry changes nothing

    // Problems the core finds: a reference drill that does not exist, a tolerance beyond half the spacing.
    REQUIRE(form.setField("referenceDrill", "9").ok);
    CHECK_FALSE(form.problems().empty());
    REQUIRE(form.setField("referenceDrill", "1").ok);
    CHECK(form.problems().empty());
    REQUIRE(form.setField("pitchTolerance", "13").ok);
    bool found = false;
    for (const std::string& s : form.problems()) {
        found = found || s.find("half the distance") != std::string::npos;
    }
    CHECK(found);
    // Defaults on reset.
    form.resetValues();
    CHECK(params(form) == toolpath::DrillBankParams{});
}

TEST_CASE("drill bank form: per-spindle lists", "[toolpath][form][drill_bank]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::DrillBank, Units::Millimetres);
    REQUIRE(form.setTool(db, source("drill-3mm")).ok);
    REQUIRE(form.setField("cutDepth", "12").ok);
    REQUIRE(form.setField("horizontalDrills", "3").ok);
    REQUIRE(form.setBankSpindles({2}, {5, 5, 8}, {0, 10, 15}).ok);
    CHECK(params(form).inactiveSpindles == std::vector<int>{2});
    CHECK(params(form).spindleDiameters == std::vector<double>{5, 5, 8});
    CHECK(params(form).spindleDepths == std::vector<double>{0, 10, 15});
    CHECK(form.problems().empty());
    // Lists that do not fit the head are a problem, not a refused entry.
    REQUIRE(form.setBankSpindles({}, {5, 5}, {}).ok);
    CHECK_FALSE(form.problems().empty());
    // Bad lists change nothing.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(form.setBankSpindles({1, 1}, {}, {}).ok);
    CHECK_FALSE(form.setBankSpindles({0}, {}, {}).ok);
    CHECK_FALSE(form.setBankSpindles({65}, {}, {}).ok);
    CHECK_FALSE(form.setBankSpindles({}, {0}, {}).ok);
    CHECK_FALSE(form.setBankSpindles({}, {nan}, {}).ok);
    CHECK_FALSE(form.setBankSpindles({}, {}, {-1}).ok);
    CHECK_FALSE(form.setBankSpindles({}, {}, {nan}).ok);
    CHECK_FALSE(form.setBankSpindles(std::vector<int>(65, 1), {}, {}).ok);
    CHECK_FALSE(form.setBankSpindles({}, std::vector<double>(65, 3.0), {}).ok);
    CHECK_FALSE(form.setBankSpindles({}, {}, std::vector<double>(65, 3.0)).ok);
    CHECK(params(form).spindleDiameters == std::vector<double>{5, 5});
    // Only a drill bank has spindles.
    ToolpathForm drill(ToolpathOperation::Drill, Units::Millimetres);
    CHECK_FALSE(drill.setBankSpindles({}, {}, {}).ok);
}

TEST_CASE("drill bank form: drills and end mills are accepted as the tool", "[toolpath][form][drill_bank]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::DrillBank, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(operationName(ToolpathOperation::DrillBank) == std::string("drill_bank"));
}

TEST_CASE("drill bank session: add, refuse, undo, save, load and recalculate", "[toolpath][session][drill_bank]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    for (int i = 0; i < 4; ++i) {
        addVector(doc, geometry::Contour::circle({10.0 + 32.0 * i, 20.0}, 3.0));
    }
    doc.undoStack().clear();
    ToolpathForm form(ToolpathOperation::DrillBank, Units::Millimetres);
    REQUIRE(form.setTool(db, source("drill-3mm")).ok);
    REQUIRE(form.setField("cutDepth", "6").ok);
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::DrillBank);
    REQUIRE(rec.hasMoves());
    CHECK(rec.moveCount() == 3);  // one stroke
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "drill_bank";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    CHECK(loaded.value().formatMinor == document::kCarvFormatMinorBankSpindles);  // T-0335: the strokes name spindles
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

    // A fifth hole that fits no stroke: refused with the hole named, nothing added.
    addVector(doc, geometry::Contour::circle({500.0, 500.0}, 3.0));
    const ToolpathOutcome refused = session.add(form.definition());
    CHECK_FALSE(refused.ok);
    CHECK(doc.toolpaths().empty());
    CHECK(refused.message.find("drill bank") != std::string::npos);
    // Skipping unmatched holes adds the toolpath, with the hole reported.
    REQUIRE(form.setChoice("unmatched", 1).ok);
    const ToolpathOutcome skipped = session.add(form.definition());
    INFO(skipped.message);
    CHECK(skipped.ok);
    CHECK(doc.toolpaths().size() == 1);
    bool warned = false;
    for (const ToolpathMessage& m : skipped.messages) {
        warned = warned || m.issue == toolpath::ToolpathIssue::BankHoleUnmatched;
    }
    CHECK(warned);
}
