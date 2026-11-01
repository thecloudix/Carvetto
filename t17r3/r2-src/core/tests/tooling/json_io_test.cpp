#include "carvetto/tooling/json_io.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <string>

using namespace carvetto::tooling;
using carvetto::geometry::kPi;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

/// A database exercising every tool type, nested groups, materials and machines, with
/// "awkward" numbers (binary-inexact decimals, negative zero, long fractions).
ToolDatabase sampleDb() {
    ToolDatabase db;
    db.addGroup({"g-root", "Cutters", ""});
    db.addGroup({"g-fine", "Fine detail", "g-root"});
    db.addMaterial({"mat-oak", "Oak"});
    db.addMaterial({"mat-mdf", "MDF"});

    Machine m;
    m.id = "mc-router";
    m.name = "Router \"big\" / ünïcode";
    m.accelerationX = 1234.5678901;
    m.accelerationY = 0.1 + 0.2;
    m.accelerationZ = -0.0;
    m.accelerationAngular = kPi / 7.0;
    m.associatedPosts = {"grbl_mm", "grbl_inch"};
    m.defaultPost = "grbl_mm";
    db.addMachine(m);
    Machine bare;
    bare.id = "mc-bare";
    db.addMachine(bare);

    CuttingData cd;
    cd.passDepth = 1.5;
    cd.stepover = 0.1 + 0.2;
    cd.finalPassStepover = 0.05;
    cd.clearancePassStepover = 2.0;
    cd.spindleSpeed = 18000.0;
    cd.feedRate = 2540.0;
    cd.plungeRate = 635.0;
    cd.maxBurnRate = 0.0;
    cd.toolNumber = 7;

    int n = 0;
    for (ToolType type : allToolTypes()) {
        Tool t;
        t.id = "tool-" + std::to_string(n++);
        t.name = std::string("Tool ") + std::string(toString(type));
        t.type = type;
        t.notes = "line1\nline2 é";
        t.groupId = (n % 2 == 0) ? "g-fine" : "g-root";
        t.diameter = 12.7;
        t.includedAngle = kPi / 2.0;
        t.tipDiameter = (type == ToolType::Engraving) ? 0.2 : 3.175;
        t.cornerRadius = 1.5875;
        t.flutes = 2;
        if (type == ToolType::FormTool) t.profile = {{1.0, 0.0}, {3.3333333, 2.5}, {6.35, 7.0}};
        t.defaultCuttingData = cd;
        t.setCuttingData("mc-router", "mat-oak", cd);
        CuttingData mdf = cd;
        mdf.feedRate = 3000.0;
        mdf.toolNumber = 3;
        t.setCuttingData("mc-router", "mat-mdf", mdf);
        db.addTool(t);
    }
    return db;
}

}  // namespace

TEST_CASE("Export -> import -> export is byte-identical (AC2)", "[tooling][json]") {
    const ToolDatabase db = sampleDb();
    const std::string first = exportToolDatabaseJson(db);
    const ImportResult imported = importToolDatabaseJson(first);
    REQUIRE(imported.ok);
    CHECK(imported.error.empty());
    const std::string second = exportToolDatabaseJson(imported.database);
    CHECK(first == second);

    // And again, to make sure the fixed point is stable.
    const ImportResult again = importToolDatabaseJson(second);
    REQUIRE(again.ok);
    CHECK(exportToolDatabaseJson(again.database) == second);
}

TEST_CASE("Export is deterministic and independent of insertion order", "[tooling][json]") {
    const ToolDatabase a = sampleDb();
    // Build the same content inserting in reverse.
    ToolDatabase b;
    for (auto it = a.groups().rbegin(); it != a.groups().rend(); ++it) {
        // Parents must exist first; add root-level groups in a second pass.
        if (!it->second.parentId.empty()) continue;
        b.addGroup(it->second);
    }
    for (auto it = a.groups().rbegin(); it != a.groups().rend(); ++it) {
        if (it->second.parentId.empty()) continue;
        b.addGroup(it->second);
    }
    for (auto it = a.materials().rbegin(); it != a.materials().rend(); ++it) b.addMaterial(it->second);
    for (auto it = a.machines().rbegin(); it != a.machines().rend(); ++it) b.addMachine(it->second);
    for (auto it = a.tools().rbegin(); it != a.tools().rend(); ++it) b.addTool(it->second);
    CHECK(exportToolDatabaseJson(a) == exportToolDatabaseJson(b));

    // Keys are sorted: "format" precedes "groups" precedes "machines" ...
    const std::string text = exportToolDatabaseJson(a);
    const auto pFormat = text.find("\"format\"");
    const auto pGroups = text.find("\"groups\"");
    const auto pMachines = text.find("\"machines\"");
    const auto pMaterials = text.find("\"materials\"");
    const auto pTools = text.find("\"tools\"");
    const auto pVersion = text.find("\"version\"");
    REQUIRE(pFormat != std::string::npos);
    CHECK(pFormat < pGroups);
    CHECK(pGroups < pMachines);
    CHECK(pMachines < pMaterials);
    CHECK(pMaterials < pTools);
    CHECK(pTools < pVersion);
    CHECK(text.back() == '\n');
}

TEST_CASE("Export writes fixed precision: 1e-6 rounding, degrees, no negative zero", "[tooling][json]") {
    ToolDatabase db;
    Tool t;
    t.id = "v";
    t.name = "V";
    t.type = ToolType::VBit;
    t.diameter = 0.1 + 0.2;  // 0.30000000000000004
    t.includedAngle = kPi / 2.0;
    t.tipDiameter = -0.0;
    db.addTool(t);
    const std::string text = exportToolDatabaseJson(db);
    CHECK_THAT(text, ContainsSubstring("\"diameter_mm\": 0.3,"));
    CHECK_THAT(text, ContainsSubstring("\"included_angle_deg\": 90.0,"));
    CHECK_THAT(text, ContainsSubstring("\"tip_diameter_mm\": 0.0,"));
    CHECK_THAT(text, !ContainsSubstring("-0.0"));
    CHECK_THAT(text, ContainsSubstring("\"type\": \"v_bit\""));

    const ImportResult back = importToolDatabaseJson(text);
    REQUIRE(back.ok);
    CHECK_THAT(back.database.findTool("v")->includedAngle, WithinAbs(kPi / 2.0, 1e-12));
    CHECK_THAT(back.database.findTool("v")->diameter, WithinAbs(0.3, 1e-12));
}

TEST_CASE("Import reproduces the database contents", "[tooling][json]") {
    const ToolDatabase db = sampleDb();
    const ImportResult imported = importToolDatabaseJson(exportToolDatabaseJson(db));
    REQUIRE(imported.ok);
    const ToolDatabase& in = imported.database;
    CHECK(in.groups().size() == db.groups().size());
    CHECK(in.materials() == db.materials());
    CHECK(in.tools().size() == db.tools().size());
    CHECK(in.findGroup("g-fine")->parentId == "g-root");

    const Machine* m = in.findMachine("mc-router");
    REQUIRE(m != nullptr);
    CHECK(m->name == "Router \"big\" / ünïcode");
    CHECK_THAT(m->accelerationX, WithinAbs(1234.56789, 1e-9));
    CHECK_THAT(m->accelerationY, WithinAbs(0.3, 1e-9));
    CHECK(m->accelerationZ == 0.0);
    CHECK_THAT(m->accelerationAngular, WithinAbs(kPi / 7.0, 1e-7));  // 1e-6 deg grid
    CHECK(m->associatedPosts == std::vector<std::string>{"grbl_mm", "grbl_inch"});
    CHECK(m->defaultPost == "grbl_mm");
    CHECK(in.findMachine("mc-bare") != nullptr);

    for (const auto& [id, original] : db.tools()) {
        const Tool* t = in.findTool(id);
        REQUIRE(t != nullptr);
        CHECK(t->type == original.type);
        CHECK(t->name == original.name);
        CHECK(t->notes == original.notes);
        CHECK(t->groupId == original.groupId);
        CHECK(t->flutes == original.flutes);
        CHECK_THAT(t->diameter, WithinAbs(original.diameter, 1e-9));
        CHECK_THAT(t->includedAngle, WithinAbs(original.includedAngle, 1e-9));
        CHECK_THAT(t->tipDiameter, WithinAbs(original.tipDiameter, 1e-9));
        CHECK_THAT(t->cornerRadius, WithinAbs(original.cornerRadius, 1e-9));
        REQUIRE(t->profile.size() == original.profile.size());
        for (std::size_t i = 0; i < t->profile.size(); ++i) {
            CHECK_THAT(t->profile[i].x, WithinAbs(original.profile[i].x, 1e-6));
            CHECK_THAT(t->profile[i].y, WithinAbs(original.profile[i].y, 1e-6));
        }
        // stepover was 0.1+0.2 and is quantised to 0.3 on export; everything else is exact.
        const auto sameData = [](const CuttingData& x, const CuttingData& y) {
            CHECK_THAT(x.stepover, WithinAbs(y.stepover, 1e-6));
            CuttingData yy = y;
            yy.stepover = x.stepover;
            CHECK(x == yy);
        };
        sameData(t->defaultCuttingData, original.defaultCuttingData);
        REQUIRE(t->cuttingData.size() == original.cuttingData.size());
        for (const auto& [key, data] : original.cuttingData) {
            REQUIRE(t->cuttingData.contains(key));
            sameData(t->cuttingData.at(key), data);
        }
        // Cutting width behaviour survives the round trip.
        CHECK_THAT(cuttingWidthAtDepth(*t, 2.0), WithinAbs(cuttingWidthAtDepth(original, 2.0), 1e-6));
    }
}

TEST_CASE("Empty database round-trips", "[tooling][json]") {
    const std::string text = exportToolDatabaseJson(ToolDatabase{});
    const ImportResult r = importToolDatabaseJson(text);
    REQUIRE(r.ok);
    CHECK(r.database.isEmpty());
    CHECK(exportToolDatabaseJson(r.database) == text);
}

TEST_CASE("Import accepts groups listed before their parents and fills defaults", "[tooling][json]") {
    const std::string text = R"({
        "format": "carvetto-tool-database",
        "version": 1,
        "groups": [{"id": "child", "name": "C", "parent_id": "parent"}, {"id": "parent", "name": "P"}],
        "materials": [],
        "machines": [{"id": "m"}],
        "tools": [{"id": "t", "type": "end_mill", "diameter_mm": 6, "group_id": "child", "future_key": 42}]
    })";
    const ImportResult r = importToolDatabaseJson(text);
    REQUIRE(r.ok);
    CHECK(r.database.findGroup("child")->parentId == "parent");
    const Tool* t = r.database.findTool("t");
    REQUIRE(t != nullptr);
    CHECK(t->flutes == 1);
    CHECK(t->name.empty());
    CHECK(t->cuttingData.empty());
    CHECK(t->defaultCuttingData == CuttingData{});
    CHECK(r.database.findMachine("m")->accelerationX == 0.0);
}

TEST_CASE("Import rejects malformed input with a descriptive error", "[tooling][json]") {
    const auto expectError = [](const std::string& text, const char* needle) {
        const ImportResult r = importToolDatabaseJson(text);
        CHECK_FALSE(r.ok);
        CHECK_FALSE(static_cast<bool>(r));
        CHECK(r.database.isEmpty());
        CHECK_THAT(r.error, ContainsSubstring(needle));
    };
    const std::string head =
        R"({"format": "carvetto-tool-database", "version": 1, "groups": [], "materials": [], "machines": [], )";

    expectError("not json at all", "invalid JSON");
    expectError("[]", "must be an object");
    expectError(R"({"version": 1})", "missing key 'format'");
    expectError(R"({"format": "something-else", "version": 1})", "unrecognised format");
    expectError(R"({"format": "carvetto-tool-database", "version": 99})", "unsupported version");
    expectError(R"({"format": "carvetto-tool-database", "version": 1})", "missing key 'groups'");
    expectError(head + R"("tools": {}})", "'tools' must be an array");
    expectError(head + R"("tools": [{"id": "t", "type": "laser", "diameter_mm": 6}]})",
                "unknown tool type 'laser'");
    expectError(head + R"("tools": [{"id": "t", "type": "end_mill", "diameter_mm": "six"}]})",
                "must be a number");
    expectError(head + R"("tools": [{"id": "t", "type": "end_mill", "diameter_mm": 0}]})",
                "diameter must be positive");
    expectError(
        head + R"("tools": [{"id": "t", "type": "v_bit", "diameter_mm": 6, "included_angle_deg": 0}]})",
        "included angle");
    expectError(head + R"("tools": [{"id": "t", "type": "end_mill", "diameter_mm": 6, "flutes": 1.5}]})",
                "must be an integer");
    expectError(
        head +
            R"("tools": [{"id": "t", "type": "end_mill", "diameter_mm": 6}, {"id": "t", "type": "end_mill", "diameter_mm": 3}]})",
        "duplicate id");
    expectError(head + R"("tools": [{"id": "t", "type": "end_mill", "diameter_mm": 6, "group_id": "nope"}]})",
                "unknown group 'nope'");
    expectError(head + R"("tools": [{"type": "end_mill", "diameter_mm": 6}]})", "missing key 'id'");
    expectError(head + R"("tools": [{"id": "t", "type": "form_tool", "profile_mm": [[1, 0], [2]]}]})",
                "[radius, height]");
    expectError(head + R"("tools": [{"id": "t", "type": "end_mill", "diameter_mm": 6,
                 "cutting_data": [{"machine_id": "m", "material_id": "x", "data": {}},
                                  {"machine_id": "m", "material_id": "x", "data": {}}]}]})",
                "duplicate entry");
    expectError(head + R"("tools": [{"id": "t", "type": "end_mill", "diameter_mm": 6,
                 "cutting_data": [{"machine_id": "m", "data": {}}]}]})",
                "missing key 'material_id'");
    expectError(
        R"({"format": "carvetto-tool-database", "version": 1, "groups": [{"id": "a", "parent_id": "zzz"}], "materials": [], "machines": [], "tools": []})",
        "unknown parent 'zzz'");
    expectError(
        R"({"format": "carvetto-tool-database", "version": 1, "groups": [{"id": "a", "parent_id": "b"}, {"id": "b", "parent_id": "a"}], "materials": [], "machines": [], "tools": []})",
        "unknown parent");
    expectError(
        R"({"format": "carvetto-tool-database", "version": 1, "groups": [], "materials": [{"id": "x"}, {"id": "x"}], "machines": [], "tools": []})",
        "duplicate id");
    expectError(
        R"({"format": "carvetto-tool-database", "version": 1, "groups": [], "materials": [], "machines": [{"id": "m", "acceleration_mm_s2": [1, 2]}], "tools": []})",
        "three numbers");
}

TEST_CASE("Imported file can be merged into an existing database", "[tooling][json]") {
    ToolDatabase mine;
    Tool t;
    t.id = "tool-0";
    t.name = "Mine";
    t.type = ToolType::EndMill;
    t.diameter = 1.0;
    mine.addTool(t);

    const ImportResult theirs = importToolDatabaseJson(exportToolDatabaseJson(sampleDb()));
    REQUIRE(theirs.ok);
    const MergeReport report = mine.merge(theirs.database, MergePolicy::KeepExisting);
    CHECK(report.tools.kept == 1);
    CHECK(report.tools.added == static_cast<int>(sampleDb().tools().size()) - 1);
    CHECK(mine.findTool("tool-0")->name == "Mine");
}
