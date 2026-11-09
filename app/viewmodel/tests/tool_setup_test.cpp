// T-0235: tool database / machine / material setup logic: persistence round trip, hostile files,
// validation, ids, defaults per tool type, usage query and the effect of edits on toolpaths.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "carvetto/tooling/json_io.hpp"
#include "carvetto/viewmodel/tool_setup.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
namespace fs = std::filesystem;

namespace {

fs::path workDir(const std::string& name) {
    fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "tool_setup" / name;
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);
    return dir;
}

void writeText(const fs::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << text;
}

tooling::ToolDatabase starter() {
    const tooling::ImportResult r =
        tooling::importToolDatabaseJson(readTextFile(CARVETTO_STARTER_TOOLS));
    REQUIRE(r.ok);
    return r.database;
}

}  // namespace

TEST_CASE("setup: the starter database validates and the generic router has a rapid rate", "[tool_setup]") {
    const tooling::ToolDatabase db = starter();
    CHECK_FALSE(validateSetup(db).has_value());
    const tooling::Machine* m = activeMachine(db, "nope");
    REQUIRE(m != nullptr);
    CHECK(m->rapidRate > 0.0);
    CHECK(activeMachine(db, m->id) == m);
    CHECK(activeMachine(tooling::ToolDatabase{}, "x") == nullptr);
}

TEST_CASE("setup: persistence round trip keeps tools, machines and materials", "[tool_setup]") {
    const fs::path dir = workDir("roundtrip");
    tooling::ToolDatabase db = starter();

    tooling::Machine m = defaultMachine("Shop router");
    m.id = newMachineId(db);
    m.rapidRate = 7500.0;
    m.workAreaX = 1200.0;
    m.workAreaY = 800.0;
    m.workAreaZ = 120.0;
    m.maxSpindleSpeed = 24000.0;
    m.maxFeedRate = 9000.0;
    m.accelerationX = 400.0;
    m.associatedPosts = {"grbl", "mach3"};
    m.defaultPost = "grbl";
    REQUIRE(db.addMachine(m));
    tooling::Material mat{newMaterialId(db), "Oak"};
    REQUIRE(db.addMaterial(mat));
    tooling::Tool t = defaultTool(tooling::ToolType::VBit);
    t.id = newToolId(db);
    t.name = "My V-bit";
    t.defaultCuttingData.feedRate = 1500.0;
    t.setCuttingData(m.id, mat.id, t.defaultCuttingData);
    REQUIRE(db.addTool(t));

    const fs::path file = dir / "config" / kSetupFileName;  // parent folder does not exist yet
    std::string error;
    REQUIRE(writeToolDatabaseFile(file, db, error));
    CHECK(error.empty());
    CHECK_FALSE(fs::exists(fs::path(file) += ".tmp"));

    const SetupLoadResult loaded = loadToolSetup(file, tooling::ToolDatabase{});
    REQUIRE(loaded.status == SetupLoadStatus::Loaded);
    CHECK(loaded.database == db);
    CHECK(tooling::exportToolDatabaseJson(loaded.database) == tooling::exportToolDatabaseJson(db));
    const tooling::Machine* back = loaded.database.findMachine(m.id);
    REQUIRE(back != nullptr);
    CHECK(back->rapidRate == 7500.0);
    CHECK(back->workAreaY == 800.0);
    CHECK(back->defaultPost == "grbl");

    // Writing again replaces the file atomically.
    db.removeTool(t.id);
    REQUIRE(writeToolDatabaseFile(file, db, error));
    CHECK(loadToolSetup(file, tooling::ToolDatabase{}).database.findTool(t.id) == nullptr);
}

TEST_CASE("setup: a missing file gives the defaults without a message", "[tool_setup]") {
    const fs::path dir = workDir("missing");
    const tooling::ToolDatabase defaults = starter();
    const SetupLoadResult r = loadToolSetup(dir / "none.json", defaults);
    CHECK(r.status == SetupLoadStatus::NotFound);
    CHECK(r.message.empty());
    CHECK(r.database == defaults);
}

TEST_CASE("setup: hostile files are refused and the defaults kept", "[tool_setup]") {
    const fs::path dir = workDir("hostile");
    const tooling::ToolDatabase defaults = starter();
    const std::string good = tooling::exportToolDatabaseJson(defaults);

    const auto expectRefused = [&](const std::string& label, const std::string& text) {
        INFO(label);
        const fs::path f = dir / "bad.json";
        writeText(f, text);
        const SetupLoadResult r = loadToolSetup(f, defaults);
        CHECK(r.status == SetupLoadStatus::Refused);
        CHECK_FALSE(r.message.empty());
        CHECK(r.database == defaults);
        CHECK_FALSE(readToolDatabaseFile(f).ok);
    };

    expectRefused("empty", "");
    expectRefused("garbage", "\x01\x02 not json {{{");
    expectRefused("truncated", good.substr(0, good.size() / 2));
    expectRefused("wrong format", "{\"format\":\"other\",\"version\":1}");
    expectRefused("future version",
                  "{\"format\":\"carvetto-tool-database\",\"version\":99,\"groups\":[],\"materials\":[],"
                  "\"machines\":[],\"tools\":[]}");
    expectRefused("array root", "[1,2,3]");
    expectRefused("deep nesting", std::string(200000, '[') + std::string(200000, ']'));

    const auto withMachine = [](const std::string& fields) {
        return "{\"format\":\"carvetto-tool-database\",\"version\":1,\"groups\":[],\"materials\":[],"
               "\"machines\":[{\"id\":\"m\",\"name\":\"M\"" +
               fields + "}],\"tools\":[]}";
    };
    expectRefused("string for number", withMachine(",\"rapid_mm_min\":\"fast\""));
    expectRefused("overflowing number", withMachine(",\"rapid_mm_min\":1e999"));
    expectRefused("absurd magnitude", withMachine(",\"rapid_mm_min\":1e300"));
    expectRefused("negative rate", withMachine(",\"rapid_mm_min\":-5"));
    expectRefused("NaN literal", withMachine(",\"rapid_mm_min\":NaN"));
    expectRefused("default post not associated", withMachine(",\"default_post\":\"x\""));
    expectRefused("nameless machine",
                  "{\"format\":\"carvetto-tool-database\",\"version\":1,\"groups\":[],\"materials\":[],"
                  "\"machines\":[{\"id\":\"m\"}],\"tools\":[]}");
    expectRefused("tool with zero diameter",
                  "{\"format\":\"carvetto-tool-database\",\"version\":1,\"groups\":[],\"materials\":[],"
                  "\"machines\":[],\"tools\":[{\"id\":\"t\",\"name\":\"T\",\"type\":\"end_mill\","
                  "\"diameter_mm\":0}]}");
    expectRefused("tool with huge feed",
                  "{\"format\":\"carvetto-tool-database\",\"version\":1,\"groups\":[],\"materials\":[],"
                  "\"machines\":[],\"tools\":[{\"id\":\"t\",\"name\":\"T\",\"type\":\"end_mill\","
                  "\"diameter_mm\":3,\"default_cutting_data\":{\"feed_mm_min\":1e12}}]}");

    // A directory and an oversized file are refused without being read in full.
    const SetupLoadResult asDir = loadToolSetup(dir, defaults);
    CHECK(asDir.status == SetupLoadStatus::Refused);
    const fs::path big = dir / "big.json";
    {
        std::ofstream out(big, std::ios::binary);
        const std::string chunk(1024 * 1024, ' ');
        for (std::uintmax_t i = 0; i <= kMaxSetupFileBytes / chunk.size(); ++i) {
            out << chunk;
        }
    }
    const tooling::ImportResult huge = readToolDatabaseFile(big);
    CHECK_FALSE(huge.ok);
    CHECK(huge.error.find("too large") != std::string::npos);
}

TEST_CASE("setup: a failed write leaves the previous file intact", "[tool_setup]") {
    const fs::path dir = workDir("atomic");
    const tooling::ToolDatabase db = starter();
    const fs::path file = dir / kSetupFileName;
    std::string error;
    REQUIRE(writeToolDatabaseFile(file, db, error));
    const std::string before = readTextFile(file.string());

    tooling::ToolDatabase bad = db;
    tooling::Machine m = *bad.findMachine("machine-generic-router");
    m.rapidRate = std::numeric_limits<double>::quiet_NaN();
    REQUIRE(bad.updateMachine(m));
    CHECK_FALSE(writeToolDatabaseFile(file, bad, error));
    CHECK_FALSE(error.empty());
    CHECK(readTextFile(file.string()) == before);

    // The target cannot be created below a regular file.
    CHECK_FALSE(writeToolDatabaseFile(file / "sub" / "x.json", db, error));
}

TEST_CASE("setup: export and import of the tool database use the same file", "[tool_setup]") {
    const fs::path dir = workDir("import");
    const tooling::ToolDatabase db = starter();
    std::string error;
    REQUIRE(writeToolDatabaseFile(dir / "export.json", db, error));
    const tooling::ImportResult r = readToolDatabaseFile(dir / "export.json");
    REQUIRE(r.ok);
    tooling::ToolDatabase target;
    const tooling::MergeReport report = target.merge(r.database, tooling::MergePolicy::KeepExisting);
    CHECK(report.tools.added == static_cast<int>(db.tools().size()));
    CHECK(target == db);
}

TEST_CASE("setup: validation of machines and tools", "[tool_setup]") {
    tooling::Machine m = defaultMachine("M");
    m.id = "m";
    CHECK_FALSE(validateMachine(m));
    m.name.clear();
    CHECK(validateMachine(m));
    m.name = "M";
    m.rapidRate = -1.0;
    CHECK(validateMachine(m));
    m.rapidRate = 100.0;
    m.defaultPost = "p";
    CHECK(validateMachine(m));
    m.associatedPosts = {"p"};
    CHECK_FALSE(validateMachine(m));

    tooling::Tool t = defaultTool(tooling::ToolType::EndMill);
    t.id = "t";
    CHECK(validateToolForEdit(t));  // nameless
    t.name = "T";
    CHECK_FALSE(validateToolForEdit(t));
    t.diameter = std::numeric_limits<double>::infinity();
    CHECK(validateToolForEdit(t));
    t.diameter = 6.0;
    t.defaultCuttingData.feedRate = -3.0;
    CHECK(validateToolForEdit(t));
}

TEST_CASE("setup: a default tool of every type is valid and shows the right fields", "[tool_setup]") {
    for (const tooling::ToolType type : tooling::allToolTypes()) {
        INFO(toolTypeLabel(type));
        tooling::Tool t = defaultTool(type);
        t.id = "t";
        t.name = "T";
        CHECK_FALSE(validateToolForEdit(t));
        CHECK_FALSE(toolTypeLabel(type).empty());
    }
    CHECK(toolFields(tooling::ToolType::VBit).includedAngle);
    CHECK_FALSE(toolFields(tooling::ToolType::EndMill).includedAngle);
    CHECK(toolFields(tooling::ToolType::Engraving).tipDiameter);
    CHECK(toolFields(tooling::ToolType::RadiusedEndMill).cornerRadius);
    CHECK_FALSE(toolFields(tooling::ToolType::FormTool).diameter);
    CHECK(toolFields(tooling::ToolType::FormTool).profile);
}

TEST_CASE("setup: ids and copy names are unique", "[tool_setup]") {
    tooling::ToolDatabase db;
    CHECK(newToolId(db) == "tool-1");
    tooling::Tool t = defaultTool(tooling::ToolType::EndMill);
    t.id = newToolId(db);
    t.name = "A";
    REQUIRE(db.addTool(t));
    CHECK(newToolId(db) == "tool-2");
    CHECK(copyNameForTool(db, "A") == "A copy");
    REQUIRE(db.copyTool("tool-1", newToolId(db), copyNameForTool(db, "A")));
    CHECK(copyNameForTool(db, "A") == "A copy 2");
    CHECK(newMachineId(db) == "machine-1");
    CHECK(newMaterialId(db) == "material-1");
    CHECK(newGroupId(db) == "group-1");
}

TEST_CASE("setup: deleting or editing a tool is reflected in the toolpaths using it", "[tool_setup]") {
    document::Document doc;
    tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId big = addVector(doc, square(0, 0, 30));
    ToolpathDefinition def = pocketDefinition(db);
    def.selector = VectorSelector::fromObjects({big});
    const ToolpathOutcome out = session.add(def);
    REQUIRE(out.ok);
    const document::ToolpathId id = out.ids.front();

    const auto uses = toolpathsUsingTool(doc, "end-mill-6mm");
    REQUIRE(uses.size() == 1);
    CHECK(uses.front().id == id);
    CHECK(toolpathsUsingTool(doc, "end-mill-3mm").empty());

    // Renaming keeps the id: the toolpath still finds its tool.
    tooling::Tool renamed = *db.findTool("end-mill-6mm");
    renamed.name = "Renamed cutter";
    REQUIRE(db.updateTool(renamed));
    session.setToolDatabase(db);
    CHECK(session.item(id)->canRecalculate);

    // Deleting it: the toolpath stays, cannot be recalculated, and says why.
    db.removeTool("end-mill-6mm");
    session.setToolDatabase(db);
    const auto item = session.item(id);
    REQUIRE(item.has_value());
    CHECK_FALSE(item->canRecalculate);
    CHECK_FALSE(item->note.empty());
}
