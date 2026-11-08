// T-0220 AC1: the session's toolpath records are byte-identical to those of carvetto-cli for the
// same inputs, for each of the five operations (CLI end-to-end data of T-0122/T-0150/T-0151/T-0153
// and the DXF fixtures under core/tests/fixtures/dxf).

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cli.hpp"
#include "carvetto/document/carv_io.hpp"
#include "toolpath_test_support.hpp"

namespace fs = std::filesystem;
using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;

namespace {

const std::string kData = CARVETTO_CLI_DATA_DIR;

struct CliRun {
    int code{0};
    std::string out;
    std::string err;
};

CliRun runCli(const std::vector<std::string>& args) {
    std::ostringstream out;
    std::ostringstream err;
    const int code = cli::run(args, out, err);
    return {code, out.str(), err.str()};
}

std::string workDir(const std::string& name) {
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / name;
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir.generic_string();
}

std::string bytesOf(const std::string& path) {
    std::ifstream in(fs::path(path), std::ios::binary);
    REQUIRE(in);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

VectorSelector layersNamed(const document::Document& doc, const std::vector<std::string>& names) {
    std::vector<document::LayerId> ids;
    for (const std::string& n : names) {
        for (const document::Layer& l : doc.layers()) {
            if (l.name == n) {
                ids.push_back(l.id);
            }
        }
    }
    return VectorSelector::fromLayers(ids);
}

struct Case {
    std::string name;
    std::string dxf;
    std::string op;  // CLI operation
    /// The CLI params file (inline JSON with the tool database path filled in).
    std::string paramsJson;
    /// Builds the session's definition through the form contract.
    std::function<ToolpathDefinition(const tooling::ToolDatabase&, const document::Document&)> make;
};

std::string tool(const std::string& id, const std::string& extra = {}) {
    return "\"tool\": {\"database\": \"" + kData + "/tools.json\", \"id\": \"" + id + "\"" + extra + "}";
}

void runCase(const Case& c) {
    const std::string dir = workDir("identity_" + c.name);
    const tooling::ToolDatabase db = cliToolDatabase();
    REQUIRE(runCli({"import", c.dxf, dir + "/in.carv"}).code == 0);
    {
        std::ofstream p(dir + "/params.json", std::ios::binary);
        p << c.paramsJson;
    }
    const CliRun cli = runCli({"toolpath", c.op, dir + "/params.json", dir + "/in.carv", dir + "/cli.carv"});
    INFO(cli.err);
    REQUIRE(cli.code == 0);

    auto loaded = document::loadCarv(fs::path(dir + "/in.carv"));
    REQUIRE(loaded);
    document::Document& doc = loaded.value().document;
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const ToolpathDefinition def = c.make(db, doc);
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.checkIntegrity().empty());
    REQUIRE(document::saveCarv(doc, fs::path(dir + "/mine.carv"), loaded.value().extras));

    // The native records: equal as records, and the whole files equal byte for byte.
    auto cliDoc = document::loadCarv(fs::path(dir + "/cli.carv"));
    REQUIRE(cliDoc);
    REQUIRE(cliDoc.value().document.toolpaths().size() == doc.toolpaths().size());
    for (std::size_t i = 0; i < doc.toolpaths().size(); ++i) {
        const document::ToolpathRecord& mine = doc.toolpaths()[i];
        document::ToolpathRecord theirs = cliDoc.value().document.toolpaths()[i];
        theirs.id = mine.id;
        CHECK(mine == theirs);
        CHECK(mine.moveCount() > 0);
    }
    const std::string mineBytes = bytesOf(dir + "/mine.carv");
    CHECK(mineBytes == bytesOf(dir + "/cli.carv"));
    std::size_t moves = 0;
    for (const document::ToolpathRecord& r : doc.toolpaths()) {
        moves += r.moveCount();
    }
    Fingerprint fp;
    fp.add(std::string_view(mineBytes));
    std::cout << "[T-0220 CLI identity] " << c.name << ": " << doc.toolpaths().size() << " record(s), " << moves
              << " moves, .carv " << mineBytes.size() << " bytes, hash " << std::hex << fp.value() << std::dec
              << " (session = CLI)" << std::endl;
}

ToolpathForm formFor(document::ToolpathOperation op, const tooling::ToolDatabase& db, const std::string& toolId,
                     const std::string& machine = {}, const std::string& material = {}) {
    ToolpathForm form(op, document::Units::Millimetres);
    REQUIRE(form.setTool(db, source(toolId, machine, material)).ok);
    return form;
}

void ok(const FieldResult& r) {
    INFO(r.message);
    REQUIRE(r.ok);
}

const std::string kPart = kData + "/part.dxf";
const std::string kSign = kData + "/sign.dxf";

}  // namespace

TEST_CASE("toolpath session records equal the CLI: profile with tabs", "[toolpath][cli]") {
    runCase({"profile", kPart, "profile", R"({"name": "Cut out", "layers": ["Outline"], "side": "outside",
        "direction": "climb", "passes": {"cut_depth": 6, "pass_depth": 3}, "safe_z": 5,
        "tabs": {"enabled": true, "count": 4, "length": 8, "thickness": 2}, )" + tool("end-mill-3mm") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Profile, db, "end-mill-3mm");
                 ok(f.setName("Cut out"));
                 ok(f.setField("cutDepth", "6"));
                 ok(f.setField("passDepth", "3"));
                 ok(f.setField("safeZ", "5"));
                 ok(f.setChoice("tabsEnabled", 1));
                 ok(f.setField("tabCount", "4"));
                 ok(f.setField("tabLength", "8"));
                 ok(f.setField("tabThickness", "2"));
                 f.setSelector(layersNamed(doc, {"Outline"}));
                 return f.definition();
             }});
}

TEST_CASE("toolpath session records equal the CLI: pocket with machine and material", "[toolpath][cli]") {
    runCase({"pocket", kPart, "pocket", R"({"name": "Pockets", "layers": ["Pockets"], "start_depth": 0, "cut_depth": 4,
        "safe_z": 5, "strategy": "offset", "direction": "climb", "profile_pass": "last", )" +
                                            tool("end-mill-6mm", ", \"machine\": \"machine-generic-router\", "
                                                                  "\"material\": \"material-hardwood-mdf\"") +
                                            "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Pocket, db, "end-mill-6mm",
                                          "machine-generic-router", "material-hardwood-mdf");
                 ok(f.setName("Pockets"));
                 ok(f.setField("startDepth", "0"));
                 ok(f.setField("cutDepth", "4"));
                 ok(f.setField("safeZ", "5"));
                 f.setSelector(layersNamed(doc, {"Pockets"}));
                 return f.definition();
             }});
}

TEST_CASE("toolpath session records equal the CLI: raster pocket in inches", "[toolpath][cli]") {
    // The same 4 mm depth typed as 0.15748031496 in would differ in the last bits; type exact mm
    // values and check that unit suffixes work through the form.
    runCase({"pocket_raster", kPart, "pocket", R"({"name": "Raster", "layers": ["Pockets"], "cut_depth": 3.175,
        "strategy": "raster", "raster_angle": 30, "ramp_plunge": true, "ramp_distance": 6, )" +
                                                   tool("end-mill-3mm") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Pocket, db, "end-mill-3mm");
                 ok(f.setName("Raster"));
                 ok(f.setField("cutDepth", "0.125in"));
                 ok(f.setChoice("strategy", 1));
                 ok(f.setField("rasterAngle", "30"));
                 ok(f.setChoice("rampPlunge", 1));
                 ok(f.setField("rampDistance", "6"));
                 f.setSelector(layersNamed(doc, {"Pockets"}));
                 return f.definition();
             }});
}

TEST_CASE("toolpath session records equal the CLI: drill with peck and dwell", "[toolpath][cli]") {
    runCase({"drill", kPart, "drill", R"({"name": "Holes", "layers": ["Pockets"], "cut_depth": 7, "safe_z": 5,
        "peck": true, "peck_retract": "above_start", "retract_gap": 1, "dwell_time": 0.5, )" +
                                          tool("drill-3mm") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Drill, db, "drill-3mm");
                 ok(f.setName("Holes"));
                 ok(f.setField("cutDepth", "7"));
                 ok(f.setField("safeZ", "5"));
                 ok(f.setChoice("peck", 1));
                 ok(f.setChoice("peckRetract", 0));
                 ok(f.setField("retractGap", "1"));
                 ok(f.setField("dwellTime", "0.5"));
                 f.setSelector(layersNamed(doc, {"Pockets"}));
                 return f.definition();
             }});
}

TEST_CASE("toolpath session records equal the CLI: engrave fill and outline", "[toolpath][cli]") {
    runCase({"engrave_fill", kPart, "engrave", R"({"name": "Fill", "layers": ["Pockets"], "depth": 1, "passes": 2,
        "safe_z": 5, "strategy": "fill", "fill_pattern": "raster", "raster_angle": 45, "stepover": 0.8, )" +
                                                   tool("v-bit-90") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Engrave, db, "v-bit-90");
                 ok(f.setName("Fill"));
                 ok(f.setField("depth", "1"));
                 ok(f.setField("passes", "2"));
                 ok(f.setChoice("strategy", 1));
                 ok(f.setChoice("fillPattern", 1));
                 ok(f.setField("rasterAngle", "45"));
                 ok(f.setField("stepover", "0.8"));
                 f.setSelector(layersNamed(doc, {"Pockets"}));
                 return f.definition();
             }});
    runCase({"engrave_outline", kPart, "engrave", R"({"name": "Border", "layers": ["Outline"], "depth": 0.5,
        "safe_z": 5, "strategy": "outline", )" + tool("v-bit-90") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Engrave, db, "v-bit-90");
                 ok(f.setName("Border"));
                 ok(f.setField("depth", "0.5"));
                 f.setSelector(layersNamed(doc, {"Outline"}));
                 return f.definition();
             }});
}

TEST_CASE("toolpath session records equal the CLI: V-carve with clearance tools and without", "[toolpath][cli]") {
    runCase({"v_carve_flat", kSign, "v_carve", R"({"name": "Panel", "layers": ["Panel"], "safe_z": 5,
        "flat_depth": 1.5, "clearance_tools": [{"database": ")" + kData + R"(/tools.json", "id": "end-mill-3mm"}],
        "clearance_strategy": "raster", "clearance_raster_angle": 45, "tolerance": {"chordal": 0.05}, )" +
                                                  tool("v-bit-90") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::VCarving, db, "v-bit-90");
                 ok(f.setName("Panel"));
                 ok(f.setField("safeZ", "5"));
                 ok(f.setField("flatDepth", "1.5"));
                 ok(f.setClearanceTools(db, {source("end-mill-3mm")}));
                 ok(f.setChoice("clearanceStrategy", 1));
                 ok(f.setField("clearanceRasterAngle", "45"));
                 ok(f.setField("chordal", "0.05"));
                 f.setSelector(layersNamed(doc, {"Panel"}));
                 return f.definition();
             }});
    runCase({"v_carve_letters", kSign, "v_carve", R"({"name": "Letters", "layers": ["Letters"], "safe_z": 5,
        "order": {"left_to_right": true}, "tolerance": {"chordal": 0.05}, )" + tool("v-bit-90") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::VCarving, db, "v-bit-90");
                 ok(f.setName("Letters"));
                 ok(f.setChoice("orderLeftToRight", 1));
                 ok(f.setField("chordal", "0.05"));
                 f.setSelector(layersNamed(doc, {"Letters"}));
                 return f.definition();
             }});
}

TEST_CASE("toolpath session records equal the CLI on the DXF fixtures", "[toolpath][cli]") {
    const std::string fixtures = CARVETTO_DXF_FIXTURE_DIR;
    // All vectors of the drawing (no layers key): profile on the line, drill and quick engrave.
    runCase({"fx_nested_profile", fixtures + "/nested_inserts.dxf", "profile",
             R"({"name": "All", "side": "on", "passes": {"cut_depth": 2, "pass_depth": 1}, )" +
                 tool("end-mill-3mm") + "}",
             [](const tooling::ToolDatabase& db, const document::Document&) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Profile, db, "end-mill-3mm");
                 ok(f.setName("All"));
                 ok(f.setChoice("side", 2));
                 ok(f.setField("cutDepth", "2"));
                 ok(f.setField("passDepth", "1"));
                 return f.definition();
             }});
    runCase({"fx_nested_pocket", fixtures + "/nested_inserts.dxf", "pocket",
             R"({"name": "All", "cut_depth": 2, )" + tool("end-mill-3mm") + "}",
             [](const tooling::ToolDatabase& db, const document::Document&) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Pocket, db, "end-mill-3mm");
                 ok(f.setName("All"));
                 ok(f.setField("cutDepth", "2"));
                 return f.definition();
             }});
    runCase({"fx_all_entities_engrave", fixtures + "/all_entities.dxf", "engrave",
             R"({"name": "Marks", "depth": 0.3, )" + tool("v-bit-90") + "}",
             [](const tooling::ToolDatabase& db, const document::Document&) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Engrave, db, "v-bit-90");
                 ok(f.setName("Marks"));
                 ok(f.setField("depth", "0.3"));
                 return f.definition();
             }});
    runCase({"fx_r12_drill", fixtures + "/r12_entities.dxf", "drill",
             R"({"name": "Holes", "cut_depth": 4, )" + tool("drill-3mm") + "}",
             [](const tooling::ToolDatabase& db, const document::Document&) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Drill, db, "drill-3mm");
                 ok(f.setName("Holes"));
                 ok(f.setField("cutDepth", "4"));
                 return f.definition();
             }});
}

TEST_CASE("toolpath session records equal the CLI past the save-trial threshold", "[toolpath][cli]") {
    // More than 2000 moves: the session tries the save out on a scratch copy; ids and bytes must not differ.
    runCase({"pocket_large", kPart, "pocket", R"({"name": "Fine", "layers": ["Pockets"], "cut_depth": 4, )" +
                                                  tool("end-mill-3mm") + "}",
             [](const tooling::ToolDatabase& db, const document::Document& doc) {
                 ToolpathForm f = formFor(document::ToolpathOperation::Pocket, db, "end-mill-3mm");
                 ok(f.setName("Fine"));
                 ok(f.setField("cutDepth", "4"));
                 f.setSelector(layersNamed(doc, {"Pockets"}));
                 return f.definition();
             }});
}
