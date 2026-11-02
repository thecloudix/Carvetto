// T-0122: carvetto-cli commands, driven in-process through cli::run.

#include "cli.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "starter_tools.hpp"
#include "toolpath_store.hpp"

namespace fs = std::filesystem;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using namespace carvetto;

namespace {

const std::string kData = CARVETTO_CLI_DATA_DIR;
const std::string kGolden = std::string(CARVETTO_GOLDEN_DIR) + "/cli/part_grbl.nc";
const std::string kGrbl = std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp";

struct Outcome {
    int code{0};
    std::string out;
    std::string err;
};

Outcome runCli(const std::vector<std::string>& args) {
    std::ostringstream out;
    std::ostringstream err;
    const int code = cli::run(args, out, err);
    return {code, out.str(), err.str()};
}

/// A fresh scratch directory under the build tree, one per test case (ctest runs cases in parallel).
std::string workDir(const std::string& name) {
    const fs::path dir = fs::path(CARVETTO_CLI_WORK_DIR) / name;
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir.generic_string();
}

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void writeFile(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    REQUIRE(out);
    out << text;
}

/// The end-to-end pipeline of AC1; returns the G-code.
std::string pipeline(const std::string& dir) {
    const std::string carv = dir + "/part.carv";
    const std::string nc = dir + "/part.nc";
    for (const auto& args : std::vector<std::vector<std::string>>{
             {"import", kData + "/part.dxf", carv},
             {"toolpath", "pocket", kData + "/pocket.json", carv, carv},
             {"toolpath", "profile", kData + "/profile.json", carv, carv},
             {"post", kGrbl, carv, nc},
         }) {
        const Outcome r = runCli(args);
        INFO(args[0] << ": " << r.err);
        REQUIRE(r.code == cli::kExitOk);
        CHECK(r.err.empty());
    }
    return readFile(nc);
}

/// A .carv with the fixture imported.
std::string importedFixture(const std::string& dir) {
    const std::string carv = dir + "/part.carv";
    REQUIRE(runCli({"import", kData + "/part.dxf", carv}).code == 0);
    return carv;
}

}  // namespace

// ------------------------------------------------------------------------------------- AC1

TEST_CASE("AC1: DXF -> pocket + profile with tabs -> Grbl matches the golden", "[cli][golden]") {
    const std::string gcode = pipeline(workDir("ac1"));
    CHECK(gcode == readFile(kGolden));
}

TEST_CASE("the golden program is sane: two tools, tabs, safe start and end", "[cli][golden]") {
    const std::string g = readFile(kGolden);
    CHECK_THAT(g, StartsWith("(part)\n"));
    CHECK_THAT(g, ContainsSubstring("(Pockets)"));
    CHECK_THAT(g, ContainsSubstring("(Cut out)"));
    CHECK_THAT(g, ContainsSubstring("(Change to tool 5: End Mill 3 mm)"));
    CHECK_THAT(g, ContainsSubstring("G21"));
    CHECK_THAT(g, ContainsSubstring("Z-6"));  // profile bottom
    CHECK_THAT(g, ContainsSubstring("Z-4"));  // tab top (6 - 2) and pocket bottom
    CHECK_THAT(g, ContainsSubstring("M30"));
}

TEST_CASE("the pipeline is deterministic", "[cli]") {
    const std::string a = workDir("det_a");
    const std::string b = workDir("det_b");
    CHECK(pipeline(a) == pipeline(b));
    CHECK(readFile(a + "/part.carv") == readFile(b + "/part.carv"));
}

// Hidden (tag [.regen], never run by ctest): rewrites the CLI golden. Run via tools/regen_golden.py
// and hand-check the diff against posts/README.md before committing.
TEST_CASE("Regenerate the CLI golden", "[.regen]") {
    writeFile(kGolden, pipeline(workDir("regen")));
}

// ------------------------------------------------------------------------------------ help

TEST_CASE("AC2: --help documents every command", "[cli][help]") {
    const Outcome r = runCli({"--help"});
    CHECK(r.code == 0);
    CHECK(r.out == cli::helpText());
    CHECK(r.err.empty());
    for (const char* usage : {"import <in.dxf> <out.carv>", "toolpath profile <params.json> <in.carv> <out.carv>",
                              "toolpath pocket <params.json> <in.carv> <out.carv>", "post <post.pp> <in.carv> <out.nc>",
                              "info <file.carv>", "Exit status"}) {
        CHECK_THAT(r.out, ContainsSubstring(usage));
    }
    CHECK(runCli({"-h"}).out == cli::helpText());
    CHECK(runCli({"help"}).out == cli::helpText());
}

TEST_CASE("AC2: every command has its own help", "[cli][help]") {
    for (const std::string command : {"import", "toolpath", "post", "info"}) {
        INFO(command);
        const Outcome a = runCli({"help", command});
        CHECK(a.code == 0);
        CHECK_THAT(a.out, StartsWith("Usage: carvetto-cli " + command));
        const Outcome b = runCli({command, "--help"});
        CHECK(b.code == 0);
        CHECK(b.out == a.out);
    }
    CHECK_THAT(runCli({"help", "toolpath"}).out, ContainsSubstring("params.json reference"));
    CHECK_THAT(runCli({"--version"}).out, StartsWith("carvetto-cli "));
}

// ------------------------------------------------------------------------------ usage errors

TEST_CASE("AC2: wrong command lines exit with 2 and say why", "[cli][errors]") {
    struct Case {
        std::vector<std::string> args;
        const char* message;
    };
    const std::vector<Case> cases{
        {{}, "no command given"},
        {{"frobnicate"}, "unknown command 'frobnicate'"},
        {{"--frob"}, "unknown option '--frob'"},
        {{"import", "a.dxf"}, "'import' takes 2 argument(s), got 1"},
        {{"import", "a.dxf", "b.carv", "c"}, "'import' takes 2 argument(s), got 3"},
        {{"post", "--force", "a", "b"}, "unknown option '--force'"},
        {{"toolpath", "drill", "p.json", "a.carv", "b.carv"}, "unknown toolpath operation 'drill'"},
        {{"toolpath", "profile", "a.carv", "b.carv"}, "'toolpath' takes 4 argument(s), got 3"},
        {{"info"}, "'info' takes 1 argument(s), got 0"},
        {{"help", "nope"}, "unknown command 'nope'"},
    };
    for (const Case& c : cases) {
        const Outcome r = runCli(c.args);
        INFO(r.err);
        CHECK(r.code == cli::kExitUsage);
        CHECK(r.out.empty());
        CHECK_THAT(r.err, StartsWith("carvetto-cli: error: "));
        CHECK_THAT(r.err, ContainsSubstring(c.message));
        CHECK_THAT(r.err, ContainsSubstring("for usage."));
    }
    CHECK_THAT(runCli({"post", "a"}).err, ContainsSubstring("Usage: carvetto-cli post <post.pp> <in.carv> <out.nc>"));
}

// ---------------------------------------------------------------------------- input errors

TEST_CASE("AC2: failing commands exit with 1 and a clear message", "[cli][errors]") {
    const std::string dir = workDir("errors");
    const std::string carv = importedFixture(dir);
    const std::string missing = dir + "/missing";
    writeFile(dir + "/garbage.dxf", "this is not a drawing\n");
    writeFile(dir + "/bad.json", "{\"tool\": ");
    writeFile(dir + "/typo.json", R"({"tool": {"id": "end-mill-6mm"}, "tabs": {"lenght": 3}})");
    writeFile(dir + "/notool.json", R"({"tool": {"id": "no-such-tool"}, "passes": {"cut_depth": 3}})");
    writeFile(dir + "/nolayer.json", R"({"tool": {"id": "end-mill-6mm"}, "layers": ["Nope"]})");
    writeFile(dir + "/nodb.json", R"({"tool": {"id": "end-mill-6mm", "database": "nope.json"}})");
    writeFile(dir + "/huge.json",
              R"({"tool": {"id": "end-mill-6mm"}, "side": "inside", "layers": ["Pockets"], "passes": {"cut_depth": 1},
                  "allowance": 30})");
    writeFile(dir + "/emptypocket.json", R"({"tool": {"id": "end-mill-6mm"}, "layers": ["Pockets"]})");
    writeFile(dir + "/bad.pp", "begin HEADER\n\"[NO_SUCH_TAG]\"\nend\n");
    writeFile(dir + "/dup.json", R"({"tool": {"id": "end-mill-6mm"}, "safe_z": 5, "safe_z": 50})");
    writeFile(dir + "/overflow.json", R"({"tool": {"id": "end-mill-6mm"}, "safe_z": 1e400})");

    struct Case {
        std::vector<std::string> args;
        std::string message;
    };
    const std::vector<Case> cases{
        {{"import", missing + ".dxf", dir + "/x.carv"}, "cannot read DXF file '" + missing + ".dxf'"},
        {{"import", dir + "/garbage.dxf", dir + "/x.carv"}, "cannot import '" + dir + "/garbage.dxf'"},
        {{"info", missing + ".carv"}, "cannot load '" + missing + ".carv'"},
        {{"info", dir + "/garbage.dxf"}, "cannot load '" + dir + "/garbage.dxf'"},
        {{"toolpath", "profile", missing + ".json", carv, carv}, "cannot read params file"},
        {{"toolpath", "profile", dir + "/bad.json", carv, carv}, "bad.json: params: not valid JSON"},
        {{"toolpath", "profile", dir + "/typo.json", carv, carv}, "unknown key 'tabs.lenght'"},
        {{"toolpath", "profile", dir + "/dup.json", carv, carv}, "dup.json: params: duplicate key 'safe_z'"},
        {{"toolpath", "profile", dir + "/overflow.json", carv, carv}, "overflow.json: params: not valid JSON"},
        {{"toolpath", "profile", dir + "/notool.json", carv, carv},
         "the built-in starter tool database has no tool with id 'no-such-tool'"},
        {{"toolpath", "profile", dir + "/nolayer.json", carv, carv}, "has no layer named 'Nope'"},
        {{"toolpath", "profile", dir + "/nodb.json", carv, carv}, "cannot read tool database"},
        {{"toolpath", "profile", dir + "/huge.json", carv, carv}, "no profile toolpath was produced: "},
        {{"toolpath", "pocket", dir + "/emptypocket.json", carv, carv}, "no pocket toolpath was produced: "},
        {{"post", kGrbl, carv, dir + "/x.nc"}, "holds no toolpaths"},
        {{"post", missing + ".pp", carv, dir + "/x.nc"}, "cannot read post-processor"},
        {{"post", dir + "/bad.pp", carv, dir + "/x.nc"}, "post-processor '" + dir + "/bad.pp': line 2"},
    };
    for (const Case& c : cases) {
        const Outcome r = runCli(c.args);
        INFO(r.err);
        CHECK(r.code == cli::kExitFailure);
        CHECK_THAT(r.err, StartsWith("carvetto-cli: error: "));
        CHECK_THAT(r.err, ContainsSubstring(c.message));
    }
    // Nothing was written by the failing commands.
    CHECK_FALSE(fs::exists(dir + "/x.carv"));
    CHECK_FALSE(fs::exists(dir + "/x.nc"));
    const auto loaded = document::loadCarv(fs::path(carv));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().extras.memberNames().empty());
}

// --------------------------------------------------------------------------------- import

TEST_CASE("import maps DXF layers and contours into the document", "[cli][import]") {
    const std::string dir = workDir("import");
    const Outcome r = runCli({"import", kData + "/part.dxf", dir + "/part.carv"});
    REQUIRE(r.code == 0);
    CHECK(r.out == "imported 3 vector(s) on 2 layer(s) from '" + kData + "/part.dxf' into '" + dir + "/part.carv'\n");
    const auto loaded = document::loadCarv(fs::path(dir + "/part.carv"));
    REQUIRE(loaded.ok());
    const document::Document& doc = loaded.value().document;
    REQUIRE(doc.layers().size() == 2);
    CHECK(doc.layers()[0].name == "Outline");
    CHECK(doc.layers()[1].name == "Pockets");
    CHECK(doc.layers()[1].colour == document::Colour{255, 0, 0});
    CHECK(doc.objectsOnLayer(doc.layers()[0].id).size() == 1);
    CHECK(doc.objectsOnLayer(doc.layers()[1].id).size() == 2);
    CHECK(doc.checkIntegrity().empty());

    const Outcome info = runCli({"info", dir + "/part.carv"});
    REQUIRE(info.code == 0);
    CHECK_THAT(info.out, ContainsSubstring("'Outline': 1 object(s), 1 closed and 0 open vector(s)"));
    CHECK_THAT(info.out, ContainsSubstring("'Pockets': 2 object(s), 2 closed and 0 open vector(s)"));
    CHECK_THAT(info.out, ContainsSubstring("vectors extent: X 0 .. 120, Y 0 .. 80 mm"));
    CHECK_THAT(info.out, ContainsSubstring("toolpaths: 0"));
}

TEST_CASE("import warns about skipped text and DXF warnings but succeeds", "[cli][import]") {
    const std::string dir = workDir("import_text");
    const Outcome r = runCli({"import", std::string(CARVETTO_DXF_FIXTURE_DIR) + "/all_entities.dxf", dir + "/a.carv"});
    REQUIRE(r.code == 0);
    CHECK_THAT(r.err, ContainsSubstring("carvetto-cli: warning: "));
    CHECK_THAT(r.err, ContainsSubstring("text item(s) skipped"));
    CHECK(fs::exists(dir + "/a.carv"));
}

// ------------------------------------------------------------------------------- toolpath

TEST_CASE("toolpaths are appended as numbered members holding their params", "[cli][toolpath]") {
    const std::string dir = workDir("append");
    const std::string carv = importedFixture(dir);
    const std::string out1 = dir + "/one.carv";
    const Outcome a = runCli({"toolpath", "pocket", kData + "/pocket.json", carv, out1});
    REQUIRE(a.code == 0);
    CHECK_THAT(a.out, ContainsSubstring("added pocket toolpath 'Pockets'"));
    CHECK_THAT(a.out, ContainsSubstring("as toolpaths/0001.json"));
    const Outcome b = runCli({"toolpath", "profile", kData + "/profile.json", out1, out1});
    REQUIRE(b.code == 0);
    CHECK_THAT(b.out, ContainsSubstring("as toolpaths/0002.json"));

    const auto loaded = document::loadCarv(fs::path(out1));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().extras.memberNames() ==
          std::vector<std::string>{"toolpaths/0001.json", "toolpaths/0002.json"});
    const auto stored = cli::storedToolpaths(loaded.value().extras);
    REQUIRE(stored.size() == 2);
    CHECK(stored[0].operation == "pocket");
    CHECK(stored[0].toolpath.tool.name == "End Mill 6 mm");
    CHECK(stored[0].toolpath.tool.feedRate == 2300.0);  // hardwood/MDF cutting data
    CHECK_THAT(stored[0].paramsJson, ContainsSubstring("\"material\":\"material-hardwood-mdf\""));
    CHECK(stored[1].operation == "profile");
    CHECK(stored[1].toolpath.name == "Cut out");
    CHECK(*stored[1].toolpath.minZ() == -6.0);

    const Outcome info = runCli({"info", out1});
    REQUIRE(info.code == 0);
    CHECK_THAT(info.out, ContainsSubstring("toolpaths: 2"));
    CHECK_THAT(info.out, ContainsSubstring("toolpaths/0002.json: profile 'Cut out', tool T5 'End Mill 3 mm' D3 mm"));
    CHECK_THAT(info.out, ContainsSubstring("Z -6 .. 5 mm"));
    // The input file is untouched when writing elsewhere.
    CHECK(document::loadCarv(fs::path(carv)).value().extras.memberNames().empty());
}

TEST_CASE("without a database the built-in starter database is used", "[cli][toolpath]") {
    CHECK(std::string(cli::starterToolsJson()) == readFile(CARVETTO_STARTER_TOOLS));
    const std::string dir = workDir("starter");
    const std::string carv = importedFixture(dir);
    writeFile(dir + "/p.json",
              R"({"tool": {"id": "end-mill-6mm"}, "layers": ["Outline"], "passes": {"cut_depth": 3}})");
    const Outcome r = runCli({"toolpath", "profile", dir + "/p.json", carv, carv});
    INFO(r.err);
    REQUIRE(r.code == 0);
    const auto stored = cli::storedToolpaths(document::loadCarv(fs::path(carv)).value().extras);
    REQUIRE(stored.size() == 1);
    CHECK(stored[0].toolpath.tool.name == "End Mill 6 mm");
    CHECK(stored[0].toolpath.tool.feedRate == 2300.0);  // the tool's default cutting data
    CHECK(stored[0].toolpath.name == "Profile");
}

TEST_CASE("a tool without data for the machine/material falls back with a warning", "[cli][toolpath]") {
    const std::string dir = workDir("fallback");
    const std::string carv = importedFixture(dir);
    auto db = tooling::importToolDatabaseJson(readFile(kData + "/tools.json"));
    REQUIRE(db.ok);
    tooling::Tool tool = *db.database.findTool("end-mill-6mm");
    tool.cuttingData.clear();
    REQUIRE(db.database.updateTool(tool));
    writeFile(dir + "/db.json", tooling::exportToolDatabaseJson(db.database));
    writeFile(dir + "/p.json", R"({"tool": {"id": "end-mill-6mm", "database": "db.json",
        "machine": "machine-generic-router", "material": "material-softwood"},
        "layers": ["Outline"], "passes": {"cut_depth": 3}})");
    const Outcome r = runCli({"toolpath", "profile", dir + "/p.json", carv, carv});
    REQUIRE(r.code == 0);
    CHECK_THAT(r.err, ContainsSubstring("carvetto-cli: warning: tool 'end-mill-6mm' has no cutting data for machine"));

    writeFile(dir + "/q.json", R"({"tool": {"id": "end-mill-6mm", "database": "db.json",
        "machine": "machine-nope", "material": "material-softwood"}})");
    const Outcome q = runCli({"toolpath", "profile", dir + "/q.json", carv, carv});
    CHECK(q.code == cli::kExitFailure);
    CHECK_THAT(q.err, ContainsSubstring("has no machine with id 'machine-nope'"));
}

TEST_CASE("toolpath diagnostics are printed as warnings", "[cli][toolpath]") {
    const std::string dir = workDir("diag");
    const std::string carv = importedFixture(dir);
    // Sharp corner options with an end mill are ignored with a diagnostic (CornerNeedsVBit).
    writeFile(dir + "/p.json", R"({"tool": {"id": "end-mill-6mm"}, "layers": ["Outline"], "passes": {"cut_depth": 3},
        "corners": {"sharp_external": true}})");
    const Outcome r = runCli({"toolpath", "profile", dir + "/p.json", carv, carv});
    REQUIRE(r.code == 0);
    CHECK_THAT(r.err, StartsWith("carvetto-cli: warning: "));
}

// ----------------------------------------------------------------------------------- post

TEST_CASE("post names the program after the output file and refuses unwritable paths", "[cli][post]") {
    const std::string dir = workDir("post");
    pipeline(dir);
    const Outcome r = runCli({"post", kGrbl, dir + "/part.carv", dir + "/other_name.nc"});
    REQUIRE(r.code == 0);
    CHECK(r.out == "wrote 2 toolpath(s) to '" + dir + "/other_name.nc'\n");
    CHECK_THAT(readFile(dir + "/other_name.nc"), StartsWith("(other_name)\n"));

    const Outcome bad = runCli({"post", kGrbl, dir + "/part.carv", dir + "/no/such/dir/x.nc"});
    CHECK(bad.code == cli::kExitFailure);
    CHECK_THAT(bad.err, ContainsSubstring("cannot write"));
}

TEST_CASE("a damaged toolpath member is reported, not crashed on", "[cli][post]") {
    const std::string dir = workDir("damaged");
    const std::string carv = importedFixture(dir);
    auto loaded = document::loadCarv(fs::path(carv));
    REQUIRE(loaded.ok());
    const std::string bad = R"({"format":"carvetto-toolpath","format_version":1,"moves":[["warp",0,0,0]]})";
    const auto extras =
        loaded.value().extras.withMember("toolpaths/0001.json", std::vector<std::uint8_t>(bad.begin(), bad.end()));
    REQUIRE(document::saveCarv(loaded.value().document, fs::path(carv), extras).ok());
    const Outcome r = runCli({"post", kGrbl, carv, dir + "/x.nc"});
    CHECK(r.code == cli::kExitFailure);
    CHECK_THAT(r.err, ContainsSubstring("toolpath member 'toolpaths/0001.json': missing 'operation'"));
    const Outcome i = runCli({"info", carv});
    CHECK(i.code == cli::kExitFailure);
}

TEST_CASE("toolpath refuses to write a file the loader would reject", "[cli][toolpath][limits]") {
    // Fill the unknown-content budget (2,000,000 JSON values, ADR-0013 §9) almost completely with
    // a preserved member; the pocket's ~1,500 moves then push the file over it.
    const std::string dir = workDir("limit");
    const std::string carv = importedFixture(dir);
    auto loaded = document::loadCarv(fs::path(carv));
    REQUIRE(loaded.ok());
    std::string filler;
    // Distinct numbers, so the member does not trip the zip ratio check for JSON members.
    filler += "{\"a\":[";
    for (int i = 0; i < 999'000; ++i) {
        filler += (i == 0 ? "" : ",") + std::to_string(static_cast<long long>(i) * 7919 % 1'000'003);
    }
    filler += "],\"b\":[";
    for (int i = 0; i < 999'000; ++i) {
        filler += (i == 0 ? "" : ",") + std::to_string(static_cast<long long>(i) * 104'729 % 1'000'003);
    }
    filler += "]}";
    const auto extras =
        loaded.value().extras.withMember("filler.json", std::vector<std::uint8_t>(filler.begin(), filler.end()));
    REQUIRE(document::saveCarv(loaded.value().document, fs::path(carv), extras).ok());
    {
        const auto check = document::loadCarv(fs::path(carv));
        INFO((check.ok() ? std::string() : check.error().message));
        REQUIRE(check.ok());
    }

    const Outcome r = runCli({"toolpath", "pocket", kData + "/pocket.json", carv, dir + "/out.carv"});
    CHECK(r.code == cli::kExitFailure);
    CHECK_THAT(r.err, ContainsSubstring("the result would not load again"));
    CHECK_THAT(r.err, ContainsSubstring("interim toolpath storage"));
    CHECK_FALSE(fs::exists(dir + "/out.carv"));
    CHECK_FALSE(fs::exists(dir + "/out.carv.tmp"));
    CHECK(r.out.empty());  // T-0134: no "added ..." for a toolpath that was never written
}

TEST_CASE("T-0134: toolpath reports added toolpaths only after a successful write", "[cli][toolpath]") {
    const std::string dir = workDir("added");
    const std::string carv = importedFixture(dir);
    const Outcome bad = runCli({"toolpath", "pocket", kData + "/pocket.json", carv, dir + "/no/such/dir/out.carv"});
    CHECK(bad.code == cli::kExitFailure);
    CHECK_THAT(bad.err, ContainsSubstring("no/such/dir/out.carv"));
    CHECK(bad.out.empty());

    const Outcome good = runCli({"toolpath", "pocket", kData + "/pocket.json", carv, dir + "/out.carv"});
    REQUIRE(good.code == 0);
    CHECK_THAT(good.out, StartsWith("added pocket toolpath '"));
    CHECK_THAT(good.out, ContainsSubstring("as toolpaths/0001.json\nwrote '" + dir + "/out.carv'\n"));
}
