// T-0122: carvetto-cli commands, driven in-process through cli::run.

#include "cli.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carv_zip.hpp"
#include "starter_tools.hpp"

namespace fs = std::filesystem;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using namespace carvetto;

namespace {

const std::string kData = CARVETTO_CLI_DATA_DIR;
const std::string kGolden = std::string(CARVETTO_GOLDEN_DIR) + "/cli/part_grbl.nc";
const std::string kDrillGolden = std::string(CARVETTO_GOLDEN_DIR) + "/cli/part_drill_grbl.nc";
const std::string kEngraveGolden = std::string(CARVETTO_GOLDEN_DIR) + "/cli/part_engrave_grbl.nc";
const std::string kVCarvingGolden = std::string(CARVETTO_GOLDEN_DIR) + "/cli/sign_v_carve_grbl.nc";
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

/// The T-0150 drilling pipeline: DXF -> peck-drilled holes with dwell -> profile cut-out with
/// tabs -> Grbl; returns the G-code.
std::string drillPipeline(const std::string& dir) {
    const std::string carv = dir + "/part.carv";
    const std::string nc = dir + "/part.nc";
    for (const auto& args : std::vector<std::vector<std::string>>{
             {"import", kData + "/part.dxf", carv},
             {"toolpath", "drill", kData + "/drill.json", carv, carv},
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

/// The T-0151 quick engrave pipeline: DXF -> V-bit raster fill of the pockets layer in two passes
/// -> V-bit outline of the outline layer -> Grbl; returns the G-code.
std::string engravePipeline(const std::string& dir) {
    const std::string carv = dir + "/part.carv";
    const std::string nc = dir + "/part.nc";
    for (const auto& args : std::vector<std::vector<std::string>>{
             {"import", kData + "/part.dxf", carv},
             {"toolpath", "engrave", kData + "/engrave_fill.json", carv, carv},
             {"toolpath", "engrave", kData + "/engrave_outline.json", carv, carv},
             {"post", kGrbl, carv, nc},
         }) {
        const Outcome r = runCli(args);
        INFO(args[0] << ": " << r.err);
        REQUIRE(r.code == cli::kExitOk);
        CHECK(r.err.empty());
    }
    return readFile(nc);
}

std::string vCarvingPipeline(const std::string& dir) {
    const std::string carv = dir + "/sign.carv";
    const std::string nc = dir + "/sign.nc";
    for (const auto& args : std::vector<std::vector<std::string>>{
             {"import", kData + "/sign.dxf", carv},
             {"toolpath", "v_carve", kData + "/v_carve_letters.json", carv, carv},
             {"toolpath", "v_carve", kData + "/v_carve_flat.json", carv, carv},
             {"post", kGrbl, carv, nc},
         }) {
        const Outcome r = runCli(args);
        INFO(args[0] << ": " << r.err);
        REQUIRE(r.code == cli::kExitOk);
        CHECK(r.err.empty());
    }
    return readFile(nc);
}

/// Writes `path` again with `extra` members appended (raw, as the T-0122 CLI added its interim
/// toolpath members; `CarvExtras::withMember` refuses those names since T-0132).
void addRawMembers(const std::string& path, const std::vector<std::pair<std::string, std::string>>& extra) {
    const std::string bytes = readFile(path);
    auto zip = document::detail::readZip(std::span<const std::uint8_t>(
        reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size()));
    REQUIRE(zip.ok);
    for (const auto& [name, text] : extra) {
        zip.members.push_back({name, std::vector<std::uint8_t>(text.begin(), text.end())});
    }
    const auto out = document::detail::writeZip(zip.members);
    REQUIRE(out.ok);
    writeFile(path, std::string(out.bytes.begin(), out.bytes.end()));
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

// ------------------------------------------------------------------------------ T-0150 drill

TEST_CASE("T-0150: DXF -> drill + profile -> Grbl matches the drilling golden", "[cli][golden][drill]") {
    const std::string a = workDir("drill_e2e");
    const std::string b = workDir("drill_e2e_again");
    CHECK(drillPipeline(a) == readFile(kDrillGolden));
    CHECK(drillPipeline(b) == readFile(kDrillGolden));  // deterministic
    CHECK(readFile(a + "/part.carv") == readFile(b + "/part.carv"));
}

TEST_CASE("T-0150: the drilling golden is sane: drill then end mill, pecks, dwells, no canned cycles",
          "[cli][golden][drill]") {
    const std::string g = readFile(kDrillGolden);
    CHECK_THAT(g, StartsWith("(part)\n"));
    CHECK_THAT(g, ContainsSubstring("(Tool 28: Drill 3 mm)"));
    CHECK_THAT(g, ContainsSubstring("(Holes)"));
    CHECK_THAT(g, ContainsSubstring("(Change to tool 5: End Mill 3 mm)"));
    CHECK_THAT(g, ContainsSubstring("G0 X40.000 Y40.000 Z5.000"));  // slot centre (nearest first)
    CHECK_THAT(g, ContainsSubstring("G0 X90.000\n"));                // circle centre (Y modal)
    CHECK_THAT(g, ContainsSubstring("G1 Z-2.000 F300"));             // first peck at the plunge rate
    CHECK_THAT(g, ContainsSubstring("G0 Z1.000"));                   // peck retract: surface + gap
    CHECK_THAT(g, ContainsSubstring("G1 Z-7.000"));                  // hole bottom
    CHECK_THAT(g, ContainsSubstring("G4 P0.50"));
    CHECK(g.find("G8") == std::string::npos);  // no G81-G89 canned cycles
    CHECK(g.find("G73") == std::string::npos);
    CHECK_THAT(g, ContainsSubstring("M30"));
}

// Hidden (tag [.regen]): rewrites the drilling CLI golden (tools/regen_golden.py).
TEST_CASE("Regenerate the drilling CLI golden", "[.regen]") {
    writeFile(kDrillGolden, drillPipeline(workDir("regen_drill")));
}

TEST_CASE("T-0150: toolpath drill stores a drill record; info names it", "[cli][toolpath][drill]") {
    const std::string dir = workDir("drill_record");
    const std::string carv = importedFixture(dir);
    const Outcome r = runCli({"toolpath", "drill", kData + "/drill.json", carv, carv});
    INFO(r.err);
    REQUIRE(r.code == 0);
    CHECK(r.err.empty());
    CHECK_THAT(r.out, ContainsSubstring("added drill toolpath 'Holes' ("));
    CHECK_THAT(r.out, ContainsSubstring("tool 'Drill 3 mm') as toolpath 1"));
    auto loaded = document::loadCarv(fs::path(carv));
    REQUIRE(loaded.ok());
    const auto& records = loaded.value().document.toolpaths();
    REQUIRE(records.size() == 1);
    CHECK(records[0].operation() == document::ToolpathOperation::Drill);
    const auto& p = std::get<toolpath::DrillParams>(records[0].params);
    CHECK(p.name == "Holes");
    CHECK(p.cutDepth == 7.0);
    CHECK(p.peck);
    CHECK(p.dwellTime == 0.5);
    CHECK(records[0].toolSource.toolId == "drill-3mm");
    CHECK(records[0].tool.plungeRate == 300.0);
    CHECK(records[0].selection.layers.size() == 1);
    const Outcome info = runCli({"info", carv});
    CHECK_THAT(info.out, ContainsSubstring("  1: drill 'Holes', tool T28 'Drill 3 mm' D3 mm, "));
}

TEST_CASE("T-0150: toolpath drill reports diagnostics and refuses unusable tools", "[cli][toolpath][drill][errors]") {
    const std::string dir = workDir("drill_errors");
    const std::string carv = importedFixture(dir);
    // A V-bit from the starter database: refused, nothing written.
    writeFile(dir + "/vbit.json", R"({"tool": {"id": "v-bit-90-1-4in"}, "cut_depth": 2})");
    const Outcome v = runCli({"toolpath", "drill", dir + "/vbit.json", carv, dir + "/out.carv"});
    CHECK(v.code == cli::kExitFailure);
    CHECK_THAT(v.err, ContainsSubstring("no drill toolpath was produced: tool 'V-Bit 90"));
    CHECK_THAT(v.err, ContainsSubstring("drilling needs a drill or an end mill"));
    CHECK_FALSE(fs::exists(dir + "/out.carv"));
    // Without layers every closed vector gets a hole (the outline's centre too); the end mill fits.
    writeFile(dir + "/all.json", R"({"tool": {"id": "end-mill-6mm"}, "cut_depth": 2, "order": {"selection_order": true}})");
    const Outcome a = runCli({"toolpath", "drill", dir + "/all.json", carv, carv});
    INFO(a.err);
    REQUIRE(a.code == 0);
    CHECK_THAT(a.out, ContainsSubstring("(9 moves, tool 'End Mill 6 mm')"));
    // Unknown keys are refused, naming the file and the key.
    writeFile(dir + "/typo.json", R"({"tool": {"id": "drill-6mm"}, "cut_depht": 2})");
    const Outcome t = runCli({"toolpath", "drill", dir + "/typo.json", carv, carv});
    CHECK(t.code == cli::kExitFailure);
    CHECK_THAT(t.err, ContainsSubstring("typo.json: params: unknown key 'cut_depht'"));
}

// ---------------------------------------------------------------------------- T-0151 engrave

TEST_CASE("T-0151: DXF -> engrave fill + outline -> Grbl matches the engraving golden", "[cli][golden][engrave]") {
    const std::string a = workDir("engrave_e2e");
    const std::string b = workDir("engrave_e2e_again");
    CHECK(engravePipeline(a) == readFile(kEngraveGolden));
    CHECK(engravePipeline(b) == readFile(kEngraveGolden));  // deterministic
    CHECK(readFile(a + "/part.carv") == readFile(b + "/part.carv"));
}

TEST_CASE("T-0151: the engraving golden is sane: one V-bit, two fill passes, outline at depth",
          "[cli][golden][engrave]") {
    const std::string g = readFile(kEngraveGolden);
    CHECK_THAT(g, StartsWith("(part)\n"));
    CHECK_THAT(g, ContainsSubstring("(Tool 12: V-Bit 90)"));
    CHECK_THAT(g, ContainsSubstring("(Fill)"));
    CHECK_THAT(g, ContainsSubstring("(Border)"));
    CHECK(g.find("Change to tool") == std::string::npos);  // the same tool throughout
    CHECK_THAT(g, ContainsSubstring("M3 S18000"));
    CHECK_THAT(g, ContainsSubstring("G1 Z-0.500 F500"));  // first fill pass / the outline depth
    CHECK_THAT(g, ContainsSubstring("G1 Z-1.000"));       // second fill pass
    CHECK(g.find("Z-1.001") == std::string::npos);
    CHECK_THAT(g, ContainsSubstring("G0 X0.000 Y0.000 Z5.000"));  // the outline starts at its first corner
    CHECK_THAT(g, ContainsSubstring("M30"));
}

// Hidden (tag [.regen]): rewrites the engraving CLI golden (tools/regen_golden.py).
TEST_CASE("Regenerate the engraving CLI golden", "[.regen]") {
    writeFile(kEngraveGolden, engravePipeline(workDir("regen_engrave")));
}

TEST_CASE("T-0151: toolpath engrave stores an engrave record; info names it", "[cli][toolpath][engrave]") {
    const std::string dir = workDir("engrave_record");
    const std::string carv = importedFixture(dir);
    const Outcome r = runCli({"toolpath", "engrave", kData + "/engrave_fill.json", carv, carv});
    INFO(r.err);
    REQUIRE(r.code == 0);
    CHECK(r.err.empty());
    CHECK_THAT(r.out, ContainsSubstring("added engrave toolpath 'Fill' ("));
    CHECK_THAT(r.out, ContainsSubstring("tool 'V-Bit 90') as toolpath 1"));
    auto loaded = document::loadCarv(fs::path(carv));
    REQUIRE(loaded.ok());
    const auto& records = loaded.value().document.toolpaths();
    REQUIRE(records.size() == 1);
    CHECK(records[0].operation() == document::ToolpathOperation::Engrave);
    const auto& p = std::get<toolpath::EngraveParams>(records[0].params);
    CHECK(p.name == "Fill");
    CHECK(p.depth == 1.0);
    CHECK(p.passes == 2);
    CHECK(p.strategy == toolpath::EngraveStrategy::Fill);
    CHECK(p.fillPattern == toolpath::PocketStrategy::Raster);
    CHECK(p.stepover == 0.8);
    CHECK(records[0].toolSource.toolId == "v-bit-90");
    CHECK(records[0].tool.feedRate == 1500.0);
    CHECK(records[0].selection.layers.size() == 1);
    const Outcome info = runCli({"info", carv});
    CHECK_THAT(info.out, ContainsSubstring("  1: engrave 'Fill', tool T12 'V-Bit 90' D12.7 mm, "));
}

TEST_CASE("T-0151: toolpath engrave reports diagnostics and refuses unusable tools",
          "[cli][toolpath][engrave][errors]") {
    const std::string dir = workDir("engrave_errors");
    const std::string carv = importedFixture(dir);
    // A drill: refused, nothing written.
    writeFile(dir + "/drill.json", R"({"tool": {"id": "drill-3mm", "database": ")" + kData +
                                       R"(/tools.json"}, "depth": 0.5})");
    const Outcome d = runCli({"toolpath", "engrave", dir + "/drill.json", carv, dir + "/out.carv"});
    CHECK(d.code == cli::kExitFailure);
    CHECK_THAT(d.err, ContainsSubstring("no engrave toolpath was produced: tool 'Drill 3 mm' is a drill"));
    CHECK_FALSE(fs::exists(dir + "/out.carv"));
    // Fill with the V-bit's cutting-data stepover (2.54 mm) at 0.5 mm deep (1 mm wide): refused.
    writeFile(dir + "/wide.json", R"({"tool": {"id": "v-bit-90", "database": ")" + kData +
                                      R"(/tools.json"}, "depth": 0.5, "strategy": "fill"})");
    const Outcome w = runCli({"toolpath", "engrave", dir + "/wide.json", carv, dir + "/out.carv"});
    CHECK(w.code == cli::kExitFailure);
    CHECK_THAT(w.err, ContainsSubstring("the stepover 2.54 mm is wider than the"));
    CHECK_FALSE(fs::exists(dir + "/out.carv"));
    // Unknown keys are refused, naming the file and the key.
    writeFile(dir + "/typo.json", R"({"tool": {"id": "v-bit-90"}, "dpeth": 2})");
    const Outcome t = runCli({"toolpath", "engrave", dir + "/typo.json", carv, carv});
    CHECK(t.code == cli::kExitFailure);
    CHECK_THAT(t.err, ContainsSubstring("typo.json: params: unknown key 'dpeth'"));
}

// ------------------------------------------------------------------------------------ help

TEST_CASE("AC2: --help documents every command", "[cli][help]") {
    const Outcome r = runCli({"--help"});
    CHECK(r.code == 0);
    CHECK(r.out == cli::helpText());
    CHECK(r.err.empty());
    for (const char* usage : {"import <in.dxf> <out.carv>", "toolpath profile <params.json> <in.carv> <out.carv>",
                              "toolpath pocket <params.json> <in.carv> <out.carv>",
                              "toolpath drill <params.json> <in.carv> <out.carv>",
                              "toolpath engrave <params.json> <in.carv> <out.carv>", "post <post.pp> <in.carv> <out.nc>",
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
        {{"toolpath", "fluting", "p.json", "a.carv", "b.carv"},
         "unknown toolpath operation 'fluting' (expected profile, pocket, drill, engrave or v_carve)"},
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
    writeFile(dir + "/negallow.json",
              R"({"tool": {"id": "end-mill-6mm"}, "side": "outside", "passes": {"cut_depth": 1}, "allowance": -4})");
    writeFile(dir + "/emptypocket.json", R"({"tool": {"id": "end-mill-6mm"}, "layers": ["Pockets"]})");
    writeFile(dir + "/bad.pp", "begin HEADER\n\"[NO_SUCH_TAG]\"\nend\n");
    writeFile(dir + "/dup.json", R"({"tool": {"id": "end-mill-6mm"}, "safe_z": 5, "safe_z": 50})");
    writeFile(dir + "/overflow.json", R"({"tool": {"id": "end-mill-6mm"}, "safe_z": 1e400})");
    writeFile(dir + "/deep.json", R"({"tool": {"id": "end-mill-6mm"}, "zz": )" + std::string(1'000'000, '[') +
                                      std::string(1'000'000, ']') + "}");

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
        {{"toolpath", "profile", dir + "/deep.json", carv, carv}, "deep.json: params: objects and arrays are nested"},
        {{"toolpath", "profile", dir + "/notool.json", carv, carv},
         "the built-in starter tool database has no tool with id 'no-such-tool'"},
        {{"toolpath", "profile", dir + "/nolayer.json", carv, carv}, "has no layer named 'Nope'"},
        {{"toolpath", "profile", dir + "/nodb.json", carv, carv}, "cannot read tool database"},
        {{"toolpath", "profile", dir + "/huge.json", carv, carv}, "no profile toolpath was produced: "},
        {{"toolpath", "profile", dir + "/negallow.json", carv, carv},
         "no profile toolpath was produced: allowance -4 mm is below minus the tool radius (-3 mm at the cut depth)"},
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

TEST_CASE("T-0141: an invalid tolerance is refused with the reason, profile and pocket alike", "[cli][errors]") {
    const std::string dir = workDir("badtol");
    const std::string carv = importedFixture(dir);
    writeFile(dir + "/profile.json", R"({"tool": {"id": "end-mill-6mm"}, "passes": {"cut_depth": 1},
                                         "tolerance": {"distance": 0.00001, "chordal": 0.00001}})");
    writeFile(dir + "/pocket.json", R"({"tool": {"id": "end-mill-6mm"}, "layers": ["Pockets"], "cut_depth": 1,
                                        "tolerance": {"distance": 0.00001, "chordal": 0.00001}})");
    const std::string reason = "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal";
    for (const std::string op : {"profile", "pocket"}) {
        const Outcome r = runCli({"toolpath", op, dir + "/" + op + ".json", carv, dir + "/out.carv"});
        INFO(r.err);
        CHECK(r.code == cli::kExitFailure);
        CHECK_THAT(r.err, StartsWith("carvetto-cli: error: no " + op + " toolpath was produced: "));
        CHECK_THAT(r.err, ContainsSubstring(reason));
    }
    CHECK_FALSE(fs::exists(dir + "/out.carv"));
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

TEST_CASE("toolpaths are appended as native toolpath records holding their params", "[cli][toolpath]") {
    const std::string dir = workDir("append");
    const std::string carv = importedFixture(dir);
    const std::string out1 = dir + "/one.carv";
    const Outcome a = runCli({"toolpath", "pocket", kData + "/pocket.json", carv, out1});
    REQUIRE(a.code == 0);
    CHECK_THAT(a.out, ContainsSubstring("added pocket toolpath 'Pockets'"));
    CHECK_THAT(a.out, ContainsSubstring("as toolpath 1\n"));
    const Outcome b = runCli({"toolpath", "profile", kData + "/profile.json", out1, out1});
    REQUIRE(b.code == 0);
    CHECK_THAT(b.out, ContainsSubstring("as toolpath 2\n"));

    const auto loaded = document::loadCarv(fs::path(out1));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().extras.memberNames().empty());  // no interim members any more (T-0132)
    const document::Document& doc = loaded.value().document;
    const auto& records = doc.toolpaths();
    REQUIRE(records.size() == 2);
    CHECK(records[0].operation() == document::ToolpathOperation::Pocket);
    CHECK(records[0].tool.name == "End Mill 6 mm");
    CHECK(records[0].tool.feedRate == 2300.0);  // hardwood/MDF cutting data
    CHECK(records[0].toolSource.toolId == "end-mill-6mm");
    CHECK(records[0].toolSource.material == "material-hardwood-mdf");
    CHECK(std::get<toolpath::PocketParams>(records[0].params).name == "Pockets");
    CHECK(records[1].operation() == document::ToolpathOperation::Profile);
    CHECK(records[1].name == "Cut out");
    CHECK(std::get<toolpath::ProfileParams>(records[1].params).tabs.enabled);
    CHECK_FALSE(std::get<toolpath::ProfileParams>(records[1].params).cuttingData.has_value());
    CHECK(*records[1].toolpath().minZ() == -6.0);
    // The selection names the layers the params file asked for, by id.
    for (const auto& r : records) {
        CHECK_FALSE(r.selection.all);
        REQUIRE(r.selection.layers.size() == 1);
        CHECK(doc.findLayer(r.selection.layers[0]) != nullptr);
    }
    CHECK(doc.findLayer(records[1].selection.layers[0])->name == "Outline");

    const Outcome info = runCli({"info", out1});
    REQUIRE(info.code == 0);
    CHECK_THAT(info.out, ContainsSubstring("toolpaths: 2"));
    CHECK_THAT(info.out, ContainsSubstring("  2: profile 'Cut out', tool T5 'End Mill 3 mm' D3 mm"));
    CHECK_THAT(info.out, ContainsSubstring("Z -6 .. 5 mm"));
    // The input file is untouched when writing elsewhere.
    CHECK(document::loadCarv(fs::path(carv)).value().document.toolpaths().empty());
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
    const auto records = document::loadCarv(fs::path(carv)).value().document.toolpaths();
    REQUIRE(records.size() == 1);
    CHECK(records[0].tool.name == "End Mill 6 mm");
    CHECK(records[0].tool.feedRate == 2300.0);  // the tool's default cutting data
    CHECK(records[0].name == "Profile");
    CHECK(records[0].toolSource.machine.empty());
}

TEST_CASE("without layers in the params file every vector is selected", "[cli][toolpath]") {
    const std::string dir = workDir("all_layers");
    const std::string carv = importedFixture(dir);
    writeFile(dir + "/p.json", R"({"tool": {"id": "end-mill-6mm"}, "passes": {"cut_depth": 3}})");
    const Outcome r = runCli({"toolpath", "profile", dir + "/p.json", carv, carv});
    INFO(r.err);
    REQUIRE(r.code == 0);
    const auto records = document::loadCarv(fs::path(carv)).value().document.toolpaths();
    REQUIRE(records.size() == 1);
    CHECK(records[0].selection == document::VectorSelection::allVectors());
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

TEST_CASE("a file with T-0122 interim toolpath members is migrated and posts the same program",
          "[cli][post][migration]") {
    // Before T-0132 the CLI stored each toolpath as an archive member toolpaths/NNNN.json
    // (ADR-0019). Rebuild such a file from the native one and post it: same G-code.
    const std::string dir = workDir("migrate");
    pipeline(dir);
    const auto native = document::loadCarv(fs::path(dir + "/part.carv"));
    REQUIRE(native.ok());
    document::Document legacyDoc;
    {
        // Same layers and vectors; the toolpaths go in as interim members.
        auto copy = document::loadCarv(fs::path(dir + "/part.carv"));
        REQUIRE(copy.ok());
        legacyDoc = std::move(copy.value().document);
    }
    std::vector<std::pair<std::string, std::string>> interim;
    int n = 0;
    for (const auto& r : native.value().document.toolpaths()) {
        legacyDoc.takeToolpath(r.id);
        const auto num = [](double v) {
            std::ostringstream ss;
            ss.precision(17);
            ss << v;
            return ss.str();
        };
        std::string moves;
        for (const toolpath::Move& m : *r.moves) {
            static const char* kinds[] = {"rapid", "feed", "plunge", "retract", "arc_cw", "arc_ccw", "dwell"};
            moves += moves.empty() ? "[" : ",[";
            moves += "\"" + std::string(kinds[static_cast<int>(m.kind)]) + "\"";
            for (double v : {m.to.x, m.to.y, m.to.z}) {
                moves += "," + num(v);
            }
            if (toolpath::isArc(m.kind)) {
                moves += "," + num(m.centre.x) + "," + num(m.centre.y);
            }
            if (m.feedOverride && toolpath::isCutting(m.kind)) {
                moves += ",{\"feed\":" + num(*m.feedOverride) + "}";
            }
            moves += "]";
        }
        const bool pocket = r.operation() == document::ToolpathOperation::Pocket;
        const std::string params = readFile(kData + (pocket ? "/pocket.json" : "/profile.json"));
        const std::string text = R"({"format":"carvetto-toolpath","format_version":1,"operation":")" +
                                 std::string(pocket ? "pocket" : "profile") + R"(","name":")" + r.name +
                                 R"(","params":)" + params + R"(,"safe_z":)" + num(r.safeZ()) +
                                 R"(,"tool":{"name":")" + r.tool.name + R"(","tool_number":)" +
                                 std::to_string(r.tool.toolNumber) + R"(,"diameter":)" + num(r.tool.diameter) +
                                 R"(,"feed_rate":)" + num(r.tool.feedRate) + R"(,"plunge_rate":)" +
                                 num(r.tool.plungeRate) + R"(,"spindle_speed":)" + num(r.tool.spindleSpeed) +
                                 R"(},"moves":[)" + moves + "]}";
        interim.emplace_back("toolpaths/000" + std::to_string(++n) + ".json", text);
    }
    REQUIRE(legacyDoc.toolpaths().empty());
    const std::string legacy = dir + "/legacy.carv";
    REQUIRE(document::saveCarv(legacyDoc, fs::path(legacy)).ok());
    addRawMembers(legacy, interim);

    const Outcome r = runCli({"post", kGrbl, legacy, dir + "/part.nc"});
    INFO(r.err);
    REQUIRE(r.code == 0);
    CHECK(readFile(dir + "/part.nc") == readFile(kGolden));
    const auto migrated = document::loadCarv(fs::path(legacy));
    REQUIRE(migrated.ok());
    CHECK(migrated.value().extras.memberNames().empty());
    REQUIRE(migrated.value().document.toolpaths().size() == 2);
    for (std::size_t i = 0; i < 2; ++i) {
        const auto& a = migrated.value().document.toolpaths()[i];
        const auto& b = native.value().document.toolpaths()[i];
        CHECK(a.name == b.name);
        CHECK(a.params == b.params);
        CHECK(a.tool == b.tool);
        CHECK(a.toolSource == b.toolSource);
        CHECK(*a.moves == *b.moves);
        CHECK(a.selection == b.selection);
    }
}

TEST_CASE("a damaged interim toolpath member is reported, not crashed on", "[cli][post]") {
    const std::string dir = workDir("damaged");
    const std::string carv = importedFixture(dir);
    addRawMembers(carv, {{"toolpaths/0001.json",
                          R"({"format":"carvetto-toolpath","format_version":1,"moves":[["warp",0,0,0]]})"}});
    const Outcome r = runCli({"post", kGrbl, carv, dir + "/x.nc"});
    CHECK(r.code == cli::kExitFailure);
    CHECK_THAT(r.err, ContainsSubstring("toolpaths/0001.json"));
    CHECK_THAT(r.err, ContainsSubstring("unknown move kind 'warp'"));
    const Outcome i = runCli({"info", carv});
    CHECK(i.code == cli::kExitFailure);
}

TEST_CASE("toolpaths no longer count against the loader's budget for unknown content", "[cli][toolpath][limits]") {
    // Before T-0132 the interim JSON toolpath members counted against the 2,000,000-value budget
    // for unknown content (ADR-0013 §9), and this case was refused. With native records the moves
    // are in a binary cache with its own budget, so the same file is written and loads.
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

    const Outcome r = runCli({"toolpath", "pocket", kData + "/pocket.json", carv, dir + "/out.carv"});
    INFO(r.err);
    CHECK(r.code == cli::kExitOk);
    const auto out = document::loadCarv(fs::path(dir + "/out.carv"));
    INFO((out.ok() ? std::string() : out.error().message));
    REQUIRE(out.ok());
    CHECK(out.value().document.toolpaths().size() == 1);
    CHECK(out.value().extras.memberNames() == std::vector<std::string>{"filler.json"});
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
    CHECK_THAT(good.out, ContainsSubstring("as toolpath 1\nwrote '" + dir + "/out.carv'\n"));
}

// ---------------------------------------------------------------------------- T-0153 v_carve

TEST_CASE("T-0153: DXF -> V-carve letters + flat panel -> Grbl matches the V-carve golden", "[cli][golden][v_carve]") {
    const std::string a = workDir("v_carve_e2e");
    const std::string b = workDir("v_carve_e2e_again");
    CHECK(vCarvingPipeline(a) == readFile(kVCarvingGolden));
    CHECK(vCarvingPipeline(b) == readFile(kVCarvingGolden));  // deterministic
    CHECK(readFile(a + "/sign.carv") == readFile(b + "/sign.carv"));
}

TEST_CASE("T-0153: the V-carve golden is sane: V-bit letters then end mill clearance then V-bit walls",
          "[cli][golden][v_carve]") {
    const std::string g = readFile(kVCarvingGolden);
    CHECK_THAT(g, StartsWith("(sign)\n"));
    CHECK_THAT(g, ContainsSubstring("(Tool 12: V-Bit 90)"));
    CHECK_THAT(g, ContainsSubstring("(Letters)"));
    CHECK_THAT(g, ContainsSubstring("(Change to tool 5: End Mill 3 mm)"));
    CHECK_THAT(g, ContainsSubstring("(Panel [End Mill 3 mm])"));
    CHECK_THAT(g, ContainsSubstring("(Panel [V-Bit 90])"));
    CHECK(g.find("(Panel [End Mill 3 mm])") < g.find("(Panel [V-Bit 90])"));  // clearance first
    CHECK_THAT(g, ContainsSubstring("G1 Z-1.500"));  // the flat depth
    CHECK(g.find("Z-1.501") == std::string::npos);
    CHECK(g.find("Z-3.") == std::string::npos);  // the letters' strokes are 4 mm wide: under 2 mm deep
    CHECK_THAT(g, ContainsSubstring("M30"));
}

// Hidden (tag [.regen]): rewrites the V-carve CLI golden (tools/regen_golden.py).
TEST_CASE("Regenerate the V-carve CLI golden", "[.regen]") {
    writeFile(kVCarvingGolden, vCarvingPipeline(workDir("regen_v_carve")));
}

TEST_CASE("T-0153: toolpath v_carve stores one record per tool; info names them", "[cli][toolpath][v_carve]") {
    const std::string dir = workDir("v_carve_record");
    const std::string carv = dir + "/sign.carv";
    REQUIRE(runCli({"import", kData + "/sign.dxf", carv}).code == 0);
    const Outcome r = runCli({"toolpath", "v_carve", kData + "/v_carve_flat.json", carv, carv});
    INFO(r.err);
    REQUIRE(r.code == 0);
    CHECK(r.err.empty());
    CHECK_THAT(r.out, ContainsSubstring("added v_carve toolpath 'Panel [End Mill 3 mm]' ("));
    CHECK_THAT(r.out, ContainsSubstring("tool 'End Mill 3 mm') as toolpath 1"));
    CHECK_THAT(r.out, ContainsSubstring("added v_carve toolpath 'Panel [V-Bit 90]' ("));
    CHECK_THAT(r.out, ContainsSubstring("tool 'V-Bit 90') as toolpath 2"));
    auto loaded = document::loadCarv(fs::path(carv));
    REQUIRE(loaded.ok());
    const auto& records = loaded.value().document.toolpaths();
    REQUIRE(records.size() == 2);
    for (const auto& rec : records) {
        CHECK(rec.operation() == document::ToolpathOperation::VCarving);
        const auto& p = std::get<toolpath::VCarvingParams>(rec.params);
        CHECK(p.name == "Panel");
        CHECK(p.flatDepth == 1.5);
        CHECK(p.clearanceStrategy == toolpath::PocketStrategy::Raster);
        CHECK(std::fabs(p.clearanceRasterAngle - 0.7853981633974483) < 1e-15);
        CHECK(p.tolerance.chordal == 0.05);
        CHECK(rec.selection.layers.size() == 1);
    }
    CHECK(records[0].toolSource.toolId == "end-mill-3mm");
    CHECK(records[1].toolSource.toolId == "v-bit-90");
    CHECK(records[1].tool.feedRate == 1500.0);
    const Outcome info = runCli({"info", carv});
    CHECK_THAT(info.out, ContainsSubstring("  1: v_carve 'Panel [End Mill 3 mm]', tool T5 'End Mill 3 mm' D3 mm, "));
    CHECK_THAT(info.out, ContainsSubstring("  2: v_carve 'Panel [V-Bit 90]', tool T12 'V-Bit 90' D12.7 mm, "));
}

TEST_CASE("T-0153: toolpath v_carve reports diagnostics and refuses unusable tools", "[cli][toolpath][v_carve][errors]") {
    const std::string dir = workDir("v_carve_errors");
    const std::string carv = dir + "/sign.carv";
    REQUIRE(runCli({"import", kData + "/sign.dxf", carv}).code == 0);
    // An end mill as the carving tool: refused, nothing written.
    writeFile(dir + "/em.json", R"({"tool": {"id": "end-mill-3mm", "database": ")" + kData + R"(/tools.json"}})");
    const Outcome e = runCli({"toolpath", "v_carve", dir + "/em.json", carv, dir + "/out.carv"});
    CHECK(e.code == cli::kExitFailure);
    CHECK_THAT(e.err, ContainsSubstring("no v_carve toolpath was produced: tool 'End Mill 3 mm' is an end mill"));
    CHECK_FALSE(fs::exists(dir + "/out.carv"));
    // A V-bit as a clearance tool: refused.
    writeFile(dir + "/vclear.json", R"({"tool": {"id": "v-bit-90", "database": ")" + kData +
                                        R"(/tools.json"}, "flat_depth": 1, "clearance_tools": [{"id": "v-bit-90", "database": ")" +
                                        kData + R"(/tools.json"}]})");
    const Outcome v = runCli({"toolpath", "v_carve", dir + "/vclear.json", carv, dir + "/out.carv"});
    CHECK(v.code == cli::kExitFailure);
    CHECK_THAT(v.err, ContainsSubstring("clearance tools must be end mills"));
    // A clearance tool missing from the database: an error naming it.
    writeFile(dir + "/missing.json", R"({"tool": {"id": "v-bit-90", "database": ")" + kData +
                                         R"(/tools.json"}, "clearance_tools": [{"id": "nope", "database": ")" + kData +
                                         R"(/tools.json"}]})");
    const Outcome m = runCli({"toolpath", "v_carve", dir + "/missing.json", carv, dir + "/out.carv"});
    CHECK(m.code == cli::kExitFailure);
    CHECK_THAT(m.err, ContainsSubstring("has no tool with id 'nope'"));
    // Warnings are printed and the toolpath is still written (the panel is wider than the bit
    // can carve without a flat depth).
    writeFile(dir + "/wide.json", R"({"tool": {"id": "v-bit-60-1-4in"}, "layers": ["Panel"], "tolerance": {"chordal": 0.05}})");
    const Outcome w = runCli({"toolpath", "v_carve", dir + "/wide.json", carv, carv});
    INFO(w.err);
    CHECK(w.code == 0);
    CHECK_THAT(w.err, ContainsSubstring("warning: 1 region(s) are wider than tool"));
}
