// T-0151 AC1: quick engrave toolpaths as native toolpath records (ADR-0021 schema, ADR-0024): the
// EngraveParams variant alternative, its job.json params (snake_case keys like carvetto-cli's
// params.json), exact round trips, and refusal of damaged params.

#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/engrave.hpp"

using namespace carvetto;
using namespace carvetto::document;
using Bytes = std::vector<std::uint8_t>;

namespace {

Bytes save(const Document& doc, const CarvExtras& extras = {}) {
    auto r = saveCarv(doc, extras);
    INFO((r ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return r.value();
}

LoadedCarv load(const Bytes& bytes) {
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    INFO((r ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return std::move(r.value());
}

CarvError loadError(const Bytes& bytes) {
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    REQUIRE_FALSE(r.ok());
    return r.error();
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

std::string jobText(const Bytes& archive) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (const auto& m : r.members) {
        if (m.name == "job.json") {
            return {m.data.begin(), m.data.end()};
        }
    }
    FAIL("no job.json");
    return {};
}

Bytes withJob(const Bytes& archive, const std::string& job) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (auto& m : r.members) {
        if (m.name == "job.json") {
            m.data.assign(job.begin(), job.end());
        }
    }
    auto w = detail::writeZip(r.members);
    REQUIRE(w.ok);
    return w.bytes;
}

std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const std::size_t pos = text.find(from);
    INFO("looking for: " << from);
    REQUIRE(pos != std::string::npos);
    text.replace(pos, from.size(), to);
    return text;
}

/// Engrave parameters with every value away from its default, with awkward doubles.
toolpath::EngraveParams fullEngrave() {
    toolpath::EngraveParams p;
    p.name = "Sign \"A\" \xC3\xA9";
    p.depth = 0.30000000000000004;
    p.passes = 3;
    p.safeZ = 12.000000000000002;
    p.strategy = toolpath::EngraveStrategy::Fill;
    p.fillPattern = toolpath::PocketStrategy::Raster;
    p.stepover = 0.1;
    p.rasterAngle = 0.7853981633974483;
    p.startPosition = {-10.5, -0.0};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool vBit() {
    tooling::Tool t;
    t.id = "v-bit-90";
    t.name = "V-bit 90";
    t.type = tooling::ToolType::VBit;
    t.diameter = 6.35;
    t.includedAngle = 1.5707963267948966;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    return t;
}

tooling::Tool diamondDrag() {
    tooling::Tool t;
    t.id = "drag-90";
    t.name = "Diamond drag 90";
    t.type = tooling::ToolType::DiamondDrag;
    t.diameter = 0.3;
    t.defaultCuttingData = {0.1, 0.1, 0.0, 0.0, 0.0, 1500.0, 400.0, 0.0, 7};
    return t;
}

/// A document with two squares and a computed engrave record.
Document engraveDocument(const toolpath::EngraveParams& params, const tooling::Tool& tool = vBit()) {
    Document doc;
    const ObjectId a = doc.insertObject(Object::vector(geometry::Contour::rectangle({{0, 0}, {10, 10}}),
                                                       doc.defaultLayer(), doc.activeSheet()));
    doc.insertObject(
        Object::vector(geometry::Contour::rectangle({{20, 0}, {30, 10}}), doc.defaultLayer(), doc.activeSheet()));
    std::vector<geometry::Contour> contours;
    for (const ObjectId id : doc.selectedVectors(VectorSelection::allVectors())) {
        const auto& cs = doc.findObject(id)->asVector()->contours;
        contours.insert(contours.end(), cs.begin(), cs.end());
    }
    toolpath::EngraveResult r = toolpath::engraveToolpath(contours, tool, std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.selection.layers = {doc.defaultLayer()};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"v-bit-90", "machine-generic-router", "material-hardwood-mdf"};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

#include <fstream>
#include <sstream>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/pocket.hpp"
namespace {
std::string postGrbl(const Document& d) {
    std::ifstream in(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp"); std::stringstream ss; ss << in.rdbuf();
    auto pp = post::PostProcessor::parse(ss.str()); REQUIRE(pp.ok());
    std::vector<toolpath::Toolpath> tps; for (auto& r : d.toolpaths()) tps.push_back(r.toolpath());
    auto r = pp.value().run(tps, post::PostContext{});
    return r.ok() ? std::string("OK\n") + r.value().text : std::string("ERR ") + r.error().describe();
}
ToolpathRecord baseRec(ToolpathParams params) {
    ToolpathRecord rec; rec.name = "R"; rec.params = params; rec.selection = VectorSelection::allVectors();
    rec.tool.name = "Mill"; rec.tool.toolNumber = 1; rec.tool.diameter = 3.0; rec.tool.feedRate = 1000.0;
    rec.tool.plungeRate = 300.0; rec.tool.spindleSpeed = 12000.0;
    rec.moves = shareMoves({toolpath::Move::rapid({0, 0, 5}), toolpath::Move::plunge({0, 0, -1}), toolpath::Move::retract({0,0,5})});
    return rec;
}
}
#include "carvetto/toolpath/profile.hpp"
namespace {
Bytes dragDocBytes(std::optional<toolpath::EngraveStrategy> s = std::nullopt) {
    toolpath::EngraveParams params = fullEngrave(); params.depth = 0.1; params.passes = 1;
    if (s) params.strategy = *s;
    return save(engraveDocument(params, diamondDrag()));
}
}
TEST_CASE("V2: drag engrave record has spindle_off + tool_type in job.json and posts") {
    const Bytes b = dragDocBytes();
    const std::string job = jobText(b);
    CHECK(contains(job, "\"spindle_off\": true"));
    CHECK(contains(job, "\"tool_type\": \"diamond_drag\""));
    auto l = load(b);
    CHECK(l.document.toolpaths()[0].tool.spindleOff);
    CHECK(l.document.toolpaths()[0].tool.spindleSpeed == 0.0);
    const std::string out = postGrbl(l.document);
    INFO(out);
    CHECK(out.rfind("OK", 0) == 0);
    CHECK(contains(out, "unlock"));
    CHECK_FALSE(contains(out, "M3"));
    // fill variant
    auto f = load(dragDocBytes(toolpath::EngraveStrategy::Fill));
    CHECK(f.document.toolpaths()[0].tool.spindleOff);
    CHECK(f.document.toolpaths()[0].tool.toolType == "diamond_drag");
    CHECK(postGrbl(f.document).rfind("OK", 0) == 0);
}
TEST_CASE("V2: crafted tool_type values are refused by the loader") {
    const Bytes b = dragDocBytes();
    const std::string job = jobText(b);
    for (const char* t : {"\"end_mill\"", "\"Diamond_Drag\"", "\"\"", "\"diamond_drag \"", "\"v_bit\""}) {
        auto r = loadCarv(withJob(b, replaced(job, "\"diamond_drag\"", t)));
        INFO(t); CHECK_FALSE(r.ok());
        if (!r.ok()) WARN(t << " -> " << r.error().message);
    }
    // tool_type non-string
    auto r2 = loadCarv(withJob(b, replaced(job, "\"tool_type\": \"diamond_drag\"", "\"tool_type\": 3")));
    CHECK_FALSE(r2.ok());
    // spindle_off removed, tool_type kept -> refused
    auto r3 = loadCarv(withJob(b, replaced(job, "\"spindle_off\": true", "\"spindle_off\": false")));
    CHECK_FALSE(r3.ok());
    if (!r3.ok()) WARN("spindle_off false + tool_type: " << r3.error().message);
    // spindle_off as string/number
    CHECK_FALSE(loadCarv(withJob(b, replaced(job, "\"spindle_off\": true", "\"spindle_off\": 1"))).ok());
    CHECK_FALSE(loadCarv(withJob(b, replaced(job, "\"spindle_off\": true", "\"spindle_off\": \"true\""))).ok());
    // duplicated key
    auto r4 = loadCarv(withJob(b, replaced(job, "\"spindle_off\": true", "\"spindle_off\": true, \"spindle_off\": true")));
    WARN("duplicate spindle_off: " << (r4.ok() ? "ACCEPTED" : r4.error().message));
}
TEST_CASE("V2: old-format drag file (spindle speed 0, no spindle_off/tool_type) is refused by the post") {
    const Bytes b = dragDocBytes();
    std::string job = jobText(b);
    job = replaced(job, "\"spindle_off\": true", "\"spindle_off\": false");
    job = replaced(job, ", \"tool_type\": \"diamond_drag\"", "");
    auto r = loadCarv(withJob(b, job));
    if (!r.ok()) { WARN("old-format loader refused: " << r.error().message); return; }
    const std::string out = postGrbl(r.value().document);
    INFO(out.substr(0, 300));
    CHECK(out.rfind("ERR", 0) == 0);
    WARN("old format post: " << out.substr(0, 200));
}
TEST_CASE("V2: spindle_off on non-engrave records (pocket, drill, profile) with drag tool_type refused everywhere") {
    toolpath::PocketParams pk; pk.cutDepth = 1.0;
    toolpath::DrillParams dr; dr.cutDepth = 1.0;
    toolpath::ProfileParams pr; pr.passes.cutDepth = 1.0;
    for (ToolpathParams p : {ToolpathParams{pk}, ToolpathParams{dr}, ToolpathParams{pr}}) {
        Document doc;
        doc.insertObject(Object::vector(geometry::Contour::rectangle({{0, 0}, {10, 10}}), doc.defaultLayer(), doc.activeSheet()));
        ToolpathRecord rec = baseRec(p);
        REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
        const Bytes good = save(doc);
        const std::string job = jobText(good);
        const std::string crafted = replaced(job, "\"spindle_speed\": 12000.0", "\"spindle_off\": true, \"spindle_speed\": 0.0, \"tool_type\": \"diamond_drag\"");
        auto r = loadCarv(withJob(good, crafted));
        CHECK_FALSE(r.ok());
        if (!r.ok()) WARN(r.error().message);
        // commands
        ToolpathRecord bad = baseRec(p); bad.tool.spindleOff = true; bad.tool.spindleSpeed = 0; bad.tool.toolType = "diamond_drag";
        Document d2;
        d2.insertObject(Object::vector(geometry::Contour::rectangle({{0, 0}, {10, 10}}), d2.defaultLayer(), d2.activeSheet()));
        CHECK_FALSE(d2.execute(std::make_unique<AddToolpathCommand>(bad)));
        ToolpathRecord upd = doc.toolpaths()[0]; upd.tool.spindleOff = true; upd.tool.spindleSpeed = 0; upd.tool.toolType = "diamond_drag";
        CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(upd)));
        // tool_type alone on a spinning record
        ToolpathRecord tt = baseRec(p); tt.tool.toolType = "diamond_drag";
        CHECK_FALSE(d2.execute(std::make_unique<AddToolpathCommand>(tt)));
        // backdoor: insertToolpath then save -> refused; post -> refused?? (type guard passes: tool type is drag)
        Document d3;
        d3.insertObject(Object::vector(geometry::Contour::rectangle({{0, 0}, {10, 10}}), d3.defaultLayer(), d3.activeSheet()));
        d3.insertToolpath(bad);
        CHECK_FALSE(saveCarv(d3).ok());
        WARN("insertToolpath backdoor (non-engrave + drag type) post: " << postGrbl(d3).substr(0, 80));
        ToolpathRecord nt = bad; nt.tool.toolType = "end_mill";
        Document d4; d4.insertToolpath(nt);
        CHECK(postGrbl(d4).rfind("ERR", 0) == 0);
    }
}
TEST_CASE("V2: engrave record with V-bit snapshot + spindle_off is refused (no drag type)") {
    toolpath::EngraveParams params = fullEngrave(); params.depth = 0.5; params.passes = 1;
    Document d = engraveDocument(params);
    const Bytes good = save(d);
    const std::string job = jobText(good);
    auto r = loadCarv(withJob(good, replaced(job, "\"spindle_speed\": ", "\"spindle_off\": true, \"spindle_speed\": ")));
    CHECK_FALSE(r.ok());
    ToolpathRecord upd = d.toolpaths()[0]; upd.tool.spindleOff = true;
    CHECK_FALSE(d.execute(std::make_unique<SetToolpathCommand>(upd)));
    upd.tool.toolType = "diamond_drag";
    // This is allowed by design (snapshot claims drag); just report.
    WARN("V-bit engrave + spindleOff + drag type via SetToolpathCommand: " << d.execute(std::make_unique<SetToolpathCommand>(upd)));
}
TEST_CASE("V2: Profile and Pocket refuse drag tools; Pocket skips only that tool") {
    std::vector<geometry::Contour> sq{geometry::Contour::rectangle({{0, 0}, {20, 20}})};
    toolpath::ProfileParams pr; pr.passes.cutDepth = 1.0;
    auto prr = toolpath::profileToolpath(sq, diamondDrag(), pr);
    CHECK(prr.toolpath.moves.empty());
    bool inv = false; for (auto& d : prr.diagnostics) { WARN(d.message); inv = inv || d.issue == toolpath::ProfileIssue::InvalidTool; }
    CHECK(inv);
    toolpath::PocketParams pk; pk.cutDepth = 1.0;
    tooling::Tool mill = vBit(); mill.type = tooling::ToolType::EndMill; mill.name = "EM"; mill.diameter = 3; mill.includedAngle = 0;
    mill.defaultCuttingData.stepover = 1.0; mill.defaultCuttingData.passDepth = 1.0;
    auto pr2 = toolpath::pocketToolpath(sq, {toolpath::PocketTool{diamondDrag(), std::nullopt}, toolpath::PocketTool{mill, std::nullopt}}, pk);
    for (auto& d : pr2.diagnostics) WARN("pocket: " << d.message);
    CHECK_FALSE(pr2.toolpaths.empty());
    for (auto& t : pr2.toolpaths) { CHECK_FALSE(t.tool.spindleOff); CHECK(t.tool.name == "EM"); }
    auto pr3 = toolpath::pocketToolpath(sq, {toolpath::PocketTool{diamondDrag(), std::nullopt}}, pk);
    bool any = false; for (auto& t : pr3.toolpaths) any = any || !t.moves.empty();
    CHECK_FALSE(any);
}
