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

TEST_CASE("T-0151: engrave records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][engrave]") {
    const Document doc = engraveDocument(fullEngrave());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Engrave);
    CHECK(doc.toolpaths()[0].safeZ() == 12.000000000000002);
    const Bytes once = save(doc);
    const std::string job = jobText(once);
    CHECK(contains(job, "\"operation\": \"engrave\""));
    for (const char* line : {
             "\"depth\": 0.30000000000000004,", "\"fill_pattern\": \"raster\",",
             "\"name\": \"Sign \\\"A\\\" \xC3\xA9\",", "\"passes\": 3,", "\"raster_angle_rad\": 0.7853981633974483,",
             "\"safe_z\": 12.000000000000002,", "\"start_position\": [-10.5, -0.0],", "\"stepover\": 0.1,",
             "\"strategy\": \"fill\",",
             "\"tolerance\": {\"angle_rad\": 1.7453292519943296e-05, \"chordal\": 0.02, \"distance\": 0.002}"}) {
        INFO(line);
        CHECK(contains(job, std::string("\n        ") + line + "\n"));
    }
    LoadedCarv back = load(once);
    REQUIRE(back.document.toolpaths().size() == 1);
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(std::get<toolpath::EngraveParams>(back.document.toolpaths()[0].params) == fullEngrave());
    CHECK(back.document.checkIntegrity().empty());
    CHECK(save(back.document, back.extras) == once);
}

TEST_CASE("T-0151: every engrave option round-trips, defaults included", "[document][toolpath][carv][engrave]") {
    for (const auto strategy : {toolpath::EngraveStrategy::Outline, toolpath::EngraveStrategy::Fill}) {
        for (const auto pattern : {toolpath::PocketStrategy::Offset, toolpath::PocketStrategy::Raster}) {
            toolpath::EngraveParams p;
            p.depth = 0.5;
            p.passes = 2;
            p.strategy = strategy;
            p.fillPattern = pattern;
            const Document doc = engraveDocument(p);
            const LoadedCarv back = load(save(doc));
            CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
            CHECK(std::get<toolpath::EngraveParams>(back.document.toolpaths()[0].params) == p);
        }
    }
    // A record without moves holding the default parameters.
    Document doc;
    ToolpathRecord pending;
    pending.name = "Not computed";
    pending.params = toolpath::EngraveParams{};
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(pending)));
    const LoadedCarv back = load(save(doc));
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.document.toolpaths()[0].operation() == ToolpathOperation::Engrave);
}

TEST_CASE("T-0151: engrave params keys are optional, unknown keys dropped, bad values refused",
          "[document][toolpath][carv][engrave][errors]") {
    toolpath::EngraveParams p;
    p.depth = 0.5;
    const Document doc = engraveDocument(p);
    const Bytes good = save(doc);
    const std::string job = jobText(good);
    const std::size_t from = job.find("\"params\": {", job.find("\"operation\": \"engrave\""));
    REQUIRE(from != std::string::npos);
    const std::string paramsObject = job.substr(from, job.find("\n      },", from) + 8 - from);

    // Every key optional: an empty params object loads the defaults.
    const LoadedCarv empty = load(withJob(good, replaced(job, paramsObject, "\"params\": {}")));
    CHECK(std::get<toolpath::EngraveParams>(empty.document.toolpaths()[0].params) == toolpath::EngraveParams{});

    // Unknown keys inside params (and the params.json spelling of the angle) are dropped.
    const LoadedCarv extra = load(withJob(
        good, replaced(job, "\"passes\": 1,", "\"passes\": 1, \"zz_future\": [1], \"raster_angle\": 45,")));
    CHECK(extra.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK_FALSE(contains(jobText(save(extra.document, extra.extras)), "zz_"));

    struct Bad {
        std::string from;
        std::string to;
        std::string message;
    };
    const std::vector<Bad> bad{
        {"\"strategy\": \"outline\"", "\"strategy\": \"burn\"", "job.json.toolpaths[0].params.strategy"},
        {"\"fill_pattern\": \"offset\"", "\"fill_pattern\": \"spiral\"", "job.json.toolpaths[0].params.fill_pattern"},
        {"\"passes\": 1,", "\"passes\": 1.5,", "job.json.toolpaths[0].params.passes"},
        {"\"passes\": 1,", "\"passes\": \"2\",", "job.json.toolpaths[0].params.passes"},
        {"\"depth\": 0.5", "\"depth\": \"NaN\"", "job.json.toolpaths[0].params.depth: must be finite"},
        {"\"stepover\": 0.0", "\"stepover\": \"-Infinity\"", "job.json.toolpaths[0].params.stepover: must be finite"},
        {"\"raster_angle_rad\": 0.0", "\"raster_angle_rad\": \"Infinity\"",
         "job.json.toolpaths[0].params.raster_angle_rad: must be finite"},
        {"\"start_position\": [0.0, 0.0]", "\"start_position\": [0.0]", "job.json.toolpaths[0].params.start_position"},
        {"\"operation\": \"engrave\"", "\"operation\": \"burn\"", "unsupported toolpath operation 'burn'"},
    };
    for (const Bad& b : bad) {
        INFO(b.to);
        const CarvError e = loadError(withJob(good, replaced(job, b.from, b.to)));
        CHECK(e.code == CarvError::Code::InvalidContent);
        CHECK(contains(e.message, b.message));
    }
}

TEST_CASE("T-0151: engrave records go through the toolpath commands with undo", "[document][toolpath][engrave][undo]") {
    Document doc = engraveDocument(fullEngrave());
    ToolpathRecord edit = doc.toolpaths()[0];
    std::get<toolpath::EngraveParams>(edit.params).passes = 5;
    REQUIRE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    CHECK(std::get<toolpath::EngraveParams>(doc.toolpaths()[0].params).passes == 5);
    REQUIRE(doc.undo());
    CHECK(std::get<toolpath::EngraveParams>(doc.toolpaths()[0].params) == fullEngrave());
    // Non-finite engrave parameters are refused by the commands (nonFiniteParam names them).
    std::get<toolpath::EngraveParams>(edit.params).stepover = std::numeric_limits<double>::infinity();
    CHECK(nonFiniteParam(edit.params) == "stepover");
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
}

TEST_CASE("T-0154: spindle-off engrave records save and load exactly",
          "[document][toolpath][carv][engrave][spindle-off]") {
    toolpath::EngraveParams params = fullEngrave();
    params.depth = 0.1;
    params.passes = 1;
    const Document doc = engraveDocument(params, diamondDrag());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].tool.spindleOff);
    CHECK(doc.toolpaths()[0].tool.spindleSpeed == 0.0);
    CHECK(spindleOffAllowed(doc.toolpaths()[0]));
    const Bytes once = save(doc);
    const std::string job = jobText(once);
    CHECK(contains(job, "\"spindle_off\": true"));
    LoadedCarv back = load(once);
    REQUIRE(back.document.toolpaths().size() == 1);
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.document.toolpaths()[0].toolpath().tool.spindleOff);
    CHECK(save(back.document, back.extras) == once);

    // Other records do not write the key (their job.json is unchanged).
    CHECK_FALSE(contains(jobText(save(engraveDocument(fullEngrave()))), "spindle_off"));
    // The key must be a boolean; false is accepted and equals an absent key.
    CHECK(contains(loadError(withJob(once, replaced(job, "\"spindle_off\": true", "\"spindle_off\": 1"))).message,
                   "must be true or false"));
    const LoadedCarv off = load(withJob(once, replaced(job, "\"spindle_off\": true", "\"spindle_off\": false")));
    CHECK_FALSE(off.document.toolpaths()[0].tool.spindleOff);
}

TEST_CASE("T-0154: only engrave records may run with the spindle off", "[document][toolpath][carv][spindle-off]") {
    // A profile record with the flag: refused by the commands, by saveCarv and by the loader.
    Document doc;
    doc.insertObject(Object::vector(geometry::Contour::rectangle({{0, 0}, {10, 10}}), doc.defaultLayer(),
                                    doc.activeSheet()));
    ToolpathRecord profile;
    profile.name = "Profile";
    toolpath::ProfileParams pp;
    pp.passes.cutDepth = 1.0;
    profile.params = pp;
    profile.selection = VectorSelection::allVectors();
    profile.tool.name = "Mill";
    profile.tool.toolNumber = 1;
    profile.tool.diameter = 3.0;
    profile.tool.feedRate = 1000.0;
    profile.tool.plungeRate = 300.0;
    profile.moves = shareMoves({toolpath::Move::rapid({0, 0, 5}), toolpath::Move::plunge({0, 0, -1})});
    ToolpathRecord flagged = profile;
    flagged.tool.spindleOff = true;
    CHECK_FALSE(spindleOffAllowed(flagged));
    for (const ToolpathParams& params :
         {ToolpathParams{toolpath::PocketParams{}}, ToolpathParams{toolpath::DrillParams{}}}) {
        ToolpathRecord other = flagged;
        other.params = params;
        CHECK_FALSE(spindleOffAllowed(other));
    }
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(flagged)));
    CHECK(doc.toolpaths().empty());
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(profile)));
    ToolpathRecord edit = doc.toolpaths()[0];
    edit.tool.spindleOff = true;
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    CHECK_FALSE(doc.toolpaths()[0].tool.spindleOff);

    const Bytes good = save(doc);
    const std::string job = jobText(good);

    // saveCarv refuses a document that holds one anyway.
    doc.insertToolpath(flagged);
    const auto refused = saveCarv(doc);
    REQUIRE_FALSE(refused.ok());
    CHECK(contains(refused.error().message,
                   "only engrave toolpaths (diamond-drag tools) may run with the spindle off"));

    // The loader refuses the key on a profile record.
    const CarvError e = loadError(
        withJob(good, replaced(job, "\"spindle_speed\": ", "\"spindle_off\": true, \"spindle_speed\": ")));
    INFO(e.message);
    CHECK(contains(e.message, "tool.spindle_off"));
    CHECK(contains(e.message, "is only allowed on engrave toolpaths"));
}
