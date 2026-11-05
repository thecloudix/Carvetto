// T-0153: V-carve toolpaths as native toolpath records (ADR-0021 schema, ADR-0025): the
// VCarvingParams variant alternative, its job.json params (snake_case keys like carvetto-cli's
// params.json), one record per toolpath of a V-carve (clearance tools, then the carving tool),
// exact round trips, and refusal of damaged params.

#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
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
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/v_carve.hpp"

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

/// V-carve parameters with every value away from its default, with awkward doubles.
toolpath::VCarvingParams fullVCarving() {
    toolpath::VCarvingParams p;
    p.name = "Sign \"V\" \xC3\xA9";
    p.startDepth = 0.30000000000000004;
    p.flatDepth = 1.2000000000000002;
    p.safeZ = 12.000000000000002;
    p.clearanceStrategy = toolpath::PocketStrategy::Raster;
    p.clearanceDirection = toolpath::CutDirection::Conventional;
    p.clearanceRasterAngle = 0.7853981633974483;
    p.rampPlunge = true;
    p.rampDistance = 4.5;
    p.cornerSharpen = false;
    p.order.leftToRight = true;
    p.order.grid = true;
    p.startPosition = {-10.5, -0.0};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool vBit() {
    tooling::Tool t;
    t.id = "v-bit-90";
    t.name = "V-bit 90";
    t.type = tooling::ToolType::VBit;
    t.diameter = 12.7;
    t.includedAngle = 1.5707963267948966;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 18000.0, 1500.0, 500.0, 0.0, 4};
    return t;
}

tooling::Tool endMill() {
    tooling::Tool t;
    t.id = "end-mill-2";
    t.name = "End mill 2";
    t.type = tooling::ToolType::EndMill;
    t.diameter = 2.0;
    t.defaultCuttingData = {1.0, 0.8, 0.0, 0.0, 20000.0, 1200.0, 400.0, 0.0, 2};
    return t;
}

/// A document with a framed square (flat bottom) and a computed V-carve with one clearance tool:
/// two records (clearance, then V-bit) sharing the parameters.
Document vCarvingDocument(const toolpath::VCarvingParams& params) {
    Document doc;
    const ObjectId a = doc.insertObject(Object::vector(geometry::Contour::rectangle({{0, 0}, {20, 12}}),
                                                       doc.defaultLayer(), doc.activeSheet()));
    std::vector<geometry::Contour> contours;
    for (const ObjectId id : doc.selectedVectors(VectorSelection::allVectors())) {
        const auto& cs = doc.findObject(id)->asVector()->contours;
        contours.insert(contours.end(), cs.begin(), cs.end());
    }
    toolpath::VCarvingResult r =
        toolpath::vCarvingToolpath(contours, vBit(), std::nullopt, {toolpath::PocketTool{endMill(), std::nullopt}}, params);
    REQUIRE(r.ok());
    const std::vector<toolpath::Toolpath> tps = r.toolpaths();
    REQUIRE(tps.size() == 2);
    for (const toolpath::Toolpath& tp : tps) {
        ToolpathRecord rec;
        rec.name = tp.name;
        rec.params = params;
        rec.selection.objects = {a};
        rec.tool = tp.tool;
        rec.toolSource = {tp.tool.name == "V-bit 90" ? "v-bit-90" : "end-mill-2", "", ""};
        rec.moves = shareMoves(tp.moves);
        REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    }
    return doc;
}

}  // namespace

TEST_CASE("T-0153: V-carve records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][v_carve]") {
    const Document doc = vCarvingDocument(fullVCarving());
    REQUIRE(doc.toolpaths().size() == 2);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::VCarving);
    CHECK(doc.toolpaths()[1].operation() == ToolpathOperation::VCarving);
    CHECK(doc.toolpaths()[0].name == "Sign \"V\" \xC3\xA9 [End mill 2]");
    CHECK(doc.toolpaths()[1].name == "Sign \"V\" \xC3\xA9 [V-bit 90]");
    CHECK(doc.toolpaths()[0].safeZ() == 12.000000000000002);
    const Bytes once = save(doc);
    const std::string job = jobText(once);
    CHECK(contains(job, "\"operation\": \"v_carve\""));
    for (const char* line : {
             "\"clearance_direction\": \"conventional\",", "\"clearance_raster_angle_rad\": 0.7853981633974483,",
             "\"clearance_strategy\": \"raster\",", "\"corner_sharpen\": false,",
             "\"flat_depth\": 1.2000000000000002,",
             "\"name\": \"Sign \\\"V\\\" \xC3\xA9\",",
             "\"order\": {\"bottom_to_top\": false, \"grid\": true, \"left_to_right\": true, \"selection_order\": false},",
             "\"ramp_distance\": 4.5,", "\"ramp_plunge\": true,", "\"safe_z\": 12.000000000000002,",
             "\"start_depth\": 0.30000000000000004,", "\"start_position\": [-10.5, -0.0],",
             "\"tolerance\": {\"angle_rad\": 1.7453292519943296e-05, \"chordal\": 0.02, \"distance\": 0.002}"}) {
        INFO(line);
        CHECK(contains(job, std::string("\n        ") + line + "\n"));
    }
    LoadedCarv back = load(once);
    REQUIRE(back.document.toolpaths().size() == 2);
    CHECK(back.document.toolpaths() == doc.toolpaths());
    CHECK(std::get<toolpath::VCarvingParams>(back.document.toolpaths()[1].params) == fullVCarving());
    CHECK(back.document.checkIntegrity().empty());
    CHECK(save(back.document, back.extras) == once);

    // T-0160: a record written before corner sharpening (no "corner_sharpen" key; removed here from
    // the first record only) gets the default, on.
    const LoadedCarv old = load(withJob(once, replaced(job, "\"corner_sharpen\": false,", "")));
    REQUIRE(old.document.toolpaths().size() == 2);
    toolpath::VCarvingParams expected = fullVCarving();
    expected.cornerSharpen = true;
    CHECK(std::get<toolpath::VCarvingParams>(old.document.toolpaths()[0].params) == expected);
    CHECK(std::get<toolpath::VCarvingParams>(old.document.toolpaths()[1].params) == fullVCarving());
}

TEST_CASE("T-0153: every V-carve option round-trips with the defaults", "[document][toolpath][carv][v_carve]") {
    for (const auto strategy : {toolpath::PocketStrategy::Offset, toolpath::PocketStrategy::Raster}) {
        for (const auto direction : {toolpath::CutDirection::Climb, toolpath::CutDirection::Conventional}) {
            toolpath::VCarvingParams p;
            p.flatDepth = 1.0;
            p.clearanceStrategy = strategy;
            p.clearanceDirection = direction;
            p.order.selectionOrder = direction == toolpath::CutDirection::Climb;
            const Document doc = vCarvingDocument(p);
            const LoadedCarv back = load(save(doc));
            CHECK(back.document.toolpaths() == doc.toolpaths());
            CHECK(std::get<toolpath::VCarvingParams>(back.document.toolpaths()[0].params) == p);
        }
    }
    Document doc;
    ToolpathRecord pending;
    pending.name = "Not computed";
    pending.params = toolpath::VCarvingParams{};
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(pending)));
    const LoadedCarv back = load(save(doc));
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.document.toolpaths()[0].operation() == ToolpathOperation::VCarving);
}

TEST_CASE("T-0153: V-carve params keys are optional; unknown keys dropped; bad values refused",
          "[document][toolpath][carv][v_carve][errors]") {
    toolpath::VCarvingParams p;
    p.flatDepth = 1.0;
    const Document doc = vCarvingDocument(p);
    const Bytes good = save(doc);
    const std::string job = jobText(good);
    const std::size_t from = job.find("\"params\": {", job.find("\"operation\": \"v_carve\""));
    REQUIRE(from != std::string::npos);
    const std::string paramsObject = job.substr(from, job.find("\n      },", from) + 8 - from);

    // Every key optional: an empty params object loads the defaults.
    const LoadedCarv empty = load(withJob(good, replaced(job, paramsObject, "\"params\": {}")));
    CHECK(std::get<toolpath::VCarvingParams>(empty.document.toolpaths()[0].params) == toolpath::VCarvingParams{});

    // Unknown keys inside params (and the params.json spelling of the angle) are dropped.
    const LoadedCarv extra = load(withJob(good, replaced(job, "\"flat_depth\": 1.0,",
                                                         "\"flat_depth\": 1.0, \"zz_future\": [1], "
                                                         "\"clearance_raster_angle\": 45, \"clearance_tools\": [],")));
    CHECK(extra.document.toolpaths() == doc.toolpaths());
    CHECK_FALSE(contains(jobText(save(extra.document, extra.extras)), "zz_"));

    struct Bad {
        std::string from;
        std::string to;
        std::string message;
    };
    const std::vector<Bad> bad{
        {"\"clearance_strategy\": \"offset\"", "\"clearance_strategy\": \"spiral\"",
         "job.json.toolpaths[0].params.clearance_strategy"},
        {"\"clearance_direction\": \"climb\"", "\"clearance_direction\": \"up\"",
         "job.json.toolpaths[0].params.clearance_direction"},
        {"\"ramp_plunge\": false", "\"ramp_plunge\": 0", "job.json.toolpaths[0].params.ramp_plunge"},
        {"\"corner_sharpen\": true", "\"corner_sharpen\": \"yes\"", "job.json.toolpaths[0].params.corner_sharpen"},
        {"\"flat_depth\": 1.0", "\"flat_depth\": \"NaN\"", "job.json.toolpaths[0].params.flat_depth: must be finite"},
        {"\"start_depth\": 0.0", "\"start_depth\": \"-Infinity\"",
         "job.json.toolpaths[0].params.start_depth: must be finite"},
        {"\"clearance_raster_angle_rad\": 0.0", "\"clearance_raster_angle_rad\": \"Infinity\"",
         "job.json.toolpaths[0].params.clearance_raster_angle_rad: must be finite"},
        {"\"start_position\": [0.0, 0.0]", "\"start_position\": [0.0]", "job.json.toolpaths[0].params.start_position"},
        {"\"grid\": false", "\"grid\": 1", "job.json.toolpaths[0].params.order.grid"},
        {"\"operation\": \"v_carve\"", "\"operation\": \"v_carve2\"", "unsupported toolpath operation 'v_carve2'"},
    };
    for (const Bad& b : bad) {
        INFO(b.to);
        const CarvError e = loadError(withJob(good, replaced(job, b.from, b.to)));
        CHECK(e.code == CarvError::Code::InvalidContent);
        CHECK(contains(e.message, b.message));
    }
}

TEST_CASE("T-0153: V-carve records go through the toolpath commands with undo", "[document][toolpath][v_carve][undo]") {
    Document doc = vCarvingDocument(fullVCarving());
    ToolpathRecord edit = doc.toolpaths()[1];
    std::get<toolpath::VCarvingParams>(edit.params).flatDepth = 0.5;
    REQUIRE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    CHECK(std::get<toolpath::VCarvingParams>(doc.toolpaths()[1].params).flatDepth == 0.5);
    REQUIRE(doc.undo());
    CHECK(std::get<toolpath::VCarvingParams>(doc.toolpaths()[1].params) == fullVCarving());
    // Non-finite V-carve parameters are refused by the commands (nonFiniteParam names them).
    std::get<toolpath::VCarvingParams>(edit.params).rampDistance = std::numeric_limits<double>::infinity();
    CHECK(nonFiniteParam(edit.params) == "ramp_distance");
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
}
