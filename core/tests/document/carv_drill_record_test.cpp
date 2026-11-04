// T-0150 AC3: drilling toolpaths as native toolpath records (ADR-0021 schema, ADR-0022): the
// DrillParams variant alternative, its job.json params (snake_case keys like carvetto-cli's
// params.json), exact round trips, and refusal of damaged params.

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
#include "carvetto/toolpath/drill.hpp"

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

/// Drill parameters with every value away from its default, with awkward doubles.
toolpath::DrillParams fullDrill() {
    toolpath::DrillParams p;
    p.name = "Holes \"M4\" \xC3\xA9";
    p.startDepth = 0.30000000000000004;
    p.cutDepth = 9.7;
    p.safeZ = 12.000000000000002;
    p.peck = true;
    p.peckRetract = toolpath::PeckRetract::AbovePreviousPass;
    p.retractGap = 0.75;
    p.dwellTime = 0.1;
    p.order = {true, false, true, true};
    p.startPosition = {-10.5, -0.0};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool drill() {
    tooling::Tool t;
    t.id = "drill-3mm";
    t.name = "Drill 3 mm";
    t.type = tooling::ToolType::Drill;
    t.diameter = 3.0;
    t.includedAngle = 2.0594885173533086;
    t.defaultCuttingData = {2.5, 0.0, 0.0, 0.0, 10000.0, 600.0, 250.0, 0.0, 2};
    return t;
}

/// A document with two circles and a computed drilling record.
Document drillDocument(const toolpath::DrillParams& params) {
    Document doc;
    const ObjectId a = doc.insertObject(
        Object::vector(geometry::Contour::circle({10, 10}, 3), doc.defaultLayer(), doc.activeSheet()));
    doc.insertObject(Object::vector(geometry::Contour::circle({30, 10}, 3), doc.defaultLayer(), doc.activeSheet()));
    std::vector<geometry::Contour> contours;
    for (const ObjectId id : doc.selectedVectors(VectorSelection::allVectors())) {
        const auto& cs = doc.findObject(id)->asVector()->contours;
        contours.insert(contours.end(), cs.begin(), cs.end());
    }
    toolpath::DrillResult r = toolpath::drillToolpath(contours, drill(), std::nullopt, params);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.selection.layers = {doc.defaultLayer()};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"drill-3mm", "machine-generic-router", "material-hardwood-mdf"};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

}  // namespace

TEST_CASE("AC3: drill records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][drill]") {
    const Document doc = drillDocument(fullDrill());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Drill);
    CHECK(doc.toolpaths()[0].safeZ() == 12.000000000000002);
    const Bytes once = save(doc);
    const std::string job = jobText(once);
    CHECK(contains(job, "\"operation\": \"drill\""));
    for (const char* line : {
             "\"cut_depth\": 9.7,", "\"dwell_time\": 0.1,", "\"name\": \"Holes \\\"M4\\\" \xC3\xA9\",",
             "\"order\": {\"bottom_to_top\": true, \"grid\": true, \"left_to_right\": false, \"selection_order\": true},",
             "\"peck\": true,", "\"peck_retract\": \"above_previous_pass\",", "\"retract_gap\": 0.75,",
             "\"safe_z\": 12.000000000000002,", "\"start_depth\": 0.30000000000000004,",
             "\"start_position\": [-10.5, -0.0],",
             "\"tolerance\": {\"angle_rad\": 1.7453292519943296e-05, \"chordal\": 0.02, \"distance\": 0.002}"}) {
        INFO(line);
        CHECK(contains(job, std::string("\n        ") + line + "\n"));
    }
    LoadedCarv back = load(once);
    REQUIRE(back.document.toolpaths().size() == 1);
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(std::get<toolpath::DrillParams>(back.document.toolpaths()[0].params) == fullDrill());
    CHECK(back.document.checkIntegrity().empty());
    CHECK(save(back.document, back.extras) == once);
}

TEST_CASE("AC3: every drill option round-trips, defaults included", "[document][toolpath][carv][drill]") {
    for (const bool peck : {false, true}) {
        for (const auto retract : {toolpath::PeckRetract::AboveStart, toolpath::PeckRetract::AbovePreviousPass}) {
            toolpath::DrillParams p;
            p.cutDepth = 4.0;
            p.peck = peck;
            p.peckRetract = retract;
            const Document doc = drillDocument(p);
            const LoadedCarv back = load(save(doc));
            CHECK(std::get<toolpath::DrillParams>(back.document.toolpaths()[0].params) == p);
        }
    }
    // A record without moves holding the default parameters.
    Document doc;
    ToolpathRecord pending;
    pending.name = "Not computed";
    pending.params = toolpath::DrillParams{};
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(pending)));
    const LoadedCarv back = load(save(doc));
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.document.toolpaths()[0].operation() == ToolpathOperation::Drill);
}

TEST_CASE("AC3: drill params keys are optional, unknown nested keys dropped, bad values refused",
          "[document][toolpath][carv][drill][errors]") {
    toolpath::DrillParams p;
    p.cutDepth = 4.0;
    const Document doc = drillDocument(p);
    const Bytes good = save(doc);
    const std::string job = jobText(good);
    const std::size_t from = job.find("\"params\": {", job.find("\"operation\": \"drill\""));
    REQUIRE(from != std::string::npos);
    const std::string paramsObject = job.substr(from, job.find("\n      },", from) + 8 - from);

    // Every key optional: an empty params object loads the defaults.
    const LoadedCarv empty = load(withJob(good, replaced(job, paramsObject, "\"params\": {}")));
    CHECK(std::get<toolpath::DrillParams>(empty.document.toolpaths()[0].params) == toolpath::DrillParams{});

    // Unknown keys inside params (and inside order) are dropped.
    const LoadedCarv extra = load(withJob(
        good, replaced(replaced(job, "\"peck\": false", "\"peck\": false, \"zz_future\": [1]"), "\"grid\": false, \"left",
                       "\"grid\": false, \"zz_order\": 2, \"left")));
    CHECK(extra.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK_FALSE(contains(jobText(save(extra.document, extra.extras)), "zz_"));

    struct Bad {
        std::string from;
        std::string to;
        std::string message;
    };
    const std::vector<Bad> bad{
        {"\"peck_retract\": \"above_start\"", "\"peck_retract\": \"sideways\"",
         "job.json.toolpaths[0].params.peck_retract"},
        {"\"peck\": false", "\"peck\": 1", "job.json.toolpaths[0].params.peck"},
        {"\"dwell_time\": 0.0", "\"dwell_time\": \"NaN\"", "job.json.toolpaths[0].params.dwell_time: must be finite"},
        {"\"retract_gap\": 1.0", "\"retract_gap\": \"Infinity\"",
         "job.json.toolpaths[0].params.retract_gap: must be finite"},
        {"\"start_position\": [0.0, 0.0]", "\"start_position\": [0.0]", "job.json.toolpaths[0].params.start_position"},
        {"\"operation\": \"drill\"", "\"operation\": \"bore\"", "unsupported toolpath operation 'bore'"},
    };
    for (const Bad& b : bad) {
        INFO(b.to);
        const CarvError e = loadError(withJob(good, replaced(job, b.from, b.to)));
        CHECK(e.code == CarvError::Code::InvalidContent);
        CHECK(contains(e.message, b.message));
    }
}

TEST_CASE("AC3: drill records go through the toolpath commands with undo", "[document][toolpath][drill][undo]") {
    Document doc = drillDocument(fullDrill());
    ToolpathRecord edit = doc.toolpaths()[0];
    std::get<toolpath::DrillParams>(edit.params).dwellTime = 2.0;
    REQUIRE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    CHECK(std::get<toolpath::DrillParams>(doc.toolpaths()[0].params).dwellTime == 2.0);
    REQUIRE(doc.undo());
    CHECK(std::get<toolpath::DrillParams>(doc.toolpaths()[0].params) == fullDrill());
    // Non-finite drill parameters are refused by the commands (nonFiniteParam names them).
    std::get<toolpath::DrillParams>(edit.params).retractGap = std::numeric_limits<double>::infinity();
    CHECK(nonFiniteParam(edit.params) == "retract_gap");
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
}
